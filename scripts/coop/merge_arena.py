#!/usr/bin/env python3
"""Roadmap 074 S3 in the arena: the merge at a lane drop on CARLA's physics.

arena.xodr_lane_drop: K parallel roads whose lane -2 runs at full width to X_END (the model's lane
end) and then tapers to nothing over TAPER m, so there is ground wherever the model has lane; one
pair per road: A in lane -2, B in lane -1. The cars are arena.py's (the spec's physics,
acb's controls). Both drive by merge_game.Rules, the same object the kinematic model steps, on
what each perceives of the other through arena.Observer (10 Hz, delayed, noisy), held OBS + DECIDE
s and predicted to now as in the kinematic model. A's lane change is the rules' cosine, steered by
pure pursuit aimed at the path's point one look-ahead ahead.

A warm-up brings both to the context (d_end, d0, dv at vB) on reference trajectories. The run is
scored as merge_game.simulate scores it (lost progress over exactly H, risk from
measures.push_poses on the true poses, peak braking from the speed drop over 0.5 s, contact; A's
front past the lane's end with A less than half across counts as contact).

Per context the game on a rule subset is solved on the arena's mean payoffs and on merge_game's
(with the car's command lags) and compared on D2's yardstick: payoffs inside the arena's 95 %
interval (mean +- 2 sd over seeds, at least +- 0.02), outcomes the same (A ahead, contact, a forced
stop), and the class.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/merge_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--vb 12]
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import random
import statistics as st
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_game as X  # noqa: E402
import measures as M  # noqa: E402
import merge_game as G  # noqa: E402

ROAD_LEN, X_END, TAPER = 700.0, 500.0, 30.0
WARMUP, A_REF = 15.0, 2.0
A_SUB = ("e0.6d", "e1.2yd", "e1.2f", "z1.2d", "z1.2yf", "z0.6f")
B_SUB = ("hold", "react", "c20", "c40")


def ramp(t, v):
    t1 = v / A_REF if v > 0 else 0.0
    if t < t1:
        return 0.5 * A_REF * t * t, A_REF * t, A_REF
    return 0.5 * A_REF * t1 * t1 + v * (t - t1), v, 0.0


class Seen:
    """What one car has perceived of the other (arena.Observer's samples), held OBS + DECIDE s."""

    def __init__(self, obs):
        self.obs, self.hist, self.last = obs, [], None

    def push(self, t, x, lat, v):
        s = self.obs.push(t, x, lat, v)
        if s is not None and s[0] >= -1e-9 and s[0] != self.last:
            self.last = s[0]
            self.hist.append(s)  # (stamp, x, lat, v, acc)

    def at(self, t, truth):
        h = next((h for h in reversed(self.hist) if h[0] <= t - G.OBS - G.DECIDE), self.hist[0] if self.hist else None)
        if h is None:
            return truth
        return h[1] + h[3] * (t - h[0]), h[3], h[2], h[4]  # x predicted to now, v, lat, acc


class Pair:
    def __init__(self, ar, road, a_car, b_car, job, rng, args):
        self.ar, self.road, self.A, self.B, self.job = ar, road, a_car, b_car, job
        self.yB, _ = ar.lane_y(road, -1)
        self.yA0, _ = ar.lane_y(road, -2)
        vb, dv = job["vb"], job["dv"]
        self.va0 = max(0.0, vb + dv)
        self.xa_end = X_END - job["d_end"]  # A's front at the run's start
        self.xb_end = self.xa_end - G.L - job["d0"]  # B's front
        ra = next(r for r in G.A_RULES if r[0] == job["a"])
        rb = next(r for r in G.B_RULES if r[0] == job["b"])
        self.R = G.Rules(ra, rb, X_END, self.va0, vb, job.get("b_cont", "hold"))
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.seen_b, self.seen_a = Seen(mk()), Seen(mk())  # what A sees of B, what B sees of A
        self.phase, self.rec = "warmup", None

    def front(self, car):
        return car.state()[0] + car.length / 2

    def lat(self):
        return max(0.0, min(1.0, (self.A.state()[1] - self.yA0) / (self.yB - self.yA0)))

    def place(self):
        for car, x_end, v, y in ((self.A, self.xa_end, self.va0, self.yA0), (self.B, self.xb_end, self.job["vb"], self.yB)):
            loc = car.a.get_transform().location
            x0 = x_end - ramp(WARMUP, v)[0]
            car.park(carla.Transform(carla.Location(x=x0 - car.length / 2, y=y, z=loc.z + 0.3), carla.Rotation(yaw=0)))

    def start_warmup(self):
        self.tw = self.ar.t

    def step_warmup(self):
        t = self.ar.t - self.tw
        for car, x_end, v, y in ((self.A, self.xa_end, self.va0, self.yA0), (self.B, self.xb_end, self.job["vb"], self.yB)):
            d, vr, ar_ = ramp(t, v)
            x_ref = x_end - (ramp(WARMUP, v)[0] - d)
            a = max(-4.0, min(3.0, ar_ + 0.6 * (x_ref - self.front(car)) + 1.8 * (vr - car.state()[3])))
            car.drive(a, y)
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.ar.t
        self.meas = M.Measures(b_max=(G.CAP.b_max, G.CAP.b_max))
        self.vh = {"A": deque(), "B": deque()}
        self.peak = {"A": 0.0, "B": 0.0}
        self.contact = self.ran_out = False
        self.vb0 = self.B.state()[3]
        self.va0_real = self.A.state()[3]
        self.d0_real = (self.front(self.A) - self.A.length) - self.front(self.B)
        self.dend_real = X_END - self.front(self.A)

    def step_run(self):
        t = self.ar.t - self.t0
        xa, xb = self.front(self.A), self.front(self.B)
        va, vb = self.A.state()[3], self.B.state()[3]
        lat = self.lat()
        self.seen_b.push(t, xb, 0.0, vb)
        self.seen_a.push(t, xa, lat, va)
        sb = self.seen_b.at(t, (xb, vb, 0.0, 0.0))
        sa = self.seen_a.at(t, (xa, va, lat, 0.0))
        # A's rules use its own plan's lateral progress (as the kinematic model, where plan and motion
        # are one); B sees A's measured position, and running out is judged on it
        aa, ab = self.R.step(t, xa, va, self.R.lat_of(t)[0], xb, vb, (sb[0], sb[1], sb[3]), (sa[0], sa[1], sa[2]))
        lead = max(6.0, 0.7 * va) / max(va, 1.0)  # pure pursuit's point is look / v ahead on the path
        lat_ref, _ = self.R.lat_of(t + lead)
        self.A.drive(aa, self.yA0 + (self.yB - self.yA0) * lat_ref)
        self.B.drive(ab, self.yB)
        self.R.after(xa, va)
        # running out: judged on A's plan at the lane's end, as the kinematic model does (its plan is
        # its motion), and physically past the taper's ground with A mostly still in the ending lane
        if (xa > X_END and self.R.lat_of(t)[0] < 0.5) or (xa > X_END + TAPER and lat < 0.5):
            self.ran_out = True
        for k, vv in (("A", va), ("B", vb)):
            h = self.vh[k]
            h.append((t, vv))
            while t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[k] = max(self.peak[k], (h[0][1] - vv) / AR.PEAK_WINDOW)
        self.meas.push_poses(t, self.A.pose(), self.B.pose())
        if self.meas.clear <= 0.0 or self.ran_out:
            self.contact = True
        done_lat = self.R.lat_of(t)[0] >= 1.0
        if self.contact or t >= G.T_MAX - 1e-9 or (t >= G.HORIZON - 1e-9 and done_lat):
            self.finish(t, xa, xb, done_lat)

    def finish(self, t, xa, xb, merged):
        if self.contact or self.ran_out:
            self.A.dirty = self.B.dirty = True
        # put both cars back at their start, at rest, before either can roll off the lane's end
        for car, x_end, v, y in ((self.A, self.xa_end, self.va0, self.yA0), (self.B, self.xb_end, self.job["vb"], self.yB)):
            loc = car.a.get_transform().location
            car.park(carla.Transform(carla.Location(x=x_end - ramp(WARMUP, v)[0] - car.length / 2, y=y, z=loc.z + 0.3),
                                     carla.Rotation(yaw=0)))
        m = self.meas
        lost_a = 1.0 if self.contact or not merged else M.lost_progress(self.va0, m.s_h[0] if m.s_h[0] is not None else m.s[0])
        lost_b = 1.0 if self.contact else M.lost_progress(self.job["vb"], m.s_h[1] if m.s_h[1] is not None else m.s[1])
        o = G.Outcome((lost_a, lost_b), (self.peak["A"], self.peak["B"]), self.contact, m.ttc2d, m.thw, m.clear,
                      merged and xa > xb, merged, self.R.stopped, self.R.forced, self.ran_out)
        self.rec = {**self.job, "uA": round(G.utility(o, 0), 4), "uB": round(G.utility(o, 1), 4),
                    "ahead": int(o.ahead), "merged": int(merged), "contact": int(self.contact), "stopped": int(self.R.stopped),
                    "forced": int(self.R.forced), "ran_out": int(self.ran_out), "lostA": round(lost_a, 4), "lostB": round(lost_b, 4),
                    "peakA": round(self.peak["A"], 2), "peakB": round(self.peak["B"], 2),
                    "ttc": round(m.ttc2d, 3) if math.isfinite(m.ttc2d) else "inf", "clear": round(m.clear, 3),
                    "d0_real": round(self.d0_real, 2), "dend_real": round(self.dend_real, 2),
                    "va_real": round(self.va0_real, 2), "vb_real": round(self.vb0, 2), "t_run": round(t, 2)}
        self.phase = "done"


def contexts(args):
    return list(itertools.product([float(x) for x in args.d_ends.split(",")], [float(x) for x in args.d0s.split(",")],
                                  [float(x) for x in args.dvs.split(",")]))


def jobs(args):
    out = []
    for de, d0, dv in contexts(args):
        for a, b in itertools.product(A_SUB, B_SUB):
            for c in (("yield", "hold") if b == "react" else (None,)):
                for s in range(args.seeds):
                    j = {"d_end": de, "d0": d0, "dv": dv, "vb": args.vb, "a": a, "b": b, "seed": s}
                    if c:
                        j["b_cont"] = c
                    out.append(j)
    return out


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k, AR.xodr_lane_drop(args.k, ROAD_LEN, X_END, TAPER))
    rows, t_start = [], time.monotonic()
    lost_cars = [0]
    try:
        actors = []
        for r in range(args.k):
            actors += [ar.spawn(r, -2, 60.0), ar.spawn(r, -1, 30.0)]
        ar.put_physics(actors)
        cars = [AR.Car(ar, a) for a in actors]
        while work:
            pairs = []
            for r in range(args.k):
                if not work:
                    break
                for i in (2 * r, 2 * r + 1):
                    # a car that ran off the lane's end or touched the other is not to be trusted
                    # (CARLA can leave it stuck): a fresh one, its physics applied and checked
                    if not cars[i].a.is_alive or getattr(cars[i], "dirty", False):
                        if cars[i].a.is_alive:
                            cars[i].a.destroy()
                        a = ar.spawn(r, -2 if i % 2 == 0 else -1, 60.0 if i % 2 == 0 else 30.0)
                        ar.put_physics([a])
                        cars[i] = AR.Car(ar, a)
                        lost_cars[0] += 1
                p = Pair(ar, r, cars[2 * r], cars[2 * r + 1], work.popleft(), rng, args)
                p.place()
                pairs.append(p)
            for _ in range(3):
                for p in pairs:
                    for c in (p.A, p.B):
                        c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            for p in pairs:
                p.start_warmup()
            while any(p.phase != "done" for p in pairs):
                for p in pairs:
                    if p.phase == "warmup":
                        p.step_warmup()
                    elif p.phase == "run":
                        p.step_run()
                    else:
                        for c in (p.A, p.B):
                            c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            rows += [p.rec for p in pairs]
            print(f"  {len(rows)}/{n_all} runs, {time.monotonic() - t_start:.0f} s, {lost_cars[0]} cars replaced", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k) if k in rows[0] else 99))
        w.writeheader()
        w.writerows(rows)
    return rows, time.monotonic() - t_start


def compare(args, rows):
    lags = X.car_lags()
    by = {}
    for r in rows:
        b = r["b"] if r["b"] != "react" else f"react:{r['b_cont']}"
        by.setdefault((float(r["d_end"]), float(r["d0"]), float(r["dv"]), r["a"], b), []).append(r)
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    maj = lambda rs, k: round(st.mean(int(r[k]) for r in rs))  # noqa: E731
    for de, d0, dv in contexts(args):
        ja_a, jb_a, ja_k, jb_k, res_a = {}, {}, {}, {}, {}
        kin = {}
        for a in A_SUB:
            ra = next(x for x in G.A_RULES if x[0] == a)
            for b in B_SUB:
                rb = next(x for x in G.B_RULES if x[0] == b)
                if b == "react":
                    cands_a = [by[(de, d0, dv, a, f"react:{c}")] for c in ("yield", "hold")]
                    rs = max(cands_a, key=lambda x: st.mean(float(r["uB"]) for r in x))
                    ok = max((G.simulate(ra, rb, de, d0, dv, args.vb, c, lags) for c in ("yield", "hold")),
                             key=lambda o: G.utility(o, 1))
                else:
                    rs, ok = by[(de, d0, dv, a, b)], G.simulate(ra, rb, de, d0, dv, args.vb, lags=lags)
                kin[(a, b)], res_a[(a, b)] = ok, rs
                ja_a[(a, b)] = st.mean(float(r["uA"]) for r in rs)
                jb_a[(a, b)] = st.mean(float(r["uB"]) for r in rs)
                ja_k[(a, b)], jb_k[(a, b)] = G.utility(ok, 0), G.utility(ok, 1)
                ok_all = True
                for key, kv in (("uA", ja_k[(a, b)]), ("uB", jb_k[(a, b)])):
                    xs = [float(r[key]) for r in rs]
                    mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                    hit = abs(kv - mu) <= max(2 * sd, 0.02)
                    inside += hit
                    total += 1
                    ok_all &= hit
                same = (maj(rs, "ahead") == int(ok.ahead) and maj(rs, "contact") == int(ok.contact)
                        and maj(rs, "stopped") == int(ok.stopped))
                agree += same
                n_out += 1
                lines.append(f"end {de:3g} d0 {d0:+5.1f} dv {dv:+3.0f}  {a:>6}/{b:<5} | arena uA {ja_a[(a, b)]:+.3f} uB {jb_a[(a, b)]:+.3f} "
                             f"ahead {maj(rs, 'ahead')} contact {maj(rs, 'contact')} stop {maj(rs, 'stopped')} | kin uA {ja_k[(a, b)]:+.3f} "
                             f"uB {jb_k[(a, b)]:+.3f} ahead {int(ok.ahead)} contact {int(ok.contact)} stop {int(ok.stopped)} | "
                             f"{'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cls = {}
        for name, ja, jb in (("arena", ja_a, jb_a), ("kinematic", ja_k, jb_k)):
            pure, mixed = X.solve(ja, jb, A_SUB, B_SUB)
            if name == "arena":
                ahead = {maj(res_a[p], "ahead") for p in pure}
                contact = any(maj(res_a[p], "contact") for p in pure)
                stop = any(maj(res_a[p], "stopped") for p in pure)
            else:
                ahead = {int(kin[p].ahead) for p in pure}
                contact = any(kin[p].contact for p in pure)
                stop = any(kin[p].stopped for p in pure)
            c = "!" if contact else "S" if stop else "C" if len(ahead) > 1 else ("A" if ahead.pop() else "B") if ahead else (
                "M" if mixed else "?")
            cls[name] = (c, " ".join(f"{a}/{b}" for a, b in pure))
        summary["cells"].append({"d_end": de, "d0": d0, "dv": dv, "class_arena": cls["arena"][0], "class_kinematic": cls["kinematic"][0],
                                 "eq_arena": cls["arena"][1], "eq_kinematic": cls["kinematic"][1]})
        lines.append(f"## end {de:g} d0 {d0:+g} dv {dv:+g}: class arena {cls['arena'][0]} ({cls['arena'][1]}) | "
                     f"kinematic {cls['kinematic'][0]} ({cls['kinematic'][1]})")
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
                    "classes_agree": sum(c["class_arena"] == c["class_kinematic"] for c in summary["cells"])})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); "
                 f"outcomes agree in {agree} of {n_out}; classes agree in {summary['classes_agree']} of {len(summary['cells'])}")
    return lines, summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--vb", type=float, default=12.0)
    p.add_argument("--d-ends", default="40,80")
    p.add_argument("--d0s", default="-5,2.5,7.5,15")
    p.add_argument("--dvs", default="-2,0")
    p.add_argument("--obs-delay", type=float, default=G.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--compare-only", action="store_true")
    args = p.parse_args()
    if args.compare_only:
        rows = list(csv.DictReader(open(os.path.join(args.out, "runs.csv"))))
        wall = None
    else:
        rows, wall = run(args)
    lines, summary = compare(args, rows)
    summary["wall_s"] = None if wall is None else round(wall, 1)
    summary["runs"] = len(rows)
    with open(os.path.join(args.out, "compare.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print("\n".join(lines[-(len(summary["cells"]) + 1):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
