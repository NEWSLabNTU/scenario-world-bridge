#!/usr/bin/env python3
"""Roadmap 074 N5 (a) in the arena: the 45 deg Y merge on CARLA's physics.

K Y merges (arena.xodr_ymerge: Town03 j1901's geometry, a one-lane diagonal bending onto a one-lane
main road, no junction), one pair per Y: A on the diagonal, B on the main road. The cars are
arena.py's (the spec's physics, acb's controls: pedal maps, pedal lags, steering geometry), steered
by pure pursuit along their lane's centre line (ymerge_game.Path, checked against the generated
map's waypoints at start). Each drives by ymerge_game.YPolicy (its rule and safety layer) on what
it perceives of the other through arena.Observer (10 Hz, delayed, noisy).

A warm-up brings both cars to speed v on a reference trajectory that puts A's front
v (T_MEAN + dt / 2) and B's v (T_MEAN - dt / 2) before the zone when the run starts, as in the
kinematic model. The run is scored as ymerge_game.simulate scores it, and the comparison is
crossing_arena's: per cell the game on a rule subset solved on the arena's mean payoffs and on the
kinematic model's (with the car's command lags), on D2's yardstick: each kinematic payoff inside the
arena's 95 % interval (mean +- 2 sd over the seeds, at least +- 0.02), the outcomes the same
(first at M, contact, both stopped: the arena's majority), and the class.

Usage (a CARLA of its own, not synchronous; this script makes it synchronous and puts it back):
  scripts/coop/ymerge_arena.py --port 2600 --out <dir> [--k 6] [--seeds 3] [--v 10] [--dts ...]
  scripts/coop/ymerge_arena.py --port 2600 --out <dir> --compare-only
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import statistics as st
import sys
import time
from collections import deque
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_game as G  # noqa: E402
import measures as M  # noqa: E402
import ymerge_game as Y  # noqa: E402

APPROACH, BEYOND, DIAG, SPACING = 330.0, 420.0, 330.0, 300.0
WARMUP = 15.0
A_REF = 2.0
A_SUB = ("go", "t0.5yp2", "t1.5p2", "t1.5yp5", "tinfp2", "tinfp5")
B_SUB = ("hold", "react", "yb1p2", "yb4p5")
STEP = 0.25  # m, the path polyline's spacing


class PathLine:
    """A lane centre line in CARLA's frame, sampled every STEP m: ymerge_game.Path, shifted to Y i
    and y-flipped (CARLA y = -OpenDRIVE y)."""

    def __init__(self, kind, i, before, after):
        self.p = Y.Path(kind, before)
        self.before = before
        n = int((before + after) / STEP) + 1
        self.s = [k * STEP for k in range(n)]
        self.pts = []
        for s in self.s:
            x, y, h = self.p.pose(s)
            self.pts.append((x, -(y - i * SPACING), -h))
        self.last = {}

    def project(self, key, x, y):
        """Arc length of the point nearest (x, y), searched near the last one for this car."""
        k0 = self.last.get(key)
        lo, hi = (0, len(self.pts)) if k0 is None else (max(0, k0 - 80), min(len(self.pts), k0 + 80))
        k = min(range(lo, hi), key=lambda j: (self.pts[j][0] - x) ** 2 + (self.pts[j][1] - y) ** 2)
        self.last[key] = k
        # refine on the segment to its neighbour
        j = k + 1 if k + 1 < len(self.pts) else k - 1
        ax, ay, _ = self.pts[k]
        bx, by, _ = self.pts[j]
        dx, dy = bx - ax, by - ay
        f = ((x - ax) * dx + (y - ay) * dy) / max(dx * dx + dy * dy, 1e-9)
        return self.s[k] + f * (self.s[j] - self.s[k])

    def at(self, s):
        k = min(max(int(round(s / STEP)), 0), len(self.pts) - 1)
        return self.pts[k]


class PCar(AR.Car):
    """arena.Car steering along a PathLine by pure pursuit."""

    def drive_path(self, accel, line, key):
        x, y, yaw, v = self.state()
        th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, self.arena.t)
        look = max(6.0, 0.7 * v)
        s = line.project(key, x, y)
        tx, ty, _ = line.at(s + look)
        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        angle = math.atan2(2 * self.wheelbase * math.sin(alpha), look)
        self.arena.pending.append(carla.command.ApplyVehicleControl(self.a.id, carla.VehicleControl(
            throttle=th, brake=br, steer=self.steering.command(angle))))


def ramp(t, v):
    t1 = v / A_REF
    if t < t1:
        return 0.5 * A_REF * t * t, A_REF * t, A_REF
    return 0.5 * A_REF * t1 * t1 + v * (t - t1), v, 0.0


class YSite:
    """One pair's Y: the two lane lines and each car's u (front past Z's near edge)."""

    def __init__(self, ar, i):
        self.ar, self.i = ar, i
        self.line = {"A": PathLine("diag", i, DIAG + Y.ARC_LEN, BEYOND), "B": PathLine("main", i, APPROACH, BEYOND)}
        w = ar.map.get_waypoint(carla.Location(x=-50.0, y=i * SPACING, z=0.0), project_to_road=True)
        self.z = w.transform.location.z

    def check(self):
        """The largest distance from the analytic lane centre to the map's, on each line."""
        dev = {}
        for k, ln in self.line.items():
            worst = 0.0
            for s in range(10, int(ln.before) - 1, 10):
                x, y, _ = ln.at(s)
                wp = self.ar.map.get_waypoint(carla.Location(x=x, y=y, z=0.0), project_to_road=True)
                worst = max(worst, math.hypot(wp.transform.location.x - x, wp.transform.location.y - y))
            dev[k] = round(worst, 3)
        return dev

    def u(self, k, car):
        x, y, _, _ = car.state()
        ln = self.line[k]
        return ln.project(id(car), x, y) - (ln.before - Y.Z_LEN) + car.length / 2

    def place_at(self, k, car, u):
        ln = self.line[k]
        s = u + ln.before - Y.Z_LEN - car.length / 2
        x, y, h = ln.at(s)
        ln.last.pop(id(car), None)
        car.park(carla.Transform(carla.Location(x=x, y=y, z=self.z + 0.3), carla.Rotation(yaw=math.degrees(h))))


class Run:
    def __init__(self, site, a_car, b_car, job, rng, args):
        self.x, self.A, self.B, self.job, self.args = site, a_car, b_car, job, args
        v, dt = job["v"], job["dt"]
        self.target_u = {"A": -v * (G.T_MEAN + dt / 2), "B": -v * (G.T_MEAN - dt / 2)}
        d_ramp, _, _ = ramp(WARMUP, v)
        self.u_spawn = {k: self.target_u[k] - d_ramp for k in "AB"}
        ra = next(r for r in Y.A_RULES if r[0] == job["a"])
        rb = next(r for r in Y.B_RULES if r[0] == job["b"])
        self.rb = rb
        self.pol = {"A": Y.YPolicy("A", ra, v), "B": Y.YPolicy("B", rb, v, job.get("b_cont", "hold"))}
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.obs = {"A": mk(), "B": mk()}
        self.last_stamp = {"A": None, "B": None}
        self.phase = "warmup"
        self.rec = None

    def car(self, k):
        return self.A if k == "A" else self.B

    def place(self):
        for k in "AB":
            self.x.place_at(k, self.car(k), self.u_spawn[k])

    def start_warmup(self):
        self.tw = self.x.ar.t

    def _perceive(self, t):
        for me, other in (("A", "B"), ("B", "A")):
            c = self.car(other)
            seen = self.obs[me].push(t, self.x.u(other, c), 0.0, c.state()[3])
            if seen is not None and seen[0] >= -1e-9 and seen[0] != self.last_stamp[me]:
                self.last_stamp[me] = seen[0]
                self.pol[me].perceive(seen[0], seen[1], seen[3], seen[4])

    def step_warmup(self):
        t = self.x.ar.t - self.tw
        v = self.job["v"]
        for k in "AB":
            car = self.car(k)
            d, vr, ar_ = ramp(t, v)
            u_ref = self.u_spawn[k] + d
            u = self.x.u(k, car)
            a = max(-4.0, min(3.0, ar_ + 0.6 * (u_ref - u) + 1.8 * (vr - car.state()[3])))
            car.drive_path(a, self.x.line[k], id(car))
        for me, other in (("A", "B"), ("B", "A")):
            c = self.car(other)
            self.obs[me].push(t - WARMUP, self.x.u(other, c), 0.0, c.state()[3])
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.x.ar.t
        self.u0 = {k: self.x.u(k, self.car(k)) for k in "AB"}
        self.v0 = {k: self.car(k).state()[3] for k in "AB"}
        self.meas = M.Measures(b_max=(G.B_MAX, G.B_MAX), pet_sample=G.PET_SAMPLE)
        self.vhist = {"A": deque(), "B": deque()}
        self.peak = {"A": 0.0, "B": 0.0}
        self.at_m = {"A": None, "B": None}
        self.stopped_both = 0.0
        self.both_stopped = False
        self.contact = False

    def step_run(self):
        t = self.x.ar.t - self.t0
        self._perceive(t)
        st_ = {}
        for k in "AB":
            car = self.car(k)
            u = self.x.u(k, car)
            v = car.state()[3]
            st_[k] = (u, v)
            a = self.pol[k].step(t, u, v)
            car.drive_path(a, self.x.line[k], id(car))
            self.pol[k].after(t, u, v)
            h = self.vhist[k]
            h.append((t, v))
            while h and t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[k] = max(self.peak[k], (h[0][1] - v) / AR.PEAK_WINDOW)
            if u >= Y.Z_LEN and self.at_m[k] is None:
                self.at_m[k] = t
        (ua, va), (ub, vb) = st_["A"], st_["B"]
        if va < G.WAIT_V and vb < G.WAIT_V and ua < 0 and ub < 0:
            self.stopped_both += AR.DT
            self.both_stopped = self.both_stopped or self.stopped_both >= 0.5
        else:
            self.stopped_both = 0.0
        self.meas.push_poses(t, self.A.pose(), self.B.pose())
        if self.meas.clear <= 0.0:
            self.contact = True
        if self.contact or t >= G.T_MAX or (ua >= Y.EXIT + 2 and ub >= Y.EXIT + 2 and t >= G.HORIZON - 1e-9):
            self.finish(t, st_)

    def finish(self, t, st_):
        lost = []
        for k in "AB":
            u, _ = st_[k]
            lost.append(1.0 if self.contact or u < Y.EXIT else M.lost_progress(self.v0[k], self.meas.s_h["AB".index(k)]))
        ta = self.at_m["A"] if self.at_m["A"] is not None else math.inf
        tb = self.at_m["B"] if self.at_m["B"] is not None else math.inf
        first = "" if math.isinf(ta) and math.isinf(tb) else ("A" if ta < tb else "B")
        m = self.meas
        o = G.Outcome(tuple(lost), (self.peak["A"], self.peak["B"]), self.contact, m.ttc2d, m.thw, m._pet(), m.clear,
                      first, self.both_stopped, self.job.get("b_cont", "") if self.job["b"] == "react" else "",
                      (self.pol["A"].shield, self.pol["B"].shield), (self.pol["A"].rested, self.pol["B"].rested))
        self.rec = {**self.job, "first": first, "contact": int(self.contact), "both_stopped": int(self.both_stopped),
                    "uA": round(G.utility(o, 0), 4), "uB": round(G.utility(o, 1), 4),
                    "lostA": round(lost[0], 4), "lostB": round(lost[1], 4),
                    "peakA": round(self.peak["A"], 2), "peakB": round(self.peak["B"], 2),
                    "pet": round(o.pet, 3) if math.isfinite(o.pet) else "inf",
                    "ttc": round(o.ttc, 3) if math.isfinite(o.ttc) else "inf", "clear": round(o.clear, 3),
                    "uA0": round(self.u0["A"], 2), "uB0": round(self.u0["B"], 2),
                    "vA0": round(self.v0["A"], 2), "vB0": round(self.v0["B"], 2),
                    "dt_real": round(-self.u0["A"] / max(self.v0["A"], 0.1) + self.u0["B"] / max(self.v0["B"], 0.1), 3),
                    "t_run": round(t, 2), "shieldA": int(o.shield[0]), "shieldB": int(o.shield[1])}
        self.phase = "done"


def jobs(args):
    out = []
    for dt in [float(x) for x in args.dts.split(",")]:
        for a in A_SUB:
            for b in B_SUB:
                for c in (("yield", "hold") if b == "react" else (None,)):
                    for s in range(args.seeds):
                        j = {"dt": dt, "v": args.v, "a": a, "b": b, "seed": s}
                        if c:
                            j["b_cont"] = c
                        out.append(j)
    return out


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k, AR.xodr_ymerge(args.k, APPROACH, BEYOND, DIAG, Y.R_ARC, SPACING))
    rows = []
    t_start = time.monotonic()
    try:
        sites = [YSite(ar, i) for i in range(args.k)]
        devs = [s.check() for s in sites]
        print("lane centre vs map, worst (m):", devs, flush=True)
        if any(d > 0.3 for dv in devs for d in dv.values()):
            raise SystemExit(f"the generated lanes are not where ymerge_game.Path puts them: {devs}")
        actors = []
        for s in sites:
            for k in "AB":
                x, y, h = s.line[k].at(50.0)
                a = ar.world.spawn_actor(ar.bp, carla.Transform(carla.Location(x=x, y=y, z=s.z + 0.6),
                                                                carla.Rotation(yaw=math.degrees(h))))
                ar.actors.append(a)
                actors.append(a)
        ar.put_physics(actors)
        cars = [PCar(ar, a) for a in actors]
        while work:
            runs = []
            for i, s in enumerate(sites):
                if not work:
                    break
                r = Run(s, cars[2 * i], cars[2 * i + 1], work.popleft(), rng, args)
                r.place()
                runs.append(r)
            for _ in range(3):
                for r in runs:
                    for c in (r.A, r.B):
                        c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            for r in runs:
                r.start_warmup()
            while any(r.phase != "done" for r in runs):
                for r in runs:
                    if r.phase == "warmup":
                        r.step_warmup()
                    elif r.phase == "run":
                        r.step_run()
                    else:
                        for k in "AB":
                            c = r.car(k)
                            c.drive_path(-3.0, r.x.line[k], id(c))
                ar.tick()
            rows += [r.rec for r in runs]
            print(f"  {len(rows)}/{n_all} runs, {time.monotonic() - t_start:.0f} s", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        keys = sorted({k for r in rows for k in r}, key=lambda k: list(rows[0]).index(k) if k in rows[0] else 99)
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    json.dump({"lane_dev_m": devs}, open(os.path.join(args.out, "lanes.json"), "w"))
    return rows, time.monotonic() - t_start


def compare(args, rows):
    """crossing_arena.compare on the Y merge: the arena's game per cell against the kinematic model's."""
    by = {}
    for r in rows:
        key = (float(r["dt"]), r["a"], r["b"] if r["b"] != "react" else f"react:{r['b_cont']}")
        by.setdefault(key, []).append(r)
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    for dt in [float(x) for x in args.dts.split(",")]:
        kin = Y.outcomes(dt, args.v, args.v, G.car_lags() if args.car_lags else None)
        ja_ar, jb_ar, ja_k, jb_k, res_k, res_ar = {}, {}, {}, {}, {}, {}
        for a in A_SUB:
            for b in B_SUB:
                if b == "react":
                    cands_ar = [(c, by[(dt, a, f"react:{c}")]) for c in ("yield", "hold")]
                    _, rs = max(cands_ar, key=lambda x: st.mean(float(r["uB"]) for r in x[1]))
                    cands_k = [(c, kin[(a, f"react:{c}")]) for c in ("yield", "hold")]
                    _, ok_ = max(cands_k, key=lambda x: G.utility(replace(x[1], peak=x[1].peak_w), 1))
                else:
                    rs, ok_ = by[(dt, a, b)], kin[(a, b)]
                ja_ar[(a, b)] = st.mean(float(r["uA"]) for r in rs)
                jb_ar[(a, b)] = st.mean(float(r["uB"]) for r in rs)
                ok_ = replace(ok_, peak=ok_.peak_w)
                ja_k[(a, b)], jb_k[(a, b)] = G.utility(ok_, 0), G.utility(ok_, 1)
                res_k[(a, b)], res_ar[(a, b)] = ok_, rs
                ok_all = True
                for mkey, kv in (("uA", ja_k[(a, b)]), ("uB", jb_k[(a, b)])):
                    xs = [float(r[mkey]) for r in rs]
                    mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                    ok = abs(kv - mu) <= max(2 * sd, 0.02)
                    inside += ok
                    total += 1
                    ok_all &= ok
                maj = lambda f: max(set(f(r) for r in rs), key=lambda x: sum(f(r) == x for r in rs))  # noqa: E731
                ar_first, ar_contact, ar_wait = maj(lambda r: r["first"]), maj(lambda r: int(r["contact"])), maj(lambda r: int(r["both_stopped"]))
                same = (ar_first == ok_.first and ar_contact == int(ok_.contact) and ar_wait == int(ok_.both_stopped))
                agree += same
                n_out += 1
                lines.append(f"dt {dt:+4.1f}  {a:>8}/{b:<6} | arena uA {ja_ar[(a, b)]:+.3f} uB {jb_ar[(a, b)]:+.3f} "
                             f"first {ar_first or '-'} contact {ar_contact} wait {ar_wait} | kin uA {ja_k[(a, b)]:+.3f} "
                             f"uB {jb_k[(a, b)]:+.3f} first {ok_.first or '-'} contact {int(ok_.contact)} "
                             f"wait {int(ok_.both_stopped)} | {'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cls = {}
        for name, ja, jb, res in (("arena", ja_ar, jb_ar, None), ("kinematic", ja_k, jb_k, res_k)):
            pure, mixed = G.solve(ja, jb, A_SUB, B_SUB)
            if res is None:
                firsts = {max(set(r["first"] for r in res_ar[p]), key=lambda x: sum(r["first"] == x for r in res_ar[p])) for p in pure}
                contact = any(st.mean(int(r["contact"]) for r in res_ar[p]) >= 0.5 for p in pure)
                wait = any(st.mean(int(r["both_stopped"]) for r in res_ar[p]) >= 0.5 for p in pure)
            else:
                firsts = {res[p].first for p in pure}
                contact = any(res[p].contact for p in pure)
                wait = any(res[p].both_stopped for p in pure)
            c = "!" if contact else "W" if wait else "C" if len(firsts) > 1 else (firsts.pop() or "?") if firsts else ("M" if mixed else "?")
            cls[name] = (c, " ".join(f"{a}/{b}" for a, b in pure))
        summary["cells"].append({"dt": dt, "v": args.v, "class_arena": cls["arena"][0], "class_kinematic": cls["kinematic"][0],
                                 "eq_arena": cls["arena"][1], "eq_kinematic": cls["kinematic"][1]})
        lines.append(f"## dt {dt:+.1f}: class arena {cls['arena'][0]} ({cls['arena'][1]}) | kinematic {cls['kinematic'][0]} ({cls['kinematic'][1]})")
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
                    "classes_agree": sum(c["class_arena"] == c["class_kinematic"] for c in summary["cells"])})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); "
                 f"outcomes agree in {agree} of {n_out}; classes agree in {summary['classes_agree']} of {len(summary['cells'])}")
    return lines, summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=6)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", type=float, default=10.0)
    p.add_argument("--dts", default="-1.5,-0.7,-0.3,-0.1,0,0.1,0.3,0.7,1.5")
    p.add_argument("--ideal-car", dest="car_lags", action="store_false")
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
    open(os.path.join(args.out, "compare.txt"), "w").write("\n".join(lines) + "\n")
    json.dump(summary, open(os.path.join(args.out, "summary.json"), "w"), indent=1)
    print("\n".join(lines[-(len(summary["cells"]) + 1):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
