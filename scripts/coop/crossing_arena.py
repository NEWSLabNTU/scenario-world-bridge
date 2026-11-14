#!/usr/bin/env python3
"""Roadmap 074 S2 in the arena: the unsignalled crossing on CARLA's physics.

K crossings at grade (arena.xodr_crossings: two one-lane roads per pair, no junction), one pair
per crossing. A drives along +x, B across it. The cars are arena.py's: the spec's physics and
acb's controls (pedal maps, pedal lags, steering geometry). Each car drives by crossing_game's
Policy (its rule and safety layer) on what it perceives of the other through arena.Observer
(10 Hz, delayed, noisy), not on the truth.

A warm-up brings both cars to speed v on a reference trajectory that puts A's front
v * (T_MEAN + dt / 2) and B's v * (T_MEAN - dt / 2) before the box when the run starts, as in the
kinematic model. The run is scored like crossing_game.simulate: lost progress, the risk from
measures.push_poses on the cars' true poses, peak braking (speed drop over 0.5 s, arena.py's),
contact; who entered the box first, whether both stopped. The kinematic side of the comparison
is scored the same way (its peak braking from the speed drop over 0.5 s too).

Then, per cell, the game on a subset of the rules (A_SUB x B_SUB) is solved on the arena's mean
payoffs and on the kinematic model's, and compared on D2's yardstick: each kinematic payoff
inside the arena's 95 % interval (mean +- 2 sd over the seeds, at least +- 0.02), the outcomes
the same (first, contact, both stopped: the arena's majority), and the class.

Usage (a CARLA of its own, not synchronous; this script makes it synchronous and puts it back):
  scripts/coop/crossing_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--v 10] [--dts -2,-1,...]
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

APPROACH, BEYOND, SPACING = 330.0, 80.0, 450.0
WARMUP = 15.0  # s
A_REF = 2.0  # m/s^2, the warm-up's reference acceleration
A_SUB = ("go", "t0.5yp2", "t1.5p2", "t1.5yp5", "tinfp2", "tinfp5")
B_SUB = ("hold", "react", "yb1p2", "yb4p5")


class XCar(AR.Car):
    """arena.Car steering along any straight line, not only +x."""

    def drive_line(self, accel, p0, d):
        x, y, yaw, v = self.state()
        th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, self.arena.t)
        look = max(6.0, 0.7 * v)
        s = (x - p0[0]) * d[0] + (y - p0[1]) * d[1]
        tx, ty = p0[0] + d[0] * (s + look), p0[1] + d[1] * (s + look)
        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        angle = math.atan2(2 * self.wheelbase * math.sin(alpha), look)
        self.arena.pending.append(carla.command.ApplyVehicleControl(self.a.id, carla.VehicleControl(
            throttle=th, brake=br, steer=self.steering.command(angle))))


def ramp(t, v):
    """Distance and speed of the warm-up's reference: A_REF up to v, then v."""
    t1 = v / A_REF
    if t < t1:
        return 0.5 * A_REF * t * t, A_REF * t, A_REF
    return 0.5 * A_REF * t1 * t1 + v * (t - t1), v, 0.0


class Crossing:
    """One pair on crossing i: lanes, the box, each car's distance u to the box's near edge."""

    def __init__(self, ar, i):
        self.ar, self.i = ar, i
        cx, cy = i * SPACING, i * SPACING  # CARLA: OpenDRIVE y flipped
        m = ar.map
        wa = m.get_waypoint(carla.Location(x=cx - 50.0, y=cy, z=0.0), project_to_road=True)
        wb = m.get_waypoint(carla.Location(x=cx, y=cy + 50.0, z=0.0), project_to_road=True)
        self.dir = {}
        self.line = {}
        for k, w in (("A", wa), ("B", wb)):
            yaw = math.radians(w.transform.rotation.yaw)
            self.dir[k] = (math.cos(yaw), math.sin(yaw))
            self.line[k] = (w.transform.location.x, w.transform.location.y)
            self.z = w.transform.location.z
        # the box's centre: where the two lane centre lines meet
        self.centre = (self.line["B"][0], self.line["A"][1])

    def u(self, k, car):
        """Front's distance past the box's near edge along k's road."""
        x, y, _, _ = car.state()
        d = self.dir[k]
        s = (x - self.centre[0]) * d[0] + (y - self.centre[1]) * d[1]
        return s + car.length / 2 + G.W_BOX / 2

    def place_at(self, k, car, u):
        d = self.dir[k]
        s = u - car.length / 2 - G.W_BOX / 2
        x, y = self.centre[0] + d[0] * s, self.centre[1] + d[1] * s
        if k == "A":
            y = self.line["A"][1]
        else:
            x = self.line["B"][0]
        yaw = math.degrees(math.atan2(d[1], d[0]))
        car.park(carla.Transform(carla.Location(x=x, y=y, z=self.z + 0.3), carla.Rotation(yaw=yaw)))


class Run:
    def __init__(self, xing, a_car, b_car, job, rng, args):
        self.x, self.A, self.B, self.job, self.args = xing, a_car, b_car, job, args
        v, dt = job["v"], job["dt"]
        self.target_u = {"A": -v * (G.T_MEAN + dt / 2), "B": -v * (G.T_MEAN - dt / 2)}
        d_ramp, _, _ = ramp(WARMUP, v)
        self.u_spawn = {k: self.target_u[k] - d_ramp for k in "AB"}
        ra = next(r for r in G.A_RULES if r[0] == job["a"])
        rb = next(r for r in G.B_RULES if r[0] == job["b"])
        self.ra, self.rb = ra, rb
        self.pol = {"A": G.Policy("A", ra, v), "B": G.Policy("B", rb, v, job.get("b_cont", "hold"))}
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.obs = {"A": mk(), "B": mk()}  # what A sees of B, what B sees of A
        self.last_stamp = {"A": None, "B": None}
        self.phase = "warmup"
        self.rec = None

    def place(self):
        for k, car in (("A", self.A), ("B", self.B)):
            self.x.place_at(k, car, self.u_spawn[k])

    def start_warmup(self):
        self.tw = self.x.ar.t

    def _perceive(self, t):
        for me, other, car in (("A", "B", self.B), ("B", "A", self.A)):
            uo = self.x.u(other, car)
            _, _, _, vo = car.state()
            seen = self.obs[me].push(t, uo, 0.0, vo)
            # from the run's start on, as the kinematic model, whose rules see the truth at t = 0
            if seen is not None and seen[0] >= -1e-9 and seen[0] != self.last_stamp[me]:
                self.last_stamp[me] = seen[0]
                self.pol[me].perceive(seen[0], seen[1], seen[3], seen[4])

    def step_warmup(self):
        t = self.x.ar.t - self.tw
        v = self.job["v"]
        for k, car in (("A", self.A), ("B", self.B)):
            d, vr, ar_ = ramp(t, v)
            u_ref = self.u_spawn[k] + d
            u = self.x.u(k, car)
            _, _, _, vv = car.state()
            a = max(-4.0, min(3.0, ar_ + 0.6 * (u_ref - u) + 1.8 * (vr - vv)))
            car.drive_line(a, self.x.line[k], self.x.dir[k])
        for k, other in (("A", self.B), ("B", self.A)):  # the observers fill; the policies start at t = 0
            self.obs[k].push(t - WARMUP, self.x.u("B" if k == "A" else "A", other), 0.0, other.state()[3])
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.x.ar.t
        self.u0 = {k: self.x.u(k, c) for k, c in (("A", self.A), ("B", self.B))}
        self.v0 = {k: c.state()[3] for k, c in (("A", self.A), ("B", self.B))}
        self.meas = M.Measures(b_max=(G.B_MAX, G.B_MAX))
        self.vhist = {"A": deque(), "B": deque()}
        self.peak = {"A": 0.0, "B": 0.0}
        self.seen_in = {"A": None, "B": None}
        self.stopped_both = 0.0
        self.both_stopped = False
        self.contact = False

    def step_run(self):
        t = self.x.ar.t - self.t0
        self._perceive(t)
        st_ = {}
        for k, car in (("A", self.A), ("B", self.B)):
            u = self.x.u(k, car)
            _, _, _, v = car.state()
            st_[k] = (u, v)
            a = self.pol[k].step(t, u, v)
            car.drive_line(a, self.x.line[k], self.x.dir[k])
            self.pol[k].after(t, u, v)
            h = self.vhist[k]
            h.append((t, v))
            while h and t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[k] = max(self.peak[k], (h[0][1] - v) / AR.PEAK_WINDOW)
            if u >= 0 and self.seen_in[k] is None:
                self.seen_in[k] = t
        (ua, va), (ub, vb) = st_["A"], st_["B"]
        if va < 0.1 and vb < 0.1 and ua < 0 and ub < 0:
            self.stopped_both += AR.DT
            self.both_stopped = self.both_stopped or self.stopped_both >= 0.5
        else:
            self.stopped_both = 0.0
        self.meas.push_poses(t, self.A.pose(), self.B.pose())
        if self.meas.clear <= 0.0:
            self.contact = True
        if self.contact or t >= G.T_MAX or (ua >= G.EXIT + 2 and ub >= G.EXIT + 2):
            self.finish(t, st_)

    def finish(self, t, st_):
        lost = []
        for k in "AB":
            u, _ = st_[k]
            if self.contact or u < G.EXIT:
                lost.append(1.0)
            else:
                v0 = self.v0[k]
                lost.append(min(1.0, max(0.0, (v0 * t - self.meas.s["AB".index(k)]) / max(v0 * G.HORIZON, 1.0))))
        ta = self.seen_in["A"] if self.seen_in["A"] is not None else math.inf
        tb = self.seen_in["B"] if self.seen_in["B"] is not None else math.inf
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
                    "ttc": round(o.ttc, 3) if math.isfinite(o.ttc) else "inf",
                    "clear": round(o.clear, 3),
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
                conts = ("yield", "hold") if b == "react" else (None,)
                for c in conts:
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
    ar = AR.Arena(args.port, args.k, AR.xodr_crossings(args.k, APPROACH, BEYOND, SPACING))
    rows = []
    t_start = time.monotonic()
    try:
        xs = [Crossing(ar, i) for i in range(args.k)]
        actors = []
        for x in xs:
            for k in "AB":
                d = x.dir[k]
                s = -100.0
                loc = carla.Location(x=x.centre[0] + d[0] * s, y=x.centre[1] + d[1] * s, z=x.z + 0.6)
                tf = carla.Transform(loc, carla.Rotation(yaw=math.degrees(math.atan2(d[1], d[0]))))
                a = ar.world.spawn_actor(ar.bp, tf)
                ar.actors.append(a)
                actors.append(a)
        ar.put_physics(actors)
        cars = [XCar(ar, a) for a in actors]
        while work:
            runs = []
            for i, x in enumerate(xs):
                if not work:
                    break
                r = Run(x, cars[2 * i], cars[2 * i + 1], work.popleft(), rng, args)
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
                        for k, c in (("A", r.A), ("B", r.B)):
                            c.drive_line(-3.0, r.x.line[k], r.x.dir[k])
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
    return rows, time.monotonic() - t_start


def compare(args, rows):
    """The arena's game per cell against the kinematic model's, on the rule subset."""
    by = {}
    for r in rows:
        key = (float(r["dt"]), r["a"], r["b"] if r["b"] != "react" else f"react:{r['b_cont']}")
        by.setdefault(key, []).append(r)
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    for dt in [float(x) for x in args.dts.split(",")]:
        kin = G.outcomes(dt, args.v, args.v)
        # the arena's mean outcome per pair as a payoff table, and the kinematic model's, on the subset
        ja_ar, jb_ar, ja_k, jb_k, res_k, res_ar = {}, {}, {}, {}, {}, {}
        for a in A_SUB:
            for b in B_SUB:
                if b == "react":  # B's continuation: its best reply, on each side's own payoffs
                    cands_ar = [(c, by[(dt, a, f"react:{c}")]) for c in ("yield", "hold")]
                    c_ar, rs = max(cands_ar, key=lambda x: st.mean(float(r["uB"]) for r in x[1]))
                    cands_k = [(c, kin[(a, f"react:{c}")]) for c in ("yield", "hold")]
                    c_k, ok_ = max(cands_k, key=lambda x: G.utility(replace(x[1], peak=x[1].peak_w), 1))
                else:
                    rs, ok_ = by[(dt, a, b)], kin[(a, b)]
                ja_ar[(a, b)] = st.mean(float(r["uA"]) for r in rs)
                jb_ar[(a, b)] = st.mean(float(r["uB"]) for r in rs)
                # scored as the arena scores: peak braking from the speed drop over 0.5 s (074 D2)
                ok_ = replace(ok_, peak=ok_.peak_w)
                ja_k[(a, b)], jb_k[(a, b)] = G.utility(ok_, 0), G.utility(ok_, 1)
                res_k[(a, b)], res_ar[(a, b)] = ok_, rs
                # D2: the kinematic payoff inside the arena's interval; the outcomes the same
                ok_all = True
                for mkey, kv in (("uA", ja_k[(a, b)]), ("uB", jb_k[(a, b)])):
                    xs = [float(r[mkey]) for r in rs]
                    mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                    ok = abs(kv - mu) <= max(2 * sd, 0.02)
                    inside += ok
                    total += 1
                    ok_all &= ok
                maj = lambda f: max(set(f(r) for r in rs), key=lambda x: sum(f(r) == x for r in rs))  # noqa: E731
                ar_first = maj(lambda r: r["first"])
                ar_contact = maj(lambda r: int(r["contact"]))
                ar_wait = maj(lambda r: int(r["both_stopped"]))
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
                firsts = {max(set(r["first"] for r in res_ar[p]), key=lambda x: sum(r["first"] == x for r in res_ar[p]))
                          for p in pure}
                contact = any(st.mean(int(r["contact"]) for r in res_ar[p]) >= 0.5 for p in pure)
                wait = any(st.mean(int(r["both_stopped"]) for r in res_ar[p]) >= 0.5 for p in pure)
            else:
                firsts = {res[p].first for p in pure}
                contact = any(res[p].contact for p in pure)
                wait = any(res[p].both_stopped for p in pure)
            c = "!" if contact else "W" if wait else "C" if len(firsts) > 1 else (firsts.pop() or "?") if firsts else (
                "M" if mixed else "?")
            cls[name] = (c, " ".join(f"{a}/{b}" for a, b in pure))
        summary["cells"].append({"dt": dt, "v": args.v, "class_arena": cls["arena"][0], "class_kinematic": cls["kinematic"][0],
                                 "eq_arena": cls["arena"][1], "eq_kinematic": cls["kinematic"][1]})
        lines.append(f"## dt {dt:+.1f}: class arena {cls['arena'][0]} ({cls['arena'][1]}) | kinematic {cls['kinematic'][0]} "
                     f"({cls['kinematic'][1]})")
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
    p.add_argument("--v", type=float, default=10.0)
    p.add_argument("--dts", default="-2,-1,-0.5,0,0.5,1,2")
    p.add_argument("--obs-delay", type=float, default=G.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--compare-only", action="store_true", help="re-run the comparison on <out>/runs.csv")
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
