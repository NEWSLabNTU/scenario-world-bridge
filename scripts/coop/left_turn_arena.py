#!/usr/bin/env python3
"""Roadmap 074 S5 in the arena: the unprotected left turn against an oncoming queue on CARLA's physics.

arena.xodr_left_turn: K junctions at grade, each a two-way main road with a side road leaving its
left side. One group per junction: A at rest at its stop line in lane -1, turning left along
left_turn_queue's arc (radius R_TURN) onto the side road; an oncoming chain of n cars in lane +1.
The cars are arena.py's (the spec's physics, acb's controls). Every car is stepped by
left_turn_queue.LTRules, the object the kinematic model steps, on what it perceives through
arena.Observer (10 Hz, delayed, noisy), held OBS + DECIDE s (the safety layer REACT s) and
predicted to now. A steers by pure pursuit at the arc's point one look-ahead on; its progress s
is its projection on the arc.

The chain warms up to v at its context's positions while A waits at its line. Each run is scored
as left_turn_queue.simulate scores it. Per context the game on a threshold subset (G_SUB x {hold,
yield}) is solved and classed on the arena's mean payoffs and on the model's (with the car's command
lags), and they are compared on D2's yardstick: payoffs inside the arena's 95 % interval, outcomes
(A first, contact, both stopped) and the class; PET at the conflict point against the 1.5 s
reference too.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/left_turn_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--v 10]
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
import crossing_arena as XA  # noqa: E402
import crossing_game as X  # noqa: E402
import left_turn_queue as T  # noqa: E402
import measures as M  # noqa: E402

MAIN_LEN, SIDE_LEN, GAP = 900.0, 300.0, 400.0
XJ = 500.0  # the model's x origin on the main road
X_SIDE = XJ - T.L / 2 + T.R_TURN - AR.LANE_W / 2  # the side road's reference line
WARMUP = XA.WARMUP
CONTACT_EPS = 0.01  # m (074 S4 D2)
LOOK_MIN, LOOK_GAIN = 3.0, 0.4  # pure pursuit (074 S4 D2)
G_SUB = (1.0, 2.0, 3.0, 3.5, 4.0, 5.0, 6.0)
CELLS = [(n, h) for n in (1, 2, 4) for h in (1.5, 2.5, 4.0)]


class Junction:
    """Model frame <-> CARLA on junction i: x_c = XJ + x, y_c = i GAP - y (OpenDRIVE's y flipped)."""

    def __init__(self, ar, i):
        self.ar, self.i = ar, i
        self.yc0 = i * GAP
        for lane, ym in ((-1, T.Y_A), (1, T.Y_C)):
            wp = ar.map.get_waypoint(carla.Location(x=XJ - 100.0, y=self.yc0 - ym, z=0.0), project_to_road=True,
                                     lane_type=carla.LaneType.Driving)
            if wp.lane_id != lane or abs(wp.transform.location.y - (self.yc0 - ym)) > 0.05:
                raise RuntimeError(f"junction {i}: lane {lane} expected at y {self.yc0 - ym:.2f}, got {wp.lane_id} at "
                                   f"{wp.transform.location.y:.2f}")
            self.z = wp.transform.location.z

    def to_c(self, x, y):
        return XJ + x, self.yc0 - y

    def to_m(self, xc, yc):
        return xc - XJ, self.yc0 - yc

    def place_a(self, car, s):
        x, y, yaw, _ = T.path(s)
        xc, yc = self.to_c(x, y)
        car.park(carla.Transform(carla.Location(x=xc, y=yc, z=self.z + 0.3), carla.Rotation(yaw=-math.degrees(yaw))))

    def place_c(self, car, x_front):
        xc, yc = self.to_c(x_front + car.length / 2, T.Y_C)
        car.park(carla.Transform(carla.Location(x=xc, y=yc, z=self.z + 0.3), carla.Rotation(yaw=180)))

    def c_front(self, car):
        return self.to_m(car.state()[0], 0.0)[0] - car.length / 2


def project(s_guess, x, y):
    """A's arc length nearest (x, y), searched around the last one."""
    best, bs = math.inf, s_guess
    for k in range(-40, 81):
        s = max(0.0, s_guess + 0.05 * k)
        px, py, _, _ = T.path(s)
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best:
            best, bs = d, s
    return bs


class Seen:
    def __init__(self, obs):
        self.obs, self.hist, self.last = obs, [], None

    def push(self, t, x, v):
        s = self.obs.push(t, x, 0.0, v)
        if s is not None and s[0] >= -1e-9 and s[0] != self.last:
            self.last = s[0]
            self.hist.append(s)

    def at(self, t, age, truth):
        h = next((h for h in reversed(self.hist) if h[0] <= t - age), self.hist[0] if self.hist else None)
        return truth if h is None else (h[1] + h[3] * (t - h[0]), h[3])


class Run:
    def __init__(self, j, a_car, c_cars, job, rng, args):
        self.j, self.A, self.C, self.job = j, a_car, c_cars, job
        n, h, v = job["n"], job["h"], job["v"]
        self.n, self.v = n, v
        self.R = T.LTRules(job["G"], job["c1"], n, v)
        self.x_start = [T.XZ_HI + v * (T.FIRST_T + i * h) for i in range(n)]
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.seen_c = [Seen(mk()) for _ in range(n)]  # A's view of each oncoming car (x, signed v along +x)
        self.seen_a = Seen(mk())  # the chain's view of A (s, v)
        self.phase, self.rec = "warmup", None
        self.s = 0.0

    def cars(self):
        return [self.A] + self.C

    def park_all(self):
        self.j.place_a(self.A, 0.0)
        d = XA.ramp(WARMUP, self.v)[0]
        for car, x in zip(self.C, self.x_start):
            self.j.place_c(car, x + d)

    def start_warmup(self):
        self.tw = self.j.ar.t

    def drive_c(self, car, a):
        _, yc = self.j.to_c(0.0, T.Y_C)
        car.drive_line(a, (0.0, yc), (-1.0, 0.0))

    def drive_a(self, a):
        xc, yc, yaw, v = self.A.state()
        look = max(LOOK_MIN, LOOK_GAIN * v)
        tx, ty, _, _ = T.path(self.s + look)
        txc, tyc = self.j.to_c(tx, ty)
        th, br = self.A.lon.command(a, v)
        th, br = self.A.lag.step(th, br, self.j.ar.t)
        alpha = math.atan2(tyc - yc, txc - xc) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        dist = max(math.hypot(txc - xc, tyc - yc), 0.5)
        angle = math.atan2(2 * self.A.wheelbase * math.sin(alpha), dist)
        self.j.ar.pending.append(carla.command.ApplyVehicleControl(self.A.a.id, carla.VehicleControl(
            throttle=th, brake=br, steer=self.A.steering.command(angle), hand_brake=a <= 0 and v < 0.05)))

    def step_warmup(self):
        t = self.j.ar.t - self.tw
        dr, vr, ar_ = XA.ramp(t, self.v)
        dist = XA.ramp(WARMUP, self.v)[0] - dr
        for car, x in zip(self.C, self.x_start):
            err = self.j.c_front(car) - (x + dist)
            self.drive_c(car, max(-4.0, min(3.0, ar_ + 0.6 * err + 1.8 * (vr - car.state()[3]))))
        self.A.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.j.ar.t
        mk = lambda: M.Measures(b_max=(T.CAP.b_max, T.CAP.b_max), pet_sample=X.PET_SAMPLE)  # noqa: E731
        self.meas = [mk() for _ in range(self.n)]
        self.vh = [deque() for _ in range(self.n + 1)]
        self.peaks = [0.0] * (self.n + 1)
        self.sc = [0.0] * self.n
        self.prev = [self.j.c_front(c) for c in self.C]
        self.s_h_a, self.s_h_c = None, None
        self.contact = self.both = False
        self.st_both = 0.0
        self.c1_in = self.a_out = None
        xr = [self.j.c_front(c) for c in self.C]
        self.real = {"x1_real": round(xr[0] - T.XZ_HI, 2), "v1_real": round(self.C[0].state()[3], 2),
                     "h_real": round((xr[1] - xr[0]) / max(self.v, 0.1), 3) if self.n > 1 else ""}

    def step_run(self):
        t = self.j.ar.t - self.t0
        xa_c, ya_c, _, va = self.A.state()
        xm, ym = self.j.to_m(xa_c, ya_c)
        self.s = project(self.s, xm, ym)
        x = [self.j.c_front(c) for c in self.C]
        vc = [c.state()[3] for c in self.C]
        for i in range(self.n):
            self.seen_c[i].push(t, x[i], -vc[i])
        self.seen_a.push(t, self.s, va)
        sc_ = []
        for i in range(self.n):
            px, pv = self.seen_c[i].at(t, T.OBS + T.DECIDE, (x[i], -vc[i]))
            sc_.append((px, -pv))
        aa = self.R.a_step(t, self.s, va, sc_)
        ac = self.R.c_step(t, x, vc, self.seen_a.at(t, T.REACT, (self.s, va)), self.seen_a.at(t, T.OBS + T.DECIDE, (self.s, va)))
        self.drive_a(aa)
        for car, a in zip(self.C, ac):
            self.drive_c(car, a)
        for i in range(self.n):
            self.sc[i] += abs(self.prev[i] - x[i])
            self.prev[i] = x[i]
        if self.s_h_a is None and t >= T.HORIZON - 1e-9:
            self.s_h_a, self.s_h_c = self.s, list(self.sc)
        for k, vv in enumerate([va] + vc):
            hq = self.vh[k]
            hq.append((t, vv))
            while t - hq[0][0] > AR.PEAK_WINDOW + 1e-9:
                hq.popleft()
            if t - hq[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peaks[k] = max(self.peaks[k], (hq[0][1] - vv) / AR.PEAK_WINDOW)
        kap = -T.path(self.s)[3]  # CARLA's y is flipped: a left turn's curvature is negative there
        pa = self.A.pose()
        for i in range(self.n):
            self.meas[i].push_poses(t, pa, self.C[i].pose(), kappa=(kap, 0.0))
        if any(m.clear <= CONTACT_EPS for m in self.meas):
            self.contact = True
        if self.c1_in is None and x[0] < T.XZ_HI:
            self.c1_in = t
        if self.a_out is None and self.s >= T.S_OUT:
            self.a_out = t
        if va < T.WAIT_V and vc[0] < T.WAIT_V and self.s < T.S_IN and x[0] > T.XZ_HI:
            self.st_both += AR.DT
            self.both = self.both or self.st_both >= 0.5
        else:
            self.st_both = 0.0
        done = t >= T.HORIZON - 1e-9 and self.s >= T.S_OUT + 10 and all(xx + T.L < T.XZ_LO for xx in x)
        cut = min(x) < -(XJ - 30) or self.s > T.STRAIGHT0 + T.R_TURN * math.pi / 2 + SIDE_LEN - 40
        if self.contact or done or cut or t >= T.T_MAX - 1e-9:
            self.finish(t, cut)

    def finish(self, t, cut):
        if self.contact:
            for c in self.cars():
                c.dirty = True
        if self.s_h_a is None:
            self.s_h_a, self.s_h_c = self.s, list(self.sc)
        u, lost, pet = T.score(self.n, self.v, self.contact, self.s_h_a, self.s_h_c, self.peaks, self.meas[0].lat_peak[0], self.meas)
        a_first = self.a_out is not None and (self.c1_in is None or self.a_out <= self.c1_in)
        self.rec = {**self.job, **{f"u{k}": round(x, 4) for k, x in enumerate(u)}, "C": round(-sum(u), 4),
                    "contact": int(self.contact), "a_first": int(a_first), "both_stopped": int(self.both),
                    "a_went": "" if self.R.started is None else round(self.R.started, 2),
                    "pet": round(pet, 3) if math.isfinite(pet) else "inf",
                    "clear": round(min(m.clear for m in self.meas), 3), "lostA": round(lost[0], 4),
                    "lat_peakA": round(self.meas[0].lat_peak[0], 2), "cut": int(cut), **self.real, "t_run": round(t, 2)}
        self.park_all()
        self.phase = "done"


def jobs(args):
    return [{"n": n, "h": h, "v": args.v, "G": G, "c1": c, "seed": s}
            for n, h in CELLS for G, c in itertools.product(G_SUB, ("hold", "yield")) for s in range(args.seeds)]


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    n_max = max(n for n, _ in CELLS)
    ar = AR.Arena(args.port, args.k, AR.xodr_left_turn(args.k, MAIN_LEN, X_SIDE, SIDE_LEN, GAP))
    rows, t_start, replaced = [], time.monotonic(), 0
    try:
        js = [Junction(ar, i) for i in range(args.k)]

        def spawn(j, k):
            if k == 0:
                x, y, yaw, _ = T.path(0.0)
                xc, yc = j.to_c(x - 30.0, y)
                rot = 0.0
            else:
                xc, yc = j.to_c(150.0 + 20.0 * k, T.Y_C)
                rot = 180.0
            a = ar.world.spawn_actor(ar.bp, carla.Transform(carla.Location(x=xc, y=yc, z=j.z + 0.6), carla.Rotation(yaw=rot)))
            ar.actors.append(a)
            return a

        group = []
        for j in js:
            acts = [spawn(j, k) for k in range(n_max + 1)]
            ar.put_physics(acts)
            group.append([XA.XCar(ar, a) for a in acts])
        while work:
            runs = []
            for gi, j in enumerate(js):
                if not work:
                    break
                cs = group[gi]
                for k, c in enumerate(cs):
                    if not c.a.is_alive or getattr(c, "dirty", False):
                        if c.a.is_alive:
                            c.a.destroy()
                        a = spawn(j, k)
                        ar.put_physics([a])
                        cs[k] = XA.XCar(ar, a)
                        replaced += 1
                job = work.popleft()
                r = Run(j, cs[0], cs[1:1 + job["n"]], job, rng, args)
                for k, c in enumerate(cs[1 + job["n"]:]):  # the cars this run does not use: parked out of the way
                    xc, yc = j.to_c(-300.0 - 10.0 * k, T.Y_C)
                    c.park(carla.Transform(carla.Location(x=xc, y=yc, z=j.z + 0.3), carla.Rotation(yaw=180)))
                r.park_all()
                r.idle = cs[1 + job["n"]:]
                runs.append(r)
            for _ in range(3):
                for r in runs:
                    for c in r.cars() + r.idle:
                        c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            for r in runs:
                r.start_warmup()
            while any(r.phase != "done" for r in runs):
                for r in runs:
                    for c in r.idle:
                        c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                    if r.phase == "warmup":
                        r.step_warmup()
                    elif r.phase == "run":
                        r.step_run()
                    else:
                        for c in r.cars():
                            c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            rows += [r.rec for r in runs]
            print(f"  {len(rows)}/{n_all} runs, {time.monotonic() - t_start:.0f} s, {replaced} cars replaced", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        keys = list(dict.fromkeys(k for r in rows for k in r))
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    return rows, time.monotonic() - t_start


def compare(args, rows):
    lags = X.car_lags()
    by = {}
    for r in rows:
        by.setdefault((int(r["n"]), float(r["h"]), float(r["G"]), r["c1"]), []).append(r)
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    pets = {"arena": [], "kinematic": []}
    for n, h in CELLS:
        oa, ok = {}, {}
        for G, c in itertools.product(G_SUB, ("hold", "yield")):
            rs = by[(n, h, G, c)]
            ko = T.simulate(G, c, n, h, args.v, lags)
            ua = [st.mean(float(r[f"u{k}"]) for r in rs) for k in range(n + 1)]
            maj = lambda key: round(st.mean(int(r[key]) for r in rs))  # noqa: E731
            pa_ = [float(r["pet"]) for r in rs]
            oa[(G, c)] = {"u": ua, "contact": maj("contact"), "a_first": maj("a_first"), "both": maj("both_stopped"),
                          "pet": st.median(pa_)}
            ok[(G, c)] = {"u": ko.u, "contact": int(ko.contact), "a_first": int(ko.a_first), "both": int(ko.both_stopped), "pet": ko.pet}
            pets["arena"] += pa_
            pets["kinematic"] += [ko.pet] * len(rs)
            hit_all = True
            for k in range(n + 1):
                xs = [float(r[f"u{k}"]) for r in rs]
                mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                hit = abs(ko.u[k] - mu) <= max(2 * sd, 0.02)
                inside += hit
                total += 1
                hit_all &= hit
            same = all(oa[(G, c)][k] == ok[(G, c)][k] for k in ("contact", "a_first", "both"))
            agree += same
            n_out += 1
            lines.append(f"n {n} h {h:3.1f} G {G:3.1f}/{c:<5} | arena uA {ua[0]:+.3f} uC1 {ua[1]:+.3f} C {-sum(ua):.3f} "
                         f"first {oa[(G, c)]['a_first']} contact {oa[(G, c)]['contact']} both {oa[(G, c)]['both']} PET {oa[(G, c)]['pet']:.2f} | "
                         f"kin uA {ko.u[0]:+.3f} uC1 {ko.u[1]:+.3f} C {-sum(ko.u):.3f} first {int(ko.a_first)} contact {int(ko.contact)} "
                         f"both {int(ko.both_stopped)} PET {ko.pet:.2f} | {'in' if hit_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cell = {"n": n, "h": h}
        for name, oo in (("arena", oa), ("kinematic", ok)):
            ja = {k: v["u"][0] for k, v in oo.items()}
            jc = {k: v["u"][1] for k, v in oo.items()}
            pure, mixed = X.solve(ja, jc, list(G_SUB), ["hold", "yield"])
            cell[f"class_{name}"] = T.klass(pure, mixed, lambda p, oo=oo: bool(oo[p]["contact"]))
            cell[f"eq_{name}"] = " ".join(f"{G:g}/{c}" for G, c in pure)
            cell[f"eq_pet_{name}"] = [round(oo[p]["pet"], 2) if math.isfinite(oo[p]["pet"]) else None for p in pure]
            cost = {k: -sum(v["u"]) for k, v in oo.items()}
            cell[f"C_opt_{name}"] = round(min(cost.values()), 3)
            cell[f"C_eq_{name}"] = [round(cost[p], 3) for p in pure]
        cell["class_U1"] = T.u1_class(n, h, args.v)[0]
        summary["cells"].append(cell)
        lines.append(f"## n {n} h {h:g}: class arena {cell['class_arena']} ({cell['eq_arena']}) | kinematic {cell['class_kinematic']} "
                     f"({cell['eq_kinematic']}) | U1 {cell['class_U1']}")
    ca = sum(c["class_arena"] == c["class_kinematic"] for c in summary["cells"])
    cu = sum(c["class_arena"] == c["class_U1"] for c in summary["cells"])
    below = {k: (sum(p < M.PET_REF for p in v), len(v)) for k, v in pets.items()}
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
                    "classes_agree_kinematic": ca, "classes_agree_U1": cu, "pet_below_ref": below})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); outcomes "
                 f"(A first, contact, both stopped) agree in {agree} of {n_out}; classes agree with the kinematic model in {ca} of "
                 f"{len(CELLS)}, with U1 in {cu}")
    lines.append("PET below 1.5 s (runs / pairs x seeds): " + ", ".join(f"{k} {a} of {b}" for k, (a, b) in below.items()))
    return lines, summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", type=float, default=10.0)
    p.add_argument("--obs-delay", type=float, default=X.OBS)
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
    print("\n".join(lines[-(len(CELLS) + 2):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
