#!/usr/bin/env python3
"""Roadmap 074 U2 in the arena: the roundabout entry on CARLA's physics.

arena.xodr_roundabout: K roundabouts, each a one-lane ring (roundabout_game.R_RING) with one entry
whose lane follows roundabout_game's approach and entry arc. One group per ring: A at rest at its
yield line, B and up to two followers circulating. The cars are arena.py's (the spec's physics, acb's
controls). Every car is stepped by roundabout_game.RBRules, the object the kinematic model steps, on
what it perceives through arena.Observer (10 Hz, delayed, noisy), held OBS + DECIDE s (the safety
layer REACT s) and predicted to now. Cars steer by pure pursuit at their path's point one look-ahead
on (A along a_path, the ring cars along the ring); progress is each car's projection on its path.

The ring cars warm up to v at their context's positions while A is held at its line (re-parked
twice before the run: 074 S5's creep). Each run is scored as roundabout_game.simulate scores it.
Per context the game on a rule subset is solved and classed under no right of way and under B's at
faults 1 and 2, on the arena's mean payoffs and on the model's (with the car's command lags), and
the two are compared on D2's yardstick: payoffs inside the arena's 95 % interval, outcomes (A
ahead, A forced B, contact) and the class.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/roundabout_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--v 7]
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
import measures as M  # noqa: E402
import roundabout_game as G  # noqa: E402

SPACING = 200.0
WARMUP = XA.WARMUP
CONTACT_EPS = 0.01
LOOK_MIN, LOOK_GAIN = 3.0, 0.4
A_SUB = ("g1.5", "g2.5", "g3.5", "g5", "g8", "ginf", "g2.5c")
B_SUB = G.B_RULES
CELLS = [(n, 1.5, tb) for n in (1, 3) for tb in (1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0)]
RIGHTS = (("none", 0.0), ("B", 1.0), ("B", 2.0))


class Ring:
    """Model frame (the ring centred at the origin, OpenDRIVE's axes) <-> CARLA on ring i."""

    def __init__(self, ar, i):
        self.ar, self.i, self.ox = ar, i, i * SPACING
        x, y, _, _ = G.ring(0.0)
        xc, yc = self.to_c(x, y)
        wp = ar.map.get_waypoint(carla.Location(x=xc, y=yc, z=0.0), project_to_road=True, lane_type=carla.LaneType.Driving)
        if math.hypot(wp.transform.location.x - xc, wp.transform.location.y - yc) > 0.1:
            raise RuntimeError(f"ring {i}: lane centre {wp.transform.location} away from the model's ({xc:.2f}, {yc:.2f})")
        x, y, _, _ = G.a_path(G.S_YIELD)
        xc, yc = self.to_c(x, y)
        wp = ar.map.get_waypoint(carla.Location(x=xc, y=yc, z=0.0), project_to_road=True, lane_type=carla.LaneType.Driving)
        if math.hypot(wp.transform.location.x - xc, wp.transform.location.y - yc) > 0.1:
            raise RuntimeError(f"ring {i}: entry lane {wp.transform.location} away from the model's ({xc:.2f}, {yc:.2f})")
        self.z = wp.transform.location.z

    def to_c(self, x, y):
        return self.ox + x, -y

    def to_m(self, xc, yc):
        return xc - self.ox, -yc

    def park(self, car, pose):
        x, y, yaw, _ = pose
        xc, yc = self.to_c(x, y)
        car.park(carla.Transform(carla.Location(x=xc, y=yc, z=self.z + 0.3), carla.Rotation(yaw=-math.degrees(yaw))))

    def ring_s(self, car, s_prev):
        """A ring car's front arc length, unwrapped near its last."""
        xc, yc, _, _ = car.state()
        x, y = self.to_m(xc, yc)
        sc = G.R_RING * math.atan2(y, x)
        circ = 2 * math.pi * G.R_RING
        sc += circ * round((s_prev - G.L / 2 - sc) / circ)
        return sc + G.L / 2


def a_project(s_guess, x, y):
    best, bs = math.inf, s_guess
    for k in range(-40, 81):
        s = max(0.0, s_guess + 0.05 * k)
        px, py, _, _ = G.a_path(s)
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best:
            best, bs = d, s
    return bs


def pursue(ar, rg, car, a, target_xy, hand=False):
    xc, yc, yaw, v = car.state()
    txc, tyc = rg.to_c(*target_xy)
    th, br = car.lon.command(a, v)
    th, br = car.lag.step(th, br, ar.t)
    alpha = math.atan2(tyc - yc, txc - xc) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    dist = max(math.hypot(txc - xc, tyc - yc), 0.5)
    angle = math.atan2(2 * car.wheelbase * math.sin(alpha), dist)
    ar.pending.append(carla.command.ApplyVehicleControl(car.a.id, carla.VehicleControl(
        throttle=th, brake=br, steer=car.steering.command(angle), hand_brake=hand)))


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
    def __init__(self, rg, a_car, q_cars, job, rng, args):
        self.rg, self.A, self.Q, self.job = rg, a_car, q_cars, job
        n, h, v, tb = job["n"], job["h"], job["v"], job["tb"]
        self.n, self.v = n, v
        ra = next(r for r in G.A_RULES if r[0] == job["a"])
        self.R = G.RBRules(ra, job["b"], n, v)
        self.sq_start = [G.SZ0 - v * (tb + i * h) for i in range(n)]
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.seen_q = [Seen(mk()) for _ in range(n)]
        self.seen_a = Seen(mk())
        self.phase, self.rec = "warmup", None
        self.s = G.S_YIELD
        self.sq = list(self.sq_start)
        self.reparked = [False, False]

    def cars(self):
        return [self.A] + self.Q

    def park_all(self):
        self.rg.park(self.A, G.a_path(G.S_YIELD))
        d = XA.ramp(WARMUP, self.v)[0]
        for i, car in enumerate(self.Q):
            self.sq[i] = self.sq_start[i] - d
            self.rg.park(car, G.ring(self.sq[i] - G.L / 2))
        self.s = G.S_YIELD

    def start_warmup(self):
        self.tw = self.rg.ar.t

    def drive_q(self, i, a):
        _, _, _, v = self.Q[i].state()
        look = max(LOOK_MIN, LOOK_GAIN * v)
        x, y, _, _ = G.ring(self.sq[i] - G.L / 2 + look)
        pursue(self.rg.ar, self.rg, self.Q[i], a, (x, y))

    def drive_a(self, a):
        _, _, _, v = self.A.state()
        look = max(LOOK_MIN, LOOK_GAIN * v)
        x, y, _, _ = G.a_path(self.s + look)
        pursue(self.rg.ar, self.rg, self.A, a, (x, y), hand=a <= 0 and v < 0.05)

    def step_warmup(self):
        t = self.rg.ar.t - self.tw
        dr, vr, ar_ = XA.ramp(t, self.v)
        dist = XA.ramp(WARMUP, self.v)[0] - dr
        for i, car in enumerate(self.Q):
            self.sq[i] = self.rg.ring_s(car, self.sq[i])
            err = (self.sq_start[i] - dist) - self.sq[i]
            self.drive_q(i, max(-4.0, min(3.0, ar_ + 0.6 * err + 1.8 * (vr - car.state()[3]))))
        for k, tt in ((0, WARMUP - 3.0), (1, WARMUP - 1.0)):
            if not self.reparked[k] and t >= tt:
                self.rg.park(self.A, G.a_path(G.S_YIELD))
                self.reparked[k] = True
        self.A.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True, steer=0.0))
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.rg.ar.t
        mk = lambda: M.Measures(b_max=(G.CAP.b_max, G.CAP.b_max), pet_sample=X.PET_SAMPLE)  # noqa: E731
        self.meas = [mk() for _ in range(self.n)]
        self.vh = [deque() for _ in range(self.n + 1)]
        self.peaks = [0.0] * (self.n + 1)
        xa, ya, _, va = self.A.state()
        self.s = a_project(G.S_YIELD, *self.rg.to_m(xa, ya))
        self.sa0 = self.s
        self.sq = [self.rg.ring_s(c, s) for c, s in zip(self.Q, self.sq)]
        self.sq0 = list(self.sq)
        self.sa_h = self.sq_h = None
        self.contact = self.both = False
        self.st_both = 0.0
        self.real = {"a_off0": round(self.s - G.S_YIELD, 3), "a_v0": round(va, 3),
                     "tb_real": round((G.SZ0 - self.sq[0]) / max(self.Q[0].state()[3], 0.1), 3),
                     "vB_real": round(self.Q[0].state()[3], 2)}

    def step_run(self):
        t = self.rg.ar.t - self.t0
        xa, ya, _, va = self.A.state()
        self.s = a_project(self.s, *self.rg.to_m(xa, ya))
        vq = []
        for i, c in enumerate(self.Q):
            self.sq[i] = self.rg.ring_s(c, self.sq[i])
            vq.append(c.state()[3])
        for i in range(self.n):
            self.seen_q[i].push(t, self.sq[i], vq[i])
        self.seen_a.push(t, self.s, va)
        seen_q = [self.seen_q[i].at(t, G.OBS + G.DECIDE, (self.sq[i], vq[i])) for i in range(self.n)]
        aa = self.R.a_step(t, self.s, va, seen_q)
        aq = self.R.q_step(t, list(self.sq), vq, self.seen_a.at(t, G.REACT, (self.s, va)), self.seen_a.at(t, G.OBS + G.DECIDE, (self.s, va)))
        self.R.after(t, va)
        self.drive_a(aa)
        for i, a in enumerate(aq):
            self.drive_q(i, a)
        if self.sa_h is None and t >= G.HORIZON - 1e-9:
            self.sa_h, self.sq_h = self.s - self.sa0, [self.sq[i] - self.sq0[i] for i in range(self.n)]
        for k, vv in enumerate([va] + vq):
            hq = self.vh[k]
            hq.append((t, vv))
            while t - hq[0][0] > AR.PEAK_WINDOW + 1e-9:
                hq.popleft()
            if t - hq[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peaks[k] = max(self.peaks[k], (hq[0][1] - vv) / AR.PEAK_WINDOW)
        ka = -G.a_path(self.s)[3]  # CARLA's y is flipped: every curvature changes sign
        kq = -1.0 / G.R_RING
        pa = self.A.pose()
        for i in range(self.n):
            self.meas[i].push_poses(t, pa, self.Q[i].pose(), kappa=(ka, kq))
        if any(m.clear <= CONTACT_EPS for m in self.meas):
            self.contact = True
        if self.R.joined_ahead is None and self.s >= G.A_JOINED:
            self.R.joined_ahead = G.a_ring_s(self.s) + G.L / 2 > self.sq[0]
        if va < G.WAIT_V and vq[0] < G.WAIT_V and self.s < G.A_IN and self.sq[0] < G.SZ0:
            self.st_both += AR.DT
            self.both = self.both or self.st_both >= 0.5
        else:
            self.st_both = 0.0
        if self.contact or t >= G.T_MAX - 1e-9 or (t >= G.HORIZON - 1e-9 and self.s >= G.A_JOINED + 5):
            self.finish(t)

    def finish(self, t):
        if self.contact:
            for c in self.cars():
                c.dirty = True
        if self.sa_h is None:
            self.sa_h, self.sq_h = self.s - self.sa0, [self.sq[i] - self.sq0[i] for i in range(self.n)]
        lat = [self.meas[0].lat_peak[0]] + [m.lat_peak[1] for m in self.meas]
        merged = self.s >= G.A_JOINED
        u, lost = G.score(self.n, self.v, self.contact, merged, self.sa_h, self.sq_h, self.peaks, lat, self.meas)
        a_ahead = bool(self.R.joined_ahead) and not self.contact
        forced = a_ahead and (self.R.shield[0] or (self.peaks[1] > X.YIELD_SEEN and self.job["b"] != "slow"))
        pet = min(m._pet() for m in self.meas)
        self.rec = {**self.job, **{f"u{k}": round(x, 4) for k, x in enumerate(u)}, "contact": int(self.contact),
                    "a_ahead": int(a_ahead), "forced": int(forced), "both_stopped": int(self.both),
                    "a_went": "" if self.R.started is None else round(self.R.started, 2),
                    "pet": round(pet, 3) if math.isfinite(pet) else "inf", "clear": round(min(m.clear for m in self.meas), 3),
                    "lostA": round(lost[0], 4), "lat_peakA": round(lat[0], 2), "lat_peakB": round(lat[1], 2),
                    **self.real, "t_run": round(t, 2)}
        self.park_all()
        self.phase = "done"


def jobs(args):
    return [{"n": n, "h": h, "tb": tb, "v": args.v, "a": a, "b": b, "seed": s}
            for n, h, tb in CELLS for a, b in itertools.product(A_SUB, B_SUB) for s in range(args.seeds)]


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    n_max = max(n for n, _, _ in CELLS)
    xs = G.CE[1] - G.APPROACH
    ar = AR.Arena(args.port, args.k, AR.xodr_roundabout(args.k, SPACING, G.R_RING, G.X_APP, xs, G.APPROACH, G.R_ENTRY, G.ARC_LEN))
    rows, t_start, replaced = [], time.monotonic(), 0
    try:
        rgs = [Ring(ar, i) for i in range(args.k)]

        def spawn(rg, k):
            for tries in range(20):
                if k == 0:
                    x, y, yaw, _ = G.a_path(5.0 + 5.0 * tries)
                else:
                    x, y, yaw, _ = G.ring(G.SZ1 + 15.0 + 12.0 * k + 3.0 * tries)
                xc, yc = rg.to_c(x, y)
                a = ar.world.try_spawn_actor(ar.bp, carla.Transform(carla.Location(x=xc, y=yc, z=rg.z + 0.6),
                                                                    carla.Rotation(yaw=-math.degrees(yaw))))
                if a is not None:
                    ar.actors.append(a)
                    return a
            raise RuntimeError(f"ring {rg.i}: no free spawn point for car {k}")

        group = []
        for rg in rgs:
            acts = [spawn(rg, k) for k in range(n_max + 1)]
            ar.put_physics(acts)
            group.append([XA.XCar(ar, a) for a in acts])
        while work:
            runs = []
            for gi, rg in enumerate(rgs):
                if not work:
                    break
                cs = group[gi]
                for k, c in enumerate(cs):
                    if not c.a.is_alive or getattr(c, "dirty", False):
                        if c.a.is_alive:
                            c.a.destroy()
                        a = spawn(rg, k)
                        ar.put_physics([a])
                        cs[k] = XA.XCar(ar, a)
                        replaced += 1
                job = work.popleft()
                r = Run(rg, cs[0], cs[1:1 + job["n"]], job, rng, args)
                r.idle = cs[1 + job["n"]:]
                for k, c in enumerate(r.idle):  # cars this run does not use: on the entry road, behind A
                    rg.park(c, G.a_path(3.0 + 8.0 * k))
                r.park_all()
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
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader()
        w.writerows(rows)
    return rows, time.monotonic() - t_start


def compare(args, rows):
    lags = X.car_lags()
    by = {}
    for r in rows:
        by.setdefault((int(r["n"]), float(r["h"]), float(r["tb"]), r["a"], r["b"]), []).append(r)
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    cls_agree = {f"{r}{f:g}": 0 for r, f in RIGHTS}
    for n, h, tb in CELLS:
        oa, ok = {}, {}
        for a, b in itertools.product(A_SUB, B_SUB):
            rs = by[(n, h, tb, a, b)]
            ra = next(x for x in G.A_RULES if x[0] == a)
            ko = G.simulate(ra, b, n, h, args.v, tb, lags)
            ua = [st.mean(float(r[f"u{k}"]) for r in rs) for k in range(n + 1)]
            maj = lambda key: round(st.mean(int(r[key]) for r in rs))  # noqa: E731
            oa[(a, b)] = {"u": ua, "a_ahead": maj("a_ahead"), "forced": maj("forced"), "contact": maj("contact"),
                          "pet": st.median(float(r["pet"]) for r in rs)}
            ok[(a, b)] = {"u": ko.u, "a_ahead": int(ko.a_ahead), "forced": int(ko.forced), "contact": int(ko.contact), "pet": ko.pet}
            hit_all = True
            for k in range(n + 1):
                xs = [float(r[f"u{k}"]) for r in rs]
                mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                hit = abs(ko.u[k] - mu) <= max(2 * sd, 0.02)
                inside += hit
                total += 1
                hit_all &= hit
            same = all(oa[(a, b)][k] == ok[(a, b)][k] for k in ("a_ahead", "forced", "contact"))
            agree += same
            n_out += 1
            lines.append(f"n {n} tb {tb:3.1f} {a:>6}/{b:<5} | arena uA {ua[0]:+.3f} uB {ua[1]:+.3f} ahead {oa[(a, b)]['a_ahead']} "
                         f"forced {oa[(a, b)]['forced']} contact {oa[(a, b)]['contact']} PET {oa[(a, b)]['pet']:.2f} | kin uA {ko.u[0]:+.3f} "
                         f"uB {ko.u[1]:+.3f} ahead {int(ko.a_ahead)} forced {int(ko.forced)} contact {int(ko.contact)} PET {ko.pet:.2f} | "
                         f"{'in' if hit_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cell = {"n": n, "h": h, "tb": tb}
        for r, f in RIGHTS:
            key = f"{r}{f:g}"
            for name, oo in (("arena", oa), ("kinematic", ok)):
                ja = {p: v["u"][0] - (f if r == "B" and v["forced"] else 0.0) for p, v in oo.items()}
                jb = {p: v["u"][1] for p, v in oo.items()}
                pure, mixed = X.solve(ja, jb, list(A_SUB), list(B_SUB))
                if any(oo[p]["contact"] for p in pure):
                    c = "!"
                else:
                    ah = {oo[p]["a_ahead"] for p in pure}
                    c = "A" if ah == {1} else "B" if ah == {0} else "C" if ah else ("M" if mixed else "?")
                    c += "*" if any(oo[p]["forced"] for p in pure) else ""
                cell[f"class_{name}_{key}"] = c
                cell[f"eq_{name}_{key}"] = " ".join(f"{a}/{b}" for a, b in pure)
            cls_agree[key] += cell[f"class_arena_{key}"] == cell[f"class_kinematic_{key}"]
        summary["cells"].append(cell)
        lines.append(f"## n {n} tb {tb:g}: " + " | ".join(
            f"{r}{f:g}: arena {cell[f'class_arena_{r}{f:g}']} kin {cell[f'class_kinematic_{r}{f:g}']}" for r, f in RIGHTS))
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out, "classes_agree": cls_agree})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); outcomes "
                 f"(A ahead, forced, contact) agree in {agree} of {n_out}; classes agree " +
                 ", ".join(f"{k} {v} of {len(CELLS)}" for k, v in cls_agree.items()))
    return lines, summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", type=float, default=7.0)
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
    print("\n".join(lines[-(len(CELLS) + 1):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
