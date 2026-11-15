#!/usr/bin/env python3
"""Roadmap 074 S4 in the arena: the narrow passage on CARLA's physics.

arena.xodr_narrow: K parallel two-way roads whose lane -1 (A's, along +x) is closed over W_n m at
X0, with a TAPER m cubic taper on each side; lane +1 (B's, along -x) is open. One pair per road. The
cars are arena.py's (the spec's physics, acb's controls). Each drives by crossing_game.Policy with
narrow_game's rules and zone (W_n + 2 Z_PAD), on what it perceives of the other through
arena.Observer (10 Hz, delayed, noisy). A steers by pure pursuit along narrow_game's swerve
(narrow_game.lat_a at the look-ahead point); B keeps its lane.

A warm-up brings both cars to speed v at v * (T_MEAN -+ dt / 2) before their ends of the zone, as
in the kinematic model. The run is scored like narrow_game.simulate (risk from measures.push_poses on
the true poses, peak braking from the speed drop over 0.5 s, contact, who entered first, both
stopped, deadlock, restart crash). The kinematic side is scored the same way, with the car's command
lags. Per cell, the game on a rule subset (SUB x SUB, both cars) is solved for r = none and r = B
on the arena's mean payoffs and on the model's, and compared on D2's yardstick: payoffs inside the
arena's 95 % interval, outcomes the same (first, contact, both stopped: the arena's majority), and
the class.

Usage (a CARLA of its own; this script makes it synchronous and puts it back; one W_n per world):
  scripts/coop/narrow_arena.py --port 2600 --out <dir> --wn 30 [--k 8] [--seeds 3] [--v 6,10]
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
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_arena as XA  # noqa: E402
import crossing_game as X  # noqa: E402
import measures as M  # noqa: E402
import narrow_game as N  # noqa: E402

ROAD_LEN, X0 = 1100.0, 520.0
WARMUP = XA.WARMUP
SUB = ("go", "t0.5p2", "t1.5yp5", "tinfp2", "flash", "yb2p5")
# CARLA's collision shape reaches past the bounding box: every run whose boxes came within 1 cm braked
# at over 8 m/s^2 against each other and stood stuck (074 S4 D2: the 20 "deadlocks"). Contact is a
# clearance of at most this
CONTACT_EPS = 0.01
# A's pure-pursuit look-ahead, max(LOOK_MIN, LOOK_GAIN v). arena.Car.drive's max(6, 0.7 v) left A up
# to 0.60-0.71 m off the swerve at 6-10 m/s; this one 0.28-0.31 m (mean 0.08); max(2.5, 0.3 v) oscillates
# at 10 m/s (074 S4 D2)
LOOK_MIN, LOOK_GAIN = 3.0, 0.4


class Passage:
    """Road i: the lanes' centre lines and each car's distance u past its own end of the zone."""

    def __init__(self, ar, i, wn):
        self.ar, self.i, self.wn = ar, i, wn
        self.wz = wn + 2 * N.Z_PAD
        self.z0, self.z1 = X0 - N.Z_PAD, X0 + wn + N.Z_PAD  # the zone along x
        y_ref = i * AR.ROAD_GAP  # CARLA y of the reference line (OpenDRIVE y flipped)
        self.y = {}
        for k, lane, guess in (("A", -1, y_ref + AR.LANE_W / 2), ("B", 1, y_ref - AR.LANE_W / 2)):
            wp = ar.map.get_waypoint(carla.Location(x=100.0, y=guess, z=0.0), project_to_road=True,
                                     lane_type=carla.LaneType.Driving)
            if wp.lane_id != lane:
                raise RuntimeError(f"road {i}: expected lane {lane} near y {guess:.2f}, got {wp.lane_id}")
            self.y[k] = wp.transform.location.y
            self.z = wp.transform.location.z

    def u(self, k, car):
        x = car.state()[0]
        return (x + car.length / 2 - self.z0) if k == "A" else (self.z1 - (x - car.length / 2))

    def y_a(self, x_centre):
        """A's swerve: narrow_game.lat_a mapped onto the two lanes' centre lines."""
        f = (N.lat_a(x_centre - self.z0, self.wz) + AR.LANE_W / 2) / AR.LANE_W
        return self.y["A"] + (self.y["B"] - self.y["A"]) * f

    def place_at(self, k, car, u):
        if k == "A":
            tf = carla.Transform(carla.Location(x=self.z0 + u - car.length / 2, y=self.y["A"], z=self.z + 0.3), carla.Rotation(yaw=0))
        else:
            tf = carla.Transform(carla.Location(x=self.z1 - u + car.length / 2, y=self.y["B"], z=self.z + 0.3), carla.Rotation(yaw=180))
        car.park(tf)

    def drive(self, k, car, a):
        if k == "A":
            x, y, yaw, v = car.state()
            look = max(LOOK_MIN, LOOK_GAIN * v)
            th, br = car.lon.command(a, v)
            th, br = car.lag.step(th, br, self.ar.t)
            # pure pursuit on the path's point one look-ahead on (arena.Car.drive, with its look-ahead a parameter)
            alpha = math.atan2(self.y_a(x + look) - y, look) - yaw
            angle = math.atan2(2 * car.wheelbase * math.sin(alpha), look)
            self.ar.pending.append(carla.command.ApplyVehicleControl(car.a.id, carla.VehicleControl(
                throttle=th, brake=br, steer=car.steering.command(angle))))
        else:
            car.drive_line(a, (0.0, self.y["B"]), (-1.0, 0.0))


class Run:
    def __init__(self, p, a_car, b_car, job, rng, args):
        self.p, self.A, self.B, self.job = p, a_car, b_car, job
        v, dt = job["v"], job["dt"]
        self.exit = p.wz + X.L
        self.target_u = {"A": -v * (X.T_MEAN + dt / 2), "B": -v * (X.T_MEAN - dt / 2)}
        d_ramp = XA.ramp(WARMUP, v)[0]
        self.u_spawn = {k: self.target_u[k] - d_ramp for k in "AB"}
        self.pol = {}
        for k in "AB":
            role, rule, cont = N.rule_of(job[k.lower()])
            self.pol[k] = X.Policy(role, rule, v, cont, self.exit, hold_stop=X.ARENA_HOLD_STOP)
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.obs = {"A": mk(), "B": mk()}  # what A sees of B, what B sees of A
        self.last_stamp = {"A": None, "B": None}
        self.phase, self.rec = "warmup", None

    def cars(self):
        return (("A", self.A), ("B", self.B))

    def place(self):
        for k, car in self.cars():
            self.p.place_at(k, car, self.u_spawn[k])

    def start_warmup(self):
        self.tw = self.p.ar.t

    def step_warmup(self):
        t = self.p.ar.t - self.tw
        v = self.job["v"]
        for k, car in self.cars():
            d, vr, ar_ = XA.ramp(t, v)
            a = max(-4.0, min(3.0, ar_ + 0.6 * (self.u_spawn[k] + d - self.p.u(k, car)) + 1.8 * (vr - car.state()[3])))
            self.p.drive(k, car, a)
        for me, (ok, other) in (("A", ("B", self.B)), ("B", ("A", self.A))):  # the observers fill
            self.obs[me].push(t - WARMUP, self.p.u(ok, other), 0.0, other.state()[3])
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.p.ar.t
        self.u0 = {k: self.p.u(k, c) for k, c in self.cars()}
        self.v0 = {k: c.state()[3] for k, c in self.cars()}
        self.meas = M.Measures(b_max=(X.B_MAX, X.B_MAX), pet_sample=X.PET_SAMPLE)
        self.vhist = {"A": deque(), "B": deque()}
        self.peak = {"A": 0.0, "B": 0.0}
        self.seen_in = {"A": None, "B": None}
        self.stopped_both, self.both_stopped, self.contact, self.restart = 0.0, False, False, False

    def step_run(self):
        t = self.p.ar.t - self.t0
        for me, (ok, other) in (("A", ("B", self.B)), ("B", ("A", self.A))):
            seen = self.obs[me].push(t, self.p.u(ok, other), 0.0, other.state()[3])
            if seen is not None and seen[0] >= -1e-9 and seen[0] != self.last_stamp[me]:
                self.last_stamp[me] = seen[0]
                self.pol[me].perceive(seen[0], seen[1], seen[3], seen[4])
        st_ = {}
        for k, car in self.cars():
            u, v = self.p.u(k, car), car.state()[3]
            st_[k] = (u, v)
            self.p.drive(k, car, self.pol[k].step(t, u, v))
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
        if va < X.WAIT_V and vb < X.WAIT_V and ua < 0 and ub < 0:
            self.stopped_both += AR.DT
            self.both_stopped = self.both_stopped or self.stopped_both >= 0.5
        else:
            self.stopped_both = 0.0
        self.meas.push_poses(t, self.A.pose(), self.B.pose())
        if self.meas.clear <= CONTACT_EPS:
            self.contact = True
            self.restart = self.both_stopped
        if self.contact or t >= X.T_MAX - 1e-9 or (ua >= self.exit + N.TAPER and ub >= self.exit + N.TAPER and t >= X.HORIZON - 1e-9):
            self.finish(t, st_)

    def finish(self, t, st_):
        if self.contact:
            self.A.dirty = self.B.dirty = True
        lost = tuple(1.0 if self.contact or st_[k][0] < self.exit else M.lost_progress(self.v0[k], self.meas.s_h["AB".index(k)])
                     for k in "AB")
        ta = self.seen_in["A"] if self.seen_in["A"] is not None else math.inf
        tb = self.seen_in["B"] if self.seen_in["B"] is not None else math.inf
        first = "" if math.isinf(ta) and math.isinf(tb) else ("A" if ta < tb else "B")
        m = self.meas
        o = X.Outcome(lost, (self.peak["A"], self.peak["B"]), self.contact, m.ttc2d, m.thw, m._pet(), m.clear, first,
                      self.both_stopped, "", (self.pol["A"].shield, self.pol["B"].shield),
                      (self.pol["A"].rested, self.pol["B"].rested))
        deadlock = not self.contact and st_["A"][0] < self.exit and st_["B"][0] < self.exit
        self.rec = {**self.job, "first": first, "contact": int(self.contact), "both_stopped": int(self.both_stopped),
                    "deadlock": int(deadlock), "restart": int(self.restart),
                    "uA": round(X.utility(o, 0), 4), "uB": round(X.utility(o, 1), 4),
                    "uA_rB": round(X.utility(o, 0, "B"), 4), "uB_rB": round(X.utility(o, 1, "B"), 4),
                    "lostA": round(lost[0], 4), "lostB": round(lost[1], 4),
                    "peakA": round(self.peak["A"], 2), "peakB": round(self.peak["B"], 2),
                    "pet": round(o.pet, 3) if math.isfinite(o.pet) else "inf",
                    "ttc": round(o.ttc, 3) if math.isfinite(o.ttc) else "inf", "clear": round(o.clear, 3),
                    "uA0": round(self.u0["A"], 2), "uB0": round(self.u0["B"], 2),
                    "vA0": round(self.v0["A"], 2), "vB0": round(self.v0["B"], 2),
                    "dt_real": round(-self.u0["A"] / max(self.v0["A"], 0.1) + self.u0["B"] / max(self.v0["B"], 0.1), 3),
                    "t_run": round(t, 2), "shieldA": int(o.shield[0]), "shieldB": int(o.shield[1]),
                    "restedA": int(o.stopped[0]), "restedB": int(o.stopped[1])}
        for k, car in self.cars():  # back at the start, at rest
            self.p.place_at(k, car, self.u_spawn[k])
        self.rec.update({**self.pol["A"].clip_record("A"), **self.pol["B"].clip_record("B")})
        self.phase = "done"


def cells(args):
    return list(itertools.product([float(x) for x in args.v.split(",")], [float(x) for x in args.dts.split(",")]))


def jobs(args):
    return [{"wn": args.wn, "v": v, "dt": dt, "a": a, "b": b, "seed": s}
            for v, dt in cells(args) for a, b in itertools.product(SUB, SUB) for s in range(args.seeds)]


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k, AR.xodr_narrow(args.k, ROAD_LEN, X0, args.wn, N.TAPER))
    rows, t_start, replaced = [], time.monotonic(), 0
    try:
        ps = [Passage(ar, i, args.wn) for i in range(args.k)]

        def spawn(p, k):
            x = 200.0 if k == "A" else 900.0
            tf = carla.Transform(carla.Location(x=x, y=p.y[k], z=p.z + 0.6), carla.Rotation(yaw=0 if k == "A" else 180))
            a = ar.world.spawn_actor(ar.bp, tf)
            ar.actors.append(a)
            return a

        actors = [spawn(p, k) for p in ps for k in "AB"]
        ar.put_physics(actors)
        cars = [XA.XCar(ar, a) for a in actors]
        while work:
            runs = []
            for i, p in enumerate(ps):
                if not work:
                    break
                for j, k in ((2 * i, "A"), (2 * i + 1, "B")):
                    if not cars[j].a.is_alive or getattr(cars[j], "dirty", False):
                        if cars[j].a.is_alive:
                            cars[j].a.destroy()
                        a = spawn(p, k)
                        ar.put_physics([a])
                        cars[j] = XA.XCar(ar, a)
                        replaced += 1
                r = Run(p, cars[2 * i], cars[2 * i + 1], work.popleft(), rng, args)
                r.place()
                runs.append(r)
            for _ in range(3):
                for r in runs:
                    for _, c in r.cars():
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
                        for _, c in r.cars():
                            c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            rows += [r.rec for r in runs]
            print(f"  {len(rows)}/{n_all} runs, {time.monotonic() - t_start:.0f} s, {replaced} cars replaced", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return rows, time.monotonic() - t_start


def klass(pure, mixed, first, contact, wait):
    if any(contact(p) for p in pure):
        return "!"
    if any(wait(p) for p in pure):
        return "W"
    fs = {first(p) for p in pure}
    if len(fs) > 1:
        return "C"
    return (fs.pop() or "?") if fs else ("M" if mixed else "?")


def compare(args, rows):
    lags = X.car_lags()
    by = {}
    for r in rows:
        by.setdefault((float(r["v"]), float(r["dt"]), r["a"], r["b"]), []).append(r)
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    tallies = {k: {"arena": 0, "kinematic": 0} for k in ("contact", "both_stopped", "deadlock", "restart")}
    for v, dt in cells(args):
        res_ar, res_k, kin_out = {}, {}, {}
        u_ar = {"none": ({}, {}), "B": ({}, {})}
        u_k = {"none": ({}, {}), "B": ({}, {})}
        for a, b in itertools.product(SUB, SUB):
            rs = by[(v, dt, a, b)]
            ko = N.simulate(a, b, dt, v, v, args.wn, lags)
            ok_ = replace(ko.o, peak=ko.o.peak_w)  # scored as the arena scores (074 D2)
            kin_out[(a, b)] = ko
            res_ar[(a, b)], res_k[(a, b)] = rs, ok_
            for r, (ka, kb) in (("none", ("uA", "uB")), ("B", ("uA_rB", "uB_rB"))):
                u_ar[r][0][(a, b)] = st.mean(float(x[ka]) for x in rs)
                u_ar[r][1][(a, b)] = st.mean(float(x[kb]) for x in rs)
                u_k[r][0][(a, b)] = X.utility(ok_, 0, r)
                u_k[r][1][(a, b)] = X.utility(ok_, 1, r)
            ok_all = True
            for key, kv in (("uA", u_k["none"][0][(a, b)]), ("uB", u_k["none"][1][(a, b)])):
                xs = [float(x[key]) for x in rs]
                mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                hit = abs(kv - mu) <= max(2 * sd, 0.02)
                inside += hit
                total += 1
                ok_all &= hit
            maj = lambda f: max(set(f(x) for x in rs), key=lambda y: sum(f(x) == y for x in rs))  # noqa: E731
            ar_first, ar_contact, ar_wait = maj(lambda x: x["first"]), maj(lambda x: int(x["contact"])), maj(lambda x: int(x["both_stopped"]))
            same = ar_first == ok_.first and ar_contact == int(ok_.contact) and ar_wait == int(ok_.both_stopped)
            agree += same
            n_out += 1
            for k, kv in (("contact", int(ok_.contact)), ("both_stopped", int(ok_.both_stopped)), ("deadlock", int(ko.deadlock)),
                          ("restart", int(ko.restart))):
                tallies[k]["kinematic"] += kv
                tallies[k]["arena"] += maj(lambda x, k=k: int(x[k]))
            lines.append(f"v {v:3g} dt {dt:+4.1f} {a:>8}/{b:<8} | arena uA {u_ar['none'][0][(a, b)]:+.3f} uB {u_ar['none'][1][(a, b)]:+.3f} "
                         f"first {ar_first or '-'} contact {ar_contact} wait {ar_wait} | kin uA {u_k['none'][0][(a, b)]:+.3f} "
                         f"uB {u_k['none'][1][(a, b)]:+.3f} first {ok_.first or '-'} contact {int(ok_.contact)} "
                         f"wait {int(ok_.both_stopped)} | {'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cell = {"wn": args.wn, "v": v, "dt": dt}
        for r in ("none", "B"):
            pa, ma = X.solve(*u_ar[r], SUB, SUB)
            mj = lambda p, f: max(set(f(x) for x in res_ar[p]), key=lambda y: sum(f(x) == y for x in res_ar[p]))  # noqa: E731
            ca = klass(pa, ma, lambda p: mj(p, lambda x: x["first"]), lambda p: mj(p, lambda x: int(x["contact"])),
                       lambda p: mj(p, lambda x: int(x["both_stopped"])))
            pk, mk = X.solve(*u_k[r], SUB, SUB)
            ck = klass(pk, mk, lambda p: res_k[p].first, lambda p: res_k[p].contact, lambda p: res_k[p].both_stopped)
            cell[f"class_arena_{r}"], cell[f"class_kinematic_{r}"] = ca, ck
            cell[f"eq_arena_{r}"] = " ".join(f"{a}/{b}" for a, b in pa)
            cell[f"eq_kinematic_{r}"] = " ".join(f"{a}/{b}" for a, b in pk)
        summary["cells"].append(cell)
        lines.append(f"## v {v:g} dt {dt:+.1f}: class r none arena {cell['class_arena_none']} kin {cell['class_kinematic_none']} | "
                     f"r B arena {cell['class_arena_B']} kin {cell['class_kinematic_B']} || eq arena {cell['eq_arena_none']} | "
                     f"kin {cell['eq_kinematic_none']}")
    agree_cls = {r: sum(c[f"class_arena_{r}"] == c[f"class_kinematic_{r}"] for c in summary["cells"]) for r in ("none", "B")}
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
                    "classes_agree": agree_cls, "tallies": tallies})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); "
                 f"outcomes agree in {agree} of {n_out}; classes agree (r none) {agree_cls['none']}, (r B) {agree_cls['B']} "
                 f"of {len(summary['cells'])}")
    lines.append("rule pairs (arena majority / kinematic): " + ", ".join(
        f"{k} {v['arena']}/{v['kinematic']}" for k, v in tallies.items()))
    return lines, summary


def set_look(look_min, look_gain):
    global LOOK_MIN, LOOK_GAIN
    LOOK_MIN, LOOK_GAIN = look_min, look_gain


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--wn", type=float, required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", default="6,10")
    p.add_argument("--dts", default="-1.5,-0.7,-0.3,-0.1,0,0.3,1.5")
    p.add_argument("--obs-delay", type=float, default=X.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--look-min", type=float, default=LOOK_MIN)
    p.add_argument("--look-gain", type=float, default=LOOK_GAIN)
    p.add_argument("--compare-only", action="store_true")
    X.add_legacy_args(p)
    args = p.parse_args()
    X.apply_legacy_args(args)
    set_look(args.look_min, args.look_gain)

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
    print("\n".join(lines[-(len(summary["cells"]) + 2):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
