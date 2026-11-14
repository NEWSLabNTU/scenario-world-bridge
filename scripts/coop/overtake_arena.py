#!/usr/bin/env python3
"""Roadmap 074 S6 in the arena: overtaking via the oncoming lane on CARLA's physics.

arena.xodr_two_way: K parallel two-way roads (lane -1 along +x, lane +1 along -x, a 1.5 m strip
outside each). One group per road: L slow in lane -1 (cruise at overtake_game.V_L), A behind it, B
oncoming in lane +1. The cars are arena.py's (the spec's physics, acb's controls). A and B are
stepped by overtake_game.RulesA / RulesB, the objects the kinematic model steps, on what each
perceives of the other through arena.Observer (10 Hz, delayed, noisy), held OBS + DECIDE s and
predicted to now. A's misjudgement of B's speed is the observer's noise plus overtake_game's bias
option (fading as B closes). A steers by pure pursuit at its lane change's point one look-ahead
ahead; B steers onto its shoulder when its rule moves right.

A warm-up brings the three cars to the context (g, vB, tm). Each run is scored as
overtake_game.simulate scores it. Per context, the game on a rule subset (A_SUB x B_SUB) is
solved and classed on the arena's mean payoffs and on the model's (with the car's command lags), and
the two are compared on D2's yardstick: payoffs inside the arena's 95 % interval, outcomes the
same (passed, contact, aborted), and the class. The commit point is compared too: where A was,
relative to L's front, when its predicted margin first went negative (RulesA.neg_rel); over 0, the
danger showed only after A was past L's front, and an abort could no longer drop back.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/overtake_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--biases 0,-0.3]
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
import overtake_game as O  # noqa: E402

ROAD_LEN = 1500.0
X_A0 = 300.0  # A's front at the run's start
WARMUP = XA.WARMUP
CONTACT_EPS = 0.01  # m: CARLA's collision shape reaches past the bounding box (074 S4 D2)
LOOK_MIN, LOOK_GAIN = 3.0, 0.4  # A's pure pursuit (074 S4 D2)
A_SUB = ("wait", "go", "g0.5", "g1.5", "g0.5a", "g1.5a")
B_SUB = ("hold", "brake", "right", "speedup")
# the chicken band of each (g, vB) (overtake_game's map), and cells either side of it
CELLS = [(10.0, 15.0, tm) for tm in (5.0, 6.0, 6.5, 7.0, 8.0, 10.0)] + \
        [(10.0, 22.0, tm) for tm in (5.0, 6.0, 6.5, 7.0, 8.0, 10.0)] + \
        [(25.0, 15.0, tm) for tm in (7.0, 8.0, 9.0, 10.0, 11.5, 13.0)] + \
        [(25.0, 22.0, tm) for tm in (7.0, 8.0, 8.5, 9.0, 11.0, 13.0)]


class Road:
    def __init__(self, ar, i):
        self.ar = ar
        y_ref = i * AR.ROAD_GAP
        self.y = {}
        for k, lane, guess in (("A", -1, y_ref + AR.LANE_W / 2), ("B", 1, y_ref - AR.LANE_W / 2)):
            wp = ar.map.get_waypoint(carla.Location(x=100.0, y=guess, z=0.0), project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp.lane_id != lane:
                raise RuntimeError(f"road {i}: expected lane {lane} near y {guess:.2f}, got {wp.lane_id}")
            self.y[k] = wp.transform.location.y
            self.z = wp.transform.location.z
        self.y["L"] = self.y["A"]
        self.right_b = math.copysign(1.0, self.y["B"] - self.y["A"])  # B's right: away from A's lane

    def place(self, car, k, x_front):
        if k == "B":
            tf = carla.Transform(carla.Location(x=x_front + car.length / 2, y=self.y["B"], z=self.z + 0.3), carla.Rotation(yaw=180))
        else:
            tf = carla.Transform(carla.Location(x=x_front - car.length / 2, y=self.y[k], z=self.z + 0.3), carla.Rotation(yaw=0))
        car.park(tf)


def pursue(ar, car, a, y_target_at):
    """Pure pursuit along +x with LOOK_MIN / LOOK_GAIN, the target's y from y_target_at(look)."""
    x, y, yaw, v = car.state()
    look = max(LOOK_MIN, LOOK_GAIN * v)
    th, br = car.lon.command(a, v)
    th, br = car.lag.step(th, br, ar.t)
    alpha = math.atan2(y_target_at(look, v) - y, look) - yaw
    angle = math.atan2(2 * car.wheelbase * math.sin(alpha), look)
    ar.pending.append(carla.command.ApplyVehicleControl(car.a.id, carla.VehicleControl(
        throttle=th, brake=br, steer=car.steering.command(angle))))


class Seen:
    """One car's perception of another (arena.Observer's samples), held OBS + DECIDE s."""

    def __init__(self, obs):
        self.obs, self.hist, self.last = obs, [], None

    def push(self, t, x, lat, v):
        s = self.obs.push(t, x, lat, v)
        if s is not None and s[0] >= -1e-9 and s[0] != self.last:
            self.last = s[0]
            self.hist.append(s)  # (stamp, x, lat, v, acc); v signed along +x

    def at(self, t):
        return next((h for h in reversed(self.hist) if h[0] <= t - O.OBS - O.DECIDE), self.hist[0] if self.hist else None)


class Run:
    def __init__(self, road, cars, job, rng, args):
        self.road, self.job = road, job
        self.L, self.A, self.B = cars
        g, vb, tm = job["g"], job["vb"], job["tm"]
        self.vb0 = vb
        self.start_x = {"A": X_A0, "L": X_A0 + g + O.L, "B": X_A0 + tm * (O.V_L + vb)}
        self.v_ref = {"A": O.V_L, "L": O.V_L, "B": vb}
        self.RA, self.RB = O.RulesA(job["a"]), O.RulesB(job["b"], vb)
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.seen_b, self.seen_a = Seen(mk()), Seen(mk())
        self.bias = job["bias"]
        self.phase, self.rec = "warmup", None

    def cars(self):
        return (("L", self.L), ("A", self.A), ("B", self.B))

    def front(self, k, car):
        x = car.state()[0]
        return x - car.length / 2 if k == "B" else x + car.length / 2

    def park_all(self):
        for k, car in self.cars():
            d = XA.ramp(WARMUP, self.v_ref[k])[0]
            self.road.place(car, k, self.start_x[k] + (d if k == "B" else -d))

    def start_warmup(self):
        self.tw = self.road.ar.t

    def step_warmup(self):
        t = self.road.ar.t - self.tw
        for k, car in self.cars():
            v = self.v_ref[k]
            d, vr, ar_ = XA.ramp(t, v)
            dist = XA.ramp(WARMUP, v)[0] - d  # still to go to the start point
            pos = self.front(k, car)
            err = (self.start_x[k] - dist - pos) if k != "B" else (pos - (self.start_x[k] + dist))
            a = max(-4.0, min(3.0, ar_ + 0.6 * err + 1.8 * (vr - car.state()[3])))
            if k == "B":
                car.drive_line(a, (0.0, self.road.y["B"]), (-1.0, 0.0))
            else:
                car.drive(a, self.road.y[k])
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.road.ar.t
        mk = lambda: M.Measures(b_max=(O.CAP.b_max, O.CAP.b_max), pet_sample=X.PET_SAMPLE)  # noqa: E731
        self.mab, self.mal = mk(), mk()
        self.vh = {"A": deque(), "B": deque()}
        self.peak = {"A": 0.0, "B": 0.0}
        self.s = {"A": 0.0, "B": 0.0}
        self.s_h = {"A": None, "B": None}
        self.prev = {k: self.front(k, c) for k, c in self.cars()}
        self.contact, self.who, self.cut = False, "", False
        self.real = {"g_real": round(self.front("L", self.L) - O.L - self.front("A", self.A), 2),
                     "vA_real": round(self.A.state()[3], 2), "vB_real": round(self.B.state()[3], 2),
                     "tm_real": round((self.front("B", self.B) - self.front("A", self.A)) / max(self.A.state()[3] + self.B.state()[3], 0.1), 3)}

    def lat_a(self):
        y = self.A.state()[1]
        return max(0.0, min(1.0, (y - self.road.y["A"]) / (self.road.y["B"] - self.road.y["A"])))

    def step_run(self):
        t = self.road.ar.t - self.t0
        ar = self.road.ar
        xa, xl, xb = self.front("A", self.A), self.front("L", self.L), self.front("B", self.B)
        va, vl, vb = self.A.state()[3], self.L.state()[3], self.B.state()[3]
        lat = self.lat_a()
        self.seen_b.push(t, xb, 0.0, -vb)
        self.seen_a.push(t, xa, lat, va)
        hb, ha = self.seen_b.at(t), self.seen_a.at(t)
        if hb is None:
            sb = (xb, vb)
        else:
            age = t - hb[0]
            vs = -hb[3] * (1.0 + self.bias * min(1.0, max(0.0, hb[1] - xa) / O.BIAS_D))
            sb = (hb[1] - vs * age, vs)
        sa = (xa, lat) if ha is None else (ha[1] + ha[3] * (t - ha[0]), ha[2])
        aa = self.RA.step(xa, va, xl, vl, sb)
        ab = self.RB.step(t, xb, vb, sa)
        u, state = self.RA.u, self.RA.state

        def y_a(look, v):
            lead = look / max(v, 1.0) / O.T_LC
            uf = u + lead if state == "out" else (u - lead if state in ("back", "abort_back") else u)
            return self.road.y["A"] + (self.road.y["B"] - self.road.y["A"]) * O.cos_lat(uf)[0]

        pursue(ar, self.A, aa, y_a)
        off = self.RB.offset(t)
        self.B.drive_line(ab, (0.0, self.road.y["B"] + self.road.right_b * off), (-1.0, 0.0))
        self.L.drive(max(-1.0, min(3.0, 1.5 * (O.V_L - vl))), self.road.y["L"])
        self.RA.lateral(X.DT)
        for k, car in (("A", self.A), ("B", self.B)):
            pos = self.front(k, car)
            self.s[k] += abs(pos - self.prev[k])
            self.prev[k] = pos
            if self.s_h[k] is None and t >= O.HORIZON - 1e-9:
                self.s_h[k] = self.s[k]
            v = car.state()[3]
            h = self.vh[k]
            h.append((t, v))
            while t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[k] = max(self.peak[k], (h[0][1] - v) / AR.PEAK_WINDOW)
        pa = self.A.pose()
        self.mab.push_poses(t, pa, self.B.pose())
        self.mal.push_poses(t, pa, self.L.pose())
        if self.mab.clear <= CONTACT_EPS or self.mal.clear <= CONTACT_EPS:
            self.contact, self.who = True, "B" if self.mab.clear <= CONTACT_EPS else "L"
        if max(xa, xl) > ROAD_LEN - 30 or xb < 30:
            self.cut = True
        if (self.contact or self.cut or t >= O.T_MAX - 1e-9
                or (t >= O.HORIZON - 1e-9 and xb < xa - 30 and self.RA.state in ("behind", "done", "behind_done"))):
            self.finish(t)

    def finish(self, t):
        if self.contact:
            for _, c in self.cars():
                c.dirty = True
        lost = tuple(1.0 if self.contact else M.lost_progress(v0, self.s_h[k] if self.s_h[k] is not None else self.s[k])
                     for k, v0 in (("A", O.V_DES), ("B", self.vb0)))
        r_ab = M.rho(self.mab.ttc2d, self.mab.thw, math.inf, self.mab.clear)
        r_al = M.rho(self.mal.ttc2d, self.mal.thw, math.inf, self.mal.clear)
        ua = M.utility(lost[0], max(r_ab, r_al), self.peak["A"], O.CAP.b_max, 0.0, self.contact)
        ub = M.utility(lost[1], r_ab, self.peak["B"], O.CAP.b_max, 0.0, self.contact)
        gave = self.RB.t_trig is not None and self.job["b"] in ("brake", "right", "brakeright")
        self.rec = {**self.job, "uA": round(ua, 4), "uB": round(ub, 4), "passed": int(self.RA.passed and not self.contact),
                    "aborted": int(self.RA.aborted), "contact": int(self.contact), "who": self.who, "b_gave": int(gave),
                    "neg_rel": "" if self.RA.neg_rel is None else round(self.RA.neg_rel, 2), "cut": int(self.cut),
                    "lostA": round(lost[0], 4), "lostB": round(lost[1], 4), "peakA": round(self.peak["A"], 2),
                    "peakB": round(self.peak["B"], 2), "clear_ab": round(self.mab.clear, 3), "clear_al": round(self.mal.clear, 3),
                    **self.real, "t_run": round(t, 2)}
        self.park_all()
        self.phase = "done"


def jobs(args):
    biases = [float(x) for x in args.biases.split(",")]
    return [{"bias": bias, "g": g, "vb": vb, "tm": tm, "a": a, "b": b, "seed": s}
            for bias in biases for g, vb, tm in CELLS for a, b in itertools.product(A_SUB, B_SUB) for s in range(args.seeds)]


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k, AR.xodr_two_way(args.k, ROAD_LEN))
    rows, t_start, replaced = [], time.monotonic(), 0
    try:
        roads = [Road(ar, i) for i in range(args.k)]

        def spawn(rd, k):
            x = {"L": 200.0, "A": 150.0, "B": 1300.0}[k]
            tf = carla.Transform(carla.Location(x=x, y=rd.y[k], z=rd.z + 0.6), carla.Rotation(yaw=180 if k == "B" else 0))
            a = ar.world.spawn_actor(ar.bp, tf)
            ar.actors.append(a)
            return a

        group_cars = []
        for rd in roads:
            acts = [spawn(rd, k) for k in "LAB"]
            ar.put_physics(acts)
            group_cars.append([XA.XCar(ar, a) for a in acts])
        while work:
            runs = []
            for i, rd in enumerate(roads):
                if not work:
                    break
                cs = group_cars[i]
                for j, k in enumerate("LAB"):
                    if not cs[j].a.is_alive or getattr(cs[j], "dirty", False):
                        if cs[j].a.is_alive:
                            cs[j].a.destroy()
                        a = spawn(rd, k)
                        ar.put_physics([a])
                        cs[j] = XA.XCar(ar, a)
                        replaced += 1
                r = Run(rd, cs, work.popleft(), rng, args)
                r.park_all()
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


def klass(pure, mixed, passed, contact, gave):
    if any(contact(p) for p in pure):
        return "!"
    ps = {passed(p) for p in pure}
    if ps == {True}:
        return "P"
    if ps == {False}:
        return "W"
    if ps:
        chicken = any(passed(p) and gave(p) for p in pure) and any(not passed(p) and p[1] == "hold" for p in pure)
        return "C" if chicken else "X"
    return "M" if mixed else "?"


def compare(args, rows):
    lags = X.car_lags()
    by = {}
    for r in rows:
        by.setdefault((float(r["bias"]), float(r["g"]), float(r["vb"]), float(r["tm"]), r["a"], r["b"]), []).append(r)
    biases = [float(x) for x in args.biases.split(",")]
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    commit = {"arena": [], "kinematic": []}  # neg_rel of every pass whose margin went negative
    abort_tab = {}  # (bias, rule) -> [contacts arena, contacts kin, aborts arena, aborts kin, runs]
    for bias in biases:
        for g, vb, tm in CELLS:
            ja, jb, ka, kb, ar_o, kin_o = {}, {}, {}, {}, {}, {}
            for a, b in itertools.product(A_SUB, B_SUB):
                rs = by[(bias, g, vb, tm, a, b)]
                ko = O.simulate(a, b, g, vb, tm, lags, bias)
                ja[(a, b)] = st.mean(float(r["uA"]) for r in rs)
                jb[(a, b)] = st.mean(float(r["uB"]) for r in rs)
                ka[(a, b)], kb[(a, b)] = ko.u
                maj = lambda key: round(st.mean(int(r[key]) for r in rs))  # noqa: E731
                ar_o[(a, b)] = {"passed": maj("passed"), "contact": maj("contact"), "aborted": maj("aborted"), "gave": maj("b_gave")}
                kin_o[(a, b)] = {"passed": int(ko.passed), "contact": int(ko.contact), "aborted": int(ko.aborted), "gave": int(ko.b_gave)}
                ok_all = True
                for key, kv in (("uA", ko.u[0]), ("uB", ko.u[1])):
                    xs = [float(r[key]) for r in rs]
                    mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                    hit = abs(kv - mu) <= max(2 * sd, 0.02)
                    inside += hit
                    total += 1
                    ok_all &= hit
                same = all(ar_o[(a, b)][k] == kin_o[(a, b)][k] for k in ("passed", "contact", "aborted"))
                agree += same
                n_out += 1
                for r in rs:
                    if r["neg_rel"] != "":
                        commit["arena"].append(float(r["neg_rel"]))
                if ko.neg_rel is not None:
                    commit["kinematic"].extend([ko.neg_rel] * len(rs))
                t_ = abort_tab.setdefault((bias, a), [0, 0, 0, 0, 0])
                t_[0] += sum(int(r["contact"]) for r in rs)
                t_[1] += int(ko.contact) * len(rs)
                t_[2] += sum(int(r["aborted"]) for r in rs)
                t_[3] += int(ko.aborted) * len(rs)
                t_[4] += len(rs)
                lines.append(f"bias {bias:+.1f} g {g:3g} vB {vb:3g} tm {tm:4.1f} {a:>6}/{b:<8} | arena uA {ja[(a, b)]:+.3f} uB {jb[(a, b)]:+.3f} "
                             f"{ar_o[(a, b)]} | kin uA {ko.u[0]:+.3f} uB {ko.u[1]:+.3f} {kin_o[(a, b)]} | "
                             f"{'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
            cell = {"bias": bias, "g": g, "vb": vb, "tm": tm}
            for name, ua, ub, oo in (("arena", ja, jb, ar_o), ("kinematic", ka, kb, kin_o)):
                pure, mixed = X.solve(ua, ub, list(A_SUB), list(B_SUB))
                cell[f"class_{name}"] = klass(pure, mixed, lambda p, oo=oo: bool(oo[p]["passed"]), lambda p, oo=oo: bool(oo[p]["contact"]),
                                              lambda p, oo=oo: bool(oo[p]["gave"]))
                cell[f"eq_{name}"] = " ".join(f"{a}/{b}" for a, b in pure)
            summary["cells"].append(cell)
            lines.append(f"## bias {bias:+.1f} g {g:g} vB {vb:g} tm {tm:g}: class arena {cell['class_arena']} ({cell['eq_arena']}) | "
                         f"kinematic {cell['class_kinematic']} ({cell['eq_kinematic']})")
    cls_agree = sum(c["class_arena"] == c["class_kinematic"] for c in summary["cells"])
    after = {k: (sum(x > 0 for x in v), len(v), (st.median(v) if v else None)) for k, v in commit.items()}
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out, "classes_agree": cls_agree,
                    "commit_after_L_front": {k: {"after": a, "of": n, "median_rel_m": m} for k, (a, n, m) in after.items()},
                    "abort_table": {f"{b:+.1f}:{a}": v for (b, a), v in abort_tab.items()}})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); outcomes "
                 f"(passed, contact, aborted) agree in {agree} of {n_out}; classes agree in {cls_agree} of {len(summary['cells'])}")
    for k, (a, n, m) in after.items():
        lines.append(f"commit point, {k}: of {n} passes whose margin went negative, {a} first saw it past L's front "
                     f"(median A's front {'-' if m is None else f'{m:+.1f}'} m from L's)")
    lines.append("contacts / aborts per A rule over its runs (arena vs kinematic):")
    for (b, a), v in sorted(abort_tab.items()):
        lines.append(f"  bias {b:+.1f} {a:>6}: contact {v[0]}/{v[1]}, aborted {v[2]}/{v[3]}, of {v[4]}")
    return lines, summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--biases", default="0,-0.3")
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
    print("\n".join(lines[-(len(summary["cells"]) + 16):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
