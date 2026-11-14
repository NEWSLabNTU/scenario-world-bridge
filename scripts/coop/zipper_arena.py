#!/usr/bin/env python3
"""Roadmap 074 S3 in the arena: the lane drop with a queue, and its conventions, on CARLA's physics.

merge_arena's road (arena.xodr_lane_drop: lane -2 full width to X_END, then a TAPER m taper) with one
group per road: A in lane -2 and a queue of n cars (B, C, D) in lane -1 at time headway hf. The cars
are arena.py's (the spec's physics, acb's controls). All of them are stepped by
zipper_queue.QueueRules, the object the kinematic model steps. A sees each queue car, and the queue
sees A, through arena.Observer (10 Hz, delayed, noisy), held OBS + DECIDE s and predicted to now;
queue cars see each other as they are, as in the model. A's rules use its own lane-change plan;
the queue sees A's measured position.

A warm-up brings every car to the context (d_end, d0, dv, hf at vB) on reference trajectories. Each
run is scored as zipper_queue.simulate scores it (zipper_queue.utilities on the true poses). The run
ends at H once A is across and the last car is THRU_X past the lane's end, at contact, or when a car
nears the road's end (the model runs to T_MAX; such cuts are counted).

Per context, the game on a rule subset (A_SUB against the queue's GAME_Q) and the conventions are
solved and scored on the arena's mean payoffs and on zipper_queue's (with the car's command lags).
They are compared on D2's yardstick: payoffs inside the arena's 95 % interval (mean +- 2 sd over
seeds, at least +- 0.02), outcomes the same (A's slot, contact, a forced stop), and per convention
the cost, the part of the gap it recovers and the throughput.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/zipper_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--vb 12] [--n 3]
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
import merge_arena as MA  # noqa: E402
import merge_game as G  # noqa: E402
import zipper_queue as Z  # noqa: E402

ROAD_LEN, X_END, TAPER = 1200.0, MA.X_END, MA.TAPER  # long enough for T_MAX at 12 m/s past the end
WARMUP = MA.WARMUP
ROAD_GUARD = 40.0  # m: a run ends when any car's front is this close to the road's end
A_SUB = ("e0.6d", "e0.6f", "e1.2d", "z0.6d", "z0.6yd", "z1.2yf")
Q_SUB = ("hold", "c20", "c40", "zip", "yend")
GAME_Q = Z.GAME_Q
CONV = {"zipper": ("z0.6yd", "zip"), "zipper*": (None, "zip"), "early": ("e1.2d", "hold"), "yend": (None, "yend")}


class Group:
    def __init__(self, ar, road, cars, job, rng, args):
        self.ar, self.road, self.cars, self.job = ar, road, cars, job
        self.A, self.Q = cars[0], cars[1:]
        self.n = len(self.Q)
        self.yB, _ = ar.lane_y(road, -1)
        self.yA0, _ = ar.lane_y(road, -2)
        vb, dv, hf = job["vb"], job["dv"], job["hf"]
        self.va0 = max(0.0, vb + dv)
        xa_end = X_END - job["d_end"]  # A's front at the run's start
        xq = [xa_end - G.L - job["d0"] - i * (G.L + G.S0 + vb * hf) for i in range(self.n)]
        self.start_x = [xa_end] + xq
        self.v_ref = [self.va0] + [vb] * self.n
        self.y = [self.yA0] + [self.yB] * self.n
        ra = Z.a_rule(job["a"])
        self.R = Z.QueueRules(ra, job["q"], X_END, self.va0, vb, self.n)
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.seen_q = [MA.Seen(mk()) for _ in range(self.n)]  # what A sees of each queue car
        self.seen_a = MA.Seen(mk())  # what the queue sees of A
        self.phase, self.rec = "warmup", None

    def front(self, car):
        return car.state()[0] + car.length / 2

    def lat(self):
        return max(0.0, min(1.0, (self.A.state()[1] - self.yA0) / (self.yB - self.yA0)))

    def park_all(self):
        for car, x, v, y in zip(self.cars, self.start_x, self.v_ref, self.y):
            loc = car.a.get_transform().location
            x0 = x - MA.ramp(WARMUP, v)[0]
            car.park(carla.Transform(carla.Location(x=x0 - car.length / 2, y=y, z=loc.z + 0.3), carla.Rotation(yaw=0)))

    def start_warmup(self):
        self.tw = self.ar.t

    def step_warmup(self):
        t = self.ar.t - self.tw
        for car, x, v, y in zip(self.cars, self.start_x, self.v_ref, self.y):
            d, vr, ar_ = MA.ramp(t, v)
            x_ref = x - (MA.ramp(WARMUP, v)[0] - d)
            a = max(-4.0, min(3.0, ar_ + 0.6 * (x_ref - self.front(car)) + 1.8 * (vr - car.state()[3])))
            car.drive(a, y)
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.ar.t
        mk = lambda: M.Measures(b_max=(G.CAP.b_max, G.CAP.b_max))  # noqa: E731
        self.meas = {i: mk() for i in range(self.n)}
        self.qmeas = {i: mk() for i in range(1, self.n)}
        self.vh = [deque() for _ in self.cars]
        self.peak = [0.0] * len(self.cars)
        self.s = [0.0] * len(self.cars)
        self.s_h = [None] * len(self.cars)
        self.x_prev = [self.front(c) for c in self.cars]
        self.contact = self.ran_out = self.cut = False
        self.t_thru = math.inf
        self.real = {"d0_real": round((self.front(self.A) - self.A.length) - self.front(self.Q[0]), 2),
                     "dend_real": round(X_END - self.front(self.A), 2), "va_real": round(self.A.state()[3], 2),
                     "vb_real": round(self.Q[0].state()[3], 2)}

    def step_run(self):
        t = self.ar.t - self.t0
        xs = [self.front(c) for c in self.cars]
        vs = [c.state()[3] for c in self.cars]
        xa, va, xq, vq = xs[0], vs[0], xs[1:], vs[1:]
        lat = self.lat()
        for i in range(self.n):
            self.seen_q[i].push(t, xq[i], 0.0, vq[i])
        self.seen_a.push(t, xa, lat, va)
        sq = [self.seen_q[i].at(t, (xq[i], vq[i], 0.0, 0.0)) for i in range(self.n)]
        sa = self.seen_a.at(t, (xa, va, lat, 0.0))
        lat_plan = self.R.lat_of(t)[0]
        aa, aq = self.R.step(t, xa, va, lat_plan, xq, vq, [(s[0], s[1], s[3]) for s in sq], (sa[0], sa[1], sa[2]))
        lead = max(6.0, 0.7 * va) / max(va, 1.0)  # pure pursuit's point is look / v ahead on the path
        lat_ref, _ = self.R.lat_of(t + lead)
        self.A.drive(aa, self.yA0 + (self.yB - self.yA0) * lat_ref)
        for c, a in zip(self.Q, aq):
            c.drive(a, self.yB)
        self.R.after(xa, va)
        if (xa > X_END and lat_plan < 0.5) or (xa > X_END + TAPER and lat < 0.5):
            self.ran_out = True
        for k, vv in enumerate(vs):
            self.s[k] += max(0.0, xs[k] - self.x_prev[k])
            self.x_prev[k] = xs[k]
            if self.s_h[k] is None and t >= G.HORIZON - 1e-9:
                self.s_h[k] = self.s[k]
            h = self.vh[k]
            h.append((t, vv))
            while t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[k] = max(self.peak[k], (h[0][1] - vv) / AR.PEAK_WINDOW)
        pa = self.A.pose()
        pq = [c.pose() for c in self.Q]
        for i in range(self.n):
            self.meas[i].push_poses(t, pa, pq[i])
        for i in range(1, self.n):
            self.qmeas[i].push_poses(t, pq[i], pq[i - 1])
        if self.ran_out or any(m.clear <= 0.0 for m in list(self.meas.values()) + list(self.qmeas.values())):
            self.contact = True
        done_lat = lat_plan >= 1.0
        if math.isinf(self.t_thru) and done_lat and min(xs) >= X_END + Z.THRU_X:
            self.t_thru = t
        if max(xs) > ROAD_LEN - ROAD_GUARD:
            self.cut = True
        if (self.contact or self.cut or t >= Z.T_MAX - 1e-9
                or (t >= G.HORIZON - 1e-9 and done_lat and not math.isinf(self.t_thru))):
            self.finish(t, xa, xq, done_lat)

    def finish(self, t, xa, xq, merged):
        if self.contact or self.ran_out:
            for c in self.cars:
                c.dirty = True
        self.park_all()  # back at their start, at rest, before any can roll off the lane's or road's end
        u, lost = Z.utilities(self.contact, merged, self.v_ref, [self.s_h[k] if self.s_h[k] is not None else self.s[k]
                                                                for k in range(len(self.cars))],
                              self.meas, self.qmeas, self.peak)
        self.rec = {**self.job, **{f"u{k}": round(x, 4) for k, x in enumerate(u)}, "C": round(-sum(u), 4),
                    "slot": sum(1 for x in xq if x > xa) if merged else -1, "merged": int(merged),
                    "contact": int(self.contact), "stopped": int(self.R.stopped), "forced": int(self.R.forced),
                    "ran_out": int(self.ran_out), "cut": int(self.cut),
                    "t_thru": round(self.t_thru, 2) if math.isfinite(self.t_thru) else "inf",
                    "lostA": round(lost[0], 4), "peakA": round(self.peak[0], 2),
                    "clear_min": round(min(m.clear for m in list(self.meas.values()) + list(self.qmeas.values())), 3),
                    **self.real, "t_run": round(t, 2)}
        self.phase = "done"


def contexts(args):
    return list(itertools.product([float(x) for x in args.d_ends.split(",")], [float(x) for x in args.hfs.split(",")],
                                  [float(x) for x in args.dvs.split(",")], [float(x) for x in args.d0s.split(",")]))


def jobs(args):
    return [{"d_end": de, "hf": hf, "dv": dv, "d0": d0, "vb": args.vb, "n": args.n, "a": a, "q": q, "seed": s}
            for de, hf, dv, d0 in contexts(args) for a, q in itertools.product(A_SUB, Q_SUB) for s in range(args.seeds)]


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k, AR.xodr_lane_drop(args.k, ROAD_LEN, X_END, TAPER))
    spawn_x = [100.0] + [70.0 - 20.0 * i for i in range(args.n)]
    lanes = [-2] + [-1] * args.n
    rows, t_start, replaced = [], time.monotonic(), 0
    try:
        groups_cars = []
        for r in range(args.k):
            actors = [ar.spawn(r, ln, x) for ln, x in zip(lanes, spawn_x)]
            ar.put_physics(actors)
            groups_cars.append([AR.Car(ar, a) for a in actors])
        while work:
            groups = []
            for r in range(args.k):
                if not work:
                    break
                cars = groups_cars[r]
                for i, c in enumerate(cars):
                    # a car that left the ground or touched another is not to be trusted (CARLA can leave
                    # it stuck): a fresh one, its physics applied and checked
                    if not c.a.is_alive or getattr(c, "dirty", False):
                        if c.a.is_alive:
                            c.a.destroy()
                        a = ar.spawn(r, lanes[i], spawn_x[i])
                        ar.put_physics([a])
                        cars[i] = AR.Car(ar, a)
                        replaced += 1
                g = Group(ar, r, cars, work.popleft(), rng, args)
                g.park_all()
                groups.append(g)
            for _ in range(3):
                for g in groups:
                    for c in g.cars:
                        c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            for g in groups:
                g.start_warmup()
            while any(g.phase != "done" for g in groups):
                for g in groups:
                    if g.phase == "warmup":
                        g.step_warmup()
                    elif g.phase == "run":
                        g.step_run()
                    else:
                        for c in g.cars:
                            c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            rows += [g.rec for g in groups]
            print(f"  {len(rows)}/{n_all} runs, {time.monotonic() - t_start:.0f} s, {replaced} cars replaced", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return rows, time.monotonic() - t_start


def score(outs, a_acts):
    """outs[(a, q)] = (u list, C, slot, contact, stopped, t_thru): the game on a_acts x GAME_Q and the conventions."""
    ja = {(a, q): outs[(a, q)][0][0] for a in a_acts for q in GAME_Q}
    jq = {(a, q): sum(outs[(a, q)][0][1:]) for a in a_acts for q in GAME_Q}
    pure, mixed = X.solve(ja, jq, list(a_acts), list(GAME_Q))
    cost = {k: o[1] for k, o in outs.items()}
    c_opt = min(cost.values())
    eqc = [cost[p] for p in pure] + [sum(pa[a] * pq[q] * cost[(a, q)] for a in pa for q in pq) for pa, pq in mixed]
    c_worst, c_best = (max(eqc), min(eqc)) if eqc else (None, None)
    res = {"C_opt": c_opt, "C_worst": c_worst, "C_best": c_best, "pure": pure, "n_mixed": len(mixed), "conv": {}}
    for name, (a, q) in CONV.items():
        if a is None:
            a = max(a_acts, key=lambda x: outs[(x, q)][0][0])
        o = outs[(a, q)]
        rec = None if c_worst is None or c_worst - c_opt < Z.GAP_MIN else (c_worst - o[1]) / (c_worst - c_opt)
        res["conv"][name] = {"A": a, "C": o[1], "rec": rec, "slot": o[2], "contact": o[3], "stopped": o[4], "t_thru": o[5]}
    return res


def fmt(x, spec):
    return "-" if x is None else format(x, spec)


def finite(x):
    """inf and nan as None, recursively, for JSON."""
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {k: finite(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [finite(v) for v in x]
    return x


def compare(args, rows):
    lags = X.car_lags()
    n = args.n
    by = {}
    for r in rows:
        by.setdefault((float(r["d_end"]), float(r["hf"]), float(r["dv"]), float(r["d0"]), r["a"], r["q"]), []).append(r)
    maj = lambda rs, k: round(st.mean(int(r[k]) for r in rs))  # noqa: E731
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    for de, hf, dv, d0 in contexts(args):
        outs_a, outs_k = {}, {}
        for a, q in itertools.product(A_SUB, Q_SUB):
            rs = by[(de, hf, dv, d0, a, q)]
            ok = Z.simulate(Z.a_rule(a), q, de, d0, dv, args.vb, hf, n, lags)
            ua = [st.mean(float(r[f"u{k}"]) for r in rs) for k in range(n + 1)]
            slots = [int(r["slot"]) for r in rs]
            ts = [float(r["t_thru"]) for r in rs]
            outs_a[(a, q)] = (ua, -sum(ua), max(set(slots), key=slots.count), maj(rs, "contact"), maj(rs, "stopped"),
                              st.median(ts))
            outs_k[(a, q)] = (ok.u, -sum(ok.u), ok.slot, int(ok.contact), int(ok.stopped), ok.t_thru)
            ok_all = True
            for k in range(n + 1):
                xs = [float(r[f"u{k}"]) for r in rs]
                mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                hit = abs(ok.u[k] - mu) <= max(2 * sd, 0.02)
                inside += hit
                total += 1
                ok_all &= hit
            same = outs_a[(a, q)][2:5] == outs_k[(a, q)][2:5]
            agree += same
            n_out += 1
            lines.append(f"end {de:3g} hf {hf:3.1f} dv {dv:+2.0f} d0 {d0:+5.1f} {a:>6}/{q:<4} | arena C {outs_a[(a, q)][1]:6.3f} "
                         f"slot {outs_a[(a, q)][2]:2d} contact {outs_a[(a, q)][3]} stop {outs_a[(a, q)][4]} thru {outs_a[(a, q)][5]:5.1f} | "
                         f"kin C {outs_k[(a, q)][1]:6.3f} slot {ok.slot:2d} contact {int(ok.contact)} stop {int(ok.stopped)} "
                         f"thru {ok.t_thru:5.1f} | {'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cell = {"d_end": de, "hf": hf, "dv": dv, "d0": d0}
        for name, outs in (("arena", outs_a), ("kinematic", outs_k)):
            s = score(outs, A_SUB)
            cell[name] = {"C_opt": s["C_opt"], "C_worst": s["C_worst"], "C_best": s["C_best"], "pure": [f"{a}/{q}" for a, q in s["pure"]],
                          "n_mixed": s["n_mixed"], "conv": s["conv"]}
        summary["cells"].append(cell)
        f = lambda x: "  -  " if x is None else fmt(x, "5.2f")  # noqa: E731
        conv_s = lambda c: " ".join(  # noqa: E731
            f"{k}:{v['C']:.2f}{'!' if v['contact'] else 'S' if v['stopped'] else ''}({fmt(v['rec'], '+.2f')})"
            for k, v in c["conv"].items())
        lines.append(f"## end {de:g} hf {hf:g} dv {dv:+g} d0 {d0:+g}: arena opt {f(cell['arena']['C_opt'])} worst {f(cell['arena']['C_worst'])} "
                     f"| {conv_s(cell['arena'])} || kin opt {f(cell['kinematic']['C_opt'])} worst {f(cell['kinematic']['C_worst'])} "
                     f"| {conv_s(cell['kinematic'])}")
    # per convention: the arena against the model
    agg = {}
    for name in CONV:
        for side in ("arena", "kinematic"):
            cs = [c[side]["conv"][name] for c in summary["cells"]]
            recs = [x["rec"] for x in cs if x["rec"] is not None]
            ts = [x["t_thru"] for x in cs if math.isfinite(x["t_thru"])]
            agg[f"{name}:{side}"] = {"C": st.mean(x["C"] for x in cs), "rec": st.mean(recs) if recs else None, "n_rec": len(recs),
                                     "stopped": sum(x["stopped"] for x in cs), "contact": sum(x["contact"] for x in cs),
                                     "thru": st.mean(ts) if ts else None, "never": len(cs) - len(ts)}
    rank = lambda c, side: sorted(CONV, key=lambda k: c[side]["conv"][k]["C"])[0]  # noqa: E731
    best_agree = sum(rank(c, "arena") == rank(c, "kinematic") for c in summary["cells"])
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
                    "best_convention_agrees": best_agree, "per_convention": agg,
                    "cut_runs": sum(int(r["cut"]) for r in rows)})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); "
                 f"outcomes (slot, contact, stop) agree in {agree} of {n_out}; the cheapest convention agrees in "
                 f"{best_agree} of {len(summary['cells'])} contexts; runs cut at the road's end: {summary['cut_runs']}")
    for k, v in agg.items():
        lines.append(f"  {k:20} C {v['C']:.3f} recovers {fmt(v['rec'], '+.2f')} (n {v['n_rec']}) S {v['stopped']} "
                     f"! {v['contact']} thru {fmt(v['thru'], '.2f')} never {v['never']}")
    return lines, summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--vb", type=float, default=12.0)
    p.add_argument("--n", type=int, default=3)
    p.add_argument("--d-ends", default="40,80")
    p.add_argument("--hfs", default="0.8,1.5")
    p.add_argument("--dvs", default="0")
    p.add_argument("--d0s", default="-12,-5,2.5,10")
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
        json.dump(finite(summary), f, indent=1)
    print("\n".join(lines[-(len(summary["cells"]) + 1 + len(CONV) * 2):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
