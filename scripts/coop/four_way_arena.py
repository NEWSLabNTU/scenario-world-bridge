#!/usr/bin/env python3
"""Roadmap 074 U3 in the arena: the four-way stop on CARLA's physics.

crossing_arena.py's arena (arena.xodr_crossings: K crossings at grade, two one-lane roads each, the
spec's physics and acb's controls, arena.Observer's perception), with four_way_game.FWPolicy --
the stop-sign rules and crossing_game's safety layer, the object the kinematic model steps -- in
place of S2's policies. The warm-up puts each car's front v * (T_MEAN +- dt / 2) before its stop
line when the run starts, as four_way_game.simulate does. A run is scored as there (with the
arena's peak braking), plus a deadlock (both cars still before the box at the end) and each
car's restarts.

Per dt the game on the rule subset (SUB x SUB) is solved on the arena's mean payoffs and on the
kinematic model's (with the car's command lags), and compared on D2's yardstick: each kinematic
payoff inside the arena's mean +- max(2 sd, 0.02), the outcomes (first, contact, deadlock) by
majority, and the class.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/four_way_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--v 6] [--dts -6,-3,...]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import statistics as st
import sys
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import crossing_arena as XA  # noqa: E402
import crossing_game as G  # noqa: E402
import four_way_game as F  # noqa: E402

SUB = ("aggr", "yield_c1.5", "stop1y_c1.5", "fcfs_c1.5", "fcfsr_c1.5", "rbl_c1.5")


class FWRun(XA.Run):
    def __init__(self, xing, a_car, b_car, job, rng, args):
        self.x, self.A, self.B, self.job, self.args = xing, a_car, b_car, job, args
        v, dt = job["v"], job["dt"]
        # fronts v * (T_MEAN +- dt / 2) before the stop line (four_way_game.simulate)
        self.target_u = {"A": G.STOP - v * (G.T_MEAN + dt / 2), "B": G.STOP - v * (G.T_MEAN - dt / 2)}
        d_ramp, _, _ = XA.ramp(XA.WARMUP, v)
        self.u_spawn = {k: self.target_u[k] - d_ramp for k in "AB"}
        rule = {r[0]: r for r in F.RULES}
        self.pol = {"A": F.FWPolicy("A", rule[job["a"]], v, hold_stop=G.ARENA_HOLD_STOP),
                    "B": F.FWPolicy("B", rule[job["b"]], v, hold_stop=G.ARENA_HOLD_STOP)}
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.obs = {"A": mk(), "B": mk()}
        self.last_stamp = {"A": None, "B": None}
        self.phase = "warmup"
        self.rec = None

    def finish(self, t, st_):
        super().finish(t, st_)
        self.rec["deadlock"] = int(not self.contact and st_["A"][0] < 0 and st_["B"][0] < 0)


def jobs(args):
    return [{"dt": dt, "v": args.v, "a": a, "b": b, "seed": s}
            for dt in [float(x) for x in args.dts.split(",")] for a in SUB for b in SUB for s in range(args.seeds)]


def compare(args, rows):
    lags = G.car_lags()
    rule = {r[0]: r for r in F.RULES}
    by = {}
    for r in rows:
        by.setdefault((float(r["dt"]), r["a"], r["b"]), []).append(r)
    lines, inside, total, agree, n_out, cells = [], 0, 0, 0, 0, []
    for dt in [float(x) for x in args.dts.split(",")]:
        ja_ar, jb_ar, ja_k, jb_k, outs_k, rs_ar = {}, {}, {}, {}, {}, {}
        for a in SUB:
            for b in SUB:
                rs = by[(dt, a, b)]
                ko = F.simulate(rule[a], rule[b], dt, args.v, args.v, lags)
                ok_ = replace(ko.o, peak=ko.o.peak_w)  # scored as the arena scores (074 D2)
                ja_ar[(a, b)] = st.mean(float(r["uA"]) for r in rs)
                jb_ar[(a, b)] = st.mean(float(r["uB"]) for r in rs)
                ja_k[(a, b)], jb_k[(a, b)] = G.utility(ok_, 0), G.utility(ok_, 1)
                outs_k[(a, b)], rs_ar[(a, b)] = ko, rs
                ok_all = True
                for key, kv in (("uA", ja_k[(a, b)]), ("uB", jb_k[(a, b)])):
                    xs = [float(r[key]) for r in rs]
                    mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                    hit = abs(kv - mu) <= max(2 * sd, 0.02)
                    inside += hit
                    total += 1
                    ok_all &= hit
                maj = lambda f: max(set(f(r) for r in rs), key=lambda x: sum(f(r) == x for r in rs))  # noqa: E731
                ar_o = (maj(lambda r: r["first"]), maj(lambda r: int(r["contact"])), maj(lambda r: int(r["deadlock"])))
                k_o = (ko.o.first, int(ko.o.contact), int(ko.deadlock))
                same = ar_o == k_o
                agree += same
                n_out += 1
                lines.append(f"dt {dt:+4.1f} {a:>11}/{b:<11} | arena uA {ja_ar[(a, b)]:+.3f} uB {jb_ar[(a, b)]:+.3f} first "
                             f"{ar_o[0] or '-'} contact {ar_o[1]} deadlock {ar_o[2]} | kin uA {ja_k[(a, b)]:+.3f} uB "
                             f"{jb_k[(a, b)]:+.3f} first {k_o[0] or '-'} contact {k_o[1]} deadlock {k_o[2]} | "
                             f"{'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cls = {}
        for name, ja, jb in (("arena", ja_ar, jb_ar), ("kinematic", ja_k, jb_k)):
            pure, mixed = G.solve(ja, jb, list(SUB), list(SUB))
            if name == "arena":
                fmaj = lambda p: max(set(r["first"] for r in rs_ar[p]), key=lambda x: sum(r["first"] == x for r in rs_ar[p]))  # noqa: E731
                firsts = {fmaj(p) for p in pure}
                contact = any(st.mean(int(r["contact"]) for r in rs_ar[p]) >= 0.5 for p in pure)
                dead = any(st.mean(int(r["deadlock"]) for r in rs_ar[p]) >= 0.5 for p in pure)
            else:
                firsts = {outs_k[p].o.first for p in pure}
                contact = any(outs_k[p].o.contact for p in pure)
                dead = any(outs_k[p].deadlock for p in pure)
            c = "!" if contact else "W" if dead else ("C" if len(firsts) > 1 else (firsts.pop() or "?")) if pure else (
                "M" if mixed else "?")
            cls[name] = (c, " ".join(f"{a}/{b}" for a, b in pure))
        cells.append({"dt": dt, "v": args.v, "class_arena": cls["arena"][0], "class_kinematic": cls["kinematic"][0],
                      "eq_arena": cls["arena"][1], "eq_kinematic": cls["kinematic"][1]})
        lines.append(f"## dt {dt:+.1f}: class arena {cls['arena'][0]} ({cls['arena'][1]}) | kinematic {cls['kinematic'][0]} "
                     f"({cls['kinematic'][1]})")
    summ = {"cells": cells, "payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
            "classes_agree": sum(c["class_arena"] == c["class_kinematic"] for c in cells),
            "arena_hold_stop": G.ARENA_HOLD_STOP}
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); "
                 f"outcomes agree in {agree} of {n_out}; classes agree in {summ['classes_agree']} of {len(cells)}")
    return lines, summ


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", type=float, default=6.0)
    # off the rules' thresholds: TIE 0.5 s, RBL_NEAR 2 s
    p.add_argument("--dts", default="-6,-3,-1.5,-0.25,0,0.25,1.5,3,6")
    p.add_argument("--obs-delay", type=float, default=G.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--compare-only", action="store_true")
    G.add_legacy_args(p)
    args = p.parse_args()
    G.apply_legacy_args(args)
    os.makedirs(args.out, exist_ok=True)
    if args.compare_only:
        rows = list(csv.DictReader(open(os.path.join(args.out, "runs.csv"))))
        wall = None
    else:
        XA.Run, XA.jobs = FWRun, jobs  # crossing_arena's loop, with the stop-sign runs
        rows, wall = XA.run(args)
    lines, summ = compare(args, rows)
    summ["wall_s"] = None if wall is None else round(wall, 1)
    summ["runs"] = len(rows)
    with open(os.path.join(args.out, "compare.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summ, f, indent=1)
    print("\n".join(l for l in lines if l.startswith("##") or l.startswith("payoffs")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
