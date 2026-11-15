#!/usr/bin/env python3
"""Roadmap 078 V4 (arena) and EV1 (arena side): czp in the arenas of S2 (the crossing), S4 (the narrow passage) and
S3 (the lane drop), as BAR: czp in a place against every rule of the study's arena subset in the other place, the
study's stock rule in that place against the same rules, and czp in self-play.

The arenas run as they are (crossing_arena, narrow_arena, merge_arena: their worlds, warm-ups, cars, observers,
scoring); only the jobs are this script's, and a rule named "czp" is czp_adapter's car (CZPPassage for the crossing
and the narrow passage, the merge's CZPMergeRules for A, its B side for B).

Stock per study (the study's rule nearest stock Autoware: a gap test with no go on a seen yield, a yield band for the
crossing's B, the through lane's car keeping its lane):
  cross   A t1.5p5, B yb2p5         v 10 and 14 (S2's arena), its dts
  narrow  t1.5p5 in either place    v 6 and 10, W_n given (S4's arena: 6 and 30, one W_n per world)
  merge   A e1.2d, B hold           vB 12, S3's arena contexts

Usage (a CARLA of its own; the arena makes it synchronous and puts it back):
  scripts/coop/czp_rows_arena.py --row cross  --port 2600 --out <dir> [--seeds 5]
  scripts/coop/czp_rows_arena.py --row narrow --port 2600 --out <dir> --wn 30 [--seeds 5]
  scripts/coop/czp_rows_arena.py --row merge  --port 2600 --out <dir> [--seeds 5]
Then scripts/coop/czp_rows_g0.py on <dir>/runs.csv.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import crossing_game as X  # noqa: E402
import czp_adapter as CA  # noqa: E402

STOCK = {"cross": ("t1.5p5", "yb2p5"), "narrow": ("t1.5p5", "t1.5p5"), "merge": ("e1.2d", "hold")}
CROSS_A = ("go", "t0.5yp2", "t1.5p2", "t1.5yp5", "tinfp2", "tinfp5")
CROSS_B = ("hold", "react", "yb1p2", "yb4p5")
NARROW = ("go", "t0.5p2", "t1.5yp5", "tinfp2", "flash", "yb2p5")
MERGE_A = ("e0.6d", "e1.2yd", "e1.2f", "z1.2d", "z1.2yf", "z0.6f")
MERGE_B = ("hold", "react", "c20", "c40")
CZP_A = ("czp", -math.inf, False, math.inf)  # crossing_game rule tuples naming czp (CZPPassage decides)
CZP_B = ("czp", None, None)


def opponents(row):
    """(rules czp in A meets in B, rules czp in B meets in A), the stock rule of the other place included."""
    sa, sb = STOCK[row]
    if row == "cross":
        return tuple(dict.fromkeys(CROSS_B + (sb,))), tuple(dict.fromkeys(CROSS_A + (sa,)))
    if row == "narrow":
        r = tuple(dict.fromkeys(NARROW + (sa,)))
        return r, r
    return tuple(dict.fromkeys(MERGE_B + (sb,))), tuple(dict.fromkeys(MERGE_A + (sa,)))


def pairs(row):
    """Every (a, b) the BAR table needs: czp and stock in A against B's opponents, czp and stock in B against A's,
    czp self-play."""
    sa, sb = STOCK[row]
    ob, oa = opponents(row)
    out = [("czp", "czp")]
    out += [(p, b) for p in ("czp", sa) for b in ob]
    out += [(a, p) for p in ("czp", sb) for a in oa]
    return list(dict.fromkeys(out))


def install(row):
    """Register czp's rule names with the study's model and patch its policy factory (before the arena runs)."""
    if row in ("cross", "narrow"):
        if CZP_A not in X.A_RULES:
            X.A_RULES.append(CZP_A)
        if CZP_B not in X.B_RULES:
            X.B_RULES.append(CZP_B)
        orig = X.Policy
        places = itertools.cycle("AB")  # the arenas build each run's A, then its B
        zk = CA.CM.CROSS_UNSIGNALLED if row == "cross" else CA.CM.NARROW_PASSAGE

        def make(role, rule, v0, b_cont="hold", exit_=X.EXIT, hold_stop=None):
            place = next(places)
            if rule[0] == "czp":
                return CA.CZPPassage(role, rule, v0, b_cont, exit_, hold_stop, zk, "czp", place)
            return orig(role, rule, v0, b_cont, exit_, hold_stop)
        X.Policy = make
    else:
        import merge_game as MG
        if CA.MERGE_CZP_A not in MG.A_RULES:
            MG.A_RULES.append(CA.MERGE_CZP_A)
        if CA.MERGE_CZP_B not in MG.B_RULES:
            MG.B_RULES.append(CA.MERGE_CZP_B)
        MG._orig_rules = MG.Rules
        MG.Rules = CA.merge_rules_factory("czp")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--row", choices=("cross", "narrow", "merge"), required=True)
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--k", type=int, default=8)
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--v", default=None, help="cross: 10,14; narrow: 6,10")
    ap.add_argument("--wn", type=float, default=30.0)
    ap.add_argument("--dts", default=None)
    ap.add_argument("--obs-delay", type=float, default=X.OBS)
    ap.add_argument("--obs-sigma-x", type=float, default=0.3)
    ap.add_argument("--obs-sigma-v", type=float, default=0.2)
    ap.add_argument("--list", action="store_true", help="print the job count and exit")
    a = ap.parse_args()
    install(a.row)
    prs = pairs(a.row)
    jobs = []
    if a.row == "cross":
        import crossing_arena as M
        dts = [float(x) for x in (a.dts or "-1.5,-0.7,-0.3,-0.1,0,0.1,0.3,0.7,1.5").split(",")]
        for v in [float(x) for x in (a.v or "10,14").split(",")]:
            for dt in dts:
                for pa, pb in prs:
                    for c in (("yield", "hold") if pb == "react" else (None,)):
                        for s in range(a.seeds):
                            j = {"dt": dt, "v": v, "a": pa, "b": pb, "seed": s}
                            if c:
                                j["b_cont"] = c
                            jobs.append(j)
    elif a.row == "narrow":
        import narrow_arena as M
        dts = [float(x) for x in (a.dts or "-1.5,-0.7,-0.3,-0.1,0,0.3,1.5").split(",")]
        jobs = [{"wn": a.wn, "v": v, "dt": dt, "a": pa, "b": pb, "seed": s}
                for v in [float(x) for x in (a.v or "6,10").split(",")] for dt in dts for pa, pb in prs
                for s in range(a.seeds)]
    else:
        import merge_arena as M
        a.vb, a.d_ends, a.d0s, a.dvs = 12.0, "40,80", "-5,2.5,7.5,15", "-2,0"
        for de, d0, dv in M.contexts(a):
            for pa, pb in prs:
                for c in (("yield", "hold") if pb == "react" else (None,)):
                    for s in range(a.seeds):
                        j = {"d_end": de, "d0": d0, "dv": dv, "vb": a.vb, "a": pa, "b": pb, "seed": s}
                        if c:
                            j["b_cont"] = c
                        jobs.append(j)
    print(f"{a.row}: {len(prs)} pairs, {len(jobs)} runs", flush=True)
    if a.list:
        return 0
    M.jobs = lambda _args: jobs
    os.makedirs(a.out, exist_ok=True)
    M.run(a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
