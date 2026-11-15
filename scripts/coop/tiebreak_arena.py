#!/usr/bin/env python3
"""Roadmap 074 C7 in the arena: the tie-breaks of tiebreak.py at the crossing (crossing_arena) and
the narrow passage (narrow_arena), on CARLA's physics, against the kinematic model on D2's yardstick.

The arenas are run as they are (their worlds, warm-ups, cars, observers and scoring); only each
car's policy is tiebreak.ConvPolicy (the rule with the convention on top, through tiebreak.patched)
and each run records the convention's payoffs, its wait and the coin's signal. Behaviour variants
per rule pair: none (also rB's and bind_fault's runs), bind, coin with A winning, coin with B
winning, coin_stop (the coin sent when the first car stops) with A or B winning, stagger.

Per cell and convention the game on the arena's rule subset is solved on the arena's mean payoffs
and on the kinematic model's (tiebreak.run_one on the same subset), and compared on D2's yardstick:
outcomes (who entered first, contact, both stopped: the arena's majority over seeds) per behaviour
variant and rule pair, and classes per cell. Reported per convention, arena and model side by side:
classes, restart crashes among the runs where both stopped, mean wait, the worst equilibrium's
welfare cost and the share of the none-convention's worst-best gap it recovers.

Usage (a CARLA of its own; the arena makes it synchronous and puts it back):
  scripts/coop/tiebreak_arena.py --geom cross  --port 2600 --out <dir> [--v 10] [--seeds 3]
  scripts/coop/tiebreak_arena.py --geom narrow --port 2610 --out <dir> --wn 30 [--v 6,10] [--seeds 3]
  scripts/coop/tiebreak_arena.py --geom ... --out <dir> --compare-only
  scripts/coop/tiebreak_arena.py --selftest      # the comparison on synthetic rows, no CARLA
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import measures as M  # noqa: E402
import tiebreak as TB  # noqa: E402

VARIANTS = (("none", ""), ("bind", ""), ("coin", "A"), ("coin", "B"), ("coin_stop", "A"), ("coin_stop", "B"),
            ("stagger", ""))
MODELS = ("none", "rB", "bind", "coin")
DTS = "-0.7,-0.3,-0.1,0,0.1,0.3,0.7"
CROSS_A = ("go", "t0.5yp2", "t1.5p2", "t1.5yp5", "tinfp2", "tinfp5")
CROSS_B = ("hold", "react", "yb1p2", "yb4p5")
NARROW = ("go", "t0.5p2", "t1.5yp5", "tinfp2", "flash", "yb2p5")


def subset(geom):
    """(A's actions, B's actions, the run keys behind each B action)."""
    if geom == "cross":
        return CROSS_A, CROSS_B, {b: ([f"react:{c}" for c in ("yield", "hold")] if b == "react" else [b]) for b in CROSS_B}
    return NARROW, NARROW, {b: [b] for b in NARROW}


def extra(run, rec):
    """The convention's fields of a finished arena run: payoffs under every fault model, wait, signal."""
    m = run.meas
    lost = (float(rec["lostA"]), float(rec["lostB"]))
    o = X.Outcome(lost, (run.peak["A"], run.peak["B"]), bool(run.contact), m.ttc2d, m.thw, m._pet(), m.clear, rec["first"],
                  bool(run.both_stopped), "", (run.pol["A"].shield, run.pol["B"].shield),
                  (run.pol["A"].rested, run.pol["B"].rested))
    r = TB.Res(o, bool(run.contact and run.both_stopped), False, (run.made[0].wait + run.made[1].wait) / 2,
               run.ref is not None and run.ref.t_sig is not None, run.job.get("winner", ""))
    for model in MODELS:
        rec[f"uA_{model}"] = round(TB.utility(r, 0, model), 4)
        rec[f"uB_{model}"] = round(TB.utility(r, 1, model), 4)
    rec.update({"restart": int(r.restart), "wait": round(r.wait, 2), "sig": int(r.sig), "thw": round(m.thw, 3) if math.isfinite(m.thw) else "inf"})


def wrap_run(base):
    class CRun(base):
        def __init__(self, place, a_car, b_car, job, rng, args):
            with TB.patched(job["beh"], job["beh"], job.get("winner") or None) as (made, ref):
                super().__init__(place, a_car, b_car, job, rng, args)
            self.made, self.ref = made, ref

        def finish(self, *a):
            super().finish(*a)
            extra(self, self.rec)

    return CRun


def jobs_cross(args):
    out = []
    for dt in [float(x) for x in args.dts.split(",")]:
        for v in [float(x) for x in args.v.split(",")]:
            for beh, w in VARIANTS:
                for a in CROSS_A:
                    for b in CROSS_B:
                        for c in (("yield", "hold") if b == "react" else (None,)):
                            for s in range(args.seeds):
                                j = {"dt": dt, "v": v, "a": a, "b": b, "seed": s, "beh": beh, "winner": w}
                                if c:
                                    j["b_cont"] = c
                                out.append(j)
    return out


def jobs_narrow(args):
    return [{"wn": args.wn, "v": v, "dt": dt, "a": a, "b": b, "seed": s, "beh": beh, "winner": w}
            for v in [float(x) for x in args.v.split(",")] for dt in [float(x) for x in args.dts.split(",")]
            for beh, w in VARIANTS for a, b in itertools.product(NARROW, NARROW) for s in range(args.seeds)]


def run_arena(args):
    if args.geom == "cross":
        import crossing_arena as A
        A.Run, A.jobs = wrap_run(A.Run), jobs_cross
        if len({float(x) for x in args.v.split(",")}) != 1:
            sys.exit("crossing: one --v per invocation (crossing_arena's Run reads job['v'], the compare one speed)")
        args.v = str(float(args.v))
    else:
        import narrow_arena as A
        A.Run, A.jobs = wrap_run(A.Run), jobs_narrow
    return A.run(args)


def key_b(r):
    return f"react:{r['b_cont']}" if r["b"] == "react" and r.get("b_cont") else r["b"]


def side_game(conv, a_acts, b_acts, keys, u_of, flags_of):
    """The convention's game on one side: u_of(beh, w, a, k, who, model) -> payoff; flags_of(beh, w, a,
    k) -> (first, contact, both_stopped)."""
    beh, model = TB.BEH[conv], TB.FAULT_MODEL[conv]
    ws = ("A", "B") if beh in TB.COINS else ("",)
    ua, ub, phys, fl = {}, {}, {}, {}
    for a in a_acts:
        for b in b_acts:
            def mu(k, who, m=model):
                return st.mean(u_of(beh, w, a, k, who, m) for w in ws)
            k = max(keys[b], key=lambda kk: mu(kk, 1))
            ua[(a, b)], ub[(a, b)] = mu(k, 0), mu(k, 1)
            phys[(a, b)] = -(mu(k, 0, "none") + mu(k, 1, "none"))
            fl[(a, b)] = [flags_of(beh, w, a, k) for w in ws]
    pure, mixed = X.solve(ua, ub, a_acts, b_acts)
    if any(any(f[1] for f in fl[p]) for p in pure):
        c = "!"
    elif any(any(f[2] for f in fl[p]) for p in pure):
        c = "W"
    else:
        fs = {f[0] for p in pure for f in fl[p]}
        c = "C" if len(fs) > 1 else ((fs.pop() or "?") if fs else ("M" if mixed else "?"))
    worst, best = TB.eq_costs(phys, pure, mixed)
    return {"class": c, "pure": " ".join(f"{a}/{b}" for a, b in pure), "C_worst": worst, "C_best": best,
            "C_opt": min(phys.values()), "C_pairs": st.mean(phys.values())}


def compare(args, rows):
    geom = args.geom
    a_acts, b_acts, keys = subset(geom)
    lags = X.car_lags()
    by = {}
    for r in rows:
        cell = (float(r["v"]), float(r["dt"]))
        by.setdefault((cell, r["beh"], r.get("winner") or "", r["a"], key_b(r)), []).append(r)
    cells = sorted({k[0] for k in by})
    wn = float(rows[0].get("wn") or 0.0)
    lines, cell_rows = [], []
    agg = {c: {"arena": {"restart": 0, "waited": 0, "wait": [], "contact": 0, "n": 0},
               "kinematic": {"restart": 0, "waited": 0, "wait": [], "contact": 0, "n": 0}} for c in TB.CONVS}
    out_agree = out_total = 0
    for cell in cells:
        v, dt = cell
        kin = {}
        for beh, w in VARIANTS:
            for a in a_acts:
                for b in b_acts:
                    for k in keys[b]:
                        kin[(beh, w, a, k)] = TB.run_one(geom, a, k, (dt, v, wn), (beh, beh), w or None, lags)
                        rs = by[(cell, beh, w, a, k)]
                        maj = lambda f, rs=rs: max(set(f(x) for x in rs), key=lambda y: sum(f(x) == y for x in rs))  # noqa: E731
                        ko = kin[(beh, w, a, k)].o
                        same = (maj(lambda x: x["first"]) == ko.first and maj(lambda x: int(x["contact"])) == int(ko.contact)
                                and maj(lambda x: int(x["both_stopped"])) == int(ko.both_stopped))
                        out_agree += same
                        out_total += 1

        def u_ar(beh, w, a, k, who, m):
            return st.mean(float(x[f"u{'AB'[who]}_{m}"]) for x in by[(cell, beh, w, a, k)])

        def f_ar(beh, w, a, k):
            rs = by[(cell, beh, w, a, k)]
            maj = lambda f: max(set(f(x) for x in rs), key=lambda y: sum(f(x) == y for x in rs))  # noqa: E731
            return maj(lambda x: x["first"]), maj(lambda x: int(x["contact"])), maj(lambda x: int(x["both_stopped"]))

        def u_k(beh, w, a, k, who, m):
            return TB.utility(kin[(beh, w, a, k)], who, m)

        def f_k(beh, w, a, k):
            o = kin[(beh, w, a, k)].o
            return o.first, int(o.contact), int(o.both_stopped)

        row = {"geom": geom, "wn": wn, "v": v, "dt": dt}
        base = {}
        for conv in TB.CONVS:
            for side, uf, ff in (("arena", u_ar, f_ar), ("kinematic", u_k, f_k)):
                g = side_game(conv, a_acts, b_acts, keys, uf, ff)
                if conv == "none":
                    base[side] = g
                gap = None if g["C_worst"] is None or base[side]["C_worst"] is None else base[side]["C_worst"] - base[side]["C_best"]
                row[f"{conv}_{side}_class"] = g["class"]
                row[f"{conv}_{side}_C_worst"] = g["C_worst"]
                row[f"{conv}_{side}_loss"] = None if g["C_worst"] is None else g["C_worst"] - g["C_opt"]
                row[f"{conv}_{side}_recovered"] = ((base[side]["C_worst"] - g["C_worst"]) / gap
                                                   if gap is not None and gap >= TB.GAP_MIN else None)
                row[f"{conv}_{side}_eq"] = g["pure"]
            beh = TB.BEH[conv]
            for w in (("A", "B") if beh in TB.COINS else ("",)):
                for a in a_acts:
                    for b in b_acts:
                        for k in keys[b]:
                            for x in by[(cell, beh, w, a, k)]:
                                g = agg[conv]["arena"]
                                g["n"] += 1
                                g["contact"] += int(x["contact"])
                                g["waited"] += int(x["both_stopped"])
                                g["restart"] += int(x["restart"])
                                g["wait"].append(float(x["wait"]))
                            r = kin[(beh, w, a, k)]
                            g = agg[conv]["kinematic"]
                            g["n"] += 1
                            g["contact"] += int(r.o.contact)
                            g["waited"] += int(r.o.both_stopped)
                            g["restart"] += int(r.restart)
                            g["wait"].append(r.wait)
        cell_rows.append(row)
        lines.append(f"v {v:3g} dt {dt:+4.1f}: " + "  ".join(
            f"{c} {row[f'{c}_arena_class']}/{row[f'{c}_kinematic_class']}" for c in TB.CONVS))
    cls_agree = {c: sum(r[f"{c}_arena_class"] == r[f"{c}_kinematic_class"] for r in cell_rows) for c in TB.CONVS}
    summary = {"geom": geom, "wn": wn, "runs": len(rows), "outcomes_agree": out_agree, "outcomes": out_total,
               "classes_agree": cls_agree, "cells": len(cell_rows), "per_conv": {}}
    lines.append(f"\noutcomes agree (first, contact, both stopped) in {out_agree} of {out_total} variant pairs "
                 f"({100 * out_agree / max(out_total, 1):.0f} %); classes agree per convention: "
                 + ", ".join(f"{c} {n}/{len(cell_rows)}" for c, n in cls_agree.items()))
    lines.append(f"\n{'conv':11} | {'restart arena':>16} {'kin':>14} | {'contact ar':>10} {'kin':>6} | {'wait ar':>7} {'kin':>5} | "
                 f"{'loss ar':>7} {'kin':>6} | {'rec_eq ar':>9} {'kin':>6}")
    for c in TB.CONVS:
        pc = {}
        for side in ("arena", "kinematic"):
            g = agg[c][side]
            recs = [r[f"{c}_{side}_recovered"] for r in cell_rows if r[f"{c}_{side}_recovered"] is not None]
            losses = [r[f"{c}_{side}_loss"] for r in cell_rows if r[f"{c}_{side}_loss"] is not None]
            pc[side] = {"restart": g["restart"], "waited": g["waited"],
                        "restart_share": g["restart"] / g["waited"] if g["waited"] else None,
                        "contact_share": g["contact"] / g["n"], "wait": st.mean(g["wait"]),
                        "loss": st.mean(losses) if losses else None,
                        "recovered": st.mean(recs) if recs else None, "n_gap": len(recs)}
        summary["per_conv"][c] = pc
        f = lambda x, s="{:.2f}": "" if x is None else s.format(x)  # noqa: E731
        a_, k_ = pc["arena"], pc["kinematic"]
        lines.append(f"{c:11} | {f(a_['restart_share'], '{:.0%}'):>5} {a_['restart']:>4}/{a_['waited']:<5} "
                     f"{f(k_['restart_share'], '{:.0%}'):>5} {k_['restart']:>3}/{k_['waited']:<4} | "
                     f"{a_['contact_share']:10.1%} {k_['contact_share']:6.1%} | {a_['wait']:7.2f} {k_['wait']:5.2f} | "
                     f"{f(a_['loss'], '{:.3f}'):>7} {f(k_['loss'], '{:.3f}'):>6} | "
                     f"{f(a_['recovered'], '{:+.2f}'):>6} n{a_['n_gap']:<2} {f(k_['recovered'], '{:+.2f}'):>5} n{k_['n_gap']}")
    return lines, summary, cell_rows


def selftest():
    """The comparison on synthetic arena rows made from the kinematic model itself: everything agrees."""
    class A:
        geom, dts, v = "narrow", "0", "6"
    lags = X.car_lags()
    rows = []
    for beh, w in VARIANTS:
        for a, b in itertools.product(NARROW, NARROW):
            r = TB.run_one("narrow", a, b, (0.0, 6.0, 6.0), (beh, beh), w or None, lags)
            rec = {"wn": 6.0, "v": 6.0, "dt": 0.0, "a": a, "b": b, "beh": beh, "winner": w, "first": r.o.first,
                   "contact": int(r.o.contact), "both_stopped": int(r.o.both_stopped), "restart": int(r.restart),
                   "wait": r.wait}
            for m in MODELS:
                rec[f"uA_{m}"], rec[f"uB_{m}"] = TB.utility(r, 0, m), TB.utility(r, 1, m)
            rows.append(rec)
    lines, s, _ = compare(A, rows)
    assert s["outcomes_agree"] == s["outcomes"], s
    assert all(n == 1 for n in s["classes_agree"].values()), s
    assert s["per_conv"]["none"]["arena"]["restart"] == s["per_conv"]["none"]["kinematic"]["restart"], s
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--geom", choices=("cross", "narrow"))
    p.add_argument("--port", type=int)
    p.add_argument("--out")
    p.add_argument("--wn", type=float, default=30.0)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", default=None, help="default 10 (cross), 6,10 (narrow)")
    p.add_argument("--dts", default=DTS)
    p.add_argument("--obs-delay", type=float, default=X.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--compare-only", action="store_true")
    p.add_argument("--selftest", action="store_true")
    args = p.parse_args()
    if args.selftest:
        return selftest()
    args.v = args.v or ("10" if args.geom == "cross" else "6,10")
    if args.compare_only:
        rows = list(csv.DictReader(open(os.path.join(args.out, "runs.csv"))))
        wall = None
    else:
        rows, wall = run_arena(args)
    lines, summary, cell_rows = compare(args, rows)
    summary["wall_s"] = None if wall is None else round(wall, 1)
    with open(os.path.join(args.out, "compare.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    with open(os.path.join(args.out, "cells.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(cell_rows[0].keys()))
        w.writeheader()
        w.writerows(cell_rows)
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
