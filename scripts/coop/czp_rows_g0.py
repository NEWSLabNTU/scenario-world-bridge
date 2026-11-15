#!/usr/bin/env python3
"""Roadmap 078 V4 / G0 (arena) and EV1 (arena side) on czp_rows_arena.py's runs.

G0 (stated before the runs, 078): no contact in self-play, no deadlock, and never worse than stock against
non-compliers -- per BAR cell (czp in a place, A or B, against a rule X in the other), no more contacts and no
more deadlocks than the study's stock rule in that place against the same X, over the same contexts and seeds
(as v2b_g0.py for the all-way stop). A deadlock is a run with no contact in which a car is not through the zone
(merge: A not merged) by T_MAX.

EV1, arena side: czp's arena outcomes against the kinematic model's, the same pairs and contexts (czp in the
same places, the study's kinematic model with the car's lags, truth perception). Outcome per (context, pair):
cross / narrow (first, contact, both stopped), merge (A ahead, contact, forced stop); the arena's majority
over seeds. In the near-tie band (|dt| <= 0.1 s; cross and narrow) parity is on the contact share (074's S2
follow-up, 078 EV1): the arena's share against the kinematic one, within 2 pooled SE or 0.1.

Usage: scripts/coop/czp_rows_g0.py --row cross|narrow|merge --runs <runs.csv> [--runs ...] --out <file.json>
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import crossing_game as X  # noqa: E402
import czp_adapter as CA  # noqa: E402
import czp_rows_arena as RA  # noqa: E402

NEAR = 0.1


def ctx_key(row, r):
    if row == "merge":
        return (float(r["d_end"]), float(r["d0"]), float(r["dv"]))
    if row == "narrow":
        return (float(r["wn"]), float(r["v"]), float(r["dt"]))
    return (float(r["v"]), float(r["dt"]))


def pair_key(r):
    b = r["b"] if r["b"] != "react" else f"react:{r.get('b_cont', 'hold')}"
    return r["a"], b


def deadlock(row, r):
    if int(r["contact"]):
        return 0
    if row == "merge":
        return int(not int(r["merged"]))
    return int(float(r["lostA"]) >= 1.0 or float(r["lostB"]) >= 1.0)


def g0(row, rows):
    by = defaultdict(list)
    for r in rows:
        by[pair_key(r)].append(r)
    sa, sb = RA.STOCK[row]
    ob, oa = RA.opponents(row)

    def cnt(a, b):
        rs = [x for k, v in by.items() if k[0] == a and (k[1] == b or k[1].startswith(b + ":")) for x in v]
        return (len(rs), sum(int(r["contact"]) for r in rs), sum(deadlock(row, r) for r in rs),
                sum(float(r["uA"]) for r in rs) / max(len(rs), 1), sum(float(r["uB"]) for r in rs) / max(len(rs), 1))
    n, c, d, ua, ub = cnt("czp", "czp")
    res = {"self_play": {"runs": n, "contacts": c, "deadlocks": d, "uA": round(ua, 3), "uB": round(ub, 3)}, "cells": []}
    lines = [f"## {row}: czp self-play {c} contacts, {d} deadlocks of {n}"]
    worse = 0
    for place, opp, stock in (("A", ob, sa), ("B", oa, sb)):
        for x in opp:
            a, b = ("czp", x) if place == "A" else (x, "czp")
            s_a, s_b = (stock, x) if place == "A" else (x, stock)
            n1, c1, d1, ua1, ub1 = cnt(a, b)
            n0, c0, d0, ua0, ub0 = cnt(s_a, s_b)
            if not n1 or not n0:
                continue
            ok = c1 <= c0 and d1 <= d0
            worse += not ok
            um, us = (ua1, ua0) if place == "A" else (ub1, ub0)
            res["cells"].append({"place": place, "vs": x, "runs": n1, "contacts": c1, "deadlocks": d1, "stock": stock,
                                 "stock_contacts": c0, "stock_deadlocks": d0, "never_worse": ok,
                                 "u_me": round(um, 3), "u_me_stock": round(us, 3)})
            lines.append(f"  czp in {place} vs {x:<8} contact {c1:>2} (stock {stock} {c0:>2}) deadlock {d1:>2} (stock {d0:>2}) "
                         f"of {n1:>3} | u_me {um:+.3f} (stock {us:+.3f}) | {'ok' if ok else 'WORSE'}")
    res["worse_cells"] = worse
    res["cells_n"] = len(res["cells"])
    res["g0_arena"] = c == 0 and d == 0 and worse == 0 and all(cl["deadlocks"] == 0 for cl in res["cells"])
    lines.append(f"  worse than stock in {worse} of {len(res['cells'])} cells; deadlocks in czp cells "
                 f"{sum(cl['deadlocks'] for cl in res['cells'])}; G0 (arena) {'PASS' if res['g0_arena'] else 'FAIL'}")
    return res, lines


def kin_outcome(row, ctx, a, b, b_cont, lags):
    cars = ("A" if a == "czp" else "") + ("B" if b == "czp" else "")
    if row == "cross":
        v, dt = ctx
        ra = next(r for r in X.A_RULES if r[0] == (a if a != "czp" else "go"))
        rb = next(r for r in X.B_RULES if r[0] == (b if b != "czp" else "hold"))
        with CA.patched_crossing(tuple(cars), CA.CM.CROSS_UNSIGNALLED, "czp"):
            o = X.simulate(ra, rb, dt, v, v, b_cont or "hold", lags=lags)
        return (o.first, int(o.contact), int(o.both_stopped))
    if row == "narrow":
        import narrow_game as N
        wn, v, dt = ctx
        with CA.patched_crossing(tuple(cars), CA.CM.NARROW_PASSAGE, "czp"):
            res = N.simulate(a if a != "czp" else "go", b if b != "czp" else "go", dt, v, v, wn, lags)
        return (res.o.first, int(res.o.contact), int(res.o.both_stopped))
    import merge_game as MG
    d_end, d0, dv = ctx
    ra = CA.MERGE_CZP_A if a == "czp" else next(r for r in MG.A_RULES if r[0] == a)
    rb = CA.MERGE_CZP_B if b == "czp" else next(r for r in MG.B_RULES if r[0] == b)
    with CA.patched_merge("czp", cars):
        o = MG.simulate(ra, rb, d_end, d0, dv, 12.0, b_cont or "hold", lags)
    return (int(o.ahead), int(o.contact), int(o.stopped))


def arena_outcome(row, r):
    if row == "merge":
        return (int(r["ahead"]), int(r["contact"]), int(r["stopped"]))
    return (r["first"], int(r["contact"]), int(r["both_stopped"]))


def ev1(row, rows):
    lags = X.car_lags()
    by = defaultdict(list)
    for r in rows:
        if "czp" in (r["a"], r["b"]):
            by[(ctx_key(row, r), r["a"], r["b"], r.get("b_cont") or None)].append(r)
    n = agree = 0
    near = {"arena": [0, 0], "kin": [0, 0]}
    near_cells = []
    miss = []
    for (ctx, a, b, bc), rs in sorted(by.items(), key=lambda kv: str(kv[0])):
        k = kin_outcome(row, ctx, a, b, bc, lags)
        dt = ctx[-1] if row != "merge" else None
        if dt is not None and abs(dt) <= NEAR + 1e-9:
            ca = sum(int(r["contact"]) for r in rs)
            near["arena"][0] += ca
            near["arena"][1] += len(rs)
            near["kin"][0] += k[1]
            near["kin"][1] += 1
            near_cells.append({"ctx": ctx, "a": a, "b": b, "arena": f"{ca}/{len(rs)}", "kin": k[1]})
            continue
        outs = [arena_outcome(row, r) for r in rs]
        maj = max(set(outs), key=outs.count)
        n += 1
        agree += maj == k
        if maj != k:
            miss.append(f"{ctx} {a}/{b}{':' + bc if bc else ''}: arena {maj} kin {k}")
    pa = near["arena"][0] / max(near["arena"][1], 1)
    pk = near["kin"][0] / max(near["kin"][1], 1)
    se = math.sqrt(max(pa * (1 - pa) / max(near["arena"][1], 1) + pk * (1 - pk) / max(near["kin"][1], 1), 0.0))
    return {"outcomes_agree": f"{agree}/{n}", "pct": round(100 * agree / max(n, 1), 1),
            "near_tie_contact_share": {"arena": f"{near['arena'][0]}/{near['arena'][1]}", "kin": f"{near['kin'][0]}/{near['kin'][1]}",
                                       "within": abs(pa - pk) <= max(2 * se, 0.1)},
            "near_cells": near_cells, "misses": miss}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--row", choices=("cross", "narrow", "merge"), required=True)
    ap.add_argument("--runs", action="append", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--no-ev1", action="store_true")
    a = ap.parse_args()
    rows = [r for p in a.runs for r in csv.DictReader(open(p))]
    res, lines = g0(a.row, rows)
    out = {"row": a.row, "runs": len(rows), "g0": res}
    if not a.no_ev1:
        e = ev1(a.row, rows)
        out["ev1"] = e
        lines.append(f"  EV1 arena vs kinematic (czp's pairs): outcomes {e['outcomes_agree']} ({e['pct']} %); near-tie contact "
                     f"share arena {e['near_tie_contact_share']['arena']} kin {e['near_tie_contact_share']['kin']} "
                     f"({'within' if e['near_tie_contact_share']['within'] else 'OUTSIDE'})")
    json.dump(out, open(a.out, "w"), indent=1)
    open(os.path.splitext(a.out)[0] + ".txt", "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
