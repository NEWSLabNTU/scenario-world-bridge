#!/usr/bin/env python3
"""Roadmap 078 V2b: G0's criterion for the all-way stop on four_way_arena.py's runs (--czp --pairs bar).

G0 (stated before the runs, 078): no contact in self-play, no deadlock, and never worse than stock against
non-compliers -- per BAR cell (the protocol's rule in a place, A or B, against a rule X in the other),
no more contacts and no more deadlocks than stock Autoware (yield_c1.5) in that place against the same
X, over the same dts and seeds. Reported for czp (C8's three fixes), czp0 (without) and fcfse (C8's (e)).

Usage:
  scripts/coop/v2b_g0.py --runs <runs.csv> [--runs ...] --out <file.json>
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import defaultdict

STOCK = "yield_c1.5"
OTHERS = ("aggr", "yield_c1.5", "stop1y_c1.5", "fcfs_c1.5", "fcfsr_c1.5", "rbl_c1.5")
PROTO = ("czp_c1.5", "czp0_c1.5", "fcfse_c1.5")


def table(path):
    rows = list(csv.DictReader(open(path)))
    by = defaultdict(list)
    for r in rows:
        by[(r["a"], r["b"])].append(r)

    def cnt(a, b):
        rs = by.get((a, b), [])
        return (len(rs), sum(int(r["contact"]) for r in rs), sum(int(r["deadlock"]) for r in rs),
                sorted({float(r["dt"]) for r in rs if int(r["contact"])}),
                sum(float(r["uA"]) for r in rs) / max(len(rs), 1), sum(float(r["uB"]) for r in rs) / max(len(rs), 1))
    out = {"runs": len(rows), "protocols": {}}
    lines = [f"## {path} ({len(rows)} runs)"]
    for p in PROTO:
        if (p, p) not in by:
            continue
        n, c, d, cdt, ua, ub = cnt(p, p)
        res = {"self_play": {"runs": n, "contacts": c, "deadlocks": d, "contact_dts": cdt}, "cells": []}
        lines.append(f"{p}: self-play {c} contacts, {d} deadlocks of {n}")
        worse = 0
        for place in "AB":
            for x in OTHERS:
                a, b = (p, x) if place == "A" else (x, p)
                sa, sb = (STOCK, x) if place == "A" else (x, STOCK)
                n1, c1, d1, cdt1, ua1, ub1 = cnt(a, b)
                n0, c0, d0, _, ua0, ub0 = cnt(sa, sb)
                if not n1 or not n0:
                    continue
                ok = c1 <= c0 and d1 <= d0
                worse += not ok
                um, us = (ua1, ua0) if place == "A" else (ub1, ub0)
                res["cells"].append({"place": place, "vs": x, "runs": n1, "contacts": c1, "deadlocks": d1,
                                     "stock_contacts": c0, "stock_deadlocks": d0, "never_worse": ok, "contact_dts": cdt1,
                                     "u_me": round(um, 3), "u_me_stock": round(us, 3)})
                lines.append(f"  {p} in {place} vs {x:<12} contact {c1:>2} (stock {c0:>2}) deadlock {d1:>2} (stock {d0:>2}) "
                             f"of {n1:>3} | u_me {um:+.3f} (stock {us:+.3f}) | {'ok' if ok else 'WORSE'} {cdt1 or ''}")
        res["worse_cells"] = worse
        res["cells_n"] = len(res["cells"])
        res["g0"] = (res["self_play"]["contacts"] == 0 and res["self_play"]["deadlocks"] == 0 and worse == 0
                     and all(cl["deadlocks"] == 0 for cl in res["cells"]))
        lines.append(f"  {p}: worse than stock in {worse} of {len(res['cells'])} cells; G0 {'PASS' if res['g0'] else 'FAIL'}")
        out["protocols"][p] = res
    return out, lines


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", action="append", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    allo, lines = {}, []
    for p in a.runs:
        o, ls = table(p)
        allo[os.path.basename(os.path.dirname(p))] = o
        lines += ls
    json.dump(allo, open(a.out, "w"), indent=1)
    open(os.path.splitext(a.out)[0] + ".txt", "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
