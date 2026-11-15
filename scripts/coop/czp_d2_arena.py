#!/usr/bin/env python3
"""Roadmap 078 EV1, the arena side for U3: czp's outcomes in the arena (four_way_arena --czp runs) against the
same pairs in the kinematic model (truth perception, the car's lags), on D2's yardstick: the arena's majority
outcome (first, contact, deadlock) per (dt, pair) against the kinematic one, off the near-tie band; in the band
(|dt| <= 0.1 s, here dt 0) the contact shares.

Usage: scripts/coop/czp_d2_arena.py --runs <runs.csv> --v 6 --out <file.json>
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import czp_adapter as CA  # noqa: E402
import four_way_game as F  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", required=True)
    ap.add_argument("--v", type=float, default=6.0)
    ap.add_argument("--rules", default="czp_c1.5,czp0_c1.5,fcfse_c1.5")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    CA.enable(bias=0.0)
    F.set_fm(bias=0.0)
    rule = {r[0]: r for r in F.RULES}
    lags = X.car_lags()
    by = defaultdict(list)
    for r in csv.DictReader(open(a.runs)):
        by[(float(r["dt"]), r["a"], r["b"])].append(r)
    out = {}
    for p in a.rules.split(","):
        n = agree = 0
        near = {"arena": [0, 0], "kin": [0, 0]}
        miss = []
        for (dt, ra, rb), rs in sorted(by.items()):
            if p not in (ra, rb):
                continue
            ko = F.simulate(rule[ra], rule[rb], dt, a.v, a.v, lags)
            k = (ko.o.first, int(ko.o.contact), int(ko.deadlock))
            maj = lambda f: max(set(f(r) for r in rs), key=lambda x: sum(f(r) == x for r in rs))  # noqa: E731
            ar = (maj(lambda r: r["first"]), maj(lambda r: int(r["contact"])), maj(lambda r: int(r["deadlock"])))
            if abs(dt) <= 0.1:
                near["arena"][0] += sum(int(r["contact"]) for r in rs)
                near["arena"][1] += len(rs)
                near["kin"][0] += k[1]
                near["kin"][1] += 1
                continue
            n += 1
            agree += ar == k
            if ar != k:
                miss.append(f"dt{dt:+g} {ra}/{rb}: arena {ar} kin {k}")
        out[p] = {"outcomes_agree": f"{agree}/{n}", "pct": round(100 * agree / max(n, 1), 1),
                  "near_tie_contact_share": {k: f"{x[0]}/{x[1]}" for k, x in near.items()}, "misses": miss}
        print(p, out[p]["outcomes_agree"], out[p]["pct"], out[p]["near_tie_contact_share"])
    json.dump(out, open(a.out, "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
