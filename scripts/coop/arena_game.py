#!/usr/bin/env python3
"""Roadmap 074: the empirical game from the arena's runs, against C1's prediction.

Reads one or more arena `runs.csv`, averages the payoffs per (context, profile) over the seeds,
forms B's REACT as its better continuation (react_yield or react_hold, by B's mean payoff) per A
strategy, as C1 does, solves the game with C1's solver, and prints, per context: the arena's
equilibria and class, C1's, and the payoffs' spread over the seeds.

Usage:
  scripts/coop/arena_game.py [--type=<C1 type>] <runs.csv> [<runs.csv> ...]
"""

import csv
import statistics as st
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cutin_game as C1  # noqa: E402


def load(paths):
    cells = defaultdict(list)
    for p in paths:
        for r in csv.DictReader(open(p)):
            cells[(float(r["d0"]), float(r["dv"]), r["a"], r["b"])].append(r)
    return cells


def mean(rows, k):
    return st.mean(float(r[k]) for r in rows)


def spread(rows, k):
    return st.pstdev(float(r[k]) for r in rows) if len(rows) > 1 else 0.0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--type=")]
    ctype = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--type=")), "car")
    cells = load(args)
    contexts = sorted({(d0, dv) for d0, dv, _, _ in cells})
    car = C1.TYPES[ctype]
    pref = C1.Preference()
    agree = 0
    print("d0    dv   | arena: class  equilibria                         | C1: class  equilibria"
          "                         | max sd uA, uB")
    for d0, dv in contexts:
        ja, jb, went, contact = {}, {}, {}, {}
        sd = 0.0
        for a in C1.A_STRATS:
            for b in ("pre_yield", "react_yield", "react_hold"):
                rows = cells.get((d0, dv, a, b), [])
                if not rows:
                    continue
                sd = max(sd, spread(rows, "uA"), spread(rows, "uB"))
            ry, rh = cells[(d0, dv, a, "react_yield")], cells[(d0, dv, a, "react_hold")]
            react = ry if mean(ry, "uB") >= mean(rh, "uB") else rh
            for b, rows in ((C1.PRE_YIELD, cells[(d0, dv, a, "pre_yield")]), (C1.REACT, react)):
                ja[(a, b)], jb[(a, b)] = mean(rows, "uA"), mean(rows, "uB")
                went[(a, b)] = a != C1.WAIT and mean(rows, "got_in") > 0.5
                contact[(a, b)] = mean(rows, "contact") > 0.5
                if b == C1.REACT:
                    went[("cont", a)] = "yield" if rows is ry else "hold"
        pure, mixed = C1.solve(ja, jb, C1.A_STRATS, (C1.PRE_YIELD, C1.REACT))
        kinds = set()
        for a, b in pure:
            if contact[(a, b)]:
                kinds.add("!")
            elif not went[(a, b)]:
                kinds.add("W")
            else:
                y = b == C1.PRE_YIELD or went[("cont", a)] == "yield"
                kinds.add(("Y" if y else "y") if a == C1.GO_IF_YIELD else ("G" if y else "g"))
        lab = "!" if "!" in kinds else "C" if len(kinds) > 1 else (kinds.pop() if kinds else ("M" if mixed else "?"))
        sol = C1.play(d0, dv, 12.0, car, car, pref, pref)
        lab1 = C1.label(sol)
        agree += lab == lab1
        fmt = lambda ps: " ".join(f"{a}/{b}" for a, b in ps) or "-"
        print(f"{d0:5.1f} {dv:+4.1f} | {lab:5s}  {fmt(pure):40.40s} | {lab1:5s}  {fmt(sol.pure):40.40s} | {sd:.3f}")
    print(f"class agrees in {agree} of {len(contexts)} contexts")


if __name__ == "__main__":
    main()
