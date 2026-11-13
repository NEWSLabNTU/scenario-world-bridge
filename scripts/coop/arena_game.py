#!/usr/bin/env python3
"""Roadmap 074: the empirical game from the arena's runs, against C1's prediction.

Reads one or more arena `runs.csv`, averages the payoffs per (context, profile) over the seeds,
forms B's REACT as its better continuation (react_yield or react_hold, by B's mean payoff) per A
strategy, as C1 does, solves the game with C1's solver, and prints, per context: the arena's
equilibria and class, C1's, and the payoffs' spread over the seeds.

Usage:
  scripts/coop/arena_game.py [--type=<C1 type>] <runs.csv> [<runs.csv> ...]            # table
  scripts/coop/arena_game.py --grid [--boot=200] [--type=carla_model3] <runs.csv> ...  # map, bootstrap
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


def classify(cells_ctx):
    """The class and equilibria of one context from {(a, b): rows}."""
    ja, jb, went, contact = {}, {}, {}, {}
    for a in C1.A_STRATS:
        ry, rh = cells_ctx[(a, "react_yield")], cells_ctx[(a, "react_hold")]
        react, cont = (ry, "yield") if mean(ry, "uB") >= mean(rh, "uB") else (rh, "hold")
        for b, rows in ((C1.PRE_YIELD, cells_ctx[(a, "pre_yield")]), (C1.REACT, react)):
            ja[(a, b)], jb[(a, b)] = mean(rows, "uA"), mean(rows, "uB")
            went[(a, b)] = mean(rows, "ahead" if "ahead" in rows[0] else "got_in") > 0.5 and a != C1.WAIT
            contact[(a, b)] = mean(rows, "contact") > 0.5
        went[("cont", a)] = cont
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
    return lab, pure


def boot(cells_ctx, n, rng):
    """The class's bootstrap distribution: seeds resampled per profile."""
    from collections import Counter
    c = Counter()
    for _ in range(n):
        res = {k: [rng.choice(v) for _ in v] for k, v in cells_ctx.items()}
        c[classify(res)[0]] += 1
    lab, k = c.most_common(1)[0]
    return lab, k / n


def grid_main(args, nboot, ctype):
    import random
    cells = load(args)
    rng = random.Random(0)
    contexts = sorted({(d0, dv) for d0, dv, _, _ in cells})
    d0s = sorted({c[0] for c in contexts})
    dvs = sorted({c[1] for c in contexts})
    car = C1.TYPES[ctype]
    pref = C1.Preference()
    arena, conf, c1 = {}, {}, {}
    for d0, dv in contexts:
        cc = {(a, b): cells[(d0, dv, a, b)] for a in C1.A_STRATS for b in ("pre_yield", "react_yield", "react_hold")}
        lab, _ = classify(cc)
        blab, f = boot(cc, nboot, rng)
        arena[(d0, dv)], conf[(d0, dv)] = lab, f
        c1[(d0, dv)] = C1.label(C1.play(d0, dv, 12.0, car, car, pref, pref))
    print(f"# arena class (upper case where the bootstrap agrees >= 90 %, '.' marks below 60 %) | C1 ({ctype})")
    print("# columns d0 " + " ".join(f"{d:g}" for d in d0s))
    agree = sure = 0
    for dv in reversed(dvs):
        row, row1 = "", ""
        for d0 in d0s:
            lab, f = arena[(d0, dv)], conf[(d0, dv)]
            row += lab if f >= 0.9 else ("." if f < 0.6 else lab.lower() if lab.isupper() else lab)
            row1 += c1[(d0, dv)]
            if f >= 0.9:
                sure += 1
                agree += lab == c1[(d0, dv)]
        print(f"{dv:+5.1f}  {row}   |  {row1}")
    print(f"confident cells (bootstrap >= 90 %): {sure} of {len(contexts)}; of those, class equal to C1's: {agree}")


def main():
    if "--grid" in sys.argv:
        args = [a for a in sys.argv[1:] if not a.startswith("--")]
        nboot = int(next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--boot=")), "200"))
        ctype = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--type=")), "carla_model3")
        return grid_main(args, nboot, ctype)
    _table()


def _table():
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
