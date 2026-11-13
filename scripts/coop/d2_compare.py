#!/usr/bin/env python3
"""Roadmap 074 D2: does the arena measure what swb measures? C2b's swb runs against the arena's
runs of the same cells and rules (c2b.py --arena-port).

For each (cell, rule pair): the arena's mean and standard deviation over its seeds, swb's runs,
and whether each swb payoff falls in the arena's 95 % prediction interval (mean +- 2 sd, at least
+- 0.02 so that a deterministic cell is not a zero-width target); and whether the outcomes
(contact, A merged ahead) agree.

  scripts/coop/d2_compare.py <dir with c2b.jsonl and arena.jsonl>
"""
import json
import statistics as st
import sys
from collections import defaultdict


def load(p):
    return [json.loads(l) for l in open(p) if l.strip()]


def main():
    d = sys.argv[1]
    swb = [r for r in load(f"{d}/c2b.jsonl") if "uA" in r]
    ar = load(f"{d}/arena.jsonl")
    g = defaultdict(list)
    for r in ar:
        g[(r["d0"], r["dv"], r["a"], r["b"])].append(r)
    inside = total = agree = n_out = 0
    print("cell (d0, dv)  rules            | arena uA mean+-sd   uB mean+-sd   ahead contact | swb uA     uB     ahead contact | in")
    for r in swb:
        k = (r["d0"], r["dv"], r["a"], r["b"])
        rows = g.get(k, [])
        if not rows:
            continue
        line = []
        ok_all = True
        for m in ("uA", "uB"):
            xs = [float(x[m]) for x in rows]
            mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
            half = max(2 * sd, 0.02)
            ok = abs(float(r[m]) - mu) <= half
            inside += ok
            total += 1
            ok_all &= ok
            line.append((mu, sd))
        a_ahead = st.mean(x["ahead"] for x in rows)
        a_con = st.mean(x["contact"] for x in rows)
        same = (round(a_ahead) == r["ahead"]) and (round(a_con) == r["contact"])
        agree += same
        n_out += 1
        print(f"({r['d0']:>4}, {r['dv']:>4})  {r['a']:>9}/{r['b']:<7} | {line[0][0]:+.3f}+-{line[0][1]:.3f}  "
              f"{line[1][0]:+.3f}+-{line[1][1]:.3f}  {a_ahead:.2f}  {a_con:.2f}   | {float(r['uA']):+.3f} {float(r['uB']):+.3f} "
              f"{r['ahead']}     {r['contact']}       | {'yes' if ok_all else 'NO'}{'' if same else ' (outcome differs)'}"
              f"  ctx {r.get('d0_real')}/{r.get('dv_real')}")
    print(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %);"
          f" outcomes agree in {agree} of {n_out} runs")


if __name__ == "__main__":
    main()
