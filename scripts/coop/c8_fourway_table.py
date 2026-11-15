#!/usr/bin/env python3
"""Roadmap 074 C8: the four-way stop's tables, kinematic and arena, from four_way_game.py's
four_way_payoffs.csv (run with --c8) and four_way_arena.py's runs.csv.

  self-play  per rule against itself: contacts and deadlocks over the dt grid, mean welfare near the
             tie (|dt| <= NEAR) and away from it (|dt| >= AWAY)
  BAR        (c) / (e) in one place against a rule X in the other, beside stock yield in that place
             against the same X: contacts, deadlocks, the convention car's mean payoff and the other's;
             never worse = no more contacts and no more deadlocks than stock
  cost       (c) / (e) self-play welfare minus fcfsr's and stop1y's, near and away (C7's yardstick)

Usage:
  scripts/coop/c8_fourway_table.py --kin <payoffs.csv> [--arena <runs.csv> ...] --out <file>
"""

from __future__ import annotations

import argparse
import csv
import statistics as st
import sys

NEAR, AWAY = 0.5, 1.5
MINE = ("fcfsm_c1.5", "fcfse_c1.5")
STOCK = "yield_c1.5"
OTHERS = ("aggr", "yield_c1.5", "stop1y_c1.5", "fcfs_c1.5", "fcfsr_c1.5", "rbl_c1.5")
SELF = ("aggr", "yield_c1.5", "stop1y_c1.5", "fcfs_c1.5", "fcfsr_c1.5", "rbl_c1.5", "fcfsm_c1.5", "fcfse_c1.5")


def load(path, kind):
    rows = []
    for r in csv.DictReader(open(path)):
        rows.append({"v": float(r["v"]), "dt": float(r["dt"]), "a": r["a"], "b": r["b"], "uA": float(r["uA"]),
                     "uB": float(r["uB"]), "contact": int(r["contact"]), "deadlock": int(r["deadlock"])})
    return rows


def cell(rows, a, b, v, pred=lambda dt: True):
    rs = [r for r in rows if r["a"] == a and r["b"] == b and r["v"] == v and pred(r["dt"])]
    if not rs:
        return None
    return {"n": len(rs), "contact": sum(r["contact"] for r in rs), "deadlock": sum(r["deadlock"] for r in rs),
            "uA": st.mean(r["uA"] for r in rs), "uB": st.mean(r["uB"] for r in rs),
            "W": st.mean(r["uA"] + r["uB"] for r in rs),
            "cdt": sorted({r["dt"] for r in rs if r["contact"]}), "ddt": sorted({r["dt"] for r in rs if r["deadlock"]})}


def fmt_dts(xs):
    return " ".join(f"{x:g}" for x in xs) or "-"


def tables(label, rows):
    out = [f"## {label}"]
    for v in sorted({r["v"] for r in rows}):
        out.append(f"### v {v:g}: self-play (contacts / deadlocks over runs; welfare near |dt|<={NEAR:g}, away |dt|>={AWAY:g})")
        for n in SELF:
            c = cell(rows, n, n, v)
            if not c:
                continue
            cn = cell(rows, n, n, v, lambda d: abs(d) <= NEAR)
            ca = cell(rows, n, n, v, lambda d: abs(d) >= AWAY)
            out.append(f"{n:>12} | contact {c['contact']:>3}/{c['n']:<4} deadlock {c['deadlock']:>3} | W near "
                       f"{cn['W'] if cn else float('nan'):+.3f} away {ca['W'] if ca else float('nan'):+.3f} | contact dt "
                       f"{fmt_dts(c['cdt'])} | deadlock dt {fmt_dts(c['ddt'])}")
        out.append(f"### v {v:g}: BAR (the rule in a place against X; stock = {STOCK} in that place)")
        for mine in MINE:
            for place in "AB":
                for x in OTHERS:
                    a, b = (mine, x) if place == "A" else (x, mine)
                    sa, sb = (STOCK, x) if place == "A" else (x, STOCK)
                    c, s = cell(rows, a, b, v), cell(rows, sa, sb, v)
                    if not c or not s:
                        continue
                    um, uo = (c["uA"], c["uB"]) if place == "A" else (c["uB"], c["uA"])
                    sm, so = (s["uA"], s["uB"]) if place == "A" else (s["uB"], s["uA"])
                    ok = c["contact"] <= s["contact"] and c["deadlock"] <= s["deadlock"]
                    out.append(f"{mine:>11} in {place} vs {x:<12} | contact {c['contact']:>3} (stock {s['contact']:>3}) deadlock "
                               f"{c['deadlock']:>3} (stock {s['deadlock']:>3}) of {c['n']:>3} | u_me {um:+.3f} (stock {sm:+.3f}) "
                               f"u_other {uo:+.3f} (stock {so:+.3f}) | {'never worse' if ok else 'WORSE'} | contact dt "
                               f"{fmt_dts(c['cdt'])}")
        out.append(f"### v {v:g}: cost of (c)/(e) in self-play, welfare difference")
        for mine in MINE:
            for ref in ("fcfsr_c1.5", "stop1y_c1.5", "yield_c1.5"):
                for nm, pred in (("near", lambda d: abs(d) <= NEAR), ("away", lambda d: abs(d) >= AWAY),
                                 ("all", lambda d: True)):
                    c, s = cell(rows, mine, mine, v, pred), cell(rows, ref, ref, v, pred)
                    if c and s:
                        out.append(f"{mine:>11} - {ref:<12} {nm:<4}: {c['W'] - s['W']:+.3f} (per car {(c['W'] - s['W']) / 2:+.3f})")
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--kin", action="append", default=[])
    p.add_argument("--arena", action="append", default=[])
    p.add_argument("--out", required=True)
    a = p.parse_args()
    lines = []
    for path in a.kin:
        lines += tables(f"kinematic {path}", load(path, "kin"))
    for path in a.arena:
        lines += tables(f"arena {path}", load(path, "arena"))
    open(a.out, "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
