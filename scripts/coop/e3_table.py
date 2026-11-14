#!/usr/bin/env python3
"""Roadmap 074 E3: the knob-to-behaviour table from e3_knobs.sh's runs.

    python3 scripts/coop/e3_table.py <runs dir> [--params <params dir>] [--base stock]

Reads <runs>/<variant>/c3.jsonl (one row per cell x A rule x seed, scored by c2b.score with the
general utility, S0) and prints, per variant, each cell's outcome next to the base variant's:
A ahead or behind, uA, uB, minimum body clearance, footprint overlap, B's peak braking demand
(dracB) and the time B starts braking. A variant "moves" a cell when A's order flips, uB moves by
more than --du, or clearance by more than --dclear. Writes table.md and table.json beside the runs.
"""
import argparse
import json
import os
import statistics as st

import yaml

FIELDS = ("ahead", "uA", "uB", "clear", "pen", "thw", "dracB", "t_brakeB", "react_b", "stopsB", "jerkB")


def load(runs, v):
    p = os.path.join(runs, v, "c3.jsonl")
    if not os.path.exists(p):
        return {}
    out = {}
    for line in open(p):
        r = json.loads(line)
        out.setdefault((r["d0"], r["dv"], r["a"]), []).append(r)
    return out


def agg(rows):
    m = {}
    for f in FIELDS:
        xs = [r.get(f) for r in rows if isinstance(r.get(f), (int, float))]
        m[f] = round(st.mean(xs), 3) if xs else None
    m["n"] = len(rows)
    m["contact"] = sum(1 for r in rows if r.get("contact"))
    m["errors"] = sum(1 for r in rows if r.get("error") or r.get("cause"))
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs")
    ap.add_argument("--params", default="")
    ap.add_argument("--base", default="stock")
    ap.add_argument("--du", type=float, default=0.02)
    ap.add_argument("--dclear", type=float, default=0.3)
    a = ap.parse_args()
    variants = (list(yaml.safe_load(open(os.path.join(a.params, "variants.yaml"))))
                if a.params else sorted(d for d in os.listdir(a.runs) if os.path.isdir(os.path.join(a.runs, d))))
    data = {v: {k: agg(rs) for k, rs in load(a.runs, v).items()} for v in variants}
    base = data.get(a.base, {})
    keys = sorted({k for d in data.values() for k in d})
    lines = ["| variant | cell (d0, dv) | A rule | A ahead | uA | uB | clear m | pen m | dracB | B brakes at s | moves vs " + a.base + " |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    summary = {}
    for v in variants:
        moved = 0
        for k in keys:
            m = data[v].get(k)
            if not m:
                continue
            b = base.get(k)
            why = []
            if b and v != a.base:
                if m["ahead"] is not None and b["ahead"] is not None and round(m["ahead"]) != round(b["ahead"]):
                    why.append("order")
                if m["uB"] is not None and b["uB"] is not None and abs(m["uB"] - b["uB"]) > a.du:
                    why.append(f"uB {m['uB'] - b['uB']:+.3f}")
                if m["clear"] is not None and b["clear"] is not None and abs(m["clear"] - b["clear"]) > a.dclear:
                    why.append(f"clear {m['clear'] - b['clear']:+.2f}")
                if m["contact"] != b["contact"]:
                    why.append("contact")
            moved += bool(why)
            lines.append(f"| {v} | {k[0]:g}, {k[1]:+g} | {k[2]} | {m['ahead']} | {m['uA']} | {m['uB']} | {m['clear']} | "
                         f"{m['pen']} | {m['dracB']} | {m['t_brakeB']} | {', '.join(why)} |")
        summary[v] = {"cells_moved": moved, "cells": len(data[v])}
    md = "\n".join(lines) + "\n\n" + "\n".join(f"- {v}: moves {s['cells_moved']} of {s['cells']}" for v, s in summary.items()) + "\n"
    open(os.path.join(a.runs, "table.md"), "w").write(md)
    json.dump({"rows": {v: {f"{k[0]:g},{k[1]:+g},{k[2]}": m for k, m in d.items()} for v, d in data.items()},
               "summary": summary}, open(os.path.join(a.runs, "table.json"), "w"), indent=1)
    print(md)


if __name__ == "__main__":
    main()
