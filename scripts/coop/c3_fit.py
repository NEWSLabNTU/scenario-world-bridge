#!/usr/bin/env python3
"""Roadmap 074 D4: refit the arena's imitation of Autoware as B against C3's runs.

C3a showed the Autoware-like rule B_AW reacting too late: it follows A only once A is in its
lane, while Autoware brakes on the lane change's onset. Here candidate B rules from the double
oracle's family (react to A's lateral motion within `react_gap` m, braking at `b_y`) run C2b's
cells in the arena at C3's speed, and each is scored against Autoware's per-cell means:

  error = mean over (cell, A rule) of (uA - uA_aw)^2 + (uB - uB_aw)^2
          + 0.25 * [ahead differs] + 1.0 * [contact differs]   (outcome shares, not single runs)

Needs a CARLA server on --carla-port (the arena loads its own road).

  scripts/coop/c3_fit.py --c3 /mnt/p5plus/aeon/coop/c3/c3a /mnt/p5plus/aeon/coop/c3/c3a_6_0 \\
      --out /mnt/p5plus/aeon/coop/c3/fit --carla-port 2400
"""

import argparse
import collections
import itertools
import json
import os
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c2b  # noqa: E402
import double_oracle as DO  # noqa: E402

VB = 6.0
REACT_GAPS = tuple(float(x) for x in os.environ.get("FIT_GAPS", "12,20,30").split(","))
B_YS = tuple(float(x) for x in os.environ.get("FIT_BYS", "2.5,4").split(","))
REACTS = tuple(float(x) for x in os.environ.get("FIT_REACTS", "0.5,0.8,1.0").split(","))
# C3a: Autoware (B) sees ground-truth objects; A, the agent, sees through its observer (E2's
# CenterPoint numbers), as in the arena's default
OBS_GT = {"obs_b_delay": 0.0, "obs_b_sigma_x": 0.0, "obs_b_sigma_v": 0.0}


def autoware_means(dirs):
    """{(d0, dv, A rule): (uA, uB, ahead share, contact share, n)} from C3's c3.jsonl files;
    a later directory's cell replaces an earlier one's (a rerun)."""
    by = {}
    for d in dirs:
        rows = collections.defaultdict(list)
        for line in open(Path(d) / "c3.jsonl"):
            r = json.loads(line)
            if "uA" in r:
                rows[(r["d0"], r["dv"], r["a"])].append(r)
        by.update(rows)
    return {k: (st.mean(r["uA"] for r in v), st.mean(r["uB"] for r in v), st.mean(r["ahead"] for r in v),
                st.mean(r["contact"] for r in v), len(v)) for k, v in by.items()}


def score(arena_rows, target):
    rows = collections.defaultdict(list)
    for r in arena_rows:
        rows[(r["d0"], r["dv"], r["a"])].append(r)
    errs, detail = [], {}
    for k, (ua, ub, ah, co, _) in target.items():
        a = rows.get(k)
        if not a:
            continue
        m = (st.mean(r["uA"] for r in a), st.mean(r["uB"] for r in a), st.mean(r["ahead"] for r in a),
             st.mean(r["contact"] for r in a))
        e = (m[0] - ua) ** 2 + (m[1] - ub) ** 2 + 0.25 * abs(m[2] - ah) + 1.0 * abs(m[3] - co)
        errs.append(e)
        detail[k] = (round(e, 4), [round(x, 3) for x in m])
    return st.mean(errs), detail


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--c3", nargs="+", required=True, help="C3 output dirs (a later one's cells replace)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--carla-port", type=int, default=2400)
    ap.add_argument("--seeds", type=int, default=4)
    a = ap.parse_args()
    target = autoware_means(a.c3)
    os.makedirs(a.out, exist_ok=True)
    c2b.VB = VB
    results = []
    for gap, by, rt in itertools.product(REACT_GAPS, B_YS, REACTS):
        cand = {"name": f"B_r{gap:g}_b{by:g}_t{rt:g}", "pre_band": -99.0, "react_gap": gap, "b_y": by}
        d = Path(a.out) / cand["name"]
        if not (d / "arena.jsonl").exists():
            c2b.PAIRS = [(c2b.A_EQ, cand), (DO.A_AW_PERM, cand)]
            d.mkdir(parents=True, exist_ok=True)
            c2b.arena_compare(a.carla_port, str(d), seeds=a.seeds, obs=OBS_GT, react_b=rt)
        rows = [json.loads(line) for line in open(d / "arena.jsonl")]
        err, detail = score(rows, target)
        results.append((err, cand["name"], detail))
        print(f"{cand['name']:16s} error {err:.4f}", flush=True)
    results.sort()
    with open(Path(a.out) / "fit.json", "w") as f:
        json.dump([{"error": e, "rule": n, "cells": {str(k): v for k, v in d.items()}} for e, n, d in results], f, indent=1)
    print("best:", results[0][1], round(results[0][0], 4))
    for k, v in sorted(results[0][2].items()):
        print("  ", k, "arena", v[1], "autoware", [round(x, 3) for x in target[k][:4]])


if __name__ == "__main__":
    main()
