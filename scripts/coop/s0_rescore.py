#!/usr/bin/env python3
"""Roadmap 074 S0: re-score recorded swb runs (C2b, C3a, C3b) with the general utility.

Reads a run set's result file (c2b.jsonl or c3.jsonl) and each run's agent log, scores the run
again with c2b.score (which now uses measures' general risk: 2D TTC, headway, PET, clearance;
arc-length progress; lateral comfort; contact as overlapping bodies), and writes the old and new
payoffs side by side to <out>/<name>.s0.jsonl. The source files are not touched. Prints, per
cell and rule pair, the mean old and new payoffs, contact and merge order, and the new risk terms.

Usage:
  scripts/coop/s0_rescore.py --out <dir> <name>=<run dir> [...]
  e.g. c2b=/mnt/p5plus/aeon/coop/c2b/run1 c3a=/mnt/p5plus/aeon/coop/c3/c3a
"""

import argparse
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import c2b  # noqa: E402
import double_oracle as DO  # noqa: E402

KEEP = ("uA", "uB", "contact", "ahead", "got_in", "ttc")
NEW = ("ttc2d", "thw", "pet", "clear", "pen", "v_touch", "rho", "latA", "latB")


def run_dir(base, r):
    b = r.get("b", "aw")
    b = "aw" if b == "autoware" else b
    return base / f"d{r['d0']:g}_v{r['dv']:g}_{r['a']}_{b}_s{r['seed']}"


def t_prep(a):
    return {"A_eq": c2b.A_EQ, "A_aw_perm": DO.A_AW_PERM}[a]["t_prep"]


def rescore(name, base, out):
    res = base / ("c3.jsonl" if (base / "c3.jsonl").exists() else "c2b.jsonl")
    rows = []
    for line in open(res):
        if not line.strip():
            continue
        r = json.loads(line)
        if "error" in r:
            continue
        log = run_dir(base, r) / "coop_a.jsonl"
        if not log.exists():
            print(f"{name}: no log {log}", file=sys.stderr)
            continue
        new = c2b.score(log, log, r["d0"], r["dv"], t_prep(r["a"]))
        if "error" in new:
            continue
        row = {k: r[k] for k in ("d0", "dv", "a", "b", "seed")}
        row.update({f"old_{k}": r.get(k) for k in KEEP})
        row.update({k: new.get(k) for k in KEEP + NEW})
        rows.append(row)
    with open(out / f"{name}.s0.jsonl", "w") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")
    return rows


def summary(name, rows):
    g = defaultdict(list)
    for r in rows:
        g[(r["d0"], r["dv"], r["a"])].append(r)
    m = lambda rs, k: statistics.mean(float(x[k]) for x in rs)
    print(f"\n{name}: {len(rows)} runs")
    print(f"{'cell':>9} {'A rule':>10} {'n':>2} | {'uA old':>7} {'uA new':>7} {'uB old':>7} {'uB new':>7} | "
          f"{'contact':>7} {'ahead o/n':>9} | {'rho':>5} {'clear':>5} {'thw':>5} {'ttc2d':>5} {'latA':>4}")
    changed = 0
    for (d0, dv, a), rs in sorted(g.items()):
        ah_o, ah_n = m(rs, "old_ahead"), m(rs, "ahead")
        ct_o, ct_n = m(rs, "old_contact"), m(rs, "contact")
        changed += sum(1 for x in rs if x["ahead"] != x["old_ahead"] or x["contact"] != x["old_contact"])
        num = lambda k: statistics.mean(float(x[k]) if x[k] is not None else float("nan") for x in rs)
        fin = lambda k: min((x[k] for x in rs if x[k] is not None), default=None)
        f = lambda v: f"{v:5.2f}" if v is not None else "  inf"
        print(f"{d0:>4g}/{dv:<4g} {a:>10} {len(rs):>2} | {m(rs, 'old_uA'):7.3f} {m(rs, 'uA'):7.3f} "
              f"{m(rs, 'old_uB'):7.3f} {m(rs, 'uB'):7.3f} | {ct_o:.2f}/{ct_n:.2f} {ah_o:.2f}/{ah_n:.2f} | "
              f"{num('rho'):5.2f} {f(fin('clear'))} {f(fin('thw'))} {f(fin('ttc2d'))} {num('latA'):4.2f}")
    print(f"{name}: runs whose outcome (contact or merge order) changed: {changed}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("sets", nargs="+", help="name=dir")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for s in a.sets:
        name, d = s.split("=", 1)
        summary(name, rescore(name, Path(d).expanduser(), out))


if __name__ == "__main__":
    main()
