#!/usr/bin/env python3
"""Roadmap 074: the impact of arena.Pair's merge-end fix (found in C6).

Until the fix, arena.Pair ended a run past the 20 s horizon at the planned end of A's lane change
(decision + prepare + lane change) and scored got_in on the measured lateral (>= 0.9 of a lane).
CARLA's car steers about 0.6 s behind its command, so a merge planned to end after about 19.4 s was
cut short with A at 0.7-0.85 of a lane and scored as never in (lost_a = 1, uA about -1). The run
now goes on arena.MERGE_SETTLE (1.5 s) past the planned end; `arena.LEGACY_MERGE_END = True`
restores the old end. With the fix on, each arena record also carries what the old end would have
scored on the same run (`uA_legacy`, ...), so the impact is a paired difference, free of CARLA's
run-to-run variation.

Modes:
  --scan <dir>...      offline: in every arena.jsonl under the dirs, the runs the old end mis-scored
                       (A moved laterally, no contact, not in, the run ended at or past the horizon)
  --c3fit              re-run only the flagged cells of an old c3_fit directory with the fix, splice
                       them into its rows, rescore against Autoware (c3_fit.score), compare rankings
  --c2b                re-run C2b's arena cells (c2b.arena_compare) with the fix; D2 against swb
  --do6 <pops.json>    re-evaluate do6's final table (its A and B pools, DO.CONTEXTS x 2 seeds, the
                       table's own seeds) with the fix and its shadow; the equilibria of both
  --selftest

  scripts/coop/merge_end.py --scan <nas>/2026-11-15\\ coop\\ do5\\ refit
  scripts/coop/merge_end.py --c3fit --old <fit dir> --c3 <dirs> --obs gt --stop 0 --port 2600 --out <dir>
  scripts/coop/merge_end.py --c2b --old <c2b dir> --port 2600 --out <dir>
  scripts/coop/merge_end.py --do6 <populations.json> --ports 2600,2610,2620 --out <dir> [--seeds 2 --salt 0]
"""

from __future__ import annotations

import argparse
import collections
import glob
import json
import os
import statistics as st
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

HORIZON = 20.0


def misscored(r, horizon=HORIZON):
    """A run the legacy end scored as never in while A was changing lanes: A's lateral move had
    begun (t_lat), no contact, got_in 0, and the run ended at or past the horizon (a run cut at its
    planned merge end, or at the horizon with the lane change planned to end just before it)."""
    return (r.get("t_lat") is not None and not int(r.get("contact", 0)) and not int(r.get("got_in", 0))
            and float(r.get("t_run", 0.0)) >= horizon - 1e-6)


def scan(dirs):
    """{arena.jsonl path: {(d0, dv, a): [flagged, runs of that cell]}} over the dirs."""
    out = {}
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(d, "**", "arena.jsonl"), recursive=True)):
            rows = [json.loads(line) for line in open(f) if line.strip()]
            cells = collections.defaultdict(lambda: [0, 0])
            for r in rows:
                k = (r["d0"], r["dv"], r["a"])
                cells[k][1] += 1
                cells[k][0] += misscored(r)
            out[f] = {k: v for k, v in cells.items() if v[0]}
    return out


def splice(old_rows, new_rows, cells):
    """old_rows with every run of the cells (d0, dv, a) replaced by new_rows' runs of them."""
    keep = [r for r in old_rows if (r["d0"], r["dv"], r["a"]) not in cells]
    return keep + [r for r in new_rows if (r["d0"], r["dv"], r["a"]) in cells]


def legacy_view(rows):
    """The rows as the legacy end scored them (their `*_legacy` shadow)."""
    out = []
    for r in rows:
        q = dict(r)
        for k in ("uA", "uB", "got_in", "ahead", "contact", "t_run"):
            if f"{k}_legacy" in r and r[f"{k}_legacy"] is not None:
                q[k] = r[f"{k}_legacy"]
        out.append(q)
    return out


def flagged_cells(rows):
    return sorted({(r["d0"], r["dv"], r["a"]) for r in rows if misscored(r)})


# --- C3's refits ---------------------------------------------------------------------------------
def c3fit(a):
    import c2b
    import c3_fit as F
    import double_oracle as DO
    target = F.autoware_means(a.c3)
    old = Path(a.old)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    c2b.VB = F.VB
    obs = {"gt": F.OBS_GT, "cp": {}}[a.obs]
    rules = sorted(p.name for p in old.iterdir() if (p / "arena.jsonl").exists())
    res = []
    for name in rules:
        rows = [json.loads(line) for line in open(old / name / "arena.jsonl")]
        cells = flagged_cells(rows)
        d = out / name
        d.mkdir(exist_ok=True)
        if cells and not (d / "arena.jsonl").exists():
            # B's rule from its name (c3_fit's grid): B_r<gap>_b<b_y>_t<react>[_s]
            parts = name.split("_")
            gap, by, rt = float(parts[1][1:]), float(parts[2][1:]), float(parts[3][1:])
            cand = {"name": name, "pre_band": -99.0, "react_gap": gap, "b_y": by, "stop": int(name.endswith("_s"))}
            a_by = {"A_eq": c2b.A_EQ, "A_aw_perm": DO.A_AW_PERM}
            c2b.CELLS = sorted({(c[0], c[1]) for c in cells})
            c2b.PAIRS = [(a_by[x], cand) for x in sorted({c[2] for c in cells})]
            c2b.arena_compare(a.port, str(d), seeds=a.seeds, obs=obs, react_b=rt)
        new = [json.loads(line) for line in open(d / "arena.jsonl")] if cells else []
        new = [r for r in new if (r["d0"], r["dv"], r["a"]) in set(cells)]
        fixed = splice(rows, new, set(cells))
        e_old, det_old = F.score(rows, target)
        e_fix, det_fix = F.score(fixed, target)
        e_shadow, _ = F.score(splice(rows, legacy_view(new), set(cells)), target)
        cell_rows = {str(c): {"old": [round(x, 3) for x in det_old[c][1]] if c in det_old else None,
                              "fixed": [round(x, 3) for x in det_fix[c][1]] if c in det_fix else None,
                              "old_err": det_old.get(c, (None,))[0], "fixed_err": det_fix.get(c, (None,))[0],
                              "flagged_old": sum(misscored(r) for r in rows if (r["d0"], r["dv"], r["a"]) == c),
                              "flagged_new_legacy": sum(misscored(r) for r in legacy_view(new) if (r["d0"], r["dv"], r["a"]) == c),
                              "got_in_new": [int(r["got_in"]) for r in new if (r["d0"], r["dv"], r["a"]) == c],
                              "t_plan_new": [r.get("t_plan") for r in new if (r["d0"], r["dv"], r["a"]) == c]}
                     for c in cells}
        edge = (14.0, -3.0, "A_aw_perm")  # the contact-edge cell that carries most of the error
        t11 = {k: v for k, v in target.items() if k != edge}
        res.append({"rule": name, "error_old": round(e_old, 4), "error_fixed": round(e_fix, 4),
                    "error_shadow_legacy": round(e_shadow, 4),
                    "error_old_11": round(F.score(rows, t11)[0], 4), "error_fixed_11": round(F.score(fixed, t11)[0], 4),
                    "cells": cell_rows})
        print(f"{name:20s} old {e_old:.4f}  fixed {e_fix:.4f}  (new runs, legacy end {e_shadow:.4f})  {cell_rows}", flush=True)
    rank = lambda key: [r["rule"] for r in sorted(res, key=lambda r: r[key])]
    summary = {"best_old": rank("error_old")[0], "best_fixed": rank("error_fixed")[0],
               "rank_old": rank("error_old")[:5], "rank_fixed": rank("error_fixed")[:5],
               "best_old_11": rank("error_old_11")[0], "best_fixed_11": rank("error_fixed_11")[0],
               "rank_fixed_11": rank("error_fixed_11")[:5], "rules": res}
    json.dump(summary, open(out / "fit_fixed.json", "w"), indent=1)
    # c3_fit's format (the atlas's fit loader): the rules by their error with the fix
    json.dump([{"error": r["error_fixed"], "rule": r["rule"], "cells": r["cells"]}
               for r in sorted(res, key=lambda r: r["error_fixed"])], open(out / "fit.json", "w"), indent=1)
    print("best old", summary["best_old"], "best fixed", summary["best_fixed"])
    return summary


# --- C2b -----------------------------------------------------------------------------------------
def c2b_check(a):
    import shutil
    import subprocess
    import c2b
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(Path(a.old) / "c2b.jsonl", out / "c2b.jsonl")
    if not (out / "arena.jsonl").exists():
        c2b.arena_compare(a.port, str(out))
    rows = [json.loads(line) for line in open(out / "arena.jsonl")]
    print("flagged under the legacy end (shadow):", flagged_cells(legacy_view(rows)))
    print("runs whose end moved:", sum(1 for r in rows if r.get("t_run") != r.get("t_run_legacy")))
    rep = subprocess.run([sys.executable, str(HERE / "d2_compare.py"), str(out)], capture_output=True, text=True).stdout
    (out / "d2_compare.txt").write_text(rep)
    print(rep.strip().splitlines()[-3:])


# --- do6 -----------------------------------------------------------------------------------------
def tables(runs, pop_a, pop_b, legacy=False):
    """Mean uA, uB, ahead per rule pair from per-run rows (or their legacy shadow)."""
    sfx = "_legacy" if legacy else ""
    t = {}
    for a in pop_a:
        for b in pop_b:
            rs = list(runs[(a["name"], b["name"])].values())
            t[(a["name"], b["name"])] = {m: st.mean(float(r[m + sfx]) for r in rs) for m in ("uA", "uB", "ahead")}
    return t


def equilibria(t, pop_a, pop_b):
    import numpy as np
    import double_oracle as DO
    A = np.array([[t[(a["name"], b["name"])]["uA"] for b in pop_b] for a in pop_a])
    B = np.array([[t[(a["name"], b["name"])]["uB"] for b in pop_b] for a in pop_a])
    eqs = DO.nash_all(A, B)
    out = []
    for P, Q in eqs:
        out.append({"A": {a["name"]: round(float(w), 3) for a, w in zip(pop_a, P) if w > 1e-3},
                    "B": {b["name"]: round(float(w), 3) for b, w in zip(pop_b, Q) if w > 1e-3},
                    "vA": round(float(P @ A @ Q), 4), "vB": round(float(P @ B @ Q), 4),
                    "welfare": round(float(P @ (A + B) @ Q), 4)})
    out.sort(key=lambda e: -e["welfare"])
    return out


def do6(a):
    import multiprocessing as mp
    import double_oracle as DO
    pops = json.load(open(a.do6))
    pop_a, pop_b = pops["A"], pops["B"]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    f = out / f"runs_s{a.seeds}_salt{a.salt}.jsonl"
    if not f.exists():
        mp.set_start_method("spawn", force=True)
        pool = DO.Pool([int(p) for p in a.ports.split(",")], a.k)
        try:
            _, runs = pool.evaluate([(x, y) for x in pop_a for y in pop_b], seeds=a.seeds, salt=a.salt, per_run=True)
        finally:
            pool.close()
        with open(f, "w") as fh:
            for key, rr in runs.items():
                for (ctx, s), r in rr.items():
                    fh.write(json.dumps({"a": key[0], "b": key[1], "d0": ctx[0], "dv": ctx[1], "seed": s, **r}) + "\n")
    runs = collections.defaultdict(dict)
    for line in open(f):
        r = json.loads(line)
        runs[(r["a"], r["b"])][((r["d0"], r["dv"]), r["seed"])] = r
    t_fix, t_leg = tables(runs, pop_a, pop_b), tables(runs, pop_a, pop_b, legacy=True)
    old = {tuple(k.split("|")): v for k, v in pops["table"].items()}
    late = collections.Counter()
    moved = collections.Counter()
    for key, rr in runs.items():
        for (ctx, s), r in rr.items():
            if r.get("t_plan") is not None and r["t_plan"] > HORIZON - 1.5:
                late[(key[0], ctx)] += 1
            if r["uA"] != r["uA_legacy"] or r["uB"] != r["uB_legacy"]:
                moved[(key[0], key[1], ctx, int(r["got_in_legacy"]), int(r["got_in"]))] += 1
    res = {"seeds": a.seeds, "salt": a.salt,
           "eq_fixed": equilibria(t_fix, pop_a, pop_b), "eq_legacy_shadow": equilibria(t_leg, pop_a, pop_b),
           "eq_do6_table": equilibria(old, pop_a, pop_b),
           "late_plans": {f"{k[0]} {k[1]}": v for k, v in sorted(late.items())},
           "runs_moved": {f"{k[0]}|{k[1]} {k[2]} in {k[3]}->{k[4]}": v for k, v in sorted(moved.items())},
           "table": {f"{x}|{y}": {"fixed": {m: round(v, 4) for m, v in t_fix[(x, y)].items()},
                                  "legacy": {m: round(v, 4) for m, v in t_leg[(x, y)].items()},
                                  "do6": {m: round(old[(x, y)][m], 4) for m in ("uA", "uB", "ahead")}}
                     for x in [p["name"] for p in pop_a] for y in [p["name"] for p in pop_b]}}
    json.dump(res, open(out / f"do6_fixed_s{a.seeds}_salt{a.salt}.json", "w"), indent=1)
    for k in ("eq_do6_table", "eq_legacy_shadow", "eq_fixed"):
        print(k, json.dumps(res[k][:2]))
    print("late plans", res["late_plans"])
    print("moved", res["runs_moved"])
    return res


def selftest():
    import arena as AR
    # the end rule: legacy ends at the planned end, the fix 1.5 s later; nothing ends before the
    # horizon (but contact and T_MAX); an undecided A that may still go keeps the run going
    assert AR.run_ends(20.05, 20.0, False, False, 0.0)
    assert not AR.run_ends(20.05, 20.0, False, False, AR.MERGE_SETTLE)
    assert AR.run_ends(21.5, 20.0, False, False, AR.MERGE_SETTLE)
    assert not AR.run_ends(19.0, 10.0, False, False, AR.MERGE_SETTLE)
    assert AR.run_ends(20.0, 10.0, False, False, AR.MERGE_SETTLE)
    assert AR.run_ends(20.0, None, False, False, AR.MERGE_SETTLE)
    assert not AR.run_ends(20.0, None, True, False, AR.MERGE_SETTLE)
    assert AR.run_ends(5.0, None, True, True, AR.MERGE_SETTLE)
    assert AR.run_ends(40.0, 39.5, False, False, AR.MERGE_SETTLE)
    assert not AR.LEGACY_MERGE_END and AR.MERGE_SETTLE == 1.5
    # misscored: the 6/0 rows of C3's refits (t_lat 17.45, t_run 20.05, got_in 0)
    r = {"d0": 6.0, "dv": 0.0, "a": "A_aw_perm", "t_lat": 17.45, "t_run": 20.05, "got_in": 0, "contact": 0}
    assert misscored(r)
    assert not misscored(dict(r, got_in=1)) and not misscored(dict(r, contact=1))
    assert not misscored(dict(r, t_lat=None)) and not misscored(dict(r, t_run=12.0))
    # splice and the legacy view
    old = [dict(r, uA=-1.04), dict(r, a="A_eq", uA=-0.27)]
    new = [dict(r, uA=-0.27, uA_legacy=-1.04, got_in=1, got_in_legacy=0)]
    sp = splice(old, new, {(6.0, 0.0, "A_aw_perm")})
    assert sorted(x["uA"] for x in sp) == [-0.27, -0.27]
    lv = legacy_view(new)
    assert lv[0]["uA"] == -1.04 and lv[0]["got_in"] == 0 and flagged_cells(lv) == [(6.0, 0.0, "A_aw_perm")]
    # tables and equilibria on a toy 2x2 (prisoner's dilemma: one pure equilibrium)
    pa, pb = [{"name": "a0"}, {"name": "a1"}], [{"name": "b0"}, {"name": "b1"}]
    pay = {("a0", "b0"): (3, 3), ("a0", "b1"): (0, 4), ("a1", "b0"): (4, 0), ("a1", "b1"): (1, 1)}
    runs = {k: {((0.0, 0.0), 0): {"uA": v[0], "uB": v[1], "ahead": 0, "uA_legacy": v[0], "uB_legacy": v[1],
                                   "ahead_legacy": 0}} for k, v in pay.items()}
    eq = equilibria(tables(runs, pa, pb), pa, pb)
    assert len(eq) == 1 and eq[0]["A"] == {"a1": 1.0} and eq[0]["B"] == {"b1": 1.0}
    print("merge_end selftest: ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", nargs="+")
    ap.add_argument("--c3fit", action="store_true")
    ap.add_argument("--c2b", action="store_true")
    ap.add_argument("--do6")
    ap.add_argument("--old")
    ap.add_argument("--c3", nargs="+")
    ap.add_argument("--obs", default="gt")
    ap.add_argument("--stop", type=int, default=0)
    ap.add_argument("--port", type=int, default=2600)
    ap.add_argument("--ports", default="2600,2610,2620")
    ap.add_argument("--k", type=int, default=4)
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--salt", type=int, default=0)
    ap.add_argument("--out")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    elif a.scan:
        for f, cells in scan(a.scan).items():
            if cells:
                print(f, {f"{k[0]:g}/{k[1]:g} {k[2]}": f"{v[0]}/{v[1]}" for k, v in cells.items()})
    elif a.c3fit:
        c3fit(a)
    elif a.c2b:
        c2b_check(a)
    elif a.do6:
        do6(a)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
