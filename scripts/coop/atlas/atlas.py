#!/usr/bin/env python3
"""Roadmap 074, the strategy atlas in one command: export (sources.yaml -> atlas.json), then build
(atlas.json -> atlas.html), into one output directory.

Usage:
  python3 scripts/coop/atlas/atlas.py --out <dir> [--sources <yaml>] [--only cutin,crossing]
  python3 scripts/coop/atlas/atlas.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import atlas_build  # noqa: E402
import atlas_export  # noqa: E402


def run(sources, out, only=None):
    atlas_export.export(sources, out, only)
    n = atlas_build.build(os.path.join(out, "atlas.json"), os.path.join(out, "atlas.html"))
    print(f"{os.path.join(out, 'atlas.html')}: {n / 1e6:.2f} MB")


def _fixture(d):
    """A tiny result of every format the selftest loads."""
    d = Path(d)
    rows = []
    for d0, dv, cls in ((2.0, 0.0, "G"), (6.0, 0.0, "W")):
        r = {"d0": d0, "dv": dv, "class": cls, "pure": "go/react" if cls == "G" else "wait/react", "mixed": "",
             "C_opt": 0.1, "C_worst_eq": 0.15}
        for a in ("go", "wait"):
            for b in ("pre_yield", "react"):
                r[f"uA_{a}_{b}"] = -0.1 if a == "go" else -0.2
                r[f"uB_{a}_{b}"] = -0.05 if b == "react" else -0.1
        rows.append(r)
    with open(d / "cutin_map.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(d / "runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["dt", "v", "a", "b", "seed", "uA", "uB", "contact"])
        w.writeheader()
        for a, b, ua, ub in (("go", "hold", -0.1, -0.3), ("go", "react", -0.1, -0.05), ("wait", "hold", -0.3, -0.02),
                             ("wait", "react", -0.4, -0.03)):
            w.writerow({"dt": 0.0, "v": 10.0, "a": a, "b": b, "seed": 0, "uA": ua, "uB": ub, "contact": 0})
    (d / "do").mkdir()
    with open(d / "do" / "log.jsonl", "w") as f:
        f.write(json.dumps({"round": 0, "A_mix": {"A_x": 1.0}, "B_mix": {"B_y": 1.0}, "vA": -0.1, "vB": -0.1,
                            "BR_A": {"name": "A_br", "tau": 1.0}, "gain_A": 0.01, "BR_B": None, "gain_B": 0.0}) + "\n")
        f.write(json.dumps({"round": 1, "A_mix": {"A_br": 1.0}, "B_mix": {"B_y": 1.0}, "vA": -0.09, "vB": -0.1,
                            "BR_A": None, "gain_A": 0.0, "BR_B": None, "gain_B": 0.0}) + "\n")
        f.write(json.dumps({"round": 1, "converged": True}) + "\n")
    json.dump({"A": [{"name": "A_x", "tau": 2.0}, {"name": "A_br", "tau": 1.0}], "B": [{"name": "B_y", "b_y": 2.0}],
               "table": {"A_x|B_y": {"uA": -0.1, "uB": -0.1, "contact": 0, "ahead": 0},
                         "A_br|B_y": {"uA": -0.09, "uB": -0.1, "contact": 0, "ahead": 1}}},
              open(d / "do" / "populations.json", "w"))
    with open(d / "c3.jsonl", "w") as f:
        for s in range(2):
            f.write(json.dumps({"d0": 6.0, "dv": 0.0, "a": "A_eq", "b": "autoware", "seed": s, "uA": -0.2, "uB": -0.01,
                                "contact": 0, "ahead": 0}) + "\n")
    with open(d / "n3_cells.jsonl", "w") as f:
        for d0 in (2.0, 6.0):
            f.write(json.dumps({"set": "ho", "model": "n3 shift+residual", "r": 0.01 * d0, "tol": 0.02, "within": d0 < 5,
                                "cls": "q:A:drop", "u": -0.3, "pos": "behind", "pos_ab": "behind",
                                "key": ["ho", "a5", d0, 0.0, 1.0, 12.0, "A_aw_perm", "A"]}) + "\n")
    json.dump({"near|none": {"classes": "W3", "restart_share": 0.4, "restart": 4, "waited": 10, "contact_share": 0.2,
                             "wait_pairs": 2.0, "wait_eq": 1.0, "loss_worst": 0.1, "C_mean_pairs": 3.0, "recovered_eq": None,
                             "n_gap_eq": 0, "self_enforcing": 1.0}}, open(d / "tb.json", "w"))
    json.dump({"zipper|1.2": {"rate_vpm": 30.0, "t_last": 80.0, "queue_max": 120.0, "queue_mean": 70.0, "delay": 18.0,
                              "delay_l1": 9.0, "delay_l2": 30.0, "forced": 2.0, "hard": 3.0, "contact": 0.0, "rules": "z0.6yd"}},
              open(d / "zs.json", "w"))
    hit = {"hit": 1, "cells": 2, "share": 0.5}
    m = {"mean": 0.01, "max": 0.02, "welfare_mean": 0.0, "vs_stock": hit, "vs_worst_eq": hit, "prior_weighted_vs_stock": hit}
    json.dump({"selfish": {"regret_class": m, "C": {"prior": 0.1, "truth": 0.09}}, "prosocial": {"regret_class": m, "C": {}},
               "L_stock_mean": 0.04, "L_worst_mean": 0.06, "by_type": {}, "confusion": {"probe class@0": {"m3/hold": {"m3/hold": 1.0}}},
               "assumed": {"s | A m3 | assumed m3/hold | true m3/hold": {"dU": 0.0, "dC": 0.0, "contact": 0.0, "worst_dU": 0.0}}},
              open(d / "c6_summary.json", "w"))
    with open(d / "c6_cells.jsonl", "w") as f:
        for d0 in (2.0, 4.0):
            f.write(json.dumps({"d0": d0, "dv": 0.0, "vb": 12.0, "capA": "m3", "true": "m3/hold", "L_stock": 0.05,
                                "L_worst": 0.06, "s_regret_class": 0.01 * d0, "s_regret_class_se": 0.001}) + "\n")
    src = {
        "tiers": ["kinematic", "arena", "swb_autoware", "double_oracle"],
        "scenarios": {
            "cutin": {"title": "Cut-in", "axes": ["d0", "dv"], "legend": {"G": "go, B yields", "W": "drop"},
                      "rules": {"A": ["go", "wait"], "B": ["pre_yield", "react"]},
                      "tiers": {"kinematic": [{"loader": "kinematic_map", "path": str(d / "cutin_map.csv"), "payoff_cols": True},
                                              {"loader": "kinematic_resim", "game": "cutin", "fixed": {"vb": 12.0},
                                               "episodes": [{"ctx": {"d0": 6.0, "dv": 0.0}, "a": "go", "b": "react"}]}],
                                "swb_autoware": [{"loader": "c3", "path": str(d / "c3.jsonl")}],
                                "double_oracle": [{"loader": "double_oracle", "path": str(d / "do")}]}},
            "crossing": {"title": "Crossing", "axes": ["dt", "v"], "legend": {"NE": "pure NE"},
                         "tiers": {"arena": [{"loader": "arena_runs", "path": str(d / "runs.csv")}]}},
            "conventions": {"title": "Conventions", "conventions": True, "notes": ["fixture"],
                            "tiers": {"kinematic": [{"loader": "conventions", "kind": "tiebreak_kin", "path": str(d / "tb.json")},
                                                    {"loader": "conventions", "kind": "stream", "path": str(d / "zs.json")}]}},
            "beliefs": {"title": "Beliefs", "beliefs": True, "notes": ["fixture"],
                        "tiers": {"kinematic": [{"loader": "beliefs", "kind": "kin", "path": str(d / "c6_summary.json"),
                                                 "cells": str(d / "c6_cells.jsonl")}]}},
            "ncar": {"title": "N cars", "ncar": True, "tiers": {"arena": [{"loader": "n3_cells", "path": str(d / "n3_cells.jsonl")},
                                                                          {"loader": "n4", "path": str(d / "missing.jsonl")}]}},
        },
    }
    import yaml
    p = d / "sources.yaml"
    yaml.safe_dump(src, open(p, "w"))
    return p


def selftest():
    with tempfile.TemporaryDirectory() as td:
        src = _fixture(td)
        out = os.path.join(td, "out")
        atlas = atlas_export.export(src, out, log=lambda *a: None)
        S = atlas["scenarios"]["cutin"]
        assert len(S["tiers"]["kinematic"]["cells"]) == 2 and S["tiers"]["kinematic"]["cells"][0]["eq"] == [["go", "react"]]
        assert abs(S["tiers"]["kinematic"]["cells"][0]["loss"] - 0.05) < 1e-9
        assert len(S["tiers"]["kinematic"]["payoffs"][0]["pairs"]) == 4
        assert S["tiers"]["swb_autoware"]["cells"][0]["class"] == "W"
        m = S["meta"][0]
        assert m["rounds"][0]["A_pop"] == ["A_x"] and m["rounds"][1]["A_pop"] == ["A_x", "A_br"], m["rounds"]
        ep = S["episodes"][0]
        assert len(ep["t"]) > 50 and len(ep["A"]) == len(ep["t"]) and ep["refs"]["ttc"] == 3.0
        X = atlas["scenarios"]["crossing"]["tiers"]["arena"]
        assert X["cells"][0]["eq"] == [["go", "react"]], X["cells"]  # pure NE of the 2x2
        assert len(atlas["ncar"]["ncar_residuals"][0]["cells"]) == 2
        assert [c["kind"] for c in atlas["conventions"]] == ["tiebreak_kin", "stream"], atlas["conventions"]
        assert atlas["conventions"][0]["rows"][0]["conv"] == "none" and atlas["conventions"][1]["rows"][0]["hm"] == 1.2
        assert atlas["beliefs"][0]["kind"] == "kin" and len(atlas["beliefs"][0]["cells"]) == 2, atlas["beliefs"]
        assert atlas["beliefs"][0]["cells"][1]["regret_class"] == 0.04
        n = atlas_build.build(os.path.join(out, "atlas.json"), os.path.join(out, "atlas.html"))
        html = open(os.path.join(out, "atlas.html")).read()
        assert n < atlas_build.MAX_BYTES and "<title>Strategy atlas</title>" in html
        assert ':root:not([data-theme="light"])' in html and ':root[data-theme="dark"]' in html
        assert "<script src" not in html  # self-contained
        js = html.split("<script>", 1)[1].rsplit("</script>", 1)[0]
        jp = os.path.join(td, "page.js")
        open(jp, "w").write(js)
        try:
            r = subprocess.run(["node", "--check", jp], capture_output=True, text=True)
            assert r.returncode == 0, r.stderr
        except FileNotFoundError:
            pass  # no node on this host: the syntax check is skipped
    print("atlas selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out")
    ap.add_argument("--sources", default=str(HERE / "sources.yaml"))
    ap.add_argument("--only", default="")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.out:
        ap.error("--out is required")
    run(a.sources, a.out, [s for s in a.only.split(",") if s] or None)


if __name__ == "__main__":
    main()
