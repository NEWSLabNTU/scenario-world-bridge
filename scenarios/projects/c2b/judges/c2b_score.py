#!/usr/bin/env python3
"""c2b_score: a C2b run scored as the arena scores it, against the arena's runs of its cell
(roadmap 074 D2: does the arena measure what scensim measures?).

From the two agents' logs in the run directory (coop_a.jsonl, coop_b.jsonl), scripts/coop/
c2b.score gives each car's payoff (uA, uB) and the outcome (A merged ahead, contact). The
cell is the scenario's (its ParameterDeclarations), the rule pair the run's `pair`
parameter. The reference is the arena's runs of the same cell and rules
(reference/arena.jsonl; NAS "2026-11-15 coop route curvature"). The run passes when its
outcome is the arena's; whether each payoff falls in the arena's 95 % interval (mean +- 2 sd,
at least +- 0.02) is reported (uA_in, uB_in), as d2_compare.py counts it, not judged: 074's
reference itself is 42 of 48 (88 %), the misses explained by Town03's geometry.

    python3 judges/c2b_score.py [--reference reference/arena.jsonl] <run dir>
"""
import argparse
import json
import statistics as st
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from coop_pilots_dir import coop_dir  # noqa: E402
sys.path.insert(0, str(coop_dir()))
import c2b  # noqa: E402
import make_scenarios  # noqa: E402

PAIRS = {"eq": c2b.PAIRS[0], "aw": c2b.PAIRS[1]}


def interval(rows, m):
    xs = [float(x[m]) for x in rows]
    mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
    return mu, max(2 * sd, 0.02)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reference", default=str(HERE.parent / "reference" / "arena.jsonl"))
    ap.add_argument("run_dir")
    a = ap.parse_args()
    rd = Path(a.run_dir)
    resolved = yaml.safe_load((rd / "resolved.yaml").read_text())
    d0, dv = make_scenarios.cell(resolved["scenario"]["file"])
    pair = str((resolved["scenario"].get("parameters") or {}).get("pair", "eq"))
    pa, pb = PAIRS[pair]
    for log in ("coop_a.jsonl", "coop_b.jsonl"):
        if not (rd / log).is_file():
            print(json.dumps({"message": f"no {log} in the run: the pilot logged nothing"}))
            return 2
    r = c2b.score(rd / "coop_a.jsonl", rd / "coop_b.jsonl", d0, dv, pa["t_prep"])
    if "error" in r:
        print(json.dumps({"message": r["error"]}))
        return 2
    ref = [x for x in map(json.loads, open(a.reference)) if
           (x["d0"], x["dv"], x["a"], x["b"]) == (d0, dv, pa["name"], pb["name"])]
    if not ref:
        print(json.dumps({"message": f"no arena runs for d0 {d0} dv {dv} {pa['name']}/{pb['name']}"}))
        return 2
    out = {"d0": d0, "dv": dv, "a": pa["name"], "b": pb["name"], "arena_runs": len(ref)}
    for m in ("uA", "uB"):
        mu, half = interval(ref, m)
        out.update({m: r[m], f"{m}_arena": round(mu, 4), f"{m}_half": round(half, 4),
                    f"{m}_in": int(abs(float(r[m]) - mu) <= half)})
    ahead = round(st.mean(x["ahead"] for x in ref))
    contact = round(st.mean(x["contact"] for x in ref))
    same = r["ahead"] == ahead and r["contact"] == contact
    out.update({"ahead": r["ahead"], "contact": r["contact"], "ahead_arena": ahead,
                "contact_arena": contact, "d0_real": r.get("d0_real"), "dv_real": r.get("dv_real"),
                "verdict": "passed" if same else "failed",
                "message": (f"outcome {'agrees' if same else 'differs'} (ahead {r['ahead']}/{ahead}, "
                            f"contact {r['contact']}/{contact}); payoffs in the arena's interval: "
                            f"uA {'yes' if out['uA_in'] else 'no'}, uB {'yes' if out['uB_in'] else 'no'}")})
    print(json.dumps(out))
    return 0 if same else 1


if __name__ == "__main__":
    sys.exit(main())
