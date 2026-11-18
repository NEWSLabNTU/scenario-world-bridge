#!/usr/bin/env python3
"""s5_score: an S5 left-turn run scored as scripts/coop/s5_aw.py scores it, against the
reference batch (074 S5 with full CenterPoint, 2026-11-15, reference/s5_full_centerpoint.jsonl).

From A's log (coop_a.jsonl, both cars' truth): who crossed first (go: B, the Autoware ego,
through before A), whether B stopped short, the lag at B's decision, PET, contact, each car's
peak braking and S0 utility. B's decision is "go" or "yield". The run passes when the decision
is the reference's at its gap where the reference's runs all agree (yield at 6.5 s and below, go
at 8 s and above); at a gap where they split (7 s: Autoware's critical lag, 6.92 s) either
decision passes. A contact is reported, not judged: the reference has one at 7 s.

    python3 judges/s5_score.py [--reference reference/s5_full_centerpoint.jsonl] <run dir>
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from coop_pilots_dir import coop_dir  # noqa: E402
sys.path.insert(0, str(coop_dir()))
import make_scenarios  # noqa: E402
import s5_aw  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reference", default=str(HERE.parent / "reference" / "s5_full_centerpoint.jsonl"))
    ap.add_argument("run_dir")
    a = ap.parse_args()
    rd = Path(a.run_dir)
    resolved = yaml.safe_load((rd / "resolved.yaml").read_text())
    v, gap = make_scenarios.cell(resolved["scenario"]["file"])
    log = rd / "coop_a.jsonl"
    if not log.is_file():
        print(json.dumps({"message": "no coop_a.jsonl: the oncoming pilot logged nothing"}))
        return 2
    r = s5_aw.score(log)
    if "error" in r:
        print(json.dumps({"message": f"score: {r['error']}"}))
        return 2
    ref = defaultdict(list)
    for x in map(json.loads, open(a.reference)):
        ref[(x["v"], x["gap"])].append("go" if x["go"] else "yield")
    seen = ref.get((v, gap), [])
    decision = "go" if r["go"] else "yield"
    expected = seen[0] if seen and len(set(seen)) == 1 else None
    ok = expected is None or decision == expected
    out = {"v": v, "gap": gap, "decision": decision, "reference": seen, **r,
           "verdict": "passed" if ok else "failed",
           "message": (f"B {decision} at gap {gap} (lag {r.get('lag')} s; reference "
                       f"{'/'.join(seen) or 'none'}); contact {r['contact']}, PET {r.get('pet')}")}
    print(json.dumps(out))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
