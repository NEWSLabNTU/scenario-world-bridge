#!/usr/bin/env python3
"""The C2b cut-in pilot (a `command` pilot): one light-policy car of the pair.

scensim starts it per run with `--pair <eq|aw> --relay R --entity E --seed S --run-dir D`. The
cell (d0, dv) is the scenario's (its ParameterDeclarations, read from the run's
resolved.yaml); the rule is the pair's for this car's side (coop_a: A, coop_b: B), as
scripts/coop/c2b.py gives it. It then runs scripts/coop/cutin_agent.py, which registers the
entity with the relay and logs both cars' true states to <run dir>/<entity>.jsonl, where
the judge scores them.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from coop_pilots_dir import coop_dir  # noqa: E402
COOP = coop_dir()
sys.path.insert(0, str(COOP))
import c2b  # noqa: E402
import make_scenarios  # noqa: E402

PAIRS = {"eq": c2b.PAIRS[0], "aw": c2b.PAIRS[1]}
OTHER = {"coop_a": ("coop_b", "A", 0), "coop_b": ("coop_a", "B", 1)}


def carla_port(resolved):
    seat = ((resolved.get("stack") or {}).get("seat") or {}).get("carla_port")
    if seat:
        return int(seat)
    from scensim import config
    return int(config.host_settings(resolved.get("project"))["carla"]["port"])


def carla_pythonapi():
    from scensim import config
    root = config.host_settings()["carla"].get("root") or ""
    return os.path.join(os.path.expanduser(root), "PythonAPI", "carla") if root else ""


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pair", required=True, choices=sorted(PAIRS))
    ap.add_argument("--relay", required=True)
    ap.add_argument("--entity", required=True, choices=sorted(OTHER))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--run-dir", required=True)
    a = ap.parse_args()
    rd = Path(a.run_dir)
    resolved = yaml.safe_load((rd / "resolved.yaml").read_text())
    d0, dv = make_scenarios.cell(resolved["scenario"]["file"])
    other, side, k = OTHER[a.entity]
    cfg = {"entity": a.entity, "other": other, "side": side, "d0": d0, "dv": dv, "vb": c2b.VB,
           "warmup": c2b.WARMUP, "rule": PAIRS[a.pair][k], "seed": a.seed,
           "log": str(rd / f"{a.entity}.jsonl"), "carla_port": carla_port(resolved),
           "find_timeout": 60, "carla_pythonapi": carla_pythonapi()}
    path = rd / f"{a.entity}.json"
    path.write_text(json.dumps(cfg))
    os.execv(sys.executable, [sys.executable, str(COOP / "cutin_agent.py"), "--relay", a.relay,
                              "--entity", a.entity, "--config", str(path)])


if __name__ == "__main__":
    main()
