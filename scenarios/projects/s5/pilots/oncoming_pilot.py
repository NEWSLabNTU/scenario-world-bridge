#!/usr/bin/env python3
"""The S5 oncoming car (a `command` pilot): A, a light agent, going straight through junction
861 at v, timed to reach the conflict `gap` s after B's front reaches its stop line, then
holding its speed (left_turn_game's C1 "hold", with its safety layer).

scensim starts it per run with `--relay R --entity coop_a --seed S --run-dir D`. v and gap are the
scenario's (its ParameterDeclarations, read from the run's resolved.yaml); B's speed cap is
--b-vmax, the ego's max_velocity. It runs scripts/coop/oncoming_agent.py with s5_aw.py's
configuration, which logs both cars' truth to <run dir>/coop_a.jsonl for the judge.
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
import make_scenarios  # noqa: E402


def host():
    from scensim import config
    return config.host_settings()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--b-vmax", type=float, default=8.0)
    ap.add_argument("--relay", required=True)
    ap.add_argument("--entity", required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--run-dir", required=True)
    a = ap.parse_args()
    rd = Path(a.run_dir)
    resolved = yaml.safe_load((rd / "resolved.yaml").read_text())
    v, gap = make_scenarios.cell(resolved["scenario"]["file"])
    h = host()
    port = ((resolved.get("stack") or {}).get("seat") or {}).get("carla_port") or h["carla"]["port"]
    root = h["carla"].get("root") or ""
    cfg = {"entity": a.entity, "other": "hero", "v": v, "gap": gap, "seed": a.seed, "b_vmax": a.b_vmax,
           "b_from": [4, 2], "b_to": [62, -1], "junction": 861, "a_ref": [-85.0, -30.0],
           "log": str(rd / "coop_a.jsonl"), "carla_port": int(port), "find_timeout": 90,
           "carla_pythonapi": os.path.join(os.path.expanduser(root), "PythonAPI", "carla") if root else ""}
    path = rd / "coop_a.json"
    path.write_text(json.dumps(cfg))
    os.execv(sys.executable, [sys.executable, str(COOP / "oncoming_agent.py"), "--relay", a.relay,
                              "--entity", a.entity, "--config", str(path)])


if __name__ == "__main__":
    main()
