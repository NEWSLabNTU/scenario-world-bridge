"""isaac_t2/command's pilot: a program of its own (kind command) that drives the ego.

swb starts it as `python3 pilots/simple_cmd.py --relay <url> --entity ego --seed <n> [--run-dir <dir>]`.
A command pilot is not told the map, so this takes the project's own Town01 (assets/maps/Town01,
linked by `assets fetch`) and runs swb_agent's simple_pilot on it, on the true pose
(`inputs: [ground_truth]` turns Isaac's /ground_truth/odom on).
"""
import json
import os
import sys
import tempfile
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]


def main(argv):
    out = tempfile.mkdtemp(prefix="simple_cmd_")
    if "--run-dir" in argv:
        out = argv[argv.index("--run-dir") + 1]
        os.makedirs(out, exist_ok=True)
    config = os.path.join(out, "simple_cmd.json")
    with open(config, "w") as f:
        json.dump({"map_path": str(PROJECT / "assets" / "maps" / "Town01"), "sim": "isaac"}, f)
    os.execvp("ros2", ["ros2", "run", "swb_agent", "simple_pilot", *argv, "--config", config])


if __name__ == "__main__":
    main(sys.argv[1:])
