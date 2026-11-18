#!/usr/bin/env python3
"""Write autoware-side.repos: the car's Autoware packages, as source, at this checkout's pin.

    autoware_side_repos.py [--root <scensim checkout>] [-o autoware-side.repos]

A scensim release ships the simulation side only. The car's Autoware packages (its vehicle and
sensor kit launches, individual_params, and the optional autoware_sim_profile) live in
autoware_sim_vehicle, and the user builds them in their own Autoware workspace:

    vcs import src < autoware-side.repos
    colcon build --packages-up-to tesla_model3_launch tesla_model3_sensor_kit_launch \\
        individual_params [autoware_sim_profile]

The version is the commit this checkout pins (src/autoware_sim_vehicle), so the file always
matches the release it is attached to (docs/how-to/connect-autoware.md).
"""
from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path

URL = "https://github.com/NEWSLabNTU/autoware_sim_vehicle.git"
PATH = "src/autoware_sim_vehicle"


def pin(root: Path) -> str:
    out = subprocess.run(["git", "-C", str(root), "ls-tree", "HEAD", PATH],
                         capture_output=True, text=True, check=True).stdout.split()
    if len(out) < 3 or out[1] != "commit" or not re.fullmatch(r"[0-9a-f]{40}", out[2]):
        raise SystemExit(f"{PATH} is not a submodule pin in {root}")
    return out[2]


def text(sha: str) -> str:
    return ("# The car's Autoware packages for this scensim release: build them in your\n"
            "# Autoware workspace (docs/how-to/connect-autoware.md).\n"
            "repositories:\n"
            "  autoware_sim_vehicle:\n"
            "    type: git\n"
            f"    url: {URL}\n"
            f"    version: {sha}\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    ap.add_argument("-o", "--output", type=Path, default=Path("autoware-side.repos"))
    a = ap.parse_args()
    a.output.write_text(text(pin(a.root)))
    print(f"wrote {a.output} ({PATH} at {pin(a.root)[:12]})")


if __name__ == "__main__":
    main()
