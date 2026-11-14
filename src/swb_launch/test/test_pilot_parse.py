"""Every pilot kind's launch parses under play_launch's own parser (roadmap 077, tier T1).

Static checks (test_pilots.py) read the XML; this one resolves each launch the way the stack
does, with `play_launch dump`, so a clash only the resolver sees fails here and not in a live
run. (One did: the autoware pilot's `vehicle` collided with the Autoware profile's `vehicle`
switch, and play_launch refused the launch.) Skipped where play_launch 0.30 or newer, or the
packages a launch includes, are not installed.
"""
import os
import re
import shutil
import subprocess

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PILOTS = os.path.join(os.path.dirname(HERE), "launch", "pilots")
MAP = "/nonexistent/Town01"   # resolving needs a path, not a map


def play_launch_version():
    exe = shutil.which("play_launch")
    if not exe:
        return None
    try:
        out = subprocess.run([exe, "--version"], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r"(\d+)\.(\d+)\.(\d+)", out)
    return tuple(int(x) for x in m.groups()) if m else None


def have_package(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(name)
        return True
    except Exception:  # noqa: BLE001 (no ROS environment, or not installed)
        return False


VERSION = play_launch_version()
needs_play_launch = pytest.mark.skipif(VERSION is None or VERSION < (0, 30, 0),
                                       reason=f"play_launch >= 0.30 needed (found {VERSION})")

CASES = {
    "autoware": ("autoware.launch.xml", ["autoware_sim_profile", "acb_bridge"],
                 ["entity:=ego", "relay:=tcp://localhost:5560", "vehicle:=tesla_model3",
                  "sim:=carla", f"map_path:={MAP}"]),
    "autoware_isaac": ("autoware.launch.xml", ["autoware_sim_profile"],
                       ["entity:=ego", "relay:=tcp://localhost:5560", "vehicle:=tesla_model3",
                        "sim:=isaac", f"map_path:={MAP}"]),
    "vehicle_side_carla": ("vehicle_side_carla.launch.xml", ["acb_bridge"],
                           ["entity:=ego", "vehicle:=tesla_model3", f"map_path:={MAP}"]),
    "carla_basic": ("carla_basic.launch.py", ["swb_agent"],
                    ["entity:=lead", "relay:=tcp://localhost:5560", f"map_path:={MAP}"]),
    "command": ("command.launch.py", [],
                ["entity:=npc", "relay:=tcp://localhost:5560", "run:=python3 -c pass"]),
}


@needs_play_launch
@pytest.mark.parametrize("kind", sorted(CASES))
def test_pilot_resolves_under_play_launch(kind, tmp_path):
    name, packages, args = CASES[kind]
    missing = [p for p in packages if not have_package(p)]
    if missing:
        pytest.skip(f"not installed: {', '.join(missing)}")
    out = tmp_path / "model.yaml"
    p = subprocess.run(["play_launch", "dump", "-o", str(out), "launch",
                        os.path.join(PILOTS, name), *args],
                       capture_output=True, text=True, timeout=600, cwd=str(tmp_path))
    tail = "\n".join((p.stdout + p.stderr).strip().splitlines()[-8:])
    assert p.returncode == 0 and out.is_file(), f"{kind}: play_launch refused {name}:\n{tail}"
