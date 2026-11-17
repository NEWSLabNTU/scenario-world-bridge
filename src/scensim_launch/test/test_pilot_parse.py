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


# play_launch options per case. 2.0.0's agnocast_env.launch.py (included by its control and
# perception launches) calls context.perform_substitution(EnvironmentVariable(...)), which
# play_launch's Rust parser (0.35.0) cannot evaluate ("'EnvironmentVariable' object has no
# attribute 'perform'"); its Python parser resolves it.
PARSER = {"autoware_2.0.0": ["--parser", "python"]}

VERSION = play_launch_version()
needs_play_launch = pytest.mark.skipif(VERSION is None or VERSION < (0, 30, 0),
                                       reason=f"play_launch >= 0.30 needed (found {VERSION})")

CASES = {
    # the profile's default release, 1.5.0 (tier4_localization_launch exists only there)
    "autoware": ("autoware.launch.xml", ["autoware_sim_profile", "acb_bridge", "tier4_localization_launch"],
                 ["entity:=ego", "relay:=tcp://localhost:5560", "vehicle:=tesla_model3",
                  "sim:=carla", f"map_path:={MAP}"]),
    # Autoware 2.0.0 (autoware_sim_profile's autoware_release:=2.0.0): runs where 2.0.0 is the
    # installed Autoware (autoware_control_config exists only there), e.g. inside
    # ghcr.io/autowarefoundation/autoware:universe-humble-2.0.0
    "autoware_2.0.0": ("autoware.launch.xml", ["autoware_sim_profile", "acb_bridge", "autoware_control_config"],
                       ["entity:=ego", "relay:=tcp://localhost:5560", "vehicle:=tesla_model3",
                        "sim:=carla", f"map_path:={MAP}", "autoware_release:=2.0.0"]),
    "autoware_isaac": ("autoware.launch.xml", ["autoware_sim_profile", "tier4_localization_launch"],
                       ["entity:=ego", "relay:=tcp://localhost:5560", "vehicle:=tesla_model3",
                        "sim:=isaac", f"map_path:={MAP}"]),
    "vehicle_side_carla": ("vehicle_side_carla.launch.xml", ["acb_bridge"],
                           ["entity:=ego", "vehicle:=tesla_model3", f"map_path:={MAP}"]),
    "carla_basic": ("carla_basic.launch.py", ["scensim_agent"],
                    ["entity:=lead", "relay:=tcp://localhost:5560", f"map_path:={MAP}"]),
    "simple": ("simple.launch.py", ["scensim_agent", "acb_bridge", "tesla_model3_description"],
               ["entity:=ego", "relay:=tcp://localhost:5560", "vehicle:=tesla_model3",
                "sim:=carla", f"map_path:={MAP}"]),
    "simple_isaac": ("simple.launch.py", ["scensim_agent", "tesla_model3_description"],
                     ["entity:=ego", "relay:=tcp://localhost:5560", "vehicle:=tesla_model3",
                      "sim:=isaac", f"map_path:={MAP}"]),
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
                        os.path.join(PILOTS, name), *PARSER.get(kind, []), *args],
                       capture_output=True, text=True, timeout=600, cwd=str(tmp_path))
    tail = "\n".join((p.stdout + p.stderr).strip().splitlines()[-8:])
    assert p.returncode == 0 and out.is_file(), f"{kind}: play_launch refused {name}:\n{tail}"


@needs_play_launch
def test_an_own_car_simple_pilot_resolves_without_the_cars_autoware_packages(tmp_path):
    """roadmap 077, G5: the stack gives an own car's pilot its generated files; the vehicle
    side must take them, not look up <car>_description (play_launch's parser scopes an
    include to the arguments it names, so simple.launch.py forwards them all)."""
    for p in ("scensim_agent", "acb_bridge", "sim_vehicle_spec"):
        if not have_package(p):
            pytest.skip(f"not installed: {p}")
    from ament_index_python.packages import get_package_share_directory
    sensors = os.path.join(get_package_share_directory("sim_vehicle_spec"), "generated",
                           "tesla_model3", "carla_sensors.yaml")
    urdf = tmp_path / "description.urdf"
    urdf.write_text('<?xml version="1.0"?><robot name="vehicle"><link name="base_link"/></robot>')
    hashes = tmp_path / "spec_hash.yaml"
    hashes.write_text("spec_hash: abc\n")
    out = tmp_path / "model.yaml"
    p = subprocess.run(["play_launch", "dump", "-o", str(out), "launch",
                        os.path.join(PILOTS, "simple.launch.py"), "entity:=ego",
                        "relay:=tcp://localhost:5560", "vehicle:=my_lincoln", "sim:=carla",
                        f"map_path:={MAP}", f"vehicle_config:={sensors}",
                        f"autoware_spec_hash_file:={hashes}", f"description:={urdf}",
                        f"spec_hash_file:={hashes}", f"vehicle_maps_dir:={tmp_path}"],
                       capture_output=True, text=True, timeout=600, cwd=str(tmp_path))
    tail = "\n".join((p.stdout + p.stderr).strip().splitlines()[-8:])
    assert p.returncode == 0 and out.is_file(), f"play_launch refused the own car:\n{tail}"
    assert str(hashes) in out.read_text()
