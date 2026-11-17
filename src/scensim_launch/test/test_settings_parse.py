"""What a project's settings reach, resolved by play_launch (roadmap 077, gaps G2, G3, G10).

The stack hands the simulation side `side_args` (the bridge's gap_trace_dir and
contact_report_dir per stack, a project's weather, npc_motion and bridge config) and each run
hands SSv2 its run settings (frame_rate, managed_ego, global_timeout,
global_real_time_factor, initialize_duration). Here both launches are resolved with
`play_launch dump`, so a name a launch does not declare, or one that does not reach the node,
fails here and not in a live run. Skipped without play_launch 0.30 or the packages.
"""
import os
import subprocess

import pytest
import yaml

from test_pilot_parse import have_package, needs_play_launch

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(os.path.dirname(HERE), "launch")
SCENARIO = os.path.join(os.path.dirname(os.path.dirname(HERE)), "scensim", "scensim", "templates",
                        "carla-first-drive", "scenarios", "drive_to_goal.xosc")


def dump(tmp_path, launch, args):
    out = tmp_path / "model.yaml"
    p = subprocess.run(["play_launch", "dump", "-o", str(out), "launch",
                        os.path.join(LAUNCH, launch), *args],
                       capture_output=True, text=True, timeout=600, cwd=str(tmp_path))
    tail = "\n".join((p.stdout + p.stderr).strip().splitlines()[-8:])
    assert p.returncode == 0 and out.is_file(), f"play_launch refused {launch}:\n{tail}"
    return yaml.safe_load(out.read_text())


def nodes(model):
    found = {}

    def walk(x):
        if isinstance(x, dict):
            for k, v in x.items():
                if isinstance(v, dict) and "exec" in v:
                    found[k] = v
                walk(v)
        elif isinstance(x, list):
            for v in x:
                walk(v)
    walk(model)
    return found


@needs_play_launch
def test_side_args_reach_the_bridge(tmp_path):
    if not (have_package("scenario_world_bridge") and have_package("scenario_agent_relay")):
        pytest.skip("scenario_world_bridge or scenario_agent_relay not installed")
    model = dump(tmp_path, "simulation.launch.xml", [
        "world_endpoint:=tcp://127.0.0.1:5591", "state_dir:=/p/state",
        "gap_trace_dir:=/l/stacks/s/episodes/gap_trace",
        "contact_report_dir:=/l/stacks/s/episodes/contacts", "weather:=WetNoon",
        "npc_motion:=kinematic"])
    [bridge] = [n for n in nodes(model).values() if n["exec"] == "scenario_world_bridge"]
    params = bridge["params"]
    assert params["gap_trace_dir"] == "/l/stacks/s/episodes/gap_trace"
    assert params["contact_report_dir"] == "/l/stacks/s/episodes/contacts"
    assert (params["weather"], params["npc_motion"], params["state_dir"]) == (
        "WetNoon", "kinematic", "/p/state")


@needs_play_launch
def test_isaac_options_reach_the_world_server(tmp_path):
    """G6: isaac: in swb.yaml becomes these sim_args (scensim.world.sim_args)."""
    if not have_package("scensim_isaac_world"):
        pytest.skip("scensim_isaac_world not installed")
    from ament_index_python.packages import get_package_share_directory
    launch = os.path.join(get_package_share_directory("scensim_isaac_world"), "launch",
                          "scensim_isaac_world.launch.xml")
    out = tmp_path / "model.yaml"
    p = subprocess.run(["play_launch", "dump", "-o", str(out), "launch", launch,
                        "ego:=/p/ego.yaml", "world:=/m/Town01,/m/Town05", "render:=headless",
                        "follow:=editor", "physics_hz:=240", "wheels_topic:=false",
                        "state_dir:=/p/state"],
                       capture_output=True, text=True, timeout=600, cwd=str(tmp_path))
    assert p.returncode == 0, p.stderr[-2000:]
    [server] = list(nodes(yaml.safe_load(out.read_text())).values())
    args = server["args"]
    pairs = dict(zip(args, args[1:]))
    assert pairs["--ego"] == "/p/ego.yaml" and pairs["--world"] == "/m/Town01,/m/Town05"
    assert (pairs["--render"], pairs["--follow"], pairs["--physics-hz"],
            pairs["--wheels-topic"]) == ("headless", "editor", "240", "false")
    assert "--state-dir=/p/state" in args


@needs_play_launch
def test_run_settings_reach_ssv2(tmp_path):
    if not have_package("scenario_test_runner"):
        pytest.skip("scenario_test_runner not installed")
    model = dump(tmp_path, "scenario.launch.xml", [
        f"scenario:={SCENARIO}", "frame_rate:=100", "managed_ego:=false",
        "global_real_time_factor:=0.5", "global_timeout:=900", "initialize_duration:=60"])
    found = nodes(model)
    [runner] = [n for n in found.values() if n["exec"].startswith("scenario_test_runner")]
    args = runner["args"]
    pairs = dict(zip(args[::2], args[1::2]))
    assert pairs["--global-frame-rate"] == "100"
    assert pairs["--global-real-time-factor"] == "0.5"
    assert pairs["--global-timeout"] == "900"
    [interp] = [n for n in found.values() if n["exec"] == "openscenario_interpreter_node"]
    assert interp["params"]["managed_ego"] is False
    assert interp["params"]["initialize_duration"] == 60
