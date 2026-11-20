"""The settings a project hands the simulation side and SSv2 (scensim.world, scensim.ssv2, scensim.profile),
and what a run takes back from the bridge (scensim.episodes, scensim.frames, scensim.gap). No simulator."""
import json
import os
import time
from pathlib import Path

import pytest

from scensim import episodes, frames, gap, profile, slices, ssv2, world

PROFILE_SCRIPT = (Path(__file__).resolve().parents[2] / "autoware_sim_vehicle" /
                  "autoware_sim_profile" / "scripts" / "check_launch_args")


# --- G3: SSv2's run settings ----------------------------------------------------------------

def test_ssv2_problems_and_merge():
    assert ssv2.problems(None) == []
    assert ssv2.problems({"frame_rate": 100, "managed_ego": False}) == []
    bad = dict(ssv2.problems({"frame_rate": 0, "managed_ego": "no", "global_timeout": True,
                              "speed": 2}))
    assert bad["ssv2.frame_rate"] == "0 is not a positive number"
    assert bad["ssv2.managed_ego"] == "'no' is not true or false"
    assert bad["ssv2.global_timeout"] == "True is not a positive number"
    assert "unknown key speed" in bad["ssv2"]
    assert ssv2.problems([1]) == [("ssv2", "is not a mapping: {global_timeout: 600, "
                                           "frame_rate: 20}")]
    merged = ssv2.merge({"frame_rate": 20, "global_timeout": 400}, None,
                        {"frame_rate": 100, "nope": 1}, {"managed_ego": False})
    assert merged == {"frame_rate": 100, "global_timeout": 400, "managed_ego": False}
    assert not ssv2.managed(merged) and ssv2.managed({})
    assert ssv2.launch_args(merged) == {"global_timeout": "400", "frame_rate": "100",
                                        "managed_ego": "false"}
    assert ssv2.launch_args({"global_real_time_factor": 0.5}) == {
        "global_real_time_factor": "0.5"}


# --- G10, G6: world and isaac ---------------------------------------------------------------

def test_world_problems(tmp_path):
    (tmp_path / "bridge.yaml").write_text("{}\n")
    ok = {"world": {"weather": {"preset": "ClearNoon", "cloudiness": 30},
                    "npc_motion": "kinematic", "contact_report_dir": False,
                    "config_file": "bridge.yaml"}}
    assert world.problems(ok, "carla", tmp_path) == []
    bad = world.problems({"world": {"weather": {"cloudiness": "lots"}, "npc_motion": "fast",
                                    "state_dir": "/x", "gap_trace_dir": "/y", "color": 1,
                                    "config_file": "gone.yaml"}}, "carla", tmp_path)
    bad += world.problems({"world": {"world_config_file": "bridge.yaml",
                                     "weather": "ClearNoon"}}, "isaac", tmp_path)
    text = [f"{w}: {m}" for w, m in bad]
    assert "world.weather: Isaac has no weather service: a scenario's EnvironmentAction " \
           "sets the sky, fog and rain" in text
    assert "world.weather.cloudiness: 'lots' is not a number (a weather parameter of the " \
           "world server, e.g. cloudiness: 30)" in text
    assert "world.npc_motion: 'fast' is not one of tracked, kinematic" in text
    assert any(t.startswith("world: state_dir is scensim's") for t in text)
    assert any(t.startswith("world: gap_trace_dir is scensim's (logging.gap_trace: true") for t in text)
    assert any("unknown key color" in t for t in text)
    assert "world.config_file: gone.yaml: no such file" in text
    assert any(t.startswith("world.world_config_file: is CARLA's") for t in text)
    # isaac: on carla refused, keys checked, scensim's refused by name
    assert world.problems({"isaac": {"render": "headless"}}, "carla") == [
        ("isaac", "is for simulator: isaac, this project's is carla")]
    (tmp_path / "assets" / "maps" / "Town05").mkdir(parents=True)
    (tmp_path / "ego.yaml").write_text("{}\n")
    good = {"isaac": {"ego": "ego.yaml", "render": "headless", "follow": "editor",
                      "physics_hz": 240, "preload": ["Town05"], "wheels_topic": False,
                      "wheel_query": "sweep"}}
    assert world.problems(good, "isaac", tmp_path) == []
    text = [f"{w}: {m}" for w, m in world.problems(
        {"isaac": {"ego": "missing.yaml", "render": "vr", "follow": "maybe", "physics_hz": -1,
                   "preload": ["Town09"], "world": "x", "wheel_query": "ray"}}, "isaac",
        tmp_path)]
    assert "isaac.wheel_query: 'ray' is not one of raycast, sweep" in text
    assert "isaac.ego: missing.yaml: no such file" in text
    assert "isaac.render: 'vr' is not one of auto, window, headless" in text
    assert "isaac.follow: 'maybe' is not one of true, false, editor" in text
    assert "isaac.physics_hz: -1 is not a positive number" in text
    assert "isaac.preload: Town09 is not fetched (assets/maps/Town09)" in text
    assert any(t.startswith("isaac: world is scensim's") for t in text)


def test_world_args(tmp_path):
    w = {"weather": {"preset": "WetNoon", "cloudiness": 30}, "npc_motion": "kinematic",
         "config_file": "b.yaml", "world_config_file": "c.yaml", "contact_report_dir": "c"}
    side = world.side_args(tmp_path, w)
    assert side == {"weather": "WetNoon", "npc_motion": "kinematic",
                    "config_file": str((tmp_path / "b.yaml").resolve()),
                    "contact_report_dir": str((tmp_path / "c").resolve())}
    assert world.side_args(tmp_path, {"contact_report_dir": False}) == {
        "contact_report_dir": "off"}
    assert world.weather_parameters(w) == {"cloudiness": 30.0}
    assert world.weather_parameters({"weather": "ClearNoon"}) == {}
    assert world.sim_args(tmp_path, "carla", w, None) == {
        "config_file": str((tmp_path / "c.yaml").resolve())}
    assert world.sim_args(tmp_path, "isaac", w, {"ego": "none", "render": "headless",
                                                 "follow": True, "physics_hz": 240,
                                                 "wheels_topic": False,
                                                 "wheel_query": "sweep"}) == {
        "ego": "none", "render": "headless", "physics_hz": "240", "follow": "true",
        "wheels_topic": "false"}
    # stack_args merge: the user's win, a key scensim owns is refused naming what to give
    assert world.merge_user({"state_dir": "/s", "weather": "A"}, {"weather": "B"},
                            world.SIDE_OWNED, "side_args") == {"state_dir": "/s", "weather": "B"}
    with pytest.raises(ValueError, match=r"side_args: state_dir:= is scensim's"):
        world.merge_user({"state_dir": "/s"}, {"state_dir": "/t"}, world.SIDE_OWNED, "side_args")
    assert world.named("a:=1 'b:=x y'", "w") == {"a": "1", "b": "x y"}
    assert world.named(world.joined({"a": "x y", "b": "2"}), "w") == {"a": "x y", "b": "2"}


def test_weather_call():
    argv = world.weather_argv({"cloudiness": 30, "sun_altitude_angle": 60.5})
    assert argv[:5] == ["ros2", "service", "call", "/scenario_world_bridge/set_weather",
                        "scensim_interfaces/srv/SetWeather"]
    assert argv[5] == "{names: [cloudiness, sun_altitude_angle], values: [30.0, 60.5]}"
    out = ("requester: making request: ...\n\nresponse:\nscensim_interfaces.srv.SetWeather_Response("
           "success=False, message='unknown parameter cloudy', preset='', names=[], values=[])\n")
    assert world.weather_answer(out) == (False, "unknown parameter cloudy")
    assert world.weather_answer("waiting for service to become available...\n") == (None, "")


# --- G12: the profile's argument check ------------------------------------------------------

@pytest.mark.skipif(not PROFILE_SCRIPT.is_file(), reason="autoware_sim_vehicle not checked out")
def test_profile_arg_problems(tmp_path):
    check = profile.checker(PROFILE_SCRIPT)
    assert check is not None
    assert profile.arg_problems({"max_velocity": 11.11, "ndt": "cuda"}, check=check) == []
    msgs = profile.arg_problems({"carla_control": False, "ndt": "gpu"}, check=check)
    assert "carla_control:= was renamed; pass sim_control:=false instead" in msgs
    assert "ndt:=gpu is not one of autoware, cuda, none" in msgs
    # a run substitution is known per run only: left out
    assert profile.arg_problems({"ndt": "{param:ndt}"}, check=check) == []
    # a file relative to the project, as the pair check reads it
    (tmp_path / "graph.yaml").write_text("units: [ndt_scan_matcher]\n")
    [m] = profile.arg_problems({"ndt": "none", "diagnostic_graph_path": "graph.yaml"},
                               tmp_path, check=check)
    assert "judges ndt_scan_matcher, which ndt:=none does not start" in m
    # the default lookup finds the same script (installed or in this source tree)
    assert profile.checker() is not None


# --- G2: per-run episode files --------------------------------------------------------------

def test_episodes_collected_per_run(tmp_path):
    dirs = episodes.stack_dirs(tmp_path / "stack", True, True)
    for d in dirs.values():
        Path(d).mkdir(parents=True)
    old = Path(dirs["gap_trace"]) / "20261115T000000Z_1.jsonl"
    old.write_text('{"type": "end"}\n')
    mark = episodes.Mark.take(dirs)
    trace = Path(dirs["gap_trace"]) / "20261115T000100Z_2.jsonl"
    trace.write_text('{"type": "episode"}\n{"type": "end"}\n')
    report = Path(dirs["contacts"]) / "20261115T000100Z_e2.json"
    report.write_text("{}\n")
    run_dir = tmp_path / "run"
    got = episodes.collect(mark, run_dir, timeout=2.0)
    assert got == {"gap_trace": [run_dir / "gap_trace" / trace.name],
                   "contacts": [run_dir / "contacts" / report.name]}
    assert old.exists() and not trace.exists()   # an earlier run's file stays
    assert episodes.collect(episodes.Mark.take({"gap_trace": None}), run_dir) == {}


def test_episodes_wait_for_a_trace_end(tmp_path):
    d = tmp_path / "g"
    d.mkdir()
    mark = episodes.Mark.take({"gap_trace": str(d)})
    t = d / "x.jsonl"
    t.write_text('{"type": "episode"}\n')
    t0 = time.time()
    got = episodes.settle(mark, timeout=0.6)
    assert time.time() - t0 >= 0.5 and got == {"gap_trace": [t]}   # waited, then took it


# --- G23: the frame budget ------------------------------------------------------------------

WINDOW = ("\x1b[2m2026-10-19T23:{m}:17.162518Z\x1b[0m \x1b[32m INFO\x1b[0m "
          "\x1b[2mcarla_scenario_bridge::zmq_server\x1b[0m\x1b[2m:\x1b[0m Frame budget, last 200 "
          "frames: {n} frames, p50/p95/max ms: processing {p}, tick {t}, UpdateEntityStatus "
          "0.13/0.17/0.23; entities 1/1/1; processing > 25 ms: {op}, tick > 100 ms: {ot}")
WHOLE = ("2026-10-19T23:25:40.545463Z  INFO carla_scenario_bridge::zmq_server: Frame budget, "
         "whole run: 602 frames, p50/p95/max ms: processing 9.0/9.0/90.0, tick 1.26/1.57/5.07, "
         "UpdateEntityStatus 0.13/0.17/0.36; entities 1/1/1; processing > 25 ms: 7, "
         "tick > 100 ms: 0")


def test_frame_windows_and_metrics():
    lines = [WHOLE,
             WINDOW.format(m="20", n=200, p="0.23/0.28/1.59", t="1.26/1.57/2.03", op=0, ot=0),
             WINDOW.format(m="21", n=200, p="0.30/1.79/30.5", t="1.30/2.10/120.0", op=1, ot=1),
             WINDOW.format(m="22", n=100, p="0.40/0.50/0.90", t="1.00/1.10/1.20", op=0, ot=0)]
    text = "\n".join(lines + [lines[2]])   # the same line twice (two files of the group)
    ws = frames.windows(text)
    assert len(ws) == 3   # the whole-run line is the previous run's; the duplicate is one
    m = frames.metrics(ws)
    assert m == {"frame_windows": 3, "frame_count": 500, "frame_p50_ms": 0.3,
                 "frame_p95_ms": 1.79, "frame_max_ms": 30.5, "tick_p50_ms": 1.26,
                 "tick_p95_ms": 2.1, "tick_max_ms": 120.0, "frames_over_budget": 1,
                 "ticks_over_budget": 1}
    assert frames.metrics([]) == {}


def test_frame_metrics_from_the_slice(tmp_path):
    proc = tmp_path / "proc"
    out = proc / "bridge" / "t" / "node" / "scenario_world_bridge" / "out"
    other = proc / "pilot-ego" / "console.log"
    out.parent.mkdir(parents=True)
    other.parent.mkdir(parents=True)
    out.write_text(WINDOW.format(m="00", n=200, p="5.0/6.0/7.0", t="1/1/1", op=0, ot=0) + "\n")
    other.write_text("")
    m0 = slices.LogMark.take(proc)
    with open(out, "a") as f:
        f.write(WINDOW.format(m="01", n=200, p="0.2/0.3/0.4", t="1/2/3", op=0, ot=0) + "\n")
    with open(other, "a") as f:   # another group's lines are not the bridge's
        f.write(WINDOW.format(m="02", n=200, p="9/9/9", t="9/9/9", op=0, ot=0) + "\n")
    m = frames.from_marks(m0, slices.LogMark.take(proc))
    assert (m["frame_windows"], m["frame_p95_ms"], m["tick_max_ms"]) == (1, 0.3, 3.0)


# --- G2: the gap tools as judges ------------------------------------------------------------

BOX = {"location": [0.0, 0.0, 0.72], "extent": [2.35, 1.05, 0.72]}


def trace_text(contact=False) -> str:
    recs = [{"type": "episode", "schema": 1, "episode": 1, "start_unix": 1.0,
             "npc_motion": "tracked"},
            {"type": "entity", "frame": 0, "name": "ego", "kind": "ego",
             "declared": {"center": [1.5, 0.0, 0.9], "dimensions": [4.5, 2.1, 1.8]},
             "origin_offset": [1.5, 0.0, 0.0], "actor_id": 1, "blueprint": "vehicle.tesla.model3",
             "blueprint_box": BOX}]
    for n in range(1, 4):
        x = float(n)
        recs.append({"type": "frame", "frame": n, "sim_time": 0.05 * n, "entities": {
            "ego": {"ssv2": [x, 0, 0, 0, 0, 0], "carla": [x + 1.5, 0, 0, 0, 0, 0],
                    "velocity": [1.0, 0, 0]}}})
    if contact:
        recs.append({"type": "contact", "frame": 2, "carla_frame": 2, "sim_time": 0.1,
                     "ego_id": 1, "other_id": None, "name": None, "other_type": "static.wall",
                     "other_kind": "static", "impulse": 1.0})
    recs.append({"type": "end", "frame": 3, "sim_time": 0.15, "frames": 3, "dropped_frames": 0,
                 "complete": True})
    return "".join(json.dumps(r) + "\n" for r in recs)


PASSED_JUNIT = ('<testsuites><testsuite name="s" tests="1" failures="0" errors="0">'
                '<testcase name="s"/></testsuite></testsuites>')


def run_dir_with(tmp_path, text):
    d = tmp_path / "run"
    (d / "gap_trace").mkdir(parents=True)
    (d / "gap_trace" / "20261115T000000Z_1.jsonl").write_text(text)
    (d / "junit.xml").write_text(PASSED_JUNIT)
    return d


def test_gap_oracle_judge(tmp_path, capsys):
    d = run_dir_with(tmp_path, trace_text())
    assert gap.main(["oracle", str(d)]) == 0
    doc = json.loads(capsys.readouterr().out)
    assert doc["verdict"] == "passed" and doc["traces"][0]["g4"] == "agree"
    assert (d / "gap_trace" / "20261115T000000Z_1.findings.json").is_file()
    d2 = run_dir_with(tmp_path / "b", trace_text(contact=True))
    assert gap.main(["oracle", str(d2)]) == 1
    doc = json.loads(capsys.readouterr().out)
    assert doc["verdict"] == "failed" and "PASS+contact" in doc["message"]
    assert gap.main(["oracle", "--fail-on", "never", str(d2)]) == 0
    capsys.readouterr()
    # no trace: an error, never a pass
    empty = tmp_path / "none"
    empty.mkdir()
    assert gap.main(["oracle", str(empty)]) == 2
    assert json.loads(capsys.readouterr().out)["verdict"] == "error"


def test_gap_incident_needs_its_context(tmp_path, capsys):
    d = run_dir_with(tmp_path, trace_text())
    assert gap.main(["incident", "--incident", "2", "--context", "context.jsonl", str(d)]) == 2
    assert "no context log" in json.loads(capsys.readouterr().out)["message"]
    (d / "context.jsonl").write_text("")
    code = gap.main(["incident", "--incident", "2", "--context", "context.jsonl", str(d)])
    doc = json.loads(capsys.readouterr().out)
    # the trace has no i2_* actors: incident_judge fails it (the incident never started)
    assert code == 1 and doc["verdict"] == "failed", doc
    assert os.path.isfile(d / "gap_trace" / "20261115T000000Z_1.incident.json")


def test_provenance_names_the_carla_world_servers_map_props(tmp_path):
    """077 G58: map_props changes results, so a run records it, from the config file the
    stack gave the world server, else the installed one; frozen when the key is absent."""
    share = tmp_path / "share"
    (share / "config").mkdir(parents=True)
    (share / "config" / "scensim_carla_world.yaml").write_text("spectator: off\n")
    rec = {"simulator": "carla", "launch_args": {"sim_args": "state_dir:=/x"}}
    assert world.carla_world_settings(rec, share=share) == {"map_props": "frozen"}
    own = tmp_path / "carla_world.yaml"
    own.write_text("map_props: live\n")
    rec["launch_args"]["sim_args"] = f"state_dir:=/x config_file:={own}"
    assert world.carla_world_settings(rec, share=share) == {"map_props": "live"}
    own.write_text("map_props: [\n")
    assert world.carla_world_settings(rec, share=share) == {"map_props": None}
    assert world.carla_world_settings({"simulator": "isaac"}, share=share) == {}
