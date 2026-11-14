"""The town before the pilots on a CARLA map switch (roadmap 077, G9): the keeper's call of the
bridge's ~/load_map (swb.load_map), and what the switch records of it. No bridge, no ROS: the
command is a stand-in printing the answer swb.load_map would."""
import json
import sys

import pytest
import yaml

from swb import assets, load_map, project, stack


@pytest.fixture
def where(host):
    tmp = host["tmp"]
    (tmp / "system" / "host.yaml").write_text(yaml.safe_dump(
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 2}))
    return host


TWO_TOWNS = """simulator: carla
maps: [Town01, Town05]
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01, Town05]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
  lead: {kind: carla_basic}
  follower: {kind: command, run: [python3, pilots/follower.py]}
"""


def record(where, sim="carla"):
    d = where["tmp"] / "proj"
    project.new_project(d, "multi-pilot")
    (d / "swb.yaml").write_text(TWO_TOWNS.replace("carla", sim, 1))
    assets.fetch(d, out=lambda s: None)
    return stack.plan(project_arg=str(d), map_name="Town01").record


def answering(monkeypatch, out="", err="", code=0):
    """swb.load_map replaced by a command that prints ``out``/``err`` and exits ``code``; the
    command lines it was given."""
    seen, real = [], stack.preload_command

    def cmd(rec, name):
        seen.append((rec["domains"]["scenario"], name, real(rec, name)))
        return [sys.executable, "-c",
                f"import sys; print({out!r}); print({err!r}, file=sys.stderr); sys.exit({code})"]
    monkeypatch.setattr(stack, "preload_command", cmd)
    monkeypatch.setattr(stack, "bridge_serving", lambda rec: True)
    return seen


def test_the_command_is_swb_load_map_on_the_maps_directory(where, maps_hub):
    rec = record(where)
    argv = stack.preload_command(rec, "Town05")
    assert argv[1:4] == ["-m", "swb.load_map", str(where["tmp"] / "proj/assets/maps/Town05")]
    assert argv[argv.index("--timeout") + 1] == str(stack.PRELOAD_TIMEOUT)
    # the whole switch fits in the runner's wait for it
    from swb import run
    assert stack.STOP_GRACE + stack.PRELOAD_WAIT + stack.PRELOAD_TIMEOUT < run.SWITCH_TIMEOUT


def test_a_loaded_town(where, maps_hub, monkeypatch):
    rec = record(where)
    seen = answering(monkeypatch, out=json.dumps(
        {"success": True, "message": "carla loaded Town05, episode 7 in 41.2 s"}))
    got = stack.preload_map(rec, "Town05")
    assert got["state"] == stack.LOADED and "loaded Town05" in got["message"]
    # asked in the scenario domain, where the bridge is
    assert seen[0][0] == rec["domains"]["scenario"] and seen[0][1] == "Town05"


def test_a_refusal_is_recorded_with_the_bridges_reason(where, maps_hub, monkeypatch):
    rec = record(where)
    answering(monkeypatch, code=1, out=json.dumps(
        {"success": False, "message": "not loaded: a scenario is running on this bridge"}))
    got = stack.preload_map(rec, "Town05")
    assert got == {"state": stack.NOT_LOADED, "seconds": got["seconds"],
                   "error": "not loaded: a scenario is running on this bridge"}


def test_a_crash_is_recorded_with_its_last_words(where, maps_hub, monkeypatch):
    rec = record(where)
    answering(monkeypatch, code=2, err="Traceback ...\nImportError: no rclpy")
    got = stack.preload_map(rec, "Town05")
    assert got["state"] == stack.NOT_LOADED and got["error"] == "ImportError: no rclpy"


def test_no_bridge_or_isaac_is_skipped(where, maps_hub, monkeypatch):
    rec = record(where)
    monkeypatch.setattr(stack, "bridge_serving", lambda rec: False)
    assert stack.preload_map(rec, "Town05")["state"] == stack.SKIPPED
    rec["simulator"] = "isaac"
    monkeypatch.setattr(stack, "bridge_serving", lambda rec: True)
    assert stack.preload_map(rec, "Town05")["state"] == stack.SKIPPED


class Pilots:
    """The keeper's pilots, as far as a switch uses them: the order of what happened."""

    def __init__(self, log):
        self.log, self.procs = log, {"ego": object()}

    def stop_all(self, rec):
        self.log.append("stop")

    def start_all(self, rec):
        self.log.append(("start", rec["map"]))


@pytest.mark.parametrize("loaded", [True, False])
def test_the_switch_loads_the_town_between_stop_and_start(where, maps_hub, monkeypatch,
                                                         loaded):
    rec = record(where)
    log = []
    monkeypatch.setattr(stack, "bridge_serving", lambda rec: True)

    def preload(r, name):
        log.append(("load", name, r["map"]))
        return ({"state": stack.LOADED, "message": "loaded"} if loaded
                else {"state": stack.NOT_LOADED, "error": "no answer"})
    monkeypatch.setattr(stack, "preload_map", preload)
    stack_dir = where["tmp"] / "stackdir"
    stack_dir.mkdir()
    assert stack.switch_map(stack_dir, rec, {"id": "r1", "map": "Town05"}, Pilots(log))
    # the old pilots stopped first, the town loaded, then the pilots on it
    assert log == ["stop", ("load", "Town05", "Town01"), ("start", "Town05")]
    sw = json.loads((stack_dir / "stack.json").read_text())["switch"]
    # a failed load still switches (the town then loads at Initialize), and says why
    assert sw["state"] == stack.DONE
    assert sw["preload"]["state"] == (stack.LOADED if loaded else stack.NOT_LOADED)
    ev = [json.loads(ln) for ln in (stack_dir / "events.jsonl").read_text().splitlines()]
    assert [e["event"] for e in ev] == ["map.switch", "map.preload", "map.switched"]


def test_swb_load_map_without_ros_is_not_reached(monkeypatch, capsys):
    monkeypatch.setattr(load_map, "call", lambda *a: (False, "cannot call: no rclpy", False))
    assert load_map.main(["/maps/Town05"]) == load_map.NOT_REACHED
    assert json.loads(capsys.readouterr().out) == {"success": False,
                                                   "message": "cannot call: no rclpy"}
    monkeypatch.setattr(load_map, "call", lambda *a: (False, "a scenario is running", True))
    assert load_map.main(["/maps/Town05"]) == 1
    monkeypatch.setattr(load_map, "call", lambda *a: (True, "carla loaded Town05", True))
    assert load_map.main(["/maps/Town05", "--timeout", "5"]) == 0
