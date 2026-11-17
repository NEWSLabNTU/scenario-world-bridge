"""scensim.drive (roadmap 077, G14): a free drive on the stack, the goal through the relay, a
result like a run's. No simulator: scensim_free_drive is a sleeping stand-in, the relay a fake
commander whose agent's phase the test scripts."""
import io
import json
import sys
from pathlib import Path

import pytest

from scensim import drive, results, run


class FakeCommander:
    def __init__(self, phases, goal_status="OK"):
        self.phases = list(phases)   # (phase, has_pose) per query; the last repeats
        self.goal_status = goal_status
        self.commands = []
        self.at = (0.0, 0.0)   # where a pose of True puts the car

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

    def query(self, entity):
        phase, pose = self.phases.pop(0) if len(self.phases) > 1 else self.phases[0]
        if phase is None:
            return {"registered": False}
        if pose is True:
            pose = {"x": self.at[0], "y": self.at[1]}
        return {"registered": True, "state": {"phase": phase, "pose": pose or None}}

    def command(self, entity, name, timeout=None, **args):
        self.commands.append((entity, name, args))
        return {"status": self.goal_status, "message": "no route" if self.goal_status != "OK"
                else ""}


def make_ctx(tmp_path, pilots=None):
    project = tmp_path / "proj"
    (project / "assets" / "maps" / "Town01").mkdir(parents=True)
    (project / "swb.yaml").write_text("simulator: carla\n")
    stack_dir = tmp_path / "stack"
    (stack_dir / "proc").mkdir(parents=True)
    rec = {"stack_id": "s1", "simulator": "carla", "map": "Town01", "maps": ["Town01"],
           "proc": str(stack_dir / "proc"), "ports": {"ssv2": 5555, "relay": 5560},
           "domains": {"scenario": 1}, "env": {}, "seat": {"seat": 0},
           "pilots": pilots if pilots is not None else [
               {"entity": "ego", "kind": "simple", "domain": 2,
                "spec": {"vehicle": "tesla_model3", "inputs": ["map", "ground_truth"]}}]}
    return run.Context(project=project, host={}, stack=rec, runs_root=tmp_path / "runs",
                       out=io.StringIO())


SLEEP = [sys.executable, "-c", "import time; time.sleep(60)"]
EXIT = [sys.executable, "-c", "import sys; sys.exit(4)"]


@pytest.fixture(autouse=True)
def fast(monkeypatch):
    monkeypatch.setattr(drive, "POLL", 0.01)


def test_parse_pose():
    assert drive.parse_pose("1,2,0.5,1.57", "--spawn") == (1.0, 2.0, 0.5, 1.57)
    assert drive.parse_pose("1,2,3", "--spawn") == (1.0, 2.0, 0.0, 3.0)
    with pytest.raises(ValueError, match="--goal"):
        drive.parse_pose("1,x", "--goal")
    q = drive.pose_dict((0, 0, 0, 3.141592653589793))
    assert abs(q["qz"] - 1.0) < 1e-9 and abs(q["qw"]) < 1e-9


def test_free_drive_argv_takes_the_seats_port_and_the_pilots_car(tmp_path):
    ctx = make_ctx(tmp_path)
    a = drive.free_drive_argv(ctx.stack, "/m/Town01", "ego", "tesla_model3", (1, 2, 0, 0.5),
                              False, catalog="/c/vehicles.xosc")
    assert a[:4] == ["ros2", "run", "scenario_world_bridge", "scensim_free_drive"]
    assert a[a.index("--port") + 1] == "5555" and a[a.index("--entry") + 1] == "tesla_model3"
    assert a[a.index("--pose") + 1] == "1,2,0,0.5" and a[a.index("--realtime") + 1] == "false"


def test_the_goal_is_sent_once_the_pilot_is_up_and_arrival_passes(tmp_path):
    ctx = make_ctx(tmp_path)
    c = FakeCommander([(None, False), ("INITIALIZING", False), ("IDLE", True),
                       ("DRIVING", True), ("DRIVING", True), ("ARRIVED", True)])
    c.at = (1.0, 2.0)
    r = drive.drive(ctx, "ego", (1, 2, 0, 0), goal=(30, 2, 0, 0), timeout=10,
                    argv_fn=lambda *a: SLEEP, commander_fn=lambda rec: c)
    assert r.verdict == results.PASSED
    [(entity, name, args)] = c.commands
    assert (entity, name) == ("ego", "set_goal") and args["goal"]["x"] == 30
    assert [e["type"] for e in r.events] == ["route_complete"]
    tl = [json.loads(x)["event"] for x in
          (Path(ctx.runs_root) / r.run_id / "timeline.jsonl").read_text().splitlines()]
    assert tl == ["drive.start", "drive.up", "drive.goal", "route_complete"]
    d = json.loads((Path(ctx.runs_root) / r.run_id / "result.json").read_text())
    assert d["verdict"] == "passed" and d["pilots"]["ego"]["kind"] == "simple"
    assert (Path(ctx.runs_root) / r.run_id / "resolved.yaml").is_file()


def test_no_arrival_within_the_timeout_fails(tmp_path):
    ctx = make_ctx(tmp_path)
    c = FakeCommander([("IDLE", True), ("DRIVING", True)])
    r = drive.drive(ctx, "ego", (0, 0, 0, 0), goal=(9, 9, 0, 0), timeout=0.3,
                    argv_fn=lambda *a: SLEEP, commander_fn=lambda rec: c)
    assert r.verdict == results.FAILED and "did not arrive" in r.reason


def test_a_refused_goal_fails_naming_the_pilots_answer(tmp_path):
    ctx = make_ctx(tmp_path)
    c = FakeCommander([("IDLE", True)], goal_status="FAILED")
    r = drive.drive(ctx, "ego", (0, 0, 0, 0), goal=(9, 9, 0, 0), timeout=5,
                    argv_fn=lambda *a: SLEEP, commander_fn=lambda rec: c)
    assert r.verdict == results.FAILED and "refused the goal: FAILED no route" in r.reason


def test_no_goal_steps_until_the_timeout_and_passes(tmp_path):
    ctx = make_ctx(tmp_path)
    c = FakeCommander([("IDLE", True)])
    r = drive.drive(ctx, "ego", (0, 0, 0, 0), timeout=0.3, argv_fn=lambda *a: SLEEP,
                    commander_fn=lambda rec: c)
    assert r.verdict == results.PASSED and c.commands == [] and "no goal" in r.reason


def test_free_drive_ending_is_an_error(tmp_path):
    ctx = make_ctx(tmp_path)
    c = FakeCommander([(None, False)])
    r = drive.drive(ctx, "ego", (0, 0, 0, 0), goal=(1, 0, 0, 0), timeout=5,
                    argv_fn=lambda *a: EXIT, commander_fn=lambda rec: c)
    assert r.verdict == results.ERROR and "scensim_free_drive ended (4)" in r.reason


def test_a_pilot_that_never_comes_up_is_an_error(tmp_path):
    ctx = make_ctx(tmp_path)
    c = FakeCommander([("INITIALIZING", False)])
    r = drive.drive(ctx, "ego", (0, 0, 0, 0), goal=(1, 0, 0, 0), timeout=5, up_timeout=0.2,
                    argv_fn=lambda *a: SLEEP, commander_fn=lambda rec: c)
    assert r.verdict == results.ERROR and "was not on the episode" in r.reason


def test_an_entity_without_a_stack_pilot_is_refused(tmp_path):
    ctx = make_ctx(tmp_path)
    with pytest.raises(run.Refused, match="entity lead has no pilot"):
        drive.drive(ctx, "lead", (0, 0, 0, 0), argv_fn=lambda *a: SLEEP,
                    commander_fn=lambda rec: FakeCommander([("IDLE", True)]))
    run.release_turn(ctx)
    ctx2 = make_ctx(tmp_path / "b", pilots=[{"entity": "ego", "kind": "command", "domain": 2,
                                             "lifetime": "run", "spec": {}}])
    with pytest.raises(run.Refused, match="lifetime: run"):
        drive.drive(ctx2, "ego", (0, 0, 0, 0), argv_fn=lambda *a: SLEEP,
                    commander_fn=lambda rec: FakeCommander([("IDLE", True)]))


def test_main_refuses_a_bad_pose(capsys):
    assert drive.main(["--spawn", "1,2"]) == run.EXIT_REFUSED
    assert "four numbers" in capsys.readouterr().err


def test_a_pilot_left_up_elsewhere_is_not_sent_the_goal_until_it_is_at_the_spawn(tmp_path):
    """Seen live on Isaac (2026-11-16): a pilot up from an earlier run reported its old pose,
    got the goal before the spawn, planned from there and never arrived."""
    ctx = make_ctx(tmp_path)
    old = {"x": 41.8, "y": -66.3}
    c = FakeCommander([("ARRIVED", old), ("ARRIVED", old), ("IDLE", {"x": 99.0, "y": -20.5}),
                       ("DRIVING", True), ("ARRIVED", True)])
    seen = []
    q = c.query
    c.query = lambda e: (seen.append(len(c.commands)), q(e))[1]
    r = drive.drive(ctx, "ego", (99.19, -20.04, 0, -1.57), goal=(41.77, -66.28, 0, 3.14),
                    timeout=10, argv_fn=lambda *a: SLEEP, commander_fn=lambda rec: c)
    assert r.verdict == results.PASSED
    assert seen[:3] == [0, 0, 0]   # no goal while it reported the old pose
    assert drive.at_spawn({"x": 99.0, "y": -20.5}, (99.19, -20.04)) is True
    assert drive.at_spawn({"x": 41.8, "y": -66.3}, (99.19, -20.04)) is False


def test_a_poses_file_gives_the_spawn_and_the_goal(tmp_path):
    """Item 47: acb's pilot poses (initial_pose, goal_pose with a quaternion)."""
    f = tmp_path / "poses.yaml"
    f.write_text("initial_pose: {x: 190.77, y: -130.1, z: 0.0, qx: 0, qy: 0, "
                 "qz: 0.9993321134127607, qw: 0.03654212777035594}\n"
                 "goal_pose: {x: 153.63, y: -36.27, z: 0.0, qx: 0, qy: 0, "
                 "qz: -0.613963783246778, qw: 0.7893341959280007}\n")
    spawn, goal = drive.read_poses(f)
    assert spawn[:2] == (190.77, -130.1) and abs(spawn[3] - 3.0685) < 1e-3
    assert goal[:2] == (153.63, -36.27) and abs(goal[3] + 1.3216) < 1e-3
    g = tmp_path / "goal_only.yaml"
    g.write_text("goal_pose: {x: 88.4, y: -100.0, z: 0.0, qx: 0, qy: 0, qz: -0.7071, qw: 0.7071}\n")
    assert drive.read_poses(g)[0] is None


def test_main_needs_a_spawn(tmp_path, capsys):
    g = tmp_path / "goal_only.yaml"
    g.write_text("goal_pose: {x: 88.4, y: -100.0}\n")
    assert drive.main(["--poses", str(g)]) == run.EXIT_REFUSED
    assert "give --spawn" in capsys.readouterr().err
