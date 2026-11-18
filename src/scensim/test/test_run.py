"""scensim.run and scensim.api: run expansion, the stack lookup and refusals, and runs end to end
against a stack whose keeper is real and whose SSv2 is a stub writing a JUnit."""
import io
import json
import os
import re
import signal
import socket
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest
import yaml

from scensim import api, assets, project, record as recordmod, results, run, ssv2, stack, timeline

SCENSIM_ROOT = Path(__file__).resolve().parents[1]

FAKE_SSV2 = textwrap.dedent('''\
    import sys, time
    from pathlib import Path
    run_dir, outcome, proc = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
    t = time.time()
    print(f"[WARN] [{t:.6f}] [simulation.openscenario_interpreter]: slow frame", flush=True)
    err = proc / "bridge" / "2026-11-15_00-00-00" / "node" / "scenario_world_bridge" / "err"
    with open(err, "a") as f:
        f.write(f"[ERROR] [{t:.6f}] [scenario_world_bridge]: tick late\\n")
    if outcome == "error":
        sys.exit(1)
    j = run_dir / "logs" / "ssv2" / "scenario_test_runner" / "result.junit.xml"
    j.parent.mkdir(parents=True, exist_ok=True)
    case = ('<failure type="SimulationFailure" message="ego collided"/>'
            if outcome == "failed" else "")
    j.write_text('<testsuites><testsuite name="s"><testcase name="three_pilots">'
                 + case + '</testcase></testsuite></testsuites>')
''')


# a stack pilot in place of play_launch: notes its start (entity, map_path) and a ready file
# named for its map, then sleeps until the keeper stops it
FAKE_PILOT = textwrap.dedent('''\
    import json, signal, sys, time
    from pathlib import Path
    out, argv = Path(sys.argv[1]), sys.argv[2:]
    words = dict(w.split(":=", 1) for w in argv if ":=" in w)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "starts.jsonl", "a") as f:
        f.write(json.dumps({"entity": words.get("entity"), "map_path": words.get("map_path"),
                            "argv": argv}) + "\\n")
    ready = out / (words.get("entity", "x") + "." + Path(words.get("map_path", "")).name)
    ready.touch()
    signal.signal(signal.SIGINT, lambda *a: (ready.unlink(missing_ok=True), sys.exit(0)))
    time.sleep(600)
''')


@pytest.fixture
def where(host):
    tmp = host["tmp"]
    (tmp / "system" / "host.yaml").write_text(yaml.safe_dump(
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 1,
         # no CARLA here: the stack uses whatever serves its port and is not refused for
         # the lack of one (G39)
         "carla": {"start": "external"}}))
    (tmp / "fake_ssv2.py").write_text(FAKE_SSV2)
    (tmp / "fake_pilot.py").write_text(FAKE_PILOT)
    return host


def make(where, testbench_yaml=None):
    d = where["tmp"] / "proj"
    project.new_project(d, "multi-pilot")
    if testbench_yaml is not None:
        (d / "testbench.yaml").write_text(testbench_yaml)
    assets.fetch(d, out=lambda s: None)
    return d


def fake_bridge(rec):
    """A listener on the seat's SSv2 port, as the bridge binds it once its world server is
    up: the keeper marks the stack ready only then. None when the port is taken (then
    something serves it already)."""
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("127.0.0.1", stack.seat_of(rec).ssv2_port))
    except OSError:
        sock.close()
        return None
    sock.listen(8)
    return sock


def start_stack(d, ready=("true",), map_name="Town01"):
    p = stack.plan(project_arg=str(d), map_name=map_name)
    for x in p.record["pilots"]:
        x["ready"] = list(ready) if not callable(ready) else ready(x)
    stack.write_plan(p)
    err = p.stack_dir / "proc" / "bridge" / "2026-11-15_00-00-00" / "node" / "scenario_world_bridge"
    err.mkdir(parents=True)
    (err / "err").write_text("[ERROR] [1.0] [scenario_world_bridge]: before the run\n")
    env = dict(os.environ, PYTHONPATH=f"{SCENSIM_ROOT}:{os.environ.get('PYTHONPATH', '')}")
    pilots_out = d.parent / "pilots"
    wrap = f"{sys.executable} {d.parent / 'fake_pilot.py'} {pilots_out}"
    proc = subprocess.Popen([sys.executable, "-m", "scensim.stack", "keep", str(p.stack_dir),
                             "--poll", "0.05", "--pilot-wrap", wrap], env=env)
    proc.bridge = fake_bridge(p.record)
    deadline = time.time() + 20
    while time.time() < deadline:
        rec = json.loads(p.stack_json.read_text())
        if rec.get("pid") and (rec["state"] == "ready" or ready == ("false",)):
            break
        time.sleep(0.05)
    return p, proc


@pytest.fixture
def live(where, maps_hub):
    d = make(where)
    p, proc = start_stack(d)
    yield d, p
    proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=10)


def fake(where):
    script = str(where["tmp"] / "fake_ssv2.py")

    def cmd(rec, spec, run_dir, scenario):
        return [sys.executable, script, str(run_dir), str(spec.parameters.get("outcome", "passed")),
                rec["proc"]]
    return cmd


# --- expansion ------------------------------------------------------------------------------

def test_scenario_and_repeat(tmp_path):
    (tmp_path / "s.xosc").write_text("<OpenSCENARIO/>")
    specs, suite = run.expand(tmp_path / "s.xosc", tmp_path, repeat=3, params={"gap": 6})
    assert suite is None
    assert [s.key for s in specs] == ["s_gap-6_r0", "s_gap-6_r1", "s_gap-6_r2"]
    assert all(s.seed is None for s in specs)


def test_suite_grid_seeds_expect_distribution(tmp_path):
    (tmp_path / "scenarios" / "awf").mkdir(parents=True)
    for n in ("cut_in.xosc", "awf/a.xosc", "awf/known_issue.xosc", "dist.xosc"):
        (tmp_path / "scenarios" / n).write_text("<OpenSCENARIO/>")
    (tmp_path / "suites").mkdir()
    (tmp_path / "suites" / "c3.yaml").write_text(yaml.safe_dump({
        "scenarios": ["scenarios/cut_in.xosc", "scenarios/awf/"],
        "parameters": {"gap_m": [6, 10, 14], "policy": ["yield", "push"]},
        "distribution": "scenarios/dist.xosc",
        "repetitions": 2, "seed": 1000,
        "expect": {"awf/known_issue.xosc": "failed"},
        "pilot_args": {"cut_in": {"policy": "{param:policy}"}},
        "logging": {"keep": {"passed": "result"}}}))
    specs, suite = run.expand(tmp_path / "suites" / "c3.yaml", tmp_path)
    assert len(specs) == 3 * 6 * 2 + 2
    first = specs[0]
    assert first.key == "cut_in_gap_m-6_policy-yield_r0" and first.seed == 1000
    assert specs[1].key == "cut_in_gap_m-6_policy-yield_r1" and specs[1].seed == 1001
    assert specs[2].parameters == {"gap_m": 6, "policy": "push"}
    known = [s for s in specs if s.scenario.name == "known_issue.xosc"]
    assert known and all(s.expect == "failed" for s in known)
    assert all(s.expect is None for s in specs if s.scenario.name == "a.xosc")
    dist = [s for s in specs if s.distribution]
    assert [s.key for s in dist] == ["dist_r0", "dist_r1"] and dist[0].parameters == {}
    assert first.pilot_args == {"cut_in": {"policy": "{param:policy}"}}
    assert len({s.key for s in specs}) == len(specs)


def test_suite_refusals(tmp_path):
    (tmp_path / "s.yaml").write_text("scenarios: [nope.xosc]\n")
    with pytest.raises(run.Refused, match="nope.xosc: no such file"):
        run.expand(tmp_path / "s.yaml", tmp_path)
    (tmp_path / "s.yaml").write_text("scenario: [x]\n")
    with pytest.raises(run.Refused, match="unknown key scenario"):
        run.expand(tmp_path / "s.yaml", tmp_path)
    with pytest.raises(run.Refused, match="no such file"):
        run.expand(tmp_path / "missing.xosc", tmp_path)


def test_a_suites_pkg_scenario_is_the_packages_installed_file(tmp_path, monkeypatch):
    """Issue 054: a testbench names another package's scenario as pkg://<package>/<path>."""
    pytest.importorskip("ament_index_python")
    prefix = tmp_path / "install" / "shared_scenarios"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "shared_scenarios").write_text("")
    (prefix / "share" / "shared_scenarios" / "scenarios").mkdir(parents=True)
    xosc = prefix / "share" / "shared_scenarios" / "scenarios" / "drive.xosc"
    xosc.write_text("<OpenSCENARIO/>")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    tb = tmp_path / "tb"
    (tb / "suites").mkdir(parents=True)
    (tb / "suites" / "s.yaml").write_text(
        "scenarios: [pkg://shared_scenarios/scenarios/drive.xosc, pkg://shared_scenarios/scenarios]\n")
    specs, _ = run.expand(tb / "suites" / "s.yaml", tb)
    assert [s.scenario for s in specs] == [xosc, xosc]
    (tb / "suites" / "s.yaml").write_text("scenarios: [pkg://not_built/scenarios/x.xosc]\n")
    with pytest.raises(run.Refused, match="package not_built is not installed"):
        run.expand(tb / "suites" / "s.yaml", tb)


def test_shard():
    specs = list(range(10))
    assert run.shard(specs, "1/3") == [0, 3, 6, 9]
    assert run.shard(specs, "3/3") == [2, 5, 8]
    for bad in ("0/3", "4/3", "x", "1/0"):
        with pytest.raises(run.Refused):
            run.shard(specs, bad)


def test_scenario_argv_parameters():
    rec = {"ports": {"ssv2": 5655}, "pilots": [{"entity": "ego", "vehicle": "golf_cart"}]}
    spec = run.RunSpec(Path("/p/s.xosc"), {"gap_m": 6}, timeout=100)
    argv = run.scenario_argv(rec, spec, Path("/r"), Path("/p/s.xosc"))
    assert argv[:7] == ["play_launch", "launch", "--disable-web-ui", "--disable-diagnostics",
                        "--log-dir", "/r/logs/scenario", "scensim_launch"]
    assert "port:=5655" in argv and "vehicle_model:=golf_cart" in argv
    assert 'override_parameters:={"gap_m": "6"}' in argv
    assert "output_directory:=/r/logs/ssv2" in argv and "global_timeout:=100" in argv


# --- the stack ------------------------------------------------------------------------------

def test_no_stack_refused(where, maps_hub, capsys):
    d = make(where)
    assert run.main([str(d / "scenarios" / "three_pilots.xosc"), "--project", str(d)]) == 3
    assert "no stack of project proj" in capsys.readouterr().err


def test_not_ready_refused(where, maps_hub, capsys):
    d = make(where)
    p, proc = start_stack(d, ready=("false",))
    try:
        assert run.main(["scenarios/three_pilots.xosc", f"project:={d}"]) == 3
        err = capsys.readouterr().err
        assert "is not ready (state starting; waiting for ego, follower, lead)" in err
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=10)
    # stopped: not running
    assert json.loads(p.stack_json.read_text())["state"] == "stopped"
    assert run.main(["scenarios/three_pilots.xosc", f"project:={d}"]) == 3
    assert "is not running (state stopped)" in capsys.readouterr().err


def test_a_runs_provenance_lists_the_pilots_overlays(live, where):
    d, p = live
    rec = json.loads(p.stack_json.read_text())
    ov = [{"entry": "{workspace:drift}/install/setup.bash", "path": "/w/install/setup.bash",
           "commit": "abc"}]
    for x in rec["pilots"]:
        if x["entity"] == "ego":
            x["overlays"] = ov
    p.stack_json.write_text(json.dumps(rec))
    r = api.run_one(d, "scenarios/three_pilots.xosc",
                    {"logging": {"keep": {}}, "scenario_command": fake(where), "ready_timeout": 5})
    stored = results.RunResult.read(where["tmp"] / "logs" / "proj" / "runs" / r.run_id /
                                    "result.json")
    assert stored.provenance["overlays"] == {"ego": ov}


def test_fingerprint_mismatch_refused(live, capsys):
    d, _ = live
    (d / "testbench.yaml").write_text((d / "testbench.yaml").read_text().replace("kind: carla_basic",
                                                                       "kind: none"))
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d)]) == 3
    err = capsys.readouterr().err
    assert "testbench.yaml (stack sha256:" in err and "pilots: (stack" in err


# --- runs end to end ------------------------------------------------------------------------

def test_run_one_end_to_end(live, where):
    d, p = live
    r = api.run_one(d, "scenarios/three_pilots.xosc",
                    {"seed": 5, "logging": {"keep": {}}, "scenario_command": fake(where),
                     "ready_timeout": 5})
    assert r.verdict == "passed" and r.reason == ""
    run_dir = where["tmp"] / "logs" / "proj" / "runs" / r.run_id
    assert (d / "runs").resolve() == run_dir.parent.resolve()
    assert r.run_id.endswith("_three_pilots_r0")
    stored = results.RunResult.read(run_dir / "result.json")
    assert stored.scenario["file"] == "scenarios/three_pilots.xosc" and stored.scenario["seed"] == 5
    assert stored.provenance["stack_id"] == p.stack_id
    assert stored.provenance["launcher"] in ("play_launch", "ros2")  # 079: from stack.json
    assert stored.provenance["simulator"] == "carla"
    load = stored.provenance["load"]   # issue 046: the host's load at the start and the end
    assert len(load["start"]["loadavg"]) == 3 and load["end"]["cores"] >= 1
    assert load["start"]["t_wall"] <= load["end"]["t_wall"]
    assert "overlays" not in stored.provenance   # no pilot names a workspace of its own
    assert set(stored.pilots) == {"ego", "lead", "follower"}
    assert stored.ssv2["junit"] == "junit.xml" and (run_dir / "junit.xml").is_file()
    # the slice holds only the run's line of the stack's bridge log
    sl = run_dir / "logs" / "bridge" / "2026-11-15_00-00-00" / "node" / "scenario_world_bridge" / "err"
    assert sl.read_text().count("\n") == 1 and "tick late" in sl.read_text()
    tl = timeline.read(run_dir / "timeline.jsonl")
    events = [e["event"] for e in tl]
    assert events[0] == "run.start" and "run.end" in events
    assert "pilot.ready" in events and "ssv2.exit" in events
    groups = {(e["group"], e.get("message")) for e in tl if e.get("level") in ("WARN", "ERROR")}
    assert ("bridge", "tick late") in groups and ("scenario", "slow frame") in groups
    resolved = yaml.safe_load((run_dir / "resolved.yaml").read_text())
    assert resolved["stack"]["stack_id"] == p.stack_id
    assert resolved["env"]["ROS_DOMAIN_ID"] == "9"
    assert resolved["env"]["SCENSIM_MAPS"] == str(d / "assets" / "maps")


def test_keep_prunes_a_passed_run(live, where):
    d, _ = live
    r = api.run_one(d, "scenarios/three_pilots.xosc", {"scenario_command": fake(where)})
    run_dir = where["tmp"] / "logs" / "proj" / "runs" / r.run_id
    assert sorted(x.name for x in run_dir.iterdir()) == [
        "junit.xml", "resolved.yaml", "result.json", "timeline.jsonl"]


def test_error_when_ssv2_writes_nothing(live, where):
    d, _ = live
    r = api.run_one(d, "scenarios/three_pilots.xosc",
                    {"parameters": {"outcome": "error"}, "scenario_command": fake(where)})
    assert r.verdict == "error" and "no JUnit" in r.reason


def test_map_not_fetched_refused(live, where):
    d, p = live
    other = d / "scenarios" / "three_pilots.xosc"
    town05 = d / "town05.xosc"
    town05.write_text(other.read_text().replace("SCENSIM_MAPS)/Town01", "SCENSIM_MAPS)/Town05"))
    with pytest.raises(run.Refused, match="map Town05 is not fetched"):
        api.run_one(d, town05, {"scenario_command": fake(where)})
    assert not (p.stack_dir / stack.SWITCH_REQUEST).exists()  # nothing asked of the stack


def test_suite_cli_summary_resume_retry(live, where, monkeypatch):
    d, _ = live
    monkeypatch.setattr(run, "scenario_argv", fake(where))
    (d / "suites").mkdir()
    (d / "suites" / "s.yaml").write_text(yaml.safe_dump({
        "scenarios": ["scenarios/three_pilots.xosc"],
        "parameters": {"outcome": ["passed", "failed"]},
        "repetitions": 2, "seed": 1000,
        "logging": {"keep": {}}}))
    out = io.StringIO()
    assert run.main(["suites/s.yaml", "--project", str(d)], out=out) == 1
    lines = out.getvalue().splitlines()
    assert sum(ln.startswith("passed") for ln in lines) == 2
    assert sum(ln.startswith("failed") for ln in lines) == 2
    suite_dir = where["tmp"] / "logs" / "proj" / "suites" / "s"
    rows = (suite_dir / "summary.csv").read_text().splitlines()
    assert len(rows) == 5 and (suite_dir / "junit.xml").is_file()
    n_runs = len(list((where["tmp"] / "logs" / "proj" / "runs").iterdir()))
    out = io.StringIO()
    assert run.main(["suites/s.yaml", "--project", str(d), "--resume"], out=out) == 1
    assert all(ln.startswith(("skipped", "summary")) for ln in out.getvalue().splitlines())
    assert len(list((where["tmp"] / "logs" / "proj" / "runs").iterdir())) == n_runs
    # expect makes the failure the plan; retry repeats only errors
    (d / "suites" / "s.yaml").write_text(yaml.safe_dump({
        "scenarios": ["scenarios/three_pilots.xosc"],
        "parameters": {"outcome": ["failed", "error"]},
        "expect": {"three_pilots.xosc": "failed"},
        "retry": {"on": "error", "times": 1}}))
    out = io.StringIO()
    assert run.main(["suites/s.yaml", "--project", str(d), "--shard", "2/2"], out=out) == 2
    lines = [ln for ln in out.getvalue().splitlines() if not ln.startswith("summary")]
    assert len(lines) == 2 and all(ln.startswith("error") for ln in lines)
    out = io.StringIO()
    assert run.main(["suites/s.yaml", "--project", str(d), "--shard", "1/2"], out=out) == 0
    assert out.getvalue().startswith("failed (expected failed) ")


TWO_TOWNS = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01, Town05]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
  lead: {kind: carla_basic}
  follower: {kind: command, run: [python3, pilots/follower.py]}
"""


def two_towns(where):
    d = make(where, TWO_TOWNS)
    sc = d / "scenarios" / "three_pilots.xosc"
    (d / "scenarios" / "town05.xosc").write_text(
        sc.read_text().replace("SCENSIM_MAPS)/Town01", "SCENSIM_MAPS)/Town05"))
    return d


def starts(where):
    f = where["tmp"] / "pilots" / "starts.jsonl"
    return [json.loads(ln) for ln in f.read_text().splitlines()] if f.is_file() else []


def map_ready(where):
    """Each pilot's ready check: its fake's ready file for the stack's current map would need
    the map; the file of either town will do, the map is checked through starts.jsonl."""
    out = where["tmp"] / "pilots"
    return lambda x: ["sh", "-c", f"ls {out}/{x['entity']}.Town* >/dev/null 2>&1"]


def test_lazy_stack_switches_maps(where, maps_hub):
    d = two_towns(where)
    p, proc = start_stack(d, ready=map_ready(where), map_name=None)
    try:
        rec = json.loads(p.stack_json.read_text())
        assert rec["state"] == "ready" and rec["map"] is None
        assert rec["maps"] == ["Town01", "Town05"]
        assert starts(where) == []  # lazy: nothing started before the first run
        out = io.StringIO()
        r = api.run_one(d, "scenarios/town05.xosc", {"scenario_command": fake(where),
                                                     "ready_timeout": 20, "out": out})
        assert r.verdict == "passed", r.reason
        assert "map      Town05  (pilots restarted from none" in out.getvalue()
        town05 = str(d / "assets" / "maps" / "Town05")
        first = starts(where)
        assert sorted(x["entity"] for x in first) == ["ego", "follower", "lead"]
        assert all(x["map_path"] == town05 for x in first)
        rec = json.loads(p.stack_json.read_text())
        assert rec["map"] == "Town05" and rec["map_path"] == town05
        assert rec["switch"]["state"] == "done" and rec["switch"]["map"] == "Town05"
        assert rec["pilot_contract"]["map_path"] == town05
        assert rec["pilot_contract"]["traffic_light_state_path"].endswith(
            "state/Town05/carla/traffic_lights.yaml")
        run_dir = where["tmp"] / "logs" / "proj" / "runs" / r.run_id
        assert yaml.safe_load((run_dir / "resolved.yaml").read_text())["stack"]["map"] == "Town05"
        # the same map again: nothing restarted
        out = io.StringIO()
        r = api.run_one(d, "scenarios/town05.xosc", {"scenario_command": fake(where), "out": out})
        assert r.verdict == "passed" and "map " not in out.getvalue()
        assert len(starts(where)) == 3
        # another map: the pilots stopped and started with its map_path
        pids = {x["entity"]: x["pid"] for x in rec["pilots"]}
        r = api.run_one(d, "scenarios/three_pilots.xosc", {"scenario_command": fake(where),
                                                           "ready_timeout": 20})
        assert r.verdict == "passed", r.reason
        later = starts(where)[3:]
        assert sorted(x["entity"] for x in later) == ["ego", "follower", "lead"]
        assert all(x["map_path"] == str(d / "assets" / "maps" / "Town01") for x in later)
        rec = json.loads(p.stack_json.read_text())
        assert rec["map"] == "Town01"
        assert all(not os.path.exists(f"/proc/{pid}") or
                   open(f"/proc/{pid}/stat").read().split()[2] == "Z" for pid in pids.values())
        assert all(x["pid"] and x["pid"] not in pids.values() for x in rec["pilots"])
        ev = [json.loads(ln) for ln in (p.stack_dir / "events.jsonl").read_text().splitlines()]
        assert [e["map"] for e in ev if e["event"] == "map.switched"] == ["Town05", "Town01"]
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=60)
    # the keeper stopped its pilots on the way out
    rec = json.loads(p.stack_json.read_text())
    assert rec["state"] == "stopped" and all(x["pid"] is None for x in rec["pilots"])
    assert not list((where["tmp"] / "pilots").glob("*.Town*"))


def test_keeper_respawns_a_pilot(where, maps_hub):
    d = make(where)
    p, proc = start_stack(d)
    try:
        deadline = time.time() + 20
        while len(starts(where)) < 3 and time.time() < deadline:
            time.sleep(0.05)
        ego = json.loads(p.stack_json.read_text())["pilots"][0]
        os.killpg(ego["pid"], signal.SIGKILL)  # an episode end
        deadline = time.time() + 30
        while len(starts(where)) < 4 and time.time() < deadline:
            time.sleep(0.1)
        again = starts(where)[3:]
        assert [x["entity"] for x in again] == ["ego"]
        assert again[0]["map_path"] == str(d / "assets" / "maps" / "Town01")
        ev = [json.loads(ln)["event"] for ln in
              (p.stack_dir / "events.jsonl").read_text().splitlines()]
        assert "pilot.exit" in ev
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=60)


def test_switch_failure_is_an_error(where, maps_hub):
    d = two_towns(where)
    p, proc = start_stack(d, map_name="Town01")
    try:
        ctx = run.context(str(d))
        # the keeper finds the map gone after the runner checked it: an infrastructure error
        rec = dict(ctx.stack)
        real = stack.switch_problem
        calls = {"n": 0}

        def once_ok(r, m):
            calls["n"] += 1
            return None if calls["n"] == 1 else real(r, m)
        import shutil
        try:
            stack.switch_problem = once_ok
            target = d / "assets" / "maps" / "Town05"
            moved = d / "Town05.away"
            shutil.move(str(target), str(moved))
            msg = run.ensure_map(ctx, "Town05", timeout=20)
        finally:
            stack.switch_problem = real
            shutil.move(str(moved), str(target))
        assert msg and "could not switch to map Town05: map Town05 is not fetched" in msg
        assert ctx.stack["map"] == "Town01" == rec["map"]
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=60)


def test_order_by_map(tmp_path):
    maps = {"a": "Town01", "b": "Town05", "c": "Town01", "d": "Town03", "e": None}
    for n, m in maps.items():
        lf = f'<LogicFile filepath="$(env SCENSIM_MAPS)/{m}"/>' if m else ""
        (tmp_path / f"{n}.xosc").write_text(f"<OpenSCENARIO><RoadNetwork>{lf}</RoadNetwork>"
                                            "</OpenSCENARIO>")
    (tmp_path / "dist.xosc").write_text(
        '<OpenSCENARIO><ParameterValueDistribution><ScenarioFile filepath="b.xosc"/>'
        "</ParameterValueDistribution></OpenSCENARIO>")
    specs = [run.RunSpec(tmp_path / f"{n}.xosc", {}, k)
             for n in ("a", "b", "c", "dist", "d", "e", "a") for k in (0, 1)]
    names = lambda ss: [f"{s.scenario.stem}{s.repetition}" for s in ss]  # noqa: E731
    assert names(run.order_by_map(specs)) == [
        "a0", "a1", "c0", "c1", "a0", "a1",          # Town01, in their order
        "b0", "b1", "dist0", "dist1",                 # Town05 (the distribution's target)
        "d0", "d1", "e0", "e1"]
    assert names(run.order_by_map(specs, "Town03"))[:2] == ["d0", "d1"]
    assert run.scenario_map(tmp_path / "dist.xosc") == "Town05"


def test_suite_runs_ordered_by_map(where, maps_hub, monkeypatch):
    d = two_towns(where)
    p, proc = start_stack(d, ready=map_ready(where), map_name="Town05")
    monkeypatch.setattr(run, "scenario_argv", fake(where))
    (d / "suites").mkdir()
    (d / "suites" / "s.yaml").write_text(yaml.safe_dump({
        "scenarios": ["scenarios/three_pilots.xosc", "scenarios/town05.xosc"],
        "repetitions": 2, "logging": {"keep": {}}}))
    try:
        out = io.StringIO()
        assert run.main(["suites/s.yaml", "--project", str(d), "--ready-timeout", "20"],
                        out=out) == 0
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=60)
    lines = [ln for ln in out.getvalue().splitlines() if not ln.startswith("summary")]
    # the stack's own map first, then one switch
    assert [ln.split()[0] for ln in lines] == ["passed", "passed", "map", "passed", "passed"]
    assert "_town05_r0" in lines[0] and "_town05_r1" in lines[1]
    assert lines[2].startswith("map      Town01")
    assert "_three_pilots_r0" in lines[3]
    assert len(starts(where)) == 6  # started on Town05, then once more on Town01


RUN_PILOT = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
  lead: {kind: carla_basic}
  follower: {kind: command, lifetime: run,
             run: [python3, pilots/follower.py, --policy, "{param:policy}"]}
"""


def test_run_pilot_started_and_stopped(where, maps_hub):
    d = make(where, RUN_PILOT)
    p, proc = start_stack(d)
    seen = where["tmp"] / "pilot_argv.json"

    def pilot_cmd(rec, pilot, entry, log_dir):
        argv = run.pilot_argv(rec, pilot, entry, log_dir)
        return [sys.executable, "-c",
                f"import json, sys, time; json.dump(sys.argv[1:], open({str(seen)!r}, 'w'));"
                " time.sleep(60)", *argv]
    try:
        t = time.time()
        r = api.run_one(d, "scenarios/three_pilots.xosc",
                        {"parameters": {"policy": "push"}, "seed": 11, "logging": {"keep": {}},
                         "scenario_command": fake(where), "pilot_command": pilot_cmd})
        assert time.time() - t < 40  # the pilot was stopped, not waited for
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=10)
    assert r.verdict == "passed"
    argv = json.loads(seen.read_text())
    assert argv[:5] == ["play_launch", "launch", "--disable-web-ui", "--disable-diagnostics",
                        "--log-dir"]
    assert argv[5].endswith("/logs/pilot-follower")
    assert argv[argv.index("--composable-respawn") + 2].endswith("command.launch.py")
    words = dict(w.split(":=", 1) for w in argv if ":=" in w)
    assert words["entity"] == "follower" and words["seed"] == "11"
    assert words["run"].endswith("pilots/follower.py --policy push")
    assert words["relay"] == "tcp://localhost:5660"
    assert words["run_dir"].endswith(r.run_id)
    assert r.pilots["follower"]["lifetime"] == "run"


JUDGED = """simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
  lead: {kind: carla_basic}
  follower: {kind: command, run: [python3, pilots/follower.py]}
judges:
  - {name: gap, version: "2", run: [python3, judge.py]}
logging: {keep: {passed: result}, budget: {per_run: 1KB}}
"""


def test_judges_run_and_budget_refuses(where, maps_hub, monkeypatch):
    monkeypatch.setattr(run, "scenario_argv", fake(where))
    d = make(where, JUDGED)
    (d / "judge.py").write_text(
        "import json, sys\n"
        "r = json.load(open(sys.argv[1] + '/result.json'))\n"
        "print(json.dumps({'verdict': 'failed', 'run_verdict': r['verdict']}))\n")
    p, proc = start_stack(d)
    try:
        out = io.StringIO()
        assert run.main(["scenarios/three_pilots.xosc", "--project", str(d)], out=out) == 0
        assert "judges: gap=failed" in out.getvalue()
        run_dir = next((where["tmp"] / "logs" / "proj" / "runs").iterdir())
        rec = json.loads((run_dir / "judges" / "gap.json").read_text())
        assert rec["run_verdict"] == "passed" and rec["version"] == "2"
        # the run's verdict is the scenario's; the judge's sits beside it
        assert results.RunResult.read(run_dir / "result.json").verdict == "passed"
        assert any(e.get("group") == "judge" for e in timeline.read(run_dir / "timeline.jsonl"))
        # a suite larger than the budget's total is refused before any run
        (d / "suites").mkdir()
        (d / "suites" / "big.yaml").write_text(yaml.safe_dump({
            "scenarios": ["scenarios/three_pilots.xosc"], "repetitions": 3,
            "logging": {"budget": {"per_run": "1GB", "total": "2GB"}}}))
        assert run.main(["suites/big.yaml", "--project", str(d)]) == 3
        assert len(list((where["tmp"] / "logs" / "proj" / "runs").iterdir())) == 1
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=10)


def test_capture_full_records_each_pilot_domain(live, where, monkeypatch):
    d, _ = live
    monkeypatch.setattr(run, "scenario_argv", fake(where))
    started = []

    def fake_argv(out_dir, level, topics=None):
        started.append((out_dir.name, level))
        # a recorder that writes its bag directory and closes it on SIGINT
        return [sys.executable, "-c",
                "import signal, sys, time, pathlib\n"
                f"p = pathlib.Path({str(out_dir)!r}); p.mkdir(parents=True)\n"
                "signal.signal(signal.SIGINT, lambda *a: ((p / 'metadata.yaml').write_text('ok'),"
                " sys.exit(0)))\n"
                "time.sleep(60)\n"]
    monkeypatch.setattr(recordmod, "argv", fake_argv)
    out = io.StringIO()
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--capture", "full"],
                    out=out) == 0
    run_dir = next((where["tmp"] / "logs" / "proj" / "runs").iterdir())
    bags = sorted(x.name for x in (run_dir / "record").iterdir())
    assert bags and all((run_dir / "record" / b / "metadata.yaml").is_file() for b in bags)
    assert {lv for _, lv in started} == {"full"}
    assert (run_dir / "host.json").is_file()  # --capture keeps whatever the verdict


def test_record_plan():
    pilots = [{"entity": "ego", "domain": 1}, {"entity": "lead", "domain": 9},
              {"entity": "npc", "domain": 1}]
    full = recordmod.plan(Path("/r"), "full", pilots, 9)
    assert [(n, dm) for n, dm, _ in full] == [("ego", 1), ("lead", 9)]
    assert "/localization/kinematic_state" in full[0][2]
    dbg = recordmod.plan(Path("/r"), "debug", [{"entity": "ego", "domain": 1}], 9)
    assert [(n, a[-1]) for n, _, a in dbg] == [("ego", "-a"), ("scenario", "-a")]
    assert recordmod.plan(Path("/r"), "standard", pilots, 9) == []
    assert recordmod.topics_problems(["no_slash"]) and not recordmod.topics_problems(["/a"])


# --- a stack the runner starts --------------------------------------------------------------

# scensim.launch.xml in miniature, as the runner's stack_command: the plan (its pilots ready at
# once, run through fake_pilot.py), stack.json, then the keeper in this process
FAKE_LAUNCH = textwrap.dedent('''\
    import sys
    from scensim import stack
    fake_pilot, pilots_out = sys.argv[1], sys.argv[2]
    words = dict(w.split(":=", 1) for w in sys.argv[3:] if ":=" in w)
    try:
        p = stack.plan(project_arg=words["project"], seat_arg=words.get("seat"),
                       logs=words.get("logs"), map_name="Town01")
    except stack.StackRefused as e:
        print(f"scensim: the stack is refused (3): {e}", flush=True)
        sys.exit(1)
    for x in p.record["pilots"]:
        x["ready"] = ["true"]
    stack.write_plan(p)
    # the bridge's log, which fake_ssv2.py writes to
    (p.stack_dir / "proc" / "bridge" / "2026-11-15_00-00-00" / "node"
     / "scenario_world_bridge").mkdir(parents=True)
    wrap = [sys.executable, fake_pilot, pilots_out]
    def cmd(rec, pl, entry, log_dir):
        argv = stack.pilot_argv(rec, pl, entry, log_dir)
        return wrap + argv if argv else None
    # the bridge binds SSv2's port once its world server is up; the keeper waits for it
    import socket
    bridge = socket.socket()
    bridge.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        bridge.bind(("127.0.0.1", stack.seat_of(p.record).ssv2_port))
        bridge.listen(8)
    except OSError:
        pass
    sys.exit(stack.keep(p.stack_dir, poll=0.05, pilot_command=cmd))
''')


@pytest.fixture
def launcher(where, monkeypatch):
    """The runner's stack command, and the argv it was given."""
    script = where["tmp"] / "fake_launch.py"
    script.write_text(FAKE_LAUNCH)
    monkeypatch.setenv("PYTHONPATH", f"{SCENSIM_ROOT}:{os.environ.get('PYTHONPATH', '')}")
    monkeypatch.setattr(run, "scenario_argv", fake(where))
    calls = []

    def cmd(project, host, seat, log_dir):
        real = run.stack_argv(project, host, seat, log_dir)
        calls.append(real)
        return [sys.executable, str(script), str(where["tmp"] / "fake_pilot.py"),
                str(where["tmp"] / "pilots")] + [w for w in real if ":=" in w]
    yield cmd, calls
    # a stack a failed test left up (its keeper is the fake launch, a session leader)
    for f in (where["tmp"] / "logs" / "proj").glob("seat-*-current/stack.json"):
        rec = json.loads(f.read_text())
        if run.stack_alive(rec):
            os.killpg(rec["pid"], signal.SIGKILL)


def _stack_json(where):
    link = where["tmp"] / "logs" / "proj" / "seat-1-current" / "stack.json"
    return json.loads(link.read_text())


def test_stop_stack_starts_runs_and_stops(where, maps_hub, launcher):
    d = make(where)
    cmd, calls = launcher
    out = io.StringIO()
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--stop-stack",
                     "--ready-timeout", "5"], out=out, stack_command=cmd) == 0
    text = out.getvalue()
    assert "stack    proj on seat 1: starting (pid" in text and text.splitlines()[1].startswith("passed")
    assert "stopping the stack this run started" in text
    # the stack as the user starts it: play_launch of scensim.launch.xml, web UI on the seat's port
    assert calls[0][:4] == ["play_launch", "launch", "--web-addr", "127.0.0.1:8090"]
    assert calls[0][6:9] == ["scensim_launch", "scensim.launch.xml", f"project:={d}"]
    assert "seat:=1" in calls[0] and f"logs:={where['tmp'] / 'logs'}" in calls[0]
    rec = _stack_json(where)
    assert rec["state"] == "stopped" and not run.stack_alive(rec)
    console = next((where["tmp"] / "logs" / "proj" / "stack-launches").iterdir()) / "console.log"
    assert console.is_file()


def test_start_stack_leaves_it_up_and_reuses_a_live_one(where, maps_hub, launcher):
    d = make(where)
    cmd, calls = launcher
    out = io.StringIO()
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--start-stack",
                     "--ready-timeout", "5"], out=out, stack_command=cmd) == 0
    m = re.search(r"proj on seat 1: starting \(pid (\d+),", out.getvalue())
    assert m
    assert re.search(rf"proj on seat 1: started( \([^)]*\))?; left up; stop it with: "
                     rf"scensim stack {re.escape(str(d))} --stop", out.getvalue())
    pid = int(m.group(1))
    try:
        rec = _stack_json(where)
        assert run.stack_alive(rec)
        # the next --stop-stack uses it and leaves it: it did not start it
        out = io.StringIO()
        assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--stop-stack",
                         "--ready-timeout", "5"], out=out, stack_command=cmd) == 0
        assert len(calls) == 1 and "stack " not in out.getvalue()
        assert run.stack_alive(_stack_json(where))
    finally:
        os.killpg(pid, signal.SIGINT)
        deadline = time.time() + 20
        while time.time() < deadline and run.stack_alive(_stack_json(where)):
            time.sleep(0.05)
    assert _stack_json(where)["state"] == "stopped"


def test_a_started_stack_that_fails(where, maps_hub, launcher, capsys):
    d = make(where)
    refusing = lambda *a: [sys.executable, "-c",  # noqa: E731
                           "print('scensim: the stack is refused (3): project check found problems')"
                           "; raise SystemExit(1)"]
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--start-stack"],
                    stack_command=refusing) == 3
    assert "project check found problems" in capsys.readouterr().err
    dying = lambda *a: [sys.executable, "-c", "raise SystemExit(4)"]  # noqa: E731
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--start-stack"],
                    stack_command=dying) == 2
    assert "the stack ended (4) before it was ready" in capsys.readouterr().err
    slow = lambda *a: [sys.executable, "-c", "import time; time.sleep(60)"]  # noqa: E731
    t0 = time.time()
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--start-stack",
                     "--stack-timeout", "1"], stack_command=slow) == 2
    assert "not ready within 1 s" in capsys.readouterr().err and time.time() - t0 < 30


def test_the_stack_is_planned_here_first_and_a_refusal_said_plainly(where, maps_hub,
                                                                     monkeypatch, capsys):
    """G68: `--start-stack` plans the stack in its own process (stack.plan, the launch's call)
    before play_launch starts: a refusal exits 3 with the reason, not play_launch's parser
    error, and nothing starts."""
    d = make(where)
    asked, started = [], []

    def refusing_plan(**kw):
        asked.append(kw)
        raise run.stackmod.StackRefused("project check found problems:\ntestbench.yaml: pilots: ego: "
                                        "kind 'autowar' is not one of autoware, command")
    monkeypatch.setattr(run.stackmod, "plan", refusing_plan)
    monkeypatch.setattr(run.stackmod, "start_process", lambda *a, **k: started.append(a))
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d), "--start-stack"]) == 3
    err = capsys.readouterr().err
    assert "the stack is refused: project check found problems" in err
    assert "kind 'autowar' is not one of autoware, command" in err
    assert "--parser" not in err and started == []
    assert asked == [{"project_arg": str(d), "seat_arg": "1", "logs": str(where["tmp"] / "logs")}]


def test_api_stack_block(where, maps_hub, launcher):
    d = make(where)
    cmd, _ = launcher
    with api.stack(d, stack_command=cmd, out=io.StringIO()) as ctx:
        assert ctx.started is not None and run.stack_alive(ctx.stack)
        r = api.run_one(d, "scenarios/three_pilots.xosc",
                        {"scenario_command": fake(where), "ready_timeout": 5})
        assert r.verdict == "passed"
    assert _stack_json(where)["state"] == "stopped"


def test_sigterm_stops_what_the_runner_started(tmp_path):
    """SIGTERM is an interrupt: main's finally stops a stack this run started."""
    marker = tmp_path / "stopped"
    script = tmp_path / "runner.py"
    script.write_text(textwrap.dedent(f'''
        import sys, time, types
        from scensim import run
        class Started:
            proc = types.SimpleNamespace(pid=1)
            seat = 0
            def stop(self, **kw):
                open({str(marker)!r}, "w").write("yes")
        def context(*a, **kw):
            return types.SimpleNamespace(started=Started(), stack={{"map": None}},
                                         project=None, host={{}})
        run.context = context
        run.expand = lambda *a, **k: ([], None)
        run.order_by_map = lambda specs, m: specs
        def run_suite(*a, **k):
            print("running", flush=True)
            time.sleep(60)
        run.run_suite = run_suite
        sys.exit(run.cli(["x.xosc", "--stop-stack"]))
    '''))
    env = dict(os.environ, PYTHONPATH=f"{SCENSIM_ROOT}:{os.environ.get('PYTHONPATH', '')}")
    p = subprocess.Popen([sys.executable, str(script)], env=env, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE, text=True)
    assert p.stdout.readline().strip() == "running"
    p.send_signal(signal.SIGTERM)
    out, err = p.communicate(timeout=20)
    assert p.returncode == 130, err
    assert marker.read_text() == "yes"
    assert "stopping the stack this run started" in out and "interrupted" in err


def test_an_interrupted_run_stops_ssv2(live, where, monkeypatch):
    """An interrupt while SSv2 runs stops SSv2; left alone it joins the next run's bridge."""
    d, _ = live
    pidfile = where["tmp"] / "ssv2.pid"
    sleeper = where["tmp"] / "sleeper.py"
    sleeper.write_text(f"import os, time\nopen({str(pidfile)!r}, 'w').write(str(os.getpid()))\n"
                       "time.sleep(120)\n")
    started = {}
    real_start = run._start

    def start(argv, env, log, cwd=None):
        proc = real_start(argv, env, log, cwd)
        if str(sleeper) in argv:
            started["ssv2"] = proc

            def wait(timeout=None):
                deadline = time.time() + 10
                while not pidfile.exists() and time.time() < deadline:
                    time.sleep(0.05)
                raise KeyboardInterrupt
            proc.wait = wait
        return proc
    monkeypatch.setattr(run, "_start", start)
    with pytest.raises(KeyboardInterrupt):
        api.run_one(d, "scenarios/three_pilots.xosc",
                    {"scenario_command": lambda *a: [sys.executable, str(sleeper)]})
    pid = int(pidfile.read_text())
    deadline = time.time() + 30
    while time.time() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.1)
    else:
        os.kill(pid, signal.SIGKILL)
        raise AssertionError("SSv2 outlived the interrupted run")


def test_a_crash_looping_pilot_fails_the_run_fast(where, maps_hub):
    d = make(where, RUN_PILOT)
    p, proc = start_stack(d, ready=lambda x: ["false"] if x["entity"] == "follower" else ["true"])
    deadline = time.time() + 20
    while json.loads(p.stack_json.read_text()).get("state") != "ready" and time.time() < deadline:
        time.sleep(0.05)
    loop = ("import sys, time\n"
            "for n in range(1, 9):\n"
            "    print(f'2026-11-15T00:00:0{n}Z  WARN [node:/command_follower] Exited with code 1'"
            " f' (crash {n} since last healthy run)', flush=True)\n"
            "    time.sleep(0.2)\n"
            "time.sleep(60)\n")

    def pilot_cmd(rec, pilot, entry, log_dir):
        return [sys.executable, "-c", loop]
    try:
        t = time.time()
        r = api.run_one(d, "scenarios/three_pilots.xosc",
                        {"parameters": {"policy": "push"}, "scenario_command": fake(where),
                         "pilot_command": pilot_cmd})
        assert time.time() - t < 40   # not the readiness timeout
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=10)
    assert r.verdict == "error"
    assert "pilot follower is not coming up: its node /command_follower crashed" in r.reason


def test_crash_watch_counts_only_new_lines(tmp_path):
    log = tmp_path / "console.log"
    log.write_text("x [node:/a] Exited with code 1 (crash 9 since last healthy run)\n")
    w = run.CrashWatch(log)
    assert w.crashing() is None                       # before the watch: not this run's
    with open(log, "a") as f:
        f.write("x [node:/b] Exited with code 1 (crash 4 since last healthy run)\n")
    assert w.crashing() is None
    with open(log, "a") as f:
        f.write("x [node:/b] Exited without code (crash 5 since last healthy run)\n")
    assert w.crashing() == ("/b", 5)
    assert run.CrashWatch(tmp_path / "none.log").crashing() is None


def test_one_run_at_a_time_per_stack(live, where):
    """Two SSv2s in one domain drive each other's nodes: a second runner is refused."""
    d, _ = live
    holder = subprocess.Popen(
        [sys.executable, "-c",
         "import fcntl, json, os, sys, time\n"
         "fd = os.open(sys.argv[1], os.O_RDWR | os.O_CREAT)\n"
         "fcntl.flock(fd, fcntl.LOCK_EX)\n"
         "os.write(fd, json.dumps({'pid': os.getpid(), 'target': 'suites/x.yaml'}).encode())\n"
         "print('held', flush=True); time.sleep(60)\n",
         str(Path(json.loads(next(where["tmp"].glob("logs/proj/stacks/*/stack.json"))
                             .read_text())["proc"]).parent / run.RUN_LOCK)],
        stdout=subprocess.PIPE, text=True)
    try:
        assert holder.stdout.readline().strip() == "held"
        with pytest.raises(run.Refused, match=r"another run is going on stack .* suites/x.yaml"):
            api.run_one(d, "scenarios/three_pilots.xosc", {"scenario_command": fake(where)})
    finally:
        holder.kill()
        holder.wait()
    r = api.run_one(d, "scenarios/three_pilots.xosc", {"scenario_command": fake(where)})
    assert r.verdict == "passed"   # the turn is free once its holder is gone


# --- roadmap 077: SSv2's run settings, the gap trace per run, the frame budget --------------

def test_scenario_argv_takes_the_run_settings():
    rec = {"ports": {"ssv2": 5655}, "pilots": []}
    spec = run.RunSpec(Path("/p/s.xosc"), timeout=900,
                       ssv2={"frame_rate": 100, "managed_ego": False,
                             "global_real_time_factor": 0.5, "global_timeout": 300})
    argv = run.scenario_argv(rec, spec, Path("/r"), Path("/p/s.xosc"))
    assert "frame_rate:=100" in argv and "managed_ego:=false" in argv
    assert "global_real_time_factor:=0.5" in argv
    assert "global_timeout:=900" in argv and "global_timeout:=300" not in argv  # timeout wins


def test_suite_and_item_settings(tmp_path):
    (tmp_path / "scenarios").mkdir()
    for n in ("a.xosc", "b.xosc"):
        (tmp_path / "scenarios" / n).write_text("<OpenSCENARIO/>")
    (tmp_path / "s.yaml").write_text(yaml.safe_dump({
        "ssv2": {"global_timeout": 600, "frame_rate": 20},
        "scenarios": ["scenarios/a.xosc",
                      {"path": "scenarios/b.xosc", "ssv2": {"frame_rate": 100,
                                                           "managed_ego": False}}]}))
    specs, _ = run.expand(tmp_path / "s.yaml", tmp_path)
    by = {s.scenario.name: s.ssv2 for s in specs}
    assert by == {"a.xosc": {"global_timeout": 600, "frame_rate": 20},
                  "b.xosc": {"global_timeout": 600, "frame_rate": 100, "managed_ego": False}}
    (tmp_path / "s.yaml").write_text("scenarios: [{path: scenarios/a.xosc, ssv2: {fast: 1}}]\n")
    with pytest.raises(run.Refused, match="unknown key fast"):
        run.expand(tmp_path / "s.yaml", tmp_path)


FAKE_SSV2_TRACED = textwrap.dedent('''\
    import json, sys
    from pathlib import Path
    run_dir, proc, traces, contacts, trace = (Path(a) for a in sys.argv[1:6])
    window = ("2026-11-15T00:00:{s:02d}.000000Z  INFO carla_scenario_bridge::zmq_server: Frame "
              "budget, last 200 frames: 200 frames, p50/p95/max ms: processing {p}, tick "
              "1.0/2.0/3.0, UpdateEntityStatus 0.1/0.1/0.1; entities 3/3/3; processing > 25 ms: "
              "0, tick > 100 ms: 0")
    err = proc / "bridge" / "2026-11-15_00-00-00" / "node" / "scenario_world_bridge" / "err"
    with open(err, "a") as f:
        f.write(window.format(s=1, p="0.2/0.3/0.4") + "\\n")
        f.write(window.format(s=2, p="0.2/1.8/2.5") + "\\n")
    (traces / "20261115T000000Z_7.jsonl").write_text(trace.read_text())
    (contacts / "20261115T000000Z_e7.json").write_text(json.dumps({"contacts": []}))
    j = run_dir / "logs" / "ssv2" / "scenario_test_runner" / "result.junit.xml"
    j.parent.mkdir(parents=True, exist_ok=True)
    j.write_text('<testsuites><testsuite name="s"><testcase name="three_pilots"/>'
                 '</testsuite></testsuites>')
''')


def test_gap_trace_per_run_judged_and_frame_metrics(where, maps_hub, monkeypatch):
    """G2 and G23: the bridge's trace and contact report of a run land in its directory, a
    gap_oracle judge reads the trace there, and the frame budget becomes metrics."""
    from test_settings import trace_text
    monkeypatch.setenv("PYTHONPATH", f"{SCENSIM_ROOT}:{os.environ.get('PYTHONPATH', '')}")
    d = make(where, (project.TEMPLATES / "multi-pilot" / "testbench.yaml").read_text()
             + "judges:\n  - {name: gap, run: [python3, -m, scensim.gap, oracle],"
               " needs: [gap_trace]}\nlogging: {keep: {}}\n")
    (where["tmp"] / "fake_traced.py").write_text(FAKE_SSV2_TRACED)
    (where["tmp"] / "trace.jsonl").write_text(trace_text())
    p, proc = start_stack(d)

    def cmd(rec, spec, run_dir, scenario):
        e = rec["episodes"]
        return [sys.executable, str(where["tmp"] / "fake_traced.py"), str(run_dir), rec["proc"],
                e["gap_trace"], e["contacts"], str(where["tmp"] / "trace.jsonl")]
    try:
        r = api.run_one(d, "scenarios/three_pilots.xosc",
                        {"scenario_command": cmd, "ready_timeout": 5})
    finally:
        proc.send_signal(signal.SIGTERM)
        proc.wait(timeout=10)
    run_dir = where["tmp"] / "logs" / "proj" / "runs" / r.run_id
    assert r.verdict == "passed", r.reason
    assert [x.name for x in (run_dir / "gap_trace").glob("*.jsonl")] == [
        "20261115T000000Z_7.jsonl"]
    assert (run_dir / "contacts" / "20261115T000000Z_e7.json").is_file()
    assert not list(Path(p.record["episodes"]["gap_trace"]).iterdir())   # moved, not copied
    judged = json.loads((run_dir / "judges" / "gap.json").read_text())
    assert judged["verdict"] == "passed" and judged["traces"][0]["g4"] == "agree"
    m = results.RunResult.read(run_dir / "result.json").metrics
    assert (m["frame_windows"], m["frame_count"], m["frame_p95_ms"], m["frame_max_ms"]) == \
        (2, 400, 1.8, 2.5)
    events = [e["event"] for e in timeline.read(run_dir / "timeline.jsonl")]
    assert "gap_trace.collected" in events and "contacts.collected" in events


def test_managed_ego_false_hands_the_ego_to_no_pilot(live, where):
    d, _ = live
    seen = []

    def cmd(rec, spec, run_dir, scenario):
        seen.append(spec.ssv2)
        return fake(where)(rec, spec, run_dir, scenario)
    r = api.run_one(d, "scenarios/three_pilots.xosc",
                    {"scenario_command": cmd, "ready_timeout": 5, "logging": {"keep": {}},
                     "ssv2": {"managed_ego": False, "frame_rate": 100}})
    assert r.verdict == "passed"
    # global_timeout: three_pilots ends itself by 300 s of simulation time (G53)
    assert seen == [{"managed_ego": False, "frame_rate": 100, "global_timeout": 480}]
    run_dir = where["tmp"] / "logs" / "proj" / "runs" / r.run_id
    resolved = yaml.safe_load((run_dir / "resolved.yaml").read_text())
    assert set(resolved["pilots"]) == {"lead", "follower"}   # no ego: SSv2 drives it not
    assert set(results.RunResult.read(run_dir / "result.json").pilots) == {"lead", "follower"}


def test_the_stacks_play_launch_runs_on_the_seats_dds():
    """roadmap 077, G28: play_launch itself on the scenario domain and the stack's Cyclone
    profile, not the terminal's (Autoware's profile, domain 0)."""
    from scensim import config, seat as seatmod
    s = seatmod.table(config.DEFAULTS)[1]
    env = run.stack_env(s, {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1", "PATH": "/bin",
                            "CYCLONEDDS_URI": "file:///opt/autoware/1.5.0/config/cyclonedds.xml"})
    assert env["ROS_DOMAIN_ID"] == str(s.scenario_domain) == "9"
    assert env["CYCLONEDDS_URI"] == f"file://{seatmod.cyclone_profile()}"
    assert env["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert "ROS_LOCALHOST_ONLY" not in env and env["PATH"] == "/bin"


def test_wait_ssv2_stops_it_when_the_relay_says_a_goal_is_lost():
    """roadmap 077, G32: SSv2 is not left to its own timeout once a pilot's goal is gone."""
    proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                            start_new_session=True)   # as run._start: its own group
    calls = []

    def probe(rec, entities, since):
        calls.append(list(entities))
        return {"ego": "its agent registered again (it was DRIVING) and refused its goal"} \
            if len(calls) >= 2 else {}
    t0 = time.time()
    code, lost = run.wait_ssv2(proc, 30, {}, ["ego"], probe)
    assert code is None and "refused its goal" in lost["ego"]
    assert proc.poll() is not None and time.time() - t0 < 15
    assert calls[0] == ["ego"]
    quick = subprocess.Popen([sys.executable, "-c", "pass"])
    assert run.wait_ssv2(quick, 30, {}, ["ego"], lambda *a: {}) == (0, {})


def test_a_goal_lost_before_the_run_began_is_not_this_runs(monkeypatch):
    """Seen live 2026-11-16: the keeper restarted the ego's pilot for a map switch between
    runs, the relay marked it lost (its last phase was DRIVING), and the next run ended error
    at its first poll. A loss older than this run's SSv2 is ignored."""
    import types
    replies = {"ego": {"registered": True, "lost": "gone", "lost_at": 100.0}}

    class C:
        def __init__(self, *a, **k):
            pass
        __enter__ = lambda self: self
        __exit__ = lambda self, *e: None
        query = lambda self, e: replies[e]
    fake = types.ModuleType("scensim_agent.commander")
    fake.Commander, fake.CommanderError = C, RuntimeError
    monkeypatch.setitem(sys.modules, "scensim_agent.commander", fake)
    rec = {"ports": {"relay": 1}}
    assert run.lost_goals(rec, ["ego"], since=200.0) == {}
    assert run.lost_goals(rec, ["ego"], since=50.0) == {"ego": "gone"}


def test_a_lost_goal_ends_the_run_error_with_the_relays_reason(live, where):
    d, _ = live
    sleeper = where["tmp"] / "sleeper.py"
    sleeper.write_text("import time\ntime.sleep(60)\n")
    lost = {"ego": "its agent registered again (it was DRIVING) and reported no state"}
    r = api.run_one(d, "scenarios/three_pilots.xosc",
                    {"scenario_command": lambda *a: [sys.executable, str(sleeper)],
                     "lost_probe": lambda rec, entities, since: lost if "ego" in entities else {}})
    assert r.verdict == results.ERROR
    assert "pilot ego: its agent registered again (it was DRIVING)" in r.reason



@pytest.mark.parametrize("line", [
    "ERROR container:/pc: Composable node 'composable:/euclidean_cluster' crashed: killed by "
    "signal 11 (Segmentation fault)",
    "[ERROR] [component_container_mt-58]: process has died [pid 9, exit code -11, cmd "
    "'component_container_mt --ros-args -r __node:=pointcloud_container -r __ns:=/']."])
def test_a_pilot_crash_during_the_run_ends_it_error_under_either_launcher(live, where, line):
    """079 step 5: part of the autopilot crashed, so the run did not test it as configured:
    `error`, naming the crash, whichever launcher's line says so; then the stack's pilots
    start afresh, before any other run finds them broken."""
    d, _ = live

    def starts():
        rec = _stack_json(where)
        text = (Path(rec["proc"]).parent / "events.jsonl").read_text()
        return sum(json.loads(x).get("event") == "pilot.start" for x in text.splitlines())

    crasher = where["tmp"] / "crasher.py"
    crasher.write_text("import sys, time\ntime.sleep(0.5)\n"
                       "open(sys.argv[1], 'a').write(sys.argv[2] + '\\n')\ntime.sleep(60)\n")
    before = starts()
    r = api.run_one(d, "scenarios/three_pilots.xosc",
                    {"scenario_command": lambda rec, *a: [
                        sys.executable, str(crasher),
                        str(Path(rec["proc"]) / "pilot-ego" / "console.log"), line]})
    assert r.verdict == results.ERROR, r.reason
    assert r.reason.startswith("pilot ego: ") and "during the run" in r.reason
    assert ("euclidean_cluster" in r.reason) or ("pointcloud_container" in r.reason)
    assert starts() > before
    # `runs why` names the crash (079's exit criterion)
    import io
    from scensim import runs as runsmod
    out = io.StringIO()
    runsmod.main(["why", "--root", str(where["tmp"] / "logs" / "proj" / "runs"), r.run_id],
                 out=out)
    assert "reason   pilot ego: " in out.getvalue() and "during the run" in out.getvalue()


def test_a_display_only_crash_during_the_run_is_a_warning_and_the_run_stands(live, where):
    """079 step 5: rviz2 shows, it does not drive; its death mid-run is a warning in the
    run's output and timeline, and a passing run stays `passed`."""
    import io
    d, _ = live
    line = ("[ERROR] [rviz2-32]: process has died [pid 9, exit code -6, cmd "
            "'/opt/ros/humble/lib/rviz2/rviz2 -d x']")
    out = io.StringIO()

    def cmd(rec, *a):
        console = str(Path(rec["proc"]) / "pilot-ego" / "console.log")
        return ["sh", "-c", 'printf "%s\\n" "$1" >> "$0"; shift; exec "$@"', console, line,
                *fake(where)(rec, *a)]
    r = api.run_one(d, "scenarios/three_pilots.xosc", {"scenario_command": cmd, "out": out})
    assert r.verdict == results.PASSED, r.reason
    assert "warning  pilot ego: rviz2 died (exit code -6) during the run" in out.getvalue()
    tl = timeline.read(where["tmp"] / "logs" / "proj" / "runs" / r.run_id / "timeline.jsonl")
    assert any(e.get("event") == "pilot.warning" and e.get("level") == "WARN" for e in tl)


def test_pilots_that_do_not_come_back_are_said_once(monkeypatch):
    """079 step 5: a restart after a crash whose pilots never become ready is reported once;
    the command's later runs end `error` with it at once, not after their own wait."""
    import types
    monkeypatch.setattr(stack, "check_ready", lambda *a, **k: False)
    ctx = types.SimpleNamespace(
        stack={"pilots": [{"entity": "ego", "domain": 2, "ready": ["false"]}],
               "domains": {"scenario": 1}, "env": {}},
        ready_timeout=0.1)
    msg = run.wait_pilots_ready(ctx)
    assert msg.startswith("pilot ego never became ready")


def test_each_run_ends_its_pilots_episodes_at_the_relay(live, where):
    """G57: the relay forgets a standing goal only at an episode boundary; run_suite sent
    episode_end, scensim run did not, and the next run's agent got the old goal."""
    d, _ = live
    seen = []
    r = api.run_one(d, "scenarios/three_pilots.xosc",
                    {"end_episodes": lambda rec, entities, verdict, reason, run_id:
                     seen.append((sorted(entities), verdict, run_id))})
    assert seen == [(["ego", "follower", "lead"], r.verdict, r.run_id)]

HARUNA_LIKE = """<?xml version="1.0"?>
<OpenSCENARIO>
  <ParameterDeclarations><ParameterDeclaration name="limit" parameterType="double" value="600"/>
  </ParameterDeclarations>
  <RoadNetwork><LogicFile filepath="$(env SCENSIM_MAPS)/Town01"/></RoadNetwork>
  <Entities><ScenarioObject name="ego"><ObjectController><Controller name="Autoware"/>
  </ObjectController></ScenarioObject></Entities>
  <Storyboard><Story name="s"><Act name="a"><ManeuverGroup name="g"><Maneuver name="m">
    <Event name="arrived" priority="parallel">
      <Action name="ok"><UserDefinedAction><CustomCommandAction type="exitSuccess"/>
      </UserDefinedAction></Action>
      <StartTrigger><ConditionGroup>
        <Condition name="t" delay="0" conditionEdge="none"><ByValueCondition>
          <SimulationTimeCondition value="5" rule="greaterThan"/></ByValueCondition></Condition>
        <Condition name="there" delay="0" conditionEdge="rising"><ByEntityCondition/>
        </Condition>
      </ConditionGroup></StartTrigger>
    </Event>
    <Event name="say" priority="parallel">
      <Action name="note"><UserDefinedAction><CustomCommandAction type="echo"/>
      </UserDefinedAction></Action>
      <StartTrigger><ConditionGroup><Condition name="t" delay="0" conditionEdge="none">
        <ByValueCondition><SimulationTimeCondition value="30" rule="greaterThan"/>
        </ByValueCondition></Condition></ConditionGroup></StartTrigger>
    </Event>
  </Maneuver></ManeuverGroup></Act></Story>
  <StopTrigger><ConditionGroup><Condition name="limit" delay="0" conditionEdge="none">
    <ByValueCondition><SimulationTimeCondition value="$limit" rule="greaterThan"/>
    </ByValueCondition></Condition></ConditionGroup></StopTrigger>
  </Storyboard>
</OpenSCENARIO>
"""


def test_a_scenarios_own_limit(tmp_path):
    """G53: the simulation time by which a scenario ends itself, as haruna_down's 600 s."""
    f = tmp_path / "h.xosc"
    f.write_text(HARUNA_LIKE)
    # the StopTrigger's 600 s (a parameter); the 5 s is ANDed with an arrival, the 30 s
    # event does not exit
    assert project.read_scenario(f).time_limit == 600
    f.write_text(HARUNA_LIKE.replace('type="echo"', 'type="exitFailure"'))
    assert project.read_scenario(f).time_limit == 30   # the earliest exit
    f.write_text(HARUNA_LIKE.replace('value="$limit"', 'value="${limit} * 2"'))
    assert project.read_scenario(f).time_limit is None   # an expression: not worked out
    # a TIER IV YAML scenario
    acc = (Path(__file__).resolve().parents[2] / "scensim_examples" / "scenarios" / "awf"
           / "UC-ACC-001-0001_Preceding_vehicle_partial_deceleration.yaml")
    if acc.is_file():
        assert project.read_scenario(acc).time_limit == 180
    # a distribution's variants are its scenario file's runs
    (tmp_path / "d.xosc").write_text(
        '<OpenSCENARIO><ParameterValueDistribution><ScenarioFile filepath="h.xosc"/>'
        "</ParameterValueDistribution></OpenSCENARIO>")
    f.write_text(HARUNA_LIKE)
    assert project.read_scenario(tmp_path / "d.xosc").time_limit == 600


def test_timeout_for_limit():
    assert ssv2.timeout_for_limit(None, {}) is None
    # haruna_down: 600 s of simulation time, the concealer's 120 s, a 60 s margin
    assert ssv2.timeout_for_limit(600, {}) == 780
    assert ssv2.timeout_for_limit(600, {"global_real_time_factor": 0.5,
                                        "initialize_duration": 60}) == 1380


def test_a_scenario_longer_than_the_timeout_gets_its_own_limit(live, where):
    """G53: haruna_down (600 s) against SSv2's default 400 s ran out as error before it
    decided; the run now gives SSv2 its limit plus the margin, unless the call says."""
    d, _ = live
    sc = (d / "scenarios" / "three_pilots.xosc")
    sc.write_text(sc.read_text().replace('SimulationTimeCondition value="300"',
                                         'SimulationTimeCondition value="600"'))
    seen = []

    def cmd(rec, spec, run_dir, scenario):
        seen.append(run.scenario_argv(rec, spec, run_dir, scenario))
        return fake(where)(rec, spec, run_dir, scenario)
    base = {"scenario_command": cmd, "ready_timeout": 5, "logging": {"keep": {}}}
    r = api.run_one(d, "scenarios/three_pilots.xosc", base)
    assert r.verdict == "passed"
    assert "global_timeout:=780" in seen[-1]
    run_dir = where["tmp"] / "logs" / "proj" / "runs" / r.run_id
    resolved = yaml.safe_load((run_dir / "resolved.yaml").read_text())
    assert resolved["ssv2"]["timeout_raised"] == {"scenario_limit_s": 600.0, "configured_s": 400,
                                                  "given_s": 780}
    assert resolved["ssv2"]["timeout_s"] == 780 + run.END_MARGIN
    assert any(e["event"] == "ssv2.timeout_raised"
               for e in timeline.read(run_dir / "timeline.jsonl"))
    # a configured timeout already long enough is kept
    r = api.run_one(d, "scenarios/three_pilots.xosc", {**base, "ssv2": {"global_timeout": 900}})
    assert "global_timeout:=900" in seen[-1]
    resolved = yaml.safe_load((where["tmp"] / "logs" / "proj" / "runs" / r.run_id
                               / "resolved.yaml").read_text())
    assert "timeout_raised" not in resolved["ssv2"]
    # the call's --timeout is the last word, even shorter
    api.run_one(d, "scenarios/three_pilots.xosc", {**base, "timeout": 300})
    assert "global_timeout:=300" in seen[-1]


def test_the_stack_check_finds_a_suites_pkg_scenario(tmp_path, monkeypatch):
    """Issue 054, part 3: the check a stack runs before it starts (collect_scenarios) resolves
    pkg:// as the run does; an uninstalled package is the host's finding."""
    pytest.importorskip("ament_index_python")
    from scensim import project as projectmod
    prefix = tmp_path / "install"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "shared_scenarios").write_text("")
    (prefix / "share" / "shared_scenarios" / "scenarios").mkdir(parents=True)
    xosc = prefix / "share" / "shared_scenarios" / "scenarios" / "drive.xosc"
    xosc.write_text("<OpenSCENARIO/>")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    tb = tmp_path / "tb"
    (tb / "suites").mkdir(parents=True)
    (tb / "suites" / "s.yaml").write_text(
        "scenarios: [pkg://shared_scenarios/scenarios/drive.xosc, pkg://not_built/x.xosc]\n")
    files, found = projectmod.collect_scenarios(tb)
    assert files == [xosc]
    assert [(f.host, "package not_built is not installed" in f.what) for f in found] == [(True, True)]
