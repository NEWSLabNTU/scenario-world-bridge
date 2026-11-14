"""swb.run and swb.api: run expansion, the stack lookup and refusals, and runs end to end
against a stack whose keeper is real and whose SSv2 is a stub writing a JUnit."""
import io
import json
import os
import signal
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest
import yaml

from swb import api, assets, project, results, run, stack, timeline

SWB_ROOT = Path(__file__).resolve().parents[1]

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
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 1}))
    (tmp / "fake_ssv2.py").write_text(FAKE_SSV2)
    (tmp / "fake_pilot.py").write_text(FAKE_PILOT)
    return host


def make(where, swb_yaml=None):
    d = where["tmp"] / "proj"
    project.new_project(d, "multi-pilot")
    if swb_yaml is not None:
        (d / "swb.yaml").write_text(swb_yaml)
    assets.fetch(d, out=lambda s: None)
    return d


def start_stack(d, ready=("true",), map_name="Town01"):
    p = stack.plan(project_arg=str(d), map_name=map_name)
    for x in p.record["pilots"]:
        x["ready"] = list(ready) if not callable(ready) else ready(x)
    stack.write_plan(p)
    err = p.stack_dir / "proc" / "bridge" / "2026-11-15_00-00-00" / "node" / "scenario_world_bridge"
    err.mkdir(parents=True)
    (err / "err").write_text("[ERROR] [1.0] [scenario_world_bridge]: before the run\n")
    env = dict(os.environ, PYTHONPATH=f"{SWB_ROOT}:{os.environ.get('PYTHONPATH', '')}")
    pilots_out = d.parent / "pilots"
    wrap = f"{sys.executable} {d.parent / 'fake_pilot.py'} {pilots_out}"
    proc = subprocess.Popen([sys.executable, "-m", "swb.stack", "keep", str(p.stack_dir),
                             "--poll", "0.05", "--pilot-wrap", wrap], env=env)
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
    assert argv[:6] == ["play_launch", "launch", "--disable-web-ui", "--log-dir", "/r/logs/scenario",
                        "swb_launch"]
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


def test_fingerprint_mismatch_refused(live, capsys):
    d, _ = live
    (d / "swb.yaml").write_text((d / "swb.yaml").read_text().replace("kind: carla_basic",
                                                                       "kind: none"))
    assert run.main(["scenarios/three_pilots.xosc", "--project", str(d)]) == 3
    err = capsys.readouterr().err
    assert "swb.yaml (stack sha256:" in err and "pilots: (stack" in err


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
    assert stored.provenance["simulator"] == "carla"
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
    assert resolved["env"]["SWB_MAPS"] == str(d / "assets" / "maps")


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
    town05.write_text(other.read_text().replace("SWB_MAPS)/Town01", "SWB_MAPS)/Town05"))
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
        sc.read_text().replace("SWB_MAPS)/Town01", "SWB_MAPS)/Town05"))
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
        lf = f'<LogicFile filepath="$(env SWB_MAPS)/{m}"/>' if m else ""
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
    assert argv[:4] == ["play_launch", "launch", "--disable-web-ui", "--log-dir"]
    assert argv[4].endswith("/logs/pilot-follower")
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
