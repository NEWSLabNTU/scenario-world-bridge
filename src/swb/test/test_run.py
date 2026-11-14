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


@pytest.fixture
def where(host):
    tmp = host["tmp"]
    (tmp / "system" / "host.yaml").write_text(yaml.safe_dump(
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 1}))
    (tmp / "fake_ssv2.py").write_text(FAKE_SSV2)
    return host


def make(where, swb_yaml=None):
    d = where["tmp"] / "proj"
    project.new_project(d, "multi-pilot")
    if swb_yaml is not None:
        (d / "swb.yaml").write_text(swb_yaml)
    assets.fetch(d, out=lambda s: None)
    return d


def start_stack(d, ready=("true",)):
    p = stack.plan(project_arg=str(d))
    for x in p.record["pilots"]:
        x["ready"] = list(ready)
    stack.write_plan(p)
    err = p.stack_dir / "proc" / "bridge" / "2026-11-15_00-00-00" / "node" / "scenario_world_bridge"
    err.mkdir(parents=True)
    (err / "err").write_text("[ERROR] [1.0] [scenario_world_bridge]: before the run\n")
    env = dict(os.environ, PYTHONPATH=f"{SWB_ROOT}:{os.environ.get('PYTHONPATH', '')}")
    proc = subprocess.Popen([sys.executable, "-m", "swb.stack", "keep", str(p.stack_dir),
                             "--poll", "0.05"], env=env)
    deadline = time.time() + 20
    while time.time() < deadline:
        rec = json.loads(p.stack_json.read_text())
        if rec.get("pid") and (rec["state"] == "ready" or ready != ("true",)):
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


def test_map_mismatch_refused(live, where):
    d, _ = live
    other = d / "scenarios" / "three_pilots.xosc"
    town05 = d / "town05.xosc"
    town05.write_text(other.read_text().replace("SWB_MAPS)/Town01", "SWB_MAPS)/Town05"))
    with pytest.raises(run.Refused, match="map Town05, the stack's pilots on Town01"):
        api.run_one(d, town05, {"scenario_command": fake(where)})


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
