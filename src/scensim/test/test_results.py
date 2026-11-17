import json
import subprocess

import pytest

from scensim import results
from scensim.results import ResultError, RunResult

PASS_XML = """<?xml version="1.0"?>
<testsuites name="/x/scenario_test_runner" failures="0" errors="0" tests="1">
  <testsuite name="town01_ego_drive" failures="0" errors="0" tests="1">
    <testcase name="town01_ego_drive" />
  </testsuite>
</testsuites>
"""
ERROR_XML = """<?xml version="1.0"?>
<testsuites name="/x" failures="0" errors="1" tests="1">
  <testsuite name="town01_ego_drive" failures="0" errors="1" tests="1">
    <testcase name="town01_ego_drive">
      <error type="SimulationError" message="No response from the simulator at tcp://localhost:5555 within 300 s" />
    </testcase>
  </testsuite>
</testsuites>
"""
FAIL_XML = """<?xml version="1.0"?>
<testsuites name="/x" failures="1" errors="0" tests="1">
  <testsuite name="s" failures="1" errors="0" tests="1">
    <testcase name="cut_in">
      <failure type="SimulationFailure" message="ego collided with cut_in" />
    </testcase>
  </testsuite>
</testsuites>
"""


def good():
    return {
        "schema": "swb.result/1",
        "run_id": "2026-11-14T10-21-07_cut_in_gap6_yield_r2",
        "verdict": "failed",
        "reason": "collision.vehicle with cut_in at 18.4 s",
        "scenario": {"file": "scenarios/cut_in.xosc", "sha256": "ab", "parameters": {"gap_m": 6},
                     "repetition": 2, "seed": 1002},
        "ssv2": {"junit": "junit.xml", "failures": 1, "errors": 0, "message": "x"},
        "events": [{"t_sim": 18.4, "t_wall": "2026-11-14T10:21:25Z", "type": "collision.vehicle",
                    "entity": "ego", "other": "cut_in"}],
        "metrics": {"duration_s": 24.0, "route_completion": 0.62},
        "pilots": {"ego": {"kind": "autoware", "vehicle": "tesla_model3"}},
        "provenance": {"swb": "abc"},
    }


def test_round_trip(tmp_path):
    r = RunResult.from_dict(good())
    assert r.to_dict() == good()
    again = RunResult.from_json(r.to_json())
    assert again == r
    p = r.write(tmp_path / "result.json")
    assert RunResult.read(p) == r
    assert json.loads(p.read_text())["schema"] == "swb.result/1"


def test_minimal_defaults():
    r = RunResult(run_id="x_r0", verdict="passed")
    d = json.loads(r.to_json())
    assert d["scenario"]["repetition"] == 0 and d["events"] == []


def test_validate_names_every_problem():
    d = good()
    d["verdict"] = "ok"
    d["extra"] = 1
    d["events"][0]["type"] = "crash"
    d["metrics"]["duration_s"] = "long"
    d["scenario"]["repetition"] = -1
    del d["run_id"]
    with pytest.raises(ResultError) as e:
        results.validate(d)
    msg = str(e.value)
    for s in ("verdict: 'ok'", "unknown key 'extra'", "events[0].type: 'crash'",
              "metrics.duration_s", "scenario.repetition", "missing key 'run_id'"):
        assert s in msg
    assert len(e.value.problems) == 6


def test_validate_schema_and_types():
    assert results.problems([]) == ["expected an object at the top, got list"]
    d = good()
    d["schema"] = "swb.result/2"
    d["reason"] = "two\nlines"
    p = results.problems(d)
    assert any("schema" in x for x in p) and any("one line" in x for x in p)
    with pytest.raises(ResultError):
        RunResult.from_json("{not json")


def test_event_types_fixed():
    assert results.EVENT_TYPES == ("collision.vehicle", "collision.pedestrian", "collision.static",
                                   "red_light", "off_lane", "stuck", "route_complete",
                                   "pilot.restart", "stack.error")


def write(tmp_path, text, name="junit.xml"):
    p = tmp_path / name
    p.write_text(text)
    return p


def test_verdict_passed(tmp_path):
    assert results.verdict_from(write(tmp_path, PASS_XML), []) == ("passed", "")


def test_verdict_failed(tmp_path):
    v, reason = results.verdict_from(write(tmp_path, FAIL_XML), [])
    assert v == "failed" and reason == "SimulationFailure: ego collided with cut_in"


def test_verdict_ssv2_error_is_error(tmp_path):
    v, reason = results.verdict_from(write(tmp_path, ERROR_XML), [])
    assert v == "error" and reason.startswith("SimulationError: No response")


def test_verdict_infra_overrides(tmp_path):
    v, reason = results.verdict_from(write(tmp_path, FAIL_XML),
                                     ["", "pilot ego never ready\n(120 s)"])
    assert (v, reason) == ("error", "pilot ego never ready (120 s)")


def test_verdict_missing_broken_empty(tmp_path):
    assert results.verdict_from(tmp_path / "none.xml")[0] == "error"
    assert results.verdict_from(None)[0] == "error"
    assert results.verdict_from(write(tmp_path, "<testsuites"))[0] == "error"
    v, reason = results.verdict_from(write(tmp_path, "<testsuites/>"))
    assert v == "error" and "no testcase" in reason


def test_ssv2_block(tmp_path):
    b = results.ssv2_block(write(tmp_path, ERROR_XML))
    assert b["errors"] == 1 and b["failures"] == 0 and b["message"].startswith("SimulationError")
    assert results.ssv2_block(None)["junit"] is None


def test_read_junit_bare_testsuite(tmp_path):
    j = results.read_junit(write(tmp_path, '<testsuite name="s"><testcase name="a"/></testsuite>'))
    assert j["tests"] == 1 and j["cases"][0]["suite"] == "s"


def git(cwd, *a):
    subprocess.run(["git", *a], cwd=cwd, check=True, capture_output=True,
                   env={"GIT_AUTHOR_NAME": "t", "GIT_AUTHOR_EMAIL": "t@t",
                        "GIT_COMMITTER_NAME": "t", "GIT_COMMITTER_EMAIL": "t@t",
                        "PATH": "/usr/bin:/bin", "HOME": str(cwd)})


def test_provenance(tmp_path):
    ws = tmp_path / "ws"
    ws.mkdir()
    git(ws, "init", "-q")
    (ws / "f").write_text("1")
    git(ws, "add", "f")
    git(ws, "commit", "-qm", "c")
    proj = tmp_path / "proj"
    proj.mkdir()
    (proj / "scensim.lock").write_text("version: 1\n")
    p = results.provenance(ws, proj)
    assert len(p["swb"]) == 40
    assert p["submodules"] == {}
    assert p["swb_lock"].startswith("sha256:") and len(p["swb_lock"]) == 71
    assert p["host"]
    (ws / "f").write_text("2")
    assert results.provenance(ws)["swb"].endswith("+dirty")


def test_provenance_tolerates_nothing(tmp_path):
    p = results.provenance(tmp_path / "missing", tmp_path)
    assert p["swb"] is None and p["submodules"] == {} and p["swb_lock"] is None


def test_submodule_pins_parse(monkeypatch):
    sha = "a" * 40
    text = f" {sha} src/a (v1)\n+{sha} src/b (heads/main)\n-{sha} src/c\n"
    monkeypatch.setattr(results, "_out", lambda *a, **k: text)
    assert results.submodule_pins(".") == {"src/a": sha, "src/b": sha + "+", "src/c": sha + "-"}


def test_provenance_names_the_build_and_the_checkout_apart(tmp_path):
    """roadmap 077, G29: result.json's swb is the install's build, not the checkout's HEAD
    at run time; the checkout is recorded beside it when it moved."""
    import subprocess
    ws = tmp_path / "ws"
    ws.mkdir()
    subprocess.run(["git", "init", "-q", str(ws)], check=True)
    g = ["git", "-C", str(ws), "-c", "user.email=t@t", "-c", "user.name=t"]
    (ws / "a").write_text("1")
    subprocess.run(g + ["add", "a"], check=True)
    subprocess.run(g + ["commit", "-qm", "one"], check=True)
    install = tmp_path / "install"
    (install / "scensim").mkdir(parents=True)
    results.stamp_build(ws, install)
    built = results.installed_build(install / "scensim")
    assert len(built["swb"]) == 40 and built["built"]
    p = results.provenance(ws, build=built)
    assert p["swb"] == built["swb"] and "checkout" not in p
    (ws / "a").write_text("2")
    subprocess.run(g + ["commit", "-qam", "two"], check=True)
    p = results.provenance(ws, build=built)
    assert p["swb"] == built["swb"] and p["checkout"] != built["swb"] and p["built"]
    assert results.installed_build(tmp_path / "nowhere") is None
    assert results.provenance(ws)["swb"] == p["checkout"]   # no stamp: the checkout, as before


def test_no_junit_says_why_from_ssv2s_own_log(tmp_path):
    """G48: an SSv2 that timed out wrote no JUnit; the reason said only that."""
    from scensim import results
    err = tmp_path / "2026-11-16_01-46-25" / "node" / "scenario_test_runner" / "err"
    err.parent.mkdir(parents=True)
    err.write_text("[INFO] [1.0] [simulation.scenario_test_runner]: starting\n"
                   "[ERROR] [1794765193.07] [simulation.scenario_test_runner]: The simulation "
                   "has timed out. Forcibly inactivate.\n")
    interp = tmp_path / "2026-11-16_01-46-25" / "node" / "openscenario_interpreter_node-1" / "err"
    interp.parent.mkdir(parents=True)
    interp.write_text("[ERROR] [1794765032.6] [status_monitor]: main is unresponsive\n")
    hint = results.ssv2_hint(tmp_path)
    assert hint == "The simulation has timed out. Forcibly inactivate."
    v, why = results.verdict_from(None, (), hint=hint)
    assert v == results.ERROR and why == "SSv2 wrote no JUnit: " + hint
    err.write_text("[INFO] [1.0] [x]: fine\n")
    assert results.ssv2_hint(tmp_path) == "main is unresponsive"
    assert results.ssv2_hint(tmp_path / "none") == ""
    assert "say nothing why" in results.verdict_from(None)[1]


def test_no_junit_names_the_runners_traceback(tmp_path):
    """swb 241: the runner died on a missing module and the reason said nothing."""
    from scensim import results
    err = tmp_path / "t" / "node" / "scenario_test_runner" / "err"
    err.parent.mkdir(parents=True)
    err.write_text("Traceback (most recent call last):\n  File \"x.py\", line 3, in <module>\n"
                   "    import xmlschema\nModuleNotFoundError: No module named 'xmlschema'\n")
    assert results.ssv2_hint(tmp_path) == "ModuleNotFoundError: No module named 'xmlschema'"


def test_provenance_git_calls_take_no_index_lock(monkeypatch, tmp_path):
    """Every git call of the run's provenance passes --no-optional-locks: `git status` would
    otherwise refresh the index under index.lock, and a call killed at its timeout on a loaded
    host left that lock behind, breaking the user's next commit (077, G73)."""
    from scensim import results
    seen = []

    class R:
        returncode, stdout = 0, "abc123\n"

    def fake_run(argv, **kw):
        seen.append(list(argv))
        return R()

    monkeypatch.setattr(results.subprocess, "run", fake_run)
    results.provenance(tmp_path)
    git = [a for a in seen if a and a[0] == "git" and any(w in a for w in ("status", "submodule"))]
    assert git, seen
    assert all("--no-optional-locks" in a for a in git), git
