import csv
import io
import os
import time
import xml.etree.ElementTree as ET

import pytest

from scensim import runs, timeline
from scensim.results import RunResult


def make_run(root, run_id, verdict, reason="", age_days=0, params=None, levels=True):
    d = root / run_id
    d.mkdir(parents=True)
    RunResult(run_id=run_id, verdict=verdict, reason=reason,
              scenario={"file": "scenarios/cut_in.xosc", "sha256": None,
                        "parameters": params or {}, "repetition": 1, "seed": 1001},
              metrics={"duration_s": 24.0}).write(d / "result.json")
    (d / "resolved.yaml").write_text("x: 1\n")
    (d / "junit.xml").write_text(
        '<testsuites><testsuite name="s"><testcase name="cut_in">'
        + ('<failure type="SimulationFailure" message="hit"/>' if verdict == "failed" else "")
        + "</testcase></testsuite></testsuites>")
    timeline.write([
        {"t_wall": 100.0, "t_sim": None, "group": "run", "event": "run.start"},
        {"t_wall": 101.5, "t_sim": 1.0, "group": "bridge", "process": "scensim", "event": "log.warn",
         "level": "WARN", "message": "slow tick"},
        {"t_wall": 102.0, "t_sim": None, "group": "pilot-ego", "process": "acb_bridge",
         "event": "log.error", "level": "ERROR", "message": "lost CARLA"},
    ], d / "timeline.jsonl")
    if levels:
        (d / "logs" / "bridge").mkdir(parents=True)
        (d / "logs" / "bridge" / "scensim.log").write_text("x")
        (d / "record").mkdir()
        (d / "record" / "bag.mcap").write_text("x")
        (d / "host.json").write_text("{}")
        (d / "debug").mkdir()
        (d / "debug" / "all.mcap").write_text("x")
    if age_days:
        t = time.time() - age_days * 86400
        os.utime(d / "result.json", (t, t))
    return d


def call(*argv):
    out = io.StringIO()
    code = runs.main(list(argv), out=out)
    return code, out.getvalue()


@pytest.fixture
def root(tmp_path):
    r = tmp_path / "runs"
    make_run(r, "2026-11-14T10-00-00_cut_in_r0", "passed")
    make_run(r, "2026-11-14T10-01-00_cut_in_r1", "failed", "SimulationFailure: hit", age_days=10)
    make_run(r, "2026-11-14T10-02-00_cut_in_r2", "error", "pilot ego never ready", age_days=10)
    (r / "2026-11-14T10-03-00_running_r0").mkdir()  # no result yet
    return r


def names(d):
    return sorted(p.name for p in d.iterdir())


def test_list(root):
    code, out = call("list", "--root", str(root))
    assert code == 0
    lines = out.splitlines()
    assert lines[0].split() == ["RUN", "VERDICT", "SCENARIO", "DURATION", "WORKSPACE"]
    assert lines[2].split() == ["2026-11-14T10-01-00_cut_in_r1", "failed", "cut_in.xosc", "24.0s", "-"]
    assert "(none)" in lines[4]
    # columns aligned
    assert len({ln.index(ln.split()[1]) for ln in lines}) == 1


def test_list_from_project(tmp_path, monkeypatch):
    proj = tmp_path / "p"
    proj.mkdir()
    (proj / "testbench.yaml").write_text("simulator: carla\n")
    make_run(proj / "runs", "a_r0", "passed")
    code, out = call("list", "--project", str(proj))
    assert code == 0 and "a_r0" in out
    monkeypatch.chdir(tmp_path)
    assert call("list")[0] == 3


def test_show(root):
    code, out = call("show", "--root", str(root), "cut_in_r1")
    assert code == 0
    assert "verdict   failed" in out and "reason    SimulationFailure: hit" in out
    rows = [ln for ln in out.splitlines() if "log." in ln]
    assert "+  1.500" in rows[0] and "slow tick" in rows[0]
    assert rows[0].index("slow tick") == rows[1].index("lost CARLA")


def test_show_unknown_and_ambiguous(root):
    assert call("show", "--root", str(root), "nope")[0] == 3
    assert call("show", "--root", str(root), "cut_in")[0] == 3
    assert call("show", "--root", str(root / "missing"), "x")[0] == 3


def test_why(root):
    code, out = call("why", "--root", str(root), "cut_in_r2")
    assert code == 0
    assert "reason   pilot ego never ready" in out
    assert "first    WARN in bridge/scensim: slow tick" in out
    assert "error    ERROR in pilot-ego/acb_bridge: lost CARLA" in out


def test_prune_verdict_dry_run_then_remove(root):
    code, out = call("prune", "--root", str(root), "--verdict", "passed", "--dry-run")
    assert code == 0 and "would remove 2026-11-14T10-00-00_cut_in_r0" in out
    assert (root / "2026-11-14T10-00-00_cut_in_r0").exists()
    call("prune", "--root", str(root), "--verdict", "passed")
    assert not (root / "2026-11-14T10-00-00_cut_in_r0").exists()
    assert (root / "2026-11-14T10-03-00_running_r0").exists()  # unfinished: never pruned


def test_prune_older_than_and_keep_last(root):
    call("prune", "--root", str(root), "--older-than", "5d", "--keep-last", "1")
    assert names(root) == ["2026-11-14T10-00-00_cut_in_r0", "2026-11-14T10-02-00_cut_in_r2",
                           "2026-11-14T10-03-00_running_r0"]


def test_prune_to_level(root):
    d = root / "2026-11-14T10-01-00_cut_in_r1"
    code, out = call("prune", "--root", str(root), "--verdict", "failed", "--to", "result")
    assert code == 0
    assert names(d) == ["junit.xml", "resolved.yaml", "result.json", "timeline.jsonl"]


def test_prune_bad_args(root):
    assert call("prune", "--root", str(root), "--older-than", "soon")[0] == 3
    assert call("prune", "--root", str(root), "--keep-last", "-1")[0] == 3
    assert call("prune", "--root", str(root), "--to", "debug")[0] == 3
    assert call("frobnicate")[0] == 3


def test_apply_keep_levels(tmp_path):
    full = {"result.json", "resolved.yaml", "junit.xml", "timeline.jsonl", "logs", "record",
            "host.json"}
    cases = [("result", {"result.json", "resolved.yaml", "junit.xml", "timeline.jsonl"}),
             ("standard", {"result.json", "resolved.yaml", "junit.xml", "timeline.jsonl",
                           "logs"}),
             ("full", full), ("debug", full | {"debug"})]
    for i, (level, expect) in enumerate(cases):
        d = make_run(tmp_path, f"r{i}", "passed")
        assert runs.apply_keep(d, "passed", {"passed": level}) == level
        assert set(names(d)) == expect
    d = make_run(tmp_path, "unnamed", "error")
    assert runs.apply_keep(d, "error", {"passed": "result"}) == "debug"
    assert "debug" in names(d)
    with pytest.raises(ValueError):
        runs.apply_keep(d, "error", {"error": "everything"})


def test_a_judges_failure_keeps_what_a_failed_run_keeps(tmp_path):
    """G52: SSv2 passed, the incident judge failed; the run kept no logs to look into."""
    keep = {"passed": "result", "failed": "full", "error": "standard"}
    d = make_run(tmp_path, "judged", "passed")
    assert runs.apply_keep(d, "passed", keep, judged=["passed", "failed"]) == "full"
    assert "logs" in names(d) and "record" in names(d)
    d = make_run(tmp_path, "judge_error", "passed")
    assert runs.apply_keep(d, "passed", keep, judged=["error"]) == "standard"
    assert "logs" in names(d) and "record" not in names(d)
    d = make_run(tmp_path, "all_good", "passed")
    assert runs.apply_keep(d, "passed", keep, judged=["passed"]) == "result"
    d = make_run(tmp_path, "unnamed", "passed")
    assert runs.apply_keep(d, "passed", {"passed": "result"}, judged=["failed"]) == "debug"


def test_summary(root, tmp_path):
    (root / "2026-11-14T10-00-00_cut_in_r0" / "junit.xml").unlink()  # synthesised from verdict
    code, out = call("summary", "--root", str(root))
    assert code == 0
    rows = list(csv.DictReader(open(root / "summary.csv")))
    assert [r["verdict"] for r in rows] == ["passed", "failed", "error"]
    assert list(rows[0]) == list(runs.SUMMARY_FIELDS)
    assert rows[1]["reason"] == "SimulationFailure: hit" and rows[1]["seed"] == "1001"
    assert rows[1]["duration"] == "24.0" and rows[1]["repetition"] == "1"
    j = ET.parse(root / "junit.xml").getroot()
    assert (j.get("tests"), j.get("failures"), j.get("errors")) == ("3", "1", "1")
    assert [s.get("name") for s in j] == [r["run_id"] for r in rows]
    # an explicit directory
    other = tmp_path / "suite"
    make_run(other, "x_gap6_r0", "passed", params={"gap_m": 6, "policy": "yield"})
    assert call("summary", str(other))[0] == 0
    rows = list(csv.DictReader(open(other / "summary.csv")))
    assert rows[0]["params"] == "gap_m=6;policy=yield"
    assert call("summary", str(tmp_path / "nothing"))[0] == 3


def test_why_names_the_bridges_refusal_not_ssv2s_error_nor_play_launchs_note():
    """G67, on 241's run of 2026-11-16 (host paths replaced): SSv2's reason is a SemanticError
    ("Spawn entity ... resulted in failure"); the cause is the bridge's "Cannot spawn ..." line
    after it in frame 2; before it come play_launch's cap_sys_ptrace notes and SSv2's own WARNs."""
    from pathlib import Path
    root = Path(__file__).resolve().parent / "data" / "runs"
    code, out = call("why", "--root", str(root), "2026-11-16T18-27-38_autopilot_r0")
    assert code == 0
    assert "reason   SemanticError:" in out and 'Spawn entity "npc_autopilot"' in out
    assert ("first    WARN in bridge/scenario_world_bridge: spawn_vehicle_entity{frame=2}: "
            "scenario_world_bridge::coordinator: Returning error: Cannot spawn 'npc_autopilot' "
            "(simulator_autopilot): the isaac world server does not offer AUTOPILOT") in out
    assert "cap_sys_ptrace" not in out
