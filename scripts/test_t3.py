"""scripts/t3.py: a suite's results against its reference (roadmap 077, T3)."""
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location("t3", Path(__file__).with_name("t3.py"))
t3 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t3)

HEAD = "run_id,scenario,params,repetition,seed,verdict,reason,duration,judges,path\n"


def row(scen, verdict, judges="physical_verdict=passed", dur="100", rep="0", reason=""):
    return f"x,/a/{scen},,{rep},,{verdict},{reason},{dur},{judges},/p\n"


def test_a_retry_counts_by_its_last_try(tmp_path):
    p = tmp_path / "summary.csv"
    p.write_text(HEAD + row("i7.xosc", "error") + row("i7.xosc", "failed"))
    rows = t3.read_summary(p)
    assert [(r["scenario"], r["verdict"]) for r in rows] == [("i7.xosc", "failed")]


def test_a_changed_verdict_or_judge_is_a_problem_a_slow_run_a_note(tmp_path):
    ref, got = tmp_path / "ref.csv", tmp_path / "got.csv"
    ref.write_text(HEAD + row("a.xosc", "passed") + row("b.xosc", "passed") + row("c.xosc", "passed"))
    got.write_text(HEAD + row("a.xosc", "failed", reason="contact")
                   + row("b.xosc", "passed", judges="physical_verdict=failed")
                   + row("c.xosc", "passed", dur="200") + row("d.xosc", "passed"))
    problems, notes = t3.compare_rows(t3.read_summary(ref), t3.read_summary(got))
    assert problems == ["a.xosc r0: failed, the reference passed (contact)",
                        "b.xosc r0: judge physical_verdict failed, the reference passed"]
    assert notes == ["c.xosc r0: 200 s, the reference 100 s",
                     "d.xosc r0: not in the reference (passed)"]


def test_a_missing_run_is_a_problem(tmp_path):
    ref, got = tmp_path / "ref.csv", tmp_path / "got.csv"
    ref.write_text(HEAD + row("a.xosc", "passed") + row("a.xosc", "passed", rep="1"))
    got.write_text(HEAD + row("a.xosc", "passed"))
    problems, _ = t3.compare_rows(t3.read_summary(ref), t3.read_summary(got))
    assert problems == ["a.xosc r1: no result (the reference has passed)"]


def test_the_manifest_names_existing_suites():
    doc = t3.load()
    for e in doc["suites"]:
        assert (t3.ROOT / e["project"] / "suites" / f"{e['suite']}.yaml").is_file(), e["key"]
    assert len({e["key"] for e in doc["suites"]}) == len(doc["suites"])


def test_parts_cover_every_suite_once_keeping_projects_whole():
    doc = t3.load()
    parts = [t3.selected(doc, [], f"{i}/3") for i in (1, 2, 3)]
    keys = [e["key"] for p in parts for e in p]
    assert sorted(keys) == sorted(e["key"] for e in doc["suites"])
    owners = {}
    for i, p in enumerate(parts):
        for e in p:
            assert owners.setdefault(e["project"], i) == i
    assert all(parts)


def test_a_busy_gpu_is_one_at_half_its_memory_or_load():
    quiet = [{"gpu": 0, "used_mib": 3000, "total_mib": 32000, "util_pct": 10}]
    assert not t3.busy(quiet)
    assert t3.busy([dict(quiet[0], util_pct=97)])
    assert t3.busy([dict(quiet[0], used_mib=25000)])
    assert t3.busy(quiet, load=330.0, cores=48)
    assert not t3.busy(quiet, load=11.0, cores=48)


def test_list_runs_on_the_shipped_manifest(capsys):
    """main() end to end on scenarios/t3.yaml (a local once shadowed load(): list crashed)."""
    assert t3.main(["list", "--part", "2/3"]) == 0
    assert "isaac_road_physics/road" in capsys.readouterr().out
