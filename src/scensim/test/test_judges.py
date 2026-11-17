import json
import sys

from scensim import budget, judges


def judge(tmp_path, body, name="j", **kw):
    script = tmp_path / f"{name}.py"
    script.write_text(body)
    return {"name": name, "run": [sys.executable, str(script)], **kw}


def run_dir(tmp_path):
    d = tmp_path / "run"
    (d / "logs").mkdir(parents=True)
    (d / "result.json").write_text(json.dumps({"verdict": "passed"}))
    (d / "timeline.jsonl").write_text("")
    return d


def test_problems():
    assert judges.problems(None) == []
    assert judges.problems({}) == [("judges", "is not a list")]
    bad = judges.problems([{"name": "a/b", "run": [], "on": ["maybe"], "timeout": 0, "x": 1},
                           {"name": "ok", "run": ["true"]}, {"name": "ok", "run": ["true"]}])
    msgs = " | ".join(m for _, m in bad)
    for part in ("unknown key x", "name: a plain", "run: a non-empty", "on: a list",
                 "timeout: a positive", "name: ok twice"):
        assert part in msgs


def test_judge_output_recorded(tmp_path):
    d = run_dir(tmp_path)
    j = judge(tmp_path, "import json, sys, os\n"
              "r = json.load(open(sys.argv[1] + '/result.json'))\n"
              "assert os.environ['SCENSIM_RUN_DIR'] == sys.argv[1]\n"
              "print(json.dumps({'jerk_max': 1.5, 'seen': r['verdict']}))\n", version=3)
    rec = judges.run_one(j, d, tmp_path)
    assert rec["verdict"] == "passed" and rec["version"] == "3" and rec["seen"] == "passed"
    assert json.loads((d / "judges" / "j.json").read_text())["jerk_max"] == 1.5


def test_judge_verdicts(tmp_path):
    d = run_dir(tmp_path)
    fail = judge(tmp_path, "import sys; print('{\"why\": 1}'); sys.exit(1)", "f")
    assert judges.run_one(fail, d, tmp_path)["verdict"] == "failed"
    own = judge(tmp_path, "print('{\"verdict\": \"failed\", \"message\": \"too fast\"}')", "o")
    r = judges.run_one(own, d, tmp_path)
    assert r["verdict"] == "failed" and r["message"] == "too fast"
    crash = judge(tmp_path, "import sys; sys.stderr.write('boom\\n'); sys.exit(4)", "c")
    r = judges.run_one(crash, d, tmp_path)
    assert r["verdict"] == "error" and "exit 4: boom" in r["message"]
    assert (d / "logs" / "judge" / "c.log").read_text() == "boom\n"
    plain = judge(tmp_path, "import sys; print('one contact'); sys.exit(1)", "s")
    r = judges.run_one(plain, d, tmp_path)
    assert r["verdict"] == "failed" and r["message"] == "one contact"
    (d / "resolved.yaml").write_text("scenario: {path: /p/s.xosc}\n")
    sub = judge(tmp_path, "import sys, json; print(json.dumps({'a': sys.argv[1:]}))", "u")
    sub["run"] += ["{run_dir}/junit.xml", "--scenario", "{scenario}"]
    assert judges.run_one(sub, d, tmp_path)["a"] == [f"{d}/junit.xml", "--scenario", "/p/s.xosc"]
    slow = judge(tmp_path, "import time; time.sleep(5)", "t", timeout=0.2)
    assert "did not end" in judges.run_one(slow, d, tmp_path)["message"]
    assert judges.summary(d) == {"c": "error", "f": "failed", "o": "failed", "s": "failed",
                                 "t": "error", "u": "passed"}


def test_run_all_filters_by_verdict_and_appends_timeline(tmp_path):
    d = run_dir(tmp_path)
    a = judge(tmp_path, "print('{}')", "a")
    b = judge(tmp_path, "print('{}')", "b", on=["error"])
    recs = judges.run_all([a, b], d, tmp_path, "passed")
    assert [r["name"] for r in recs] == ["a"]
    ev = [json.loads(x) for x in (d / "timeline.jsonl").read_text().splitlines()]
    assert ev[0]["group"] == "judge" and ev[0]["judge"] == "a"


def test_budget_sizes():
    assert budget.parse_size("2GB") == 2 * 1024 ** 3
    assert budget.parse_size("1.5 mb") == int(1.5 * 1024 ** 2)
    assert budget.parse_size(10) == 10
    for bad in ("lots", "-1", True, "2PB"):
        try:
            budget.parse_size(bad)
        except ValueError:
            continue
        raise AssertionError(bad)
    assert budget.problems({"per_run": "x", "max": 1}) == [
        "unknown key max (use only per_run, total)", "per_run: 'x' is not a size (e.g. 500MB, 2GB)"]


def test_budget_check(tmp_path):
    b = {"per_run": "1GB", "total": "10GB"}
    assert budget.check(None, 5, tmp_path) == ""
    assert budget.check(b, 5, tmp_path, free=100 * 1024 ** 3, used=0) == ""
    assert "is free under" in budget.check(b, 5, tmp_path, free=2 * 1024 ** 3, used=0)
    assert "of logging.budget.total" in budget.check(b, 5, tmp_path, free=10 ** 13,
                                                     used=6 * 1024 ** 3)
    (tmp_path / "r1").mkdir()
    (tmp_path / "r1" / "f").write_bytes(b"x" * 1000)
    (tmp_path / "link").symlink_to(tmp_path / "r1")
    assert budget.dir_size(tmp_path) == 1000
