"""Unit tests for scripts/gap_live.py, the gap campaign's live loop (roadmap 025 steps 4, 6
and 7). No ROS, no simulator: the run is a fake over a synthetic gap.

    python3 -m pytest src/scensim_launch/test/test_gap_live.py
"""

import importlib.machinery
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parents[3] / "scripts"
sys.path.insert(0, str(_SCRIPTS))


def _load(name):
    loader = importlib.machinery.SourceFileLoader(name, str(_SCRIPTS / f"{name}.py"))
    spec = importlib.util.spec_from_loader(name, loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


gl = _load("gap_live")
gc, gg = gl.gc, gl.gg

TARGET = "C1:vehicle:ssv2"


def junit(fail=False):
    body = '<failure type="SimulationFailure" message="collision" />' if fail else ""
    return ('<?xml version="1.0"?><testsuites><testsuite name="s">'
            f'<testcase name="x">{body}</testcase></testsuite></testsuites>')


def run_dir(root, name, fail, signature, dist=None):
    d = Path(root) / name
    (d / "gap_trace").mkdir(parents=True)
    (d / "junit.xml").write_text(junit(fail))
    margins = {} if dist is None else {"min_disagreement_distance": dist}
    (d / "gap_trace" / "t.findings.json").write_text(json.dumps(
        {"signature": signature, "margins": margins}))
    return d


class FakeRun:
    """A synthetic gap on lead_brake: the declared lead longer than 9 m signs TARGET; one in
    `flaky` runs is infrastructure."""

    def __init__(self, flaky=0):
        self.n, self.flaky, self.last_run_dir = 0, flaky, None

    def __call__(self, genome):
        self.n += 1
        if self.flaky and self.n % self.flaky == 0:
            return gc.RunResult("infrastructure", None, "none", "fake", {})
        v = genome.values
        if v.get("lead_body") == "box" and v.get("lead_length", 0) > 9.0:
            return gc.RunResult("valid", gc.gs.Outcome(True, TARGET, 1.0), "failure", "", {})
        close = max(0.0, (v.get("lead_length", 4.79) - 4.0) / 6.0) if v.get("lead_body") == "box" else 0.0
        return gc.RunResult("valid", gc.gs.Outcome(True, None, close), "pass", "", {})


class TestClassifyRun(unittest.TestCase):
    def test_a_run_with_a_signature(self):
        with tempfile.TemporaryDirectory() as t:
            r = gl.classify_run(run_dir(t, "a", True, [TARGET]), "failed")
            self.assertEqual((r.kind, r.outcome.signature, r.outcome.closeness), ("valid", TARGET, 1.0))

    def test_an_agreeing_run_has_closeness_from_its_margin(self):
        with tempfile.TemporaryDirectory() as t:
            r = gl.classify_run(run_dir(t, "a", False, [], dist=0.5), "passed")
            self.assertEqual(r.kind, "valid")
            self.assertIsNone(r.outcome.signature)
            self.assertTrue(0.0 < r.outcome.closeness < 1.0)

    def test_error_and_no_findings_are_infrastructure_refused_is_invalid(self):
        with tempfile.TemporaryDirectory() as t:
            d = run_dir(t, "a", False, [])
            self.assertEqual(gl.classify_run(d, "error").kind, "infrastructure")
            self.assertEqual(gl.classify_run(None, "error").kind, "infrastructure")
            (d / "gap_trace" / "t.findings.json").unlink()
            self.assertEqual(gl.classify_run(d, "passed").kind, "infrastructure")
            self.assertEqual(gl.classify_run(d, "refused").kind, "invalid")

    def test_the_verdict_line(self):
        out = "map Town01\npassed   2026-11-16T12-00-00_x_r0  /logs/runs/2026-11-16T12-00-00_x_r0  judges: gap=passed\n"
        m = gl.VERDICT.search(out)
        self.assertEqual((m.group(1), m.group(3)), ("passed", "/logs/runs/2026-11-16T12-00-00_x_r0"))


def store(t, budget=200, seed=0):
    return gc.Store.create(Path(t) / "c", {"sim": "carla", "families": ["lead_brake"],
                                            "budget": budget, "seed": seed})


class TestLoop(unittest.TestCase):
    def test_rediscovers_the_bucket_and_stops(self):
        with tempfile.TemporaryDirectory() as t:
            s = store(t)
            n, found = gl.loop(s, FakeRun(), until=TARGET, out=lambda *_: None)
            self.assertIsNotNone(found)
            self.assertEqual(found["signature"], TARGET)
            self.assertEqual(len(s.runs()), n)
            self.assertTrue((s.root / "findings" / TARGET).is_dir())

    def test_infrastructure_is_run_again_and_never_told(self):
        with tempfile.TemporaryDirectory() as t:
            s = store(t, budget=6)
            fake = FakeRun(flaky=2)
            n, _ = gl.loop(s, fake, out=lambda *_: None)
            self.assertEqual(n, 6)
            self.assertTrue(all(r["kind"] != "infrastructure" for r in s.runs()))
            self.assertGreater(fake.n, 6)

    def test_stops_when_the_pipeline_keeps_failing(self):
        with tempfile.TemporaryDirectory() as t:
            s = store(t, budget=50)
            broken = lambda g: gc.RunResult("infrastructure", None, "none", "stack down", {})
            n, _ = gl.loop(s, broken, out=lambda *_: None)
            self.assertEqual(n, gl.MAX_INFRASTRUCTURE_IN_A_ROW)

    def test_resumes_the_genome_a_killed_loop_left(self):
        with tempfile.TemporaryDirectory() as t:
            s = store(t, budget=5)

            class Killed(Exception):
                pass

            calls = []

            def dies_on_third(genome):
                calls.append(genome.id)
                if len(calls) == 3:
                    raise Killed()
                return FakeRun()(genome)

            with self.assertRaises(Killed):
                gl.loop(s, dies_on_third, out=lambda *_: None)
            left = s.unfinished()
            self.assertIsNotNone(left)
            self.assertEqual(left.id, calls[2])
            again = []

            def records(genome):
                again.append(genome.id)
                return FakeRun()(genome)

            n, _ = gl.loop(gc.Store.open(s.root), records, out=lambda *_: None)
            self.assertEqual(again[0], calls[2])  # the unfinished genome first
            self.assertEqual(len(s.runs()), 5)  # 2 before, then the resumed and the rest
            self.assertIsNone(s.unfinished())


class TestExploit(unittest.TestCase):
    def test_the_positive_control(self):
        g = gg.Genome("lead_brake", dict(gg.default("lead_brake").values, lead_length=12.0))
        rep = gl.exploit(g, FakeRun(), tol=0.1)
        self.assertTrue(rep["confirm"]["confirmed"])
        self.assertEqual(rep["signature"], TARGET)
        self.assertIn("lead_length", rep["relevant"]["genes"])
        b = next(x for x in rep["boundaries"] if x["gene"] == "lead_length")
        self.assertAlmostEqual(b["estimate"], 9.0, delta=0.1)
        self.assertEqual(rep["shrunk"]["genome"]["values"]["lead_length"], 12.0)


if __name__ == "__main__":
    unittest.main()
