"""Unit tests for scripts/gap_campaign.py, the gap campaign's run outcome and store (roadmap
025 step 4, the offline half). No ROS, no simulator:

    python3 -m pytest src/swb_launch/test/test_gap_campaign.py
"""

import importlib.machinery
import importlib.util
import json
import math
import random
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


gc = _load("gap_campaign")
gg = gc.gg
gs = gc.gs

RECORDED = Path("/mnt/wd/aeon/csb-044/runs")


def junit(tag=None, kind="", message="", extra=""):
    body = f'<{tag} type="{kind}" message="{message}" />' if tag else ""
    return ('<?xml version="1.0"?><testsuites><testsuite name="s">'
            f'<testcase name="town01_x">{body}</testcase>{extra}</testsuite></testsuites>')


def oracle(signature=(), distance=None):
    margins = {} if distance is None else {"min_disagreement_distance": distance}
    return {"signature": list(signature), "margins": margins, "verdict": "pass"}


class Classify(unittest.TestCase):
    def test_a_pass_is_valid_with_closeness_from_the_distance(self):
        r = gc.classify(junit(), oracle(distance=0.5))
        self.assertEqual(r.kind, "valid")
        self.assertEqual(r.verdict, "pass")
        self.assertAlmostEqual(r.outcome.closeness, math.exp(-0.5))
        self.assertIsNone(r.outcome.signature)

    def test_a_signature_is_a_disagreement(self):
        r = gc.classify(junit(), oracle(["C1:vehicle:ssv2", "C2:static:carla"], 1.2))
        self.assertEqual(r.outcome.signature, "C1:vehicle:ssv2+C2:static:carla")
        self.assertEqual(r.outcome.closeness, 1.0)

    def test_no_comparable_frame_is_closeness_zero(self):
        self.assertEqual(gc.classify(junit(), oracle()).outcome.closeness, 0.0)

    def test_a_scenario_failure_is_still_a_valid_outcome(self):
        r = gc.classify(junit("failure", "SimulationFailure",
                              "CustomCommandAction typed exitFailure was triggered"),
                        oracle(distance=3.0))
        self.assertEqual((r.kind, r.verdict), ("valid", "failure"))

    def test_the_simulator_not_answering_is_infrastructure(self):
        for kind, msg in (("SimulationError", "zmq_multi_client.cpp:103: No response from the simulator"),
                          ("SimulationError", "The simulator reported a failed frame (UpdateEntityStatus)"),
                          ("AutowareError", "Autoware did not engage")):
            r = gc.classify(junit("error", kind, msg), oracle(distance=1.0))
            self.assertEqual(r.kind, "infrastructure", msg)
            self.assertIsNone(r.outcome)
        self.assertEqual(gc.classify(None, None).kind, "infrastructure")
        self.assertEqual(gc.classify(junit(), oracle(), bridge_exit=75).kind, "infrastructure")

    def test_a_scenario_ssv2_refuses_is_invalid(self):
        for kind in ("SyntaxError", "SemanticError", "SpecificationViolation"):
            r = gc.classify(junit("error", kind, "bad scenario"), None)
            self.assertEqual(r.kind, "invalid", kind)
            self.assertFalse(r.outcome.valid)

    def test_an_entity_out_of_ssv2s_range_is_its_own_signature(self):
        msg = "entity_base.cpp:712: Entity: ego - current acceleration (which is 35.2813) is out of range"
        r = gc.classify(junit("error", "InternalError", msg), oracle(distance=2.0))
        self.assertEqual(r.kind, "valid")
        self.assertEqual(r.outcome.signature, "S:entity_limit")

    def test_run_suites_contact_testcase_is_not_ssv2s_verdict(self):
        extra = '<testcase name="carla_contact"><failure type="x" message="hit" /></testcase>'
        self.assertEqual(gc.classify(junit(extra=extra), oracle(distance=1.0)).verdict, "pass")

    @unittest.skipUnless((RECORDED / "w-i2-2/oracle.json").exists(), "no recorded runs here")
    def test_recorded_runs(self):
        bad = gc.classify_dir(RECORDED / "v-i2-1")
        self.assertEqual(bad.kind, "infrastructure")
        good = gc.classify_dir(RECORDED / "w-i2-2")
        self.assertEqual((good.kind, good.verdict), ("valid", "pass"))
        # an oracle older than min_disagreement_distance: no gradient, closeness 0
        self.assertNotIn("min_disagreement_distance", good.margins)
        self.assertEqual(good.outcome.closeness, 0.0)


class Store(unittest.TestCase):
    def test_runs_and_search_survive_a_restart(self):
        with tempfile.TemporaryDirectory() as d:
            st = gc.Store.create(Path(d) / "c", {"sim": "carla", "families": ["cyclist"],
                                                 "budget": 50, "seed": 4})
            s = st.search()
            g = s.ask()
            st.begin(g, s.pending[g.id])
            r = gc.classify(junit(), oracle(distance=0.7))
            s.tell(g, r.outcome)
            st.finish(g, r, run_dir="/tmp/run1")
            st.save_search(s)
            # killed mid-run: asked but never finished
            g2 = s.ask()
            st.begin(g2, s.pending[g2.id])
            again = gc.Store.open(Path(d) / "c")
            self.assertEqual(again.campaign["sim"], "carla")
            self.assertEqual([x["genome"]["id"] for x in again.runs()], [g.id])
            self.assertEqual(again.unfinished().id, g2.id)
            # the restored search asks the unfinished genome again
            self.assertEqual(again.search().ask(), g2)

    def test_infrastructure_failures_are_logged_not_told(self):
        with tempfile.TemporaryDirectory() as d:
            st = gc.Store.create(Path(d) / "c", {"sim": "carla", "families": ["cyclist"],
                                                 "budget": 50, "seed": 0})
            g = gg.default("cyclist")
            st.finish(g, gc.classify(None, None), run_dir="/tmp/r")
            line = st.runs()[0]
            self.assertEqual(line["kind"], "infrastructure")
            self.assertIsNone(line["outcome"])

    def test_a_finding_keeps_its_scenario_and_runs(self):
        with tempfile.TemporaryDirectory() as d:
            st = gc.Store.create(Path(d) / "c", {"sim": "carla", "families": ["lead_brake"],
                                                 "budget": 50, "seed": 0})
            g = gg.default("lead_brake")
            st.finish(g, gc.classify(junit(), oracle(["C1:vehicle:ssv2"], 0.0)), run_dir="/tmp/r")
            f = Path(d) / "c/findings/C1:vehicle:ssv2"
            self.assertTrue((f / f"{g.id}.xosc").exists())
            self.assertEqual(json.loads((f / f"{g.id}.json").read_text())["genome"]["id"], g.id)


if __name__ == "__main__":
    unittest.main()
