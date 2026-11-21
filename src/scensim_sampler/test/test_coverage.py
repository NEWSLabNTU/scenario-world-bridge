"""The coverage report: three layers, the self-normalised rate, exposure, seeds not applied."""
import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _files import EXAMPLES  # noqa: E402
from scensim_sampler import Sampler  # noqa: E402
from scensim_sampler import coverage as cov  # noqa: E402
from scensim_sampler.cli import main  # noqa: E402


def rec(category="R2.walker_crossing.dart_out", cell=None, proposal=None, log_weight=0.0):
    return {"category": category, "cell": cell or {"surface": "dry"},
            "proposal": {"id": proposal, "sha256": "0" * 64} if proposal else None,
            "base": {"id": "m.yaml", "sha256": "1" * 64} if proposal else None,
            "log_weight": log_weight, "weight": math.exp(log_weight)}


def out(failed=False, contact=False, error=False, unseeded=False, run="r"):
    return {"failed": failed, "contact": contact, "error": error,
            "seeds_not_applied": unseeded, "run": run}


class Catalogue(unittest.TestCase):
    def write(self, text):
        f = Path(tempfile.mkdtemp()) / "c.yaml"
        f.write_text(text)
        return f

    def test_categories_cells_t_and_exposure(self):
        c = cov.load_catalogue(self.write("""catalogue: 1
categories: [R2.walker_crossing.dart_out, R1.rear_end.lead_brakes]
cells: {R2.walker_crossing.dart_out: {surface: [dry, wet], light: [day, night]}}
t: 2
exposure: {R2.walker_crossing.dart_out: 3, R1.rear_end.lead_brakes: 7}
"""))
        self.assertEqual(c["t"], 2)
        self.assertEqual(c["exposure"]["R1.rear_end.lead_brakes"], 7)

    def test_a_bad_catalogue_is_refused_by_name(self):
        for text, words in (("categories: [R2.a.b]", "catalogue: 1"),
                            ("catalogue: 1\ncategories: [R9.a.b]", "not R<family>"),
                            ("catalogue: 1\ncategories: [R2.a.b]\nexposure: {R1.a.b: 1}",
                             "exposure names R1.a.b"),
                            ("catalogue: 1\ncategories: [R2.a.b]\ncells: {R2.a.b: {x: []}}",
                             "each non-empty")):
            with self.assertRaisesRegex(cov.CoverageError, words, msg=text):
                cov.load_catalogue(self.write(text))


class Layers(unittest.TestCase):
    CAT = {"categories": ["R2.walker_crossing.dart_out", "R1.rear_end.lead_brakes"],
           "cells": {"R2.walker_crossing.dart_out": {"surface": ["dry", "wet"],
                                                     "light": ["day", "night"]}},
           "t": 2, "exposure": None, "path": "c.yaml"}

    def test_category_coverage_names_the_uncovered_and_the_families_without_one(self):
        items = [(rec(), out(run="a")), (rec("R5.slip.wet_curve"), out(run="b"))]
        rep = cov.report(items, self.CAT)
        self.assertEqual(rep["layer1"]["uncovered"], ["R1.rear_end.lead_brakes"])
        self.assertEqual(rep["layer1"]["outside_catalogue"], ["R5.slip.wet_curve"])
        fam = rep["layer1"]["families_without_a_category"]
        self.assertTrue(any(f.startswith("R3 ") for f in fam))
        self.assertFalse(any(f.startswith("R2 ") for f in fam))

    def test_t_wise_coverage_lists_the_missing_tuples(self):
        items = [(rec(cell={"surface": "dry", "light": "day"}), out()),
                 (rec(cell={"surface": "wet", "light": "day"}), out()),
                 (rec(cell={"surface": "icy", "light": "day"}), out(run="odd"))]
        rep = cov.report(items, self.CAT)
        l2 = next(x for x in rep["layer2"] if x["category"] == "R2.walker_crossing.dart_out")
        self.assertEqual((l2["t"], l2["tuples"], l2["covered"]), (2, 4, 2))
        self.assertEqual(l2["missing"], [{"light": "night", "surface": "dry"},
                                         {"light": "night", "surface": "wet"}])
        self.assertEqual(l2["outside_levels"], [{"run": "odd", "outside": ["surface=icy"]}])
        t1 = cov.report(items, self.CAT, t=1)
        l2 = next(x for x in t1["layer2"] if x["category"] == "R2.walker_crossing.dart_out")
        self.assertEqual((l2["tuples"], l2["covered"]), (4, 3))   # night is missing

    def test_unweighted_rate_and_wilson_interval_and_what_counts_as_failure(self):
        items = ([(rec(), out(failed=True, run=f"f{i}")) for i in range(3)]
                 + [(rec(), out(contact=True, run=f"c{i}")) for i in range(2)]
                 + [(rec(), out(run=f"p{i}")) for i in range(5)]
                 + [(rec(), out(error=True, run="e"))])
        r = cov.report(items, self.CAT)["layer3"][0]
        self.assertEqual((r["runs"], r["n"], r["failures"], r["errors"]), (11, 10, 3, 1))
        self.assertAlmostEqual(r["rate"], 0.3)
        lo, hi = r["interval"]
        self.assertAlmostEqual(lo, 0.1078, 3)   # Wilson 3/10
        self.assertAlmostEqual(hi, 0.6032, 3)
        self.assertEqual(cov.report(items, self.CAT, failure="either")["layer3"][0]["failures"],
                         5)
        zero = cov.report([(rec(), out(run=f"p{i}")) for i in range(30)], self.CAT)
        self.assertAlmostEqual(zero["layer3"][0]["interval"][1], 0.1135, 3)  # ~ rule of 3

    def test_seeds_not_applied_are_listed_and_kept_out_of_the_rate(self):
        items = [(rec(), out(failed=True, run="a")), (rec(), out(unseeded=True, run="b"))]
        r = cov.report(items, self.CAT)["layer3"][0]
        self.assertEqual((r["n"], r["failures"], r["seeds_not_applied"]), (1, 1, ["b"]))
        self.assertIn("seeds not applied (not in the rates): b",
                      cov.format_report(cov.report(items, self.CAT)))

    def test_replays_are_left_out_and_a_category_without_attributes_has_no_tuples(self):
        items = [(rec(), out(failed=True, run="a")),
                 (rec(), dict(out(failed=True, run="a_replay"), replay=True))]
        rep = cov.report(items, self.CAT)
        self.assertEqual((rep["records"], rep["replays"]), (1, ["a_replay"]))
        self.assertEqual(rep["layer3"][0]["n"], 1)
        l2 = next(x for x in rep["layer2"] if x["category"] == "R1.rear_end.lead_brakes")
        self.assertEqual((l2["tuples"], l2["missing"], l2["share"]), (0, [], None))

    def test_mixed_weights_or_proposals_are_refused_not_blended(self):
        mixed = [(rec(), out()), (rec(proposal="q", log_weight=-1.0), out())]
        self.assertIn("not blended", cov.report(mixed, self.CAT)["layer3"][0]["refused"])
        two = [(rec(proposal="q1"), out()), (rec(proposal="q2"), out())]
        self.assertIn("more than one proposal", cov.report(two, self.CAT)["layer3"][0]["refused"])

    def test_the_self_normalised_rate_recovers_the_bases_from_a_proposal(self):
        """Base: x ~ N(0, 1), failure x > 2 (p = 0.02275). Proposal: N(2, 1). Weights from the
        log densities; the estimate's interval covers the truth, far tighter than plain
        sampling would be at this n."""
        rng = np.random.default_rng(5)
        x = rng.normal(2.0, 1.0, 4000)
        lw = (-0.5 * x ** 2) - (-0.5 * (x - 2.0) ** 2)
        items = [(rec(proposal="q", log_weight=float(w)), out(failed=bool(v > 2), run=str(i)))
                 for i, (v, w) in enumerate(zip(x, lw))]
        r = cov.report(items, self.CAT)["layer3"][0]
        truth = 0.5 * math.erfc(2 / math.sqrt(2))
        self.assertTrue(r["weighted"] and r["approximate"])
        self.assertAlmostEqual(r["rate"], truth, delta=0.004)
        self.assertLess(r["interval"][0], truth)
        self.assertGreater(r["interval"][1], truth)
        self.assertTrue(r["interval_method"].startswith("delta (normal; approximate, ~90 %"))
        self.assertIn("~90 % measured coverage at nominal 95 %",
                      cov.format_report(cov.report(items, self.CAT)))
        # the interval is the estimator's spread, not Kish's (8x too wide here; 243)
        self.assertLess(r["interval"][1] - r["interval"][0], 0.02)
        # tiny weights: log space, no underflow
        tiny = [(rec(proposal="q", log_weight=-900.0 + w), o) for (rr, o), w in
                zip(items, lw)]
        self.assertAlmostEqual(cov.report(tiny, self.CAT)["layer3"][0]["rate"], r["rate"], 12)

    def test_the_weighted_intervals_coverage_is_as_the_spec_says(self):
        """200 repetitions of 1000 draws: the delta interval covers the N(0,1) tail about 90 %
        of the time (nominal 95 %; the spec says it can under-cover), and is about as wide as
        the estimator's real spread, where Kish's n_eff made it 8x wider."""
        rng = np.random.default_rng(11)
        truth = 0.5 * math.erfc(2 / math.sqrt(2))
        hit, widths, est = 0, [], []
        for _ in range(200):
            x = rng.normal(2.0, 1.0, 1000)
            lw = (-0.5 * x ** 2) - (-0.5 * (x - 2.0) ** 2)
            items = [(rec(proposal="q", log_weight=float(w)), out(failed=bool(v > 2)))
                     for v, w in zip(x, lw)]
            r = cov.report(items, self.CAT)["layer3"][0]
            lo, hi = r["interval"]
            hit += lo <= truth <= hi
            widths.append(hi - lo)
            est.append(r["rate"])
        # the label says ~90 %: a change that worsens it below 0.85 fails here (seeded: 0.94)
        self.assertGreater(hit / 200, 0.85)
        self.assertLess(np.mean(widths), 6 * np.std(est))   # ~2 x 1.96 sd, not 8x that

    def test_no_failures_under_weights_gives_a_conservative_upper_bound(self):
        items = [(rec(proposal="q", log_weight=-float(i % 3)), out()) for i in range(50)]
        r = cov.report(items, self.CAT)["layer3"][0]
        self.assertEqual(r["rate"], 0.0)
        self.assertAlmostEqual(r["interval"][0], 0.0, 12)
        self.assertGreater(r["interval"][1], 0.0)
        self.assertIn("upper bound", r["interval_method"])

    def test_an_overall_rate_only_with_exposure_and_refused_when_a_category_has_none(self):
        items = ([(rec(), out(failed=i < 2, run=f"a{i}")) for i in range(10)]
                 + [(rec("R1.rear_end.lead_brakes"), out(failed=i < 1, run=f"b{i}"))
                    for i in range(10)])
        self.assertNotIn("overall", cov.report(items, self.CAT))
        cat = dict(self.CAT, exposure={"R2.walker_crossing.dart_out": 1,
                                       "R1.rear_end.lead_brakes": 3})
        o = cov.report(items, cat)["overall"]
        self.assertAlmostEqual(o["rate"], (1 * 0.2 + 3 * 0.1) / 4)
        self.assertIn("study's input", o["note"])
        o = cov.report(items[:10], cat)["overall"]
        self.assertIn("no rate for R1.rear_end.lead_brakes (no runs)", o["refused"])


class Inputs(unittest.TestCase):
    def test_jsonl_and_the_command_line(self):
        b = Sampler.from_file(EXAMPLES / "walker_dartout_dist.xosc").draw(n=6)
        d = Path(tempfile.mkdtemp())
        f = d / "o.jsonl"
        f.write_text("".join(json.dumps({"record": r, "outcome": {"failed": i % 3 == 0}}) + "\n"
                             for i, r in enumerate(b.records)))
        import io
        from contextlib import redirect_stdout
        s = io.StringIO()
        with redirect_stdout(s):
            self.assertEqual(main(["coverage", str(f), "--json", str(d / "c.json")]), 0)
        self.assertIn("3. criticality", s.getvalue())
        rep = json.loads((d / "c.json").read_text())
        self.assertEqual(rep["schema"], "scensim.coverage/1")
        self.assertEqual(sum(x["failures"] for x in rep["layer3"]), 2)

    def test_scensim_run_directories(self):
        d = Path(tempfile.mkdtemp())
        r = Sampler.from_file(EXAMPLES / "walker_dartout_dist.xosc").draw(n=2).records
        for i, (verdict, notes) in enumerate((("failed", []),
                                              ("passed", ["pilot ego: a stack pilot, ..."]))):
            rd = d / f"run{i}"
            (rd / "contacts").mkdir(parents=True)
            (rd / "sample.json").write_text(json.dumps(r[i]))
            (rd / "result.json").write_text(json.dumps(
                {"verdict": verdict, "sample": {"unbound": notes}}))
        (d / "plain").mkdir()
        (d / "plain" / "result.json").write_text('{"verdict": "passed"}')   # not sampled
        items = cov.outcomes_from_runs([d])
        self.assertEqual([o["run"] for _, o in items], ["run0", "run1"])
        self.assertEqual([(o["failed"], o["seeds_not_applied"]) for _, o in items],
                         [(True, False), (False, True)])


if __name__ == "__main__":
    unittest.main()
