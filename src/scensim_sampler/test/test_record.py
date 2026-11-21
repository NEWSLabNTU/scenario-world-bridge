"""The sample record's checker (and, where jsonschema is installed, the schema agreeing)."""
import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _files import EXAMPLES  # noqa: E402
from scensim_sampler import Sampler, check, record_id  # noqa: E402
from scensim_sampler.record import SCHEMA  # noqa: E402
from scensim_sampler.xosc import Scenario  # noqa: E402

try:
    import jsonschema
except ImportError:
    jsonschema = None

SC = Scenario(EXAMPLES / "walker_dartout.xosc")


def good(proposal=False):
    name = "walker_dartout_importance" if proposal else "walker_dartout_dist"
    return Sampler.from_file(EXAMPLES / f"{name}.xosc").draw(n=1).records[0]


def rehash(r):
    try:
        r["id"] = record_id(r)
    except ValueError:  # NaN: not JSON, so not hashable; the checker says so
        pass
    return r


class Checker(unittest.TestCase):
    def test_a_drawn_record_is_valid(self):
        for p in (False, True):
            self.assertEqual(check(good(p), scenario=SC), [])

    def test_the_id_is_over_content_not_paths(self):
        r = good()
        moved = copy.deepcopy(r)
        moved["scenario"]["path"] = "/elsewhere/walker_dartout.xosc"
        self.assertEqual(record_id(moved), r["id"])
        changed = copy.deepcopy(r)
        changed["values"]["ego_speed"] += 1
        self.assertIn("id: not the hash of this record", " ".join(check(changed)))

    def assert_problem(self, r, text, **kw):
        problems = check(rehash(r), **kw)
        self.assertTrue(any(text in p for p in problems), f"{text!r} not in {problems}")

    def test_each_defect_is_named(self):
        r = good()
        cases = [
            (lambda x: x.update(record=2), "record: 1"),
            (lambda x: x.update(category="R9.walker.dart"), "category: R<family>"),
            (lambda x: x.update(category="R2.Walker Crossing.x"), "category: R<family>"),
            (lambda x: x["seeds"].update(world=2 ** 32), "seeds.world: an integer in [0, 2^32)"),
            (lambda x: x["seeds"]["pilots"].update(ego=-1), "seeds.pilots.ego"),
            (lambda x: x.update(weight=0.5), "weight 1 and log_weight 0 without a proposal"),
            (lambda x: x["values"].update(ego_speed=float("nan")), "values.ego_speed"),
            (lambda x: x.update(derived=["nope"]), "derived: nope is not in values"),
            (lambda x: x["scenario"].update(sha256="abc"), "scenario.sha256: 64 lowercase hex"),
            (lambda x: x.update(extra=1), "unknown extra"),
            (lambda x: x.pop("seeds"), "missing seeds"),
            (lambda x: x.update(search={"id": "ce", "generation": -1}), "search: null or"),
            (lambda x: x.update(car={"name": "m3", "spec_hash": "x"}), "car.spec_hash"),
        ]
        for f, text in cases:
            x = copy.deepcopy(r)
            f(x)
            self.assert_problem(x, text)

    def test_the_weight_must_be_its_logarithms_exponent(self):
        r = good(proposal=True)
        r["weight"] *= 2
        self.assert_problem(r, "is not exp(log_weight)")
        r = good(proposal=True)
        r["weight"], r["log_weight"] = 0.0, -900.0  # an underflowed weight is still valid
        self.assertEqual(check(rehash(r)), [])

    def test_a_proposal_needs_a_base(self):
        r = good(proposal=True)
        r["base"] = None
        self.assert_problem(r, "a proposal is weighted against a base")

    def test_against_the_scenario(self):
        r = good()
        r["values"]["speed_limit"] = 3.0
        self.assert_problem(r, "values.speed_limit: not a parameter", scenario=SC)
        r = good()
        r["cell"].pop("occluder")
        self.assert_problem(r, "cell: has illumination, surface", scenario=SC)
        r = good()
        r["seeds"]["pilots"]["bus"] = 1
        self.assert_problem(r, "seeds.pilots: bus not entities", scenario=SC)
        r = good()
        r["values"]["walker_type"] = 3
        self.assert_problem(r, "values.walker_type: 3 is not a string", scenario=SC)
        r = good()
        r["scenario"]["sha256"] = "0" * 64
        self.assert_problem(r, "scenario.sha256: ", scenario=SC)

    def test_bound_requires_world_and_car(self):
        r = good()
        self.assert_problem(r, "missing world, car", bound=True)
        r["world"] = {"name": "town_street", "hash": "stage:" + "0" * 64}
        r["car"] = {"name": "tesla_model3", "spec_hash": "a" * 64}
        self.assertEqual(check(rehash(r), bound=True), [])


@unittest.skipIf(jsonschema is None, "jsonschema is not installed in this Python")
class Schema(unittest.TestCase):
    def test_the_schema_is_valid_and_accepts_what_the_sampler_writes(self):
        schema = json.loads(SCHEMA.read_text())
        jsonschema.Draft202012Validator.check_schema(schema)
        v = jsonschema.Draft202012Validator(schema)
        for p in (False, True):
            self.assertEqual(list(v.iter_errors(good(p))), [])

    def test_the_schema_refuses_what_the_checker_refuses_structurally(self):
        v = jsonschema.Draft202012Validator(json.loads(SCHEMA.read_text()))
        for f in (lambda x: x.update(record=2), lambda x: x.update(category="R9.a.b"),
                  lambda x: x["seeds"].update(world=-1), lambda x: x.pop("seeds"),
                  lambda x: x.update(extra=1), lambda x: x.update(weight=-1)):
            x = good()
            f(x)
            self.assertTrue(list(v.iter_errors(x)))
            self.assertTrue(check(rehash(x)))


if __name__ == "__main__":
    unittest.main()
