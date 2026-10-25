"""Unit tests for scripts/gap_genome.py, the gap campaign's genomes over 033's incident
templates (roadmap 025 step 3). No ROS, no simulator; SSv2's XSD validator runs where it is
installed:

    python3 -m pytest src/swb_launch/test/test_gap_genome.py
"""

import importlib.machinery
import importlib.util
import json
import os
import random
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
_SCRIPT = _ROOT / "scripts" / "gap_genome.py"
_loader = importlib.machinery.SourceFileLoader("gap_genome", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("gap_genome", _loader)
gg = importlib.util.module_from_spec(_spec)
_loader.exec_module(gg)

SAMPLE = 200  # mutants per family
VALIDATOR = _ROOT / "install/openscenario_validator/lib/openscenario_validator/validate"


def mutants(family, n, seed=1):
    """`n` genomes of `family`: random ones, their mutants and splices, seeded"""
    rng = random.Random(seed)
    pool = [gg.default(family)]
    while len(pool) < n:
        r = rng.random()
        if r < 0.3:
            g = gg.random_genome(family, rng)
        elif r < 0.85:
            g, _ = gg.mutate(rng.choice(pool), rng)
        else:
            g = gg.splice(rng.choice(pool), rng.choice(pool), rng)
        pool.append(g)
    return pool


def structural_failures(text):
    """the authoring rules a mutant must keep (033 step 2): what SSv2 would refuse to load"""
    root = ET.fromstring(text)
    out = []
    for tag in ("ScenarioObject", "ManeuverGroup", "Event"):
        names = [e.get("name") for e in root.iter(tag)]
        dup = {n for n in names if names.count(n) > 1}
        if dup:
            out.append(f"duplicate {tag} names {sorted(dup)}")
    decl = {p.get("name") for p in root.iter("ParameterDeclaration")}
    # (a CatalogReference's ParameterAssignment names the entry's parameters, not the file's)
    refs = {e.get("parameterRef") for e in root.iter()
            if e.get("parameterRef") and e.tag != "ParameterAssignment"}
    if refs - decl:
        out.append(f"undeclared parameters {sorted(refs - decl)}")
    # a StoryboardElementStateCondition names an event of its own ManeuverGroup
    for grp in root.iter("ManeuverGroup"):
        own = {e.get("name") for e in grp.iter("Event")}
        for c in grp.iter("StoryboardElementStateCondition"):
            if c.get("storyboardElementRef") not in own:
                out.append(f"{grp.get('name')}: refers to {c.get('storyboardElementRef')}")
    objects = {o.get("name") for o in root.iter("ScenarioObject")}
    for e in root.iter("EntityRef"):
        if e.get("entityRef") not in objects:
            out.append(f"unknown entity {e.get('entityRef')}")
    return out


class Genes(unittest.TestCase):
    def test_families_cover_the_four_incidents(self):
        self.assertEqual(
            {f: gg.FAMILIES[f].incident for f in gg.FAMILIES},
            {"lead_brake": 1, "crossing_pedestrian": 2, "parked_obstacle": 4,
             "cross_traffic": 5, "cyclist": 6})

    def test_defaults_come_from_the_sites_file(self):
        g = gg.default("lead_brake")
        S = gg.SITES["incidents"][1]
        self.assertEqual(g.values["lead_mps"], S["lead_mps"])
        self.assertEqual(g.values["brake_gap_m"], S["brake_gap_m"])
        self.assertEqual(g.values["lead_decel"], 6.0)  # the generator's own default

    def test_every_default_is_within_its_range(self):
        for f, fam in gg.FAMILIES.items():
            for name, value in gg.default(f).values.items():
                gene = fam.genes[name]
                if isinstance(gene, gg.Real):
                    self.assertTrue(gene.lo <= value <= gene.hi, (f, name, value))
                else:
                    self.assertIn(value, gene.options, (f, name))

    def test_mutants_stay_in_range_and_round_trip(self):
        for f, fam in gg.FAMILIES.items():
            for g in mutants(f, 50):
                self.assertEqual(set(g.values), set(fam.genes))
                for name, value in g.values.items():
                    gene = fam.genes[name]
                    if isinstance(gene, gg.Real):
                        self.assertTrue(gene.lo <= value <= gene.hi, (f, name, value))
                    else:
                        self.assertIn(value, gene.options)
                back = gg.Genome.from_json(json.loads(json.dumps(g.to_json())))
                self.assertEqual(back, g)
                self.assertEqual(back.id, g.id)

    def test_id_is_stable_and_tells_genomes_apart(self):
        a = gg.default("cyclist")
        self.assertEqual(a.id, gg.default("cyclist").id)
        b, _ = gg.mutate(a, random.Random(3), op="nudge")
        self.assertNotEqual(a.id, b.id)
        self.assertNotEqual(a.id, gg.default("lead_brake").id)

    def test_mutation_is_seeded(self):
        a = [gg.mutate(gg.default("crossing_pedestrian"), random.Random(7))[0] for _ in range(2)]
        self.assertEqual(a[0], a[1])

    def test_each_operator_does_what_it_says(self):
        rng = random.Random(5)
        d = gg.default("lead_brake")
        g, op = gg.mutate(d, rng, op="boundary")
        changed = [k for k in d.values if g.values[k] != d.values[k]]
        self.assertEqual(op, "boundary")
        self.assertTrue(changed)
        fam = gg.FAMILIES["lead_brake"]
        for k in changed:
            gene = fam.genes[k]
            if isinstance(gene, gg.Real):
                self.assertIn(g.values[k], (gene.lo, gene.hi))
        far = gg.random_genome("lead_brake", rng)
        r, _ = gg.mutate(far, rng, op="reset")
        reset = [k for k in far.values if r.values[k] != far.values[k]]
        for k in reset:
            self.assertEqual(r.values[k], d.values[k])
        s, op = gg.mutate(d, rng, op="swap")
        self.assertEqual(op, "swap")
        self.assertNotEqual(s.values["lead_body"], d.values["lead_body"])

    def test_splice_takes_each_gene_from_a_parent(self):
        rng = random.Random(11)
        a, b = gg.random_genome("cyclist", rng), gg.random_genome("cyclist", rng)
        c = gg.splice(a, b, rng)
        for k in c.values:
            self.assertIn(c.values[k], (a.values[k], b.values[k]))
        with self.assertRaises(ValueError):
            gg.splice(a, gg.default("lead_brake"), rng)


class Render(unittest.TestCase):
    def test_the_default_genome_is_the_committed_incident(self):
        for f, fam in gg.FAMILIES.items():
            committed = (gg.gen.DEFAULT_OUT / gg.gen.FILES[fam.incident]).read_text()
            self.assertEqual(gg.render(gg.default(f)), committed, f)

    def test_genes_reach_the_scenario(self):
        g = gg.default("crossing_pedestrian")
        v = dict(g.values, ped_mps=2.4, ped_hold_s=1.0, dart_shift_m=5.0)
        text = gg.render(gg.Genome("crossing_pedestrian", v))
        root = ET.fromstring(text)
        walk = [a for a in root.iter("Action") if a.get("name") == "i2_ped_walk"][0]
        self.assertEqual(float(walk.find(".//AbsoluteTargetSpeed").get("value")), 2.4)
        held = [c for c in root.iter("Condition") if c.get("name") == "i2_ped_held"][0]
        self.assertEqual(float(held.get("delay")), 1.0)
        # the dart point moved 5 m along the road (the road heads south: y falls)
        S = gg.SITES["incidents"][2]
        near = [c for c in root.iter("Condition") if c.get("name") == "i2_ego_near"][0]
        y = float(near.find(".//WorldPosition").get("y"))
        self.assertAlmostEqual(y, S["dart"]["y"] - 5.0, places=1)

    def test_a_declared_box_or_a_blueprint_reaches_the_lead(self):
        d = gg.default("lead_brake")
        box = gg.Genome("lead_brake", dict(d.values, lead_length=8.0, lead_height=3.0))
        lead = [o for o in ET.fromstring(gg.render(box)).iter("ScenarioObject")
                if o.get("name") == "i1_lead"][0]
        dims = lead.find(".//BoundingBox/Dimensions")
        self.assertEqual((float(dims.get("length")), float(dims.get("height"))), (8.0, 3.0))
        # the rear overhang is kept: the box grows forward from the rear axle
        centre = lead.find(".//BoundingBox/Center")
        self.assertAlmostEqual(float(centre.get("x")), 8.0 / 2 - 0.75)
        bp = gg.Genome("lead_brake", dict(d.values, lead_body="vehicle.mercedes.sprinter"))
        lead = [o for o in ET.fromstring(gg.render(bp)).iter("ScenarioObject")
                if o.get("name") == "i1_lead"][0]
        self.assertEqual(lead.find("Vehicle").get("name"), "vehicle.mercedes.sprinter")

    def test_a_seeded_sample_of_mutants_is_valid(self):
        table = gg.check_scenario_vehicles.scenario_vehicle.load_table(
            gg.check_scenario_vehicles.scenario_vehicle.TABLE)
        catalog = gg.check_scenario_vehicles.catalog_parameters()
        bodies = gg.catalog_scenarios.entries()
        with tempfile.TemporaryDirectory() as d:
            for f in gg.FAMILIES:
                for g in mutants(f, SAMPLE):
                    text = gg.render(g)
                    self.assertEqual(structural_failures(text), [], (f, g.values))
                    p = Path(d) / f"{g.id}.xosc"
                    p.write_text(text)
                    failures, _, _ = gg.check_scenario_vehicles.check_file(
                        p, table, 0.02, catalog, bodies)
                    self.assertEqual(failures, [], (f, g.values))

    @unittest.skipUnless(VALIDATOR.exists() and shutil.which("bash")
                         and Path("/opt/ros/humble/setup.bash").exists(),
                         "SSv2's openscenario_validator is not built here")
    def test_mutants_pass_ssv2s_schema(self):
        with tempfile.TemporaryDirectory() as d:
            paths = []
            for f in gg.FAMILIES:
                for g in mutants(f, 5, seed=2):
                    p = Path(d) / f"{f}_{g.id}.xosc"
                    p.write_text(gg.render(g))
                    paths.append(str(p))
            cmd = ("source /opt/ros/humble/setup.bash >/dev/null; "
                   f"for f in {' '.join(paths)}; do {VALIDATOR} \"$f\" || exit 1; done")
            r = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True,
                               timeout=1800, env=dict(os.environ))
            self.assertEqual(r.returncode, 0, r.stdout[-2000:] + r.stderr[-2000:])


if __name__ == "__main__":
    unittest.main()
