"""Unit tests for scripts/gap_search.py, the gap campaign's search (roadmap 025 step 5),
against a synthetic objective with a known gap region. No ROS, no simulator:

    python3 -m pytest src/swb_launch/test/test_gap_search.py
"""

import importlib.machinery
import importlib.util
import json
import math
import random
import statistics
import sys
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


gg = _load("gap_genome")
gs = _load("gap_search")

SEEDS = 20
BUDGET = 400


class Region:
    """a hidden gap: the genome disagrees inside a box of normalised gene values, and comes
    smoothly closer to it the nearer it is"""

    def __init__(self, family, box, signature="C1/vehicle/overlap", invalid=None):
        self.family, self.box, self.signature, self.invalid = family, box, signature, invalid

    def distance(self, genome):
        u = gs.normalise(genome)
        return math.sqrt(sum(max(lo - u[k], 0.0, u[k] - hi) ** 2
                             for k, (lo, hi) in self.box.items()))

    def __call__(self, genome):
        if genome.family != self.family:
            return gs.Outcome(True, None, 0.0)
        if self.invalid and self.invalid(gs.normalise(genome)):
            return gs.Outcome(False, None, 0.0)
        d = self.distance(genome)
        if d == 0.0:
            return gs.Outcome(True, self.signature, 1.0)
        return gs.Outcome(True, None, math.exp(-4.0 * d) * 0.99)


LEAD = Region("lead_brake", {"lead_length": (0.75, 0.85), "brake_gap_m": (0.15, 0.25)})
PED = Region("crossing_pedestrian", {"ped_mps": (0.6, 0.7), "dart_shift_m": (0.3, 0.4)})


def evaluations_to_hit(next_genome, tell, objective, budget=BUDGET):
    for n in range(1, budget + 1):
        g = next_genome()
        out = objective(g)
        tell(g, out)
        if out.signature:
            return n
    return budget + 1


def random_search(family, objective, seed):
    rng = random.Random(seed)
    return evaluations_to_hit(lambda: gg.random_genome(family, rng), lambda g, o: None, objective)


def guided_search(families, objective, seed):
    s = gs.Search(families, budget=BUDGET, seed=seed)
    return evaluations_to_hit(s.ask, s.tell, objective)


class Search(unittest.TestCase):
    def test_reaches_the_region_sooner_than_random(self):
        for region in (LEAD, PED):
            rnd = [random_search(region.family, region, s) for s in range(SEEDS)]
            srch = [guided_search([region.family], region, s) for s in range(SEEDS)]
            r, m = statistics.median(rnd), statistics.median(srch)
            self.assertLess(m, r, (region.family, sorted(srch), sorted(rnd)))
            self.assertLessEqual(sum(n > BUDGET for n in srch), 1, region.family)

    def test_finds_the_family_that_holds_the_gap(self):
        # the gap is in one family of five: against random over every family
        fams, budget = list(gg.FAMILIES), 1500

        def rnd(seed):
            rng = random.Random(seed)
            return evaluations_to_hit(lambda: gg.random_genome(rng.choice(fams), rng),
                                      lambda g, o: None, LEAD, budget)

        def srch(seed):
            s = gs.Search(fams, budget=budget, seed=seed)
            return evaluations_to_hit(s.ask, s.tell, LEAD, budget)

        r = [rnd(s) for s in range(SEEDS)]
        m = [srch(s) for s in range(SEEDS)]
        self.assertLess(statistics.median(m), statistics.median(r), (sorted(m), sorted(r)))
        self.assertEqual(sum(n > budget for n in m), 0, sorted(m))

    def test_a_resumed_search_continues_exactly(self):
        a = gs.Search(["lead_brake", "cyclist"], budget=BUDGET, seed=3)
        for _ in range(30):
            g = a.ask()
            a.tell(g, LEAD(g))
        snap = json.dumps(a.to_json())
        b = gs.Search.from_json(json.loads(snap))
        for _ in range(20):
            ga, gb = a.ask(), b.ask()
            self.assertEqual(ga, gb)
            a.tell(ga, LEAD(ga))
            b.tell(gb, LEAD(gb))
        self.assertEqual(a.to_json(), b.to_json())

    def test_the_corpus_takes_new_signatures_and_closer_genomes_only(self):
        s = gs.Search(["lead_brake"], budget=BUDGET, seed=0)
        g1, g2, g3, g4 = (gg.random_genome("lead_brake", random.Random(k)) for k in range(4))
        s.tell(g1, gs.Outcome(True, None, 0.3))
        s.tell(g2, gs.Outcome(True, None, 0.2))   # not closer: no entry
        s.tell(g3, gs.Outcome(False, None, 0.9))  # invalid: never
        s.tell(g4, gs.Outcome(True, "C2/walker/contact", 0.1))
        s.tell(g4, gs.Outcome(True, "C2/walker/contact", 0.1))  # known signature
        self.assertEqual([e["genome"]["id"] for e in s.corpus], [g1.id, g4.id])
        self.assertEqual(s.signatures, {"C2/walker/contact"})
        self.assertEqual(s.invalid, 1)

    def test_the_archive_keeps_the_closest_genome_per_cell(self):
        s = gs.Search(["lead_brake"], budget=BUDGET, seed=0)
        d = gg.default("lead_brake")
        near = gg.Genome("lead_brake", dict(d.values, lead_mps=d.values["lead_mps"] + 0.1))
        s.tell(d, gs.Outcome(True, None, 0.2))
        s.tell(near, gs.Outcome(True, None, 0.4))
        self.assertEqual(gs.cell(d), gs.cell(near))
        elites = s.archive["lead_brake"]
        self.assertEqual(len(elites), 1)
        self.assertEqual(next(iter(elites.values()))["genome"]["id"], near.id)

    def test_operators_that_pay_are_chosen_more(self):
        s = gs.Search(["lead_brake"], budget=BUDGET, seed=0)
        for _ in range(40):
            s.credit_operator("boundary", True)
            s.credit_operator("reset", False)
        rng = random.Random(1)
        picks = [s.pick_operator(rng, elites=2) for _ in range(200)]
        self.assertGreater(picks.count("boundary"), picks.count("reset") * 5)

    def test_random_share_anneals(self):
        s = gs.Search(["lead_brake"], budget=100, seed=0)
        self.assertAlmostEqual(s.random_share(), 0.5)
        s.asked = 100
        self.assertAlmostEqual(s.random_share(), 0.05)

    def test_invalid_genomes_feed_the_familys_validity_rate(self):
        bad = Region("cyclist", {"cyclist_mps": (0.9, 1.0)}, invalid=lambda u: u["cyclist_mps"] < 0.5)
        s = gs.Search(["cyclist"], budget=BUDGET, seed=2)
        for _ in range(60):
            g = s.ask()
            s.tell(g, bad(g))
        self.assertGreater(s.invalid, 0)
        self.assertTrue(0.0 < s.validity("cyclist") < 1.0)


if __name__ == "__main__":
    unittest.main()
