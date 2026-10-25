#!/usr/bin/env python3
"""The gap campaign's search (roadmap 025 step 5): which genome to run next.

Ask/tell, no simulator and no clock: `Search.ask()` returns the next genome
(`gap_genome.Genome`), `Search.tell(genome, outcome)` takes what its run gave. Its whole
state is JSON (`to_json`, `from_json`), so the campaign store can resume it.

- **Outcome**: `valid`, `signature` (the oracle's gap signature, or None when SSv2 and the
  simulator agree) and `closeness` in [0, 1], which rises as the run's two geometries come
  nearer to disagreeing (1 at a disagreement). The search only compares closeness.
- **Corpus**: a valid genome enters on a new signature, or on a closeness above its
  family's best so far (the gradient before the first disagreement).
- **Archive** (MAP-Elites): per family, cells over two normalised genes (`DESCRIPTORS`,
  `BINS` bins each); a cell keeps its closest genome, and seeds come from these elites.
- **Seeds**: Exp3 over a family's elites; reward 1 for a new signature, else the closeness
  gained, less a pick cost, so a seed that keeps giving nothing loses weight. A seed's
  energy adds its closeness over a temperature that anneals from `TEMP_START` to
  `TEMP_END`: broad at first, then the seeds nearest a disagreement.
- **Operators**: discounted Thompson sampling, a Beta posterior per operator shrunk by
  `OP_DISCOUNT` at every credit; success = the child entered the corpus.
- **Families**: Exp3 one level up, rewarded by each run's closeness (1 at a disagreement):
  the families whose runs come nearest get more of them. **Schedule**: the share of fresh random genomes anneals
  from `RANDOM_START` to `RANDOM_END` over the budget; with no elite yet, every ask is random.
"""

import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gap_genome as gg  # noqa: E402

BINS = 4
DESCRIPTORS = {
    "lead_brake": ("lead_length", "brake_gap_m"),
    "crossing_pedestrian": ("ped_mps", "dart_shift_m"),
    "parked_obstacle": ("broken_length", "broken_lateral_m"),
    "cross_traffic": ("cross_mps",),
    "cyclist": ("cyclist_mps", "cyclist_lateral_m"),
}
OPERATORS = ("nudge", "boundary", "reset", "swap", "splice")
GAMMA = 0.1  # Exp3's exploration share
PICK_COST = 0.05  # an Exp3 seed's reward is less this per pick
RANDOM_START, RANDOM_END = 0.5, 0.05
# A seed's energy: its Exp3 log weight plus closeness / temperature, which anneals from broad
# to the seeds nearest a disagreement over the budget (AFLGo)
TEMP_START, TEMP_END = 0.1, 0.02
OP_DISCOUNT = 0.98  # each operator credit shrinks every Beta posterior toward its prior


@dataclass(frozen=True)
class Outcome:
    valid: bool
    signature: str | None
    closeness: float


def normalise(genome):
    """{gene: value in [0, 1]}: a Real by its range, a Choice by its option's index"""
    out = {}
    for k, g in gg.FAMILIES[genome.family].genes.items():
        v = genome.values[k]
        if isinstance(g, gg.Real):
            out[k] = (v - g.lo) / (g.hi - g.lo)
        else:
            out[k] = g.options.index(v) / max(1, len(g.options) - 1)
    return out


def cell(genome):
    u = normalise(genome)
    return "/".join(str(min(BINS - 1, int(u[k] * BINS))) for k in DESCRIPTORS[genome.family])


class Exp3:
    """Exp3 over named arms in log-weight form (arms appear as they are added)"""

    @staticmethod
    def probabilities(logw):
        if not logw:
            return {}
        top = max(logw.values())
        w = {k: math.exp(v - top) for k, v in logw.items()}
        total, n = sum(w.values()), len(w)
        return {k: (1 - GAMMA) * w[k] / total + GAMMA / n for k in w}

    @staticmethod
    def pick(rng, logw, bias=None):
        """(arm, its probability); `bias` adds to each arm's log weight for this pick only"""
        p = Exp3.probabilities({k: v + (bias or {}).get(k, 0.0) for k, v in logw.items()})
        keys = sorted(p)
        x, acc = rng.random(), 0.0
        for k in keys:
            acc += p[k]
            if x < acc:
                return k, p[k]
        return keys[-1], p[keys[-1]]

    @staticmethod
    def reward(logw, arm, p, r):
        logw[arm] += GAMMA * r / (p * len(logw))


class Search:
    def __init__(self, families, budget, seed=0):
        self.families = list(families)
        self.budget = budget
        self.rng = random.Random(seed)
        self.asked = 0
        self.corpus = []  # entries: {"genome", "closeness", "signature"}
        self.signatures = set()
        self.best = {f: 0.0 for f in self.families}
        self.archive = {f: {} for f in self.families}  # family -> cell -> entry
        self.seed_logw = {f: {} for f in self.families}  # family -> genome id -> log weight
        self.family_logw = {f: 0.0 for f in self.families}
        self.beta = {op: [1.0, 1.0] for op in OPERATORS}
        self.told = {f: 0 for f in self.families}
        self.valid = {f: 0 for f in self.families}
        self.invalid = 0
        self.pending = {}  # genome id -> what produced it

    # ------------------------------------------------------------------ choices

    def progress(self):
        return min(1.0, self.asked / max(1, self.budget))

    def random_share(self):
        return RANDOM_START + (RANDOM_END - RANDOM_START) * self.progress()

    def temperature(self):
        return TEMP_START + (TEMP_END - TEMP_START) * self.progress()

    def pick_operator(self, rng, elites):
        ops = [op for op in OPERATORS if op != "splice" or elites >= 2]
        draws = {op: rng.betavariate(*self.beta[op]) for op in ops}
        return max(ops, key=lambda op: draws[op])

    def credit_operator(self, op, success):
        # discounted: what pays changes as the search moves
        for ab in self.beta.values():
            ab[0], ab[1] = 1.0 + (ab[0] - 1.0) * OP_DISCOUNT, 1.0 + (ab[1] - 1.0) * OP_DISCOUNT
        self.beta[op][0 if success else 1] += 1.0

    def ask(self):
        rng = self.rng
        family, fp = Exp3.pick(rng, self.family_logw)
        elites = self.archive[family]
        info = {"family": family, "family_p": fp, "op": "random", "seed": None, "seed_p": None}
        if not elites or rng.random() < self.random_share():
            child = gg.random_genome(family, rng)
        else:
            op = self.pick_operator(rng, len(elites))
            t = self.temperature()
            energy = {e["genome"]["id"]: e["closeness"] / t for e in elites.values()}
            sid, sp = Exp3.pick(rng, self.seed_logw[family], energy)
            seed = gg.Genome.from_json(self._elite(family, sid)["genome"])
            if op == "splice":
                other = gg.Genome.from_json(elites[sorted(elites)[rng.randrange(len(elites))]]["genome"])
                child = gg.splice(seed, other, rng)
            else:
                child, _ = gg.mutate(seed, rng, op=op)
            info.update(op=op, seed=sid, seed_p=sp)
        self.asked += 1
        self.pending[child.id] = info
        return child

    def _elite(self, family, gid):
        return next(e for e in self.archive[family].values() if e["genome"]["id"] == gid)

    # ------------------------------------------------------------------ outcomes

    def tell(self, genome, outcome):
        family = genome.family
        info = self.pending.pop(genome.id, None)
        self.told[family] += 1
        gained, entered = 0.0, False
        if not outcome.valid:
            self.invalid += 1
        else:
            self.valid[family] += 1
            entry = {"genome": genome.to_json(), "closeness": outcome.closeness,
                     "signature": outcome.signature}
            if outcome.signature and outcome.signature not in self.signatures:
                self.signatures.add(outcome.signature)
                gained, entered = 1.0, True
            elif outcome.closeness > self.best[family] + 1e-12:
                gained, entered = outcome.closeness - self.best[family], True
            self.best[family] = max(self.best[family], outcome.closeness)
            if entered:
                self.corpus.append(entry)
            took_cell = self._place(family, entry)
            entered = entered or took_cell
        if info is None:
            return
        reward = gained if gained else (0.1 if entered else 0.0)
        # a family is worth what its runs come to: how near they get to disagreeing
        level = (1.0 if outcome.signature else outcome.closeness) if outcome.valid else 0.0
        Exp3.reward(self.family_logw, family, info["family_p"], level)
        if info["op"] != "random":
            self.credit_operator(info["op"], gained > 0.0)
            if info["seed"] in self.seed_logw[family]:
                Exp3.reward(self.seed_logw[family], info["seed"], info["seed_p"], reward - PICK_COST)

    def _place(self, family, entry):
        c = cell(gg.Genome.from_json(entry["genome"]))
        cells = self.archive[family]
        old = cells.get(c)
        if old is not None and old["closeness"] >= entry["closeness"]:
            return False
        if old is not None:
            self.seed_logw[family].pop(old["genome"]["id"], None)
        cells[c] = entry
        self.seed_logw[family].setdefault(entry["genome"]["id"], 0.0)
        return True

    def validity(self, family):
        return self.valid[family] / self.told[family] if self.told[family] else 1.0

    # ------------------------------------------------------------------ state

    def to_json(self):
        version, internal, gauss = self.rng.getstate()
        return {
            "families": self.families, "budget": self.budget, "asked": self.asked,
            "rng": [version, list(internal), gauss], "corpus": self.corpus,
            "signatures": sorted(self.signatures), "best": self.best, "archive": self.archive,
            "seed_logw": self.seed_logw, "family_logw": self.family_logw, "beta": self.beta,
            "told": self.told, "valid": self.valid, "invalid": self.invalid,
            "pending": self.pending,
        }

    @staticmethod
    def from_json(d):
        s = Search(d["families"], d["budget"])
        version, internal, gauss = d["rng"]
        s.rng.setstate((version, tuple(internal), gauss))
        s.asked, s.corpus, s.invalid = d["asked"], d["corpus"], d["invalid"]
        s.signatures = set(d["signatures"])
        for k in ("best", "archive", "seed_logw", "family_logw", "beta", "told", "valid",
                  "pending"):
            setattr(s, k, d[k])
        return s
