"""The sampler: a distribution file (or a scenario and a model file) to concrete parameter
sets, each with its sample record.

    from scensim_sampler import Sampler
    batch = Sampler.from_file("walker_dartout_dist.xosc").draw(n=100, seed=7)
    batch.records[0]["values"], batch.rejected

Per sample i of a run seeded S, numpy's SeedSequence(S, spawn_key=(i,)) gives four
independent streams: the draw, the scenario's seed, the world's and the pilots'. A sample is
therefore the same whichever other samples are drawn, and a record names S and i.
"""
import itertools
import math
from pathlib import Path

import numpy as np

from . import __version__, record as R
from .dist import SpecError
from .plugins.importance import Importance
from .plugins.joint import Joint
from .xosc import Distribution, Scenario

MAX_REJECTS = 1000  # per sample: draws refused by constraints or outside the base's support


class SamplingError(ValueError):
    pass


class Batch:
    def __init__(self, records, rejected, rejected_why):
        self.records, self.rejected, self.rejected_why = records, rejected, rejected_why

    def summary(self):
        return {"records": len(self.records), "rejected": self.rejected,
                "rejected_why": dict(self.rejected_why)}


class Sampler:
    def __init__(self, scenario, plugins, distribution=None, stochastic=True, runs=None,
                 seed=None, knobs=None):
        self.scenario, self.plugins = scenario, list(plugins)
        self.distribution, self.stochastic = distribution, stochastic
        self.runs, self.seed = runs, seed
        self.knobs = {}
        for p in self.plugins:
            self.knobs.update(p.knobs)
        self.knobs.update(knobs or {})
        # a factor drawn first (twise rows, set by index) may condition what follows
        self.plugins.sort(key=lambda p: not p.deterministic)
        bases = [p.base for p in self.plugins if p.base is not None]
        if len({b["sha256"] for b in bases}) > 1:
            raise SpecError("more than one fitted base model; a record (version 1) names one")
        self.base = bases[0] if bases else None
        props = [p.proposal for p in self.plugins if p.proposal is not None]
        if len(props) > 1:
            raise SpecError("more than one proposal; a record (version 1) names one")
        self.proposal = props[0] if props else None
        if not self.scenario.category:
            raise SpecError(f"{self.scenario.path}: its FileHeader has no scensim.category "
                            f"property (R<family>.<category>.<subtype>)")
        if not R.CATEGORY.match(self.scenario.category):
            raise SpecError(f"{self.scenario.path}: scensim.category "
                            f"{self.scenario.category!r} is not R<family>.<category>.<subtype>")

    @classmethod
    def from_file(cls, path, model=None, proposal=None, knobs=None):
        """A ParameterValueDistribution file; or a scenario with `model`, a model file whose
        variables it draws. `proposal`: a proposal file, replacing the model of its base."""
        path = Path(path)
        sc = Scenario(path)
        if sc.is_distribution:
            if model is not None:
                raise SpecError(f"{path} is a distribution file; --model is for a scenario")
            d = Distribution(path)
            s = cls(d.scenario, d.plugins, distribution={"path": str(path), "sha256": d.sha256},
                    stochastic=d.stochastic, runs=d.runs, seed=d.seed, knobs=knobs)
        else:
            if model is None:
                raise SpecError(f"{path} declares no distribution: give a "
                                f"ParameterValueDistribution file, or a model file (--model)")
            plugin = Joint(str(model), base_dir=".", parameter=None, scenario=sc)
            plugin.name = "joint"
            s = cls(sc, [plugin], knobs=knobs)
        if proposal is not None:
            s._propose(proposal)
        return s

    def _propose(self, path):
        for i, p in enumerate(self.plugins):
            if isinstance(p, Joint):
                imp = Importance(proposal_path=path, base_plugin=p)
                imp.name = "importance"
                self.plugins[i] = imp
                self.proposal = imp.proposal
                return
        raise SpecError(f"{path}: a proposal replaces a fitted model, and this scenario's "
                        f"distributions have none")

    # -- one sample

    def _finish(self, values):
        env = dict(self.knobs)
        env.update(values)
        derived = []
        for p in self.plugins:
            d = p.derived(env)
            env.update(d)
            derived += list(d)
        why = []
        for p in self.plugins:
            why += p.violations(env)
        log_w = sum(p.log_weight(env) for p in self.plugins)
        if log_w == -math.inf:
            why.append("outside the base's support")
        cell = dict(self.scenario.fixed_cell())
        for p in self.plugins:
            cell.update(p.cell(env))
        drawn = {k: env[k] for k in itertools.chain(values, derived)}
        return drawn, derived, cell, log_w, why

    def _record(self, run_seed, index, drawn, derived, cell, log_w, streams):
        sc = self.scenario
        params, cell = {}, dict(cell)
        for k, v in drawn.items():
            if k in sc.parameters:
                params[k] = sc.cast(k, v)
            elif k in sc.cell_attributes:
                cell[k] = v
            elif k in derived:
                continue  # an intermediate the constraints or the cell read; not recorded
            else:
                raise SpecError(f"{k} is drawn, and it is neither a parameter of "
                                f"{sc.path.name} nor one of its cell attributes")
        missing = [a for a in sc.cell_attributes if a not in cell]
        if missing:
            raise SpecError(f"cell attributes {', '.join(missing)} have no value: draw them, "
                            f"map them in the model's cell, or fix them in the scenario's "
                            f"FileHeader (scensim.cell.<attribute>)")
        cell = {a: _plain(cell[a]) for a in sc.cell_attributes}
        _, scen_ss, world_ss, pilot_ss = streams
        pilots = pilot_ss.generate_state(max(1, len(sc.entities)))
        weight = math.exp(log_w) if self.proposal is not None else 1.0
        rec = {
            "record": R.VERSION,
            "scenario": {"path": str(sc.path), "sha256": sc.sha256},
            "distribution": self.distribution,
            "category": sc.category,
            "cell": cell,
            "values": params,
            "derived": [k for k in derived if k in params],
            "seeds": {"scenario": int(scen_ss.generate_state(1)[0]),
                      "world": int(world_ss.generate_state(1)[0]),
                      "pilots": {e: int(s) for e, s in zip(sc.entities, pilots)}},
            "base": self.base,
            "proposal": self.proposal,
            "weight": weight,
            "log_weight": float(log_w) if self.proposal is not None else 0.0,
            "search": None,
            "sampler": {"name": "scensim_sampler", "version": __version__,
                        "plugins": [p.name for p in self.plugins], "run_seed": int(run_seed),
                        "index": int(index)},
        }
        rec["id"] = R.record_id(rec)
        return {k: rec[k] for k in R.KEYS if k in rec}

    # -- a batch

    def draw(self, n=None, seed=None):
        """`n` samples (stochastic; default numberOfTestRuns), or every row (deterministic;
        `n` takes the first n). `seed`: the run seed (default the file's randomSeed; 0 for a
        deterministic file); a stochastic one requires it, so a batch can be drawn again."""
        seed = self.seed if seed is None else seed
        if seed is None and not self.stochastic:
            seed = 0  # rows are not drawn; the seed only derives the runs' seeds
        if seed is None:
            raise SpecError("a run seed is required (--seed, or the Stochastic randomSeed)")
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise SpecError(f"the run seed is an integer >= 0, not {seed!r}")
        if self.stochastic:
            n = self.runs if n is None else n
            if n is None:
                raise SpecError("how many samples: give n (-n)")
            return self._stochastic(n, seed)
        return self._deterministic(n, seed)

    def _stochastic(self, n, seed):
        records, rejected, why_count = [], 0, {}
        for i in range(n):
            streams = np.random.SeedSequence(seed, spawn_key=(i,)).spawn(4)
            rng = np.random.Generator(np.random.PCG64(streams[0]))
            for tries in range(MAX_REJECTS + 1):
                values = {}
                for p in self.plugins:
                    env = dict(self.knobs)
                    env.update(values)
                    values.update(p.draw(rng, i, env))
                drawn, derived, cell, log_w, why = self._finish(values)
                if not why:
                    break
                rejected += 1
                for w in why:
                    why_count[w] = why_count.get(w, 0) + 1
            else:
                raise SamplingError(f"sample {i}: {MAX_REJECTS} draws in a row rejected "
                                    f"(last: {'; '.join(why)}); the constraints leave almost "
                                    f"nothing of the distribution")
            records.append(self._record(seed, i, drawn, derived, cell, log_w, streams))
        return Batch(records, rejected, why_count)

    def _deterministic(self, n, seed):
        records, rejected, why_count = [], 0, {}
        lists = [p.rows() for p in self.plugins]
        for i, combo in enumerate(itertools.product(*lists)):
            if n is not None and len(records) >= n:
                break
            values = {}
            for row in combo:
                values.update(row)
            drawn, derived, cell, log_w, why = self._finish(values)
            if why:
                rejected += 1
                for w in why:
                    why_count[w] = why_count.get(w, 0) + 1
                continue
            streams = np.random.SeedSequence(seed, spawn_key=(i,)).spawn(4)
            records.append(self._record(seed, i, drawn, derived, cell, log_w, streams))
        return Batch(records, rejected, why_count)


def _plain(v):
    if isinstance(v, (np.generic,)):
        return v.item()
    return v
