#!/usr/bin/env python3
"""Genomes over the demo tour's incident templates: the gap campaign's scenario space
(roadmap 025 step 3).

A scenario is a template plus a genome. The templates are 033's incidents
(`gen_demo_incidents.py`): they keep the authoring rules SSv2 needs, run on the real ego and
sit on Town01 route C. A genome is a family and a value per gene; rendering edits a copy of
the incident's sites entry (`demo_town01_sites.yaml`) and lets the generator write the
scenario, so a mutant is as valid as its template. A gene's default is today's template:
the default genome renders the committed incident file byte for byte.

Gene kinds: a scalar the generator reads from the sites entry (with its own default when
the entry has none), a shift of a site along or across its own heading (the incident sites
are on straight road, so a few metres stay in lane), and an actor's body (a declared box,
or a blueprint declaring its measured box).

    scripts/gap_genome.py default lead_brake                 # the genome as JSON
    scripts/gap_genome.py render genome.json --out x.xosc    # a genome's scenario
    scripts/gap_genome.py sample cyclist -n 5 --seed 1 --out DIR
"""

import argparse
import copy
import hashlib
import json
import math
import random
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parent))
import catalog_scenarios  # noqa: E402  scripts/, beside this file
import check_scenario_vehicles  # noqa: E402
import gen_demo_incidents as gen  # noqa: E402
import yaml  # noqa: E402

SITES = yaml.safe_load(gen.SITES.read_text())
DECIMALS = 3  # a Real gene's resolution: renders and ids are stable at it


@dataclass(frozen=True)
class Real:
    lo: float
    hi: float
    sigma: float  # a nudge's standard deviation
    default: Callable[[dict], float]  # from the incident's sites entry

    def clip(self, v):
        return round(min(self.hi, max(self.lo, v)), DECIMALS)


@dataclass(frozen=True)
class Choice:
    options: tuple
    default: Callable[[dict], object]


def site(key, fallback):
    """a default read from the sites entry, else the generator's own"""
    return lambda S: S.get(key, fallback)


def const(v):
    return lambda S: v


@dataclass(frozen=True)
class Family:
    incident: int
    genes: dict
    apply: Callable[[dict, dict], None]  # (sites entry copy, values): edit it in place


@dataclass(frozen=True)
class Genome:
    family: str
    values: dict = field(compare=True)

    @property
    def id(self):
        blob = json.dumps([self.family, sorted(self.values.items())], sort_keys=True)
        return hashlib.sha1(blob.encode()).hexdigest()[:12]

    def to_json(self):
        return {"family": self.family, "values": dict(self.values), "id": self.id}

    @staticmethod
    def from_json(d):
        return Genome(d["family"], dict(d["values"]))


# ---------------------------------------------------------------------------- sites edits


def shift(p, along=0.0, across=0.0):
    """`p` moved `along` its heading and `across` it (left positive); untouched when both
    are 0, so a default genome renders the sites file's own numbers"""
    if not along and not across:
        return p
    c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
    return dict(p, x=round(p["x"] + along * c - across * s, DECIMALS),
                y=round(p["y"] + along * s + across * c, DECIMALS))


def set_if_changed(S, key, value, fallback):
    if value != S.get(key, fallback):
        S[key] = value


STARTER_BOX = (4.5, 2.1, 1.8)


def body(S, name, kind, box):
    """an actor's body: a blueprint (`kind`), or the declared `box` when it is not the
    starter car's"""
    if kind != "box":
        S.setdefault("bodies", {})[name] = kind
    elif tuple(box) != STARTER_BOX:
        S.setdefault("boxes", {})[name] = list(box)


def apply_lead_brake(S, v):
    set_if_changed(S, "lead_mps", v["lead_mps"], 3.0)
    set_if_changed(S, "brake_gap_m", v["brake_gap_m"], 12.0)
    set_if_changed(S, "lead_decel", v["lead_decel"], 6.0)
    body(S, "i1_lead", v["lead_body"], (v["lead_length"], v["lead_width"], v["lead_height"]))


def apply_crossing_pedestrian(S, v):
    set_if_changed(S, "ped_mps", v["ped_mps"], 1.5)
    set_if_changed(S, "ped_hold_s", v["ped_hold_s"], 4.0)
    S["dart"] = shift(S["dart"], along=v["dart_shift_m"])
    if v["ped_start_m"]:
        # back from the lane (positive) or toward it, along the way he walks
        p, far = S["ped_spawn"], S["ped_far"]
        u = math.atan2(far["y"] - p["y"], far["x"] - p["x"])
        S["ped_spawn"] = dict(p, x=round(p["x"] - v["ped_start_m"] * math.cos(u), DECIMALS),
                              y=round(p["y"] - v["ped_start_m"] * math.sin(u), DECIMALS))


def apply_parked_obstacle(S, v):
    set_if_changed(S, "oncoming_mps", v["oncoming_mps"], 6.0)
    S["broken"] = shift(S["broken"], across=v["broken_lateral_m"])
    body(S, "i4_broken", "box", (v["broken_length"], STARTER_BOX[1], STARTER_BOX[2]))


def apply_cross_traffic(S, v):
    set_if_changed(S, "cross_mps", v["cross_mps"], 8.0)


def apply_cyclist(S, v):
    set_if_changed(S, "cyclist_mps", v["cyclist_mps"], 5.5)
    for k in ("cyclist_spawn", "cyclist_turn", "cyclist_far"):
        S[k] = shift(S[k], across=v["cyclist_lateral_m"])


LEAD_BODIES = ("box", "vehicle.tesla.model3", "vehicle.mercedes.sprinter",
               "vehicle.carlamotors.carlacola", "vehicle.mini.cooper_s",
               "vehicle.tesla.cybertruck", "vehicle.volkswagen.t2")

FAMILIES = {
    "lead_brake": Family(1, {
        "lead_mps": Real(1.5, 11.0, 1.0, site("lead_mps", 3.0)),
        "brake_gap_m": Real(3.0, 30.0, 3.0, site("brake_gap_m", 12.0)),
        "lead_decel": Real(2.0, 9.0, 1.0, site("lead_decel", 6.0)),
        "lead_body": Choice(LEAD_BODIES, const("box")),
        "lead_length": Real(3.5, 12.0, 1.0, const(STARTER_BOX[0])),
        "lead_width": Real(1.6, 2.6, 0.15, const(STARTER_BOX[1])),
        "lead_height": Real(1.4, 3.8, 0.3, const(STARTER_BOX[2])),
    }, apply_lead_brake),
    "crossing_pedestrian": Family(2, {
        "ped_mps": Real(0.6, 3.0, 0.3, site("ped_mps", 1.5)),
        "ped_hold_s": Real(0.0, 8.0, 1.0, site("ped_hold_s", 4.0)),
        "dart_shift_m": Real(-10.0, 10.0, 2.0, const(0.0)),
        "ped_start_m": Real(-0.5, 1.5, 0.3, const(0.0)),
    }, apply_crossing_pedestrian),
    "parked_obstacle": Family(4, {
        "oncoming_mps": Real(3.0, 12.0, 1.0, site("oncoming_mps", 6.0)),
        "broken_lateral_m": Real(-1.0, 1.0, 0.2, const(0.0)),
        "broken_length": Real(3.5, 12.0, 1.0, const(STARTER_BOX[0])),
    }, apply_parked_obstacle),
    "cross_traffic": Family(5, {
        "cross_mps": Real(3.0, 12.0, 1.0, site("cross_mps", 8.0)),
    }, apply_cross_traffic),
    "cyclist": Family(6, {
        "cyclist_mps": Real(3.0, 9.0, 0.7, site("cyclist_mps", 5.5)),
        "cyclist_lateral_m": Real(-0.6, 0.6, 0.15, const(0.0)),
    }, apply_cyclist),
}


# ---------------------------------------------------------------------------- genomes


def entry(family):
    return SITES["incidents"][FAMILIES[family].incident]


def default(family):
    S = entry(family)
    return Genome(family, {k: g.default(S) for k, g in FAMILIES[family].genes.items()})


def random_genome(family, rng):
    vals = {}
    for k, g in FAMILIES[family].genes.items():
        vals[k] = g.clip(rng.uniform(g.lo, g.hi)) if isinstance(g, Real) else rng.choice(g.options)
    return Genome(family, vals)


OPERATORS = ("nudge", "boundary", "reset", "swap")


def mutate(genome, rng, op=None):
    """(mutant, the operator applied). `op` None picks one; `swap` on a family without a
    Choice gene nudges instead."""
    genes = FAMILIES[genome.family].genes
    reals = [k for k, g in genes.items() if isinstance(g, Real)]
    choices = [k for k, g in genes.items() if isinstance(g, Choice)]
    op = op or rng.choice(OPERATORS)
    if op == "swap" and not choices:
        op = "nudge"
    v = dict(genome.values)
    if op == "nudge":
        for k in rng.sample(reals, min(len(reals), 1 if rng.random() < 0.7 else 2)):
            g = genes[k]
            v[k] = g.clip(v[k] + rng.gauss(0.0, g.sigma))
    elif op == "boundary":
        k = rng.choice(reals)
        g = genes[k]
        v[k] = g.hi if v[k] == g.lo else g.lo if v[k] == g.hi else rng.choice((g.lo, g.hi))
    elif op == "reset":
        d = default(genome.family).values
        off = [k for k in genes if v[k] != d[k]] or list(genes)
        k = rng.choice(off)
        v[k] = d[k]
    elif op == "swap":
        k = rng.choice(choices)
        v[k] = rng.choice([o for o in genes[k].options if o != v[k]])
    else:
        raise ValueError(op)
    return Genome(genome.family, v), op


def splice(a, b, rng):
    """each gene from one parent or the other; both of one family"""
    if a.family != b.family:
        raise ValueError(f"splice across families: {a.family}, {b.family}")
    return Genome(a.family, {k: rng.choice((a.values[k], b.values[k])) for k in a.values})


# ---------------------------------------------------------------------------- rendering


def render(genome):
    """the genome's scenario, as `gen_demo_incidents.py` writes an incident"""
    fam = FAMILIES[genome.family]
    S = copy.deepcopy(SITES["incidents"][fam.incident])
    fam.apply(S, genome.values)
    text = gen.template(fam.incident, {"incidents": {fam.incident: S}})
    return catalog_scenarios.rewrite(text)[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("default")
    d.add_argument("family", choices=FAMILIES)
    r = sub.add_parser("render")
    r.add_argument("genome", type=Path)
    r.add_argument("--out", type=Path)
    s = sub.add_parser("sample")
    s.add_argument("family", choices=FAMILIES)
    s.add_argument("-n", type=int, default=10)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    if a.cmd == "default":
        print(json.dumps(default(a.family).to_json(), indent=1))
    elif a.cmd == "render":
        text = render(Genome.from_json(json.loads(a.genome.read_text())))
        if a.out:
            a.out.write_text(text)
        else:
            sys.stdout.write(text)
    else:
        rng = random.Random(a.seed)
        a.out.mkdir(parents=True, exist_ok=True)
        for _ in range(a.n):
            g, _ = mutate(default(a.family), rng)
            (a.out / f"{a.family}_{g.id}.json").write_text(json.dumps(g.to_json(), indent=1))
            (a.out / f"{a.family}_{g.id}.xosc").write_text(render(g))
            print(a.out / f"{a.family}_{g.id}.xosc")


if __name__ == "__main__":
    main()
