"""The sample record (version 1): what one drawn scenario is, enough to run it again anywhere
and to audit a rate computed from many. The spec is docs/design/sample-record.md; the JSON
Schema, for other tools, is scensim_sampler/schema/sample-record-1.json. `check()` gives every
problem of a record as a message, the schema's rules and those a schema cannot state (the id,
the weight against its logarithm, the record against its scenario).
"""
import hashlib
import json
import math
import re
from pathlib import Path

import yaml

VERSION = 1
FAMILIES = {
    "R1": "traffic conflicts with other vehicles",
    "R2": "vulnerable road users",
    "R3": "infrastructure, rules and maps",
    "R4": "perception disturbance",
    "R5": "vehicle motion disturbance",
    "R6": "the ego's own faults and degraded modes",
    "R7": "interaction and social behaviour",
    "R8": "ODD boundaries",
}
CATEGORY = re.compile(r"^(R[1-8])\.[a-z0-9_]+\.[a-z0-9_]+$")
HEX = re.compile(r"^[0-9a-f]{64}$")
KEYS = ("record", "id", "scenario", "distribution", "category", "cell", "values", "derived",
        "seeds", "base", "proposal", "weight", "log_weight", "search", "world", "car",
        "sampler")
REQUIRED = tuple(k for k in KEYS if k not in ("world", "car"))
UINT32 = 2 ** 32
SCHEMA = Path(__file__).parent / "schema" / "sample-record-1.json"


def canonical(value):
    """JSON with sorted keys and no spaces: the form the id is the hash of."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
                      allow_nan=False)


def _strip_paths(v):
    if isinstance(v, dict):
        return {k: _strip_paths(x) for k, x in v.items() if k != "path"}
    if isinstance(v, list):
        return [_strip_paths(x) for x in v]
    return v


def record_id(rec):
    """sha256 of the canonical record without its id and without any `path` (where a file was
    is not what it was: its sha256 is)."""
    body = {k: v for k, v in rec.items() if k != "id"}
    return hashlib.sha256(canonical(_strip_paths(body)).encode()).hexdigest()


def _scalar(v):
    return isinstance(v, (str, bool, int)) or (isinstance(v, float) and math.isfinite(v))


def _uint32(v):
    return isinstance(v, int) and not isinstance(v, bool) and 0 <= v < UINT32


def _file(v, at, out, nullable):
    if v is None:
        if not nullable:
            out.append(f"{at}: required")
        return
    if not isinstance(v, dict) or set(v) - {"path", "sha256"} or "sha256" not in v:
        out.append(f"{at}: {{path, sha256}}")
        return
    if not isinstance(v.get("path", ""), str):
        out.append(f"{at}.path: a string")
    if not isinstance(v["sha256"], str) or not HEX.match(v["sha256"]):
        out.append(f"{at}.sha256: 64 lowercase hex digits")


def _ident(v, at, out):
    if v is None:
        return
    if not isinstance(v, dict) or set(v) != {"id", "sha256"}:
        out.append(f"{at}: null or {{id, sha256}}")
        return
    if not isinstance(v["id"], str) or not v["id"]:
        out.append(f"{at}.id: a non-empty string")
    if not isinstance(v["sha256"], str) or not HEX.match(v["sha256"]):
        out.append(f"{at}.sha256: 64 lowercase hex digits")


def _number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def check(rec, scenario=None, bound=False):
    """Every problem of one record, as messages ([] when it is valid). `scenario`: a
    scensim_sampler.xosc.Scenario to check it against; `bound`: world and car are required
    (an executor has bound them)."""
    out = []
    if not isinstance(rec, dict):
        return ["a record is a mapping"]
    unknown = [k for k in rec if k not in KEYS]
    if unknown:
        out.append(f"unknown {', '.join(map(str, unknown))}")
    missing = [k for k in REQUIRED if k not in rec]
    if bound:
        missing += [k for k in ("world", "car") if k not in rec]
    if missing:
        out.append(f"missing {', '.join(missing)}")
    if rec.get("record") != VERSION:
        out.append(f"record: {VERSION} (the format's version), not {rec.get('record')!r}")
    if "id" in rec:
        if not isinstance(rec["id"], str) or not HEX.match(rec["id"]):
            out.append("id: 64 lowercase hex digits")
        else:
            try:
                if rec["id"] != record_id(rec):
                    out.append("id: not the hash of this record (it was changed after it was "
                               "drawn, or the id was not computed as the spec says)")
            except (TypeError, ValueError) as e:
                out.append(f"id: the record cannot be hashed ({e})")
    if "scenario" in rec:
        _file(rec["scenario"], "scenario", out, nullable=False)
    if "distribution" in rec:
        _file(rec["distribution"], "distribution", out, nullable=True)
    cat = rec.get("category")
    if "category" in rec:
        m = CATEGORY.match(cat) if isinstance(cat, str) else None
        if not m:
            out.append(f"category: R<family>.<category>.<subtype>, family R1 to R8, the rest "
                       f"lowercase words (survey-vehicle-risk-taxonomy), not {cat!r}")
    for key in ("cell", "values"):
        v = rec.get(key)
        if key in rec:
            if not isinstance(v, dict):
                out.append(f"{key}: a mapping")
            else:
                for k, x in v.items():
                    if not isinstance(k, str) or not k:
                        out.append(f"{key}: a key {k!r} is not a name")
                    elif not _scalar(x):
                        out.append(f"{key}.{k}: a string, number or boolean, not {x!r}")
    values = rec.get("values") if isinstance(rec.get("values"), dict) else {}
    if "derived" in rec:
        d = rec["derived"]
        if not isinstance(d, list) or not all(isinstance(x, str) for x in d):
            out.append("derived: a list of value names")
        else:
            for x in d:
                if x not in values:
                    out.append(f"derived: {x} is not in values")
    if "seeds" in rec:
        s = rec["seeds"]
        if not isinstance(s, dict) or set(s) != {"scenario", "world", "pilots"}:
            out.append("seeds: {scenario, world, pilots}")
        else:
            for k in ("scenario", "world"):
                if not _uint32(s[k]):
                    out.append(f"seeds.{k}: an integer in [0, 2^32)")
            if not isinstance(s["pilots"], dict):
                out.append("seeds.pilots: {entity: seed}")
            else:
                for k, x in s["pilots"].items():
                    if not _uint32(x):
                        out.append(f"seeds.pilots.{k}: an integer in [0, 2^32)")
    for k in ("base", "proposal"):
        if k in rec:
            _ident(rec[k], k, out)
    w, lw = rec.get("weight"), rec.get("log_weight")
    if "weight" in rec and not (_number(w) and w >= 0):
        out.append(f"weight: a finite number >= 0, not {w!r}")
    if "log_weight" in rec and not _number(lw):
        out.append(f"log_weight: a finite number, not {lw!r}")
    if _number(w) and _number(lw) and w >= 0:
        expect = math.exp(lw) if lw < 709 else math.inf
        if not math.isclose(w, expect, rel_tol=1e-9, abs_tol=1e-300):
            out.append(f"weight {w!r} is not exp(log_weight) = {expect!r}")
    if "proposal" in rec and rec.get("proposal") is None:
        if w != 1 or lw != 0:
            out.append("weight 1 and log_weight 0 without a proposal (a draw from the base "
                       "itself is not weighted)")
    elif rec.get("proposal") is not None and rec.get("base") is None:
        out.append("proposal: a proposal is weighted against a base, and base is null")
    if "search" in rec and rec["search"] is not None:
        s = rec["search"]
        if (not isinstance(s, dict) or set(s) != {"id", "generation"}
                or not isinstance(s["id"], str)
                or not (isinstance(s["generation"], int) and s["generation"] >= 0)):
            out.append("search: null or {id, generation >= 0}")
    if "world" in rec:
        v = rec["world"]
        if (not isinstance(v, dict) or set(v) != {"name", "hash"}
                or not all(isinstance(x, str) and x for x in v.values())):
            out.append("world: {name, hash}")
    if "car" in rec:
        v = rec["car"]
        if (not isinstance(v, dict) or set(v) != {"name", "spec_hash"}
                or not all(isinstance(x, str) and x for x in v.values())):
            out.append("car: {name, spec_hash}")
        elif not HEX.match(v["spec_hash"]):
            out.append("car.spec_hash: 64 lowercase hex digits (sim_vehicle_spec's spec_hash)")
    if "sampler" in rec:
        s = rec["sampler"]
        keys = {"name", "version", "plugins", "run_seed", "index"}
        if not isinstance(s, dict) or set(s) != keys:
            out.append(f"sampler: {{{', '.join(sorted(keys))}}}")
        else:
            if not (isinstance(s["name"], str) and isinstance(s["version"], str)):
                out.append("sampler.name, sampler.version: strings")
            if not (isinstance(s["plugins"], list)
                    and all(isinstance(x, str) for x in s["plugins"])):
                out.append("sampler.plugins: a list of plug-in names")
            for k in ("run_seed", "index"):
                if isinstance(s[k], bool) or not isinstance(s[k], int) or s[k] < 0:
                    out.append(f"sampler.{k}: an integer >= 0")
    if scenario is not None:
        out += against(rec, scenario)
    return out


def against(rec, scenario):
    """What a record says about its scenario, against the scenario file."""
    out = []
    sc = rec.get("scenario")
    if isinstance(sc, dict) and sc.get("sha256") != scenario.sha256:
        out.append(f"scenario.sha256: {scenario.path} hashes to {scenario.sha256}, the record "
                   f"names {sc.get('sha256')}")
    if isinstance(rec.get("category"), str) and rec["category"] != scenario.category:
        out.append(f"category: {rec['category']}, the scenario's is {scenario.category}")
    cell = rec.get("cell")
    if isinstance(cell, dict) and set(cell) != set(scenario.cell_attributes):
        out.append(f"cell: has {', '.join(sorted(cell)) or 'nothing'}; the scenario's cell "
                   f"attributes are {', '.join(scenario.cell_attributes) or 'none'}")
    values = rec.get("values")
    if isinstance(values, dict):
        from .xosc import cast  # noqa: PLC0415 - xosc imports plug-ins; the record does not
        from .dist import SpecError  # noqa: PLC0415
        for k, v in values.items():
            if k not in scenario.parameters:
                out.append(f"values.{k}: not a parameter of {scenario.path.name}")
                continue
            ptype = scenario.parameters[k][0]
            try:
                if cast(ptype, v, k) != v or type(cast(ptype, v, k)) is not type(v):
                    out.append(f"values.{k}: {v!r} is not a {ptype} as written")
            except SpecError as e:
                out.append(f"values.{e}")
    seeds = rec.get("seeds")
    if isinstance(seeds, dict) and isinstance(seeds.get("pilots"), dict):
        extra = set(seeds["pilots"]) - set(scenario.entities)
        if extra:
            out.append(f"seeds.pilots: {', '.join(sorted(extra))} not entities of the scenario")
    return out


def read(path):
    """Records from a file: JSON lines (.jsonl), or YAML documents (one or several)."""
    path = Path(path)
    text = path.read_text()
    if path.suffix == ".jsonl":
        return [json.loads(line) for line in text.splitlines() if line.strip()]
    return [d for d in yaml.safe_load_all(text) if d is not None]
