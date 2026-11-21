"""Sampled runs (roadmap 092 K3): a suite's `sample:` draws sample records with
scensim_sampler (docs/design/sample-record.md), one run per record; a run binds its record to
the world and car that ran it; `scensim run --replay` runs a record again; `scensim runs
fidelity` compares originals with their replays.

    sample:                                   # a suite key; or a list of these
      distribution: scenarios/walker_dist.xosc
      n: 20
      seed: 7
      proposal: scenarios/near_ttc_short.yaml # optional
      knobs: {speed_limit: 13.9}              # optional

Seeds: a record's pilot seed reaches its entity's pilot as `{seed}`; its scenario seed reaches
SSv2 as the parameter `scensim_seed` when the scenario declares one; its world seed is recorded
only (no world server takes a per-run seed; issue 092).
"""
import hashlib
import json
from pathlib import Path

import yaml

SAMPLE_KEYS = ("distribution", "n", "seed", "proposal", "knobs")
SCENARIO_SEED = "scensim_seed"
RECORD_FILE = "sample.json"


class SampleRefused(ValueError):
    pass


def _sampler():
    try:
        import scensim_sampler  # noqa: PLC0415 - only a sampled suite needs it
        from scensim_sampler import xosc  # noqa: PLC0415
    except ImportError:
        raise SampleRefused("sample: needs scensim_sampler (src/scensim_sampler; just build)") \
            from None
    return scensim_sampler, xosc


def entries(value, where):
    """The suite's `sample:` as a list of mappings, each checked; problems raise."""
    items = value if isinstance(value, list) else [value]
    out = []
    for i, s in enumerate(items):
        at = f"{where}: sample" + (f"[{i}]" if isinstance(value, list) else "")
        if not isinstance(s, dict):
            raise SampleRefused(f"{at}: a mapping {{distribution, n, seed, proposal?, knobs?}}")
        bad = [k for k in s if k not in SAMPLE_KEYS]
        if bad:
            raise SampleRefused(f"{at}: unknown key {bad[0]} (use only {', '.join(SAMPLE_KEYS)})")
        if not isinstance(s.get("distribution"), str):
            raise SampleRefused(f"{at}: distribution: the ParameterValueDistribution file")
        for k in ("n", "seed"):
            v = s.get(k)
            if v is not None and (isinstance(v, bool) or not isinstance(v, int) or v < 0
                                  or (k == "n" and v < 1)):
                raise SampleRefused(f"{at}: {k}: an integer{' >= 1' if k == 'n' else ' >= 0'}, "
                                    f"not {v!r}")
        if s.get("knobs") is not None and not isinstance(s["knobs"], dict):
            raise SampleRefused(f"{at}: knobs: {{name: value}}")
        out.append((at, s))
    return out


def draw(value, resolve, where):
    """`[(record, scenario path)]` for a suite's `sample:`. `resolve(str) -> Path` resolves a
    suite-relative path."""
    ss, xosc = _sampler()
    out = []
    for at, s in entries(value, where):
        dist = Path(resolve(s["distribution"])).resolve()
        if not dist.is_file():
            raise SampleRefused(f"{at}: distribution {s['distribution']}: no such file")
        proposal = resolve(s["proposal"]) if s.get("proposal") else None
        try:
            sampler = ss.Sampler.from_file(dist, proposal=proposal, knobs=s.get("knobs"))
            batch = sampler.draw(n=s.get("n"), seed=s.get("seed"))
        except (ss.SpecError, ss.SamplingError, ValueError) as e:
            raise SampleRefused(f"{at}: {e}") from None
        for r in batch.records:
            # the sampler names the scenario as it opened it, dist.parent / ScenarioFile, so
            # absolute here (dist is); a relative one is the distribution's, not the cwd's
            sc = Path(r["scenario"]["path"])
            out.append((r, (sc if sc.is_absolute() else dist.parent / sc).resolve()))
    return out


def parameters(record, scenario):
    """The record's values as SSv2's override parameters, and `scensim_seed` when the scenario
    declares it."""
    _, xosc = _sampler()
    params = dict(record["values"])
    try:
        declared = xosc.Scenario(scenario).parameters
    except ValueError:
        declared = {}
    if SCENARIO_SEED in declared:
        params[SCENARIO_SEED] = record["seeds"]["scenario"]
    return params


def pilot_seed(record, entity, default):
    """The seed an entity's pilot gets: the record's for it, else the run's."""
    if not record:
        return default
    return (record.get("seeds") or {}).get("pilots", {}).get(entity, default)


# --- binding a record to what ran it --------------------------------------------------------

def _file_sha256(p: Path):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def map_digest(map_dir: Path):
    """sha256 over the map directory's files the stack reads (lanelet2_map.osm and carla/**):
    each `relpath sha256` line, sorted. The carla/ part changes the world without touching the
    lanelet (traffic_lights.yaml: issue 073's junction 861), so it counts. None without a
    lanelet2 file."""
    map_dir = Path(map_dir)
    if not (map_dir / "lanelet2_map.osm").is_file():
        return None
    files = [map_dir / "lanelet2_map.osm"]
    if (map_dir / "carla").is_dir():
        files += [f for f in (map_dir / "carla").rglob("*") if f.is_file()]
    lines = sorted(f"{f.relative_to(map_dir).as_posix()} {_file_sha256(f)}" for f in files)
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()


def world_of(rec: dict, prov: dict):
    """`{name, hash}` of the world a run used. Isaac: the authored stage's record (091 W15).
    CARLA: a town has no content hash, so the map directory's digest (map_digest) stands for
    it, with the simulator's version."""
    name = str(rec.get("map") or "")
    stage = (prov or {}).get("stage") or {}
    if isinstance(stage, dict) and isinstance(stage.get("sha256"), str):
        return {"name": name or str(stage.get("root") or "stage"),
                "hash": f"stage:sha256:{stage['sha256']}"}
    sim = str(rec.get("simulator") or "unknown")
    version = ((prov or {}).get("carla") or {}).get("version")
    mp = rec.get("map_path")
    digest = map_digest(mp) if mp else None
    tag = f"{sim}-{version}" if version else sim
    return {"name": name or "unknown",
            "hash": f"{tag}:{name}:" + (f"map:sha256:{digest}" if digest else "unhashed")}


def car_of(rec: dict, pilots):
    """`{name, spec_hash}` of the ego's car, or None when the stack names no spec_hash."""
    ego = next((p for p in pilots if p.get("entity") == "ego"), None)
    if ego is None:
        return None
    h = (rec.get("spec_hash") or {}).get("ego")
    if not ego.get("vehicle") or not isinstance(h, str) or len(h) != 64:
        return None
    return {"name": ego["vehicle"], "spec_hash": h}


def applied_seeds(record, pilots):
    """`({entity: seed}, notes)`: the seed each pilot ran with. A `lifetime: run` pilot gets
    the record's; a stack pilot was started before the run, with the stack's seed (0 unless
    its spec gives one), and the record's for it is not applied."""
    want = (record.get("seeds") or {}).get("pilots") or {}
    out, notes = {}, []
    for p in pilots:
        e = p.get("entity")
        if p.get("lifetime") == "run":
            out[e] = want.get(e)
            continue
        have = p.get("seed", 0)
        out[e] = have
        if e in want and want[e] != have:
            notes.append(f"pilot {e}: a stack pilot, its seed {want[e]} not applied; it ran "
                         f"with the stack's ({have})")
    return out, notes


def bind(record, rec, prov, pilots):
    """`(bound record, notes)`: the record with world and car filled and its id recomputed;
    notes say what could not be bound. The record keeps the seeds drawn (what a replay asks
    for); which reached the pilots is applied_seeds'."""
    ss, _ = _sampler()
    out = {k: v for k, v in record.items() if k not in ("id", "world", "car")}
    notes = []
    out["world"] = world_of(rec, prov)
    if out["world"]["hash"].endswith(":unhashed"):
        notes.append(f"world {out['world']['name']}: no lanelet2 map to hash")
    car = car_of(rec, pilots)
    if car is not None:
        out["car"] = car
    else:
        notes.append("car: the stack names no spec_hash for the ego (or there is no ego)")
    out["id"] = ss.record_id(out)
    keys = ("record", "id", "scenario", "distribution", "category", "cell", "values", "derived",
            "seeds", "base", "proposal", "weight", "log_weight", "search", "world", "car",
            "sampler")
    return {k: out[k] for k in keys if k in out}, notes


def summary(record, bound):
    """result.json's `sample`: what a reader needs without opening sample.json."""
    return {"id": bound["id"], "drawn_id": record["id"],
            "index": record["sampler"]["index"], "run_seed": record["sampler"]["run_seed"],
            "category": record["category"], "cell": record["cell"],
            "base": (record.get("base") or {}).get("id"),
            "proposal": (record.get("proposal") or {}).get("id"),
            "weight": record["weight"], "log_weight": record["log_weight"],
            "file": RECORD_FILE}


# --- replay ---------------------------------------------------------------------------------

def load_record(target, index=None):
    """`(record, of_run)` from a run directory (its sample.json), a .json or .yaml record, or
    a .jsonl file (`index`: which line, required when it holds more than one)."""
    p = Path(target)
    of_run = None
    if p.is_dir():
        f = p / RECORD_FILE
        if not f.is_file():
            raise SampleRefused(f"{p}: no {RECORD_FILE} (not a sampled run)")
        of_run, p = p.name, f
    if not p.is_file():
        raise SampleRefused(f"{target}: no such file or run directory")
    text = p.read_text()
    if p.suffix == ".jsonl":
        rows = [json.loads(line) for line in text.splitlines() if line.strip()]
    elif p.suffix == ".json":
        rows = [json.loads(text)]
    else:
        rows = [d for d in yaml.safe_load_all(text) if d is not None]
    if index is None:
        if len(rows) != 1:
            raise SampleRefused(f"{p}: {len(rows)} records; give --index")
        index = 0
    if not 0 <= index < len(rows):
        raise SampleRefused(f"{p}: no record {index} (it holds {len(rows)})")
    r = rows[index]
    ss, _ = _sampler()
    problems = ss.check(r)
    if problems:
        raise SampleRefused(f"{p}: record {index}: {problems[0]}")
    return r, of_run


def replay_scenario(record, base: Path):
    """The scenario file a record names, found and checked against its sha256."""
    sc = record["scenario"]
    cands = [Path(sc["path"])]
    if not cands[0].is_absolute():
        cands = [base / sc["path"], Path.cwd() / sc["path"]]
    for c in cands:
        if c.is_file():
            h = _file_sha256(c)
            if h != sc["sha256"]:
                raise SampleRefused(f"{c} has changed since the record was drawn: sha256 {h}, "
                                    f"the record names {sc['sha256']}")
            return c.resolve()
    raise SampleRefused(f"the record's scenario {sc['path']} is not found (looked in "
                        f"{', '.join(str(c) for c in cands)})")


def unbound(record):
    """A bound record as drawn: without world and car (its drawn id is not kept, the id is
    recomputed)."""
    ss, _ = _sampler()
    out = {k: v for k, v in record.items() if k not in ("id", "world", "car")}
    out["id"] = ss.record_id(out)
    return out


# --- fidelity -------------------------------------------------------------------------------

def outcome(run_dir: Path):
    """`{verdict, contact, end}` of a run: its verdict, whether a contact report names the
    ego, and how it ended (SSv2's JUnit outcome, else the first stopping event, else the
    reason's first words)."""
    from scensim import runs as runsmod  # noqa: PLC0415
    r = json.loads((Path(run_dir) / "result.json").read_text())
    ssv2 = r.get("ssv2") or {}
    stop_events = [e["type"] for e in r.get("events") or []
                   if e.get("type", "").startswith(("collision.", "stuck", "route_complete",
                                                    "stack.error"))]
    if ssv2.get("failures"):
        end = "ssv2.failure"
    elif ssv2.get("errors"):
        end = "ssv2.error"
    elif stop_events:
        end = stop_events[0]
    elif r.get("verdict") == "passed":
        end = "ssv2.success"
    else:
        end = (r.get("reason") or "").split(":")[0][:60] or "unknown"
    return {"verdict": r.get("verdict"), "contact": runsmod.has_contact(run_dir), "end": end}


def _key(run_dir: Path):
    """The record a run is of: a replay's original id, else its own drawn id."""
    try:
        r = json.loads((Path(run_dir) / "result.json").read_text())
    except (OSError, ValueError):
        return None, None
    rep = r.get("replay") or {}
    s = r.get("sample") or {}
    return (rep.get("of") or s.get("drawn_id")), bool(rep)


def _unapplied(run_dir):
    try:
        r = json.loads((Path(run_dir) / "result.json").read_text())
    except (OSError, ValueError):
        return []
    return [n for n in (r.get("sample") or {}).get("unbound") or [] if n.startswith("pilot ")]


def fidelity(dirs):
    """Pair each original with its replays by record and compare outcomes. Returns
    `{pairs: [...], agree, total, share}`."""
    originals, replays = {}, {}
    for d in dirs:
        k, is_replay = _key(d)
        if k is None:
            continue
        (replays.setdefault(k, []) if is_replay else originals.setdefault(k, [])).append(d)
    pairs = []
    for k, reps in sorted(replays.items()):
        for o in originals.get(k, []):
            a = outcome(o)
            for rd in reps:
                b = outcome(rd)
                diff = [f for f in ("verdict", "contact", "end") if a[f] != b[f]]
                # a seed the record names that did not reach a pilot: the two runs are not
                # the same sample, so a difference is not a fidelity gap
                unapplied = _unapplied(o) + _unapplied(rd)
                pairs.append({"record": k, "original": Path(o).name, "replay": Path(rd).name,
                              "original_outcome": a, "replay_outcome": b, "differs": diff,
                              "seeds_not_applied": unapplied})
    agree = sum(not p["differs"] for p in pairs)
    return {"pairs": pairs, "agree": agree, "total": len(pairs),
            "with_seeds_not_applied": sum(bool(p["seeds_not_applied"]) for p in pairs),
            "share": (agree / len(pairs)) if pairs else None,
            "unpaired_replays": sorted(k for k in replays if k not in originals)}
