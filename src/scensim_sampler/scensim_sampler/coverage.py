"""The coverage report: sample records with their outcomes, grouped by category and cell, in
the three layers of survey-vehicle-risk-taxonomy.md section 4 (docs/design/sample-record.md,
"Coverage").

1. Category coverage: which of the catalogue's categories have runs; the uncovered, by name.
2. Condition coverage: per category, the t-wise coverage of its cell attributes' levels.
3. Criticality: per (category, cell), runs, failures and the failure rate with a 95 % interval.
   A cell drawn from a proposal gets the self-normalised estimate.

An outcome is `{failed, contact, error, seeds_not_applied, run}`. scensim's runs give it from
their result.json; another simulator's runs give it in a JSONL of `{record, outcome}`. Nothing
here needs a simulator.

Weights between categories (exposure, crash frequencies) are the study's input, not this
module's: an overall rate is given only when the catalogue names `exposure:`.
"""
import itertools
import json
import math
from pathlib import Path

import yaml

from .record import CATEGORY, FAMILIES

SCHEMA = "scensim.coverage/1"
FAILURE = ("verdict", "contact", "either")
Z = 1.959963984540054   # 95 %
# the weighted (delta-method) interval's coverage of its nominal 95 %, as measured on
# rare-event tests (sample-record.md, section 8, the table; test_coverage holds one): 0.93 to
# 0.94 with light weights, falling as the weights get heavier, 0.61 at the worst. Printed with
# the interval, and each weighted cell shows n_eff/n; under HEAVY every measured case covered
# 0.90 or less
DELTA_COVERAGE = "0.61-0.94 measured at nominal 95 %, lower with heavier weights"
HEAVY = 0.1   # n_eff / n below this: "heavy weights, likely under-covers"
MISSING_SHOWN = 20


class CoverageError(ValueError):
    pass


def wilson(k, n, z=Z):
    """The Wilson score interval for k of n (n may be an effective, non-integer count)."""
    if n <= 0:
        return None
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(max(0.0, p * (1 - p) / n + z * z / (4 * n * n))) / d
    return max(0.0, c - h), min(1.0, c + h)


# --- input ----------------------------------------------------------------------------------

def load_catalogue(path):
    """`{categories, cells, t, exposure}` from a catalogue file (format in the spec)."""
    path = Path(path)
    try:
        doc = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as e:
        raise CoverageError(f"{path}: {e}") from None
    if not isinstance(doc, dict) or doc.get("catalogue") != 1:
        raise CoverageError(f"{path}: `catalogue: 1` (the format's version) is required")
    extra = set(doc) - {"catalogue", "categories", "cells", "t", "exposure", "note"}
    if extra:
        raise CoverageError(f"{path}: unknown {sorted(extra)}")
    cats = doc.get("categories")
    if not isinstance(cats, list) or not cats:
        raise CoverageError(f"{path}: categories is a non-empty list of category ids")
    for c in cats:
        if not isinstance(c, str) or not CATEGORY.match(c):
            raise CoverageError(f"{path}: {c!r} is not R<family>.<category>.<subtype>")
    cells = doc.get("cells") or {}
    if not isinstance(cells, dict):
        raise CoverageError(f"{path}: cells is {{category: {{attribute: [levels]}}}}")
    for c, attrs in cells.items():
        if c not in cats:
            raise CoverageError(f"{path}: cells names {c}, which is not in categories")
        if (not isinstance(attrs, dict) or not attrs
                or any(not isinstance(v, list) or not v for v in attrs.values())):
            raise CoverageError(f"{path}: cells.{c}: {{attribute: [level, ...]}}, each non-empty")
    t = doc.get("t", 2)
    if isinstance(t, bool) or not isinstance(t, int) or t < 1:
        raise CoverageError(f"{path}: t is an integer >= 1")
    exposure = doc.get("exposure")
    if exposure is not None:
        if not isinstance(exposure, dict) or not exposure:
            raise CoverageError(f"{path}: exposure is {{category: weight}}")
        for c, w in exposure.items():
            if c not in cats:
                raise CoverageError(f"{path}: exposure names {c}, which is not in categories")
            if isinstance(w, bool) or not isinstance(w, (int, float)) or w < 0:
                raise CoverageError(f"{path}: exposure.{c}: a number >= 0")
    return {"categories": list(cats), "cells": cells, "t": t, "exposure": exposure,
            "path": str(path)}


def outcomes_from_jsonl(path):
    """`[(record, outcome)]` from a JSONL of `{record, outcome}` (another simulator's runs)."""
    out = []
    for i, line in enumerate(Path(path).read_text().splitlines()):
        if not line.strip():
            continue
        row = json.loads(line)
        if not isinstance(row, dict) or "record" not in row or "outcome" not in row:
            raise CoverageError(f"{path}: line {i + 1}: {{record, outcome}}")
        o = row["outcome"]
        out.append((row["record"], {"failed": bool(o.get("failed")),
                                    "contact": bool(o.get("contact")),
                                    "error": bool(o.get("error")),
                                    "seeds_not_applied": bool(o.get("seeds_not_applied")),
                                    "run": o.get("run") or f"{Path(path).name}:{i + 1}"}))
    return out


def _contact(run_dir: Path) -> bool:
    for f in sorted((run_dir / "contacts").glob("*.json")):
        try:
            if json.loads(f.read_text()).get("contacts"):
                return True
        except (OSError, ValueError, AttributeError):
            continue
    return False


def outcomes_from_runs(paths):
    """`[(record, outcome)]` from scensim run directories (or directories of them) that hold a
    sample.json; runs without one are skipped. A replay is marked (`replay`): the report keeps
    it out of every layer, since it is the same sample again, and counts it apart."""
    dirs = []
    for p in map(Path, paths):
        if (p / "result.json").is_file():
            dirs.append(p)
        elif p.is_dir():
            dirs += [c for c in sorted(p.iterdir()) if (c / "result.json").is_file()]
        else:
            raise CoverageError(f"{p}: not a run or a directory of runs")
    out = []
    for d in dirs:
        if not (d / "sample.json").is_file():
            continue
        r = json.loads((d / "result.json").read_text())
        rec = json.loads((d / "sample.json").read_text())
        notes = (r.get("sample") or {}).get("unbound") or []
        # grouped by what the world applied (093 D7): a bound knob's cell value is the value the
        # run's knobs report applied; a run whose knobs did not apply is an error run already
        kb = r.get("knobs") or {}
        if kb.get("ok") and isinstance(rec.get("cell"), dict):
            def applied(a, v):
                e = (kb.get("knobs") or {}).get(a)
                if e is None:
                    return v
                # a knob applied but not read back (issue 095) is its own cell, named so, until
                # the world reports it: not grouped with the confirmed ones (243)
                if (e.get("surface_map") or {}).get("read") == "unverified":
                    return f"{e['value']} (unverified)"
                return e["value"]
            rec = dict(rec, cell={a: applied(a, v) for a, v in rec["cell"].items()})
        out.append((rec, {"failed": r.get("verdict") == "failed", "contact": _contact(d),
                          "error": r.get("verdict") == "error",
                          "seeds_not_applied": any(n.startswith("pilot ") for n in notes),
                          "replay": bool(r.get("replay")), "run": d.name}))
    return out


# --- the report -----------------------------------------------------------------------------

def _failed(o, failure):
    if failure == "verdict":
        return o["failed"]
    if failure == "contact":
        return o["contact"]
    return o["failed"] or o["contact"]


def _cell_key(cell):
    return tuple(sorted((str(k), str(v)) for k, v in (cell or {}).items()))


def _rate(rows, failure):
    """`{n, failures, rate, interval, weighted, n_eff}` or `{refused: why}` for one group's
    judgeable rows."""
    if not rows:
        return {"n": 0, "failures": 0, "rate": None, "interval": None, "weighted": False}
    props = {((r.get("proposal") or {}).get("id"), (r.get("base") or {}).get("id"))
             for r, _ in rows}
    weighted = {r.get("proposal") is not None for r, _ in rows}
    k = sum(_failed(o, failure) for _, o in rows)
    n = len(rows)
    if weighted == {False}:
        return {"n": n, "failures": k, "rate": k / n, "interval": wilson(k, n),
                "weighted": False}
    if len(weighted) > 1:
        return {"n": n, "failures": k, "refused": "weighted and unweighted runs in one cell; "
                                                  "their rates are not blended"}
    if len(props) > 1:
        return {"n": n, "failures": k,
                "refused": "runs from more than one proposal or base: "
                           + ", ".join(sorted(f"{p}/{b}" for p, b in props))}
    lw = [float(r["log_weight"]) for r, _ in rows]
    top = max(lw)
    w = [math.exp(x - top) for x in lw]   # in log space: a small weight does not underflow
    f = [1.0 if _failed(o, failure) else 0.0 for _, o in rows]
    sw, sw2 = sum(w), sum(x * x for x in w)
    p = sum(wi * fi for wi, fi in zip(w, f)) / sw
    n_eff = sw * sw / sw2   # Kish: a diagnostic of the weights, not the interval's count
    if k == 0:
        # nothing seen: the delta variance is 0; Wilson on n_eff, conservative, as the bound
        interval, method = wilson(0.0, n_eff), "wilson-kish (no failures: an upper bound)"
    else:
        # the delta-method variance of the self-normalised estimate, sum w^2 (f-p)^2 / (sum w)^2:
        # the estimator's real spread (Kish's n_eff ignores that a rare-event proposal ties w to
        # f, and gave intervals ~8x too wide; 243). It can under-cover at small n: measured
        # 0.89-0.93 for 50-5000 draws on an N(0,1) tail from N(2,1) (sample-record.md, 8)
        se = math.sqrt(sum(wi * wi * (fi - p) ** 2 for wi, fi in zip(w, f))) / sw
        interval = (max(0.0, p - Z * se), min(1.0, p + Z * se))
        method = f"delta (normal; approximate, {DELTA_COVERAGE})"
    return {"n": n, "failures": k, "rate": p, "interval": interval, "interval_method": method,
            "weighted": True, "n_eff": n_eff, "n_eff_share": n_eff / n,
            "heavy_weights": n_eff / n < HEAVY,
            "proposal": next(iter(props))[0], "approximate": True}


def report(items, catalogue=None, failure="verdict", t=None):
    """The coverage report as a dict (schema scensim.coverage/1)."""
    if failure not in FAILURE:
        raise CoverageError(f"failure is one of {', '.join(FAILURE)}, not {failure!r}")
    items = list(items)
    replays = [o["run"] for _, o in items if o.get("replay")]
    items = [(r, o) for r, o in items if not o.get("replay")]
    t = t or (catalogue or {}).get("t") or 2
    observed = sorted({r["category"] for r, _ in items})
    cats = list((catalogue or {}).get("categories") or observed)
    by_cat = {}
    for r, o in items:
        by_cat.setdefault(r["category"], []).append((r, o))

    # layer 1
    l1 = []
    for c in cats:
        rows = by_cat.get(c, [])
        l1.append({"category": c, "runs": len(rows),
                   "judged": sum(not o["error"] for _, o in rows), "covered": bool(rows)})
    outside = [c for c in observed if c not in cats]
    families = sorted({c.split(".")[0] for c in cats})
    layer1 = {"categories": l1, "uncovered": [x["category"] for x in l1 if not x["covered"]],
              "outside_catalogue": outside,
              "families_without_a_category": [f"{f} ({FAMILIES[f]})" for f in FAMILIES
                                              if f not in families],
              "from_catalogue": catalogue is not None}

    # layer 2
    l2 = []
    for c in cats:
        rows = [(r, o) for r, o in by_cat.get(c, []) if not o["error"]]
        declared = ((catalogue or {}).get("cells") or {}).get(c)
        if declared:
            levels = {a: [str(x) for x in v] for a, v in declared.items()}
        else:
            levels = {}
            for r, _ in rows:
                for a, v in (r.get("cell") or {}).items():
                    levels.setdefault(a, [])
                    if str(v) not in levels[a]:
                        levels[a].append(str(v))
        attrs = sorted(levels)
        tt = min(t, len(attrs)) if attrs else 0
        need, have, off = set(), set(), []
        for combo in (itertools.combinations(attrs, tt) if attrs else ()):
            for lv in itertools.product(*(levels[a] for a in combo)):
                need.add(tuple(zip(combo, lv)))
        for r, o in rows:
            cell = {a: str(v) for a, v in (r.get("cell") or {}).items()}
            bad = [f"{a}={v}" for a, v in cell.items() if a in levels and v not in levels[a]]
            if bad:
                off.append({"run": o["run"], "outside": bad})
            for combo in (itertools.combinations(attrs, tt) if attrs else ()):
                if all(a in cell for a in combo):
                    have.add(tuple((a, cell[a]) for a in combo))
        missing = sorted(need - have)
        l2.append({"category": c, "t": tt, "attributes": {a: levels[a] for a in attrs},
                   "levels_from": "catalogue" if declared else "observed",
                   "tuples": len(need), "covered": len(need & have),
                   "share": (len(need & have) / len(need)) if need else None,
                   "missing": [dict(m) for m in missing], "outside_levels": off})

    # layer 3
    l3 = []
    for c in cats:
        groups = {}
        for r, o in by_cat.get(c, []):
            groups.setdefault(_cell_key(r.get("cell")), []).append((r, o))
        for key, rows in sorted(groups.items()):
            errors = [o["run"] for _, o in rows if o["error"]]
            unseeded = [o["run"] for _, o in rows if not o["error"] and o["seeds_not_applied"]]
            judged = [(r, o) for r, o in rows if not o["error"] and not o["seeds_not_applied"]]
            entry = {"category": c, "cell": dict(key), "runs": len(rows), "errors": len(errors),
                     "seeds_not_applied": unseeded, **_rate(judged, failure)}
            l3.append(entry)

    out = {"schema": SCHEMA, "failure": failure, "t": t,
           "catalogue": (catalogue or {}).get("path"),
           "records": len(items), "replays": replays,
           "errors": sum(o["error"] for _, o in items),
           "seeds_not_applied": sum(o["seeds_not_applied"] and not o["error"]
                                    for _, o in items),
           "layer1": layer1, "layer2": l2, "layer3": l3}
    if catalogue and catalogue.get("exposure"):
        out["overall"] = _overall(catalogue["exposure"], by_cat, failure)
    return out


def _overall(exposure, by_cat, failure):
    """The exposure-weighted rate over categories: sum(e_c r_c) / sum(e_c), each category's
    rate pooled over its cells, the interval by the normal approximation from each category's
    Wilson half-width. Refused when a category with exposure has no rate."""
    total = sum(exposure.values())
    if total <= 0:
        return {"refused": "the exposure weights sum to 0"}
    parts, var, missing = [], 0.0, []
    for c, e in sorted(exposure.items()):
        rows = [(r, o) for r, o in by_cat.get(c, [])
                if not o["error"] and not o["seeds_not_applied"]]
        rt = _rate(rows, failure)
        if rt.get("rate") is None:
            missing.append(c + (f" ({rt['refused']})" if rt.get("refused") else " (no runs)"))
            continue
        lo, hi = rt["interval"]
        sd = (hi - lo) / (2 * Z)
        parts.append({"category": c, "exposure": e, "rate": rt["rate"]})
        var += (e / total) ** 2 * sd * sd
    if missing:
        return {"refused": "no rate for " + "; ".join(missing)}
    rate = sum(p["exposure"] * p["rate"] for p in parts) / total
    h = Z * math.sqrt(var)
    return {"rate": rate, "interval": (max(0.0, rate - h), min(1.0, rate + h)),
            "approximate": True, "by_category": parts,
            "note": "exposure weights are the study's input (the catalogue's), not scensim's"}


# --- text -----------------------------------------------------------------------------------

def _pct(x):
    return "-" if x is None else f"{100 * x:.1f}%"


def _iv(iv):
    return "-" if not iv else f"[{_pct(iv[0])}, {_pct(iv[1])}]"


def _table(rows):
    if not rows:
        return []
    widths = [max(len(str(r[i])) for r in rows) for i in range(len(rows[0]))]
    return ["  ".join(str(c).ljust(w) for c, w in zip(r, widths)).rstrip() for r in rows]


def format_report(rep):
    lines = [f"coverage: {rep['records']} records, {rep['errors']} errors, "
             f"{rep['seeds_not_applied']} with seeds not applied; failure = {rep['failure']}"
             + (f"; {len(rep['replays'])} replays left out (the same samples again)"
                if rep["replays"] else "")]
    l1 = rep["layer1"]
    lines.append("")
    lines.append("1. categories" + ("" if l1["from_catalogue"] else
                                    " (no catalogue: the records' own)"))
    lines += _table([["category", "runs", "judged", "covered"]]
                    + [[x["category"], x["runs"], x["judged"], "yes" if x["covered"] else "NO"]
                       for x in l1["categories"]])
    if l1["uncovered"]:
        lines.append("uncovered: " + ", ".join(l1["uncovered"]))
    if l1["outside_catalogue"]:
        lines.append("outside the catalogue: " + ", ".join(l1["outside_catalogue"]))
    if l1["families_without_a_category"]:
        lines.append("families with no category: " + ", ".join(l1["families_without_a_category"]))
    lines.append("")
    lines.append(f"2. conditions (t = {rep['t']})")
    lines += _table([["category", "t", "covered", "of", "share", "levels"]]
                    + [[x["category"], x["t"], x["covered"], x["tuples"], _pct(x["share"]),
                        x["levels_from"]] for x in rep["layer2"]])
    for x in rep["layer2"]:
        for m in x["missing"][:MISSING_SHOWN]:
            lines.append(f"  missing {x['category']}: "
                         + ", ".join(f"{a}={v}" for a, v in m.items()))
        if len(x["missing"]) > MISSING_SHOWN:
            lines.append(f"  ... {len(x['missing']) - MISSING_SHOWN} more (coverage.json)")
        for o in x["outside_levels"]:
            lines.append(f"  outside the levels: {o['run']}: {', '.join(o['outside'])}")
    lines.append("")
    lines.append("3. criticality (unweighted: Wilson 95 %; weighted: self-normalised, the "
                 f"delta interval, approximate, {DELTA_COVERAGE})")
    rows = [["category", "cell", "runs", "judged", "failures", "rate", "interval", "note"]]
    for x in rep["layer3"]:
        cell = ",".join(f"{k}={v}" for k, v in x["cell"].items()) or "-"
        note = x.get("refused") or (
            f"weighted, n_eff/n {x['n_eff_share']:.2f}"
            + ("; an upper bound" if "upper bound" in x.get("interval_method", "") else
               "; heavy weights: likely under-covers" if x.get("heavy_weights") else "")
            if x.get("weighted") else "")
        if x["seeds_not_applied"]:
            note = (note + "; " if note else "") + f"{len(x['seeds_not_applied'])} seeds not applied"
        if x["errors"]:
            note = (note + "; " if note else "") + f"{x['errors']} errors"
        rows.append([x["category"], cell, x["runs"], x["n"], x["failures"],
                     _pct(x.get("rate")), _iv(x.get("interval")), note])
    lines += _table(rows)
    unseeded = [r for x in rep["layer3"] for r in x["seeds_not_applied"]]
    if unseeded:
        lines.append("seeds not applied (not in the rates): " + ", ".join(unseeded))
    if "overall" in rep:
        o = rep["overall"]
        lines.append("")
        if o.get("refused"):
            lines.append(f"overall: refused ({o['refused']})")
        else:
            lines.append(f"overall, exposure-weighted: {_pct(o['rate'])} {_iv(o['interval'])} "
                         f"(approximate; {o['note']})")
    return "\n".join(lines) + "\n"
