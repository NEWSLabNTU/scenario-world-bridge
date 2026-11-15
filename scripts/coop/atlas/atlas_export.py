#!/usr/bin/env python3
"""Roadmap 074, the strategy atlas: every result registered in sources.yaml, normalized into
one atlas.json for atlas_build.py.

Per scenario: its context axes and facets, per tier the cells (class, equilibria, welfare and
the equilibrium's loss C_eq - C_opt), the payoffs per rule pair (U_A, U_B) for the payoff view,
the rules with their parameters, the double oracle's rounds, and episodes (poses of A and B
over time with 2D TTC, clearance and headway, PET as a value). Episodes come from the agents'
logs (swb: coop_a.jsonl, x, y, yaw of both cars) or are re-simulated with the kinematic games'
simulate() for the cells sources.yaml names, by recording the poses each game hands to
measures.Measures.push_poses. Re-simulations are cached under <out>/cache.

A loader is a function `load_<name>(entry, scenario) -> dict` that returns any of
{"cells": [...], "payoffs": [...], "meta": {...}, "episodes": [...], "rules": {...},
"progress": {...}}; adding a result is adding an entry to sources.yaml, adding a format is
adding a loader here.

Usage:
  scripts/coop/atlas/atlas_export.py --sources scripts/coop/atlas/sources.yaml --out <dir>
"""

from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import math
import os
import re
import statistics as st
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
COOP = HERE.parent
sys.path.insert(0, str(COOP))

import yaml  # noqa: E402

import measures as M  # noqa: E402

CAR_L, CAR_W = 4.79, 2.163  # the spec's Model 3 (vehicle_info), for agent logs without a size
EP_DT = 0.2  # s between episode samples in atlas.json (downsampled)
CACHE_VERSION = 2  # bump when a re-simulation changes what it returns


# --- helpers -------------------------------------------------------------------------------------
def num(x):
    """A CSV/JSON field as a float, None when empty; inf stays inf (JSON gets None)."""
    if x is None or x == "":
        return None
    try:
        v = float(x)
    except (TypeError, ValueError):
        return x
    return v


def clean(x):
    """JSON-safe: inf and nan become None, floats rounded."""
    if isinstance(x, float):
        if math.isinf(x) or math.isnan(x):
            return None
        return round(x, 4)
    if isinstance(x, dict):
        return {str(k): clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [clean(v) for v in x]
    return x


def paths(entry):
    p = entry.get("path")
    ps = p if isinstance(p, list) else [p]
    out = []
    for q in ps:
        q = os.path.expanduser(str(q))
        out += sorted(glob.glob(q)) if any(c in q for c in "*?[") else [q]
    return out


def ctx_of(row, axes, facets, fixed=None):
    c = {k: num(row[k]) for k in list(axes) + list(facets) if k in row}
    c.update(fixed or {})
    return c


def ctx_key(c):
    return json.dumps(c, sort_keys=True)


def read_csv(p, filt=None):
    rows = list(csv.DictReader(open(p)))
    for k, v in (filt or {}).items():
        rows = [r for r in rows if str(r.get(k)) == str(v) or num(r.get(k)) == num(v)]
    return rows


def read_jsonl(p):
    return [json.loads(l) for l in open(p) if l.strip()]


def parse_pairs(s, sep="/"):
    """'a/b a/b' -> [[a, b], ...]; a B rule may carry ':cont' (crossing's react:yield)."""
    out = []
    for tok in (s or "").split():
        if sep in tok:
            a, b = tok.split(sep, 1)
            out.append([a, b.split(":")[0]])
    return out


def pure_ne(ua, ub, a_acts, b_acts, eps=1e-9):
    """Pure Nash equilibria of a bimatrix {(a, b): u}."""
    ne = []
    for a in a_acts:
        for b in b_acts:
            if (a, b) not in ua:
                continue
            if all(ua[(a, b)] >= ua[(x, b)] - eps for x in a_acts if (x, b) in ua) and \
               all(ub[(a, b)] >= ub[(a, y)] - eps for y in b_acts if (a, y) in ub):
                ne.append([a, b])
    return ne


# --- kinematic maps ------------------------------------------------------------------------------
CLASS_FNS = {}


def class_fn(name):
    def deco(f):
        CLASS_FNS[name] = f
        return f
    return deco


@class_fn("cutout_cell")
def _cutout_cell(row):
    import cutout_game as O
    if num(row.get("contact_eq")):
        return "!"
    r = num(row.get("rhoB_eq"))
    if r is None:
        return "?"
    return "#" if r >= 0.5 else "+" if r >= O.RISK_OK else "."


def load_kinematic_map(entry, sc):
    """cutin_game / crossing_game / cutout_game / merge_game *_map.csv: one row per context."""
    cells, payoffs = [], []
    axes, facets = sc["axes"], sc.get("facets", [])
    ra, rb = sc.get("rules", {}).get("A", []), sc.get("rules", {}).get("B", [])
    names_a = [r["name"] if isinstance(r, dict) else r for r in ra]
    names_b = [r["name"] if isinstance(r, dict) else r for r in rb]
    for p in paths(entry):
        for row in read_csv(p, entry.get("filter")):
            c = ctx_of(row, axes, facets, entry.get("fixed"))
            cls = CLASS_FNS[entry["class_fn"]](row) if entry.get("class_fn") else row.get(entry.get("class_col", "class"))
            eq = parse_pairs(row.get(entry.get("eq_col", "pure"), ""))
            c_opt, c_eq = num(row.get("C_opt")), num(row.get("C_worst_eq"))
            cell = {"ctx": c, "class": cls, "eq": eq, "C_opt": c_opt, "C_eq": c_eq,
                    "loss": None if c_opt is None or c_eq is None or isinstance(c_eq, str) else c_eq - c_opt,
                    "n_mixed": num(row.get(entry.get("mixed_col", "n_mixed"))) or (1 if row.get("mixed") else 0)}
            cells.append(cell)
            if entry.get("payoff_cols") and names_a and names_b:  # cutin: uA_<a>_<b>, uB_<a>_<b>
                pairs = []
                for a in names_a:
                    for b in names_b:
                        ka, kb = f"uA_{a}_{b}", f"uB_{a}_{b}"
                        if ka in row and kb in row:
                            pairs.append([a, b, num(row[ka]), num(row[kb])])
                if pairs:
                    payoffs.append({"ctx": c, "pairs": pairs, "eq": eq})
    return {"cells": cells, "payoffs": payoffs}


# --- kinematic re-simulation: payoffs per rule pair and episodes ---------------------------------
class PoseRecorder:
    """Records every pose pair a game hands to measures.Measures.push_poses (per instance)."""

    def __init__(self):
        self.log = {}
        self._orig = None

    def __enter__(self):
        self._orig = M.Measures.push_poses
        rec = self

        def push(m, t, pa, pb):
            rec.log.setdefault(id(m), []).append((t, tuple(pa), tuple(pb)))
            return rec._orig(m, t, pa, pb)
        M.Measures.push_poses = push
        return self

    def __exit__(self, *a):
        M.Measures.push_poses = self._orig

    def first(self):
        return next(iter(self.log.values()), [])


def _cache(out, key, fn):
    d = os.path.join(out, "cache")
    os.makedirs(d, exist_ok=True)
    h = hashlib.sha1(json.dumps([CACHE_VERSION, key], sort_keys=True).encode()).hexdigest()[:16]
    p = os.path.join(d, f"{h}.json")
    if os.path.exists(p):
        return json.load(open(p))
    v = fn()
    json.dump(v, open(p, "w"))
    return v


def _game_payoffs(game, ctx):
    """[(a, b, uA, uB)], eq, from the kinematic game at one context (every rule pair)."""
    if game == "crossing":
        import crossing_game as X
        outs = X.outcomes(ctx["dt"], ctx["v"], ctx.get("vb") or ctx["v"])
        res, ua, ub, pure, _ = X.play(outs, ctx.get("r", "none"))
    elif game == "merge":
        import merge_game as G
        outs = G.outcomes(ctx["d_end"], ctx["d0"], ctx["dv"], ctx["vb"])
        res, ua, ub, pure, _ = G.play(outs)
    elif game == "cutout":
        import cutout_game as O
        v = ctx["v"]
        v_adj = v if ctx.get("adj", "same") == "same" else O.ADJ_SLOW * v
        outs = {(tl, tb): O.simulate(tl, tb, ctx["h0"], v, ctx["O"], ctx.get("look", "off"), v_adj, ctx.get("t_lc", 3.0))
                for tl in O.L_TIMINGS for tb in O.B_GAPS}
        ua = {(f"{k[0]:g}", f"{k[1]:g}"): o.uL for k, o in outs.items()}
        ub = {(f"{k[0]:g}", f"{k[1]:g}"): o.uB for k, o in outs.items()}
        pure = pure_ne(ua, ub, [f"{t:g}" for t in O.L_TIMINGS], [f"{t:g}" for t in O.B_GAPS])
    else:
        raise ValueError(game)
    return [[a, b, ua[(a, b)], ub[(a, b)]] for (a, b) in ua], [list(p) for p in pure]


def _rule(rules, name):
    for r in rules:
        if r[0] == name:
            return r
    raise KeyError(name)


def _game_episode(game, ctx, a, b, cont="hold"):
    """Poses of A and B (the first measured pair) for one rule pair at one context."""
    with PoseRecorder() as rec:
        if game == "cutin":
            import cutin_game as C1
            cap = C1.TYPES["car"]
            C1.simulate(a, b, ctx["d0"], ctx["dv"], ctx.get("vb", 12.0), cap, cap, cont)
        elif game == "crossing":
            import crossing_game as X
            X.simulate(_rule(X.A_RULES, a), _rule(X.B_RULES, b), ctx["dt"], ctx["v"], ctx.get("vb") or ctx["v"], cont)
        elif game == "merge":
            import merge_game as G
            G.simulate(_rule(G.A_RULES, a), _rule(G.B_RULES, b), ctx["d_end"], ctx["d0"], ctx["dv"], ctx["vb"], cont)
        elif game == "cutout":
            import cutout_game as O
            v = ctx["v"]
            v_adj = v if ctx.get("adj", "same") == "same" else O.ADJ_SLOW * v
            O.simulate(float(a), float(b), ctx["h0"], v, ctx["O"], ctx.get("look", "off"), v_adj, ctx.get("t_lc", 3.0))
        else:
            raise ValueError(game)
    if game == "cutout":  # the second pair measured is (B, L): A in the atlas is L, the car that moves
        log = list(rec.log.values())[1]
        return [[t, list(pb[:7]), list(pa[:7])] for t, pa, pb in log]
    log = rec.first()
    return [[t, list(pa[:7]), list(pb[:7])] for t, pa, pb in log]


def episode_from_poses(samples, meta):
    """samples: [(t, pose_A, pose_B)], pose (x, y, yaw, vx, vy, length, width). Downsampled to
    EP_DT, with 2D TTC, clearance and headway per sample and PET over the run."""
    if not samples:
        return None
    m = M.Measures()
    for t, pa, pb in samples:
        m.push_poses(t, tuple(pa), tuple(pb))
    pet = m._pet()
    out = {"t": [], "A": [], "B": [], "ttc": [], "clear": [], "thw": [], **meta}
    h0a, h0b = samples[0][1][2], samples[0][2][2]
    t_next = samples[0][0]
    for t, pa, pb in samples:
        if t + 1e-9 < t_next:
            continue
        t_next = t + EP_DT
        ca, cb = M.corners(*pa[:3], *pa[5:7]), M.corners(*pb[:3], *pb[5:7])
        clear = max(0.0, M.clearance(ca, cb))
        hs = [h for h in (M.headway(pa, pb), M.headway(pb, pa)) if h is not None]
        out["t"].append(round(t, 2))
        for k, p, h0 in (("A", pa, h0a), ("B", pb, h0b)):
            s = p[0] * math.cos(h0) + p[1] * math.sin(h0)  # along the car's initial heading
            out[k].append([round(p[0], 2), round(p[1], 2), round(p[2], 3), round(s, 2), round(math.hypot(p[3], p[4]), 2)])
        out["ttc"].append(M.ttc2d(pa, pb, d_now=clear))
        out["clear"].append(clear)
        out["thw"].append(min(hs) if hs else None)
    out["pet"] = pet
    out["refs"] = {"ttc": M.TAU_REF, "thw": M.H_REF, "pet": M.PET_REF, "clear": M.D_REF}
    return clean(out)


def load_kinematic_resim(entry, sc, out):
    """Payoffs per rule pair at the contexts entry['contexts'] (or a sample of the map's), and
    episodes for entry['episodes'] [{ctx, a, b, cont}]."""
    game = entry["game"]
    res = {"payoffs": [], "episodes": []}
    ctxs = list(entry.get("contexts", []))
    for c in ctxs:
        c = {**entry.get("fixed", {}), **c}
        if game == "cutin":
            continue  # cutin's map has every pair's payoffs
        pairs, eq = _cache(out, ["pay", game, c], lambda: _game_payoffs(game, c))
        res["payoffs"].append({"ctx": c, "pairs": pairs, "eq": eq})
    for e in entry.get("episodes", []):
        c = {**entry.get("fixed", {}), **e["ctx"]}
        smp = _cache(out, ["ep", game, c, e["a"], e["b"], e.get("cont", "hold")],
                     lambda: _game_episode(game, c, e["a"], e["b"], e.get("cont", "hold")))
        ep = episode_from_poses(smp, {"id": f"kin {e['a']}/{e['b']} {ctx_label(c)}", "tier": "kinematic", "ctx": c,
                                      "a": e["a"], "b": e["b"], "group": e.get("group", ctx_label(c))})
        if ep:
            res["episodes"].append(ep)
    return res


def ctx_label(c):
    return " ".join(f"{k} {v:g}" if isinstance(v, (int, float)) else f"{k} {v}" for k, v in c.items())


# --- arena ---------------------------------------------------------------------------------------
def load_arena_runs(entry, sc):
    """An arena runs.csv (crossing_arena, cutout_arena) or arena.jsonl: mean payoffs per
    (context, a, b); classes from a summary.json beside it (`summary`), else pure NE only."""
    axes, facets = sc["axes"], sc.get("facets", [])
    acol, bcol = entry.get("a_col", "a"), entry.get("b_col", "b")
    uac, ubc = entry.get("ua_col", "uA"), entry.get("ub_col", "uB")
    fmt = entry.get("rule_fmt", "{}")
    by = {}
    for p in paths(entry):
        rows = read_jsonl(p) if p.endswith(".jsonl") else read_csv(p, entry.get("filter"))
        for r in rows:
            c = ctx_of(r, axes, facets, entry.get("fixed"))
            a, b = fmt.format(num(r[acol]) if entry.get("rule_num") else r[acol]), r[bcol]
            if entry.get("rule_num"):
                b = fmt.format(num(r[bcol]))
            by.setdefault(ctx_key(c), {"ctx": c, "u": {}})["u"].setdefault((a, b, r.get("b_cont") or ""), []).append(
                (num(r[uac]), num(r[ubc]), num(r.get("contact", 0)) or 0))
    payoffs, cells = [], []
    for k, v in by.items():
        # B's react continuation (yield or hold, run separately): the one B prefers, as the games choose it
        best = {}
        for (a, b, cont), xs in v["u"].items():
            ubm = st.mean(x[1] for x in xs)
            if (a, b) not in best or ubm > best[(a, b)][0]:
                best[(a, b)] = (ubm, xs)
        v["u"] = {ab: xs for ab, (_, xs) in best.items()}
        ua = {ab: st.mean(x[0] for x in xs) for ab, xs in v["u"].items()}
        ub = {ab: st.mean(x[1] for x in xs) for ab, xs in v["u"].items()}
        acts_a = sorted({a for a, _ in ua})
        acts_b = sorted({b for _, b in ua})
        eq = pure_ne(ua, ub, acts_a, acts_b)
        payoffs.append({"ctx": v["ctx"], "pairs": [[a, b, ua[(a, b)], ub[(a, b)]] for a, b in ua], "eq": eq,
                        "n": sum(len(xs) for xs in v["u"].values())})
        cost = {ab: -(ua[ab] + ub[ab]) for ab in ua}
        c_opt = min(cost.values())
        c_eq = max((cost[tuple(e)] for e in eq), default=None)
        cells.append({"ctx": v["ctx"], "class": None, "eq": eq, "C_opt": c_opt, "C_eq": c_eq,
                      "loss": None if c_eq is None else c_eq - c_opt})
    if entry.get("summary"):
        classes = {}
        for p in glob.glob(os.path.expanduser(entry["summary"])):
            for c in json.load(open(p)).get("cells", []):
                v = c.get(entry.get("summary_class", "class_arena"))
                if v is None:
                    continue
                cls, eq = (v.split(" ", 1) + [""])[:2] if entry.get("summary_split") else (v, c.get("eq_arena", ""))
                key = {k: num(c[k]) if k in c else None for k in entry.get("summary_keys", axes)}
                classes[ctx_key(key)] = (cls, eq)
        for cell in cells:
            key = {k: cell["ctx"].get(k) for k in entry.get("summary_keys", axes)}
            if ctx_key(key) in classes:
                cls, eq = classes[ctx_key(key)]
                cell["class"] = cls
                if entry.get("summary_split"):
                    cell["eq_summary"] = eq
    for cell in cells:
        if cell["class"] is None:
            cell["class"] = "?" if not cell["eq"] else "NE"
    return {"cells": cells, "payoffs": payoffs}


def load_arena_cutin(entry, sc):
    """C2a's arena grid (runs.csv of arena.py): classes by arena_game.classify, payoffs per pair
    with B's react continuation chosen as in the kinematic game."""
    import arena_game as AG
    import cutin_game as C1
    cells_all = AG.load(paths(entry))
    by_ctx = {}
    for (d0, dv, a, b), rows in cells_all.items():
        by_ctx.setdefault((d0, dv), {})[(a, b)] = rows
    cells, payoffs = [], []
    for (d0, dv), cc in sorted(by_ctx.items()):
        try:
            lab, pure = AG.classify(cc)
        except KeyError:
            continue
        c = {"d0": d0, "dv": dv, **entry.get("fixed", {})}
        pairs = []
        for a in C1.A_STRATS:
            ry, rh = cc[(a, "react_yield")], cc[(a, "react_hold")]
            react = ry if AG.mean(ry, "uB") >= AG.mean(rh, "uB") else rh
            for b, rows in ((C1.PRE_YIELD, cc[(a, "pre_yield")]), (C1.REACT, react)):
                pairs.append([a, b, AG.mean(rows, "uA"), AG.mean(rows, "uB")])
        eq = [list(p) for p in pure]
        cost = {(p[0], p[1]): -(p[2] + p[3]) for p in pairs}
        c_opt = min(cost.values())
        c_eq = max((cost[tuple(e)] for e in eq), default=None)
        cells.append({"ctx": c, "class": lab, "eq": eq, "C_opt": c_opt, "C_eq": c_eq,
                      "loss": None if c_eq is None else c_eq - c_opt})
        payoffs.append({"ctx": c, "pairs": pairs, "eq": eq})
    return {"cells": cells, "payoffs": payoffs}


# --- swb (C2b light rules, C3 Autoware) ---------------------------------------------------------
def load_c3(entry, sc):
    """c3.jsonl / c2b.jsonl: one row per swb run (d0, dv, a, b, seed, uA, uB, contact, ahead). The
    cell's class is the outcome of the fixed pair (B is one strategy): W A behind, g A ahead, !
    contact; payoff points per (context, A rule). Episodes from <dir>/d<d0>_v<dv>_<a>_aw_s<k>/coop_a.jsonl."""
    by = {}
    for p in paths(entry):
        for r in read_jsonl(p):
            if r.get("scenario_rc", 0) not in (0, None):
                continue
            c = {"d0": num(r["d0"]), "dv": num(r["dv"]), **entry.get("fixed", {})}
            b = entry.get("b_name") or r["b"]
            by.setdefault(ctx_key(c), {"ctx": c, "u": {}})["u"].setdefault((r["a"], b), []).append(r)
    cells, payoffs = [], []
    for v in by.values():
        pairs = []
        outcomes = []
        for (a, b), rs in v["u"].items():
            pairs.append([a, b, st.mean(r["uA"] for r in rs), st.mean(r["uB"] for r in rs)])
            outcomes += ["!" if r.get("contact") else ("g" if r.get("ahead") else "W") for r in rs]
        cls = max(set(outcomes), key=outcomes.count) if len(set(outcomes)) == 1 else "C"
        cells.append({"ctx": v["ctx"], "class": cls, "eq": [], "n": len(outcomes),
                      "outcomes": "".join(sorted(outcomes))})
        payoffs.append({"ctx": v["ctx"], "pairs": pairs, "eq": [], "aw": [pairs[0][0], pairs[0][1]] if pairs else None,
                        "label": entry.get("label")})
    episodes = []
    for d in entry.get("episode_dirs", []):
        for p in sorted(glob.glob(os.path.join(os.path.expanduser(d), "*", "coop_a.jsonl")))[: entry.get("max_episodes", 8)]:
            ep = episode_from_agent_log(p, entry.get("label", "swb"))
            if ep:
                episodes.append(ep)
    return {"cells": cells, "payoffs": payoffs, "episodes": episodes}


def episode_from_agent_log(p, tier):
    """The swb cut-in agent's log: x, y, yaw of A and ox, oy, oyaw of B (Autoware), speeds v, ov."""
    rows = [r for r in read_jsonl(p) if "t" in r and r.get("phase") not in ("warmup", None)]
    if not rows:
        return None
    name = Path(p).parent.name  # d6_v0_A_eq_aw_s3
    m = re.match(r"d(-?[\d.]+)_v(-?[\d.]+)_(.+)_aw_s(\d+)", name)
    ctx = {"d0": float(m.group(1)), "dv": float(m.group(2))} if m else {}
    a = m.group(3) if m else "?"
    t0 = rows[0]["t"]
    smp = []
    for r in rows:
        va, vb = r.get("v", 0.0), r.get("ov", 0.0)
        pa = (r["x"], r["y"], r["yaw"], va * math.cos(r["yaw"]), va * math.sin(r["yaw"]), CAR_L, CAR_W)
        pb = (r["ox"], r["oy"], r["oyaw"], vb * math.cos(r["oyaw"]), vb * math.sin(r["oyaw"]), CAR_L, CAR_W)
        smp.append((round(r["t"] - t0, 3), pa, pb))
    dedup = {}
    for s in smp:
        dedup[s[0]] = s
    smp = [dedup[k] for k in sorted(dedup)]
    return episode_from_poses(smp, {"id": f"{tier} {name}", "tier": tier, "ctx": ctx, "a": a, "b": "autoware",
                                    "group": ctx_label(ctx)})


# --- double oracle -------------------------------------------------------------------------------
def load_double_oracle(entry, sc):
    """double_oracle.py's log.jsonl (one line per round) and populations.json (the rules, the
    payoff table of every pair played, Autoware-like rules exploited)."""
    d = os.path.expanduser(entry["path"])
    log = [r for r in read_jsonl(os.path.join(d, "log.jsonl")) if "A_mix" in r]
    pop = json.load(open(os.path.join(d, "populations.json")))
    params = {r["name"]: {k: v for k, v in r.items() if k != "name"} for side in ("A", "B") for r in pop[side]}
    br_a = [r["BR_A"]["name"] for r in log if r.get("BR_A")]
    br_b = [r["BR_B"]["name"] for r in log if r.get("BR_B")]
    order_a = [r["name"] for r in pop["A"]]
    order_b = [r["name"] for r in pop["B"]]
    init_a = [n for n in order_a if n not in br_a]
    init_b = [n for n in order_b if n not in br_b]
    rounds = []
    for r in log:
        k = r["round"]
        ca = init_a + [n for n in br_a[:k] if n in order_a]
        cb = init_b + [n for n in br_b[:k] if n in order_b]
        rounds.append({"round": k, "A_pop": ca, "B_pop": cb, "A_mix": r["A_mix"], "B_mix": r["B_mix"],
                       "vA": r.get("vA"), "vB": r.get("vB"), "welfare": r.get("welfare"),
                       "BR_A": (r.get("BR_A") or {}).get("name"), "gain_A": r.get("gain_A"),
                       "BR_B": (r.get("BR_B") or {}).get("name"), "gain_B": r.get("gain_B"),
                       "exploitability_lb": r.get("exploitability_lb")})
    # the round after the last: every BR in
    table = {k: [v.get("uA"), v.get("uB"), v.get("contact"), v.get("ahead")] for k, v in pop["table"].items()}
    final = rounds[-1] if rounds else {}
    weights = {}
    for side in ("A", "B"):
        for n, w in (final.get(f"{side}_mix") or {}).items():
            weights[n] = w
    return {"meta": {"label": entry.get("label", Path(d).name), "rounds": rounds, "table": table, "A": order_a,
                     "B": order_b, "params": params, "weights": weights,
                     "exploited": pop.get("autoware_like_exploited", {}), "contexts": entry.get("contexts")}}


def load_do_rerun(entry, sc):
    """A double oracle's final table re-run (merge_end.py --do6): path the do6_fixed_*.json (the
    table per pair: do6 / legacy / fixed, the equilibria), populations the input populations.json
    (the rules and their order), runs the re-run's runs_*.jsonl (contact rates). One round: the
    re-run table (`fixed`) and its equilibrium, for the meta-game view beside the original run."""
    p = os.path.expanduser(entry["path"])
    d = json.load(open(p))
    pop = json.load(open(os.path.expanduser(entry["populations"])))
    contact = {}
    rp = os.path.expanduser(entry["runs"]) if entry.get("runs") else None
    if rp and os.path.exists(rp):
        for r in read_jsonl(rp):
            contact.setdefault(f"{r['a']}|{r['b']}", []).append(r.get("contact", 0))
    table = {k: [v["fixed"].get("uA"), v["fixed"].get("uB"), st.mean(contact[k]) if k in contact else None,
                 v["fixed"].get("ahead")] for k, v in d["table"].items()}
    order_a = [r["name"] for r in pop["A"]]
    order_b = [r["name"] for r in pop["B"]]
    params = {r["name"]: {k: v for k, v in r.items() if k != "name"} for side in ("A", "B") for r in pop[side]}
    eq = (d.get("eq_fixed") or [{}])[0]
    rnd = {"round": "re-run", "A_pop": order_a, "B_pop": order_b, "A_mix": eq.get("A", {}), "B_mix": eq.get("B", {}),
           "vA": eq.get("vA"), "vB": eq.get("vB"), "welfare": eq.get("welfare"), "BR_A": None, "gain_A": None, "BR_B": None,
           "gain_B": None, "exploitability_lb": None}
    weights = {**eq.get("A", {}), **eq.get("B", {})}
    return {"meta": {"label": entry.get("label", Path(p).name), "rounds": [rnd], "table": table, "A": order_a, "B": order_b,
                     "params": params, "weights": weights, "exploited": pop.get("autoware_like_exploited", {}),
                     "contexts": entry.get("contexts"), "seeds": d.get("seeds")}}


# --- fitted Autoware rules (C3 fit.json) ---------------------------------------------------------
def load_fit(entry, sc):
    """c3_fit.py's fit.json: candidate B rules by error; the best is the fitted Autoware rule.
    Names B_r<react_gap>_b<b_y>_t<threshold>[_s] are parsed into the arena rule's parameters."""
    out = []
    for p in paths(entry):
        fits = json.load(open(p))
        best = min(fits, key=lambda f: f["error"])
        m = re.match(r"B_r([\d.]+)_b([\d.]+)_t([\d.]+)(_s)?", best["rule"])
        prm = {"pre_band": -99.0, "react_gap": float(m.group(1)), "b_y": float(m.group(2))} if m else {}
        out.append({"name": f"{entry.get('label', 'fit')}: {best['rule']}", "params": prm, "error": best["error"],
                    "side": "B"})
    return {"fitted": out}


# --- N cars --------------------------------------------------------------------------------------
def load_n3_cells(entry, sc):
    """n3_model's n3_cells*.jsonl: per (set, scene, d0, dv, x, v_B, profile, car) and model the
    residual r, the tolerance and whether it is within."""
    out = []
    model = entry.get("model", "n3 shift+residual")
    for p in paths(entry):
        for r in read_jsonl(p):
            if r["model"] != model:
                continue
            t, scene, d0, dv, x, vb, a, car = r["key"]
            out.append({"set": t, "scene": scene, "d0": d0, "dv": dv, "x": x, "vb": vb, "a": a, "car": car,
                        "r": r["r"], "tol": r["tol"], "within": r["within"], "cls": r["cls"]})
    return {"ncar_residuals": {"label": entry.get("label", model), "cells": out}}


def load_n1_sizes(entry, sc):
    return {"ncar_n1": json.load(open(paths(entry)[0]))}


def load_n4(entry, sc):
    """n4_far_field's n4_cells.jsonl: per (d0, dv, h_mean, profile, N) the explicit U_A per draw
    and the mean-field and composition predictions."""
    rows = []
    for p in paths(entry):
        if not os.path.exists(p):
            continue
        for r in read_jsonl(p):
            d0, dv, hm, a, n = r["key"]
            xs = r["runs"]
            rows.append({"d0": d0, "dv": dv, "hm": hm, "a": a, "n": n, "u": st.mean(x["u"] for x in xs),
                         "mf": st.mean(x["mf"] for x in xs), "comp": st.mean(x["comp"] for x in xs),
                         "sd": st.pstdev([x["u"] for x in xs]) if len(xs) > 1 else 0.0, "k": len(xs)})
    return {"ncar_n4": rows} if rows else {}


# --- conventions (C7) ------------------------------------------------------------------------------
def load_conventions(entry, sc):
    """C7's results, one table per source. kind:
      tiebreak_kin    tiebreak.py's tiebreak_<geom>.json: per band (near, away, all) and convention
      tiebreak_arena  tiebreak_arena.py's summary.json: per convention, arena and model side by side
      stream          zipper_stream.py's stream_<side>.json: per convention and mean headway"""
    out = []
    for p in paths(entry):
        if not os.path.exists(p):
            raise FileNotFoundError(2, "missing", p)
        d = json.load(open(p))
        kind = entry["kind"]
        if kind == "tiebreak_kin":
            rows = [{"band": k.split("|")[0], "conv": k.split("|")[1], **v} for k, v in d.items()]
        elif kind == "tiebreak_arena":
            rows = []
            for conv, sides in d["per_conv"].items():
                for side, v in sides.items():
                    rows.append({"conv": conv, "side": side, "class_agree": d["classes_agree"].get(conv), **v})
            rows.append({"conv": "D2", "side": "outcomes", "outcomes_agree": d["outcomes_agree"], "outcomes": d["outcomes"],
                         "cells": d["cells"], "runs": d.get("runs")})
        else:
            rows = [{"conv": k.split("|")[0], "hm": float(k.split("|")[1]), **v} for k, v in d.items()]
        out.append({"kind": kind, "label": entry.get("label", os.path.basename(p)), "scene": entry.get("scene", ""),
                    "path": p, "rows": rows})
    return {"conventions": out}


def load_beliefs(entry, sc):
    """C6's results. kind:
      kin    c6_belief.py's c6_summary.json (path) and c6_cells.jsonl (cells): the D6 measures per
             objective, by type, the type-confusion matrix, acting on a fixed belief, and per cell
             the measures against the stock loss (the maps)
      arena  c6_arena.py's summary.json: D2 against the kinematic model, the measures side by side"""
    out = []
    for p in paths(entry):
        if not os.path.exists(p):
            raise FileNotFoundError(2, "missing", p)
        d = json.load(open(p))
        item = {"kind": entry["kind"], "label": entry.get("label", os.path.basename(p)), "path": p}
        if entry["kind"] == "kin":
            keep = ("regret_class", "regret_motion", "voi_v2x", "voi_class", "voi_belief", "voi_probe", "voi_all_p", "voi_all")
            item["summary"] = {o: {k: d[o][k] for k in keep if k in d[o]} for o in ("selfish", "prosocial")}
            item["C"] = {o: d[o].get("C") for o in ("selfish", "prosocial")}
            item["L_stock_mean"], item["L_worst_mean"] = d.get("L_stock_mean"), d.get("L_worst_mean")
            item["by_type"] = d.get("by_type", {})
            want = entry.get("confusion", ["probe motion@0", "probe class@0", "probe motion@3", "probe class@3", "probe_informative class@3",
                                           "A_aw_perm class@5", "A_aw_perm motion@8"])
            item["confusion"] = {k: v for k, v in d.get("confusion", {}).items() if k in want}
            item["assumed"] = {k[4:]: v for k, v in d.get("assumed", {}).items() if k.startswith("s |")}
            item["b_side"] = d.get("b_side")
            cells = []
            cp = os.path.expanduser(entry["cells"]) if entry.get("cells") else None
            if cp and os.path.exists(cp):
                for c in read_jsonl(cp):
                    e = {k: c.get(k) for k in ("d0", "dv", "vb", "capA", "true", "L_stock", "L_worst")}
                    for k in keep:
                        e[k] = c.get("s_" + k)
                        e[k + "_se"] = c.get("s_" + k + "_se")
                        e["p_" + k] = c.get("p_" + k)
                    e["picks"] = c.get("s_picks")
                    cells.append(e)
            item["cells"] = cells
        else:
            item.update({k: d.get(k) for k in ("runs", "d2", "context_error", "compare", "bar_agree", "bar_arena", "bar_kin",
                                                "n_compare")})
            if item["d2"]:
                item["d2"] = {k: v for k, v in item["d2"].items() if k != "misses"}
        out.append(item)
    res = {"beliefs": out}
    d2s = [i["d2"] for i in out if i["kind"] == "arena" and i.get("d2")]
    if d2s:  # the progress grid's D2: payoffs inside the arena's intervals
        w, n = sum(d["payoffs_inside"] for d in d2s), sum(d["payoffs"] for d in d2s)
        res["d2"] = {"within": w, "n": n, "share": w / n if n else None}
    return res


# --- explanations (C4) and the merge-end fix ------------------------------------------------------
def load_explain(entry, sc):
    """C4's and the merge-end fix's results, one item per source. kind:
      c4         c4_fit.py's c4_fit.json: per play set the models' fits (estimate, LR 95 % sets,
                 cell bootstrap, held-out payoff RMSE and log loss), residuals by region, the
                 recovery of planted SVO types, the capability/preference confound, the real stack
      merge_end  merge_end.py's do6_fixed_*.json: do6's final table with the legacy end and the fix
                 on the same runs, the equilibria of both, the runs whose score moved"""
    out = []
    for p in paths(entry):
        if not os.path.exists(p):
            raise FileNotFoundError(2, "missing", p)
        d = json.load(open(p))
        out.append({"kind": entry["kind"], "label": entry.get("label", os.path.basename(p)), "path": p, "data": d})
    return {"explain": out}


# --- S5 on Autoware, the Y merge stream (swb 118) ------------------------------------------------
def _json_member(p, member):
    """A JSON file, or one inside a tar (`member`: its name in the tar)."""
    if not member:
        return json.load(open(p))
    import tarfile
    with tarfile.open(p) as t:
        return json.load(t.extractfile(member))


@class_fn("roundabout_cell")
def _roundabout_cell(row):
    """roundabout_map.csv's class, with '*' when an equilibrium has A forcing B (eq_forced), as
    roundabout_arena classes the arena's cells."""
    c = row.get("class") or "?"
    return c + "*" if (num(row.get("eq_forced")) or 0) > 0 and c not in ("!", "?", "M") else c


def load_s5_autoware(entry, sc):
    """s5_aw.py's s5_table.json (runs: v, gap, seed, go, contact, pet, uA, uB): one cell per
    (v, lag) for the entry's configuration (`config`, the class map's y; `config_name` in the
    tooltip). Class: g B turned ahead of A in every seed, w B waited in every seed, ! a contact,
    c both over seeds."""
    by = {}
    for p in paths(entry):
        if not os.path.exists(p):
            raise FileNotFoundError(2, "missing", p)
        for r in _json_member(p, entry.get("member"))["runs"]:
            by.setdefault((r["v"], r["gap"]), []).append(r)
    cells, name = [], entry.get("config_name", entry.get("label", ""))
    for (v, gap), rs in sorted(by.items()):
        go, contact = sum(bool(r["go"]) for r in rs), sum(bool(r["contact"]) for r in rs)
        cls = "!" if contact else "g" if go == len(rs) else "w" if go == 0 else "c"
        pets = "/".join(f"{r['pet']:g}" for r in rs if r.get("pet") is not None) or "-"
        lags = "/".join(f"{r['lag']:g}" for r in rs if r.get("lag") is not None)
        ub = st.mean(r["uB"] for r in rs)
        summ = f"{name}: went {go}/{len(rs)}, contact {contact}, PET {pets} s, uB {ub:+.3f}, measured lag {lags} s"
        c = {"gap": gap, "config": entry["config"], "v": v}
        c.update(entry.get("fixed") or {})
        cells.append({"ctx": c, "class": cls, "eq": [], "outcomes": f"{go} went / {len(rs)}", "eq_summary": summ})
    return {"cells": cells}


def load_u2_swb(entry, sc):
    """u2_swb.py's u2_swb.jsonl (one row per swb run: n, h, tb, a, b, seed, u0, u1, contact, a_ahead,
    forced, valid): one cell per context over the fixed pairs played there. Class: the pairs'
    outcome, A ahead in every run, B behind in every run, C both, ! a contact; * when A forced B."""
    by = {}
    for p in paths(entry):
        for r in read_jsonl(p):
            if not r.get("valid", 1) or r.get("scenario_exit", 0) not in (0, None):
                continue
            c = {"n": num(r["n"]), "h": num(r["h"]), "tb": num(r["tb"]), **(entry.get("fixed") or {})}
            by.setdefault(ctx_key(c), {"ctx": c, "u": {}})["u"].setdefault((r["a"], r["b"]), []).append(r)
    cells, payoffs = [], []
    for v in by.values():
        pairs, outs, forced, contact = [], [], 0, 0
        for (a, b), rs in v["u"].items():
            pairs.append([a, b, st.mean(r["u0"] for r in rs), st.mean(r["u1"] for r in rs)])
            outs += ["A" if r["a_ahead"] else "B" for r in rs]
            forced += sum(bool(r.get("forced")) for r in rs)
            contact += sum(bool(r.get("contact")) for r in rs)
        cls = "!" if contact else (outs[0] if len(set(outs)) == 1 else "C") + ("*" if forced else "")
        summ = "; ".join(f"{a}/{b} uA {ua:+.3f} uB {ub:+.3f}" for a, b, ua, ub in pairs)
        cells.append({"ctx": v["ctx"], "class": cls, "eq": [], "n": len(outs),
                      "outcomes": f"A ahead {outs.count('A')}/{len(outs)}, forced {forced}, contact {contact}",
                      "eq_summary": summ})
        payoffs.append({"ctx": v["ctx"], "pairs": pairs, "eq": [], "label": entry.get("label")})
    return {"cells": cells, "payoffs": payoffs}


def load_ymerge_stream(entry, sc):
    """ymerge_game's stream.jsonl (one row per draw, dt, rule and N): one cell per (N, h_mean, rule),
    classed by A's externality on the stream (its cost minus the same stream's with A far back):
    . below 0.05, + below 0.2, # from 0.2, ! a contact."""
    by = {}
    for p in paths(entry):
        for r in read_jsonl(p):
            by.setdefault((r["n"], r["h_mean"], r["rule"]), []).append(r)
    cells = []
    for (n, hm, rule), rs in sorted(by.items()):
        ext, ua = st.mean(r["externality"] for r in rs), st.mean(r["uA"] for r in rs)
        k = sum(bool(r["contact"]) for r in rs)
        cls = "!" if k else "." if ext < 0.05 else "+" if ext < 0.2 else "#"
        c = {"n": n, "h_mean": hm, "rule": rule}
        c.update(entry.get("fixed") or {})
        cost = st.mean(r["cost_stream"] for r in rs)
        slots = [r["slot"] for r in rs if r.get("slot") is not None]
        slot = f"{st.mean(slots):.2f}" if slots else "-"
        cells.append({"ctx": c, "class": cls, "eq": [], "outcomes": f"contact {k}/{len(rs)}",
                      "eq_summary": f"A's externality {ext:.3f}, stream cost {cost:.3f}, uA {ua:+.3f}, slot {slot}"})
    return {"cells": cells}


def load_e3_knobs(entry, sc):
    """E3's runs tars (runs/<variant>/c3.jsonl inside): Autoware's cruise-knob settings as B
    strategies, B named aw_<variant>. Payoff points per (d0, dv): (A rule, aw_<variant>), means over
    seeds; no cells (the class map keeps C3a's stock outcome)."""
    import tarfile
    by = {}
    for p in paths(entry):
        if not os.path.exists(p):
            raise FileNotFoundError(2, "missing", p)
        with tarfile.open(p) as t:
            for m in t.getmembers():
                parts = m.name.split("/")
                if len(parts) != 3 or parts[2] != "c3.jsonl":
                    continue
                for line in t.extractfile(m).read().decode().splitlines():
                    if not line.strip():
                        continue
                    r = json.loads(line)
                    if r.get("scenario_rc", 0) not in (0, None):
                        continue
                    c = {"d0": num(r["d0"]), "dv": num(r["dv"]), **entry.get("fixed", {})}
                    by.setdefault(ctx_key(c), {"ctx": c, "u": {}})["u"].setdefault((r["a"], f"aw_{parts[1]}"), []).append(r)
    payoffs = []
    for v in by.values():
        pairs = [[a, b, st.mean(r["uA"] for r in rs), st.mean(r["uB"] for r in rs)] for (a, b), rs in sorted(v["u"].items())]
        payoffs.append({"ctx": v["ctx"], "pairs": pairs, "eq": [], "label": entry.get("label")})
    return {"payoffs": payoffs}


# --- sites (E6, E7) --------------------------------------------------------------------------------
def _image_uri(p, max_px):
    """A PNG as a data: URI, downscaled to max_px on its longer side when PIL is there."""
    import base64
    import io
    raw = open(p, "rb").read()
    try:
        from PIL import Image
        im = Image.open(io.BytesIO(raw))
        if max(im.size) > max_px:
            im.thumbnail((max_px, max_px), Image.LANCZOS)
            buf = io.BytesIO()
            im.save(buf, "PNG", optimize=True)
            if buf.tell() < len(raw):
                raw = buf.getvalue()
    except Exception:  # no PIL, or a PNG PIL cannot read: embedded as it is
        pass
    return "data:image/png;base64," + base64.b64encode(raw).decode()


def load_sites(entry, sc):
    """Site search results (sites.py, stop_sign_map.py), one item per entry. kind:
      table   a CSV (e6_four_way.csv), rows as they are; `rank` lists "<town>/<junction>" in rank
              order (the report's), shown first; `cols` the columns to show (default all)
      images  `images`: [{path, caption}], PNGs embedded (downscaled to max_px, default 1000);
              a missing image is listed, not fatal, unless every image is missing"""
    kind = entry["kind"]
    item = {"kind": kind, "label": entry.get("label", kind), "notes": entry.get("notes", [])}
    if kind == "table":
        p = paths(entry)[0]
        if not os.path.exists(p):
            raise FileNotFoundError(2, "missing", p)
        rows = read_csv(p)
        rank = [str(x) for x in entry.get("rank", [])]
        for r in rows:
            k = f"{r.get('town')}/{r.get('junction')}"
            r["rank"] = rank.index(k) + 1 if k in rank else None
        rows.sort(key=lambda r: r["rank"] if r["rank"] is not None else len(rank) + 1)  # stable: CSV order after
        cols = entry.get("cols") or ([c for c in rows[0] if c != "rank"] if rows else [])
        item.update({"path": p, "cols": ["rank"] + cols, "rows": rows})
    else:
        imgs, missing = [], []
        for im in entry["images"]:
            p = os.path.expanduser(im["path"])
            if not os.path.exists(p):
                missing.append(p)
                continue
            imgs.append({"caption": im.get("caption", os.path.basename(p)), "path": p,
                         "src": _image_uri(p, entry.get("max_px", 1000))})
        if not imgs:
            raise FileNotFoundError(2, "missing", missing[0] if missing else "no images")
        item.update({"images": imgs, "missing": missing})
    return {"sites": [item]}


# --- progress ------------------------------------------------------------------------------------
def load_d2(entry, sc):
    """A D2 share as text ('payoffs inside the arena's 95 % interval: 44 of 48') or a summary.json's
    payoffs_inside / payoffs."""
    out = []
    for p in paths(entry):
        if not os.path.exists(p):
            continue
        if p.endswith(".json"):
            d = json.load(open(p))
            if "payoffs" in d:
                out.append((d.get("payoffs_inside", 0), d["payoffs"]))
        else:
            m = re.search(r"(\d+) of (\d+)", open(p).read())
            if m:
                out.append((int(m.group(1)), int(m.group(2))))
    if not out:
        return {}
    w, n = sum(a for a, _ in out), sum(b for _, b in out)
    return {"d2": {"within": w, "n": n, "share": w / n if n else None}}


LOADERS = {k[5:]: v for k, v in globals().items() if k.startswith("load_")}


def export(sources, out, only=None, log=print):
    cfg = yaml.safe_load(open(sources))
    atlas = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "sources": str(sources), "tiers": cfg["tiers"],
             "scenarios": {}, "progress": [], "ncar": {}, "fitted": [], "conventions": [], "beliefs": [], "explain": [], "sites": [],
             "notes": cfg.get("notes", [])}
    for sname, sc in cfg["scenarios"].items():
        if only and sname not in only:
            continue
        S = {k: sc.get(k) for k in ("title", "axes", "facets", "legend", "rules", "labels", "state_machines", "ncar", "conventions", "beliefs", "explain", "sites", "notes")}
        S["tiers"] = {}
        S["episodes"] = []
        S["meta"] = []
        for tier, entries in (sc.get("tiers") or {}).items():
            for entry in entries or []:
                t0 = time.monotonic()
                name = entry["loader"]
                fn = LOADERS[name]
                try:
                    res = fn(entry, sc, out) if name == "kinematic_resim" else fn(entry, sc)
                except FileNotFoundError as e:
                    log(f"  [{sname}/{tier}] {name}: missing {e.filename}; skipped")
                    atlas["progress"].append({"scenario": sname, "tier": tier, "status": "missing",
                                              "label": entry.get("label"), "link": entry.get("link")})
                    continue
                T = S["tiers"].setdefault(tier, {"cells": [], "payoffs": [], "labels": [], "d2": None, "links": [], "items": 0})
                T["items"] += (len(res.get("episodes", [])) + bool(res.get("meta")) + len(res.get("fitted", []))
                               + sum(bool(res.get(k)) for k in ("ncar_residuals", "ncar_n1", "ncar_n4"))
                               + len(res.get("conventions", [])) + len(res.get("beliefs", [])) + len(res.get("explain", []))
                               + len(res.get("sites", [])))
                T["cells"] += res.get("cells", [])
                T["payoffs"] += res.get("payoffs", [])
                if res.get("d2"):
                    T["d2"] = res["d2"]
                if entry.get("label"):
                    T["labels"].append(entry["label"])
                if entry.get("link"):
                    T["links"].append(entry["link"])
                S["episodes"] += res.get("episodes", [])
                if res.get("meta"):
                    S["meta"].append(res["meta"])
                for k in ("ncar_residuals", "ncar_n1", "ncar_n4"):
                    if res.get(k):
                        atlas["ncar"].setdefault(k, []).append(res[k]) if k == "ncar_residuals" else atlas["ncar"].__setitem__(k, res[k])
                atlas["fitted"] += res.get("fitted", [])
                atlas["conventions"] += res.get("conventions", [])
                atlas["beliefs"] += res.get("beliefs", [])
                atlas["explain"] += res.get("explain", [])
                atlas["sites"] += res.get("sites", [])
                log(f"  [{sname}/{tier}] {name}: {len(res.get('cells', []))} cells, {len(res.get('payoffs', []))} payoff "
                    f"contexts, {len(res.get('episodes', []))} episodes ({time.monotonic() - t0:.1f} s)")
        for tier, T in S["tiers"].items():
            status = "done" if (T["cells"] or T["payoffs"] or T["d2"] or T["items"]) else "empty"
            atlas["progress"].append({"scenario": sname, "tier": tier, "status": status,
                                      "cells": len(T["cells"]), "d2": T["d2"], "label": ", ".join(T["labels"]),
                                      "link": T["links"][0] if T["links"] else None})
        for tier, st_ in (sc.get("planned") or {}).items():
            atlas["progress"].append({"scenario": sname, "tier": tier, "status": "planned", "label": st_})
        atlas["scenarios"][sname] = S
    atlas["e3_knobs"] = cfg.get("e3_knobs", [])
    atlas = clean(atlas)
    os.makedirs(out, exist_ok=True)
    p = os.path.join(out, "atlas.json")
    json.dump(atlas, open(p, "w"), separators=(",", ":"))
    log(f"atlas.json: {os.path.getsize(p) / 1e6:.2f} MB")
    return atlas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sources", default=str(HERE / "sources.yaml"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--only", default="", help="comma-separated scenarios")
    a = ap.parse_args()
    export(a.sources, a.out, [s for s in a.only.split(",") if s] or None)


if __name__ == "__main__":
    main()
