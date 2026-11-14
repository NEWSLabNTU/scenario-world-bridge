#!/usr/bin/env python3
"""Roadmap 074 N3: a residual model for N cars, fitted on three-car runs, tested on held-out ones.

N2 (n2_composition.py) composed pair terms per measure (risk max, delay max-plus, braking max)
and found a systematic residual, carried by a discrete change of the A-B outcome: (scene a) a
B-C slot shorter than A's gap check, so A drops behind C too; (scene b) a lead L that slows A
before it decides, so the outcome is the A-B pair's at the context L imposes. N3:

  v1   N2's composition as specified (risk max, delay max-plus down the queue, braking max)
  v2   N2's relation-aware composition, unchanged, extended to a queue of any length: a pair
       term enters only if its relation exists given the A-B pair's outcome (A ahead: A meets
       only B; the queue meets as adjacent followers), and A, dropping, walks down the queue
       slot by slot, paying (gap + L) / v for every slot shorter than its gap check
  n3   v2 with the context L imposes: A meets L before it decides when, closing at dv > 0, its
       gap to L reaches the following gap S0 + v_L HEADWAY before the A-B pair's decision time
       (t_meet = (gl - g_eq) / dv). Then the A-B pair term is read at the shifted context
       (d0 + gl - g_eq, dv = v_L - v_B) from the table of pair runs (scene p), A's lost progress
       is rescaled to its own start speed, 1 - (v_L / v_A)(1 - lost'), and B's is the shifted
       pair's. What remains is fitted per outcome class by a small linear model (fit_residual),
       on the three-car runs only.

Run sets (each a list of groups for arena_n; 8 seeds, the two profiles of N2):
  fit     N2's grid: scene a hf {1, 2} s, scene b gl {15, 30} m, v_B 12 m/s
  ho_hf   scene a, hf {1.5, 3} s
  ho_vb   v_B {8, 16} m/s, scene a hf {1, 2} and scene b gl {15, 30}
  ho_gl   scene b, gl 22 m
  ho_a4   the four-car queue (B, C, D at hf {1, 2} s): ABCD and its six pairs
  pairs   A and B alone, d0 -4..34 m at dv 0, v_B {8, 12, 16}: the pair terms at shifted contexts

Usage:
  scripts/coop/n3_model.py run --set fit,ho_hf,... --ports 2600,2610,2620 --out <dir>
  scripts/coop/n3_model.py analyze --out <dir>
  scripts/coop/n3_model.py d2 --port 2600 --out <dir>   # C2b's D2 under the fixed window
  scripts/coop/n3_model.py --selftest
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import multiprocessing as mp
import os
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cutin_game as C1  # noqa: E402
import measures  # noqa: E402
import n2_composition as N2  # noqa: E402

H = C1.HORIZON
L_CAR = 4.79
PROFILES = N2.PROFILES
PA = N2.PA
D0S, DVS = N2.D0S, N2.DVS
SEEDS = 8
GROUPS = {"a": ("ABC", ("AB", "AC", "BC")), "b": ("ABL", ("AB", "AL", "BL")),
          "a4": ("ABCD", ("AB", "AC", "AD", "BC", "BD", "CD"))}
QUEUE = {"a": "BC", "a4": "BCD", "b": "B"}
PAIR_D0S = tuple(float(d) for d in range(-4, 35))


def _ctx_jobs(scene, xs, vbs, tag):
    full, pairs = GROUPS[scene]
    key = "gl" if scene == "b" else "hf"
    out = []
    for (d0, dv, x, vb), (pa, pb), s in itertools.product(itertools.product(D0S, DVS, xs, vbs), PROFILES, range(SEEDS)):
        for roles in (full, *pairs):
            out.append({"set": tag, "scene": scene, "d0": d0, "dv": dv, key: x, "vb": vb, "pa": pa, "pb": pb,
                        "seed": s, "roles": roles, "trace": False})
    return out


def jobs(name):
    if name == "fit":
        return _ctx_jobs("a", (1.0, 2.0), (12.0,), name) + _ctx_jobs("b", (15.0, 30.0), (12.0,), name)
    if name == "ho_hf":
        return _ctx_jobs("a", (1.5, 3.0), (12.0,), name)
    if name == "ho_vb":
        return _ctx_jobs("a", (1.0, 2.0), (8.0, 16.0), name) + _ctx_jobs("b", (15.0, 30.0), (8.0, 16.0), name)
    if name == "ho_gl":
        return _ctx_jobs("b", (22.0,), (12.0,), name)
    if name == "ho_a4":
        return _ctx_jobs("a4", (1.0, 2.0), (12.0,), name)
    if name == "pairs":
        return [{"set": name, "scene": "p", "d0": d0, "dv": 0.0, "vb": vb, "pa": pa, "pb": pb, "seed": s,
                 "roles": "AB", "trace": False}
                for d0, vb, (pa, pb), s in itertools.product(PAIR_D0S, (8.0, 12.0, 16.0), PROFILES, range(SEEDS))]
    raise SystemExit(f"unknown set {name}")


# --- composition -------------------------------------------------------------------------------
def ctx_x(r):
    return r.get("gl") if r["scene"] == "b" else r.get("hf")


def lead_of(scene, car):
    if scene == "b":
        return "L" if car == "A" else None
    q = QUEUE[scene]
    return q[q.index(car) - 1] if car in q and q.index(car) > 0 else None


def slack(scene, x, v):
    """The time headway a follower holds beyond the desired HEADWAY (s)."""
    if scene == "b":
        return max(0.0, (x - C1.S0) / max(v, 0.1) - C1.HEADWAY)
    return max(0.0, x - C1.HEADWAY)


def delay(scene, car, pr, x, memo=None):
    """Car's delay, max-plus down the leader-follower chain of the scene's initial lanes."""
    mine = [r for p, r in pr.items() if car in p]
    d = max(r[f"lost{car}"] * H for r in mine)
    lead = lead_of(scene, car)
    if lead is not None and any(lead in p and car in p for p in pr):
        v = next(r for p, r in pr.items() if car in p)[f"v0{car}"]
        d = max(d, delay(scene, lead, pr, x) - slack(scene, x, v))
    return d


def compose(scene, car, pr, x):
    """v1: car's composed (U_hat, parts) from its pair rows pr {roles: row} (n2's compose,
    the max-plus recursive down a queue of any length)."""
    mine = [(p, r) for p, r in pr.items() if car in p]
    contact = any(r[f"contact{car}"] for _, r in mine)
    rho = max(r[f"rho_{p}"] for p, r in mine)
    peak = max(r[f"peak{car}"] for _, r in mine)
    lat = max(r[f"lat{car}"] for _, r in mine)
    d = delay(scene, car, pr, x)
    lost = 1.0 if contact else min(1.0, d / H)
    u = measures.utility(lost, rho, peak, C1.TYPES["car"].b_max, lat, contact)
    return u, {"lost": lost, "rho": rho, "peak": peak, "lat": lat, "contact": int(contact)}


def with_lost(parts, lost):
    lost = 1.0 if parts["contact"] else min(1.0, max(0.0, lost))
    u = measures.utility(lost, parts["rho"], parts["peak"], C1.TYPES["car"].b_max, parts["lat"], parts["contact"])
    return u, dict(parts, lost=lost)


def queue_walk(scene, ab, x, pa, vb):
    """A dropping back in the A-B pair: walk down the queue (n2.compose_v2's slot test, slot by
    slot). Returns (pairs kept, A's extra delay in s, how many slots A skipped)."""
    q = QUEUE[scene]
    keep = {"AB"} | {q[i] + q[i + 1] for i in range(len(q) - 1)}
    if ab["a_pos"] == "ahead":
        return keep, 0.0, 0
    v = ab["v0A"]
    gap = C1.S0 + vb * x
    need = pa["buffer"] + v * pa["tau"]
    extra, skipped = 0.0, 0
    for i in range(1, len(q)):
        if gap >= C1.S0 + v * C1.HEADWAY + L_CAR + need:
            keep.add("A" + q[i])  # A fits in front of q[i]: the literal pair (not reached in N2)
            break
        extra += (gap + L_CAR) / max(v, 0.1)
        skipped += 1
    return keep, extra, skipped


def compose_v2(scene, car, pr, x, pa, vb):
    """N2's relation-aware composition, unchanged; for a queue of any length (scene a4)."""
    if scene == "b":
        return compose(scene, car, pr, x)
    keep, extra, _ = queue_walk(scene, pr["AB"], x, pa, vb)
    u, parts = compose(scene, car, {p: r for p, r in pr.items() if p in keep}, x)
    if extra and car == "A":
        u, parts = with_lost(parts, parts["lost"] + extra / H)
    return u, parts


class PairTable:
    """The A-B pair at d0 -4..34 m, dv 0 (scene p), per (v_B, profile, seed): linear in d0
    between grid points for the continuous terms; the outcome is the nearest point's."""

    def __init__(self, rows):
        self.t = {}
        for r in rows:
            self.t.setdefault((r["vb"], r["a"], r["seed"]), {})[r["d0"]] = r

    def at(self, vb, a, seed, d0):
        g = self.t.get((vb, a, seed))
        if not g:
            return None
        ks = sorted(g)
        d0 = min(max(d0, ks[0]), ks[-1])
        lo = max(k for k in ks if k <= d0)
        hi = min(k for k in ks if k >= d0)
        if lo == hi or g[lo]["a_pos"] != g[hi]["a_pos"]:
            return g[lo if d0 - lo <= hi - d0 else hi]
        w = (d0 - lo) / (hi - lo)
        out = dict(g[lo])
        for k, v in g[lo].items():
            if isinstance(v, (int, float)) and not isinstance(v, bool) and isinstance(g[hi].get(k), (int, float)) \
                    and k.startswith(("lost", "rho", "peak", "lat", "v0", "u")):
                out[k] = (1 - w) * v + w * g[hi][k]
        return out


TAU_S = 0.5  # s: L shifts the A-B context only if A decides at least this long after it meets L


def g_meet(va, dv):
    """The gap at which IDM's interaction term starts to brake A toward L (its desired gap s*)."""
    cap = C1.TYPES["car"]
    return C1.S0 + va * C1.HEADWAY + va * max(0.0, dv) / (2.0 * math.sqrt(2.0 * cap.a_max))


def shift(scene, pr, x, vb):
    """Scene b: does L slow A before it decides? (t_meet, shifted d0) or None.

    A, closing on L at dv = v_A - v_L, starts braking when its gap reaches IDM's s* (g_meet), at
    t_meet = (gl - g_meet) / dv (0 if it starts inside). If the A-B pair's decision comes later
    than t_meet + TAU_S, A decides at L's speed, having gained gl - g_eq on B, g_eq = S0 +
    v_L HEADWAY its following gap behind L: the A-B pair at (d0 + gl - g_eq, dv 0)."""
    if scene != "b":
        return None
    ab = pr["AB"]
    va, vl = ab["v0A"], ab["v0B"]  # L runs at B's speed
    dv = va - vl
    gm = g_meet(va, dv)
    if dv > 0.1:
        t_meet = max(0.0, (x - gm) / dv)
    elif x < gm - 0.5:
        t_meet = 0.0  # already inside L's gap, at L's speed or slower
    else:
        return None  # A never reaches L's gap
    t_dec = ab.get("dec_t")
    if t_dec is not None and t_dec <= t_meet + TAU_S:
        return None  # A decides before L slows it
    g_eq = C1.S0 + vl * C1.HEADWAY
    return t_meet, ab["d0"] + (x - g_eq)


def compose_n3(scene, car, pr, x, pa, vb, table, a, seed, theta=None, feats=None):
    """v2 with the shifted context (scene b), plus the fitted residual theta per class."""
    sh = shift(scene, pr, x, vb) if table is not None else None
    if sh is None or car == "L":
        u, parts = compose_v2(scene, car, pr, x, pa, vb)
        cls = klass(scene, car, pr, x, pa, vb, None)
    else:
        _, d0s = sh
        ab = table.at(vb, a, seed, d0s)
        if ab is None:
            u, parts = compose_v2(scene, car, pr, x, pa, vb)
        else:
            ab = dict(ab, d0=d0s)
            pr2 = dict(pr, AB=ab)
            u, parts = compose(scene, car, pr2, x)
            if car == "A":
                va = pr["AB"]["v0A"]
                vl = pr["AL"]["v0L"]
                u, parts = with_lost(parts, 1.0 - (vl / va) * (1.0 - ab["lostA"]))
        cls = klass(scene, car, pr, x, pa, vb, sh)
    if theta is not None and cls in theta:
        f = feats(scene, car, pr, x, pa, vb, sh)
        u = u + sum(c * fi for c, fi in zip(theta[cls], f))
    return u, parts, cls


def klass(scene, car, pr, x, pa, vb, sh):
    """The outcome class the residual is fitted per: (scene kind, role, A-B outcome, shifted)."""
    kind = "b" if scene == "b" else "q"  # q: a target-lane queue
    role = car if car in "ABL" else "F"  # every queue follower is one role: C and D alike
    out = pr["AB"]["a_pos"] == "ahead"
    return f"{kind}:{role}:{'ahead' if out else 'drop'}{':shift' if sh else ''}"


def features(scene, car, pr, x, pa, vb, sh):
    """The residual's regressors: a constant, the queue's slot against A's need (scene a), A's
    lead (scene b: how far inside L's braking gap A starts, alone and times A's margin on its
    gap check against B, (d0 - need) / need: pressed by L, A cuts in short of its own check). Permutation-invariant over the followers: a follower's features
    are its own slot's, the same function for C and D."""
    ab = pr["AB"]
    v = ab["v0A"]
    if scene == "b":
        gm = g_meet(v, v - ab["v0B"])
        press = max(0.0, gm - x) / gm  # how far inside L's braking gap A starts
        need = pa["buffer"] + ab["v0B"] * pa["tau"]  # A's gap check against B at dv >= 0
        return (1.0, press, press * (ab["d0"] - need) / need)  # ... and how far short of its check
    gap = C1.S0 + vb * x
    need = pa["buffer"] + v * pa["tau"] + C1.S0 + v * C1.HEADWAY + L_CAR
    return (1.0, (gap - need) / (v * H), (ab["v0B"] - v) / max(v, 0.1))


# --- analysis ------------------------------------------------------------------------------------
def load(out, names):
    rows = []
    for n in names:
        p = os.path.join(out, f"{n}.jsonl")
        if os.path.exists(p):
            rows += [json.loads(l) for l in open(p) if l.strip()]
    return rows


def grouped(rows):
    by = {}
    for r in rows:
        if r["scene"] == "p":
            continue
        by.setdefault((r["set"], r["scene"], r["d0"], r["dv"], ctx_x(r), r["vb"], r["a"], r["b"], r["seed"]), {})[r["roles"]] = r
    return by


def cells(rows, table, theta=None):
    """Per (set, scene, context, profile, car): the full run's U and each model's prediction."""
    out = {}
    for (tag, scene, d0, dv, x, vb, a, b, seed), g in grouped(rows).items():
        full, pairs = GROUPS[scene]
        if full not in g or any(p not in g for p in pairs):
            continue
        pr = {p: g[p] for p in pairs}
        pa = PA[a]
        for car in full:
            u1, _ = compose(scene, car, pr, x)
            u2, _ = compose_v2(scene, car, pr, x, pa, vb)
            u3, _, cls = compose_n3(scene, car, pr, x, pa, vb, table, a, seed)
            f = features(scene, car, pr, x, pa, vb, shift(scene, pr, x, vb))
            out.setdefault((tag, scene, d0, dv, x, vb, a, car), []).append(
                {"seed": seed, "u": g[full][f"u{car}"], "v1": u1, "v2": u2, "n3c": u3, "cls": cls, "f": f,
                 "pos": g[full].get("a_pos"), "pos_ab": pr["AB"].get("a_pos")})
    return out


def judge(cs, model, theta=None):
    """Per cell: mean residual of `model`, the tolerance, within; theta adds the fitted residual."""
    res = []
    for key, xs in sorted(cs.items()):
        us = [z["u"] for z in xs]
        sd = st.pstdev(us) if len(us) > 1 else 0.0
        tol = max(2 * sd, 0.02)

        def pred(z):
            p = z[model]
            if theta is not None and z["cls"] in theta:
                p += sum(c * fi for c, fi in zip(theta[z["cls"]], z["f"]))
            return p
        r = st.mean(z["u"] - pred(z) for z in xs)
        cls = max(set(z["cls"] for z in xs), key=[z["cls"] for z in xs].count)
        res.append({"key": key, "r": r, "tol": tol, "within": abs(r) <= tol, "cls": cls, "u": st.mean(us),
                    "pos": max(set(z["pos"] for z in xs), key=[z["pos"] for z in xs].count),
                    "pos_ab": max(set(z["pos_ab"] for z in xs), key=[z["pos_ab"] for z in xs].count)})
    return res


def lstsq(X, y, ridge=1e-6):
    n = len(X[0])
    A = [[sum(X[k][i] * X[k][j] for k in range(len(X))) + (ridge if i == j else 0.0) for j in range(n)] for i in range(n)]
    b = [sum(X[k][i] * y[k] for k in range(len(X))) for i in range(n)]
    for i in range(n):  # Gauss-Jordan
        p = max(range(i, n), key=lambda r: abs(A[r][i]))
        A[i], A[p], b[i], b[p] = A[p], A[i], b[p], b[i]
        for r in range(n):
            if r != i and A[i][i] != 0:
                f = A[r][i] / A[i][i]
                A[r] = [A[r][c] - f * A[i][c] for c in range(n)]
                b[r] -= f * b[i]
    return [b[i] / A[i][i] if A[i][i] else 0.0 for i in range(n)]


def fit_residual(cs, model="n3c", min_cells=4):
    """theta per class: least squares of the per-cell mean residual on the cell's mean features."""
    by = {}
    for key, xs in cs.items():
        cls = max(set(z["cls"] for z in xs), key=[z["cls"] for z in xs].count)
        f = [st.mean(z["f"][i] for z in xs) for i in range(len(xs[0]["f"]))]
        r = st.mean(z["u"] - z[model] for z in xs)
        by.setdefault(cls, []).append((f, r))
    theta = {}
    for cls, pts in by.items():
        if len(pts) < min_cells:
            continue
        theta[cls] = lstsq([p[0] for p in pts], [p[1] for p in pts], ridge=1e-4)
    return theta


def share(res, pred=lambda r: True):
    rs = [r for r in res if pred(r)]
    w = sum(r["within"] for r in rs)
    return w, len(rs)


def contexts_share(res):
    ctx = {}
    for r in res:
        tag, scene, d0, dv, x, vb, a, car = r["key"]
        ctx.setdefault((tag, scene, d0, dv, x, vb, a), []).append(r["within"])
    return sum(all(v) for v in ctx.values()), len(ctx)


def analyze(out):
    names = ("fit", "ho_hf", "ho_vb", "ho_gl", "ho_a4")
    rows = load(out, names)
    table = PairTable(load(out, ("pairs",)))
    cs = cells(rows, table)
    fit = {k: v for k, v in cs.items() if k[0] == "fit"}
    theta = fit_residual(fit)
    L = []
    p = L.append
    p(f"N3: {len(rows)} group runs, {len(cs)} (set, scene, context, profile, car) cells, "
      f"pair table {sum(len(g) for g in table.t.values())} runs")
    p("")
    p("residual model, per class (theta: constant, then the class's features; fitted on 'fit' only):")
    for cls in sorted(theta):
        p(f"  {cls:22s} " + " ".join(f"{c:+.4f}" for c in theta[cls]))
    p("")
    models = (("v1", "v1", None), ("v2", "v2", None), ("n3 shift", "n3c", None), ("n3 shift+residual", "n3c", theta))
    summary = {}
    for tag in names:
        sub = {k: v for k, v in cs.items() if k[0] == tag}
        if not sub:
            continue
        p(f"[{tag}] within tolerance (car-cells; contexts with every car within):")
        for label, m, th in models:
            res = judge(sub, m, th)
            w, n = share(res)
            wc, nc = contexts_share(res)
            line = f"  {label:18s} {w:4d}/{n:<4d} ({100 * w / max(n, 1):3.0f} %)  contexts {wc:3d}/{nc:<3d} ({100 * wc / max(nc, 1):3.0f} %) |"
            for scene in ("a", "a4", "b"):
                for car in GROUPS[scene][0]:
                    ww, nn = share(res, lambda r: r["key"][1] == scene and r["key"][7] == car)
                    if nn:
                        line += f" {scene}{car} {ww}/{nn}"
            p(line)
            summary[(tag, label)] = {"cells": [w, n], "contexts": [wc, nc], "res": res}
        p("")
    ho = [t for t in names if t != "fit"]
    p("held-out, pooled (every set but 'fit'):")
    for label, m, th in models:
        res = [r for t in ho if (t, label) in summary for r in summary[(t, label)]["res"]]
        w, n = share(res)
        wc, nc = contexts_share(res)
        p(f"  {label:18s} {w:4d}/{n:<4d} ({100 * w / max(n, 1):3.0f} %)  contexts {wc:3d}/{nc:<3d} ({100 * wc / max(nc, 1):3.0f} %)")
    p("")
    p("misses of n3 shift+residual (set scene d0 dv x vB profile car: U | r tol | class | A's slot / AB pair's):")
    for tag in names:
        for r in summary.get((tag, "n3 shift+residual"), {}).get("res", []):
            if not r["within"]:
                t, scene, d0, dv, x, vb, a, car = r["key"]
                p(f"  {t:6s} {scene:2s} {d0:4.0f} {dv:+3.0f} {x:4.1f} {vb:4.0f} {a:11s} {car}: {r['u']:+.3f} | "
                  f"{r['r']:+.3f} {r['tol']:.3f} | {r['cls']} | {r['pos']}/{r['pos_ab']}")
    txt = "\n".join(L)
    open(os.path.join(out, "n3_summary.txt"), "w").write(txt + "\n")
    with open(os.path.join(out, "n3_cells.jsonl"), "w") as f:
        for (tag, label), v in summary.items():
            for r in v["res"]:
                f.write(json.dumps({"set": tag, "model": label, **{k: r[k] for k in ("r", "tol", "within", "cls", "u", "pos", "pos_ab")},
                                    "key": r["key"]}) + "\n")
    json.dump({k: v for k, v in theta.items()}, open(os.path.join(out, "n3_theta.json"), "w"), indent=1)
    print(txt)


# --- D2 under the fixed window ----------------------------------------------------------------
def d2(port, out, swb_runs="/mnt/p5plus/aeon/coop/c2b/run2"):
    """C2b's swb runs re-scored from their logs (c2b.score, progress over [0, H]) against the
    arena's runs of the same cells (c2b.arena_compare): with N3's follower and, for comparison,
    with the pre-N3 one (arena.LEGACY_FOLLOW)."""
    import subprocess
    import arena as AR
    import c2b
    import s0_rescore
    base = os.path.join(out, "d2")
    os.makedirs(base, exist_ok=True)
    s0_rescore.rescore("c2b", Path(swb_runs), Path(base))
    rows = [json.loads(l) for l in open(os.path.join(base, "c2b.s0.jsonl"))]
    for variant, legacy in (("new", False), ("legacy_follow", True)):
        d = os.path.join(base, variant)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "c2b.jsonl"), "w") as f:
            for r in rows:
                f.write(json.dumps(r) + "\n")
        AR.LEGACY_FOLLOW = legacy
        c2b.arena_compare(port, d)
        txt = subprocess.run([sys.executable, str(Path(__file__).resolve().parent / "d2_compare.py"), d],
                             capture_output=True, text=True).stdout
        open(os.path.join(d, "d2_compare.txt"), "w").write(txt)
        print(f"[{variant}]\n{txt}")
    AR.LEGACY_FOLLOW = False


def selftest():
    # the queue walk: one slot too short at hf 1 s (gap 14 m), a fitting one at hf 3 s (38 m)
    pa = PA["A_aw_perm"]
    ab = {"a_pos": "behind", "v0A": 12.0, "v0B": 12.0}
    keep, extra, k = queue_walk("a4", ab, 1.0, pa, 12.0)
    assert k == 2 and abs(extra - 2 * (14.0 + L_CAR) / 12.0) < 1e-9 and keep == {"AB", "BC", "CD"}, (keep, extra, k)
    need = C1.S0 + 12.0 * C1.HEADWAY + L_CAR + pa["buffer"] + 12.0 * pa["tau"]
    keep, extra, k = queue_walk("a", ab, (need - C1.S0) / 12.0 + 0.01, pa, 12.0)
    assert k == 0 and extra == 0.0 and "AC" in keep
    keep, extra, k = queue_walk("a", dict(ab, a_pos="ahead"), 1.0, pa, 12.0)
    assert keep == {"AB", "BC"} and extra == 0.0
    # v2 matches n2.compose_v2 on a three-car scene
    mk = lambda **k: {"lostA": 0.1, "lostB": 0.02, "lostC": 0.03, "contactA": 0, "contactB": 0, "contactC": 0,
                      "peakA": 1.0, "peakB": 2.0, "peakC": 0.5, "latA": 1.0, "latB": 0.0, "latC": 0.0,
                      "v0A": 12.0, "v0B": 12.0, "v0C": 12.0, "a_pos": "behind", **k}
    pr = {"AB": mk(rho_AB=0.1), "AC": mk(rho_AC=0.3, a_pos="ahead"), "BC": mk(rho_BC=0.0)}
    for car in "ABC":
        assert abs(compose_v2("a", car, pr, 1.0, pa, 12.0)[0] - N2.compose_v2("a", car, pr, 1.0, pa)[0]) < 1e-12, car
        assert abs(compose("a", car, pr, 1.0)[0] - N2.compose("a", car, pr, 1.0)[0]) < 1e-12, car
    # the shift: closing at 2 m/s on L 15 m ahead, IDM's s* 21.7 m: A brakes from the start
    ab = {"v0A": 14.0, "v0B": 12.0, "d0": 6.0, "dec_t": 11.0}
    assert shift("b", {"AB": ab}, 15.0, 12.0) == (0.0, 7.0)  # inside s* = 21.7 m from the start
    assert shift("b", {"AB": dict(ab, dec_t=3.0)}, 30.0, 12.0) is None  # decides first (t_meet 4.2 s)
    assert shift("b", {"AB": dict(ab, dec_t=0.3)}, 15.0, 12.0) is None  # decides within TAU_S
    assert shift("b", {"AB": dict(ab, v0A=12.0)}, 15.0, 12.0) is None  # g_meet 14 m: never reached
    # least squares recovers a plane
    X = [(1.0, a, b) for a in range(3) for b in range(3)]
    th = lstsq(X, [0.5 + 2 * a - b for _, a, b in X], ridge=0.0)
    assert all(abs(t - e) < 1e-9 for t, e in zip(th, (0.5, 2.0, -1.0))), th
    print("n3_model selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=("run", "analyze", "d2"))
    ap.add_argument("--set", default="fit")
    ap.add_argument("--ports", default="2600,2610,2620")
    ap.add_argument("--port", type=int, default=2600)
    ap.add_argument("--out")
    ap.add_argument("--k", type=int, default=6)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    os.makedirs(a.out, exist_ok=True)
    if a.mode == "run":
        ports = [int(p) for p in a.ports.split(",")]
        for name in a.set.split(","):
            work = jobs(name)
            print(f"[{name}] {len(work)} group runs on {ports}", flush=True)
            N2.run_pool(ports, a.k, "groups", work, os.path.join(a.out, f"{name}.jsonl"), chunk=4 * a.k)
    elif a.mode == "d2":
        d2(a.port, a.out)
    else:
        analyze(a.out)


if __name__ == "__main__":
    mp.set_start_method("spawn")
    main()
