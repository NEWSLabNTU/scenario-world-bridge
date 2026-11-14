#!/usr/bin/env python3
"""Roadmap 074 C4 and D5: fit explanations of the cut-in's measured play, and say which the data
identify.

The game per context (d0, dv, v_B) is C1's, on the arena's measured payoffs: A goes, waits (drops
behind) or goes if it sees B yield; B pre-yields, or reacts and then yields or holds once it has
seen A's lane change (sequential: the continuation is B's reply at that subgame). A table holds,
per context and profile, the mean utilities and the observed outcome shares over seeds.

Observed play is what a pair of cars did per run, reduced to an outcome both sides can see:
A ahead / A behind / A not in / contact, times B braked (peak braking >= YIELD_PEAK) or not. A
model gives each profile a probability; the outcome's probability is the profiles' outcome shares
weighted by it, so play is fitted on what cars do, not on strategy labels nobody observes.

Models (each a distribution over profiles from the table):
  nash       logit QRE of the selfish game (lambda)
  svo        logit QRE of the SVO game, U'_i = cos(phi_i) U_i + sin(phi_i) U_j (phi_A, phi_B, lambda)
  potential  softmax over joint profiles of Phi = cos(alpha) U_A + sin(alpha) U_B (alpha, lambda):
             the logit dynamics' stationary play when both act as if sharing one potential
  levelk     cognitive hierarchy: level 0 uniform, level k logit-replies to levels < k, Poisson(tau)
             over levels 0..4 (tau, lambda)
  kappa      threshold rules: A goes iff its belief p0 that B yields exceeds the table's
             kappa = (U_wait - U_go,hold) / (U_go,yield - U_go,hold) (a logit reply to the belief),
             B yields once it sees A's move iff U_B(yield) - U_B(hold) > -c_B, pre-yields by a
             logit reply to its belief q0 that A goes (p0, q0, c_B, lambda)
Sequential B under every model but potential: the continuation is a logit reply at the subgame.

Data (`--collect`, three own CARLAs through double_oracle's arena workers, every run seeded
crc32(context:seed:salt), so all profiles and pairs of a context meet the same noise):
  grid    C2a's grid (d0 -6..30 step 2, dv -4..4, v_B 12), C1's 9 profiles x 4 seeds: the tables
  play    the same grid played by do6's equilibrium rules (A2 / B3) and by the Autoware-like pair
          (A_aw_perm / B_aw_react), 4 seeds: measured play of a known preference (selfish) and a
          known rule
  vb6     C3's six cells at v_B 6, the 9 profiles x 8 seeds: the tables for the real stack
Then (`--fit`):
  - each model fitted to each play set by maximum likelihood; profile likelihood (LR 95 %, chi2_1
    / 2 = 1.92) and a cell bootstrap per parameter; 5-fold held-out (cells) payoff RMSE and log loss
  - recovery: play sampled from planted SVO types on the arena's tables, refitted (does the fit
    find phi, and how wide is it?)
  - the confound: kinematic tables (cutin_game) with B's reaction 0.5-2.0 s; play from a slow,
    selfish B fitted as an SVO angle on a normal car's table, and the joint (reaction, phi_B)
    profile likelihood
  - the real stack: B = Autoware (C3a ground truth, C3b CenterPoint), A the agent's two rules;
    B's yield per run against the vb6 tables (only B's continuation is a choice there)

  scripts/coop/c4_fit.py --collect --ports 2600,2610,2620 --out <dir>
  scripts/coop/c4_fit.py --fit --out <dir> --c3gt <dir> --c3cp <dir>...
  scripts/coop/c4_fit.py --selftest
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import random
import sys
import time
import zlib
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

A_ACTS = ("go", "wait", "go_if_yield")
B_ACTS = ("pre_yield", "react_yield", "react_hold")
PROFILES = [(a, b) for a in A_ACTS for b in B_ACTS]
OUTCOMES = ("ahead_y", "ahead_n", "behind_y", "behind_n", "out_y", "out_n", "contact")
YIELD_PEAK = 0.5  # m/s^2: B braked (measures.BRAKE_START)
GRID_D0 = [float(d) for d in range(-6, 31, 2)]
GRID_DV = [float(v) for v in range(-4, 5)]
C3_CELLS = [(2.0, 2.0), (6.0, 0.0), (10.0, 0.0), (10.0, -2.0), (14.0, -3.0), (20.0, 1.0)]
SMOOTH = 0.25  # pseudo-count per outcome in a profile's outcome shares
LR95 = 1.92  # chi2_1(0.95) / 2


def outcome(r):
    """One run's observed outcome."""
    if int(r.get("contact", 0)):
        return "contact"
    a = "out" if not int(r.get("got_in", 0)) else ("ahead" if int(r.get("ahead", 0)) else "behind")
    return f"{a}_{'y' if float(r.get('peakB', 0.0) or 0.0) >= YIELD_PEAK else 'n'}"


# --- tables --------------------------------------------------------------------------------------
class Tables:
    """Per cell: mean uA, uB (C x 9) and outcome shares (C x 9 x 7) over seeds."""

    def __init__(self, rows):
        by = collections.defaultdict(list)
        for r in rows:
            by[((float(r["d0"]), float(r["dv"])), (r["a"], r["b"]))].append(r)
        self.cells = sorted({k[0] for k in by})
        C = len(self.cells)
        self.uA = np.zeros((C, 9))
        self.uB = np.zeros((C, 9))
        self.share = np.zeros((C, 9, len(OUTCOMES)))
        self.n = np.zeros((C, 9))
        for i, c in enumerate(self.cells):
            for j, p in enumerate(PROFILES):
                rs = by.get((c, p), [])
                if not rs:
                    raise ValueError(f"no runs for {c} {p}")
                self.uA[i, j] = np.mean([float(r["uA"]) for r in rs])
                self.uB[i, j] = np.mean([float(r["uB"]) for r in rs])
                cnt = collections.Counter(outcome(r) for r in rs)
                v = np.array([cnt[o] + SMOOTH for o in OUTCOMES])
                self.share[i, j] = v / v.sum()
                self.n[i, j] = len(rs)

    def subset(self, idx):
        t = object.__new__(Tables)
        t.cells = [self.cells[i] for i in idx]
        t.uA, t.uB, t.share, t.n = self.uA[idx], self.uB[idx], self.share[idx], self.n[idx]
        return t

    def index(self):
        return {c: i for i, c in enumerate(self.cells)}


def from_arrays(uA, uB, share, cells=None):
    t = object.__new__(Tables)
    t.uA, t.uB, t.share = np.asarray(uA, float), np.asarray(uB, float), np.asarray(share, float)
    t.cells = cells or list(range(len(t.uA)))
    t.n = np.ones(t.uA.shape)
    return t


# --- the models ----------------------------------------------------------------------------------
def _sm(x, axis=-1):
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


def _sig(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50, 50)))


def _mats(T, ua, ub):
    """C x 3 x 3 arrays [a, b] of the two (transformed) utilities."""
    return ua.reshape(-1, 3, 3), ub.reshape(-1, 3, 3)


def _sequential(UA, UB, lam, cont_shift=0.0):
    """B's continuation under react (a logit reply at the subgame, per A action) and the reduced
    3 x 2 game (B: pre_yield, react): returns py (C x 3), VA, VB (C x 3 x 2)."""
    py = _sig(lam * (UB[:, :, 1] - UB[:, :, 2] + cont_shift))
    VA = np.stack([UA[:, :, 0], py * UA[:, :, 1] + (1 - py) * UA[:, :, 2]], axis=2)
    VB = np.stack([UB[:, :, 0], py * UB[:, :, 1] + (1 - py) * UB[:, :, 2]], axis=2)
    return py, VA, VB


def _joint(pA, pB, py):
    """C x 9 profile probabilities from A's (C x 3), B's reduced (C x 2) and the continuation."""
    P = np.zeros((pA.shape[0], 3, 3))
    P[:, :, 0] = pA * pB[:, :1]
    P[:, :, 1] = pA * pB[:, 1:2] * py
    P[:, :, 2] = pA * pB[:, 1:2] * (1 - py)
    return P.reshape(-1, 9)


def qre(VA, VB, lam, iters=300, damp=0.5):
    C = VA.shape[0]
    pA = np.full((C, 3), 1 / 3)
    pB = np.full((C, 2), 0.5)
    for _ in range(iters):
        eA = np.einsum("cab,cb->ca", VA, pB)
        eB = np.einsum("cab,ca->cb", VB, pA)
        nA, nB = _sm(lam * eA), _sm(lam * eB)
        if np.abs(nA - pA).max() < 1e-9 and np.abs(nB - pB).max() < 1e-9:
            pA, pB = nA, nB
            break
        pA, pB = damp * pA + (1 - damp) * nA, damp * pB + (1 - damp) * nB
    return pA, pB


def svo_utils(T, phi_a, phi_b):
    ca, sa, cb, sb = math.cos(math.radians(phi_a)), math.sin(math.radians(phi_a)), \
        math.cos(math.radians(phi_b)), math.sin(math.radians(phi_b))
    return ca * T.uA + sa * T.uB, cb * T.uB + sb * T.uA


def pred_nash(T, lam):
    return pred_svo(T, 0.0, 0.0, lam)


def pred_svo(T, phi_a, phi_b, lam):
    ua, ub = svo_utils(T, phi_a, phi_b)
    UA, UB = _mats(T, ua, ub)
    py, VA, VB = _sequential(UA, UB, lam)
    pA, pB = qre(VA, VB, lam)
    return _joint(pA, pB, py)


def pred_potential(T, alpha, lam):
    phi = math.cos(math.radians(alpha)) * T.uA + math.sin(math.radians(alpha)) * T.uB
    return _sm(lam * phi)


def pred_levelk(T, tau, lam, kmax=4):
    UA, UB = _mats(T, T.uA, T.uB)
    py, VA, VB = _sequential(UA, UB, lam)
    C = UA.shape[0]
    w = np.array([math.exp(-tau) * tau ** k / math.factorial(k) for k in range(kmax + 1)])
    LA, LB = [np.full((C, 3), 1 / 3)], [np.full((C, 2), 0.5)]
    PY = [np.full((C, 3), 0.5)]  # level 0 continues at random
    for k in range(1, kmax + 1):
        wk = w[:k] / w[:k].sum()
        mB = sum(wi * x for wi, x in zip(wk, LB))
        mA = sum(wi * x for wi, x in zip(wk, LA))
        # what a level-k A expects of B's continuation: the lower levels' mixture
        mpy = sum(wi * lb[:, 1:2] * pyk for wi, lb, pyk in zip(wk, LB, PY)) / np.maximum(mB[:, 1:2], 1e-12)
        VAk = np.stack([UA[:, :, 0], mpy * UA[:, :, 1] + (1 - mpy) * UA[:, :, 2]], axis=2)
        LA.append(_sm(lam * np.einsum("cab,cb->ca", VAk, mB)))
        LB.append(_sm(lam * np.einsum("cab,ca->cb", VB, mA)))
        PY.append(py)
    w = w / w.sum()
    pA = sum(wi * x for wi, x in zip(w, LA))
    P = np.zeros((C, 3, 3))
    for wi, lb, pyk in zip(w, LB, PY):
        P[:, :, 0] += wi * lb[:, :1] * np.ones((1, 3))
        P[:, :, 1] += wi * lb[:, 1:2] * pyk
        P[:, :, 2] += wi * lb[:, 1:2] * (1 - pyk)
    # A and B independent: P[a, b] = pA[a] * PB[b | a]
    return (pA[:, :, None] * P).reshape(-1, 9)


def kappa_of(T):
    """The table's kappa per cell: (U_wait - U_go,hold) / (U_go,yield - U_go,hold), on react rows."""
    UA = T.uA.reshape(-1, 3, 3)
    den = UA[:, 0, 1] - UA[:, 0, 2]
    num = UA[:, 1, 2] - UA[:, 0, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        k = np.where(np.abs(den) > 1e-9, num / den, np.where(num > 0, np.inf, -np.inf))
    return k


def pred_kappa(T, p0, q0, c_b, lam):
    UA, UB = _mats(T, T.uA, T.uB)
    py = _sig(lam * (UB[:, :, 1] - UB[:, :, 2] + c_b))
    # A's reply to its belief p0 that B yields (pre-yield or react-yield count as a yield)
    eA = p0 * UA[:, :, 1] + (1 - p0) * UA[:, :, 2]
    pA = _sm(lam * eA)
    # B's reply (pre-yield or react) to its belief q0 that A goes (else waits)
    VB = np.stack([UB[:, :, 0], py * UB[:, :, 1] + (1 - py) * UB[:, :, 2]], axis=2)
    qa = np.array([q0, 1 - q0, 0.0])
    pB = _sm(lam * np.einsum("cab,a->cb", VB, qa))
    return _joint(pA, pB, py)


# parameter vectors: (name, transform to the model's value, inverse, bounds in the model's units)
def _lam(x):
    return math.exp(min(20.0, max(-20.0, x)))


MODELS = {
    "nash": (pred_nash, [("lambda", _lam, math.log, (0.5, 1e5))]),
    "svo": (pred_svo, [("phi_A", float, float, (-60.0, 90.0)), ("phi_B", float, float, (-60.0, 90.0)),
                       ("lambda", _lam, math.log, (0.5, 1e5))]),
    "potential": (pred_potential, [("alpha", float, float, (-90.0, 180.0)), ("lambda", _lam, math.log, (0.5, 1e5))]),
    "levelk": (pred_levelk, [("tau", _lam, math.log, (0.05, 20.0)), ("lambda", _lam, math.log, (0.5, 1e5))]),
    "kappa": (pred_kappa, [("p0", lambda x: float(_sig(x)), lambda p: math.log(p / (1 - p)), (0.001, 0.999)),
                           ("q0", lambda x: float(_sig(x)), lambda p: math.log(p / (1 - p)), (0.001, 0.999)),
                           ("c_B", float, float, (-1.0, 1.0)), ("lambda", _lam, math.log, (0.5, 1e5))]),
}
STARTS = {
    "nash": [[20.0], [200.0], [3000.0]],
    "svo": [[0.0, 0.0, 20.0], [0.0, 30.0, 50.0], [20.0, 0.0, 200.0], [-20.0, 45.0, 100.0], [0.0, 0.0, 2000.0],
            [10.0, 10.0, 5000.0]],
    "potential": [[45.0, 20.0], [0.0, 100.0], [90.0, 50.0], [10.0, 2000.0]],
    "levelk": [[1.5, 20.0], [0.5, 100.0], [4.0, 200.0], [8.0, 2000.0]],
    "kappa": [[0.5, 0.5, 0.0, 20.0], [0.2, 0.8, 0.0, 100.0], [0.8, 0.5, 0.05, 50.0], [0.95, 0.05, 0.0, 500.0]],
}


def predict(model, T, theta):
    return MODELS[model][0](T, *theta)


def obs_counts(T, play_rows):
    """C x 7 observed outcome counts of the play rows on T's cells (rows of other cells ignored)."""
    idx = T.index()
    Y = np.zeros((len(T.cells), len(OUTCOMES)))
    for r in play_rows:
        c = (float(r["d0"]), float(r["dv"]))
        if c in idx:
            Y[idx[c], OUTCOMES.index(outcome(r))] += 1
    return Y


def nll(model, T, Y, theta):
    P = predict(model, T, theta)
    po = np.einsum("cp,cpo->co", P, T.share)
    return float(-(Y * np.log(np.maximum(po, 1e-12))).sum())


def _clip(model, theta):
    return [min(hi, max(lo, v)) for v, (_, _, _, (lo, hi)) in zip(theta, MODELS[model][1])]


def fit(model, T, Y, fixed=None, starts=None):
    """Maximum likelihood; fixed: {param index: value} held fixed (profile likelihood)."""
    from scipy.optimize import minimize
    spec = MODELS[model][1]
    fixed = fixed or {}
    free = [i for i in range(len(spec)) if i not in fixed]
    best = None
    for s in (starts or STARTS[model]):
        s = _clip(model, s)
        x0 = [spec[i][2](s[i]) for i in free]

        def f(x):
            th = list(s)
            for i, xi in zip(free, x):
                th[i] = spec[i][1](xi)
            for i, v in fixed.items():
                th[i] = v
            th = _clip(model, th)
            return nll(model, T, Y, th)
        if free:
            r = minimize(f, x0, method="Nelder-Mead", options={"xatol": 1e-3, "fatol": 1e-4, "maxiter": 600})
            x, v = r.x, r.fun
        else:
            x, v = [], f([])
        th = list(s)
        for i, xi in zip(free, x):
            th[i] = spec[i][1](xi)
        for i, val in fixed.items():
            th[i] = val
        th = _clip(model, th)
        if best is None or v < best[1]:
            best = (th, v)
    return best


def profile(model, T, Y, theta_hat, v_hat, i, grid):
    """The profile likelihood of parameter i over grid: [(value, nll - v_hat)], and the LR 95 % set."""
    out = []
    for g in grid:
        th, v = fit(model, T, Y, fixed={i: g}, starts=[[g if k == i else theta_hat[k] for k in range(len(theta_hat))]])
        out.append((g, min(v, v_hat + 1e9) - v_hat))
    inside = [g for g, d in out if d <= LR95]
    return out, (min(inside), max(inside)) if inside else (None, None)


def profile_ci(model, T, Y, theta_hat, v_hat, i, n_bisect=6):
    """The LR 95 % interval of parameter i: from the estimate, step outward on each side (lambda and
    the other log-scaled parameters by factors of 2, the rest by growing steps) until the profile
    rises by LR95 or the bound is reached, then bisect. Returns (lo, hi); a bound means unbounded there."""
    name, fwd, inv, (lo_b, hi_b) = MODELS[model][1][i]
    logs = fwd is _lam
    x0 = inv(theta_hat[i]) if logs else theta_hat[i]
    lo_x, hi_x = (inv(lo_b), inv(hi_b)) if logs else (lo_b, hi_b)
    if name in ("p0", "q0"):
        x0, lo_x, hi_x = theta_hat[i], lo_b, hi_b
    step0 = 0.35 if logs else max(0.02 * (hi_x - lo_x), 1e-3)

    def dv(x):
        g = math.exp(x) if logs else x
        st_ = [g if k == i else theta_hat[k] for k in range(len(theta_hat))]
        return fit(model, T, Y, fixed={i: g}, starts=[st_])[1] - v_hat

    ends = []
    for sgn, bound in ((-1, lo_x), (1, hi_x)):
        inside, step, x = x0, step0, x0
        out = None
        while True:
            x = x0 + sgn * step
            if (x - bound) * sgn >= 0:
                if dv(bound) <= LR95:
                    out = None
                    inside = bound
                else:
                    out = bound
                break
            if dv(x) > LR95:
                out = x
                break
            inside = x
            step *= 2
        if out is not None:
            for _ in range(n_bisect):
                mid = (inside + out) / 2
                if dv(mid) > LR95:
                    out = mid
                else:
                    inside = mid
        ends.append(math.exp(inside) if logs else inside)
    return tuple(ends)


def pay_pred(model, T, theta):
    P = predict(model, T, theta)
    return (P * T.uA).sum(1), (P * T.uB).sum(1)


def obs_pay(T, play_rows):
    idx = T.index()
    s = collections.defaultdict(list)
    for r in play_rows:
        c = (float(r["d0"]), float(r["dv"]))
        if c in idx:
            s[idx[c]].append((float(r["uA"]), float(r["uB"])))
    ua = np.array([np.mean([x[0] for x in s[i]]) if s[i] else np.nan for i in range(len(T.cells))])
    ub = np.array([np.mean([x[1] for x in s[i]]) if s[i] else np.nan for i in range(len(T.cells))])
    return ua, ub


def heldout(model, T, play_rows, k=5, seed=0, clip_u=-2.0, start=None):
    """k-fold over cells: payoff RMSE (uA and uB, each clipped below at clip_u so that one contact
    of -10 does not decide it) and outcome log loss per run, on the held-out cells."""
    idx = list(range(len(T.cells)))
    random.Random(seed).shuffle(idx)
    folds = [idx[i::k] for i in range(k)]
    se, n, ll, nr = 0.0, 0, 0.0, 0
    for f in folds:
        tr = sorted(set(idx) - set(f))
        Ttr, Tte = T.subset(tr), T.subset(sorted(f))
        th, _ = fit(model, Ttr, obs_counts(Ttr, play_rows), starts=[start] if start else None)
        pa, pb = pay_pred(model, Tte, th)
        oa, ob = obs_pay(Tte, play_rows)
        m = ~np.isnan(oa)
        se += float(((np.maximum(pa, clip_u) - np.maximum(oa, clip_u))[m] ** 2).sum()
                    + ((np.maximum(pb, clip_u) - np.maximum(ob, clip_u))[m] ** 2).sum())
        n += 2 * int(m.sum())
        Yte = obs_counts(Tte, play_rows)
        ll += nll(model, Tte, Yte, th)
        nr += int(Yte.sum())
    return math.sqrt(se / max(n, 1)), ll / max(nr, 1)


def bootstrap(model, T, play_rows, B=40, seed=1, start=None):
    """Cell bootstrap of the estimates: per parameter (2.5 %, 97.5 %)."""
    rng = np.random.default_rng(seed)
    ests = []
    C = len(T.cells)
    for _ in range(B):
        idx = sorted(rng.integers(0, C, C).tolist())
        Tb = T.subset(idx)
        # the play of a resampled cell counts as often as the cell is drawn
        Y = obs_counts(T, play_rows)[idx]
        ests.append(fit(model, Tb, Y, starts=[start] if start else None)[0])
    E = np.array(ests)
    return [(float(np.percentile(E[:, j], 2.5)), float(np.percentile(E[:, j], 97.5))) for j in range(E.shape[1])]


def sample_play(model, T, theta, n, seed):
    """Synthetic play rows: n runs per cell, a profile from the model, then an outcome from its shares."""
    rng = np.random.default_rng(seed)
    P = predict(model, T, theta)
    rows = []
    for i, c in enumerate(T.cells):
        for _ in range(n):
            j = rng.choice(9, p=P[i] / P[i].sum())
            o = OUTCOMES[rng.choice(len(OUTCOMES), p=T.share[i, j])]
            rows.append(_row_of(c, o, T.uA[i, j], T.uB[i, j]))
    return rows


def _row_of(c, o, ua, ub):
    a, _, y = o.partition("_")
    return {"d0": c[0], "dv": c[1], "uA": ua, "uB": ub, "contact": int(o == "contact"),
            "got_in": int(a in ("ahead", "behind")), "ahead": int(a == "ahead"), "peakB": 1.0 if y == "y" else 0.0}


# --- collection in the arena ---------------------------------------------------------------------
def submit(pool, jobs, tag):
    n = len(pool.inbox)
    shards = [jobs[i::n] for i in range(n)]
    for i, sh in enumerate(shards):
        pool.inbox[i].put((f"{tag}-{time.time()}-{i}", sh))
    out = [None] * len(jobs)
    for _ in range(n):
        t, rows = pool.outbox.get()
        i = int(t.rsplit("-", 1)[1])
        for k, r in enumerate(rows):
            out[i + k * n] = r
    pool.runs += len(jobs)
    return out


def collect(a):
    import multiprocessing as mp
    import double_oracle as DO
    pops = json.load(open(a.do6))
    by_a = {p["name"]: p for p in pops["A"]}
    by_b = {p["name"]: p for p in pops["B"]}
    eq_a, eq_b = by_a["A2_s1l40_2"], by_b["B3_s1l42_4"]
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    mp.set_start_method("spawn", force=True)
    pool = DO.Pool([int(p) for p in a.ports.split(",")], a.k)
    seed = lambda d0, dv, s, salt: zlib.crc32(f"{d0}:{dv}:{s}:{salt}".encode())
    try:
        sets = {
            "grid": [{"d0": d0, "dv": dv, "vb": 12.0, "a": x, "b": y, "seed": s, "rng_seed": seed(d0, dv, s, 40)}
                     for d0 in GRID_D0 for dv in GRID_DV for (x, y) in PROFILES for s in range(a.seeds)],
            "play": [{"d0": d0, "dv": dv, "vb": 12.0, "pa": pa, "pb": pb, "a": pa["name"], "b": pb["name"], "seed": s,
                      "rng_seed": seed(d0, dv, s, 41)}
                     for d0 in GRID_D0 for dv in GRID_DV for pa, pb in ((eq_a, eq_b), (DO.A_AW_PERM, DO.B_AW_REACT))
                     for s in range(a.seeds)],
            "vb6": [{"d0": d0, "dv": dv, "vb": 6.0, "a": x, "b": y, "seed": s, "rng_seed": seed(d0, dv, s, 42)}
                    for (d0, dv) in C3_CELLS for (x, y) in PROFILES for s in range(2 * a.seeds)],
        }
        for name, jobs in sets.items():
            f = out / f"{name}.jsonl"
            if f.exists():
                continue
            t0 = time.time()
            rows = submit(pool, jobs, name)
            with open(f, "w") as fh:
                for j, r in zip(jobs, rows):
                    fh.write(json.dumps({**{k: j[k] for k in ("d0", "dv", "vb", "a", "b", "seed")}, **r}) + "\n")
            print(f"{name}: {len(jobs)} runs, {time.time() - t0:.0f} s", flush=True)
    finally:
        pool.close()


def load(p):
    return [json.loads(line) for line in open(p) if line.strip()]


# --- the kinematic confound ----------------------------------------------------------------------
def kin_tables(reaction_b, vb=12.0, cells=None):
    """cutin_game's tables (one deterministic run per profile) with B's reaction time set."""
    import dataclasses
    import cutin_game as C1
    cap_a = C1.TYPES["carla_model3"]
    cap_b = dataclasses.replace(C1.TYPES["carla_model3"], reaction=reaction_b)
    pref = C1.Preference()
    rows = []
    for d0, dv in (cells or [(d, v) for d in GRID_D0 for v in GRID_DV]):
        for x, y in PROFILES:
            b, c = (C1.PRE_YIELD, C1.HOLD) if y == "pre_yield" else (C1.REACT, C1.YIELD if y == "react_yield" else C1.HOLD)
            o = C1.simulate(x, b, d0, dv, vb, cap_a, cap_b, c)
            ahead = o.went and not o.contact
            rows.append({"d0": d0, "dv": dv, "a": x, "b": y, "uA": C1.utility(o, 0, cap_a, pref),
                         "uB": C1.utility(o, 1, cap_b, pref), "contact": int(o.contact),
                         "got_in": int(o.progress[0] < 1.0 and not o.contact), "ahead": int(ahead),
                         "peakB": o.peak_brake[1]})
    return Tables(rows)


# --- the analysis --------------------------------------------------------------------------------
def fmt(model, th):
    names = [s[0] for s in MODELS[model][1]]
    return ", ".join(f"{n} {v:.3g}" for n, v in zip(names, th))


def analyse_play(T, rows, label, L, res, boot=True):
    L.append(f"\n### {label}: {len(rows)} runs on {len(T.cells)} cells\n")
    L.append("| model | k | nll | AIC | estimate | LR 95 % sets | bootstrap 95 % | held-out payoff RMSE | held-out log loss / run |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    Y = obs_counts(T, rows)
    out = {}
    for m in MODELS:
        th, v = fit(m, T, Y)
        spec = MODELS[m][1]
        sets = []
        for i in range(len(spec)):
            sets.append(profile_ci(m, T, Y, th, v, i))
        bs = bootstrap(m, T, rows, B=20, start=th) if boot else [(None, None)] * len(th)
        rmse, logl = heldout(m, T, rows, start=th)
        out[m] = {"theta": th, "nll": v, "aic": 2 * len(th) + 2 * v, "lr95": sets, "boot95": bs,
                  "heldout_rmse": rmse, "heldout_logloss": logl}
        sstr = "; ".join(f"{n} [{a:.3g}, {b:.3g}]" if a is not None else f"{n} -" for (n, *_), (a, b) in zip(spec, sets))
        bstr = "; ".join(f"[{a:.3g}, {b:.3g}]" if a is not None else "-" for a, b in bs)
        L.append(f"| {m} | {len(th)} | {v:.1f} | {2 * len(th) + 2 * v:.1f} | {fmt(m, th)} | {sstr} | {bstr} | {rmse:.4f} | {logl:.3f} |")
        print(label, m, fmt(m, th), round(v, 1), sets, round(rmse, 4), flush=True)
    # a table-free baseline: every held-out cell gets the training cells' mean payoff
    oa, ob = obs_pay(T, rows)
    m_ = ~np.isnan(oa)
    base = math.sqrt(float(((np.maximum(oa[m_], -2) - np.maximum(oa[m_], -2).mean()) ** 2).mean() / 2
                           + ((np.maximum(ob[m_], -2) - np.maximum(ob[m_], -2).mean()) ** 2).mean() / 2))
    L.append(f"\nTable-free baseline (the mean payoff for every cell): RMSE {base:.4f}.")
    res[label] = {"models": out, "baseline_rmse": base}
    return out


def residuals_by_region(T, rows, model, th):
    """Per region (the selfish game's class letter is not at hand here: d0 bands x dv sign) the
    mean log likelihood per run, and the observed against predicted A-ahead share."""
    Y = obs_counts(T, rows)
    P = predict(model, T, th)
    po = np.einsum("cp,cpo->co", P, T.share)
    reg = collections.defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])
    for i, (d0, dv) in enumerate(T.cells):
        band = "d0<4" if d0 < 4 else ("4<=d0<14" if d0 < 14 else "d0>=14")
        key = (band, "dv>0" if dv > 0 else ("dv=0" if dv == 0 else "dv<0"))
        r = reg[key]
        r[0] += float((Y[i] * np.log(np.maximum(po[i], 1e-12))).sum())
        r[1] += float(Y[i].sum())
        r[2] += float(Y[i, :2].sum())
        r[3] += float(po[i, :2].sum() * Y[i].sum())
    return {f"{k[0]} {k[1]}": {"ll_per_run": v[0] / max(v[1], 1), "ahead_obs": v[2] / max(v[1], 1),
                                "ahead_pred": v[3] / max(v[1], 1), "runs": int(v[1])} for k, v in sorted(reg.items())}


def run_fit(a):
    out = Path(a.out)
    grid = load(out / "grid.jsonl")
    play = load(out / "play.jsonl")
    T = Tables(grid)
    L = ["# 074 C4: fits of the cut-in's play", "",
         f"Tables: {len(T.cells)} cells x 9 profiles, {len(grid)} arena runs (v_B 12). "
         f"Outcomes: A ahead / behind / not in / contact, x B braked (peak >= {YIELD_PEAK} m/s^2)."]
    res = {}
    eq_rows = [r for r in play if r["a"] == "A2_s1l40_2"]
    aw_rows = [r for r in play if r["a"] == "A_aw_perm"]
    fits_eq = analyse_play(T, eq_rows, "do6 equilibrium rules (A2 / B3), selfish by construction", L, res)
    analyse_play(T, aw_rows, "Autoware-like rules (A_aw_perm / B_aw_react)", L, res)
    best = min(fits_eq, key=lambda m: fits_eq[m]["aic"])
    res["regions_eq"] = {m: residuals_by_region(T, eq_rows, m, fits_eq[m]["theta"]) for m in fits_eq}
    L.append(f"\nResiduals by region, equilibrium play, best model by AIC ({best}):\n")
    L.append("| region | runs | log lik / run | A ahead observed | predicted |")
    L.append("|---|---|---|---|---|")
    for k, v in res["regions_eq"][best].items():
        L.append(f"| {k} | {v['runs']} | {v['ll_per_run']:.3f} | {v['ahead_obs']:.2f} | {v['ahead_pred']:.2f} |")

    # recovery: planted SVO types on the arena's tables
    L.append("\n### Recovery: play sampled from planted SVO types on the arena's tables (4 runs per cell)\n")
    L.append("| planted phi_A, phi_B, lambda | fitted | LR 95 % phi_A | LR 95 % phi_B | lambda |")
    L.append("|---|---|---|---|---|")
    rec = []
    for k, planted in enumerate([(0.0, 0.0, 50.0), (0.0, 30.0, 50.0), (0.0, 60.0, 50.0), (30.0, 0.0, 50.0), (0.0, 30.0, 10.0)]):
        rows = sample_play("svo", T, planted, 4, seed=100 + k)
        Y = obs_counts(T, rows)
        th, v = fit("svo", T, Y)
        sets = [profile_ci("svo", T, Y, th, v, i) for i in (0, 1)]
        lam_set = profile_ci("svo", T, Y, th, v, 2)
        rec.append({"planted": planted, "fitted": th, "phi_A": sets[0], "phi_B": sets[1], "lambda": lam_set})
        L.append(f"| {planted} | {fmt('svo', th)} | {sets[0]} | {sets[1]} | {lam_set} |")
        print("recovery", planted, th, sets, flush=True)
    res["recovery"] = rec

    # the confound: B's reaction (capability) against B's SVO angle (preference), kinematic
    L.append("\n### The confound: B's reaction time against B's SVO angle (kinematic tables, cutin_game)\n")
    reacts = [0.5, 1.0, 1.5, 2.0]
    KT = {r: kin_tables(r) for r in reacts}
    conf = []
    for true_r, true_phi in ((2.0, 0.0), (1.0, 0.0), (1.0, 30.0)):
        rows = sample_play("svo", KT[true_r], (0.0, true_phi, 50.0), 4, seed=7)
        grid2 = {}
        for r in reacts:
            Y = obs_counts(KT[r], rows)
            for phi in np.linspace(-60, 90, 11):
                grid2[(r, float(phi))] = fit("svo", KT[r], Y, fixed={1: float(phi)},
                                             starts=[[0.0, float(phi), 50.0], [20.0, float(phi), 10.0]])[1]
        vmin = min(grid2.values())
        inside = sorted(k for k, v in grid2.items() if v - vmin <= 2.99)  # chi2_2(0.95) / 2
        best_k = min(grid2, key=grid2.get)
        Y1 = obs_counts(KT[1.0], rows)
        th1, v1 = fit("svo", KT[1.0], Y1)
        s1 = profile_ci("svo", KT[1.0], Y1, th1, v1, 1)
        conf.append({"true": (true_r, true_phi), "best": best_k, "joint95": inside,
                     "fit_assuming_1s": th1, "phi_B_95_assuming_1s": s1})
        L.append(f"- true B reaction {true_r} s, phi_B {true_phi}: joint best {best_k}; joint 95 % set "
                 f"{len(inside)} of {len(grid2)} grid points, reactions {sorted({k[0] for k in inside})}, "
                 f"phi_B {min(k[1] for k in inside):.0f}..{max(k[1] for k in inside):.0f}; fitted assuming 1.0 s: "
                 f"{fmt('svo', th1)}, phi_B 95 % {s1}")
        print("confound", true_r, true_phi, best_k, s1, flush=True)
    res["confound"] = conf

    # the real stack
    if a.c3gt or a.c3cp:
        T6 = Tables(load(out / "vb6.jsonl"))
        L.append("\n### The real stack (C3): B = Autoware, A the agent's two rules; B's continuation only\n")
        L.append("| perception | runs | B braked share | phi_B (svo, A's rule fixed) | LR 95 % phi_B | c_B (kappa) | LR 95 % c_B |")
        L.append("|---|---|---|---|---|---|---|")
        res["c3"] = {}
        for name, dirs in (("ground truth (C3a)", a.c3gt), ("CenterPoint (C3b)", a.c3cp)):
            if not dirs:
                continue
            rows = []
            for d in dirs:
                for r in load(Path(d) / "c3.jsonl"):
                    if "uA" in r:
                        rows.append(dict(r, peakB=1.0 if r.get("t_brakeB") is not None and r.get("t_lat") is not None
                                         and r["t_brakeB"] >= r["t_lat"] - 0.5 else 0.0))
            fr = c3_fit_b(T6, rows)
            res["c3"][name] = fr
            L.append(f"| {name} | {len(rows)} | {fr['braked']:.2f} | {fr['phi_B']:.0f} | {fr['phi_B_95']} | "
                     f"{fr['c_B']:.3f} | {fr['c_B_95']} |")
    (out / "c4_fit.json").write_text(json.dumps(res, indent=1, default=float))
    (out / "c4_fit.md").write_text("\n".join(L) + "\n")
    print("\n".join(L))


def c3_fit_b(T6, rows):
    """Autoware's B: A's action is its rule's (A went: the go rows), B's continuation a logit reply;
    the likelihood of 'B braked' per run against the vb6 tables' react_yield / react_hold rows."""
    idx = T6.index()
    UA, UB = T6.uA.reshape(-1, 3, 3), T6.uB.reshape(-1, 3, 3)
    # P(B braked | continuation) from the tables' outcome shares (the go rows)
    yshare = T6.share.reshape(-1, 3, 3, len(OUTCOMES))
    ycols = [OUTCOMES.index(o) for o in ("ahead_y", "behind_y", "out_y")]
    pb_y = yshare[:, 0, 1, ycols].sum(-1) + 0.5 * yshare[:, 0, 1, OUTCOMES.index("contact")]
    pb_h = yshare[:, 0, 2, ycols].sum(-1) + 0.5 * yshare[:, 0, 2, OUTCOMES.index("contact")]
    data = [(idx[(float(r["d0"]), float(r["dv"]))], float(r["peakB"]) >= YIELD_PEAK) for r in rows
            if (float(r["d0"]), float(r["dv"])) in idx]

    def nll_b(phi, c, lam):
        ub = math.cos(math.radians(phi)) * UB[:, 0, :] + math.sin(math.radians(phi)) * UA[:, 0, :]
        py = _sig(lam * (ub[:, 1] - ub[:, 2] + c))
        pbr = py * pb_y + (1 - py) * pb_h
        return -sum(math.log(max(1e-12, pbr[i] if y else 1 - pbr[i])) for i, y in data)
    lam = 50.0
    phis = np.linspace(-60, 90, 31)
    cs = np.linspace(-0.3, 0.3, 31)
    vp = [min(nll_b(p, 0.0, l) for l in (10.0, 50.0, 200.0)) for p in phis]
    vc = [min(nll_b(0.0, c, l) for l in (10.0, 50.0, 200.0)) for c in cs]
    ip, ic = int(np.argmin(vp)), int(np.argmin(vc))
    sp = [p for p, v in zip(phis, vp) if v - vp[ip] <= LR95]
    sc = [c for c, v in zip(cs, vc) if v - vc[ic] <= LR95]
    return {"braked": float(np.mean([y for _, y in data])), "phi_B": float(phis[ip]),
            "phi_B_95": (float(min(sp)), float(max(sp))), "c_B": float(cs[ic]), "c_B_95": (float(min(sc)), float(max(sc))),
            "nll_phi": float(vp[ip]), "nll_c": float(vc[ic]), "lam": lam}


def selftest():
    rng = np.random.default_rng(0)
    # a random table of 30 cells with outcome shares
    C = 30
    uA = rng.normal(-0.3, 0.3, (C, 9))
    uB = rng.normal(-0.2, 0.2, (C, 9))
    share = rng.dirichlet(np.ones(len(OUTCOMES)), (C, 9))
    T = from_arrays(uA, uB, share, cells=[(float(i), 0.0) for i in range(C)])
    for m in MODELS:
        th = [s[0] if False else v for s, v in zip(MODELS[m][1], STARTS[m][0])]
        P = predict(m, T, th)
        assert P.shape == (C, 9) and np.allclose(P.sum(1), 1.0) and (P >= -1e-12).all(), m
    # SVO with phi 0 is the selfish model; potential at alpha 0 / 90 is A's / B's softmax
    assert np.allclose(pred_svo(T, 0, 0, 30.0), pred_nash(T, 30.0))
    # a dominant strategy: A's go pays 1 more in every profile; with a large lambda A goes
    uA2 = uA.copy()
    uA2.reshape(C, 3, 3)[:, 0, :] += 1.0
    T2 = from_arrays(uA2, uB, share, T.cells)
    pA = pred_svo(T2, 0, 0, 200.0).reshape(C, 3, 3).sum(2)
    assert (pA[:, 0] > 0.99).all()
    # kappa: go pays 0 against hold, 1 against yield, waiting 0.25 -> kappa 0.25
    uA3 = np.zeros((1, 9))
    uA3[0, 1], uA3[0, 2], uA3[0, 5] = 1.0, 0.0, 0.25  # (go, ry), (go, rh), (wait, rh)
    T3 = from_arrays(uA3, np.zeros((1, 9)), np.full((1, 9, len(OUTCOMES)), 1 / len(OUTCOMES)))
    assert abs(kappa_of(T3)[0] - 0.25) < 1e-9
    # p0 above kappa: A goes; below: it waits
    pa_hi = pred_kappa(T3, 0.6, 0.5, 0.0, 200.0).reshape(1, 3, 3).sum(2)[0]
    pa_lo = pred_kappa(T3, 0.1, 0.5, 0.0, 200.0).reshape(1, 3, 3).sum(2)[0]
    assert pa_hi[0] > 0.9 and pa_lo[1] > 0.9, (pa_hi, pa_lo)
    # recovery on a synthetic table: plant phi_B 45, fit, the LR set holds it
    uA4 = rng.normal(-0.3, 0.3, (80, 9))
    uB4 = rng.normal(-0.2, 0.2, (80, 9))
    sh4 = np.zeros((80, 9, len(OUTCOMES)))
    for j in range(9):
        sh4[:, j, j % len(OUTCOMES)] = 1.0  # outcome reveals the profile (up to aliasing)
    sh4 = (sh4 + 0.01) / (1 + 0.01 * len(OUTCOMES))
    T4 = from_arrays(uA4, uB4, sh4, [(float(i), 1.0) for i in range(80)])
    rows = sample_play("svo", T4, (0.0, 45.0, 30.0), 8, seed=3)
    Y = obs_counts(T4, rows)
    th, v = fit("svo", T4, Y)
    _, s = profile("svo", T4, Y, th, v, 1, sorted(set(list(np.arange(31.0, 61.0, 2.0)) + [th[1]])))
    assert s[0] <= 45.0 <= s[1] and abs(th[1] - 45.0) < 20, (th, s)
    ci = profile_ci("svo", T4, Y, th, v, 1)
    assert ci[0] <= 45.0 <= ci[1] and ci[1] - ci[0] < 30, ci
    # outcome labels
    assert outcome({"contact": 1}) == "contact"
    assert outcome({"got_in": 1, "ahead": 1, "peakB": 0.8}) == "ahead_y"
    assert outcome({"got_in": 0, "ahead": 0, "peakB": 0.1}) == "out_n"
    # held-out runs
    r, ll = heldout("nash", T4, rows, k=3)
    assert math.isfinite(r) and math.isfinite(ll)
    print("c4_fit selftest: ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--collect", action="store_true")
    ap.add_argument("--fit", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--do6", help="do6's populations.json (the equilibrium rules)")
    ap.add_argument("--ports", default="2600,2610,2620")
    ap.add_argument("--k", type=int, default=4)
    ap.add_argument("--seeds", type=int, default=4)
    ap.add_argument("--c3gt", nargs="*")
    ap.add_argument("--c3cp", nargs="*")
    a = ap.parse_args()
    if a.selftest:
        selftest()
    elif a.collect:
        collect(a)
    elif a.fit:
        run_fit(a)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
