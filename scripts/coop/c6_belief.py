#!/usr/bin/env python3
"""Roadmap 074 C6: hidden capability. The cut-in with car types, beliefs and their cost.

Types. Each car has a type t = (theta, phi): a capability theta (cutin_game.Capability: `m3`, the
Model 3 as CARLA measured it; `truck`, cutin_game's truck_like: 8 m, braking 4.5 m/s^2, drive
1.5 m/s^2, a 4 s lane change; `golfcart` where B's speed allows, v_max 6.9) and, for B, a
preference phi given as its rule: a yielder (B_aw_react: yields to a lane change seen within
20 m, 0.5 s, 2.5 m/s^2) or a holder (B_aw: only its safety layer brakes). A's rules are do6's
pool (double oracle, 2026-11-15) and A_wait (drop behind at once). Both cars see each other
through the arena's observer (10 Hz, 0.19 s delay, 0.3 m and 0.2 m/s noise, acceleration as
the 1 s slope); each car's limits bind (commands clamped to its a_max and b_max, the spec's
command lags; crossing_game.Actuator).

Belief. A's posterior over B's types (capability x preference), from a class prior and the
observed motion, filtered in every run:

  capability evidence  soft bounds: B seen decelerating at d (one 1 s slope per second) has
                       likelihood Phi((b_max - d) / SIGMA_A) under a type with braking b_max, and
                       likewise for acceleration and speed: a seen 6 m/s^2 stop rules the 4.5 m/s^2
                       truck out, one noisy sample prunes no type
  intent evidence      after A's lateral start, before A is in B's lane (B's safety layer brakes
                       for either type, so it is not intent): B seen braking is a yield
                       (P_SEEN_YIELD against P_FALSE_YIELD), not seen within PROBE_T a hold
  class cue            (filter `class`) B's box length, seen once with SIGMA_LEN noise

Policies. A chooses, at t = 0, one rule of the pool, or to probe: a nudge (0.12 of a lane, which
B sees as a lane change starting) held for PROBE_T, after which A picks the pool rule that is best
under its posterior then (the rule's clock starts there; it goes from the nudge or returns). A
chooses by expected utility over its belief: a selfish A by its own U_A, a prosocial A by welfare
-C. Information modes:

  prior   the class prior only (no probe: it cannot learn)
  motion  prior + capability and intent evidence (class-blind perception)
  class   motion + the class cue (the realistic perception)
  v2x     B's capability given (a V2X capability broadcast), intent inferred
  truth   B's type given (the same policy told the truth)

The expected payoffs a policy chooses by are the type-conditional means over the other seeds
(leave one seed out), so no run chooses on its own outcome.

Measures, per context (d0, dv, v_B) x A's capability x B's true type, from the paired seeds:
  misbelief regret   U(truth) - U(motion), U(truth) - U(class)
  value of info      U(v2x) - U(class) (a capability broadcast on top of perception),
                     U(class) - U(prior) (watching), U(truth) - U(prior) (the whole)
each in the chooser's objective, and the welfare change, as a share of stock's welfare loss:
the Autoware pair's (A_aw_perm against B_aw_react) cost minus the optimum over every rule pair,
and the worst pure or 2 x 2 mixed equilibrium of the pool's game minus the optimum. Also B's
choice of reaction (its pool) under a belief over A's capability and rule, the type-confusion
matrix (true type against the MAP belief) over time, and the cost of acting on a fixed belief
(assumed type x true type).

Usage:
  scripts/coop/c6_belief.py --out <dir> [--vb 12] [--seeds 8] [--procs 12] [--golfcart]
  scripts/coop/c6_belief.py --out <dir> --analyze
  scripts/coop/c6_belief.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import gzip
import itertools
import json
import math
import os
import random
import statistics as st
import sys
import zlib
from dataclasses import dataclass
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402

DT = C1.DT
LANE_W = C1.LANE_W

# --- types -------------------------------------------------------------------------------------
CAPS = {"m3": C1.TYPES["carla_model3"], "truck": C1.TYPES["truck_like"], "golfcart": C1.TYPES["golfcart"]}
PREFS = ("hold", "yield")
B_AW = {"name": "B_aw", "pre_band": -99.0, "react_gap": -99.0, "b_y": 2.0, "react": 0.5}
B_AW_REACT = {"name": "B_aw_react", "pre_band": -99.0, "react_gap": 20.0, "b_y": 2.5, "react": 0.5}
B_PREF_RULE = {"hold": B_AW, "yield": B_AW_REACT}
# B's pool (the optimum, B's own choice): do6's B rules; all but B_court differ only after A's
# lane change is seen
B_POOL = [B_AW, B_AW_REACT,
          {"name": "B_react", "pre_band": -99.0, "react_gap": 10.0, "b_y": 2.5, "react": 0.5},
          {"name": "B_r30", "pre_band": -99.0, "react_gap": 30.0, "b_y": 2.5, "react": 0.5},
          {"name": "B3", "pre_band": -99.0, "react_gap": 12.59, "b_y": 1.0, "react": 0.5},
          {"name": "B_court", "pre_band": 8.0, "react_gap": 15.0, "b_y": 2.0, "react": 1.0}]
B_REACTIVE = [b["name"] for b in B_POOL if b["pre_band"] < -98]
PROBE_LAT = 0.12  # share of a lane: B (lat_seen at 0.08) sees a lane change start
PROBE_RISE = 0.8  # s to reach the nudge
PROBE_T = 3.0  # s the nudge waits for B's answer (the arena's lateral lags its command, TAU_LAT)
INTENT_W = 1.6  # s after A's lateral start within which a yielder is seen braking (B's delay and
# reaction 0.7, the brake's lag 0.1, A's delay and slope 0.5)
A_POOL = [
    {"name": "A_aw_cons", "tau": 2.0, "a_rear": 1.0, "buffer": 2.0, "patience": 5.0, "use_yield": 0, "t_prep": 3.0},
    {"name": "A_aw_perm", "tau": 1.0, "a_rear": 2.0, "buffer": 2.0, "patience": 5.0, "use_yield": 0, "t_prep": 3.0},
    {"name": "A_bold", "tau": 0.3, "a_rear": 4.0, "buffer": 1.0, "patience": 2.0, "use_yield": 1, "t_prep": 0.0},
    {"name": "A2", "tau": 0.24, "a_rear": 4.01, "buffer": 5.12, "patience": 6.0, "use_yield": 0, "t_prep": 0.0},
    {"name": "A_wait", "tau": 99.0, "a_rear": 1.0, "buffer": 99.0, "patience": 0.0, "use_yield": 0, "t_prep": 0.0},
    # a control: A2 started PROBE_T late without the nudge (what a probe does to a holder, at no
    # cost to a yielder), so that the probe's worth is not a delay the pool lacks
    {"name": "A2_late", "tau": 0.24, "a_rear": 4.01, "buffer": 5.12, "patience": 6.0, "use_yield": 0, "t_prep": 0.0,
     "delay": PROBE_T},
]
A_NAMES = [a["name"] for a in A_POOL]
STOCK = ("A_aw_perm", "B_aw_react")  # the Autoware pair (C3's measured B)

P_TRUCK = 0.25  # the class prior: a share of trucks in traffic
P_GOLFCART = 0.1
P_YIELD = 0.5  # an even prior on intent
SIGMA_A = 0.4  # m/s^2: the 1 s slope of 10 Hz speeds with 0.2 m/s noise has se ~0.22, plus model error
SIGMA_VMAX = 0.5  # m/s
SIGMA_LEN = 0.8  # m, the box length seen (class cue)
P_SEEN_YIELD = 0.85
P_FALSE_YIELD = 0.08  # a holder seen "braking" from noise (the arena: about 1 run in 16)
TAU_LAT = 0.6  # s: A's lateral follows its command with a first-order lag, as the arena's car steers
MERGE_SETTLE = 1.5  # s: a run ends this long after the planned end of A's lane change, so a lagging
# lateral is not scored as a merge that never happened
FILTERS = ("motion", "class", "v2x")
MODES = ("prior", "prior_p", "motion", "class", "v2x", "truth")
T_SNAP = tuple(sorted({0.0, 1.0, 2.0, PROBE_T, 3.0, 5.0, 8.0}))


def bel_caps(vb):
    """The capabilities a belief ranges over: every car that can drive at B's speed."""
    return [c for c in CAPS if CAPS[c].v_max >= vb - 1e-6]


def b_types(caps):
    return [(c, p) for c in caps for p in PREFS]


def cap_prior(c):
    return {"m3": 1.0 - P_TRUCK, "truck": P_TRUCK, "golfcart": P_GOLFCART}[c]


def type_prior(caps):
    w = [cap_prior(c) * (P_YIELD if p == "yield" else 1 - P_YIELD) for c, p in b_types(caps)]
    return [x / sum(w) for x in w]


def phi(z):
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


class Belief:
    """A posterior over B's types [(cap, pref)]."""

    def __init__(self, types, prior, mode, true_cap=None):
        self.types, self.mode = types, mode
        if mode == "v2x":
            w = [q if t[0] == true_cap else 0.0 for t, q in zip(types, prior)]
            self.p = [x / sum(w) for x in w]
        else:
            self.p = list(prior)

    def _mul(self, lik):
        w = [p * max(l, 1e-12) for p, l in zip(self.p, lik)]
        s = sum(w)
        if s > 0:
            self.p = [x / s for x in w]

    def size(self, l_seen):
        if self.mode == "class":
            self._mul([math.exp(-0.5 * ((l_seen - CAPS[c].length) / SIGMA_LEN) ** 2) for c, _ in self.types])

    def capability(self, acc, v):
        """Soft bounds from one 1 s slope `acc` and one speed `v` seen of B."""
        if self.mode == "v2x":
            return
        lik = []
        for c, _ in self.types:
            cap = CAPS[c]
            l = phi((cap.v_max - v) / SIGMA_VMAX)
            l *= phi((cap.b_max + acc) / SIGMA_A) if acc < 0 else phi((cap.a_max - acc) / SIGMA_A)
            lik.append(l)
        self._mul(lik)

    def intent(self, seen_brake):
        py, ph = (P_SEEN_YIELD, P_FALSE_YIELD) if seen_brake else (1 - P_SEEN_YIELD, 1 - P_FALSE_YIELD)
        self._mul([py if p == "yield" else ph for _, p in self.types])

    def p_yield(self):
        return sum(q for (_, p), q in zip(self.types, self.p) if p == "yield")

    def map(self):
        return max(range(len(self.types)), key=lambda i: self.p[i])


# --- the run -----------------------------------------------------------------------------------
@dataclass
class Seen:
    t: float
    x: float  # the other's centre, in the observer's frame
    lat: float
    v: float
    acc: float


class Obs:
    """arena.Observer (10 Hz, delayed, noisy; acceleration as the 1 s slope), without carla."""

    def __init__(self, delay, sx, sv, rng):
        self.delay, self.sx, self.sv, self.rng = delay, sx, sv, rng
        self.buf, self.hist = [], []
        self.seen = None
        self.last = -1.0

    def push(self, t, x, lat, v):
        if t - self.last >= 0.1 - 1e-9:
            self.last = t
            self.buf.append((t, x + self.rng.gauss(0, self.sx), lat, v + self.rng.gauss(0, self.sv)))
        fresh = False
        while self.buf and t - self.buf[0][0] >= self.delay - 1e-9:
            tt, xx, ll, vv = self.buf.pop(0)
            self.hist.append((tt, vv))
            while self.hist and tt - self.hist[0][0] > 1.0 + 1e-9:
                self.hist.pop(0)
            acc = 0.0
            if len(self.hist) >= 5:
                n = len(self.hist)
                mt = sum(h[0] for h in self.hist) / n
                mv = sum(h[1] for h in self.hist) / n
                den = sum((h[0] - mt) ** 2 for h in self.hist)
                acc = sum((h[0] - mt) * (h[1] - mv) for h in self.hist) / den if den > 0 else 0.0
            self.seen = Seen(tt, xx, ll, vv, acc)
            fresh = True
        return self.seen, fresh


def lat_profile(s0, u):
    """Lateral share from s0 to 1 along the cosine profile, u in [0, 1]."""
    u = min(1.0, max(0.0, u))
    return s0 + (1.0 - s0) * 0.5 * (1.0 - math.cos(math.pi * u))


class PolicyA:
    """A's rule (do6's gap check, arena.Pair's A), optionally after a probe: a nudge held for
    PROBE_T, the rule's clock starting at its end. Returns (accel, lateral share)."""

    def __init__(self, pa, cap_a, len_b, react_a, probe=False):
        self.p, self.cap, self.lb = pa, cap_a, len_b
        self.react = react_a
        self.t0 = PROBE_T if probe else pa.get("delay", 0.0)
        self.probe = probe
        self.decide = None
        self.dropping = False
        self.b_brake_seen = None
        self.s = 0.0
        self.s_go = 0.0
        self.drop_t = None

    def merged(self, t):
        return self.decide is not None and t >= self.decide + self.p["t_prep"] + self.cap.t_lc + MERGE_SETTLE

    def step(self, t, xa, va, va0, sb, lat_now):
        p, cap, la, lb = self.p, self.cap, self.cap.length, self.lb
        xb_s, vb_s = sb.x, sb.v
        gap_ahead = (xa - la / 2) - (xb_s + lb / 2)
        tr = t - self.t0  # the rule's clock
        if sb.acc <= -C1.YIELD_SEEN and self.b_brake_seen is None:
            self.b_brake_seen = t
        if tr >= 0 and self.decide is None and not self.dropping:
            need = p["buffer"] + vb_s * p["tau"] + max(0.0, vb_s ** 2 - va ** 2) / (2 * p["a_rear"])
            if gap_ahead >= need:
                self.decide = t
            elif (p["use_yield"] and self.b_brake_seen is not None and t >= self.b_brake_seen + self.react
                  and gap_ahead > -la / 2):
                self.decide = t
            elif tr >= p["patience"] + (C1.WATCH if p["use_yield"] else 0.0):
                self.dropping = True
                self.drop_t = t
            if self.decide is not None:
                self.s_go = self.s
        gap_behind = (xb_s - lb / 2) - (xa + la / 2)
        if self.dropping and self.decide is None:
            want = C1.S0 + vb_s * C1.HEADWAY
            if gap_behind >= want:
                self.decide = t
                self.s_go = self.s
            aa = -C1.A_WAIT_DECEL if gap_behind < want and va > 0.5 * vb_s else 0.0
            if self.decide is not None:
                aa = C1.follow_cruise(gap_behind, va, vb_s, cap, va0)
        elif lat_now > 0.05 and xa < xb_s:
            aa = C1.follow_cruise(gap_behind, va, vb_s, cap, va0)
        else:
            aa = max(-1.0, min(cap.a_max, 1.5 * (va0 - va)))
        if self.decide is not None:
            if t >= self.decide + p["t_prep"]:
                u = (t - self.decide - p["t_prep"]) / (cap.t_lc * max(0.3, 1.0 - self.s_go))
                self.s = lat_profile(self.s_go, u)
        elif self.probe and not self.dropping and tr < 0:
            self.s = PROBE_LAT * min(1.0, t / PROBE_RISE)
        elif self.probe and self.s > 0:  # back from the nudge: waiting on, or dropping behind
            self.s = max(0.0, self.s - PROBE_LAT * DT / PROBE_RISE)
        return aa, self.s


class PolicyB:
    """B's rule (double_oracle's pb; arena.Pair's B without the stop layer)."""

    def __init__(self, pb, cap_b, react_b):
        self.p, self.cap, self.react = pb, cap_b, react_b
        self.lat_seen = self.in_lane_at = None
        self.latched = False

    def step(self, t, xb, vb, vb0, sa, la):
        p, cap, lb = self.p, self.cap, self.cap.length
        xa_s, lat_s, va_s = sa.x, sa.lat, sa.v
        if lat_s > 0.08 and self.lat_seen is None:
            self.lat_seen = t
        if lat_s >= C1.LANE_LINE and xa_s > xb and self.in_lane_at is None:
            self.in_lane_at = t
        gap_b = (xa_s - la / 2) - (xb + lb / 2)
        a_ahead = xa_s > xb - lb
        pre = t >= self.react and a_ahead and lat_s < C1.LANE_LINE and gap_b < p["pre_band"]
        react = self.lat_seen is not None and t >= self.lat_seen + self.react and a_ahead and gap_b < p["react_gap"]
        self.latched = self.latched or ((pre or react) and a_ahead)
        if self.latched and a_ahead:
            ab = -p["b_y"] if gap_b < C1.S0 + vb * C1.HEADWAY else C1.follow_cruise(gap_b, vb, va_s, cap, vb0)
        else:
            ab = max(-1.0, min(cap.a_max, 1.5 * (vb0 - vb)))
        if self.in_lane_at is not None and t >= self.in_lane_at + self.react and xa_s > xb:
            ab = min(ab, C1.follow_cruise(gap_b, vb, va_s, cap, vb0))  # the safety layer
        return ab


class Filters:
    """A's beliefs about B, passive: they watch, the policy chooses offline from their snapshots."""

    def __init__(self, vb, cap_b_name, l_seen):
        caps = bel_caps(vb)
        self.types = b_types(caps)
        pr = type_prior(caps)
        self.f = {m: Belief(self.types, pr, m, cap_b_name) for m in FILTERS}
        for b in self.f.values():
            b.size(l_seen)
        self.next_cap = 1.0
        self.t_on = None  # A's lateral start
        self.intent_done = False
        self.snaps = {}  # t -> {mode: p}
        self.k = 0

    def step(self, t, sb, fresh, s_a, b_brake_seen, gap=None, len_b=4.79):
        """gap: A's rear ahead of B's front as A sees it; at A's lateral start it says whether a
        yielder would brake (B_aw_react: within its react gap and closer than its following gap),
        so whether a brake seen or not says anything (intent evidence is strategic)."""
        if fresh and t >= self.next_cap and sb.t > 0.5:
            for b in self.f.values():
                b.capability(sb.acc, sb.v)
            self.next_cap = t + 1.0
        if self.t_on is None and s_a > 0.08:
            self.t_on = t
            if gap is not None and not (-len_b < gap < min(B_AW_REACT["react_gap"], C1.S0 + sb.v * C1.HEADWAY)):
                self.intent_done = True  # a yielder would not brake here: no intent evidence
        if self.t_on is not None and not self.intent_done:
            if b_brake_seen is not None and b_brake_seen >= self.t_on:
                seen = True
            elif t >= self.t_on + INTENT_W or s_a >= C1.LANE_LINE:
                seen = False
            else:
                seen = None
            if seen is not None:
                for b in self.f.values():
                    b.intent(seen)
                self.intent_done = True
        while self.k < len(T_SNAP) and t >= T_SNAP[self.k] - 1e-9:
            self.snaps[T_SNAP[self.k]] = {m: list(b.p) for m, b in self.f.items()}
            self.k += 1


OBS = {"delay": 0.19, "sx": 0.3, "sv": 0.2, "jitter": 0.2}


def run_seed(d0, dv, vb, seed, salt="c6"):
    return zlib.crc32(f"{d0}:{dv}:{vb}:{seed}:{salt}".encode())


def simulate(d0, dv, vb, cap_a_name, pa, cap_b_name, pb, seed, probe=False, trace=False):
    """One run of A's rule pa (after a probe if `probe`) against B's rule pb, with A's beliefs."""
    cap_a, cap_b = CAPS[cap_a_name], CAPS[cap_b_name]
    rng = random.Random(run_seed(d0, dv, vb, seed))
    react_a = 1.0 + rng.uniform(-OBS["jitter"], OBS["jitter"])
    react_b = pb.get("react", 1.0) + rng.uniform(-OBS["jitter"], OBS["jitter"])
    obs_a = Obs(OBS["delay"], OBS["sx"], OBS["sv"], rng)
    obs_b = Obs(OBS["delay"], OBS["sx"], OBS["sv"], rng)
    l_seen = cap_b.length + rng.gauss(0, SIGMA_LEN)
    va0 = max(0.0, min(cap_a.v_max, vb + dv))
    vb0 = min(cap_b.v_max, vb)
    A = PolicyA(pa, cap_a, cap_b.length, react_a, probe)
    B = PolicyB(pb, cap_b, react_b)
    F = Filters(vb, cap_b_name, l_seen)
    lags = X.car_lags()
    act_a, act_b = X.Actuator(lags), X.Actuator(lags)
    xb = 0.0
    xa = cap_b.length / 2 + d0 + cap_a.length / 2  # centres: A's rear d0 ahead of B's front
    va, vbb = va0, vb0
    meas = M.Measures(b_max=(cap_a.b_max, cap_b.b_max))
    peak = [0.0, 0.0]
    vh = ([], [])
    contact = False
    t = s_prev = 0.0
    sb = Seen(0.0, xb, 0.0, vbb, 0.0)
    sa = Seen(0.0, xa, 0.0, va, 0.0)
    rows = []
    lat = 0.0  # A's lateral as it is (its command lagged by TAU_LAT)
    k_lat = 1.0 - math.exp(-DT / TAU_LAT)
    while True:
        o, fresh = obs_a.push(t, xb - xa, 0.0, vbb)
        fresh = fresh and o is not None
        # before the first observation arrives, the truth (as arena.Pair)
        sb = Seen(o.t, xa + o.x, 0.0, o.v, o.acc) if o is not None else Seen(t, xb, 0.0, vbb, 0.0)
        o2, _ = obs_b.push(t, xa - xb, lat, va)
        sa = Seen(o2.t, xb + o2.x, o2.lat, o2.v, o2.acc) if o2 is not None else Seen(t, xa, lat, va, 0.0)
        aa, s_cmd = A.step(t, xa, va, va0, sb, lat)
        F.step(t, sb, fresh, lat, A.b_brake_seen, (xa - cap_a.length / 2) - (sb.x + cap_b.length / 2), cap_b.length)
        ab = B.step(t, xb, vbb, vb0, sa, cap_a.length)
        aa = max(-cap_a.b_max, min(cap_a.a_max, aa))
        ab = max(-cap_b.b_max, min(cap_b.a_max, ab))
        ea, eb = act_a.step(aa), act_b.step(ab)
        va = max(0.0, min(cap_a.v_max, va + ea * DT))
        vbb = max(0.0, min(cap_b.v_max, vbb + eb * DT))
        xa += va * DT
        xb += vbb * DT
        t += DT
        for k, vv in ((0, va), (1, vbb)):
            h = vh[k]
            h.append((t, vv))
            while h and t - h[0][0] > 0.5 + 1e-9:
                h.pop(0)
            if t - h[0][0] >= 0.5 - 1e-9:
                peak[k] = max(peak[k], (h[0][1] - vv) / 0.5)
        lat += (s_cmd - lat) * k_lat
        ya = LANE_W * (1.0 - lat)
        vya = -LANE_W * (lat - s_prev) / DT
        s_prev = lat
        vxa = math.sqrt(max(0.0, va * va - vya * vya))
        meas.push_poses(t, (xa, ya, math.atan2(vya, max(vxa, 0.1)), vxa, vya, cap_a.length, cap_a.width),
                        (xb, 0.0, 0.0, vbb, 0.0, cap_b.length, cap_b.width))
        if trace:
            rows.append((round(t, 2), round(xa, 2), round(va, 2), round(xb, 2), round(vbb, 2), round(s_cmd, 3), round(lat, 3)))
        if meas.clear <= 0.0:
            contact = True
            break
        if t >= C1.T_MAX - 1e-9 or (t >= C1.HORIZON - 1e-9 and (A.merged(t) or t >= 10.0 + C1.HORIZON)):
            break
    F.step(max(t, T_SNAP[-1]), sb, False, lat, A.b_brake_seen)  # snapshots past a short run's end
    lost_a = M.lost_progress(va0, meas.s_h[0], C1.HORIZON)
    lost_b = M.lost_progress(vb0, meas.s_h[1], C1.HORIZON)
    got_in = lat >= 0.9
    if contact:
        lost_a = lost_b = 1.0
    elif not got_in:
        lost_a = 1.0
    meas.pet = meas._pet()
    o = C1.Outcome((lost_a, lost_b), meas.ttc2d, meas.thw, contact, (peak[0], peak[1]), got_in and not A.dropping,
                   "", meas.clear, meas.pet, tuple(meas.lat_peak))
    pref = C1.Preference()
    rec = {"uA": round(C1.utility(o, 0, cap_a, pref), 4), "uB": round(C1.utility(o, 1, cap_b, pref), 4),
           "contact": int(contact), "ahead": int(got_in and xa > xb), "got_in": int(got_in),
           "clear": round(meas.clear, 3) if math.isfinite(meas.clear) else 99.0,
           "peakB": round(peak[1], 2), "t_decide": None if A.decide is None else round(A.decide, 2),
           "types": [f"{c}/{p}" for c, p in F.types], "snaps": F.snaps}
    if trace:
        rec["trace"] = rows
    return rec


# --- the sweep ---------------------------------------------------------------------------------
A_BY = {r["name"]: r for r in A_POOL}
B_BY = {r["name"]: r for r in B_POOL}


def grid(vb):
    if vb >= 10:
        return [(float(d), float(v)) for v in range(-4, 5) for d in range(-6, 31, 2)]
    return [(float(d), float(v)) for v in range(-3, 4) for d in range(-6, 31, 3)]


def jobs_for(ctxs, caps, seeds, vb):
    out = []
    for (d0, dv), ca, cb in itertools.product(ctxs, caps, caps):
        if CAPS[cb].v_max < vb - 1e-6 or CAPS[ca].v_max < vb + dv - 1.0:
            continue  # the cut-in's speeds are beyond the car
        for s in range(seeds):
            for an in A_NAMES:
                for bn in B_BY:
                    out.append((d0, dv, vb, ca, an, cb, bn, s, 0))
                for pref in PREFS:
                    out.append((d0, dv, vb, ca, an, cb, B_PREF_RULE[pref]["name"], s, 1))
    return out


def run_job(j):
    d0, dv, vb, ca, an, cb, bn, s, probe = j
    rec = simulate(d0, dv, vb, ca, A_BY[an], cb, B_BY[bn], s, bool(probe))
    row = {"d0": d0, "dv": dv, "vb": vb, "capA": ca, "a": ("P+" if probe else "") + an, "capB": cb, "b": bn, "seed": s}
    row.update({k: rec[k] for k in ("uA", "uB", "contact", "ahead", "got_in", "clear", "peakB", "t_decide")})
    # the beliefs: the decision-time posterior (a probe's end) and the MAP over time
    row["types"] = "|".join(rec["types"])
    row["post"] = json.dumps({m: [round(x, 4) for x in rec["snaps"][PROBE_T][m]] for m in FILTERS})
    row["post0"] = json.dumps([round(x, 4) for x in rec["snaps"][0.0]["class"]])
    row["maps"] = json.dumps({f"{m}@{tq:g}": max(range(len(p)), key=lambda i: p[i])
                              for tq, d in rec["snaps"].items() for m, p in d.items()})
    return row


FIELDS = ["d0", "dv", "vb", "capA", "a", "capB", "b", "seed", "uA", "uB", "contact", "ahead", "got_in", "clear",
          "peakB", "t_decide", "types", "post", "post0", "maps"]


def sweep(args):
    caps = ["m3", "truck"] + (["golfcart"] if args.golfcart else [])
    jobs = jobs_for(grid(args.vb), caps, args.seeds, args.vb)
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "runs.csv.gz")
    print(f"{len(jobs)} runs on {args.procs} processes", flush=True)
    with gzip.open(path, "wt", newline="") as f, Pool(args.procs) as pool:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for i, row in enumerate(pool.imap(run_job, jobs, chunksize=64)):
            w.writerow(row)
            if (i + 1) % 20000 == 0:
                print(f"  {i + 1}/{len(jobs)}", flush=True)
    return path


# --- analysis ----------------------------------------------------------------------------------
def load(path):
    rows = []
    with gzip.open(path, "rt") as f:
        for r in csv.DictReader(f):
            for k in ("d0", "dv", "vb", "uA", "uB", "clear", "peakB"):
                r[k] = float(r[k])
            for k in ("seed", "contact", "ahead", "got_in"):
                r[k] = int(r[k])
            for k in ("post", "post0", "maps"):
                r[k] = json.loads(r[k])
            rows.append(r)
    return rows


def mean(xs):
    xs = list(xs)
    return sum(xs) / len(xs) if xs else float("nan")


def se(xs):
    return st.stdev(xs) / math.sqrt(len(xs)) if len(xs) > 1 else 0.0


FLOOR = 0.01  # utility: below this a difference is noise (about 0.2 s of delay at 12 m/s)
SHARE = 0.10  # D6's bar
OBJECTIVES = {"selfish": lambda r: r["uA"], "prosocial": lambda r: r["uA"] + r["uB"]}


class Ctx:
    """One context and A's capability: every run, by (A option, B's capability, B's rule, seed)."""

    def __init__(self, rs):
        self.R = {}
        for r in rs:
            self.R[(r["a"], r["capB"], r["b"], r["seed"])] = r
        self.seeds = sorted({r["seed"] for r in rs})
        self.caps_b = sorted({r["capB"] for r in rs}, key=list(CAPS).index)
        self.types = [tuple(x.split("/")) for x in rs[0]["types"].split("|")]
        have = {r["a"] for r in rs}
        self.a_names = [a for a in A_NAMES if a in have and "P+" + a in have]
        self.prior = type_prior(sorted({c for c, _ in self.types}, key=list(CAPS).index))

    def present(self, t):
        return t[0] in self.caps_b

    def run(self, a, t, s):
        return self.R[(a, t[0], B_PREF_RULE[t[1]]["name"], s)]

    def loo(self, a, t, s, obj):
        """Type t's mean objective of option a over the seeds other than s."""
        return mean(obj(self.run(a, t, x)) for x in self.seeds if x != s)

    def choose(self, belief, s, obj, mode, t_true):
        """The option (a pool rule, or the probe) with the best expected objective under the belief
        at t = 0; the probe's value is what its adaptive choice earns, in the same leave-one-out
        tables. Returns the realized run of the true type."""
        live = [(t, q) for t, q in zip(self.types, belief) if q > 0 and self.present(t)]
        z = sum(q for _, q in live)
        live = [(t, q / z) for t, q in live]
        opts = self.a_names + (["P+" + a for a in self.a_names] if mode != "prior" else [])
        val = {a: sum(q * self.loo(a, t, s, obj) for t, q in live) for a in opts}
        if mode not in ("prior", "prior_p"):
            val["probe"] = sum(q * mean(obj(self.probe_run(t, x, obj, mode, s)) for x in self.seeds if x != s)
                               for t, q in live)
        best = max(val, key=val.get)
        if best == "probe":
            return self.probe_run(t_true, s, obj, mode, s), best
        return self.run(best, t_true, s), best

    def probe_run(self, t, s, obj, mode, hold_out):
        """The probe in seed s against type t: the rule chosen on the posterior at the probe's end
        (mode's filter; the truth for `truth`), by the tables without seed `hold_out` (and s)."""
        r0 = self.run("P+" + A_NAMES[0], t, s)
        if mode == "truth":
            post = [1.0 if u == t else 0.0 for u in self.types]
        else:
            post = r0["post"][mode]
        live = [(u, q) for u, q in zip(self.types, post) if q > 1e-6 and self.present(u)]
        z = sum(q for _, q in live)
        others = [x for x in self.seeds if x not in (s, hold_out)]

        def v(a):
            return sum(q / z * mean(obj(self.run("P+" + a, u, x)) for x in others) for u, q in live)
        a = max(self.a_names, key=v)
        return self.run("P+" + a, t, s)


def cell_measures(cx, d0, dv, vb, ca):
    out = []
    cost_pairs = {}
    for (a, cb, bn, s), r in cx.R.items():
        cost_pairs.setdefault((a, cb, bn), []).append(-(r["uA"] + r["uB"]))
    for cb in cx.caps_b:
        A = cx.a_names
        Bn = list(B_BY)
        m = {(a, b): mean(cost_pairs[(a, cb, b)]) for a in A for b in Bn}
        ua = {(a, b): mean(cx.R[(a, cb, b, s)]["uA"] for s in cx.seeds) for a in A for b in Bn}
        ub = {(a, b): mean(cx.R[(a, cb, b, s)]["uB"] for s in cx.seeds) for a in A for b in Bn}
        c_opt = min(min(m.values()), min(mean(v) for (a, c, b), v in cost_pairs.items() if c == cb))
        pure, mixed = C1.solve(ua, ub, A, Bn)
        eq = [m[p] for p in pure] + [sum(pa[a] * pb[b] * m[(a, b)] for a in pa for b in pb) for pa, pb in mixed]
        L_stock = m[STOCK] - c_opt
        L_worst = max(eq) - c_opt if eq else None
        for pref in PREFS:
            t = (cb, pref)
            cell = {"d0": d0, "dv": dv, "vb": vb, "capA": ca, "capB": cb, "pref": pref, "true": f"{cb}/{pref}",
                    "C_opt": round(c_opt, 4), "C_stock": round(m[STOCK], 4), "L_stock": round(L_stock, 4),
                    "L_worst": None if L_worst is None else round(L_worst, 4),
                    "C_stock_type": round(m[("A_aw_perm", B_PREF_RULE[pref]["name"])], 4)}
            for oname, obj in OBJECTIVES.items():
                runs, picks = {}, {}
                for mode in MODES:
                    runs[mode], picks[mode] = [], []
                    for s in cx.seeds:
                        if mode == "truth":
                            b = [1.0 if u == t else 0.0 for u in cx.types]
                        elif mode in ("prior", "prior_p", "motion"):
                            b = cx.prior  # nothing is seen before t = 0 but a cruising car
                        elif mode == "class":
                            b = cx.run(A_NAMES[0], t, s)["post0"]  # the class cue at t = 0
                        else:
                            b = [q if u[0] == cb else 0.0 for u, q in zip(cx.types, cx.prior)]
                        r, pick = cx.choose(b, s, obj, mode, t)
                        runs[mode].append(r)
                        picks[mode].append(pick)
                U = {mo: [obj(r) for r in runs[mo]] for mo in MODES}
                C = {mo: [-(r["uA"] + r["uB"]) for r in runs[mo]] for mo in MODES}
                d = lambda x, y, D=U: [a - b for a, b in zip(D[x], D[y])]  # noqa: E731
                p = oname[0]  # s or p
                for name, (x, y) in PAIRS.items():
                    v = d(x, y)
                    w = d(y, x, C)  # the welfare cost the information saves
                    cell[f"{p}_{name}"] = round(mean(v), 4)
                    cell[f"{p}_{name}_se"] = round(se(v), 4)
                    cell[f"{p}_{name}_W"] = round(mean(w), 4)
                cell[f"{p}_C"] = {mo: round(mean(C[mo]), 4) for mo in MODES}
                cell[f"{p}_contact"] = {mo: round(mean(r["contact"] for r in runs[mo]), 3) for mo in MODES}
                cell[f"{p}_ahead"] = {mo: round(mean(r["ahead"] for r in runs[mo]), 3) for mo in MODES}
                cell[f"{p}_picks"] = {mo: max(set(picks[mo]), key=picks[mo].count) for mo in MODES}
                # acting on a fixed belief (no probe): the choice for an assumed type, run against the
                # true one, against the same choice for the true type
                asm, real = {}, {}
                for u in cx.types:
                    if cx.present(u):
                        rr = [cx.choose([1.0 if w_ == u else 0.0 for w_ in cx.types], s, obj, "prior", u)[1] for s in cx.seeds]
                        real[u] = [cx.run(a, t, s) for a, s in zip(rr, cx.seeds)]
                base_u = mean(obj(r) for r in real[t])
                base_c = mean(-(r["uA"] + r["uB"]) for r in real[t])
                for u, rs_ in real.items():
                    asm[f"{u[0]}/{u[1]}"] = {"dU": round(mean(obj(r) for r in rs_) - base_u, 4),
                                             "dC": round(mean(-(r["uA"] + r["uB"]) for r in rs_) - base_c, 4),
                                             "contact": round(mean(r["contact"] for r in rs_), 3)}
                cell[f"{p}_assumed"] = asm
            out.append(cell)
    return out


def b_side(cx_all):
    """B's choice of reaction (its pool's reactive rules, which differ only after A's lane change
    is seen) under a belief over A's type (capability x rule A_aw_perm or A2): the prior, the class
    cue (exact here: A's box is seen), or the truth."""
    out = []
    for (d0, dv, vb), by_ca in cx_all.items():
        caps_a = sorted(by_ca, key=list(CAPS).index)
        for cb in sorted({c for cx in by_ca.values() for c in cx.caps_b}, key=list(CAPS).index):
            ta = [(ca, a) for ca in caps_a for a in ("A_aw_perm", "A2") if cb in by_ca[ca].caps_b]
            if not ta:
                continue
            w = {u: cap_prior(u[0]) * 0.5 for u in ta}
            z = sum(w.values())
            seeds = by_ca[ta[0][0]].seeds
            for oname, obj in OBJECTIVES.items():
                def val(b, u, s):
                    return mean(obj(by_ca[u[0]].R[(u[1], cb, b, x)]) for x in seeds if x != s)
                for u in ta:
                    res = {}
                    for mode in ("prior", "class", "truth"):
                        got = []
                        for s in seeds:
                            if mode == "prior":
                                bel = {v: w[v] / z for v in ta}
                            elif mode == "class":
                                zz = sum(w[v] for v in ta if v[0] == u[0])
                                bel = {v: w[v] / zz for v in ta if v[0] == u[0]}
                            else:
                                bel = {u: 1.0}
                            b = max(B_REACTIVE, key=lambda bb: sum(q * val(bb, v, s) for v, q in bel.items()))
                            got.append(by_ca[u[0]].R[(u[1], cb, b, s)])
                        res[mode] = got
                    U = {mo: [obj(r) for r in res[mo]] for mo in res}
                    out.append({"d0": d0, "dv": dv, "vb": vb, "capB": cb, "true": f"{u[0]}/{u[1]}", "obj": oname,
                                "voi_class": round(mean(U["class"]) - mean(U["prior"]), 4),
                                "regret_class": round(mean(U["truth"]) - mean(U["class"]), 4),
                                "regret_prior": round(mean(U["truth"]) - mean(U["prior"]), 4)})
    return out


def counts(cells, key, denom="L_stock"):
    """Cells reaching D6's bar: the measure >= SHARE x the stock loss, >= FLOOR, and > 2 se."""
    n = hit = 0
    for c in cells:
        L = c.get(denom)
        if L is None:
            continue
        n += 1
        v, s = c[key], c.get(key + "_se", 0.0)
        if v >= FLOOR and v > 2 * s and v >= SHARE * max(L, 0.0):
            hit += 1
    return {"cells": n, "hit": hit, "share": round(hit / n, 4) if n else None}


def prior_weighted(cells, key):
    """Per context and A's capability, the measure averaged over B's types with the class prior
    (the expectation a car on the road meets), against the prior-weighted stock loss."""
    g = {}
    for c in cells:
        g.setdefault((c["d0"], c["dv"], c["vb"], c["capA"]), []).append(c)
    out = []
    for k, cs in g.items():
        caps = sorted({c["capB"] for c in cs}, key=list(CAPS).index)
        pr = dict(zip(b_types(caps), type_prior(caps)))
        w = [pr[(c["capB"], c["pref"])] for c in cs]
        z = sum(w)
        out.append({"d0": k[0], "dv": k[1], "vb": k[2], "capA": k[3],
                    key: sum(wi * c[key] for wi, c in zip(w, cs)) / z,
                    "L_stock": sum(wi * c["L_stock"] for wi, c in zip(w, cs)) / z,
                    "L_worst": sum(wi * (c["L_worst"] or 0.0) for wi, c in zip(w, cs)) / z})
    return out


PAIRS = {"regret_motion": ("truth", "motion"), "regret_class": ("truth", "class"), "voi_v2x": ("v2x", "class"),
         "voi_class": ("class", "motion"), "voi_belief": ("motion", "prior_p"), "voi_probe": ("prior_p", "prior"),
         "voi_all_p": ("truth", "prior_p"), "voi_all": ("truth", "prior")}
KEYS = tuple(PAIRS)


def summarize(cells, conf, bside):
    res = {"cells": len(cells), "bar": {"share": SHARE, "floor": FLOOR, "se": 2}}
    for p, oname in (("s", "selfish"), ("p", "prosocial")):
        d = {}
        for k in KEYS:
            key = f"{p}_{k}"
            pw = prior_weighted(cells, key)
            d[k] = {"mean": round(mean(c[key] for c in cells), 4), "max": round(max(c[key] for c in cells), 4),
                    "welfare_mean": round(mean(c[key + "_W"] for c in cells), 4),
                    "vs_stock": counts(cells, key, "L_stock"), "vs_worst_eq": counts(cells, key, "L_worst"),
                    "prior_weighted_vs_stock": counts(pw, key, "L_stock"),
                    "prior_weighted_vs_worst_eq": counts(pw, key, "L_worst"),
                    "prior_weighted_mean": round(mean(c[key] for c in pw), 4)}
        d["C"] = {mo: round(mean(c[f"{p}_C"][mo] for c in cells), 4) for mo in MODES}
        d["contact"] = {mo: round(mean(c[f"{p}_contact"][mo] for c in cells), 4) for mo in MODES}
        res[oname] = d
    res["L_stock_mean"] = round(mean(c["L_stock"] for c in cells), 4)
    res["L_worst_mean"] = round(mean(c["L_worst"] for c in cells if c["L_worst"] is not None), 4)
    res["C_stock_mean"] = round(mean(c["C_stock"] for c in cells), 4)
    res["by_type"] = {}
    for ca in sorted({c["capA"] for c in cells}, key=list(CAPS).index):
        for tr in sorted({c["true"] for c in cells}):
            cs = [c for c in cells if c["capA"] == ca and c["true"] == tr]
            if not cs:
                continue
            e = {"cells": len(cs), "L_stock": round(mean(c["L_stock"] for c in cs), 4)}
            for p in ("s", "p"):
                for k in ("regret_class", "regret_motion", "voi_v2x", "voi_belief", "voi_probe", "voi_all_p", "voi_all"):
                    e[f"{p}_{k}"] = round(mean(c[f"{p}_{k}"] for c in cs), 4)
                    e[f"{p}_{k}_hit"] = counts(cs, f"{p}_{k}")["hit"]
                e[f"{p}_C"] = {mo: round(mean(c[f"{p}_C"][mo] for c in cs), 4) for mo in MODES}
                e[f"{p}_contact"] = {mo: round(mean(c[f"{p}_contact"][mo] for c in cs), 4) for mo in MODES}
            res["by_type"][f"A {ca} | B {tr}"] = e
    res["confusion"] = conf
    asm = {}
    for c in cells:
        for p in ("s", "p"):
            for a, v in c[f"{p}_assumed"].items():
                asm.setdefault((p, c["capA"], a, c["true"]), []).append(v)
    res["assumed"] = {f"{p} | A {ca} | assumed {a} | true {t}": {
        "dU": round(mean(v["dU"] for v in vs), 4), "dC": round(mean(v["dC"] for v in vs), 4),
        "contact": round(mean(v["contact"] for v in vs), 4), "worst_dU": round(min(v["dU"] for v in vs), 4)}
        for (p, ca, a, t), vs in sorted(asm.items())}
    res["b_side"] = {}
    for oname in OBJECTIVES:
        xs = [x for x in bside if x["obj"] == oname]
        res["b_side"][oname] = {k: {"mean": round(mean(x[k] for x in xs), 4), "max": round(max(x[k] for x in xs), 4),
                                    "over_floor": sum(1 for x in xs if x[k] >= FLOOR), "n": len(xs)}
                                for k in ("voi_class", "regret_class", "regret_prior")}
    return res


def confusion(rows):
    """True type x MAP belief, per filter and time, for the probing runs (P+, any rule: identical up
    to the probe's end; also only those where the probe was informative) and Autoware's A
    (A_aw_perm, its intent evidence after its 3 s prepare). A tie (no evidence) goes to the first
    type listed, the holder."""
    out = {}
    for r in rows:
        if r["b"] not in ("B_aw", "B_aw_react"):
            continue
        if r["a"] == "P+" + A_NAMES[0]:
            kind = "probe"
        elif r["a"] == "A_aw_perm":
            kind = "A_aw_perm"
        else:
            continue
        types = r["types"].split("|")
        tru = f"{r['capB']}/{'yield' if r['b'] == 'B_aw_react' else 'hold'}"
        kinds = [kind]
        if kind == "probe":  # where the probe was informative: a yielder would have braked
            py = sum(q for u, q in zip(types, r["post"]["motion"]) if u.endswith("yield"))
            if abs(py - P_YIELD) > 0.05:
                kinds.append("probe_informative")
        for kd in kinds:
            for k, i in r["maps"].items():
                out.setdefault(f"{kd} {k}", {}).setdefault(tru, {}).setdefault(types[i], 0)
                out[f"{kd} {k}"][tru][types[i]] += 1
    return {k: {t: {b: round(n / sum(row.values()), 3) for b, n in sorted(row.items())} for t, row in sorted(m.items())}
            for k, m in sorted(out.items())}


def intent_outcome(post, types):
    """The intent evidence a filter took (seen, not, or none), from its posterior: the prior's
    odds of yield to hold are even and capability evidence moves both alike, so the odds are the
    likelihood ratio the filter applied."""
    lr_seen, lr_not = P_SEEN_YIELD / P_FALSE_YIELD, (1 - P_SEEN_YIELD) / (1 - P_FALSE_YIELD)
    y = sum(q for u, q in zip(types, post) if u.endswith("yield"))
    h = sum(q for u, q in zip(types, post) if u.endswith("hold"))
    if h <= 0 or y <= 0:
        return "none"
    o = math.log(y / h)
    best = min((("seen", math.log(lr_seen)), ("not", math.log(lr_not)), ("none", 0.0)), key=lambda kv: abs(o - kv[1]))
    return best[0]


def calibrate(rows):
    """The intent likelihood measured on the runs (a fitted noise model): among the probes whose
    test was informative, how often a yielder and a holder were seen braking. Then every filter's
    posterior is recomputed with it (the filters are passive, so no run changes)."""
    n = {"yield": [0, 0], "hold": [0, 0]}
    for r in rows:
        if r["a"] != "P+" + A_NAMES[0] or r["b"] not in ("B_aw", "B_aw_react"):
            continue
        types = r["types"].split("|")
        o = intent_outcome(r["post"]["motion"], types)
        if o == "none":
            continue
        k = "yield" if r["b"] == "B_aw_react" else "hold"
        n[k][0] += o == "seen"
        n[k][1] += 1
    p_seen = (n["yield"][0] + 1) / (n["yield"][1] + 2)
    p_false = (n["hold"][0] + 1) / (n["hold"][1] + 2)
    new = {"seen": (p_seen, p_false), "not": (1 - p_seen, 1 - p_false)}
    for r in rows:
        types = r["types"].split("|")
        for m in FILTERS:
            post = r["post"][m]
            o = intent_outcome(post, types)
            if o == "none":
                continue
            old = {"seen": (P_SEEN_YIELD, P_FALSE_YIELD), "not": (1 - P_SEEN_YIELD, 1 - P_FALSE_YIELD)}[o]
            w = [q * (new[o][0] / old[0] if u.endswith("yield") else new[o][1] / old[1]) for u, q in zip(types, post)]
            z = sum(w)
            r["post"][m] = [x / z for x in w]
    return {"p_seen_yield": round(p_seen, 4), "p_false_yield": round(p_false, 4), "probes": n,
            "assumed": {"p_seen_yield": P_SEEN_YIELD, "p_false_yield": P_FALSE_YIELD}}


def analyze(out, path=None, procs=12):
    rows = load(path or os.path.join(out, "runs.csv.gz"))
    cal = calibrate(rows)
    print(f"intent likelihood, calibrated on the probes: {cal}", flush=True)
    groups = {}
    for r in rows:
        groups.setdefault((r["d0"], r["dv"], r["vb"], r["capA"]), []).append(r)
    cx_all = {}
    for (d0, dv, vb, ca), rs in groups.items():
        cx_all.setdefault((d0, dv, vb), {})[ca] = Ctx(rs)
    with Pool(procs) as pool:
        parts = pool.starmap(_cell_job, [(k, rs) for k, rs in sorted(groups.items())])
    cells = [c for p in parts for c in p]
    conf = confusion(rows)
    bside = b_side(cx_all)
    res = summarize(cells, conf, bside)
    res["intent_likelihood"] = cal
    with open(os.path.join(out, "c6_cells.jsonl"), "w") as f:
        for c in cells:
            f.write(json.dumps(c) + "\n")
    with open(os.path.join(out, "c6_bside.jsonl"), "w") as f:
        for c in bside:
            f.write(json.dumps(c) + "\n")
    json.dump(res, open(os.path.join(out, "c6_summary.json"), "w"), indent=1)
    text = report(res, cells)
    open(os.path.join(out, "c6_report.txt"), "w").write(text)
    print(text)
    return res


def _cell_job(k, rs):
    return cell_measures(Ctx(rs), *k)


def class_map(cells, key, ca, tr, denom="L_stock"):
    """A text map over (d0, dv): # the measure reaches the bar, + >= FLOOR, . below, ' ' no cell."""
    cs = {(c["d0"], c["dv"]): c for c in cells if c["capA"] == ca and c["true"] == tr}
    if not cs:
        return []
    d0s = sorted({k[0] for k in cs})
    dvs = sorted({k[1] for k in cs}, reverse=True)
    L = []
    for dv in dvs:
        s = ""
        for d0 in d0s:
            c = cs.get((d0, dv))
            if c is None:
                s += " "
                continue
            v, e, Ls = c[key], c.get(key + "_se", 0.0), c[denom] or 0.0
            s += "#" if (v >= FLOOR and v > 2 * e and v >= SHARE * max(Ls, 0)) else "+" if v >= FLOOR else "."
        L.append(f"{dv:+5.1f} {s}")
    return L


def report(res, cells):
    L = [f"C6 hidden capability, kinematic: {res['cells']} cells (context x A's capability x B's true type)",
         f"bar: a measure >= {SHARE:.0%} of the stock loss, >= {FLOOR}, > 2 se over paired seeds",
         f"stock (Autoware pair) loss vs optimum: mean {res['L_stock_mean']}; worst-equilibrium loss mean {res['L_worst_mean']}"]
    for oname in OBJECTIVES:
        d = res[oname]
        L.append(f"\n[{oname} A] mean welfare cost per mode: " + ", ".join(f"{m} {d['C'][m]:.3f}" for m in MODES))
        L.append("  contact: " + ", ".join(f"{m} {d['contact'][m]:.4f}" for m in MODES))
        for k in KEYS:
            v = d[k]
            L.append(f"  {k:14s} mean {v['mean']:+.4f} (welfare {v['welfare_mean']:+.4f}) max {v['max']:+.4f} | "
                     f"bar vs stock {v['vs_stock']['hit']}/{v['vs_stock']['cells']} ({v['vs_stock']['share']:.1%}), "
                     f"vs worst eq {v['vs_worst_eq']['hit']}/{v['vs_worst_eq']['cells']} ({v['vs_worst_eq']['share']:.1%}); "
                     f"prior-weighted {v['prior_weighted_vs_stock']['hit']}/{v['prior_weighted_vs_stock']['cells']}"
                     f" ({v['prior_weighted_vs_stock']['share']:.1%})")
    L.append("\n[by type] selfish: C per mode, regret (class, motion), voi_v2x, mean and cells over the bar")
    for k, e in res["by_type"].items():
        L.append(f"  {k:26s} L_stock {e['L_stock']:.3f} | C " + " ".join(f"{m} {e['s_C'][m]:.3f}" for m in MODES)
                 + f" | regret class {e['s_regret_class']:+.4f} ({e['s_regret_class_hit']}/{e['cells']}) motion "
                 f"{e['s_regret_motion']:+.4f} ({e['s_regret_motion_hit']}) v2x {e['s_voi_v2x']:+.4f} ({e['s_voi_v2x_hit']})"
                 f" belief {e['s_voi_belief']:+.4f} ({e['s_voi_belief_hit']}) probe {e['s_voi_probe']:+.4f} ({e['s_voi_probe_hit']})"
                 f" | contact truth {e['s_contact']['truth']:.3f} class {e['s_contact']['class']:.3f} prior {e['s_contact']['prior']:.3f}")
    L.append("\n[maps] selfish regret (class) per (d0 -6..30, dv top +4): # over the bar, + >= floor, . below")
    for ca in sorted({c["capA"] for c in cells}, key=list(CAPS).index):
        for tr in sorted({c["true"] for c in cells}):
            m = class_map(cells, "s_regret_class", ca, tr)
            if m:
                L.append(f"  A {ca} | B {tr}")
                L += ["    " + x for x in m]
    L.append("\n[type confusion] true -> MAP belief")
    for k, m in res["confusion"].items():
        if not k.endswith(("@0", f"@{PROBE_T:g}", "@8")):
            continue
        L.append(f"  {k}")
        for t, row in m.items():
            L.append(f"    {t:16s} " + "  ".join(f"{b} {x:.2f}" for b, x in row.items()))
    L.append("\n[acting on a fixed belief] selfish: dU (own) and dC (welfare) against the truth, contact")
    for k, v in res["assumed"].items():
        if k.startswith("s |"):
            L.append(f"  {k[4:]:52s} dU {v['dU']:+.4f} (worst {v['worst_dU']:+.3f}) dC {v['dC']:+.4f} contact {v['contact']:.3f}")
    L.append("\n[B's choice of reaction] " + json.dumps(res["b_side"]))
    return "\n".join(L) + "\n"


# --- selftest ----------------------------------------------------------------------------------
def selftest():
    types = b_types(["m3", "truck"])
    pr = type_prior(["m3", "truck"])
    ptruck = lambda b: sum(q for (c, _), q in zip(types, b.p) if c == "truck")  # noqa: E731
    # a seen 6 m/s^2 stop rules the 4.5 m/s^2 truck out; one noisy 4.8 sample does not
    b = Belief(types, pr, "motion")
    b.capability(-6.0, 12.0)
    assert ptruck(b) < 0.01, b.p
    b = Belief(types, pr, "motion")
    b.capability(-4.8, 12.0)
    assert ptruck(b) > 0.05, b.p
    # cruising shows nothing: the class prior stays
    b = Belief(types, pr, "motion")
    for _ in range(10):
        b.capability(0.1, 12.0)
    assert abs(ptruck(b) - P_TRUCK) < 0.02, b.p
    b.intent(True)
    assert b.p_yield() > 0.85, b.p
    b = Belief(types, pr, "class")
    b.size(8.0)
    assert ptruck(b) > 0.99
    b = Belief(types, pr, "v2x", "truck")
    assert abs(b.p_yield() - P_YIELD) < 1e-9 and ptruck(b) > 0.999
    # runs: far ahead and faster, the gap-checking rules go ahead without contact
    for an in ("A_aw_perm", "A2"):
        r = simulate(24.0, 2.0, 12.0, "m3", A_BY[an], "m3", B_AW, 0)
        assert r["ahead"] and not r["contact"], (an, r)
    # A beside B and slower: no rule cuts in ahead, nobody touches
    for ar in A_POOL:
        r = simulate(-4.0, -3.0, 12.0, "m3", ar, "m3", B_AW, 0)
        assert not r["contact"] and not r["ahead"], (ar["name"], r)
    # a truck B brakes within its limit
    r = simulate(2.0, -4.0, 12.0, "m3", A_BY["A_bold"], "truck", B_AW, 0)
    assert r["peakB"] <= CAPS["truck"].b_max + 0.3, r
    # a probe: a yielder is seen braking and believed a yielder at the probe's end; a holder not
    def p_yield_after(b_rule):
        ps = []
        for k in range(8):
            r = simulate(8.0, -1.0, 12.0, "m3", A_BY["A2"], "m3", b_rule, k, probe=True)
            ps.append(sum(q for (_, p), q in zip(types, r["snaps"][PROBE_T]["motion"]) if p == "yield"))
        return mean(ps)
    assert p_yield_after(B_AW_REACT) > 0.6 and p_yield_after(B_AW) < 0.4, (p_yield_after(B_AW_REACT), p_yield_after(B_AW))
    # the probe's phase is the same whatever rule follows it (common random numbers)
    a = simulate(8.0, -1.0, 12.0, "m3", A_BY["A2"], "m3", B_AW, 3, probe=True)
    b2 = simulate(8.0, -1.0, 12.0, "m3", A_BY["A_wait"], "m3", B_AW, 3, probe=True)
    assert a["snaps"][PROBE_T] == b2["snaps"][PROBE_T]
    # the class filter sees a truck at once
    pt = [sum(q for (c, _), q in zip(types, simulate(8.0, 0.0, 12.0, "m3", A_BY["A_wait"], "truck", B_AW, k)
                                          ["snaps"][0.0]["class"]) if c == "truck") for k in range(8)]
    assert mean(pt) > 0.85, pt
    # the analysis: with the truth, the leave-one-out choice is never worse on average than the prior's
    rows = []
    for j in jobs_for([(10.0, -2.0)], ["m3", "truck"], 4, 12.0):
        rows.append(run_job(j))
    for r in rows:
        for k in ("post", "post0", "maps"):
            r[k] = json.loads(r[k])
    cells = cell_measures(Ctx([r for r in rows if r["capA"] == "m3"]), 10.0, -2.0, 12.0, "m3")
    assert len(cells) == 4
    assert mean(c["s_voi_all"] for c in cells) >= -0.02, [c["s_voi_all"] for c in cells]
    assert all(c["L_stock"] >= -1e-9 for c in cells)
    # the intent evidence is recovered from a posterior
    for seen in (True, False):
        b = Belief(types, pr, "class")
        b.size(8.0)
        b.capability(-1.0, 12.0)
        b.intent(seen)
        assert intent_outcome(b.p, [f"{c}/{p}" for c, p in types]) == ("seen" if seen else "not")
    assert intent_outcome(pr, [f"{c}/{p}" for c, p in types]) == "none"
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="c6_out")
    p.add_argument("--vb", type=float, default=12.0)
    p.add_argument("--seeds", type=int, default=8)
    p.add_argument("--procs", type=int, default=12)
    p.add_argument("--golfcart", action="store_true", help="the golf cart as a third capability (v_B <= 6.9)")
    p.add_argument("--analyze", action="store_true", help="analyze an existing runs.csv.gz")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    if not a.analyze:
        sweep(a)
    analyze(a.out, procs=a.procs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
