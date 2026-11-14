#!/usr/bin/env python3
"""Roadmap 074 C1: the two-car cut-in as a game in a kinematic model.

A, in the left lane, wants into B's lane. The cut-in is sequential with delays: B sees A's lane
change only after its reaction time, and A sees B's braking only after its own. So a strategy is
a contingent rule, not a single action:

  A  go           change lanes at once
     wait         drop behind B, then change
     go_if_yield  watch B for a window; change if B brakes, otherwise drop behind
  B  pre_yield    open a gap from the start, whatever A does (courtesy before any signal)
     react        keep going; once A's lane change is seen, continue with whichever of
                  yield / hold is B's best reply to it (sequential rationality: a threat to
                  hold that B would not carry out is not a strategy)

Under every strategy B's safety layer brakes, after its reaction time, as hard as the gap to a
car in its lane needs, up to its braking limit.

Payoffs come from the simulated trajectories with the general utility (measures.utility,
docs/design/cooperative-driving.md ①): lost route progress over a fixed horizon, risk as the
largest shortfall of 2D TTC, time headway, PET and body clearance, peak braking and lateral
acceleration relative to the car's limits, contact (the bodies overlap). A car's
payoff is J_i = cos(phi_i) U_i + sin(phi_i) U_-i (phi: its SVO angle). The game is solved per
context (d0, dv): pure Nash equilibria and the mixed ones by support enumeration. Welfare is
measured as a cost C = -(U_A + U_B) >= 0, so the price of anarchy and of stability are ratios of
non-negative numbers; the losses C_eq - C_opt are reported too, since C_opt can be near 0.

Contexts:
  d0  A's rear minus B's front at t = 0 (m); negative: A's rear is beside or behind B's nose
  dv  A's speed minus B's (m/s)

Capabilities (TYPES) are first numbers; C5 replaces them with `just vehicle-check` measurements.

Usage:
  scripts/coop/cutin_game.py --out <dir> [--a car] [--b car] [--vb 12] [--svo-a 0] [--svo-b 0]
  scripts/coop/cutin_game.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys
from dataclasses import dataclass, replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import measures as M  # noqa: E402

DT = 0.05
HORIZON = 20.0  # s; 12 s left a wait at d0 = 20 m unfinished, scored as a total loss (074 E1)
T_MAX = 40.0  # s; a run goes on past HORIZON until A's lane change is complete, up to this (074 C2a)
LANE_LINE = 0.5  # A's lateral progress (0 own lane, 1 B's lane) at which it occupies B's lane
LANE_W = 3.5  # m, as the arena's road


@dataclass(frozen=True)
class Capability:
    """theta: what the car can do. Limits, not habits."""

    length: float  # m
    a_max: float  # m/s^2, drive
    b_max: float  # m/s^2, braking
    v_max: float  # m/s
    reaction: float  # s, from an observable event to the first response
    t_prep: float  # s, from deciding to change lanes to the first lateral motion
    t_lc: float  # s, the lateral motion itself
    width: float = 2.163  # m, the Model 3's body box in CARLA (sim_vehicle_spec)
    a_lat_max: float = M.A_LAT_MAX  # m/s^2, the lateral acceleration comfort is judged against


@dataclass(frozen=True)
class Preference:
    """phi: how the car trades progress against risk and comfort."""

    w_progress: float = 1.0
    w_risk: float = 1.0
    w_contact: float = 10.0
    w_comfort: float = 0.3
    ttc_ref: float = 3.0  # s
    thw_ref: float = 1.0  # s


# First numbers. The Model 3's length from sim_vehicle_spec (4.79 m); a 1 s reaction is
# Autoware's own assumption for a rear car. t_prep 0 is a human-like immediate start; Autoware's
# lane change has a 2-4 s prepare phase first ("autoware" types). The golf cart is assumed.
TYPES = {
    "car": Capability(4.79, a_max=3.0, b_max=7.0, v_max=30.0, reaction=1.0, t_prep=0.0, t_lc=3.0),
    "car_alert": Capability(4.79, a_max=3.0, b_max=7.0, v_max=30.0, reaction=0.5, t_prep=0.0, t_lc=3.0),
    "autoware": Capability(4.79, a_max=3.0, b_max=7.0, v_max=30.0, reaction=1.0, t_prep=3.0, t_lc=3.0),
    # The Model 3 as the arena measured it on CARLA (roadmap 074 E1): 0-12 m/s in 2.55 s
    # (4.7 m/s^2 mean), full braking peaks at 9.6 m/s^2 (7.4 mean from 12 m/s with the lags).
    "carla_model3": Capability(4.79, a_max=4.7, b_max=9.6, v_max=30.0, reaction=1.0, t_prep=0.0, t_lc=3.0),
    "golfcart": Capability(2.4, a_max=1.5, b_max=4.0, v_max=6.9, reaction=1.0, t_prep=0.0, t_lc=3.0),
}

GO, WAIT, GO_IF_YIELD = "go", "wait", "go_if_yield"
PRE_YIELD, REACT = "pre_yield", "react"
A_STRATS = (GO, WAIT, GO_IF_YIELD)
B_STRATS = (PRE_YIELD, REACT)
YIELD, HOLD = "yield", "hold"  # B's continuations under REACT

B_YIELD_DECEL = 2.0  # m/s^2, a comfortable yield
A_WAIT_DECEL = 2.0  # m/s^2, A dropping back
HEADWAY = 1.0  # s, desired time gap once following
S0 = 2.0  # m, standstill gap
WATCH = 1.5  # s, how long go_if_yield watches B after its own reaction time
YIELD_SEEN = 0.5  # m/s^2, the braking A takes as a yield


@dataclass
class Outcome:
    progress: tuple[float, float]  # lost progress per car, fraction of free flow
    min_ttc: float
    min_thw: float
    contact: bool
    peak_brake: tuple[float, float]
    went: bool  # A changed lanes ahead of B
    b_cont: str  # B's continuation under REACT ("" otherwise)
    min_clear: float = math.inf  # m, the smallest body clearance
    min_pet: float = math.inf  # s, post-encroachment time (paths that cross)
    peak_lat: tuple[float, float] = (0.0, 0.0)  # m/s^2, the largest lateral acceleration per car


def follow_accel(gap, v, v_lead, cap, v_des):
    """IDM toward a leader, braking up to the car's limit."""
    s_star = S0 + max(0.0, v * HEADWAY + v * (v - v_lead) / (2.0 * math.sqrt(cap.a_max * 2.0)))
    free = 1.0 - (v / max(v_des, 0.1)) ** 4
    a = cap.a_max * (free - (s_star / max(gap, 0.1)) ** 2)
    return max(-cap.b_max, min(cap.a_max, a))


V_FREE = 1e3  # m/s: IDM's desired speed for its interaction term alone


def follow_cruise(gap, v, v_lead, cap, v_des):
    """A follower that regains its speed: IDM's interaction term toward the leader, capped by the
    car's cruise control toward v_des (gain 1.5 /s, -1 .. a_max), as the arena's cruise.

    follow_accel with v_des = the car's own start speed is IDM's free-road term at a desired speed
    equal to the leader's: the follower then approaches the leader's speed only asymptotically
    while the gap grows without bound (12 m/s behind a 12 m/s leader: 11.4 m/s and a 29 m gap
    after 20 s, 11.6 m/s and 38 m after 40 s), so its lost progress grew with the run's length.
    This one settles at the leader's speed and IDM's gap S0 + v HEADWAY (074 N3)."""
    a = follow_accel(gap, v, v_lead, cap, V_FREE)
    return min(a, max(-1.0, min(cap.a_max, 1.5 * (v_des - v))))


FOLLOW_CRUISE = False  # C1's followers: follow_accel (False, as C1 was built) or follow_cruise


def _follow(gap, v, v_lead, cap, v_des):
    return (follow_cruise if FOLLOW_CRUISE else follow_accel)(gap, v, v_lead, cap, v_des)


def cruise(v, v_des, cap):
    return min(cap.a_max, 0.5 * (v_des - v) / DT) if v < v_des else 0.0


def simulate(a_strat, b_strat, d0, dv, vb, cap_a, cap_b, b_cont=HOLD):
    """One run. `b_cont` is B's continuation under REACT once it has seen A's lane change."""
    va0 = max(0.0, min(cap_a.v_max, vb + dv))
    vb0 = min(cap_b.v_max, vb)
    xa, xb = d0 + cap_a.length, 0.0  # fronts
    va, vbb = va0, vb0
    lat = 0.0
    decide = 0.0 if a_strat == GO else None  # when A decided to change lanes
    dropping = a_strat == WAIT  # A is dropping behind B
    lat_seen = None  # when A's lateral motion began
    in_lane_at = None  # when A entered B's lane
    b_brake_seen = None  # when A could first see B yielding
    b_prev_a = 0.0
    contact = False
    meas = M.Measures()
    peak = [0.0, 0.0]
    t = 0.0
    while t < T_MAX - 1e-9:
        if t >= HORIZON - 1e-9 and (lat >= 1.0 or (decide is None and not dropping and a_strat != GO_IF_YIELD)):
            break
        in_lane = lat >= LANE_LINE
        # --- A
        if a_strat == GO_IF_YIELD and decide is None and not dropping:
            if b_brake_seen is not None and t >= b_brake_seen + cap_a.reaction:
                decide = t
            elif t >= cap_a.reaction + WATCH:
                dropping = True
        if dropping and decide is None:
            gap_behind = (xb - cap_b.length) - xa
            if gap_behind >= S0 + vbb * HEADWAY:
                decide = t
            aa = -A_WAIT_DECEL if gap_behind < S0 + vbb * HEADWAY and va > 0.5 * vbb else 0.0
            if decide is not None:
                aa = _follow(gap_behind, va, vbb, cap_a, va0)
        elif lat > 0.0 and xa < xb:
            aa = _follow((xb - cap_b.length) - xa, va, vbb, cap_a, va0)
        else:
            aa = cruise(va, va0, cap_a)
        # --- B
        if lat > 0.0 and lat_seen is None:
            lat_seen = t
        if in_lane and xa > xb and in_lane_at is None:
            in_lane_at = t
        gap_b = (xa - cap_a.length) - xb
        a_ahead = xa > xb - cap_b.length
        yielding = False
        if b_strat == PRE_YIELD and t >= cap_b.reaction and a_ahead:
            yielding = True
        if (b_strat == REACT and b_cont == YIELD and lat_seen is not None
                and t >= lat_seen + cap_b.reaction and a_ahead):
            yielding = True
        if yielding:
            ab = -B_YIELD_DECEL if gap_b < S0 + vbb * HEADWAY else _follow(gap_b, vbb, va, cap_b, vb0)
        else:
            ab = cruise(vbb, vb0, cap_b)
        if in_lane_at is not None and t >= in_lane_at + cap_b.reaction and xa > xb:
            ab = min(ab, _follow(gap_b, vbb, va, cap_b, vb0))  # the safety layer
        if ab <= -YIELD_SEEN and b_prev_a > -YIELD_SEEN and b_brake_seen is None:
            b_brake_seen = t
        b_prev_a = ab
        peak[0] = max(peak[0], -aa)
        peak[1] = max(peak[1], -ab)
        va = max(0.0, min(cap_a.v_max, va + aa * DT))
        vbb = max(0.0, min(cap_b.v_max, vbb + ab * DT))
        xa += va * DT
        xb += vbb * DT
        if decide is not None:
            lat = min(1.0, max(0.0, (t + DT - decide - cap_a.t_prep) / cap_a.t_lc))
        t += DT
        # the cars as boxes: B's lane at y = 0, A's at LANE_W; A's lateral motion follows the
        # arena's cosine profile (the same crossing of the lane line as `lat`)
        if decide is not None:
            u = (t - decide - cap_a.t_prep) / cap_a.t_lc
            ya = LANE_W * (1 - 0.5 * (1 - math.cos(math.pi * lat)))
            vya = -LANE_W * 0.5 * math.pi / cap_a.t_lc * math.sin(math.pi * lat) if 0 < u < 1 else 0.0
        else:
            ya, vya = LANE_W, 0.0
        vxa = math.sqrt(max(0.0, va * va - vya * vya))
        pa = (xa - cap_a.length / 2, ya, math.atan2(vya, max(vxa, 0.1)), vxa, vya, cap_a.length, cap_a.width)
        pb = (xb - cap_b.length / 2, 0.0, 0.0, vbb, 0.0, cap_b.length, cap_b.width)
        meas.push_poses(t, pa, pb)
        if meas.clear <= 0.0:
            contact = True
            break
    # progress is judged over the whole run, which lasts until A's merge is complete: an
    # unfinished merge no longer scores as a total loss at a fixed horizon
    # lost progress over the run, as a share of free flow over a fixed HORIZON: a run that goes on
    # past HORIZON (until A's merge is complete) adds its loss without diluting it (074 D2)
    free_a, free_b = va0 * t, vb0 * t
    lost_a = min(1.0, max(0.0, (free_a - (xa - d0 - cap_a.length)) / max(va0 * HORIZON, 1.0)))
    lost_b = min(1.0, max(0.0, (free_b - xb) / max(vb0 * HORIZON, 1.0)))
    if contact:
        lost_a = lost_b = 1.0
    if decide is None:
        lost_a = 1.0
    went = decide is not None and not dropping
    meas.pet = meas._pet()
    return Outcome((lost_a, lost_b), meas.ttc2d, meas.thw, contact, (peak[0], peak[1]), went,
                   b_cont if b_strat == REACT else "", meas.clear, meas.pet, tuple(meas.lat_peak))


def utility(o, who, cap, pref):
    """The general utility (measures.utility), with the car's preference weights."""
    risk = M.rho(o.min_ttc, o.min_thw, o.min_pet, o.min_clear)
    u = -pref.w_progress * o.progress[who] - pref.w_risk * risk
    u -= pref.w_comfort * (min(o.peak_brake[who], cap.b_max) / cap.b_max) ** 2
    u -= pref.w_comfort * (min(o.peak_lat[who], cap.a_lat_max) / cap.a_lat_max) ** 2
    if o.contact:
        u -= pref.w_contact
    return u


def svo(u_own, u_other, phi_deg):
    phi = math.radians(phi_deg)
    return math.cos(phi) * u_own + math.sin(phi) * u_other


@dataclass
class Solution:
    ja: dict  # (a, b) -> A's payoff J
    jb: dict
    ua: dict  # own utilities U, for welfare
    ub: dict
    pure: list
    mixed: list  # [(pA: dict, pB: dict)]
    outcomes: dict


def solve(ja, jb, a_acts, b_acts):
    """Pure equilibria, and the mixed ones of support size 2 x 2 (support enumeration)."""
    pure = []
    for a, b in itertools.product(a_acts, b_acts):
        if all(ja[(a, b)] >= ja[(x, b)] - 1e-9 for x in a_acts) and \
           all(jb[(a, b)] >= jb[(a, y)] - 1e-9 for y in b_acts):
            pure.append((a, b))
    mixed = []
    for (a1, a2), (b1, b2) in itertools.product(itertools.combinations(a_acts, 2),
                                                 itertools.combinations(b_acts, 2)):
        den_p = (jb[(a1, b1)] - jb[(a1, b2)]) - (jb[(a2, b1)] - jb[(a2, b2)])
        den_q = (ja[(a1, b1)] - ja[(a2, b1)]) - (ja[(a1, b2)] - ja[(a2, b2)])
        if abs(den_p) < 1e-9 or abs(den_q) < 1e-9:
            continue
        p = (jb[(a2, b2)] - jb[(a2, b1)]) / den_p  # P(a1) making B indifferent
        q = (ja[(a2, b2)] - ja[(a1, b2)]) / den_q  # P(b1) making A indifferent
        if not (1e-6 < p < 1 - 1e-6 and 1e-6 < q < 1 - 1e-6):
            continue
        v_a = q * ja[(a1, b1)] + (1 - q) * ja[(a1, b2)]
        # no unused A action may do better against q
        if any(q * ja[(x, b1)] + (1 - q) * ja[(x, b2)] > v_a + 1e-9 for x in a_acts if x not in (a1, a2)):
            continue
        mixed.append(({a1: p, a2: 1 - p}, {b1: q, b2: 1 - q}))
    return pure, mixed


def play(d0, dv, vb, cap_a, cap_b, pref_a, pref_b, svo_a=0.0, svo_b=0.0):
    outs = {}
    for a in A_STRATS:
        # B's continuation under REACT: its best reply once it has seen A's lane change
        cands = [simulate(a, REACT, d0, dv, vb, cap_a, cap_b, c) for c in (YIELD, HOLD)]
        best = max(cands, key=lambda o: svo(utility(o, 1, cap_b, pref_b), utility(o, 0, cap_a, pref_a), svo_b))
        outs[(a, REACT)] = best
        outs[(a, PRE_YIELD)] = simulate(a, PRE_YIELD, d0, dv, vb, cap_a, cap_b)
    ua = {k: utility(o, 0, cap_a, pref_a) for k, o in outs.items()}
    ub = {k: utility(o, 1, cap_b, pref_b) for k, o in outs.items()}
    ja = {k: svo(ua[k], ub[k], svo_a) for k in outs}
    jb = {k: svo(ub[k], ua[k], svo_b) for k in outs}
    pure, mixed = solve(ja, jb, A_STRATS, B_STRATS)
    return Solution(ja, jb, ua, ub, pure, mixed, outs)


def welfare(sol):
    """Costs C = -(U_A + U_B): the optimum, the worst and best equilibrium."""
    cost = {k: -(sol.ua[k] + sol.ub[k]) for k in sol.outcomes}
    c_opt = min(cost.values())
    eq = [cost[p] for p in sol.pure]
    for pa, pb in sol.mixed:
        eq.append(sum(pa[a] * pb[b] * cost[(a, b)] for a in pa for b in pb))
    if not eq:
        return c_opt, None, None
    return c_opt, max(eq), min(eq)


def kind(sol, prof):
    a, b = prof
    o = sol.outcomes[prof]
    if o.contact:
        return "!"
    if not o.went:
        return "W"
    yielded = b == PRE_YIELD or o.b_cont == YIELD
    if a == GO_IF_YIELD:
        return "Y" if yielded else "y"
    return "G" if yielded else "g"


def label(sol):
    """One letter per context, for the map.

    W  A drops behind B
    G  A goes, B yields          (B opens the gap after seeing A)
    g  A goes, B holds           (A is clear ahead)
    Y  A waits for B's yield and then goes  (a negotiated cut-in)
    y  A goes after a yield-free watch      (rare)
    C  equilibria with different outcomes: coordination; who wins is a convention or a belief
    M  only mixed equilibria
    !  an equilibrium has contact
    """
    kinds = {kind(sol, p) for p in sol.pure}
    if "!" in kinds:
        return "!"
    if len(kinds) > 1:
        return "C"
    if kinds:
        return kinds.pop()
    return "M" if sol.mixed else "?"


LEGEND = (label.__doc__ or "").strip().splitlines()


def grid(lo, hi, step):
    n = int(round((hi - lo) / step))
    return [round(lo + i * step, 6) for i in range(n + 1)]


def run_map(args):
    cap_a, cap_b = TYPES[args.a], TYPES[args.b]
    pref = Preference()
    d0s = grid(args.d0_min, args.d0_max, args.d0_step)
    dvs = grid(args.dv_min, args.dv_max, args.dv_step)
    os.makedirs(args.out, exist_ok=True)
    rows, labels, poa = [], {}, {}
    for dv in dvs:
        for d0 in d0s:
            sol = play(d0, dv, args.vb, cap_a, cap_b, pref, pref, args.svo_a, args.svo_b)
            lab = label(sol)
            labels[(d0, dv)] = lab
            c_opt, c_worst, c_best = welfare(sol)
            poa[(d0, dv)] = (c_worst - c_opt) if c_worst is not None else None
            row = {"d0": d0, "dv": dv, "class": lab,
                   "pure": " ".join(f"{a}/{b}" for a, b in sol.pure),
                   "mixed": " | ".join(" ".join(f"{k}:{v:.2f}" for k, v in {**pa, **pb}.items())
                                       for pa, pb in sol.mixed),
                   "C_opt": f"{c_opt:.3f}",
                   "C_worst_eq": "" if c_worst is None else f"{c_worst:.3f}",
                   "C_best_eq": "" if c_best is None else f"{c_best:.3f}",
                   "PoA": "" if c_worst is None or c_opt < 0.05 else f"{c_worst / c_opt:.3f}",
                   "PoS": "" if c_best is None or c_opt < 0.05 else f"{c_best / c_opt:.3f}"}
            for (a, b), o in sol.outcomes.items():
                row[f"uA_{a}_{b}"] = f"{sol.ua[(a, b)]:.3f}"
                row[f"uB_{a}_{b}"] = f"{sol.ub[(a, b)]:.3f}"
                row[f"ttc_{a}_{b}"] = f"{o.min_ttc:.2f}" if math.isfinite(o.min_ttc) else "inf"
                row[f"thw_{a}_{b}"] = f"{o.min_thw:.2f}" if math.isfinite(o.min_thw) else "inf"
                row[f"contact_{a}_{b}"] = int(o.contact)
                if b == REACT:
                    row[f"bcont_{a}"] = o.b_cont
            rows.append(row)
    with open(os.path.join(args.out, "cutin_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    head = [f"# cut-in equilibrium map: A={args.a} B={args.b} vB={args.vb} m/s "
            f"SVO A={args.svo_a} B={args.svo_b} deg",
            f"# rows: dv (A minus B, m/s), top = fastest; columns: d0 (m) {d0s[0]}..{d0s[-1]} "
            f"step {args.d0_step}"]
    head += ["#   " + l.strip() for l in LEGEND[2:] if l.strip()]
    lines = head + [f"{dv:+6.1f} " + "".join(labels[(d0, dv)] for d0 in d0s) for dv in reversed(dvs)]
    lines.append("# welfare loss of the worst equilibrium, C_worst - C_opt: . <0.05  - <0.2  + <0.5  # >=0.5")
    for dv in reversed(dvs):
        s = ""
        for d0 in d0s:
            x = poa[(d0, dv)]
            s += "?" if x is None else "." if x < 0.05 else "-" if x < 0.2 else "+" if x < 0.5 else "#"
        lines.append(f"{dv:+6.1f} " + s)
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "cutin_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    try:
        plot(args, d0s, dvs, labels)
    except ImportError:
        print("matplotlib missing: no PNG", file=sys.stderr)
    return 0


def plot(args, d0s, dvs, labels):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    order = "GgYyWCM!?"
    names = ["go / B yields", "go / B holds", "go after B's yield", "go after a watch", "drop behind",
             "coordination (several equilibria)", "mixed only", "contact in equilibrium", "none"]
    colours = ["#2a9d8f", "#8ab17d", "#6a4c93", "#b8a1d9", "#457b9d", "#e9c46a", "#f4a261",
               "#e63946", "#999999"]
    z = [[order.index(labels[(d0, dv)]) for d0 in d0s] for dv in dvs]
    fig = plt.figure(figsize=(8, 4.5))
    ax = fig.add_subplot(1, 1, 1)
    ax.imshow(z, origin="lower", aspect="auto", cmap=ListedColormap(colours), vmin=-0.5,
              vmax=len(order) - 0.5, extent=(d0s[0], d0s[-1], dvs[0], dvs[-1]))
    present = sorted({labels[k] for k in labels}, key=order.index)
    ax.legend(handles=[Patch(color=colours[order.index(c)], label=names[order.index(c)])
                       for c in present], loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    ax.set_xlabel("d0: A's rear ahead of B's front (m)")
    ax.set_ylabel("dv: A's speed minus B's (m/s)")
    ax.set_title(f"Cut-in equilibria, A={args.a}, B={args.b}, vB={args.vb} m/s (kinematic)")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "cutin_map.png"), dpi=120)


def selftest():
    car = TYPES["car"]
    pref = Preference()
    # far ahead and faster: A goes, B need not yield
    s = play(30.0, 2.0, 12.0, car, car, pref, pref)
    assert any(sol_kind in "gG" for sol_kind in {kind(s, p) for p in s.pure}), s.pure
    # A's nose beside B's and slower: no equilibrium has A going
    s = play(-4.0, -2.0, 12.0, car, car, pref, pref)
    assert all(not s.outcomes[p].went for p in s.pure), s.pure
    # dropping behind never touches
    for d0, dv in itertools.product((-4.0, 0.0, 5.0, 20.0), (-3.0, 0.0, 3.0)):
        assert not simulate(WAIT, REACT, d0, dv, 12.0, car, car).contact, (d0, dv)
    # REACT is credible: B's continuation is its best reply, so B never holds into a worse payoff
    s = play(6.0, 0.0, 12.0, car, car, pref, pref)
    o_y = simulate(GO, REACT, 6.0, 0.0, 12.0, car, car, YIELD)
    o_h = simulate(GO, REACT, 6.0, 0.0, 12.0, car, car, HOLD)
    best = max(utility(o_y, 1, car, pref), utility(o_h, 1, car, pref))
    assert abs(s.ub[(GO, REACT)] - best) < 1e-9
    # the solver on textbook chicken: two pure and one mixed; kappa = 10/11
    ja = {("go", "y"): 1, ("go", "h"): -10, ("w", "y"): 0, ("w", "h"): 0}
    jb = {("go", "y"): 0, ("go", "h"): -10, ("w", "y"): 0, ("w", "h"): 1}
    pure, mixed = solve(ja, jb, ("go", "w"), ("y", "h"))
    assert set(pure) == {("go", "y"), ("w", "h")} and len(mixed) == 1, (pure, mixed)
    assert abs(mixed[0][1]["y"] - 10 / 11) < 1e-9, mixed
    # welfare costs are non-negative, and an equilibrium never beats the optimum
    s = play(8.0, 0.0, 12.0, car, car, pref, pref)
    c_opt, c_worst, c_best = welfare(s)
    assert c_opt >= 0 and c_best is not None and c_best >= c_opt - 1e-9 and c_worst >= c_best
    # Autoware's prepare phase delays the lane change: same context, later entry
    aw = TYPES["autoware"]
    assert simulate(GO, REACT, 15.0, 0.0, 12.0, aw, car).progress[0] >= \
        simulate(GO, REACT, 15.0, 0.0, 12.0, car, car).progress[0]
    # a slower-braking B makes the same cut-in no safer for B
    weak = replace(car, b_max=4.0, reaction=1.5)
    o_car = simulate(GO, REACT, 4.0, 0.0, 12.0, car, car, HOLD)
    o_weak = simulate(GO, REACT, 4.0, 0.0, 12.0, car, weak, HOLD)
    assert o_weak.contact or o_weak.min_ttc <= o_car.min_ttc
    # follow_cruise regains the leader's speed and holds IDM's gap; follow_accel at v_des = v0 does not
    for f, ok in ((follow_cruise, True), (follow_accel, False)):
        xl, xf, vf = 10.0 + car.length, 0.0, 12.0
        for _ in range(int(30.0 / DT)):
            vf += f(xl - car.length - xf, vf, 12.0, car, 12.0) * DT
            xf += vf * DT
            xl += 12.0 * DT
        assert (abs(vf - 12.0) < 0.02 and abs(xl - car.length - xf - (S0 + 12.0 * HEADWAY)) < 0.5) == ok, (f, vf)
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="cutin_out")
    p.add_argument("--a", choices=sorted(TYPES), default="car", help="A's capability type")
    p.add_argument("--b", choices=sorted(TYPES), default="car", help="B's capability type")
    p.add_argument("--vb", type=float, default=12.0, help="B's speed (m/s)")
    p.add_argument("--svo-a", type=float, default=0.0, help="A's SVO angle (deg)")
    p.add_argument("--svo-b", type=float, default=0.0, help="B's SVO angle (deg)")
    p.add_argument("--d0-min", type=float, default=-6.0)
    p.add_argument("--d0-max", type=float, default=30.0)
    p.add_argument("--d0-step", type=float, default=1.0)
    p.add_argument("--dv-min", type=float, default=-4.0)
    p.add_argument("--dv-max", type=float, default=4.0)
    p.add_argument("--dv-step", type=float, default=0.5)
    p.add_argument("--selftest", action="store_true")
    args = p.parse_args()
    return selftest() if args.selftest else run_map(args)


if __name__ == "__main__":
    sys.exit(main())
