#!/usr/bin/env python3
"""Roadmap 074 C1: the two-car cut-in as a 2 x 2 game in a kinematic model.

A, in the left lane, wants into B's lane. A plays GO (change lanes now, ahead of B) or WAIT
(drop behind B, then change). B plays YIELD (brake to open a gap) or HOLD (keep its speed; its
safety reflex still brakes once A is in its lane). Each of the four profiles is simulated on a
straight two-lane road; the payoffs come from the trajectories: progress against free flow,
the smallest time to collision, contact and the hardest braking. The game is solved per
context (d0, dv): its pure Nash equilibria and, where there is none or two, the mixed one.

Contexts:
  d0  A's rear minus B's front at t = 0 (m); negative: A's rear is beside or behind B's nose
  dv  A's speed minus B's (m/s)

Types are capability (theta) and preference (phi). Capabilities here are first numbers, marked
assumed in TYPES; C5 replaces them with `just vehicle-check` measurements.

The output is the prediction the swb runs (C2, C3) are compared with:
  <out>/cutin_map.csv     one row per context: payoffs, equilibria, class
  <out>/cutin_map.txt     the class map as text
  <out>/cutin_map.png     the same, when matplotlib is there

Usage:
  scripts/coop/cutin_game.py --out <dir> [--a car] [--b car] [--vb 12]
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

DT = 0.05
HORIZON = 12.0
LANE_LINE = 0.5  # A's lateral progress (0 own lane, 1 B's lane) at which it occupies B's lane


@dataclass(frozen=True)
class Capability:
    """theta: what the car can do. Limits, not habits."""

    length: float  # m
    a_max: float  # m/s^2, drive
    b_max: float  # m/s^2, braking, friction or brake limited
    v_max: float  # m/s
    reaction: float  # s, from an event to the first braking: perception + planning + actuation
    t_lc: float  # s, a lane change from start to end


@dataclass(frozen=True)
class Preference:
    """phi: how the car trades progress against risk and comfort."""

    w_progress: float = 1.0  # per unit of lost progress (fraction of free-flow distance)
    w_risk: float = 1.0  # per unit of TTC or headway shortfall below its reference (0..1)
    w_contact: float = 10.0
    w_comfort: float = 0.3  # per (peak braking / b_max)^2
    ttc_ref: float = 3.0  # s
    thw_ref: float = 1.0  # s; equal speeds at 2 m have no TTC but are not safe


# First numbers. The Model 3's length and limits from sim_vehicle_spec (4.792 m, ~8 m/s^2
# drive torque, friction-limited stops); the reaction time is Autoware's own assumption for
# a rear car (lane_change rear_vehicle_reaction_time, 1-2 s) at its low end. The golf cart's
# are assumed: a 2.4 m cart, 1.5 m/s^2, 25 km/h.
TYPES = {
    "car": Capability(length=4.79, a_max=3.0, b_max=7.0, v_max=30.0, reaction=1.0, t_lc=3.0),
    "car_alert": Capability(length=4.79, a_max=3.0, b_max=7.0, v_max=30.0, reaction=0.5, t_lc=3.0),
    "golfcart": Capability(length=2.4, a_max=1.5, b_max=4.0, v_max=6.9, reaction=1.0, t_lc=3.0),
}

GO, WAIT = "go", "wait"
YIELD, HOLD = "yield", "hold"
A_ACTIONS = (GO, WAIT)
B_ACTIONS = (YIELD, HOLD)

# Driving habits shared by both strategies of a car.
B_YIELD_DECEL = 2.0  # m/s^2, a comfortable yield
A_WAIT_DECEL = 2.0  # m/s^2, A dropping back
HEADWAY = 1.0  # s, desired time gap once following
S0 = 2.0  # m, standstill gap


@dataclass
class Outcome:
    progress: tuple[float, float]  # lost progress per car, fraction of free flow (>= 0 is a loss)
    min_ttc: float  # s, in one lane, A ahead of B or B ahead of A
    min_thw: float  # s, the follower's time headway in one lane
    contact: bool
    peak_brake: tuple[float, float]  # m/s^2


def follow_accel(gap: float, v: float, v_lead: float, cap: Capability, v_des: float) -> float:
    """IDM toward a leader, braking up to the car's limit."""
    s_star = S0 + max(0.0, v * HEADWAY + v * (v - v_lead) / (2.0 * math.sqrt(cap.a_max * 2.0)))
    free = 1.0 - (v / max(v_des, 0.1)) ** 4
    a = cap.a_max * (free - (s_star / max(gap, 0.1)) ** 2)
    return max(-cap.b_max, min(cap.a_max, a))


def simulate(a_act: str, b_act: str, d0: float, dv: float, vb: float,
             cap_a: Capability, cap_b: Capability) -> Outcome:
    va0 = max(0.0, min(cap_a.v_max, vb + dv))
    vb0 = min(cap_b.v_max, vb)
    v_des_a, v_des_b = va0, vb0
    # Positions are the cars' fronts. B's front at 0; A's rear at d0, so its front at d0 + L_A.
    xa, xb = d0 + cap_a.length, 0.0
    va, vbb = va0, vb0
    lat = 0.0  # A's lateral progress
    lc_start = 0.0 if a_act == GO else None
    b_alarm = None  # time B's reflex sees A in its lane in front
    min_ttc = math.inf
    min_thw = math.inf
    contact = False
    peak = [0.0, 0.0]
    t = 0.0
    steps = int(HORIZON / DT)
    for _ in range(steps):
        in_lane = lat >= LANE_LINE
        # A's longitudinal choice
        if a_act == WAIT and lc_start is None:
            # drop back until A's front is a safe gap behind B's rear, then change
            target_gap = S0 + vbb * HEADWAY
            gap_behind = (xb - cap_b.length) - xa
            if gap_behind >= target_gap:
                lc_start = t
                aa = follow_accel(gap_behind, va, vbb, cap_a, v_des_a)
            else:
                aa = -A_WAIT_DECEL if va > 0.5 * vbb else 0.0
        elif in_lane and xa < xb:
            # A behind B in B's lane: follow B
            aa = follow_accel((xb - cap_b.length) - xa, va, vbb, cap_a, v_des_a)
        else:
            aa = min(cap_a.a_max, 0.5 * (v_des_a - va) / DT) if va < v_des_a else 0.0
        # B's longitudinal choice
        a_ahead_in_lane = in_lane and xa > xb
        if a_ahead_in_lane and b_alarm is None:
            b_alarm = t
        gap_b = (xa - cap_a.length) - xb
        if b_act == YIELD and t >= cap_b.reaction and xa > xb - cap_b.length:
            # open a gap behind A, from t = reaction, until it is a following gap
            want = S0 + vbb * HEADWAY
            ab = -B_YIELD_DECEL if gap_b < want else follow_accel(gap_b, vbb, va, cap_b, v_des_b)
        elif b_alarm is not None and t >= b_alarm + cap_b.reaction:
            ab = follow_accel(gap_b, vbb, va, cap_b, v_des_b)
        else:
            ab = min(cap_b.a_max, 0.5 * (v_des_b - vbb) / DT) if vbb < v_des_b else 0.0
        if a_ahead_in_lane and t >= cap_b.reaction:
            # whatever B planned, its safety layer brakes as hard as the gap needs
            ab = min(ab, follow_accel(gap_b, vbb, va, cap_b, v_des_b))
        peak[0] = max(peak[0], -aa)
        peak[1] = max(peak[1], -ab)
        va = max(0.0, min(cap_a.v_max, va + aa * DT))
        vbb = max(0.0, min(cap_b.v_max, vbb + ab * DT))
        xa += va * DT
        xb += vbb * DT
        if lc_start is not None:
            lat = min(1.0, (t + DT - lc_start) / cap_a.t_lc)
        t += DT
        if lat >= LANE_LINE:
            overlap = (xa > xb - cap_b.length) and (xa - cap_a.length < xb)
            if overlap:
                contact = True
                break
            if xa > xb:  # A leads
                closing, v_follow = vbb - va, vbb
                gap = (xa - cap_a.length) - xb
            else:
                closing, v_follow = va - vbb, va
                gap = (xb - cap_b.length) - xa
            if v_follow > 1.0:
                min_thw = min(min_thw, gap / v_follow)
            if closing > 1e-3:
                min_ttc = min(min_ttc, gap / closing)
    free_a = v_des_a * HORIZON
    free_b = v_des_b * HORIZON
    da = xa - (d0 + cap_a.length)
    lost_a = max(0.0, (free_a - da) / max(free_a, 1.0))
    lost_b = max(0.0, (free_b - xb) / max(free_b, 1.0))
    if contact:
        lost_a = lost_b = 1.0
    if lc_start is None:  # A never got in
        lost_a = 1.0
    return Outcome((lost_a, lost_b), min_ttc, min_thw, contact, (peak[0], peak[1]))


def utility(o: Outcome, who: int, cap: Capability, pref: Preference) -> float:
    risk = 0.0
    if math.isfinite(o.min_ttc):
        risk = max(risk, (pref.ttc_ref - o.min_ttc) / pref.ttc_ref)
    if math.isfinite(o.min_thw):
        risk = max(risk, (pref.thw_ref - o.min_thw) / pref.thw_ref)
    u = -pref.w_progress * o.progress[who] - pref.w_risk * risk
    u -= pref.w_comfort * (min(o.peak_brake[who], cap.b_max) / cap.b_max) ** 2
    if o.contact:
        u -= pref.w_contact
    return u


@dataclass
class Solution:
    ua: dict  # (a_act, b_act) -> A's utility
    ub: dict
    pure: list  # pure Nash equilibria
    mixed: tuple[float, float] | None  # (P(A go), P(B yield)) of the fully mixed one, if any
    outcomes: dict


def solve(ua: dict, ub: dict) -> tuple[list, tuple[float, float] | None]:
    pure = []
    for a, b in itertools.product(A_ACTIONS, B_ACTIONS):
        a_other = WAIT if a == GO else GO
        b_other = HOLD if b == YIELD else YIELD
        if ua[(a, b)] >= ua[(a_other, b)] - 1e-9 and ub[(a, b)] >= ub[(a, b_other)] - 1e-9:
            pure.append((a, b))
    # fully mixed: A's P(go) = p makes B indifferent, B's P(yield) = q makes A indifferent
    den_p = (ub[(GO, YIELD)] - ub[(GO, HOLD)]) - (ub[(WAIT, YIELD)] - ub[(WAIT, HOLD)])
    den_q = (ua[(GO, YIELD)] - ua[(WAIT, YIELD)]) - (ua[(GO, HOLD)] - ua[(WAIT, HOLD)])
    mixed = None
    if abs(den_p) > 1e-9 and abs(den_q) > 1e-9:
        p = (ub[(WAIT, HOLD)] - ub[(WAIT, YIELD)]) / den_p
        q = (ua[(WAIT, HOLD)] - ua[(GO, HOLD)]) / den_q
        if 0.0 < p < 1.0 and 0.0 < q < 1.0:
            mixed = (p, q)
    return pure, mixed


def play(d0, dv, vb, cap_a, cap_b, pref_a, pref_b) -> Solution:
    ua, ub, outs = {}, {}, {}
    for a, b in itertools.product(A_ACTIONS, B_ACTIONS):
        o = simulate(a, b, d0, dv, vb, cap_a, cap_b)
        outs[(a, b)] = o
        ua[(a, b)] = utility(o, 0, cap_a, pref_a)
        ub[(a, b)] = utility(o, 1, cap_b, pref_b)
    pure, mixed = solve(ua, ub)
    return Solution(ua, ub, pure, mixed, outs)


def label(sol: Solution) -> str:
    """One letter per context, for the map.

    G  A goes, B yields          (the courteous cut-in)
    g  A goes, B holds           (A is clear ahead; no yield needed)
    W  A waits, B holds          (A merges behind)
    w  A waits, B yields         (B slows, A still waits: inefficient)
    C  pure equilibria with A going in one and waiting in another, and a mixed one:
       chicken; who wins is a convention or a belief
    M  only the mixed one
    !  a pure equilibrium has contact
    """
    pure = set(sol.pure)
    if any(sol.outcomes[p].contact for p in pure):
        return "!"
    if {a for a, _ in pure} == {GO, WAIT}:
        return "C"
    if pure:
        # B indifferent (both its actions equilibria with the same A action): B's choice does
        # not matter, which is a hold
        b = HOLD if len(pure) > 1 else next(iter(pure))[1]
        a = next(iter(pure))[0]
        return {(GO, YIELD): "G", (GO, HOLD): "g", (WAIT, HOLD): "W", (WAIT, YIELD): "w"}[(a, b)]
    return "M" if sol.mixed else "?"


LEGEND = (label.__doc__ or "").strip().splitlines()


def grid(lo, hi, step):
    n = int(round((hi - lo) / step))
    return [round(lo + i * step, 6) for i in range(n + 1)]


def run_map(args) -> int:
    cap_a, cap_b = TYPES[args.a], TYPES[args.b]
    pref = Preference()
    d0s = grid(args.d0_min, args.d0_max, args.d0_step)
    dvs = grid(args.dv_min, args.dv_max, args.dv_step)
    os.makedirs(args.out, exist_ok=True)
    rows, labels = [], {}
    for dv in dvs:
        for d0 in d0s:
            sol = play(d0, dv, args.vb, cap_a, cap_b, pref, pref)
            lab = label(sol)
            labels[(d0, dv)] = lab
            row = {"d0": d0, "dv": dv, "class": lab,
                   "pure": " ".join(f"{a}/{b}" for a, b in sol.pure),
                   "mixed_p_go": f"{sol.mixed[0]:.3f}" if sol.mixed else "",
                   "mixed_q_yield": f"{sol.mixed[1]:.3f}" if sol.mixed else ""}
            for a, b in itertools.product(A_ACTIONS, B_ACTIONS):
                o = sol.outcomes[(a, b)]
                row[f"uA_{a}_{b}"] = f"{sol.ua[(a, b)]:.3f}"
                row[f"uB_{a}_{b}"] = f"{sol.ub[(a, b)]:.3f}"
                row[f"ttc_{a}_{b}"] = f"{o.min_ttc:.2f}" if math.isfinite(o.min_ttc) else "inf"
                row[f"thw_{a}_{b}"] = f"{o.min_thw:.2f}" if math.isfinite(o.min_thw) else "inf"
                row[f"contact_{a}_{b}"] = int(o.contact)
            rows.append(row)
    with open(os.path.join(args.out, "cutin_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    lines = [f"# cut-in equilibrium map: A={args.a} B={args.b} vB={args.vb} m/s",
             "# rows: dv (A minus B, m/s), top = fastest; columns: d0 (m) "
             f"{d0s[0]}..{d0s[-1]} step {args.d0_step}",
             "# " + LEGEND[0]]
    lines += ["#   " + l.strip() for l in LEGEND[2:] if l.strip()]
    for dv in reversed(dvs):
        lines.append(f"{dv:+6.1f} " + "".join(labels[(d0, dv)] for d0 in d0s))
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
    order = "GgWwCM!?"
    names = ["go / yield", "go / hold", "wait / hold", "wait / yield", "chicken (2 pure + mixed)",
             "mixed only", "contact in equilibrium", "none"]
    colours = ["#2a9d8f", "#8ab17d", "#457b9d", "#a8dadc", "#e9c46a", "#f4a261", "#e63946", "#999999"]
    z = [[order.index(labels[(d0, dv)]) for d0 in d0s] for dv in dvs]
    fig = plt.figure(figsize=(8, 4.5))
    ax = fig.add_subplot(1, 1, 1)
    ax.imshow(z, origin="lower", aspect="auto", cmap=ListedColormap(colours), vmin=-0.5,
              vmax=len(order) - 0.5, extent=(d0s[0], d0s[-1], dvs[0], dvs[-1]))
    present = sorted({labels[k] for k in labels}, key=order.index)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=colours[order.index(c)], label=names[order.index(c)])
                       for c in present], loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    ax.set_xlabel("d0: A's rear ahead of B's front (m)")
    ax.set_ylabel("dv: A's speed minus B's (m/s)")
    ax.set_title(f"Cut-in equilibria, A={args.a}, B={args.b}, vB={args.vb} m/s (kinematic)")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "cutin_map.png"), dpi=120)


def selftest() -> int:
    car = TYPES["car"]
    pref = Preference()
    # far ahead and faster: A just goes, nobody needs to yield
    s = play(30.0, 2.0, 12.0, car, car, pref, pref)
    assert (GO, HOLD) in s.pure, s.pure
    assert not s.outcomes[(GO, HOLD)].contact
    # A's nose beside B's, slower: going is a crash, A waits
    s = play(-4.0, -2.0, 12.0, car, car, pref, pref)
    assert s.outcomes[(GO, HOLD)].contact
    assert all(a == WAIT for a, _ in s.pure), s.pure
    # waiting never touches
    for d0, dv in itertools.product((-4.0, 0.0, 5.0, 20.0), (-3.0, 0.0, 3.0)):
        o = simulate(WAIT, HOLD, d0, dv, 12.0, car, car)
        assert not o.contact, (d0, dv)
    # the solver: chicken has two pure equilibria and a mixed one
    ua = {(GO, YIELD): 1, (GO, HOLD): -10, (WAIT, YIELD): 0, (WAIT, HOLD): 0}
    ub = {(GO, YIELD): 0, (GO, HOLD): -10, (WAIT, YIELD): 0, (WAIT, HOLD): 1}
    pure, mixed = solve(ua, ub)
    assert set(pure) == {(GO, YIELD), (WAIT, HOLD)} and mixed is not None, (pure, mixed)
    # kappa: A goes iff P(B yields) > (U_wait - U_crash) / (U_success - U_crash) = 10/11
    assert abs(mixed[1] - 10 / 11) < 1e-9, mixed
    # a slower-braking B makes the same cut-in worse for B
    weak = replace(car, b_max=4.0, reaction=1.5)
    o_car = simulate(GO, HOLD, 4.0, 0.0, 12.0, car, car)
    o_weak = simulate(GO, HOLD, 4.0, 0.0, 12.0, car, weak)
    assert o_weak.contact or o_weak.min_ttc <= o_car.min_ttc
    print("selftest ok")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="cutin_out")
    p.add_argument("--a", choices=sorted(TYPES), default="car", help="A's capability type")
    p.add_argument("--b", choices=sorted(TYPES), default="car", help="B's capability type")
    p.add_argument("--vb", type=float, default=12.0, help="B's speed (m/s)")
    p.add_argument("--d0-min", type=float, default=-6.0)
    p.add_argument("--d0-max", type=float, default=30.0)
    p.add_argument("--d0-step", type=float, default=1.0)
    p.add_argument("--dv-min", type=float, default=-4.0)
    p.add_argument("--dv-max", type=float, default=4.0)
    p.add_argument("--dv-step", type=float, default=0.5)
    p.add_argument("--selftest", action="store_true")
    args = p.parse_args()
    if args.selftest:
        return selftest()
    return run_map(args)


if __name__ == "__main__":
    sys.exit(main())
