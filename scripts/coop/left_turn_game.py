#!/usr/bin/env python3
"""Roadmap 074 U1: the unprotected left turn against an oncoming queue, in a kinematic model.

A waits at the stop line to turn left across one oncoming lane. The oncoming traffic is a chain
of n cars at speed v and time headway h. The game is between A and the chain's first car C1 that
A could turn in front of; the cars behind C1 only follow (IDM), so a courtesy of C1's costs every
follower too: the chain externality.

  A    gap acceptance with threshold G (s): go as soon as the next oncoming car, as A sees it
       (perception delay), will reach the conflict zone G s or more after A starts. A's rule
       space is G in GRID_G; the equilibrium uses A's best G.
  C1   hold  -- keep v; the safety layer brakes (after its reaction) only once A is in its path
       yield -- seeing A waiting within `courtesy_range`, slow at 2 m/s^2 to open a gap, then
                resume once A has cleared

Payoffs (cutin_game's form): A's delay against turning at once, each oncoming car's lost
progress, risk from the post-encroachment time (PET: the time between one car leaving the
conflict zone and the other entering it) below 1.5 s, contact. Welfare is the cost
C = -(U_A + sum over the whole chain), so a yield that delays four followers counts four times.

Contexts: n (queue length, 1-5) and h (headway, s). Output: per context, A's best threshold
against each C1 response, the equilibria and the welfare loss against the optimum.

Usage:
  scripts/coop/left_turn_game.py [--v 10] [--selftest]
"""

import argparse
import math
import sys
from dataclasses import dataclass

DT = 0.05
T_MAX = 40.0
L = 4.79  # m, car length
W_LANE = 3.5
# A's path from the stop line: it enters the oncoming lane after S_IN m and leaves it after S_OUT
S_IN, S_OUT = 6.0, 6.0 + W_LANE + L
A_ACC, A_VMAX = 2.0, 6.0  # m/s^2, m/s through the turn
OBS_DELAY = 0.19
REACT = 1.0
B_MAX = 7.0
YIELD_DECEL = 2.0
COURTESY_RANGE = 40.0
GRID_G = [round(1.0 + 0.5 * i, 1) for i in range(15)]  # 1.0 .. 8.0 s
PET_REF = 1.5


@dataclass
class Out:
    a_delay: float
    lost: list  # per oncoming car, fraction of free-flow progress
    pet: float
    contact: bool
    peak: list


def a_profile(s0_time):
    """A's arc length s(t) after starting at s0_time: accelerate at A_ACC to A_VMAX."""
    def s(t):
        tau = t - s0_time
        if tau <= 0:
            return 0.0
        t1 = A_VMAX / A_ACC
        return 0.5 * A_ACC * tau * tau if tau < t1 else 0.5 * A_ACC * t1 * t1 + A_VMAX * (tau - t1)
    return s


def simulate(G, c1, n, h, v):
    """One run: A with threshold G, C1's response c1 ('hold'/'yield'), n cars at headway h."""
    # oncoming positions: distance to the conflict zone's near edge; car 0 first at 3 s away
    x = [v * (3.0 + i * h) for i in range(n)]
    vel = [v] * n
    a_start = None
    hist = []  # what A sees, delayed: (t, positions, speeds)
    t = 0.0
    contact = False
    pet = math.inf
    peak = [0.0] * n
    occ_a = None  # (t_in, t_out) of A in the zone
    occ = [None] * n  # per car: [t_in, t_out]
    resume = [False] * n
    s_a = lambda tt: 0.0
    while t < T_MAX:
        hist.append((t, list(x), list(vel)))
        seen = next((hh for hh in reversed(hist) if hh[0] <= t - OBS_DELAY), hist[0])
        # A's decision: the next car that has not passed, as seen
        if a_start is None:
            t_need = math.sqrt(2 * S_OUT / A_ACC) if S_OUT < A_VMAX ** 2 / (2 * A_ACC) else \
                A_VMAX / A_ACC + (S_OUT - A_VMAX ** 2 / (2 * A_ACC)) / A_VMAX
            nxt = [(xs, vs) for xs, vs in zip(seen[1], seen[2]) if xs > -L]
            ok = True
            for xs, vs in nxt:
                tta = xs / max(vs, 0.1) if xs > 0 else 0.0
                if tta < G:
                    ok = False
                    break
                if tta > G + t_need + 2:  # this and the following cars are far enough
                    break
            if ok:
                a_start = t
                s_a = a_profile(a_start)
        # oncoming cars
        for i in range(n):
            lead_gap = (x[i] - x[i - 1] - L) if i > 0 else math.inf
            acc = 0.0
            if i > 0:  # IDM toward the car ahead (positions decrease toward the zone)
                s_star = 2.0 + max(0.0, vel[i] * 1.0 + vel[i] * (vel[i] - vel[i - 1]) / (2 * math.sqrt(2.0 * 2.0)))
                acc = 2.0 * (1 - (vel[i] / v) ** 4 - (s_star / max(lead_gap, 0.1)) ** 2)
            else:
                acc = 1.0 * (v - vel[i])
            if i == 0 and c1 == "yield" and a_start is None and 0 < x[0] < COURTESY_RANGE and t >= REACT:
                acc = -YIELD_DECEL
            if i == 0 and c1 == "yield" and a_start is not None and not resume[0]:
                acc = -YIELD_DECEL if s_a(t) < S_OUT else acc
                if s_a(t) >= S_OUT:
                    resume[0] = True
            # safety layer: A in this car's path, after the reaction time
            if a_start is not None and t >= a_start + REACT and S_IN - 1 <= s_a(t) < S_OUT and x[i] > 0:
                need = vel[i] ** 2 / (2 * max(x[i] - 1.0, 0.1))
                if need > 0.5:
                    acc = min(acc, -min(B_MAX, need))
            acc = max(-B_MAX, min(2.0, acc))
            peak[i] = max(peak[i], -acc)
            vel[i] = max(0.0, vel[i] + acc * DT)
            x[i] -= vel[i] * DT
        t += DT
        # the conflict zone: oncoming car i occupies it while 0 >= x > -(W_LANE + L)... along its
        # own path the zone is [0, -(L + w)] with w the crossing width of A's path (~2 m)
        for i in range(n):
            inside = -(L + 2.0) < x[i] <= 0
            if inside and occ[i] is None:
                occ[i] = [t, None]
            if not inside and occ[i] is not None and occ[i][1] is None and x[i] <= -(L + 2.0):
                occ[i][1] = t
        if a_start is not None:
            sa = s_a(t)
            if S_IN <= sa < S_OUT and occ_a is None:
                occ_a = [t, None]
            if sa >= S_OUT and occ_a is not None and occ_a[1] is None:
                occ_a[1] = t
            if occ_a is not None:
                for i in range(n):
                    if occ[i] is not None:
                        a_in, a_out = occ_a[0], occ_a[1] if occ_a[1] is not None else t
                        c_in, c_out = occ[i][0], occ[i][1] if occ[i][1] is not None else t
                        if a_in < c_out and c_in < a_out and (occ_a[1] is None or occ[i][1] is None or True):
                            if min(a_out, c_out) - max(a_in, c_in) > 0:
                                contact = True
                        pet = min(pet, max(c_in - a_out, a_in - c_out))
        if contact or (occ_a is not None and occ_a[1] is not None and all(xx < -(L + 2.0) for xx in x)):
            break
        if occ_a is not None and occ_a[1] is not None and t - occ_a[1] > 3.0 and all(
                xx < -(L + 2.0) or occ_i is None for xx, occ_i in zip(x, occ)):
            break
    # A's delay: from t=0 to leaving the zone, against an empty road
    t_free = A_VMAX / A_ACC + (S_OUT - A_VMAX ** 2 / (2 * A_ACC)) / A_VMAX
    a_delay = (occ_a[1] if occ_a and occ_a[1] is not None else T_MAX) - t_free
    lost = []
    for i in range(n):
        # each oncoming car's progress against free flow over the run
        free = v * t
        dist = v * (3.0 + i * h) - x[i]
        lost.append(max(0.0, (free - dist) / max(free, 1.0)))
    if contact:
        a_delay = T_MAX
        lost = [1.0 if i == 0 else lost[i] for i in range(n)]
    return Out(max(0.0, a_delay), lost, pet, contact, peak)


def utilities(o):
    risk = max(0.0, (PET_REF - o.pet) / PET_REF) if math.isfinite(o.pet) else 0.0
    ua = -o.a_delay / 20.0 - risk - (10.0 if o.contact else 0.0)  # 20 s of delay ~ a lost run
    uc = [-l - (risk if i == 0 else 0.0) - 0.3 * (min(o.peak[i], B_MAX) / B_MAX) ** 2 - (10.0 if o.contact and i == 0 else 0.0)
          for i, l in enumerate(o.lost)]
    return ua, uc


def solve(n, h, v):
    res = {}
    for c1 in ("hold", "yield"):
        for G in GRID_G:
            o = simulate(G, c1, n, h, v)
            ua, uc = utilities(o)
            res[(G, c1)] = (ua, uc[0], sum(uc), o)
    # A's best G against each C1 response; C1's best response to each G
    best_g = {c1: max(GRID_G, key=lambda G: res[(G, c1)][0]) for c1 in ("hold", "yield")}
    eqs = []
    for c1 in ("hold", "yield"):
        G = best_g[c1]
        other = "yield" if c1 == "hold" else "hold"
        if res[(G, c1)][1] >= res[(G, other)][1] - 1e-9:
            eqs.append((G, c1))
    cost = {k: -(r[0] + r[2]) for k, r in res.items()}
    opt = min(cost, key=cost.get)
    return res, eqs, opt, cost


def table(v):
    print(f"# unprotected left against an oncoming queue, v = {v} m/s, A's thresholds {GRID_G[0]}-{GRID_G[-1]} s")
    print("# n  h(s) | equilibria (A's G, C1)           | eq. cost | optimum (G, C1)  opt. cost | A delay at eq. (s) | PET")
    for n in (1, 2, 3, 5):
        for h in (1.5, 2.5, 4.0):
            res, eqs, opt, cost = solve(n, h, v)
            eqs_s = " ".join(f"({G},{c})" for G, c in eqs) or "none (pure)"
            worst = max((cost[e] for e in eqs), default=float("nan"))
            o = res[eqs[0]][3] if eqs else None
            print(f"  {n}  {h:3.1f}  | {eqs_s:32s} | {worst:7.3f}  | ({opt[0]},{opt[1]}) {cost[opt]:9.3f}  | "
                  f"{(o.a_delay if o else float('nan')):6.1f}            | {(o.pet if o else float('nan')):.2f}")


def selftest():
    # an empty-ish road: one far car, A goes at once and nobody is hurt
    o = simulate(4.0, "hold", 1, 2.0, 10.0)
    assert not o.contact
    # a reckless threshold into a dense queue is contact or a near miss, never safe
    o = simulate(1.0, "hold", 3, 1.5, 10.0)
    assert o.contact or o.pet < 1.0, (o.contact, o.pet)
    # a yield by the first car costs its followers
    oy = simulate(6.0, "yield", 4, 1.5, 10.0)
    oh = simulate(6.0, "hold", 4, 1.5, 10.0)
    assert sum(oy.lost[1:]) >= sum(oh.lost[1:]) - 1e-9
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--v", type=float, default=10.0)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    table(a.v)
    return 0


if __name__ == "__main__":
    sys.exit(main())
