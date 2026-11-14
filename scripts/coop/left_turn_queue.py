#!/usr/bin/env python3
"""Roadmap 074 S5 / U1 revisited: the unprotected left turn against an oncoming queue, with S2's fixes.

left_turn_game.py (U1) is the first model of this turn. This one keeps its rules and its game and
adds what the crossing (S2) learnt, so that the arena (left_turn_arena.py) steps the same rules:

  - true geometry: A turns left from rest at its stop line along an arc of radius R onto a side
    road; the conflict zone is where A's turned box overlaps the oncoming car's body band, found
    from the path (A's arc length s_in .. s_out; along the oncoming lane x in [XZ_LO, XZ_HI]);
  - perception: what A sees of each oncoming car, and they of A, is OBS + DECIDE s old and
    predicted to now at the seen speed; the safety layer acts on what was true REACT s ago;
  - the car's command lags (crossing_game.Actuator), "both stopped" as a speed band (WAIT_V);
  - the general utility (measures.py) on oriented boxes: lost progress over H (A against turning
    at once), risk (2D TTC, PET, clearance), comfort (A's lateral net of its route), contact.

  A    gap acceptance with threshold G (s): go once the next oncoming car not yet past the zone, as
       A sees it, reaches the zone G s or more from now (and the cars behind it too, until one
       is G + A's time to clear + 2 s away), as U1
  C1   hold   keep v (its safety layer still brakes once A is seen in its path)
       yield  seeing A waiting and itself within COURTESY_RANGE of the zone, slow at YIELD_DECEL
              to open a gap, until A has cleared the zone, then resume
  C2.. follow (cutin_game.follow_cruise), with the same safety layer

The game is A's G against C1's response, on C1's own payoff (U1's); welfare counts every car.
Classes per context: H (every equilibrium has C1 hold: A waits for a gap), Y (every one has C1
yield), C (both: a coordination game), ! (contact in an equilibrium), M (mixed only).

Usage:
  scripts/coop/left_turn_queue.py [--out <dir>] [--v 10] [--ns 1,2,3,4,5] [--hs 1.5,2.5,4] [--car-lags]
  scripts/coop/left_turn_queue.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from collections import deque
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import cutin_game as C1M  # noqa: E402
import left_turn_game as U1  # noqa: E402
import measures as M  # noqa: E402

DT, HORIZON, T_MAX = X.DT, X.HORIZON, 40.0
L, W_CAR = X.L, X.W_CAR
LANE_W = 3.5
CAP = C1M.TYPES["car"]
R_TURN = 9.0  # m, A's arc
STRAIGHT0 = 2.0  # m of A's centre before the arc starts
A_ACC, A_VMAX = U1.A_ACC, U1.A_VMAX
OBS, DECIDE, REACT = X.OBS, X.DECIDE, U1.REACT
YIELD_DECEL, COURTESY_RANGE = U1.YIELD_DECEL, U1.COURTESY_RANGE
FIRST_T = 3.0  # s: the first oncoming car's time to the zone at t = 0 (U1)
GRID_G = U1.GRID_G
WAIT_V = X.WAIT_V
Y_A, Y_C = -LANE_W / 2, LANE_W / 2  # lane centres: A's (along +x), the oncoming (along -x)


def path(s):
    """A's centre at arc length s from its start: (x, y, yaw, kappa). Model frame: x along A's lane,
    y to its left; the start is A at rest at its stop line."""
    x0 = -L / 2 - STRAIGHT0  # A's centre at s = 0: its front STRAIGHT0 before the arc's start
    if s <= STRAIGHT0:
        return x0 + s, Y_A, 0.0, 0.0
    a = (s - STRAIGHT0) / R_TURN
    if a <= math.pi / 2:
        xc, yc = x0 + STRAIGHT0, Y_A + R_TURN
        return xc + R_TURN * math.sin(a), yc - R_TURN * math.cos(a), a, 1.0 / R_TURN
    d = s - STRAIGHT0 - R_TURN * math.pi / 2
    return x0 + STRAIGHT0 + R_TURN, Y_A + R_TURN + d, math.pi / 2, 0.0


def _zone():
    """A's arc length where its box enters and leaves the oncoming car's body band, and the band's
    x-range A's box covers in between."""
    lo, hi = Y_C - W_CAR / 2, Y_C + W_CAR / 2
    s_in = s_out = None
    xs = []
    s = 0.0
    while s < 60.0:
        x, y, yaw, _ = path(s)
        cs = M.corners(x, y, yaw, L, W_CAR)
        ymin, ymax = min(c[1] for c in cs), max(c[1] for c in cs)
        inside = ymax > lo and ymin < hi
        if inside:
            s_in = s if s_in is None else s_in
            xs += [c[0] for c in cs]
        elif s_in is not None and s_out is None:
            s_out = s
            break
        s += 0.01
    return s_in, s_out, min(xs), max(xs)


S_IN, S_OUT, XZ_LO, XZ_HI = _zone()


def s_free(t):
    t1 = A_VMAX / A_ACC
    return 0.5 * A_ACC * t * t if t < t1 else 0.5 * A_ACC * t1 * t1 + A_VMAX * (t - t1)


class LTRules:
    """A's threshold rule, C1's response, the followers and every car's safety layer, on perceived
    states; the kinematic model and the arena step this object. Oncoming fronts x are along the
    oncoming lane (decreasing toward and through the zone)."""

    def __init__(self, G, c1, n, v):
        self.G, self.c1, self.n, self.v = G, c1, n, v
        self.started = None  # t A started
        self.resumed = False
        self.t_need = X.t_cover(S_OUT, 0.0, A_VMAX, A_ACC)

    def a_step(self, t, s, va, seen_c):
        """A's acceleration. seen_c: [(x, v)] predicted to now."""
        if self.started is None:
            ok = True
            for xs, vs in seen_c:
                if xs + L <= XZ_LO:  # past the zone
                    continue
                tta = (xs - XZ_HI) / max(vs, 0.1) if xs > XZ_HI else 0.0
                if tta < self.G:
                    ok = False
                    break
                if tta > self.G + self.t_need + 2.0:
                    break
            if ok:
                self.started = t
        if self.started is None:
            return -CAP.b_max if va > 0 else 0.0
        return max(-1.0, min(A_ACC, 1.5 * (A_VMAX - va)))

    def c_step(self, t, x, vc, seen_a_safety, seen_a):
        """Every oncoming car's acceleration. seen_a(_safety): A's (s, v) as seen DECIDE / REACT late."""
        sa, _ = seen_a
        acc = []
        for i in range(self.n):
            if i == 0:
                a = max(-1.0, min(CAP.a_max, 1.0 * (self.v - vc[0])))
            else:
                a = C1M.follow_cruise((x[i] - x[i - 1]) - L, vc[i], vc[i - 1], CAP, self.v)
            dist = x[i] - XZ_HI  # to the zone's entry along its lane
            if i == 0 and self.c1 == "yield" and not self.resumed:
                if (self.started is None and 0 < dist < COURTESY_RANGE and t >= REACT) or (self.started is not None and sa < S_OUT):
                    a = -YIELD_DECEL if vc[0] > 0 else 0.0
                if self.started is not None and sa >= S_OUT:
                    self.resumed = True
            ss, _sv = seen_a_safety
            if S_IN - 1.0 <= ss < S_OUT and dist > 0:  # A seen in this car's path
                need = vc[i] ** 2 / (2 * max(dist - 1.0, 0.1))
                if need > 0.5:
                    a = min(a, -min(CAP.b_max, need))
            acc.append(max(-CAP.b_max, min(CAP.a_max, a)))
        return acc


def a_pose(s, va):
    x, y, yaw, k = path(s)
    return (x, y, yaw, va * math.cos(yaw), va * math.sin(yaw), L, W_CAR), k


def c_pose(xf, vc):
    return (xf + L / 2, Y_C, math.pi, -vc, 0.0, L, W_CAR)


@dataclass
class Out:
    u: list  # [A, C1, C2, ...]
    contact: bool
    pet: float  # A against every oncoming car, the smallest
    a_first: bool  # A cleared the zone before C1 entered it
    a_went_t: float | None
    both_stopped: bool
    lost: list
    clear: float


def score(n, v, contact, s_h_a, s_h_c, peaks, lat_a, meas):
    lost = [1.0 if contact else max(0.0, min(1.0, 1.0 - (s_h_a or 0.0) / s_free(HORIZON)))]
    lost += [1.0 if contact else M.lost_progress(v, s_h_c[i]) for i in range(n)]
    rhos = [M.rho(m.ttc2d, m.thw, m._pet(), m.clear) for m in meas]
    u = [M.utility(lost[0], max(rhos), peaks[0], CAP.b_max, lat_a, contact)]
    u += [M.utility(lost[i + 1], rhos[i], peaks[i + 1], CAP.b_max, 0.0, contact) for i in range(n)]
    return u, lost, min(m._pet() for m in meas)


def simulate(G, c1, n, h, v, lags=None):
    R = LTRules(G, c1, n, v)
    x = [XZ_HI + v * (FIRST_T + i * h) for i in range(n)]
    vc = [v] * n
    s = va = 0.0
    act = [X.Actuator(lags) if lags else None for _ in range(n + 1)]
    hist = deque()
    meas = [M.Measures(b_max=(CAP.b_max, CAP.b_max), pet_sample=X.PET_SAMPLE) for _ in range(n)]
    vh = [deque() for _ in range(n + 1)]
    peaks = [0.0] * (n + 1)
    sc = [0.0] * n
    s_h_a, s_h_c = None, [None] * n
    contact = both = False
    st_both = 0.0
    c1_in = None
    a_out = None
    t = 0.0
    while t < T_MAX - 1e-9:
        hist.append((t, s, va, list(x), list(vc)))
        while len(hist) > 2 and hist[1][0] <= t - (REACT + 1.0):
            hist.popleft()
        sd = next((hh for hh in reversed(hist) if hh[0] <= t - OBS - DECIDE), hist[0])
        sr = next((hh for hh in reversed(hist) if hh[0] <= t - REACT), hist[0])
        age, age_r = t - sd[0], t - sr[0]
        seen_c = [(sd[3][i] - sd[4][i] * age, sd[4][i]) for i in range(n)]
        aa = R.a_step(t, s, va, seen_c)
        ac = R.c_step(t, x, vc, (sr[1] + sr[2] * age_r, sr[2]), (sd[1] + sd[2] * age, sd[2]))
        if lags:
            aa = act[0].step(aa)
            ac = [act[i + 1].step(a) for i, a in enumerate(ac)]
        va = max(0.0, va + aa * DT)
        s += va * DT
        for i in range(n):
            vc[i] = max(0.0, vc[i] + ac[i] * DT)
            x[i] -= vc[i] * DT
            sc[i] += vc[i] * DT
        t += DT
        if s_h_a is None and t >= HORIZON - 1e-9:
            s_h_a, s_h_c = s, list(sc)
        for k, vv in enumerate([va] + vc):
            hq = vh[k]
            hq.append((t, vv))
            while t - hq[0][0] > X.PEAK_W + 1e-9:
                hq.popleft()
            if t - hq[0][0] >= X.PEAK_W - 1e-9:
                peaks[k] = max(peaks[k], (hq[0][1] - vv) / X.PEAK_W)
        pa, kap = a_pose(s, va)
        for i in range(n):
            meas[i].push_poses(t, pa, c_pose(x[i], vc[i]), kappa=(kap, 0.0))
        if any(m.clear <= 0.0 for m in meas):
            contact = True
            break
        if c1_in is None and x[0] < XZ_HI:
            c1_in = t
        if a_out is None and s >= S_OUT:
            a_out = t
        if va < WAIT_V and vc[0] < WAIT_V and s < S_IN and x[0] > XZ_HI:
            st_both += DT
            both = both or st_both >= 0.5
        else:
            st_both = 0.0
        if t >= HORIZON - 1e-9 and s >= S_OUT + 10 and all(xx + L < XZ_LO for xx in x):
            break
    if s_h_a is None:
        s_h_a, s_h_c = s, list(sc)
    u, lost, pet = score(n, v, contact, s_h_a, s_h_c, peaks, meas[0].lat_peak[0], meas)
    a_first = a_out is not None and (c1_in is None or a_out <= c1_in)
    return Out(u, contact, pet, a_first, R.started, both, lost, min(m.clear for m in meas))


def solve(outs, grid):
    ja = {(G, c): outs[(G, c)].u[0] for G in grid for c in ("hold", "yield")}
    jc = {(G, c): outs[(G, c)].u[1] for G in grid for c in ("hold", "yield")}
    pure, mixed = X.solve(ja, jc, list(grid), ["hold", "yield"])
    return pure, mixed


def klass(pure, mixed, contact):
    if any(contact(p) for p in pure):
        return "!"
    cs = {p[1] for p in pure}
    if cs == {"hold"}:
        return "H"
    if cs == {"yield"}:
        return "Y"
    if cs:
        return "C"
    return "M" if mixed else "?"


def u1_class(n, h, v):
    """U1's own equilibria (left_turn_game.solve) as a class, for the three-tier comparison."""
    _, eqs, _, _ = U1.solve(n, h, v)
    cs = {c for _, c in eqs}
    return ("H" if cs == {"hold"} else "Y" if cs == {"yield"} else "C" if cs else "?"), " ".join(f"{G}/{c}" for G, c in eqs)


def context(job):
    n, h, v, lags, grid = job
    outs = {(G, c): simulate(G, c, n, h, v, lags) for G in grid for c in ("hold", "yield")}
    pure, mixed = solve(outs, grid)
    cost = {k: -sum(o.u) for k, o in outs.items()}
    eqc = [cost[p] for p in pure]
    uc, ueq = u1_class(n, h, v)
    return {"n": n, "h": h, "v": v, "class": klass(pure, mixed, lambda p: outs[p].contact), "class_U1": uc,
            "eq": " ".join(f"{G}/{c}" for G, c in pure), "eq_U1": ueq, "n_mixed": len(mixed),
            "C_opt": min(cost.values()), "C_worst_eq": max(eqc) if eqc else None, "C_best_eq": min(eqc) if eqc else None,
            "pet_eq_min": min((outs[p].pet for p in pure), default=math.inf),
            "eq_pet_below_ref": sum(outs[p].pet < M.PET_REF for p in pure),
            "pairs_contact": sum(o.contact for o in outs.values()), "pairs": len(outs)}


def run_map(args):
    from multiprocessing import Pool

    lags = X.car_lags() if args.car_lags else None
    ns = [int(x) for x in args.ns.split(",")]
    hs = [float(x) for x in args.hs.split(",")]
    jobs = [(n, h, args.v, lags, GRID_G) for n in ns for h in hs]
    with Pool(args.jobs) as pool:
        rows = pool.map(context, jobs)
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "left_turn_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.4g}" if isinstance(v, float) and math.isfinite(v) else ("inf" if isinstance(v, float) else v)) for k, v in r.items()})
    lines = [f"# unprotected left (S2's fixes; zone s {S_IN:.2f}-{S_OUT:.2f} m, x {XZ_LO:.2f}-{XZ_HI:.2f} m), v {args.v} m/s, "
             + ("the car's command lags" if lags else "an ideal car"),
             "# class: H every equilibrium C1 holds (A waits for its gap), Y every one C1 yields, C both, ! contact, M mixed only",
             " n    h | class  U1 | equilibria                         | U1's equilibria       | C opt / worst / best | PET eq min"]
    for r in rows:
        f = lambda x: "-" if x is None else f"{x:.3f}"  # noqa: E731
        lines.append(f" {r['n']} {r['h']:4.1f} |   {r['class']}    {r['class_U1']}  | {r['eq'][:34]:34} | {r['eq_U1'][:21]:21} | "
                     f"{f(r['C_opt'])} / {f(r['C_worst_eq'])} / {f(r['C_best_eq'])} | {r['pet_eq_min']:.2f}")
    lines.append(f"# classes agree with U1 in {sum(r['class'] == r['class_U1'] for r in rows)} of {len(rows)}")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "left_turn_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    return 0


def selftest():
    assert 0 < S_IN < S_OUT and XZ_LO < XZ_HI, (S_IN, S_OUT, XZ_LO, XZ_HI)
    # the path ends heading +y, past the oncoming lane
    x, y, yaw, _ = path(STRAIGHT0 + R_TURN * math.pi / 2 + 5)
    assert abs(yaw - math.pi / 2) < 1e-9 and y > LANE_W + 2
    # one far car: A goes, nobody touches
    o = simulate(4.0, "hold", 1, 2.0, 10.0)
    assert not o.contact and o.a_went_t is not None, o
    # a reckless threshold into a dense queue: contact or a near miss
    o = simulate(1.0, "hold", 3, 1.5, 10.0)
    assert o.contact or o.pet < 1.0, o
    # the first car's yield costs its followers
    oy, oh = simulate(6.0, "yield", 4, 1.5, 10.0), simulate(6.0, "hold", 4, 1.5, 10.0)
    assert sum(oy.lost[2:]) >= sum(oh.lost[2:]) - 1e-9, (oy.lost, oh.lost)
    print("selftest ok", f"zone s {S_IN:.2f}-{S_OUT:.2f}, x {XZ_LO:.2f}-{XZ_HI:.2f}")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="left_turn_out")
    p.add_argument("--v", type=float, default=10.0)
    p.add_argument("--ns", default="1,2,3,5")
    p.add_argument("--hs", default="1.5,2.5,4")
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--jobs", type=int, default=12)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
