#!/usr/bin/env python3
"""Roadmap 074 U2: the roundabout entry, in a kinematic model with S2's and S5's fixes.

A one-lane ring of radius R_RING, circulated counter-clockwise (right-hand traffic) at v. A waits at
rest at its yield line on the entry from the south and joins the ring along an arc of radius R_ENTRY
that meets the ring tangentially at PHI_JOIN. B is the next circulating car upstream, with up to two
more behind it at time headway h. Geometry is true (oriented boxes): A's yield line is 1 m before its
box first reaches the circulating band; the conflict stretch on the ring is the part of the band A's
box covers while it merges; A has joined once it is in the lane. Each car sees the others OBS +
DECIDE s late, predicted to now (the safety layer REACT s late); the car's command lags.

  A  gap acceptance with threshold G (s) on the next circulating car's time to the conflict stretch
     (U1's test: that car and those behind it until one is G + A's time to join + 2 s away),
     patience P (after P s at rest A halves its G), creep (A edges forward to the band's edge, 0.3 m
     short of it, and starts from there): A_RULES = G x P x creep
  B  hold   keep v (its safety layer brakes, REACT s after A is seen in the stretch ahead of it)
     slow   courtesy: seeing A waiting (or creeping) within COURTESY m upstream of the stretch, slow at
            2 m/s^2 to open a gap until A has joined
     speed  assert: seeing A waiting within the same range, speed up (to v + 3) to close the gap
  followers follow (cutin_game.follow_cruise) along the ring, A too once it has joined and is ahead

Payoffs: the general utility (measures.py) on oriented boxes, lateral comfort net of each car's route
curvature (push_poses' kappa: the ring's 1/R and the entry arc's), A's progress against joining at
once from rest, B's against v. The game is A's rules against B's (B's own payoff); welfare counts
every car. Classes: A (A joins ahead of B in every equilibrium), B (behind), C (chicken: both), !
(contact), M (mixed only). The pair game (n = 1) at the same B arrival is checked as a composition of
the queue's (N3's question: does the pair predict the queue?).

Usage:
  scripts/coop/roundabout_game.py --out <dir> [--v 7] [--ns 1,2,3] [--hs 1.5,3] [--tbs 0.5..8] [--car-lags]
  scripts/coop/roundabout_game.py --selftest
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
import measures as M  # noqa: E402

DT, HORIZON, T_MAX = X.DT, X.HORIZON, 40.0
L, W_CAR = X.L, X.W_CAR
CAP = C1M.TYPES["car"]
R_RING, R_ENTRY = 18.0, 12.0
PHI_JOIN = math.radians(-60.0)  # where A's entry arc meets the ring (the ring's centre at the origin)
APPROACH = 40.0  # m of straight approach before the entry arc
A_ACC = 2.0
OBS, DECIDE, REACT = X.OBS, X.DECIDE, 1.0
COURTESY, SLOW_DECEL, SPEED_ACC, SPEED_DV = 30.0, 2.0, 2.0, 3.0
GS = (1.5, 2.5, 3.5, 5.0, 8.0, math.inf)  # inf: wait until the whole queue has passed
PATIENCE = (math.inf, 5.0)
A_RULES = [(f"g{G:g}{'p' if P < math.inf else ''}{'c' if c else ''}", G, P, c) for G in GS for P in PATIENCE for c in (False, True)]
B_RULES = ("hold", "slow", "speed")
WAIT_V = X.WAIT_V


# --- geometry ------------------------------------------------------------------------------------
def ring(s):
    """A ring car's centre at arc length s (s = R phi, phi from +x, counter-clockwise): (x, y, yaw, kappa)."""
    phi = s / R_RING
    return R_RING * math.cos(phi), R_RING * math.sin(phi), phi + math.pi / 2, 1.0 / R_RING


H_JOIN = PHI_JOIN + math.pi / 2  # the ring's heading at the join
PJ = (R_RING * math.cos(PHI_JOIN), R_RING * math.sin(PHI_JOIN))
CE = (PJ[0] + R_ENTRY * math.sin(H_JOIN), PJ[1] - R_ENTRY * math.cos(H_JOIN))  # the entry arc's centre (A turns right)
X_APP = CE[0] - R_ENTRY  # the approach line (heading +y)
ARC_LEN = R_ENTRY * (math.pi / 2 - H_JOIN)  # from heading +y clockwise to the ring's heading at the join
S_JOIN_RING = R_RING * PHI_JOIN  # the ring's arc length at the join


def a_path(s):
    """A's centre at arc length s from its approach's start: (x, y, yaw, kappa)."""
    if s <= APPROACH:
        return X_APP, CE[1] - APPROACH + s, math.pi / 2, 0.0
    if s <= APPROACH + ARC_LEN:
        b = (s - APPROACH) / R_ENTRY  # clockwise from the approach's tangent point
        return CE[0] - R_ENTRY * math.cos(b), CE[1] + R_ENTRY * math.sin(b), math.pi / 2 - b, -1.0 / R_ENTRY
    return ring(S_JOIN_RING + (s - APPROACH - ARC_LEN))


def a_ring_s(s):
    """A's centre's arc length on the ring (its angle times R, near the join), once it has joined (else None)."""
    if s < A_JOINED:
        return None
    x, y, _, _ = a_path(s)
    phi = math.atan2(y, x)
    while phi < PHI_JOIN - math.pi:
        phi += 2 * math.pi
    return R_RING * phi


def _geometry():
    band_lo, band_hi = R_RING - W_CAR / 2, R_RING + W_CAR / 2
    a_in = a_joined = None
    phis = []
    s = 0.0
    while s < APPROACH + ARC_LEN + 20.0:
        x, y, yaw, _ = a_path(s)
        cs = M.corners(x, y, yaw, L, W_CAR)
        rs = [math.hypot(cx, cy) for cx, cy in cs]
        in_band = min(rs) < band_hi and max(rs) > band_lo
        if in_band and a_in is None:
            a_in = s
        if a_in is not None and a_joined is None:
            phis += [math.atan2(cy, cx) for cx, cy in cs]
            if abs(math.hypot(x, y) - R_RING) < 0.3 and abs(math.atan2(math.sin(yaw - (math.atan2(y, x) + math.pi / 2)),
                                                                     math.cos(yaw - (math.atan2(y, x) + math.pi / 2)))) < 0.05:
                a_joined = s
        s += 0.01
    sz0, sz1 = R_RING * min(phis), R_RING * max(phis)
    return a_in, a_joined, sz0, sz1


A_IN, A_JOINED, SZ0, SZ1 = _geometry()
S_YIELD = A_IN - 1.0  # A's centre's arc length at its yield line: its box 1 m (along its path) short of the band
S_CREEP = A_IN - 0.3


@dataclass
class Out:
    u: list  # [A, B, C, ...]
    contact: bool
    a_ahead: bool  # A joined ahead of B
    a_went: float | None
    both_stopped: bool
    pet: float
    clear: float
    lost: list
    forced: bool = False  # A joined ahead of B and forced it: B's safety layer, or B braking it did not choose


class RBRules:
    """A's rule, B's response, the followers and every car's safety layer, on perceived states; the
    kinematic model and the arena step this object. Ring positions are fronts' arc lengths."""

    def __init__(self, ra, rb, n, v):
        _, self.G, self.P, self.creep = ra
        self.rb, self.n, self.v = rb, n, v
        self.started = None
        self.stopped_since = None
        self.joined_ahead = None
        self.shield = [False] * n  # each ring car's safety layer braked harder than its rule
        self.t_need = X.t_cover(A_JOINED - S_YIELD, 0.0, v, A_ACC)

    def a_step(self, t, s, va, seen_q):
        """seen_q: [(ring s of the front, v)], predicted to now."""
        if self.started is None:
            G = self.G
            if self.stopped_since is not None and t - self.stopped_since >= self.P:
                G = self.G / 2
            ok = True
            for sq, vq in seen_q:
                if sq - L > SZ1:  # past the stretch
                    continue
                tta = (SZ0 - sq) / max(vq, 0.1) if sq < SZ0 else 0.0
                if tta < G:
                    ok = False
                    break
                if tta > G + self.t_need + 2.0:
                    break
            if ok:
                self.started = t
        if self.started is None:
            d = (S_CREEP if self.creep else S_YIELD) - s
            if d > 0.05:
                return max(-CAP.b_max, min(1.0, 0.8 * d - 1.2 * va))  # creep (or settle) to the line, slowly
            return -CAP.b_max if va > 0 else 0.0
        # going: toward v, following a ring car ahead once joined
        a = max(-1.0, min(A_ACC, 1.5 * (self.v - va)))
        sr = a_ring_s(s)
        if sr is not None:
            ahead = [(sq, vq) for sq, vq in seen_q if sq - L > sr + L / 2]
            if ahead:
                sq, vq = min(ahead)
                a = min(a, C1M.follow_cruise(sq - L - (sr + L / 2), va, vq, CAP, self.v))
        return a

    def after(self, t, va):
        if self.started is None and va < 0.1:
            self.stopped_since = self.stopped_since if self.stopped_since is not None else t
        elif va >= 0.1:
            self.stopped_since = None

    def q_step(self, t, sq, vq, seen_a_react, seen_a):
        """Every ring car's acceleration. seen_a(_react): A's (s, v) as seen DECIDE / REACT late."""
        sa, _ = seen_a
        sar, _ = seen_a_react
        a_front_ring = a_ring_s(sar)
        acc = []
        for i in range(self.n):
            a = max(-1.0, min(CAP.a_max, 1.5 * (self.v - vq[i])))
            # the car ahead on the ring: the previous queue car, or A once joined and ahead of this one
            if i > 0:
                a = min(a, C1M.follow_cruise(sq[i - 1] - L - sq[i], vq[i], vq[i - 1], CAP, self.v))
            if a_front_ring is not None:
                af = a_front_ring + L / 2  # A's front on the ring
                if af > sq[i] and (i == 0 or af < sq[i - 1]):
                    a = min(a, C1M.follow_cruise(af - L - sq[i], vq[i], seen_a[1], CAP, self.v))
            dist = SZ0 - sq[i]
            waiting = self.started is None or sa < A_JOINED
            if i == 0 and self.rb != "hold" and waiting and 0 < dist < COURTESY and t >= REACT and sa >= S_YIELD - 0.5:
                if self.rb == "slow":
                    a = min(a, -SLOW_DECEL if vq[i] > 0 else 0.0)
                else:
                    a = max(a, min(SPEED_ACC, 1.5 * (self.v + SPEED_DV - vq[i])))
            # safety layer: A seen in the stretch, and this car would reach the stretch before A has joined
            # (plus 1 s), as S2's layer conditions on the arrival times, not on any A in the stretch
            t_rem = X.t_cover(A_JOINED - sar, seen_a_react[1], self.v, A_ACC)
            if A_IN - 0.5 <= sar < A_JOINED and dist > 0 and dist / max(vq[i], 0.1) < t_rem + 1.0:
                need = vq[i] ** 2 / (2 * max(dist - 1.0, 0.1))
                if need > 0.5:
                    planned = a
                    a = min(a, -min(CAP.b_max, need))
                    self.shield[i] = self.shield[i] or (vq[i] > 0.5 and planned - a > 1.0)
            acc.append(max(-CAP.b_max, min(CAP.a_max, a)))
        return acc


def a_pose(s, va):
    x, y, yaw, k = a_path(s)
    return (x, y, yaw, va * math.cos(yaw), va * math.sin(yaw), L, W_CAR), k


def q_pose(sf, vq):
    x, y, yaw, k = ring(sf - L / 2)
    return (x, y, yaw, vq * math.cos(yaw), vq * math.sin(yaw), L, W_CAR), k


def score(n, v, contact, merged, sa_h, sq_h, peaks, lat, meas):
    free = 0.5 * v * v / A_ACC + v * (HORIZON - v / A_ACC)  # A from rest to v at A_ACC, then v: joining at once
    lost = [1.0 if contact or not merged else max(0.0, min(1.0, 1.0 - sa_h / free))]
    lost += [1.0 if contact else M.lost_progress(v, sq_h[i]) for i in range(n)]
    rhos = [M.rho(m.ttc2d, m.thw, m._pet(), m.clear) for m in meas]
    u = [M.utility(lost[0], max(rhos), peaks[0], CAP.b_max, lat[0], contact)]
    u += [M.utility(lost[i + 1], rhos[i], peaks[i + 1], CAP.b_max, lat[i + 1], contact) for i in range(n)]
    return u, lost


def simulate(ra, rb, n, h, v, tb, lags=None):
    """tb: B's time to the conflict stretch at t = 0 (at v)."""
    R = RBRules(ra, rb, n, v)
    sq = [SZ0 - v * (tb + i * h) for i in range(n)]
    vq = [v] * n
    s, va = S_YIELD, 0.0
    act = [X.Actuator(lags) if lags else None for _ in range(n + 1)]
    hist = deque()
    meas = [M.Measures(b_max=(CAP.b_max, CAP.b_max), pet_sample=X.PET_SAMPLE) for _ in range(n)]
    vh = [deque() for _ in range(n + 1)]
    peaks = [0.0] * (n + 1)
    sa0 = s
    sq0 = list(sq)
    sa_h = sq_h = None
    contact = both = False
    st_both = 0.0
    t = 0.0
    while t < T_MAX - 1e-9:
        hist.append((t, s, va, list(sq), list(vq)))
        while len(hist) > 2 and hist[1][0] <= t - (REACT + 1.0):
            hist.popleft()
        sd = next((hh for hh in reversed(hist) if hh[0] <= t - OBS - DECIDE), hist[0])
        sr = next((hh for hh in reversed(hist) if hh[0] <= t - REACT), hist[0])
        age, ager = t - sd[0], t - sr[0]
        seen_q = [(sd[3][i] + sd[4][i] * age, sd[4][i]) for i in range(n)]
        aa = R.a_step(t, s, va, seen_q)
        aq = R.q_step(t, sq, vq, (sr[1] + sr[2] * ager, sr[2]), (sd[1] + sd[2] * age, sd[2]))
        if lags:
            aa = act[0].step(aa)
            aq = [act[i + 1].step(a) for i, a in enumerate(aq)]
        va = max(0.0, va + aa * DT)
        s += va * DT
        R.after(t, va)
        for i in range(n):
            vq[i] = max(0.0, vq[i] + aq[i] * DT)
            sq[i] += vq[i] * DT
        t += DT
        if sa_h is None and t >= HORIZON - 1e-9:
            sa_h, sq_h = s - sa0, [sq[i] - sq0[i] for i in range(n)]
        for k, vv in enumerate([va] + vq):
            hq = vh[k]
            hq.append((t, vv))
            while t - hq[0][0] > X.PEAK_W + 1e-9:
                hq.popleft()
            if t - hq[0][0] >= X.PEAK_W - 1e-9:
                peaks[k] = max(peaks[k], (hq[0][1] - vv) / X.PEAK_W)
        pa, ka = a_pose(s, va)
        for i in range(n):
            pq, kq = q_pose(sq[i], vq[i])
            meas[i].push_poses(t, pa, pq, kappa=(ka, kq))
        if any(m.clear <= 0.0 for m in meas):
            contact = True
            break
        if R.joined_ahead is None and s >= A_JOINED:
            R.joined_ahead = a_ring_s(s) + L / 2 > sq[0]
        if va < WAIT_V and vq[0] < WAIT_V and s < A_IN and sq[0] < SZ0:
            st_both += DT
            both = both or st_both >= 0.5
        else:
            st_both = 0.0
        if t >= HORIZON - 1e-9 and s >= A_JOINED + 5:
            break
    if sa_h is None:
        sa_h, sq_h = s - sa0, [sq[i] - sq0[i] for i in range(n)]
    lat = [meas[0].lat_peak[0]] + [m.lat_peak[1] for m in meas]
    merged = s >= A_JOINED
    u, lost = score(n, v, contact, merged, sa_h, sq_h, peaks, lat, meas)
    pet = min(m._pet() for m in meas)
    a_ahead = bool(R.joined_ahead) and not contact
    forced = a_ahead and (R.shield[0] or (peaks[1] > X.YIELD_SEEN and rb != "slow"))
    return Out(u, contact, a_ahead, R.started, both, pet, min(m.clear for m in meas), lost, forced)


def u_a(o, r, fault):
    """A's payoff under right of way r ('B': the circulating car has it, A pays `fault` for forcing it). A
    always starts at rest at its yield line, so S2's excuse for a car that rested at its line does not apply."""
    return o.u[0] - (fault if r == "B" and o.forced else 0.0)


def klass(pure, mixed, outs):
    if any(outs[p].contact for p in pure):
        return "!"
    aheads = {outs[p].a_ahead for p in pure}
    if aheads == {True}:
        return "A"
    if aheads == {False}:
        return "B"
    if aheads:
        return "C"
    return "M" if mixed else "?"


FAULT = X.FAULT
FAULTS = X.FAULTS


def context(job):
    n, h, v, tb, lags, fault = job
    outs = {(ra[0], rb): simulate(ra, rb, n, h, v, tb, lags) for ra in A_RULES for rb in B_RULES}
    a_acts = [r[0] for r in A_RULES]
    rows = {}
    for sub, b_acts, r in (("all", list(B_RULES), "none"), ("no_courtesy", ["hold", "speed"], "none"),
                           ("row_B", list(B_RULES), "B"), ("row_B_no_courtesy", ["hold", "speed"], "B")):
        ja = {(a, b): u_a(outs[(a, b)], r, fault) for a in a_acts for b in b_acts}
        jb = {(a, b): outs[(a, b)].u[1] for a in a_acts for b in b_acts}
        pure, mixed = X.solve(ja, jb, a_acts, b_acts)
        cost = {k: -sum(outs[k].u) for k in ja}
        eqc = [cost[p] for p in pure]
        f_min = None
        if r == "B":
            for f in FAULTS:
                jf = {(a, b): u_a(outs[(a, b)], r, f) for a in a_acts for b in b_acts}
                pf, _ = X.solve(jf, jb, a_acts, b_acts)
                if pf and not any(outs[p].forced or outs[p].contact for p in pf):
                    f_min = f
                    break
        rows[sub] = {"n": n, "h": h, "v": v, "tb": tb, "subset": sub, "class": klass(pure, mixed, outs), "fault_min": f_min,
                     "eq_forced": sum(outs[p].forced for p in pure), "n_pure": len(pure),
                     "n_mixed": len(mixed), "C_opt": min(cost.values()), "C_worst_eq": max(eqc) if eqc else None,
                     "C_best_eq": min(eqc) if eqc else None, "eq_b_slow": sum(p[1] == "slow" for p in pure),
                     "eq_contact": sum(outs[p].contact for p in pure), "eq_pet_min": min((outs[p].pet for p in pure), default=math.inf),
                     "pairs_contact": sum(o.contact for o in outs.values()),
                     "pure": " ".join(f"{a}/{b}" for a, b in pure)}
    return list(rows.values())


def run_map(args):
    from multiprocessing import Pool

    lags = X.car_lags() if args.car_lags else None
    ns = [int(x) for x in args.ns.split(",")]
    hs = [float(x) for x in args.hs.split(",")]
    lo, hi = (float(x) for x in args.tbs.split(".."))
    tbs = [round(lo + i * args.tb_step, 3) for i in range(int(round((hi - lo) / args.tb_step)) + 1)]
    jobs = [(n, h, args.v, tb, lags, args.fault) for n in ns for h in (hs if n > 1 else hs[:1]) for tb in tbs]
    with Pool(args.jobs) as pool:
        rows = [r for rs in pool.map(context, jobs) for r in rs]
    os.makedirs(args.out, exist_ok=True)

    def fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return "inf" if math.isinf(x) else f"{x:.4g}"
        return x

    with open(os.path.join(args.out, "roundabout_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(v) for k, v in r.items()})
    by = {(r["subset"], r["n"], r["h"], r["tb"]): r for r in rows}
    lines = [f"# roundabout entry (kinematic): ring R {R_RING:g} m at {args.v:g} m/s, entry arc R {R_ENTRY:g} m; A's yield line "
             f"{A_IN - S_YIELD:.1f} m short of the band; the stretch on the ring {SZ1 - SZ0:.1f} m; "
             + ("the car's command lags" if lags else "an ideal car"),
             "# class per tb (B's time to the stretch at t = 0): A joins ahead of B in every equilibrium, B behind, C both (chicken),",
             "# ! contact, M mixed only; * an equilibrium where A forces B; no_courtesy: B without 'slow'; row_B: B has the right of",
             f"# way (A pays {args.fault:g} for forcing it); f_min: the smallest fault under which no equilibrium has A forcing B",
             "tb:                    " + " ".join(f"{t:>4g}" for t in tbs)]
    for n in ns:
        for h in (hs if n > 1 else hs[:1]):
            for sub in ("all", "no_courtesy", "row_B", "row_B_no_courtesy"):
                lines.append(f"n {n} h {h:3g} {sub:17}" + " ".join(
                    f"{by[(sub, n, h, t)]['class'] + ('*' if by[(sub, n, h, t)]['eq_forced'] else ''):>4}" for t in tbs))
            lines.append(f"n {n} h {h:3g} {'f_min (row_B)':17}" + " ".join(
                f"{'-' if by[('row_B', n, h, t)]['fault_min'] is None else format(by[('row_B', n, h, t)]['fault_min'], 'g'):>4}" for t in tbs))
    # composition: the pair game (n = 1) predicts the queue's class at the same tb
    pairs = [(by[("all", n, h, t)], by[("all", 1, hs[0], t)]) for n in ns if n > 1 for h in hs for t in tbs]
    agree = sum(q["class"] == p["class"] for q, p in pairs)
    lines.append("")
    lines.append(f"# composition: the pair (n = 1) predicts the queue's class in {agree} of {len(pairs)} contexts (n > 1)")
    for sub in ("all", "no_courtesy", "row_B", "row_B_no_courtesy"):
        rs = [r for r in rows if r["subset"] == sub]
        cls = [r["class"] for r in rs]
        lines.append(f"# {sub:17}: classes " + ", ".join(f"{c} {cls.count(c)}" for c in "ABC!M?" if cls.count(c)) +
                     f" of {len(rs)}; equilibria with B slowing {sum(r['eq_b_slow'] for r in rs)}, A forcing B "
                     f"{sum(r['eq_forced'] for r in rs)}, contact "
                     f"{sum(r['eq_contact'] for r in rs)}; mean worst / best equilibrium cost "
                     f"{sum(r['C_worst_eq'] or 0 for r in rs) / len(rs):.3f} / {sum(r['C_best_eq'] or 0 for r in rs) / len(rs):.3f}")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "roundabout_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    return 0


def selftest():
    assert A_IN is not None and A_JOINED is not None and A_IN < A_JOINED and SZ0 < SZ1, (A_IN, A_JOINED, SZ0, SZ1)
    # the path is continuous at the arc's ends
    for s0 in (APPROACH, APPROACH + ARC_LEN):
        p, q = a_path(s0 - 1e-6), a_path(s0 + 1e-6)
        assert math.hypot(p[0] - q[0], p[1] - q[1]) < 1e-3 and abs(p[2] - q[2]) < 1e-3, (s0, p, q)
    a = dict((r[0], r) for r in A_RULES)
    # B far: A joins ahead, nobody touches
    o = simulate(a["g2.5"], "hold", 1, 2.0, 7.0, 8.0)
    assert o.a_ahead and not o.contact, o
    # B near, A reckless, B holds: contact or a near miss
    o = simulate(a["g1.5"], "hold", 1, 2.0, 7.0, 1.6)
    assert o.contact or o.pet < 1.5 or o.clear < 0.5, o
    # B near, A careful: A joins behind B
    o = simulate(a["g5"], "hold", 1, 2.0, 7.0, 2.0)
    assert not o.a_ahead and not o.contact, o
    print("selftest ok", f"yield {A_IN - S_YIELD:.1f} m short; A in band at {A_IN:.2f}, joined {A_JOINED:.2f}; stretch {SZ0:.2f}..{SZ1:.2f}")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="roundabout_out")
    p.add_argument("--v", type=float, default=7.0)
    p.add_argument("--ns", default="1,2,3")
    p.add_argument("--hs", default="1.5,3")
    p.add_argument("--tbs", default="0.5..8")
    p.add_argument("--tb-step", type=float, default=0.5)
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--fault", type=float, default=FAULT, help="A's cost for forcing B under row_B")
    p.add_argument("--jobs", type=int, default=16)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
