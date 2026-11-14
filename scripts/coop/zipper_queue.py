#!/usr/bin/env python3
"""Roadmap 074 S3: the lane drop with a queue, and the conventions that decide who goes at the end.

merge_game's lane drop with a queue in the through lane: B leads, C (and D) follow at time headway
hf. A must merge into the queue before its lane ends: ahead of B, into a slot between two queue
cars, or behind the last. Everything one car sees of another is OBS + DECIDE s old, predicted to now.

  A    merge_game's gap acceptance (tau, early or zipper timing, go-on-seen-yield, drop or force at
       the deadline), generalized to slots: the slot at A's position lies between the lead car (the
       first queue car whose front is ahead of A's) and the lag car (the next one). A goes iff the lag
       gap passes the cut-in's check (REST_GAP behind a lag car at rest) or the lag car is seen to
       yield, the lead car is S0 ahead, and A can be half across before the end. In its window it
       falls in behind the lead car (at most FALL_IN). Refused, an early polite A falls back at once
       (EARLY_DROP), a late one at the deadline; a forcing A ignores the lag car, never the lead.
       Merging from rest it creeps (half across before the end).
  queue every queue car follows the car ahead in its lane (cutin_game.follow_cruise; A once A is half
       across and ahead), and yields to A by its rule when A is ahead of its front and still in its
       own lane:
         hold   never (the safety layer still follows A once A is in the lane)
         c20    courtesy: A within 20 m ahead and the lane ending within 10 s (2 m/s^2)
         c40    courtesy: 40 m, 3 m/s^2
         zip    the zipper: only near the end (A's lane change plus ZIP_T s at A's speed), the car A is
                beside or just ahead of yields
         yend   yield-at-the-end: within the last YEND_X m, any car with A ahead in its slot yields

Conventions, each a rule for A and one for the whole queue:
  none     the game: A's 16 rules against the queue's {hold, c20, c40} (one rule for all its cars,
           the queue's payoff the sum of its cars'): its worst and best equilibrium
  zipper   A z0.6yd (late, goes at the end on a seen yield, else drops back) and the queue zip
  zipper*  the queue zip and A's best reply to it
  early    A e1.2d (early, polite) and the queue hold
  yend     the queue yend and A's best reply to it

Welfare is every car's cost, C = -sum U; the part of the gap a convention recovers is
(C_worst - C_conv) / (C_worst - C_opt), C_opt the best profile in the game's rule space. Throughput:
when the last car (A or the queue's last) passes THRU_X m beyond the lane's end.

Usage:
  scripts/coop/zipper_queue.py --out <dir> [--vb 12] [--n 3] [--car-lags] [--jobs N]
  scripts/coop/zipper_queue.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys
from collections import deque
from dataclasses import dataclass, field
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402
import merge_game as G  # noqa: E402

DT, HORIZON, T_MAX = G.DT, G.HORIZON, 60.0
CAP, L, W, LANE_W = G.CAP, G.L, G.W, G.LANE_W
ZIP_T, YEND_X, THRU_X = 2.0, 60.0, 50.0  # zip: the last (T_LC + ZIP_T) s at A's speed
FALL_IN = 2.0  # m/s^2: A falls in behind its lead car at most this hard
EARLY_DROP, EARLY_DV = 1.5, 3.0  # m/s^2, m/s: an early merger refused falls back this gently, this much slower
REST_GAP = 1.0  # m: A's gap acceptance ahead of a lag car at rest
YIELD_ROOM = 1.0  # m: a yielding car leaves A this much more than S0 (A's gap acceptance at rest needs S0)
GAP_MIN = 0.05  # the welfare gap below which no share is computed (the conventions' cost difference is reported)
Q_RULES = {"hold": None, "c20": (20.0, 2.0, "courtesy"), "c40": (40.0, 3.0, "courtesy"),
           "zip": (L + 6.0, 2.5, "zip"), "yend": (60.0, 2.5, "yend")}
GAME_Q = ("hold", "c20", "c40")
CONVENTIONS = {"zipper": ("z0.6yd", "zip"), "zipper*": (None, "zip"), "early": ("e1.2d", "hold"), "yend": (None, "yend")}


class QueueRules:
    """A's rule and the queue's, step by step, on perceived states: the kinematic model and the arena
    (zipper_arena.py) step the same object. Cars are A and Q[0..n-1] (front to back); positions are
    fronts along the road, x_end the lane's end."""

    def __init__(self, ra, qrule, x_end, va0, vq0, n):
        _, self.timing, self.tau, self.go_y, self.end_mode = ra
        self.q = Q_RULES[qrule]
        self.x_end, self.va0, self.vq0, self.n = x_end, va0, vq0, n
        self.decide = None
        self.dropping = self.forced = self.stopped = self.force_wait = self.early_drop = False
        self.latched = [False] * n
        self.released = [False] * n

    def lat_of(self, t):
        if self.decide is None:
            return 0.0, 0.0
        u = min(1.0, (t - self.decide) / G.T_LC)
        return 0.5 * (1 - math.cos(math.pi * u)), (LANE_W * 0.5 * math.pi / G.T_LC * math.sin(math.pi * u) if u < 1 else 0.0)

    def slot(self, xa, xq):
        """(lead index or None, lag index or None) of the slot at A's position: by fronts, as the queue
        decides who yields (a car whose front is behind A's is behind A's slot)."""
        lag = next((i for i in range(self.n) if xq[i] < xa), None)
        lead = (lag - 1 if lag is not None else self.n - 1) if (lag is None or lag > 0) else None
        return lead, lag

    def step(self, t, xa, va, lat, xq, vq, seen_q, seen_a):
        """Accelerations (A, [queue]) at t. seen_q: [(x, v, a)] of the queue as A sees it; seen_a:
        (x, v, lat) of A as the queue sees it; all predicted to now."""
        xs = [s[0] for s in seen_q]
        rem = self.x_end - xa
        lead, lag = self.slot(xa, xs)
        gap_lag = (xa - L) - xs[lag] if lag is not None else math.inf
        gap_lead = (xs[lead] - L) - xa if lead is not None else math.inf
        v_lag = seen_q[lag][1] if lag is not None else 0.0
        a_lag = seen_q[lag][2] if lag is not None else 0.0
        if self.decide is None:
            window = self.timing == "early" or rem <= G.ZIP_T * max(va, 1.0) + G.END_MARGIN
            need = G.BUFFER + v_lag * self.tau + max(0.0, v_lag ** 2 - va ** 2) / (2 * G.A_REAR)
            if v_lag < 0.5:
                need = REST_GAP  # the lag car waits at rest: a creeping lane change needs little room
            lead_ok = gap_lead >= G.S0 + 0.5 * va
            fits = rem >= va * G.T_LC / 2 + 0.5  # half across before the end at this speed (A rests 1 m short)
            if fits and (window or (lag is None and gap_lead >= G.S0 + va * G.HEADWAY)) and lead_ok and not self.dropping:
                if gap_lag >= need or (self.go_y and a_lag <= -G.YIELD_SEEN and gap_lag > -L / 2):
                    self.decide = t
            if self.decide is None and self.timing == "early" and self.end_mode == "drop" and lag is not None:
                self.early_drop = True  # refused early: fall back a slot at a time, well before the end
            if self.decide is None and rem <= G.lane_change_dist(va):
                if self.end_mode == "force" and not self.dropping:
                    self.force_wait = True  # it ignores the lag car, never the lead
                    if lead_ok:
                        self.decide, self.forced = t, True
                else:
                    self.dropping = True
            if self.decide is None and (self.dropping or self.early_drop) and lead_ok and gap_lag >= need and fits:
                self.decide = t  # a slot has opened beside A as it dropped back (or at rest)
        # --- A's longitudinal
        if self.decide is not None and lead is not None and gap_lead < 60:
            aa = C1.follow_cruise(gap_lead, va, seen_q[lead][1], CAP, self.va0)
        elif self.early_drop and not self.dropping and self.decide is None:
            aa = max(-EARLY_DROP, min(1.0, 1.5 * (v_lag - EARLY_DV - va)))
        elif (self.dropping or self.force_wait) and self.decide is None:
            if lag is not None and v_lag < 0.5 and va < 2.0:
                aa = 0.5  # the lag car waits for A: creep forward (the lane's end still stops A) to open the gap
            else:
                aa = -C1.A_WAIT_DECEL if va > 0.5 * max(v_lag, 1.0) or gap_lag < 0 else 0.0
        else:
            aa = max(-1.0, min(CAP.a_max, 1.5 * (self.va0 - va)))
        if self.decide is None and lead is not None and gap_lead < G.S0 + va * G.HEADWAY and (
                self.timing == "early" or rem <= G.ZIP_T * max(va, 1.0) + G.END_MARGIN):
            aa = min(aa, max(-FALL_IN, C1.follow_cruise(gap_lead, va, seen_q[lead][1], CAP, self.va0)))  # fall in behind the lead
        if (self.decide is None or lat < 0.5) and rem > 0:
            d = rem - (1.0 if self.decide is None else 0.0)
            need_e = va * va / (2 * max(d, 0.05))
            if d <= 0.05:
                aa = -CAP.b_max if va > 0 else 0.0
            elif need_e >= G.BRAKE_LATE:
                aa = min(aa, -min(CAP.b_max, need_e))
        if self.decide is not None and lat < 0.5:  # half across before the lane ends, even from rest
            v_cap = max(0.0, rem - 1.0) / max(self.decide + G.T_LC / 2 - t, 0.1)
            aa = min(aa, 2.0 * (v_cap - va))
        aa = max(-CAP.b_max, min(CAP.a_max, aa))
        # --- the queue
        xa_s, va_s, lat_s = seen_a
        aq = []
        for i in range(self.n):
            x, v = xq[i], vq[i]
            # the car ahead in the lane: the previous queue car, or A once half across and ahead
            leaders = []
            if i > 0:
                leaders.append(((xq[i - 1] - L) - x, vq[i - 1]))
            if lat_s >= 0.5 and xa_s > x and (i == 0 or xa_s < xq[i - 1]):
                leaders.append(((xa_s - L) - x, va_s))
            a = max(-1.0, min(CAP.a_max, 1.5 * (self.vq0 - v)))
            for g, vl in leaders:
                a = min(a, C1.follow_cruise(g, v, vl, CAP, self.vq0))
            # yielding to A: A ahead of this car's front, still in its own lane, in this car's slot
            a_in_slot = xa_s - L > x - L and (i == 0 or xa_s < xq[i - 1]) and lat_s < 0.5
            if self.q is not None and a_in_slot:
                band, b_y, kind = self.q
                gap = (xa_s - L) - x
                rem_s = self.x_end - xa_s
                if kind == "courtesy":
                    trig = t >= G.REACT and gap < band and rem_s <= G.COURTESY_T * max(va_s, 1.0)
                elif kind == "zip":
                    trig = rem_s <= G.lane_change_dist(va_s) + ZIP_T * max(va_s, 1.0) and gap < band
                else:
                    trig = rem_s <= YEND_X
                if self.latched[i] and (gap < -L / 2 or (gap < REST_GAP and v < 0.1 and va_s < 0.1)):
                    self.released[i] = True  # A has dropped back past this car's front: it no longer waits for A
                self.latched[i] = (self.latched[i] or trig) and not self.released[i]
                if self.latched[i]:
                    g = gap - YIELD_ROOM
                    a = min(a, -b_y if g < G.S0 + v * G.HEADWAY else C1.follow_cruise(g, v, va_s, CAP, self.vq0))
            if lat_s >= 0.5 and xa_s > x and (i == 0 or xa_s < xq[i - 1]):  # the safety layer
                a = min(a, C1.follow_accel((xa_s - L) - x, v, va_s, CAP, self.vq0))
            aq.append(max(-CAP.b_max, min(CAP.a_max, a)))
        return aa, aq

    def after(self, xa, va):
        if va < 0.1 and xa < self.x_end and self.decide is None:
            self.stopped = True


@dataclass
class Out:
    u: list  # [A, Q0, Q1, ...]
    contact: bool
    stopped: bool
    forced: bool
    merged: bool
    slot: int  # A's place in the merged queue: 0 ahead of B, k behind the k-th queue car
    t_thru: float  # the last car past THRU_X beyond the lane's end
    lost: list = field(default_factory=list)
    why: str = ""


def utilities(contact, merged, v0s, dist, meas, qmeas, peak):
    """Every car's utility, [A, Q0, ...]: lost progress over H (A's is 1 if it never merged), risk the
    worst over the pairs it is in (A with each queue car, each queue car with the one ahead), comfort
    from its peak braking, contact. dist: each car's distance over H; meas[i]: A with Q_i; qmeas[i]:
    Q_i with Q_{i-1}."""
    n = len(v0s) - 1
    lost = [1.0 if contact or (k == 0 and not merged) else M.lost_progress(v0s[k], dist[k]) for k in range(n + 1)]
    rho = {(0, i + 1): M.rho(m.ttc2d, m.thw, math.inf, m.clear) for i, m in meas.items()}
    rho.update({(i, i + 1): M.rho(m.ttc2d, m.thw, math.inf, m.clear) for i, m in qmeas.items()})
    u = [M.utility(lost[k], max([v for (a, b), v in rho.items() if k in (a, b)] + [0.0]), peak[k], CAP.b_max, 0.0, contact)
         for k in range(n + 1)]
    return u, lost


def simulate(ra, qrule, d_end, d0, dv, vb, hf, n, lags=None):
    va0 = max(0.0, vb + dv)
    xa = d0 + L
    xq = [-(i * (L + G.S0 + vb * hf)) for i in range(n)]
    x_end = xa + d_end
    va, vq = va0, [vb] * n
    R = QueueRules(ra, qrule, x_end, va0, vb, n)
    act = [X.Actuator(lags) if lags else None for _ in range(n + 1)]
    hist = deque()
    aa, aq = 0.0, [0.0] * n
    lat = 0.0
    meas = {i: M.Measures(b_max=(CAP.b_max, CAP.b_max)) for i in range(n)}  # A with each queue car
    qmeas = {i: M.Measures(b_max=(CAP.b_max, CAP.b_max)) for i in range(1, n)}  # queue car with the one ahead
    vh = [deque() for _ in range(n + 1)]
    peak = [0.0] * (n + 1)
    contact = ran_out = False
    why = ""
    t = 0.0
    t_thru = math.inf
    s = [0.0] * (n + 1)
    s_h = [None] * (n + 1)
    while t < T_MAX - 1e-9:
        hist.append((t, xa, va, aa, lat, list(xq), list(vq), list(aq)))
        while len(hist) > 2 and hist[1][0] <= t - (G.OBS + G.DECIDE + 1.0):
            hist.popleft()
        sd = next((h for h in reversed(hist) if h[0] <= t - G.OBS - G.DECIDE), hist[0])
        age = t - sd[0]
        seen_q = [(sd[5][i] + sd[6][i] * age, sd[6][i], sd[7][i]) for i in range(n)]
        aa, aq = R.step(t, xa, va, lat, xq, vq, seen_q, (sd[1] + sd[2] * age, sd[2], sd[4]))
        if lags:
            aa = act[0].step(aa)
            aq = [act[i + 1].step(a) for i, a in enumerate(aq)]
        va = max(0.0, va + aa * DT)
        xa += va * DT
        for i in range(n):
            vq[i] = max(0.0, vq[i] + aq[i] * DT)
            xq[i] += vq[i] * DT
        t += DT
        for k, vv in enumerate([va] + vq):
            s[k] += vv * DT
            if s_h[k] is None and t >= HORIZON - 1e-9:
                s_h[k] = s[k]
            h = vh[k]
            h.append((t, vv))
            while t - h[0][0] > G.PEAK_W + 1e-9:
                h.popleft()
            if t - h[0][0] >= G.PEAK_W - 1e-9:
                peak[k] = max(peak[k], (h[0][1] - vv) / G.PEAK_W)
        lat, vy = R.lat_of(t)
        R.after(xa, va)
        if xa > x_end and lat < 0.5:
            ran_out = True
        vxa = math.sqrt(max(0.0, va * va - vy * vy))
        pa = (xa - L / 2, LANE_W * (1 - lat), math.atan2(-vy, max(vxa, 0.1)), vxa, -vy, L, W)
        pq = [(xq[i] - L / 2, 0.0, 0.0, vq[i], 0.0, L, W) for i in range(n)]
        for i in range(n):
            meas[i].push_poses(t, pa, pq[i])
        for i in range(1, n):
            qmeas[i].push_poses(t, pq[i], pq[i - 1])
        hit = [f"A-Q{i}" for i, m in meas.items() if m.clear <= 0.0] + [f"Q{i}-Q{i - 1}" for i, m in qmeas.items() if m.clear <= 0.0]
        if ran_out or hit:
            contact = True
            why = f"t {t:.2f} " + ("ran out" if ran_out else ",".join(hit)) + f" xa {xa - x_end:+.1f} va {va:.1f} lat {lat:.2f} xq {[round(x - x_end, 1) for x in xq]} vq {[round(v, 1) for v in vq]}"
            break
        if math.isinf(t_thru) and lat >= 1.0 and min([xa] + xq) >= x_end + THRU_X:
            t_thru = t
        if t >= HORIZON - 1e-9 and lat >= 1.0 and not math.isinf(t_thru):
            break
    merged = lat >= 1.0
    u, lost = utilities(contact, merged, [va0] + [vb] * n, [s_h[k] if s_h[k] is not None else s[k] for k in range(n + 1)],
                        meas, qmeas, peak)
    slot = sum(1 for x in xq if x > xa) if merged else -1
    return Out(u, contact, R.stopped, R.forced, merged, slot, t_thru, lost, why)


def a_rule(name):
    return next(r for r in G.A_RULES if r[0] == name)


def context(job):
    d_end, d0, dv, vb, hf, n, lags = job
    outs = {(ra[0], q): simulate(ra, q, d_end, d0, dv, vb, hf, n, lags) for ra in G.A_RULES for q in Q_RULES}
    a_acts = [r[0] for r in G.A_RULES]
    ja = {(a, q): outs[(a, q)].u[0] for a in a_acts for q in GAME_Q}
    jq = {(a, q): sum(outs[(a, q)].u[1:]) for a in a_acts for q in GAME_Q}
    pure, mixed = X.solve(ja, jq, a_acts, list(GAME_Q))
    cost = {k: -sum(o.u) for k, o in outs.items()}
    c_game = {k: cost[k] for k in ja}
    c_opt = min(cost.values())  # the best of every rule pair, the conventions' too
    eqc = [c_game[p] for p in pure] + [sum(pa[a] * pq[q] * c_game[(a, q)] for a in pa for q in pq) for pa, pq in mixed]
    c_worst, c_best = (max(eqc), min(eqc)) if eqc else (None, None)
    row = {"d_end": d_end, "d0": d0, "dv": dv, "vb": vb, "hf": hf, "n": n,
           "n_pure": len(pure), "n_mixed": len(mixed), "C_opt": c_opt, "C_worst_eq": c_worst, "C_best_eq": c_best,
           "eq_stopped": sum(outs[p].stopped for p in pure), "eq_contact": sum(outs[p].contact for p in pure),
           "t_thru_worst_eq": max((outs[p].t_thru for p in pure), default=None),
           "t_thru_best_eq": min((outs[p].t_thru for p in pure), default=None)}
    for conv, (a, q) in CONVENTIONS.items():
        if a is None:  # A's best reply to the queue's rule
            a = max(a_acts, key=lambda x: outs[(x, q)].u[0])
        o = outs[(a, q)]
        c = cost[(a, q)]
        row[f"{conv}_A"] = a
        row[f"C_{conv}"] = c
        row[f"{conv}_dC"] = None if c_worst is None else c_worst - c
        row[f"{conv}_recovers"] = None if c_worst is None or c_worst - c_opt < GAP_MIN else (c_worst - c) / (c_worst - c_opt)
        row[f"{conv}_stopped"], row[f"{conv}_contact"] = int(o.stopped), int(o.contact)
        row[f"{conv}_slot"], row[f"{conv}_t_thru"] = o.slot, o.t_thru
        # is A's prescribed rule its best reply to the queue's rule (self-enforcing)?
        row[f"{conv}_A_best_reply"] = int(o.u[0] >= max(outs[(x, q)].u[0] for x in a_acts) - 1e-9)
    return row


def run_map(args):
    lags = X.car_lags() if args.car_lags else None
    d_ends = [float(x) for x in args.d_ends.split(",")]
    d0s = [float(x) for x in args.d0s.split(",")]
    dvs = [float(x) for x in args.dvs.split(",")]
    hfs = [float(x) for x in args.hfs.split(",")]
    jobs = [(de, d0, dv, args.vb, hf, args.n, lags) for de, hf, dv, d0 in itertools.product(d_ends, hfs, dvs, d0s)]
    with Pool(args.jobs) as pool:
        rows = pool.map(context, jobs)
    os.makedirs(args.out, exist_ok=True)

    def fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return "inf" if math.isinf(x) else f"{x:.3f}"
        return x

    with open(os.path.join(args.out, "zipper_queue.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(v) for k, v in r.items()})
    lines = [f"# lane drop with a queue of {args.n} (kinematic): vB = {args.vb} m/s, " +
             ("the car's command lags" if lags else "an ideal car"),
             "# per context: welfare cost C (all cars) of the game's optimum, worst and best equilibrium, and of each",
             "# convention, with the part of the gap (C_worst - C_opt) it recovers; forced stops (S) and contact (!)",
             "# under it; throughput: the last car's time past the lane's end + 50 m (s)"]
    lines.append(f"{'end':>4} {'hf':>4} {'dv':>4} {'d0':>6} | {'C_opt':>6} {'worst':>6} {'best':>6} | "
                 + " | ".join(f"{c:>7} rec  thru" for c in CONVENTIONS) + " | thru eq worst/best")
    agg = {c: [] for c in CONVENTIONS}
    for r in rows:
        cells = []
        for c in CONVENTIONS:
            rec = r[f"{c}_recovers"]
            if rec is not None:
                agg[c].append(rec)
            flag = "!" if r[f"{c}_contact"] else "S" if r[f"{c}_stopped"] else " "
            dc = r[f"{c}_dC"]
            tag = f"{rec:+5.2f}" if rec is not None else ("" if dc is None else f"~{-dc:+.2f}")
            cells.append(f"{r[f'C_{c}']:7.3f}{flag}{tag:>6} {r[f'{c}_t_thru']:5.1f}")
        cw = "" if r["C_worst_eq"] is None else f"{r['C_worst_eq']:6.3f}"
        cb = "" if r["C_best_eq"] is None else f"{r['C_best_eq']:6.3f}"
        tw = r["t_thru_worst_eq"]
        tb = r["t_thru_best_eq"]
        lines.append(f"{r['d_end']:4g} {r['hf']:4g} {r['dv']:+4g} {r['d0']:+6g} | {r['C_opt']:6.3f} {cw:>6} {cb:>6} | "
                     + " | ".join(cells) + f" | {'' if tw is None else f'{tw:.1f}'}/{'' if tb is None else f'{tb:.1f}'}")
    lines.append("\n# mean part of the gap recovered (contexts with a gap): " +
                 ", ".join(f"{c} {sum(v) / len(v):+.2f} (n {len(v)})" for c, v in agg.items() if v))
    lines.append("# self-enforcing (A's prescribed rule is its best reply to the queue's), share of contexts: " +
                 ", ".join(f"{c} {sum(r[f'{c}_A_best_reply'] for r in rows) / len(rows):.2f}" for c in CONVENTIONS))
    fin = lambda xs: [x for x in xs if x is not None and not math.isinf(x)]  # noqa: E731
    mean = lambda xs: sum(xs) / len(xs) if xs else math.nan  # noqa: E731
    lines.append("# per convention over all contexts: mean cost C, forced stops, contact, A's slot (0 = ahead of B), and")
    lines.append("#   throughput: mean of the last car's time where it finished, the contexts where it never did")
    for c in CONVENTIONS:
        ts = [r[f"{c}_t_thru"] for r in rows]
        slots = [r[f"{c}_slot"] for r in rows]
        lines.append(f"#   {c:8} C {mean([r[f'C_{c}'] for r in rows]):.3f}  S {sum(r[f'{c}_stopped'] for r in rows):2d}  "
                     f"! {sum(r[f'{c}_contact'] for r in rows):2d}  slots " +
                     " ".join(f"{k}:{slots.count(k)}" for k in sorted(set(slots))) +
                     f"  thru {mean(fin(ts)):.2f} s, never {len(ts) - len(fin(ts))}")
    for k in ("worst", "best"):
        ts = [r[f"t_thru_{k}_eq"] for r in rows]
        lines.append(f"#   {k + ' eq':8} C {mean(fin([r[f'C_{k}_eq'] for r in rows])):.3f}"
                     f"  thru {mean(fin(ts)):.2f} s, never {len(ts) - len(fin(ts))}")
    lines.append(f"#   {'optimum':8} C {mean([r['C_opt'] for r in rows]):.3f}")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "zipper_queue.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    return 0


def selftest():
    # the zipper: A late beside the queue's second car yields a slot; no contact
    o = simulate(a_rule("z0.6yd"), "zip", 80.0, -12.0, 0.0, 12.0, 1.0, 3)
    assert o.merged and not o.contact, o
    # an early, polite A against a holding queue with room ahead of B merges ahead of B
    o = simulate(a_rule("e1.2d"), "hold", 160.0, 25.0, 2.0, 12.0, 1.0, 3)
    assert o.merged and o.slot == 0 and not o.contact, o
    # dropping behind a tight queue at a short lane end: no contact (a stop or a slot behind)
    o = simulate(a_rule("e1.2d"), "hold", 40.0, -3.0, 0.0, 12.0, 0.8, 3)
    assert not o.contact, o
    r = context((80.0, -5.0, 0.0, 12.0, 1.0, 3, None))
    assert r["n_pure"] or r["n_mixed"], r
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="zipper_out")
    p.add_argument("--vb", type=float, default=12.0)
    p.add_argument("--n", type=int, default=3, help="queue cars (B, C, D)")
    p.add_argument("--d-ends", default="40,80")
    p.add_argument("--d0s", default="-20,-12,-5,2.5,10")
    p.add_argument("--dvs", default="-2,0")
    p.add_argument("--hfs", default="0.8,1.5")
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
