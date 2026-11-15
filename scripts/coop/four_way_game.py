#!/usr/bin/env python3
"""Roadmap 074 U3: the four-way stop as a passage game in a kinematic model (Town05 j979, D8).

Two cars on perpendicular approaches through one conflict box Z, crossing_game.py's geometry: A
along +x, B along +y (B comes from A's right; right-hand traffic). The context is

  dt   t_A - t_B, each car's time to its stop line at its own speed (s); dt > 0: B arrives first
  v    both cars' speed at t = 0 (m/s)

Rules (each car's; the same set for both):

  aggr        aggressive: no stop, go (the safety layer only)
  yield_c<C>  stock Autoware (the intersection module, not_prioritized): no stop when clear; go only
              when the other's passage misses its own by the margins -- the other leaves Z
              M_BEFORE (4) s before this car enters, or enters M_AFTER (3) s after it leaves --,
              judged every step from the other as seen (a car at rest is predicted to stay);
              otherwise hold out and brake for the line
  stop<D>y_c<C>  Autoware on the stop-sign map: stop at the line for D s, then yield_c<C>'s test
  fcfs_c<C>   first come first served: stop STOP_DWELL s; go when no other car came to rest at its
              line earlier (as seen) and has not cleared Z; a tie (rests within TIE s) is a go
  fcfsr_c<C>  fcfs, a tie broken right-before-left (the car on the right goes; B is A's right)
  rbl_c<C>    right-before-left: stop; yield to a car on the right that is at or near its line
              (within RBL_NEAR s), regardless of arrival
  fcfsm_c<C>  074 C8 (c), fcfs made fail-safe (c8_fourway_check.py verifies it): the other's rest stamp
              (less FM_BIAS, the stamp's measured lag) minus its own rest reads first (> TIE + FM_M),
              second (< -(TIE + FM_M)), tie (|.| < TIE - FM_M) or ambiguous; no stamp yet reads first
              only once a stamp within TIE + FM_M would be known, or when the other is seen far; first
              goes, a tie goes for B (on the right), everything else waits -- a disagreement makes both
              wait -- and a car seen moving from its line has the turn; second round, FM_H2 s after its
              dwell: B goes if A is seen at rest; a zone-interval guard: no start while a car that is
              not braking for its line would enter Z (at its speed) before this car could leave it,
              plus FM_ZONE_MG
  fcfse_c<C>  074 C8 (e): fcfsm with the map's fixed order (A, then B) as the last resort: A goes FM_H3 s
              after its dwell if B is still seen at rest (against a B that waits for A and never goes)

C, the commit time: a car that has started from its line keeps re-checking its rule for C s and
stops again when the rule says so (C = 0: committed at once; inf: never committed). Every rule's
safety layer is crossing_game's (REACT s late: the other in Z, or about to enter first).

Payoffs: crossing_game.utility (the general utility, no right of way: at an all-way stop none has
it, the conventions are the rules); progress is lost against free flow at v, so every stop costs.
Classes per (dt, v): A / B (every equilibrium lets the same car through first), C (chicken:
equilibria with each first), W (an equilibrium where both stop and wait past T_MAX: deadlock),
! (contact in an equilibrium), M (mixed only). Rule subsets ("conventions") are solved as
games of their own to ask which convention removes the ties: all, autoware (yield and stop1y
only), and each convention against itself (fcfs vs fcfs, ...).

Usage:
  scripts/coop/four_way_game.py --out <dir> [--v 6,10] [--dt-max 4] [--dt-step 0.5] [--car-lags] [--jobs N]
  scripts/coop/four_way_game.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import measures as M  # noqa: E402

DT, HORIZON, T_MAX, INF = X.DT, X.HORIZON, X.T_MAX, X.INF
L, W_CAR, W_BOX, EXIT, STOP = X.L, X.W_CAR, X.W_BOX, X.EXIT, X.STOP
A_MAX, B_MAX = X.A_MAX, X.B_MAX
OBS, DECIDE, REACT = X.OBS, X.DECIDE, X.REACT
M_BEFORE, M_AFTER = 4.0, 3.0  # s, Autoware's not_prioritized collision margins (the peer's D8 note)
STOP_DWELL = 1.0  # s at rest at the line: the stop-sign module's stop, and the conventions' full stop
TIE = 0.5  # s: two rests this close are a tie
RBL_NEAR = 2.0  # s: a car on the right this close to its line (or at rest at it) is yielded to
B_PLAN = 2.0  # m/s^2: a planned stop at the line starts when it needs this much
COMMITS = (0.0, 1.5)
_KINDS = ("yield", "stop1y", "fcfs", "fcfsr", "rbl")
RULES = [("aggr", "aggr", 0.0)] + [(f"{k}_c{c:g}", k, c) for k in _KINDS for c in COMMITS]
NAMES = [r[0] for r in RULES]  # U3's rule set (its subsets, maps and payoff files)
# 074 C8: the fail-safe FCFS (c) and its fixed-order last resort (e), beside U3's rules (--c8 adds them to
# the games). Its margin is sized from the rest stamp's error e (c8_fourway_check.py: safe and live for
# FM_M >= e); FM_BIAS is the stamp's mean lag, 0 on the kinematic model's true perception and about 0.2 s
# through arena.Observer (c8_fourway_check.py measure-e), set by four_way_arena.py.
FM_M = 0.3  # s
FM_E = 0.25  # s, the error bound the margin is designed for (it sets FM_H3)
FM_BIAS = 0.0  # s
FM_H2 = 2.0  # s
FM_ZONE_MG = 0.5  # s
FM_SEE = 0.7 + 0.8  # s, a start until it is seen (visible motion, then the perception and decision delay)
C8_KINDS = ("fcfsm", "fcfse")
C8_RULES = [(f"{k}_c{c:g}", k, c) for k in C8_KINDS for c in COMMITS]
RULES = RULES + C8_RULES
C8_NAMES = [r[0] for r in C8_RULES]


def fm_h3():
    """(e)'s last resort: after B's second-round go would have been seen, whatever B read."""
    return FM_H2 + TIE + FM_M + FM_E + FM_SEE + 0.2
SUBSETS = {
    "all": NAMES,
    "autoware": [n for n in NAMES if n.startswith(("yield", "stop1y"))],
    "stop_yield": [n for n in NAMES if n.startswith("stop1y")],
    "fcfs": [n for n in NAMES if n.startswith("fcfs_")],
    "fcfsr": [n for n in NAMES if n.startswith("fcfsr")],
    "rbl": [n for n in NAMES if n.startswith("rbl")],
}


C8_SUBSETS = {
    "fcfsm": [n for n in C8_NAMES if n.startswith("fcfsm")],
    "fcfse": [n for n in C8_NAMES if n.startswith("fcfse")],
    "all_c8": NAMES + C8_NAMES,
}


def set_fm(m=None, bias=None):
    global FM_M, FM_BIAS
    if m is not None:
        FM_M = m
    if bias is not None:
        FM_BIAS = bias


def plan_stop(u, v, v0):
    """Brake to rest at the stop line, starting when the stop needs B_PLAN; until then keep v0."""
    d = STOP - u
    if d <= 0.05:
        return -B_MAX if v > 0 else 0.0
    need = v * v / (2 * d)
    return -min(B_MAX, need) if need >= B_PLAN else X.drive(v, v0)


class FWPolicy:
    """One car's stop-sign rule and crossing_game's safety layer, on what it perceives of the other;
    the kinematic model and the arena step this object (perceive / step / after, as X.Policy)."""

    def __init__(self, role, rule, v0, hold_stop=None, exit_=None, other_exit=None):
        """exit_ / other_exit: this car's / the other's front distance past Z's near edge when its rear has
        left Z (default EXIT, U3's box and car; a skewed junction or a longer vehicle widens them)."""
        self.role, self.v0 = role, v0
        self.name, self.kind, self.commit = rule
        self.hold_stop = X.HOLD_STOP if hold_stop is None else hold_stop
        self.exit = EXIT if exit_ is None else exit_
        self.other_exit = self.exit if other_exit is None else other_exit
        self.other_rest_u = None  # where the other was when its rest was read (fcfsm: has it moved off?)
        self.exit_clear = True  # fcfsm: never enter Z unless this car's exit lane beyond it is clear (a runner
        #                       with traffic beyond Z sets it; nothing queues beyond Z in the arena or this model)
        self.reading = ""  # fcfsm's last reading of the other (first / second / tie / amb / wait / turn)
        self.hist = []
        self.stopped_since = None
        self.rested = self.shield = False
        self.rest_t = None  # when this car first came to rest at its line
        self.moving_since = None  # when it started from the line (or kept going)
        self.go = self.kind == "aggr"
        self.other_rest_t = None  # when the other was first seen at rest at its line (observation stamp)
        self.other_cleared = False
        self.restarts = 0
        self._oa, self._raw = 0.0, (0.0, 0.0)
        self.clip_n = self.clip_moving = 0
        self.clip_vmax = self.clip_dist = 0.0

    def perceive(self, stamp, ou, ov, oa):
        self.hist.append((stamp, ou, ov, oa))

    def _seen(self, t, age, first=True):
        for h in reversed(self.hist):
            if h[0] <= t - age:
                return h
        return self.hist[0] if first and self.hist else None

    def _clear(self, t, u, v, ou, ov):
        """Autoware's mutual-yield test: the other's passage misses this car's by the margins."""
        if ou >= self.other_exit:
            return True
        t_in_e = X.t_cover(-u, v, self.v0) if u < 0 else 0.0
        t_out_e = X.t_cover(self.exit - u, v, self.v0)
        t_in_o = X.t_in(ou, ov)
        t_out_o = X.t_out(ou, ov, self.other_exit)
        return t_out_o + M_BEFORE <= t_in_e or t_in_o >= t_out_e + M_AFTER

    def _want_go(self, t, u, v, ou, ov):
        k = self.kind
        if k == "aggr":
            return True
        if k == "yield":
            return self._clear(t, u, v, ou, ov)
        at_rest_long = self.rest_t is not None and t - self.rest_t >= (1.0 if k == "stop1y" else STOP_DWELL)
        if not at_rest_long and u < 0:
            return False
        if k == "stop1y":
            return self._clear(t, u, v, ou, ov)
        if self.other_cleared or ou >= self.other_exit:
            return True
        if k in C8_KINDS:
            return self._fcfsm(t, u, v, ou, ov)
        if k in ("fcfs", "fcfsr"):
            if self.other_rest_t is None:
                return True  # nobody waiting before me
            mine = self.rest_t if self.rest_t is not None else t
            if self.other_rest_t < mine - TIE:
                return False  # the other came first
            if self.other_rest_t > mine + TIE:
                return True
            return k == "fcfs" or self.role == "B"  # a tie: fcfs goes; fcfsr: the right (B) goes
        # rbl: B has nobody on its right; A yields to B at or near its line
        if self.role == "B":
            return True
        near = (ov < 0.1 and STOP - 2.0 <= ou < 0) or (ou < 0 and X.t_in(ou, ov) <= RBL_NEAR)
        return not near

    def _stopping(self, ou, ov, oa):
        """074 C8: the other is braking for its line: the deceleration it shows is enough to stop there (at
        most X.BRAKE_LATE + 1, a comfortable stop) -- the zone contract takes it as a car that will stop. A car
        whose safety layer only slows it is not stopping for its line."""
        need = ov * ov / (2 * max(STOP - ou + 2.0, 0.1))
        return oa < -0.5 and need <= -oa + 1.0 and need <= X.BRAKE_LATE + 1.0

    def _fcfsm(self, t, u, v, ou, ov):
        """074 C8 (c) / (e) at the line, after the dwell (see the module's docstring)."""
        oa = self._oa
        # moved off its line: speed and distance together, or distance alone (one noisy sample is not a start)
        off = ou - self.other_rest_u if self.other_rest_u is not None else 0.0
        moved = self.other_rest_t is not None and ((ov > 0.5 and off > 0.3) or off > 1.0)
        if 0 <= ou < self.other_exit or moved:
            self.reading = "turn"
            return False
        if not self.exit_clear:
            self.reading = "exit"
            return False
        # the zone guard: a car not braking for its line, predicted (at its speed) into Z before I leave it
        if self.other_rest_t is None and ou < 0 and ov > 0.5:
            braking = self._stopping(*self._raw, oa)
            my_out = X.LAUNCH_DEAD + X.t_cover(self.exit - u, v, self.v0)
            if not braking and -ou / ov <= my_out + FM_ZONE_MG:
                self.reading = "zone"
                return False
        lr = OBS + DECIDE
        if self.other_rest_t is None:
            far = ou < -60.0 or (ov > 0.5 and -ou / ov > 2.0 and oa > -0.5)
            first = far or t - lr - FM_BIAS - self.rest_t >= TIE + FM_M
            self.reading = "first" if first else "wait"
        else:
            d = (self.other_rest_t - FM_BIAS) - self.rest_t
            self.reading = ("first" if d > TIE + FM_M else "second" if d < -(TIE + FM_M) else
                            "tie" if abs(d) < TIE - FM_M else "amb")
        if self.reading == "first" or (self.reading == "tie" and self.role == "B"):
            return True
        if self.reading == "wait":
            return False
        quiet = t - (self.rest_t + STOP_DWELL)
        at_rest = self.other_rest_t is not None and ou < 0
        if self.role == "B" and quiet >= FM_H2 and at_rest:
            return True
        if self.kind == "fcfse" and self.role == "A" and quiet >= fm_h3() and at_rest:
            return True
        return False

    def step(self, t, u, v):
        sd = self._seen(t, OBS + DECIDE)
        if sd is not None:
            ou, ov = sd[1] + sd[2] * (t - sd[0]), sd[2]
            self._oa, self._raw = sd[3], (sd[1], sd[2])  # fcfsm: the observation itself, not extrapolated
            if self.other_rest_t is None and sd[2] < 0.1 and STOP - 2.0 <= sd[1] < 0:
                self.other_rest_t = sd[0]
                self.other_rest_u = sd[1]
            if sd[1] >= self.other_exit:
                self.other_cleared = True
            committed = self.go and self.moving_since is not None and t - self.moving_since >= self.commit
            if not committed and (u < 0 or self.kind == "aggr"):
                want = self._want_go(t, u, v, ou, ov)
                # past the point a comfortable stop at the line is possible, a moving car is committed
                if self.go and not want and u < STOP and v * v / (2 * max(STOP - u, 0.05)) > X.BRAKE_LATE + 1.0:
                    want = True
                if self.go and not want:
                    self.restarts += 1
                self.go = want
        if self.go:
            a = X.drive(v, self.v0)
        elif self.kind == "yield":
            a = X.hold_out(u, v)
        else:
            a = plan_stop(u, v, self.v0)
        sr = self._seen(t, REACT, first=False)
        if sr is None or u >= 0:
            return a
        ou, ov = sr[1] + sr[2] * (t - sr[0]), sr[2]
        in_box = 0 <= ou < self.other_exit
        closing = (ou < 0 and ov > 0.5 and X.t_in(ou, ov) < 2.0 and X.t_in(ou, ov) <= X.t_in(u, max(v, 0.5)) + 0.3
                   and X.t_in(ou, ov) < X.t_cover(self.exit - u, v, self.v0))
        if closing and self.kind in C8_KINDS and self.other_rest_t is None and self._stopping(sr[1], sr[2], sr[3]):
            closing = False  # 074 C8: a car braking for its line will stop (the zone contract); the protocol holds
        if in_box or closing:
            need = v * v / (2 * max(-u - 0.3, 0.05))
            if need > 0.3:
                planned = a
                a = min(a, -min(B_MAX, need))
                self.shield = self.shield or (v > 0.5 and planned - a > 1.0 and need > X.BRAKE_LATE + 1.0)
            elif a > 0 and u > STOP - 1.0:
                a = min(a, 0.0)
                self.clip_n += 1
                if self.hold_stop and v < X.HOLD_STOP_V:
                    a = -min(B_MAX, v / DT)
        return a

    def clip_record(self, k):
        return {f"clip{k}_s": round(self.clip_n * DT, 2), f"restarts{k}": self.restarts}

    def after(self, t, u, v):
        if v < 0.1:
            if self.stopped_since is None:
                self.stopped_since = t
            if u < 0:
                self.rested = True
                if self.rest_t is None and u >= STOP - 2.0:
                    self.rest_t = t
            self.moving_since = None
        else:
            self.stopped_since = None
            if self.moving_since is None:
                self.moving_since = t


@dataclass
class Out:
    o: X.Outcome
    deadlock: bool
    restarts: tuple
    t_first: float


def simulate(ra, rb, dt_ctx, va, vb, lags=None):
    """One run (crossing_game.simulate with the stop-sign rules)."""
    ta, tb = X.T_MEAN + dt_ctx / 2, X.T_MEAN - dt_ctx / 2
    A, B = X.Car(STOP - va * ta, va, lags), X.Car(STOP - vb * tb, vb, lags)
    pa_, pb_ = FWPolicy("A", ra, va), FWPolicy("B", rb, vb)
    meas = M.Measures(b_max=(B_MAX, B_MAX), pet_sample=X.PET_SAMPLE)
    seen_in = {"A": None, "B": None}
    stopped_both, both_stopped, contact, t = 0.0, False, False, 0.0
    while t < T_MAX - 1e-9:
        pa_.perceive(t, B.u, B.v, B.a)
        pb_.perceive(t, A.u, A.v, A.a)
        A.a = pa_.step(t, A.u, A.v)
        B.a = pb_.step(t, B.u, B.v)
        for me in (A, B):
            if me.act is not None:
                me.a = me.act.step(me.a, me.v)
        for me, pol in ((A, pa_), (B, pb_)):
            me.peak = max(me.peak, -me.a)
            me.v = max(0.0, me.v + me.a * DT)
            me.u += me.v * DT
            pol.after(t, me.u, me.v)
            me.vh.append((t, me.v))
            while t - me.vh[0][0] > X.PEAK_W + 1e-9:
                me.vh.pop(0)
            if t - me.vh[0][0] >= X.PEAK_W - 1e-9:
                me.peak_w = max(me.peak_w, (me.vh[0][1] - me.v) / X.PEAK_W)
        t += DT
        for name, me in (("A", A), ("B", B)):
            if me.u >= 0 and seen_in[name] is None:
                seen_in[name] = t
        if A.v < X.WAIT_V and B.v < X.WAIT_V and A.u < 0 and B.u < 0:
            stopped_both += DT
            both_stopped = both_stopped or stopped_both >= 0.5
        else:
            stopped_both = 0.0
        pa = (A.u - W_BOX / 2 - L / 2, 0.0, 0.0, A.v, 0.0, L, W_CAR)
        pb = (0.0, B.u - W_BOX / 2 - L / 2, math.pi / 2, 0.0, B.v, L, W_CAR)
        meas.push_poses(t, pa, pb)
        if meas.clear <= 0.0:
            contact = True
            break
        if A.u >= EXIT + 2 and B.u >= EXIT + 2 and t >= HORIZON - 1e-9:
            break
    lost = [1.0 if contact or me.u < EXIT else M.lost_progress(me.v0, meas.s_h[k]) for k, me in enumerate((A, B))]
    first = ""
    if seen_in["A"] is not None or seen_in["B"] is not None:
        first = "A" if (seen_in["A"] or INF) < (seen_in["B"] or INF) else "B"
    deadlock = not contact and A.u < 0 and B.u < 0
    o = X.Outcome(tuple(lost), (A.peak, B.peak), contact, meas.ttc2d, meas.thw, meas._pet(), meas.clear, first,
                  both_stopped, "", (pa_.shield, pb_.shield), (pa_.rested, pb_.rested), (A.peak_w, B.peak_w))
    tf = min(v for v in (seen_in["A"], seen_in["B"]) if v is not None) if first else INF
    return Out(o, deadlock, (pa_.restarts, pb_.restarts), tf)


def klass(pure, mixed, outs):
    if any(outs[p].o.contact for p in pure):
        return "!"
    if any(outs[p].deadlock for p in pure):
        return "W"
    firsts = {outs[p].o.first for p in pure}
    if not pure:
        return "M" if mixed else "?"
    if firsts == {"A"}:
        return "A"
    if firsts == {"B"}:
        return "B"
    return "C"


def context(job):
    dt, v, lags, c8 = job
    names = NAMES + (C8_NAMES if c8 else [])
    rules = [r for r in RULES if r[0] in names]
    subsets = dict(SUBSETS, **(C8_SUBSETS if c8 else {}))
    outs = {(a[0], b[0]): simulate(a, b, dt, v, v, lags) for a in rules for b in rules}
    ua = {k: X.utility(o.o, 0) for k, o in outs.items()}
    ub = {k: X.utility(o.o, 1) for k, o in outs.items()}
    row = {"dt": dt, "v": v}
    for sub, sn in subsets.items():
        pure, mixed = X.solve(ua, ub, sn, sn)
        row[f"class_{sub}"] = klass(pure, mixed, outs)
        row[f"eq_{sub}"] = " ".join(f"{a}/{b}" for a, b in pure)
        if pure:
            row[f"welfare_{sub}"] = round(sum(ua[p] + ub[p] for p in pure) / len(pure), 4)
    # each convention against itself (the same rule for both cars, c 1.5): what it does on its own
    for n in names:
        o = outs[(n, n)]
        row[f"self_{n}"] = ("!" if o.o.contact else "W" if o.deadlock else o.o.first or "-") + \
            ("" if not sum(o.restarts) else f"r{sum(o.restarts)}")
    return row, {f"{a}|{b}": (round(ua[(a, b)], 4), round(ub[(a, b)], 4), outs[(a, b)].o.first, int(outs[(a, b)].o.contact),
                              int(outs[(a, b)].deadlock), round(outs[(a, b)].o.pet, 2) if math.isfinite(outs[(a, b)].o.pet) else "inf")
                 for a in names for b in names}


def run_map(a):
    lags = X.car_lags() if a.car_lags else None
    vs = [float(x) for x in a.v.split(",")]
    n = int(round(2 * a.dt_max / a.dt_step))
    dts = [round(-a.dt_max + i * a.dt_step, 3) for i in range(n + 1)]
    jobs = [(dt, v, lags, a.c8) for v in vs for dt in dts]
    with Pool(a.jobs) as pool:
        res = pool.map(context, jobs)
    os.makedirs(a.out, exist_ok=True)
    rows = [r for r, _ in res]
    with open(os.path.join(a.out, "four_way_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(a.out, "four_way_payoffs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dt", "v", "a", "b", "uA", "uB", "first", "contact", "deadlock", "pet"])
        for (r, pay) in res:
            for k, vals in pay.items():
                w.writerow([r["dt"], r["v"], *k.split("|"), *vals])
    lines = [f"# four-way stop (kinematic): Z {W_BOX} m, margins {M_BEFORE:g}/{M_AFTER:g} s, stop {STOP_DWELL:g} s, tie {TIE:g} s; "
             + ("the car's command lags" if lags else "an ideal car"),
             "# class per dt (t_A - t_B to the line; > 0: B arrives first): A / B first in every equilibrium, C chicken,",
             "# W an equilibrium deadlocks, ! contact, M mixed only; self_<rule>: that rule against itself (first car, rN restarts)",
             "dt:                 " + " ".join(f"{d:>5g}" for d in dts)]
    by = {(r["dt"], r["v"]): r for r in rows}
    subsets = dict(SUBSETS, **(C8_SUBSETS if a.c8 else {}))
    for v in vs:
        for sub in subsets:
            lines.append(f"v {v:g} {sub:<13}" + " ".join(f"{by[(d, v)][f'class_{sub}']:>5}" for d in dts))
        for nme in NAMES + (C8_NAMES if a.c8 else []):
            if nme.endswith("_c1.5") or nme == "aggr":
                lines.append(f"v {v:g} self {nme:<8}" + " ".join(f"{by[(d, v)][f'self_{nme}']:>5}" for d in dts))
    open(os.path.join(a.out, "four_way_map.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


def selftest():
    lags = X.car_lags()
    rule = {r[0]: r for r in RULES}
    # a clear arrival: B 3 s ahead, both stop-then-yield: B first, no contact, no deadlock
    o = simulate(rule["stop1y_c1.5"], rule["stop1y_c1.5"], 3.0, 8.0, 8.0, lags)
    assert o.o.first == "B" and not o.o.contact and not o.deadlock, o
    # fcfsr breaks a tie right-before-left: B first
    o = simulate(rule["fcfsr_c1.5"], rule["fcfsr_c1.5"], 0.0, 8.0, 8.0, lags)
    assert o.o.first == "B" and not o.o.contact and not o.deadlock, o
    # aggr against aggr at a tie: contact or a safety-layer save, never a deadlock
    o = simulate(rule["aggr"], rule["aggr"], 0.0, 8.0, 8.0, lags)
    assert not o.deadlock
    # 074 C8: fcfsm and fcfse at a tie and near the tie threshold: no contact, no deadlock; fcfse against
    # fcfsr (which reads A first and waits for it) does not deadlock
    for k in ("fcfsm_c1.5", "fcfse_c1.5"):
        for dt in (-0.75, -0.5, -0.25, 0.0, 0.25, 0.5):
            o = simulate(rule[k], rule[k], dt, 8.0, 8.0, lags)
            assert not o.o.contact and not o.deadlock, (k, dt, o)
    print("selftest ok")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="four_way_out")
    p.add_argument("--v", default="6,10")
    p.add_argument("--dt-max", type=float, default=4.0)
    p.add_argument("--dt-step", type=float, default=0.5)
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--jobs", type=int, default=16)
    p.add_argument("--c8", action="store_true", help="074 C8: add fcfsm and fcfse to the games (subsets fcfsm, fcfse, all_c8)")
    p.add_argument("--fm-m", type=float, default=FM_M, help="fcfsm's margin (s)")
    p.add_argument("--fm-bias", type=float, default=FM_BIAS, help="fcfsm's rest-stamp lag correction (s)")
    p.add_argument("--selftest", action="store_true")
    X.add_legacy_args(p)
    a = p.parse_args()
    X.apply_legacy_args(a)
    set_fm(a.fm_m, a.fm_bias)
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
