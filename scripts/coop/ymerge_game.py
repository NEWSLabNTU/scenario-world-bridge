#!/usr/bin/env python3
"""Roadmap 074 N5 (a): the 45 deg Y merge as a passage game in a kinematic model.

Town03 junction 1901 mirrored: a one-lane main road (lanelets 19049 -> 134069, straight, 40 m
through the junction) and a one-lane diagonal road (36202 -> 134982) that meets it at 45 deg
from the driver's left and bends onto it, both running into one lane (19691). The lanelets are
marked conflicting and carry no regulatory element: no right of way, no lane end, no lane
change. Autoware treats it with the intersection module (mutual yield).

The geometry (OpenDRIVE frame, main road along +x, the merge point M where the diagonal's
centre line meets the main road's, tangent):
  main      y = 0, heading 0
  diagonal  a straight at -45 deg (coming from +y), then an arc of radius R_ARC turning left
            onto the main road at M. j1901's junction part, from the E7 survey: entry 31.3 m
            before M along the main road and 22.1 m to its side, a 38 m chord; a 22.1 m straight
            and an arc of 22.2 m radius (17.4 m) give 31.3 / 22.1 and a 39.5 m path (lanelet
            134982: 39.9 m)
  Z         the conflict zone: the last Z_LEN m of each path before M, where the two lanes'
            footprints overlap (the diagonal's centre comes within one lane width, 3.5 m, of
            the main road's 12.6 m before M on its arc, 12.0 m before M on the main road)
  after M   one lane: the second car follows the first.

A is the diagonal car (the merging claimant), B the main road's. The context is

  dt   t_A^Z - t_B^Z, each car's time to Z's near edge at its own speed (s); dt > 0: B nearer
  v    both cars' speed at t = 0 (m/s)
  r    right of way: none (j1901), A or B (what a sign would do)

The rules are crossing_game's families with the merge's meaning: going second is not stopping
for the zone but falling in behind.

  A  gap acceptance tau: go iff A, at full drive, reaches M at least tau + L / v_B before B does
     (A merges ahead with tau s of headway), else fall in behind B. Options as crossing_game's:
     go-on-seen-yield (re-test every step) and patience P (both stopped: go after P s).
     tau = -inf "always go", +inf "always fall in behind".
  B  hold   keep going
     yield  band b, patience P: if B's and A's arrivals at M (as B sees them, DECIDE s in) are
            within b s, B falls in behind A
     react  keep going; DECIDE s in, if A is not seen braking, continue with whichever of
            yield / hold is B's best reply

"Fall in behind" is the other car as a leader on the merged lane, projected along its own path
(its distance to M), through cutin_game.follow_cruise; a car that falls in behind an other that
is level or behind it holds out for the zone's near edge instead (crossing_game.hold_out: brake
only when the stop needs BRAKE_LATE), so two cars that both give way stop and wait (patience
ends it). A car that goes follows the other too once the other is ahead on the merged lane.

The safety layer, REACT s late: the other level with this car (within a car length and S0 on the
merged lane) and in Z or about to enter it first: brake to stop before Z, as crossing_game's.

Payoffs: crossing_game.utility (measures.utility over exactly H; risk from measures.push_poses on
the oriented boxes along the real paths; fault for the car without right of way that forced the
other). Order: who passed M first. Classes as crossing_game: A / B, C (both orders), W (an
equilibrium with both stopped), !, M, ?.

The N-car stream (`stream`): B is the head of a main-road stream of N - 1 cars at time headways
h (shifted exponential, crossing N4's), A merges; A's gap test runs against each stream car in
turn (the slot behind the car it falls in behind), so a merge into a stream is a walk down the
slots. Composition (N3's): A's result predicted from the A-B pair term and the walk over the
realized slots, against the explicit N-car run; and how U_A and the stream's cost move with N.

Usage:
  scripts/coop/ymerge_game.py --out <dir> [--v 5,8,11,14] [--dt-max 4] [--dt-step 0.5] [--car-lags]
  scripts/coop/ymerge_game.py stream --out <dir> [--ns 2,3,5,10,20,30,50] [--draws 32]
  scripts/coop/ymerge_game.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import statistics as st
import sys
from dataclasses import dataclass
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as G  # noqa: E402
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402

DT = G.DT
HORIZON = G.HORIZON
T_MAX = G.T_MAX
L, W_CAR = G.L, G.W_CAR
A_MAX, B_MAX = G.A_MAX, G.B_MAX
OBS, DECIDE, REACT = G.OBS, G.DECIDE, G.REACT
T_MEAN = G.T_MEAN
INF = math.inf
CAP = C1.TYPES["car"]
S0 = C1.S0

# the geometry: j1901 (E7 survey, "2026-11-15 coop E6 E7 sites")
R_ARC = 22.2  # m, the diagonal's arc onto the main road
L_JUNCTION = 22.1  # m, the diagonal's straight inside the junction before the arc
ANGLE = math.pi / 4  # the diagonal meets the main road at 45 deg
Z_LEN = 12.5  # m, the conflict zone along each path before M
EXIT = Z_LEN + L  # front past Z's near edge when the rear leaves M
STOP = G.STOP
ARC_LEN = R_ARC * ANGLE


class Path:
    """A lane centre line, parameterized by arc length s from its start, in the OpenDRIVE frame
    of one Y (M at (0, 0)); `before` m of it lie before M."""

    def __init__(self, kind, before):
        self.kind, self.before = kind, before
        if kind == "diag":
            self.straight = before - ARC_LEN  # the straight's part before the arc
            # the arc's start: M - R (sin 45, -(1 - cos 45))
            self.p1 = (-R_ARC * math.sin(ANGLE), R_ARC * (1 - math.cos(ANGLE)))

    def pose(self, s):
        """(x, y, heading) at s; past M both paths run along the main road."""
        d = s - self.before  # distance past M (negative before)
        if self.kind == "main" or d >= 0:
            return d, 0.0, 0.0
        if d >= -ARC_LEN:  # on the arc: heading from -45 deg to 0, centre (0, R)
            phi = -d / R_ARC  # angle still to turn
            return -R_ARC * math.sin(phi), R_ARC * (1 - math.cos(phi)), -phi
        back = -d - ARC_LEN  # on the straight, back from the arc's start
        return (self.p1[0] - back * math.cos(ANGLE), self.p1[1] + back * math.sin(ANGLE), -ANGLE)

    def curvature(self, s):
        d = s - self.before
        return 1.0 / R_ARC if self.kind == "diag" and -ARC_LEN <= d < 0 else 0.0


def zone_overlap():
    """Where the diagonal's centre first comes within one lane width of the main road's, and
    the main road's matching point: the basis of Z_LEN (checked in the selftest)."""
    phi = math.acos(1 - G.W_BOX / R_ARC)
    return R_ARC * phi, R_ARC * math.sin(phi)


def t_cover(d, v, vmax):
    return G.t_cover(d, v, vmax)


def t_to_m(u, v):
    """Time for a car at u (front past Z's near edge) to reach M at its current speed."""
    d = Z_LEN - u
    return 0.0 if d <= 0 else (d / v if v > 0.1 else INF)


def follow_far(u, v, ou, ov, v0):
    """Follow the other on the merged lane (cutin_game.follow_cruise); while the zone is further
    than a comfortable stop away, at most cutin_game.B_YIELD_DECEL: a car that falls in behind
    has time to open its gap gently (IDM on a projected gap below zero would brake at the limit)."""
    a = C1.follow_cruise(ou - u - L, v, ov, CAP, v0)
    if -u > v * v / (2 * C1.B_YIELD_DECEL) + S0:
        a = max(a, -C1.B_YIELD_DECEL)
    return a


A_RULES = G.A_RULES
B_RULES = G.B_RULES
HOLD, REACT_B = G.HOLD, G.REACT_B


class YPolicy(G.Policy):
    """crossing_game.Policy with the merge's continuation: going second is falling in behind.

    perceive() takes the other's (u, v, a); u is the front's distance past Z's near edge along the
    car's own path, the same for both paths (Z_LEN each), so the merged-lane gap is ou - u - L."""

    def __init__(self, role, rule, v0, b_cont="hold"):
        super().__init__(role, rule, v0, b_cont, EXIT)

    def _gap_first(self, t, u, v, ou, ov):
        """A's test: at full drive, A reaches M at least tau + L / v_other before the other."""
        return t_to_m(ou, ov) - t_cover(Z_LEN - u, v, self.v0) >= self.tau + L / max(ov, 1.0)

    def behind(self, u, v, ou, ov):
        """Fall in behind the other, projected onto the merged lane; hold out for Z if it is not ahead."""
        if ou > u:
            return follow_far(u, v, ou, ov, self.v0)
        return G.hold_out(u, v)

    def step(self, t, u, v):
        sd = self._seen(t, OBS + DECIDE)
        ou = ov = oa = None
        if sd is not None:
            ou, ov, oa = sd[1] + sd[2] * (t - sd[0]), sd[2], sd[3]
            if self.role == "A" and not self.committed:
                test = self._gap_first(t, u, v, ou, ov) or ou >= EXIT
                if not self.decided or self.go_y:
                    self.committed = test
                    self.decided = True
                elif ou >= EXIT:
                    self.committed = True
                if self.stopped_since is not None and t - self.stopped_since >= self.pat:
                    self.committed = True
            if self.role == "B":
                if self.name == "react" and not self.react_done and t >= OBS + DECIDE:
                    self.react_done = True
                    if oa > -G.YIELD_SEEN and self.b_cont == "yield":
                        self.committed = False
                        self.band, self.pat = INF, G.PATIENCE[0]
                if not self.committed and t >= OBS + DECIDE:
                    if not self.decided:
                        self.decided = True
                        self.committed = abs(t_to_m(ou, ov) - t_to_m(u, max(v, 0.1))) >= self.band and ou < Z_LEN
                    if ou >= EXIT or (self.stopped_since is not None and t - self.stopped_since >= self.pat):
                        self.committed = True
        if sd is None:
            a = G.drive(v, self.v0)
        elif self.committed:
            a = G.drive(v, self.v0)
            if ou > u + L / 2:  # the other is ahead on the merged lane: follow it
                a = min(a, follow_far(u, v, ou, ov, self.v0))
        else:
            a = self.behind(u, v, ou, ov)
        sr = self._seen(t, REACT, first=False)
        if sr is None or u >= 0:
            return a
        ou, ov = sr[1] + sr[2] * (t - sr[0]), sr[2]
        level = abs(ou - u) < L + S0  # side by side on the merged lane: no following gap
        in_box = 0 <= ou < EXIT and level
        closing = (level and ou < 0 and ov > 0.5 and G.t_in(ou, ov) < 2.0
                   and G.t_in(ou, ov) <= G.t_in(u, max(v, 0.5)) + 0.3)
        if in_box or closing:
            need = v * v / (2 * max(-u - 0.3, 0.05))
            if need > 0.3:
                planned = a
                a = min(a, -min(B_MAX, need))
                self.shield = self.shield or (v > 0.5 and planned - a > 1.0 and need > G.BRAKE_LATE + 1.0)
            elif a > 0 and u > STOP - 1.0:
                a = min(a, 0.0)
                if G.HOLD_STOP and v < G.HOLD_STOP_V:  # crossing_game.HOLD_STOP
                    a = -min(B_MAX, v / DT)
        return a


@dataclass
class World:
    """One Y: the two paths; a car's path coordinate s = u + before - Z_LEN."""
    before: float = 200.0

    def __post_init__(self):
        self.paths = {"A": Path("diag", self.before), "B": Path("main", self.before)}

    def pose(self, k, u, v, length=L, width=W_CAR):
        """measures.push_poses' (x, y, yaw, vx, vy, L, W) for car k: the box's centre, the front at u."""
        p = self.paths[k]
        s_c = u + p.before - Z_LEN - length / 2
        x, y, h = p.pose(s_c)
        return (x, y, h, v * math.cos(h), v * math.sin(h), length, width)


def simulate(ra, rb, dt_ctx, va, vb, b_cont="hold", trace=None, lags=None):
    """One two-car run, scored as crossing_game.simulate (the Outcome and utility are its own)."""
    ta, tb = T_MEAN + dt_ctx / 2, T_MEAN - dt_ctx / 2
    A, B = G.Car(-va * ta, va, lags), G.Car(-vb * tb, vb, lags)
    pa_, pb_ = YPolicy("A", ra, va), YPolicy("B", rb, vb, b_cont)
    world = World(before=max(-A.u, -B.u) + Z_LEN + 50.0)
    meas = M.Measures(b_max=(B_MAX, B_MAX), pet_sample=G.PET_SAMPLE)
    at_m = {"A": None, "B": None}
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
            while t - me.vh[0][0] > G.PEAK_W + 1e-9:
                me.vh.pop(0)
            if t - me.vh[0][0] >= G.PEAK_W - 1e-9:
                me.peak_w = max(me.peak_w, (me.vh[0][1] - me.v) / G.PEAK_W)
        if trace is not None:
            trace.append((round(t, 2), A.u, A.v, A.a, pa_.committed, B.u, B.v, B.a, pb_.committed))
        t += DT
        for k, me in (("A", A), ("B", B)):
            if me.u >= Z_LEN and at_m[k] is None:
                at_m[k] = t
        if A.v < G.WAIT_V and B.v < G.WAIT_V and A.u < 0 and B.u < 0:
            stopped_both += DT
            both_stopped = both_stopped or stopped_both >= 0.5
        else:
            stopped_both = 0.0
        meas.push_poses(t, world.pose("A", A.u, A.v), world.pose("B", B.u, B.v))
        if meas.clear <= 0.0:
            contact = True
            break
        if A.u >= EXIT + 2 and B.u >= EXIT + 2 and t >= HORIZON - 1e-9:
            break
    lost = [1.0 if contact or me.u < EXIT else M.lost_progress(me.v0, meas.s_h[k]) for k, me in enumerate((A, B))]
    first = ""
    if at_m["A"] is not None or at_m["B"] is not None:
        first = "A" if (at_m["A"] if at_m["A"] is not None else INF) < (at_m["B"] if at_m["B"] is not None else INF) else "B"
    return G.Outcome(tuple(lost), (A.peak, B.peak), contact, meas.ttc2d, meas.thw, meas._pet(), meas.clear, first,
                     both_stopped, b_cont if rb[0] == "react" else "", (pa_.shield, pb_.shield), (pa_.rested, pb_.rested),
                     (A.peak_w, B.peak_w))


def outcomes(dt_ctx, va, vb, lags=None):
    outs = {}
    for ra in A_RULES:
        for rb in B_RULES:
            if rb[0] == "react":
                for c in ("yield", "hold"):
                    outs[(ra[0], f"react:{c}")] = simulate(ra, rb, dt_ctx, va, vb, c, lags=lags)
            else:
                outs[(ra[0], rb[0])] = simulate(ra, rb, dt_ctx, va, vb, lags=lags)
    return outs


def _context(job):
    dt_ctx, v, vb, fault, lags = job
    outs = outcomes(dt_ctx, v, vb if vb is not None else v, lags)
    rows = []
    for r in ("none", "A", "B"):
        res, ua, ub, pure, mixed = G.play(outs, r, fault)
        c_opt, c_worst, c_best = G.welfare(res, ua, ub, pure, mixed)
        firsts = sorted({res[p].first for p in pure})
        f_min = None
        if r != "none":
            for f in G.FAULTS:
                rf, _, _, pf, _ = G.play(outs, r, f)
                if pf and all(rf[p].first == r and not rf[p].contact for p in pf):
                    f_min = f
                    break
        eq_u = [(ua[p], ub[p]) for p in pure]
        rows.append({
            "r": r, "dt": dt_ctx, "v": v, "class": G.label(res, pure, mixed), "fault_min": f_min,
            "n_pure": len(pure), "n_mixed": len(mixed), "firsts": "".join(firsts),
            "pure": " ".join(f"{a}/{b}" + (f":{res[(a, b)].b_cont}" if b == "react" else "") for a, b in pure),
            "eq_contact": sum(res[p].contact for p in pure), "eq_wait": sum(res[p].both_stopped for p in pure),
            "min_pet_eq": min((res[p].pet for p in pure), default=INF),
            "uA_eq": ";".join(f"{x[0]:.3f}" for x in eq_u), "uB_eq": ";".join(f"{x[1]:.3f}" for x in eq_u),
            "C_opt": c_opt, "C_worst_eq": c_worst, "C_best_eq": c_best,
            "contact_pairs": sum(o.contact for o in res.values()),
        })
    return rows


def fmt(x):
    if x is None:
        return ""
    if isinstance(x, float):
        return "inf" if math.isinf(x) else f"{x:.3f}"
    return x


def run_map(args):
    n = int(round(2 * args.dt_max / args.dt_step))
    dts = [round(-args.dt_max + args.dt_step * i, 3) for i in range(n + 1)]
    vs = [float(x) for x in args.v.split(",")]
    lags = G.car_lags() if args.car_lags else None
    jobs = [(d, v, args.vb, args.fault, lags) for v in vs for d in dts]
    with Pool(args.jobs) as pool:
        rows = [row for rs in pool.map(_context, jobs) for row in rs]
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "ymerge_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for row in rows:
            w.writerow({k: fmt(v) for k, v in row.items()})
    lab = {(row["r"], row["dt"], row["v"]): row for row in rows}
    lines = [f"# Y merge equilibrium map (kinematic): {len(A_RULES)} A rules x {len(B_RULES)} B rules, fault {args.fault:g}"
             + (", the car's command lags" if args.car_lags else ", an ideal car"),
             f"# A on the diagonal, B on the main road; dt = t_A^Z - t_B^Z from {dts[0]} to {dts[-1]} s (dt > 0: B nearer)",
             "# A / B: that car merges first in every equilibrium   C: both orders are equilibria",
             "# W: an equilibrium with both stopped   !: contact in equilibrium   M: mixed only   ?: none"]
    for r in ("none", "A", "B"):
        lines.append(f"\n## right of way: {r}")
        lines.append("  v(m/s) " + "".join(f"{d:+5.1f}"[-4:].rjust(5) for d in dts))
        for v in reversed(vs):
            lines.append(f"  {v:6.1f} " + "".join(lab[(r, d, v)]["class"].rjust(5) for d in dts))
        if r != "none":
            lines.append(f"  smallest fault (of {G.FAULTS}) after which every equilibrium lets {r} through first (- : none)")
            for v in reversed(vs):
                lines.append(f"  {v:6.1f} " + "".join(("-" if lab[(r, d, v)]["fault_min"] is None else f"{lab[(r, d, v)]['fault_min']:g}").rjust(5) for d in dts))
        lines.append("  welfare loss of the worst equilibrium, C_worst - C_opt: . <0.05  - <0.2  + <0.5  # >=0.5")
        for v in reversed(vs):
            s = ""
            for d in dts:
                row = lab[(r, d, v)]
                x = None if row["C_worst_eq"] is None else row["C_worst_eq"] - row["C_opt"]
                s += ("?" if x is None else "." if x < 0.05 else "-" if x < 0.2 else "+" if x < 0.5 else "#").rjust(5)
            lines.append(f"  {v:6.1f} " + s)
    text = "\n".join(lines) + "\n"
    open(os.path.join(args.out, "ymerge_map.txt"), "w").write(text)
    print(text, end="")
    return 0


# --- the N-car stream ----------------------------------------------------------------------------
H_MIN = 1.0  # s, the shortest headway (N4's)
STREAM_A_RULES = ("t0.5yp2", "t1.5yp2", "t3yp2")  # A's gap acceptance, re-tested on every slot
STREAM_T = 60.0  # s, a stream run's length (A must get in; lost is over the window [0, H] as N4's)


def stream_draw(n, h_mean, v, rng):
    """The stream's time headways behind B (n - 2 of them), N4's shifted exponential."""
    return [H_MIN + rng.expovariate(1.0 / (h_mean - H_MIN)) for _ in range(max(0, n - 2))]


def simulate_stream(rule_a, dt_ctx, v, heads, lags=None, b_rule=HOLD, trace=None):
    """A on the diagonal, B and the stream behind it on the main road. A negotiates with the first
    stream car not clearly ahead of it (its front less than half a car ahead of A's), with a fresh
    gap test whenever that car changes (A has fallen in behind the previous one), and follows the
    stream car nearest ahead of it. Every stream car follows its leader and runs a YPolicy against A
    (B's rule b_rule, the rest hold: they keep going, follow A once A is ahead, and their safety
    layer brakes for an A level with them in the zone). Each car's lost progress is over [0, H]."""
    ra = next(r for r in A_RULES if r[0] == rule_a)
    A = G.Car(-v * (T_MEAN + dt_ctx / 2), v, lags)
    us = [-v * (T_MEAN - dt_ctx / 2)]
    for h in heads:
        us.append(us[-1] - v * h - L)
    cars = [G.Car(u, v, lags) for u in us]
    pols = [YPolicy("B", b_rule if i == 0 else HOLD, v) for i in range(len(cars))]
    world = World(before=max(-A.u, -min(us)) + Z_LEN + 50.0)
    rhos = [0.0] * len(cars)  # each stream car's largest risk against A
    hit = [False] * len(cars)  # each stream car's contact with A
    facing, pol_a = None, None
    t, contact, slot_of_a = 0.0, False, None
    s_h = [None] * (len(cars) + 1)
    while t < STREAM_T - 1e-9:
        f = next((i for i, c in enumerate(cars) if c.u < A.u + L / 2), None)
        if f != facing:
            facing = f
            pol_a = YPolicy("A", ra, v) if f is not None else None
        if pol_a is not None:
            c = cars[facing]
            pol_a.perceive(t, c.u, c.v, c.a)
            a_a = pol_a.step(t, A.u, A.v)
        else:
            a_a = G.drive(A.v, v)
        ahead = [c for c in cars if c.u >= A.u + L / 2]
        if ahead:
            lead = min(ahead, key=lambda c: c.u)
            a_a = min(a_a, follow_far(A.u, A.v, lead.u, lead.v, v))
        accs = []
        for i, c in enumerate(cars):
            pols[i].perceive(t, A.u, A.v, A.a)
            a = pols[i].step(t, c.u, c.v)
            if i > 0:
                lead = cars[i - 1]
                a = min(a, C1.follow_cruise(lead.u - c.u - L, c.v, lead.v, CAP, v))
            accs.append(a)
        A.a = a_a
        for c, a in zip(cars, accs):
            c.a = a
        for me in [A] + cars:
            if me.act is not None:
                me.a = me.act.step(me.a, me.v)
            me.peak = max(me.peak, -me.a)
            me.v = max(0.0, me.v + me.a * DT)
            me.u += me.v * DT
        if pol_a is not None:
            pol_a.after(t, A.u, A.v)
        for i, c in enumerate(cars):
            pols[i].after(t, c.u, c.v)
        if trace is not None:
            trace.append((round(t, 2), facing, pol_a.committed if pol_a else None, round(A.u, 2), round(A.v, 2), round(A.a, 2),
                          [(round(c.u, 2), round(c.v, 2), round(c.a, 2)) for c in cars[:4]]))
        t += DT
        if slot_of_a is None and A.u >= Z_LEN:
            slot_of_a = sum(c.u > A.u for c in cars)  # how many stream cars are ahead of A at M
        if s_h[0] is None and t >= HORIZON - 1e-9:
            s_h = [A.u] + [c.u for c in cars]
        pa = world.pose("A", A.u, A.v)
        for i, c in enumerate(cars):
            if abs(c.u - A.u) < 30:
                m_ = M.Measures(b_max=(B_MAX, B_MAX))
                m_.push_poses(t, pa, world.pose("B", c.u, c.v))
                if m_.clear <= 0.0:
                    contact = hit[i] = True
                rhos[i] = max(rhos[i], M.rho(m_.ttc2d, m_.thw, INF, m_.clear))
        if contact or (t >= HORIZON and slot_of_a is not None and A.u >= EXIT + 2):
            break
    u0a = -v * (T_MEAN + dt_ctx / 2)
    lost_a = 1.0 if contact or slot_of_a is None or s_h[0] is None else M.lost_progress(v, s_h[0] - u0a)
    lost_s = [1.0 if hit[i] else (M.lost_progress(v, s_h[i + 1] - us[i]) if s_h[i + 1] is not None else 0.0)
              for i in range(len(cars))]
    return {"slot": slot_of_a, "contact": contact, "lost_a": lost_a, "hit": hit,
            "lost_stream": lost_s, "peak_a": A.peak, "peak_stream": [c.peak for c in cars], "rho": rhos}


def walk_prediction(pair_lost, pair_slot, slot, heads, v):
    """N3's composition: the A-B pair term, then, for every stream car past B that A let through
    beyond the pair's outcome, that car's headway behind its leader plus a car length at v,
    capped at the window. (A pair outcome of A ahead and a stream outcome of A behind B is the
    discrete change the walk cannot see; it shows as error.)"""
    lost = pair_lost
    for i in range(max(pair_slot, 1), slot):
        lost += (v * heads[i - 1] + L) / (v * HORIZON)
    return min(1.0, lost)


def stream(args):
    ns = [int(x) for x in args.ns.split(",")]
    rng = random.Random(args.seed)
    lags = G.car_lags() if args.car_lags else None
    out = []
    jobs = []
    for h_mean in [float(x) for x in args.h_means.split(",")]:
        for d in range(args.draws):
            heads = stream_draw(max(ns), h_mean, args.v, rng)
            for dt_ctx in [float(x) for x in args.dts.split(",")]:
                for rule in STREAM_A_RULES:
                    jobs.append((h_mean, d, dt_ctx, rule, heads, ns, args.v, lags))
    with Pool(args.jobs) as pool:
        for rows in pool.imap_unordered(_stream_job, jobs, chunksize=4):
            out += rows
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "stream.jsonl"), "w") as f:
        for r in out:
            f.write(json.dumps(r) + "\n")
    lines = stream_report(out, ns)
    open(os.path.join(args.out, "stream.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


def _stream_job(job):
    h_mean, d, dt_ctx, rule, heads, ns, v, lags = job
    pair = simulate_stream(rule, dt_ctx, v, [], lags)
    rows = []
    for n in ns:
        r = simulate_stream(rule, dt_ctx, v, heads[:max(0, n - 2)], lags)
        # the same stream with A 2 km back (it never meets the stream inside the window): what the
        # stream costs itself (IDM's start at short headways), so A's externality is the difference
        base = simulate_stream(rule, 400.0, v, heads[:max(0, n - 2)], lags)
        cost_base = sum(-M.utility(ls, 0.0, pk, B_MAX) for ls, pk in zip(base["lost_stream"], base["peak_stream"]))
        pred = (walk_prediction(pair["lost_a"], pair["slot"] or 0, r["slot"], heads, v)
                if r["slot"] is not None else 1.0)
        u_a = M.utility(r["lost_a"], max(r["rho"]), r["peak_a"], B_MAX, contact=r["contact"])
        cost_stream = sum(-M.utility(ls, rh, pk, B_MAX, contact=hi)
                          for ls, rh, pk, hi in zip(r["lost_stream"], r["rho"], r["peak_stream"], r["hit"]))
        rows.append({"h_mean": h_mean, "draw": d, "dt": dt_ctx, "rule": rule, "n": n, "slot": r["slot"],
                     "contact": int(r["contact"]), "lost_a": round(r["lost_a"], 4), "lost_a_walk": round(pred, 4),
                     "lost_a_pair": round(pair["lost_a"], 4), "slot_pair": pair["slot"], "uA": round(u_a, 4),
                     "cost_stream": round(cost_stream, 4), "cost_base": round(cost_base, 4),
                     "externality": round(cost_stream - cost_base, 4), "peak_a": round(r["peak_a"], 2),
                     "max_peak_stream": round(max(r["peak_stream"]), 2)})
    return rows


def stream_report(out, ns):
    lines = ["# Y merge into a main-road stream (kinematic): A's lost progress explicit vs composed, and against N"]
    by = {}
    for r in out:
        by.setdefault((r["h_mean"], r["rule"], r["n"]), []).append(r)
    lines.append("h_mean rule      N | slot mean  lostA  walk err (mean abs, max)  uA     stream cost  A's externality  contacts")
    for (h, rule, n), rs in sorted(by.items()):
        err = [abs(r["lost_a"] - r["lost_a_walk"]) for r in rs]
        lines.append(f"{h:5.1f} {rule:>8} {n:3d} | {st.mean(r['slot'] or 0 for r in rs):5.2f}  {st.mean(r['lost_a'] for r in rs):.3f}  "
                     f"{st.mean(err):.3f} {max(err):.3f}   {st.mean(r['uA'] for r in rs):+.3f}  {st.mean(r['cost_stream'] for r in rs):.3f}"
                     f"        {st.mean(r['externality'] for r in rs):.3f}"
                     f"  {sum(r['contact'] for r in rs)}/{len(rs)}")
    # N dependence: the change of mean uA and of the stream's cost from N to the largest N
    nmax = max(ns)
    lines.append("\nN dependence: |mean uA(N) - mean uA(N_max)|, |externality(N) - externality(N_max)| per h_mean, rule")
    for h in sorted({k[0] for k in by}):
        for rule in STREAM_A_RULES:
            ref = by.get((h, rule, nmax))
            if not ref:
                continue
            ua_ref, c_ref = st.mean(r["uA"] for r in ref), st.mean(r["externality"] for r in ref)
            lines.append(f"{h:5.1f} {rule:>8} " + "  ".join(
                f"N{n}: {abs(st.mean(r['uA'] for r in by[(h, rule, n)]) - ua_ref):.3f}/{abs(st.mean(r['externality'] for r in by[(h, rule, n)]) - c_ref):.3f}"
                for n in ns if (h, rule, n) in by))
    return lines


def selftest():
    # the geometry: j1901's junction part, 31.3 m along and 22.1 m aside, 39.5 m of path
    p = Path("diag", 200.0)
    x, y, h = p.pose(200.0 - ARC_LEN - L_JUNCTION)
    assert abs(x + 31.3) < 0.3 and abs(y - 22.1) < 0.3 and abs(h + ANGLE) < 1e-9, (x, y, h)
    assert abs(ARC_LEN + L_JUNCTION - 39.5) < 0.1
    xm, ym, hm = p.pose(200.0)
    assert abs(xm) < 1e-9 and abs(ym) < 1e-9 and abs(hm) < 1e-9
    s_d, s_m = zone_overlap()
    assert 11.5 < s_d < 13.5 and 11.0 < s_m < 13.0 and abs(Z_LEN - (s_d + s_m) / 2) < 0.3, (s_d, s_m)
    go, never = A_RULES[0], next(r for r in A_RULES if r[1] == INF and not r[2] and r[3] == G.PATIENCE[1])
    # B far ahead: B merges first whatever A does, no contact
    for ra in (go, never):
        o = simulate(ra, HOLD, 4.0, 10.0, 10.0)
        assert not o.contact and o.first == "B", (ra[0], o)
    # A far ahead: A first under go
    o = simulate(go, HOLD, -4.0, 10.0, 10.0)
    assert not o.contact and o.first == "A", o
    # a tie, both going: contact or a near miss
    o = simulate(go, HOLD, 0.0, 10.0, 10.0)
    assert o.contact or o.pet < 1.0 or o.shield[0] or o.shield[1], o
    # A falling in behind a holding B at a tie: B first, no contact, and A need not stop
    o = simulate(never, HOLD, 0.0, 10.0, 10.0)
    assert not o.contact and o.first == "B" and not o.stopped[0], o
    # the stream: a dense stream at 1 s headways puts A further back than B alone does
    r1 = simulate_stream("t1.5yp2", 0.0, 10.0, [])
    r5 = simulate_stream("t1.5yp2", 0.0, 10.0, [1.0, 1.0, 1.0, 4.0])
    assert r1["slot"] is not None and r5["slot"] is not None and r5["slot"] >= r1["slot"] and not r5["contact"], (r1, r5)
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", nargs="?", default="map", choices=("map", "stream"))
    p.add_argument("--out", default="ymerge_out")
    p.add_argument("--v", default="5,8,11,14")
    p.add_argument("--vb", type=float, default=None)
    p.add_argument("--dt-max", type=float, default=4.0)
    p.add_argument("--dt-step", type=float, default=0.5)
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--fault", type=float, default=G.FAULT)
    p.add_argument("--ns", default="2,3,5,10,20,30,50")
    p.add_argument("--draws", type=int, default=32)
    p.add_argument("--h-means", default="3,4")
    p.add_argument("--dts", default="-1,0,1")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    if a.mode == "stream":
        a.v = float(a.v.split(",")[0]) if isinstance(a.v, str) else a.v
        return stream(a)
    return run_map(a)


if __name__ == "__main__":
    sys.exit(main())
