#!/usr/bin/env python3
"""Roadmap 074 S2: the unsignalled crossing as a passage game in a kinematic model.

Two cars on perpendicular straight roads through one conflict box Z (docs/design/
cooperative-driving.md ②, the "cross" instance). A drives along +x, B along +y; Z is the
W_BOX x W_BOX square where the lanes meet. The context is

  dt   t_A^Z - t_B^Z, each car's time to Z's near edge at its own speed (s); dt > 0: B is nearer
  v    both cars' speed at t = 0 (m/s; --vb fixes B's instead)
  r    right of way: none, A or B

Strategies are contingent rules, not trajectories:

  A (the claimant) gap acceptance with threshold tau: go iff
       t_B^{Z,in} - t_A^{Z,out} >= tau  (A clears Z tau s before B arrives), or
       t_A^{Z,in} - t_B^{Z,out} >= tau  (B has cleared Z tau s before A, going now, arrives),
     judged from B as A sees it (OBS + DECIDE s old) at B's seen speed, A's own exit at full drive.
     Otherwise A holds out at its speed and brakes for the stop line only when it must
     (at >= BRAKE_LATE m/s^2). Options: patience P (stopped for P s, A goes anyway) and
     go-on-seen-yield (A re-runs the gap test every step, so a B that slows opens the gap;
     without it A decides once and then goes only when B has cleared Z, or out of patience).
     tau = -inf is "always go", tau = +inf "always yield".
  B (the holder)   hold   keep going
                   yield  band b, patience P: if |t_A^Z - t_B^Z| as B sees it DECIDE s in is under
                          b, B holds out and brakes for its stop line when it must, and goes once
                          A has cleared Z or out of patience (stopped for P s)
                   react  keep going; DECIDE s in, if A is not seen braking, continue with
                          whichever of yield / hold is B's best reply (sequential rationality)

Under every rule each car's safety layer brakes, REACT s after it sees the other's body in Z, or
the other about to enter Z first (within 2 s, before this car could clear it), as hard as
stopping before Z needs, up to the car's braking limit.

Payoffs: the general utility (measures.py, ①): lost route progress over a fixed horizon, risk as
the largest shortfall of 2D TTC, headway, PET and clearance (measures.push_poses on oriented
boxes), peak braking, contact. With a right of way, the car without it pays FAULT when it passes
Z first and forces the other: the other had to slow (braking over YIELD_SEEN) while this car never
came to rest before Z, or the other's safety layer braked, or PET < PET_REF: failing to yield. A
car that stopped at its line and then went because the other waited (patience, a wave) is not at
fault unless it forced the other's safety layer. The game is solved per context: pure equilibria and mixed ones of support 2 x 2. Per
context the fault is also swept (FAULTS) for the smallest one under which every equilibrium lets
the car with right of way through first: how much enforcement the rule needs to select.

Map classes per (dt, v, r):
  A / B  every equilibrium lets the same car pass first
  C      chicken: equilibria with A first and with B first; who wins is a convention or a belief
  W      war of attrition: an equilibrium in which both cars stop and wait
  !      an equilibrium has contact
  M      only mixed equilibria;  ?  none

Usage:
  scripts/coop/crossing_game.py --out <dir> [--v 4,6,...] [--vb V] [--fault F] [--dt-max 4] [--dt-step 0.5] [--jobs N]
  scripts/coop/crossing_game.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys
from dataclasses import dataclass
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import measures as M  # noqa: E402

DT = 0.05
HORIZON = 20.0  # s, progress is judged against free flow over this (074 D2)
T_MAX = 40.0
L, W_CAR = 4.79, 2.163  # m, the Model 3's box (sim_vehicle_spec)
W_BOX = 3.5  # m, one lane each way through the crossing
EXIT = W_BOX + L  # front's distance past Z's near edge when the rear leaves Z
STOP = -1.0  # m, the stop line before Z's near edge
T_MEAN = 5.0  # s, the mean of the two arrival times at t = 0
A_MAX, B_MAX = 3.0, 7.0  # m/s^2, drive and braking limits (cutin_game's "car")
OBS = 0.19  # s, perception delay (074 E1)
DECIDE = 0.5  # s, from a seen change to a rule's decision
REACT = 1.0  # s, the safety layer's reaction (Autoware's assumption for a rear car)
BRAKE_LATE = 2.5  # m/s^2: a car holding out brakes for the stop line only when it needs this much
YIELD_SEEN = 0.5  # m/s^2: braking the other takes as a yield
PEAK_W = 0.5  # s, the window of the arena's peak braking (arena.PEAK_WINDOW)
FAULT = 0.5  # the cost of failing to yield, for the car without right of way
FAULTS = (0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0)  # swept for the smallest that makes the rule hold

INF = math.inf
TAUS = (-INF, 0.5, 1.5, 3.0, INF)
PATIENCE = (2.0, 5.0)
A_RULES = [("go", -INF, False, INF)] + [
    (f"t{tau:g}{'y' if gy else ''}p{p:g}", tau, gy, p)
    for tau in TAUS[1:] for gy in (False, True) for p in PATIENCE]
HOLD, REACT_B = ("hold", None, None), ("react", None, None)
B_RULES = [HOLD, REACT_B] + [(f"yb{b:g}p{p:g}", b, p) for b in (1.0, 2.0, 4.0) for p in PATIENCE]


def t_cover(d, v, vmax, a=A_MAX):
    """Time to cover d metres from speed v, accelerating at a up to vmax."""
    if d <= 0:
        return 0.0
    v = max(v, 0.0)
    if v >= vmax - 1e-9:
        return d / max(vmax, 0.1)
    d_acc = (vmax * vmax - v * v) / (2 * a)
    if d <= d_acc:
        return (-v + math.sqrt(v * v + 2 * a * d)) / a
    return (vmax - v) / a + (d - d_acc) / vmax


def t_in(u, v):
    return 0.0 if u >= 0 else (-u / v if v > 0.1 else INF)


def t_out(u, v):
    return (EXIT - u) / v if v > 0.1 else (0.0 if u >= EXIT else INF)


@dataclass
class Outcome:
    lost: tuple
    peak: tuple
    contact: bool
    ttc: float
    thw: float
    pet: float
    clear: float
    first: str  # "A", "B" or "" (nobody entered Z)
    both_stopped: bool
    b_cont: str = ""
    shield: tuple = (False, False)  # each car's safety layer had to brake
    stopped: tuple = (False, False)  # each car came to rest before Z
    peak_w: tuple = (0.0, 0.0)  # peak braking as the arena measures it: the speed drop over PEAK_W s


def hold_out(u, v):
    """Keep going, brake for the stop line only when it needs at least BRAKE_LATE."""
    d = STOP - u
    if d <= 0.05 and u < 0:
        return -B_MAX if v > 0 else 0.0
    if u < STOP:
        need = v * v / (2 * max(d, 0.05))
        if need >= BRAKE_LATE:
            return -min(B_MAX, need)
    return 0.0


def drive(v, v0):
    return min(A_MAX, (v0 - v) / DT) if v < v0 else 0.0


class Policy:
    """One car's rule and safety layer, on what it perceives of the other car.

    The kinematic model feeds it the truth; the arena (crossing_arena.py) its perception model.
    perceive() takes the other's (u, v, a) stamped with when it was true; the rules use what was
    true OBS + DECIDE s ago, the safety layer what was true REACT s ago, each predicted to now at
    the seen speed (so the delays hide changes of speed, not distance). u is the distance of the
    front past Z's near edge along the car's own road.
    """

    def __init__(self, role, rule, v0, b_cont="hold"):
        self.role, self.v0, self.b_cont = role, v0, b_cont
        self.decided = self.rested = self.shield = self.react_done = False
        self.stopped_since: float | None = None
        if role == "A":
            self.name, self.tau, self.go_y, self.pat = rule
            self.committed = self.name == "go"
        else:
            self.name, self.band, self.pat = rule
            self.committed = self.name in ("hold", "react")
        self.hist = []

    def perceive(self, stamp, ou, ov, oa):
        self.hist.append((stamp, ou, ov, oa))

    def _seen(self, t, age, first=True):
        for h in reversed(self.hist):
            if h[0] <= t - age:
                return h
        return self.hist[0] if first and self.hist else None

    def step(self, t, u, v):
        """The acceleration this car commands at t."""
        sd = self._seen(t, OBS + DECIDE)
        if sd is not None:
            # where the other is now, predicted from what was seen at its seen speed (a tracker's
            # constant-velocity prediction): an old position against this car's current one would
            # make every gap look larger by the delay (074 S2 arena)
            ou, ov, oa = sd[1] + sd[2] * (t - sd[0]), sd[2], sd[3]
            if self.role == "A" and not self.committed:
                ok_first = t_in(ou, ov) - t_cover(EXIT - u, v, self.v0) >= self.tau
                # A's arrival if it goes now (from its speed, at full drive), not at its current speed:
                # a nearly stopped A would otherwise see itself arriving late and go (074 S2 arena)
                ok_second = ou >= EXIT or (t_cover(-u, v, self.v0) - t_out(ou, ov) >= self.tau)
                test = ok_first or ok_second
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
                    if oa > -YIELD_SEEN and self.b_cont == "yield":
                        self.committed = False
                        self.band, self.pat = INF, PATIENCE[0]
                if not self.committed and t >= OBS + DECIDE:
                    if not self.decided:  # the band test, once: inside it B yields until A has cleared Z
                        self.decided = True
                        self.committed = abs(t_in(ou, ov) - t_in(u, max(v, 0.1))) >= self.band and ou < 0
                    if ou >= EXIT or (self.stopped_since is not None and t - self.stopped_since >= self.pat):
                        self.committed = True
        a = drive(v, self.v0) if self.committed else hold_out(u, v)
        sr = self._seen(t, REACT, first=False)
        if sr is None or u >= 0:
            return a
        ou, ov = sr[1] + sr[2] * (t - sr[0]), sr[2]
        in_box = 0 <= ou < EXIT
        # the other is about to enter Z first, before this car could clear it
        closing = (ou < 0 and ov > 0.5 and t_in(ou, ov) < 2.0
                   and t_in(ou, ov) <= t_in(u, max(v, 0.5)) + 0.3
                   and t_in(ou, ov) < t_cover(EXIT - u, v, self.v0))
        if in_box or closing:
            need = v * v / (2 * max(-u - 0.3, 0.05))
            if need > 0.3:
                planned = a
                a = min(a, -min(B_MAX, need))
                # forced: harder than the car's own plan by more than a comfortable margin
                self.shield = self.shield or (v > 0.5 and planned - a > 1.0 and need > BRAKE_LATE + 1.0)
            elif a > 0 and u > STOP - 1.0:
                a = min(a, 0.0)
        return a

    def after(self, t, u, v):
        """Bookkeeping once the car has moved: stops (patience) and whether it came to rest."""
        if v < 0.1:
            if self.stopped_since is None:
                self.stopped_since = t
                self.rested = self.rested or u < 0
        else:
            self.stopped_since = None


class Car:
    def __init__(self, u0, v0):
        self.u, self.v, self.v0, self.u0 = u0, v0, v0, u0
        self.a = 0.0
        self.peak = 0.0
        self.peak_w = 0.0
        self.vh = []  # (t, v) over the last PEAK_W s


def simulate(ra, rb, dt_ctx, va, vb, b_cont="hold", trace=None):
    """One run: A's rule ra, B's rule rb; b_cont is B's continuation under react."""
    ta, tb = T_MEAN + dt_ctx / 2, T_MEAN - dt_ctx / 2
    A, B = Car(-va * ta, va), Car(-vb * tb, vb)
    pa_, pb_ = Policy("A", ra, va), Policy("B", rb, vb, b_cont)
    meas = M.Measures(b_max=(B_MAX, B_MAX))
    seen_in: dict = {"A": None, "B": None}  # when each body was first in Z
    stopped_both = 0.0
    both_stopped = False
    contact = False
    t = 0.0
    while t < T_MAX - 1e-9:
        pa_.perceive(t, B.u, B.v, B.a)
        pb_.perceive(t, A.u, A.v, A.a)
        A.a = pa_.step(t, A.u, A.v)
        B.a = pb_.step(t, B.u, B.v)
        for me, pol in ((A, pa_), (B, pb_)):
            me.peak = max(me.peak, -me.a)
            me.v = max(0.0, me.v + me.a * DT)
            me.u += me.v * DT
            pol.after(t, me.u, me.v)
            me.vh.append((t, me.v))
            while t - me.vh[0][0] > PEAK_W + 1e-9:
                me.vh.pop(0)
            if t - me.vh[0][0] >= PEAK_W - 1e-9:
                me.peak_w = max(me.peak_w, (me.vh[0][1] - me.v) / PEAK_W)
        if trace is not None:
            trace.append((round(t, 2), A.u, A.v, A.a, pa_.committed, B.u, B.v, B.a, pb_.committed))
        t += DT
        for name, me in (("A", A), ("B", B)):
            if me.u >= 0 and seen_in[name] is None:
                seen_in[name] = t
        if A.v < 0.1 and B.v < 0.1 and A.u < 0 and B.u < 0:
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
        if A.u >= EXIT + 2 and B.u >= EXIT + 2:
            break
    lost = []
    for me in (A, B):
        if contact or me.u < EXIT:
            lost.append(1.0)
        else:
            lost.append(min(1.0, max(0.0, (me.v0 * t - (me.u - me.u0)) / max(me.v0 * HORIZON, 1.0))))
    first = ""
    if seen_in["A"] is not None or seen_in["B"] is not None:
        ta_in = seen_in["A"] if seen_in["A"] is not None else INF
        tb_in = seen_in["B"] if seen_in["B"] is not None else INF
        first = "A" if ta_in < tb_in else "B"
    pet = meas._pet()
    return Outcome(tuple(lost), (A.peak, B.peak), contact, meas.ttc2d, meas.thw, pet, meas.clear, first,
                   both_stopped, b_cont if rb[0] == "react" else "", (pa_.shield, pb_.shield), (pa_.rested, pb_.rested),
                   (A.peak_w, B.peak_w))


def utility(o, who, r="none", fault=FAULT):
    risk = M.rho(o.ttc, o.thw, o.pet, o.clear)
    u = M.utility(o.lost[who], risk, o.peak[who], B_MAX, contact=o.contact)
    me = "AB"[who]
    if r in ("A", "B") and r != me and o.first == me:
        other = 1 - who
        if (o.shield[other] or o.pet < M.PET_REF or o.contact
                or (o.peak[other] > YIELD_SEEN and not o.stopped[who])):
            u -= fault
    return u


def solve(ja, jb, a_acts, b_acts):
    """Pure equilibria, and the mixed ones of support 2 x 2 that no unused action beats."""
    pure = [(a, b) for a, b in itertools.product(a_acts, b_acts)
            if all(ja[(a, b)] >= ja[(x, b)] - 1e-9 for x in a_acts)
            and all(jb[(a, b)] >= jb[(a, y)] - 1e-9 for y in b_acts)]
    mixed = []
    for (a1, a2), (b1, b2) in itertools.product(itertools.combinations(a_acts, 2),
                                                 itertools.combinations(b_acts, 2)):
        den_p = (jb[(a1, b1)] - jb[(a1, b2)]) - (jb[(a2, b1)] - jb[(a2, b2)])
        den_q = (ja[(a1, b1)] - ja[(a2, b1)]) - (ja[(a1, b2)] - ja[(a2, b2)])
        if abs(den_p) < 1e-9 or abs(den_q) < 1e-9:
            continue
        p = (jb[(a2, b2)] - jb[(a2, b1)]) / den_p
        q = (ja[(a2, b2)] - ja[(a1, b2)]) / den_q
        if not (1e-6 < p < 1 - 1e-6 and 1e-6 < q < 1 - 1e-6):
            continue
        va_ = q * ja[(a1, b1)] + (1 - q) * ja[(a1, b2)]
        vb_ = p * jb[(a1, b1)] + (1 - p) * jb[(a2, b1)]
        if any(q * ja[(x, b1)] + (1 - q) * ja[(x, b2)] > va_ + 1e-9 for x in a_acts if x not in (a1, a2)):
            continue
        if any(p * jb[(a1, y)] + (1 - p) * jb[(a2, y)] > vb_ + 1e-9 for y in b_acts if y not in (b1, b2)):
            continue
        mixed.append(({a1: p, a2: 1 - p}, {b1: q, b2: 1 - q}))
    return pure, mixed


def outcomes(dt_ctx, va, vb):
    """Every rule pair's run; B's react continuation is chosen per right of way in play()."""
    outs = {}
    for ra in A_RULES:
        for rb in B_RULES:
            if rb[0] == "react":
                for c in ("yield", "hold"):
                    outs[(ra[0], f"react:{c}")] = simulate(ra, rb, dt_ctx, va, vb, c)
            else:
                outs[(ra[0], rb[0])] = simulate(ra, rb, dt_ctx, va, vb)
    return outs


def play(outs, r, fault=FAULT):
    a_acts = [ra[0] for ra in A_RULES]
    b_acts = [rb[0] for rb in B_RULES]
    res = {}
    for a in a_acts:
        for b in b_acts:
            if b == "react":
                cands = [outs[(a, "react:yield")], outs[(a, "react:hold")]]
                res[(a, b)] = max(cands, key=lambda o: utility(o, 1, r, fault))
            else:
                res[(a, b)] = outs[(a, b)]
    ua = {k: utility(o, 0, r, fault) for k, o in res.items()}
    ub = {k: utility(o, 1, r, fault) for k, o in res.items()}
    pure, mixed = solve(ua, ub, a_acts, b_acts)
    return res, ua, ub, pure, mixed


def label(res, pure, mixed):
    if any(res[p].contact for p in pure):
        return "!"
    if any(res[p].both_stopped for p in pure):
        return "W"
    firsts = {res[p].first for p in pure}
    if len(firsts) > 1:
        return "C"
    if firsts:
        return firsts.pop() or "?"
    return "M" if mixed else "?"


def welfare(res, ua, ub, pure, mixed):
    """Own-utility costs C = -(U_A + U_B) (own utilities, so the fault is a transfer-free cost)."""
    cost = {k: -(ua[k] + ub[k]) for k in res}
    c_opt = min(cost.values())
    eq = [cost[p] for p in pure] + [sum(pa[a] * pb[b] * cost[(a, b)] for a in pa for b in pb)
                                     for pa, pb in mixed]
    return c_opt, (max(eq) if eq else None), (min(eq) if eq else None)


def _context(job):
    dt_ctx, v, vb = job[:3]
    fault = job[3] if len(job) > 3 else FAULT
    outs = outcomes(dt_ctx, v, vb if vb is not None else v)
    rows = []
    for r in ("none", "A", "B"):
        res, ua, ub, pure, mixed = play(outs, r, fault)
        c_opt, c_worst, c_best = welfare(res, ua, ub, pure, mixed)
        firsts = sorted({res[p].first for p in pure})
        f_min = None
        if r != "none":
            for f in FAULTS:
                rf, _, _, pf, _ = play(outs, r, f)
                if pf and all(rf[p].first == r and not rf[p].contact for p in pf):
                    f_min = f
                    break
        rows.append({
            "r": r, "dt": dt_ctx, "v": v, "class": label(res, pure, mixed),
            "fault_min": f_min, "rule_first_eq": sum(res[p].first == r for p in pure) if r != "none" else None,
            "n_pure": len(pure), "n_mixed": len(mixed), "firsts": "".join(firsts),
            "pure": " ".join(f"{a}/{b}" + (f":{res[(a, b)].b_cont}" if b == "react" else "") for a, b in pure),
            "eq_contact": sum(res[p].contact for p in pure),
            "eq_wait": sum(res[p].both_stopped for p in pure),
            "min_pet_eq": min((res[p].pet for p in pure), default=INF),
            "C_opt": c_opt, "C_worst_eq": c_worst, "C_best_eq": c_best,
            "contact_pairs": sum(o.contact for o in res.values()),
        })
    return rows


def run_map(args):
    n = int(round(2 * args.dt_max / args.dt_step))
    dts = [round(-args.dt_max + args.dt_step * i, 3) for i in range(n + 1)]
    vs = [float(x) for x in args.v.split(",")]
    jobs = [(d, v, args.vb, args.fault) for v in vs for d in dts]
    with Pool(args.jobs) as pool:
        rows = [row for rs in pool.map(_context, jobs) for row in rs]
    os.makedirs(args.out, exist_ok=True)
    def fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return "inf" if math.isinf(x) else f"{x:.3f}"
        return x

    with open(os.path.join(args.out, "crossing_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for row in rows:
            w.writerow({k: fmt(v) for k, v in row.items()})
    lab = {(row["r"], row["dt"], row["v"]): row for row in rows}
    lines = [f"# crossing equilibrium map (kinematic): {len(A_RULES)} A rules x {len(B_RULES)} B rules"
             + (f", vB = {args.vb} m/s" if args.vb is not None else ", vA = vB = v") + f", fault {args.fault:g}",
             f"# columns: dt = t_A^Z - t_B^Z from {dts[0]} to {dts[-1]} s, step {args.dt_step:g} (dt > 0: B nearer)",
             "# A / B: that car passes first in every equilibrium   C: chicken (both orders are equilibria)",
             "# W: war of attrition (an equilibrium with both stopped)   !: contact in equilibrium",
             "# M: mixed equilibria only   ?: none"]
    for r in ("none", "A", "B"):
        lines.append(f"\n## right of way: {r}")
        lines.append("  v(m/s) " + "".join(f"{d:+5.1f}"[-4:].rjust(5) for d in dts))
        for v in reversed(vs):
            lines.append(f"  {v:6.1f} " + "".join(lab[(r, d, v)]["class"].rjust(5) for d in dts))
        if r != "none":
            lines.append(f"  smallest fault (of {FAULTS}) after which every equilibrium lets {r} through first (- : none)")
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
    with open(os.path.join(args.out, "crossing_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    try:
        plot(args.out, dts, vs, lab)
    except ImportError:
        print("matplotlib missing: no PNG", file=sys.stderr)
    return 0


def plot(out, dts, vs, lab):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    order = "ABCWM!?"
    names = ["A first", "B first", "chicken (both orders)", "war of attrition (both wait)",
             "mixed only", "contact in equilibrium", "none"]
    colours = ["#2a9d8f", "#457b9d", "#e9c46a", "#6a4c93", "#f4a261", "#e63946", "#999999"]
    fig, axes = plt.subplots(1, 3, figsize=(14, 3.8), sharey=True)
    step = (dts[1] - dts[0]) if len(dts) > 1 else 0.5
    for ax, r in zip(axes, ("none", "A", "B")):
        z = [[order.index(lab[(r, d, v)]["class"]) for d in dts] for v in vs]
        dv = (vs[1] - vs[0]) if len(vs) > 1 else 1.0
        ax.imshow(z, origin="lower", aspect="auto", cmap=ListedColormap(colours), vmin=-0.5,
                  vmax=len(order) - 0.5, extent=(dts[0] - step / 2, dts[-1] + step / 2, vs[0] - dv / 2, vs[-1] + dv / 2))
        ax.set_title(f"right of way: {r}")
        ax.set_xlabel("dt = t_A^Z - t_B^Z (s)")
    axes[0].set_ylabel("v (m/s)")
    present = sorted({row["class"] for row in lab.values()}, key=order.index)
    axes[-1].legend(handles=[Patch(color=colours[order.index(c)], label=names[order.index(c)]) for c in present],
                    loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    fig.suptitle("Unsignalled crossing: equilibrium classes (kinematic)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "crossing_map.png"), dpi=120)


def selftest():
    go, never = A_RULES[0], next(r for r in A_RULES if r[1] == INF and not r[2] and r[3] == PATIENCE[1])
    # B far ahead: whatever A does, B passes first and nobody touches
    for ra in (go, never):
        o = simulate(ra, HOLD, 4.0, 10.0, 10.0)
        assert not o.contact and o.first == "B", (ra[0], o)
    # both going at the same time is a crash or a near miss
    o = simulate(go, HOLD, 0.0, 10.0, 10.0)
    assert o.contact or o.pet < 1.0, o
    # A yielding to a holding B lets B through first, without contact
    o = simulate(never, HOLD, 0.0, 10.0, 10.0)
    assert not o.contact and o.first == "B", o
    # both waiting forever is a deadlock until patience runs out; patience ends it
    o = simulate(next(r for r in A_RULES if r[1] == INF and r[3] == PATIENCE[0]), B_RULES[2], 0.0, 8.0, 8.0)
    assert o.both_stopped and o.first in "AB", o
    # A's gap test: a 3 s gap is enough for tau 1.5, not for tau 3 (B's arrival 3 s after A's exit)
    t_ex = t_cover(EXIT, 10.0, 10.0)
    assert abs(t_ex - EXIT / 10.0) < 1e-9 and abs(t_in(-30.0, 10.0) - 3.0) < 1e-9
    # the solver on textbook chicken: two pure and one mixed
    ja = {("g", "y"): 1, ("g", "h"): -10, ("w", "y"): 0, ("w", "h"): 0}
    jb = {("g", "y"): 0, ("g", "h"): -10, ("w", "y"): 0, ("w", "h"): 1}
    pure, mixed = solve(ja, jb, ("g", "w"), ("y", "h"))
    assert set(pure) == {("g", "y"), ("w", "h")} and len(mixed) == 1, (pure, mixed)
    # the fault falls only on the car without right of way that forced the other
    o = simulate(go, B_RULES[2], 0.0, 10.0, 10.0)
    assert utility(o, 0, "B") <= utility(o, 0, "none") and utility(o, 0, "A") == utility(o, 0, "none")
    # right of way to A at dt = 0: every pure equilibrium lets A through first
    rows = {row["r"]: row for row in _context((0.0, 10.0, None))}
    assert rows["none"]["class"] == "C", rows["none"]  # chicken at a tie, without a rule
    # A clearly nearer: A first, and a right of way to A needs no enforcement there
    rows = {row["r"]: row for row in _context((-1.0, 10.0, None))}
    assert rows["none"]["class"] == "A" and rows["A"]["fault_min"] == 0.0, rows
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="crossing_out")
    p.add_argument("--v", default="5,8,11,14", help="speeds (m/s), comma-separated")
    p.add_argument("--vb", type=float, default=None, help="fix B's speed (m/s); default: B as fast as A")
    p.add_argument("--dt-max", type=float, default=4.0, help="dt from -dt_max to dt_max (s)")
    p.add_argument("--dt-step", type=float, default=0.5)
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--fault", type=float, default=FAULT, help="cost of failing to yield (classes and welfare)")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
