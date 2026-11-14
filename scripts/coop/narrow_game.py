#!/usr/bin/env python3
"""Roadmap 074 S4: the narrow passage, the first "oppose" relation, as a passage game in a kinematic model.

A two-way road narrowed to one lane over W_n metres: an obstacle (a parked car, roadworks, a bridge)
closes A's lane, so A swerves into B's lane over TAPER m before it and back after it; A and B
approach from opposite ends and only one fits at a time. The conflict zone Z is the stretch where
A's box overlaps B's lane: the obstacle plus Z_PAD m of each taper. Each car's u is its front's
distance past its own end of Z. The context is

  dt   t_A^Z - t_B^Z, each car's time to Z at its own speed (s); dt > 0: B is nearer
  v    each car's speed (m/s); --vb fixes B's
  W_n  the obstacle's length (m)
  r    right of way: none, or B (the obstacle is on A's side, and that side yields, as in many codes)

Both cars choose from the same rules, crossing_game's (S2) on a zone W_n + 2 Z_PAD long:

  go           enter at speed
  t<tau>[y]p<P> enter iff the gap test passes (A clears Z tau s before the other arrives, or the
               other has left Z tau s before A, going now, arrives), judged from the other as seen
               OBS + DECIDE s ago and predicted to now; else hold out and brake for the line only
               when it must; y: re-run the test every step; patience P: stopped for P s, go anyway
  flash        the courtesy flash: DECIDE s in, if the other is not seen braking (it is committed),
               yield until it has cleared Z (patience 2 s); if it is seen braking, go
  yb<b>p<P>    yield iff the two arrival times are within b s, until the other has cleared Z or for P s

Under every rule each car's safety layer brakes, REACT s after it sees the other's body in Z or
about to enter it first. Backing up is out of scope.

Payoffs: the general utility (measures.py) on oriented boxes, so 2D TTC is at the closing speed
v_A + v_B and clearance is the boxes'; with the right of way, A pays FAULT when it enters first and
forces B (crossing_game.utility). Per context, the game is solved (pure equilibria and 2 x 2 mixed),
each context is classed (crossing_game.label: A / B / C chicken / W war of attrition / ! contact /
M / ?), and under r = B the fault is swept for the smallest under which every equilibrium lets B
through first. Two further risks are counted: a deadlock (nobody through Z by T_MAX) and a restart
crash (contact after both had stopped and waited, i.e. both going on patience at once). These are
counted in the equilibria and over all rule pairs.

Usage:
  scripts/coop/narrow_game.py --out <dir> [--v 6,10] [--wn 6,30] [--dt-max 4] [--dt-step 0.5] [--car-lags] [--jobs N]
  scripts/coop/narrow_game.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import measures as M  # noqa: E402

DT, HORIZON, T_MAX = X.DT, X.HORIZON, X.T_MAX
L, W_CAR = X.L, X.W_CAR
LANE_W = 3.5
TAPER = 10.0  # m, A's swerve into B's lane before the obstacle, and back after it
Z_PAD = TAPER * (1.0 - (LANE_W - W_CAR) / LANE_W)  # m of each taper where A's box is in B's lane
INF = math.inf
FAULT, FAULTS = X.FAULT, X.FAULTS
FLASH = ("react", None, None)  # crossing_game's react, continued with a yield: the courtesy flash
RULES = [r[0] for r in X.A_RULES] + ["flash"] + [r[0] for r in X.B_RULES if r[0].startswith("yb")]


def rule_of(name):
    """(role, rule, b_cont) for crossing_game.Policy."""
    if name == "flash":
        return "B", FLASH, "yield"
    for r in X.A_RULES:
        if r[0] == name:
            return "A", r, "hold"
    return "B", next(r for r in X.B_RULES if r[0] == name), "hold"


@dataclass
class Out:
    o: X.Outcome  # crossing_game's outcome: lost, peak, contact, ttc, ..., first, both_stopped, shield, stopped
    deadlock: bool  # nobody through Z by T_MAX, without contact
    restart: bool  # contact after both had stopped and waited


def lat_a(xc, wz):
    """A's lateral position (B's lane at +LANE_W/2, A's at -LANE_W/2) at its centre's x; Z is [0, wz]."""
    x0, x1 = Z_PAD - TAPER, wz - Z_PAD  # taper start; the obstacle's far end
    if xc <= x0 or xc >= x1 + TAPER:
        return -LANE_W / 2
    if xc < Z_PAD:
        f = (xc - x0) / TAPER
    elif xc <= x1:
        f = 1.0
    else:
        f = 1.0 - (xc - x1) / TAPER
    return -LANE_W / 2 + LANE_W * f


def simulate(na, nb, dt_ctx, va, vb, wn, lags=None, trace=None):
    wz = wn + 2 * Z_PAD
    exit_ = wz + L
    ta, tb = X.T_MEAN + dt_ctx / 2, X.T_MEAN - dt_ctx / 2
    A, B = X.Car(-va * ta, va, lags), X.Car(-vb * tb, vb, lags)
    role_a, ra, ca = rule_of(na)
    role_b, rb, cb = rule_of(nb)
    pa_, pb_ = X.Policy(role_a, ra, va, ca, exit_), X.Policy(role_b, rb, vb, cb, exit_)
    meas = M.Measures(b_max=(X.B_MAX, X.B_MAX), pet_sample=X.PET_SAMPLE)
    seen_in: dict = {"A": None, "B": None}
    stopped_both, both_stopped, contact, restart = 0.0, False, False, False
    t = 0.0
    while t < T_MAX - 1e-9:
        pa_.perceive(t, B.u, B.v, B.a)
        pb_.perceive(t, A.u, A.v, A.a)
        A.a = pa_.step(t, A.u, A.v)
        B.a = pb_.step(t, B.u, B.v)
        for me in (A, B):
            if me.act is not None:
                me.a = me.act.step(me.a)
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
        if trace is not None:
            trace.append((round(t, 2), A.u, A.v, A.a, pa_.committed, B.u, B.v, B.a, pb_.committed))
        t += DT
        for name, me in (("A", A), ("B", B)):
            if me.u >= 0 and seen_in[name] is None:
                seen_in[name] = t
        if A.v < X.WAIT_V and B.v < X.WAIT_V and A.u < 0 and B.u < 0:
            stopped_both += DT
            both_stopped = both_stopped or stopped_both >= 0.5
        else:
            stopped_both = 0.0
        # A along +x (front at x = u_A), B along -x (front at x = wz - u_B, in its own lane)
        xa = A.u - L / 2
        ya = lat_a(xa, wz)
        dy = (lat_a(xa + 0.5, wz) - lat_a(xa - 0.5, wz))
        ha = math.atan2(dy, 1.0)
        pa = (xa, ya, ha, A.v * math.cos(ha), A.v * math.sin(ha), L, W_CAR)
        pb = (wz - B.u + L / 2, LANE_W / 2, math.pi, -B.v, 0.0, L, W_CAR)
        meas.push_poses(t, pa, pb)
        if meas.clear <= 0.0:
            contact = True
            restart = both_stopped
            break
        if A.u >= exit_ + TAPER and B.u >= exit_ + TAPER and t >= HORIZON - 1e-9:
            break
    lost = tuple(1.0 if contact or me.u < exit_ else M.lost_progress(me.v0, meas.s_h[k]) for k, me in enumerate((A, B)))
    first = ""
    if seen_in["A"] is not None or seen_in["B"] is not None:
        first = "A" if (seen_in["A"] or INF) < (seen_in["B"] or INF) else "B"
    o = X.Outcome(lost, (A.peak, B.peak), contact, meas.ttc2d, meas.thw, meas._pet(), meas.clear, first,
                  both_stopped, "", (pa_.shield, pb_.shield), (pa_.rested, pb_.rested), (A.peak_w, B.peak_w))
    deadlock = not contact and A.u < exit_ and B.u < exit_
    return Out(o, deadlock, restart)


def play(outs, r, fault=FAULT):
    res = {k: v.o for k, v in outs.items()}
    ua = {k: X.utility(o, 0, r, fault) for k, o in res.items()}
    ub = {k: X.utility(o, 1, r, fault) for k, o in res.items()}
    pure, mixed = X.solve(ua, ub, RULES, RULES)
    return res, ua, ub, pure, mixed


def context(job):
    dt_ctx, va, vb, wn, fault, lags = job
    outs = {(a, b): simulate(a, b, dt_ctx, va, vb, wn, lags) for a in RULES for b in RULES}
    rows = []
    allp = list(outs.values())
    for r in ("none", "B"):
        res, ua, ub, pure, mixed = play(outs, r, fault)
        c_opt, c_worst, c_best = X.welfare(res, ua, ub, pure, mixed)
        f_min = None
        if r != "none":
            for f in FAULTS:
                rf, _, _, pf, _ = play(outs, r, f)
                if pf and all(rf[p].first == r and not rf[p].contact for p in pf):
                    f_min = f
                    break
        rows.append({
            "r": r, "dt": dt_ctx, "v": va, "vb": vb, "wn": wn, "class": X.label(res, pure, mixed),
            "fault_min": f_min, "n_pure": len(pure), "n_mixed": len(mixed),
            "firsts": "".join(sorted({res[p].first for p in pure})),
            "rule_first_eq": sum(res[p].first == r for p in pure) if r != "none" else None,
            "eq_contact": sum(res[p].contact for p in pure), "eq_wait": sum(res[p].both_stopped for p in pure),
            "eq_deadlock": sum(outs[p].deadlock for p in pure), "eq_restart": sum(outs[p].restart for p in pure),
            "C_opt": c_opt, "C_worst_eq": c_worst, "C_best_eq": c_best,
            "pairs_contact": sum(o.o.contact for o in allp), "pairs_wait": sum(o.o.both_stopped for o in allp),
            "pairs_deadlock": sum(o.deadlock for o in allp), "pairs_restart": sum(o.restart for o in allp),
            "pairs": len(allp),
            "pure": " ".join(f"{a}/{b}" for a, b in pure),
        })
    return rows


def run_map(args):
    from multiprocessing import Pool

    lags = X.car_lags() if args.car_lags else None
    n = int(round(2 * args.dt_max / args.dt_step))
    dts = [round(-args.dt_max + args.dt_step * i, 3) for i in range(n + 1)]
    vs = [float(x) for x in args.v.split(",")]
    wns = [float(x) for x in args.wn.split(",")]
    jobs = [(dt, v, args.vb if args.vb else v, wn, args.fault, lags) for wn in wns for v in vs for dt in dts]
    with Pool(args.jobs) as pool:
        rows = [r for rs in pool.map(context, jobs) for r in rs]
    os.makedirs(args.out, exist_ok=True)

    def fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return "inf" if math.isinf(x) else f"{x:.4g}"
        return x

    with open(os.path.join(args.out, "narrow_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(v) for k, v in r.items()})
    by = {(r["r"], r["wn"], r["v"], r["dt"]): r for r in rows}
    lines = [f"# narrow passage (kinematic): {len(RULES)} rules each, " + ("the car's command lags" if lags else "an ideal car")
             + f"; Z = W_n + {2 * Z_PAD:.1f} m; FAULT {args.fault}",
             "# class per dt (s, t_A - t_B; > 0: B nearer): A/B one car first in every equilibrium, C chicken, W war of",
             "# attrition (an equilibrium where both stop and wait), ! contact in an equilibrium, M mixed only",
             "# r = B: the obstacle is on A's side and A yields; f_min: the smallest fault under which every equilibrium",
             "# lets B through first (- : none of the swept faults up to 8)"]
    lines.append("dt:            " + " ".join(f"{d:>5g}" for d in dts))
    for wn in wns:
        for v in vs:
            for r in ("none", "B"):
                lines.append(f"W_n {wn:3g} v {v:3g} r {r:4} " + " ".join(f"{by[(r, wn, v, d)]['class']:>5}" for d in dts))
            fm = [by[("B", wn, v, d)]["fault_min"] for d in dts]
            lines.append(f"W_n {wn:3g} v {v:3g} f_min  " + " ".join(f"{'-' if x is None else f'{x:g}':>5}" for x in fm))
    tot = lambda k, r: sum(x[k] for x in rows if x["r"] == r)  # noqa: E731
    nr = sum(1 for x in rows if x["r"] == "none")
    lines.append("")
    lines.append(f"# over all rule pairs ({tot('pairs', 'none')} runs): contact {tot('pairs_contact', 'none')}, both waited "
                 f"{tot('pairs_wait', 'none')}, deadlock {tot('pairs_deadlock', 'none')}, restart crash {tot('pairs_restart', 'none')}")
    for r in ("none", "B"):
        cls = [x["class"] for x in rows if x["r"] == r]
        lines.append(f"# r {r}: classes " + ", ".join(f"{c} {cls.count(c)}" for c in "ABCW!M?" if cls.count(c))
                     + f" of {nr}; equilibria with contact {tot('eq_contact', r)}, waiting {tot('eq_wait', r)}, "
                     f"deadlock {tot('eq_deadlock', r)}, restart crash {tot('eq_restart', r)}")
    sel = [x for x in rows if x["r"] == "B"]
    lines.append(f"# r B selects (some swept fault makes every equilibrium B-first) in {sum(x['fault_min'] is not None for x in sel)} "
                 f"of {len(sel)} contexts; at FAULT {args.fault}, every equilibrium B-first in "
                 f"{sum(x['rule_first_eq'] == x['n_pure'] and x['n_pure'] > 0 for x in sel)}")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "narrow_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    return 0


def selftest():
    assert abs(Z_PAD - 6.18) < 0.01, Z_PAD
    assert lat_a(-50, 30) == -LANE_W / 2 and lat_a(15, 30) == LANE_W / 2
    # both go at a tie on a long passage: head-on contact
    o = simulate("go", "go", 0.0, 8.0, 8.0, 30.0)
    assert o.o.contact, o
    # A always yields: B first, no contact
    o = simulate("tinfp5", "go", 0.0, 8.0, 8.0, 30.0)
    assert o.o.first == "B" and not o.o.contact, o
    # far apart, both pass, no wait; a parked car's length
    o = simulate("t1.5p2", "t1.5p2", -4.0, 8.0, 8.0, 6.0)
    assert not o.o.contact and not o.deadlock and o.o.first == "A", o
    # both flash at a tie: both yield, wait, then patience resolves it (or crashes)
    o = simulate("flash", "flash", 0.0, 8.0, 8.0, 30.0)
    assert o.o.both_stopped, o
    rows = context((0.0, 8.0, 8.0, 30.0, FAULT, None))
    assert rows[0]["n_pure"] + rows[0]["n_mixed"] > 0, rows
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="narrow_out")
    p.add_argument("--v", default="6,10")
    p.add_argument("--vb", type=float, default=None)
    p.add_argument("--wn", default="6,30")
    p.add_argument("--dt-max", type=float, default=4.0)
    p.add_argument("--dt-step", type=float, default=0.5)
    p.add_argument("--fault", type=float, default=FAULT)
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--jobs", type=int, default=16)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
