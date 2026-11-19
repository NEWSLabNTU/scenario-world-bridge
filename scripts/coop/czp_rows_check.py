#!/usr/bin/env python3
"""Roadmap 078 V4: the other rows of the convention table, checked on the czp library itself.

c8_fourway_check.py model-checks the all-way stop's rules abstracted to their decision logic. This sibling
runs the built library (czp.Protocol, through the arena adapter's acts, czp_adapter.py) in a discrete-time
model (tick 0.05 s, crossing_game's car with the spec's command lags and launch dead time) and explores an
adversary family of perception outcomes exhaustively per context: every car reads every conflicting car

  - delayed by one of DELAYS (OBS + DECIDE, and one 10 Hz sample later), predicted to now at the seen speed,
    as the adapters do;
  - (three cars at the crossing: a car also sees the car it does not conflict with, the opposite one, as a
    bystander whose start, entry and exit are junction events: czp.Protocol.step(bystanders=), --no-bystanders
    to leave it out)
  - with a time error within +-e on the other's arrival (its seen position off by e(t) v): constant -e, 0
    or +e, or a square wave between -e and +e of period 0.3 s or 0.7 s (a reading that flips sides of the
    margin while the cars approach).

Two cars: all 10 x 10 combinations per context (5 error schedules x 2 delays per observer). Three cars: the
error schedules -e, +e and a 0.5 s square wave on every conflicting ordered pair, at the first delay.
Unlike c8_fourway_check this is exhaustive over that family, not over every timing inside the bounds.

Rows (the zone's kind, conventions/generic.yaml):
  cross    cross_unsignalled: band "entry" (nearer first), tie right-before-left, then map_order. A along +x
           (order 0), B from A's right (order 1); three cars: C from the east (order 2), on B's right; A and C
           (opposite) do not conflict. Zone: the 3.5 m box, crossing_game's EXIT.
  narrow   narrow_passage: band "entry", the obstructed side (A's, west) yields, tie map_order. A from the
           west, B from the east, the zone narrow_game's W_n + 2 Z_PAD; three cars: A1 and A2 (following A1
           at a time gap) against B.
  merge    merge_lane_drop: band "gap" (who leads whom by a headway), ahead_at_end, the through lane (B)
           first in the map's order; both cars czp (czp_adapter's CZPMergeRulesAB), merge_game's world (the
           lane change, the lane's end). Two cars only: merge_game is a pair model.

No post-start shield: the study's safety layer is off (as c8_fourway_check), so a violation is a run in which
the protocol left safety to the AEB.

Adversaries (--adversary, the crossing with two cars; 078 P2e): one car is not czp's, the other is, in either place,
against the same perception family on the czp car's side. Every czp car before 078 P2e met only other czp cars,
which never restart after a mutual rest, as S2's impatient and never-yield rules do (078 P2d):
  impatient  ignores the rule: brakes for its line and waits there until the other is through, but restarts once
             both cars have been at rest for --adv-patience s (0.5) as it sees them (OBS + DECIDE late); no safety
             layer: a contact with it is the czp car's go
  never      never yields: drives through at its speed, stopped only by its own safety layer (crossing_game's,
             REACT s late), and restarts when that releases

Properties per context, over the adversary family:
  safety    cross / narrow: never two conflicting cars in the zone at once; merge: no contact and no run-out
            (A's front past the lane's end less than half across);
  liveness  every car through the zone (merge: A merged) by T_MAX (40 s); else a deadlock (every car left
            at rest) or a livelock;
  bound     the worst wait (time below WAIT_V before the zone) over every run.

Usage:
  scripts/coop/czp_rows_check.py --row cross|narrow|merge --cars 2|3 --out <dir> [--e 0.25] [--jobs 24]
  scripts/coop/czp_rows_check.py --row cross --cars 2 --adversary both --out <dir> [--e 0.55] [--adv-patience 0.5]
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from collections import Counter
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import crossing_game as X  # noqa: E402
import cutin_game as C1  # noqa: E402
import czp_adapter as CA  # noqa: E402

czp = CA.czp
CM = CA.CM
DT = X.DT
T_MAX = 40.0
DELAYS = (X.OBS + X.DECIDE, X.OBS + X.DECIDE + 0.1)
MODES2 = ("-e", "0", "+e", "sq0.3", "sq0.7")
MODES3 = ("-e", "+e", "sq0.5")
LAT_GAP = 2.0  # m, the standstill gap of a follower


def err(mode, e, t):
    if mode == "0":
        return 0.0
    if mode == "-e":
        return -e
    if mode == "+e":
        return e
    per = float(mode[2:])
    return e if int(math.floor(t / per)) % 2 == 0 else -e


class Car:
    def __init__(self, name, approach, u0, v0, exit_, lags, kind, zone, leader=None):
        self.name, self.approach, self.exit, self.leader = name, approach, exit_, leader
        self.u, self.v, self.v0, self.a = u0, v0, v0, 0.0
        self.act = X.Actuator(lags)
        self.hist = [(0.0, u0, v0, 0.0)]
        self.rest_starts = []  # when the car came to rest (078 P2g: REST_LAG)
        self.wait = 0.0
        self.t_in = None
        p = CA.params_for(True, CA.BIAS["value"], None)
        self.p = p
        self.proto = czp.Protocol(CA.conventions_for(True), p)
        self.cap = p.capability("car", v_max=v0, length=X.L)
        self.zone = zone

    def seen(self, t, delay, e):
        i = len(self.hist) - 1
        while i > 0 and self.hist[i][0] > t - delay + 1e-9:
            i -= 1
        h = self.hist[i]
        stamp, u, v, a = h
        if X.REST_LAG > 0 and v < 0.05:  # 078 P2g: the tracker still sees a car that has just come to rest rolling
            # its rest began at the last rest start at or before the stamp (kept apart from hist, which is trimmed:
            # read from hist, a long rest looked 0.76 s old after every trim, a phantom start every 0.5 s)
            began = max((s for s in self.rest_starts if s <= stamp + 1e-9), default=-math.inf)
            if stamp - began < X.REST_LAG - 1e-9:
                v = max(v, X.REST_HOLD_V)
        u_now = u + v * (t - stamp) - e * v  # a later arrival by e: further back by e v
        return stamp, u_now, v, a


class Adversary:
    """A car that is not czp's (--adversary): ``impatient`` or ``never``; same interface as Car for run_passage."""

    def __init__(self, name, approach, u0, v0, exit_, lags, kind, patience):
        self.name, self.approach, self.exit, self.leader = name, approach, exit_, None
        self.u, self.v, self.v0, self.a = u0, v0, v0, 0.0
        self.act = X.Actuator(lags)
        self.hist = [(0.0, u0, v0, 0.0)]
        self.rest_starts = []  # when the car came to rest (078 P2g: REST_LAG)
        self.wait = 0.0
        self.t_in = None
        self.kind, self.patience = kind, patience
        self.go = False
        self.mutual_since = None
        self.p = CA.params_for(True, 0.0, None)
        self.cap = self.p.capability("car", v_max=v0, length=X.L)
        if kind == "never":
            role = "A" if approach == "A" else "B"
            rule = ("go", -math.inf, False, math.inf) if role == "A" else ("hold", None, None)
            self.pol = X.Policy(role, rule, v0, "hold", exit_)

    seen = Car.seen

    def command(self, t, other):
        """The acceleration this car commands at t, from what it sees of the other car (OBS + DECIDE late, no error)."""
        if self.kind == "never":
            self.pol.perceive(t, other.u, other.v, other.a)  # truth stamped now: the policy applies its own delays
            return self.pol.step(t, self.u, self.v)
        stamp, ou, ov, _ = other.seen(t, X.OBS + X.DECIDE, 0.0)
        if not self.go:
            if ou >= other.exit:
                self.go = True  # the other is through
            elif self.v < 0.1 and self.u < 0 and ov < 0.1 and ou < 0:
                self.mutual_since = t if self.mutual_since is None else self.mutual_since
                if t - self.mutual_since >= self.patience - 1e-9:
                    self.go = True  # both at rest: it restarts after its patience, whatever the rule says
            else:
                self.mutual_since = None
        return X.drive(self.v, self.v0) if self.go else X.hold_out(self.u, self.v)


def zone_of(row, n):
    if row == "cross":
        aps = [CM.Approach("A", heading=0.0, order=0), CM.Approach("B", heading=math.pi / 2, order=1)]
        if n == 3:
            aps.append(CM.Approach("C", heading=math.pi, order=2))
        return CM.Zone("z0", CM.CROSS_UNSIGNALLED, tuple(aps))
    return CM.Zone("z0", CM.NARROW_PASSAGE, (CM.Approach("A", heading=0.0, order=0, obstructed=True),
                                             CM.Approach("B", heading=math.pi, order=1)))


def czp_lags(lags):
    """A czp car's lags: with Autoware's launch (--autoware-launch, crossing_game.AUTOWARE_LAUNCH; 078 P2g) when set.
    The adversary keeps the arena car's."""
    if X.AUTOWARE_LAUNCH is None:
        return lags
    dead, accel, v_end = X.AUTOWARE_LAUNCH
    return dict(lags, launch_dead_time=dead, launch_accel=accel, launch_v_end=v_end)


def run_passage(row, ctx, adv, e, lags, trace=None, bystanders=True, adversary=None):
    """One run of the crossing or the narrow passage. ctx: (v, wn, arrivals {name: (approach, t_arrive)},
    leaders {name: leader}); adv: {(i, j): (mode, delay)}; adversary: None or (name, kind, patience), that car not
    czp's (Adversary)."""
    import narrow_game as N
    v, wn, arr, leaders = ctx
    n = len(arr)
    zone = zone_of(row, 3 if row == "cross" and n == 3 else 2)
    exit_ = X.EXIT if row == "cross" else wn + 2 * N.Z_PAD + X.L
    cars = {}
    for name, (ap, ta) in arr.items():
        if adversary is not None and name == adversary[0]:
            cars[name] = Adversary(name, ap, -v * ta, v, exit_, lags, adversary[1], adversary[2])
        else:
            cars[name] = Car(name, ap, -v * ta, v, exit_, czp_lags(lags), row, zone, leaders.get(name))
    conflicts = {frozenset((i, j)) for i in cars for j in cars if i < j and conflict(row, cars[i].approach, cars[j].approach)}
    t = 0.0
    unsafe = None
    while t < T_MAX - 1e-9:
        cmd = {}
        for c in cars.values():
            if isinstance(c, Adversary):
                cmd[c.name] = c.command(t, next(o for o in cars.values() if o is not c))
                if trace is not None:
                    trace.append((round(t, 2), c.name, round(c.u, 2), round(c.v, 2), c.kind, "go" if c.go else ""))
                continue
            own = CM.Agent(c.name, approach=c.approach, s_entry=-c.u, s_exit=c.exit - c.u, v=c.v, a=c.a, stamp=t, cap=c.cap)
            others, bys, age = [], [], 0.0
            for o in cars.values():
                if o is c or o.approach == c.approach:
                    continue
                mode, delay = adv.get((c.name, o.name), ("0", DELAYS[0]))
                stamp, ou, ov, oa = o.seen(t, delay, err(mode, e, t))
                ag = CM.Agent(o.name, approach=o.approach, s_entry=-ou, s_exit=o.exit - ou, v=ov, a=oa, stamp=stamp,
                              cap=o.cap)
                if frozenset((c.name, o.name)) in conflicts:
                    age = max(age, t - stamp)
                    others.append(ag)
                elif bystanders:
                    bys.append(ag)  # at the junction, not in the conflict set: its events count for the quiet epoch
            inputs = czp.contracts.Inputs(obs_age=min(age, X.OBS))
            dec, _ = c.proto.step(t, own, others, zone, inputs=inputs, obs_age=age, bystanders=bys)
            if dec.kind in (CM.GO, CM.STOCK):
                a = X.drive(c.v, c.v0)
            elif dec.kind == CM.CREEP:
                a = CA.creep_accel(c.u, c.v, c.p, None if dec.reason.endswith("creep") else c.p.probe_watch_speed)
            else:
                a = X.hold_out(c.u, c.v)
            if c.leader is not None:
                ld = cars[c.leader]
                a = min(a, C1.follow_accel(ld.u - X.L - c.u, c.v, ld.v, C1.TYPES["car"], c.v0))
            cmd[c.name] = a
            if trace is not None:
                trace.append((round(t, 2), c.name, round(c.u, 2), round(c.v, 2), dec.kind, dec.reason))
        for c in cars.values():
            c.a = c.act.step(cmd[c.name], c.v)
            c.v = max(0.0, c.v + c.a * DT)
            c.u += c.v * DT
            if c.v < X.WAIT_V and c.u < 0:
                c.wait += DT
            if c.u >= 0 and c.t_in is None:
                c.t_in = t
        t += DT
        for c in cars.values():
            if c.v < 0.05 and (not c.hist or c.hist[-1][2] >= 0.05):
                c.rest_starts.append(t)
            c.hist.append((t, c.u, c.v, c.a))
            if len(c.hist) > 40:
                del c.hist[:10]
        for pair in conflicts:
            i, j = tuple(pair)
            if 0 <= cars[i].u < cars[i].exit and 0 <= cars[j].u < cars[j].exit:
                unsafe = unsafe or (round(t, 2), i, j)
        follow_contact = any(c.leader is not None and cars[c.leader].u - X.L - c.u <= 0 for c in cars.values())
        if unsafe or follow_contact:
            break
        if all(c.u >= c.exit for c in cars.values()):
            break
    done = all(c.u >= c.exit for c in cars.values())
    rest = all(c.v < 0.1 for c in cars.values() if c.u < c.exit)
    return {"unsafe": unsafe, "follow_contact": bool(follow_contact), "live": done,
            "deadlock": (not done and not unsafe and rest), "wait": max(c.wait for c in cars.values()),
            "order": "".join(sorted(cars, key=lambda k: first_in(cars[k])))}


def first_in(c):
    return c.t_in if c.t_in is not None else math.inf


def conflict(row, ai, aj):
    if ai == aj:
        return False
    if row == "cross":
        return frozenset((ai, aj)) != frozenset("AC")
    return True


def run_merge(ctx, adv, e, lags):
    """Both cars czp in merge_game's world; the adversary on what each sees of the other."""
    import merge_game as MG
    d_end, d0, dv, vb = ctx
    (ma, da), (mb, _) = adv[("A", "B")], adv[("B", "A")]
    fac = CA.merge_rules_factory("czp", "AB")
    obs0 = MG.OBS

    def make(*a, **k):
        R = fac(*a, **k)
        step = R.step

        def step_adv(t, xa, va, lat, xb, vb_, seen_b, seen_a):
            xb_s, vb_s, ab_s = seen_b
            xa_s, va_s, lat_s = seen_a
            sb = (xb_s - err(ma, e, t) * vb_s, vb_s, ab_s)
            sa = (xa_s - err(mb, e, t) * va_s, va_s, lat_s)
            return step(t, xa, va, lat, xb, vb_, sb, sa)
        R.step = step_adv
        return R

    # merge_game has one perception delay for both cars: the adversary's delay is A's (B's is the same)
    MG.OBS = obs0 + (da - DELAYS[0])
    MG._orig_rules = getattr(MG, "_orig_rules", MG.Rules)
    orig = MG.Rules
    MG.Rules = make
    try:
        o = MG.simulate(CA.MERGE_CZP_A, CA.MERGE_CZP_B, d_end, d0, dv, vb, "hold", lags)
    finally:
        MG.Rules = orig
        MG.OBS = obs0
    unsafe = (0.0, "A", "B") if o.contact else None
    return {"unsafe": unsafe, "follow_contact": False, "live": o.merged, "deadlock": (not o.merged and not o.contact),
            "wait": 0.0, "order": "AB" if o.ahead else "BA", "stopped": o.stopped, "ran_out": o.ran_out}


def advs(names, conflicts, ncars, merge=False):
    """The adversary family: {(i, j): (mode, delay)} per conflicting ordered pair."""
    pairs = [(i, j) for i in names for j in names if i != j and frozenset((i, j)) in conflicts]
    if ncars == 2 and conflicts == {frozenset("AB")} and merge:
        for ma, mb, d in itertools.product(MODES2, MODES2, DELAYS):  # one delay for both (merge_game)
            yield {("A", "B"): (ma, d), ("B", "A"): (mb, d)}
        return
    if ncars == 2:
        opts = [(m, d) for m in MODES2 for d in DELAYS]
    else:
        opts = [(m, DELAYS[0]) for m in MODES3]
    for combo in itertools.product(opts, repeat=len(pairs)):
        yield dict(zip(pairs, combo))


def contexts(args):
    out = []
    dts2 = sorted({round(-3 + 0.1 * i, 2) for i in range(61)} | {-4.0, 4.0})
    dts3 = (-1.5, -1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0, 1.5)
    T = X.T_MEAN
    if args.row == "cross" and args.cars == 2:
        for v in (6.0, 10.0, 14.0):
            for dt in dts2:
                out.append(("cross", (v, 0.0, {"A": ("A", T + dt / 2), "B": ("B", T - dt / 2)}, {}), ("A", "B"), dt))
    elif args.row == "cross":
        for dab in dts3:
            for dcb in dts3:
                out.append(("cross", (10.0, 0.0, {"A": ("A", T + dab), "B": ("B", T), "C": ("C", T + dcb)}, {}),
                            ("A", "B", "C"), (dab, dcb)))
    elif args.row == "narrow" and args.cars == 2:
        for wn in (6.0, 30.0):
            for v in (6.0, 10.0):
                for dt in dts2:
                    out.append(("narrow", (v, wn, {"A": ("A", T + dt / 2), "B": ("B", T - dt / 2)}, {}), ("A", "B"),
                                (wn, v, dt)))
    elif args.row == "narrow":
        for wn in (6.0, 30.0):
            for v in (6.0, 10.0):
                for dt in dts3:
                    for hw in (1.0, 2.0):  # A2's time gap behind A1 (front to front less a length)
                        ta1 = T + dt / 2
                        ta2 = ta1 + hw + (X.L + LAT_GAP) / v
                        out.append(("narrow", (v, wn, {"A1": ("A", ta1), "A2": ("A", ta2), "B": ("B", T - dt / 2)},
                                               {"A2": "A1"}), ("A1", "A2", "B"), (wn, v, dt, hw)))
    else:  # merge, two cars
        for d_end in (40.0, 80.0):
            for d0 in [-10.0 + 5 * i for i in range(9)]:
                for dv in (-3.0, -1.5, 0.0, 1.5, 3.0):
                    out.append(("merge", (d_end, d0, dv, 12.0), ("A", "B"), (d_end, d0, dv)))
    return out


def check_context(job):
    row, ctx, names, key, e, ncars, bystanders = job[:7]
    adversary = job[7] if len(job) > 7 else None  # (name, kind, patience): that car is not czp's
    lags = X.car_lags()
    if row == "merge":
        conflicts = {frozenset("AB")}
    else:
        aps = ctx[2]
        conflicts = {frozenset((i, j)) for i in names for j in names
                     if i < j and conflict(row, aps[i][0], aps[j][0])}
    res = Counter()
    worst_wait = 0.0
    examples = {}
    orders = Counter()
    if adversary is None:
        family = advs(names, conflicts, ncars, row == "merge")
    else:  # only the czp car's reading of the adversary varies (the adversary reads the truth, delayed)
        me = next(n for n in names if n != adversary[0])
        family = ({(me, adversary[0]): (m, d)} for m in MODES2 for d in DELAYS)
    for adv in family:
        r = run_merge(ctx, adv, e, lags) if row == "merge" else run_passage(row, ctx, adv, e, lags, bystanders=bystanders,
                                                                             adversary=adversary)
        res["runs"] += 1
        res["unsafe"] += r["unsafe"] is not None
        res["follow_contact"] += r["follow_contact"]
        res["not_live"] += not r["live"] and r["unsafe"] is None
        res["deadlock"] += r["deadlock"]
        res["stopped"] += r.get("stopped", False)
        orders[r["order"]] += 1
        worst_wait = max(worst_wait, r["wait"])
        if r["unsafe"] is not None and "unsafe" not in examples:
            examples["unsafe"] = {"adv": {f"{i}>{j}": v for (i, j), v in adv.items()}, "at": r["unsafe"]}
        if not r["live"] and r["unsafe"] is None and "not_live" not in examples:
            examples["not_live"] = {"adv": {f"{i}>{j}": v for (i, j), v in adv.items()}}
    return {"row": row, "key": key if adversary is None else [adversary[0] + ":" + adversary[1], ctx[0], key], "names": names,
            **res, "worst_wait": round(worst_wait, 2), "orders": dict(orders), "examples": examples}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--row", choices=("cross", "narrow", "merge"), required=True)
    ap.add_argument("--cars", type=int, default=2, choices=(2, 3))
    ap.add_argument("--e", type=float, default=0.25)
    ap.add_argument("--out", required=True)
    ap.add_argument("--jobs", type=int, default=24)
    ap.add_argument("--no-bystanders", action="store_true", help="a car does not see the cars outside its conflict set")
    ap.add_argument("--limit", type=int, default=0, help="the first N contexts only (a smoke test)")
    ap.add_argument("--adversary", choices=("none", "impatient", "never", "both"), default="none",
                    help="cross, 2 cars: one car not czp's, in either place (see above)")
    ap.add_argument("--adv-patience", type=float, default=0.5, help="the impatient car's patience after a mutual rest (s)")
    ap.add_argument("--autoware-launch", default=None, metavar="DEAD,ACCEL,V_END",
                    help="078 P2g: czp's cars launch from rest as Autoware's ego (measured 0.6,0.68,3.0)")
    ap.add_argument("--rest-lag", type=float, default=0.0, help="078 P2g: a car's rest is seen this late (P2f: 0.8 s)")
    ap.add_argument("--guard-extra", type=float, default=0.0, help="078 P2g: czp's guard_extra (added to every guard "
                    "counted from a rest)")
    ap.add_argument("--cross-probe", action="store_true", help="078 P2g: the crossing probes a go from a mutual rest "
                    "(creep, watch, go), as the all-way stop does")
    ap.add_argument("--probe-watch", type=float, default=None, help="078 P2g: czp's probe_watch (s)")
    CA.add_bias_args(ap)
    a = ap.parse_args()
    X.REST_LAG = a.rest_lag
    if a.probe_watch is not None:
        os.environ["P2G_PROBE_WATCH"] = str(a.probe_watch)
    if a.guard_extra or a.cross_probe or a.probe_watch is not None:
        os.environ["P2G_GUARD_EXTRA"] = str(a.guard_extra)
        os.environ["P2G_CROSS_PROBE"] = "1" if a.cross_probe else "0"
        import p2g_guard_pre  # noqa: F401  (czp_adapter.params_for and conventions_for take it)
    if a.autoware_launch:
        X.AUTOWARE_LAUNCH = tuple(float(x) for x in a.autoware_launch.split(","))
    CA.set_bias(a)
    if a.row == "merge" and a.cars == 3:
        ap.error("the merge is a pair model (merge_game)")
    if a.adversary != "none" and (a.row != "cross" or a.cars != 2):
        ap.error("--adversary: the crossing with two cars")
    ctxs = contexts(a)
    if a.limit:
        ctxs = ctxs[:a.limit]
    if a.adversary == "none":
        jobs = [(row, ctx, names, key, a.e, a.cars, not a.no_bystanders) for row, ctx, names, key in ctxs]
    else:
        kinds = ("impatient", "never") if a.adversary == "both" else (a.adversary,)
        jobs = [(row, ctx, names, key, a.e, a.cars, not a.no_bystanders, (place, k, a.adv_patience))
                for row, ctx, names, key in ctxs for k in kinds for place in ("A", "B")]
    with Pool(a.jobs) as pool:
        res = pool.map(check_context, jobs, chunksize=1)
    os.makedirs(a.out, exist_ok=True)
    tot = Counter()
    for r in res:
        for k in ("runs", "unsafe", "not_live", "deadlock", "follow_contact", "stopped"):
            tot[k] += r.get(k, 0)
    summ = {"row": a.row, "cars": a.cars, "e": a.e, "adversary": a.adversary, "adv_patience": a.adv_patience, "rest_stamp_bias": a.rest_stamp_bias, "arrival_bias": a.arrival_bias,
            "autoware_launch": a.autoware_launch, "rest_lag": a.rest_lag, "guard_extra": a.guard_extra, "cross_probe": a.cross_probe, "probe_watch": a.probe_watch,
            "contexts": len(res), **tot,
            "contexts_unsafe": sum(1 for r in res if r.get("unsafe")),
            "contexts_not_live": sum(1 for r in res if r.get("not_live")),
            "worst_wait": max(r["worst_wait"] for r in res),
            "safe": tot["unsafe"] == 0, "live": tot["not_live"] == 0}
    tag = f"{a.row}{a.cars}_e{a.e:g}" + ("_nobystanders" if a.no_bystanders else "") + (
        "" if a.adversary == "none" else f"_adv-{a.adversary}-p{a.adv_patience:g}") + (
        f"_aw{a.autoware_launch.replace(',', '-')}" if a.autoware_launch else "") + (f"_lag{a.rest_lag:g}" if a.rest_lag else "") + (
        f"_g{a.guard_extra:g}" if a.guard_extra else "") + ("_probe" if a.cross_probe else "") + (
        f"_w{a.probe_watch:g}" if a.probe_watch is not None else "")
    json.dump({"summary": summ, "contexts": res}, open(os.path.join(a.out, f"{tag}.json"), "w"), indent=1)
    lines = [f"# czp_rows_check {a.row}, {a.cars} cars, e {a.e:g}"
             + ("" if a.adversary == "none" else f", adversary {a.adversary} (patience {a.adv_patience:g} s)")
             + f": {len(res)} contexts, {tot['runs']} runs",
             f"safety {'HOLDS' if summ['safe'] else 'FAILS'}: {tot['unsafe']} unsafe runs in {summ['contexts_unsafe']} contexts",
             f"liveness {'HOLDS' if summ['live'] else 'FAILS'}: {tot['not_live']} runs not through by {T_MAX:g} s "
             f"({tot['deadlock']} deadlocks) in {summ['contexts_not_live']} contexts",
             f"follower contacts {tot['follow_contact']}; forced stops (merge) {tot['stopped']}; worst wait {summ['worst_wait']} s"]
    for r in res:
        if r.get("unsafe") or r.get("not_live"):
            lines.append(f"  {r['key']}: unsafe {r.get('unsafe', 0)} not_live {r.get('not_live', 0)} of {r['runs']} "
                         f"orders {r['orders']} e.g. {json.dumps(r['examples'])}")
    open(os.path.join(a.out, f"{tag}.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines[:4]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
