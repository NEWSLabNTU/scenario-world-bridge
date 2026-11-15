#!/usr/bin/env python3
"""Roadmap 074 C7: tie-breaks that bind after a mutual stop, at the crossing (S2) and the narrow
passage (S4), in the kinematic models.

At a near-tie (|dt| <= 0.2-0.3 s) both cars stop and wait (a war of attrition), and over all rule
pairs 38-77 % of the runs where both waited end in a restart crash: both go on patience at once.
S2's right-of-way fault excuses a car that came to rest at its line, so no rule selects there.
This script carries a convention on top of every rule of C2's rule spaces (crossing_game's 17 A
rules x 8 B rules, narrow_game's 23 rules each) and solves each context's game again.

The priority car is B in both scenes: at the crossing B comes from A's right (A drives +x, B +y:
the right-hand rule), at the narrow passage the obstacle is on A's side (the obstacle side yields).

Conventions (behaviour carried by both cars / fault model):
  none        C2's rules, no right of way                              (S2 / S4 as measured)
  rB          C2's rules; S2's fault on A, excused after a rest         (S2 / S4's r = B)
  bind_fault  C2's rules; the fault on A binds after a rest too: A pays FAULT when it enters first
              and B waited (both stopped), braked for it, or its safety layer had to act
  bind        bind_fault's fault, and the priority also binds in behaviour: while B is seen waiting
              before its line, A does not go on patience and, if it can stop, holds until B has
              cleared the zone (or FALLBACK s); B, once it sees both stopped (TIE_HOLD s), goes
  coin        a randomized tie-break with a signalled commitment (a correlation device: a flash or
              a V2X message from a fair coin): while the other waits, neither goes on patience; when
              both have been stopped TIE_HOLD s, the coin picks a winner, which flashes and goes
              FLASH_LEAD s later; the loser sees the flash OBS s after it and holds until the winner
              has cleared (or FALLBACK s); the loser pays FAULT if it enters first after the flash.
              Payoffs are the mean over the two coin outcomes
  stagger     staggered patience, no signal and no fault: the car on the right (B) waits at most
              STAG_SHORT s, A its own patience plus STAG_EXTRA s

Every variant, none included, starts patience from the "both stopped" speed band (WAIT_V), not
from standstill (S4's D2 note: creep kept a car from counting as stopped).

Per context and convention: the pure and 2 x 2 mixed equilibria on the convention's payoffs
(crossing_game.solve), the class (A / B / C / W / ! / M), the physical welfare cost C = -(U_A + U_B)
without the fault (a fault is a mechanism, not a cost) of the worst and best equilibrium, over all
rule pairs the restart-crash share (contact after both stopped, of the runs where both stopped),
deadlocks, and the mean wait (each car's time below WAIT_V before the zone), and whether the
convention is self-enforcing (no car gains more than ENF_TOL by dropping the convention's
behaviour, with any rule, in any pure equilibrium). The share of the welfare gap a convention recovers:
  vs the equilibria  (C_worst^none - C_worst^conv) / (C_worst^none - C_best^none)
  vs the optimum     (C_worst^none - C_worst^conv) / (C_worst^none - C_opt^none)
computed where the gap is at least GAP_MIN; peak braking is scored as the arena measures it (the
speed drop over 0.5 s), as in D2, and the cars have the spec's command lags.

Usage:
  scripts/coop/tiebreak.py --geom cross  --out <dir> [--v 6,10,14] [--dts ...] [--jobs N]
  scripts/coop/tiebreak.py --geom narrow --out <dir> [--v 6,10] [--wn 6,30] [--dts ...] [--jobs N]
  scripts/coop/tiebreak.py --selftest
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import itertools
import json
import math
import os
import statistics as st
import sys
from dataclasses import dataclass, replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import measures as M  # noqa: E402
import narrow_game as N  # noqa: E402

PRIO = "B"  # the priority car: from A's right at the crossing, the free side at the narrow passage
TIE_HOLD = 0.5  # s both seen stopped before the tie is declared (as crossing_game's both_stopped)
FLASH_LEAD = 0.3  # s from the winner's flash to its start
FALLBACK = 5.0  # s a loser holds for a winner that does not clear the zone
STAG_SHORT, STAG_EXTRA = 1.0, 3.0  # s: the right car's patience cap, the other's extra patience
FAULT = X.FAULT
GAP_MIN = 0.05
ENF_TOL = 0.02  # a deviation must gain more than D2's payoff floor to break self-enforcement
NEAR = 0.3  # s: the near-tie band |dt| <= NEAR
CONVS = ("none", "rB", "bind_fault", "bind", "coin", "stagger")
BEH = {"none": "none", "rB": "none", "bind_fault": "none", "bind": "bind", "coin": "coin", "stagger": "stagger"}
FAULT_MODEL = {"none": "none", "rB": "rB", "bind_fault": "bind", "bind": "bind", "coin": "coin", "stagger": "none"}
DTS = "-1,-0.5,-0.3,-0.2,-0.1,0,0.1,0.2,0.3,0.5,1"


class Referee:
    """The coin's device: it declares the tie from both cars' own states (each car reports its own
    truth every step) once both have been below WAIT_V before the zone for TIE_HOLD s."""

    def __init__(self, winner):
        self.winner, self.state, self.since, self.t_sig = winner, {}, None, None

    def report(self, ident, t, u, v):
        self.state[ident] = (t, u, v)
        if self.t_sig is not None or len(self.state) < 2:
            return
        (ta, ua, va), (tb, ub, vb) = self.state["A"], self.state["B"]
        if abs(ta - tb) > 1e-6:
            return
        if va < X.WAIT_V and vb < X.WAIT_V and ua < 0 and ub < 0:
            self.since = t if self.since is None else self.since
            if t - self.since >= TIE_HOLD - 1e-9:
                self.t_sig = t
        else:
            self.since = None


class ConvPolicy(X.Policy):
    """crossing_game.Policy (a rule and its safety layer) with a convention on top. ident is the
    car (A or B), beh the convention's behaviour (none, bind, coin, stagger)."""

    def __init__(self, ident, beh, ref, role, rule, v0, b_cont="hold", exit_=X.EXIT, hold_stop=None):
        super().__init__(role, rule, v0, b_cont, exit_, hold_stop)
        self.ident, self.beh, self.ref = ident, beh, ref
        self.prio = ident == PRIO
        self.pat = self._pat  # re-applied through the setter, now that the convention is known
        self.wait = 0.0
        self.t_prev = None
        self.mutual_since = self.tie = None
        self.lose = self.lose_done = self.won = False
        self.lose_t = None

    @property
    def pat(self):
        return self._pat

    @pat.setter
    def pat(self, x):
        # staggered patience, also when a rule resets its patience (react's continuation)
        if x is not None and getattr(self, "beh", None) == "stagger":
            x = min(x, STAG_SHORT) if self.prio else x + STAG_EXTRA
        self._pat = x

    def step(self, t, u, v):
        if self.ref is not None:
            self.ref.report(self.ident, t, u, v)
        so = self._seen(t, X.OBS, first=False)
        other_wait = other_clear = False
        if so is not None:
            ou, ov = so[1] + so[2] * (t - so[0]), so[2]
            other_wait = ov < X.WAIT_V and ou < 0
            other_clear = ou >= self.exit
        me_wait = v < X.WAIT_V and u < 0
        if me_wait and other_wait:
            self.mutual_since = t if self.mutual_since is None else self.mutual_since
            if self.tie is None and t - self.mutual_since >= TIE_HOLD - 1e-9:
                self.tie = t
        else:
            self.mutual_since = None
        if self.beh in ("bind", "coin") and other_wait and u < 0:
            self.stopped_since = None  # patience paused while the other waits: the convention decides
        start_lose = win = False
        can_stop = me_wait or (u < X.STOP and v * v / (2 * max(X.STOP - u, 0.05)) <= 3.0)
        if self.beh == "bind":
            if self.prio:
                win = self.tie is not None
            else:
                start_lose = other_wait and u < 0 and can_stop
        elif self.beh == "coin" and self.ref is not None and self.ref.t_sig is not None:
            if self.ref.winner == self.ident:
                win = t >= self.ref.t_sig + FLASH_LEAD
            else:
                start_lose = t >= self.ref.t_sig + X.OBS and u < 0
        if start_lose and not self.lose and not self.lose_done:
            self.lose, self.lose_t = True, t
        if self.lose and (other_clear or t - self.lose_t >= FALLBACK):
            self.lose, self.lose_done = False, True
        if win and not self.won:
            self.won = self.committed = True
        a = super().step(t, u, v)
        if self.lose:
            a = min(a, X.hold_out(u, v))
        return a

    def after(self, t, u, v):
        # patience from the "both stopped" band; a rest (for the fault's excuse) from standstill
        if v < X.WAIT_V:
            if self.stopped_since is None:
                self.stopped_since = t
        else:
            self.stopped_since = None
        if v < 0.1 and u < 0:
            self.rested = True
        if v < X.WAIT_V and u < 0:
            self.wait += X.DT


@contextlib.contextmanager
def patched(beh_a, beh_b, winner=None):
    """crossing_game.Policy replaced by ConvPolicy for the next simulate (A is built first, then B;
    crossing_game, narrow_game and both arenas build their policies so)."""
    orig = X.Policy
    ref = Referee(winner) if "coin" in (beh_a, beh_b) else None
    made = []
    idents = iter("AB")

    def make(role, rule, v0, b_cont="hold", exit_=X.EXIT, hold_stop=None):
        ident = next(idents)
        p = ConvPolicy(ident, beh_a if ident == "A" else beh_b, ref, role, rule, v0, b_cont, exit_, hold_stop)
        made.append(p)
        return p

    X.Policy = make
    try:
        yield made, ref
    finally:
        X.Policy = orig


@dataclass
class Res:
    o: X.Outcome
    restart: bool
    deadlock: bool
    wait: float  # mean per car (s)
    sig: bool  # the coin signalled
    winner: str


def run_one(geom, a, b, ctx, beh=("none", "none"), winner=None, lags=None):
    """One run; geom 'cross' (a: an A rule name, b: a B rule name or react:<cont>) or 'narrow'."""
    dt, v, wn = ctx
    with patched(beh[0], beh[1], winner) as (made, ref):
        if geom == "cross":
            ra = next(r for r in X.A_RULES if r[0] == a)
            bn, cont = (b.split(":") + ["hold"])[:2]
            rb = next(r for r in X.B_RULES if r[0] == bn)
            o = X.simulate(ra, rb, dt, v, v, cont, lags=lags)
            restart, deadlock = o.contact and o.both_stopped, not o.contact and o.first == ""
        else:
            out = N.simulate(a, b, dt, v, v, wn, lags)
            o, restart, deadlock = out.o, out.restart, out.deadlock
    o = replace(o, peak=o.peak_w)  # scored as the arena scores (074 D2)
    return Res(o, restart, deadlock, (made[0].wait + made[1].wait) / 2,
               ref is not None and ref.t_sig is not None, winner or "")


def utility(r: Res, who, model, fault=FAULT):
    """Own utility under a fault model (none, rB, bind, coin)."""
    o = r.o
    if model in ("none", "rB"):
        return X.utility(o, who, "B" if model == "rB" else "none", fault)
    u = X.utility(o, who, "none")
    me, other = "AB"[who], 1 - who
    if model == "bind" and me != PRIO and o.first == me and (
            o.both_stopped or o.shield[other] or o.pet < M.PET_REF or o.contact or o.peak[other] > X.YIELD_SEEN):
        u -= fault
    if model == "coin" and r.sig and r.winner and me != r.winner and o.first == me:
        u -= fault
    return u


def acts(geom):
    """(A's actions, B's actions, the run keys behind a B action)."""
    if geom == "cross":
        a = [r[0] for r in X.A_RULES]
        b = [r[0] for r in X.B_RULES]
        keys = {x: ([f"react:{c}" for c in ("yield", "hold")] if x == "react" else [x]) for x in b}
        return a, b, keys
    return list(N.RULES), list(N.RULES), {x: [x] for x in N.RULES}


def winners(beh):
    return ("A", "B") if beh == "coin" else (None,)


def all_runs(geom, ctx, lags):
    """Every rule pair under each behaviour (and each coin outcome)."""
    a_acts, b_acts, keys = acts(geom)
    runs = {}
    for beh in ("none", "bind", "coin", "stagger"):
        for w in winners(beh):
            for a in a_acts:
                for b in b_acts:
                    for k in keys[b]:
                        runs[(beh, w, a, k)] = run_one(geom, a, k, ctx, (beh, beh), w, lags)
    return runs


def game(geom, runs, conv):
    """The convention's payoff tables over actions (B's react continuation its best reply)."""
    a_acts, b_acts, keys = acts(geom)
    beh, model = BEH[conv], FAULT_MODEL[conv]
    ws = winners(beh)
    ua, ub, phys, prof = {}, {}, {}, {}
    for a in a_acts:
        for b in b_acts:
            def mean_u(k, who, m=model):
                return st.mean(utility(runs[(beh, w, a, k)], who, m) for w in ws)
            k = max(keys[b], key=lambda kk: mean_u(kk, 1))
            ua[(a, b)], ub[(a, b)] = mean_u(k, 0), mean_u(k, 1)
            phys[(a, b)] = -(mean_u(k, 0, "none") + mean_u(k, 1, "none"))
            prof[(a, b)] = [runs[(beh, w, a, k)] for w in ws]
    pure, mixed = X.solve(ua, ub, a_acts, b_acts)
    return ua, ub, phys, prof, pure, mixed


def klass(prof, pure, mixed):
    if any(any(r.o.contact for r in prof[p]) for p in pure):
        return "!"
    if any(any(r.o.both_stopped for r in prof[p]) for p in pure):
        return "W"
    fs = {r.o.first for p in pure for r in prof[p]}
    if len(fs) > 1:
        return "C"
    return (fs.pop() or "?") if fs else ("M" if mixed else "?")


def eq_costs(phys, pure, mixed):
    eq = [phys[p] for p in pure] + [sum(pa[a] * pb[b] * phys[(a, b)] for a in pa for b in pb) for pa, pb in mixed]
    return (max(eq), min(eq)) if eq else (None, None)


def self_enforcing(geom, ctx, lags, conv, ua, ub, pure, runs_cache):
    """No car gains by dropping the convention's behaviour (keeping any rule) in a pure equilibrium."""
    beh, model = BEH[conv], FAULT_MODEL[conv]
    if beh == "none":
        return True
    a_acts, b_acts, keys = acts(geom)
    ws = winners(beh)

    def run(a, k, ba, bb, w):
        key = (ba, bb, w, a, k)
        if key not in runs_cache:
            runs_cache[key] = run_one(geom, a, k, ctx, (ba, bb), w, lags)
        return runs_cache[key]

    for a, b in pure:
        for a2 in a_acts:  # A drops the convention
            best_b = max(keys[b], key=lambda kk: st.mean(utility(run(a2, kk, "none", beh, w), 1, model) for w in ws))
            if st.mean(utility(run(a2, best_b, "none", beh, w), 0, model) for w in ws) > ua[(a, b)] + ENF_TOL:
                return False
        for b2 in b_acts:  # B drops it
            for k in keys[b2]:
                if st.mean(utility(run(a, k, beh, "none", w), 1, model) for w in ws) > ub[(a, b)] + ENF_TOL:
                    return False
    return True


def context(job):
    geom, dt, v, wn, lags = job
    ctx = (dt, v, wn)
    runs = all_runs(geom, ctx, lags)
    rows, cache = [], {}
    base = None
    for conv in CONVS:
        ua, ub, phys, prof, pure, mixed = game(geom, runs, conv)
        c_worst, c_best = eq_costs(phys, pure, mixed)
        allr = [r for rs in prof.values() for r in rs]
        waited = [r for r in allr if r.o.both_stopped]
        eqr = [r for p in pure for r in prof[p]]
        row = {"geom": geom, "dt": dt, "v": v, "wn": wn, "conv": conv, "class": klass(prof, pure, mixed),
               "n_pure": len(pure), "n_mixed": len(mixed),
               "C_opt": min(phys.values()), "C_worst_eq": c_worst, "C_best_eq": c_best,
               "C_mean_pairs": st.mean(phys.values()),
               "pairs": len(allr), "pairs_waited": len(waited), "pairs_restart": sum(r.restart for r in allr),
               "pairs_contact": sum(r.o.contact for r in allr), "pairs_deadlock": sum(r.deadlock for r in allr),
               "restart_share": (sum(r.restart for r in waited) / len(waited)) if waited else None,
               "wait_pairs": st.mean(r.wait for r in allr),
               "eq_restart": sum(r.restart for r in eqr), "eq_contact": sum(r.o.contact for r in eqr),
               "eq_waited": sum(r.o.both_stopped for r in eqr),
               "wait_eq": st.mean(r.wait for r in eqr) if eqr else None,
               "eq_first": "".join(sorted({r.o.first for r in eqr})),
               "self_enforcing": int(self_enforcing(geom, ctx, lags, conv, ua, ub, pure, cache)),
               "pure": " ".join(f"{a}/{b}" for a, b in pure)}
        if conv == "none":
            base = row
        for name, ref in (("eq", base["C_best_eq"]), ("opt", base["C_opt"])):
            gap = None if base["C_worst_eq"] is None or ref is None else base["C_worst_eq"] - ref
            ok = gap is not None and gap >= GAP_MIN and c_worst is not None
            row[f"gap_{name}"] = gap
            row[f"recovered_{name}"] = (base["C_worst_eq"] - c_worst) / gap if ok else None
        row["dC_worst"] = None if c_worst is None or base["C_worst_eq"] is None else base["C_worst_eq"] - c_worst
        rows.append(row)
    return rows


def contexts(args):
    dts = [float(x) for x in args.dts.split(",")]
    vs = [float(x) for x in args.v.split(",")]
    wns = [float(x) for x in args.wn.split(",")] if args.geom == "narrow" else [0.0]
    lags = X.car_lags()
    return [(args.geom, dt, v, wn, lags) for wn in wns for v in vs for dt in dts]


def fmt(x):
    if x is None:
        return ""
    if isinstance(x, float):
        return "inf" if math.isinf(x) else f"{x:.4g}"
    return x


def summarize(rows):
    """Per convention, over the near-tie band and away from it."""
    out = {}
    for band, sel in (("near", lambda r: abs(r["dt"]) <= NEAR + 1e-9), ("away", lambda r: abs(r["dt"]) > NEAR + 1e-9),
                      ("all", lambda r: True)):
        for conv in CONVS:
            rs = [r for r in rows if r["conv"] == conv and sel(r)]
            if not rs:
                continue
            waited = sum(r["pairs_waited"] for r in rs)
            rec_eq = [r["recovered_eq"] for r in rs if r["recovered_eq"] is not None]
            rec_opt = [r["recovered_opt"] for r in rs if r["recovered_opt"] is not None]
            cls = [r["class"] for r in rs]
            out[(band, conv)] = {
                "contexts": len(rs),
                "classes": " ".join(f"{c}{cls.count(c)}" for c in "ABCW!M?" if cls.count(c)),
                "restart_share": sum(r["pairs_restart"] for r in rs) / waited if waited else None,
                "restart": sum(r["pairs_restart"] for r in rs), "waited": waited,
                "pairs": sum(r["pairs"] for r in rs),
                "contact_share": sum(r["pairs_contact"] for r in rs) / sum(r["pairs"] for r in rs),
                "deadlock": sum(r["pairs_deadlock"] for r in rs),
                "wait_pairs": st.mean(r["wait_pairs"] for r in rs),
                "wait_eq": st.mean(r["wait_eq"] for r in rs if r["wait_eq"] is not None) if any(
                    r["wait_eq"] is not None for r in rs) else None,
                "eq_restart": sum(r["eq_restart"] for r in rs), "eq_contact": sum(r["eq_contact"] for r in rs),
                "eq_waited_ctx": sum(r["eq_waited"] > 0 for r in rs),
                "loss_worst": st.mean(r["C_worst_eq"] - r["C_opt"] for r in rs if r["C_worst_eq"] is not None),
                "C_worst": st.mean(r["C_worst_eq"] for r in rs if r["C_worst_eq"] is not None),
                "C_mean_pairs": st.mean(r["C_mean_pairs"] for r in rs),
                "dC_worst": st.mean(r["dC_worst"] for r in rs if r["dC_worst"] is not None),
                "recovered_eq": st.mean(rec_eq) if rec_eq else None, "n_gap_eq": len(rec_eq),
                "recovered_opt": st.mean(rec_opt) if rec_opt else None, "n_gap_opt": len(rec_opt),
                "self_enforcing": sum(r["self_enforcing"] for r in rs) / len(rs),
            }
    return out


def report(rows, geom):
    s = summarize(rows)
    pct = lambda x: "" if x is None else f"{100 * x:.0f} %"  # noqa: E731
    num = lambda x, f="{:+.3f}": "" if x is None else f.format(x)  # noqa: E731
    lines = [f"# C7 tie-breaks, {geom} (kinematic, the car's lags): {len({(r['dt'], r['v'], r['wn']) for r in rows})} contexts",
             "# per convention: classes; restart crashes / runs where both stopped, over all rule pairs; contact share",
             "# over all pairs; mean wait per car over all pairs and in equilibrium (s); deadlocks; equilibria with a",
             "# restart crash or contact; the worst equilibrium's welfare loss against the optimum; the mean cost over all",
             "# pairs; the share of the worst-best equilibrium gap recovered (and of the worst-optimum gap); self-enforcing"]
    for band in ("near", "away", "all"):
        lines.append(f"\n## {band} ({'|dt| <= 0.3 s' if band == 'near' else '|dt| > 0.3 s' if band == 'away' else 'every context'})")
        lines.append(f"{'conv':11} {'classes':22} {'restart':>15} {'contact':>8} {'wait':>6} {'wait_eq':>7} {'dl':>4} "
                     f"{'eqR':>4} {'eq!':>4} {'loss':>6} {'Cpairs':>6} {'rec_eq':>12} {'rec_opt':>12} {'enf':>5}")
        for conv in CONVS:
            x = s.get((band, conv))
            if not x:
                continue
            lines.append(f"{conv:11} {x['classes']:22} {pct(x['restart_share']):>6} {x['restart']:>4}/{x['waited']:<4} "
                         f"{pct(x['contact_share']):>8} {x['wait_pairs']:6.2f} {num(x['wait_eq'], '{:.2f}'):>7} "
                         f"{x['deadlock']:>4} {x['eq_restart']:>4} {x['eq_contact']:>4} {x['loss_worst']:6.3f} "
                         f"{x['C_mean_pairs']:6.3f} {num(x['recovered_eq'], '{:+.2f}'):>6} (n{x['n_gap_eq']:>3}) "
                         f"{num(x['recovered_opt'], '{:+.2f}'):>6} (n{x['n_gap_opt']:>3}) {pct(x['self_enforcing']):>5}")
    lines.append("\n## classes per context (none | rB | bind_fault | bind | coin | stagger)")
    keyf = lambda r: (r["wn"], r["v"], r["dt"])  # noqa: E731
    for k in sorted({keyf(r) for r in rows}):
        rs = {r["conv"]: r for r in rows if keyf(r) == k}
        lines.append(f"wn {k[0]:3g} v {k[1]:3g} dt {k[2]:+5.2f}: " + " ".join(f"{rs[c]['class']:>2}" for c in CONVS)
                     + "   restart " + " ".join(f"{rs[c]['pairs_restart']:>3}/{rs[c]['pairs_waited']:<3}" for c in CONVS)
                     + "   worst-opt " + " ".join(f"{(rs[c]['C_worst_eq'] or 0) - rs[c]['C_opt']:.2f}" for c in CONVS))
    return "\n".join(lines) + "\n", s


def run_map(args):
    from multiprocessing import Pool

    with Pool(args.jobs) as pool:
        rows = [r for rs in pool.imap(context, contexts(args)) for r in rs]
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, f"tiebreak_{args.geom}.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(v) for k, v in r.items()})
    text, s = report(rows, args.geom)
    with open(os.path.join(args.out, f"tiebreak_{args.geom}.txt"), "w") as f:
        f.write(text)
    with open(os.path.join(args.out, f"tiebreak_{args.geom}.json"), "w") as f:
        json.dump({f"{b}|{c}": v for (b, c), v in s.items()}, f, indent=1)
    print(text, end="")
    return 0


def selftest():
    lags = X.car_lags()
    # the referee declares a tie only after TIE_HOLD s of both stopped
    ref = Referee("A")
    for i in range(12):
        t = i * X.DT
        ref.report("A", t, -2.0, 0.0)
        ref.report("B", t, -2.0, 0.0)
    assert ref.t_sig is not None and abs(ref.t_sig - TIE_HOLD) < 1e-6, ref.t_sig
    # the baseline mutual stop: two waiting rules with the same patience restart together
    na, nb = "tinfp2", "yb4p2"
    o0 = run_one("cross", na, nb, (0.0, 8.0, 0.0), ("none", "none"), None, lags)
    assert o0.o.both_stopped, o0
    # binding priority: B goes first after the stop, no restart crash
    o1 = run_one("cross", na, nb, (0.0, 8.0, 0.0), ("bind", "bind"), None, lags)
    assert o1.o.both_stopped and o1.o.first == "B" and not o1.restart, o1
    # the coin: the winner goes first, the loser waits
    for w in "AB":
        o2 = run_one("cross", na, nb, (0.0, 8.0, 0.0), ("coin", "coin"), w, lags)
        assert o2.sig and o2.o.first == w and not o2.o.contact, (w, o2)
    # staggered patience: B (right) goes after 1 s, A waits 5 s: B first
    o3 = run_one("cross", na, nb, (0.0, 8.0, 0.0), ("stagger", "stagger"), None, lags)
    assert o3.o.first == "B" and not o3.o.contact, o3
    # the narrow passage: same patience on both sides crashes on restart without a convention
    n0 = run_one("narrow", "tinfp2", "tinfp2", (0.0, 6.0, 30.0), ("none", "none"), None, lags)
    n1 = run_one("narrow", "tinfp2", "tinfp2", (0.0, 6.0, 30.0), ("bind", "bind"), None, lags)
    assert n0.o.both_stopped and n1.o.first == "B" and not n1.restart, (n0, n1)
    # the bind fault binds after a rest; S2's excuses it
    ro = Res(replace(o0.o, first="A", stopped=(True, True), both_stopped=True, shield=(False, False),
                     pet=9.0, contact=False, peak=(0.0, 0.0)), False, False, 0.0, False, "")
    assert utility(ro, 0, "rB") == utility(ro, 0, "none") and utility(ro, 0, "bind") == utility(ro, 0, "none") - FAULT
    # a context runs and every convention yields a row
    rows = context(("cross", 0.0, 10.0, 0.0, lags))
    assert [r["conv"] for r in rows] == list(CONVS) and all(r["n_pure"] + r["n_mixed"] > 0 for r in rows), rows
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--geom", choices=("cross", "narrow"), default="cross")
    p.add_argument("--out", default="tiebreak_out")
    p.add_argument("--v", default=None, help="speeds (m/s); default 6,10,14 (cross), 6,10 (narrow)")
    p.add_argument("--wn", default="6,30", help="narrow passage lengths (m)")
    p.add_argument("--dts", default=DTS)
    p.add_argument("--jobs", type=int, default=30)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    a.v = a.v or ("6,10,14" if a.geom == "cross" else "6,10")
    return run_map(a)


if __name__ == "__main__":
    sys.exit(main())
