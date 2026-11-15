#!/usr/bin/env python3
"""Roadmap 074 C8: the four-way stop as a decentralised mutual-exclusion protocol, model-checked.

An explicit-state checker over a discrete-time model (tick Q = 0.05 s, the arena's) of two or three
cars at an all-way stop with no V2X. Every car reads the others from its own perception, so the
model's nondeterminism is perception's:

  - the rest stamp a car reads for another car's stop is off by up to +-e (the Observer's noise and
    10 Hz sampling: a noisy "at rest" sample while the other still rolls, or a late one), and is
    known LR s after the stamp (four_way_game's OBS + DECIDE);
  - every other phase change of another car (it starts braking, starts moving, leaves the box, ...)
    is seen between LMIN and LMAX s after it happens (OBS + DECIDE + up to one 10 Hz sample);
  - with --round2 coin, a coin (a V2X message, for comparison only) picks the second round's winner.

Every reachable state is explored breadth-first from every context (the cars' free-flow arrival
times at their lines, f_A - f_B = dt on a grid), so a property holds over every perception outcome
inside the bounds, not over sampled ones.

Cars (all at speed v; one box Z; A from the west along +x, B from the south along +y, so B is on A's
right; C, in three-car models, from the east along -x, on B's right; A and C, opposite, do not
conflict):

  stopper   cruises, brakes at 2 m/s^2 (four_way_game.plan_stop) and comes to rest at its line at
            r = f + v / 4; once a rule says go at g, it is seen moving from g + VIS, is in Z over
            [g + ENTER, g + EXIT_T) (from rest, the launch dead time and 3 m/s^2: 1.3 and 3.1 s) and
            has left Z after that. A car never stops once it is in Z.
  flow      (rule go, or stock yield that found the way clear on its approach) never stops: in Z over
            [f + 1/v, f + (1 + EXIT)/v).

Rules (four_way_game's, abstracted to their decision logic; every one but go has crossing_game's
safety layer as a start guard, "closing": no start while another car is seen near (within 2 s of Z at
its speed), braking, moving from its line or in Z; fcfsm and fcfse take a car seen braking for its line
as one that will stop, the zone contract, and leave it to the protocol):

  go        aggressive: never stops, never yields (the BAR case: a car that ignores the convention)
  yield     stock Autoware's mutual yield: at its last comfortable stop point (where a stopper would
            start braking) it passes without stopping if the way is clear, else stops; at rest it goes
            when the way is clear; "clear": the other's passage misses its own by M_BEFORE / M_AFTER
            (4 / 3 s), and a car at rest is predicted to stay. Committed C = 1.5 s: until then (and
            before Z) it stops again when the way is no longer clear (a restart).
  stop1y    full stop, D = 1 s dwell, then yield's test and restarts
  rbl       stop, dwell; B goes; A waits while B is seen near, braking or at rest at its line
  fcfs      (a) stop, dwell; go unless the other's rest stamp is earlier than its own by more than T
            (0.5 s); no stamp known (yet) is a go; a tie is a go
  fcfsr     (b) fcfs with a tie (|stamps| <= T) broken right-before-left: B goes, A waits
  fcfse     (e) fcfsm with a last-resort tie-break by the map's fixed approach order (A, B, C, D): in the
            second round, when a car sees every arm waiting at rest (right-before-left is a cycle), it
            ranks by that order instead; and a last resort, H3 s after its epoch: go when every
            conflicting car is seen at rest and comes later in that order (against a rule that waits
            for this car and never goes, as fcfs does when it reads this car first, and against
            cycles a car cannot see whole). Every car computes the same order, with no perception error
  fcfsm     (c) the fail-safe variant, with margin m and second-round wait H2:
            - delta = the other's rest stamp - its own rest time; first: delta > T + m;
              second: delta < -(T + m); tie: |delta| < T - m; ambiguous: anything else;
            - no stamp known yet: first only once the stamp, if the other had stopped within T + m
              after it, would be known (t - LR - r >= T + m), or when the other is seen far;
              until then it waits;
            - first goes; a tie goes for the car with priority (B); everything else waits (both wait
              when the readings disagree), and a car seen moving from its line has the turn;
            - second round: a car still waiting H2 s after its epoch (its dwell's end, or the last
              time it saw any car start or leave Z: the junction has been quiet) goes if it has priority over every conflicting car that is
              seen at rest, and none is seen moving (rbl; --round2 coin: the coin's winner);
            - a zone-interval guard besides the safety layer: no start while a car that is not
              braking for its line is predicted (at constant speed) to enter Z before this car would
              leave it, plus ZONE_MG.

Properties, per context and rule pair:
  safety    never two conflicting cars in Z at once (no post-start shield: a violation is a run where
            the protocol leaves safety to the AEB);
  liveness  every car leaves Z before the horizon (40 s); a state at the horizon is a deadlock if
            every car still waiting is at rest with nothing left to observe, else a livelock;
  bound     the worst wait, over every path, from a car's rest to its go.

Usage:
  c8_fourway_check.py check --out <dir> [--v 6,10] [--e 0.25] [--m 0.3] [--h2 2.0] [--jobs 24]
  c8_fourway_check.py sweep --out <dir> [--v 6,10] [--es 0,0.1,..] [--ms 0,0.05,..] [--h2s ..]
  c8_fourway_check.py three --out <dir> [--v 6] [--e 0.25] [--m 0.3] [--h2 2.0]
  c8_fourway_check.py measure-e --out <dir> [--v 6,10] [--n 2000] [--sigma-x 0.3 --sigma-v 0.2]
  c8_fourway_check.py --selftest
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
from collections import deque
from dataclasses import dataclass, replace
from multiprocessing import Pool

Q = 0.05  # s, one tick
L_CAR, W_BOX = 4.79, 3.5
EXIT_D = 1.0 + W_BOX + L_CAR  # m from the line until the rear has left Z
M_BEFORE, M_AFTER = 4.0, 3.0  # s, stock yield's margins
CLOSE_T = 2.0  # s, the safety layer's "about to enter"
COMMIT = 1.5  # s, stock yield's commit time
ABORT_BY = 1.0  # s after a go, the last moment a stop before Z is still possible
B_PLAN = 2.0  # m/s^2, a stopper's braking to its line
RIGHT = {"A": "B", "B": "C", "C": "D", "D": "A"}  # whose arm is on my right (cyclic with four arms)
CONFLICT = {frozenset("AB"), frozenset("BC"), frozenset("CD"), frozenset("DA")}  # opposite arms do not conflict
ORDER = "ABCD"  # (e)'s fixed approach order from the map: the lower index goes first


def tk(s):
    return int(round(s / Q))


@dataclass(frozen=True)
class Par:
    v: float = 6.0
    e: float = 0.25  # s, the rest stamp's error bound
    m: float = 0.3  # s, fcfsm's margin
    h2: float = 2.0  # s, fcfsm's second-round wait
    h3: float = None  # s (None: H2 + T + m + e + VIS + LMAX + 0.2), fcfsm's last resort: a car still waiting this long after its epoch goes when
    #                  every conflicting car is seen at rest (one that waits for it and never goes: fcfs)
    T: float = 0.5  # s, the tie threshold
    D: float = 1.0  # s, the dwell at the line
    lmin: float = 0.70  # s, phase change -> seen
    lmax: float = 0.80  # (Par.det_latency: every phase change seen exactly LMIN late, for four cars)
    lr: float = 0.70  # s, rest stamp -> known
    vis: float = 0.7  # s, go -> seen moving
    enter: float = 1.3  # s, go -> in Z
    exit_t: float = 3.1  # s, go -> out of Z
    zone_mg: float = 0.5  # s, fcfsm's zone guard margin
    horizon: float = 40.0
    round2: str = "rbl"  # or "coin"
    epoch_fix: bool = True  # fcfsm's round-2/3 timers restart when any car is seen starting or leaving Z
    exit_block: tuple = None  # (car, s): that car's exit lane beyond Z is blocked until then (a queue)
    det_latency: bool = False  # every phase change but a rest stamp seen exactly LMIN late (state space)
    shield: bool = False  # the safety layer after a start too: a car not yet in Z stops when it sees a
    shield_by: float = 1.2  # conflicting car moving from its line or in Z, up to this long after its go


# ---- the model -------------------------------------------------------------------------------------
class Model:
    """Static data of one context: the cars' names, rules and free-flow line times (ticks)."""

    def __init__(self, cars, par):
        self.p = par
        self.names = tuple(c[0] for c in cars)
        self.rule = {c[0]: c[1] for c in cars}
        self.f = {c[0]: c[2] for c in cars}
        v = par.v
        if par.h3 is None:  # after the other car's second-round go has been seen, whatever it read
            par = replace(par, h3=par.h2 + par.T + par.m + par.e + par.vis + par.lmax + 0.2)
            self.p = par
        self.k = {k: tk(getattr(par, k)) for k in ("e", "m", "h2", "h3", "T", "D", "lmin", "lmax", "lr", "vis", "enter",
                                                     "exit_t", "zone_mg", "horizon")}
        if par.det_latency:
            self.k["lmax"] = self.k["lmin"]
        # rest r = f + v / (2 b) / ... : braking at b from v takes v / b, covers v^2 / (2 b), which free flow
        # covers in v / (2 b): the rest comes v / (2 b) after the free-flow arrival
        self.r = {n: self.f[n] + tk(v / (2 * B_PLAN)) for n in self.names}
        self.bstart = {n: self.r[n] - tk(v / B_PLAN) for n in self.names}
        self.near = {n: self.f[n] + tk(1.0 / v - CLOSE_T) for n in self.names}
        self.fin = {n: self.f[n] + tk(1.0 / v) for n in self.names}  # a flow car's entry into Z
        self.fout = {n: self.f[n] + tk(EXIT_D / v) for n in self.names}
        self.track_cycles = any(self.rule[n] in ("fcfsm", "fcfse") for n in self.names)
        self.conf = {n: tuple(o for o in self.names if frozenset((n, o)) in CONFLICT) for n in self.names}

    def prio(self, i, j, coin, fixed=False):
        """i has priority over j (rbl: j's right arm is i; coin: i won it; fixed: (e)'s map order)."""
        if fixed:
            return ORDER.index(i) < ORDER.index(j)
        if self.p.round2 == "coin" and coin is not None:
            return coin == i
        return RIGHT.get(j) == i

    def bucket(self, i, stamp):
        """A rest stamp as the rules use it (its comparisons with i's own rest), so that states merge."""
        k = self.k
        d = stamp - self.r[i]
        return (d < -k["T"], d <= k["T"], d > k["T"] + k["m"], d < -(k["T"] + k["m"]), abs(d) < k["T"] - k["m"])


# The state: (t, cars, coin, cycle_since); a car's: (ph, r, since, g, hist, know, restarts)
#   ph     "appr" | "rest" | "go" | "flow" | "done"
#   r      its first rest tick (or None); since: the tick its current rest began; g: its go tick
#   hist   its phase changes as others can see them: ((tick, phase), ...), phases far near brake rest go box done
#   know   per other car (in self.names order, itself skipped): (k, rs, clr): k entries of its hist seen,
#          the rest stamp read (as Model.bucket, or None), the tick its leaving Z was seen (or None)
PH, R, SINCE, G, HIST, KNOW, NRE = range(7)


def initial(M_, coin):
    cars = []
    for n in M_.names:
        know = tuple((1, None, None) for o in M_.names if o != n)
        cars.append(("appr", None, None, None, ((0, "far"),), know, 0))
    return (0, tuple(cars), coin, None)


def other_idx(M_, i, j):
    """Index of j in i's know tuple."""
    names = [o for o in M_.names if o != i]
    return names.index(j)


def seen(M_, S, i, j):
    """(phase, rs, clr) of j as i sees it."""
    cars = S[1]
    ci = cars[M_.names.index(i)]
    cj = cars[M_.names.index(j)]
    k, rs, clr = ci[KNOW][other_idx(M_, i, j)]
    return cj[HIST][k - 1][1], rs, clr


def in_z(M_, n, c, t):
    if c[PH] == "flow" or (c[PH] == "done" and c[G] is None):
        return M_.fin[n] <= t < M_.fout[n]
    if c[G] is not None and c[PH] in ("go", "done"):
        return c[G] + M_.k["enter"] <= t < c[G] + M_.k["exit_t"]
    return False


# ---- rules ------------------------------------------------------------------------------------------
def closing_ok(M_, S, i):
    # fcfsm/fcfse: a car seen braking for its line will stop (the zone contract); the others' layer holds for it
    hold = ("near", "go", "box") if M_.rule[i] in ("fcfsm", "fcfse") else ("near", "brake", "go", "box")
    for j in M_.conf[i]:
        ph, _, clr = seen(M_, S, i, j)
        if clr is None and ph in hold:
            return False
    return True


def zone_ok(M_, S, i, t):
    k = M_.k
    for j in M_.conf[i]:
        ph, _, clr = seen(M_, S, i, j)
        if clr is not None or ph == "done":
            continue
        if ph in ("go", "box"):
            return False
        if ph in ("far", "near") and M_.fin[j] <= t + k["exit_t"] + k["zone_mg"]:
            return False
    return True


def clear_test(M_, S, i, t, my_in, my_out):
    """Stock yield's test: every conflicting car's passage misses mine by the margins; one at rest stays."""
    cars = S[1]
    for j in M_.conf[i]:
        ph, _, clr = seen(M_, S, i, j)
        if clr is not None or ph in ("done", "rest"):
            continue
        cj = cars[M_.names.index(j)]
        if ph == "go":
            o_in, o_out = cj[G] + M_.k["enter"], cj[G] + M_.k["exit_t"]
        else:  # far, near, brake, box: predicted at constant speed
            o_in, o_out = M_.fin[j], M_.fout[j]
        if not (o_out + tk(M_BEFORE) <= my_in or o_in >= my_out + tk(M_AFTER)):
            return False
    return True


def decide_rest(M_, S, i, t):
    """An at-rest car's decision: True = go."""
    p, k = M_.p, M_.k
    rule = M_.rule[i]
    c = S[1][M_.names.index(i)]
    r = c[R]
    if rule in ("yield", "stop1y"):
        if rule == "stop1y" and t - r < k["D"]:
            return False
        return closing_ok(M_, S, i) and clear_test(M_, S, i, t, t + k["enter"], t + k["exit_t"])
    if t - r < k["D"]:
        return False
    if rule == "rbl":
        if not closing_ok(M_, S, i):
            return False
        for j in M_.conf[i]:
            if M_.prio(j, i, None):  # j on my right
                ph, _, clr = seen(M_, S, i, j)
                if clr is None and ph in ("near", "brake", "rest"):
                    return False
        return True
    if rule in ("fcfs", "fcfsr"):
        if not closing_ok(M_, S, i):
            return False
        for j in M_.conf[i]:
            ph, rs, clr = seen(M_, S, i, j)
            if clr is not None or ph == "done" or rs is None:
                continue
            if rs[0]:
                return False
            if rs[1] and rule == "fcfsr" and not M_.prio(i, j, None):
                return False
        return True
    if rule in ("fcfsm", "fcfse"):
        return decide_fcfsm(M_, S, i, t)
    raise ValueError(rule)


def fcfsm_class(M_, S, i, j, t):
    """(c)'s reading of j: first, second, tie, amb, wait (no stamp yet, inside the horizon), turn (j moving)."""
    k = M_.k
    c = S[1][M_.names.index(i)]
    ph, rs, clr = seen(M_, S, i, j)
    if clr is not None or ph == "done":
        return "gone"
    if ph in ("go", "box"):
        return "turn"
    if rs is None:
        if ph == "far":
            return "first"
        return "first" if t - k["lr"] - c[R] >= k["T"] + k["m"] else "wait"
    if rs[2]:
        return "first"
    if rs[3]:
        return "second"
    if rs[4]:
        return "tie"
    return "amb"


def cyclic(M_, S, i):
    """(e): i sees the right-before-left relation among the waiting cars close a cycle (all four arms
    waiting at rest). Opposite cars are seen too: they share no conflict, but do share the cycle."""
    if len(M_.names) < 4:
        return False
    for j in M_.names:
        if j != i:
            ph, _, clr = seen(M_, S, i, j)
            if clr is not None or ph != "rest":
                return False
    return True


def decide_fcfsm(M_, S, i, t):
    k = M_.k
    c = S[1][M_.names.index(i)]
    coin = S[2]
    if not (closing_ok(M_, S, i) and zone_ok(M_, S, i, t)):
        return False
    eb = M_.p.exit_block
    if eb and eb[0] == i and t < tk(eb[1]) + k["lmax"]:
        return False  # never enter Z unless its exit is clear (seen clear)
    fixed = M_.rule[i] == "fcfse" and cyclic(M_, S, i)
    cls = {j: fcfsm_class(M_, S, i, j, t) for j in M_.conf[i]}
    if all(v in ("first", "gone") or (v == "tie" and M_.prio(i, j, coin)) for j, v in cls.items()):
        return True
    # the second round
    epoch = c[R] + k["D"]
    if M_.p.epoch_fix:  # the junction quiet: no car seen starting or leaving for H2 (H3) s
        for j in M_.names:
            if j != i:
                kk = c[KNOW][other_idx(M_, i, j)][0]
                for tc, ph in S[1][M_.names.index(j)][HIST][:kk]:
                    if ph in ("go", "done"):
                        epoch = max(epoch, tc + k["lmin"])
        eb = M_.p.exit_block  # a blocked exit's release is a junction event too: everyone sees the lane
        if eb and t >= tk(eb[1]) + k["lmin"]:
            epoch = max(epoch, tk(eb[1]) + k["lmin"])
    if t - epoch < k["h2"]:
        return False
    if M_.p.epoch_fix and any(seen(M_, S, i, j)[0] in ("go", "box") for j in M_.names if j != i):
        return False  # the junction is not quiet while any car, a non-conflicting one too, is seen crossing
    if M_.rule[i] == "fcfse" and t - epoch >= k["h3"] and all(v == "gone" or (seen(M_, S, i, j)[0] == "rest" and M_.prio(i, j, None, True))
                                    for j, v in cls.items()):
        return True
    for j, v in cls.items():
        if v == "gone":
            continue
        ph, _, _ = seen(M_, S, i, j)
        if v in ("turn", "wait"):
            return False
        if ph == "rest" and not M_.prio(i, j, coin, fixed):
            return False
        if ph not in ("rest", "far"):
            return False
    return True


# ---- one step -----------------------------------------------------------------------------------------
def deliveries(M_, S, t):
    """Every combination of observations that may (or must) arrive at tick t: list of (new cars, labels)."""
    k = M_.k
    cars = S[1]
    opts = []  # per (i, j): list of (new know entry, label)
    keys = []
    for ii, i in enumerate(M_.names):
        for j in M_.names:
            if j == i:
                continue
            jj = M_.names.index(j)
            oi = other_idx(M_, i, j)
            kk, rs, clr = cars[ii][KNOW][oi]
            hist = cars[jj][HIST]
            choices = [((kk, rs, clr), None)]
            must = False
            # one entry per tick at most (entries are far apart against the windows)
            if kk < len(hist):
                tc, ph = hist[kk]
                stamp = ph == "rest" and rs is None and j in M_.conf[i]  # only a conflicting car's stamp is used
                if stamp:
                    lo, hi = tc - k["e"] + k["lr"], tc + k["e"] + k["lr"]
                else:
                    lo, hi = tc + k["lmin"], tc + k["lmax"]
                if t >= lo:
                    nrs = M_.bucket(i, t - k["lr"]) if stamp else rs
                    nclr = t if ph == "done" and clr is None else clr
                    new = ((kk + 1, nrs, nclr), f"{i} sees {j} {ph}" + (f" (stamp {(t - k['lr']) * Q:.2f})" if nrs != rs else ""))
                    if t >= hi:
                        choices = [new]
                        must = True
                    else:
                        choices.append(new)
            opts.append(choices)
            keys.append((ii, oi))
            del must
    out = []
    for combo in itertools.product(*opts):
        new_know = [list(c[KNOW]) for c in cars]
        labels = []
        for (ii, oi), (entry, lab) in zip(keys, combo):
            new_know[ii][oi] = entry
            if lab:
                labels.append(lab)
        nc = tuple(c[:KNOW] + (tuple(new_know[ii]),) + c[KNOW + 1:] for ii, c in enumerate(cars))
        out.append((nc, labels))
    return out


def wait_for(M_, S, t):
    """The wait-for graph of the waiting fcfsm/fcfse cars (past their dwell, at rest): i -> j when j blocks
    i's go; True when it has a cycle."""
    edges = {}
    for ii, i in enumerate(M_.names):
        c = S[1][ii]
        if M_.rule[i] not in ("fcfsm", "fcfse") or c[PH] != "rest" or t - c[R] < M_.k["D"]:
            continue
        fixed = M_.rule[i] == "fcfse" and cyclic(M_, S, i)
        out = set()
        for j in M_.conf[i]:
            v = fcfsm_class(M_, S, i, j, t)
            if v in ("first", "gone") or (v == "tie" and M_.prio(i, j, S[2])):
                continue
            ph, _, _ = seen(M_, S, i, j)
            if ph == "rest" and t - (c[R] + M_.k["D"]) >= M_.k["h2"] and M_.prio(i, j, S[2], fixed):
                continue
            out.add(j)
        edges[i] = out
    # a cycle among the waiting cars
    def visit(n, stack, done):
        stack.add(n)
        for m in edges.get(n, ()):
            if m in stack or (m not in done and visit(m, stack, done)):
                return True
        stack.discard(n)
        done.add(n)
        return False
    done = set()
    return any(visit(n, set(), done) for n in edges if n not in done)


def advance(M_, S, t):
    """Decisions and physics at tick t (after deliveries): the state at t + 1, with labels."""
    k = M_.k
    cars = list(S[1])
    labels = []
    for ii, n in enumerate(M_.names):
        c = list(cars[ii])
        rule = M_.rule[n]
        hist = list(c[HIST])
        if c[PH] == "appr" and rule == "go":
            c[PH] = "flow"
        if t == M_.near[n] and hist[-1][1] == "far" and (c[PH] == "flow" or (c[PH] == "appr" and t < M_.bstart[n])):
            hist.append((t, "near"))
        if c[PH] == "appr":
            if t == M_.bstart[n]:
                if rule == "yield":
                    Sx = (S[0], tuple(cars), S[2], S[3])
                    if clear_test(M_, Sx, n, t, M_.fin[n], M_.fout[n]):
                        c[PH] = "flow"
                        labels.append(f"{n} finds the way clear on its approach: no stop")
                if c[PH] == "appr":
                    hist.append((t, "brake"))
            if c[PH] == "appr" and t == M_.r[n]:
                c[PH], c[R], c[SINCE] = "rest", t, t
                hist.append((t, "rest"))
        elif c[PH] == "rest":
            Sx = (S[0], tuple(cars), S[2], S[3])
            if decide_rest(M_, Sx, n, t):
                c[PH], c[G] = "go", t
                labels.append(f"{n} goes")
        elif c[PH] == "go":
            g = c[G]
            if rule in ("yield", "stop1y") and t - g < min(tk(COMMIT), tk(ABORT_BY)):
                Sx = (S[0], tuple(cars), S[2], S[3])
                if not clear_test(M_, Sx, n, t, g + k["enter"], g + k["exit_t"]) and c[NRE] < 6:
                    c[PH], c[SINCE], c[NRE] = "rest", t, c[NRE] + 1
                    if t >= g + k["vis"]:
                        hist.append((t, "rest"))
                    labels.append(f"{n} stops again (restart {c[NRE]})")
            if c[PH] == "go" and M_.p.shield and rule != "go" and t - g < tk(M_.p.shield_by):
                Sx = (S[0], tuple(cars), S[2], S[3])
                for j in M_.conf[n]:
                    ph, _, clr = seen(M_, Sx, n, j)
                    if clr is None and ph in ("go", "box") and c[NRE] < 6:
                        c[PH], c[SINCE], c[NRE] = "rest", t, c[NRE] + 1
                        if t >= g + k["vis"]:
                            hist.append((t, "rest"))
                        labels.append(f"{n}'s safety layer stops it before Z (restart {c[NRE]})")
                        break
            if c[PH] == "go":
                if t == g + k["vis"]:
                    hist.append((t, "go"))
                if t >= g + k["exit_t"]:
                    c[PH] = "done"
                    hist.append((t, "done"))
        if c[PH] == "flow":
            if t == M_.fin[n]:
                hist.append((t, "box"))
            if t >= M_.fout[n]:
                c[PH] = "done"
                hist.append((t, "done"))
        c[HIST] = tuple(hist)
        cars[ii] = tuple(c)
    S2 = (t + 1, tuple(cars), S[2], S[3])
    if M_.track_cycles:
        cyc = wait_for(M_, S2, t + 1)
        S2 = S2[:3] + ((S[3] if S[3] is not None else t + 1) if cyc else None,)
    return S2, labels


def violation(M_, S):
    t = S[0]
    occ = [n for ii, n in enumerate(M_.names) if in_z(M_, n, S[1][ii], t)]
    eb = M_.p.exit_block
    if eb and eb[0] in occ and t < tk(eb[1]):
        return (eb[0], "its blocked exit")
    for a, b in itertools.combinations(occ, 2):
        if frozenset((a, b)) in CONFLICT:
            return (a, b)
    return None


@dataclass
class Result:
    safe: bool = True
    live: bool = True
    kind: str = ""  # contact / deadlock / livelock
    trace: list = None
    states: int = 0
    worst_wait: dict = None
    restarts: int = 0
    cycle: float = 0.0  # s, the longest a wait-for cycle survived on any path
    trace_live: list = None  # in a context with a contact too: a deadlock's or livelock's trace


def explore(M_, coins=(None,)):
    """Breadth-first over every reachable state from the context's initial states. Time is part of the
    state, so states merge within a tick only: the frontier is all that is kept, each with the chain of
    labels that led to it (shared prefixes), from which a counterexample's trace is read."""
    hz = M_.k["horizon"]
    bad_c = None
    frontier = {}
    for coin in coins:
        frontier[initial(M_, coin)] = ([f"coin: {coin}"] if coin else [], 0, None)
    res = Result(worst_wait={n: 0 for n in M_.names})
    n_states = 0
    bad = None
    while frontier:
        nxt = {}
        for S, node in frontier.items():
            n_states += 1
            t = S[0]
            cars = S[1]
            if all(c[PH] == "done" for c in cars):
                for ii, n in enumerate(M_.names):
                    c = cars[ii]
                    if c[R] is not None and c[G] is not None:
                        res.worst_wait[n] = max(res.worst_wait[n], c[G] - c[R])
                    res.restarts = max(res.restarts, c[NRE])
                continue
            if t >= hz:
                if res.live:
                    res.live = False
                    stuck = all(c[PH] in ("rest", "done") for c in cars)
                    res.kind = "deadlock" if stuck else "livelock"
                    bad = node
                continue
            for nc, lab1 in deliveries(M_, S, t):
                S1 = (t, nc, S[2], S[3])
                S2, lab2 = advance(M_, S1, t)
                if S2 in nxt:
                    continue
                n2 = (lab1 + lab2, t, node) if (lab1 or lab2) else node
                if S2[3] is not None:
                    res.cycle = max(res.cycle, (S2[0] - S2[3]) * Q)
                if violation(M_, S2):  # a contact ends that path; the others go on (liveness is checked too)
                    if res.safe:
                        res.safe = False
                        bad_c = (lab1 + lab2 + [f"{'/'.join(violation(M_, S2))} both in Z"], t, node)
                    continue
                nxt[S2] = n2
        frontier = nxt
    res.states = n_states
    kinds = (["contact"] if not res.safe else []) + ([res.kind] if not res.live else [])
    res.kind = ",".join(kinds)
    if not res.safe:
        res.trace = trace_of(bad_c)
        res.trace_live = trace_of(bad) if bad is not None else None
    elif bad is not None:
        res.trace = trace_of(bad)
    for n in M_.names:
        res.worst_wait[n] = round(res.worst_wait[n] * Q, 2)
    return res


def trace_of(node):
    out = []
    while node is not None:
        lab, t, node = node
        if lab:
            out.append((round(t * Q, 2), lab))
    return list(reversed(out))


# ---- the checks ---------------------------------------------------------------------------------------
PAIRS_SELF = ("fcfs", "fcfsr", "fcfsm", "fcfse")
OTHERS = ("yield", "stop1y", "go", "rbl", "fcfs", "fcfsr", "fcfsm", "fcfse")


def context(job):
    rule_a, rule_b, dt, par = job
    f_b = tk(8.0)
    f_a = f_b + tk(dt)
    M_ = Model((("A", rule_a, f_a), ("B", rule_b, f_b)), par)
    coins = ("A", "B") if par.round2 == "coin" and "fcfsm" in (rule_a, rule_b) else (None,)
    r = explore(M_, coins)
    return {"a": rule_a, "b": rule_b, "dt": dt, "v": par.v, "e": par.e, "m": par.m, "h2": par.h2, "safe": r.safe,
            "live": r.live, "kind": r.kind, "states": r.states, "waitA": r.worst_wait["A"], "waitB": r.worst_wait["B"],
            "restarts": r.restarts, "cycle": round(r.cycle, 2), "trace": r.trace, "trace_live": r.trace_live}


def dts(lo=-4.0, hi=4.0, step=0.05):
    n = int(round((hi - lo) / step))
    return [round(lo + i * step, 3) for i in range(n + 1)]


def summarize(rows):
    """Per pair: unsafe / not-live contexts and the worst waits."""
    by = {}
    for r in rows:
        by.setdefault((r["v"], r["a"], r["b"]), []).append(r)
    out = []
    for (v, a, b), rs in sorted(by.items()):
        uns = [r["dt"] for r in rs if not r["safe"]]
        nl = [r["dt"] for r in rs if not r["live"]]
        kinds = sorted({r["kind"] for r in rs if r["kind"]})
        out.append({"v": v, "a": a, "b": b, "contexts": len(rs), "unsafe": len(uns), "not_live": len(nl),
                    "unsafe_dt": span(uns), "not_live_dt": span(nl), "kinds": kinds,
                    "worst_waitA": max(r["waitA"] for r in rs), "worst_waitB": max(r["waitB"] for r in rs),
                    "max_restarts": max(r["restarts"] for r in rs),
                    "max_cycle_s": max(r.get("cycle", 0.0) for r in rs)})
    return out


def span(xs):
    """Contiguous dt ranges, as text."""
    if not xs:
        return ""
    xs = sorted(xs)
    parts, lo, prev = [], xs[0], xs[0]
    for x in xs[1:]:
        if x - prev > 0.051:
            parts.append(f"{lo:g}..{prev:g}" if lo != prev else f"{lo:g}")
            lo = x
        prev = x
    parts.append(f"{lo:g}..{prev:g}" if lo != prev else f"{lo:g}")
    return " ".join(parts)


def write_traces(path, rows, limit_per_pair=1):
    seen_ = {}
    with open(path, "w") as f:
        for r in rows:
            key = (r["v"], r["a"], r["b"], r["kind"])
            if not r["trace"] or seen_.get(key, 0) >= limit_per_pair:
                continue
            seen_[key] = seen_.get(key, 0) + 1
            f.write(f"## {r['a']} (A) vs {r['b']} (B), v {r['v']:g}, dt {r['dt']:+g}, e {r['e']:g}"
                    f"{', m %g, h2 %g' % (r['m'], r['h2']) if 'fcfsm' in (r['a'], r['b']) else ''}: {r['kind']}\n")
            for tt, labs in r["trace"]:
                f.write(f"  t {tt:6.2f}  " + "; ".join(labs) + "\n")
            if r.get("trace_live"):
                f.write("  -- and in the same context, a path that never finishes:\n")
                for tt, labs in r["trace_live"]:
                    f.write(f"  t {tt:6.2f}  " + "; ".join(labs) + "\n")
            f.write("\n")


def cmd_check(a):
    os.makedirs(a.out, exist_ok=True)
    mine = ("fcfsm", "fcfse")
    pairs = [(x, x) for x in PAIRS_SELF] + [(c, o) for c in mine for o in OTHERS if o not in mine] + \
            [(o, c) for c in mine for o in OTHERS if o not in mine] + [("yield", o) for o in OTHERS if o not in mine] + \
            [(o, "yield") for o in OTHERS if o not in mine + ("yield",)]
    pairs = list(dict.fromkeys(pairs))
    jobs = []
    for v in [float(x) for x in a.v.split(",")]:
        par = Par(v=v, e=a.e, m=a.m, h2=a.h2, round2=a.round2, shield=a.shield)
        for ra, rb in pairs:
            for dt in dts(-a.dt_max, a.dt_max, a.dt_step):
                jobs.append((ra, rb, dt, par))
    with Pool(a.jobs) as pool:
        rows = pool.map(context, jobs, chunksize=8)
    summ = summarize(rows)
    tag = f"e{a.e:g}_m{a.m:g}_h{a.h2:g}_{a.round2}" + ("_shield" if a.shield else "")
    with open(os.path.join(a.out, f"check_{tag}.json"), "w") as f:
        json.dump({"par": {"e": a.e, "m": a.m, "h2": a.h2, "round2": a.round2}, "summary": summ,
                   "rows": [{k: v for k, v in r.items() if k not in ("trace", "trace_live")} for r in rows]}, f, indent=0)
    write_traces(os.path.join(a.out, f"traces_{tag}.txt"), rows)
    lines = [f"# C8 four-way stop, model check: e {a.e:g} s, m {a.m:g} s, H2 {a.h2:g} s, round 2 {a.round2}, "
             f"{'with' if a.shield else 'no'} post-start safety layer; "
             f"dt {-a.dt_max:g}..{a.dt_max:g} step {a.dt_step:g}",
             f"{'v':>3} {'A':>7} {'B':>7} | {'unsafe':>6} {'notlive':>7} | worst wait A/B (s) | restarts | cycle s | unsafe dt | not live dt"]
    for s in summ:
        lines.append(f"{s['v']:>3g} {s['a']:>7} {s['b']:>7} | {s['unsafe']:>6} {s['not_live']:>7} | "
                     f"{s['worst_waitA']:5.2f} / {s['worst_waitB']:5.2f}     | {s['max_restarts']:>8} | {s['max_cycle_s']:7.2f} | {s['unsafe_dt']} | "
                     f"{s['not_live_dt']} {'(' + ','.join(s['kinds']) + ')' if s['kinds'] else ''}")
    lines += bar_table(rows)
    open(os.path.join(a.out, f"check_{tag}.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


def bar_table(rows):
    """BAR: (c) in a car's place against a rule X in the other's, against stock yield in the same place.
    Never worse than stock: (c)'s unsafe contexts a subset of stock's, and its not-live ones too."""
    by = {}
    for r in rows:
        by.setdefault((r["v"], r["a"], r["b"]), {})[r["dt"]] = r
    out = ["# BAR: rule in one place vs X in the other, against stock yield in that place (same X, same contexts)",
           f"{'v':>3} {'place':>5} {'rule':>6} {'X':>7} | unsafe rule/stock | not live rule/stock | rule unsafe where stock is safe"
           " | rule not live where stock is | never worse"]
    for (v, a, b), cell in sorted(by.items()):
        for place, mine, x in (("A", a, b), ("B", b, a)):
            if mine not in ("fcfsm", "fcfse") or x in ("fcfsm", "fcfse"):
                continue
            stock = by.get((v, "yield", x) if place == "A" else (v, x, "yield"))
            if not stock:
                continue
            u_m = {d for d, r in cell.items() if not r["safe"]}
            u_s = {d for d, r in stock.items() if not r["safe"]}
            l_m = {d for d, r in cell.items() if not r["live"]}
            l_s = {d for d, r in stock.items() if not r["live"]}
            worse_u, worse_l = sorted(u_m - u_s), sorted(l_m - l_s)
            out.append(f"{v:>3g} {place:>5} {mine:>6} {x:>7} | {len(u_m):>5} / {len(u_s):<5} | {len(l_m):>5} / {len(l_s):<5} | "
                       f"{span(worse_u) or '-'} | {span(worse_l) or '-'} | {'yes' if not worse_u and not worse_l else 'NO'}")
    return out


def sweep_job(job):
    e, m, h2, v, dtl, round2 = job
    par = Par(v=v, e=e, m=m, h2=h2, round2=round2)
    unsafe, notlive, ww = [], [], 0.0
    first_bad = None
    for dt in dtl:
        r = context(("fcfsm", "fcfsm", dt, par))
        if not r["safe"]:
            unsafe.append(dt)
        if not r["live"]:
            notlive.append(dt)
        if r["trace"] and first_bad is None:
            first_bad = r
        ww = max(ww, r["waitA"], r["waitB"])
    return {"e": e, "m": m, "h2": h2, "v": v, "unsafe": len(unsafe), "not_live": len(notlive), "unsafe_dt": span(unsafe),
            "not_live_dt": span(notlive), "worst_wait": ww, "example": first_bad}


def cmd_sweep(a):
    os.makedirs(a.out, exist_ok=True)
    es = [float(x) for x in a.es.split(",")]
    ms = [float(x) for x in a.ms.split(",")]
    h2s = [float(x) for x in a.h2s.split(",")]
    dtl = dts(-a.dt_max, a.dt_max, a.dt_step)
    jobs = [(e, m, h2, float(v), dtl, a.round2) for v in a.v.split(",") for e in es for m in ms for h2 in h2s]
    with Pool(a.jobs) as pool:
        rows = pool.map(sweep_job, jobs)
    lines = [f"# C8 margin sweep, fcfsm self-play, round 2 {a.round2}: dt {-a.dt_max:g}..{a.dt_max:g} step {a.dt_step:g};"
             " ok = safe and live in every context",
             f"{'v':>3} {'e':>5} {'m':>5} {'H2':>4} | {'unsafe':>6} {'notlive':>7} | worst wait | ok"]
    mins = {}
    for r in sorted(rows, key=lambda r: (r["v"], r["e"], r["h2"], r["m"])):
        ok = r["unsafe"] == 0 and r["not_live"] == 0
        lines.append(f"{r['v']:>3g} {r['e']:>5g} {r['m']:>5g} {r['h2']:>4g} | {r['unsafe']:>6} {r['not_live']:>7} | "
                     f"{r['worst_wait']:6.2f}     | {'ok' if ok else '-'} {r['unsafe_dt']} {r['not_live_dt']}")
        if ok:
            key = (r["v"], r["e"], r["h2"])
            if key not in mins or r["m"] < mins[key]["m"]:
                mins[key] = r
    lines.append("# the smallest margin that is safe and live, per (v, e, H2)")
    for (v, e, h2), r in sorted(mins.items()):
        lines.append(f"v {v:g} e {e:g} H2 {h2:g}: m_min {r['m']:g} (worst wait {r['worst_wait']:.2f} s)")
    open(os.path.join(a.out, f"sweep_{a.round2}.txt"), "w").write("\n".join(lines) + "\n")
    with open(os.path.join(a.out, f"sweep_{a.round2}.json"), "w") as f:
        json.dump([{k: v for k, v in r.items() if k != "example"} for r in rows], f, indent=0)
    ex = [r["example"] for r in rows if r["example"]]
    write_traces(os.path.join(a.out, f"sweep_traces_{a.round2}.txt"), ex, limit_per_pair=3)
    print("\n".join(lines[-len(mins) - 1:]))


def cmd_exitblock(a):
    """Never enter Z unless its exit is clear: B's exit lane blocked until --block-until s."""
    os.makedirs(a.out, exist_ok=True)
    jobs = []
    for v in [float(x) for x in a.v.split(",")]:
        par = Par(v=v, e=a.e, m=a.m, h2=a.h2, exit_block=("B", a.block_until))
        for rule in ("fcfs", "fcfsr", "fcfsm", "fcfse"):
            jobs += [(rule, rule, dt, par) for dt in dts(-a.dt_max, a.dt_max, a.dt_step)]
    with Pool(a.jobs) as pool:
        rows = pool.map(context, jobs, chunksize=8)
    lines = [f"# C8 exit clear: B's exit blocked until {a.block_until:g} s (B rests at 9.5 s + dt / 2...); self-play, e {a.e:g}, "
             f"m {a.m:g}; a violation is a car in Z while its exit is blocked, or a contact"]
    for s_ in summarize(rows):
        lines.append(f"v {s_['v']:g} {s_['a']:>6}: violations {s_['unsafe']}/{s_['contexts']} ({s_['unsafe_dt'] or '-'}), "
                     f"not live {s_['not_live']}, worst wait A {s_['worst_waitA']:.2f} B {s_['worst_waitB']:.2f}")
    open(os.path.join(a.out, "exitblock.txt"), "w").write("\n".join(lines) + "\n")
    write_traces(os.path.join(a.out, "exitblock_traces.txt"), rows)
    print("\n".join(lines))


# ---- three cars ---------------------------------------------------------------------------------------
def three_job(job):
    rule, offs, par = job
    f_b = tk(8.0)
    names = "ABCD"[:len(offs) + 1]
    fs = {"B": f_b}
    for n, o in zip(names.replace("B", ""), offs):
        fs[n] = f_b + tk(o)
    M_ = Model(tuple((n, rule, fs[n]) for n in names), par)
    r = explore(M_)
    return {"rule": rule, "offs": offs, "v": par.v, "safe": r.safe, "live": r.live, "kind": r.kind,
            "states": r.states, "wait": r.worst_wait, "trace": r.trace, "e": par.e, "m": par.m, "h2": par.h2,
            "epoch_fix": par.epoch_fix, "cycle": round(r.cycle, 2)}


def cmd_three(a):
    os.makedirs(a.out, exist_ok=True)
    grid = dts(-a.dt_max, a.dt_max, a.dt_step)
    jobs = []
    for rule in a.rules.split(","):
        for fix in ((True, False) if rule == "fcfsm" and a.cars == 3 else (True,)):
            par = Par(v=a.v, e=a.e, m=a.m, h2=a.h2, epoch_fix=fix, horizon=a.horizon, det_latency=a.det_latency,
                      shield=a.shield)
            jobs += [(rule, offs, par) for offs in itertools.product(grid, repeat=a.cars - 1)]
    with Pool(a.jobs) as pool:
        rows = pool.map(three_job, jobs, chunksize=4)
    lines = [f"# C8 {a.cars} cars{' (phase changes seen exactly LMIN late)' if a.det_latency else ''} (A west, B south, C east, D north; perpendicular arms conflict, opposite do not), v {a.v:g}, "
             f"e {a.e:g}, m {a.m:g}, H2 {a.h2:g}; f_X - f_B for X != B in {-a.dt_max:g}..{a.dt_max:g} step {a.dt_step:g}"]
    by = {}
    for r in rows:
        by.setdefault((r["rule"], r["epoch_fix"]), []).append(r)
    tr = open(os.path.join(a.out, f"cars{a.cars}_traces.txt"), "w")
    for (rule, fix), rs in by.items():
        uns = [r for r in rs if not r["safe"]]
        nl = [r for r in rs if not r["live"]]
        ww = {n: max(r["wait"][n] for r in rs) for n in rs[0]["wait"]}
        lines.append(f"{rule:>6}{'' if fix else ' (round-2 timer from the dwell only)'}: contexts {len(rs)}, unsafe {len(uns)}, "
                     f"not live {len(nl)} ({', '.join(sorted({r['kind'] for r in nl}))}), worst wait "
                     + " ".join(f"{n} {w:.2f}" for n, w in ww.items())
                     + f" s, longest wait-for cycle {max(r['cycle'] for r in rs):.2f} s, states max {max(r['states'] for r in rs)}")
        for bad in (uns[:1] + nl[:1]):
            tr.write(f"## {rule}{'' if fix else ' (no epoch fix)'} offsets f_X - f_B {bad['offs']}: {bad['kind']}\n")
            for tt, labs in bad["trace"]:
                tr.write(f"  t {tt:6.2f}  " + "; ".join(labs) + "\n")
            tr.write("\n")
    tr.close()
    with open(os.path.join(a.out, f"cars{a.cars}.json"), "w") as f:
        json.dump([{k: v for k, v in r.items() if k not in ("trace", "trace_live")} for r in rows], f, indent=0)
    open(os.path.join(a.out, f"cars{a.cars}.txt"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))


# ---- e, measured --------------------------------------------------------------------------------------
def cmd_measure_e(a):
    """The rest stamp's error as four_way_game.FWPolicy reads it through arena.Observer's noise: one car
    stops at its line (plan_stop, the car's command lags); the observer's latched other_rest_t minus the
    car's own rest_t."""
    import random
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import crossing_game as X
    import four_way_game as F
    from arena import Observer
    lags = X.car_lags()
    os.makedirs(a.out, exist_ok=True)
    res = {}
    for v in [float(x) for x in a.v.split(",")]:
        errs = []
        rng = random.Random(a.seed)
        for _ in range(a.n):
            car = X.Car(F.STOP - v * 5.0 - rng.uniform(0, v * 0.1), v, lags)
            me = F.FWPolicy("B", ("fcfs_c1.5", "fcfs", 1.5), v)
            watch = F.FWPolicy("A", ("fcfs_c1.5", "fcfs", 1.5), v)
            ob = Observer(a.obs_delay, a.sigma_x, a.sigma_v, rng)
            ob.last_sample = -rng.uniform(0, 0.1)  # the sampling phase
            watch.rest_t = 0.0
            t = 0.0
            last = None
            while t < 12.0:
                s = ob.push(t, car.u, 0.0, car.v)
                if s is not None and s[0] != last:
                    last = s[0]
                    watch.perceive(s[0], s[1], s[3], s[4])
                # the watcher's latch, as FWPolicy.step does it
                sd = watch._seen(t, X.OBS + X.DECIDE)
                if sd is not None and watch.other_rest_t is None and sd[2] < 0.1 and F.STOP - 2.0 <= sd[1] < 0:
                    watch.other_rest_t = sd[0]
                car.a = F.plan_stop(car.u, car.v, v)
                car.a = car.act.step(car.a, car.v)
                car.v = max(0.0, car.v + car.a * X.DT)
                car.u += car.v * X.DT
                me.after(t, car.u, car.v)
                t += X.DT
            if me.rest_t is not None and watch.other_rest_t is not None:
                errs.append(watch.other_rest_t - me.rest_t)
        errs.sort()
        q = lambda p: errs[min(len(errs) - 1, int(p * len(errs)))]  # noqa: E731
        res[v] = {"n": len(errs), "min": errs[0], "q001": q(0.001), "q01": q(0.01), "q05": q(0.05), "median": q(0.5),
                  "q95": q(0.95), "q99": q(0.99), "q999": q(0.999), "max": errs[-1]}
        res[v]["e_99"] = max(abs(res[v]["q01"]), abs(res[v]["q99"]))
        res[v]["e_999"] = max(abs(res[v]["q001"]), abs(res[v]["q999"]))
        res[v]["e_max"] = max(abs(errs[0]), abs(errs[-1]))
    tag = f"sx{a.sigma_x:g}_sv{a.sigma_v:g}"
    with open(os.path.join(a.out, f"e_{tag}.json"), "w") as f:
        json.dump(res, f, indent=1)
    for v, r in res.items():
        print(f"v {v:g} sigma {a.sigma_x:g}/{a.sigma_v:g}: n {r['n']} stamp - rest: min {r['min']:+.2f} q0.1% {r['q001']:+.2f} "
              f"q1% {r['q01']:+.2f} median {r['median']:+.2f} q99% {r['q99']:+.2f} q99.9% {r['q999']:+.2f} max {r['max']:+.2f}"
              f" -> e99 {r['e_99']:.2f} e99.9 {r['e_999']:.2f}")


def selftest():
    # fcfsr fails at e > 0 (the tie threshold read differently, or a stamp not yet known); fcfsm holds at m = e
    base = Par(v=6.0, e=0.25, m=0.3, h2=2.0)
    bad = [dt for dt in dts(-1.5, 1.5, 0.05) if not context(("fcfsr", "fcfsr", dt, base))["safe"]]
    assert bad, "fcfsr should have a counterexample at e 0.25"
    for dt in dts(-1.5, 1.5, 0.05):
        r = context(("fcfsm", "fcfsm", dt, base))
        assert r["safe"] and r["live"], (dt, r["kind"], r["trace"])
    # far from a tie: the first car goes at its dwell's end
    r = context(("fcfsm", "fcfsm", 8.0, base))
    assert r["safe"] and r["live"] and r["waitB"] <= 1.05, r
    # four cars at once: rbl alone (fcfsm) is cyclic, the map order (fcfse) breaks it
    p4 = replace(base, horizon=40.0, det_latency=True)
    f = tk(8.0)
    r4m = explore(Model(tuple((n, "fcfsm", f) for n in "ABCD"), p4))
    r4e = explore(Model(tuple((n, "fcfse", f) for n in "ABCD"), p4))
    assert r4m.safe and not r4m.live and r4m.kind == "deadlock", r4m.kind
    assert r4e.safe and r4e.live, (r4e.kind, r4e.trace)
    print("selftest ok; fcfsr unsafe at dt", span(bad))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("cmd", nargs="?", choices=("check", "sweep", "three", "measure-e", "exitblock"))
    p.add_argument("--block-until", type=float, default=12.0)
    p.add_argument("--out", default="c8_out")
    p.add_argument("--v", default="6,10")
    p.add_argument("--e", type=float, default=0.25)
    p.add_argument("--m", type=float, default=0.3)
    p.add_argument("--h2", type=float, default=2.0)
    p.add_argument("--round2", default="rbl", choices=("rbl", "coin"))
    p.add_argument("--dt-max", type=float, default=4.0)
    p.add_argument("--dt-step", type=float, default=0.05)
    p.add_argument("--es", default="0,0.05,0.1,0.15,0.2,0.25,0.3,0.4,0.5")
    p.add_argument("--ms", default="0,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.5,0.6")
    p.add_argument("--h2s", default="1,1.5,2,2.5")
    p.add_argument("--rules", default="fcfsr,fcfsm,fcfse")
    p.add_argument("--cars", type=int, default=3, choices=(2, 3, 4))
    p.add_argument("--horizon", type=float, default=40.0)
    p.add_argument("--jobs", type=int, default=24)
    p.add_argument("--n", type=int, default=2000)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--sigma-x", type=float, default=0.3)
    p.add_argument("--sigma-v", type=float, default=0.2)
    p.add_argument("--obs-delay", type=float, default=0.19)
    p.add_argument("--det-latency", action="store_true", help="Par.det_latency (four cars)")
    p.add_argument("--shield", action="store_true", help="the safety layer after a start too (Par.shield)")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    if a.cmd == "three":
        a.v = float(a.v.split(",")[0])
    return {"check": cmd_check, "sweep": cmd_sweep, "three": cmd_three, "measure-e": cmd_measure_e,
            "exitblock": cmd_exitblock}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
