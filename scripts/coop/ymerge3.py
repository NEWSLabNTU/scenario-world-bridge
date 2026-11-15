#!/usr/bin/env python3
"""Roadmap 074 N5's swb check: the 45 deg Y merge with three cars, the controllers and the scoring
shared by the arena (ymerge3_arena.py) and swb (n5_swb.py, ymerge3_agent.py).

A on the diagonal; B on the main road and C behind it at a time headway H_FOLLOW, both at v. This
is ymerge_game.simulate_stream with one stream car past B (N = 3), cut into one controller per car
so that each can run in its own process (swb's agents) or all in one (the arena), on what each car
perceives of the others (arena.Observer: 10 Hz, delayed, noisy):

  A  negotiates with the first main-road car not clearly ahead of it (its front less than half a
     car ahead of A's), with a fresh YPolicy (its gap rule) whenever that car changes, fed that
     car's observations so far; follows the main-road car nearest ahead of it
  B  YPolicy "hold" against A (keeps going, follows A once A is ahead, its safety layer brakes for
     an A level with it in the zone)
  C  the same against A, and follows B (cutin_game.follow_cruise)

Positions are u, a car's front past Z's near edge along its own path (both paths reach M at
u = Z_LEN, so ou - u - L is the merged-lane gap), as ymerge_game's.

score3 judges a run from per-tick true states: A's slot (main-road cars ahead of A when it reaches
M), contact (A with B or C on the boxes, or C into B), each car's lost progress over HORIZON, its
peak braking over PEAK_W, its risk against A (measures.rho) and its utility (measures.utility),
as simulate_stream's rows do (lost 1 for a car in a contact, and for A if it never reaches EXIT).
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as G  # noqa: E402
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402
import ymerge_game as Y  # noqa: E402

V = 10.0
H_FOLLOW = 1.5  # s, C's time headway behind B (N4's mean-field headway)
DTS = (-1.0, -0.3, 0.0, 0.3, 1.0)
A_RULES = Y.STREAM_A_RULES  # t0.5yp2, t1.5yp2, t3yp2: re-tested on every slot
L, W = Y.L, Y.W_CAR
T_END = G.T_MAX
CONTACT_EPS = 0.0


def start_u(dt, v=V, h=H_FOLLOW):
    """The cars' u at the run's start: A and B as ymerge_game's pair, C h s and a car behind B."""
    ub = -v * (G.T_MEAN - dt / 2)
    return {"A": -v * (G.T_MEAN + dt / 2), "B": ub, "C": ub - v * h - L}


class Controller:
    """One car's driving: role A, B or C (see the module's doc). observe() takes what this car
    perceives of another (stamped at the time it was seen), step() returns the commanded
    acceleration, after() closes the tick. hold_stop: the YPolicy's (the arena keeps the clip to 0
    and its actuator brakes a held car to rest)."""

    def __init__(self, role, rule_a, v=V, hold_stop=False):
        self.role, self.v0, self.hold_stop = role, v, hold_stop
        self.ra = next(r for r in Y.A_RULES if r[0] == rule_a)
        self.hist = {k: [] for k in "ABC" if k != role}  # (t, u, v, a) as seen
        self.facing = None
        self.pol = None if role == "A" else Y.YPolicy("B", Y.HOLD, v, hold_stop=hold_stop)
        self.facings = []  # (t, car) each time A's counterpart changed

    def observe(self, t, who, u, v, a):
        self.hist[who].append((t, u, v, a))
        if self.role != "A" and who == "A":
            self.pol.perceive(t, u, v, a)
        elif self.role == "A" and who == self.facing and self.pol is not None:
            self.pol.perceive(t, u, v, a)

    def seen_now(self, who, t):
        """The latest observation of `who`, carried to t at its seen speed: (u, v) or None."""
        h = self.hist.get(who)
        if not h:
            return None
        ts, u, v, _ = h[-1]
        return u + v * max(0.0, t - ts), v

    def step(self, t, u, v):
        if self.role == "A":
            seen = {k: self.seen_now(k, t) for k in "BC"}
            f = next((k for k in "BC" if seen[k] is not None and seen[k][0] < u + L / 2), None)
            if f != self.facing:
                self.facing = f
                self.facings.append((round(t, 2), f))
                self.pol = None
                if f is not None:
                    self.pol = Y.YPolicy("A", self.ra, self.v0, hold_stop=self.hold_stop)
                    for o in self.hist[f]:
                        self.pol.perceive(*o)
            a = self.pol.step(t, u, v) if self.pol is not None else G.drive(v, self.v0)
            ahead = [s for s in seen.values() if s is not None and s[0] >= u + L / 2]
            if ahead:
                lu, lv = min(ahead)
                a = min(a, Y.follow_far(u, v, lu, lv, self.v0))
            return a
        a = self.pol.step(t, u, v)
        if self.role == "C":
            sb = self.seen_now("B", t)
            if sb is not None:
                a = min(a, C1.follow_cruise(sb[0] - u - L, v, sb[1], Y.CAP, self.v0))
        return a

    def after(self, t, u, v):
        if self.pol is not None:
            self.pol.after(t, u, v)

    def record(self):
        p = self.pol
        out = {"shield": int(bool(p and p.shield))}
        if p is not None:
            out.update(p.clip_record(self.role))
        if self.role == "A":
            out["facings"] = " ".join(f"{t:g}:{k}" for t, k in self.facings)
        return out


def score3(ticks, v0=None, boxes=None, eps=CONTACT_EPS):
    """A run's judgement. ticks: [(t, {k: (u, v, x, y, yaw)})] for k in A, B, C, t from the run's
    start, poses in any one Cartesian frame (right- or left-handed alike); v0: each car's speed at
    the start (default the first tick's); boxes: {k: (L, W)} (default the model's)."""
    boxes = boxes or {k: (L, W) for k in "ABC"}
    t0, s0 = ticks[0]
    v0 = v0 or {k: s0[k][1] for k in "ABC"}
    u0 = {k: s0[k][0] for k in "ABC"}
    pair = {k: M.Measures(b_max=(G.B_MAX, G.B_MAX), pet_sample=G.PET_SAMPLE) for k in "BC"}
    hist = {k: deque() for k in "ABC"}
    peak = {k: 0.0 for k in "ABC"}
    hit = {k: False for k in "BC"}
    rear = False
    slot, t_m, s_h = None, {k: None for k in "ABC"}, None
    t_end, end = ticks[-1]
    for t, s in ticks:
        for k in "ABC":
            h = hist[k]
            h.append((t, s[k][1]))
            while h and t - h[0][0] > G.PEAK_W + 1e-9:
                h.popleft()
            if t - h[0][0] >= G.PEAK_W - 0.026:
                peak[k] = max(peak[k], (h[0][1] - s[k][1]) / (t - h[0][0]))
            if t_m[k] is None and s[k][0] >= Y.Z_LEN:
                t_m[k] = t
        if slot is None and s["A"][0] >= Y.Z_LEN:
            slot = sum(s[k][0] > s["A"][0] for k in "BC")
        if s_h is None and t >= G.HORIZON - 1e-9:
            s_h = {k: s[k][0] - u0[k] for k in "ABC"}

        def pose(k):
            u, v, x, y, yaw = s[k]
            return (x, y, yaw, v * math.cos(yaw), v * math.sin(yaw), *boxes[k])

        pa = pose("A")
        for k in "BC":
            if abs(s[k][0] - s["A"][0]) < 30:
                pair[k].push_poses(t, pa, pose(k))
                if pair[k].clear <= eps:
                    hit[k] = True
        if s["B"][0] - s["C"][0] - boxes["B"][0] <= eps:
            rear = True
        if hit["B"] or hit["C"] or rear:
            t_end, end = t, s
            break
        if t >= G.HORIZON - 1e-9 and slot is not None and s["A"][0] >= Y.EXIT + 2:
            t_end, end = t, s
            break
    contact = hit["B"] or hit["C"] or rear
    if s_h is None:
        s_h = {k: end[k][0] - u0[k] for k in "ABC"}
    lost = {"A": 1.0 if contact or slot is None or end["A"][0] < Y.EXIT else M.lost_progress(v0["A"], s_h["A"])}
    for k in "BC":
        lost[k] = 1.0 if hit[k] or rear else M.lost_progress(v0[k], s_h[k])
    rho = {k: M.rho(pair[k].ttc2d, pair[k].thw, math.inf, pair[k].clear) for k in "BC"}
    u = {"A": M.utility(lost["A"], max(rho.values()), peak["A"], G.B_MAX, contact=hit["B"] or hit["C"])}
    for k in "BC":
        u[k] = M.utility(lost[k], rho[k], peak[k], G.B_MAX, contact=hit[k] or rear)
    pet = {k: pair[k]._pet() for k in "BC"}
    clear = min(pair[k].clear for k in "BC")
    return {"slot": "" if slot is None else slot, "contact": int(contact), "rear": int(rear),
            "uA": round(u["A"], 4), "uB": round(u["B"], 4), "uC": round(u["C"], 4),
            "lostA": round(lost["A"], 4), "lostB": round(lost["B"], 4), "lostC": round(lost["C"], 4),
            "peakA": round(peak["A"], 2), "peakB": round(peak["B"], 2), "peakC": round(peak["C"], 2),
            "rhoB": round(rho["B"], 4), "rhoC": round(rho["C"], 4),
            "petB": round(pet["B"], 3) if math.isfinite(pet["B"]) else "inf",
            "petC": round(pet["C"], 3) if math.isfinite(pet["C"]) else "inf",
            "clear": round(clear, 3) if math.isfinite(clear) else "inf", "t_run": round(t_end - t0, 2)}


def kinematic(rule, dt, v=V, h=H_FOLLOW, lags=None):
    """ymerge_game.simulate_stream with N = 3 and its utilities per car. Its peaks are the commanded
    deceleration's (score3's: the speed's drop over PEAK_W), so a hard stop scores harder here."""
    r = Y.simulate_stream(rule, dt, v, [h], lags)
    ua = M.utility(r["lost_a"], max(r["rho"]), r["peak_a"], G.B_MAX, contact=r["contact"])
    ub, uc = (M.utility(ls, rh, pk, G.B_MAX, contact=hi)
              for ls, rh, pk, hi in zip(r["lost_stream"], r["rho"], r["peak_stream"], r["hit"]))
    return {"slot": "" if r["slot"] is None else r["slot"], "contact": int(r["contact"]),
            "uA": round(ua, 4), "uB": round(ub, 4), "uC": round(uc, 4)}


def selftest():
    """The controllers on exact perception, stepped as simulate_stream steps its cars, against it."""
    lags = G.car_lags()
    worst = 0.0
    for rule in A_RULES:
        for dt in DTS:
            us = start_u(dt)
            cars = {k: G.Car(us[k], V, lags) for k in "ABC"}
            ctl = {k: Controller(k, rule, V, hold_stop=G.HOLD_STOP) for k in "ABC"}
            world = Y.World(before=-min(us.values()) + Y.Z_LEN + 50.0)
            t = 0.0
            ticks = [(t, {k: (cars[k].u, V, *world.pose("A" if k == "A" else "B", cars[k].u, V)[:3]) for k in "ABC"})]
            while t < T_END - 1e-9:
                for k in "ABC":
                    for o in "ABC":
                        if o != k:
                            ctl[k].observe(t, o, cars[o].u, cars[o].v, cars[o].a)
                acc = {k: ctl[k].step(t, cars[k].u, cars[k].v) for k in "ABC"}
                for k in "ABC":
                    me = cars[k]
                    me.a = me.act.step(acc[k], me.v) if me.act is not None else acc[k]
                    me.v = max(0.0, me.v + me.a * G.DT)
                    me.u += me.v * G.DT
                    ctl[k].after(t, me.u, me.v)
                t += G.DT
                st = {}
                for k in "ABC":
                    x, y, yaw, *_ = world.pose("A" if k == "A" else "B", cars[k].u, cars[k].v)
                    st[k] = (cars[k].u, cars[k].v, x, y, yaw)
                ticks.append((t, st))
                if t > G.HORIZON + 5 and cars["A"].u > Y.EXIT + 2:
                    break
            got, ref = score3(ticks, v0={k: V for k in "ABC"}), kinematic(rule, dt, lags=lags)
            d = max(abs(got[k] - ref[k]) for k in ("uA", "uB", "uC"))
            worst = max(worst, d)
            print(f"{rule:>8} dt {dt:+.1f}: controllers slot {got['slot']} contact {got['contact']} uA {got['uA']:+.3f} "
                  f"uB {got['uB']:+.3f} uC {got['uC']:+.3f} | simulate_stream slot {ref['slot']} uA {ref['uA']:+.3f} "
                  f"uB {ref['uB']:+.3f} uC {ref['uC']:+.3f}")
    print(f"largest payoff difference {worst:.3f}")
    return worst


if __name__ == "__main__":
    selftest()
