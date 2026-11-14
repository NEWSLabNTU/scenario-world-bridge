#!/usr/bin/env python3
"""Roadmap 074 C7: the zipper under congestion. A lane drop fed on both lanes near capacity.

S3's zipper was measured on one merger and a short free-flowing queue, so the benefit claimed for
it, a shorter queue upstream, could not show. Here both lanes carry a stream into the drop: N_LANE
cars in lane 1 (the through lane, arena lane -1) and N_LANE in lane 2 (the lane that ends at X_END,
arena lane -2), all at V0, with time headways (the gap over the speed, beyond S0, as N4 draws them)
h = H_MIN + Exp(h_mean - H_MIN) per lane. At h_mean 1.2-1.5 s each lane runs at 80-95 % of IDM's
capacity at that speed (h = 1 s), so together they offer about twice what one lane can take:
a queue forms, and the conventions decide how it forms and how fast it drains.

Every lane-2 car merges by merge_game's gap acceptance (zipper_queue.QueueRules' merger, its
16 rules: tau, early or zipper timing, go-on-seen-yield, drop or force at the deadline),
generalized to a stream: its slot is between the lane-1 cars around its front, as it sees them.
Every lane-1 car (and a lane-2 car once merged) follows its leader (cutin_game.follow_cruise) and
yields to a merger in its slot by its queue rule (hold, c20, c40, zip, yend; zipper_queue's
triggers). A zip car lets one merger in, then holds (one for one); a yend car yields to every
merger in its slot in the last YEND_X m. What a car sees of another is OBS + DECIDE s old and
predicted to now (decisions); car following uses the leader as it is (its radar).

Conventions:
  none      no convention: each lane-2 car draws a merge rule from all 16, each car a queue rule
            from {hold, c20, c40} (seeded per draw)
  early     lane 2 merges early and politely (e1.2d), lane 1 holds
  zipper    the zipper with the merge fixed late (z0.6yd), lane 1 zips
  zipper*   lane 1 zips, lane 2 plays its best reply: the merge rule (one for all lane-2 cars) that
            gives lane 2 the least mean delay, of the 16
  yend      lane 1 yields at the end, lane 2 its best reply as for zipper*

Measures per run: throughput past the drop (veh/min, cars passing X_END + THRU_X between the 5th
and the 5th-last), the queue upstream (the farthest car below V_Q m/s from the end, per lane and
over both, time-mean and max, and the cars in it), mean delay (each car's time to X_END + THRU_X
against free flow at V0), forced stops (a lane-2 car at rest within FORCED_X m of its lane's end),
stops (any car at rest), hard brakes (over 4 m/s^2), contact (boxes touch, or a lane-2 car past
X_END less than half across).

Usage:
  scripts/coop/zipper_stream.py kin   --out <dir> [--hms 1.2,1.5] [--draws 8] [--jobs N]
  scripts/coop/zipper_stream.py arena --port 2600 --out <dir> --best <kin dir>/best.json [--k 6] [--draws 4] [--hms ...]
  scripts/coop/zipper_stream.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import random
import statistics as st
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cutin_game as C1  # noqa: E402
import merge_game as G  # noqa: E402
import zipper_queue as Z  # noqa: E402

DT = G.DT
CAP, L, W, LANE_W = G.CAP, G.L, G.W, G.LANE_W
V0 = 12.0
N_LANE = 20
H_MIN = 1.0
H_MEANS = (1.2, 1.5)
X_END = 850.0  # m: lane 2 ends here (the arena's road coordinate)
D_HEAD = 100.0  # m: lane 1's first car this far before the end at the start; lane 2's within +- HEAD_JIT
HEAD_JIT = 15.0
THRU_X = Z.THRU_X  # 50 m past the end: a car is through
OUT_X = THRU_X + 30.0  # past this a car leaves the run (the arena parks it)
T_MAX = 300.0
V_Q = 5.0  # m/s: a car slower than this before the end is in the queue
FORCED_X = 15.0
HARD = 4.0  # m/s^2
# a merger's body reaches into lane 1 past lat 0.38 and is still in lane 2 below 0.62 (2.163 m wide in
# 3.5 m lanes): it is a lane-1 car's leader from LEAD_LAT, and a lane-2 car's until LAG_LAT
LEAD_LAT, LAG_LAT = 0.35, 0.65
# go on a seen yield only with the merger's rear ahead of the lag car's front (merge_game allows half a
# car of overlap: in a slow queue a yielding car does not open that in the 1.3 s before the bodies meet)
GOY_GAP = 0.0
# the gap a merger accepts ahead of a lag car seen at rest: zipper_queue's 1 m assumes the lag car waits
# for it; in a queue it may be starting, 0.69 s before it is seen to; a car yielding at rest leaves S0 +
# YIELD_ROOM = 3 m, so 2.5 m still lets a yielded-to merger go
REST_GAP = 2.5
CUT_LAT = 0.1  # a lane-1 car's safety layer (its radar, as it is) brakes for a merger this far across ahead of its front
M_RULES = [r[0] for r in G.A_RULES]
CONVS = {"none": (None, None), "early": ("e1.2d", "hold"), "zipper": ("z0.6yd", "zip"),
         "zipper*": ("best", "zip"), "yend": ("best", "yend")}


def headways(hm, draw, lane, n=N_LANE):
    rng = random.Random(f"c7z|{hm}|{draw}|{lane}")
    return [H_MIN + rng.expovariate(1.0 / (hm - H_MIN)) for _ in range(n - 1)]


def start_state(hm, draw):
    """Fronts and lanes of the 2 N_LANE cars at the run's start: lane 1 first, then lane 2."""
    rng = random.Random(f"c7z|{hm}|{draw}|head")
    xs, lanes = [], []
    for lane in (1, 2):
        x = X_END - D_HEAD + (rng.uniform(-HEAD_JIT, HEAD_JIT) if lane == 2 else 0.0)
        hs = headways(hm, draw, lane)
        for i in range(N_LANE):
            xs.append(x)
            lanes.append(lane)
            if i < N_LANE - 1:
                x -= L + C1.S0 + V0 * hs[i]
    return xs, lanes


def assign(conv, draw, hm, best=None):
    """Each car's merge rule (lane 2) and queue rule."""
    mr, qr = CONVS[conv]
    n = 2 * N_LANE
    if mr is None:
        rng = random.Random(f"c7z|rules|{hm}|{draw}")
        return [rng.choice(M_RULES) for _ in range(n)], [rng.choice(Z.GAME_Q) for _ in range(n)]
    if mr == "best":
        mr = best
    return [mr] * n, [qr] * n


class StreamRules:
    """Every car's rule, step by step: the kinematic model and the arena step the same object.
    Car i: lane[i] (1 or 2 at the start), its fronts along the road."""

    def __init__(self, lanes, mrules, qrules, v0=V0, x_end=X_END):
        n = len(lanes)
        self.n, self.lane, self.v0, self.x_end = n, list(lanes), v0, x_end
        self.mr = [next(r for r in G.A_RULES if r[0] == m) for m in mrules]
        self.q = [Z.Q_RULES[q] for q in qrules]
        self.decide = [None] * n
        self.dropping = [False] * n
        self.forced = [False] * n
        self.force_wait = [False] * n
        self.early_drop = [False] * n
        self.latch = [None] * n  # the merger a lane-1 car yields to
        self.yielded = [0] * n  # mergers let in
        self.done_yield = [set() for _ in range(n)]
        self.cut_in = []

    def lat_of(self, i, t):
        if self.lane[i] == 1:
            return 1.0
        if self.decide[i] is None:
            return 0.0
        u = min(1.0, (t - self.decide[i]) / G.T_LC)
        return 0.5 * (1 - math.cos(math.pi * u))

    def merger(self, i, t):
        return self.lane[i] == 2 and self.lat_of(i, t) < 1.0

    def step(self, t, x, v, lat, seen, active):
        """Accelerations at t. x, v: fronts and speeds as they are; lat: each car's progress into lane 1
        (1 for lane-1 cars; the arena measures it, the model plans it); seen: [(x, v, lat, a)] as the
        others see each car (OBS + DECIDE s old, predicted to now); active: the cars still in the run."""
        n = self.n
        acc = [0.0] * n
        on1 = sorted((i for i in active if self.lane[i] == 1 or lat[i] >= LEAD_LAT), key=lambda i: -x[i])
        on2 = sorted((i for i in active if self.lane[i] == 2 and lat[i] < LAG_LAT), key=lambda i: -x[i])
        lead1 = {on1[k]: on1[k - 1] for k in range(1, len(on1))}
        lead2 = {on2[k]: on2[k - 1] for k in range(1, len(on2))}
        seen1 = sorted((i for i in active if self.lane[i] == 1 or seen[i][2] >= 0.5), key=lambda i: -seen[i][0])
        seen_m = sorted((i for i in active if self.lane[i] == 2 and seen[i][2] < 0.5), key=lambda i: seen[i][0])
        self.cut_in = [i for i in active if self.lane[i] == 2 and CUT_LAT <= lat[i] < LEAD_LAT]
        for i in active:
            if self.merger(i, t):
                acc[i] = self._merger(i, t, x, v, seen, seen1, lead2.get(i), lead1.get(i))
            else:
                acc[i] = self._lane1(i, t, x, v, seen, lead1.get(i), seen_m)
        return acc

    def _merger(self, i, t, x, v, seen, seen1, own_lead, new_lead=None):
        _, timing, tau, go_y, end_mode = self.mr[i]
        xa, va = x[i], v[i]
        lat = self.lat_of(i, t)
        rem = self.x_end - xa
        lane1 = [j for j in seen1 if j != i]
        lag = next((j for j in lane1 if seen[j][0] < xa), None)
        lead = None
        if lane1:
            if lag is None:
                lead = lane1[-1]
            else:
                k = lane1.index(lag)
                lead = lane1[k - 1] if k > 0 else None
        gap_lag = (xa - L) - seen[lag][0] if lag is not None else math.inf
        gap_lead = (seen[lead][0] - L) - xa if lead is not None else math.inf
        v_lag = seen[lag][1] if lag is not None else 0.0
        a_lag = seen[lag][3] if lag is not None else 0.0
        if self.decide[i] is None:
            window = timing == "early" or rem <= G.ZIP_T * max(va, 1.0) + G.END_MARGIN
            need = G.BUFFER + v_lag * tau + max(0.0, v_lag ** 2 - va ** 2) / (2 * G.A_REAR)
            if v_lag < 0.5:
                need = REST_GAP
            lead_ok = gap_lead >= G.S0 + 0.5 * va
            fits = rem >= va * G.T_LC / 2 + 0.5
            if fits and (window or (lag is None and gap_lead >= G.S0 + va * G.HEADWAY)) and lead_ok and not self.dropping[i]:
                if gap_lag >= need or (go_y and a_lag <= -G.YIELD_SEEN and gap_lag > GOY_GAP):
                    self.decide[i] = t
            if self.decide[i] is None and timing == "early" and end_mode == "drop" and lag is not None:
                self.early_drop[i] = True
            if self.decide[i] is None and rem <= G.lane_change_dist(va):
                if end_mode == "force" and not self.dropping[i]:
                    self.force_wait[i] = True
                    if lead_ok:
                        self.decide[i], self.forced[i] = t, True
                else:
                    self.dropping[i] = True
            if self.decide[i] is None and (self.dropping[i] or self.early_drop[i]) and lead_ok and gap_lag >= need and fits:
                self.decide[i] = t
        dec = self.decide[i]
        if dec is not None and lead is not None and gap_lead < 60:
            aa = C1.follow_cruise(gap_lead, va, seen[lead][1], CAP, self.v0)
        elif self.early_drop[i] and not self.dropping[i] and dec is None:
            if lag is not None and v_lag < 0.5 and va < 2.0:
                aa = 0.5  # the lag car waits at rest: creep to open the gap (the dropping merger's creep)
            else:
                aa = max(-Z.EARLY_DROP, min(1.0, 1.5 * (v_lag - Z.EARLY_DV - va)))
        elif (self.dropping[i] or self.force_wait[i]) and dec is None:
            if lag is not None and v_lag < 0.5 and va < 2.0:
                aa = 0.5
            else:
                aa = -C1.A_WAIT_DECEL if va > 0.5 * max(v_lag, 1.0) or gap_lag < 0 else 0.0
        else:
            aa = max(-1.0, min(CAP.a_max, 1.5 * (self.v0 - va)))
        if dec is None and lead is not None and gap_lead < G.S0 + va * G.HEADWAY and (
                timing == "early" or rem <= G.ZIP_T * max(va, 1.0) + G.END_MARGIN):
            aa = min(aa, max(-Z.FALL_IN, C1.follow_cruise(gap_lead, va, seen[lead][1], CAP, self.v0)))
        if own_lead is not None and lat < LAG_LAT:  # still in lane 2: the car ahead in it
            aa = min(aa, C1.follow_cruise((x[own_lead] - L) - xa, va, v[own_lead], CAP, self.v0))
        if new_lead is not None:  # partly in lane 1: the car ahead there, as it is
            aa = min(aa, C1.follow_cruise((x[new_lead] - L) - xa, va, v[new_lead], CAP, self.v0))
        if (dec is None or lat < 0.5) and rem > 0:
            d = rem - (1.0 if dec is None else 0.0)
            need_e = va * va / (2 * max(d, 0.05))
            if d <= 0.05:
                aa = -CAP.b_max if va > 0 else 0.0
            elif need_e >= G.BRAKE_LATE:
                aa = min(aa, -min(CAP.b_max, need_e))
        if dec is not None and lat < 0.5:
            v_cap = max(0.0, rem - 1.0) / max(dec + G.T_LC / 2 - t, 0.1)
            aa = min(aa, 2.0 * (v_cap - va))
        return max(-CAP.b_max, min(CAP.a_max, aa))

    def _lane1(self, i, t, x, v, seen, lead, seen_m):
        xi, vi = x[i], v[i]
        a = max(-1.0, min(CAP.a_max, 1.5 * (self.v0 - vi)))
        x_lead = x[lead] if lead is not None else math.inf
        if lead is not None:
            a = min(a, C1.follow_cruise((x[lead] - L) - xi, vi, v[lead], CAP, self.v0))
        q = self.q[i]
        m = self.latch[i]
        if m is not None:
            gap = (seen[m][0] - L) - xi
            merged = seen[m][2] >= 0.5
            if merged or gap < -L / 2 or (gap < REST_GAP and vi < 0.1 and seen[m][1] < 0.1):
                if merged:
                    self.yielded[i] += 1
                self.done_yield[i].add(m)
                self.latch[i] = m = None
        if q is not None and m is None and not (q[2] == "zip" and self.yielded[i] >= 1):
            band, _, kind = q
            # the merger in this car's slot: its front ahead of this car's, behind the leader's
            cand = next((j for j in seen_m if xi < seen[j][0] < x_lead and j not in self.done_yield[i]), None)
            if cand is not None:
                xs, vs = seen[cand][0], seen[cand][1]
                gap = (xs - L) - xi
                rem_s = self.x_end - xs
                if kind == "courtesy":
                    trig = t >= G.REACT and gap < band and rem_s <= G.COURTESY_T * max(vs, 1.0)
                elif kind == "zip":
                    trig = rem_s <= G.lane_change_dist(vs) + Z.ZIP_T * max(vs, 1.0) and gap < band
                else:
                    trig = rem_s <= Z.YEND_X
                if trig:
                    self.latch[i] = m = cand
        for j in self.cut_in:  # the safety layer: a merger moving over ahead of this car's front is a cut-in
            if xi < x[j] < x_lead:
                a = min(a, C1.follow_accel((x[j] - L) - xi, vi, v[j], CAP, self.v0))
        if m is not None:
            g = (seen[m][0] - L) - xi - Z.YIELD_ROOM
            b_y = q[1]
            a = min(a, -b_y if g < G.S0 + vi * G.HEADWAY else C1.follow_cruise(g, vi, seen[m][1], CAP, self.v0))
        return max(-CAP.b_max, min(CAP.a_max, a))


class Score:
    """The run's measures, fed the truth every step."""

    def __init__(self, xs0, lanes):
        n = len(xs0)
        self.x0, self.lane, self.n = list(xs0), list(lanes), n
        self.t_thru = [None] * n
        self.crashed = [False] * n
        self.stopped = [False] * n
        self.forced = [False] * n
        self.peak = [0.0] * n
        self.vh = [deque() for _ in range(n)]
        self.contacts = 0
        self.ran_out = 0
        self.events = []  # contacts, for the record
        self.q_samples = []  # (t, tail lane 1, tail lane 2, cars queued)
        self.t_next_q = 0.0

    def push(self, t, x, v, lat, merged, active):
        for i in active:
            if self.t_thru[i] is None and x[i] >= X_END + THRU_X:
                self.t_thru[i] = t
            if v[i] < 0.1:
                self.stopped[i] = True
                if self.lane[i] == 2 and not merged[i] and X_END - x[i] <= FORCED_X:
                    self.forced[i] = True
            h = self.vh[i]
            h.append((t, v[i]))
            while t - h[0][0] > 0.5 + 1e-9:
                h.popleft()
            if t - h[0][0] >= 0.5 - 1e-9:
                self.peak[i] = max(self.peak[i], (h[0][1] - v[i]) / 0.5)
        if t >= self.t_next_q - 1e-9:
            self.t_next_q = t + 1.0
            tails = [0.0, 0.0]
            nq = 0
            for i in active:
                if x[i] < X_END and v[i] < V_Q:
                    k = 0 if (self.lane[i] == 1 or lat[i] >= 0.5) else 1
                    tails[k] = max(tails[k], X_END - x[i])
                    nq += 1
            self.q_samples.append((t, tails[0], tails[1], nq))

    def result(self, t_end):
        ts = sorted(x for x in self.t_thru if x is not None)
        n_thru = len(ts)
        k1, k2 = 4, n_thru - 5
        rate = 60.0 * (k2 - k1) / (ts[k2] - ts[k1]) if k2 > k1 and ts[k2] > ts[k1] else None
        delays = [self.t_thru[i] - (X_END + THRU_X - self.x0[i]) / V0 for i in range(self.n) if self.t_thru[i] is not None]
        d2 = [self.t_thru[i] - (X_END + THRU_X - self.x0[i]) / V0 for i in range(self.n)
              if self.t_thru[i] is not None and self.lane[i] == 2]
        d1 = [self.t_thru[i] - (X_END + THRU_X - self.x0[i]) / V0 for i in range(self.n)
              if self.t_thru[i] is not None and self.lane[i] == 1]
        qs = self.q_samples
        tail = [max(a, b) for _, a, b, _ in qs]
        busy = [s for s in qs if s[3] > 0] or qs
        return {"thru": n_thru, "rate_vpm": rate, "t_last": ts[-1] if ts else None,
                "delay": st.mean(delays) if delays else None, "delay_l1": st.mean(d1) if d1 else None,
                "delay_l2": st.mean(d2) if d2 else None,
                "queue_max": max(tail) if tail else 0.0, "queue_mean": st.mean(max(a, b) for _, a, b, _ in busy) if busy else 0.0,
                "queue_l1_max": max((a for _, a, _, _ in qs), default=0.0), "queue_l2_max": max((b for _, _, b, _ in qs), default=0.0),
                "queued_max": max((s[3] for s in qs), default=0), "queued_mean": st.mean(s[3] for s in busy) if busy else 0.0,
                "forced": sum(self.forced), "stops": sum(self.stopped), "hard": sum(p > HARD for p in self.peak),
                "contact": self.contacts, "ran_out": self.ran_out, "crashed": sum(self.crashed),
                "stuck": self.n - n_thru - sum(self.crashed), "t_end": round(t_end, 2)}


def boxes_touch(x, y, i, j):
    return abs(x[i] - x[j]) < L and abs(y[i] - y[j]) < W


def simulate(conv, hm, draw, best=None, mrules=None, qrules=None, debug=False):
    xs0, lanes = start_state(hm, draw)
    if mrules is None:
        mrules, qrules = assign(conv, draw, hm, best)
    n = len(xs0)
    R = StreamRules(lanes, mrules, qrules)
    S = Score(xs0, lanes)
    x, v, acc = list(xs0), [V0] * n, [0.0] * n
    active = set(range(n))
    hist = deque()
    t = 0.0
    age_n = int(round((G.OBS + G.DECIDE) / DT))
    while t < T_MAX - 1e-9 and active:
        lat = [R.lat_of(i, t) for i in range(n)]
        hist.append((list(x), list(v), list(acc), lat))
        if len(hist) > age_n + 1:
            hist.popleft()
        hx, hv, ha, hl = hist[0]
        age = (len(hist) - 1) * DT
        seen = [(hx[i] + hv[i] * age, hv[i], hl[i], ha[i]) for i in range(n)]
        acc = R.step(t, x, v, lat, seen, active)
        for i in active:
            v[i] = max(0.0, v[i] + acc[i] * DT)
            x[i] += v[i] * DT
        t += DT
        lat = [R.lat_of(i, t) for i in range(n)]
        merged = [R.lane[i] == 1 or lat[i] >= 0.5 for i in range(n)]
        S.push(t, x, v, lat, merged, active)
        y = [LANE_W * (1 - lat[i]) for i in range(n)]
        hit = set()
        for i in active:
            if lanes[i] == 2 and x[i] > X_END and lat[i] < 0.5:
                hit.add(i)
                S.ran_out += 1
        act = sorted(active, key=lambda i: x[i])
        for k, i in enumerate(act):
            for j in act[k + 1:]:
                if x[j] - x[i] >= L:
                    break
                if boxes_touch(x, y, i, j):
                    hit.update((i, j))
                    S.contacts += 1
                    S.events.append(f"t {t:.2f} {i}(l{lanes[i]} lat {lat[i]:.2f} x {x[i] - X_END:+.1f} v {v[i]:.1f} {mrules[i] if lanes[i] == 2 else qrules[i]}) "
                                    f"{j}(l{lanes[j]} lat {lat[j]:.2f} x {x[j] - X_END:+.1f} v {v[j]:.1f} {mrules[j] if lanes[j] == 2 else qrules[j]})")
        for i in hit:
            S.crashed[i] = True
            active.discard(i)
        for i in list(active):
            if x[i] >= X_END + OUT_X:
                active.discard(i)
    out = S.result(t)
    out["rule_l2"] = mrules[N_LANE] if conv != "none" else "mixed"
    out["events"] = " | ".join(S.events)
    if debug:
        out["_state"] = (x, v, R, active, t)
    return out


def _kin_job(job):
    conv, hm, draw = job
    if CONVS[conv][0] == "best":
        cands = {m: simulate(conv, hm, draw, best=m) for m in M_RULES}
        pick = min(cands, key=lambda m: (cands[m]["delay_l2"] if cands[m]["delay_l2"] is not None else math.inf)
                   + 100.0 * cands[m]["crashed"])
        r = dict(cands[pick])
        r["cands"] = {m: round(c["delay_l2"], 2) if c["delay_l2"] is not None else None for m, c in cands.items()}
    else:
        r = simulate(conv, hm, draw)
    return {"conv": conv, "hm": hm, "draw": draw, **r}


def fmtv(x):
    if x is None:
        return ""
    if isinstance(x, float):
        return f"{x:.3f}"
    if isinstance(x, dict):
        return json.dumps(x)
    return x


def table(rows, side):
    """Means per convention and demand."""
    lines = [f"# {side}: per convention and mean headway (s), means over draws; queue in m, delay in s",
             f"{'conv':8} {'hm':>4} {'n':>3} | {'rate v/min':>10} {'t_last':>6} {'queue max':>9} {'mean':>6} {'l1 max':>7} {'l2 max':>7} "
             f"{'queued':>6} | {'delay':>6} {'l1':>6} {'l2':>6} | {'forced':>6} {'stops':>5} {'hard':>5} {'contact':>7} "
             f"{'stuck':>5} {'crash':>5} | rule_l2"]
    agg = {}
    for conv in CONVS:
        for hm in sorted({r["hm"] for r in rows}):
            rs = [r for r in rows if r["conv"] == conv and r["hm"] == hm]
            if not rs:
                continue
            def m(k):
                xs = [float(r[k]) for r in rs if r[k] not in (None, "")]
                return st.mean(xs) if xs else math.nan
            rules = sorted({r["rule_l2"] for r in rs})
            a = {k: m(k) for k in ("rate_vpm", "t_last", "crashed", "queue_max", "queue_mean", "queue_l1_max", "queue_l2_max", "queued_mean",
                                   "delay", "delay_l1", "delay_l2", "forced", "stops", "hard", "contact", "stuck")}
            a["n"], a["rules"] = len(rs), " ".join(rules)
            agg[(conv, hm)] = a
            lines.append(f"{conv:8} {hm:4g} {len(rs):3d} | {a['rate_vpm']:10.1f} {a['t_last']:6.1f} {a['queue_max']:9.0f} {a['queue_mean']:6.0f} "
                         f"{a['queue_l1_max']:7.0f} {a['queue_l2_max']:7.0f} {a['queued_mean']:6.1f} | {a['delay']:6.1f} "
                         f"{a['delay_l1']:6.1f} {a['delay_l2']:6.1f} | {a['forced']:6.2f} {a['stops']:5.1f} {a['hard']:5.1f} "
                         f"{a['contact']:7.2f} {a['stuck']:5.2f} {a['crashed']:5.1f} | {a['rules']}")
    return lines, agg


def run_kin(args):
    from multiprocessing import Pool

    hms = [float(x) for x in args.hms.split(",")]
    jobs = [(c, hm, d) for c in CONVS for hm in hms for d in range(args.draws)]
    with Pool(args.jobs) as pool:
        rows = pool.map(_kin_job, jobs)
    os.makedirs(args.out, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(os.path.join(args.out, "stream_kin.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: fmtv(r.get(k)) for k in keys})
    lines, agg = table(rows, "kinematic")
    best = {}
    for conv in ("zipper*", "yend"):
        for hm in hms:
            picks = [r["rule_l2"] for r in rows if r["conv"] == conv and r["hm"] == hm]
            best.setdefault(conv, {})[str(hm)] = max(set(picks), key=picks.count)
    with open(os.path.join(args.out, "best.json"), "w") as f:
        json.dump(best, f, indent=1)
    lines.append(f"# lane 2's modal best reply (for the arena): {json.dumps(best)}")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "stream_kin.txt"), "w") as f:
        f.write(text)
    with open(os.path.join(args.out, "stream_kin.json"), "w") as f:
        json.dump({f"{c}|{h}": v for (c, h), v in agg.items()}, f, indent=1)
    print(text, end="")
    return 0


# --- the arena --------------------------------------------------------------------------------------
ROAD_LEN, TAPER = 1400.0, 30.0
PARK_X0, PARK_DX = ROAD_LEN - 20.0, 8.0  # 40 cars: 1380 .. 1068 m, past X_END + OUT_X
WARMUP = 15.0


class Group:
    """One road's stream on CARLA: every car arena.Car, stepped by StreamRules."""

    def __init__(self, ar, road, cars, job, rng, args):
        import arena as AR
        import merge_arena as MA
        self.AR, self.MA = AR, MA
        self.ar, self.road, self.cars, self.job = ar, road, cars, job
        self.y1, _ = ar.lane_y(road, -1)
        self.y2, _ = ar.lane_y(road, -2)
        self.xs0, self.lanes = start_state(job["hm"], job["draw"])
        self.n = len(self.cars)
        self.R = StreamRules(self.lanes, job["mrules"], job["qrules"])
        self.S = Score(self.xs0, self.lanes)
        self.seen = [MA.Seen(AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)) for _ in range(self.n)]
        self.active = set(range(self.n))
        self.parked = set()
        self.phase, self.rec = "warmup", None

    def y_lane(self, i):
        return self.y1 if self.lanes[i] == 1 else self.y2

    def front(self, c):
        return c.state()[0] + c.length / 2

    def park(self, i):
        import carla
        c = self.cars[i]
        loc = c.a.get_transform().location
        # parked in lane -1 past the taper, single file (lane -2 has no ground there)
        c.park(carla.Transform(carla.Location(x=PARK_X0 - PARK_DX * i - c.length / 2, y=self.y1, z=loc.z + 0.3),
                               carla.Rotation(yaw=0)))
        self.parked.add(i)

    def place_all(self):
        import carla
        for i, c in enumerate(self.cars):
            loc = c.a.get_transform().location
            x0 = self.xs0[i] - self.MA.ramp(WARMUP, V0)[0]
            c.park(carla.Transform(carla.Location(x=x0 - c.length / 2, y=self.y_lane(i), z=loc.z + 0.3), carla.Rotation(yaw=0)))

    def start_warmup(self):
        self.tw = self.ar.t

    def step_warmup(self):
        t = self.ar.t - self.tw
        d, vr, ar_ = self.MA.ramp(t, V0)
        d_full = self.MA.ramp(WARMUP, V0)[0]
        for i, c in enumerate(self.cars):
            x_ref = self.xs0[i] - (d_full - d)
            a = max(-4.0, min(3.0, ar_ + 0.6 * (x_ref - self.front(c)) + 1.8 * (vr - c.state()[3])))
            c.drive(a, self.y_lane(i))
        if t >= WARMUP - 1e-9:
            self.phase, self.t0 = "run", self.ar.t

    def hold_parked(self):
        import carla
        for i in self.parked:
            self.ar.pending.append(carla.command.ApplyVehicleControl(self.cars[i].a.id, carla.VehicleControl(brake=1.0, hand_brake=True)))

    def step_run(self):
        import measures as M
        t = self.ar.t - self.t0
        st_ = [c.state() for c in self.cars]
        x = [s[0] + self.cars[i].length / 2 for i, s in enumerate(st_)]
        v = [s[3] for s in st_]
        lat_m = [1.0 if self.lanes[i] == 1 else max(0.0, min(1.0, (st_[i][1] - self.y2) / (self.y1 - self.y2)))
                 for i in range(self.n)]
        lat_plan = [self.R.lat_of(i, t) for i in range(self.n)]
        for i in self.active:
            self.seen[i].push(t, x[i], lat_m[i], v[i])
        seen = []
        for i in range(self.n):
            s = self.seen[i].at(t, (x[i], v[i], lat_m[i], 0.0))
            seen.append((s[0], s[1], s[2], s[3]))
        acc = self.R.step(t, x, v, lat_m, seen, self.active)
        for i in self.active:
            c = self.cars[i]
            if self.lanes[i] == 2 and lat_plan[i] < 1.0:
                lead = max(6.0, 0.7 * v[i]) / max(v[i], 1.0)
                lr = self.R.lat_of(i, t + lead)
                c.drive(acc[i], self.y2 + (self.y1 - self.y2) * lr)
            else:
                c.drive(acc[i], self.y1)
        self.hold_parked()
        merged = [self.lanes[i] == 1 or lat_m[i] >= 0.5 for i in range(self.n)]
        self.S.push(t, x, v, lat_m, merged, self.active)
        hit = set()
        for i in self.active:
            if self.lanes[i] == 2 and ((x[i] > X_END and lat_plan[i] < 0.5) or (x[i] > X_END + TAPER and lat_m[i] < 0.5)):
                hit.add(i)
                self.S.ran_out += 1
        act = sorted(self.active, key=lambda i: x[i])
        poses = {}
        for k, i in enumerate(act):
            for j in act[k + 1:]:
                if x[j] - x[i] >= L + 1.0:
                    break
                for q in (i, j):
                    if q not in poses:
                        p = self.cars[q].pose()
                        poses[q] = M.corners(p[0], p[1], p[2], p[5], p[6])
                if M.clearance(poses[i], poses[j]) <= 0.01:
                    hit.update((i, j))
                    self.S.contacts += 1
        for i in hit:
            self.S.crashed[i] = True
            self.cars[i].dirty = True
            self.active.discard(i)
            self.park(i)
        for i in list(self.active):
            if x[i] >= X_END + OUT_X:
                self.active.discard(i)
                self.park(i)
        if not self.active or t >= T_MAX - 1e-9:
            out = self.S.result(t)
            self.rec = {k: self.job[k] for k in ("conv", "hm", "draw")}
            self.rec.update(out)
            self.rec["rule_l2"] = self.job["mrules"][N_LANE] if self.job["conv"] != "none" else "mixed"
            for i in list(self.active):
                self.park(i)
            self.phase = "done"


def arena_jobs(args):
    best = json.load(open(args.best))
    out = []
    for hm in [float(x) for x in args.hms.split(",")]:
        for d in range(args.draws):
            for conv in CONVS:
                b = best.get(conv, {}).get(str(hm))
                mr, qr = assign(conv, d, hm, b)
                out.append({"conv": conv, "hm": hm, "draw": d, "mrules": mr, "qrules": qr})
    return out


def run_arena(args):
    import arena as AR
    import carla

    rng = random.Random(args.seed)
    work = deque(arena_jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k, AR.xodr_lane_drop(args.k, ROAD_LEN, X_END, TAPER))
    rows, t0, replaced = [], time.monotonic(), 0
    try:
        cars = []
        for r in range(args.k):
            actors = [ar.spawn(r, -1, PARK_X0 - PARK_DX * i) for i in range(2 * N_LANE)]
            ar.put_physics(actors)
            cars.append([AR.Car(ar, a) for a in actors])
        while work:
            groups = []
            for r in range(args.k):
                if not work:
                    break
                for i, c in enumerate(cars[r]):
                    if not c.a.is_alive or getattr(c, "dirty", False):
                        if c.a.is_alive:
                            c.a.destroy()
                        a = ar.spawn(r, -1, PARK_X0 - PARK_DX * i)
                        ar.put_physics([a])
                        cars[r][i] = AR.Car(ar, a)
                        replaced += 1
                g = Group(ar, r, cars[r], work.popleft(), rng, args)
                g.place_all()
                groups.append(g)
            for _ in range(3):
                for g in groups:
                    for c in g.cars:
                        c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            for g in groups:
                g.start_warmup()
            while any(g.phase != "done" for g in groups):
                for g in groups:
                    if g.phase == "warmup":
                        g.step_warmup()
                    elif g.phase == "run":
                        g.step_run()
                    else:
                        g.hold_parked()
                ar.tick()
            rows += [g.rec for g in groups]
            print(f"  {len(rows)}/{n_all} groups, {time.monotonic() - t0:.0f} s, {replaced} cars replaced", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(os.path.join(args.out, "stream_arena.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: fmtv(r.get(k)) for k in keys})
    lines, agg = table(rows, f"arena (port {args.port}, k {args.k})")
    lines.append(f"# wall {time.monotonic() - t0:.0f} s, {replaced} cars replaced")
    with open(os.path.join(args.out, "stream_arena.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "stream_arena.json"), "w") as f:
        json.dump({f"{c}|{h}": v for (c, h), v in agg.items()}, f, indent=1)
    print("\n".join(lines))
    return 0


def selftest():
    xs, lanes = start_state(1.2, 0)
    assert len(xs) == 2 * N_LANE and lanes.count(2) == N_LANE and xs[0] == X_END - D_HEAD
    # the gaps: at least IDM's at H_MIN
    assert all(xs[i] - xs[i + 1] >= L + C1.S0 + V0 * H_MIN - 1e-9 for i in range(N_LANE - 1))
    # a zipper stream drains without contact; every car gets through
    r = simulate("zipper", 1.5, 0)
    assert r["contact"] == 0 and r["ran_out"] == 0 and r["thru"] == 2 * N_LANE, r
    assert r["rate_vpm"] is not None and 10 < r["rate_vpm"] < 60 and r["queue_max"] > 0, r
    # early merge: a lane-1 queue forms, lane 2 empties early
    e = simulate("early", 1.5, 0)
    assert e["contact"] == 0 and e["thru"] == 2 * N_LANE, e
    # the rules: a zip car lets one merger in, then holds
    R = StreamRules([1, 2, 2], ["e1.2d"] * 3, ["zip"] * 3)
    R.yielded[0] = 1
    a = R._lane1(0, 5.0, [800.0, 810.0, 805.0], [5.0] * 3, [(800.0, 5.0, 1.0, 0.0), (810.0, 5.0, 0.0, 0.0), (805.0, 5.0, 0.0, 0.0)],
                 None, [2, 1])
    assert R.latch[0] is None and a > -1.0, (R.latch, a)
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", nargs="?", choices=("kin", "arena"))
    p.add_argument("--out")
    p.add_argument("--hms", default=",".join(str(h) for h in H_MEANS))
    p.add_argument("--draws", type=int, default=8)
    p.add_argument("--jobs", type=int, default=16)
    p.add_argument("--port", type=int)
    p.add_argument("--k", type=int, default=6)
    p.add_argument("--best")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--obs-delay", type=float, default=G.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    return run_kin(a) if a.mode == "kin" else run_arena(a)


if __name__ == "__main__":
    sys.exit(main())
