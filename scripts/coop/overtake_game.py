#!/usr/bin/env python3
"""Roadmap 074 S6: overtaking via the oncoming lane (an oppose plus a merge), in a kinematic model.

A two-lane two-way road. A follows a slow lead L in its own lane (lane y = -LANE_W/2, along +x) and
would drive at V_DES; B comes the other way in the oncoming lane (y = +LANE_W/2, along -x). To pass
L, A pulls into B's lane (a cosine over T_LC), drives past L, and returns once its rear is RET_GAP
ahead of L's front. L keeps its speed and plays no game. The context is

  g      A's gap to L at the start (m, A's front to L's rear), A at L's speed vL
  vB     B's speed (m/s)
  tm     B's time to meet A at the start (s): the fronts' distance over the closing speed vA + vB

Rules, each on what it perceives of the other (OBS + DECIDE s old, predicted to now):

  A  wait      never pass (follow L)
     go        pull out at once and pass, whatever B does
     g<tau>    pass iff, going now, A would be back in its lane (lat < 0.5) tau s before it meets B
               (B at its seen speed); re-tested every step until A goes
     g<tau>a   the same, with an abort: while A is not yet past L's front, it re-tests every step,
               and if A would no longer be back before meeting B it aborts: brakes behind L and
               returns to its lane once its front is behind L's rear
  B  hold      keep going
     brake     on seeing A in its lane (lat > 0.3) coming at it, brake (B_BRAKE) until A is back
     right     the same trigger, move right onto the shoulder by B_RIGHT m (a cosine over 2 s)
     brakeright both
     speedup   the same trigger, B speeds up (toward vB + SPEEDUP at full drive): it will not be passed

There is no safety layer beyond these rules: A's abort is its only one, B's brake its only one.

Payoffs: the general utility (measures.py) on oriented boxes. A's risk is the worse of A-B and
A-L, B's is A-B's; lost progress is over H against each car's desired speed (V_DES for A, vB for B),
so A loses progress by staying behind L. Per context, the game is solved (crossing_game.solve:
pure and 2 x 2 mixed) and classed:

  P   A passes in every equilibrium
  W   A stays behind L in every equilibrium (wait, or an abort)
  C   chicken with B: an equilibrium where A passes because B gives way (brake or right), and one
      where A stays behind and B holds
  X   both outcomes in equilibrium, but not that pattern
  !   contact in an equilibrium

Usage:
  scripts/coop/overtake_game.py --out <dir> [--gs 10,25] [--vbs 15,22] [--tms 4..16] [--car-lags] [--jobs N]
  scripts/coop/overtake_game.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from collections import deque
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402

DT, HORIZON, T_MAX = X.DT, X.HORIZON, 40.0
CAP = C1.TYPES["car"]
L, W_CAR = X.L, X.W_CAR
LANE_W = 3.5
V_L, V_DES = 15.0, 25.0  # m/s: the lead's speed, A's desired speed
T_LC = 3.0  # s, A's lane change out and back
RET_GAP = 5.0  # m: A returns once its rear is this far ahead of L's front
B_BRAKE, B_RIGHT, T_RIGHT = 4.0, 1.0, 2.0  # m/s^2; m onto the shoulder over s
ABORT_DECEL = 5.0  # m/s^2: A's abort braking
OBS, DECIDE = X.OBS, X.DECIDE
TAUS = (0.5, 1.5, 3.0)
A_RULES = ["wait", "go"] + [f"g{t:g}" for t in TAUS] + [f"g{t:g}a" for t in TAUS]
B_RULES = ["hold", "brake", "right", "brakeright", "speedup"]
BIAS_D = 150.0  # m: A's error in B's speed is full at this distance and fades to none as B closes
SPEEDUP = 5.0  # m/s: a B that speeds up, on seeing A pull out, toward vB + this


def cos_lat(u):
    u = max(0.0, min(1.0, u))
    return 0.5 * (1 - math.cos(math.pi * u)), 0.5 * math.pi * math.sin(math.pi * u)


def predict_margin(xa, va, lat, xl, xb, vb):
    """Time A would be back in its lane (lat < 0.5) before meeting B, going (or going on) now: a
    forward run of A at full drive toward V_DES from its lane change's current point, L at V_L, B at
    its seen speed. -inf: A would meet B while still in B's lane."""
    t, dtp = 0.0, 0.1
    u = cos_inverse(lat)
    returning = False
    while t < 30.0:
        if not returning and xa - L >= xl + RET_GAP:
            returning = True
        u = max(0.0, u - dtp / T_LC) if returning else min(1.0, u + dtp / T_LC)
        lat_t = cos_lat(u)[0]
        va = min(V_DES, va + CAP.a_max * dtp)
        xa += va * dtp
        xl += V_L * dtp
        xb -= vb * dtp
        t += dtp
        if returning and lat_t < 0.5:
            return max(0.0, xb - xa) / max(va + vb, 0.1)  # when A is back, B is this long from meeting it
        if lat_t >= 0.5 and xb <= xa:
            return -math.inf
    return -math.inf


def cos_inverse(lat):
    lat = max(0.0, min(1.0, lat))
    return math.acos(1 - 2 * lat) / math.pi


class RulesA:
    def __init__(self, name):
        self.name = name
        self.tau = None if name in ("wait", "go") else float(name[1:].rstrip("a"))
        self.abort_ok = name.endswith("a")
        self.state = "behind"  # behind -> out -> pass -> back -> done | abort -> behind_done
        self.u = 0.0
        self.aborted = self.passed = False
        # the commit point: A's front relative to L's front (m) when its predicted margin first went
        # negative during the pass (None: never). Over 0: the danger showed only past L's front, where
        # an abort can no longer drop back (074 S6)
        self.neg_rel = None

    def lateral(self, dt):
        if self.state == "out":
            self.u = min(1.0, self.u + dt / T_LC)
        elif self.state in ("back", "abort_back"):
            self.u = max(0.0, self.u - dt / T_LC)
        lat, dl = cos_lat(self.u)
        sign = -1.0 if self.state in ("back", "abort_back") else 1.0
        return lat, (LANE_W / T_LC) * dl * sign if self.state in ("out", "back", "abort_back") and 0 < self.u < 1 else 0.0

    def step(self, xa, va, xl, vl, seen_b):
        xb, vb = seen_b
        lat = cos_lat(self.u)[0]
        if self.state == "behind" and self.name != "wait":
            if self.name == "go" or predict_margin(xa, va, 0.0, xl, xb, vb) >= self.tau:
                self.state = "out"
        elif self.state in ("out", "pass"):
            m = predict_margin(xa, va, lat, xl, xb, vb)
            if m < 0.0 and self.neg_rel is None:
                self.neg_rel = xa - xl
            if self.abort_ok and xa < xl and m < 0.0:
                self.state, self.aborted = "abort", True
            elif xa - L >= xl + RET_GAP:
                self.state, self.passed = "back", True
            elif self.state == "out" and self.u >= 1.0:
                self.state = "pass"
        if self.state == "abort" and xa < xl - L - 1.0:
            self.state = "abort_back"
        if self.state in ("back",) and self.u <= 0.0:
            self.state = "done"
        if self.state == "abort_back" and self.u <= 0.0:
            self.state = "behind_done"
        # longitudinal
        gap_l = (xl - L) - xa
        if self.state in ("behind", "behind_done", "abort_back"):
            return C1.follow_cruise(gap_l, va, vl, CAP, V_DES)
        if self.state == "abort":
            return -ABORT_DECEL if xa > xl - L - 1.5 else C1.follow_cruise(gap_l, va, vl, CAP, V_DES)
        return max(-1.0, min(CAP.a_max, 1.5 * (V_DES - va)))


class RulesB:
    def __init__(self, name, vb0):
        self.name, self.vb0 = name, vb0
        self.trig = False
        self.t_trig = None

    def step(self, t, xb, vb, seen_a):
        xa_s, lat_s = seen_a
        coming = lat_s > 0.3 and xa_s < xb
        if coming and not self.trig:
            self.trig, self.t_trig = True, t
        if self.trig and not coming and lat_s <= 0.3:
            self.trig = False
        a = max(-1.0, min(CAP.a_max, 1.5 * (self.vb0 - vb)))
        if self.trig and self.name in ("brake", "brakeright"):
            a = -B_BRAKE if vb > 0 else 0.0
        if self.t_trig is not None and self.name == "speedup":
            a = max(-1.0, min(CAP.a_max, 1.5 * (self.vb0 + SPEEDUP - vb)))
        return a

    def offset(self, t):
        if self.name in ("right", "brakeright") and self.t_trig is not None:
            return B_RIGHT * cos_lat((t - self.t_trig) / T_RIGHT)[0]
        return 0.0


@dataclass
class Out:
    u: tuple
    contact: bool
    passed: bool
    aborted: bool
    b_gave: bool  # B braked or moved right
    who: str  # contact partner ("B", "L") or ""
    o_lost: tuple
    clear_ab: float
    neg_rel: float | None = None  # RulesA.neg_rel


def simulate(ra, rb, g, vb, tm, lags=None, bias=0.0):
    """bias: A's error in B's seen speed (-0.2: A takes B as 20 % slower than it is) when B is BIAS_D m
    or more away, fading linearly to none as B closes (a far car's speed is the hard one to judge)."""
    xl = 0.0  # L's front
    xa = xl - L - g  # A's front
    va = V_L
    xb = xa + tm * (va + vb)  # B's front
    vbb = vb
    A, B = RulesA(ra), RulesB(rb, vb)
    act = [X.Actuator(lags) if lags else None for _ in range(2)]
    hist = deque()
    mab = M.Measures(b_max=(CAP.b_max, CAP.b_max), pet_sample=X.PET_SAMPLE)
    mal = M.Measures(b_max=(CAP.b_max, CAP.b_max), pet_sample=X.PET_SAMPLE)
    vh = [deque(), deque()]
    peak = [0.0, 0.0]
    s = [0.0, 0.0]
    s_h = [None, None]
    contact, who = False, ""
    lat, vy = 0.0, 0.0
    aa = ab = 0.0
    t = 0.0
    while t < T_MAX - 1e-9:
        hist.append((t, xa, va, lat, xb, vbb))
        while len(hist) > 2 and hist[1][0] <= t - (OBS + DECIDE + 1.0):
            hist.popleft()
        sd = next((h for h in reversed(hist) if h[0] <= t - OBS - DECIDE), hist[0])
        age = t - sd[0]
        vb_seen = sd[5] * (1.0 + bias * min(1.0, max(0.0, sd[4] - sd[1]) / BIAS_D))
        aa = A.step(xa, va, xl, V_L, (sd[4] - vb_seen * age, vb_seen))
        ab = B.step(t, xb, vbb, (sd[1] + sd[2] * age, sd[3]))
        if lags:
            aa, ab = act[0].step(aa), act[1].step(ab)
        va = max(0.0, va + aa * DT)
        vbb = max(0.0, vbb + ab * DT)
        xa += va * DT
        xb -= vbb * DT
        xl += V_L * DT
        t += DT
        lat, vy = A.lateral(DT)
        for k, vv in enumerate((va, vbb)):
            s[k] += vv * DT
            if s_h[k] is None and t >= HORIZON - 1e-9:
                s_h[k] = s[k]
            h = vh[k]
            h.append((t, vv))
            while t - h[0][0] > X.PEAK_W + 1e-9:
                h.popleft()
            if t - h[0][0] >= X.PEAK_W - 1e-9:
                peak[k] = max(peak[k], (h[0][1] - vv) / X.PEAK_W)
        vxa = math.sqrt(max(0.0, va * va - vy * vy))
        pa = (xa - L / 2, -LANE_W / 2 + LANE_W * lat, math.atan2(vy, max(vxa, 0.1)), vxa, vy, L, W_CAR)
        pb = (xb + L / 2, LANE_W / 2 + B.offset(t), math.pi, -vbb, 0.0, L, W_CAR)
        pl = (xl - L / 2, -LANE_W / 2, 0.0, V_L, 0.0, L, W_CAR)
        mab.push_poses(t, pa, pb)
        mal.push_poses(t, pa, pl)
        if mab.clear <= 0.0 or mal.clear <= 0.0:
            contact, who = True, ("B" if mab.clear <= 0.0 else "L")
            break
        if t >= HORIZON - 1e-9 and xb < xa - 30 and A.state in ("behind", "done", "behind_done"):
            break
    lost = tuple(1.0 if contact else M.lost_progress(v0, s_h[k] if s_h[k] is not None else s[k])
                 for k, v0 in enumerate((V_DES, vb)))
    r_ab = M.rho(mab.ttc2d, mab.thw, math.inf, mab.clear)
    r_al = M.rho(mal.ttc2d, mal.thw, math.inf, mal.clear)
    ua = M.utility(lost[0], max(r_ab, r_al), peak[0], CAP.b_max, 0.0, contact)
    ub = M.utility(lost[1], r_ab, peak[1], CAP.b_max, 0.0, contact)
    return Out((ua, ub), contact, A.passed and not contact, A.aborted, B.t_trig is not None and rb in ("brake", "right", "brakeright"),
               who, lost, mab.clear, A.neg_rel)


def context(job):
    g, vb, tm, lags, bias = job
    outs = {(a, b): simulate(a, b, g, vb, tm, lags, bias) for a in A_RULES for b in B_RULES}
    rows = []
    for subset in ("all", "no_abort", "b_hold"):
        a_acts = [a for a in A_RULES if not (subset == "no_abort" and a.endswith("a"))]
        b_acts = ["hold", "speedup"] if subset == "b_hold" else B_RULES
        ua = {(a, b): outs[(a, b)].u[0] for a in a_acts for b in b_acts}
        ub = {(a, b): outs[(a, b)].u[1] for a in a_acts for b in b_acts}
        pure, mixed = X.solve(ua, ub, a_acts, b_acts)
        cost = {k: -(ua[k] + ub[k]) for k in ua}
        eqc = [cost[p] for p in pure] + [sum(pa[a] * pq[b] * cost[(a, b)] for a in pa for b in pq) for pa, pq in mixed]
        passes = {outs[p].passed for p in pure}
        if any(outs[p].contact for p in pure):
            cls = "!"
        elif passes == {True}:
            cls = "P"
        elif passes == {False}:
            cls = "W"
        elif passes:
            chicken = (any(outs[p].passed and outs[p].b_gave for p in pure)
                       and any(not outs[p].passed and p[1] == "hold" for p in pure))
            cls = "C" if chicken else "X"
        else:
            cls = "M" if mixed else "?"
        pairs = [outs[k] for k in ua]
        rows.append({"subset": subset, "bias": bias, "g": g, "vb": vb, "tm": tm, "class": cls, "n_pure": len(pure), "n_mixed": len(mixed),
                     "C_opt": min(cost.values()), "C_worst_eq": max(eqc) if eqc else None, "C_best_eq": min(eqc) if eqc else None,
                     "eq_pass": sum(outs[p].passed for p in pure), "eq_contact": sum(outs[p].contact for p in pure),
                     "eq_abort": sum(outs[p].aborted for p in pure), "eq_b_gave": sum(outs[p].b_gave for p in pure),
                     "pairs_contact": sum(o.contact for o in pairs), "pairs_contact_B": sum(o.who == "B" for o in pairs),
                     "pairs": len(pairs), "pure": " ".join(f"{a}/{b}" for a, b in pure)})
    return rows


def run_map(args):
    from multiprocessing import Pool

    lags = X.car_lags() if args.car_lags else None
    gs = [float(x) for x in args.gs.split(",")]
    vbs = [float(x) for x in args.vbs.split(",")]
    lo, hi = (float(x) for x in args.tms.split(".."))
    tms = [round(lo + i * args.tm_step, 3) for i in range(int(round((hi - lo) / args.tm_step)) + 1)]
    jobs = [(g, vb, tm, lags, args.bias) for g in gs for vb in vbs for tm in tms]
    with Pool(args.jobs) as pool:
        rows = [r for rs in pool.map(context, jobs) for r in rs]
    os.makedirs(args.out, exist_ok=True)

    def fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return "inf" if math.isinf(x) else f"{x:.4g}"
        return x

    with open(os.path.join(args.out, "overtake_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(v) for k, v in r.items()})
    by = {(r["subset"], r["g"], r["vb"], r["tm"]): r for r in rows}
    lines = [f"# overtaking (kinematic): L at {V_L:g} m/s, A wants {V_DES:g}; " + ("the car's command lags" if lags else "an ideal car")
             + f"; A's error in B's speed {args.bias:+g}",
             "# class per tm (B's time to meet A at the start, s): P A passes in every equilibrium, W A stays behind, C chicken",
             "# with B (A passes because B gives way / A stays and B holds), X both outcomes otherwise, ! contact, M mixed only",
             "# subsets: all rules; no_abort (A without its abort rules); b_hold (B can only hold or speed up: no courtesy)",
             "tm:                    " + " ".join(f"{t:>4g}" for t in tms)]
    for g in gs:
        for vb in vbs:
            for sub in ("all", "no_abort", "b_hold"):
                lines.append(f"g {g:3g} vB {vb:3g} {sub:9}" + " ".join(f"{by[(sub, g, vb, t)]['class']:>4}" for t in tms))
    lines.append("")
    for sub in ("all", "no_abort", "b_hold"):
        rs = [r for r in rows if r["subset"] == sub]
        cls = [r["class"] for r in rs]
        lines.append(f"# {sub:9}: classes " + ", ".join(f"{c} {cls.count(c)}" for c in "PWCX!M?" if cls.count(c))
                     + f" of {len(rs)}; equilibria with contact {sum(r['eq_contact'] for r in rs)}, with an abort {sum(r['eq_abort'] for r in rs)}; "
                     f"rule pairs with contact {sum(r['pairs_contact'] for r in rs)} of {sum(r['pairs'] for r in rs)} "
                     f"(with B {sum(r['pairs_contact_B'] for r in rs)}); mean worst / best equilibrium cost "
                     f"{sum(r['C_worst_eq'] or 0 for r in rs) / len(rs):.3f} / {sum(r['C_best_eq'] or 0 for r in rs) / len(rs):.3f}")
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "overtake_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    return 0


def selftest():
    # B far away: a gap-accepting A passes cleanly
    o = simulate("g1.5", "hold", 15.0, 20.0, 20.0)
    assert o.passed and not o.contact, o
    # B close: blind go against a holding B is head-on contact
    o = simulate("go", "hold", 15.0, 20.0, 4.0)
    assert o.contact and o.who == "B", o
    # the same with a braking B and an aborting A: no contact
    o = simulate("g0.5a", "brake", 15.0, 20.0, 6.0)
    assert not o.contact, o
    # wait never passes
    o = simulate("wait", "hold", 15.0, 20.0, 6.0)
    assert not o.passed and not o.contact, o
    r = context((15.0, 20.0, 8.0, None, 0.0))
    assert r[0]["n_pure"] + r[0]["n_mixed"] > 0, r
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="overtake_out")
    p.add_argument("--gs", default="10,25")
    p.add_argument("--vbs", default="15,22")
    p.add_argument("--tms", default="4..16")
    p.add_argument("--tm-step", type=float, default=1.0)
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--bias", type=float, default=0.0, help="A's error in B's seen speed (-0.2: 20 %% too slow)")
    p.add_argument("--jobs", type=int, default=16)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
