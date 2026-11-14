#!/usr/bin/env python3
"""Roadmap 074 S3: the merge at a lane drop, a game in a kinematic model.

A drives in lane -2, which ends D_END m ahead of A's front at t = 0 (a taper; A's body must be in
lane -1 before it). B drives in lane -1. A must merge into B's lane before the end: the merge
instance of docs/design/cooperative-driving.md ② with a hard deadline, where the cut-in
(cutin_game) lets A wait forever.

Context (cutin_game's, plus the deadline):
  d_end  A's front to the lane's end at t = 0 (m)
  d0     A's rear minus B's front at t = 0 (m); negative: A's rear is beside or behind B's nose
  dv     A's speed minus B's (m/s)
  vb     B's speed (m/s)

Rules (contingent; everything A and B see of each other is OBS + DECIDE s old, predicted to now):
  A  gap acceptance: go into B's lane ahead of B iff A's rear is ahead of B's front by
       BUFFER + v_B tau + max(0, v_B^2 - v_A^2) / (2 A_REAR)       (design ②, tau in TAUS)
     or merge behind B once A's front is S0 + v_A HEADWAY behind B's rear.
     timing  early: look for a gap from the start; zipper: only within ZIP_T s of the end (at A's
             speed), so the lane is used to its end
     yield   go into B's lane, ahead of it, once B is seen braking (go-on-seen-yield)
     at the deadline (the last point a lane change of T_LC s still ends before the lane does):
             drop  fall behind B, braking for the lane's end; if that is not enough, stop before
                   the end (a forced stop) and merge from rest once B has passed
             force change lanes anyway: B's safety layer must make room
  B  hold      keep going (its safety layer still brakes for A once A is in its lane, ahead)
     react     keep going; once A's lane change is seen, continue with whichever of yield / hold
               is B's best reply (sequential rationality)
     c<band>   courtesy: with A beside or ahead within band m and A's lane ending within
               COURTESY_T s, slow at the rule's deceleration until A has a gap ahead of B

Payoffs: the general utility (measures.py ①): lost progress over exactly H, risk from
measures.push_poses (2D TTC, headway, clearance on oriented boxes), peak braking, contact; A's
front passing the lane's end while A is less than half across counts as contact (A brakes for the
end until it is half across, the taper takes the rest). Pure and 2 x 2 mixed equilibria per context
(crossing_game.solve). Classes per context:

  A  A ends ahead of B in every equilibrium     B  A ends behind B in every equilibrium
  C  both orders are equilibria (who goes first is a convention or a belief)
  S  an equilibrium has A stopped at the lane's end (a forced stop)
  !  an equilibrium has contact            M  mixed equilibria only            ?  none

and the welfare: C_opt, the worst equilibrium, and the worst equilibrium of the game in which A
is held to the zipper timing (the zipper convention) or to the early one.

Usage:
  scripts/coop/merge_game.py --out <dir> [--vb 12] [--d-ends 40,80,160] [--car-lags] [--jobs N]
  scripts/coop/merge_game.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from collections import deque
from dataclasses import dataclass
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import crossing_game as X  # noqa: E402
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402

DT = 0.05
HORIZON = M.HORIZON
T_MAX = 40.0
CAP = C1.TYPES["car"]
L, W = CAP.length, CAP.width
LANE_W = C1.LANE_W
T_LC = 3.0  # s, the lane change
OBS, DECIDE, REACT = X.OBS, X.DECIDE, CAP.reaction
BUFFER, A_REAR = 2.0, 2.0
S0, HEADWAY = C1.S0, C1.HEADWAY
ZIP_T = 6.0  # s before the end at A's speed: where a zipper merge starts looking
COURTESY_T = 10.0  # s: B's courtesy applies when A's lane ends within this
BRAKE_LATE = 2.5  # m/s^2: A brakes for the lane's end only once a stop needs this much
YIELD_SEEN = C1.YIELD_SEEN
END_MARGIN = 2.0  # m: A's lane change must end this far before the lane does
PEAK_W = 0.5

TAUS = (0.6, 1.2)
A_RULES = [(f"{tm[0]}{tau:g}{'y' if gy else ''}{em[0]}", tm, tau, gy, em)
           for tm in ("early", "zipper") for tau in TAUS for gy in (False, True) for em in ("drop", "force")]
B_RULES = [("hold", None, None), ("react", None, None), ("c20", 20.0, 2.0), ("c40", 40.0, 3.0)]


@dataclass
class Outcome:
    lost: tuple
    peak: tuple  # peak braking, the speed drop over PEAK_W (as the arena measures it)
    contact: bool
    ttc: float
    thw: float
    clear: float
    ahead: bool  # A ended in lane -1 ahead of B
    merged: bool  # A got into lane -1
    stopped: bool  # A came to rest before the lane's end
    forced: bool  # A changed lanes under its "force" option
    ran_out: bool  # A reached the lane's end still not across
    b_cont: str = ""
    t_merge: float = math.inf


def lane_change_dist(v):
    """Distance A covers during a lane change at speed v, plus the margin."""
    return max(v, 1.0) * T_LC + END_MARGIN


def simulate(ra, rb, d_end, d0, dv, vb, b_cont="hold", lags=None):
    _, timing, tau, go_y, end_mode = ra
    bname, band, b_y = rb
    va0 = max(0.0, vb + dv)
    xa, xb = d0 + L, 0.0  # fronts
    x_end = xa + d_end
    va, vbb = va0, vb
    act = {k: X.Actuator(lags) if lags else None for k in "AB"}
    hist = deque()  # (t, xa, va, aa, lat, xb, vb, ab)
    aa = ab = 0.0
    decide = None
    dropping = forced = stopped = False
    lat = 0.0
    lat_seen_at = None  # when B first saw A's lane change
    b_latched = False
    meas = M.Measures(b_max=(CAP.b_max, CAP.b_max))
    vh = {"A": deque(), "B": deque()}
    peak = {"A": 0.0, "B": 0.0}
    contact = ran_out = False
    t = 0.0
    t_merge = math.inf
    while t < T_MAX - 1e-9:
        hist.append((t, xa, va, aa, lat, xb, vbb, ab))
        while len(hist) > 2 and hist[1][0] <= t - (OBS + DECIDE + REACT + 0.5):
            hist.popleft()
        sd = next((h for h in reversed(hist) if h[0] <= t - OBS - DECIDE), hist[0])
        age = t - sd[0]
        xb_s, vb_s, ab_s = sd[5] + sd[6] * age, sd[6], sd[7]  # B as A sees it, predicted to now
        xa_s, va_s, lat_s = sd[1] + sd[2] * age, sd[2], sd[4]  # A as B sees it
        rem = x_end - xa  # A's front to the lane's end
        # --- A's decision
        if decide is None:
            gap_ahead = (xa - L) - xb_s
            gap_behind = (xb_s - L) - xa
            window = timing == "early" or rem <= ZIP_T * max(va, 1.0) + END_MARGIN
            need = BUFFER + vb_s * tau + max(0.0, vb_s ** 2 - va ** 2) / (2 * A_REAR)
            if gap_behind >= S0 + va * HEADWAY:
                decide = t  # behind B: merge into its wake
            elif window and not dropping:
                if gap_ahead >= need or (go_y and ab_s <= -YIELD_SEEN and gap_ahead > -L / 2):
                    decide = t
            if decide is None and rem <= lane_change_dist(va):
                if end_mode == "force" and not dropping:
                    decide, forced = t, True
                else:
                    dropping = True
            if decide is None and stopped and xb_s - L > xa:  # at rest at the end, B gone past
                decide = t
        # --- A's longitudinal
        if decide is not None and lat > 0.0 and xa < xb:
            aa = C1.follow_cruise((xb - L) - xa, va, vbb, CAP, va0)
        elif dropping and decide is None:
            gap_behind = (xb_s - L) - xa
            aa = -C1.A_WAIT_DECEL if va > 0.5 * vb_s or gap_behind < 0 else 0.0
        else:
            aa = max(-1.0, min(CAP.a_max, 1.5 * (va0 - va)))
        if (decide is None or lat < 0.5) and rem > 0:  # not (half) across yet: brake for the lane's end
            d = rem - (1.0 if decide is None else 0.0)
            need_e = va * va / (2 * max(d, 0.05))
            if d <= 0.05:
                aa = -CAP.b_max if va > 0 else 0.0
            elif need_e >= BRAKE_LATE:
                aa = min(aa, -min(CAP.b_max, need_e))
        # --- B
        if lat_s > 0.08 and lat_seen_at is None:
            lat_seen_at = t
        a_ahead = xa_s - L > xb - L  # A's rear is ahead of B's rear: A could take the slot ahead
        gap_b = (xa_s - L) - xb
        yielding = False
        if bname == "react" and lat_seen_at is not None and t >= lat_seen_at + REACT and a_ahead:
            yielding = b_cont == "yield"
        if bname.startswith("c"):
            if (not b_latched and t >= REACT and a_ahead and gap_b < band and lat_s < 0.5
                    and (x_end - xa_s) <= COURTESY_T * max(va_s, 1.0)):
                b_latched = True
            yielding = b_latched and a_ahead
        b_dec = b_y if bname.startswith("c") else C1.B_YIELD_DECEL
        if yielding:
            ab = -b_dec if gap_b < S0 + vbb * HEADWAY else C1.follow_cruise(gap_b, vbb, va_s, CAP, vb)
        else:
            ab = max(-1.0, min(CAP.a_max, 1.5 * (vb - vbb)))
        if lat_s >= 0.5 and xa_s > xb:  # the safety layer: A is in B's lane, ahead
            ab = min(ab, C1.follow_accel(gap_b, vbb, va_s, CAP, vb))
        ab = max(-CAP.b_max, ab)
        aa = max(-CAP.b_max, min(CAP.a_max, aa))
        if lags:
            aa, ab = act["A"].step(aa), act["B"].step(ab)
        va = max(0.0, va + aa * DT)
        vbb = max(0.0, vbb + ab * DT)
        xa += va * DT
        xb += vbb * DT
        t += DT
        if decide is not None:
            u = min(1.0, (t - decide) / T_LC)
            lat = 0.5 * (1 - math.cos(math.pi * u))
            vy = LANE_W * 0.5 * math.pi / T_LC * math.sin(math.pi * u) if u < 1 else 0.0
            if u >= 1.0 and math.isinf(t_merge):
                t_merge = t
        else:
            vy = 0.0
        if va < 0.1 and xa < x_end and decide is None:
            stopped = True
        if xa > x_end and lat < 0.5:  # past the end with its centre still in the ending lane
            ran_out = True
        for k, vv in (("A", va), ("B", vbb)):
            h = vh[k]
            h.append((t, vv))
            while t - h[0][0] > PEAK_W + 1e-9:
                h.popleft()
            if t - h[0][0] >= PEAK_W - 1e-9:
                peak[k] = max(peak[k], (h[0][1] - vv) / PEAK_W)
        ya = LANE_W * (1 - lat)
        vxa = math.sqrt(max(0.0, va * va - vy * vy))
        pa = (xa - L / 2, ya, math.atan2(-vy, max(vxa, 0.1)), vxa, -vy, L, W)
        pb = (xb - L / 2, 0.0, 0.0, vbb, 0.0, L, W)
        meas.push_poses(t, pa, pb)
        if meas.clear <= 0.0 or ran_out:
            contact = True
            break
        if t >= HORIZON - 1e-9 and lat >= 1.0:
            break
    merged = lat >= 1.0
    lost_a = 1.0 if contact or not merged else M.lost_progress(va0, meas.s_h[0] if meas.s_h[0] is not None else meas.s[0])
    lost_b = 1.0 if contact else M.lost_progress(vb, meas.s_h[1] if meas.s_h[1] is not None else meas.s[1])
    return Outcome((lost_a, lost_b), (peak["A"], peak["B"]), contact, meas.ttc2d, meas.thw, meas.clear,
                   merged and xa > xb, merged, stopped, forced, ran_out, b_cont if bname == "react" else "", t_merge)


def utility(o, who):
    risk = M.rho(o.ttc, o.thw, math.inf, o.clear)
    return M.utility(o.lost[who], risk, o.peak[who], CAP.b_max, 0.0, o.contact)


def outcomes(d_end, d0, dv, vb, lags=None):
    outs = {}
    for ra in A_RULES:
        for rb in B_RULES:
            if rb[0] == "react":
                for c in ("yield", "hold"):
                    outs[(ra[0], f"react:{c}")] = simulate(ra, rb, d_end, d0, dv, vb, c, lags)
            else:
                outs[(ra[0], rb[0])] = simulate(ra, rb, d_end, d0, dv, vb, lags=lags)
    return outs


def play(outs, a_acts=None):
    a_acts = a_acts or [r[0] for r in A_RULES]
    b_acts = [r[0] for r in B_RULES]
    res = {}
    for a in a_acts:
        for b in b_acts:
            if b == "react":
                res[(a, b)] = max((outs[(a, "react:yield")], outs[(a, "react:hold")]), key=lambda o: utility(o, 1))
            else:
                res[(a, b)] = outs[(a, b)]
    ua = {k: utility(o, 0) for k, o in res.items()}
    ub = {k: utility(o, 1) for k, o in res.items()}
    pure, mixed = X.solve(ua, ub, a_acts, b_acts)
    return res, ua, ub, pure, mixed


def label(res, pure, mixed):
    if any(res[p].contact for p in pure):
        return "!"
    if any(res[p].stopped for p in pure):
        return "S"
    orders = {res[p].ahead for p in pure}
    if len(orders) > 1:
        return "C"
    if orders:
        return "A" if orders.pop() else "B"
    return "M" if mixed else "?"


def welfare(res, ua, ub, pure, mixed):
    cost = {k: -(ua[k] + ub[k]) for k in res}
    eq = [cost[p] for p in pure] + [sum(pa[a] * pb[b] * cost[(a, b)] for a in pa for b in pb) for pa, pb in mixed]
    return min(cost.values()), (max(eq) if eq else None), (min(eq) if eq else None)


def context(job):
    d_end, d0, dv, vb, lags = job
    outs = outcomes(d_end, d0, dv, vb, lags)
    res, ua, ub, pure, mixed = play(outs)
    c_opt, c_worst, c_best = welfare(res, ua, ub, pure, mixed)
    row = {"d_end": d_end, "d0": d0, "dv": dv, "vb": vb, "class": label(res, pure, mixed),
           "n_pure": len(pure), "n_mixed": len(mixed), "pure": " ".join(f"{a}/{b}" for a, b in pure),
           "C_opt": c_opt, "C_worst_eq": c_worst, "C_best_eq": c_best,
           "eq_stopped": sum(res[p].stopped for p in pure), "eq_forced": sum(res[p].forced for p in pure),
           "contact_pairs": sum(o.contact for o in res.values()),
           # does the deadline make it chicken? both A's force and B's hold are each a best reply to
           # the other's yielding, and together they make contact
           "force_vs_hold_contact": int(any(res[(a[0], "hold")].contact for a in A_RULES if a[4] == "force"))}
    for conv in ("zipper", "early"):
        acts = [r[0] for r in A_RULES if r[1] == conv]
        r2, ua2, ub2, p2, m2 = play(outs, acts)
        _, cw, cb = welfare(r2, ua2, ub2, p2, m2)
        row[f"C_worst_{conv}"], row[f"class_{conv}"] = cw, label(r2, p2, m2)
    return row


def grid(lo, hi, step):
    n = int(round((hi - lo) / step))
    return [round(lo + i * step, 6) for i in range(n + 1)]


def run_map(args):
    lags = X.car_lags() if args.car_lags else None
    d_ends = [float(x) for x in args.d_ends.split(",")]
    d0s, dvs = grid(-10.0, 30.0, 2.5), grid(-3.0, 3.0, 1.0)
    jobs = [(de, d0, dv, args.vb, lags) for de in d_ends for dv in dvs for d0 in d0s]
    with Pool(args.jobs) as pool:
        rows = pool.map(context, jobs)
    os.makedirs(args.out, exist_ok=True)

    def fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return "inf" if math.isinf(x) else f"{x:.3f}"
        return x

    with open(os.path.join(args.out, "merge_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow({k: fmt(v) for k, v in r.items()})
    lab = {(r["d_end"], r["d0"], r["dv"]): r for r in rows}
    lines = [f"# lane-drop merge equilibrium map (kinematic): vB = {args.vb} m/s, {len(A_RULES)} A rules x "
             f"{len(B_RULES)} B rules, " + ("the car's command lags" if lags else "an ideal car"),
             f"# rows: dv (A minus B, m/s); columns: d0 (A's rear minus B's front, m) {d0s[0]:g}..{d0s[-1]:g} step 2.5",
             "# A / B: A ends ahead of / behind B in every equilibrium   C: both orders are equilibria",
             "# S: an equilibrium has A stopped at the lane's end   !: contact in an equilibrium   M: mixed only",
             "# second grid: welfare loss of the worst equilibrium C_worst - C_opt (. <0.05 - <0.2 + <0.5 # >=0.5),",
             "# third: what the zipper convention recovers, C_worst - C_worst(A zipper only) (. <0.05, + >=0.05,",
             "# - <= -0.05: the zipper's worst is worse)"]
    for de in d_ends:
        lines.append(f"\n## lane end {de:g} m ahead of A")
        lines.append("    dv  " + "".join(f"{d0:g}".rjust(5) for d0 in d0s))
        for dv in reversed(dvs):
            lines.append(f"  {dv:+4.1f}  " + "".join(lab[(de, d0, dv)]["class"].rjust(5) for d0 in d0s))
        lines.append("  welfare loss")
        for dv in reversed(dvs):
            s = ""
            for d0 in d0s:
                r = lab[(de, d0, dv)]
                x = None if r["C_worst_eq"] is None else r["C_worst_eq"] - r["C_opt"]
                s += ("?" if x is None else "." if x < 0.05 else "-" if x < 0.2 else "+" if x < 0.5 else "#").rjust(5)
            lines.append(f"  {dv:+4.1f}  " + s)
        lines.append("  zipper recovers")
        for dv in reversed(dvs):
            s = ""
            for d0 in d0s:
                r = lab[(de, d0, dv)]
                if r["C_worst_eq"] is None or r["C_worst_zipper"] is None:
                    s += "?".rjust(5)
                    continue
                g = r["C_worst_eq"] - r["C_worst_zipper"]
                s += ("+" if g >= 0.05 else "-" if g <= -0.05 else ".").rjust(5)
            lines.append(f"  {dv:+4.1f}  " + s)
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "merge_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    try:
        plot(args.out, d_ends, d0s, dvs, lab, args.vb)
    except ImportError:
        print("matplotlib missing: no PNG", file=sys.stderr)
    return 0


def plot(out, d_ends, d0s, dvs, lab, vb):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    order = "ABCS!M?"
    names = ["A ahead of B", "A behind B", "both orders", "A stopped at the end", "contact", "mixed only", "none"]
    colours = ["#2a9d8f", "#457b9d", "#e9c46a", "#f4a261", "#e63946", "#6a4c93", "#999999"]
    fig, axes = plt.subplots(1, len(d_ends), figsize=(5 * len(d_ends) + 2, 3.8), sharey=True)
    axes = axes if len(d_ends) > 1 else [axes]
    for ax, de in zip(axes, d_ends):
        z = [[order.index(lab[(de, d0, dv)]["class"]) for d0 in d0s] for dv in dvs]
        ax.imshow(z, origin="lower", aspect="auto", cmap=ListedColormap(colours), vmin=-0.5, vmax=len(order) - 0.5,
                  extent=(d0s[0] - 1.25, d0s[-1] + 1.25, dvs[0] - 0.5, dvs[-1] + 0.5))
        ax.set_title(f"lane end {de:g} m ahead of A")
        ax.set_xlabel("d0: A's rear ahead of B's front (m)")
    axes[0].set_ylabel("dv: A's speed minus B's (m/s)")
    present = sorted({r["class"] for r in lab.values()}, key=order.index)
    axes[-1].legend(handles=[Patch(color=colours[order.index(c)], label=names[order.index(c)]) for c in present],
                    loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    fig.suptitle(f"Lane-drop merge: equilibrium classes (kinematic), vB = {vb:g} m/s")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "merge_map.png"), dpi=110)


def selftest():
    rule = lambda n: next(r for r in A_RULES if r[0] == n)  # noqa: E731
    # A far ahead and faster, plenty of lane: an early merge ahead of B, no contact
    o = simulate(rule("e0.6d"), B_RULES[0], 160.0, 25.0, 2.0, 12.0)
    assert o.merged and o.ahead and not o.contact, o
    # A beside B with a short lane, dropping: no contact, A ends behind B
    o = simulate(rule("e1.2d"), B_RULES[0], 40.0, -4.0, 0.0, 12.0)
    assert not o.contact and (not o.merged or not o.ahead), o
    # forcing beside a holding B near the end is contact or a near miss
    o = simulate(rule("e1.2f"), B_RULES[0], 40.0, -2.0, 0.0, 12.0)
    assert o.contact or M.rho(o.ttc, o.thw, math.inf, o.clear) >= 0.5, o
    # a zipper merge waits for the end: it merges later than an early one in the same context
    e = simulate(rule("e0.6d"), B_RULES[0], 160.0, 25.0, 0.0, 12.0)
    z = simulate(rule("z0.6d"), B_RULES[0], 160.0, 25.0, 0.0, 12.0)
    assert z.t_merge > e.t_merge, (e.t_merge, z.t_merge)
    # courtesy opens a gap: A beside B gets ahead with c40 where it does not against hold
    o_h = simulate(rule("z1.2yd"), B_RULES[0], 80.0, -2.0, 0.0, 12.0)
    o_c = simulate(rule("z1.2yd"), B_RULES[3], 80.0, -2.0, 0.0, 12.0)
    assert o_c.ahead and not o_h.ahead, (o_c, o_h)
    r = context((80.0, 0.0, 0.0, 12.0, None))
    assert r["n_pure"] or r["n_mixed"], r
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="merge_out")
    p.add_argument("--vb", type=float, default=12.0)
    p.add_argument("--d-ends", default="40,80,160")
    p.add_argument("--car-lags", action="store_true")
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
