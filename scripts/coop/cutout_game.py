#!/usr/bin/env python3
"""Roadmap 074 S1: the cut-out as a game in a kinematic model.

Three cars in one lane: B follows L; ahead of L is an obstacle O, stopped or slow. L changes
lanes away (to the left lane, free) and so reveals O to B (docs/design/cooperative-driving.md ⓪:
diverge + follow). Context:

  h0     B's time headway to L at t = 0 (s)
  v      L's and B's speed (m/s); L's time to O at t = 0 is T0_LO s
  O      stopped, or slow (O_SLOW x v)
  look   B perceives O through L from the start (look-through on: radar, or a high viewpoint),
         or only once L's body has left B's lane (off), plus perception and reaction
  v_adj  the left lane's speed L settles to after its lane change: v (no cost to changing early)
         or ADJ_SLOW x v (a slower lane: L gains by staying in its lane longer)
  t_lc   L's lane-change duration (s), --tlc

Rules:
  L  the cut-out's timing: start the lane change when L's time to O reaches T_L in L_TIMINGS
     (early 6 s ... late 2.5 s); L keeps its speed, and its safety layer brakes for O, while its
     body is in the lane, only once stopping needs 3 m/s^2 or more
  B  the following rule: IDM toward what it perceives (L, then O; both with look-through),
     with time gap tau_B in B_GAPS; braking up to B_MAX_B

Payoffs: the general utility (measures.py, ①) over a fixed horizon: lost progress, risk as the
largest shortfall of 2D TTC, headway, PET and clearance against the cars in its conflict set (B: O
and L; L: O and B), peak braking and lateral acceleration, contact. The game (L's timing x B's gap)
is solved per context, pure and 2 x 2 mixed equilibria (crossing_game.solve). Reported per context:
B's risk and contact in the worst equilibrium, L's timing in it, what that timing costs B (B's
utility against its best reply to L's earliest timing, minus its utility in the equilibrium), and
the smallest gap that keeps B's risk under RISK_OK against every timing L may choose.

Usage:
  scripts/coop/cutout_game.py --out <dir> [--tlc 3] [--jobs N]
  scripts/coop/cutout_game.py --selftest
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
from crossing_game import solve  # noqa: E402

DT = 0.05
HORIZON = 20.0
LEN, WID = 4.79, 2.163  # m, the Model 3's box; O is a car too
LANE_W = 3.5
LAT_CLEAR = (LANE_W / 2 + WID / 2) / LANE_W  # L's lateral progress at which its body leaves B's lane
T0_LO = 8.0  # s, L's time to O at t = 0
O_SLOW = 0.5
ADJ_SLOW = 0.7
A_MAX, B_MAX = 2.0, 7.0  # m/s^2: drive; braking limit of L
B_MAX_B = 7.0  # m/s^2, B's braking limit
IDM_A, IDM_B, S0 = 2.0, 2.0, 2.0  # IDM's acceleration, comfortable braking (m/s^2), standstill gap (m)
OBS, REACT = 0.19, 1.0  # s: perception delay; B's reaction to a newly seen obstacle
L_TIMINGS = (6.0, 4.5, 3.5, 2.5)
B_GAPS = (0.8, 1.5, 2.5)
RISK_OK = 0.2


@dataclass
class Out:
    uL: float
    uB: float
    rhoB: float
    rhoL: float
    contact: bool
    peakB: float
    ttcBO: float
    clearBO: float
    lostB: float
    lostL: float


def idm(gap, v, v_lead, tau, v_des):
    """IDM, its braking held to the comfortable IDM_B unless the closing itself needs more: a gap
    shorter than the car's own time gap is opened gently, not with IDM's own hard braking."""
    s_star = S0 + max(0.0, v * tau + v * (v - v_lead) / (2 * math.sqrt(IDM_A * IDM_B)))
    a = IDM_A * (1 - (v / max(v_des, 0.1)) ** 4 - (s_star / max(gap, 0.1)) ** 2)
    need = max(0.0, v - v_lead) ** 2 / (2 * max(gap - S0, 0.1))
    return max(a, -max(IDM_B, 1.2 * need))


def simulate(t_l, tau_b, h0, v, o_mode, look, v_adj, t_lc):
    v_o = 0.0 if o_mode == "stopped" else O_SLOW * v
    xl = 0.0  # fronts
    xb = -(LEN + h0 * v)
    xo = xl + T0_LO * (v - v_o) + LEN  # O's front: its rear T0_LO s of closing ahead of L
    vl, vb = v, v
    t_start = None  # L's lane change
    lat = 0.0
    t_reveal = None  # L's body left B's lane
    pairs = {k: M.Measures(b_max=(B_MAX_B, B_MAX_B)) for k in ("BO", "BL", "LO")}
    peak = {"L": 0.0, "B": 0.0}
    contact = False
    t = 0.0
    while t < HORIZON - 1e-9:
        # --- L
        if t_start is None and vl > v_o and (xo - LEN - xl) / (vl - v_o) <= t_l:
            t_start = t
        in_lane = lat < LAT_CLEAR
        if lat >= 0.5:
            al = max(-IDM_B, min(A_MAX, (v_adj - vl) / 1.0))  # settle to the left lane's speed
        else:
            al = 0.0
        if in_lane and xo - LEN - xl > 0:  # L's safety layer: brakes only when a stop is needed hard
            need = max(0.0, vl - v_o) ** 2 / (2 * max(xo - LEN - xl - 1.0, 0.1))
            if need >= 3.0:
                al = min(al, -min(B_MAX, need))
        # --- B: what it perceives
        leaders = []
        if t_reveal is None or t < t_reveal + OBS:
            leaders.append((xl - LEN - xb, vl))
        if look == "on" and t >= OBS or (t_reveal is not None and t >= t_reveal + OBS + REACT):
            leaders.append((xo - LEN - xb, v_o))
        ab = min([idm(g, vb, vlead, tau_b, v) for g, vlead in leaders] + [A_MAX])
        ab = max(-B_MAX_B, ab)
        for k, a in (("L", al), ("B", ab)):
            peak[k] = max(peak[k], -a)
        vl = max(0.0, vl + al * DT)
        vb = max(0.0, vb + ab * DT)
        xl += vl * DT
        xb += vb * DT
        xo += v_o * DT
        t += DT
        if t_start is not None:
            u = min(1.0, (t - t_start) / t_lc)
            lat = 0.5 * (1 - math.cos(math.pi * u))
            vy = LANE_W * 0.5 * math.pi / t_lc * math.sin(math.pi * u) if u < 1 else 0.0
        else:
            vy = 0.0
        if lat >= LAT_CLEAR and t_reveal is None:
            t_reveal = t
        vxl = math.sqrt(max(0.0, vl * vl - vy * vy))
        pl = (xl - LEN / 2, LANE_W * lat, math.atan2(vy, max(vxl, 0.1)), vxl, vy, LEN, WID)
        pb = (xb - LEN / 2, 0.0, 0.0, vb, 0.0, LEN, WID)
        po = (xo - LEN / 2, 0.0, 0.0, v_o, 0.0, LEN, WID)
        pairs["BO"].push_poses(t, pb, po)
        pairs["BL"].push_poses(t, pb, pl)
        pairs["LO"].push_poses(t, pl, po)
        if any(m.clear <= 0.0 for m in pairs.values()):
            contact = True
            break
    r = {k: M.rho(m.ttc2d, m.thw, math.inf, m.clear) for k, m in pairs.items()}  # no crossing: no PET
    rho_b, rho_l = max(r["BO"], r["BL"]), max(r["LO"], r["BL"])
    lost_b = 1.0 if contact else min(1.0, max(0.0, (v * HORIZON - (xb + LEN + h0 * v)) / (v * HORIZON)))
    lost_l = 1.0 if contact else min(1.0, max(0.0, (v * HORIZON - xl) / (v * HORIZON)))
    lat_l = pairs["LO"].lat_peak[0]
    u_l = M.utility(lost_l, rho_l, peak["L"], B_MAX, lat_l, contact)
    u_b = M.utility(lost_b, rho_b, peak["B"], B_MAX_B, 0.0, contact)
    m = pairs["BO"]
    return Out(u_l, u_b, rho_b, rho_l, contact, peak["B"], m.ttc2d, m.clear, lost_b, lost_l)


def context(job):
    h0, v, o_mode, look, adj, t_lc = job
    v_adj = v if adj == "same" else ADJ_SLOW * v
    outs = {(tl, tb): simulate(tl, tb, h0, v, o_mode, look, v_adj, t_lc) for tl in L_TIMINGS for tb in B_GAPS}
    ja = {k: o.uL for k, o in outs.items()}
    jb = {k: o.uB for k, o in outs.items()}
    pure, mixed = solve(ja, jb, L_TIMINGS, B_GAPS)
    cost = {k: -(o.uL + o.uB) for k, o in outs.items()}
    opt = min(cost, key=lambda k: cost[k])
    # B's trouble: the smallest gap keeping B's risk under RISK_OK whatever L's timing
    safe_gap = next((tb for tb in B_GAPS if all(outs[(tl, tb)].rhoB < RISK_OK and not outs[(tl, tb)].contact
                                                  for tl in L_TIMINGS)), None)
    worst = max(pure, key=lambda p: cost[p]) if pure else None
    # does L's timing shift cost to B? B's utility in the worst equilibrium against its best reply
    # to L's earliest timing: what L's chosen lateness costs B (risk, or the gap B must keep)
    early = max((outs[(L_TIMINGS[0], tb)] for tb in B_GAPS), key=lambda o: o.uB)
    b_cost = None if worst is None else early.uB - outs[worst].uB
    shift = bool(worst and worst[0] < L_TIMINGS[0] and b_cost is not None and b_cost > 0.05)
    return {
        "h0": h0, "v": v, "O": o_mode, "look": look, "adj": adj, "t_lc": t_lc,
        "pure": " ".join(f"{a:g}/{b:g}" for a, b in pure), "n_mixed": len(mixed),
        "eq_L_timing": "" if worst is None else worst[0], "eq_B_gap": "" if worst is None else worst[1],
        "opt": f"{opt[0]:g}/{opt[1]:g}",
        "rhoB_eq": None if worst is None else outs[worst].rhoB,
        "contact_eq": None if worst is None else int(any(outs[p].contact for p in pure)),
        "C_opt": cost[opt], "C_worst_eq": None if worst is None else cost[worst],
        "safe_gap": safe_gap, "shift": int(shift), "B_cost_of_L_timing": b_cost,
        "contact_pairs": sum(o.contact for o in outs.values()),
        "rhoB_by_gap_vs_latest": " ".join(f"{outs[(L_TIMINGS[-1], tb)].rhoB:.2f}" for tb in B_GAPS),
        "contact_by_gap_vs_latest": " ".join(str(int(outs[(L_TIMINGS[-1], tb)].contact)) for tb in B_GAPS),
    }


def cell(row):
    if row["contact_eq"]:
        return "!"
    r = row["rhoB_eq"]
    if r is None:
        return "?"
    return "#" if r >= 0.5 else "+" if r >= RISK_OK else "."


def run_map(args):
    hs = (0.6, 0.9, 1.2, 1.5, 2.0, 2.5, 3.0)
    vs = (8.0, 12.0, 16.0, 20.0)
    panels = list(itertools.product(("stopped", "slow"), ("off", "on"), ("same", "slower")))
    jobs = [(h, v, o, lk, adj, args.tlc) for o, lk, adj in panels for v in vs for h in hs]
    with Pool(args.jobs) as pool:
        rows = pool.map(context, jobs)
    os.makedirs(args.out, exist_ok=True)

    def fmt(x):
        if x is None:
            return ""
        if isinstance(x, float):
            return "inf" if math.isinf(x) else f"{x:.3f}"
        return x

    with open(os.path.join(args.out, "cutout_map.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for row in rows:
            w.writerow({k: fmt(x) for k, x in row.items()})
    lab = {(r["O"], r["look"], r["adj"], r["v"], r["h0"]): r for r in rows}
    lines = [f"# cut-out equilibrium map (kinematic), L's lane change {args.tlc:g} s; L timings {L_TIMINGS} s "
             f"(time to O at its start), B gaps {B_GAPS} s",
             "# columns: h0, B's headway to L at t = 0 (s): " + " ".join(f"{h:g}" for h in hs),
             "# per cell: B's risk in the worst equilibrium (. <0.2  + <0.5  # >=0.5  ! contact), then L's timing "
             "in it (E 6, M 4.5, m 3.5, l 2.5 s) and B's gap (s: 0.8 / 1.5 / 2.5 as 1 2 3); * where L's timing "
             "shifts cost to B (L later than its earliest, and B > 0.05 worse off than against the earliest)",
             "# and the smallest B gap that keeps B's risk < 0.2 against every L timing (- : none)"]
    tl_c = {6.0: "E", 4.5: "M", 3.5: "m", 2.5: "l"}
    for o, lk, adj in panels:
        lines.append(f"\n## O {o}, look-through {lk}, left lane {adj}")
        lines.append("  v(m/s) " + "".join(f"{h:g}".rjust(6) for h in hs) + "   | safe gap")
        for v in reversed(vs):
            s, g = "", ""
            for h in hs:
                r = lab[(o, lk, adj, v, h)]
                code = cell(r) + ("" if r["eq_L_timing"] == "" else tl_c[r["eq_L_timing"]]) + \
                    ("" if r["eq_B_gap"] == "" else str(B_GAPS.index(r["eq_B_gap"]) + 1)) + ("*" if r["shift"] else "")
                s += code.rjust(6)
                g += ("-" if r["safe_gap"] is None else f"{r['safe_gap']:g}").rjust(4)
            lines.append(f"  {v:6.1f} " + s + "   |" + g)
    text = "\n".join(lines) + "\n"
    with open(os.path.join(args.out, "cutout_map.txt"), "w") as f:
        f.write(text)
    print(text, end="")
    try:
        plot(args, panels, hs, vs, lab)
    except ImportError:
        print("matplotlib missing: no PNG", file=sys.stderr)
    return 0


def plot(args, panels, hs, vs, lab):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch
    order = ".+#!?"
    names = ["B's risk < 0.2", "B's risk 0.2-0.5", "B's risk >= 0.5", "contact in equilibrium", "none"]
    colours = ["#2a9d8f", "#e9c46a", "#f4a261", "#e63946", "#999999"]
    fig, axes = plt.subplots(2, 4, figsize=(16, 6.5), sharex=True, sharey=True)
    for ax, (o, lk, adj) in zip(axes.flat, panels):
        z = [[order.index(cell(lab[(o, lk, adj, v, h)])) for h in hs] for v in vs]
        ax.imshow(z, origin="lower", aspect="auto", cmap=ListedColormap(colours), vmin=-0.5, vmax=len(order) - 0.5)
        for i, v in enumerate(vs):
            for j, h in enumerate(hs):
                r = lab[(o, lk, adj, v, h)]
                if r["shift"]:
                    ax.text(j, i, "*", ha="center", va="center", fontsize=12)
        ax.set_title(f"O {o}, look {lk}, left lane {adj}", fontsize=9)
        ax.set_xticks(range(len(hs)), [f"{h:g}" for h in hs])
        ax.set_yticks(range(len(vs)), [f"{v:g}" for v in vs])
    for ax in axes[-1]:
        ax.set_xlabel("h0: B's headway to L (s)")
    for ax in axes[:, 0]:
        ax.set_ylabel("v (m/s)")
    axes[0, -1].legend(handles=[Patch(color=c, label=n) for c, n in zip(colours, names)] +
                       [Patch(color="white", label="* L's lateness costs B > 0.05")],
                       loc="upper left", bbox_to_anchor=(1.01, 1), fontsize=8)
    fig.suptitle(f"Cut-out: B's risk in the worst equilibrium (kinematic, L's lane change {args.tlc:g} s)")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "cutout_map.png"), dpi=110)


def selftest():
    # an early cut-out with a long gap leaves B plenty of room
    o = simulate(6.0, 2.5, 3.0, 12.0, "stopped", "off", 12.0, 3.0)
    assert not o.contact and o.rhoB < 0.5, o
    # a late cut-out into a short gap, without look-through, is contact or a near miss for B
    o = simulate(2.5, 0.8, 0.6, 20.0, "stopped", "off", 20.0, 3.0)
    assert o.contact or o.rhoB >= 0.5, o
    # look-through never makes B's outcome worse
    for tl, tb, h in ((2.5, 0.8, 0.9), (3.5, 1.5, 1.2), (4.5, 0.8, 0.6)):
        off = simulate(tl, tb, h, 16.0, "stopped", "off", 16.0, 3.0)
        on = simulate(tl, tb, h, 16.0, "stopped", "on", 16.0, 3.0)
        assert (on.contact <= off.contact) and on.uB >= off.uB - 0.05, (tl, tb, h, on, off)
    # with a slower left lane, L gains progress by staying longer (its lost progress falls with lateness)
    early = simulate(6.0, 1.5, 2.0, 16.0, "slow", "on", 0.7 * 16.0, 3.0)
    late = simulate(3.5, 1.5, 2.0, 16.0, "slow", "on", 0.7 * 16.0, 3.0)
    assert late.lostL <= early.lostL + 1e-9, (early, late)
    r = context((1.2, 12.0, "stopped", "off", "same", 3.0))
    assert r["pure"] or r["n_mixed"], r
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", default="cutout_out")
    p.add_argument("--tlc", type=float, default=3.0, help="L's lane-change duration (s)")
    p.add_argument("--jobs", type=int, default=8)
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    return selftest() if a.selftest else run_map(a)


if __name__ == "__main__":
    sys.exit(main())
