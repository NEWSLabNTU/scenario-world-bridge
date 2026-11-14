#!/usr/bin/env python3
"""Roadmap 074 S1 in the arena: the cut-out on CARLA's physics.

arena.py's parallel two-lane roads, one group of three cars per road: O (the obstacle, stopped or
slow) and ahead of it nothing; L ahead of B in lane -1; lane -2 free. L changes lanes away from O
and so reveals it to B. The cars are arena.py's (the spec's physics, acb's controls). The rules
are cutout_game's, on CARLA:

  L  start the lane change when L's time to O reaches T_L (cutout_game.L_TIMINGS); a cosine path
     into lane -2 over t_lc; cutout_game.l_accel (the left lane's speed once half across, a brake
     for O only when a stop needs 3 m/s^2 or more). L sees O as it is (its own sensors).
  B  cutout_game.b_accel (cutout_game.follow: IDM's interaction term with B's time gap, capped by
     a cruise toward its own speed, as cutin_game.follow_cruise) toward what it perceives through
     arena.Observer (10 Hz, delayed, noisy): L until it has left B's lane, then O after the
     perception delay and its reaction (look-through off), or O from the start (on).

A warm-up brings L and B to speed v with B's headway h0 behind L, and puts O (T0_LO s of closing)
ahead of L, on reference trajectories. The run lasts HORIZON; it is scored as cutout_game scores
it (risk from measures.push_poses on the true poses per pair, lost progress over exactly HORIZON,
peak braking from the speed drop over 0.5 s, L's lateral acceleration, contact).

Then per context the game (L's timing x B's gap) is solved on the arena's mean payoffs and on
cutout_game's, scored the same way, with the car's command lags (crossing_game.car_lags), and
compared on D2's yardstick: payoffs inside the arena's 95 % interval (mean +- 2 sd over seeds, at
least +- 0.02), the outcomes (contact, B's risk band) the same, and the worst equilibrium's cell
(B's risk band, L's timing, B's gap).

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/cutout_arena.py --port 2600 --out <dir> [--k 8] [--seeds 5] [--v 12] [--look off]
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
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_game as X  # noqa: E402
import cutout_game as K  # noqa: E402
import measures as M  # noqa: E402

WARMUP = 15.0
A_REF = 2.0
X0 = 40.0  # m, B's front at the warm-up's start


def ramp(t, v):
    t1 = v / A_REF if v > 0 else 0.0
    if t < t1:
        return 0.5 * A_REF * t * t, A_REF * t, A_REF
    return 0.5 * A_REF * t1 * t1 + v * (t - t1), v, 0.0


class Group:
    """One cut-out on one road: cars O, L, B; x along the road (+x), fronts."""

    def __init__(self, ar, road, cars, job, rng, args):
        self.ar, self.road, self.job, self.args = ar, road, job, args
        self.O, self.L, self.B = cars
        self.y1, _ = ar.lane_y(road, -1)
        self.y2, _ = ar.lane_y(road, -2)
        v, h0 = job["v"], job["h0"]
        self.v_o = 0.0 if job["O"] == "stopped" else K.O_SLOW * v
        self.v_adj = v if job["adj"] == "same" else K.ADJ_SLOW * v
        d_ramp = ramp(WARMUP, v)[0]
        # fronts at the run's start: B; L h0 ahead of it; O T0_LO s of closing ahead of L
        self.xb_end = X0 + d_ramp
        self.xl_end = self.xb_end + K.LEN + h0 * v
        self.xo_end = self.xl_end + K.T0_LO * (v - self.v_o) + K.LEN
        self.xo_start = self.xo_end - ramp(WARMUP, self.v_o)[0]
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.obs = {"L": mk(), "O": mk()}  # what B sees of each
        self.phase = "warmup"
        self.rec = None

    def front(self, car):
        return car.state()[0] + car.length / 2

    def place(self):
        for car, x_front, y in ((self.O, self.xo_start, self.y1), (self.B, X0, self.y1),
                                (self.L, X0 + (self.xl_end - self.xb_end), self.y1)):
            loc = car.a.get_transform().location
            car.park(carla.Transform(carla.Location(x=x_front - car.length / 2, y=y, z=loc.z + 0.3), carla.Rotation(yaw=0)))

    def start_warmup(self):
        self.tw = self.ar.t

    def step_warmup(self):
        t = self.ar.t - self.tw
        v = self.job["v"]
        for car, x_end, vv in ((self.B, self.xb_end, v), (self.L, self.xl_end, v), (self.O, self.xo_end, self.v_o)):
            d, vr, ar_ = ramp(t, vv)
            x_ref = x_end - (ramp(WARMUP, vv)[0] - d)
            _, _, _, vc = car.state()
            a = max(-4.0, min(3.0, ar_ + 0.6 * (x_ref - self.front(car)) + 1.8 * (vr - vc)))
            if vv == 0.0:
                car.drive(-3.0, self.y1)  # a stopped obstacle holds its brake
            else:
                car.drive(a, self.y1)
        if t >= WARMUP - 1e-9:
            self.start()

    def start(self):
        self.phase = "run"
        self.t0 = self.ar.t
        self.t_start = self.t_reveal = None
        self.lat = 0.0
        b = (K.B_MAX_B, K.B_MAX_B)
        self.pairs = {k: M.Measures(b_max=b) for k in ("BO", "BL", "LO")}
        self.vh = {"L": deque(), "B": deque()}
        self.peak = {"L": 0.0, "B": 0.0}
        self.contact = False
        self.h0_real = (self.front(self.L) - self.L.length - self.front(self.B)) / max(self.B.state()[3], 0.1)
        self.v_real = self.B.state()[3]
        self.tlo_real = (self.front(self.O) - self.O.length - self.front(self.L)) / max(self.L.state()[3] - self.O.state()[3], 0.1)

    def step_run(self):
        t = self.ar.t - self.t0
        job, v = self.job, self.job["v"]
        xo, xl, xb = self.front(self.O), self.front(self.L), self.front(self.B)
        _, yl, _, vl = self.L.state()
        _, _, _, vb = self.B.state()
        _, _, _, vo = self.O.state()
        self.lat = max(0.0, min(1.0, (yl - self.y1) / (self.y2 - self.y1)))
        # --- L (sees O as it is)
        if self.t_start is None and vl > vo and (xo - self.O.length - xl) / (vl - vo) <= job["t_l"]:
            self.t_start = t
        al = K.l_accel(self.lat, vl, xl, xo, vo, self.v_adj)
        # pure pursuit aims at the path's point one look-ahead distance ahead (arena.Car.drive's), which
        # the cosine path reaches look / v later: steer for the lateral offset of then, not of now
        lead = max(6.0, 0.7 * vl) / max(vl, 1.0)
        lat_ref, _ = K.lateral(t + lead, self.t_start, job["t_lc"])
        y_l = self.y1 + (self.y2 - self.y1) * lat_ref
        if self.lat >= K.LAT_CLEAR and self.t_reveal is None:
            self.t_reveal = t
        # --- B (perceives L and O through its observers)
        seen = {}
        for k, car, x in (("L", self.L, xl), ("O", self.O, xo)):
            s = self.obs[k].push(t, x, 0.0, car.state()[3])
            seen[k] = (s[1], s[3]) if s is not None else (x, car.state()[3])
        see_l, see_o = K.b_leaders(t, job["look"], self.t_reveal)
        leaders = ([(seen["L"][0] - self.L.length - xb, seen["L"][1])] if see_l else []) + \
                  ([(seen["O"][0] - self.O.length - xb, seen["O"][1])] if see_o else [])
        ab = K.b_accel(leaders, vb, job["t_b"], v)
        self.L.drive(al, y_l)
        self.B.drive(ab, self.y1)
        if self.v_o == 0.0:
            self.O.drive(-3.0, self.y1)
        else:
            self.O.drive(max(-1.0, min(2.0, 1.5 * (self.v_o - vo))), self.y1)
        # --- truth
        for k, vv in (("L", vl), ("B", vb)):
            h = self.vh[k]
            h.append((t, vv))
            while t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                self.peak[k] = max(self.peak[k], (h[0][1] - vv) / AR.PEAK_WINDOW)
        pb, pl, po = self.B.pose(), self.L.pose(), self.O.pose()
        self.pairs["BO"].push_poses(t, pb, po)
        self.pairs["BL"].push_poses(t, pb, pl)
        self.pairs["LO"].push_poses(t, pl, po)
        if any(m.clear <= 0.0 for m in self.pairs.values()):
            self.contact = True
        if self.contact or t >= K.HORIZON - 1e-9:
            self.finish()

    def finish(self):
        P = self.pairs
        r = {k: M.rho(m.ttc2d, m.thw, math.inf, m.clear) for k, m in P.items()}
        rho_b, rho_l = max(r["BO"], r["BL"]), max(r["LO"], r["BL"])
        v = self.job["v"]
        sb = P["BO"].s_h[0] if P["BO"].s_h[0] is not None else P["BO"].s[0]
        sl = P["LO"].s_h[0] if P["LO"].s_h[0] is not None else P["LO"].s[0]
        lost_b = 1.0 if self.contact else M.lost_progress(v, sb)
        lost_l = 1.0 if self.contact else M.lost_progress(v, sl)
        u_l = M.utility(lost_l, rho_l, self.peak["L"], K.B_MAX, P["LO"].lat_peak[0], self.contact)
        u_b = M.utility(lost_b, rho_b, self.peak["B"], K.B_MAX_B, 0.0, self.contact)
        m = P["BO"]
        self.rec = {**self.job, "uL": round(u_l, 4), "uB": round(u_b, 4), "rhoB": round(rho_b, 4), "rhoL": round(rho_l, 4),
                    "contact": int(self.contact), "lostB": round(lost_b, 4), "lostL": round(lost_l, 4),
                    "peakB": round(self.peak["B"], 2), "peakL": round(self.peak["L"], 2),
                    "ttcBO": round(m.ttc2d, 3) if math.isfinite(m.ttc2d) else "inf", "clearBO": round(m.clear, 3),
                    "latL": round(P["LO"].lat_peak[0], 2), "h0_real": round(self.h0_real, 3), "v_real": round(self.v_real, 2),
                    "tlo_real": round(self.tlo_real, 3), "t_reveal": None if self.t_reveal is None else round(self.t_reveal, 2)}
        self.phase = "done"


def jobs(args):
    out = []
    for o, h0 in itertools.product(args.O.split(","), [float(x) for x in args.h0.split(",")]):
        for tl, tb in itertools.product(K.L_TIMINGS, K.B_GAPS):
            for s in range(args.seeds):
                out.append({"v": args.v, "h0": h0, "O": o, "look": args.look, "adj": args.adj, "t_lc": args.tlc,
                            "t_l": tl, "t_b": tb, "seed": s})
    return out


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k)
    rows = []
    t_start = time.monotonic()
    try:
        actors = []
        for r in range(args.k):
            for x in (500.0, 60.0, 30.0):  # O, L, B
                actors.append(ar.spawn(r, -1, x))
        ar.put_physics(actors)
        cars = [AR.Car(ar, a) for a in actors]
        while work:
            groups = []
            for r in range(args.k):
                if not work:
                    break
                g = Group(ar, r, cars[3 * r:3 * r + 3], work.popleft(), rng, args)
                g.place()
                groups.append(g)
            for _ in range(3):
                for g in groups:
                    for c in (g.O, g.L, g.B):
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
                        for c in (g.O, g.L, g.B):
                            c.drive(-3.0, c.state()[1])
                ar.tick()
            rows += [g.rec for g in groups]
            print(f"  {len(rows)}/{n_all} runs, {time.monotonic() - t_start:.0f} s", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    return rows, time.monotonic() - t_start


def band(r):
    return "!" if r >= 10 else "#" if r >= 0.5 else "+" if r >= K.RISK_OK else "."


def compare(args, rows):
    lags = X.car_lags()
    by = {}
    for r in rows:
        by.setdefault((r["O"], float(r["h0"]), float(r["t_l"]), float(r["t_b"])), []).append(r)
    lines, inside, total, agree, n_out = [], 0, 0, 0, 0
    summary: dict = {"cells": []}
    for o, h0 in itertools.product(args.O.split(","), [float(x) for x in args.h0.split(",")]):
        v_adj = args.v if args.adj == "same" else K.ADJ_SLOW * args.v
        ja_a, jb_a, ja_k, jb_k, kin, arn = {}, {}, {}, {}, {}, {}
        for tl, tb in itertools.product(K.L_TIMINGS, K.B_GAPS):
            rs = by[(o, h0, tl, tb)]
            ok = K.simulate(tl, tb, h0, args.v, o, args.look, v_adj, args.tlc, lags=lags, peak_mode="window")
            kin[(tl, tb)], arn[(tl, tb)] = ok, rs
            ja_a[(tl, tb)] = st.mean(float(r["uL"]) for r in rs)
            jb_a[(tl, tb)] = st.mean(float(r["uB"]) for r in rs)
            ja_k[(tl, tb)], jb_k[(tl, tb)] = ok.uL, ok.uB
            ok_all = True
            for key, kv in (("uL", ok.uL), ("uB", ok.uB)):
                xs = [float(r[key]) for r in rs]
                mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                hit = abs(kv - mu) <= max(2 * sd, 0.02)
                inside += hit
                total += 1
                ok_all &= hit
            a_contact = round(st.mean(int(r["contact"]) for r in rs))
            a_band = band(10 if a_contact else st.mean(float(r["rhoB"]) for r in rs))
            k_band = band(10 if ok.contact else ok.rhoB)
            same = a_contact == int(ok.contact) and a_band == k_band
            agree += same
            n_out += 1
            lines.append(f"O {o:7s} h0 {h0:3.1f}  L {tl:g}/B {tb:g} | arena uL {ja_a[(tl, tb)]:+.3f} uB {jb_a[(tl, tb)]:+.3f} "
                         f"rhoB {st.mean(float(r['rhoB']) for r in rs):.2f} contact {a_contact} | kin uL {ok.uL:+.3f} uB {ok.uB:+.3f} "
                         f"rhoB {ok.rhoB:.2f} contact {int(ok.contact)} | {'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cells = {}
        for name, ja, jb, rb in (("arena", ja_a, jb_a, lambda p: 10 if round(st.mean(int(r["contact"]) for r in arn[p])) else st.mean(float(r["rhoB"]) for r in arn[p])),
                                 ("kinematic", ja_k, jb_k, lambda p: 10 if kin[p].contact else kin[p].rhoB)):
            pure, mixed = X.solve(ja, jb, K.L_TIMINGS, K.B_GAPS)
            if pure:
                worst = max(pure, key=lambda p: -(ja[p] + jb[p]))
                cells[name] = band(rb(worst)) + f" L{worst[0]:g}/B{worst[1]:g}"
            else:
                cells[name] = "M" if mixed else "?"
        summary["cells"].append({"O": o, "h0": h0, "arena": cells["arena"], "kinematic": cells["kinematic"]})
        lines.append(f"## O {o}, h0 {h0:g}: worst equilibrium arena {cells['arena']} | kinematic {cells['kinematic']}")
    summary.update({"payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
                    "cells_agree": sum(c["arena"] == c["kinematic"] for c in summary["cells"]),
                    "risk_band_agree": sum(c["arena"][0] == c["kinematic"][0] for c in summary["cells"])})
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); "
                 f"outcomes agree in {agree} of {n_out}; worst-equilibrium cells agree in {summary['cells_agree']} of "
                 f"{len(summary['cells'])} (B's risk band in {summary['risk_band_agree']})")
    return lines, summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", type=float, default=12.0)
    p.add_argument("--h0", default="0.9,1.5,3.0")
    p.add_argument("--O", default="stopped,slow")
    p.add_argument("--look", default="off", choices=("off", "on"))
    p.add_argument("--adj", default="same", choices=("same", "slower"))
    p.add_argument("--tlc", type=float, default=3.0)
    p.add_argument("--obs-delay", type=float, default=K.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--compare-only", action="store_true")
    args = p.parse_args()
    if args.compare_only:
        rows = list(csv.DictReader(open(os.path.join(args.out, "runs.csv"))))
        wall = None
    else:
        rows, wall = run(args)
    lines, summary = compare(args, rows)
    summary["wall_s"] = None if wall is None else round(wall, 1)
    summary["runs"] = len(rows)
    with open(os.path.join(args.out, "compare.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    print("\n".join(lines[-(len(summary["cells"]) + 1):]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
