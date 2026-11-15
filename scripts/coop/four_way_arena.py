#!/usr/bin/env python3
"""Roadmap 074 U3 in the arena: the four-way stop on CARLA's physics.

crossing_arena.py's arena (arena.xodr_crossings: K crossings at grade, two one-lane roads each, the
spec's physics and acb's controls, arena.Observer's perception), with four_way_game.FWPolicy --
the stop-sign rules and crossing_game's safety layer, the object the kinematic model steps -- in
place of S2's policies. The warm-up puts each car's front v * (T_MEAN +- dt / 2) before its stop
line when the run starts, as four_way_game.simulate does. A run is scored as there (with the
arena's peak braking), plus a deadlock (both cars still before the box at the end) and each
car's restarts.

Per dt the game on the rule subset (SUB x SUB) is solved on the arena's mean payoffs and on the
kinematic model's (with the car's command lags), and compared on D2's yardstick: each kinematic
payoff inside the arena's mean +- max(2 sd, 0.02), the outcomes (first, contact, deadlock) by
majority, and the class.

074 C8 adds (every default as U3 ran it): --sub (the rule subset; C8: SUB plus fcfsm_c1.5, fcfse_c1.5),
--pairs bar (only the pairs with fcfsm, fcfse or stock yield in a place, for the BAR table; no game),
--angle (road B's heading, deg: a skewed junction, Z's length along each path W_BOX (1 + |cos|) / sin),
--heavy A|B with --heavy-bp and --e4 (that car a stock-physics blueprint, driven through E4's pedal maps
for it, as c5_arena), --fm-m / --fm-bias (fcfsm's margin and the rest stamp's lag correction), and per run
each car's own rest time and its reading of the other's (restA, readA, ...: the stamp error e, measured).
The C8 table (c8_table.txt): per pair and dt, contacts, deadlocks and mean payoffs.

Usage (a CARLA of its own; this script makes it synchronous and puts it back):
  scripts/coop/four_way_arena.py --port 2600 --out <dir> [--k 8] [--seeds 3] [--v 6] [--dts -6,-3,...]
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import time
from collections import deque
import statistics as st
import sys
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_arena as XA  # noqa: E402
import crossing_game as G  # noqa: E402
import four_way_game as F  # noqa: E402

SUB = ("aggr", "yield_c1.5", "stop1y_c1.5", "fcfs_c1.5", "fcfsr_c1.5", "rbl_c1.5")
C8_SUB = SUB + ("fcfsm_c1.5", "fcfse_c1.5")
BAR_RULES = ("fcfsm_c1.5", "fcfse_c1.5", "yield_c1.5")  # --pairs bar: one of these in a place


def zone_len(angle_deg):
    """Z's length along each path at a crossing of this angle (W_BOX at 90 deg)."""
    th = math.radians(angle_deg)
    return G.W_BOX * (1 + abs(math.cos(th))) / math.sin(th)


class SkewCrossing(XA.Crossing):
    """crossing_arena.Crossing with road B at any angle: the box's centre where the lane centres meet,
    Z the box's length along each path."""

    def __init__(self, ar, i, angle_deg):
        self.ar, self.i = ar, i
        th = math.radians(angle_deg)
        cx, cy = i * XA.SPACING, i * XA.SPACING  # CARLA: OpenDRIVE y flipped
        m = ar.map
        wa = m.get_waypoint(carla.Location(x=cx - 50.0, y=cy, z=0.0), project_to_road=True)
        wb = m.get_waypoint(carla.Location(x=cx - 50.0 * math.cos(th), y=cy + 50.0 * math.sin(th), z=0.0),
                            project_to_road=True)
        self.dir, self.line = {}, {}
        for k, w in (("A", wa), ("B", wb)):
            yaw = math.radians(w.transform.rotation.yaw)
            self.dir[k] = (math.cos(yaw), math.sin(yaw))
            self.line[k] = (w.transform.location.x, w.transform.location.y)
            self.z = w.transform.location.z
        (ax, ay), (bx, by) = self.line["A"], self.line["B"]
        (dax, day), (dbx, dby) = self.dir["A"], self.dir["B"]
        den = dax * dby - day * dbx
        s_a = ((bx - ax) * dby - (by - ay) * dbx) / den
        self.centre = (ax + dax * s_a, ay + day * s_a)
        self.wz = zone_len(angle_deg)

    def u(self, k, car):
        x, y, _, _ = car.state()
        d = self.dir[k]
        s = (x - self.centre[0]) * d[0] + (y - self.centre[1]) * d[1]
        return s + car.length / 2 + self.wz / 2

    def place_at(self, k, car, u):
        d = self.dir[k]
        s = u - car.length / 2 - self.wz / 2
        x, y = self.centre[0] + d[0] * s, self.centre[1] + d[1] * s
        yaw = math.degrees(math.atan2(d[1], d[0]))
        car.park(carla.Transform(carla.Location(x=x, y=y, z=self.z + 0.3), carla.Rotation(yaw=yaw)))


class FWRun(XA.Run):
    def __init__(self, xing, a_car, b_car, job, rng, args):
        self.x, self.A, self.B, self.job, self.args = xing, a_car, b_car, job, args
        v, dt = job["v"], job["dt"]
        wz = getattr(xing, "wz", G.W_BOX)
        ex = {"A": wz + a_car.length, "B": wz + b_car.length}
        # fronts v * (T_MEAN +- dt / 2) before the stop line (four_way_game.simulate)
        self.target_u = {"A": G.STOP - v * (G.T_MEAN + dt / 2), "B": G.STOP - v * (G.T_MEAN - dt / 2)}
        d_ramp, _, _ = XA.ramp(XA.WARMUP, v)
        self.u_spawn = {k: self.target_u[k] - d_ramp for k in "AB"}
        rule = {r[0]: r for r in F.RULES}
        # U3's run: the defaults (EXIT); C8's skew or heavy car: each car's own Z and the other's
        same = abs(wz - G.W_BOX) < 1e-9 and abs(a_car.length - G.L) < 0.05 and abs(b_car.length - G.L) < 0.05
        kw = {"A": {}, "B": {}} if same else {"A": {"exit_": ex["A"], "other_exit": ex["B"]},
                                              "B": {"exit_": ex["B"], "other_exit": ex["A"]}}
        self.pol = {"A": F.make_policy("A", rule[job["a"]], v, hold_stop=G.ARENA_HOLD_STOP, **kw["A"]),
                    "B": F.make_policy("B", rule[job["b"]], v, hold_stop=G.ARENA_HOLD_STOP, **kw["B"])}
        mk = lambda: AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)  # noqa: E731
        self.obs = {"A": mk(), "B": mk()}
        self.last_stamp = {"A": None, "B": None}
        self.phase = "warmup"
        self.rec = None

    def finish(self, t, st_):
        super().finish(t, st_)
        self.rec["deadlock"] = int(not self.contact and st_["A"][0] < 0 and st_["B"][0] < 0)
        r = lambda x: "" if x is None else round(x, 3)  # noqa: E731
        for k in "AB":  # 074 C8: each car's own rest and its reading of the other's (the stamp error e)
            self.rec[f"rest{k}"] = r(self.pol[k].rest_t)
            self.rec[f"read{k}"] = r(self.pol[k].other_rest_t)
            self.rec[f"reading{k}"] = self.pol[k].reading


def pairs(args):
    sub = args.sub.split(",")
    if args.pairs == "bar":
        return [(a, b) for a in sub for b in sub if a in BAR_RULES or b in BAR_RULES]
    return [(a, b) for a in sub for b in sub]


def jobs(args):
    return [{"dt": dt, "v": args.v, "a": a, "b": b, "seed": s}
            for dt in [float(x) for x in args.dts.split(",")] for a, b in pairs(args) for s in range(args.seeds)]


def run(args):
    """crossing_arena.run with C8's options: a skewed junction, one car a heavy blueprint."""
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    XA.WARMUP = args.warmup
    ar = AR.Arena(args.port, args.k, AR.xodr_crossings(args.k, XA.APPROACH, XA.BEYOND, XA.SPACING,
                                                       angle=math.radians(args.angle)))
    rows = []
    t_start = time.monotonic()
    try:
        xs = [SkewCrossing(ar, i, args.angle) if args.angle != 90.0 else XA.Crossing(ar, i) for i in range(args.k)]
        lib = ar.world.get_blueprint_library()
        actors, m3 = [], []
        for x in xs:
            for k in "AB":
                d = x.dir[k]
                s = -100.0
                loc = carla.Location(x=x.centre[0] + d[0] * s, y=x.centre[1] + d[1] * s, z=x.z + 0.6)
                tf = carla.Transform(loc, carla.Rotation(yaw=math.degrees(math.atan2(d[1], d[0]))))
                heavy = args.heavy == k
                a = ar.world.spawn_actor(lib.find(args.heavy_bp) if heavy else ar.bp, tf)
                ar.actors.append(a)
                actors.append((a, heavy))
                if not heavy:
                    m3.append(a)
        ar.put_physics(m3)
        if args.heavy:
            import c5_arena
            StockCar, _ = c5_arena.make_classes(args.e4)
            c5_arena.BP["heavy"] = args.heavy_bp

            class HeavyXCar(StockCar):
                drive_line = XA.XCar.drive_line
        cars = [HeavyXCar(ar, a, "heavy") if h else XA.XCar(ar, a) for a, h in actors]
        while work:
            runs = []
            for i, x in enumerate(xs):
                if not work:
                    break
                r = FWRun(x, cars[2 * i], cars[2 * i + 1], work.popleft(), rng, args)
                r.place()
                runs.append(r)
            for _ in range(3):
                for r in runs:
                    for c in (r.A, r.B):
                        c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            for r in runs:
                r.start_warmup()
            while any(r.phase != "done" for r in runs):
                for r in runs:
                    if r.phase == "warmup":
                        r.step_warmup()
                    elif r.phase == "run":
                        r.step_run()
                    else:
                        for k, c in (("A", r.A), ("B", r.B)):
                            c.drive_line(-3.0, r.x.line[k], r.x.dir[k])
                ar.tick()
            rows += [r.rec for r in runs]
            print(f"  {len(rows)}/{n_all} runs, {time.monotonic() - t_start:.0f} s", flush=True)
    finally:
        ar.close()
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        keys = list(dict.fromkeys(k for r in rows for k in r))
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    return rows, time.monotonic() - t_start


def c8_table(rows):
    """Per pair (and per dt): runs, contacts, deadlocks, mean payoffs; the readings' stamp error."""
    by, by_dt = {}, {}
    for r in rows:
        by.setdefault((r["a"], r["b"]), []).append(r)
        by_dt.setdefault((r["a"], r["b"], float(r["dt"])), []).append(r)
    lines = ["# per pair over every dt and seed: runs, contacts, deadlocks, mean uA, uB, welfare; dt of contacts / deadlocks"]
    out = []
    for (a, b), rs in sorted(by.items()):
        n = len(rs)
        c = sum(int(r["contact"]) for r in rs)
        d = sum(int(r["deadlock"]) for r in rs)
        ua = st.mean(float(r["uA"]) for r in rs)
        ub = st.mean(float(r["uB"]) for r in rs)
        cdt = sorted({float(r["dt"]) for r in rs if int(r["contact"])})
        ddt = sorted({float(r["dt"]) for r in rs if int(r["deadlock"])})
        out.append({"a": a, "b": b, "runs": n, "contacts": c, "deadlocks": d, "uA": round(ua, 4), "uB": round(ub, 4)})
        lines.append(f"{a:>11}/{b:<11} {n:>4} contact {c:>3} deadlock {d:>3} uA {ua:+.3f} uB {ub:+.3f} W {ua + ub:+.3f} | "
                     f"{' '.join(f'{x:g}' for x in cdt) or '-'} | {' '.join(f'{x:g}' for x in ddt) or '-'}")
    lines.append("# per pair and dt: contacts / runs, deadlocks, mean welfare")
    for (a, b, dt), rs in sorted(by_dt.items()):
        lines.append(f"dt {dt:+5.2f} {a:>11}/{b:<11} contact {sum(int(r['contact']) for r in rs)}/{len(rs)} deadlock "
                     f"{sum(int(r['deadlock']) for r in rs)} W {st.mean(float(r['uA']) + float(r['uB']) for r in rs):+.3f}")
    errs = []
    for r in rows:  # stamp - the other's own rest
        for me, ot in (("A", "B"), ("B", "A")):
            if r.get(f"read{me}", "") not in ("", None) and r.get(f"rest{ot}", "") not in ("", None):
                errs.append(float(r[f"read{me}"]) - float(r[f"rest{ot}"]))
    if errs:
        errs.sort()
        q = lambda p: errs[min(len(errs) - 1, int(p * len(errs)))]  # noqa: E731
        lines.append(f"# rest stamp error (read - the other's own rest), n {len(errs)}: min {errs[0]:+.2f} q1% {q(0.01):+.2f} "
                     f"median {q(0.5):+.2f} q99% {q(0.99):+.2f} max {errs[-1]:+.2f}")
    return lines, out


def compare(args, rows):
    bias = F.FM_BIAS
    F.set_fm(bias=0.0)  # the kinematic model perceives the truth: no stamp lag to correct
    try:
        return _compare(args, rows)
    finally:
        F.set_fm(bias=bias)


def _compare(args, rows):
    SUB = tuple(args.sub.split(","))  # noqa: N806
    lags = G.car_lags()
    rule = {r[0]: r for r in F.RULES}
    by = {}
    for r in rows:
        by.setdefault((float(r["dt"]), r["a"], r["b"]), []).append(r)
    lines, inside, total, agree, n_out, cells = [], 0, 0, 0, 0, []
    for dt in [float(x) for x in args.dts.split(",")]:
        ja_ar, jb_ar, ja_k, jb_k, outs_k, rs_ar = {}, {}, {}, {}, {}, {}
        for a in SUB:
            for b in SUB:
                rs = by[(dt, a, b)]
                ko = F.simulate(rule[a], rule[b], dt, args.v, args.v, lags)
                ok_ = replace(ko.o, peak=ko.o.peak_w)  # scored as the arena scores (074 D2)
                ja_ar[(a, b)] = st.mean(float(r["uA"]) for r in rs)
                jb_ar[(a, b)] = st.mean(float(r["uB"]) for r in rs)
                ja_k[(a, b)], jb_k[(a, b)] = G.utility(ok_, 0), G.utility(ok_, 1)
                outs_k[(a, b)], rs_ar[(a, b)] = ko, rs
                ok_all = True
                for key, kv in (("uA", ja_k[(a, b)]), ("uB", jb_k[(a, b)])):
                    xs = [float(r[key]) for r in rs]
                    mu, sd = st.mean(xs), (st.pstdev(xs) if len(xs) > 1 else 0.0)
                    hit = abs(kv - mu) <= max(2 * sd, 0.02)
                    inside += hit
                    total += 1
                    ok_all &= hit
                maj = lambda f: max(set(f(r) for r in rs), key=lambda x: sum(f(r) == x for r in rs))  # noqa: E731
                ar_o = (maj(lambda r: r["first"]), maj(lambda r: int(r["contact"])), maj(lambda r: int(r["deadlock"])))
                k_o = (ko.o.first, int(ko.o.contact), int(ko.deadlock))
                same = ar_o == k_o
                agree += same
                n_out += 1
                lines.append(f"dt {dt:+4.1f} {a:>11}/{b:<11} | arena uA {ja_ar[(a, b)]:+.3f} uB {jb_ar[(a, b)]:+.3f} first "
                             f"{ar_o[0] or '-'} contact {ar_o[1]} deadlock {ar_o[2]} | kin uA {ja_k[(a, b)]:+.3f} uB "
                             f"{jb_k[(a, b)]:+.3f} first {k_o[0] or '-'} contact {k_o[1]} deadlock {k_o[2]} | "
                             f"{'in' if ok_all else 'OUT'}{'' if same else ' (outcome differs)'}")
        cls = {}
        for name, ja, jb in (("arena", ja_ar, jb_ar), ("kinematic", ja_k, jb_k)):
            pure, mixed = G.solve(ja, jb, list(SUB), list(SUB))
            if name == "arena":
                fmaj = lambda p: max(set(r["first"] for r in rs_ar[p]), key=lambda x: sum(r["first"] == x for r in rs_ar[p]))  # noqa: E731
                firsts = {fmaj(p) for p in pure}
                contact = any(st.mean(int(r["contact"]) for r in rs_ar[p]) >= 0.5 for p in pure)
                dead = any(st.mean(int(r["deadlock"]) for r in rs_ar[p]) >= 0.5 for p in pure)
            else:
                firsts = {outs_k[p].o.first for p in pure}
                contact = any(outs_k[p].o.contact for p in pure)
                dead = any(outs_k[p].deadlock for p in pure)
            c = "!" if contact else "W" if dead else ("C" if len(firsts) > 1 else (firsts.pop() or "?")) if pure else (
                "M" if mixed else "?")
            cls[name] = (c, " ".join(f"{a}/{b}" for a, b in pure))
        cells.append({"dt": dt, "v": args.v, "class_arena": cls["arena"][0], "class_kinematic": cls["kinematic"][0],
                      "eq_arena": cls["arena"][1], "eq_kinematic": cls["kinematic"][1]})
        lines.append(f"## dt {dt:+.1f}: class arena {cls['arena'][0]} ({cls['arena'][1]}) | kinematic {cls['kinematic'][0]} "
                     f"({cls['kinematic'][1]})")
    summ = {"cells": cells, "payoffs_inside": inside, "payoffs": total, "outcomes_agree": agree, "pairs": n_out,
            "classes_agree": sum(c["class_arena"] == c["class_kinematic"] for c in cells),
            "arena_hold_stop": G.ARENA_HOLD_STOP}
    lines.append(f"payoffs inside the arena's 95 % interval: {inside} of {total} ({100 * inside / max(total, 1):.0f} %); "
                 f"outcomes agree in {agree} of {n_out}; classes agree in {summ['classes_agree']} of {len(cells)}")
    return lines, summ


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=8)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", type=float, default=6.0)
    # off the rules' thresholds: TIE 0.5 s, RBL_NEAR 2 s
    p.add_argument("--dts", default="-6,-3,-1.5,-0.25,0,0.25,1.5,3,6")
    p.add_argument("--obs-delay", type=float, default=G.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    p.add_argument("--compare-only", action="store_true")
    p.add_argument("--sub", default=",".join(SUB), help="the rule subset (C8: " + ",".join(C8_SUB) + ")")
    p.add_argument("--pairs", default="full", choices=("full", "bar"))
    p.add_argument("--angle", type=float, default=90.0, help="road B's heading, deg (a skewed junction)")
    p.add_argument("--heavy", default="", choices=("", "A", "B"))
    p.add_argument("--heavy-bp", default="vehicle.carlamotors.european_hgv")
    p.add_argument("--e4", default="", help="E4's output (its maps/<bp>/ pedal maps), for --heavy")
    p.add_argument("--warmup", type=float, default=XA.WARMUP)
    p.add_argument("--fm-m", type=float, default=F.FM_M)
    p.add_argument("--fm-bias", type=float, default=0.2, help="the rest stamp's lag through arena.Observer (C8 measure-e)")
    p.add_argument("--czp", action="store_true", help="078 I3: czp's rules (czp_c1.5, czp0_c1.5) may be named in --sub; "
                   "--pairs bar then counts them as the protocol's rules")
    G.add_legacy_args(p)
    args = p.parse_args()
    G.apply_legacy_args(args)
    F.set_fm(args.fm_m, args.fm_bias)
    if args.czp:
        import czp_adapter
        global BAR_RULES
        names = czp_adapter.enable(angle=args.angle, bias=args.fm_bias, margin=args.fm_m)
        BAR_RULES = BAR_RULES + tuple(names)
        if args.heavy:
            import czp
            p_ = czp.load_params()
            czp_adapter.CAPS[args.heavy] = p_.capability("truck", v_max=args.v)
    os.makedirs(args.out, exist_ok=True)
    if args.compare_only:
        rows = list(csv.DictReader(open(os.path.join(args.out, "runs.csv"))))
        wall = None
    else:
        rows, wall = run(args)
    tl, tab = c8_table(rows)
    with open(os.path.join(args.out, "c8_table.txt"), "w") as f:
        f.write("\n".join(tl) + "\n")
    if args.pairs != "full":
        json.dump({"pairs": tab, "runs": len(rows), "wall_s": None if wall is None else round(wall, 1),
                   "args": vars(args)}, open(os.path.join(args.out, "summary.json"), "w"), indent=1)
        print("\n".join(tl[:len(tab) + 1]))
        return 0
    lines, summ = compare(args, rows)
    summ["wall_s"] = None if wall is None else round(wall, 1)
    summ["runs"] = len(rows)
    summ["pairs_c8"] = tab
    summ["args"] = vars(args)
    with open(os.path.join(args.out, "compare.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summ, f, indent=1)
    print("\n".join(l for l in lines if l.startswith("##") or l.startswith("payoffs")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
