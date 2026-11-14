#!/usr/bin/env python3
"""Roadmap 074 C6 in the arena: car types, beliefs and their cost on CARLA's physics.

arena.py's parallel two-lane roads and cars (the spec's Model 3 physics, acb's controls), one
pair per road, warmed up to the context as arena.Pair does. The run is c6_belief's: A's rule
(do6's pool, or a probe then a rule) and B's rule (do6's B pool; B's preference types B_aw and
B_aw_react), each seeing the other through c6_belief.Obs (arena.Observer's model, seeded per run),
and A's belief filters (motion, class, v2x) watching B. A car's type is imposed on the Model 3:
its commanded acceleration clamped to the type's a_max and b_max, its box (for the gap rules,
the observers and every measure) the type's length and width, its lane change the type's t_lc.
The truck's body in CARLA stays the Model 3's, so its physical mass is not a truck's: only its
limits, its size as the rules and measures see it, and its lane change are.

Jobs are c6_belief's for a sample of contexts (`--cells`, d0/dv/capA/capB), so the same analysis
(c6_belief.cell_measures) runs on both, and D2's yardstick compares them: per option and B rule,
the kinematic mean payoffs inside the arena's 95 % interval (mean +- 2 sd over seeds, at least
+- 0.02), and the outcome (A ahead, contact) the same by majority.

Usage (a CARLA of its own per shard; this script makes it synchronous and puts it back):
  scripts/coop/c6_arena.py --port 2600 --shard 0/3 --out <dir> --cells <cells.json> [--k 4] [--seeds 8]
  scripts/coop/c6_arena.py --out <dir> --cells <cells.json> --analyze
  scripts/coop/c6_arena.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import os
import random
import statistics as st
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import c6_belief as K  # noqa: E402
import cutin_game as C1  # noqa: E402
import measures as M  # noqa: E402


def jobs(cells, seeds, vb, only_a=None):
    """c6_belief's jobs for the cells; only_a: A rules to keep (a top-up run for a rule added later)."""
    out = []
    for c in cells:
        for j in K.jobs_for([(c["d0"], c["dv"])], sorted({c["capA"], c["capB"]}), seeds, vb):
            if j[3] == c["capA"] and j[5] == c["capB"] and (not only_a or j[4] in only_a):
                out.append(j)
    return out


def make_pair_class():
    import arena as AR
    import carla

    class C6Pair(AR.Pair):
        """One c6 job on one road."""

        def __init__(self, arena, road, a_car, b_car, job, args):
            d0, dv, vb, ca, an, cb, bn, s, probe = job
            self.cj = job
            self.cap_a, self.cap_b = K.CAPS[ca], K.CAPS[cb]
            a_car.length, a_car.width = self.cap_a.length, self.cap_a.width
            b_car.length, b_car.width = self.cap_b.length, self.cap_b.width
            super().__init__(arena, road, a_car, b_car, {"d0": d0, "dv": dv, "vb": vb}, random.Random(0), args)
            self.rng = random.Random(K.run_seed(d0, dv, vb, s))
            rng = self.rng
            self.rule_a, self.rule_b = K.A_BY[an], K.B_BY[bn]
            self.react_a = 1.0 + rng.uniform(-K.OBS["jitter"], K.OBS["jitter"])
            self.react_b = self.rule_b.get("react", 1.0) + rng.uniform(-K.OBS["jitter"], K.OBS["jitter"])
            self.oa = K.Obs(K.OBS["delay"], K.OBS["sx"], K.OBS["sv"], rng)
            self.ob = K.Obs(K.OBS["delay"], K.OBS["sx"], K.OBS["sv"], rng)
            self.l_seen = self.cap_b.length + rng.gauss(0, K.SIGMA_LEN)
            self.probe = bool(probe)

        def start(self):
            super().start()
            vb = self.cj[2]
            self.va0 = max(0.0, min(self.cap_a.v_max, self.va0))
            self.vb0 = min(self.cap_b.v_max, self.vb0)
            self.PA = K.PolicyA(self.rule_a, self.cap_a, self.cap_b.length, self.react_a, self.probe)
            self.PB = K.PolicyB(self.rule_b, self.cap_b, self.react_b)
            self.F = K.Filters(vb, self.cj[5], self.l_seen)
            self.meas = M.Measures(b_max=(self.cap_a.b_max, self.cap_b.b_max))
            xa, _, _, va = self.A.state()
            xb, _, _, vbb = self.B.state()
            self.sb = K.Seen(0.0, xb, 0.0, vbb, 0.0)
            self.sa = K.Seen(0.0, xa, 0.0, va, 0.0)

        def step_run(self):
            t = self.ar.t - self.t0
            A, B = self.A, self.B
            xa, ya, _, va = A.state()
            xb, yb, _, vb = B.state()
            lat = self.lat()
            o, fresh = self.oa.push(t, xb - xa, 0.0, vb)
            fresh = fresh and o is not None
            # before the first observation arrives, the truth (as arena.Pair)
            self.sb = K.Seen(o.t, xa + o.x, 0.0, o.v, o.acc) if o is not None else K.Seen(t, xb, 0.0, vb, 0.0)
            o2, _ = self.ob.push(t, xa - xb, lat, va)
            self.sa = K.Seen(o2.t, xb + o2.x, o2.lat, o2.v, o2.acc) if o2 is not None else K.Seen(t, xa, lat, va, 0.0)
            aa, s_cmd = self.PA.step(t, xa, va, self.va0, self.sb, lat)
            self.F.step(t, self.sb, fresh, lat, self.PA.b_brake_seen,
                        (xa - self.cap_a.length / 2) - (self.sb.x + self.cap_b.length / 2), self.cap_b.length)
            ab = self.PB.step(t, xb, vb, self.vb0, self.sa, self.cap_a.length)
            aa = max(-self.cap_a.b_max, min(self.cap_a.a_max, aa))
            ab = max(-self.cap_b.b_max, min(self.cap_b.a_max, ab))
            A.drive(aa, self.yA0 + (self.yB - self.yA0) * s_cmd)
            B.drive(ab, self.yB)
            for k, vv in ((0, va), (1, vb)):
                h = self.vhist[k]
                h.append((t, vv))
                while h and t - h[0][0] > AR.PEAK_WINDOW + 1e-9:
                    h.popleft()
                if t - h[0][0] >= AR.PEAK_WINDOW - 1e-9:
                    self.peak[k] = max(self.peak[k], (h[0][1] - vv) / AR.PEAK_WINDOW)
            self.meas.push_poses(t, A.pose(), B.pose())
            if getattr(self.args, "trace", False):
                self.trace.append((round(t, 2), round(xa - self.xa0, 2), round(va, 2), round(xb - self.xa0, 2), round(vb, 2),
                                   round(s_cmd, 3), round(lat, 3), round(aa, 2), round(ab, 2), self.PA.decide, self.PA.dropping))
            if self.meas.clear <= 0.0:
                self.contact = True
            if self.contact or t >= C1.T_MAX - 1e-9 or (t >= C1.HORIZON - 1e-9 and (self.PA.merged(t) or t >= 10.0 + C1.HORIZON)):
                self.finish()

        def finish(self):
            xa, _, _, _ = self.A.state()
            xb, _, _, _ = self.B.state()
            t = self.ar.t - self.t0
            self.F.step(max(t, K.T_SNAP[-1]), self.sb, False, self.lat(), self.PA.b_brake_seen)
            m = self.meas
            lost_a = M.lost_progress(self.va0, m.s_h[0], C1.HORIZON)
            lost_b = M.lost_progress(self.vb0, m.s_h[1], C1.HORIZON)
            got_in = self.lat() >= 0.9
            if self.contact:
                lost_a = lost_b = 1.0
            elif not got_in:
                lost_a = 1.0
            m.pet = m._pet()
            o = C1.Outcome((lost_a, lost_b), m.ttc2d, m.thw, self.contact, (self.peak[0], self.peak[1]),
                           got_in and not self.PA.dropping, "", m.clear, m.pet, tuple(m.lat_peak))
            pref = C1.Preference()
            d0, dv, vb, ca, an, cb, bn, s, probe = self.cj
            snaps = self.F.snaps
            self.rec = {"d0": d0, "dv": dv, "vb": vb, "capA": ca, "a": ("P+" if probe else "") + an, "capB": cb,
                        "b": bn, "seed": s,
                        "uA": round(C1.utility(o, 0, self.cap_a, pref), 4), "uB": round(C1.utility(o, 1, self.cap_b, pref), 4),
                        "contact": int(self.contact), "ahead": int(got_in and xa > xb), "got_in": int(got_in),
                        "clear": round(m.clear, 3) if math.isfinite(m.clear) else 99.0, "peakB": round(self.peak[1], 2),
                        "t_decide": None if self.PA.decide is None else round(self.PA.decide, 2),
                        "types": "|".join(f"{c}/{p}" for c, p in self.F.types),
                        "post": json.dumps({k: [round(x, 4) for x in snaps[K.PROBE_T][k]] for k in K.FILTERS}),
                        "post0": json.dumps([round(x, 4) for x in snaps[0.0]["class"]]),
                        "maps": json.dumps({f"{k}@{tq:g}": max(range(len(p)), key=lambda i: p[i])
                                            for tq, dd in snaps.items() for k, p in dd.items()}),
                        "d0_real": round(self.d0_real, 2), "dv_real": round(self.dv_real, 2)}
            if getattr(self.args, "trace", False):
                self.rec["trace"] = json.dumps(self.trace)
            self.phase = "done"

    return C6Pair, AR, carla


def run_shard(args):
    if args.jobs:  # explicit jobs (a JSON list of c6 job tuples), traced
        work = [tuple(j) for j in json.load(open(args.jobs))]
    else:
        work = jobs(json.load(open(args.cells)), args.seeds, args.vb, args.only_a.split(",") if args.only_a else None)
    i, n = (int(x) for x in args.shard.split("/"))
    work = work[i::n]
    C6Pair, AR, carla = make_pair_class()

    class Args:
        pass
    a = Args()
    a.obs_delay, a.obs_sigma_x, a.obs_sigma_v, a.reaction_jitter = 0.19, 0.3, 0.2, 0.2
    a.trace = bool(args.jobs)
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, f"arena_{args.tag}{i}.csv.gz")
    ar = AR.Arena(args.port, args.k)
    t_start = time.monotonic()
    try:
        cars = AR.setup_cars(ar, args.k)
        real = [(c[0].length, c[0].width, c[1].length, c[1].width) for c in cars]
        queue = deque(work)
        with gzip.open(path, "wt", newline="") as f:
            w = None
            done = 0
            while queue:
                pairs = []
                for r in range(args.k):
                    if not queue:
                        break
                    ac, bc = cars[r]
                    ac.length, ac.width, bc.length, bc.width = real[r]
                    p = C6Pair(ar, r, ac, bc, queue.popleft(), a)
                    p.place()
                    pairs.append(p)
                for _ in range(3):
                    for p in pairs:
                        for c in (p.A, p.B):
                            c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                    ar.tick()
                for p in pairs:
                    p.tw = ar.t
                while any(p.phase != "done" for p in pairs):
                    for p in pairs:
                        if p.phase == "warmup":
                            p.step_warmup()
                        elif p.phase == "run":
                            p.step_run()
                        else:
                            p.A.drive(-2.0, p.yA0 if not p.rec["got_in"] else p.yB)
                            p.B.drive(-2.0, p.yB)
                    ar.tick()
                for p in pairs:
                    if w is None:
                        w = csv.DictWriter(f, fieldnames=list(p.rec))
                        w.writeheader()
                    w.writerow(p.rec)
                done += len(pairs)
                f.flush()
                print(f"  {done}/{len(work)} runs, {time.monotonic() - t_start:.0f} s", flush=True)
    finally:
        ar.close()
    return path


# --- analysis: the arena's measures, and D2 against the kinematic model on the same jobs ---------
def kinematic_rows(cells, seeds, vb, procs):
    from multiprocessing import Pool
    with Pool(procs) as pool:
        return pool.map(K.run_job, jobs(cells, seeds, vb), chunksize=16)


def parse(rows):
    for r in rows:
        for k in ("d0", "dv", "vb", "uA", "uB", "clear", "peakB"):
            r[k] = float(r[k])
        for k in ("seed", "contact", "ahead", "got_in"):
            r[k] = int(r[k])
        for k in ("post", "post0", "maps"):
            if isinstance(r[k], str):
                r[k] = json.loads(r[k])
    return rows


def d2(arena_rows, kin_rows):
    """D2's yardstick per (cell, option, B rule): the kinematic mean inside the arena's mean +- 2 sd
    (at least +- 0.02), for uA and uB; outcomes (A ahead, contact) the same by majority."""
    g = {}
    for side, rows in (("arena", arena_rows), ("kin", kin_rows)):
        for r in rows:
            g.setdefault((r["d0"], r["dv"], r["capA"], r["capB"], r["a"], r["b"]), {}).setdefault(side, []).append(r)
    inside = total = agree = n_out = 0
    misses = []
    for k, d in sorted(g.items()):
        if "arena" not in d or "kin" not in d:
            continue
        for f in ("uA", "uB"):
            xs = [r[f] for r in d["arena"]]
            m, s = st.mean(xs), (st.stdev(xs) if len(xs) > 1 else 0.0)
            km = st.mean(r[f] for r in d["kin"])
            total += 1
            if abs(km - m) <= max(2 * s, 0.02):
                inside += 1
            else:
                misses.append({"key": list(k), "f": f, "arena": round(m, 3), "sd": round(s, 3), "kin": round(km, 3)})
        oa = (round(st.mean(r["ahead"] for r in d["arena"])), round(st.mean(r["contact"] for r in d["arena"])))
        ok = (round(st.mean(r["ahead"] for r in d["kin"])), round(st.mean(r["contact"] for r in d["kin"])))
        n_out += 1
        agree += oa == ok
    return {"payoffs": total, "payoffs_inside": inside, "share": round(inside / total, 4) if total else None,
            "outcomes": n_out, "outcomes_agree": agree, "outcome_share": round(agree / n_out, 4) if n_out else None,
            "misses": misses[:200]}


def analyze(args):
    cells = json.load(open(args.cells))
    rows = []
    for p in sorted(os.listdir(args.out)):
        if p.startswith("arena_") and p.endswith(".csv.gz"):
            with gzip.open(os.path.join(args.out, p), "rt") as f:
                rows += list(csv.DictReader(f))
    rows = parse(rows)
    have = {r["a"] for r in rows}  # the arena's options (a run made before a rule joined the pool lacks it)
    kin = [r for r in parse(kinematic_rows(cells, args.seeds, args.vb, args.procs)) if r["a"] in have]
    out = {"runs": len(rows), "d2": d2(rows, kin), "cells": [],
           "intent_likelihood": {"arena": K.calibrate(rows), "kin": K.calibrate(kin)}}
    ctx_err = [abs(float(r["d0_real"]) - r["d0"]) for r in rows]
    out["context_error"] = {"d0_max": round(max(ctx_err), 2),
                            "dv_max": round(max(abs(float(r["dv_real"]) - r["dv"]) for r in rows), 2)}
    for side, rs_all in (("arena", rows), ("kin", kin)):
        g = {}
        for r in rs_all:
            g.setdefault((r["d0"], r["dv"], r["vb"], r["capA"]), []).append(r)
        for k, rs in sorted(g.items()):
            for c in K.cell_measures(K.Ctx(rs), *k):
                c["side"] = side
                out["cells"].append(c)
    # the D6 measures side by side
    pairs = {}
    for c in out["cells"]:
        pairs.setdefault((c["d0"], c["dv"], c["capA"], c["capB"], c["pref"]), {})[c["side"]] = c
    comp = []
    for k, d in sorted(pairs.items()):
        if "arena" not in d or "kin" not in d:
            continue
        a, kk = d["arena"], d["kin"]
        e = {"d0": k[0], "dv": k[1], "capA": k[2], "true": f"{k[3]}/{k[4]}"}
        for f in ("L_stock", "s_regret_class", "s_regret_motion", "s_voi_v2x", "s_voi_all", "p_regret_class", "p_voi_v2x"):
            e[f] = [a[f], kk[f]]
        for f in ("s_regret_class", "s_voi_v2x", "s_voi_all"):
            e[f + "_bar"] = [bar(a, f), bar(kk, f)]
        comp.append(e)
    out["compare"] = comp
    n = len(comp)
    out["bar_agree"] = {f: sum(1 for e in comp if e[f + "_bar"][0] == e[f + "_bar"][1]) for f in ("s_regret_class", "s_voi_v2x", "s_voi_all")}
    out["bar_arena"] = {f: sum(1 for e in comp if e[f + "_bar"][0]) for f in ("s_regret_class", "s_voi_v2x", "s_voi_all")}
    out["bar_kin"] = {f: sum(1 for e in comp if e[f + "_bar"][1]) for f in ("s_regret_class", "s_voi_v2x", "s_voi_all")}
    out["n_compare"] = n
    json.dump(out, open(os.path.join(args.out, "summary.json"), "w"), indent=1)
    lines = [f"C6 arena: {len(rows)} runs; D2 payoffs {out['d2']['payoffs_inside']}/{out['d2']['payoffs']} "
             f"({out['d2']['share']:.0%}), outcomes {out['d2']['outcomes_agree']}/{out['d2']['outcomes']} "
             f"({out['d2']['outcome_share']:.0%}); context error {out['context_error']}",
             f"cells over the bar (arena / kinematic, of {n}): " + ", ".join(
                 f"{f} {out['bar_arena'][f]}/{out['bar_kin'][f]} (agree {out['bar_agree'][f]})" for f in out["bar_agree"]),
             "cell: [arena, kinematic]"]
    for e in comp:
        lines.append(f"  {e['d0']:+5.1f} {e['dv']:+4.1f} A {e['capA']:5s} B {e['true']:12s} L_stock {e['L_stock']} "
                     f"regret_class {e['s_regret_class']} voi_v2x {e['s_voi_v2x']} voi_all {e['s_voi_all']}")
    text = "\n".join(lines) + "\n"
    open(os.path.join(args.out, "summary.txt"), "w").write(text)
    print(text)


def bar(c, f):
    v, s, L = c[f], c.get(f + "_se", 0.0), c["L_stock"]
    return int(v >= K.FLOOR and v > 2 * s and v >= K.SHARE * max(L, 0.0))


def selftest():
    cells = [{"d0": 10.0, "dv": -2.0, "capA": "m3", "capB": "truck"}]
    js = jobs(cells, 2, 12.0)
    assert js and all(j[5] == "truck" and j[3] == "m3" for j in js), js[:3]
    assert len(js) == 2 * len(K.A_POOL) * (len(K.B_POOL) + 2)
    # d2 on identical rows agrees everywhere
    rows = parse([K.run_job(j) for j in js[:16]])
    d = d2(rows, rows)
    assert d["payoffs_inside"] == d["payoffs"] and d["outcomes_agree"] == d["outcomes"], d
    print("selftest ok")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int)
    p.add_argument("--out", required=False)
    p.add_argument("--cells")
    p.add_argument("--shard", default="0/1")
    p.add_argument("--k", type=int, default=4)
    p.add_argument("--seeds", type=int, default=8)
    p.add_argument("--vb", type=float, default=12.0)
    p.add_argument("--procs", type=int, default=12)
    p.add_argument("--only-a", default="", help="comma-separated A rules: a top-up run")
    p.add_argument("--jobs", default="", help="a JSON list of c6 job tuples, run with traces")
    p.add_argument("--tag", default="", help="shard file prefix (arena_<tag><i>.csv.gz)")
    p.add_argument("--analyze", action="store_true")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args()
    if a.selftest:
        return selftest()
    if a.analyze:
        analyze(a)
    else:
        run_shard(a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
