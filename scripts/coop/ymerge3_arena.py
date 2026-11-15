#!/usr/bin/env python3
"""Roadmap 074 N5's swb check, the arena side: the 45 deg Y merge with three cars (ymerge3).

ymerge_arena.py's world (arena.xodr_ymerge, Town03 j1901's geometry), cars, pure pursuit and
warm-up, with a third car: C on the main road H_FOLLOW s and a car behind B. Each car is driven by
its ymerge3.Controller on what it perceives of the other two through arena.Observer (10 Hz,
delayed, noisy; one observer per watcher and car). The run is judged by ymerge3.score3 from the
cars' true states each tick. This is the reference n5_swb.py compares swb's runs with.

Usage (a CARLA of its own; the arena makes it synchronous and puts it back):
  scripts/coop/ymerge3_arena.py --port 2600 --out <dir> [--k 6] [--seeds 3] [--v 10]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import random
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_game as G  # noqa: E402
import ymerge3 as Y3  # noqa: E402
import ymerge_arena as YA  # noqa: E402
import ymerge_game as Y  # noqa: E402

CARS = "ABC"
LINE = {"A": "A", "B": "B", "C": "B"}  # C runs on the main road


class Run3:
    def __init__(self, site, cars, job, rng, args):
        self.x, self.cars, self.job = site, dict(zip(CARS, cars)), job
        v = job["v"]
        self.target_u = Y3.start_u(job["dt"], v, job["h"])
        d_ramp, _, _ = YA.ramp(YA.WARMUP, v)
        self.u_spawn = {k: self.target_u[k] - d_ramp for k in CARS}
        self.ctl = {k: Y3.Controller(k, job["a"], v, hold_stop=G.ARENA_HOLD_STOP) for k in CARS}
        self.obs = {(me, o): AR.Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)
                    for me in CARS for o in CARS if o != me}
        self.last_stamp = {key: None for key in self.obs}
        self.phase, self.rec = "warmup", None

    def u(self, k):
        return self.x.u(LINE[k], self.cars[k])

    def place(self):
        for k in CARS:
            self.x.place_at(LINE[k], self.cars[k], self.u_spawn[k])

    def start_warmup(self):
        self.tw = self.x.ar.t

    def _push(self, t, feed):
        for (me, o), ob in self.obs.items():
            seen = ob.push(t, self.u(o), 0.0, self.cars[o].state()[3])
            if feed and seen is not None and seen[0] >= -1e-9 and seen[0] != self.last_stamp[(me, o)]:
                self.last_stamp[(me, o)] = seen[0]
                self.ctl[me].observe(seen[0], o, seen[1], seen[3], seen[4])

    def step_warmup(self):
        t = self.x.ar.t - self.tw
        v = self.job["v"]
        for k in CARS:
            car = self.cars[k]
            d, vr, ar_ = YA.ramp(t, v)
            a = max(-4.0, min(3.0, ar_ + 0.6 * (self.u_spawn[k] + d - self.u(k)) + 1.8 * (vr - car.state()[3])))
            car.drive_path(a, self.x.line[LINE[k]], id(car))
        self._push(t - YA.WARMUP, False)
        if t >= YA.WARMUP - 1e-9:
            self.phase = "run"
            self.t0 = self.x.ar.t
            self.u0 = {k: self.u(k) for k in CARS}
            self.v0 = {k: self.cars[k].state()[3] for k in CARS}
            self.ticks = [(0.0, self._state())]

    def _state(self):
        out = {}
        for k in CARS:
            x, y, yaw, v = self.cars[k].state()
            out[k] = (self.u(k), v, x, y, yaw)
        return out

    def step_run(self):
        t = self.x.ar.t - self.t0
        self._push(t, True)
        s = self.ticks[-1][1]
        for k in CARS:
            u, v = s[k][0], s[k][1]
            a = self.ctl[k].step(t, u, v)
            self.cars[k].drive_path(a, self.x.line[LINE[k]], id(self.cars[k]))
            self.ctl[k].after(t, u, v)

    def after_tick(self):
        t = self.x.ar.t - self.t0
        s = self._state()
        self.ticks.append((t, s))
        # scored in full every second (score3 stops at the first contact, so a later look loses nothing) and at the end
        end = t >= Y3.T_END or (t >= G.HORIZON - 1e-9 and s["A"][0] >= Y.EXIT + 2)
        if not end and len(self.ticks) % 20:
            return
        res = Y3.score3(self.ticks, self.v0, boxes={k: (self.cars[k].length, self.cars[k].width) for k in CARS})
        if res["contact"] or end:
            self.rec = {**self.job, **res, "uA0": round(self.u0["A"], 2), "uB0": round(self.u0["B"], 2), "uC0": round(self.u0["C"], 2),
                        "vA0": round(self.v0["A"], 2), "vB0": round(self.v0["B"], 2), "vC0": round(self.v0["C"], 2),
                        "dt_real": round(-self.u0["A"] / max(self.v0["A"], 0.1) + self.u0["B"] / max(self.v0["B"], 0.1), 3),
                        "h_real": round((self.u0["B"] - self.u0["C"] - Y.L) / max(self.v0["C"], 0.1), 3)}
            for k in CARS:
                self.rec.update({f"{k}_{x}": v for x, v in self.ctl[k].record().items()})
            self.phase = "done"


def jobs(args):
    return [{"dt": dt, "v": args.v, "h": args.h, "a": a, "seed": s}
            for dt in [float(x) for x in args.dts.split(",")] for a in Y3.A_RULES for s in range(args.seeds)]


def run(args):
    rng = random.Random(args.seed)
    work = deque(jobs(args))
    n_all = len(work)
    ar = AR.Arena(args.port, args.k, AR.xodr_ymerge(args.k, YA.APPROACH, YA.BEYOND, YA.DIAG, Y.R_ARC, YA.SPACING))
    rows = []
    t_start = time.monotonic()
    try:
        sites = [YA.YSite(ar, i) for i in range(args.k)]
        devs = [s.check() for s in sites]
        print("lane centre vs map, worst (m):", devs, flush=True)
        if any(d > 0.3 for dv in devs for d in dv.values()):
            raise SystemExit(f"the generated lanes are not where ymerge_game.Path puts them: {devs}")
        actors = []
        for s in sites:
            for k, at in (("A", 50.0), ("B", 50.0), ("C", 20.0)):
                x, y, h = s.line[LINE[k]].at(at)
                a = ar.world.spawn_actor(ar.bp, carla.Transform(carla.Location(x=x, y=y, z=s.z + 0.6), carla.Rotation(yaw=h * 57.29578)))
                ar.actors.append(a)
                actors.append(a)
        ar.put_physics(actors)
        cars = [YA.PCar(ar, a) for a in actors]
        while work:
            runs = []
            for i, s in enumerate(sites):
                if not work:
                    break
                r = Run3(s, cars[3 * i:3 * i + 3], work.popleft(), rng, args)
                r.place()
                runs.append(r)
            for _ in range(3):
                for r in runs:
                    for c in r.cars.values():
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
                        for k in CARS:
                            c = r.cars[k]
                            c.drive_path(-3.0, r.x.line[LINE[k]], id(c))
                ar.tick()
                for r in runs:
                    if r.phase == "run":
                        r.after_tick()
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
    json.dump({"lane_dev_m": devs, "wall_s": round(time.monotonic() - t_start, 1), "runs": len(rows)},
              open(os.path.join(args.out, "summary.json"), "w"), indent=1)
    return rows


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--k", type=int, default=6)
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--v", type=float, default=Y3.V)
    p.add_argument("--h", type=float, default=Y3.H_FOLLOW)
    p.add_argument("--dts", default=",".join(str(d) for d in Y3.DTS))
    p.add_argument("--obs-delay", type=float, default=G.OBS)
    p.add_argument("--obs-sigma-x", type=float, default=0.3)
    p.add_argument("--obs-sigma-v", type=float, default=0.2)
    G.add_legacy_args(p)
    args = p.parse_args()
    G.apply_legacy_args(args)
    run(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
