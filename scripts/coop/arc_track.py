#!/usr/bin/env python3
"""Roadmap 074 arc tracking: the arena controller's lateral and longitudinal error against the
kinematic path, on the Y merge's arc (arena.xodr_ymerge) and S4's swerve (arena.xodr_narrow).

One car per road, alone, along the path the kinematic model assumes (ymerge_game.Path's diagonal;
narrow_game.lat_a's swerve), under the study's own controller: pure pursuit with look-ahead
max(6, 0.7 v) on the Y merge (ymerge_arena.PCar), max(3, 0.4 v) on S4 (narrow_arena.Passage). Per run:

  lateral   the car centre's signed offset from the path (m; + is right of the path in CARLA's frame,
            on the Y merge's left turn - is inside), over the curved part: max |e|, rms, and the mean
            (a steady cut); the front axle's too
  speed     a test of the longitudinal channel on the curve: closed-loop cruise at v up to TEST_START m
            before the curve, then the open-loop command 0 (hold speed) and, from the curve's start,
            -3 m/s^2 for 1.5 s, then 0 again. The same command series through the kinematic model's
            car (crossing_game.Car with the spec's command lags) gives the reference: max |dv| and
            |du| between the two over the test (scrub on the curve, pedal-map error, the lag model)

Usage (a CARLA of its own):
  scripts/coop/arc_track.py --port 2600 --out <dir> --geometry ymerge [--v 10,14]
  scripts/coop/arc_track.py --port 2600 --out <dir> --geometry narrow [--v 6,10] [--wn 30]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics as st
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_game as G  # noqa: E402

DT = AR.DT
A_STEP, T_STEP = -3.0, 1.5


def offset(path, key, x, y):
    s = path.project(key, x, y)
    px, py, ph = path.at(s)
    return s, (x - px) * -math.sin(ph) + (y - py) * math.cos(ph)


class Swerve:
    """S4's swerve (narrow_arena.Passage.y_a) as a path: arc length taken as x (slope at most 0.22)."""

    def __init__(self, passage):
        self.p = passage

    def project(self, key, x, y):
        return x

    def at(self, s):
        d = 0.5
        return s, self.p.y_a(s), math.atan2(self.p.y_a(s + d) - self.p.y_a(s - d), 2 * d)


class Probe:
    """One car on one path: a cruise, then the open-loop longitudinal test from the curve's start."""

    def __init__(self, ar, car, path, drive, s_spawn, s_curve0, s_curve1, s_end, v, place):
        self.ar, self.car, self.path, self.drive_fn = ar, car, path, drive
        self.s_curve0, self.s_curve1, self.s_end, self.v = s_curve0, s_curve1, s_end, v
        self.test_start = s_curve0 - 30.0
        place(s_spawn)
        self.phase = "cruise"
        self.rows, self.cmds = [], []
        self.t_test = None
        self.done = False

    def step(self):
        x, y, yaw, v = self.car.state()
        key = ("probe", id(self.car))
        s, e = offset(self.path, key, x, y)
        _, ef = offset(self.path, key, x + 0.5 * self.car.wheelbase * math.cos(yaw), y + 0.5 * self.car.wheelbase * math.sin(yaw))
        self.path.project(key, x, y)  # keep the search window on the centre
        if self.phase == "cruise":
            a = max(-3.0, min(2.0, 1.5 * (self.v - v)))
            if s >= self.test_start:
                self.phase, self.t_test, self.v_test0, self.s_test0 = "test", self.ar.t, v, s
        if self.phase == "test":
            tt = self.ar.t - self.t_test
            if not hasattr(self, "t_step0") and s >= self.s_curve0:
                self.t_step0 = tt
            a = A_STEP if hasattr(self, "t_step0") and self.t_step0 <= tt < self.t_step0 + T_STEP else 0.0
            self.cmds.append(a)
            self.rows.append((round(tt, 2), round(s, 3), round(v, 3), round(e, 3), round(ef, 3), a))
            if s >= self.s_end:
                self.done = True
        self.drive_fn(a)

    def summary(self):
        on = [r for r in self.rows if self.s_curve0 <= r[1] <= self.s_curve1]
        es, efs = [r[3] for r in on], [r[4] for r in on]
        # the kinematic reference: the same commands through crossing_game.Car with the car's lags
        kc = G.Car(0.0, self.v_test0, G.car_lags())
        du = dv = 0.0
        for r, a in zip(self.rows, self.cmds):
            kc.a = kc.act.step(a)
            kc.v = max(0.0, kc.v + kc.a * DT)
            kc.u += kc.v * DT
            dv = max(dv, abs(kc.v - r[2]))
            du = max(du, abs(kc.u - (r[1] - self.s_test0)))
        # the lateral acceleration the curve asks at the test speed
        return {"v": self.v, "n_curve": len(on),
                "e_max": round(max(abs(x) for x in es), 3) if es else None,
                "e_rms": round(math.sqrt(st.mean(x * x for x in es)), 3) if es else None,
                "e_mean": round(st.mean(es), 3) if es else None,
                "ef_max": round(max(abs(x) for x in efs), 3) if efs else None,
                "lon_dv_max": round(dv, 3), "lon_du_max": round(du, 3)}


def run(args):
    vs = [float(x) for x in args.v.split(",")]
    k = len(vs)
    out = {"geometry": args.geometry, "runs": []}
    if args.geometry == "ymerge":
        import ymerge_arena as YA
        import ymerge_game as Y
        ar = AR.Arena(args.port, k, AR.xodr_ymerge(k, YA.APPROACH, YA.BEYOND, YA.DIAG, Y.R_ARC, YA.SPACING))
    else:
        import narrow_arena as NA
        import narrow_game as N
        ar = AR.Arena(args.port, k, AR.xodr_narrow(k, NA.ROAD_LEN, NA.X0, args.wn, N.TAPER))
    try:
        actors, paths, places, spans = [], [], [], []
        for i in range(k):
            if args.geometry == "ymerge":
                site = YA.YSite(ar, i)
                ln = site.line["A"]
                x, y, h = ln.at(50.0)
                a = ar.world.spawn_actor(ar.bp, carla.Transform(carla.Location(x=x, y=y, z=site.z + 0.6), carla.Rotation(yaw=math.degrees(h))))
                s_m = ln.before
                spans.append((s_m - 220.0, s_m - Y.ARC_LEN, s_m, s_m + 60.0))
                paths.append(ln)
                places.append((site, ln))
            else:
                p = NA.Passage(ar, i, args.wn)
                x0 = p.z0 + N.Z_PAD - N.TAPER  # where the swerve starts (A's centre x)
                a = ar.world.spawn_actor(ar.bp, carla.Transform(carla.Location(x=100.0, y=p.y["A"], z=p.z + 0.6), carla.Rotation(yaw=0)))
                spans.append((x0 - 250.0, x0, p.z0 + p.wz - N.Z_PAD + N.TAPER, p.z0 + p.wz - N.Z_PAD + N.TAPER + 40.0))
                paths.append(Swerve(p))
                places.append((p, None))
            ar.actors.append(a)
            actors.append(a)
        ar.put_physics(actors)
        for _ in (0,):
            cars = [YA.PCar(ar, a) if args.geometry == "ymerge" else AR.Car(ar, a) for a in actors]
            probes = []
            for i, (car, v) in enumerate(zip(cars, vs)):
                s_spawn, c0, c1, s_end = spans[i]
                obj, ln = places[i]
                if args.geometry == "ymerge":
                    def place(s, car=car, ln=ln, site=obj):
                        x, y, h = ln.at(s)
                        ln.last.clear()
                        car.park(carla.Transform(carla.Location(x=x, y=y, z=site.z + 0.3), carla.Rotation(yaw=math.degrees(h))))
                    drive = (lambda a, car=car, ln=ln: car.drive_path(a, ln, ("probe", id(car))))
                else:
                    def place(s, car=car, p=obj):
                        car.park(carla.Transform(carla.Location(x=s, y=p.y["A"], z=p.z + 0.3), carla.Rotation(yaw=0)))
                    drive = (lambda a, car=car, p=obj: p.drive("A", car, a))
                probes.append(Probe(ar, car, paths[i], drive, s_spawn, c0, c1, s_end, v, place))
            for _ in range(3):
                for pr in probes:
                    pr.car.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()
            n = 0
            while not all(pr.done for pr in probes) and n < 6000:
                for pr in probes:
                    if not pr.done:
                        pr.step()
                    else:
                        pr.car.a.apply_control(carla.VehicleControl(brake=1.0))
                ar.tick()
                n += 1
            for pr in probes:
                rec = {"controller": "pure pursuit", **pr.summary()}
                out["runs"].append(rec)
                print(rec, flush=True)
                json.dump(pr.rows, open(os.path.join(args.out, f"trace_{args.geometry}_v{pr.v:g}.json"), "w"))
    finally:
        ar.close()
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--geometry", choices=("ymerge", "narrow"), required=True)
    p.add_argument("--v", default=None)
    p.add_argument("--wn", type=float, default=30.0)
    a = p.parse_args()
    a.v = a.v or ("10,14" if a.geometry == "ymerge" else "6,10")
    os.makedirs(a.out, exist_ok=True)
    out = run(a)
    json.dump(out, open(os.path.join(a.out, f"track_{a.geometry}.json"), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
