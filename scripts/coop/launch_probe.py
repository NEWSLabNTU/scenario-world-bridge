#!/usr/bin/env python3
"""Roadmap 074 arc tracking: the arena car's launch from rest against the kinematic model's.

One car at rest on arena.py's straight road, held 2 s with -3 m/s^2 commanded (both sides), then the command crossing_game.drive gives a car that
goes (min(A_MAX, (v0 - v) / DT) up to v0), through arena.py's controls (pedal maps, pedal lags); the
same command series through crossing_game.Car with the spec's command lags is the reference. Also the
stop: from v0 the command -B_MAX (a safety-layer stop) to rest. Prints u(t), v(t) for both and the gap.

Usage: scripts/coop/launch_probe.py --port 2600 --out <dir> [--v0 6,10]
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_game as G  # noqa: E402


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--v0", default="6,10")
    a = p.parse_args()
    v0s = [float(x) for x in a.v0.split(",")]
    ar = AR.Arena(a.port, len(v0s))
    res = {}
    try:
        actors = [ar.spawn(i, -1, 50.0) for i in range(len(v0s))]
        ar.put_physics(actors)
        cars = [AR.Car(ar, x) for x in actors]
        ys = [ar.lane_y(i, -1)[0] for i in range(len(v0s))]
        for c, y in zip(cars, ys):
            c.park(carla.Transform(carla.Location(x=50.0, y=y, z=ar.lane_y(0, -1)[1] + 0.3)))
        for _ in range(40):  # settle at rest
            for c, y in zip(cars, ys):
                c.drive(-3.0, y)
            ar.tick()
        kin = [G.Car(0.0, 0.0, G.car_lags()) for _ in v0s]
        for k in kin:  # the same 2 s of braking at rest as the arena car's settle: the brake to release
            for _ in range(40):
                k.act.step(-3.0)
        x0 = [c.state()[0] for c in cars]
        logs = [[] for _ in v0s]
        phase = ["launch"] * len(v0s)
        t = 0.0
        while t < 12.0:
            for i, (c, y, v0) in enumerate(zip(cars, ys, v0s)):
                x, _, _, v = c.state()
                if phase[i] == "launch" and t >= 6.0:
                    phase[i] = "stop"
                cmd = G.drive(v, v0) if phase[i] == "launch" else -G.B_MAX
                c.drive(cmd, y)
                k = kin[i]
                # the reference integrates its own command (from its own speed), as the model's car would
                kc = G.drive(k.v, v0) if phase[i] == "launch" else -G.B_MAX
                k.a = k.act.step(kc)
                k.v = max(0.0, k.v + k.a * G.DT)
                k.u += k.v * G.DT
                logs[i].append((round(t, 2), round(x - x0[i], 3), round(v, 3), round(k.u, 3), round(k.v, 3), round(cmd, 2)))
            ar.tick()
            t += G.DT
        for v0, lg in zip(v0s, logs):
            launch = [r for r in lg if r[0] < 6.0]
            gaps = {f"du_at_{ts}s": next(round(r[3] - r[1], 2) for r in launch if r[0] >= ts) for ts in (0.5, 1.0, 1.5, 2.0, 3.0, 5.0)}
            t90a = next((r[0] for r in launch if r[2] >= 0.9 * v0), None)
            t90k = next((r[0] for r in launch if r[4] >= 0.9 * v0), None)
            stop = [r for r in lg if r[0] >= 6.0]
            res[v0] = {"t_to_90pct_arena": t90a, "t_to_90pct_kin": t90k, **gaps,
                       "stop_dist_arena": round(next(r[1] for r in stop if r[2] < 0.05) - stop[0][1], 2) if any(r[2] < 0.05 for r in stop) else None,
                       "stop_dist_kin": round(next(r[3] for r in stop if r[4] < 0.05) - stop[0][3], 2) if any(r[4] < 0.05 for r in stop) else None}
            print(v0, res[v0], flush=True)
        os.makedirs(a.out, exist_ok=True)
        json.dump({"summary": res, "logs": {str(v): lg for v, lg in zip(v0s, logs)}}, open(os.path.join(a.out, "launch.json"), "w"))
    finally:
        ar.close()


if __name__ == "__main__":
    main()
