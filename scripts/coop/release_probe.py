#!/usr/bin/env python3
"""Roadmap 074: the restart after a held stop, arena against the kinematic model.

One car per hold time on arena.py's straight road. Each car launches from rest to V_IN (crossing_game.drive), brakes to rest as
a held car does (-STOP_A while v >= HOLD_STOP_V, then the rules' hold: the kinematic model's
HOLD_STOP; in the arena the command 0, which arena.Longitudinal floors to -3.4 m/s^2 below 0.5 m/s,
or coasts with --legacy-arena-coast), stays held for HOLD s after it reaches rest, then goes
(crossing_game.drive to V_GO). The kinematic car (crossing_game.Car with the spec's command lags and
LAUNCH_DEAD) runs the same command logic on its own state. Aligned on the go command, per hold time:
when each first moves 0.05 m, the gap in u at 0.5-5 s, and the arena's brake pedal at the go.

Usage: scripts/coop/release_probe.py --port 2600 --out <dir> [--holds 0.2,0.5,1,2] [--legacy-arena-coast]
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arena as AR  # noqa: E402
import carla  # noqa: E402
import crossing_game as G  # noqa: E402

V_IN, V_GO, STOP_A = 3.0, 10.0, 3.0
T_UP = 4.0  # s of launch to V_IN before the stop


class Script:
    """The command series a held car gets, from its own state: up, stop, hold, go."""

    def __init__(self, hold, kin):
        self.hold, self.kin = hold, kin
        self.phase, self.t_rest, self.t_go = "up", None, None

    def cmd(self, t, v):
        if self.phase == "up" and t >= T_UP:
            self.phase = "stop"
        if self.phase == "stop":
            if v < 0.05:
                self.t_rest = self.t_rest if self.t_rest is not None else t
                if t - self.t_rest >= self.hold - 1e-9:
                    self.phase, self.t_go = "go", t
            if self.phase == "stop":
                if v >= G.HOLD_STOP_V:
                    return -STOP_A
                # held: the rules' clip; the kinematic car brakes to rest (HOLD_STOP), the arena commands 0
                return -min(G.B_MAX, v / G.DT) if (self.kin and G.HOLD_STOP) else 0.0
        return G.drive(v, V_IN if self.phase == "up" else V_GO)


def first_move(log, t_go, du=0.05):
    u0 = next(r[1] for r in log if r[0] >= t_go - 1e-9)
    return next((round(r[0] - t_go, 2) for r in log if r[0] >= t_go and r[1] - u0 >= du), None), u0


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--holds", default="0.2,0.5,1,2")
    G.add_legacy_args(p)
    a = p.parse_args()
    G.apply_legacy_args(a)
    holds = [float(x) for x in a.holds.split(",")]
    ar = AR.Arena(a.port, len(holds))
    res, logs_out = {}, {}
    try:
        actors = [ar.spawn(i, -1, 50.0) for i in range(len(holds))]
        ar.put_physics(actors)
        cars = [AR.Car(ar, x) for x in actors]
        ys = [ar.lane_y(i, -1)[0] for i in range(len(holds))]
        for c, y in zip(cars, ys):
            c.park(carla.Transform(carla.Location(x=50.0, y=y, z=ar.lane_y(0, -1)[1] + 0.3)))
        for _ in range(40):  # settle at rest, braked
            for c, y in zip(cars, ys):
                c.drive(-3.0, y)
            ar.tick()
        kin = [G.Car(0.0, 0.0, G.car_lags()) for _ in holds]
        for k in kin:
            for _ in range(40):
                k.act.step(-3.0, 0.0)
        sa = [Script(h, False) for h in holds]
        sk = [Script(h, True) for h in holds]
        x0 = [c.state()[0] for c in cars]
        la, lk = [[] for _ in holds], [[] for _ in holds]
        t = 0.0
        while t < T_UP + 20.0:
            for i, (c, y) in enumerate(zip(cars, ys)):
                x, _, _, v = c.state()
                cmd = sa[i].cmd(t, v)
                c.drive(cmd, y)
                la[i].append((round(t, 2), round(x - x0[i], 4), round(v, 4), round(cmd, 2), round(c.lag.brake, 3), round(c.lag.throttle, 3)))
                k = kin[i]
                kc = sk[i].cmd(t, k.v)
                k.a = k.act.step(kc, k.v)
                k.v = max(0.0, k.v + k.a * G.DT)
                k.u += k.v * G.DT
                lk[i].append((round(t, 2), round(k.u, 4), round(k.v, 4), round(kc, 2), round(k.act.brake, 3), round(k.act.drive, 3)))
            ar.tick()
            t += G.DT
            if all(s.t_go is not None and t - s.t_go > 6.0 for s in sa + sk):
                break
        for h, s_a, s_k, l_a, l_k in zip(holds, sa, sk, la, lk):
            ma, ua = first_move(l_a, s_a.t_go)
            mk, uk = first_move(l_k, s_k.t_go)
            ga = {r[0] - s_a.t_go: r[1] - ua for r in l_a if r[0] >= s_a.t_go - 1e-9}
            gk = {r[0] - s_k.t_go: r[1] - uk for r in l_k if r[0] >= s_k.t_go - 1e-9}
            at = lambda g, ts: next(v for dt_, v in sorted(g.items()) if dt_ >= ts - 1e-9)  # noqa: E731
            go_a = next(r for r in l_a if r[0] >= s_a.t_go - 1e-9)
            res[h] = {"rest_to_go_s": h, "move_arena_s": ma, "move_kin_s": mk,
                      **{f"du_kin_minus_arena_{ts}s": round(at(gk, ts) - at(ga, ts), 3) for ts in (0.5, 1.0, 1.5, 2.0, 3.0, 5.0)},
                      "arena_brake_at_go": go_a[4], "arena_stop_dist": round(ua - next(r[1] for r in l_a if r[0] >= T_UP - 1e-9), 3),
                      "kin_stop_dist": round(uk - next(r[1] for r in l_k if r[0] >= T_UP - 1e-9), 3)}
            print(h, res[h], flush=True)
            logs_out[str(h)] = {"arena": l_a, "kin": l_k, "t_go_arena": s_a.t_go, "t_go_kin": s_k.t_go}
        os.makedirs(a.out, exist_ok=True)
        json.dump({"coast": bool(AR.LEGACY_COAST), "lags": G.car_lags(), "summary": res, "logs": logs_out},
                  open(os.path.join(a.out, "release.json"), "w"))
    finally:
        ar.close()


if __name__ == "__main__":
    main()
