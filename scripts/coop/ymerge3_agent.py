#!/usr/bin/env python3
"""Roadmap 074 N5's swb check: the three-car Y merge's controllers as swb agents, on Town03 j1901.

One process per car, on roundabout_agent.py's pattern (carla_basic without BasicAgent): it finds
its vehicle and the other two by CARLA role_name, never ticks the world, never spawns or moves an
actor, applies control to its own vehicle only, on world ticks. Each car warms up to v on its
context's reference trajectory (ymerge3_arena's ramp at 2 m/s^2 from where the scenario puts it,
slowed through Town03's kink on A's approach: ymerge3_town03.WarmupPlan), then drives by its
ymerge3.Controller on what it perceives of the other two through arena.Observer (10 Hz, 0.19 s, noise); pure pursuit along its lane (ymerge3_town03) at one
look-ahead (max(6, 0.7 v)), as the arena's; acb's controls (pedal map, lags, steering) through
arena.Longitudinal, so a held car brakes to rest as in the arena.

Each tick all three cars' true states go to the log (JSON lines: t, phase, and per car k in A, B,
C: u, v, x, y, yaw in CARLA's frame), for n5_swb.py to score with ymerge3.score3.

Config (JSON, written per run by n5_swb.py):
    {"entity": "coop_a", "role": "A", "roles": {"A": "coop_a", "B": "coop_b", "C": "coop_c"},
     "dt": 0, "v": 10, "h": 1.5, "warmup": 15, "a": "t1.5yp2", "seed": 0, "lanes": "<lanes.json>",
     "log": "<dir>/coop_a.jsonl", "carla_port": 2010}

    python3 scripts/coop/ymerge3_agent.py --relay tcp://localhost:5560 --entity coop_a --config <json>
"""

import json
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import scensim_agent  # noqa: E402
from scensim_agent import protocol as P  # noqa: E402
from scensim_agent.adapters import carla_basic  # noqa: E402
from scensim_agent.adapters.carla_basic import CarlaBasicAgent  # noqa: E402

import arena as AR  # noqa: E402
import cutin_agent  # noqa: E402
import ymerge3 as Y3  # noqa: E402
import ymerge3_town03 as T  # noqa: E402
import crossing_game as G  # noqa: E402
import cutin_game as C1  # noqa: E402
import ymerge_arena as YA  # noqa: E402
import ymerge_game as Y  # noqa: E402

CARS = "ABC"
PACE_A = 1.0  # m/s^2: the pace car's (Autoware's) assumed launch, for its predicted arrival


class YMerge3Agent(CarlaBasicAgent):
    name = "coop_ymerge3"

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("role_name", config.get("entity", "coop_a"))
        carla_basic.CarlaWorld = cutin_agent.LightWorld  # this process runs only this agent
        super().__init__(config)
        c = self.config
        self.role = c["role"]
        self.roles = c["roles"]
        self.v, self.warmup = float(c["v"]), float(c.get("warmup", YA.WARMUP))
        self.lanes = T.load(c["lanes"])
        self.target = Y3.start_u(float(c["dt"]), self.v, float(c["h"]))
        self.plan = {k: T.WarmupPlan(k, self.target[k], self.v, self.warmup) for k in CARS}
        self.u_spawn = {k: self.plan[k].u_spawn for k in CARS}
        self.ctl = Y3.Controller(self.role, c["a"], self.v, hold_stop=False)
        self.pace = c.get("pace")  # a car this process does not drive, whose progress sets the warm-up's clock
        self.park = bool(c.get("park"))  # held at its spawn throughout (a baseline without this car), still logging
        rng = random.Random(int(c.get("seed", 0)) * 3 + CARS.index(self.role))
        self.obs = {o: AR.Observer(float(c.get("obs_delay", 0.19)), float(c.get("obs_sigma_x", 0.3)),
                                   float(c.get("obs_sigma_v", 0.2)), rng) for o in CARS if o != self.role}
        self.last_stamp = {o: None for o in self.obs}
        self.logf = open(c["log"], "w") if c.get("log") else None
        self.actors = {}
        self.t0 = None
        self.stage = "idle"
        self.mode = "idle"

    def _truth(self, actor):
        tf = actor.get_transform()
        v = actor.get_velocity()
        return tf.location.x, tf.location.y, math.radians(tf.rotation.yaw), math.hypot(v.x, v.y)

    def _find(self, role_name):
        for a in self.world.world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name") == role_name:
                return a
        return None

    def on_goal(self, goal, waypoints):
        found, t_give_up = {}, time.monotonic() + float(self.config.get("find_timeout", 60))
        for k in CARS:  # the other cars (the Autoware ego among them) may spawn after this one
            while k not in found:
                a = self.world.vehicle if k == self.role else self._find(self.roles[k])
                if a is not None:
                    found[k] = a
                elif time.monotonic() > t_give_up:
                    raise scensim_agent.Failed(f"no vehicle with role_name {self.roles[k]!r}")
                else:
                    time.sleep(0.5)
        with self._ctl:
            self.actors.update(found)
            v = self.world.vehicle
            w = v.get_physics_control().wheels
            self.wb, track = AR.wheel_geometry(w)   # 0.9.16 and 1.0 alike
            self.steer = AR.Steering(math.radians(w[0].max_steer_angle), track / self.wb)
            doc = AR.carla_physics.load(str(AR.PHYSICS))
            self.lag = AR.PedalLag(doc.get("command_lags", {}))
            self.lon = AR.Longitudinal()
            off = AR.carla_physics.differences(v, doc)
            self.box = {k: (2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y) for k, a in self.actors.items()}
            self.t0 = self.world.world.get_snapshot().timestamp.elapsed_seconds
            self.stage = "warmup"
            self.mode = "drive"
            st = self._states()
        self.report(P.DRIVING, detail="")
        self.log_event("start", role=self.role, t0=self.t0, box=self.box, physics_off=bool(off),
                       u={k: st[k][0] for k in CARS}, u_spawn=self.u_spawn, friction=self._friction())
        self.log.info(f"{self.role}: dt {self.config['dt']} v {self.v} h {self.config['h']}, A's rule {self.config['a']}; "
                      f"u {', '.join(f'{k} {st[k][0]:.2f}' for k in CARS)} (spawn targets "
                      f"{', '.join(f'{k} {self.u_spawn[k]:.2f}' for k in CARS)}); physics {'OFF ' + str(off) if off else 'as the spec'}")

    def _states(self):
        out = {}
        for k in CARS:
            x, y, yaw, v = self._truth(self.actors[k])
            out[k] = (self.lanes[k].u_of(k, x, y, self.box[k][0] if hasattr(self, "box") else Y3.L), v, x, y, yaw)
        return out

    def _friction(self):
        return {k: round(a.get_physics_control().wheels[0].tire_friction, 2) for k, a in self.actors.items()}

    def _on_world_tick(self):
        with self._ctl:
            if self.mode == "drive" and self.t0 is not None:
                self._drive()
            elif self.mode == "hold":
                self.world.hold()
        now = time.monotonic()
        if now - self._last_pose >= 0.2:
            self._last_pose = now
            pose = self._pose()
            if pose is not None:
                self.report(pose=pose)

    def _drive(self):
        import carla

        snap_t = self.world.world.get_snapshot().timestamp.elapsed_seconds
        t = snap_t - self.t0
        st = self._states()
        u, v = st[self.role][0], st[self.role][1]
        x, y, yaw = st[self.role][2:]
        if self.stage == "warmup":
            if self.pace:
                accel, go = self._paced(t, st, u, v)
            else:
                ur, vr, ar_ = self.plan[self.role].at(t)
                accel = max(-4.0, min(3.0, ar_ + 0.6 * (ur - u) + 1.8 * (vr - v)))
                go = t >= self.warmup - 1e-9
            for o in self.obs:
                self.obs[o].push(t, st[o][0], 0.0, st[o][1])
            if go:
                self.stage = "run"
                self.tr0 = t
                self.log_event("run", t=t, u={k: st[k][0] for k in CARS}, v={k: st[k][1] for k in CARS},
                               friction=self._friction())
        if self.stage == "run":
            tr = t - self.tr0
            for o, ob in self.obs.items():
                seen = ob.push(t, st[o][0], 0.0, st[o][1])  # the observers' clock is this agent's t
                if seen is not None:
                    seen = (seen[0] - self.tr0, *seen[1:])
                if seen is not None and seen[0] >= -1e-9 and seen[0] != self.last_stamp[o]:
                    self.last_stamp[o] = seen[0]
                    self.ctl.observe(seen[0], o, seen[1], seen[3], seen[4])
            accel = self.ctl.step(tr, u, v)
            self.ctl.after(tr, u, v)
        if self.park:
            accel = -3.0
        lane = self.lanes[self.role]
        look = max(6.0, 0.7 * v)
        s = lane.project(("pp", self.role), x, y)
        tx, ty, _, _ = lane.at(s + look)
        th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, snap_t)
        alpha = math.atan2(ty - y, tx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        angle = math.atan2(2 * self.wb * math.sin(alpha), look)
        self.world.vehicle.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=self.steer.command(angle)))
        if self.logf:
            row = {"t": round(t, 3), "phase": self.stage, "accel": round(accel, 3), "th": round(th, 3), "br": round(br, 3)}
            for k in CARS:
                uk, vk, xk, yk, yk_ = st[k]
                row[k] = [round(uk, 3), round(vk, 3), round(xk, 3), round(yk, 3), round(yk_, 5)]
            if self.stage == "run":
                row["rec"] = self.ctl.record()
            elif self.pace:
                row["tau"] = round(getattr(self, "last_tau", -99.0), 3)
            self.logf.write(json.dumps(row) + "\n")

    def _paced(self, t, st, u, v):
        """The warm-up when the pace car (config "pace", e.g. B as the Autoware ego) is not this
        agent's: the run starts when the pace car's u reaches its context's; until then this car's
        plan time is warmup minus the pace car's predicted time to get there (from its speed, at
        PACE_A up to v), so it arrives at its own start u with v then. A follower (C) also keeps
        cutin_game's cruise gap to the pace car."""
        k = self.pace
        ub, vb = st[k][0], st[k][1]
        rem = self.target[k] - ub
        if rem <= 0.0:
            return G.drive(v, self.v), True
        t_up = max(0.0, (self.v - vb) / PACE_A)
        d_up = vb * t_up + 0.5 * PACE_A * t_up * t_up
        t_left = (math.sqrt(vb * vb + 2 * PACE_A * rem) - vb) / PACE_A if rem < d_up else t_up + (rem - d_up) / self.v
        tau = self.warmup - t_left
        self.last_tau = tau
        if tau < 0.0 and u - self.u_spawn[self.role] < 0.5 and v < 0.3:
            return -3.0, False  # not yet: held where the scenario put it
        ur, vr, ar_ = self.plan[self.role].at(max(tau, 0.0))
        if self.role == "A":
            # dt is against B's arrival at Z, and Autoware reaches the run below v and keeps
            # accelerating (T.B_LAUNCH): A's start moves to dt + B's predicted time to Z from its
            # predicted speed at the run's start, phased in over the plan
            vb_arr = min(self.v, math.sqrt(vb * vb + 2 * PACE_A * rem))
            u_tgt = -self.v * (T.t_reach(-self.target[k], vb_arr, self.v) + float(self.config["dt"]))
            ur += (u_tgt - self.target["A"]) * min(max(tau / self.warmup, 0.0), 1.0)
        accel = max(-4.0, min(3.0, ar_ + 0.6 * (ur - u) + 1.8 * (vr - v)))
        if self.role == "C" and k == "B":
            accel = min(accel, C1.follow_cruise(ub - u - Y3.L, v, vb, Y.CAP, self.v))
        return accel, False

    def log_event(self, what, **kw):
        if self.logf:
            self.logf.write(json.dumps({"event": what, **kw}) + "\n")
            self.logf.flush()

    def on_stop(self):
        super().on_stop()
        if self.logf:
            self.logf.flush()


def main(argv=None):
    return scensim_agent.main(YMerge3Agent, argv)


if __name__ == "__main__":
    sys.exit(main())
