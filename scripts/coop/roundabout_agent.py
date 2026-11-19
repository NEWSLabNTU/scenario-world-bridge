#!/usr/bin/env python3
"""Roadmap 074 U2 swb: the roundabout entry's light policies as swb agents, on Town03's roundabout.

One process per car, on cutin_agent.py's pattern (carla_basic without BasicAgent): it finds its
vehicle and the other car by CARLA role_name, never ticks the world, never spawns or moves an actor,
applies control to its own vehicle only, on world ticks. The driving is roundabout_arena.py's on
Town03's lanes (roundabout_town03): A, entering, holds at its yield line through the warm-up, then
plays its rule; B, circulating on the outer lane, warms up to v and its context (its front tb s from
the conflict stretch at v when the run starts), then plays its rule. Both are stepped by
roundabout_game.RBRules, the object the kinematic model and the arena step, on what each sees of the
other through arena.Observer (10 Hz, 0.19 s, noise), held OBS + DECIDE s (B's safety layer REACT s)
and predicted to now; pure pursuit at the lane's point one look-ahead (max(3, 0.4 v)) on; acb's
controls (pedal map, lags, steering).

B in its own process does not know when A decided to go; its courtesy waits for A to have joined,
which is what RBRules' condition comes to whenever A goes before it has joined (it always does).

Each tick both cars' true states go to the log (JSON lines: t, phase, x, y, yaw, v and the other's,
CARLA's frame; s, the other's s; kappa, okappa: each car's lane curvature in CARLA's frame, from the
lanes' waypoints smoothed over 5 m as route.lane_kappa does), for u2_swb.py to score.

Config (JSON, written per run by u2_swb.py):
    {"entity": "coop_a", "other": "coop_b", "side": "A", "tb": 3, "v": 7, "warmup": 10,
     "a": "g2.5", "b": "hold", "seed": 0, "lanes": "<geom.json>", "log": "<dir>/coop_a.jsonl",
     "carla_port": 2600}

    python3 scripts/coop/roundabout_agent.py --relay tcp://localhost:5560 --entity coop_a --config <json>
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
import crossing_arena as XA  # noqa: E402
import cutin_agent  # noqa: E402
import roundabout_arena as RA  # noqa: E402
import roundabout_game as G  # noqa: E402
import roundabout_town03 as T  # noqa: E402


def b_start(tb, v, warmup):
    """B's front's ring position at the warm-up's start (it ends tb s from the stretch at v)."""
    return G.SZ0 - v * tb - XA.ramp(warmup, v)[0]


class RoundaboutAgent(CarlaBasicAgent):
    name = "coop_roundabout"

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("role_name", config.get("entity", "coop_a"))
        carla_basic.CarlaWorld = cutin_agent.LightWorld  # this process runs only this agent
        super().__init__(config)
        c = self.config
        self.side = c["side"]
        self.other_role = c["other"]
        self.tb, self.v, self.warmup = float(c["tb"]), float(c["v"]), float(c.get("warmup", 10.0))
        self.apath, self.rpoly, self.fr = T.install(T.load(c["lanes"]))
        ra = next(r for r in G.A_RULES if r[0] == c["a"])
        self.R = G.RBRules(ra, c["b"], 1, self.v)
        if self.side == "B":
            self.R.started = 0.0  # B cannot know A's decision; see the module's doc
        rng = random.Random(int(c.get("seed", 0)) * 2 + (0 if self.side == "A" else 1))
        mk = lambda: AR.Observer(float(c.get("obs_delay", 0.19)), float(c.get("obs_sigma_x", 0.3)),  # noqa: E731
                                 float(c.get("obs_sigma_v", 0.2)), rng)
        self.seen = RA.Seen(mk())  # what this car sees of the other
        self.logf = open(c["log"], "w") if c.get("log") else None
        self.other = None
        self.t0 = None
        self.stage = "idle"

    # -- geometry --------------------------------------------------------------------------
    def _truth(self, actor):
        tf = actor.get_transform()
        v = actor.get_velocity()
        return tf.location.x, tf.location.y, math.radians(tf.rotation.yaw), math.hypot(v.x, v.y)

    def _find_other(self):
        for a in self.world.world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name") == self.other_role:
                return a
        return None

    def _a_s(self, s_prev, xc, yc):
        return self.apath.project(s_prev, *self.fr.to_m(xc, yc))

    def _ring_s(self, s_prev, xc, yc):
        """A ring car's front, unwrapped near its last (roundabout_arena.Ring.ring_s)."""
        x, y = self.fr.to_m(xc, yc)
        sc = G.R_RING * math.atan2(y, x)
        circ = 2 * math.pi * G.R_RING
        sc += circ * round((s_prev - G.L / 2 - sc) / circ)
        return sc + G.L / 2

    # -- hooks -----------------------------------------------------------------------------
    def on_goal(self, goal, waypoints):
        with self._ctl:
            self.other = self._find_other()
            if self.other is None:
                raise scensim_agent.Failed(f"no vehicle with role_name {self.other_role!r}")
            v = self.world.vehicle
            pc = v.get_physics_control()
            w = pc.wheels
            self.wb, track = AR.wheel_geometry(w)   # 0.9.16 and 1.0 alike
            self.steer = AR.Steering(math.radians(w[0].max_steer_angle), track / self.wb)
            doc = AR.carla_physics.load(str(AR.PHYSICS))
            self.lag = AR.PedalLag(doc.get("command_lags", {}))
            self.lon = AR.Longitudinal()
            off = AR.carla_physics.differences(v, doc)
            bb, obb = v.bounding_box.extent, self.other.bounding_box.extent
            xa, ya, _, _ = self._truth(v if self.side == "A" else self.other)
            xb, yb, _, _ = self._truth(self.other if self.side == "A" else v)
            self.s = self._a_s(G.S_YIELD, xa, ya)
            self.sq = self._ring_s(b_start(self.tb, self.v, self.warmup), xb, yb)
            self.sq_start = G.SZ0 - self.v * self.tb
            self.t0 = self.world.world.get_snapshot().timestamp.elapsed_seconds
            self.stage = "warmup"
            self.mode = "drive"
        self.report(P.DRIVING, detail="")
        self.log_event("start", side=self.side, t0=self.t0, L=2 * bb.x, W=2 * bb.y, oL=2 * obb.x, oW=2 * obb.y,
                       s=self.s, sq=self.sq, physics_off=bool(off), yield_s=G.S_YIELD, sz0=G.SZ0, sz1=G.SZ1,
                       a_in=G.A_IN, a_joined=G.A_JOINED, r_ring=G.R_RING, **self._friction())
        self.log.info(f"{self.side}: tb {self.tb} v {self.v}, rules {self.config['a']}/{self.config['b']}; A at s {self.s:.2f} "
                      f"(yield line {G.S_YIELD:.2f}), B at {self.sq:.2f}; physics {'OFF ' + str(off) if off else 'as the spec'}")

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

    # -- the drive -------------------------------------------------------------------------
    def _drive(self):
        import carla

        snap_t = self.world.world.get_snapshot().timestamp.elapsed_seconds
        t = snap_t - self.t0
        me, other = self.world.vehicle, self.other
        x, y, yaw, v = self._truth(me)
        ox, oy, oyaw, ov = self._truth(other)
        if self.side == "A":
            self.s = self._a_s(self.s, x, y)
            self.sq = self._ring_s(self.sq, ox, oy)
            va, vq = v, ov
        else:
            self.sq = self._ring_s(self.sq, x, y)
            self.s = self._a_s(self.s, ox, oy)
            va, vq = ov, v
        hand = False
        if self.stage == "warmup":
            if self.side == "A":
                # held outside the pedal lag, as roundabout_arena holds A: through it, the brake's release lag
                # delayed every start by 0.4 s
                me.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True, steer=0.0))
                if t >= self.warmup - 1e-9:
                    self._start_run(t, va, vq)
                return
            else:
                dr, vr, ar_ = XA.ramp(t, self.v)
                dist = XA.ramp(self.warmup, self.v)[0] - dr
                err = (self.sq_start - dist) - self.sq
                accel = max(-4.0, min(3.0, ar_ + 0.6 * err + 1.8 * (vr - v)))
            if t >= self.warmup - 1e-9:
                self._start_run(t, va, vq)
        if self.stage == "run":
            tr = t - self.tr0
            if self.side == "A":
                self.seen.push(tr, self.sq, vq)
                sq_seen = self.seen.at(tr, G.OBS + G.DECIDE, (self.sq, vq))
                accel = self.R.a_step(tr, self.s, va, [sq_seen])
                self.R.after(tr, va)
                hand = accel <= 0 and v < 0.05
            else:
                self.seen.push(tr, self.s, va)
                accel = self.R.q_step(tr, [self.sq], [vq], self.seen.at(tr, G.REACT, (self.s, va)),
                                      self.seen.at(tr, G.OBS + G.DECIDE, (self.s, va)))[0]
        # pure pursuit along my lane, one look-ahead on
        look = max(RA.LOOK_MIN, RA.LOOK_GAIN * v)
        tx, ty = (G.a_path(self.s + look) if self.side == "A" else G.ring(self.sq - G.L / 2 + look))[:2]
        txc, tyc = self.fr.to_c(tx, ty)
        th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, snap_t)
        alpha = math.atan2(tyc - y, txc - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        dist = max(math.hypot(txc - x, tyc - y), 0.5)
        angle = math.atan2(2 * self.wb * math.sin(alpha), dist)
        me.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=self.steer.command(angle), hand_brake=hand))
        if self.logf:
            # curvature in CARLA's frame: the model frame is a reflection of it, every curvature changes sign
            ka, kq = -G.a_path(self.s)[3], -G.ring(self.sq - G.L / 2)[3]
            kap, okap = (ka, kq) if self.side == "A" else (kq, ka)
            row = {"t": round(t, 3), "phase": self.stage, "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 5),
                   "v": round(v, 3), "ox": round(ox, 3), "oy": round(oy, 3), "oyaw": round(oyaw, 5), "ov": round(ov, 3),
                   "s": round(self.s if self.side == "A" else self.sq, 3), "os": round(self.sq if self.side == "A" else self.s, 3),
                   "kappa": round(kap, 5), "okappa": round(okap, 5), "accel": round(accel, 3), "th": round(th, 3),
                   "br": round(br, 3)}
            if self.side == "A":
                row["started"] = self.R.started
            else:
                row["shield"] = int(self.R.shield[0])
            self.logf.write(json.dumps(row) + "\n")

    def _friction(self):
        return {"friction": [round(w.tire_friction, 2) for w in self.world.vehicle.get_physics_control().wheels],
                "ofriction": [round(w.tire_friction, 2) for w in self.other.get_physics_control().wheels]}

    def _start_run(self, t, va, vq):
        self.stage = "run"
        self.tr0 = t
        self.log_event("run", t=t, s=self.s, sq=self.sq, va=va, vq=vq, tb_real=(G.SZ0 - self.sq) / max(vq, 0.1),
                       **self._friction())

    def log_event(self, what, **kw):
        if self.logf:
            self.logf.write(json.dumps({"event": what, **{k: round(v, 3) if isinstance(v, float) else v
                                                          for k, v in kw.items()}}) + "\n")
            self.logf.flush()

    def on_stop(self):
        super().on_stop()
        if self.logf:
            self.logf.flush()


def main(argv=None):
    return scensim_agent.main(RoundaboutAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
