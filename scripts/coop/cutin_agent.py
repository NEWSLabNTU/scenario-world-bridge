#!/usr/bin/env python3
"""Roadmap 074 C2b: the cut-in's light policies as swb agents, so the arena's runs can be
checked through swb itself (SSv2, the bridge, the world server, the relay).

One process per car, on carla_basic's pattern (swb_agent.adapters.carla_basic): it finds its
vehicle and the other car by CARLA role_name (the entity names), never ticks the world, never
spawns or moves an actor, applies control to its own vehicle only on world ticks, and exits
(status 3) when the server stops answering. The driving is arena.py's: the warm-up to the
context, the parameterized rule (A: the lane-change gap check; B: the yield rule), the
perception model (10 Hz, delay, noise) and the controls (pedal maps, lags, steering). Lateral
targets follow CARLA's lane waypoints, so the road may curve.

The relay passes no scenario parameters to agents, so the run's context and rules come in the
agent's config (JSON), written per run by the C2b driver:

    {"entity": "coop_a", "other": "coop_b", "side": "A", "d0": 6, "dv": 0, "vb": 8,
     "warmup": 8, "rule": {...}, "seed": 0, "log": "<dir>/coop_a.jsonl",
     "carla_port": 2400}

    python3 scripts/coop/cutin_agent.py --relay tcp://localhost:5560 --entity coop_a --config <json>
"""

import json
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import swb_agent  # noqa: E402
from swb_agent import protocol as P  # noqa: E402
from swb_agent.adapters import carla_basic  # noqa: E402
from swb_agent.adapters.carla_basic import CarlaBasicAgent  # noqa: E402


class LightWorld(carla_basic.CarlaWorld):
    """carla_basic's world without BasicAgent and its route planner (shapely, networkx): the
    cut-in agent plans nothing, it follows lane waypoints."""

    def __init__(self, config, log):
        import os
        api = config.get("carla_pythonapi") or ""
        if api and api not in sys.path:
            sys.path.insert(0, api)
        import carla
        self.config, self.log, self.carla = config, log, carla
        self.client = carla.Client(config["carla_host"], int(config["carla_port"]))
        self.client.set_timeout(10.0)
        self.world = self.world_id = self.map = self.grp = self.tick_id = None
        self.vehicle = self.agent = self.on_tick = None
        self.last_tick = time.monotonic()
        del os

    def _attach_world(self):
        world = self.client.get_world()
        if world.id == self.world_id:
            return
        if self.tick_id is not None and self.world is not None:
            try:
                self.world.remove_on_tick(self.tick_id)
            except RuntimeError:
                pass
        self.world, self.world_id = world, world.id
        self.map = world.get_map()
        self.tick_id = world.on_tick(self._tick)
        self.log.info(f"world {world.id}: {self.map.name}")

import arena as AR  # noqa: E402
import cutin_game as C1  # noqa: E402


def _yaw_off(wp, ref_yaw):
    """Degrees between a waypoint's heading and `ref_yaw`."""
    return abs((wp.transform.rotation.yaw - ref_yaw + 180.0) % 360.0 - 180.0)


def _branch_off(wp, ref_yaw, probe=6.0):
    """How far a branch turns from `ref_yaw`, judged `probe` m along it: at a junction the
    straight and the turning branches start at the same point with the same heading."""
    nxt = wp.next(probe)
    return _yaw_off(nxt[0] if nxt else wp, ref_yaw)


class CutInAgent(CarlaBasicAgent):
    name = "coop_cutin"

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("role_name", config.get("entity", "coop_a"))
        carla_basic.CarlaWorld = LightWorld  # this process runs only this agent
        super().__init__(config)
        c = self.config
        self.side = c["side"]
        self.other_role = c["other"]
        self.ctx = {k: float(c[k]) for k in ("d0", "dv", "vb")}
        self.warmup = float(c.get("warmup", 8.0))
        self.rule = c["rule"]
        rng = random.Random(int(c.get("seed", 0)) * 2 + (0 if self.side == "A" else 1))
        self.react = 1.0 + rng.uniform(-0.2, 0.2)
        self.obs = AR.Observer(float(c.get("obs_delay", 0.19)), float(c.get("obs_sigma_x", 0.3)),
                               float(c.get("obs_sigma_v", 0.2)), rng)
        self.logf = open(c["log"], "w") if c.get("log") else None
        self.other = None
        self.lon = AR.Longitudinal()
        self.lag = None
        self.steer = None
        self.t0 = None
        self.stage = "idle"

    # -- geometry --------------------------------------------------------------------------
    def _truth(self, actor):
        tf = actor.get_transform()
        v = actor.get_velocity()
        yaw = math.radians(tf.rotation.yaw)
        return tf.location, yaw, math.hypot(v.x, v.y)

    def _find_other(self):
        for a in self.world.world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name") == self.other_role:
                return a
        return None

    def _lane_wp(self, loc, like):
        """The waypoint at `loc` on the lane of waypoint `like` (or its neighbour)."""
        wp = self.world.map.get_waypoint(loc)
        if (wp.road_id, wp.lane_id) == (like.road_id, like.lane_id):
            return wp
        for n in (wp.get_left_lane(), wp.get_right_lane()):
            if n is not None and (n.road_id, n.lane_id) == (like.road_id, like.lane_id):
                return n
        # another road segment: the lane of wp nearest to `like`'s lateral position
        cands = [w for w in (wp, wp.get_left_lane(), wp.get_right_lane()) if w is not None]
        return min(cands, key=lambda w: w.transform.location.distance(like.transform.location))

    @staticmethod
    def _ahead(wp, dist, ref_yaw, step=2.0):
        """The waypoint `dist` m on along wp's lane, going straight through junctions:
        Waypoint.next returns one waypoint per branch, and the first may be a turn (on
        Town03's west straight it was a left turn, and A left the road; 074 C3)."""
        left = dist
        while left > 1e-6:
            cands = wp.next(min(step, left))
            if not cands:
                return None
            wp = min(cands, key=lambda w: _branch_off(w, ref_yaw))
            left -= step
        return wp

    @staticmethod
    def _advance(wp, loc, ref_yaw):
        """Follow a lane from waypoint `wp` to the point level with `loc`, by lane continuity
        (next/previous), never by a fresh nearest-lane lookup that may pick a neighbour on the
        next road segment."""
        for _ in range(3):
            f = wp.transform.get_forward_vector()
            d = (loc.x - wp.transform.location.x) * f.x + (loc.y - wp.transform.location.y) * f.y
            if abs(d) < 0.05:
                break
            cands = wp.next(d) if d > 0 else wp.previous(-d)
            if not cands:
                break
            # through a junction: the branch that goes straight on, never a turn (the nearest
            # branch to a car changing lanes there was a left turn, and A followed it; 074 C3)
            wp = min(cands, key=lambda w: (round(_branch_off(w, ref_yaw)), w.transform.location.distance(loc)))
        return wp

    # -- hooks -----------------------------------------------------------------------------
    def on_goal(self, goal, waypoints):
        with self._ctl:
            self.other = self._find_other()
            if self.other is None:
                raise swb_agent.Failed(f"no vehicle with role_name {self.other_role!r}")
            v = self.world.vehicle
            pc = v.get_physics_control()
            wb = abs(pc.wheels[0].position.x - pc.wheels[2].position.x) / 100.0
            tr = abs(pc.wheels[0].position.y - pc.wheels[1].position.y) / 100.0
            self.wb = wb if wb > 1.0 else 3.0
            self.steer = AR.Steering(math.radians(pc.wheels[0].max_steer_angle), tr / self.wb if self.wb else 0.5)
            doc = AR.carla_physics.load(str(AR.PHYSICS))
            self.lag = AR.PedalLag(doc.get("command_lags", {}))
            self.L = 2 * v.bounding_box.extent.x
            self.Lo = 2 * self.other.bounding_box.extent.x
            self.own_lane = self.world.map.get_waypoint(v.get_location())
            self.b_lane = self.world.map.get_waypoint(
                (v if self.side == "B" else self.other).get_location())
            # tracked waypoints at my position: my lane, and B's lane beside me
            self.trk_own = self.own_lane
            # the road's heading here: through a junction the lanes and the aim follow the branch
            # nearest to it, so the cars go straight on (both branches start tangent; 074 C3)
            self.ref_yaw = self.own_lane.transform.rotation.yaw
            if self.side == "A":
                nb = [w for w in (self.own_lane.get_left_lane(), self.own_lane.get_right_lane())
                      if w is not None and (w.road_id, w.lane_id) == (self.b_lane.road_id, self.b_lane.lane_id)]
                self.trk_b = nb[0] if nb else self._lane_wp(v.get_location(), self.b_lane)
            else:
                self.trk_b = self.own_lane
            self.t0 = self.world.world.get_snapshot().timestamp.elapsed_seconds
            self.stage = "warmup"
            self.decide = None
            self.dropping = False
            self.latched = False
            self.lat_seen = self.in_lane_at = self.brake_seen = None
            self.mode = "drive"
        self.report(P.DRIVING, detail="")
        lane = lambda w: (w.road_id, w.lane_id, round(w.transform.location.x, 1), round(w.transform.location.y, 1))
        self.log.info(f"{self.side}: context {self.ctx}, rule {self.rule}; own lane {lane(self.own_lane)}, "
                      f"B's lane {lane(self.b_lane)}, tracked B lane {lane(self.trk_b)}")
        self.log_event("lanes", own=lane(self.own_lane), b=lane(self.b_lane), trk_b=lane(self.trk_b))

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
        snap_t = self.world.world.get_snapshot().timestamp.elapsed_seconds
        t = snap_t - self.t0
        me, other = self.world.vehicle, self.other
        loc, yaw, v = self._truth(me)
        oloc, oyaw, ov = self._truth(other)
        hx, hy = math.cos(yaw), math.sin(yaw)
        along = (oloc.x - loc.x) * hx + (oloc.y - loc.y) * hy  # the other's centre ahead of mine
        # lanes at my position, tracked by lane continuity
        self.trk_own = self._advance(self.trk_own, loc, self.ref_yaw)
        self.trk_b = self._advance(self.trk_b, loc, self.ref_yaw) if self.side == "A" else self.trk_own
        my_own, my_b = self.trk_own, self.trk_b
        width = max(1.0, my_own.transform.location.distance(my_b.transform.location))
        lat = 0.0
        if self.side == "A":
            # lateral progress: my offset from my lane toward B's, along the line between them
            ox, oy = my_own.transform.location.x, my_own.transform.location.y
            ux, uy = (my_b.transform.location.x - ox) / width, (my_b.transform.location.y - oy) / width
            lat = max(0.0, min(1.0, ((loc.x - ox) * ux + (loc.y - oy) * uy) / width))
        look = max(6.0, 0.7 * v)
        if self.stage == "warmup":
            accel, s_lat = self._warmup(t, along, v, ov)
            if t >= self.warmup:
                self.stage = "run"
                self.tr0 = t
                self.v0 = v
                self.log_event("start", t=t, along=along, v=v, ov=ov)
        else:
            accel, s_lat = self._run(t - self.tr0, along, v, ov, lat)
        # steering: pure pursuit to a point between my lane and B's lane, `look` ahead
        na, nb = self._ahead(my_own, look, self.ref_yaw), self._ahead(my_b, look, self.ref_yaw)
        pa = (na or my_own).transform.location  # the road's end: aim at the lane here
        pb = (nb or my_b).transform.location
        tx, ty = pa.x + (pb.x - pa.x) * s_lat, pa.y + (pb.y - pa.y) * s_lat
        alpha = math.atan2(ty - loc.y, tx - loc.x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        angle = math.atan2(2 * self.wb * math.sin(alpha), look)
        th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, snap_t)
        import carla
        me.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=self.steer.command(angle)))
        if self.logf and int(t * 20) % 2 == 0:
            self.logf.write(json.dumps({"t": round(t, 3), "phase": self.stage, "along": round(along, 2),
                                        "x": round(loc.x, 2), "y": round(loc.y, 2), "ox": round(oloc.x, 2), "oy": round(oloc.y, 2),
                                        "yaw": round(yaw, 4), "oyaw": round(oyaw, 4),
                                        "v": round(v, 2), "ov": round(ov, 2), "lat": round(lat, 3),
                                        "accel": round(accel, 2), "th": round(th, 3), "br": round(br, 3), "decide": self.decide,
                                        "dropping": self.dropping, "latched": self.latched}) + "\n")

    def log_event(self, what, **kw):
        if self.logf:
            self.logf.write(json.dumps({"event": what, **{k: round(v, 3) if isinstance(v, float) else v
                                                          for k, v in kw.items()}}) + "\n")
            self.logf.flush()

    def _warmup(self, t, along, v, ov):
        d0, dv, vb = self.ctx["d0"], self.ctx["dv"], self.ctx["vb"]
        if self.side == "B":
            return max(-3.0, min(3.0, 1.5 * (vb - v))), 0.0
        # A: rear of A ahead of B's front by d0 - dv (warmup - t), relative speed dv
        r = along * -1 - (self.L + self.Lo) / 2  # `along` is B ahead of A; r = A's rear - B's front
        r_ref = d0 - dv * max(0.0, self.warmup - t)
        ab = max(-3.0, min(3.0, 1.5 * (vb - ov)))
        a = ab + 0.6 * (r_ref - r) + 1.8 * (dv - (v - ov))
        return max(-4.0, min(3.0, a)), 0.0

    def _run(self, t, along, v, ov, lat):
        other_lat = self._other_lat(along) if self.side == "B" else 0.0
        seen = self.obs.push(t, along, other_lat, ov)
        if seen is None:
            seen = (t, along, other_lat, ov, 0.0)
        _, along_s, lat_s, ov_s, oa_s = seen
        cap = C1.TYPES["car"]
        if self.side == "A":
            return self._run_a(t, along_s, v, ov_s, oa_s, lat, cap)
        return self._run_b(t, along_s, v, ov_s, cap, lat_s)

    def _other_lat(self, along):
        """B: A's lateral progress into my lane (truth; B sees it through its observer)."""
        oloc = self.other.get_location()
        wp = self.trk_own
        cands = wp.next(along) if along > 0.05 else (wp.previous(-along) if along < -0.05 else [wp])
        if cands:
            wp = min(cands, key=lambda w: w.transform.location.distance(oloc))
        off = oloc.distance(wp.transform.location)
        return max(0.0, min(1.0, 1.0 - off / max(1.0, wp.lane_width)))

    def _run_a(self, t, along_s, v, vb_s, ab_s, lat, cap):
        pa = self.rule
        gap_ahead = -along_s - (self.L + self.Lo) / 2  # A's rear ahead of B's front, as seen
        gap_behind = along_s - (self.L + self.Lo) / 2  # B's rear ahead of A's front
        if self.decide is None and not self.dropping:
            need = pa["buffer"] + vb_s * pa["tau"] + max(0.0, vb_s ** 2 - v ** 2) / (2 * pa["a_rear"])
            if ab_s <= -C1.YIELD_SEEN and self.brake_seen is None:
                self.brake_seen = t
            if gap_ahead >= need:
                self.decide = t
            elif pa["use_yield"] and self.brake_seen is not None and t >= self.brake_seen + self.react \
                    and gap_ahead > -self.L / 2:
                self.decide = t
            elif t >= pa["patience"] + (C1.WATCH if pa["use_yield"] else 0.0):
                self.dropping = True
            if self.decide is not None:
                self.log_event("decide", t=t, gap=gap_ahead, need=need)
        if self.dropping and self.decide is None:
            want = C1.S0 + vb_s * C1.HEADWAY
            if gap_behind >= want:
                self.decide = t
                self.log_event("decide_behind", t=t, gap=gap_behind)
            accel = -C1.A_WAIT_DECEL if gap_behind < want and v > 0.5 * vb_s else 0.0
        elif lat > 0.05 and gap_behind > -(self.L + self.Lo) / 2:
            accel = C1.follow_cruise(gap_behind, v, vb_s, cap, self.v0)
        else:
            accel = max(-1.0, min(cap.a_max, 1.5 * (self.v0 - v)))
        s_lat = 0.0
        if self.decide is not None and t >= self.decide + pa["t_prep"]:
            s = min(1.0, (t - self.decide - pa["t_prep"]) / cap.t_lc)
            s_lat = 0.5 * (1 - math.cos(math.pi * s))
        return accel, s_lat

    def _run_b(self, t, along_s, v, va_s, cap, lat_s):
        pb = self.rule
        # along_s, lat_s: A's centre ahead of mine and its progress into my lane, as seen
        # (delayed, 10 Hz), as arena.py's B sees them
        if lat_s > 0.08 and self.lat_seen is None:
            self.lat_seen = t
        a_ahead = along_s > -self.L
        gap_b = along_s - (self.L + self.Lo) / 2
        if lat_s >= C1.LANE_LINE and along_s > 0 and self.in_lane_at is None:
            self.in_lane_at = t
        pre = t >= self.react and a_ahead and lat_s < C1.LANE_LINE and gap_b < pb["pre_band"]
        react = self.lat_seen is not None and t >= self.lat_seen + self.react and a_ahead and gap_b < pb["react_gap"]
        self.latched = self.latched or ((pre or react) and a_ahead)
        if self.latched and a_ahead:
            accel = -pb["b_y"] if gap_b < C1.S0 + v * C1.HEADWAY else C1.follow_cruise(gap_b, v, va_s, cap, self.v0)
        else:
            accel = max(-1.0, min(cap.a_max, 1.5 * (self.v0 - v)))
        if self.in_lane_at is not None and t >= self.in_lane_at + self.react and along_s > 0:
            accel = min(accel, C1.follow_cruise(gap_b, v, va_s, cap, self.v0))
        return accel, 0.0

    def on_stop(self):
        super().on_stop()
        if self.logf:
            self.logf.flush()


def main(argv=None):
    return swb_agent.main(CutInAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
