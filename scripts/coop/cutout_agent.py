#!/usr/bin/env python3
"""Roadmap 074 S1 on the real stack: the cut-out's lead L as an swb agent.

Three cars on Town03's west straight, all in the left lane: B (the Autoware ego, role "hero")
follows L (this agent); ahead of L stands O (an SSv2 vehicle with speed 0). L changes lanes to
the right and so reveals O to B. The rules are cutout_game's:

  warm-up  L starts h0 x v ahead of B (rear to front) and leads at v, faster only to keep B's
           headway from falling under h0; a longer headway is Autoware's own following gap and
           stays (the achieved h0 is logged)
  run      t = 0 when L's time to O is T0_LO (8 s). L holds v and starts its lane change (cosine,
           t_lc s) when its time to O reaches T_L; while its body is in B's lane it brakes for O
           only when a stop needs 3 m/s^2 or more (cutout_game.l_accel); after the lane change it
           keeps v (the left lane "same")

The driving (pedal maps, lags, steering, lane tracking through the junction) is cutin_agent's.
Logs every second tick (20 Hz world, so 10 Hz) the three cars' truth in CARLA's frame:

    python3 scripts/coop/cutout_agent.py --relay tcp://localhost:5560 --entity coop_l --config <json>

Config: {"entity": "coop_l", "other": "hero", "obstacle": "coop_o", "v": 6, "h0": 1.5, "t_l": 6,
         "t_lc": 3, "warmup_min": 8, "log": "<dir>/coop_l.jsonl", "carla_port": 2010}
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import scensim_agent  # noqa: E402
from scensim_agent import protocol as P  # noqa: E402
import arena as AR  # noqa: E402
import cutin_agent as CA  # noqa: E402
import cutout_game as CG  # noqa: E402


class CutOutAgent(CA.CutInAgent):
    name = "coop_cutout"

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("side", "L")
        for k in ("d0", "dv", "vb"):
            config.setdefault(k, 0.0)
        config.setdefault("rule", {})
        super().__init__(config)
        c = self.config
        self.v_ref = float(c["v"])
        self.h0 = float(c["h0"])
        self.t_l = float(c["t_l"])
        self.t_lc = float(c.get("t_lc", 3.0))
        self.warmup_min = float(c.get("warmup_min", 8.0))
        self.obstacle_role = c.get("obstacle", "coop_o")
        self.obst = None

    def _find_role(self, role):
        for a in self.world.world.get_actors().filter("vehicle.*"):
            if a.attributes.get("role_name") == role:
                return a
        return None

    def on_goal(self, goal, waypoints):
        with self._ctl:
            self.other = self._find_role(self.other_role)
            if self.other is None:
                raise scensim_agent.Failed(f"no vehicle with role_name {self.other_role!r}")
            me = self.world.vehicle
            self.obst = self._find_role(self.obstacle_role)
            if self.obst is None:  # the one vehicle that is neither me nor B
                rest = [a for a in self.world.world.get_actors().filter("vehicle.*")
                        if a.id not in (me.id, self.other.id)]
                self.obst = rest[0] if len(rest) == 1 else None
            if self.obst is None:
                raise scensim_agent.Failed(f"no obstacle vehicle {self.obstacle_role!r}")
            pc = me.get_physics_control()
            (fx, fy), (rx, ry), (lx, ly) = (AR.wheel_xy(w) for w in pc.wheels[:3])
            wb, tr = abs(fx - lx), abs(fy - ry)
            self.wb = wb if wb > 1.0 else 3.0
            self.steer = AR.Steering(math.radians(pc.wheels[0].max_steer_angle), tr / self.wb if self.wb else 0.5)
            doc = AR.carla_physics.load(str(AR.PHYSICS))
            self.lag = AR.PedalLag(doc.get("command_lags", {}))
            self.ext = {k: (2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y)
                        for k, a in (("L", me), ("B", self.other), ("O", self.obst))}
            self.own_lane = self.world.map.get_waypoint(me.get_location())
            self.ref_yaw = self.own_lane.transform.rotation.yaw
            # the lane L leaves to: the neighbour with the same direction (on the west straight, the right)
            nb = [w for w in (self.own_lane.get_right_lane(), self.own_lane.get_left_lane())
                  if w is not None and w.lane_type == self.world.carla.LaneType.Driving
                  and w.lane_id * self.own_lane.lane_id > 0]
            if not nb:
                raise scensim_agent.Failed("no neighbouring lane in L's direction")
            self.trk_own, self.trk_tgt = self.own_lane, nb[0]
            self.t0 = self.world.world.get_snapshot().timestamp.elapsed_seconds
            self.stage = "warmup"
            self.cruise = False  # B has reached v once: L no longer drives slower than v
            self.t_run0 = None  # elapsed seconds at the run's t = 0
            self.t_start = None  # the lane change's start (run time)
            self.mode = "drive"
        self.report(P.DRIVING, detail="")
        lane = lambda w: (w.road_id, w.lane_id, round(w.transform.location.x, 1), round(w.transform.location.y, 1))
        self.log.info(f"L: v {self.v_ref}, h0 {self.h0}, T_L {self.t_l}, t_lc {self.t_lc}; own lane "
                      f"{lane(self.own_lane)}, target {lane(self.trk_tgt)}, O {self.obst.id}")
        self.log_event("setup", own=lane(self.own_lane), tgt=lane(self.trk_tgt), ext=self.ext,
                       v=self.v_ref, h0=self.h0, t_l=self.t_l, t_lc=self.t_lc,
                       o=[round(self.obst.get_location().x, 2), round(self.obst.get_location().y, 2)])

    def _pose_of(self, actor):
        tf, v = actor.get_transform(), actor.get_velocity()
        return [round(tf.location.x, 3), round(tf.location.y, 3), round(math.radians(tf.rotation.yaw), 4),
                round(v.x, 3), round(v.y, 3)]

    def _drive(self):
        import carla
        el = self.world.world.get_snapshot().timestamp.elapsed_seconds
        t = el - self.t0
        me = self.world.vehicle
        loc, yaw, v = self._truth(me)
        bloc, _, vb = self._truth(self.other)
        oloc, _, _ = self._truth(self.obst)
        hx, hy = math.cos(math.radians(self.ref_yaw)), math.sin(math.radians(self.ref_yaw))
        fwd = lambda p: p.x * hx + p.y * hy  # along the road
        ll, lb = self.ext["L"][0], self.ext["B"][0]
        lo = self.ext["O"][0]
        gap = (fwd(loc) - ll / 2) - (fwd(bloc) + lb / 2)  # L's rear ahead of B's front
        to_o = (fwd(oloc) - lo / 2) - (fwd(loc) + ll / 2)  # O's rear ahead of L's front
        self.trk_own = self._advance(self.trk_own, loc, self.ref_yaw)
        self.trk_tgt = self._advance(self.trk_tgt, loc, self.ref_yaw)
        width = max(1.0, self.trk_own.transform.location.distance(self.trk_tgt.transform.location))
        ox, oy = self.trk_own.transform.location.x, self.trk_own.transform.location.y
        ux = (self.trk_tgt.transform.location.x - ox) / width
        uy = (self.trk_tgt.transform.location.y - oy) / width
        lat = max(0.0, min(1.0, ((loc.x - ox) * ux + (loc.y - oy) * uy) / width))
        g_ref = self.h0 * self.v_ref
        if self.stage == "warmup":
            # lead at v; speed up only to keep B's headway from falling under h0 (B, Autoware, picks
            # its own following gap: a longer one stays its own)
            vt = min(self.v_ref + 3.0, self.v_ref + 0.5 * max(0.0, g_ref - gap))
            accel = max(-3.0, min(1.0, 1.5 * (vt - v)))
            if vb >= self.v_ref - 0.3:
                self.cruise = True
            if t >= self.warmup_min and v > 0.5 and to_o / max(v, 0.1) <= CG.T0_LO:
                self.stage = "run"
                self.t_run0 = el
                self.log_event("start", t=t, gap=gap, v=v, vb=vb, h0_achieved=gap / max(vb, 0.1), to_o=to_o,
                               cruise=self.cruise)
        else:
            accel = max(-1.0, min(2.0, 1.5 * (self.v_ref - v)))
        tr = None if self.t_run0 is None else el - self.t_run0
        s_lat = 0.0
        if self.stage == "run":
            if self.t_start is None and v > 0.1 and to_o / v <= self.t_l:
                self.t_start = tr
                self.log_event("cutout", t=tr, to_o=to_o, gap=gap, v=v, vb=vb)
            if self.t_start is not None:
                u = min(1.0, (tr - self.t_start) / self.t_lc)
                s_lat = 0.5 * (1 - math.cos(math.pi * u))
            if lat < CG.LAT_CLEAR and to_o > 0:  # L's safety layer, cutout_game.l_accel
                need = v ** 2 / (2 * max(to_o - 1.0, 0.1))
                if need >= 3.0:
                    accel = min(accel, -min(CG.B_MAX, need))
        look = max(6.0, 0.7 * v)
        s_aim = s_lat
        if self.t_start is not None:  # steer at the path's point one look-ahead on (as cutout_arena)
            u = min(1.0, (tr + look / max(v, 1.0) - self.t_start) / self.t_lc)
            s_aim = 0.5 * (1 - math.cos(math.pi * u))
        na, nb = self._ahead(self.trk_own, look, self.ref_yaw), self._ahead(self.trk_tgt, look, self.ref_yaw)
        pa = (na or self.trk_own).transform.location
        pb = (nb or self.trk_tgt).transform.location
        tx, ty = pa.x + (pb.x - pa.x) * s_aim, pa.y + (pb.y - pa.y) * s_aim
        alpha = math.atan2(ty - loc.y, tx - loc.x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        angle = math.atan2(2 * self.wb * math.sin(alpha), look)
        th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, el)
        me.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=self.steer.command(angle)))
        if self.logf and int(t * 20) % 2 == 0:
            self.logf.write(json.dumps({"el": round(el, 3), "t": round(t, 3), "tr": None if tr is None else round(tr, 3),
                                        "phase": self.stage, "L": self._pose_of(me), "B": self._pose_of(self.other),
                                        "O": self._pose_of(self.obst), "gap": round(gap, 2), "to_o": round(to_o, 2),
                                        "lat": round(lat, 3), "s_lat": round(s_lat, 3), "accel": round(accel, 2)}) + "\n")


def main(argv=None):
    return scensim_agent.main(CutOutAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
