#!/usr/bin/env python3
"""Roadmap 074 S5 on the real stack: the oncoming car A at an unprotected left turn, as an swb agent.

Town03's junction 861 (the west road, unsignalized for Autoware: its Lanelet2 map links no
lanelet to a traffic light). B, the Autoware ego (role "hero"), comes south on road 4 lane 2 and
turns left onto road 62 lane -1, crossing the northbound lanes of road 3/4. A (this agent) comes
north in the inner lane (road 3 lane -1) and goes straight on. The rules are left_turn_game's C1:

  wait     A stands at its start until launching (accelerating at A_ACC to v, then cruising)
           brings its front to the conflict point C when B's front is `gap` s past its stop line,
           by B's estimated arrival (accelerating at B_ACC up to its speed limit b_vmax)
  approach A drives toward that arrival, its speed in [v - DV, v + DV], re-aimed every tick; the
           run's t = 0 is when B's estimated arrival at its stop line is T0 s away
  run      from B's front at its stop line (or B stopped short of it) A holds v ("hold"), and
           brakes only once B's body is in its lane ahead of it, after REACT s and only when
           stopping needs more than 0.5 m/s^2 (left_turn_game's safety layer)

C is where B's turn path crosses A's lane centre, less C_BACK m (B's body reaches A's lane
before its centre does). The driving (pedal maps, lags, steering, lane tracking straight through
the junction) is cutin_agent's. Logs every second tick (20 Hz world, so 10 Hz) both cars' truth
in CARLA's frame, B's distance to its stop line and its path's curvature at B (for S0's lateral
comfort, kappa), A's distance to C:

    python3 scripts/coop/oncoming_agent.py --relay tcp://localhost:5560 --entity coop_a --config <json>

Config: {"entity": "coop_a", "other": "hero", "v": 10, "gap": 4, "b_vmax": 8, "b_from": [4, 2], "b_to": [62, -1],
         "junction": 861, "a_ref": [-85.0, -30.0], "log": "<dir>/coop_a.jsonl", "carla_port": 2010}
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import swb_agent  # noqa: E402
from swb_agent import protocol as P  # noqa: E402
import arena as AR  # noqa: E402
import cutin_agent as CA  # noqa: E402

A_ACC = 2.0  # m/s^2, A's launch
DV = 3.0  # m/s, A's speed band while it aims its arrival
B_ACC = 1.0  # m/s^2: B's arrival is estimated accelerating at this up to its speed limit
T0 = 4.0  # s: the run starts (A holds) when B's estimated arrival at its stop line is this far
REACT = 1.0  # s, as left_turn_game.REACT
B_MAX = 7.0
C_BACK = 2.5  # m before the crossing of the centre lines


class OncomingAgent(CA.CutInAgent):
    name = "coop_oncoming"

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("side", "A")
        for k in ("d0", "dv", "vb"):
            config.setdefault(k, 0.0)
        config.setdefault("rule", {})
        super().__init__(config)
        c = self.config
        self.v_ref = float(c["v"])
        self.gap = float(c["gap"])
        self.b_from = tuple(c.get("b_from", (4, 2)))
        self.b_to = tuple(c.get("b_to", (62, -1)))
        self.junction = int(c.get("junction", 861))
        self.b_vmax = float(c.get("b_vmax", 8.0))
        self.a_ref = c.get("a_ref", (-85.0, -30.0))  # CARLA frame: a point on A's lane before the junction

    def _b_path(self):
        """B's turn through the junction: points (x, y, yaw_rad) every 0.5 m, its stop line first."""
        carla = self.world.carla
        m = self.world.map
        j = None
        for w in m.generate_waypoints(4.0):
            if w.is_junction and w.get_junction().id == self.junction:
                j = w.get_junction()
                break
        if j is None:
            raise swb_agent.Failed(f"no junction {self.junction}")
        for e, x in j.get_waypoints(carla.LaneType.Driving):
            p, n = e.previous(1.0), x.next(1.0)
            if p and n and (p[0].road_id, p[0].lane_id) == self.b_from and (n[0].road_id, n[0].lane_id) == self.b_to:
                pts, w = [], e
                while True:
                    tf = w.transform
                    pts.append((tf.location.x, tf.location.y, math.radians(tf.rotation.yaw)))
                    nx = w.next(0.5)
                    if not nx or not nx[0].is_junction or len(pts) > 400:
                        break
                    w = nx[0]
                return pts
        raise swb_agent.Failed(f"no connection {self.b_from} -> {self.b_to} in junction {self.junction}")

    def on_goal(self, goal, waypoints):
        with self._ctl:
            self.other = self._find_other()
            if self.other is None:
                raise swb_agent.Failed(f"no vehicle with role_name {self.other_role!r}")
            me = self.world.vehicle
            pc = me.get_physics_control()
            wb = abs(pc.wheels[0].position.x - pc.wheels[2].position.x) / 100.0
            tr = abs(pc.wheels[0].position.y - pc.wheels[1].position.y) / 100.0
            self.wb = wb if wb > 1.0 else 3.0
            self.steer = AR.Steering(math.radians(pc.wheels[0].max_steer_angle), tr / self.wb if self.wb else 0.5)
            doc = AR.carla_physics.load(str(AR.PHYSICS))
            self.lag = AR.PedalLag(doc.get("command_lags", {}))
            self.ext = {k: (2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y) for k, a in (("A", me), ("B", self.other))}
            self.own_lane = self.world.map.get_waypoint(me.get_location())
            self.trk = self.own_lane
            # A's frame (along and across its lane) from its lane near the junction: the road at
            # A's start may run a few degrees off (south of junction 730), which skews both
            ref = self.world.map.get_waypoint(self.world.carla.Location(x=float(self.a_ref[0]), y=float(self.a_ref[1])))
            self.ref_wp = ref
            self.ref_yaw = ref.transform.rotation.yaw
            self.path = self._b_path()
            # B's path curvature per point (CARLA's frame: positive toward increasing yaw)
            self.kap = []
            for i in range(len(self.path)):
                a, b = self.path[max(0, i - 2)], self.path[min(len(self.path) - 1, i + 2)]
                ds = 0.5 * (min(len(self.path) - 1, i + 2) - max(0, i - 2))
                dy = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
                self.kap.append(dy / ds if ds > 0 else 0.0)
            # the conflict: where B's path crosses A's lane centre (A's lane is straight here)
            hx, hy = math.cos(math.radians(self.ref_yaw)), math.sin(math.radians(self.ref_yaw))
            o = self.ref_wp.transform.location
            lat = lambda x, y: -(x - o.x) * hy + (y - o.y) * hx  # left of A's lane centre, CARLA's frame
            self.fwd = lambda x, y: (x - o.x) * hx + (y - o.y) * hy
            self.lat = lat
            k = next((i for i in range(1, len(self.path)) if lat(*self.path[i - 1][:2]) * lat(*self.path[i][:2]) <= 0), None)
            if k is None:
                raise swb_agent.Failed("B's path does not cross A's lane")
            self.s_cross = 0.5 * k  # along B's path, from its stop line
            self.c_fwd = self.fwd(*self.path[k][:2]) - C_BACK
            sx, sy, syaw = self.path[0]
            self.stop = (sx, sy, syaw)
            self.t0 = self.world.world.get_snapshot().timestamp.elapsed_seconds
            self.stage = "wait"
            self.t_run0 = None
            self.b_in_lane = None
            self.t_b_stop = None
            self.t_a_c = None
            self.v_cmd = 0.0
            self.mode = "drive"
        self.report(P.DRIVING, detail="")
        self.log.info(f"A: v {self.v_ref}, gap {self.gap}; B's path {len(self.path)} pts, crossing at s {self.s_cross}, "
                      f"C at {self.c_fwd:.1f} m along A's lane")
        self.log_event("setup", ext=self.ext, v=self.v_ref, gap=self.gap, s_cross=self.s_cross, c_fwd=round(self.c_fwd, 2),
                       stop=[round(sx, 2), round(sy, 2), round(syaw, 4)],
                       path=[[round(p[0], 2), round(p[1], 2), round(p[2], 4)] for p in self.path[::4]],
                       lane=[self.ref_wp.road_id, self.ref_wp.lane_id, round(o.x, 2), round(o.y, 2), round(self.ref_yaw, 2)])

    def _pose_of(self, actor):
        tf, v = actor.get_transform(), actor.get_velocity()
        return [round(tf.location.x, 3), round(tf.location.y, 3), round(math.radians(tf.rotation.yaw), 4),
                round(v.x, 3), round(v.y, 3)]

    def _b_state(self, bloc, byaw):
        """B's front's distance to its stop line (along the approach, > 0 before it), and the
        curvature of B's route at B (0 off the turn)."""
        sx, sy, syaw = self.stop
        lb = self.ext["B"][0]
        fx, fy = bloc.x + 0.5 * lb * math.cos(byaw), bloc.y + 0.5 * lb * math.sin(byaw)
        d = (sx - fx) * math.cos(syaw) + (sy - fy) * math.sin(syaw)
        i = min(range(len(self.path)), key=lambda k: (self.path[k][0] - bloc.x) ** 2 + (self.path[k][1] - bloc.y) ** 2)
        near = math.hypot(self.path[i][0] - bloc.x, self.path[i][1] - bloc.y) < 2.0
        return d, (self.kap[i] if near and 0 < i < len(self.path) - 1 else 0.0)

    @staticmethod
    def t_need(d, v0, v, a=A_ACC):
        """Time to cover d from speed v0, accelerating at a up to v, then cruising."""
        if d <= 0:
            return 0.0
        if v0 >= v:
            return d / max(v0, 0.1)
        t1 = (v - v0) / a
        d1 = (v0 + v) / 2 * t1
        if d <= d1:
            return (-v0 + math.sqrt(v0 * v0 + 2 * a * d)) / a
        return t1 + (d - d1) / v

    def _drive(self):
        import carla
        el = self.world.world.get_snapshot().timestamp.elapsed_seconds
        t = el - self.t0
        me = self.world.vehicle
        loc, yaw, v = self._truth(me)
        bloc, byaw, vb = self._truth(self.other)
        la, wa = self.ext["A"]
        lb, wb = self.ext["B"]
        a_front = self.fwd(loc.x, loc.y) + la / 2
        d_a = self.c_fwd - a_front
        d_b, kap_b = self._b_state(bloc, byaw)
        eta_b = self.t_need(d_b, vb, max(vb, self.b_vmax), B_ACC) if d_b > 0 else 0.0
        if self.t_b_stop is None and d_b <= 0:
            self.t_b_stop = t
            self.log_event("b_stopline", t=t, tr=None if self.t_run0 is None else el - self.t_run0, d_a=d_a, v_a=v)
        if self.t_a_c is None and d_a <= 0:
            self.t_a_c = t
            self.log_event("a_at_c", t=t, tr=None if self.t_run0 is None else el - self.t_run0)
        accel = 0.0
        if self.stage == "wait":
            accel = -3.0
            if t > 2.0 and eta_b + self.gap <= self.t_need(d_a, v, self.v_ref):
                self.stage = "approach"
                self.log_event("launch", t=t, d_a=d_a, d_b=d_b, vb=vb, eta_b=eta_b)
        if self.stage == "approach":
            want = eta_b + self.gap
            dv = DV if eta_b > 3.0 else 1.0  # near the lock, close to v: A holds v from the lock on
            self.v_cmd = max(self.v_ref - dv, min(self.v_ref + dv, d_a / max(want, 0.1))) if d_a > 0 else self.v_ref
            if v < self.v_cmd - 0.5 and self.t_need(d_a, v, self.v_cmd) > want:
                self.v_cmd = self.v_ref + dv  # behind the aim: catch up
            accel = max(-3.0, min(A_ACC, 1.5 * (self.v_cmd - v)))
            if self.t_run0 is None and (eta_b <= T0 or d_b <= 0):
                self.t_run0 = el
                self.log_event("start", t=t, d_a=d_a, v_a=v, d_b=d_b, vb=vb, eta_b=eta_b,
                               gap_achieved=(d_a / max(v, 0.1) - eta_b), lag=d_a / max(v, 0.1))
            # the lock: B at its stop line, or stopped short of it; from then on A holds v
            if d_b <= 0 or (vb < 0.3 and d_b < 20 and self.t_run0 is not None):
                self.stage = "run"
                if self.t_run0 is None:
                    self.t_run0 = el
                    self.log_event("start", t=t, d_a=d_a, v_a=v, d_b=d_b, vb=vb, eta_b=eta_b,
                                   gap_achieved=(d_a / max(v, 0.1) - eta_b), lag=d_a / max(v, 0.1))
                self.log_event("lock", t=t, tr=el - self.t_run0, d_a=d_a, v_a=v, d_b=d_b, vb=vb,
                               lag_v=d_a / self.v_ref, lag=d_a / max(v, 0.1))
        if self.stage == "run":
            accel = max(-1.0, min(A_ACC, 1.5 * (self.v_ref - v)))
        # the safety layer: B's body in my lane ahead of me, after REACT
        lat_b = self.lat(bloc.x, bloc.y)
        b_in = abs(lat_b) < 1.75 + wb / 2 and self.fwd(bloc.x, bloc.y) - lb / 2 > a_front - 1.0 and d_b < 0
        if b_in and self.b_in_lane is None:
            self.b_in_lane = t
            self.log_event("b_in_lane", t=t, tr=None if self.t_run0 is None else el - self.t_run0, d_a=d_a)
        if not b_in:
            self.b_in_lane = None
        if b_in and t >= self.b_in_lane + REACT:
            gap = self.fwd(bloc.x, bloc.y) - lb / 2 - a_front
            need = v ** 2 / (2 * max(gap - 1.0, 0.1))
            if need > 0.5:
                accel = min(accel, -min(B_MAX, need))
        look = max(6.0, 0.7 * v)
        self.trk = self._advance(self.trk, loc, self.ref_yaw)
        na = self._ahead(self.trk, look, self.ref_yaw)
        pa = (na or self.trk).transform.location
        alpha = math.atan2(pa.y - loc.y, pa.x - loc.x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        angle = math.atan2(2 * self.wb * math.sin(alpha), look)
        if self.stage == "wait":
            th, br = 0.0, 1.0
        else:
            th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, el)
        me.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=self.steer.command(angle) if v > 0.3 else 0.0))
        tr = None if self.t_run0 is None else el - self.t_run0
        if self.logf and int(t * 20) % 2 == 0:
            self.logf.write(json.dumps({"el": round(el, 3), "t": round(t, 3), "tr": None if tr is None else round(tr, 3),
                                        "phase": self.stage, "A": self._pose_of(me), "B": self._pose_of(self.other),
                                        "d_a": round(d_a, 2), "d_b": round(d_b, 2), "eta_b": round(eta_b, 2),
                                        "kB": round(kap_b, 4), "lat_b": round(lat_b, 2), "accel": round(accel, 2)}) + "\n")


def main(argv=None):
    return swb_agent.main(OncomingAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
