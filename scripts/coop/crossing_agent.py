#!/usr/bin/env python3
"""Roadmap 078 Phase 2: the crossing car A at an unsignalled 4-arm junction, as an swb agent.

Town05 junction 979 (E6's first site: four straight two-lane arms, its OpenDRIVE lights linked to no
Lanelet2 lanelet, so Autoware's intersection module treats it as unsignalled, `not_prioritized`).
B, the Autoware ego (role "hero"), comes along its road (by default south in ROS, +y in CARLA, the
outer lane) and goes straight on; A (this agent) comes along the crossing road (by default west, from
the ego's left, the outer lane) and goes straight on. The pair is S2's crossing game (074 S2,
crossing_game.py): Z is the W_BOX square where the two lane centre lines meet, u the front's distance
past Z's near edge along a car's own road, dt = t_A^Z - t_B^Z (dt > 0: B, the ego, is nearer).

  wait     A stands at its start until launching (A_ACC up to v, then cruising) brings its front to Z
           dt s after B's estimated arrival
  approach A aims its arrival at B's estimated arrival + dt, its speed in [v - DV, v + DV], re-aimed
           every tick. B's arrival is paced on B's actual acceleration: the slope of B's true speed
           over the last PACE_WIN s (swb 118: constant-speed pacing misses dt by ~0.3 s against an ego
           that is still accelerating), up to B's cruise speed, PACE_A before B is seen accelerating
  run      from the lock (B's estimated arrival T_LOCK s away, S2's T_MEAN) A drives by
           crossing_game.Policy with its rule, on what it perceives of B through arena.Observer
           (10 Hz, 0.19 s, noise), as S2's arena car: its gap test, its hold-out at the stop line, its
           patience and its safety layer. Policy's cruise speed is A's speed at the lock, so that the
           paced arrival is kept
  done     60 m past Z: brake to rest

Rules (crossing_game.A_RULES): "t1.5p5" (the arena's stock-like A: gap 1.5 s, patience 5 s), "go"
(aggressive: always go, its safety layer only), "tinfp5" (yielding: goes only once B has cleared Z,
or after 5 s at rest).

The driving is the arena's: acb's pedal maps and lags (arena.Longitudinal, PedalLag), pure pursuit on
A's lane line. Logs every world tick (20 Hz) both cars' truth in CARLA's frame, u and v of each, B's
estimated arrival and acceleration, the phase and the commanded acceleration:

    python3 scripts/coop/crossing_agent.py --relay tcp://localhost:5560 --entity coop_a --config <json>

Config: {"entity": "coop_a", "other": "hero", "dt": 0, "v": 8, "rule": "t1.5p5", "seed": 0,
         "a_ref": [-100.0, -4.2], "b_ref": [-132.0, -20.0], "b_vmax": 8, "log": "<dir>/coop_a.jsonl",
         "carla_port": 2700}
a_ref / b_ref: a point (CARLA frame) on each car's lane before the junction; the lane's waypoint
there gives the lane line (both roads are straight through j979).
"""

import json
import math
import random
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import swb_agent  # noqa: E402
from swb_agent import protocol as P  # noqa: E402
import arena as AR  # noqa: E402
import crossing_game as G  # noqa: E402
import cutin_agent as CA  # noqa: E402

A_ACC = 3.0  # m/s^2, A's launch from rest
DV = 4.0  # m/s, A's speed band while it aims its arrival
PACE_A = 1.0  # m/s^2: B's acceleration before it is seen accelerating (Autoware from rest)
ENGAGE = 1.5  # s: B at rest starts this long after the goal (Autoware's engage, measured in the first trial)
PACE_WIN = 1.0  # s: B's acceleration is the slope of its speed over this window
T_LOCK = G.T_MEAN  # s: the run starts when B's estimated arrival is this far (S2: both 5 s out)
PAST = 60.0  # m past Z: the run is over, A brakes to rest


def t_cover(d, v, vmax, a):
    """Time to cover d from speed v, accelerating at a up to vmax (constant speed when a <= 0)."""
    if d <= 0:
        return 0.0
    v = max(v, 0.0)
    if a <= 1e-3 or v >= vmax - 1e-6:
        return d / max(v, 0.1)
    d_acc = (vmax * vmax - v * v) / (2 * a)
    if d <= d_acc:
        return (-v + math.sqrt(v * v + 2 * a * d)) / a
    return (vmax - v) / a + (d - d_acc) / vmax


class CrossingAgent(CA.CutInAgent):
    name = "coop_crossing"

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("side", "A")
        for k in ("d0", "dv", "vb"):
            config.setdefault(k, 0.0)
        rule_name = config.get("rule", "t1.5p5")
        config["rule"] = {}
        super().__init__(config)
        c = self.config
        self.rule_name = rule_name
        self.rule = next(r for r in G.A_RULES if r[0] == rule_name)
        self.dt = float(c["dt"])
        self.v_ref = float(c.get("v", 8.0))
        self.b_vmax = float(c.get("b_vmax", 8.0))
        self.a_ref = c.get("a_ref", (-100.0, -4.2))
        self.b_ref = c.get("b_ref", (-132.0, -20.0))
        rng = random.Random(int(c.get("seed", 0)) * 2 + 1)
        self.obs = AR.Observer(float(c.get("obs_delay", G.OBS)), float(c.get("obs_sigma_x", 0.3)),
                               float(c.get("obs_sigma_v", 0.2)), rng)

    # -- geometry ----------------------------------------------------------------------------------
    def _line(self, ref):
        w = self.world.map.get_waypoint(self.world.carla.Location(x=float(ref[0]), y=float(ref[1])))
        yaw = math.radians(w.transform.rotation.yaw)
        return (w.transform.location.x, w.transform.location.y), (math.cos(yaw), math.sin(yaw)), w

    def _u(self, k, loc):
        """Front's distance past Z's near edge along k's road (as crossing_arena.Crossing.u)."""
        p0, d = self.lines[k]
        s = (loc.x - self.centre[0]) * d[0] + (loc.y - self.centre[1]) * d[1]
        return s + self.ext[k][0] / 2 + G.W_BOX / 2

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
            off = AR.carla_physics.differences(me, doc)
            self.ext = {k: (2 * a.bounding_box.extent.x, 2 * a.bounding_box.extent.y) for k, a in (("A", me), ("B", self.other))}
            pa, da, wa = self._line(self.a_ref)
            pb, db, wbp = self._line(self.b_ref)
            self.lines = {"A": (pa, da), "B": (pb, db)}
            # Z's centre: where the two lane centre lines meet
            den = da[0] * db[1] - da[1] * db[0]
            if abs(den) < 0.3:
                raise swb_agent.Failed("the two lanes do not cross")
            s = ((pb[0] - pa[0]) * db[1] - (pb[1] - pa[1]) * db[0]) / den
            self.centre = (pa[0] + da[0] * s, pa[1] + da[1] * s)
            self.exit = G.W_BOX + self.ext["A"][0]
            self.pol = G.Policy("A", self.rule, self.v_ref, hold_stop=G.ARENA_HOLD_STOP, exit_=self.exit)
            self.vb_hist = deque()
            self.t0 = self.world.world.get_snapshot().timestamp.elapsed_seconds
            self.stage = "wait"
            self.el_lock = None
            self.seen = {}
            self.v_cmd = 0.0
            self.mode = "drive"
        self.report(P.DRIVING, detail="")
        la = lambda w: [w.road_id, w.lane_id, round(w.transform.location.x, 2), round(w.transform.location.y, 2),
                         round(w.transform.rotation.yaw, 2)]  # noqa: E731
        self.log.info(f"A: rule {self.rule_name}, dt {self.dt}, v {self.v_ref}; Z at {self.centre[0]:.2f}, {self.centre[1]:.2f}; "
                      f"physics {'OFF ' + str(off) if off else 'as the spec'}")
        self.log_event("setup", rule=self.rule_name, dt=self.dt, v=self.v_ref, ext=self.ext, centre=list(self.centre),
                       lane_a=la(wa), lane_b=la(wbp), exit=self.exit, t0=self.t0, physics_off=bool(off),
                       u0={"A": self._u("A", me.get_location()), "B": self._u("B", self.other.get_location())})

    def log_event(self, what, **kw):
        if self.logf:
            r = lambda v: round(v, 3) if isinstance(v, float) else v  # noqa: E731
            self.logf.write(json.dumps({"event": what, **{k: r(v) for k, v in kw.items()}}) + "\n")
            self.logf.flush()

    def _b_accel(self, el, vb):
        h = self.vb_hist
        h.append((el, vb))
        while h and el - h[0][0] > PACE_WIN + 1e-9:
            h.popleft()
        if len(h) < 5 or h[-1][0] - h[0][0] < 0.5 * PACE_WIN:
            return 0.0
        n = len(h)
        mt = sum(x[0] for x in h) / n
        mv = sum(x[1] for x in h) / n
        den = sum((x[0] - mt) ** 2 for x in h)
        return sum((x[0] - mt) * (x[1] - mv) for x in h) / den if den > 0 else 0.0

    def _eta_b(self, ub, vb, ab):
        """B's estimated time to Z's near edge, on its actual acceleration up to its cruise speed."""
        if ub >= 0:
            return 0.0
        if vb < 0.3 and ab < 0.3:
            return ENGAGE + t_cover(-ub, 0.0, self.b_vmax, PACE_A)  # not started yet
        if vb < 1.0 and ab < 0.3:
            a = PACE_A  # just starting: Autoware's launch
        elif vb < self.b_vmax - 0.3 and ab > 0.05:
            a = ab
        else:
            a = 0.0
        return t_cover(-ub, vb, self.b_vmax, a)

    def _drive(self):
        import carla
        el = self.world.world.get_snapshot().timestamp.elapsed_seconds
        t = el - self.t0
        me = self.world.vehicle
        loc, yaw, v = self._truth(me)
        bloc, byaw, vb = self._truth(self.other)
        ua, ub = self._u("A", loc), self._u("B", bloc)
        ab = self._b_accel(el, vb)
        eta_b = self._eta_b(ub, vb, ab)
        for k, u in (("A", ua), ("B", ub)):
            if u >= 0 and f"{k}_in" not in self.seen:
                self.seen[f"{k}_in"] = t
                self.log_event(f"{k}_in", t=t, tr=None if self.el_lock is None else el - self.el_lock, v=v if k == "A" else vb)
            if u >= self.exit if k == "A" else u >= G.W_BOX + self.ext["B"][0]:
                if f"{k}_out" not in self.seen:
                    self.seen[f"{k}_out"] = t
                    self.log_event(f"{k}_out", t=t, tr=None if self.el_lock is None else el - self.el_lock)
        accel = 0.0
        tr = None if self.el_lock is None else el - self.el_lock
        # the observer runs from the start, on the run's clock (negative before the lock)
        seen = self.obs.push(t, ub, 0.0, vb)
        if self.stage == "wait":
            accel = -3.0
            if t > 0.5 and eta_b + self.dt <= t_cover(-ua, v, self.v_ref, A_ACC) + G.LAUNCH_DEAD:
                self.stage = "approach"
                self.log_event("launch", t=t, ua=ua, ub=ub, vb=vb, ab=ab, eta_b=eta_b)
        if self.stage == "approach":
            want = eta_b + self.dt
            dv = DV  # the whole band to the lock: at dt -1 A needs ~12 m/s against an ego still accelerating
            self.v_cmd = max(self.v_ref - dv, min(self.v_ref + dv, -ua / max(want, 0.1))) if ua < 0 else self.v_ref
            if v < self.v_cmd - 0.5 and t_cover(-ua, v, self.v_cmd, A_ACC) > want:
                self.v_cmd = self.v_ref + dv  # behind the aim: catch up
            accel = max(-3.0, min(A_ACC, 1.5 * (self.v_cmd - v)))
            if eta_b <= T_LOCK or ub >= -1.0:
                self.stage = "run"
                self.el_lock = el
                self.t_lock = t
                tr = 0.0
                t_a = -ua / max(v, 0.1)
                self.pol.v0 = max(v, 1.0)
                self.log_event("lock", t=t, ua=ua, va=v, ub=ub, vb=vb, ab=ab, eta_b=eta_b, t_a=t_a,
                               dt_lock=t_a - eta_b, dt_const=t_a - (-ub / max(vb, 0.1)))
        if self.stage == "run":
            if seen is not None and seen[0] >= self.t_lock - 1e-9 and self.last_stamp != seen[0]:
                self.last_stamp = seen[0]
                self.pol.perceive(seen[0] - self.t_lock, seen[1], seen[3], seen[4])
            accel = self.pol.step(tr, ua, v)
            self.pol.after(tr, ua, v)
            if ua > self.exit + PAST:
                self.stage = "done"
                self.log_event("done", t=t, tr=tr)
        if self.stage == "done":
            accel = -3.0
        # pure pursuit on A's lane line
        p0, d = self.lines["A"]
        look = max(6.0, 0.7 * v)
        s = (loc.x - p0[0]) * d[0] + (loc.y - p0[1]) * d[1]
        tx, ty = p0[0] + d[0] * (s + look), p0[1] + d[1] * (s + look)
        alpha = math.atan2(ty - loc.y, tx - loc.x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        angle = math.atan2(2 * self.wb * math.sin(alpha), look)
        if self.stage == "wait":
            th, br = 0.0, 1.0
        else:
            th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, el)
        me.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=self.steer.command(angle) if v > 0.3 else 0.0))
        if self.logf:
            bv, av = self.other.get_velocity(), me.get_velocity()
            self.logf.write(json.dumps({
                "el": round(el, 3), "t": round(t, 3), "tr": None if tr is None else round(tr, 3), "phase": self.stage,
                "A": [round(loc.x, 3), round(loc.y, 3), round(yaw, 4), round(av.x, 3), round(av.y, 3)],
                "B": [round(bloc.x, 3), round(bloc.y, 3), round(byaw, 4), round(bv.x, 3), round(bv.y, 3)],
                "ua": round(ua, 3), "ub": round(ub, 3), "va": round(v, 3), "vb": round(vb, 3), "ab": round(ab, 3),
                "eta_b": round(min(eta_b, 999.0), 3), "accel": round(accel, 3),
                "committed": bool(self.pol.committed), "th": round(th, 3), "br": round(br, 3)}) + "\n")

    last_stamp = None


def main(argv=None):
    return swb_agent.main(CrossingAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
