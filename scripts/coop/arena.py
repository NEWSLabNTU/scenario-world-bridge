#!/usr/bin/env python3
"""Roadmap 074 E1: the arena spike. Light policies play the cut-in on CARLA's physics, many pairs
at once, so the cost of a run is CARLA's physics and nothing else.

A study tool, not part of the swb stack. It runs on a CARLA of its own (its own port): it loads
a generated world there, makes it synchronous and puts it back to asynchronous on the way out.
It refuses a CARLA that is already synchronous (someone else owns the tick). No sensors are
spawned, so no sensor can outlive its car.

The world: K parallel straight roads, each two lanes one way (OpenDRIVE right lanes -1 and -2,
travel along +x), 60 m apart. One pair per road: B in lane -1, A in lane -2 wanting into -1.

The car is the spec's: `sim_vehicle_spec`'s `carla_physics.yaml` applied and read back, the
physics timing swb_carla_world uses (0.05 s ticks, substeps of DT/16), and the controls the way
acb gives them: a requested acceleration turned into throttle or brake through the car's
measured pedal maps, then the spec's pedal lags (brake dead time and time constants); a tire
angle turned into CARLA's steer command through the wheel geometry.

The policies are C1's (cutin_game.py), on CARLA instead of a point mass, and they do not see
the truth: each sees the other car through a perception model (sampled at 10 Hz, delayed by
`--obs-delay`, with position and speed noise).

Modes:
  --parity   one car runs vehicle-check's manoeuvres (autoware_sim_check.dynamics: acceleration
             and brake steps, a steering sweep) through the arena's controller and judges them
             with the same code and tolerances; then full braking from 12 m/s and full
             acceleration to 12 m/s
  (default)  the cut-in: contexts x profiles in batches of K pairs, one CSV row per run, and
             the throughput (runs per hour, tick time) in summary.json

Usage:
  scripts/coop/arena.py --port 2600 --out <dir> --parity
  scripts/coop/arena.py --port 2600 --out <dir> --k 4 [--contexts 8] [--seed 1]
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import random
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import carla

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/sim_vehicle_spec"))
sys.path.insert(0, str(ROOT / "src/autoware_sim_vehicle/autoware_sim_check"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_vehicle_spec import carla_physics  # noqa: E402

import cutin_game as C1  # noqa: E402
import measures  # noqa: E402

CAR = "tesla_model3"
PHYSICS = ROOT / f"src/sim_vehicle_spec/generated/{CAR}/carla_physics.yaml"
MAPS = ROOT / f"src/autoware_sim_vehicle/cars/{CAR}/{CAR}_description/config"
DT = 0.05
ROAD_LEN = 1100.0  # warm-up and a run of up to T_MAX at up to 16 m/s
ROAD_GAP = 60.0
LANE_W = 3.5
WARMUP = 15.0  # s, to bring a pair to its context
PEAK_WINDOW = 0.5  # s, over which peak braking is measured
HORIZON = C1.HORIZON


# --- the car's controls, as acb gives them -----------------------------------------------------
class PedalMap:
    """acb's longitudinal_map.rs: pedal rows x speed columns -> acceleration."""

    def __init__(self, path):
        rows = [l.strip() for l in open(path) if l.strip() and not l.startswith("#")]
        self.speeds = [float(x) for x in rows[0].split(",")[1:]]
        self.pedals, self.acc = [], []
        for r in rows[1:]:
            c = [float(x) for x in r.split(",")]
            self.pedals.append(c[0])
            self.acc.append(c[1:])

    def at(self, i, v):
        row = self.acc[i]
        for j, s in enumerate(self.speeds):
            if s >= v:
                if j == 0:
                    return row[0]
                s0 = self.speeds[j - 1]
                return row[j - 1] + (v - s0) / (s - s0) * (row[j] - row[j - 1])
        return row[-1]

    def passive(self, v):
        return self.at(0, v)

    def pedal_for(self, target, v, rising):
        last = len(self.pedals) - 1
        if (target >= self.at(last, v)) if rising else (target <= self.at(last, v)):
            return self.pedals[last]
        if (target <= self.at(0, v)) if rising else (target >= self.at(0, v)):
            return self.pedals[0]
        for i in range(1, last + 1):
            a0, a1 = self.at(i - 1, v), self.at(i, v)
            if (a0 <= target <= a1) if rising else (a1 <= target <= a0):
                f = 0.0 if a1 == a0 else (target - a0) / (a1 - a0)
                return self.pedals[i - 1] + f * (self.pedals[i] - self.pedals[i - 1])
        return self.pedals[last]


class Longitudinal:
    """acb's command_for: a requested acceleration -> (throttle, brake)."""

    def __init__(self):
        self.accel = PedalMap(MAPS / "accel_map.csv")
        self.brake = PedalMap(MAPS / "brake_map.csv")

    def command(self, a, v):
        brake_coast = self.brake.passive(v)
        coasting = max(self.accel.passive(v), brake_coast)
        if a >= coasting:
            return min(1.0, max(0.0, self.accel.pedal_for(a, v, True))), 0.0
        target = brake_coast + (a - coasting)
        return 0.0, min(1.0, max(0.0, self.brake.pedal_for(target, v, False)))


class PedalLag:
    """acb's pedal_lag.rs: the brake's dead time and first-order lags, on sim time."""

    def __init__(self, lags):
        self.dead = lags.get("brake_dead_time", 0.0)
        self.tb = lags.get("brake_time_constant", 0.0)
        self.tm = lags.get("motor_time_constant", 0.0)
        self.pending = deque()
        self.req = self.brake = self.throttle = 0.0
        self.last = None

    @staticmethod
    def _lag(x, target, dt, tau):
        if tau <= 0:
            return target
        nxt = x + (target - x) * (1 - math.exp(-dt / tau))
        return target if abs(nxt - target) < 1e-3 else nxt

    def step(self, throttle, brake, now):
        if self.last is None or now < self.last:
            self.pending.clear()
            self.req = self.brake = brake
            self.throttle = throttle
            self.last = now
            return throttle, brake
        dt = now - self.last
        self.last = now
        self.pending.append((now, brake))
        while self.pending and now - self.pending[0][0] + 1e-9 >= self.dead:
            self.req = self.pending.popleft()[1]
        self.brake = self._lag(self.brake, self.req, dt, self.tb)
        self.throttle = self._lag(self.throttle, throttle, dt, self.tm)
        return min(1, max(0, self.throttle)), min(1, max(0, self.brake))


class Steering:
    """acb's steer_command_for: the steer command that delivers a tire angle (Ackermann mean)."""

    def __init__(self, max_steer, track_over_wb):
        self.max = max_steer
        self.k = track_over_wb

    def tire(self, cmd):
        inner = min(1.0, abs(cmd)) * self.max
        if inner <= 1e-9:
            return 0.0
        outer = math.atan(1.0 / (1.0 / math.tan(inner) + self.k))
        return math.copysign(0.5 * (inner + outer), cmd)

    def command(self, angle):
        t = abs(angle)
        if t <= 1e-9:
            return 0.0
        if t >= abs(self.tire(1.0)):
            return math.copysign(1.0, angle)
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi)
            if abs(self.tire(mid)) < t:
                lo = mid
            else:
                hi = mid
        return math.copysign(0.5 * (lo + hi), angle)


# --- the world ----------------------------------------------------------------------------------
def xodr(k, length):
    roads = []
    for i in range(k):
        roads.append(f"""
  <road name="r{i}" length="{length}" id="{i + 1}" junction="-1">
    <link/>
    <planView><geometry s="0" x="0" y="{-i * ROAD_GAP}" hdg="0" length="{length}"><line/></geometry></planView>
    <elevationProfile/><lateralProfile/>
    <lanes><laneSection s="0">
      <center><lane id="0" type="none" level="false"/></center>
      <right>
        <lane id="-1" type="driving" level="false"><width sOffset="0" a="{LANE_W}" b="0" c="0" d="0"/></lane>
        <lane id="-2" type="driving" level="false"><width sOffset="0" a="{LANE_W}" b="0" c="0" d="0"/></lane>
      </right>
    </laneSection></lanes>
  </road>""")
    return ('<?xml version="1.0" standalone="yes"?>\n<OpenDRIVE>\n'
            '  <header revMajor="1" revMinor="4" name="arena" version="1"/>' + "".join(roads) + "\n</OpenDRIVE>\n")


class Arena:
    def __init__(self, port, k):
        self.client = carla.Client("localhost", port)
        self.client.set_timeout(120.0)
        w = self.client.get_world()
        if w.get_settings().synchronous_mode:
            sys.exit("this CARLA is already synchronous: another process owns the tick")
        params = carla.OpendriveGenerationParameters(
            vertex_distance=2.0, max_road_length=50.0, wall_height=0.0, additional_width=0.6,
            smooth_junctions=True, enable_mesh_visibility=True)
        self.world = self.client.generate_opendrive_world(xodr(k, ROAD_LEN), params)
        s = self.world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = DT
        s.substepping = True
        s.max_substep_delta_time, s.max_substeps = DT / 16, 16  # swb_carla_world's
        s.no_rendering_mode = True
        self.world.apply_settings(s)
        self.map = self.world.get_map()
        self.doc = carla_physics.load(str(PHYSICS))
        self.bp = self.world.get_blueprint_library().find(self.doc["blueprint"])
        self.k = k
        self.t = 0.0
        self.actors = []
        self.pending = []  # controls for the next tick, sent in one batch
        self.snap = None  # every actor's state after the last tick, one call

    def lane_y(self, road, lane):
        """CARLA y of a lane's centre on a road (asked of the map, not assumed)."""
        y_ref = road * ROAD_GAP  # OpenDRIVE y = -i*gap, CARLA y = -OpenDRIVE y
        guess = y_ref + (LANE_W / 2 if lane == -1 else 1.5 * LANE_W)
        wp = self.map.get_waypoint(carla.Location(x=50.0, y=guess, z=0.0), project_to_road=True)
        return wp.transform.location.y, wp.transform.location.z

    def spawn(self, road, lane, x):
        y, z = self.lane_y(road, lane)
        tf = carla.Transform(carla.Location(x=x, y=y, z=z + 0.6), carla.Rotation(yaw=0.0))
        a = self.world.spawn_actor(self.bp, tf)
        self.actors.append(a)
        return a

    def put_physics(self, actors):
        for a in actors:
            carla_physics.apply(a, self.doc)
        for _ in range(5):
            self.tick()
        for a in actors:
            off = carla_physics.differences(a, self.doc)
            if off:
                raise RuntimeError(f"physics did not take on {a.id}: {off}")

    def tick(self):
        if self.pending:
            self.client.apply_batch(self.pending)
            self.pending = []
        self.world.tick()
        self.snap = self.world.get_snapshot()
        self.t += DT

    def close(self):
        try:
            s = self.world.get_settings()
            s.synchronous_mode = False
            s.no_rendering_mode = False
            self.world.apply_settings(s)
        finally:
            for a in self.actors:
                a.destroy()


class Car:
    def __init__(self, arena, actor):
        self.arena = arena
        self.a = actor
        pc = actor.get_physics_control()
        wheels = pc.wheels
        wb = abs(wheels[0].position.x - wheels[2].position.x) / 100.0
        track = abs(wheels[0].position.y - wheels[1].position.y) / 100.0
        self.wheelbase = wb
        self.steering = Steering(math.radians(wheels[0].max_steer_angle), track / wb)
        self.lon = Longitudinal()
        self.lag = PedalLag(arena.doc.get("command_lags", {}))
        bb = actor.bounding_box
        self.length, self.width = 2 * bb.extent.x, 2 * bb.extent.y
        self.v_prev = 0.0
        self.acc = 0.0

    def state(self):
        s = self.arena.snap.find(self.a.id) if self.arena.snap is not None else None
        if s is not None:
            tf, v = s.get_transform(), s.get_velocity()
        else:
            tf, v = self.a.get_transform(), self.a.get_velocity()
        return tf.location.x, tf.location.y, math.radians(tf.rotation.yaw), math.hypot(v.x, v.y)

    def pose(self):
        """(x, y, yaw, vx, vy, length, width), for measures.push_poses."""
        s = self.arena.snap.find(self.a.id) if self.arena.snap is not None else None
        tf, v = (s.get_transform(), s.get_velocity()) if s is not None else (self.a.get_transform(), self.a.get_velocity())
        return (tf.location.x, tf.location.y, math.radians(tf.rotation.yaw), v.x, v.y, self.length, self.width)

    def drive(self, accel, y_target, hand_brake=False):
        x, y, yaw, v = self.state()
        th, br = self.lon.command(accel, v)
        th, br = self.lag.step(th, br, self.arena.t)
        look = max(6.0, 0.7 * v)
        alpha = math.atan2(y_target - y, look) - yaw
        angle = math.atan2(2 * self.wheelbase * math.sin(alpha), look)
        self.arena.pending.append(carla.command.ApplyVehicleControl(self.a.id, carla.VehicleControl(
            throttle=th, brake=br, steer=self.steering.command(angle), hand_brake=hand_brake)))

    def measure(self):
        _, _, _, v = self.state()
        a = (v - self.v_prev) / DT
        self.acc = 0.7 * self.acc + 0.3 * a  # 3-tick smoothing of a differenced speed
        self.v_prev = v
        return v

    def park(self, tf):
        self.a.set_target_velocity(carla.Vector3D(0, 0, 0))
        self.a.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        self.a.set_transform(tf)
        self.arena.snap = None  # the snapshot no longer says where it is
        self.lag = PedalLag(self.arena.doc.get("command_lags", {}))
        self.v_prev = self.acc = 0.0


# --- parity ------------------------------------------------------------------------------------
def parity(args):
    from autoware_sim_check import dynamics as D

    ar = Arena(args.port, 1)
    out = {}
    try:
        actor = ar.spawn(0, -1, 20.0)
        ar.put_physics([actor])
        car = Car(ar, actor)
        y0, _ = ar.lane_y(0, -1)
        start = actor.get_transform()

        def settle():
            car.park(start)
            for _ in range(20):
                car.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
                ar.tick()

        results = []
        settle()
        for step in D.plan():
            samples, t0 = [], ar.t
            x0, y_hold = actor.get_transform().location.x, y0
            for _ in range(int(25 / DT)):
                ts = ar.t - t0
                v = car.measure()
                acc, _, steer = D.command(step, ts, v)
                if step.kind == "steer":
                    th, br = car.lon.command(acc, v)
                    th, br = car.lag.step(th, br, ar.t)
                    sc = car.steering.command(steer)
                    car.a.apply_control(carla.VehicleControl(throttle=th, brake=br, steer=sc))
                else:
                    car.drive(acc, y_hold)
                ar.tick()
                w = car.a.get_angular_velocity()
                samples.append({"t": ts, "v": car.state()[3],
                                "steer": car.steering.tire(car.a.get_control().steer),
                                "yaw_rate": math.radians(abs(w.z)) * (1 if steer >= 0 else -1)})
                if D.done(step, ts, samples[-1]["v"]):
                    break
            r = D.judge(step, samples, car.wheelbase)
            results.append(r)
            print(f"  {'PASS' if r['pass'] else 'FAIL'} {r}")
            if step.kind == "steer":
                settle()
            elif car.a.get_transform().location.x > ROAD_LEN - 200:
                settle()
        out["vehicle_check_steps"] = results

        # full braking from 12 m/s, full acceleration to 12 m/s
        settle()
        t0 = ar.t
        while car.state()[3] < 12.0 and ar.t - t0 < 30:
            car.drive(10.0, y0)
            ar.tick()
        t_acc = ar.t - t0
        x_b = car.a.get_transform().location.x
        v_b = car.state()[3]
        t0, peak = ar.t, 0.0
        while car.state()[3] > 0.05 and ar.t - t0 < 15:
            car.drive(-20.0, y0)
            ar.tick()
            car.measure()
            peak = max(peak, -car.acc)
        out["full_accel_0_to_12_s"] = round(t_acc, 2)
        out["full_brake_from"] = round(v_b, 2)
        out["full_brake_distance_m"] = round(car.a.get_transform().location.x - x_b, 2)
        out["full_brake_mean_decel"] = round(v_b ** 2 / 2 / max(0.01, out["full_brake_distance_m"]), 2)
        out["full_brake_peak_decel"] = round(peak, 2)
        out["wheelbase"] = round(car.wheelbase, 3)
        out["pass"] = all(r["pass"] for r in results)
    finally:
        ar.close()
    return out


# --- the cut-in ---------------------------------------------------------------------------------
@dataclass
class Observer:
    """What one car sees of the other: sampled at 10 Hz, delayed, noisy."""

    delay: float
    sigma_x: float
    sigma_v: float
    rng: random.Random
    buf: deque = field(default_factory=deque)
    hist: deque = field(default_factory=deque)
    seen: tuple | None = None
    last_sample: float = -1.0

    def push(self, t, x, lat, v):
        if t - self.last_sample >= 0.1 - 1e-9:
            self.last_sample = t
            self.buf.append((t, x + self.rng.gauss(0, self.sigma_x), lat, v + self.rng.gauss(0, self.sigma_v)))
        while self.buf and t - self.buf[0][0] >= self.delay - 1e-9:
            tt, xx, ll, vv = self.buf.popleft()
            # acceleration as the slope of the seen speeds over the last second: differencing
            # 10 Hz speeds with 0.1 m/s noise gives ~1.4 m/s^2 of noise, which reads as a yield
            self.hist.append((tt, vv))
            while self.hist and tt - self.hist[0][0] > 1.0 + 1e-9:
                self.hist.popleft()
            acc = 0.0
            if len(self.hist) >= 5:
                n = len(self.hist)
                mt = sum(h[0] for h in self.hist) / n
                mv = sum(h[1] for h in self.hist) / n
                den = sum((h[0] - mt) ** 2 for h in self.hist)
                acc = sum((h[0] - mt) * (h[1] - mv) for h in self.hist) / den if den > 0 else 0.0
            self.seen = (tt, xx, ll, vv, acc)
        return self.seen


class Pair:
    """One cut-in on one road: A (lane -2) and B (lane -1), one profile at one context."""

    def __init__(self, arena, road, a_car, b_car, job, rng, args):
        self.ar, self.road, self.A, self.B = arena, road, a_car, b_car
        self.job = job
        self.rng = rng
        self.args = args
        self.yB, _ = arena.lane_y(road, -1)
        self.yA0, _ = arena.lane_y(road, -2)
        self.react_a = C1.TYPES["car"].reaction + rng.uniform(-args.reaction_jitter, args.reaction_jitter)
        self.react_b = job.get("react_b", C1.TYPES["car"].reaction) + rng.uniform(-args.reaction_jitter, args.reaction_jitter)
        mk = lambda: Observer(args.obs_delay, args.obs_sigma_x, args.obs_sigma_v, rng)
        # B may perceive differently from A (C3a: Autoware on ground-truth objects, A the agent's
        # observer): obs_b_* when the args carry them
        gb = lambda k: getattr(args, "obs_b_" + k, None)
        mk_b = (lambda: Observer(gb("delay"), gb("sigma_x"), gb("sigma_v"), rng)) if gb("delay") is not None else mk
        self.obs_a, self.obs_b = mk(), mk_b()  # what A sees of B, what B sees of A
        self.phase = "warmup"
        self.t0 = None
        self.rec = None

    # geometry
    def rear(self, car):
        return car.state()[0] - car.length / 2

    def front(self, car):
        return car.state()[0] + car.length / 2

    def lat(self):
        """A's lateral progress from its lane (0) to B's (1)."""
        y = self.A.state()[1]
        return max(0.0, min(1.0, (self.yA0 - y) / (self.yA0 - self.yB)))

    def place(self):
        d0, dv = self.job["d0"], self.job["dv"]
        r0 = d0 - dv * WARMUP
        xb = 30.0 + max(0.0, -r0 + self.A.length + 5.0)
        xa = xb + self.B.length / 2 + r0 + self.A.length / 2
        for car, x, y in ((self.B, xb, self.yB), (self.A, xa, self.yA0)):
            loc = car.a.get_transform().location
            car.park(carla.Transform(carla.Location(x=x, y=y, z=loc.z + 0.3), carla.Rotation(yaw=0)))
        self.tw = self.ar.t

    def step_warmup(self):
        d0, dv, vb = self.job["d0"], self.job["dv"], self.job["vb"]
        t = self.ar.t - self.tw
        _, _, _, va = self.A.state()
        _, _, _, vbb = self.B.state()
        ab = max(-3.0, min(3.0, 1.5 * (vb - vbb)))
        r = self.rear(self.A) - self.front(self.B)
        r_ref = d0 - dv * max(0.0, WARMUP - t)
        aa = ab + 0.6 * (r_ref - r) + 1.8 * (dv - (va - vbb))
        aa = max(-4.0, min(3.0, aa))
        self.B.drive(ab, self.yB)
        self.A.drive(aa, self.yA0)
        if t >= WARMUP:
            self.start()

    def start(self):
        _, _, _, va = self.A.state()
        _, _, _, vb = self.B.state()
        self.phase = "run"
        self.t0 = self.ar.t
        self.va0, self.vb0 = va, vb
        self.xa0, self.xb0 = self.A.state()[0], self.B.state()[0]
        self.d0_real = self.rear(self.A) - self.front(self.B)
        self.dv_real = va - vb
        self.pa, self.pb = self.job.get("pa"), self.job.get("pb")  # parameterized rules (C2a)
        self.decide = 0.0 if self.job.get("a") == C1.GO else None
        self.dropping = self.job.get("a") == C1.WAIT
        self.b_latched = False
        self.trace = []
        self.vhist = (deque(), deque())
        self.lat_seen = self.in_lane_at = self.b_brake_seen = None
        self.min_ttc = self.min_thw = math.inf
        self.contact = False
        self.peak = [0.0, 0.0]
        self.prev_ab = 0.0
        b = C1.TYPES["car"].b_max
        self.meas = measures.Measures(b_max=(b, b))

    def step_run(self):
        t = self.ar.t - self.t0
        A, B = self.A, self.B
        xa, ya, _, va = A.state()
        xb, yb, _, vb = B.state()
        lat = self.lat()
        sb = self.obs_a.push(t, xb, 0.0, vb)  # A sees B
        sa = self.obs_b.push(t, xa, lat, va)  # B sees A
        job, cap = self.job, C1.TYPES["car"]
        la, lb = A.length, B.length
        # --- A
        if sb is not None:
            _, xb_s, _, vb_s, ab_s = sb
        else:
            xb_s, vb_s, ab_s = xb, vb, 0.0
        if self.pa is not None and self.decide is None and not self.dropping:
            pa = self.pa
            gap_ahead = (xa - la / 2) - (xb_s + lb / 2)  # A's rear ahead of B's front, as seen
            need = pa["buffer"] + vb_s * pa["tau"] + max(0.0, vb_s ** 2 - va ** 2) / (2 * pa["a_rear"])
            if ab_s <= -C1.YIELD_SEEN and self.b_brake_seen is None:
                self.b_brake_seen = t
            if gap_ahead >= need:
                self.decide = t
            elif (pa["use_yield"] and self.b_brake_seen is not None and t >= self.b_brake_seen + self.react_a
                  and gap_ahead > -la / 2):
                self.decide = t
            elif t >= pa["patience"] + (C1.WATCH if pa["use_yield"] else 0.0):
                self.dropping = True
        elif job.get("a") == C1.GO_IF_YIELD and self.decide is None and not self.dropping:
            if ab_s <= -C1.YIELD_SEEN and self.b_brake_seen is None:
                self.b_brake_seen = t
            if self.b_brake_seen is not None and t >= self.b_brake_seen + self.react_a:
                self.decide = t
            elif t >= self.react_a + C1.WATCH:
                self.dropping = True
        gap_behind = (xb_s - lb / 2) - (xa + la / 2)
        if self.dropping and self.decide is None:
            want = C1.S0 + vb_s * C1.HEADWAY
            if gap_behind >= want:
                self.decide = t
            aa = -C1.A_WAIT_DECEL if gap_behind < want and va > 0.5 * vb_s else 0.0
            if self.decide is not None:
                aa = C1.follow_accel(gap_behind, va, vb_s, cap, self.va0)
        elif lat > 0.05 and xa < xb_s:
            aa = C1.follow_accel(gap_behind, va, vb_s, cap, self.va0)
        else:
            aa = max(-1.0, min(cap.a_max, 1.5 * (self.va0 - va)))
        t_prep = self.pa["t_prep"] if self.pa is not None else cap.t_prep
        if self.decide is not None and t >= self.decide + t_prep:
            s = min(1.0, (t - self.decide - t_prep) / cap.t_lc)
            y_t = self.yA0 + (self.yB - self.yA0) * 0.5 * (1 - math.cos(math.pi * s))
        else:
            y_t = self.yA0
        # --- B
        if sa is not None:
            _, xa_s, lat_s, va_s, _ = sa
        else:
            xa_s, lat_s, va_s = xa, 0.0, va
        if lat_s > 0.08 and self.lat_seen is None:
            self.lat_seen = t
        if lat_s >= C1.LANE_LINE and xa_s > xb and self.in_lane_at is None:
            self.in_lane_at = t
        gap_b = (xa_s - la / 2) - (xb + lb / 2)
        a_ahead = xa_s > xb - lb
        if self.pb is not None:
            pb = self.pb
            pre = t >= self.react_b and a_ahead and lat_s < C1.LANE_LINE and gap_b < pb["pre_band"]
            react = (self.lat_seen is not None and t >= self.lat_seen + self.react_b and a_ahead
                     and gap_b < pb["react_gap"])
            self.b_latched = self.b_latched or ((pre or react) and a_ahead)
            yielding = self.b_latched and a_ahead
            b_decel = pb["b_y"]
        else:
            yielding = (job["b"] == C1.PRE_YIELD and t >= self.react_b and a_ahead) or (
                job["b"] == "react_yield" and self.lat_seen is not None
                and t >= self.lat_seen + self.react_b and a_ahead)
            b_decel = C1.B_YIELD_DECEL
        if yielding:
            ab = -b_decel if gap_b < C1.S0 + vb * C1.HEADWAY else \
                C1.follow_accel(gap_b, vb, va_s, cap, self.vb0)
        else:
            ab = max(-1.0, min(cap.a_max, 1.5 * (self.vb0 - vb)))
        if self.in_lane_at is not None and t >= self.in_lane_at + self.react_b and xa_s > xb:
            ab = min(ab, C1.follow_accel(gap_b, vb, va_s, cap, self.vb0))
        A.drive(aa, y_t)
        B.drive(ab, self.yB)
        # --- truth
        # peak braking: the largest speed drop over any 0.5 s, per 0.5 s, so that it does not
        # depend on how often speed is sampled (the arena ticks at 0.05 s, swb's logs at 0.1 s;
        # 074 D2)
        for k, vv in ((0, va), (1, vb)):
            h = self.vhist[k]
            h.append((t, vv))
            while h and t - h[0][0] > PEAK_WINDOW + 1e-9:
                h.popleft()
            if t - h[0][0] >= PEAK_WINDOW - 1e-9:
                self.peak[k] = max(self.peak[k], (h[0][1] - vv) / PEAK_WINDOW)
        dy = abs(ya - yb)
        dx = xa - xb
        # contact: the oriented bodies overlap (measures, the same rule in C1 and swb's scoring)
        self.meas.push_poses(t, A.pose(), B.pose())
        if self.meas.clear <= 0.0:
            self.contact = True
        follower = gap = closing = None
        if dy < (A.width + B.width) / 2 + 0.3:  # same lane
            if dx > 0:
                follower, closing, vf, gap = 1, vb - va, vb, dx - (la + lb) / 2
            else:
                follower, closing, vf, gap = 0, va - vb, va, -dx - (la + lb) / 2
            if closing > 1e-3 and gap > 0:
                self.min_ttc = min(self.min_ttc, gap / closing)
            if vf > 1.0 and gap > 0:
                self.min_thw = min(self.min_thw, gap / vf)
        self.meas.push(t, (va, vb), lat_a=lat, follower=follower, gap=gap, closing=closing,
                       contact=self.contact)
        if getattr(self.args, "trace", False):
            self.trace.append((round(t, 2), round(xa - self.xa0, 2), round(va, 3), round(xb - self.xb0, 2), round(vb, 3),
                               round(self.lat(), 3), self.decide, self.dropping, round(aa, 2), round(ab, 2)))
        # merged: A's lane change is complete by its own plan (decision + prepare + lane change),
        # the same rule in the arena, swb (c2b.py) and C1, not a measured lateral threshold
        t_prep = self.pa["t_prep"] if self.pa is not None else cap.t_prep
        merged = self.decide is not None and t >= self.decide + t_prep + cap.t_lc
        waiting = self.decide is None and (self.dropping or job.get("a") == C1.GO_IF_YIELD or self.pa is not None)
        if self.contact or t >= C1.T_MAX or (t >= HORIZON and (merged or not waiting and self.decide is None)):
            self.finish()

    def finish(self):
        xa, ya, _, _ = self.A.state()
        xb, _, _, _ = self.B.state()
        t = self.ar.t - self.t0  # the run lasts until A's merge is complete (C1.T_MAX at most)
        free_a, free_b = self.va0 * t, self.vb0 * t  # as a share of free flow over HORIZON (C1)
        # progress along the route: the arc length covered (measures), not x
        lost_a = min(1.0, max(0.0, (free_a - self.meas.s[0]) / max(self.va0 * HORIZON, 1.0)))
        lost_b = min(1.0, max(0.0, (free_b - self.meas.s[1]) / max(self.vb0 * HORIZON, 1.0)))
        got_in = self.lat() >= 0.9
        if self.contact:
            lost_a = lost_b = 1.0
        elif not got_in:
            lost_a = 1.0
        m = self.meas
        m.pet = m._pet()
        o = C1.Outcome((lost_a, lost_b), m.ttc2d, m.thw, self.contact,
                       (self.peak[0], self.peak[1]), got_in and not self.dropping, "",
                       m.clear, m.pet, tuple(m.lat_peak))
        pref, cap = C1.Preference(), C1.TYPES["car"]
        self.rec = {**self.job, "road": self.road, "d0_real": round(self.d0_real, 2),
                    "dv_real": round(self.dv_real, 2), "vb_real": round(self.vb0, 2),
                    "react_a": round(self.react_a, 3), "react_b": round(self.react_b, 3),
                    "uA": round(C1.utility(o, 0, cap, pref), 4), "uB": round(C1.utility(o, 1, cap, pref), 4),
                    "lostA": round(lost_a, 4), "lostB": round(lost_b, 4),
                    "ttc": round(self.min_ttc, 3) if math.isfinite(self.min_ttc) else "inf",
                    "thw": round(self.min_thw, 3) if math.isfinite(self.min_thw) else "inf",
                    "contact": int(self.contact), "got_in": int(got_in),
                    "ahead": int(got_in and not self.dropping and xa > xb),  # merged in front of B
                    "peakA": round(self.peak[0], 2), "peakB": round(self.peak[1], 2),
                    "y_end_A": round(ya, 2), "t_run": round(t, 2), **self.meas.result(self.decide)}
        if getattr(self.args, "trace", False):
            self.rec["trace"] = self.trace
        self.phase = "done"


B_PROFILES = (C1.PRE_YIELD, "react_yield", "react_hold")


def jobs(args, rng):
    if args.grid:
        # C1's map, coarser: d0 -6..30 m step 2, dv -4..4 m/s step 1
        ctx = list(itertools.product([float(d) for d in range(-6, 31, 2)], [float(v) for v in range(-4, 5)]))
    else:
        d0s = [4.0, 8.0, 12.0, 20.0] if args.contexts >= 8 else [8.0, 16.0]
        dvs = [0.0, 2.0] if args.contexts >= 8 else [0.0]
        ctx = list(itertools.product(d0s, dvs))[: args.contexts]
    i, n = (int(x) for x in args.shard.split("/"))
    ctx = ctx[i::n]
    out = []
    for (d0, dv), a, b in itertools.product(ctx, C1.A_STRATS, B_PROFILES):
        for s in range(args.seeds):
            out.append({"d0": d0, "dv": dv, "vb": args.vb, "a": a, "b": b, "seed": s})
    # spread one context's runs over the batch so a batch mixes contexts (no effect on results)
    return out


def cutin(args):
    rng = random.Random(args.seed)
    work = jobs(args, rng)
    t_setup = time.monotonic()
    ar = Arena(args.port, args.k)
    rows, tick_wall = [], []
    try:
        cars = []
        for r in range(args.k):
            b = ar.spawn(r, -1, 30.0)
            a = ar.spawn(r, -2, 60.0)
            cars.append((a, b))
        ar.put_physics([x for p in cars for x in p])
        cars = [(Car(ar, a), Car(ar, b)) for a, b in cars]
        setup_s = time.monotonic() - t_setup
        t_run = time.monotonic()
        rows = run_jobs(ar, cars, work, rng, args, tick_wall, log=True)
        run_s = time.monotonic() - t_run
    finally:
        ar.close()
    return write_summary(args, rows, setup_s, run_s, tick_wall)


def setup_cars(ar, k):
    cars = []
    for r in range(k):
        b = ar.spawn(r, -1, 30.0)
        a = ar.spawn(r, -2, 60.0)
        cars.append((a, b))
    ar.put_physics([x for p in cars for x in p])
    return [(Car(ar, a), Car(ar, b)) for a, b in cars]


def run_jobs(ar, cars, work, rng, args, tick_wall=None, log=False):
    """Run jobs K at a time on the arena's roads; one result row per job, in job order."""
    rows = []
    queue = deque(work)
    t_run = time.monotonic()
    while queue:
        pairs = []
        for r in range(args.k):
            if not queue:
                break
            p = Pair(ar, r, cars[r][0], cars[r][1], queue.popleft(), rng, args)
            p.place()
            pairs.append(p)
        for _ in range(3):  # let the parked cars settle on the road
            for p in pairs:
                for c in (p.A, p.B):
                    c.a.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            ar.tick()
        for p in pairs:
            p.tw = ar.t
        while any(p.phase != "done" for p in pairs):
            for p in pairs:
                if p.phase == "warmup":
                    p.step_warmup()
                elif p.phase == "run":
                    p.step_run()
                else:
                    p.A.drive(-2.0, p.yA0 if not p.rec["got_in"] else p.yB)
                    p.B.drive(-2.0, p.yB)
            w0 = time.monotonic()
            ar.tick()
            if tick_wall is not None:
                tick_wall.append(time.monotonic() - w0)
        rows += [p.rec for p in pairs]
        if log:
            print(f"  {len(rows)}/{len(work)} runs, {time.monotonic() - t_run:.0f} s", flush=True)
    return rows


def write_summary(args, rows, setup_s, run_s, tick_wall):
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    tick_wall.sort()
    n = len(tick_wall)
    per_run_sim = WARMUP + HORIZON
    summary = {
        "k": args.k, "runs": len(rows), "setup_s": round(setup_s, 1), "run_s": round(run_s, 1),
        "runs_per_hour": round(len(rows) / run_s * 3600, 0),
        "sim_s_per_run_min": per_run_sim,
        "realtime_factor": round(len(rows) / args.k * per_run_sim / run_s, 2),
        "tick_wall_ms": {"p50": round(1000 * tick_wall[n // 2], 2), "p95": round(1000 * tick_wall[int(n * .95)], 2),
                         "max": round(1000 * tick_wall[-1], 2)},
        "ticks": n,
        "context_error": {
            "d0_max_abs": round(max(abs(r["d0_real"] - r["d0"]) for r in rows), 2),
            "dv_max_abs": round(max(abs(r["dv_real"] - r["dv"]) for r in rows), 2)},
        "obs": {"delay": args.obs_delay, "sigma_x": args.obs_sigma_x, "sigma_v": args.obs_sigma_v,
                "reaction_jitter": args.reaction_jitter},
    }
    return summary


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--parity", action="store_true")
    p.add_argument("--k", type=int, default=1, help="pairs per world")
    p.add_argument("--contexts", type=int, default=8)
    p.add_argument("--seeds", type=int, default=1)
    p.add_argument("--grid", action="store_true", help="C1's grid (171 contexts) instead of --contexts")
    p.add_argument("--shard", default="0/1", help="i/n: this process's share of the contexts")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--vb", type=float, default=12.0)
    p.add_argument("--obs-delay", type=float, default=0.19, help="s; E2: 0.19 p50, 0.21 p95 (CenterPoint)")
    p.add_argument("--obs-sigma-x", type=float, default=0.3, help="m; E2: tracked position error p50 0.28")
    p.add_argument("--obs-sigma-v", type=float, default=0.2, help="m/s; E2: tracked speed error p50 0.13")
    p.add_argument("--reaction-jitter", type=float, default=0.2)
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)
    res = parity(args) if args.parity else cutin(args)
    name = "parity.json" if args.parity else f"summary_k{args.k}.json"
    with open(os.path.join(args.out, name), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "vehicle_check_steps"}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
