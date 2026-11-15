#!/usr/bin/env python3
"""Roadmap 078 EV8: full-brake bounds per class, measured in CARLA with no Autoware.

Each run drives one blueprint on a long straight up to v0 on its own throttle (no velocity
injection: acb's probe_longitudinal.py measured a 1.34 m/s^2 bias from one), holds v0 for
HOLD_S, then commands throttle 0 and brake 1 and records every tick until the car stops.
Ticks are swb's: DT 0.05 s, physics substeps of DT/16, synchronous.

  scripts/coop/ev8_brake.py --port 2010 --out <dir> [--surfaces dry,mu0.6] [--speeds 6,10,14]
      [--only <bp,...>] [--weather-check]

Per run (one JSON line in <out>/runs.jsonl):
- dead_s: from the command to the first tick whose deceleration reaches DEAD_A (linear
  interpolation between ticks).
- a_mean: v0^2 / (2 d_brake), d_brake the distance from that moment to standstill.
- a_eff: v0^2 / (2 d_stop), d_stop the distance from the command (dead time included).
- a_peak: the largest deceleration over PEAK_WIN s, ticks below V_STOP_ART excluded (the last
  ticks of a stop discretise the stop itself).
- d_stop, t_stop.

Surfaces: dry (ClearNoon); wet_<preset> (a CARLA weather preset, the same tyres; wet_max is
wetness, precipitation and deposits at 100); mu<k> (every wheel's tire_friction times k,
ClearNoon); f<x> (every wheel's tire_friction set to x, ClearNoon). CARLA 0.9.16's tyre friction
does not follow the weather (--weather-check), so wet is mu0.6. The spec Model 3 is vehicle.tesla.model3 with
sim_vehicle_spec's carla_physics.yaml applied ("spec.model3"); its `+lags` variant puts the
spec's command_lags on the brake pedal (a dead time, then a first-order rise), as acb_bridge does.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import carla

DT = 0.05
SUBSTEPS = 16
HOLD_S = 1.5
HOLD_LOOSE_S = 8.0   # s of holding after which |v - v0| < 0.3 is enough (gear hunting)
DEAD_A = 1.0          # m/s^2: braking has begun
PEAK_WIN = 0.25       # s
V_STOP = 0.05         # m/s: stopped
V_STOP_ART = 0.5      # m/s: below this the per-tick decel is the stop's discretisation
RUNUP_MAX_S = 60.0
TOWN = "Town04"
R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SPEC = os.path.join(R, "src/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml")
CLASS_OF_BASE = {"car": "car", "truck": "truck_van", "van": "truck_van", "bus": "truck_van",
                 "motorcycle": "two_wheeler", "bicycle": "two_wheeler"}


def straight(m, length=300.0):
    """The spawn point with the straightest LENGTH m ahead (largest heading change, deg)."""
    best = None
    for sp in m.get_spawn_points():
        w = m.get_waypoint(sp.location)
        y0, dev, s, ok = w.transform.rotation.yaw, 0.0, 0.0, True
        while s < length:
            n = w.next(5.0)
            if not n:
                ok = False
                break
            w = min(n, key=lambda c: abs((c.transform.rotation.yaw - y0 + 180) % 360 - 180))
            dev = max(dev, abs((w.transform.rotation.yaw - y0 + 180) % 360 - 180))
            s += 5.0
        if ok and (best is None or dev < best[0]):
            best = (dev, sp)
    return best


def fwd_speed(v, yaw=None):
    """Speed along the body's heading, or along a fixed horizontal heading (deg) when given: a
    car that pitches or spins under braking keeps being measured along the road."""
    vel = v.get_velocity()
    if yaw is not None:
        r = math.radians(yaw)
        return vel.x * math.cos(r) + vel.y * math.sin(r)
    f = v.get_transform().get_forward_vector()
    return vel.x * f.x + vel.y * f.y + vel.z * f.z


def steer_to(v, m, look=8.0):
    tf = v.get_transform()
    w = m.get_waypoint(tf.location)
    n = w.next(look)
    if not n:
        return 0.0
    tgt = n[0].transform.location
    yaw = math.radians(tf.rotation.yaw)
    dx, dy = tgt.x - tf.location.x, tgt.y - tf.location.y
    ang = math.atan2(dy, dx) - yaw
    ang = math.atan2(math.sin(ang), math.cos(ang))
    return max(-1.0, min(1.0, 2.0 * ang))


def metrics(trace, v0):
    """trace: [(t since command, v, s since command)]."""
    a = [(trace[i][0], -(trace[i][1] - trace[i - 1][1]) / DT, trace[i][1]) for i in range(1, len(trace))]
    dead = None
    for i, (t, ai, _) in enumerate(a):
        if ai >= DEAD_A:
            if i == 0:
                dead = t - DT * (ai - DEAD_A) / max(ai, 1e-9)
            else:
                tp, ap = a[i - 1][0], a[i - 1][1]
                dead = tp + (t - tp) * (DEAD_A - ap) / max(ai - ap, 1e-9)
            break
    stop = next((p for p in trace if p[1] <= V_STOP), None)
    if dead is None or stop is None:
        return {"stopped": stop is not None}
    s_dead = next(p[2] for p in trace if p[0] >= dead)
    d_stop, t_stop = stop[2], stop[0]
    k = max(1, int(round(PEAK_WIN / DT)))
    peak = max(((trace[i - k][1] - trace[i][1]) / (k * DT) for i in range(k, len(trace)) if trace[i][1] >= V_STOP_ART),
               default=None)
    return {"stopped": True, "dead_s": round(dead, 4), "d_stop": round(d_stop, 3), "t_stop": round(t_stop, 3),
            "a_mean": round(v0 * v0 / (2 * max(d_stop - s_dead, 1e-6)), 3), "a_eff": round(v0 * v0 / (2 * d_stop), 3),
            "a_peak": None if peak is None else round(peak, 3)}


class Bench:
    def __init__(self, port):
        self.c = carla.Client("localhost", port)
        self.c.set_timeout(300)
        self.w = self.c.get_world()
        if not self.w.get_map().name.endswith(TOWN):
            self.w = self.c.load_world(TOWN)
        s = self.w.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = True, DT
        s.substepping, s.max_substep_delta_time, s.max_substeps = True, DT / SUBSTEPS, SUBSTEPS
        self.w.apply_settings(s)
        self.w.tick()
        self.m = self.w.get_map()
        dev, self.start = straight(self.m)
        self.road = {"town": TOWN, "heading_dev_deg_300m": round(dev, 2),
                     "start": [round(self.start.location.x, 2), round(self.start.location.y, 2)]}
        self.lib = self.w.get_blueprint_library()

    def weather(self, name):
        if name == "dry":
            self.w.set_weather(carla.WeatherParameters.ClearNoon)
        elif name == "max":
            self.w.set_weather(carla.WeatherParameters(cloudiness=100, precipitation=100, precipitation_deposits=100,
                                                       wetness=100, sun_altitude_angle=45, wind_intensity=50))
        else:
            self.w.set_weather(getattr(carla.WeatherParameters, name))
        self.w.tick()

    def run(self, bp_id, v0, surface, lags=None):
        name = bp_id
        spec = bp_id.startswith("spec.")
        bp = self.lib.find("vehicle.tesla.model3" if spec else bp_id)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "ev8")
        tf = carla.Transform(self.start.location + carla.Location(z=0.4), self.start.rotation)
        v = self.w.try_spawn_actor(bp, tf)
        if v is None:
            return {"bp": name, "error": "spawn"}
        try:
            self.w.tick()
            if spec:
                sys.path.insert(0, os.path.join(R, "src/sim_vehicle_spec"))
                from sim_vehicle_spec import carla_physics as CP
                CP.apply(v, CP.load(SPEC))
            self.weather(surface[4:] if surface.startswith("wet_") else "dry")
            mu = float(surface[2:]) if surface.startswith("mu") else None
            fabs = float(surface[1:]) if surface.startswith("f") else None
            pc = v.get_physics_control()
            if mu is not None or fabs is not None:
                ws = pc.wheels
                for wh in ws:
                    wh.tire_friction = wh.tire_friction * mu if mu is not None else fabs
                pc.wheels = ws
                v.apply_physics_control(pc)
                pc = v.get_physics_control()
            fric = [round(wh.tire_friction, 3) for wh in pc.wheels]
            v.apply_control(carla.VehicleControl(hand_brake=True))
            for _ in range(20):
                self.w.tick()
            # run-up on its own throttle, then hold v0
            t, held, integ, ramp = 0.0, 0.0, 0.0, True
            vmax, v_mark, t_mark, t_hold = 0.0, 0.0, 0.0, 0.0
            while held < HOLD_S:
                sp = fwd_speed(v)
                e = v0 - sp
                if ramp and sp < v0 - 0.3:
                    thr, brk = 1.0, 0.0
                else:
                    ramp = False
                    integ = max(-3.0, min(3.0, integ + e * DT))
                    u = 0.2 + 0.8 * e + 0.6 * integ
                    thr, brk = max(0.0, min(1.0, u)), 0.0
                    held = held + DT if abs(e) < 0.15 else 0.0
                    t_hold = t_hold + DT
                    if t_hold > HOLD_LOOSE_S and abs(e) < 0.3:
                        held = HOLD_S
                v.apply_control(carla.VehicleControl(throttle=thr, brake=brk, steer=steer_to(v, self.m), hand_brake=False))
                self.w.tick()
                t += DT
                vmax = max(vmax, sp)
                if sp > v_mark + 0.05:
                    v_mark, t_mark = sp, t
                if t > RUNUP_MAX_S or (ramp and t - t_mark > 4.0):
                    return {"bp": name, "v0_cmd": v0, "surface": surface, "error": f"runup: top {vmax:.2f} m/s"}
            yaw0 = v.get_transform().rotation.yaw
            vs = fwd_speed(v, yaw0)
            loc0 = v.get_location()
            trace, s, tb, last = [(0.0, vs, 0.0)], 0.0, 0.0, loc0
            att = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
            while True:
                if lags:
                    dead, tau = lags
                    b = 0.0 if tb < dead else 1.0 - math.exp(-(tb - dead + DT) / tau)
                else:
                    b = 1.0
                v.apply_control(carla.VehicleControl(throttle=0.0, brake=b, steer=steer_to(v, self.m)))
                self.w.tick()
                tb += DT
                loc = v.get_location()
                s += loc.distance(last)
                last = loc
                sp = fwd_speed(v, yaw0)
                rot = v.get_transform().rotation
                att = {"pitch": max(att["pitch"], abs(rot.pitch)), "roll": max(att["roll"], abs(rot.roll)),
                       "yaw": max(att["yaw"], abs((rot.yaw - yaw0 + 180) % 360 - 180))}
                trace.append((round(tb, 3), round(sp, 4), round(s, 4)))
                if sp <= V_STOP or tb > 20.0:
                    break
            out = {"bp": name, "base_type": bp.get_attribute("base_type").as_str() if bp.has_attribute("base_type") else "",
                   "v0_cmd": v0, "v0": round(vs, 3), "surface": surface, "lags": lags, "friction": fric,
                   "runup_s": round(t, 2), **metrics(trace, vs)}
            out["max_attitude_deg"] = {k: round(x, 2) for k, x in att.items()}
            out["trace"] = trace
            return out
        finally:
            v.destroy()
            self.w.tick()


def blueprints(lib):
    out = []
    for bp in lib.filter("vehicle.*"):
        bt = bp.get_attribute("base_type").as_str().lower() if bp.has_attribute("base_type") else ""
        if bt in CLASS_OF_BASE:
            out.append((bp.id, bt))
    return sorted(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=2010)
    ap.add_argument("--out", required=True)
    ap.add_argument("--speeds", default="6,10,14")
    ap.add_argument("--surfaces", default="dry,mu0.6")
    ap.add_argument("--only")
    ap.add_argument("--weather-check", action="store_true",
                    help="spec.model3 and two others at 10 m/s, dry against every wet preset")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    B = Bench(a.port)
    bps = blueprints(B.lib)
    json.dump({"road": B.road, "dt": DT, "substeps": SUBSTEPS, "blueprints": bps,
               "carla": B.c.get_server_version()}, open(os.path.join(a.out, "setup.json"), "w"), indent=1)
    print("road", B.road, "blueprints", len(bps), flush=True)
    jobs = []
    if a.weather_check:
        wets = ["dry", "wet_WetNoon", "wet_MidRainyNoon", "wet_HardRainNoon", "wet_SoftRainNoon", "wet_max"]
        for bp in ["spec.model3", "vehicle.carlamotors.european_hgv", "vehicle.yamaha.yzf"]:
            for s in wets:
                jobs.append((bp, 10.0, s, None))
        jobs.append(("spec.model3", 10.0, "dry", None))  # determinism
        fn = "weather.jsonl"
    else:
        ids = a.only.split(",") if a.only else ["spec.model3"] + [b for b, _ in bps]
        for s in a.surfaces.split(","):
            for bp in ids:
                for v0 in (float(x) for x in a.speeds.split(",")):
                    jobs.append((bp, v0, s, None))
            if not a.only:
                import yaml
                cl = yaml.safe_load(open(SPEC))["command_lags"]
                for v0 in (float(x) for x in a.speeds.split(",")):
                    jobs.append(("spec.model3", v0, s, (cl["brake_dead_time"], cl["brake_time_constant"])))
        fn = "runs.jsonl"
    with open(os.path.join(a.out, fn), "a") as f:
        for i, (bp, v0, s, lags) in enumerate(jobs):
            t0 = time.time()
            r = B.run(bp, v0, s, lags)
            r["wall_s"] = round(time.time() - t0, 1)
            f.write(json.dumps(r) + "\n")
            f.flush()
            print(i + 1, len(jobs), {k: r.get(k) for k in ("bp", "v0", "surface", "lags", "dead_s", "a_mean", "a_peak", "d_stop", "error")},
                  flush=True)
    s = B.w.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = False, None
    B.w.apply_settings(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
