#!/usr/bin/env python3
"""Roadmap 074 E4: heavy vehicles on CARLA 0.9.16 -- which blueprints, what their physics give.

`sim_vehicle_spec` holds two cars (tesla_model3, drift_mustang) and no truck or bus. This tool
measures the heavy blueprints CARLA ships, with their stock physics (nothing applied), on the
arena's world (arena.Arena: generated straight roads, scensim_carla_world's timing, 0.05 s ticks and
16 substeps), one car per road, all in parallel:

  physics   what get_physics_control() says: mass, drag, engine (max rpm, torque curve), gears,
            wheels (radius, brake torque, friction, steer), the body box, wheelbase, track
  t1        full throttle from rest (E1's 0-12 m/s time, and 0-5, 0-10, the speed at 20 s), then
            at 12 m/s full brake to a stop (E1's distance, mean and peak deceleration over
            arena.PEAK_WINDOW) and the brake step's dead time and t63
  throttle  each pedal 0.1..0.9 held from rest: acceleration against speed (the accel map a spec
            needs, the Model 3's columns), and the 0.5 step's t63
  brake     each pedal 0.1..1.0 from 16 m/s (set_target_velocity, then held): deceleration
            against speed (the brake map), and coasting (pedal 0)

The Model 3 is measured the same way twice, stock and with the spec's physics applied
(carla_physics.apply), as the yardstick: E1 measured the latter through acb's controller.

Outputs: <out>/e4.json, <out>/e4_summary.txt and per blueprint <out>/maps/<bp>/{accel,brake}_map.csv
in the format of autoware_sim_vehicle's maps (arena.PedalMap reads them). They are measurements
for a spec, not a spec: nothing is written into sim_vehicle_spec.

Usage:
  scripts/coop/e4_heavy.py --port 2600 --out <dir>
  scripts/coop/e4_heavy.py --selftest
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

HEAVY = ("vehicle.carlamotors.carlacola", "vehicle.carlamotors.european_hgv", "vehicle.carlamotors.firetruck",
         "vehicle.mitsubishi.fusorosa", "vehicle.tesla.cybertruck", "vehicle.mercedes.sprinter",
         "vehicle.ford.ambulance")
REF = "vehicle.tesla.model3"
SPEEDS = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 11.0, 14.0, 17.0, 20.0, 24.0)  # the Model 3 maps' columns
THROTTLES = tuple(round(0.1 * i, 1) for i in range(1, 11))
BRAKES = tuple(round(0.1 * i, 1) for i in range(0, 11))
DT = 0.05
PEAK_WINDOW = 0.5


# --- the analysis (no CARLA) -------------------------------------------------------------------
def accel_of(trace, half=0.25):
    """(t, v, a) with a the central difference of v over +-half s."""
    out = []
    k = max(1, int(round(half / DT)))
    for i in range(k, len(trace) - k):
        t, v = trace[i][0], trace[i][1]
        a = (trace[i + k][1] - trace[i - k][1]) / (trace[i + k][0] - trace[i - k][0])
        out.append((t, v, a))
    return out


def binned(samples, speeds=SPEEDS, t_skip=0.3, width=0.5):
    """Mean acceleration per map speed column (samples within +-width of it, after t_skip); a
    column the run never reached holds the nearest reached column's value (marked in `filled`)."""
    vals, filled = {}, []
    for s in speeds:
        xs = [a for t, v, a in samples if t >= t_skip and abs(v - s) <= width]
        if s == 0.0 and not xs:
            xs = [a for t, v, a in samples if t >= t_skip and abs(v - 1.0) <= width]
        vals[s] = st.mean(xs) if xs else None
    have = [s for s in speeds if vals[s] is not None]
    for s in speeds:
        if vals[s] is None:
            filled.append(s)
            vals[s] = vals[min(have, key=lambda h: abs(h - s))] if have else 0.0
    return [vals[s] for s in speeds], filled


def peak_decel(trace, window=PEAK_WINDOW):
    """The largest speed drop over any `window` s, per `window` (arena.Pair's peak braking)."""
    best, j = 0.0, 0
    for i in range(len(trace)):
        while trace[i][0] - trace[j][0] > window + 1e-9:
            j += 1
        if trace[i][0] - trace[j][0] >= window - 1e-9:
            best = max(best, (trace[j][1] - trace[i][1]) / window)
    return best


def t63(samples, t_on, ref):
    """Time from t_on until |a| first reaches 63 % of |ref| (None if it never does)."""
    for t, _, a in samples:
        if t >= t_on and abs(a) >= 0.63 * abs(ref):
            return round(t - t_on, 3)
    return None


def dead_time(samples, t_on, thr=0.3):
    for t, _, a in samples:
        if t >= t_on and abs(a) >= thr:
            return round(t - t_on, 3)
    return None


def write_map(path, kind, bp, rows):
    """rows: [(pedal, [accel per SPEEDS])]; the format of autoware_sim_vehicle's maps."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(f"# {kind} response of {bp} on CARLA 0.9.16 (stock physics), measured by scripts/coop/e4_heavy.py\n")
        f.write("# CARLA tick 0.050 s, physics substeps of 0.0031 s (max 16) -- the timing scensim_carla_world uses\n")
        f.write("# Rows: pedal position. Columns: vehicle speed in m/s.\n")
        f.write("# Cells: measured longitudinal acceleration in m/s^2.\n")
        f.write("default," + ",".join(f"{s:.1f}" for s in SPEEDS) + "\n")
        f.write("0.00," + ",".join("0.000" for _ in SPEEDS) + "\n")
        for p, accs in rows:
            if p > 0:
                f.write(f"{p:.2f}," + ",".join(f"{a:.3f}" for a in accs) + "\n")


def monotone(rows, sign):
    """Pedal maps must be monotone in the pedal at every speed (acb's lookup assumes it): each
    row at least as strong as the one before, sign +1 (accel) or -1 (brake)."""
    out, prev = [], None
    for p, accs in rows:
        if prev is not None:
            accs = [max(a, b) if sign > 0 else min(a, b) for a, b in zip(accs, prev)]
        out.append((p, accs))
        prev = accs
    return out


# --- CARLA -------------------------------------------------------------------------------------
def physics_of(actor):
    import carla  # noqa: F401
    pc = actor.get_physics_control()
    d = {}
    for k in ("mass", "drag_coefficient", "max_rpm", "moi", "final_ratio", "gear_switch_time", "clutch_strength",
              "use_gear_autobox", "damping_rate_full_throttle"):
        if hasattr(pc, k):
            v = getattr(pc, k)
            d[k] = round(v, 4) if isinstance(v, float) else v
    try:
        d["torque_curve"] = [(round(p.x, 1), round(p.y, 1)) for p in pc.torque_curve]
        d["max_torque_nm"] = round(max(p.y for p in pc.torque_curve), 1)
    except Exception:
        pass
    try:
        d["forward_gears"] = [round(g.ratio, 3) for g in pc.forward_gears]
    except Exception:
        pass
    try:
        com = pc.center_of_mass
        d["center_of_mass"] = (round(com.x, 3), round(com.y, 3), round(com.z, 3))
    except Exception:
        pass
    ws = []
    for w in pc.wheels:
        ws.append({k: round(getattr(w, k), 3) for k in ("tire_friction", "damping_rate", "max_steer_angle", "radius",
                                                         "max_brake_torque", "max_handbrake_torque") if hasattr(w, k)})
        ws[-1]["pos_cm"] = (round(w.position.x, 1), round(w.position.y, 1))
    d["wheels"] = ws
    xs = [w["pos_cm"] for w in ws]
    # positions are world coordinates (cm) of a car at yaw 0: wheelbase along x, track along y
    d["wheelbase_m"] = round((max(p[0] for p in xs) - min(p[0] for p in xs)) / 100.0, 3)
    front = [p for p in xs if p[0] >= max(q[0] for q in xs) - 1.0]
    d["track_m"] = round((max(p[1] for p in front) - min(p[1] for p in front)) / 100.0, 3)
    bb = actor.bounding_box
    d["length_m"], d["width_m"], d["height_m"] = (round(2 * bb.extent.x, 3), round(2 * bb.extent.y, 3),
                                                  round(2 * bb.extent.z, 3))
    d["n_wheels"] = len(ws)
    return d


class Rig:
    def __init__(self, port, cars, apply_spec):
        import arena as AR
        import carla
        self.carla, self.AR = carla, AR
        AR.ROAD_LEN = 1500.0
        self.ar = AR.Arena(port, len(cars))
        lib = self.ar.world.get_blueprint_library()
        self.cars = []
        for r, (name, bp_id) in enumerate(cars):
            y, z = self.ar.lane_y(r, -1)
            tf = carla.Transform(carla.Location(x=30.0, y=y, z=z + 1.0), carla.Rotation(yaw=0.0))
            a = self.ar.world.spawn_actor(lib.find(bp_id), tf)
            self.ar.actors.append(a)
            self.cars.append({"name": name, "bp": bp_id, "a": a, "start": tf, "y": y})
        for _ in range(10):
            self.ar.tick()
        for c in self.cars:
            if c["name"] in apply_spec:
                from sim_vehicle_spec import carla_physics
                carla_physics.apply(c["a"], self.ar.doc)
        for _ in range(10):
            self.ar.tick()

    def state(self, c):
        tf, v = c["a"].get_transform(), c["a"].get_velocity()
        return tf.location.x, math.hypot(v.x, v.y)

    def settle(self):
        C = self.carla
        for c in self.cars:
            c["a"].set_target_velocity(C.Vector3D(0, 0, 0))
            c["a"].set_target_angular_velocity(C.Vector3D(0, 0, 0))
            c["a"].set_transform(c["start"])
        for _ in range(30):
            for c in self.cars:
                c["a"].apply_control(C.VehicleControl(brake=1.0, hand_brake=True))
            self.ar.tick()

    def run(self, ctl, t_max, v0=0.0):
        """ctl(i, t, v, mem) -> (throttle, brake, done); one trace [(t, v, x)] per car."""
        C = self.carla
        self.settle()
        if v0:
            for c in self.cars:
                c["a"].apply_control(C.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
            self.ar.tick()
            for c in self.cars:
                c["a"].set_target_velocity(C.Vector3D(v0, 0, 0))
            self.ar.tick()
        traces = [[] for _ in self.cars]
        mem = [{} for _ in self.cars]
        done = [False] * len(self.cars)
        t = 0.0
        while t < t_max and not all(done):
            for i, c in enumerate(self.cars):
                x, v = self.state(c)
                traces[i].append((round(t, 3), v, x))
                if done[i]:
                    c["a"].apply_control(C.VehicleControl(brake=1.0))
                    continue
                th, br, dn = ctl(i, t, v, mem[i])
                done[i] = dn or x > 1300.0
                if done[i]:
                    mem[i]["to"] = t
                tf = c["a"].get_transform()
                yaw = math.radians(tf.rotation.yaw)
                steer = max(-1.0, min(1.0, -0.05 * (tf.location.y - c["y"]) - 0.8 * yaw))
                c["a"].apply_control(C.VehicleControl(throttle=th, brake=br, steer=steer, hand_brake=False))
            self.ar.tick()
            t += DT
        return traces, mem

    def close(self):
        self.ar.close()


def measure(port, out):
    cars = [(bp.split(".", 1)[1], bp) for bp in HEAVY] + [("tesla.model3", REF), ("tesla.model3+spec", REF)]
    rig = Rig(port, cars, apply_spec={"tesla.model3+spec"})
    res = {}
    try:
        for c in rig.cars:
            res[c["name"]] = {"bp": c["bp"], "physics": physics_of(c["a"])}
        # t1: full throttle from rest, full brake once at 12 m/s
        def t1(i, t, v, m):
            if "brake_at" not in m and v >= 12.0:
                m["brake_at"] = t
            if "brake_at" in m:
                return 0.0, 1.0, v < 0.05 and t > m["brake_at"] + 0.5
            if t >= 60.0:
                m["never12"] = True
                return 0.0, 1.0, True
            return 1.0, 0.0, False
        traces, mem = rig.run(t1, 90.0)
        for c, tr, m in zip(rig.cars, traces, mem):
            r = {}
            ts = lambda thr: next((t for t, v, _ in tr if v >= thr), None)
            for thr in (5.0, 10.0, 12.0):
                r[f"t_0_{thr:.0f}_s"] = ts(thr)
            r["mean_accel_0_12"] = round(12.0 / r["t_0_12_s"], 3) if r["t_0_12_s"] else None
            if "brake_at" in m:
                tb = m["brake_at"]
                br = [(t, v) for t, v, _ in tr if t >= tb]
                x0 = next(x for t, _, x in tr if t >= tb)
                x1 = tr[-1][2]
                vb = br[0][1]
                r["brake_from"] = round(vb, 3)
                r["brake_distance_m"] = round(x1 - x0, 2)
                r["brake_mean_decel"] = round(vb ** 2 / (2 * max(0.01, x1 - x0)), 3)
                r["brake_peak_decel"] = round(peak_decel(br), 3)
                samp = accel_of([(t, v) for t, v, _ in tr])
                steady = [a for t, v, a in samp if tb + 0.5 <= t and v > 2.0]
                ref = st.median(steady) if steady else -1.0
                r["brake_dead_s"] = dead_time(samp, tb)
                r["brake_t63_s"] = t63([s for s in samp if s[2] < 0], tb, ref)
            res[c["name"]]["t1"] = r
        # throttle map rows, from rest (samples until the run's end: its braking after is not the map)
        th_rows = {c["name"]: [] for c in rig.cars}
        for p in THROTTLES:
            traces, mem = rig.run(lambda i, t, v, m, p=p: (p, 0.0, v >= 24.5), 35.0)
            for c, tr, m in zip(rig.cars, traces, mem):
                samp = [s for s in accel_of([(t, v) for t, v, _ in tr]) if s[0] < m.get("to", 1e9) - 0.3]
                accs, filled = binned(samp)
                th_rows[c["name"]].append((p, accs))
                if abs(p - 0.5) < 1e-9:
                    steady = [a for t, v, a in samp if 1.0 <= t <= 3.0]
                    res[c["name"]]["throttle05_t63_s"] = t63(samp, 0.0, st.median(steady) if steady else 1.0)
                if abs(p - 1.0) < 1e-9:
                    res[c["name"]]["v_at_35s_full_throttle"] = round(tr[-1][1], 2)
                res[c["name"]].setdefault("throttle_filled", {})[str(p)] = filled
        # brake map rows: full throttle to 16 m/s (or for 40 s), then the pedal held to a stop.
        # Not set_target_velocity: it moves the body with the wheels at rest, which then skid
        # (-10 to -18 m/s^2 at any pedal)
        def brake_ctl(i, t, v, m, p):
            if "from" not in m and (v >= 16.0 or t >= 40.0):
                m["from"] = t
            if "from" not in m:
                return 1.0, 0.0, False
            return 0.0, p, v < 0.3 or t > m["from"] + 60.0
        br_rows = {c["name"]: [] for c in rig.cars}
        for p in BRAKES:
            traces, mem = rig.run(lambda i, t, v, m, p=p: brake_ctl(i, t, v, m, p), 110.0)
            for c, tr, m in zip(rig.cars, traces, mem):
                t0 = m.get("from", 0.0)
                samp = [(t - t0, v, a) for t, v, a in accel_of([(t, v) for t, v, _ in tr]) if t0 <= t < m.get("to", 1e9)]
                accs, filled = binned(samp, t_skip=0.5)
                if p == 0.0:
                    res[c["name"]]["coast_decel_at"] = {str(s): round(a, 3) for s, a in zip(SPEEDS, accs)}
                else:
                    br_rows[c["name"]].append((p, accs))
                    res[c["name"]].setdefault("brake_filled", {})[str(p)] = filled
        for c in rig.cars:
            n = c["name"]
            d = os.path.join(out, "maps", n)
            write_map(os.path.join(d, "accel_map.csv"), "Throttle", c["bp"], monotone(th_rows[n], +1))
            write_map(os.path.join(d, "brake_map.csv"), "Brake", c["bp"], monotone(br_rows[n], -1))
            res[n]["accel_map_full"] = [round(a, 3) for a in th_rows[n][-1][1]]
            res[n]["brake_map_full"] = [round(a, 3) for a in br_rows[n][-1][1]]
            res[n]["brake_map_rows_at_11"] = {str(p): round(a[SPEEDS.index(11.0)], 3) for p, a in br_rows[n]}
            res[n]["accel_map_rows_at_5"] = {str(p): round(a[SPEEDS.index(5.0)], 3) for p, a in th_rows[n]}
    finally:
        rig.close()
    json.dump(res, open(os.path.join(out, "e4.json"), "w"), indent=1)
    txt = summary(res)
    open(os.path.join(out, "e4_summary.txt"), "w").write(txt)
    print(txt)


def summary(res):
    L = ["E4: CARLA 0.9.16 heavy blueprints, stock physics (the Model 3 twice: stock and with the spec)",
         f"{'car':20s} {'mass':>7s} {'L x W':>11s} {'wb':>5s} {'whl':>3s} {'0-12 s':>7s} {'a 0-12':>7s} "
         f"{'brake m':>8s} {'mean':>6s} {'peak':>6s} {'dead':>5s} {'t63':>5s} {'v35':>5s} {'brk_trq':>8s}"]
    for n, r in res.items():
        p, t = r["physics"], r.get("t1", {})
        bt = max(w.get("max_brake_torque", 0) for w in p["wheels"])
        f = lambda x, fmt: (fmt % x) if isinstance(x, (int, float)) else "-"
        L.append(f"{n:20s} {p.get('mass', 0):7.0f} {p['length_m']:5.2f}x{p['width_m']:4.2f} {p['wheelbase_m']:5.2f} "
                 f"{p['n_wheels']:3d} {f(t.get('t_0_12_s'), '%7.2f')} {f(t.get('mean_accel_0_12'), '%7.2f')} "
                 f"{f(t.get('brake_distance_m'), '%8.2f')} {f(t.get('brake_mean_decel'), '%6.2f')} "
                 f"{f(t.get('brake_peak_decel'), '%6.2f')} {f(t.get('brake_dead_s'), '%5.2f')} {f(t.get('brake_t63_s'), '%5.2f')} "
                 f"{f(r.get('v_at_35s_full_throttle'), '%5.1f')} {bt:8.0f}")
    L.append("")
    L.append("brake map at 11 m/s, by pedal (m/s^2):")
    for n, r in res.items():
        L.append(f"  {n:20s} " + " ".join(f"{float(k):.1f}:{v:+.2f}" for k, v in r.get("brake_map_rows_at_11", {}).items()))
    L.append("accel map at 5 m/s, by pedal (m/s^2):")
    for n, r in res.items():
        L.append(f"  {n:20s} " + " ".join(f"{float(k):.1f}:{v:+.2f}" for k, v in r.get("accel_map_rows_at_5", {}).items()))
    L.append("coasting at 11 m/s (m/s^2): " + ", ".join(f"{n} {r.get('coast_decel_at', {}).get('11.0', 0):+.2f}" for n, r in res.items()))
    return "\n".join(L) + "\n"


def selftest():
    # a car at 2 m/s^2 from rest: the binned map holds 2 everywhere it reached, the rest filled
    tr = [(i * DT, min(10.0, 2.0 * i * DT)) for i in range(200)]
    accs, filled = binned(accel_of(tr), t_skip=0.0)
    assert all(abs(a - 2.0) < 1e-6 for s, a in zip(SPEEDS, accs) if s <= 8.0), accs
    assert filled == [11.0, 14.0, 17.0, 20.0, 24.0] and abs(accs[-1] - accs[SPEEDS.index(8.0)]) < 1e-9, filled
    # braking at 6 m/s^2 from 12: peak decel over 0.5 s is 6, t63 at once, dead time 0
    br = [(i * DT, max(0.0, 12.0 - 6.0 * i * DT)) for i in range(60)]
    assert abs(peak_decel(br) - 6.0) < 1e-6
    s = accel_of(br)
    assert t63(s, 0.0, -6.0) is not None and dead_time(s, 0.0) is not None
    # the map file reads back through the arena's PedalMap (acb's lookup), and is monotone
    import tempfile
    rows = monotone([(0.5, [1.0] * len(SPEEDS)), (1.0, [0.8] * len(SPEEDS))], +1)
    assert rows[1][1][0] == 1.0
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "accel_map.csv")
        write_map(p, "Throttle", "test", rows)
        import arena as AR
        pm = AR.PedalMap(p)
        assert pm.pedals == [0.0, 0.5, 1.0] and abs(pm.at(1, 3.0) - 1.0) < 1e-9 and pm.speeds == list(SPEEDS)
    print("e4_heavy selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=2600)
    ap.add_argument("--out")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    os.makedirs(a.out, exist_ok=True)
    measure(a.port, a.out)


if __name__ == "__main__":
    main()
