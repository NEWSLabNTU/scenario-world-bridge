"""Roadmap 091 W10: fitting the twin to the real car's logs, stage by stage. Pure: numpy and
scipy, and the user's component. The logs are what twin_runs.py writes (or the real car's, in
the same form); the fit never reads anything else.

1. `fit_component`: the drive unit's parameters by replay. The logged commands and ERPM go
   into the component, and its current is matched to the logged current (W11 (a)'s replay).
   No physics is involved. It gives the speed loop's gains, the current limit and the latency.
2. `fit_mass`: the mass, from M1's accelerations under the logged current and M2's coast-down.
3. `fit_steering`: the servo's offset, from M5's left and right circles.
4. `fit_tyres`: the tyres per axle, from M5 and M6, by the single-track model
   (tools/single_track.py) with PhysX's curve, corrected by the twin's own runs.
5. `fit_noise`: the sensors' noise, from the residuals of steady stretches.
"""
import math
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "component" / "roboracer_vesc"))
sys.path.insert(0, str(HERE.parent / "tools"))
import quantities as qmod  # noqa: E402

G = 9.81
# The replay steps the component as the twin does, at its physics rate (W7: 240 Hz), so the
# twin's own timing (a command reaches the component at the next physics step) is part of what
# the fitted latency reproduces.
REPLAY_DT = 1 / 240
MOTOR_RUNS = ("M1", "M2", "M3", "M3b")


def _runs(runs, manoeuvres):
    return [(s, st) for s, st in runs if s["manoeuvre"] in manoeuvres]


# --- 1. the component, by replay --------------------------------------------------------------

def replay(component_cls, car, params, streams, motor_modes, dt=REPLAY_DT):
    """The component's motor current at each `core_t`, driven by the logged commands and the
    logged ERPM (the car's speed as the VESC measured it)."""
    from sim_vehicle_ecu.signals import Sensors, VescCommand

    c = component_cls(car, params, 0)
    erpm_per_wheel = c.gear * c.p["erpm_per_motor_rad_s"]  # ERPM per wheel rad/s
    ct, cm, cv, cs = (streams[k] for k in ("cmd_t", "cmd_mode", "cmd_value", "cmd_servo"))
    kt, ke = streams["core_t"], streams["core_erpm"]
    out = np.zeros_like(kt)
    j, k, t = -1, 0, 0.0
    n = int(math.ceil(kt[-1] / dt)) + 1
    for i in range(n):
        t = (i + 1) * dt
        while j + 1 < len(ct) and ct[j + 1] <= t - dt + 1e-12:
            j += 1
        cmd = (VescCommand(motor_modes[int(cm[j])], float(cv[j]), float(cs[j]), t - dt - ct[j])
               if j >= 0 else VescCommand())
        w = float(np.interp(t - dt, kt, ke)) / erpm_per_wheel
        a = c.step(cmd, Sensors(wheel_speeds=(w,) * 4), dt)
        while k < len(kt) and kt[k] <= t + 1e-12:
            out[k] = a.telemetry["current_motor"]
            k += 1
    return out


FIT_COMPONENT = ("speed_kp", "speed_ki", "current_max", "latency")


def fit_current_max(runs, protocol_doc):
    """The current limit from M1c: a current command beyond the limit, whose logged current
    plateaus at it. None when the logs have no M1c (the limit is then not excited)."""
    vals = []
    for s, st in _runs(runs, ("M1c",)):
        t0 = float(protocol_doc["manoeuvres"]["M1c"]["segments"][0]["t"])
        t = st["core_t"]
        sel = (t > t0 + 0.15) & (t < t0 + 0.4)
        vals.append(float(np.median(st["core_current_motor"][sel])))
    return float(np.median(vals)) if vals else None


def _replay_cost(component_cls, car, params, sel, motor_modes):
    err = []
    for _, st in sel:
        pred = replay(component_cls, car, params, st, motor_modes)
        err.append(pred - st["core_current_motor"])
    return float(np.mean(np.concatenate(err) ** 2))


def fit_component(runs, component_cls, car, motor_modes, protocol_doc,
                  latencies=np.arange(0.0, 0.1001, 0.004)):
    """The drive unit's parameters by replay over the speed-loop runs (M1 to M3b):
    - the current limit from M1c's plateau;
    - the latency on a grid, the speed loop's gains fitted at each step by least squares;
    - then the latency refined around the best.
    Returns ({key: value}, rms A, notes)."""
    from roboracer_vesc.component import DEFAULTS
    sel = _runs(runs, ("M1", "M2", "M3", "M3b"))
    if not sel:
        raise ValueError("no speed-loop runs (M1 to M3b) in the logs")
    notes = []
    cmax = fit_current_max(runs, protocol_doc)
    if cmax is None:
        cmax = float(DEFAULTS["current_max"])
        notes.append("current_max: not excited (no M1c); the default kept")
    k0 = np.array([DEFAULTS["speed_kp"], DEFAULTS["speed_ki"]])

    def gains_at(latency, x0, iters):
        def cost(z):
            if np.any(z <= 0):
                return 1e9
            return _replay_cost(component_cls, car, {"speed_kp": z[0] * k0[0],
                                                     "speed_ki": z[1] * k0[1],
                                                     "current_max": cmax, "latency": latency},
                                sel, motor_modes)
        r = minimize(cost, x0, method="Nelder-Mead",
                     options={"xatol": 1e-3, "fatol": 1e-7, "maxiter": iters})
        return r.fun, r.x

    best = None
    for lat in latencies:
        f, z = gains_at(float(lat), np.ones(2), 40)
        if best is None or f < best[0]:
            best = (f, float(lat), z)
    _, lat0, z0 = best
    for lat in np.arange(max(0.0, lat0 - 0.004), lat0 + 0.0041, 0.001):
        f, z = gains_at(float(lat), z0, 80)
        if f < best[0]:
            best = (f, float(lat), z)
    f, lat, z = best
    return ({"speed_kp": float(z[0] * k0[0]), "speed_ki": float(z[1] * k0[1]),
             "current_max": cmax, "latency": lat}, math.sqrt(f), notes)


# --- 2. the mass ------------------------------------------------------------------------------

def coast(runs, protocol_doc):
    """M2's coast-down deceleration a(v) = c0 + c1 v + c2 v^2 (rolling resistance, the wheels'
    damping, drag), fitted over every M2 run's coasting samples."""
    conv = protocol_doc["conversion"]
    vs, accs = [], []
    for s, st in _runs(runs, ("M2",)):
        t0 = float(protocol_doc["manoeuvres"]["M2"]["segments"][0]["t"])
        t, v = qmod.speed(st, conv)
        sel = (t >= t0 + 0.2) & (v > 0.3)
        if sel.sum() < 10:
            continue
        # the deceleration from a smooth fit of the speed (the ERPM's noise differentiated
        # sample by sample would swamp it)
        tt, vv = t[sel], v[sel]
        p = np.polyfit(tt - tt[0], vv, 4)
        accs.append(-np.polyval(np.polyder(p), tt - tt[0]))
        vs.append(vv)
    v, a = np.concatenate(vs), np.concatenate(accs)
    A = np.stack([np.ones_like(v), v, v * v], axis=1)
    c, *_ = np.linalg.lstsq(A, a, rcond=None)
    return tuple(float(x) for x in c)


def holds(runs, protocol_doc):
    """The steady straight holds: [(speed m/s, mean motor current A)], from the last 1.5 s of
    each speed segment that is held (M1's step, the first segment of M2, M3 and M3b)."""
    out = []
    conv = protocol_doc["conversion"]
    for s, st in _runs(runs, ("M1", "M2", "M3", "M3b")):
        segs = protocol_doc["manoeuvres"][s["manoeuvre"]]["segments"]
        i = 1 if s["manoeuvre"] == "M1" else 0
        start = sum(float(x["t"]) for x in segs[:i])
        end = start + float(segs[i]["t"])
        t, v = qmod.speed(st, conv)
        sel = (t > end - 1.5) & (t < end - 0.05)
        out.append((float(v[sel].mean()), float(st["core_current_motor"][sel].mean())))
    return out


def fit_mass(runs, protocol_doc, drive_force_per_amp, wheel_inertia_mass=0.0):
    """The mass from steady holds: holding speed v, the drive force k I meets the road load,
    which the coast-down gives as (m + the wheels' inertia) a(v). So m = k I / a(v) - the
    wheels' inertia as a mass (4 I_w / r^2)."""
    c = coast(runs, protocol_doc)
    est = [drive_force_per_amp * i / (c[0] + c[1] * v + c[2] * v * v) - wheel_inertia_mass
           for v, i in holds(runs, protocol_doc) if v > 0.5]
    return float(np.median(est)), c


# --- 3. the steering ------------------------------------------------------------------------

def fit_servo_offset(runs, protocol_doc, wheel_base, servo_gain):
    """The servo's offset from M5 at 1 m/s (the kinematic regime): a car whose straight-ahead
    servo position is off steers more to one side; the offset makes the left and right radii
    agree. Returns the offset's correction to add to the protocol's."""
    est = []
    for s, st in _runs(runs, ("M5",)):
        if float(s["condition"]["speed"]) != 1.0 or s["condition"]["angle"] == "full":
            continue
        q = qmod.quantities(s, st, protocol_doc)
        angle = float(s["condition"]["angle"]) * float(s["condition"]["side"])
        actual = math.atan(wheel_base / q["radius"]) * float(s["condition"]["side"])
        # servo = gain x angle + offset_cmd; the car's angle = (servo - offset_true) / gain
        # so offset_true - offset_cmd = gain x (angle - actual)
        est.append(servo_gain * (angle - actual))
    return float(np.median(est)) if est else 0.0


# --- 5. the noise ----------------------------------------------------------------------------

def fit_noise(runs, protocol_doc):
    """Each sensor's noise, from steady stretches: the reference's position from the residual
    of a smooth fit of M5's last 3 s, the IMU's from its spread there, the ERPM's in M1's hold."""
    ref, yaw, acc, gyro, erpm, cur = [], [], [], [], [], []
    for s, st in _runs(runs, ("M5",)):
        t = st["ref_t"]
        sel = t >= t[-1] - 3.0
        for key, out in (("ref_x", ref), ("ref_y", ref), ("ref_yaw", yaw)):
            y = st[key][sel]
            tt = t[sel]
            y = np.unwrap(y) if key == "ref_yaw" else y
            p = np.polyfit(tt - tt[0], y, 6)
            out.append(np.std(y - np.polyval(p, tt - tt[0])))
        it = st["imu_t"]
        isel = it >= it[-1] - 3.0
        acc.append(np.std(st["imu_ax"][isel]))
        gyro.append(np.std(st["imu_wz"][isel]))
    for s, st in _runs(runs, ("M1",)):
        t = st["core_t"]
        sel = (t > 3.5) & (t < 4.9)
        erpm.append(np.std(st["core_erpm"][sel]))
        cur.append(np.std(st["core_current_motor"][sel]))
    med = lambda xs: float(np.median(xs)) if xs else None  # noqa: E731
    return {"ref_pos": med(ref), "ref_yaw": med(yaw), "imu_accel": med(acc),
            "imu_gyro": med(gyro), "erpm": med(erpm), "current": med(cur)}


# --- 4. the tyres, per axle ------------------------------------------------------------------

def circles(runs, protocol_doc, max_steer):
    """M5's steady circles, the repetitions averaged: [(tire angle, side, speed, radius)], the
    angle as commanded (rad, unsigned)."""
    groups = {}
    for s, st in _runs(runs, ("M5",)):
        c = s["condition"]
        angle = max_steer if c["angle"] == "full" else float(c["angle"])
        q = qmod.quantities(s, st, protocol_doc)
        if q.get("radius"):
            key = (angle, int(c["side"]), float(c["speed"]))
            groups.setdefault(key, []).append((q["radius"], st))
    out = []
    for (angle, side, speed), vals in sorted(groups.items()):
        # the speed the car held, from its ERPM (the commanded one may not be reached)
        vs = np.mean([qmod.speed(st, protocol_doc["conversion"])[1][-75:].mean()
                      for _, st in vals])
        out.append((angle, side, round(float(vs), 1), float(np.mean([r for r, _ in vals]))))
    return out


def grip(runs, protocol_doc):
    """M6's lateral acceleration at the slide, the mean over its runs."""
    vals = [qmod.quantities(s, st, protocol_doc)["lateral_acceleration_at_slide"]
            for s, st in _runs(runs, ("M6",))]
    return float(np.mean(vals)) if vals else None


def circle_key(angle, side, speed):
    return f"{angle:g},{int(side)},{speed:.1f}"


def _geometry(car):
    lf = car["inertia"]["com_front_fraction"] * car["geometry"]["wheel_base"]
    return {"m": car.get("mass", car["inertia"]["mass"]), "lf": lf,
            "lr": car["geometry"]["wheel_base"] - lf}


def tyre_model(x, circ, car, offset_angle=0.0):
    """The single-track model's radii for `circ` ([(angle, side, speed, ...)]) and its grip
    limit at 0.2 rad, for tyres x = (cf, mf, cr, mr) in PhysX's curve: ({key: radius}, ay)."""
    import single_track as stm
    from tyre_compare import physx
    cf, mf, cr, mr = x
    tf, tr = physx(cf, mf), physx(cr, mr)
    geo = _geometry(car)
    radii = {}
    for c in circ:
        angle, side, v = c[0], c[1], c[2]
        s = stm.steady_cont(geo, tf, tr, angle + side * offset_angle, v)
        radii[circle_key(angle, side, v)] = abs(s["radius"]) if s else None
    m6 = stm.m6(geo, tf, tr, 0.2)
    return radii, (m6["ay"] if m6 else None)


def fit_tyres(circ, ay_slide, car, offset_angle=0.0, correction=None, start=(6.0, 0.9, 6.0, 0.9)):
    """Each axle's (cornering stiffness per rad per N of load, peak friction) of PhysX's curve
    that best gives the measured circles and grip limit, by the single-track model.
    `offset_angle`: the steering's offset, rad, added to every commanded angle (the servo's).
    `correction`: {circle_key: twin radius / model radius, "m6": twin ay / model ay}, from the
    twin's runs at the previous fit: what the model misses of the twin."""
    corr = correction or {}

    def predict(x):
        radii, ay = tyre_model(x, circ, car, offset_angle)
        out = [radii[circle_key(*c[:3])] * corr.get(circle_key(*c[:3]), 1.0)
               if radii[circle_key(*c[:3])] else None for c in circ]
        return out, (ay * corr.get("m6", 1.0) if ay else None)

    def cost(x):
        if min(x) <= 0.05:
            return 1e6
        radii, ay = predict(x)
        e = [((r - c[3]) / c[3]) if r else 1.0 for r, c in zip(radii, circ)]
        e6 = ((ay - ay_slide) / ay_slide) if (ay and ay_slide) else 0.0
        return float(np.sum(np.square(e)) + 4 * e6 * e6)

    r = minimize(cost, np.array(start), method="Nelder-Mead",
                 options={"xatol": 1e-3, "fatol": 1e-8, "maxiter": 600})
    radii, ay = predict(r.x)
    return [float(v) for v in r.x], {"radii": radii, "ay_slide": ay, "cost": float(r.fun)}


def correction_of(twin_circ, twin_ay, x, car, offset_angle):
    """{circle_key: twin / model, "m6": twin / model} at tyres x: what the model misses."""
    radii, ay = tyre_model(x, twin_circ, car, offset_angle)
    out = {}
    for c in twin_circ:
        k = circle_key(*c[:3])
        if radii.get(k):
            out[k] = c[3] / radii[k]
    if ay and twin_ay:
        out["m6"] = twin_ay / ay
    return out


def axle_points(runs, protocol_doc, car, offset_angle=0.0, window=0.2):
    """Each axle's (slip angle rad, lateral force over static load) from the reference, as
    ForzaETH identify a tyre on track: the steady circles (M5's last 3 s) and M6's slow ramp.
    The reference is the rear axle (base_link): its velocity's angle from the heading is the
    rear's slip angle. The front axle's velocity adds the yaw rate times the wheel base. At a
    steady turn, each axle's force over its static load is a_y / g (the front's over cos delta),
    so the mass drops out. Returns {"front": [(alpha, mu)], "rear": [...]}."""
    L = car["geometry"]["wheel_base"]
    max_steer = car["geometry"]["max_steer_angle"]
    out = {"front": [], "rear": []}
    for s, st in _runs(runs, ("M5", "M6")):
        t, x, y = st["ref_t"], st["ref_x"], st["ref_y"]
        yaw = np.unwrap(st["ref_yaw"])
        k = max(1, int(round(window / np.median(np.diff(t)))))
        tm = (t[k:] + t[:-k]) / 2
        vx, vy = (x[k:] - x[:-k]) / (t[k:] - t[:-k]), (y[k:] - y[:-k]) / (t[k:] - t[:-k])
        heading = (yaw[k:] + yaw[:-k]) / 2
        r = (yaw[k:] - yaw[:-k]) / (t[k:] - t[:-k])
        v = np.hypot(vx, vy)
        beta = np.arctan2(vy, vx) - heading
        beta = np.arctan2(np.sin(beta), np.cos(beta))
        c = s["condition"]
        side = float(c.get("side", 1))
        if s["manoeuvre"] == "M5":
            angle = max_steer if c["angle"] == "full" else float(c["angle"])
            sel = tm >= tm[-1] - 3.0
            segs = [sel]
        else:
            angle = 0.2
            n = np.arange(len(tm))
            segs = [(n // k == i) & (tm > 4.0) for i in range(int(n.max() // k) + 1)]
        delta = side * angle + offset_angle
        for sel in segs:
            if sel.sum() < 2 or v[sel].mean() < 0.5:
                continue
            vm, rm, bm = v[sel].mean(), r[sel].mean(), beta[sel].mean()
            ay = vm * rm
            alpha_r = -bm
            alpha_f = delta - math.atan2(vm * math.sin(bm) + L * rm, vm * math.cos(bm))
            out["rear"].append((float(alpha_r * side), float(ay * side / G)))
            out["front"].append((float(alpha_f * side), float(ay * side / (G * math.cos(delta)))))
    return out


def fit_axle(points, bin_width=0.01):
    """PhysX's curve (stiffness per rad, peak) through one axle's (alpha, mu) points. The points
    are averaged in bins of slip angle first, so the many near-straight ones do not outweigh the
    few near the limit; the fit starts from the best of a coarse grid (the stiffness and the
    peak trade off where the data stops short of the limit)."""
    from tyre_compare import physx
    pts = np.array([(a, m) for a, m in points if a > 0.002])
    if len(pts) == 0:
        return None
    bins = np.floor(pts[:, 0] / bin_width)
    mean = np.array([pts[bins == b].mean(axis=0) for b in np.unique(bins)])

    def cost(x):
        if min(x) <= 0.05:
            return 1e6
        f = physx(*x)
        return float(sum((f(a) - m) ** 2 for a, m in mean))
    grid = [(c, m) for c in np.geomspace(2.0, 40.0, 24) for m in np.linspace(0.3, 1.6, 27)]
    x0 = min(grid, key=cost)
    r = minimize(cost, np.array(x0), method="Nelder-Mead",
                 options={"xatol": 1e-4, "fatol": 1e-10, "maxiter": 600})
    return [float(v) for v in r.x]


def peak_reached(points, fitted, share=0.9):
    """Whether an axle's points reach `share` of the fitted peak: if not, the axle never came
    near its limit in the logs, and its friction is not identified, only bounded from below by
    the highest force seen."""
    top = max(m for _, m in points) if points else 0.0
    return top >= share * fitted[1], top


# PhysX's curve on the example spec (W8): the slope 4.4 per rad at lat_stiff_value 5.0, the
# peak 1.1 at tire_friction 1.05. An axle's (stiffness per rad, peak) in aib's terms:
SLOPE_PER_LAT_STIFF = 4.4 / 5.0
PEAK_PER_FRICTION = 1.1 / 1.05


def axle_tires_of(fit_values, base_lat_stiff_max_load=None):
    """aib's axle_tires from fit_tyres' (cf, mf, cr, mr)."""
    cf, mf, cr, mr = fit_values
    return {"front": {"lat_stiff_value": round(cf / SLOPE_PER_LAT_STIFF, 4),
                      "tire_friction": round(mf / PEAK_PER_FRICTION, 4)},
            "rear": {"lat_stiff_value": round(cr / SLOPE_PER_LAT_STIFF, 4),
                     "tire_friction": round(mr / PEAK_PER_FRICTION, 4)}}


# --- the stage intervals (W10's drift rule) ---------------------------------------------------

# A stage's resolution: the interval is never narrower than this half-width. The latency's is
# the component stage's grid step after its refinement (1 ms); the rest are continuous.
INTERVAL_FLOOR = {"component.latency": 0.001}
SECTIONS = ("vehicle", "component")


def flatten(fitted):
    """{dotted name: value} of a fit's vehicle and component parameters."""
    out = {}

    def walk(prefix, d):
        for k, v in d.items():
            if isinstance(v, dict):
                walk(f"{prefix}{k}.", v)
            elif isinstance(v, (int, float)) and not isinstance(v, bool):
                out[f"{prefix}{k}"] = float(v)
    for s in SECTIONS:
        walk(f"{s}.", fitted.get(s) or {})
    return out


def intervals(full, leave_one_out, lower=None):
    """{name: [low, high]}: the full fit's estimate plus or minus twice the jackknife's standard
    error over the leave-one-out fits, at least INTERVAL_FLOOR. A name in `lower` (a friction the
    logs do not identify) is bounded below only: [bound, None]."""
    lower = lower or {}
    out = {}
    n = len(leave_one_out)
    for name, est in full.items():
        if name in lower:
            out[name] = [float(lower[name]), None]
            continue
        vals = np.array([f[name] for f in leave_one_out if name in f], dtype=float)
        if n < 2 or len(vals) < n:
            continue
        se = math.sqrt((n - 1) / n * float(np.sum((vals - vals.mean()) ** 2)))
        half = max(2.0 * se, INTERVAL_FLOOR.get(name, 0.0))
        out[name] = [round(est - half, 6), round(est + half, 6)]
    return out
