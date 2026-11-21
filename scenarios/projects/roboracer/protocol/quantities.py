"""Design note 5.2's quantities, from one run's streams, and 5.3's bands over runs: scensim
roadmap 091 W23, the first cut of W11 (b). Pure: numpy only.

A run is its `session.yaml` and its streams, `{name: array}`, as twin_runs.py writes them
(`cmd_*`, `core_*`, `imu_*`, `ref_*`, each with its own `*_t`). The speed is the VESC's ERPM over
the protocol's gain. Positions, the radius and stopping distances come from the reference.

    quantities(session, streams, conversion) -> {quantity: value}
    band(real_values, res) -> (low, high)
    verdicts(real_runs, twin_runs, conversion) -> [Verdict]
"""
import math
from dataclasses import dataclass

import numpy as np

# Each quantity's resolution (design note 5.3): 1 / the rate of the topic it is read from for
# times, the reference's position noise for distances; the rest are stated here
CORE_HZ = 50.0
IMU_HZ = 50.0


def speed(streams, conversion):
    """(t, m/s) from the VESC's ERPM."""
    g, o = conversion["speed_to_erpm_gain"], conversion["speed_to_erpm_offset"]
    return streams["core_t"], (streams["core_erpm"] - o) / g


def _segment_starts(session, protocol_doc):
    segs = protocol_doc["manoeuvres"][session["manoeuvre"]]["segments"]
    out, t = [], 0.0
    for s in segs:
        out.append(t)
        t += float(s["t"])
    return out


def _cross(t, x, level, after=0.0, rising=True):
    """The first time after `after` that x crosses `level`, linearly interpolated; None."""
    m = t >= after
    tt, xx = t[m], x[m]
    hit = np.nonzero(xx >= level)[0] if rising else np.nonzero(xx <= level)[0]
    if len(hit) == 0:
        return None
    i = hit[0]
    if i == 0:
        return float(tt[0])
    t0, t1, x0, x1 = tt[i - 1], tt[i], xx[i - 1], xx[i]
    return float(t0 + (level - x0) * (t1 - t0) / (x1 - x0)) if x1 != x0 else float(t1)


def _ref_speed(streams, window=0.1):
    """(t, m/s) along the path from the reference's positions, differenced over `window`."""
    t, x, y = streams["ref_t"], streams["ref_x"], streams["ref_y"]
    k = max(1, int(round(window / np.median(np.diff(t)))))
    d = np.hypot(x[k:] - x[:-k], y[k:] - y[:-k]) / (t[k:] - t[:-k])
    return (t[k:] + t[:-k]) / 2, d


def quantities(session, streams, protocol_doc):
    """{quantity: value} of one run. Names follow design note 5.2."""
    m = session["manoeuvre"]
    cond = session["condition"]
    conv = protocol_doc["conversion"]
    starts = _segment_starts(session, protocol_doc)
    t, v = speed(streams, conv)
    out = {}
    if m == "M1":
        target, t0 = float(cond["speed"]), starts[1]
        tm = _cross(t, v, 0.05 * target, t0)
        t10, t90 = _cross(t, v, 0.1 * target, t0), _cross(t, v, 0.9 * target, t0)
        hold = (t >= t0 + 2.0) & (t < starts[2])
        out["dead_time"] = tm - t0 if tm is not None else None
        out["rise_time"] = t90 - t10 if t10 is not None and t90 is not None else None
        out["overshoot"] = float(100.0 * max(0.0, v[(t >= t0) & (t < starts[2])].max() / target - 1.0))
        out["steady_speed"] = float(v[hold].mean())
    elif m == "M2":
        t0 = starts[1]
        sel = (t >= t0 + 0.2) & (v > 0.2)
        tt, vv = t[sel], v[sel]
        if len(tt) > 10:
            a = np.gradient(vv, tt)
            # a(v) = -(c0 + c2 v^2): a least-squares fit of the deceleration
            A = np.stack([np.ones_like(vv), vv ** 2], axis=1)
            c, *_ = np.linalg.lstsq(A, -a, rcond=None)
            out["c0"], out["c2"] = float(c[0]), float(c[1])
        out["coast_time_to_half"] = (_cross(t, v, 0.5 * float(cond["speed"]), t0, rising=False)
                                     or float("nan")) - t0
    elif m in ("M3", "M3b"):
        t0 = starts[1]
        tr, vr = _ref_speed(streams)
        ts = _cross(tr, vr, 0.05, t0, rising=False)
        out["stop_time"] = ts - t0 if ts is not None else None
        if ts is not None:
            xs = np.interp([t0, ts], streams["ref_t"], streams["ref_x"])
            out["stop_distance"] = float(xs[1] - xs[0])
    elif m == "M4":
        t0 = starts[1]
        wt, wz = streams["imu_t"], streams["imu_wz"] * float(cond["side"])
        steady = wz[(wt >= t0 + 1.0) & (wt < starts[2])].mean()
        tm = _cross(wt, wz, 0.1 * steady, t0)
        t90 = _cross(wt, wz, 0.9 * steady, t0)
        out["steer_dead_time"] = tm - t0 if tm is not None else None
        out["yaw_rise_time"] = t90 - tm if tm is not None and t90 is not None else None
        out["yaw_rate_full_lock"] = float(steady)
    elif m == "M5":
        t1 = t[-1] - 3.0
        wt, wz = streams["imu_t"], streams["imu_wz"]
        r = float(wz[wt >= t1].mean())
        vs = float(v[t >= t1].mean())
        out["yaw_rate"] = abs(r)
        out["radius"] = abs(vs / r) if abs(r) > 1e-6 else None
        out["lateral_acceleration"] = abs(vs * r)
    elif m == "M6":
        wt, wz = streams["imu_t"], np.abs(streams["imu_wz"])
        vi = np.interp(wt, t, v)
        ay = vi * wz
        # the slide: the lateral acceleration's highest value, over a 0.5 s window, before it
        # falls away
        k = max(1, int(round(0.5 * IMU_HZ)))
        smooth = np.convolve(ay, np.ones(k) / k, mode="same")
        out["lateral_acceleration_at_slide"] = float(smooth.max())
    return out


def trace(session, streams, protocol_doc, step=0.02):
    """M1's and M2's speed traces on a common clock from the step's segment, for 5.3's curve
    rule; None for the other manoeuvres."""
    if session["manoeuvre"] not in ("M1", "M2"):
        return None
    t, v = speed(streams, protocol_doc["conversion"])
    t0 = _segment_starts(session, protocol_doc)[1]
    grid = np.arange(t0 - 0.2, t[-1], step)
    return grid - t0, np.interp(grid, t, v)


# --- 5.3's bands ------------------------------------------------------------------------------

RES = {  # a quantity's resolution: what the measurement cannot tell apart
    "dead_time": 1 / CORE_HZ, "rise_time": 1 / CORE_HZ, "overshoot": 1.0,
    "steady_speed": 0.01, "c0": 0.02, "c2": 0.01, "coast_time_to_half": 1 / CORE_HZ,
    "stop_time": 0.02, "stop_distance": None,  # the reference's noise (session)
    "steer_dead_time": 1 / IMU_HZ, "yaw_rise_time": 1 / IMU_HZ, "yaw_rate_full_lock": 0.02,
    "yaw_rate": 0.01, "radius": None, "lateral_acceleration": 0.05,
    "lateral_acceleration_at_slide": 0.1,
}


def resolution(quantity, session):
    r = RES.get(quantity)
    if r is None:  # a distance: the reference's position noise, at least 2 cm (5.3)
        r = max(float((session.get("reference") or {}).get("noise_m") or 0.0), 0.02)
    return r


def band(values, res):
    """5.3's band over the real runs' values: [min - res, max + res], or mean +/- 2 res when
    their spread is below the resolution."""
    vals = [v for v in values if v is not None and not math.isnan(v)]
    if not vals:
        return None
    lo, hi = min(vals), max(vals)
    if hi - lo < res:
        m = sum(vals) / len(vals)
        return m - 2 * res, m + 2 * res
    return lo - res, hi + res


@dataclass
class Verdict:
    manoeuvre: str
    condition: str
    quantity: str
    twin: float
    low: float
    high: float

    @property
    def passed(self):
        return self.twin is not None and self.low <= self.twin <= self.high


def condition_key(session):
    return "_".join(f"{k}{v}" for k, v in sorted(session["condition"].items()))


def verdicts(real, twin, protocol_doc):
    """5.3's verdict per quantity and condition. `real` and `twin`: [(session, streams)]. The
    twin is deterministic: one run per condition (its first). Curves (M1, M2) pass when the
    twin's RMS from the real mean trace is at most 1.5 times the real runs' largest."""
    groups = {}
    for sess, st in real:
        groups.setdefault((sess["manoeuvre"], condition_key(sess)), []).append((sess, st))
    twins = {}
    for sess, st in twin:
        twins.setdefault((sess["manoeuvre"], condition_key(sess)), (sess, st))
    out = []
    for key, runs in sorted(groups.items()):
        if key not in twins:
            continue
        ts, tst = twins[key]
        tq = quantities(ts, tst, protocol_doc)
        rq = [quantities(s, st, protocol_doc) for s, st in runs]
        for q in tq:
            b = band([r.get(q) for r in rq], resolution(q, runs[0][0]))
            if b is not None:
                out.append(Verdict(key[0], key[1], q, tq[q], *b))
        tr = trace(ts, tst, protocol_doc)
        if tr is not None:
            real_tr = [trace(s, st, protocol_doc) for s, st in runs]
            n = min(len(x[1]) for x in real_tr + [tr])
            mean = np.mean([x[1][:n] for x in real_tr], axis=0)
            worst = max(float(np.sqrt(np.mean((x[1][:n] - mean) ** 2))) for x in real_tr)
            rms = float(np.sqrt(np.mean((tr[1][:n] - mean) ** 2)))
            out.append(Verdict(key[0], key[1], "speed_trace_rms", rms, 0.0, 1.5 * worst))
    return out
