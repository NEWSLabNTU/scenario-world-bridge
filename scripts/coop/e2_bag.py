#!/usr/bin/env python3
"""Roadmap 074 E2: read an E2 bag (ego domain) and print what the light policies need.

  scripts/coop/e2_bag.py <bag dir>                                   # summary, writes <bag>.e2.json
  scripts/coop/e2_bag.py <bag dir> --measures --trace <gap trace .jsonl>  # E2's numbers

Prints: the ego's track (speed, lateral position), lane-change status changes, the perception
pipeline's latency (sim time at receipt minus the message's stamp, per topic), and, with a
trace, the tracked object's speed and acceleration against the NPC's truth.
"""
import json
import math
import sys
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, data, t_recv = r.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), t_recv * 1e-9


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    bag = sys.argv[1]
    trace = sys.argv[sys.argv.index("--trace") + 1] if "--trace" in sys.argv else None
    clock = []  # (wall, sim)
    ego, ctrl, lat = [], [], defaultdict(list)
    tracked, lc = [], []
    last_lc = {}
    for topic, m, tw in read(bag):
        if topic == "/clock":
            clock.append((tw, m.clock.sec + m.clock.nanosec * 1e-9))
            continue
        sim_now = clock[-1][1] + (tw - clock[-1][0]) if clock else None
        if topic == "/localization/kinematic_state":
            p = m.pose.pose.position
            ego.append((stamp(m.header), p.x, p.y, m.twist.twist.linear.x))
        elif topic == "/control/command/control_cmd":
            ctrl.append((stamp(m), m.longitudinal.acceleration, m.lateral.steering_tire_angle))
        elif topic.startswith("/perception"):
            if sim_now is not None and len(m.objects) > 0:
                lat[topic].append(sim_now - stamp(m.header))
            if topic.endswith("tracking/objects"):
                for o in m.objects:
                    k = o.kinematics
                    tracked.append((stamp(m.header), k.pose_with_covariance.pose.position.x,
                                    k.pose_with_covariance.pose.position.y, k.twist_with_covariance.twist.linear.x,
                                    k.acceleration_with_covariance.accel.linear.x))
        elif "lane_change" in topic or "steering_factors" in topic:
            s = str(m)
            key = topic
            short = s[:300]
            if last_lc.get(key) != short:
                last_lc[key] = short
                lc.append((sim_now, topic, short))
    print(f"ego samples {len(ego)}, sim {ego[0][0]:.1f}..{ego[-1][0]:.1f} s" if ego else "no ego")
    t0 = ego[0][0] if ego else 0
    for i in range(0, len(ego), max(1, len(ego) // 25)):
        t, x, y, v = ego[i]
        print(f"  t={t - t0:6.1f} x={x:8.2f} y={y:8.2f} v={v:5.2f}")
    print("latency (sim s at receipt - stamp):")
    for k, v in lat.items():
        v = sorted(v)
        if v:
            print(f"  {k}: n={len(v)} p50={v[len(v)//2]:.3f} p95={v[int(.95*len(v))]:.3f} max={v[-1]:.3f}")
    print("lane-change status changes:")
    for t, topic, s in lc[:40]:
        print(f"  t={(t or 0) - t0:6.1f} {topic}: {s[:220]}")
    out = {"ego": ego, "ctrl": ctrl, "tracked": tracked}
    json.dump(out, open(bag.rstrip("/") + ".e2.json", "w"))


if __name__ == "__main__" and "--measures" not in sys.argv:
    main()


# --- E2 measures from the saved JSON and the gap trace -------------------------------------------
def trace_track(path, name):
    """[(sim_time, x, y, speed)] of an entity's CARLA body, map frame, from the gap trace."""
    out, prev = [], None
    for line in open(path):
        d = json.loads(line)
        if d.get("type") != "frame" or name not in d["entities"]:
            continue
        x, y = d["entities"][name]["carla"][:2]
        t = d["sim_time"]
        v = 0.0 if prev is None or t <= prev[0] else math.hypot(x - prev[1], y - prev[2]) / (t - prev[0])
        out.append((t, x, y, v))
        prev = (t, x, y)
    return out


def measures(bag, trace):
    d = json.load(open(bag.rstrip("/") + ".e2.json"))
    ego, ctrl, tracked = d["ego"], d["ctrl"], d["tracked"]
    tr_ego = trace_track(trace, "ego")
    # align the trace's sim_time with the bag's clock on the ego's first motion
    t_bag = next(t for t, x, y, v in ego if v > 0.5)
    t_tr = next(t for t, x, y, v in tr_ego if v > 0.5)
    off = t_bag - t_tr
    res = {"clock_offset": round(off, 3)}
    lead = trace_track(trace, "lead")
    if lead:
        # the lead's brake: first frame after which its speed falls by 0.5 m/s within 0.5 s
        vs = [(t + off, v) for t, x, y, v in lead]
        t_brake = None
        for i in range(len(vs) - 10):
            if vs[i][1] > 3.0 and vs[i + 10][1] < vs[i][1] - 0.5:
                t_brake = vs[i][0]
                break
        res["lead_brake_start"] = t_brake
        if t_brake:
            onset_cmd = next((t for t, a, s in ctrl if t > t_brake and a < -0.5), None)
            res["ego_cmd_decel_after_s"] = None if onset_cmd is None else round(onset_cmd - t_brake, 3)
            ev = [(t, v) for t, x, y, v in ego if t > t_brake - 1]
            v0 = next(v for t, v in ev if t >= t_brake)
            onset_v = next((t for t, v in ev if t > t_brake and v < v0 - 0.3), None)
            res["ego_speed_drop_after_s"] = None if onset_v is None else round(onset_v - t_brake, 3)
            res["ego_speed_at_brake"] = round(v0, 2)
            gap = None
            for t, x, y, v in lead:
                if abs(t + off - t_brake) < 0.03:
                    e = min(ego, key=lambda s: abs(s[0] - t_brake))
                    gap = math.hypot(x - e[1], y - e[2]) - 4.7
            res["gap_at_brake_m"] = None if gap is None else round(gap, 1)
        # tracking error: tracked objects within 3 m of the lead's truth at the same time
        lt = [(t + off, x, y, v) for t, x, y, v in lead]
        errs_v, errs_p, accs = [], [], []
        j = 0
        for t, x, y, v, a in tracked:
            while j + 1 < len(lt) and lt[j + 1][0] <= t:
                j += 1
            T = lt[j]
            if abs(T[0] - t) > 0.06:
                continue
            dp = math.hypot(x - T[1], y - T[2])
            if dp < 3.0:
                errs_p.append(dp)
                errs_v.append(v - T[3])
                accs.append(a)
        if errs_v:
            sv = sorted(abs(e) for e in errs_v)
            res["tracking_n"] = len(errs_v)
            res["tracking_speed_err_mean"] = round(sum(errs_v) / len(errs_v), 3)
            res["tracking_speed_err_p50_p95"] = (round(sv[len(sv) // 2], 3), round(sv[int(.95 * len(sv))], 3))
            sp = sorted(errs_p)
            res["tracking_pos_err_p50_p95"] = (round(sp[len(sp) // 2], 2), round(sp[int(.95 * len(sp))], 2))
            res["tracked_accel_reported_nonzero"] = sum(1 for a in accs if abs(a) > 1e-6)
    # lateral: the ego's x across the run (lanes run along +y here)
    xs = [(t, x, y) for t, x, y, v in ego if v > 0.3]
    if xs:
        base = xs[0][1] - 0.025 * xs[0][2]
        lat = [(t, (x - 0.025 * y) - base) for t, x, y in xs]
        start = next((t for t, l in lat if abs(l) > 0.2), None)
        end = next((t for t, l in lat if abs(l) > 3.3), None)
        res["lateral_max_m"] = round(max(abs(l) for t, l in lat), 2)
        res["lane_change_start_t"] = start
        res["lane_change_duration_s"] = None if start is None or end is None else round(end - start, 2)
    print(json.dumps(res, indent=1))


if __name__ == "__main__" and "--measures" in sys.argv:
    measures(sys.argv[1], sys.argv[sys.argv.index("--trace") + 1])
