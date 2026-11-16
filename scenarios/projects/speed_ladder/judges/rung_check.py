#!/usr/bin/env python3
"""rung_check: one rung of the speed ladder recorded as 063 step 4 needs it.

Reads the run's recording (record/<entity>/) and its rung (resolved.yaml's scenario parameter
`rung`, m/s, Autoware's max_velocity). Over the ground truth's span (its header stamps,
simulated time):

- ground truth rate: /ground_truth/odom messages per simulated second, at least --min-gt-hz
  (90; aib publishes it every 10 ms frame);
- camera rate: /sensing/camera/zed/image_raw images per simulated second, at least
  --min-camera-hz (25; the ZED X left camera at 30 Hz);
- top speed: the largest /vehicle/status/velocity_status longitudinal speed, within
  --speed-tol (0.10) of the rung, so the car reached its rung on the ring's straights and
  did not run past it;
- the lap: the ground truth's last position within --finish-m (10) of the scenario's finish
  line (x 90, y 204.1, the scenario's ReachPositionCondition), after at least --min-lap-m
  (1000) driven.

Passes when all four hold; prints them with the distance driven. No recording, no ground
truth or no rung is an error.

    python3 judges/rung_check.py [--min-gt-hz 90] [--min-camera-hz 25] [--speed-tol 0.10]
                                 [--finish-m 10] [--min-lap-m 1000] <run dir>
"""
import argparse
import json
import math
import sys
from pathlib import Path

import yaml

GT = "/ground_truth/odom"
CAMERA = "/sensing/camera/zed/image_raw"
SPEED = "/vehicle/status/velocity_status"
FINISH = (90.0, 204.1)


def read(bag_dir: Path, topics: list) -> dict:
    """Per topic, the messages' header stamps (s) and, for GT and speed, what is checked;
    the camera's images are only counted."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage = "mcap" if any(bag_dir.glob("*.mcap")) else "sqlite3"
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    out = {t: [] for t in topics}
    if not want:
        return out
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == CAMERA:   # counted, not decoded (an image is ~1.7 MB)
            out[topic].append((None,))
            continue
        m = deserialize_message(data, cls[topic])
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if topic == GT:
            p = m.pose.pose.position
            out[topic].append((st, p.x, p.y))
        elif topic == SPEED:
            out[topic].append((st, m.longitudinal_velocity))
        else:
            out[topic].append((st,))
    return out


def rung_of(run_dir: Path):
    try:
        r = yaml.safe_load(Path(run_dir, "resolved.yaml").read_text()) or {}
        return float(((r.get("scenario") or {}).get("parameters") or {})["rung"])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def rate(stamps: list, t0: float, t1: float) -> float:
    """Messages per simulated second over [t0, t1]. An undecoded message (the camera's: no
    stamp) always counts, so the camera's rate assumes it was recorded over the same span as
    the ground truth (both start and end with the run's recording); a camera publishing before
    or after the ground truth would read high. The bag's receive times are not used instead:
    they are the recorder's clock, not the header stamps' simulated time."""
    n = sum(1 for s in stamps if s[0] is None or t0 <= s[0] <= t1)
    return n / (t1 - t0) if t1 > t0 else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-gt-hz", type=float, default=90.0)
    ap.add_argument("--min-camera-hz", type=float, default=25.0)
    ap.add_argument("--speed-tol", type=float, default=0.10)
    ap.add_argument("--finish-m", type=float, default=10.0)
    ap.add_argument("--min-lap-m", type=float, default=1000.0)
    ap.add_argument("run_dir")
    a = ap.parse_args()
    rung = rung_of(a.run_dir)
    if rung is None:
        print(json.dumps({"message": "no rung: resolved.yaml has no scenario parameter rung"}))
        return 2
    bags = sorted(p.parent for p in Path(a.run_dir, "record").glob("*/metadata.yaml"))
    if not bags:
        print(json.dumps({"message": "no recording in the run (logging.capture: full)"}))
        return 2
    msgs = {GT: [], CAMERA: [], SPEED: []}
    for b in bags:
        for t, v in read(b, list(msgs)).items():
            msgs[t].extend(v)
    gt = sorted(msgs[GT])
    if len(gt) < 2:
        print(json.dumps({"message": f"no {GT} in the recording"}))
        return 2
    t0, t1 = gt[0][0], gt[-1][0]
    dist = sum(math.hypot(b[1] - a_[1], b[2] - a_[2]) for a_, b in zip(gt, gt[1:]))
    gt_hz = rate(gt, t0, t1)
    cam_hz = rate(msgs[CAMERA], t0, t1)
    top = max((v for _, v in msgs[SPEED]), default=0.0)
    finish = math.hypot(gt[-1][1] - FINISH[0], gt[-1][2] - FINISH[1])
    checks = {
        "gt_rate": gt_hz >= a.min_gt_hz,
        "camera_rate": cam_hz >= a.min_camera_hz,
        "top_speed": abs(top - rung) <= a.speed_tol * rung,
        "lap": finish <= a.finish_m and dist >= a.min_lap_m,
    }
    ok = all(checks.values())
    out = {"verdict": "passed" if ok else "failed", "rung_mps": rung,
           "top_speed_mps": round(top, 2), "distance_m": round(dist, 1),
           "sim_s": round(t1 - t0, 1), "gt_hz": round(gt_hz, 1), "camera_hz": round(cam_hz, 1),
           "finish_m": round(finish, 1), "checks": checks,
           "bounds": {"min_gt_hz": a.min_gt_hz, "min_camera_hz": a.min_camera_hz,
                      "speed_tol": a.speed_tol, "finish_m": a.finish_m,
                      "min_lap_m": a.min_lap_m},
           "message": f"rung {rung:g} m/s: top {top:.1f}, {dist:.0f} m, GT {gt_hz:.0f} Hz, "
                      f"camera {cam_hz:.1f} Hz, finish {finish:.1f} m"
                      + ("" if ok else "; failed: " + ", ".join(k for k, v in checks.items()
                                                                  if not v))}
    print(json.dumps(out))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
