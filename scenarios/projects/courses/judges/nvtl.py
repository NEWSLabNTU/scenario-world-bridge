#!/usr/bin/env python3
"""nvtl: NDT's scan-to-map score along the run, against roadmap 058 C9's bound.

Reads the run's recording (record/<entity>/, `logging.capture: full` with the NVTL topic in
`logging.topics`): NDT's nearest-voxel transformation likelihood, and the ego's kinematic
state for the distance it drove. Samples from the first 10 s after NDT starts are left out
(initial alignment). Passes when the 5th percentile of the rest is at least --min-p05 (058
C9: "NVTL at least 2.5 along the road") and the ego drove at least --min-distance m. No
samples, or no recording, is an error, not a pass.

    python3 judges/nvtl.py [--min-p05 2.5] [--min-distance 50] <run dir>
"""
import argparse
import json
import math
import sys
from pathlib import Path

NVTL = "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
STATE = "/localization/kinematic_state"
SETTLE_S = 10.0


def read(bag_dir: Path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    storage = "mcap" if any(bag_dir.glob("*.mcap")) else "sqlite3"
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    if not want:
        return {}
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    out = {t: [] for t in want}
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        out[topic].append((stamp * 1e-9, deserialize_message(data, cls[topic])))
    return out


def pct(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, max(0, int(math.floor(q * (len(s) - 1)))))]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-p05", type=float, default=2.5)
    ap.add_argument("--min-distance", type=float, default=50.0)
    ap.add_argument("run_dir")
    a = ap.parse_args()
    bags = sorted(p.parent for p in Path(a.run_dir, "record").glob("*/metadata.yaml"))
    if not bags:
        print(json.dumps({"message": "no recording in the run (logging.capture: full)"}))
        return 2
    msgs = {}
    for b in bags:
        for t, v in read(b, [NVTL, STATE]).items():
            msgs.setdefault(t, []).extend(v)
    nv = msgs.get(NVTL) or []
    if not nv:
        print(json.dumps({"message": f"no {NVTL} in the recording"}))
        return 2
    t0 = nv[0][0]
    vals = [m.data for t, m in nv if t - t0 >= SETTLE_S]
    if not vals:
        print(json.dumps({"message": f"NVTL recorded for under {SETTLE_S:g} s"}))
        return 2
    dist = 0.0
    prev = None
    for _, m in msgs.get(STATE) or []:
        p = m.pose.pose.position
        if prev is not None:
            dist += math.hypot(p.x - prev[0], p.y - prev[1])
        prev = (p.x, p.y)
    p05 = pct(vals, 0.05)
    ok = p05 >= a.min_p05 and dist >= a.min_distance
    out = {"verdict": "passed" if ok else "failed", "bound_p05": a.min_p05,
           "samples": len(vals), "nvtl_min": round(min(vals), 3), "nvtl_p05": round(p05, 3),
           "nvtl_median": round(pct(vals, 0.5), 3),
           "below_bound": round(sum(v < a.min_p05 for v in vals) / len(vals), 4),
           "distance_m": round(dist, 1), "min_distance_m": a.min_distance,
           "message": f"NVTL p05 {p05:.2f} {'>=' if p05 >= a.min_p05 else '<'} {a.min_p05}"
                      f" over {len(vals)} samples, {dist:.0f} m driven"}
    print(json.dumps(out))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
