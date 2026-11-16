#!/usr/bin/env python3
"""drift_lap: the drifts of a Town05 ring lap and its contacts, against roadmap 057's lap 32.

Reads the run's recording (record/<entity>/, `logging.capture: full` with /drift/state in
`logging.topics`): drift_controller's state, a line of JSON per phase change and once a
second (phase cruise, brake, turn_in, hold, exit, recover, rejoin). A drift is one entry into
`hold`; for each, its drift point, the speed at turn-in and the largest sideslip while held.
Contacts are the run's contact reports (contacts/*.json, the ego's `contacts`).

Passes when the lap had at least --min-drifts drifts and no contact (lap 32, 2026-10-28: a
full lap, one drift at the NE bend, no contact). Whether the lap was finished is SSv2's verdict
(the scenario's exitSuccess at the finish line). No /drift/state at all is an error: the drift
controller did not run.

    python3 judges/drift_lap.py [--min-drifts 1] <run dir>
"""
import argparse
import json
import sys
from pathlib import Path

STATE = "/drift/state"


def read(bag_dir: Path, topic: str) -> list:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String

    reader = rosbag2_py.SequentialReader()
    storage = "mcap" if any(bag_dir.glob("*.mcap")) else "sqlite3"
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    if topic not in {t.name for t in reader.get_all_topics_and_types()}:
        return []
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while reader.has_next():
        _, data, stamp = reader.read_next()
        try:
            out.append((stamp * 1e-9, json.loads(deserialize_message(data, String).data)))
        except ValueError:
            continue
    return out


def drifts(states: list) -> list:
    """One entry per drift: its point, speed at turn-in, peak sideslip in hold, how it ended."""
    out, cur, prev, entry_speed = [], None, None, None
    for t, s in states:
        phase = s.get("phase")
        if phase == "turn_in" and prev != "turn_in":
            entry_speed = s.get("speed")
        if phase == "hold" and prev != "hold":
            cur = {"point": s.get("point"), "t": round(t, 2),
                   "entry_speed": entry_speed if entry_speed is not None else s.get("speed"),
                   "beta_max_deg": abs(s.get("beta_deg") or 0.0)}
            out.append(cur)
        elif phase == "hold" and cur is not None:
            cur["beta_max_deg"] = max(cur["beta_max_deg"], abs(s.get("beta_deg") or 0.0))
        elif prev == "hold" and cur is not None:
            cur["ended"] = phase
            cur, entry_speed = None, None
        prev = phase
    return out


def contacts(run_dir: Path) -> int:
    n = 0
    for f in sorted(Path(run_dir, "contacts").glob("*.json")):
        try:
            n += len(json.loads(f.read_text()).get("contacts") or [])
        except (OSError, ValueError):
            continue
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-drifts", type=int, default=1)
    ap.add_argument("run_dir")
    a = ap.parse_args()
    bags = sorted(p.parent for p in Path(a.run_dir, "record").glob("*/metadata.yaml"))
    if not bags:
        print(json.dumps({"message": "no recording in the run (logging.capture: full)"}))
        return 2
    states = sorted((x for b in bags for x in read(b, STATE)), key=lambda x: x[0])
    if not states:
        print(json.dumps({"message": f"no {STATE} in the recording: the drift controller"
                                     " did not run"}))
        return 2
    ds = drifts(states)
    hits = contacts(a.run_dir)
    ok = len(ds) >= a.min_drifts and hits == 0
    out = {"verdict": "passed" if ok else "failed", "drifts": len(ds),
           "min_drifts": a.min_drifts, "contacts": hits,
           "detail": [dict(d, beta_max_deg=round(d["beta_max_deg"], 1)) for d in ds],
           "message": f"{len(ds)} drift(s) (at least {a.min_drifts}), {hits} contact(s)"}
    print(json.dumps(out))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
