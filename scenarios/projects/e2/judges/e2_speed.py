#!/usr/bin/env python3
"""e2_speed: roadmap 074 E2's timing, measured on a run and held to E2's numbers.

From the run's rosbag of the ego (record/ego: /localization/kinematic_state,
/control/command/control_cmd) and the bridge's gap trace, as scripts/coop/e2_bag.py measures:

- a lane-change scenario: when the ego's lateral move starts (0.2 m off its first heading),
  counted from when it starts moving (0.5 m/s), and how long the move takes to 3.3 m.
  E2: 4.05-4.10 s and 4.80-4.85 s.
- a lead-brake scenario: the lead's brake (its speed falling 0.5 m/s within 0.5 s, from the
  trace), then Autoware's first braking command (below -0.5 m/s^2) and the ego's speed falling
  0.3 m/s. E2 on the default perception: 2.3 s and 2.7 s.

Each measure passes within --tol seconds (default 1) of E2's; a measure that never happens
fails the run.

    python3 judges/e2_speed.py [--tol 1.0] <run dir>
"""
import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from coop_pilots_dir import coop_dir  # noqa: E402
sys.path.insert(0, str(coop_dir()))

REFERENCE = {
    "lane_change": {"lane_change_start_s": 4.075, "lane_change_duration_s": 4.825},
    "lead_brake": {"cmd_decel_after_s": 2.3, "speed_drop_after_s": 2.7},
}


def read(bag):
    """(topic, message) of a bag in whichever storage its metadata names (scensim records sqlite3;
    e2_bag.read opens only mcap)."""
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    meta = yaml.safe_load((Path(bag) / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=meta["storage_identifier"]),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    while r.has_next():
        topic, data, _ = r.read_next()
        yield topic, deserialize_message(data, get_message(types[topic]))


def bag_rows(bag):
    from e2_bag import stamp
    ego, ctrl = [], []
    for topic, m in read(bag):
        if topic == "/localization/kinematic_state":
            p = m.pose.pose.position
            ego.append((stamp(m.header), p.x, p.y, m.twist.twist.linear.x))
        elif topic == "/control/command/control_cmd":
            ctrl.append((stamp(m), m.longitudinal.acceleration))
    return ego, ctrl


def lane_change(ego):
    # a bag can start with the ego still rolling from the stack's previous run: count from
    # its first standstill then
    if ego and ego[0][3] > 0.5:
        i = next((k for k, s in enumerate(ego) if s[3] < 0.1), len(ego))
        ego = ego[i:]
    moving = [s for s in ego if s[3] > 0.5]
    if not moving:
        return {"error": "the ego never moved"}
    t_move, x0, y0 = moving[0][:3]
    far = next((s for s in moving if math.hypot(s[1] - x0, s[2] - y0) > 3.0), None)
    if far is None:
        return {"error": "the ego moved less than 3 m"}
    hx, hy = far[1] - x0, far[2] - y0
    n = math.hypot(hx, hy)
    hx, hy = hx / n, hy / n
    lat = [(t, abs((x - x0) * hy - (y - y0) * hx)) for t, x, y, v in moving]
    start = next((t for t, d in lat if d > 0.2), None)
    end = next((t for t, d in lat if d > 3.3), None)
    return {"lateral_max_m": round(max(d for t, d in lat), 2),
            "lane_change_start_s": None if start is None else round(start - t_move, 2),
            "lane_change_duration_s": None if start is None or end is None else round(end - start, 2)}


def align(ego, tr_ego):
    """The offset from the trace's sim time to the bag's stamps: the one that best matches the
    ego's speed in both, 0.05 s steps (the bag can start with the ego still rolling from a
    stack's previous run, so its first motion is no anchor)."""
    import bisect
    ts = [s[0] for s in ego]

    def v_bag(t):
        i = bisect.bisect_left(ts, t)
        return ego[min(i, len(ego) - 1)][3] if 0 <= i < len(ego) and abs(ts[min(i, len(ego) - 1)] - t) < 0.2 else None

    lo, hi = ts[0] - tr_ego[-1][0], ts[-1] - tr_ego[0][0]
    best, best_off = math.inf, lo
    off = lo
    while off <= hi:
        errs = [abs(vb - v) for t, x, y, v in tr_ego[::5] if (vb := v_bag(t + off)) is not None]
        if len(errs) > 0.5 * len(tr_ego[::5]):
            e = sum(errs) / len(errs)
            if e < best:
                best, best_off = e, off
        off += 0.05
    return best_off


def lead_brake(ego, ctrl, trace):
    from e2_bag import trace_track
    tr_ego, lead = trace_track(trace, "ego"), trace_track(trace, "lead")
    if not ego or not tr_ego or not lead:
        return {"error": "no ego in the bag, or no ego or lead in the trace"}
    off = align(ego, tr_ego)
    vs = [(t + off, v) for t, x, y, v in lead]
    t_brake = next((vs[i][0] for i in range(len(vs) - 10)
                    if vs[i][1] > 3.0 and vs[i + 10][1] < vs[i][1] - 0.5), None)
    if t_brake is None:
        return {"error": "the lead never braked"}
    cmd = next((t for t, a in ctrl if t > t_brake and a < -0.5), None)
    v0 = next((v for t, x, y, v in ego if t >= t_brake), None)
    drop = next((t for t, x, y, v in ego if t > t_brake and v0 is not None and v < v0 - 0.3), None)
    return {"clock_offset_s": round(off, 3), "ego_speed_at_brake": None if v0 is None else round(v0, 2),
            "cmd_decel_after_s": None if cmd is None else round(cmd - t_brake, 2),
            "speed_drop_after_s": None if drop is None else round(drop - t_brake, 2)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tol", type=float, default=1.0)
    ap.add_argument("run_dir")
    a = ap.parse_args()
    rd = Path(a.run_dir)
    import yaml
    scenario = Path(yaml.safe_load((rd / "resolved.yaml").read_text())["scenario"]["file"]).stem
    kind = "lane_change" if "lane_change" in scenario else "lead_brake" if "lead_brake" in scenario else None
    if kind is None:
        print(json.dumps({"message": f"{scenario}: not an E2 scenario"}))
        return 2
    bag = rd / "record" / "ego"
    if not bag.is_dir():
        print(json.dumps({"message": "no record/ego bag (capture full)"}))
        return 2
    ego, ctrl = bag_rows(bag)
    if kind == "lane_change":
        m = lane_change(ego)
    else:
        traces = sorted((rd / "gap_trace").glob("*.jsonl"))
        if not traces:
            print(json.dumps({"message": "no gap trace"}))
            return 2
        m = lead_brake(ego, ctrl, traces[0])
    if "error" in m:
        print(json.dumps({"verdict": "failed", "message": m["error"], "kind": kind}))
        return 1
    ok, notes = True, []
    for k, ref in REFERENCE[kind].items():
        v = m.get(k)
        good = v is not None and abs(v - ref) <= a.tol
        ok &= good
        notes.append(f"{k} {v} (E2 {ref})")
    print(json.dumps({"kind": kind, **m, "reference": REFERENCE[kind], "tol_s": a.tol,
                      "verdict": "passed" if ok else "failed", "message": "; ".join(notes)}))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
