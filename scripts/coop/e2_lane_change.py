#!/usr/bin/env python3
"""Roadmap 074 E2: Autoware's lane change, timed, from an E2 bag (run_one.sh).

The ego's lateral position is measured against the lanelet map (the signed distance from the
start lanelet's centreline, by lanelet2's arc coordinates), so the road may curve. Events:
  module   the lane-change module's first RTC status (lane_change_left/right)
  states   each change of the status's state (1 waiting for execution, 2 running, ...)
  start    the ego 0.2 m off its start lane's centre, moving away from it
  end      the ego within 0.3 m of the target lane's centre
Reported: the prepare time (module -> start), the lateral move (start -> end), speeds.

  scripts/coop/e2_lane_change.py <bag dir> <map dir> <start lanelet id> <target lanelet id>
"""
import math
import sys

import lanelet2
import rosbag2_py
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    bag, mapdir, l_start, l_target = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    m = lanelet2.io.load(f"{mapdir}/lanelet2_map.osm", LocalCartesianProjector(Origin(0, 0)))
    c_start = lanelet2.geometry.to2D(m.laneletLayer[l_start].centerline)
    c_target = lanelet2.geometry.to2D(m.laneletLayer[l_target].centerline)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    ego, states = [], []
    last_state = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        if topic == "/localization/kinematic_state":
            p = msg.pose.pose.position
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pt = lanelet2.core.BasicPoint2d(p.x, p.y)
            d_s = lanelet2.geometry.toArcCoordinates(c_start, pt).distance
            d_t = lanelet2.geometry.distance(c_target, pt)
            ego.append((t, p.x, p.y, msg.twist.twist.linear.x, d_s, d_t))
        elif topic.startswith("/planning/cooperate_status/lane_change"):
            for s in msg.statuses:
                t = s.stamp.sec + s.stamp.nanosec * 1e-9
                k = (s.state.type, s.safe, s.command_status.type)
                if last_state.get(topic) != k:
                    last_state[topic] = k
                    states.append((t, topic.rsplit("/", 1)[1], s.state.type, s.safe, s.auto_mode,
                                   round(s.start_distance, 1), round(s.finish_distance, 1)))
    if not ego:
        print("no ego samples")
        return
    moving = [e for e in ego if e[3] > 0.5]
    t_move = moving[0][0] if moving else ego[0][0]
    # RTC stamps are wall-clock (seen in E2); map the first one onto the bag's sim time by order
    start = next((e for e in ego if e[0] >= t_move and abs(e[4]) > 0.2), None)
    end = next((e for e in ego if start and e[0] > start[0] and e[5] < 0.3), None)
    print(f"ego moving at {t_move - ego[0][0]:.1f} s; samples {len(ego)}")
    print("RTC state changes (stamp, module, state, safe, auto, start m, finish m):")
    for s in states[:12]:
        print("  ", s)
    if start:
        print(f"lateral start: t={start[0] - t_move:.2f} s after moving, at ({start[1]:.1f}, {start[2]:.1f}), v={start[3]:.2f} m/s")
    if end:
        dur = end[0] - start[0]
        vmean = sum(e[3] for e in ego if start[0] <= e[0] <= end[0]) / max(1, sum(1 for e in ego if start[0] <= e[0] <= end[0]))
        print(f"lateral end:   t={end[0] - t_move:.2f} s after moving; the lateral move took {dur:.2f} s at {vmean:.2f} m/s mean")
    else:
        print("no lane change completed; max offset from the start lane "
              f"{max(abs(e[4]) for e in ego):.2f} m; final position ({ego[-1][1]:.1f}, {ego[-1][2]:.1f})")


if __name__ == "__main__":
    main()
