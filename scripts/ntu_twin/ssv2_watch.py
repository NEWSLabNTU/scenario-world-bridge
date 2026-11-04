#!/usr/bin/env python3
"""Did the NPCs do their part? Roadmaps 059 N8 and 072 (parity P7): SSv2's own view of a running
scenario, whatever the simulator. Once a period, every entity's pose, speed and current action
(`/simulation/entity/status`); and each named storyboard element's state when it changes
(`/simulation/context`). Run in the scenario domain, with the stack's DDS transport:

    source /opt/autoware/1.5.0/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=$SCENARIO_ROS_DOMAIN_ID \\
        CYCLONEDDS_URI=file://$PWD/src/swb_launch/config/cyclonedds-localhost.xml
    python3 scripts/ntu_twin/ssv2_watch.py --event cross --event stand_in_the_lane > watch.log

Why: a scenario passes on its success condition alone. On 2026-11-04 a pedestrian moved by a
timed FollowTrajectoryAction stood still for 45 s and reached the lane after the ego had gone,
and the run still passed. A pass with NPCs counts only with a log showing that they acted.
"""
import argparse
import re
import time

import rclpy
from openscenario_interpreter_msgs.msg import Context
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from traffic_simulator_msgs.msg import EntityStatusWithTrajectoryArray

ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
ap.add_argument("--event", action="append", default=[],
                help="a storyboard element's name whose state to report (repeatable)")
ap.add_argument("--period", type=float, default=1.0, help="s between pose lines")
a = ap.parse_args()

rclpy.init()
node = rclpy.create_node("ntu_ssv2_watch")
last = {"pose": 0.0, "state": {}}


def on_status(m):
    if time.time() - last["pose"] < a.period:
        return
    last["pose"] = time.time()
    parts = []
    for e in m.data:
        p = e.status.pose.position
        parts.append(f"{e.name} ({p.x:.1f},{p.y:.1f},{p.z:.1f}) "
                     f"v={e.status.action_status.twist.linear.x:.2f} "
                     f"{e.status.action_status.current_action}")
    print(f"{time.strftime('%T')} " + " | ".join(parts), flush=True)


def on_context(m):
    for name in a.event:
        i = m.data.find(f'"{name}"')
        if i < 0:
            continue
        st = re.findall(r'"currentState"\s*:\s*"([^"]+)"', m.data[i:i + 600])
        st = st[0] if st else "?"
        if last["state"].get(name) != st:
            last["state"][name] = st
            print(f"{time.strftime('%T')} CONTEXT {name}: {st}", flush=True)


node.create_subscription(EntityStatusWithTrajectoryArray, "/simulation/entity/status", on_status,
                         QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
node.create_subscription(Context, "/simulation/context", on_context,
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
rclpy.spin(node)
