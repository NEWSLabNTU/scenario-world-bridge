#!/usr/bin/env python3
"""Roadmap 072, the tour: what the ego stack does, as JSON lines, in the ego's ROS domain.

Each line has `wall` (time.time()) and `kind`:
- `s`: a sample every 0.5 s: map x, y (map frame), speed, the latest NDT NVTL, the MRM state and
  behaviour, the operation mode, the route state;
- `mrm`: the MRM state or behaviour changed;
- `vf`: a velocity factor appeared or changed (behaviour, status, distance, detail), so a stop
  can be placed and explained;
- `nvtl_low`: NVTL under --nvtl-warn (once per dip).

    source /opt/autoware/1.5.0/setup.bash; source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=1 CYCLONEDDS_URI=file://.../cyclonedds-localhost.xml
    python3 scripts/ntu_carla/tour_monitor.py OUT.jsonl
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState, RouteState, VelocityFactorArray
from autoware_internal_debug_msgs.msg import Float32Stamped

MRM_STATE = {0: "UNKNOWN", 1: "NORMAL", 2: "MRM_OPERATING", 3: "MRM_SUCCEEDED", 4: "MRM_FAILED"}
MRM_BEHAVIOR = {0: "UNKNOWN", 1: "NONE", 2: "PULL_OVER", 3: "COMFORTABLE_STOP", 4: "EMERGENCY_STOP"}
OP_MODE = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
ROUTE = {0: "UNKNOWN", 1: "UNSET", 2: "SET", 3: "ARRIVED", 4: "CHANGING"}
VF_STATUS = {0: "UNKNOWN", 1: "APPROACHING", 2: "STOPPED"}


class Monitor(Node):
    def __init__(self, out, warn):
        super().__init__("tour_monitor")
        self.out, self.warn = open(out, "a", buffering=1), warn
        self.pose, self.speed, self.nvtl = None, None, None
        self.mrm, self.op, self.route = None, None, None
        self.vf, self.low = {}, False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 10)
        self.create_subscription(Float32Stamped,
                                 "/localization/pose_estimator/nearest_voxel_transformation_likelihood",
                                 self.on_nvtl, 10)
        self.create_subscription(MrmState, "/api/fail_safe/mrm_state", self.on_mrm, latched)
        self.create_subscription(OperationModeState, "/api/operation_mode/state", self.on_op, latched)
        self.create_subscription(RouteState, "/api/routing/state", self.on_route, latched)
        self.create_subscription(VelocityFactorArray, "/api/planning/velocity_factors", self.on_vf, 10)
        self.create_timer(0.5, self.sample)

    def put(self, kind, **kw):
        kw.update(wall=round(time.time(), 3), kind=kind)
        self.out.write(json.dumps(kw) + "\n")

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pose = (round(p.x, 2), round(p.y, 2), round(p.z, 2))
        self.speed = round(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y), 2)

    def on_nvtl(self, m):
        self.nvtl = round(m.data, 3)
        if self.nvtl < self.warn and not self.low:
            self.put("nvtl_low", nvtl=self.nvtl, pose=self.pose)
        self.low = self.nvtl < self.warn

    def on_mrm(self, m):
        v = (MRM_STATE.get(m.state, m.state), MRM_BEHAVIOR.get(m.behavior, m.behavior))
        if v != self.mrm:
            self.put("mrm", state=v[0], behavior=v[1], pose=self.pose, speed=self.speed)
        self.mrm = v

    def on_op(self, m):
        self.op = OP_MODE.get(m.mode, m.mode) + ("" if m.is_autoware_control_enabled else "/manual")

    def on_route(self, m):
        self.route = ROUTE.get(m.state, m.state)

    def on_vf(self, m):
        seen = {}
        for f in m.factors:
            key = f.behavior
            v = (VF_STATUS.get(f.status, f.status), f.detail)
            seen[key] = v
            old = self.vf.get(key)
            if old is None or old != v:
                p = f.pose.position
                self.put("vf", behavior=f.behavior, status=v[0], detail=f.detail,
                         distance=round(f.distance, 2), at=(round(p.x, 2), round(p.y, 2)),
                         pose=self.pose, speed=self.speed)
        for key in set(self.vf) - set(seen):
            self.put("vf", behavior=key, status="GONE", pose=self.pose, speed=self.speed)
        self.vf = seen

    def sample(self):
        self.put("s", pose=self.pose, speed=self.speed, nvtl=self.nvtl,
                 mrm=self.mrm, op=self.op, route=self.route, vf=sorted(self.vf))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--nvtl-warn", type=float, default=2.3)
    a = ap.parse_args()
    rclpy.init()
    n = Monitor(a.out, a.nvtl_warn)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
