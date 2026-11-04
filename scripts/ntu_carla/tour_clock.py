#!/usr/bin/env python3
"""Roadmap 072: /clock against wall time, in the ego domain, to find sim-time bursts.

Writes OUT.csv: kind, wall (time.time() at arrival), stamp. Kinds: `c` every /clock message, `e` the
EKF's biased pose (what ndt_scan_matcher buffers to interpolate a scan's initial pose), `p` the
scan NDT matches (its stamp). Every /clock message, so a
burst -- sim time advancing several frames in a few ms of wall time, which leaves the ego stack's
timers (the EKF's) no time to run between scans -- shows as a run of rows with small wall steps.

    python3 scripts/ntu_carla/tour_clock.py OUT.csv
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2


class C(Node):
    def __init__(self, out):
        super().__init__("tour_clock")
        self.f = open(out, "a", buffering=1 << 16)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", lambda m: self.put("c", m.clock), qos)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/localization/pose_twist_fusion_filter/biased_pose_with_covariance",
                                 lambda m: self.put("e", m.header.stamp), 100)
        self.create_subscription(PointCloud2, "/localization/util/downsample/pointcloud",
                                 lambda m: self.put("p", m.header.stamp), qos_profile_sensor_data)

    def put(self, k, t):
        self.f.write(f"{k},{time.time():.4f},{t.sec + t.nanosec * 1e-9:.4f}\n")


def main():
    rclpy.init()
    n = C(sys.argv[1])
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.f.close()


if __name__ == "__main__":
    main()
