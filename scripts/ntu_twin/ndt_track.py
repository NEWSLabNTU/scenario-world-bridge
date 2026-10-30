#!/usr/bin/env python3
"""Autoware's pose against the Isaac ego's true pose, for roadmap 059 N8 (aib's runner).

    ndt_track.py OUT.csv [--map-origin 52500,67700,0] [--mgrs 51RUH] [--duration 0]

aib's runner publishes no ground truth, but its GNSS fix is the GNSS link's position, exact.
With `--mgrs` (aib's runner given a map's world.yaml) the fix is MGRS-projected and the map
x/y come back through UTM (pyproj) less the square's corner; without it, aib's old fix about
(0, 0) (aib.frames.lat_lon) is inverted and map_origin added. base_link is the link
less its mount (tesla_model3 rig: 0.797, 0.004 m) turned by the heading. Map = world +
map_origin. Each row, at the kinematic state's rate: sim time, truth and estimate (map x, y,
yaw), the error along and across the true heading, and the latest NVTL. Prints a summary line
every 10 s and at the end.
"""
import argparse
import csv
import math
import sys

import rclpy
from autoware_internal_debug_msgs.msg import Float32Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS = 6378137.0
GNSS_MOUNT = (0.797072306, 0.003548091)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Track(Node):
    def __init__(self, out, origin, mgrs=""):
        super().__init__("ndt_track")
        self.origin = origin
        self.utm = None
        if mgrs:
            from pyproj import Transformer

            zone = int(mgrs[:-3])
            self.utm = Transformer.from_crs("EPSG:4326", f"EPSG:{32600 + zone}", always_xy=True)
            cols = ("ABCDEFGH", "JKLMNPQR", "STUVWXYZ")[(zone - 1) % 3]
            self.e0 = (cols.index(mgrs[-2]) + 1) * 100_000.0
            self.row = "ABCDEFGHJKLMNPQRSTUV".index(mgrs[-1]) - (5 if zone % 2 == 0 else 0)
        self.fix = None
        self.nvtl = float("nan")
        self.rows = []
        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "true_x", "true_y", "est_x", "est_y", "est_yaw", "along", "across",
                         "speed", "nvtl"])
        self.create_subscription(NavSatFix, "/sensing/gnss/gnss_link/nav_sat_fix", self.on_fix,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_state, 10)
        self.create_subscription(
            Float32Stamped, "/localization/pose_estimator/nearest_voxel_transformation_likelihood",
            lambda m: setattr(self, "nvtl", m.data), 10)
        self.create_timer(10.0, self.summary)

    def on_fix(self, m):
        if self.utm is not None:  # map frame directly: no origin to add later
            e, n = self.utm.transform(m.longitude, m.latitude)
            self.fix = (e - self.e0 - self.origin[0],
                        n % 2_000_000.0 - (self.row % 20) * 100_000.0 - self.origin[1])
            return
        lat = math.radians(m.latitude)
        y = lat * EARTH_RADIUS
        x = math.radians(m.longitude) * EARTH_RADIUS * math.cos(lat)
        self.fix = (x, y)

    def on_state(self, m):
        if self.fix is None:
            return
        p = m.pose.pose
        yaw = yaw_of(p.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        tx = self.fix[0] - (c * GNSS_MOUNT[0] - s * GNSS_MOUNT[1]) + self.origin[0]
        ty = self.fix[1] - (s * GNSS_MOUNT[0] + c * GNSS_MOUNT[1]) + self.origin[1]
        dx, dy = p.position.x - tx, p.position.y - ty
        along, across = c * dx + s * dy, -s * dx + c * dy
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        v = m.twist.twist.linear.x
        row = [round(t, 3), round(tx, 3), round(ty, 3), round(p.position.x, 3),
               round(p.position.y, 3), round(yaw, 5), round(along, 3), round(across, 3),
               round(v, 3), round(self.nvtl, 3)]
        self.w.writerow(row)
        self.rows.append(row)

    def summary(self):
        if not self.rows:
            self.get_logger().info("no rows yet")
            return
        r = self.rows[-200:]
        h = sorted(math.hypot(a[6], a[7]) for a in r)
        nv = [a[9] for a in r if a[9] == a[9]]
        self.get_logger().info(
            f"rows {len(self.rows)}  last: true ({r[-1][1]:.2f}, {r[-1][2]:.2f}) along "
            f"{r[-1][6]:+.2f} across {r[-1][7]:+.2f} v {r[-1][8]:.2f}  recent err p50 "
            f"{h[len(h) // 2]:.2f} max {h[-1]:.2f}  nvtl min {min(nv) if nv else float('nan'):.2f}")
        self.f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--map-origin", default="52500,67700,0")
    ap.add_argument("--mgrs", default="", help="the map's MGRS grid, e.g. 51RUH")
    ap.add_argument("--duration", type=float, default=0.0)
    a = ap.parse_args()
    rclpy.init()
    n = Track(a.out, [float(v) for v in a.map_origin.split(",")], a.mgrs)
    if a.duration:
        n.create_timer(a.duration, lambda: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.summary()
    n.f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
