#!/usr/bin/env python3
"""Roadmap 072, the tour: which points the ego's lidar sees that the map's point cloud does not.

Subscribes, in the ego domain, to a point cloud in base_link (default the obstacle points
Autoware's planners stop for, /perception/obstacle_segmentation/pointcloud) and to
/localization/kinematic_state. Once a second it puts the cloud into the map frame with the pose
nearest its stamp and keeps the points farther than --far from every point of the map's PCD
(pointcloud_map/, 50 m tiles, loaded as the ego reaches them). Those are geometry the simulator has
and the scan had not: a ghost mesh, a shard, a vehicle the simulator spawned -- or the other way
round (the scan's own parked cars the CARLA map leaves out show up when the lidar sees through
where they were). Writes OUT.npz (x, y, z, d, wall) on SIGINT and every 60 s.

    python3 scripts/ntu_carla/tour_lidar_diff.py OUT.npz --pcd MAP_DIR/pointcloud_map
"""
import argparse
import bisect
import math
import pathlib
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial import cKDTree


def read_pcd(path):
    with open(path, "rb") as f:
        head = {}
        while True:
            line = f.readline().decode().strip()
            if line.startswith("#"):
                continue
            k, *v = line.split()
            head[k] = v
            if k == "DATA":
                break
        n = int(head["POINTS"][0])
        fields = head["FIELDS"]
        assert head["DATA"][0] == "binary" and all(s == "4" for s in head["SIZE"]), path
        a = np.frombuffer(f.read(n * 4 * len(fields)), np.float32).reshape(n, len(fields))
    return a[:, :3].astype(np.float64)


class Diff(Node):
    def __init__(self, a):
        super().__init__("tour_lidar_diff")
        self.a = a
        self.tiles, self.trees = {}, {}
        self.poses = []  # (t, x, y, z, qx, qy, qz, qw)
        self.keep = []
        self.last = 0.0
        self.saved = time.time()
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 50)
        self.create_subscription(PointCloud2, a.topic, self.on_cloud, qos_profile_sensor_data)

    def tree(self, x, y):
        k = (int(math.floor(x / 50.0)), int(math.floor(y / 50.0)))
        if k not in self.trees:
            p = pathlib.Path(self.a.pcd) / f"tile_{k[0]}_{k[1]}.pcd"
            self.trees[k] = cKDTree(read_pcd(p)) if p.exists() else None
        return self.trees[k]

    def on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.poses.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if len(self.poses) > 400:
            del self.poses[:200]

    def on_cloud(self, m):
        now = time.time()
        if now - self.last < self.a.every or not self.poses:
            return
        self.last = now
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        ts = [p[0] for p in self.poses]
        i = min(range(max(0, bisect.bisect_left(ts, t) - 1), min(len(ts), bisect.bisect_left(ts, t) + 1)),
                key=lambda j: abs(ts[j] - t))
        if abs(ts[i] - t) > 0.1:
            return
        _, px, py, pz, qx, qy, qz, qw = self.poses[i]
        r = point_cloud2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
        pts = np.stack([r["x"], r["y"], r["z"]], axis=1).astype(np.float64)
        if len(pts) == 0:
            return
        R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                      [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                      [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        w = pts @ R.T + (px, py, pz)
        w = w[np.hypot(w[:, 0] - px, w[:, 1] - py) < self.a.range]
        d = np.full(len(w), np.inf)
        keys = np.floor(w[:, :2] / 50.0).astype(int)
        for k in {tuple(x) for x in keys}:
            tr = self.tree(k[0] * 50.0 + 1, k[1] * 50.0 + 1)
            sel = (keys[:, 0] == k[0]) & (keys[:, 1] == k[1])
            if tr is not None:
                d[sel] = tr.query(w[sel], distance_upper_bound=5.0)[0]
        far = d > self.a.far
        if far.any():
            self.keep.append(np.c_[w[far], np.minimum(d[far], 9.9), np.full(far.sum(), now)])
        if now - self.saved > 60:
            self.save()

    def save(self):
        a = np.concatenate(self.keep) if self.keep else np.zeros((0, 5))
        np.savez_compressed(self.a.out, pts=a)
        self.saved = time.time()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("out")
    ap.add_argument("--pcd", default="/mnt/p5plus/aeon/ntu-twin/map/pointcloud_map")
    ap.add_argument("--topic", default="/perception/obstacle_segmentation/pointcloud")
    ap.add_argument("--far", type=float, default=0.5, help="m from the nearest PCD point")
    ap.add_argument("--range", type=float, default=40.0, help="m around the ego")
    ap.add_argument("--every", type=float, default=1.0, help="s between clouds")
    a = ap.parse_args()
    rclpy.init()
    n = Diff(a)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.save()


if __name__ == "__main__":
    main()
