#!/usr/bin/env python3
"""Roadmap 078: the interaction_strategy node's watchdog, tested by killing the node during a held decision.

A synthetic Autoware (this script's ``fake`` role, rclpy): the ego and one other car on the Town01 fixture's
crossing, both 20 m from the zone at 6 m/s (a tie: the node waits, not what stock would do), the intersection
module's RTC status, and the two RTC services the node and the watchdog call (enable_auto_mode, cooperate_commands),
each call logged with its wall time. The real node (czp.node, the fixture's zones) and the real watchdog
(czp.watchdog) run as separate processes in their own process groups.

Cases, each from a fresh node: ``kill`` (SIGKILL while the node holds the module in manual mode: no shutdown
handler runs), ``hang`` (SIGSTOP: the process stays, the cycle stops; then SIGCONT), ``term`` (SIGTERM: the
node's own restore). Per case: was the module in manual mode before, when did auto mode come back after the
signal, and who sent it. Pass: auto mode back within timeout_s + 0.5 s in every case.

Needs Autoware's messages (source /opt/autoware/1.5.0/setup.bash); the ROS domain and transport from the
environment (keep ROS_DOMAIN_ID <= 57 with the Cyclone localhost profile).

Usage: scripts/coop/czp_watchdog_check.py --out <dir> [--timeout 0.5]
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
CZP = os.path.join(REPO, "src", "czp")
FIX = os.path.join(CZP, "test", "fixtures", "town01_j35161.yaml")


def fake(log_path):
    sys.path.insert(0, CZP)
    import rclpy
    from autoware_perception_msgs.msg import ObjectClassification, PredictedObject, PredictedObjects, Shape
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tier4_rtc_msgs.msg import CooperateResponse, CooperateStatus, CooperateStatusArray, Module
    from tier4_rtc_msgs.srv import AutoModeWithModule, CooperateCommands
    from unique_identifier_msgs.msg import UUID

    from czp.geom import point_at
    from czp.zones import load_zones

    _, zones = load_zones(FIX)
    z = next(z for z in zones if z.kind == "cross_unsignalled")
    a0, a1 = z.approaches

    def pose_on(ap, s, length=4.79):
        (x, y), h = point_at(list(ap.centerline), ap.entry_s - s - length / 2)
        return x, y, h

    ego, oth = pose_on(a0, 20.0), pose_on(a1, 20.0)
    log = open(log_path, "a", buffering=1)

    class Fake(Node):
        def __init__(self):
            super().__init__("fake_autoware")
            rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self.po = self.create_publisher(Odometry, "/localization/kinematic_state", rel)
            self.pb = self.create_publisher(PredictedObjects, "/perception/object_recognition/objects", rel)
            self.ps = self.create_publisher(CooperateStatusArray, "/planning/cooperate_status/intersection", rel)
            self.create_service(AutoModeWithModule, "/planning/enable_auto_mode/intersection", self.on_auto)
            self.create_service(CooperateCommands, "/planning/cooperate_commands/intersection", self.on_cmd)
            self.auto_mode = True
            self.create_timer(0.1, self.tick)

        def on_auto(self, req, res):
            self.auto_mode = bool(req.enable)
            log.write(json.dumps({"t": time.time(), "call": "auto_mode", "enable": bool(req.enable)}) + "\n")
            res.success = True
            return res

        def on_cmd(self, req, res):
            for c in req.commands:
                log.write(json.dumps({"t": time.time(), "call": "command", "type": int(c.command.type)}) + "\n")
                res.responses.append(CooperateResponse(uuid=c.uuid, module=c.module, success=True))
            return res

        def tick(self):
            import math
            now = self.get_clock().now().to_msg()
            o = Odometry()
            o.header.stamp, o.header.frame_id = now, "map"
            o.pose.pose.position.x, o.pose.pose.position.y = ego[0], ego[1]
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(ego[2] / 2), math.cos(ego[2] / 2)
            o.twist.twist.linear.x = 6.0
            self.po.publish(o)
            objs = PredictedObjects()
            objs.header.stamp, objs.header.frame_id = now, "map"
            po = PredictedObject()
            po.object_id = UUID(uuid=[1] * 16)
            po.existence_probability = 1.0
            po.classification = [ObjectClassification(label=1, probability=1.0)]
            k = po.kinematics
            k.initial_pose_with_covariance.pose.position.x, k.initial_pose_with_covariance.pose.position.y = oth[0], oth[1]
            k.initial_pose_with_covariance.pose.orientation.z = math.sin(oth[2] / 2)
            k.initial_pose_with_covariance.pose.orientation.w = math.cos(oth[2] / 2)
            k.initial_twist_with_covariance.twist.linear.x = 6.0
            po.shape = Shape(type=Shape.BOUNDING_BOX)
            po.shape.dimensions.x, po.shape.dimensions.y, po.shape.dimensions.z = 4.79, 2.16, 1.5
            objs.objects = [po]
            self.pb.publish(objs)
            st = CooperateStatusArray()
            st.stamp = now
            s = CooperateStatus()
            s.stamp = now
            s.uuid = UUID(uuid=[0xab] * 16)
            s.module = Module(type=8)
            s.safe = True
            s.auto_mode = self.auto_mode
            s.start_distance, s.finish_distance = 20.0, 30.0
            st.statuses = [s]
            self.ps.publish(st)

    rclpy.init()
    n = Fake()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):  # noqa: BLE001
        pass


def start(cmd, log, env):
    return subprocess.Popen(cmd, stdout=open(log, "a"), stderr=subprocess.STDOUT, env=env, start_new_session=True)


def calls(path, since):
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if d["t"] >= since:
            out.append(d)
    return out


def wait_for(pred, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = pred()
        if r:
            return r
        time.sleep(0.02)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--role", default="check", choices=("check", "fake"))
    ap.add_argument("--log", default="")
    a = ap.parse_args()
    if a.role == "fake":
        fake(a.log)
        return 0
    os.makedirs(a.out, exist_ok=True)
    env = dict(os.environ, PYTHONPATH=CZP + os.pathsep + os.environ.get("PYTHONPATH", ""), PYTHONUNBUFFERED="1")
    calls_log = os.path.join(a.out, "rtc_calls.jsonl")
    open(calls_log, "w").close()
    procs = {}
    procs["fake"] = start([sys.executable, __file__, "--role", "fake", "--out", a.out, "--log", calls_log],
                          os.path.join(a.out, "fake.log"), env)
    procs["watchdog"] = start([sys.executable, "-m", "czp.watchdog", "--ros-args", "-p", f"timeout_s:={a.timeout}"],
                              os.path.join(a.out, "watchdog.log"), env)
    results = []
    try:
        for case in ("kill", "hang", "term"):
            t_start = time.time()
            node = start([sys.executable, "-m", "czp.node", "--ros-args", "-p", f"zones_file:={FIX}"],
                         os.path.join(a.out, f"node_{case}.log"), env)
            manual = wait_for(lambda: next((c for c in calls(calls_log, t_start)
                                            if c["call"] == "auto_mode" and not c["enable"]), None), 60.0)
            time.sleep(1.0)  # the decision held
            held = manual is not None and not any(c["call"] == "auto_mode" and c["enable"]
                                                  for c in calls(calls_log, manual["t"] if manual else t_start))
            t_sig = time.time()
            os.killpg(node.pid, {"kill": signal.SIGKILL, "hang": signal.SIGSTOP, "term": signal.SIGTERM}[case])
            back = wait_for(lambda: next((c for c in calls(calls_log, t_sig) if c["call"] == "auto_mode" and c["enable"]),
                                         None), 10.0)
            lat = None if back is None else round(back["t"] - t_sig, 3)
            r = {"case": case, "manual_before": manual is not None, "held_1s": held, "auto_back_s": lat,
                 "pass": manual is not None and held and lat is not None and lat <= a.timeout + 0.5}
            if case == "hang":
                os.killpg(node.pid, signal.SIGCONT)
                r["rearmed"] = wait_for(lambda: "re-armed" in open(os.path.join(a.out, "watchdog.log")).read(), 5.0) is not None
                os.killpg(node.pid, signal.SIGTERM)
            try:
                node.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(node.pid, signal.SIGKILL)
                node.wait(5)
            results.append(r)
            print(json.dumps(r), flush=True)
            time.sleep(1.5)
    finally:
        for p in procs.values():
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for p in procs.values():
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
    out = {"timeout_s": a.timeout, "cases": results, "pass": all(r["pass"] for r in results)}
    json.dump(out, open(os.path.join(a.out, "watchdog_check.json"), "w"), indent=1)
    print("PASS" if out["pass"] else "FAIL")
    return 0 if out["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
