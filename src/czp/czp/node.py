"""interaction_strategy: the conflict-zone protocol beside Autoware, in the ego's domain (roadmap 078 I4).

Inputs: /perception/object_recognition/objects, /localization/kinematic_state,
/planning/scenario_planning/trajectory, the RTC status of each driven module
(/planning/cooperate_status/<module>). Outputs: RTC cooperate commands
(/planning/cooperate_commands/<module>), the module's mode (/planning/enable_auto_mode/<module>:
manual only while the node holds a decision that is not stock's, auto again after), a velocity limit
as a candidate of Autoware's external velocity limit selector
(/planning/scenario_planning/max_velocity_candidates, sender interaction_strategy, so it composes with
the API's limit: the selector takes the lowest) and its clear command, and the decision record on
/interaction_strategy/decision (std_msgs/String, one JSON line per decision change; also logged).

Zones: ``zones_file`` (czp-zones' output for the map), else ``<map_path>/czp_zones.yaml``, else built
from ``<map_path>/lanelet2_map.osm`` in the background (minutes for a large town) and cached; until
then the node is stock (it sends nothing). With the node dead, late or unsure, Autoware runs stock
(R9): the contracts hand back to stock, and the modules are put back in auto mode on shutdown; a node that
dies without its shutdown (SIGKILL, a crash) or stops cycling is caught by ``czp.watchdog`` from the
heartbeat it publishes every cycle (/interaction_strategy/heartbeat).
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import threading
import time

import rclpy
import rclpy.parameter
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import zones as ZN
from .record import to_json
from .strategy import Obj, Status, Strategy

MODULE_TYPES = {"intersection": 8, "intersection_occlusion": 9, "blind_spot": 11, "lane_change_left": 1,
                "lane_change_right": 2}


def uuid_hex(u):
    return bytes(bytearray(u.uuid)).hex()


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class InteractionStrategy(Node):
    def __init__(self):
        super().__init__("interaction_strategy")
        from autoware_perception_msgs.msg import PredictedObjects
        from autoware_planning_msgs.msg import Trajectory
        from nav_msgs.msg import Odometry
        from tier4_planning_msgs.msg import VelocityLimit, VelocityLimitClearCommand
        from tier4_rtc_msgs.msg import CooperateStatusArray
        from tier4_rtc_msgs.srv import AutoModeWithModule, CooperateCommands
        self.VelocityLimit, self.VelocityLimitClearCommand = VelocityLimit, VelocityLimitClearCommand
        self.CooperateCommands, self.AutoModeWithModule = CooperateCommands, AutoModeWithModule
        self.declare_parameter("map_path", "")
        self.declare_parameter("zones_file", "")
        self.declare_parameter("modules", ["intersection"])
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("record_log", "")
        self.modules = list(self.get_parameter("modules").value)
        for k in ("map_path", "zones_file", "record_log"):  # the launch spells empty as none (rcl rejects "")
            if self.get_parameter(k).value == "none":
                self.set_parameters([rclpy.parameter.Parameter(k, value="")])
        self.strategy = None
        self.objects, self.ego, self.traj, self.obj_stamp = [], None, None, None
        self.statuses = {m: [] for m in self.modules}
        self.cycle_time = 0.0
        self.cycle_times = []
        rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects", self.on_objects, rel)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, rel)
        self.create_subscription(Trajectory, "/planning/scenario_planning/trajectory", self.on_traj, rel)
        for m in self.modules:
            self.create_subscription(CooperateStatusArray, f"/planning/cooperate_status/{m}",
                                     lambda msg, m=m: self.on_status(m, msg), rel)
        self.cmd = {m: self.create_client(CooperateCommands, f"/planning/cooperate_commands/{m}") for m in self.modules}
        self.auto = {m: self.create_client(AutoModeWithModule, f"/planning/enable_auto_mode/{m}") for m in self.modules}
        self.pub_limit = self.create_publisher(VelocityLimit, "/planning/scenario_planning/max_velocity_candidates",
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_clear = self.create_publisher(VelocityLimitClearCommand, "/planning/scenario_planning/clear_velocity_limit",
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub_rec = self.create_publisher(String, "/interaction_strategy/decision", 10)
        # the watchdog's heartbeat (czp.watchdog): every cycle, after the decision and before its commands
        self.pub_hb = self.create_publisher(String, "/interaction_strategy/heartbeat",
                                            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.n_cycles = 0
        self.log_path = self.get_parameter("record_log").value
        threading.Thread(target=self.load_zones, daemon=True).start()
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)

    # --- zones ---------------------------------------------------------------------------------------
    def load_zones(self):
        zf = self.get_parameter("zones_file").value
        mp = self.get_parameter("map_path").value
        cands = [zf] if zf else []
        if mp:
            cands.append(os.path.join(mp, "czp_zones.yaml"))
        for c in cands:
            if c and os.path.exists(c):
                _, zones = ZN.load_zones(c)
                self.ready(zones, c)
                return
        osm = os.path.join(mp, "lanelet2_map.osm") if mp else ""
        if not os.path.exists(osm):
            self.get_logger().warn("no zones and no lanelet2 map: the node stays stock")
            return
        sha = hashlib.sha256(open(osm, "rb").read()).hexdigest()[:16]
        cache = os.path.join(os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")), "czp", f"zones-{sha}.yaml")
        if not os.path.exists(cache):
            self.get_logger().info(f"building zones from {osm} (stock until done)")
            from .zones_tool import main as zones_main
            zones_main(["--map", osm, "--out", cache])
        _, zones = ZN.load_zones(cache)
        self.ready(zones, cache)

    def ready(self, zones, path):
        self.strategy = Strategy(zones, modules=self.modules)
        self.get_logger().info(f"{len(zones)} zones from {path}; driving modules {self.modules}")

    # --- inputs --------------------------------------------------------------------------------------
    def on_objects(self, msg):
        out = []
        for o in msg.objects:
            k = o.kinematics
            p = k.initial_pose_with_covariance.pose
            tw = k.initial_twist_with_covariance.twist.linear
            ac = k.initial_acceleration_with_covariance.accel.linear
            label = max(o.classification, key=lambda c: c.probability).label if o.classification else 0
            d = o.shape.dimensions
            out.append(Obj(uuid_hex(o.object_id), label, p.position.x, p.position.y, yaw_of(p.orientation),
                           math.hypot(tw.x, tw.y), ac.x, max(d.x, 0.5), max(d.y, 0.5)))
        self.objects = out
        self.obj_stamp = msg.header.stamp

    def on_odom(self, msg):
        p, tw = msg.pose.pose, msg.twist.twist.linear
        v = math.hypot(tw.x, tw.y)
        a = 0.0 if self.ego is None else self.ego[4]
        self.ego = (p.position.x, p.position.y, yaw_of(p.orientation), v, a)

    def on_traj(self, msg):
        self.traj = [(pt.pose.position.x, pt.pose.position.y) for pt in msg.points[::5]][:60]

    def on_status(self, module, msg):
        self.statuses[module] = [Status(module, uuid_hex(s.uuid), float(s.start_distance), bool(s.safe))
                                 for s in msg.statuses]

    # --- the cycle -----------------------------------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def heartbeat(self):
        st = self.strategy
        self.n_cycles += 1
        self.pub_hb.publish(String(data=json.dumps({"cycle": self.n_cycles, "manual": sorted(st.manual) if st else [],
                                                    "limited": bool(st.limited) if st else False})))

    def tick(self):
        if self.strategy is None or self.ego is None:
            self.heartbeat()
            return
        t0 = time.perf_counter()
        t = self.now_s()
        age = 0.0 if self.obj_stamp is None else max(0.0, t - (self.obj_stamp.sec + self.obj_stamp.nanosec * 1e-9))
        sts = [s for m in self.modules for s in self.statuses[m]]
        acts = self.strategy.cycle(t, self.ego, self.objects, self.traj, sts, age, self.cycle_time)
        self.heartbeat()  # names the modules this cycle switches to manual before the switch is sent
        self.apply(acts)
        for rec in acts.records:
            line = to_json(rec)
            self.pub_rec.publish(String(data=line))
            if self.log_path:
                with open(self.log_path, "a") as f:
                    f.write(line + "\n")
        self.cycle_time = time.perf_counter() - t0
        self.cycle_times.append(self.cycle_time)

    def apply(self, acts):
        from tier4_rtc_msgs.msg import Command, CooperateCommand, Module
        for m, enable in acts.auto_mode:
            req = self.AutoModeWithModule.Request()
            req.module = Module(type=MODULE_TYPES.get(m, 8))
            req.enable = enable
            self.auto[m].call_async(req)
            self.get_logger().info(f"RTC {m}: auto mode {enable}")
        by_mod = {}
        for m, u, activate in acts.rtc:
            from unique_identifier_msgs.msg import UUID
            c = CooperateCommand()
            c.uuid = UUID(uuid=list(bytes.fromhex(u)))
            c.module = Module(type=MODULE_TYPES.get(m, 8))
            c.command = Command(type=Command.ACTIVATE if activate else Command.DEACTIVATE)
            by_mod.setdefault(m, []).append(c)
        for m, cmds in by_mod.items():
            req = self.CooperateCommands.Request()
            req.stamp = self.get_clock().now().to_msg()
            req.commands = cmds
            self.cmd[m].call_async(req)
        if acts.velocity_limit == "clear":
            self.pub_clear.publish(self.VelocityLimitClearCommand(stamp=self.get_clock().now().to_msg(), command=True,
                                                                  sender="interaction_strategy"))
        elif acts.velocity_limit is not None:
            self.pub_limit.publish(self.VelocityLimit(stamp=self.get_clock().now().to_msg(),
                                                      max_velocity=float(acts.velocity_limit),
                                                      sender="interaction_strategy"))

    def restore(self):
        """Back to stock: every module this node put in manual mode to auto, the limit cleared."""
        if self.strategy is None:
            return
        from tier4_rtc_msgs.msg import Module
        for m in sorted(self.strategy.manual):
            req = self.AutoModeWithModule.Request()
            req.module = Module(type=MODULE_TYPES.get(m, 8))
            req.enable = True
            self.auto[m].call_async(req)
        if self.strategy.limited:
            self.pub_clear.publish(self.VelocityLimitClearCommand(command=True, sender="interaction_strategy"))
        if self.cycle_times:
            xs = sorted(self.cycle_times)
            self.get_logger().info(json.dumps({"cycles": len(xs), "p50_ms": round(1e3 * xs[len(xs) // 2], 3),
                                               "p99_ms": round(1e3 * xs[int(0.99 * (len(xs) - 1))], 3),
                                               "commands": self.strategy.counts}))


def main(args=None):
    rclpy.init(args=args)
    node = InteractionStrategy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.restore()
            rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
