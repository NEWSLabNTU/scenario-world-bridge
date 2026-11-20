"""interaction_strategy_watchdog: puts Autoware back to stock when the interaction_strategy node dies or stops cycling
(roadmap 078, R9).

The node puts the RTC modules it drives in manual mode while it holds a decision that is not stock's. If it then
dies (a crash, SIGKILL, an OOM kill) or stops cycling (a hang), its own shutdown handler never runs and the module
stays in manual mode: it waits at its line for a cooperate command that never comes, rather than run stock. The
node publishes a heartbeat every cycle (``/interaction_strategy/heartbeat``, std_msgs/String, JSON: its cycle
count, the modules it holds in manual mode, whether it holds a velocity limit), after the cycle's decision and
before its commands, so the heartbeat names a module before the module is switched. This companion, a separate
process with no state of the protocol, watches it: when no heartbeat has arrived for ``timeout_s`` (wall time),
it puts every module the last heartbeat held in manual mode (and every module in ``modules``) back in auto mode
through Autoware's own service (``/planning/enable_auto_mode/<module>``) and clears the node's velocity limit
(sender interaction_strategy), retrying each call every ``retry_s`` until Autoware answers success. A heartbeat
that arrives again (the node respawned) re-arms it. Nothing in Autoware changes: auto mode is the module's own
default, and the service is the one the node itself uses.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


HEARTBEAT = "/interaction_strategy/heartbeat"


class Watchdog(Node):
    def __init__(self):
        super().__init__("interaction_strategy_watchdog")
        from autoware_internal_planning_msgs.msg import VelocityLimitClearCommand
        from tier4_rtc_msgs.srv import AutoMode
        self.AutoMode, self.VelocityLimitClearCommand = AutoMode, VelocityLimitClearCommand
        self.declare_parameter("modules", ["intersection"])
        self.declare_parameter("timeout_s", 0.5)
        self.declare_parameter("retry_s", 1.0)
        self.modules = list(self.get_parameter("modules").value)
        self.timeout = float(self.get_parameter("timeout_s").value)
        self.retry = float(self.get_parameter("retry_s").value)
        self.last_rx = None  # monotonic time of the last heartbeat
        self.last = {"manual": [], "limited": False}
        self.tripped = False
        self.pending = {}  # module -> (future, sent at)
        self.done = set()
        self.auto = {}
        self.create_subscription(String, HEARTBEAT, self.on_hb, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.pub_clear = self.create_publisher(VelocityLimitClearCommand, "/planning/scenario_planning/clear_velocity_limit",
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.05, self.check)

    def client(self, m):
        if m not in self.auto:
            self.auto[m] = self.create_client(self.AutoMode, f"/planning/enable_auto_mode/{m}")  # rtc_interface's type
        return self.auto[m]

    def on_hb(self, msg):
        try:
            self.last = json.loads(msg.data)
        except ValueError:
            return
        self.last_rx = time.monotonic()
        if self.tripped:
            self.get_logger().info("heartbeat back: re-armed")
        self.tripped = False
        self.pending.clear()
        self.done.clear()

    def check(self):
        if self.last_rx is None:
            return  # the node has not started yet: nothing it could have switched
        now = time.monotonic()
        if not self.tripped and now - self.last_rx > self.timeout:
            self.tripped = True
            held = list(self.last.get("manual") or [])
            self.targets = sorted(set(held) | set(self.modules))
            self.get_logger().warn(f"no heartbeat for {now - self.last_rx:.2f} s (it held {held or 'none'} in manual mode): "
                                   f"modules {self.targets} back to auto mode, velocity limit cleared")
            self.pub_clear.publish(self.VelocityLimitClearCommand(stamp=self.get_clock().now().to_msg(), command=True,
                                                                  sender="interaction_strategy"))
        if not self.tripped:
            return
        for m in self.targets:
            if m in self.done:
                continue
            fut, sent = self.pending.get(m, (None, -1e9))
            if fut is not None and fut.done():
                res = fut.result()
                if res is not None and getattr(res, "success", True):
                    self.done.add(m)
                    self.get_logger().info(f"RTC {m}: auto mode restored")
                    continue
            if now - sent >= self.retry:
                req = self.AutoMode.Request()
                req.enable = True
                self.pending[m] = (self.client(m).call_async(req), now)


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
