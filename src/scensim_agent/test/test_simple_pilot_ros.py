"""simple_pilot's ROS side against a stand-in vehicle bridge: it reads the pose and speed and
publishes Autoware's control, gear, indicator and hazard commands at its rate. Wall time (no
/clock), its own ROS domain on localhost so it never meets a live stack; skipped without
Autoware's message packages."""

import os

os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ.pop("CYCLONEDDS_URI", None)  # as test_ros_adapter.py: Cyclone and localhost-only

import threading  # noqa: E402
import time  # noqa: E402

import pytest  # noqa: E402

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("autoware_control_msgs")
pytest.importorskip("autoware_vehicle_msgs")

from autoware_control_msgs.msg import Control  # noqa: E402
from autoware_vehicle_msgs.msg import (GearCommand, HazardLightsCommand,  # noqa: E402
                                       TurnIndicatorsCommand, VelocityReport)
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from scensim_agent import Episode  # noqa: E402
from scensim_agent import protocol as P  # noqa: E402
from scensim_agent.adapters.simple_pilot import SimplePilotAgent, pose_dict  # noqa: E402

ODOM = "/test_simple_pilot/odom"
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class FakeBridge:
    """Publishes the car's odometry and speed at 20 Hz; records the commands."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("test_simple_pilot_bridge")
        self.x = 0.0
        self.controls, self.gears, self.turns, self.hazards = [], [], [], []
        n = self.node
        self.odom = n.create_publisher(Odometry, ODOM, RELIABLE)
        self.speed = n.create_publisher(VelocityReport, "/vehicle/status/velocity_status",
                                        RELIABLE)
        n.create_subscription(Control, "/control/command/control_cmd",
                              lambda m: self.controls.append((time.monotonic(), m)), RELIABLE)
        n.create_subscription(GearCommand, "/control/command/gear_cmd",
                              lambda m: self.gears.append(m), RELIABLE)
        n.create_subscription(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd",
                              lambda m: self.turns.append(m), RELIABLE)
        n.create_subscription(HazardLightsCommand, "/control/command/hazard_lights_cmd",
                              lambda m: self.hazards.append(m), RELIABLE)
        n.create_timer(0.05, self._publish)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(n)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _publish(self):
        o = Odometry()
        o.header.stamp = self.node.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = "map", "base_link"
        o.pose.pose.position.x = self.x
        o.pose.pose.orientation.w = 1.0
        self.odom.publish(o)
        v = VelocityReport()
        v.header.stamp = o.header.stamp
        v.longitudinal_velocity = 0.0
        self.speed.publish(v)

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()


def wait_for(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def test_commands_on_the_vehicle_interface():
    bridge = FakeBridge()
    agent = SimplePilotAgent({"odom_topic": ODOM, "use_sim_time": False, "route": "direct",
                              "pose_timeout": 10.0})
    try:
        agent.on_reset(Episode(pose=pose_dict(0.0, 0.0, 0.0, 0.0)))
        assert agent.phase == P.IDLE
        assert wait_for(lambda: len(bridge.controls) > 5)
        assert all(m.longitudinal.velocity == 0.0 and m.longitudinal.acceleration < 0
                   for _, m in bridge.controls[-5:])  # holding before a goal
        agent.on_goal(pose_dict(40.0, 0.0, 0.0, 0.0), [])
        assert agent.phase == P.DRIVING
        start = len(bridge.controls)
        time.sleep(1.0)
        driving = bridge.controls[start:]
        assert len(driving) >= 20  # at least 20 Hz
        assert all(m.longitudinal.acceleration > 0 and m.longitudinal.velocity > 0
                   for _, m in driving[2:])
        assert all(abs(m.lateral.steering_tire_angle) < 1e-6 for _, m in driving)
        assert bridge.gears and all(g.command == GearCommand.DRIVE for g in bridge.gears)
        assert bridge.turns[-1].command == TurnIndicatorsCommand.NO_COMMAND
        assert bridge.hazards[-1].command == HazardLightsCommand.NO_COMMAND
        assert agent.state_message()["pose"]["x"] == pytest.approx(0.0)
        agent.on_stop()
        n = len(bridge.controls)
        assert wait_for(lambda: len(bridge.controls) > n + 3)
        assert all(m.longitudinal.velocity == 0.0 and m.longitudinal.acceleration < 0
                   for _, m in bridge.controls[n + 1:])
    finally:
        agent.close()
        bridge.close()


def test_reset_fails_without_a_pose():
    agent = SimplePilotAgent({"odom_topic": "/test_simple_pilot/nothing", "use_sim_time": False,
                              "pose_timeout": 1.0})
    try:
        import scensim_agent
        with pytest.raises(scensim_agent.Failed, match="/test_simple_pilot/nothing"):
            agent.on_reset(Episode(pose=None))
    finally:
        agent.close()
