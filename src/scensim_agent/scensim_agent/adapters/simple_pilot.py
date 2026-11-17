"""`simple_pilot`: a small autopilot on the vehicle interface, no Autoware (roadmap 070 D).

The SDK's reference for a non-Autoware autopilot, and the agent of the `simple` pilot kind. On
a goal it routes along the scenario's lanelet2 map from where the car is to the goal (through
the waypoints; simple_route.py) and drives the route: pure pursuit for the steering, a speed PID
for the acceleration, stopping at the goal. It drives through the vehicle interface every
vehicle bridge offers (acb on CARLA, aib on Isaac Sim), as Autoware's control would:

- `/control/command/control_cmd` (autoware_control_msgs/Control): the road wheels' angle, the
  acceleration and the target speed, `rate_hz` (30) times a second of simulation time;
- `/control/command/gear_cmd` (GearCommand): DRIVE, at the same rate;
- `/control/command/turn_indicators_cmd`, `/control/command/hazard_lights_cmd`: NO_COMMAND.

It reads the car's true pose (it does not localize): `odom_topic`, nav_msgs/Odometry of
base_link in the map frame. On CARLA that is acb's `/carla/ground_truth/odom`, always there; on
Isaac aib's `ground_truth.topic` of the ego config, `/ground_truth/odom` (off unless the ego
config turns it on: scensim's stack does for a `simple` pilot). Its speed is
`/vehicle/status/velocity_status`.

It ignores traffic: no lights, no other vehicles, no pedestrians. It follows the lanes and the
speed limits and stops at the scenario's goal; for a car that obeys the road, use Autoware.

    ros2 run scensim_agent simple_pilot --relay tcp://localhost:5560 --entity ego \\
        --config simple_pilot.yaml

Config (JSON or YAML, all optional):

    map_path: <map dir>            # the scenario's map (lanelet2_map.osm, map_projector_info.yaml)
    sim: carla                     # carla | isaac: picks odom_topic's default
    odom_topic: ""                 # the pose; empty: /carla/ground_truth/odom (carla),
                                   # /ground_truth/odom (isaac)
    vehicle: tesla_model3          # the car: its spec (sim_vehicle_spec) gives the wheelbase
    spec: ""                       # a spec file instead of the vehicle's
    wheel_base: null               # m; over the spec's
    max_steer_angle: null          # rad, the road wheels'; over the spec's
    rate_hz: 30                    # control commands per simulated second (>= 20)
    max_speed_kmh: 50              # never faster, whatever the map says
    default_speed_limit_kmh: 30    # a lanelet with no speed limit lanelet2 knows
    lateral_accel: 2.0             # m/s^2 in curves: the curvature speed limit
    accel_max: 1.5                 # m/s^2
    decel: 2.0                     # m/s^2, planned (to the goal, into curves, at a stop)
    brake_max: 5.0                 # m/s^2, the most it ever asks for
    kp: 0.8                        # speed PID, (m/s^2) per (m/s)
    ki: 0.15
    lookahead_min: 3.5             # m; pure pursuit's lookahead is min + time * speed
    lookahead_time: 0.6            # s
    lookahead_max: 20.0            # m
    arrive_tolerance: 1.0          # m from the goal, and below 0.3 m/s: ARRIVED
    pose_timeout: 45.0             # s at a reset to wait for the first pose
    route: auto                    # lanelet (needs map_path) | direct (straight lines through
                                   # the points) | auto: lanelet, direct in the fake world
                                   # without a map
    world: ros                     # fake: a kinematic car and no ROS (scensim-agent check, tests)
    fake_realtime: true            # the fake car ticks itself (false: the caller advances it)

Phases: IDLE after a reset, DRIVING once a goal has a route, ARRIVED at the goal, STOPPED on
stop, IDLE on clear_goal. A goal with no route (off the map, unreachable) fails the command
with the reason.
"""

import math
import sys
import threading
import time
from dataclasses import dataclass

import scensim_agent
from scensim_agent import protocol as P

from .simple_route import Path, RouteError, Router

DEFAULTS = {
    "map_path": "",
    "sim": "carla",
    "odom_topic": "",
    "vehicle": "tesla_model3",
    "spec": "",
    "wheel_base": None,
    "max_steer_angle": None,
    "rate_hz": 30.0,
    "max_speed_kmh": 50.0,
    "default_speed_limit_kmh": 30.0,
    "lateral_accel": 2.0,
    "accel_max": 1.5,
    "decel": 2.0,
    "brake_max": 5.0,
    "kp": 0.8,
    "ki": 0.15,
    "lookahead_min": 3.5,
    "lookahead_time": 0.6,
    "lookahead_max": 20.0,
    "arrive_tolerance": 1.0,
    "pose_timeout": 45.0,
    "route": "auto",
    "world": "ros",
    "fake_realtime": True,
    "use_sim_time": True,
}
ODOM_TOPICS = {"carla": "/carla/ground_truth/odom", "isaac": "/ground_truth/odom"}
# the Tesla Model 3's, when no spec can be read
FALLBACK_WHEEL_BASE = 3.005
FALLBACK_MAX_STEER = 0.70
HOLD_DECEL = 1.5  # m/s^2 asked for while holding still
POSE_STALE = 0.5  # s of sim time: an older pose is not driven on
FEEDFORWARD_TIME = 1.0  # s: how far ahead along the speed profile the feedforward looks
POSE_PERIOD = 0.2  # s between pose reports
SETTLE_DISTANCE = 3.0  # m: at a reset, the pose counts as the scenario's within this


# --- the car -------------------------------------------------------------------------------

def vehicle_geometry(cfg, log=None):
    """(wheel_base, max_steer_angle, spec_hash) from the config's overrides, else the car's
    spec (sim_vehicle_spec's vehicles/<vehicle>.yaml, or `spec`), else the Tesla's."""
    wheel_base, max_steer, spec_hash = None, None, ""
    path = cfg.get("spec") or ""
    if not path and cfg.get("vehicle"):
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory("sim_vehicle_spec")
            path = f"{share}/vehicles/{cfg['vehicle']}.yaml"
        except Exception:  # noqa: BLE001 -- no ROS environment, or not installed
            path = ""
    if path:
        try:
            import yaml
            with open(path) as f:
                doc = yaml.safe_load(f) or {}
            geometry = doc.get("geometry") or {}
            wheel_base = geometry.get("wheel_base")
            max_steer = geometry.get("max_steer_angle")
            try:
                from sim_vehicle_spec import spec as specmod
                spec_hash = specmod.spec_hash(specmod.load(path))
            except Exception:  # noqa: BLE001 -- the hash is optional
                spec_hash = ""
        except OSError as e:
            if log:
                log.warning(f"spec {path}: {e}; the Tesla Model 3's geometry")
    if cfg.get("wheel_base") is not None:
        wheel_base = cfg["wheel_base"]
    if cfg.get("max_steer_angle") is not None:
        max_steer = cfg["max_steer_angle"]
    return (float(wheel_base or FALLBACK_WHEEL_BASE), float(max_steer or FALLBACK_MAX_STEER),
            spec_hash)


@dataclass
class Command:
    steer: float = 0.0  # rad, the road wheels'
    accel: float = 0.0  # m/s^2
    velocity: float = 0.0  # m/s, the speed it is heading for
    arrived: bool = False


# --- the controller ------------------------------------------------------------------------

class Controller:
    """Pure pursuit and a speed PID along one `Path`; no ROS, no clock of its own."""

    def __init__(self, cfg, wheel_base, max_steer):
        self.cfg = cfg
        self.L = wheel_base
        self.max_steer = max_steer
        self.path = None
        self.s = 0.0
        self.limit = None  # m/s, the scenario's; None: none
        self.integral = 0.0
        self.last_steer = 0.0

    def set_path(self, path):
        self.path = path
        self.s = 0.0
        self.integral = 0.0

    def clear(self):
        self.path = None
        self.integral = 0.0

    def hold(self, v):
        """Brake to a stop and stay."""
        self.integral = 0.0
        if v > 0.3:
            return Command(self.last_steer, -float(self.cfg["decel"]) * 1.5, 0.0)
        return Command(self.last_steer, -HOLD_DECEL, 0.0)

    def target_speed(self, s, v):
        """The speed to drive at s: the map's limit, the scenario's, the config's maximum, each
        curve ahead and the goal, each reachable at `decel`."""
        c, path = self.cfg, self.path
        cap = float(c["max_speed_kmh"]) / 3.6
        if self.limit is not None:
            cap = min(cap, self.limit)
        decel, lat = float(c["decel"]), float(c["lateral_accel"])
        ref = min(cap, path.limit[path.index(s)])
        horizon = s + cap * cap / (2.0 * decel) + 5.0
        i = path.index(s)
        while i < len(path.s) and path.s[i] <= horizon:
            if path.s[i] > path.end:
                break
            vi = min(path.limit[i], cap)
            k = abs(path.curvature[i])
            if k > 1e-4:
                vi = min(vi, math.sqrt(lat / k))
            ref = min(ref, math.sqrt(vi * vi + 2.0 * decel * max(path.s[i] - s, 0.0)))
            i += 1
        remaining = max(path.end - s, 0.0)
        return min(ref, math.sqrt(2.0 * decel * remaining))

    def step(self, x, y, yaw, v, dt):
        """The command for a car at (x, y, yaw) at speed v, dt after the last step."""
        path, c = self.path, self.cfg
        if path is None:
            return self.hold(v)
        i = path.index(self.s)
        lo, hi = max(i - 3, 0), min(i + 40, len(path.s) - 1)
        self.s = max(self.s - 1.0, path.locate(x, y, lo, hi))
        remaining = path.end - self.s
        tol = float(c["arrive_tolerance"])
        if remaining <= tol and v < 0.3:
            self.last_steer = self.steer(x, y, yaw, v)
            return Command(self.last_steer, -HOLD_DECEL, 0.0, arrived=True)

        ref = 0.0 if remaining <= tol else self.target_speed(self.s, v)
        # feedforward: the acceleration the speed profile itself asks for here (slowing for a
        # curve or the goal), so the PID only corrects; without it the car lags the profile
        ds = max(v * FEEDFORWARD_TIME, 1.0)
        ahead = 0.0 if remaining - ds <= tol else self.target_speed(self.s + ds, v)
        feedforward = min((ahead * ahead - ref * ref) / (2.0 * ds), 0.0)
        err = ref - v
        self.integral = max(-2.0, min(2.0, self.integral + err * dt))
        accel = feedforward + float(c["kp"]) * err + float(c["ki"]) * self.integral
        decel = float(c["decel"])
        if remaining <= v * v / (2.0 * decel) + 1.0:  # the stop: brake for it, never pass it
            accel = min(accel, -v * v / (2.0 * max(remaining, 0.2)))
        accel = max(-float(c["brake_max"]), min(float(c["accel_max"]), accel))
        if accel >= float(c["accel_max"]) or accel <= -float(c["brake_max"]):
            self.integral -= err * dt  # anti-windup: no integration at the limits
        self.last_steer = self.steer(x, y, yaw, v)
        return Command(self.last_steer, accel, ref)

    def steer(self, x, y, yaw, v):
        """Pure pursuit from the rear axle (base_link) to the path point a lookahead ahead."""
        c = self.cfg
        ld = max(float(c["lookahead_min"]),
                 min(float(c["lookahead_max"]),
                     float(c["lookahead_min"]) + float(c["lookahead_time"]) * max(v, 0.0)))
        tx, ty = self.path.point(self.s + ld)
        dx, dy = tx - x, ty - y
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
        d = math.hypot(lx, ly)
        if d < 1e-3:
            return 0.0
        alpha = math.atan2(ly, lx)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), d)
        return max(-self.max_steer, min(self.max_steer, delta))


# --- the world: ROS, or a fake car -----------------------------------------------------------

def yaw_of(q):
    """Yaw of a quaternion (x, y, z, w)."""
    qx, qy, qz, qw = q
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def pose_dict(x, y, z, yaw):
    return {"x": x, "y": y, "z": z, "qx": 0.0, "qy": 0.0,
            "qz": math.sin(yaw / 2.0), "qw": math.cos(yaw / 2.0)}


@dataclass
class State:
    x: float
    y: float
    z: float
    yaw: float
    v: float
    stamp: float  # s, the world's clock


class FakeWorld:
    """A kinematic bicycle (rear axle) with a lag on the acceleration and the steering, ticked
    on a thread: what `scensim-agent check` and the tests drive. No ROS."""

    TICK = 0.05

    def __init__(self, cfg, wheel_base, log, realtime=True):
        self.L = wheel_base
        self.log = log
        self.x = self.y = self.z = self.yaw = 0.0
        self.v = 0.0
        self.accel = 0.0
        self.steer = 0.0
        self.command = Command()
        self.t = 0.0
        self.commands = 0
        self.on_tick = None
        self._stop = threading.Event()
        if realtime:
            threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop.wait(self.TICK):
            self.advance(self.TICK)

    def advance(self, dt):
        """Move the car by dt, then let the pilot command it."""
        c = self.command
        self.accel += (c.accel - self.accel) * min(dt / 0.2, 1.0)
        self.steer += (c.steer - self.steer) * min(dt / 0.1, 1.0)
        self.v = max(0.0, self.v + self.accel * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / self.L * math.tan(self.steer) * dt
        self.t += dt
        if self.on_tick:
            self.on_tick(self.t)

    def place(self, pose):
        self.x, self.y, self.z = pose["x"], pose["y"], pose["z"]
        self.yaw = yaw_of((pose["qx"], pose["qy"], pose["qz"], pose["qw"]))
        self.v = self.accel = 0.0

    def state(self):
        return State(self.x, self.y, self.z, self.yaw, self.v, self.t)

    def now(self):
        return self.t

    def publish(self, cmd):
        self.command = cmd
        self.commands += 1

    def close(self):
        self._stop.set()


class RosWorld:
    """The vehicle interface in the pilot's ROS domain: the pose and speed in, Autoware's
    control commands out, on a sim-time timer."""

    def __init__(self, cfg, entity, log):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from autoware_control_msgs.msg import Control
        from autoware_vehicle_msgs.msg import (GearCommand, HazardLightsCommand,
                                               TurnIndicatorsCommand, VelocityReport)
        from nav_msgs.msg import Odometry

        self.log = log
        self.on_tick = None
        self.m = {"Control": Control, "GearCommand": GearCommand,
                  "HazardLightsCommand": HazardLightsCommand,
                  "TurnIndicatorsCommand": TurnIndicatorsCommand}
        if not rclpy.ok():
            rclpy.init()
        name = "simple_pilot" if entity == "ego" else f"simple_pilot_{entity}"
        self.node = rclpy.create_node(name, parameter_overrides=[
            Parameter("use_sim_time", value=bool(cfg["use_sim_time"]))])
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.topic = cfg["odom_topic"] or ODOM_TOPICS.get(cfg["sim"], ODOM_TOPICS["carla"])
        self._lock = threading.Lock()
        self._odom = None  # (x, y, z, yaw, speed from the twist, stamp)
        self._speed = None  # (longitudinal velocity, stamp)
        n = self.node
        n.create_subscription(Odometry, self.topic, self._on_odom, reliable)
        n.create_subscription(VelocityReport, "/vehicle/status/velocity_status",
                              self._on_velocity, reliable)
        self.pub = {
            "control": n.create_publisher(Control, "/control/command/control_cmd", reliable),
            "gear": n.create_publisher(GearCommand, "/control/command/gear_cmd", reliable),
            "turn": n.create_publisher(TurnIndicatorsCommand,
                                       "/control/command/turn_indicators_cmd", reliable),
            "hazard": n.create_publisher(HazardLightsCommand,
                                         "/control/command/hazard_lights_cmd", reliable),
        }
        rate = max(float(cfg["rate_hz"]), 20.0)
        n.create_timer(1.0 / rate, self._timer)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(n)
        threading.Thread(target=self._executor.spin, daemon=True).start()
        log.info(f"pose from {self.topic}; commands at {rate:.0f} Hz of "
                 f"{'simulation' if cfg['use_sim_time'] else 'wall'} time")

    @staticmethod
    def _seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        tw = msg.twist.twist.linear
        with self._lock:
            self._odom = (p.x, p.y, p.z, yaw_of((q.x, q.y, q.z, q.w)),
                          math.hypot(tw.x, tw.y), self.now())

    def _on_velocity(self, msg):
        with self._lock:
            self._speed = (float(msg.longitudinal_velocity), self.now())

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def state(self):
        with self._lock:
            odom, speed = self._odom, self._speed
        if odom is None:
            return None
        x, y, z, yaw, v_twist, stamp = odom
        v = speed[0] if speed is not None and abs(speed[1] - stamp) < 1.0 else v_twist
        return State(x, y, z, yaw, v, stamp)

    def _timer(self):
        if self.on_tick:
            self.on_tick(self.now())

    def place(self, pose):
        """Nothing: the bridge placed the car."""

    def publish(self, cmd):
        m = self.m
        stamp = self.node.get_clock().now().to_msg()
        c = m["Control"]()
        c.stamp = c.control_time = stamp
        c.lateral.stamp = c.lateral.control_time = stamp
        c.lateral.steering_tire_angle = float(cmd.steer)
        c.lateral.steering_tire_rotation_rate = 0.0
        c.longitudinal.stamp = c.longitudinal.control_time = stamp
        c.longitudinal.velocity = float(cmd.velocity)
        c.longitudinal.acceleration = float(cmd.accel)
        self.pub["control"].publish(c)
        g = m["GearCommand"]()
        g.stamp, g.command = stamp, m["GearCommand"].DRIVE
        self.pub["gear"].publish(g)
        t = m["TurnIndicatorsCommand"]()
        t.stamp, t.command = stamp, m["TurnIndicatorsCommand"].NO_COMMAND
        self.pub["turn"].publish(t)
        h = m["HazardLightsCommand"]()
        h.stamp, h.command = stamp, m["HazardLightsCommand"].NO_COMMAND
        self.pub["hazard"].publish(h)

    def close(self):
        self._executor.shutdown()
        self.node.destroy_node()


# --- the agent -----------------------------------------------------------------------------

class SimplePilotAgent(scensim_agent.Agent):
    """Route on the lanelet map, pure pursuit, a speed PID. See the module docstring."""

    name = "simple_pilot"

    def __init__(self, config=None):
        super().__init__(config)
        self.cfg = {**DEFAULTS, **{k: v for k, v in self.config.items() if k in DEFAULTS}}
        self.wheel_base, self.max_steer, self.spec_hash = vehicle_geometry(self.cfg, self.log)
        self.ctl = Controller(self.cfg, self.wheel_base, self.max_steer)
        self.router = None
        if self.cfg["route"] in ("lanelet", "auto") and self.cfg["map_path"]:
            default = float(self.cfg["default_speed_limit_kmh"]) / 3.6
            self.router = Router.load(self.cfg["map_path"], default_speed=default)
            self.log.info(f"map {self.cfg['map_path']}: {len(self.router.lmap.laneletLayer)} "
                          "lanelets")
        if self.cfg["world"] == "fake":
            self.world = FakeWorld(self.cfg, self.wheel_base, self.log,
                                   realtime=bool(self.cfg["fake_realtime"]))
        else:
            self.world = RosWorld(self.cfg, self.entity, self.log)
        self.mode = "hold"  # hold | drive
        self._ctl = threading.RLock()
        self._last = None  # the world's clock at the last tick
        self._last_pose = 0.0
        self._stale_logged = False
        self.world.on_tick = self._tick
        self.log.info(f"wheel base {self.wheel_base:.3f} m, steering up to "
                      f"{math.degrees(self.max_steer):.0f} deg")

    # -- the control loop (the world's ticks) ----------------------------------------------
    def _tick(self, now):
        with self._ctl:
            dt = 0.0 if self._last is None else max(now - self._last, 0.0)
            self._last = now
            st = self.world.state()
            if st is None or now - st.stamp > POSE_STALE:
                if not self._stale_logged and self.mode == "drive":
                    self.log.warning("no fresh pose: holding")
                    self._stale_logged = True
                cmd = self.ctl.hold(0.0 if st is None else st.v)
            else:
                self._stale_logged = False
                cmd = (self.ctl.step(st.x, st.y, st.yaw, st.v, dt) if self.mode == "drive"
                       else self.ctl.hold(st.v))
            self.world.publish(cmd)
            if self.mode == "drive" and cmd.arrived:
                self.mode = "hold"
                self.ctl.clear()
                self.report(P.ARRIVED, detail="")
                self.log.info("arrived")
        if st is not None and time.monotonic() - self._last_pose >= POSE_PERIOD:
            self._last_pose = time.monotonic()
            self.report(pose=pose_dict(st.x, st.y, st.z, st.yaw))

    # -- hooks -----------------------------------------------------------------------------
    def on_reset(self, ep):
        with self._ctl:
            self.mode = "hold"
            self.ctl.clear()
            self.ctl.limit = None
            if ep.pose is not None:
                self.world.place(ep.pose)
        end = time.monotonic() + float(self.cfg["pose_timeout"])
        topic = getattr(self.world, "topic", "the fake world")
        while self.world.state() is None:
            if time.monotonic() > end:
                raise scensim_agent.Failed(f"no pose on {topic} after "
                                       f"{float(self.cfg['pose_timeout']):.0f} s")
            time.sleep(0.1)
        if ep.pose is not None:  # the bridge has teleported the car: wait for it to show
            settle = time.monotonic() + 5.0
            while time.monotonic() < settle:
                st = self.world.state()
                if math.hypot(st.x - ep.pose["x"], st.y - ep.pose["y"]) < SETTLE_DISTANCE:
                    break
                time.sleep(0.1)
            else:
                st = self.world.state()
                self.log.warning(f"the pose ({st.x:.1f}, {st.y:.1f}) is not the scenario's "
                                 f"({ep.pose['x']:.1f}, {ep.pose['y']:.1f}) after 5 s")
        self.report(P.IDLE, detail="")

    def plan(self, goal, waypoints):
        """The `Path` from the car to `goal` through `waypoints`."""
        st = self.world.state()
        if st is None:
            raise scensim_agent.Failed("no pose yet: cannot plan from where the car is")
        points = [(p["x"], p["y"], yaw_of((p["qx"], p["qy"], p["qz"], p["qw"])))
                  for p in list(waypoints) + [goal]]
        if self.router is None:
            if self.cfg["route"] == "lanelet" or (self.cfg["route"] == "auto"
                                                   and self.cfg["world"] != "fake"):
                raise scensim_agent.Failed("no map: give map_path (the scenario's map directory), "
                                       "or route: direct")
            limit = float(self.cfg["default_speed_limit_kmh"]) / 3.6
            return Path.from_points([(st.x, st.y)] + [(x, y) for x, y, _ in points], limit)
        try:
            return self.router.route((st.x, st.y, st.yaw), points)
        except RouteError as e:
            raise scensim_agent.Failed(f"no route: {e}") from None

    def on_goal(self, goal, waypoints):
        path = self.plan(goal, waypoints)
        with self._ctl:
            self.ctl.set_path(path)
            self.mode = "drive"
        self.report(P.DRIVING, detail="")
        self.log.info(f"driving {path.end:.0f} m to ({goal['x']:.1f}, {goal['y']:.1f}) through "
                      f"{len(waypoints)} waypoint(s)")

    def on_speed_limit(self, mps):
        with self._ctl:
            self.ctl.limit = None if mps is None else max(float(mps), 0.0)

    def on_stop(self):
        with self._ctl:
            self.mode = "hold"
            self.ctl.clear()

    def on_clear_goal(self):
        self.on_stop()
        self.report(P.IDLE)

    def close(self):
        self.world.close()


def main(argv=None):
    return scensim_agent.main(SimplePilotAgent, argv)


if __name__ == "__main__":
    sys.exit(main())
