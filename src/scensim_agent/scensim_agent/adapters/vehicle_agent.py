"""vehicle_agent: the Autoware vehicle agent as a ROS node (roadmap 051; was acb_pilot's agent).

The agent itself is `scensim_agent.adapters.autoware.AutowareAgent`: Autoware's ADAPI as a
`scensim_agent.Agent`, simulator-neutral. This node runs it with ROS parameters, so a launch file
starts it beside any vehicle side: scensim's ego launches pass
`scensim_launch vehicle_agent.launch.xml` to the vehicle bridge's launch as `extra_launch`.

Two modes:

- relay (param `relay`, e.g. tcp://sim-host:5560): register as `entity`, take commands
  from the scenario (protocol v2: a `reset` at each scenario start), report state.
  Reconnects whenever the relay goes away; runs until stopped.
- local (no `relay`): drive to `goal_poses_file`'s goal_pose once Autoware is localized,
  and exit 0 on arrival, 1 on timeout -- a vehicle side testable with no scenario at all.

ROS parameters:
    relay (str, default ""): tcp://host:port of the agent relay; empty = local mode
    entity (str, default "ego"): the scenario entity this vehicle is
    goal_poses_file (str, default ""): local mode's goal (autoware_sim_pilot poses format)
    timeout (float, default 600.0): local mode's budget, from start to arrival
    agent_config (str, default ""): a YAML/JSON file of AutowareAgent settings (its
        DEFAULTS: converge_timeout, ready_timeout, scan_wait, ...)
    vehicle_model (str, default "", or "none"): Autoware's vehicle model (`<name>_description`); its
        `config/spec_hash.yaml` (autoware_sim_vehicle, roadmap 051) is the spec_hash the
        agent registers with, which the scenario bridge compares with the world's and the
        catalog's. Empty, or a package without the file: none is sent.
    spec_hash_file (str, default "", or "none"): a YAML file with key `spec_hash`, used instead
        of `vehicle_model`'s package (a car whose description lives in another repository).
        A file that is missing or unreadable: an error, and none is sent.

Usage:
    ros2 run scensim_agent vehicle_agent --ros-args -p relay:=tcp://localhost:5560 -p entity:=ego \\
        -p use_sim_time:=true
"""

import logging
import os
import signal
import sys
import threading
import time

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node

import scensim_agent
from scensim_agent import protocol as P
from scensim_agent.adapters.autoware import AutowareAgent
from scensim_agent.adapters.autoware_ros import AutowarePort


class AgentNode(Node):
    def __init__(self):
        super().__init__("vehicle_agent")
        self.declare_parameter("relay", "")
        self.declare_parameter("entity", "ego")
        self.declare_parameter("goal_poses_file", "")
        self.declare_parameter("timeout", 600.0)
        self.declare_parameter("agent_config", "")
        self.declare_parameter("vehicle_model", "")
        self.declare_parameter("spec_hash_file", "")
        self.relay = self.get_parameter("relay").value
        self.entity = self.get_parameter("entity").value
        self.goal_file = self.get_parameter("goal_poses_file").value
        self.timeout = float(self.get_parameter("timeout").value)
        path = self.get_parameter("agent_config").value
        config = scensim_agent.load_config(path) if path else {}
        config["entity"] = self.entity
        self.agent = AutowareAgent(config, port=AutowarePort(self))
        self.agent.spec_hash = spec_hash_of(self.get_parameter("vehicle_model").value,
                                            self.get_logger(),
                                            self.get_parameter("spec_hash_file").value)


def spec_hash_of(vehicle_model, log, spec_hash_file=""):
    """The spec_hash in `spec_hash_file` when one is given, else in
    `<vehicle_model>_description/config/spec_hash.yaml`; "" when there is none (a vehicle
    description not generated from a sim_vehicle_spec spec)."""
    if spec_hash_file not in ("", "none"):
        try:
            with open(spec_hash_file) as f:
                value = str((yaml.safe_load(f) or {}).get("spec_hash") or "")
        except (OSError, yaml.YAMLError, AttributeError) as e:
            log.error(f"no spec_hash from spec_hash_file {spec_hash_file!r}: {e}")
            return ""
        log.info(f"spec_hash_file {spec_hash_file}: spec_hash {value[:12]}")
        return value
    if vehicle_model in ("", "none"):
        return ""
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

    try:
        share = get_package_share_directory(f"{vehicle_model}_description")
        with open(f"{share}/config/spec_hash.yaml") as f:
            value = str((yaml.safe_load(f) or {}).get("spec_hash") or "")
    except (PackageNotFoundError, OSError) as e:
        log.warning(f"no spec_hash for vehicle model {vehicle_model!r}: {e}")
        return ""
    log.info(f"vehicle model {vehicle_model}: spec_hash {value[:12]}")
    return value


def run_relay(node, stop):
    host, port = scensim_agent.parse_relay(node.relay)
    client = scensim_agent.Client(node.agent, host, port)
    thread = threading.Thread(target=client.run, daemon=True)
    thread.start()
    node.get_logger().info(f"agent for entity {node.entity!r}: relay tcp://{host}:{port}")
    try:
        while not stop.wait(0.5):
            pass
    finally:
        client.stop()
    return 0


def run_local(node, stop):
    """No relay: one drive to the goal file's goal_pose, like auto_drive."""
    log = node.get_logger()
    if not node.goal_file:
        log.fatal("Neither `relay` nor `goal_poses_file` is set: nothing to drive to")
        return 1
    with open(node.goal_file) as f:
        goal = yaml.safe_load(f)["goal_pose"]
    goal = {k: float(goal.get(k, 0.0 if k != "qw" else 1.0)) for k in P.POSE_KEYS}
    log.info(f"local mode: goal ({goal['x']:.2f}, {goal['y']:.2f}) from {node.goal_file}")
    agent = node.agent
    end = time.monotonic() + node.timeout
    sent = False
    last_phase = None
    while not stop.is_set() and time.monotonic() < end:
        phase = agent.derived_phase()
        if phase != last_phase:
            log.info(f"phase {phase}")
            last_phase = phase
        if not sent and phase == P.IDLE:
            try:
                agent.on_goal(goal, [])
                sent = True
            except scensim_agent.Failed as e:
                log.warning(f"route refused ({e}); retrying")
                time.sleep(5.0)
        elif sent and phase == P.ARRIVED:
            log.info("arrived")
            return 0
        time.sleep(0.1)
    log.error(f"did not arrive within {node.timeout:.0f} s (phase {last_phase})")
    return 1


CLOSE_BUDGET = 3.0  # s for the whole close, well under play_launch's 2 s + 5 s to SIGKILL
JOIN_TIMEOUT = 1.0  # s per thread join: a stuck thread costs this, never a hang


def install_stop_handlers(stop: threading.Event):
    """SIGINT and SIGTERM set ``stop``. rclpy's own handlers are not installed
    (`SignalHandlerOptions.NO`): on SIGTERM they shut the context down and raise nothing, so
    the agent's threads failed on a dead context while the process ran on until play_launch
    SIGKILLed it; on SIGINT the context went while the executor still spun, and the process
    aborted ("terminate called without an active exception", 077 G72)."""
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())


def close_in_order(steps, budget=CLOSE_BUDGET, log=None) -> bool:
    """Run ``steps`` (``(name, fn)``) in order on a thread, within ``budget`` seconds in all.
    False, naming the step it was in, when the budget ran out: the caller then leaves at once
    (os._exit) rather than wait on a stuck rclpy call."""
    at = {"step": None}

    def run():
        for name, fn in steps:
            at["step"] = name
            try:
                fn()
            except Exception as e:  # a step that fails must not stop the ones after it
                if log is not None:
                    log.warning(f"close: {name} failed: {e}")
        at["step"] = None

    t = threading.Thread(target=run, daemon=True)
    t.start()
    t.join(timeout=budget)
    if t.is_alive():
        if log is not None:
            log.error(f"close: still in {at['step']} after {budget:g} s; exiting without it")
        return False
    return True


def main():
    # the SDK logs through `logging` (commands, phases, reconnects): to stderr, as before
    logging.basicConfig(level=logging.INFO,
                        format="[%(levelname)s] [%(created).6f] [%(name)s]: %(message)s")
    stop = threading.Event()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    install_stop_handlers(stop)
    node = AgentNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    code = 1
    log = logging.getLogger("scensim_agent.vehicle_agent")
    try:
        code = (run_relay if node.relay else run_local)(node, stop)
    except KeyboardInterrupt:
        code = 0
    except Exception:  # an adapter or protocol bug: said once, then the same bounded close
        log.exception("the agent failed")
        code = 1
    stop.set()
    # the agent's step thread before the executor, the executor (and its spin thread) before
    # the node, the node before the context: nothing left running on what is torn down
    clean = close_in_order([
        ("agent.close", lambda: node.agent.close(timeout=JOIN_TIMEOUT)),
        ("executor.shutdown", lambda: executor.shutdown(timeout_sec=JOIN_TIMEOUT)),
        ("spin thread", lambda: spinner.join(timeout=JOIN_TIMEOUT)),
        ("destroy_node", node.destroy_node),
        ("rclpy shutdown", rclpy.try_shutdown),
    ], log=log)
    sys.stdout.flush()
    sys.stderr.flush()
    if not clean:
        os._exit(code)
    sys.exit(code)


if __name__ == "__main__":
    main()
