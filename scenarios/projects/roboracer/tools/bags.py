"""Roadmap 091 W23: a run's log as the real car's rosbag2 (sqlite3), and back.

    python3 bags.py convert LOGS     every LOGS/<run>/streams.npz to LOGS/<run>/bag, then drop it

The topics are the ones the real car records (design note 5.1; f1tenth_system's vesc driver):

| topic | type | from the streams |
|---|---|---|
| `/commands/motor/speed`, `/commands/motor/current`, `/commands/motor/brake` | std_msgs/Float64 | `cmd_*`, on the mode's topic |
| `/commands/servo/position` | std_msgs/Float64 | `cmd_servo` |
| `/sensors/core` | vesc_msgs/VescStateStamped | `core_*`: speed (ERPM), current_motor, duty_cycle, voltage_input, displacement |
| `/sensors/imu/raw` | sensor_msgs/Imu | `imu_*` |
| the session's `reference.topic` (default `/reference/pose`) | geometry_msgs/PoseStamped | `ref_*` |

The stamps are the bag's clock. The session's `t0_ns` is the protocol's time zero on that clock:
when the protocol driver sent its first command. The real car's driver records it the same way.
A run's stream times are `stamp - t0_ns`, so the readers need no other clock.

Reading needs `vesc_msgs` in the ROS environment, as the real car's own stack has it (f1tenth's
`vesc` repository; 118 built it from commit 153998df8545fe1781b975df88e411b4e71d4bfe).
"""
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

BASE_NS = 1_000_000_000_000  # 1000 s: scensim's base time (time-and-ticking)
MOTOR_TOPICS = {"speed": "/commands/motor/speed", "current": "/commands/motor/current",
                "brake": "/commands/motor/brake"}
SERVO_TOPIC = "/commands/servo/position"
CORE_TOPIC = "/sensors/core"
IMU_TOPIC = "/sensors/imu/raw"
REF_TOPIC = "/reference/pose"
TYPES = {"std": "std_msgs/msg/Float64", "core": "vesc_msgs/msg/VescStateStamped",
         "imu": "sensor_msgs/msg/Imu", "ref": "geometry_msgs/msg/PoseStamped"}


def _ns(t):
    return BASE_NS + int(round(float(t) * 1e9))


def _stamp(msg, ns):
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)


def write(run_dir, motor_modes):
    """`run_dir`/streams.npz as `run_dir`/bag; the session gets `t0_ns` and the reference's
    topic. Returns the bag's path."""
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float64
    from vesc_msgs.msg import VescStateStamped

    run_dir = Path(run_dir)
    with np.load(run_dir / "streams.npz") as z:
        st = {k: z[k] for k in z.files}
    sess_path = run_dir / "session.yaml"
    sess = yaml.safe_load(sess_path.read_text())
    ref_topic = (sess.get("reference") or {}).get("topic") or REF_TOPIC
    out = run_dir / "bag"
    if out.exists():
        shutil.rmtree(out)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(out), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for name, kind in [(t, "std") for t in MOTOR_TOPICS.values()] + [
            (SERVO_TOPIC, "std"), (CORE_TOPIC, "core"), (IMU_TOPIC, "imu"), (ref_topic, "ref")]:
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=TYPES[kind],
                                                serialization_format="cdr"))
    msgs = []
    for t, mode, value, servo in zip(st["cmd_t"], st["cmd_mode"], st["cmd_value"],
                                     st["cmd_servo"]):
        msgs.append((_ns(t), MOTOR_TOPICS[motor_modes[int(mode)]], Float64(data=float(value))))
        msgs.append((_ns(t), SERVO_TOPIC, Float64(data=float(servo))))
    for i, t in enumerate(st["core_t"]):
        m = VescStateStamped()
        _stamp(m, _ns(t))
        m.state.speed = float(st["core_erpm"][i])
        m.state.current_motor = float(st["core_current_motor"][i])
        m.state.duty_cycle = float(st["core_duty"][i])
        m.state.voltage_input = float(st["core_voltage_input"][i])
        m.state.displacement = int(st["core_tachometer"][i])
        msgs.append((_ns(t), CORE_TOPIC, m))
    for i, t in enumerate(st["imu_t"]):
        m = Imu()
        _stamp(m, _ns(t))
        m.header.frame_id = "imu"
        a, g = m.linear_acceleration, m.angular_velocity
        a.x, a.y, a.z = (float(st[f"imu_a{c}"][i]) for c in "xyz")
        g.x, g.y, g.z = (float(st[f"imu_w{c}"][i]) for c in "xyz")
        msgs.append((_ns(t), IMU_TOPIC, m))
    for i, t in enumerate(st["ref_t"]):
        m = PoseStamped()
        _stamp(m, _ns(t))
        m.header.frame_id = "reference"
        p, q = m.pose.position, m.pose.orientation
        p.x, p.y, p.z = (float(st[f"ref_{c}"][i]) for c in "xyz")
        yaw = float(st["ref_yaw"][i])
        q.w, q.z = math.cos(yaw / 2), math.sin(yaw / 2)
        msgs.append((_ns(t), ref_topic, m))
    msgs.sort(key=lambda x: x[0])
    for ns, topic, m in msgs:
        w.write(topic, serialize_message(m), ns)
    del w
    sess["t0_ns"] = BASE_NS
    sess.setdefault("reference", {})["topic"] = ref_topic
    sess["log"] = "bag"
    sess_path.write_text(yaml.safe_dump(sess, sort_keys=False))
    return out


def read(run_dir, motor_modes):
    """(session, streams) of a run whose log is `run_dir`/bag: the streams as twin_runs.py
    writes them, times from the session's t0_ns."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    run_dir = Path(run_dir)
    sess = yaml.safe_load((run_dir / "session.yaml").read_text())
    t0 = int(sess["t0_ns"])
    ref_topic = (sess.get("reference") or {}).get("topic") or REF_TOPIC
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(run_dir / "bag"), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    mode_of = {v: k for k, v in MOTOR_TOPICS.items()}
    cmd, servo, core, imu, ref = [], {}, [], [], []
    while r.has_next():
        topic, data, ns = r.read_next()
        m = deserialize_message(data, types[topic])
        if topic in mode_of:
            cmd.append(((ns - t0) * 1e-9, motor_modes.index(mode_of[topic]), m.data))
        elif topic == SERVO_TOPIC:
            servo[ns] = m.data
        elif topic == CORE_TOPIC:
            s = m.state
            core.append((_t(m, t0), s.speed, s.current_motor, s.duty_cycle, s.voltage_input,
                         s.displacement))
        elif topic == IMU_TOPIC:
            a, g = m.linear_acceleration, m.angular_velocity
            imu.append((_t(m, t0), a.x, a.y, a.z, g.x, g.y, g.z))
        elif topic == ref_topic:
            p, q = m.pose.position, m.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ref.append((_t(m, t0), p.x, p.y, p.z, yaw))
    del r
    # a motor command and its servo position share a stamp (the driver sends both together)
    stamps = sorted(servo)
    cmd_rows = []
    for t, mode, value in cmd:
        ns = t0 + int(round(t * 1e9))
        k = min(stamps, key=lambda s: abs(s - ns)) if stamps else None
        cmd_rows.append((t, mode, value, servo.get(k, float("nan"))))
    out = {}
    for name, rows, cols in (("cmd", cmd_rows, ("t", "mode", "value", "servo")),
                             ("core", core, ("t", "erpm", "current_motor", "duty",
                                             "voltage_input", "tachometer")),
                             ("imu", imu, ("t", "ax", "ay", "az", "wx", "wy", "wz")),
                             ("ref", ref, ("t", "x", "y", "z", "yaw"))):
        a = np.asarray(rows, dtype=float)
        for j, c in enumerate(cols):
            out[f"{name}_{c}"] = a[:, j]
    return sess, out


def _t(m, t0):
    return (m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec - t0) * 1e-9


def convert(logs, motor_modes, keep_npz=False):
    """Every run under `logs` with a streams.npz to a bag; the npz removed unless kept."""
    n = 0
    for d in sorted(Path(logs).iterdir()):
        if (d / "streams.npz").is_file():
            write(d, motor_modes)
            if not keep_npz:
                (d / "streams.npz").unlink()
            n += 1
    return n


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "protocol"))
    import protocol as protomod
    if len(sys.argv) >= 3 and sys.argv[1] == "convert":
        print(f"{convert(sys.argv[2], protomod.MOTORS, '--keep-npz' in sys.argv)} runs to bags")
    else:
        sys.exit(__doc__)
