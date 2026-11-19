"""The spec's top lidar on CARLA at the comparison scene, recorded straight to a bag of
base_link clouds as acb_bridge would publish them (roadmap 084e): the spec's semantic lidar with
its generated attributes, intensity = reflectance[tag] x cos(incidence) clamped (acb's rule),
mounted where an acb-recorded bag's /tf_static puts `top` in base_link. Synchronous at 0.05 s.

The lidar spins at --rotation Hz, by default 20.00268 against the 20 Hz tick, so each scan
starts ~0.048 deg (an eighth of a column) further on and the columns sweep the world over the
run, as a real lidar's do. At exactly 20 Hz every scan fires the same azimuths, and a thin
object is hit in every scan or in none (084e).

carla_record.py PORT OUT SECONDS --mount-bag BAG [--rotation HZ] [--shift M] [--scene YAML]
                [--spec carla_sensors.yaml]
"""
import argparse, json, math, os, shutil, sys, time

import carla
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scene as scenemod  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
ap.add_argument("port", type=int)
ap.add_argument("out")
ap.add_argument("seconds", type=float)
ap.add_argument("--mount-bag", required=True,
                help="a bag acb_bridge recorded with /tf_static (top's mount in base_link)")
ap.add_argument("--rotation", type=float, default=20.00268, help="lidar rotation, Hz")
ap.add_argument("--shift", type=float, default=0.0, help="the whole scene along the heading, m")
ap.add_argument("--scene", default=None)
ap.add_argument("--spec", default=os.path.join(HERE, "..", "..", "src", "sim_vehicle_spec",
                                               "generated", "tesla_model3", "carla_sensors.yaml"))
args = ap.parse_args()
port, out, secs, rot, shift = args.port, args.out, args.seconds, args.rotation, args.shift
top = yaml.safe_load(open(args.spec))["sensors"]["top"]
refl = np.array(top["intensity"]["reflectance"], dtype=np.float64)

# top in base_link, from the acb bag's tf_static
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=args.mount_bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
st = {}
while r.has_next():
    t, data, _ = r.read_next()
    if t == "/tf_static":
        for tr in deserialize_message(data, TFMessage).transforms: st[tr.child_frame_id] = tr
if "top" not in st:
    sys.exit(f"{args.mount_bag}: no `top` in /tf_static")
R, tv, frame = np.eye(3), np.zeros(3), "top"
while frame != "base_link":
    tr = st[frame]; q = tr.transform.rotation; p = tr.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    Rq = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    R, tv = Rq @ R, Rq @ tv + np.array([p.x, p.y, p.z]); frame = tr.header.frame_id
roll = math.atan2(R[2, 1], R[2, 2]); pitch = -math.asin(R[2, 0]); yaw = math.atan2(R[1, 0], R[0, 0])

s = scenemod.load(args.scene) if args.scene else scenemod.load()
c = carla.Client("localhost", port); c.set_timeout(60)
if not c.get_server_version().startswith(s["body"].split("-", 1)[1]):
    raise SystemExit(f"the scene's bodies are {s['body']}'s; this CARLA is {c.get_server_version()}")
w = c.get_world()
if not w.get_map().name.endswith(s["map"]):
    w = c.load_world(s["map"])
old = w.get_settings(); st_ = w.get_settings()
st_.synchronous_mode, st_.fixed_delta_seconds = True, 0.05
w.apply_settings(st_)
bl = w.get_blueprint_library()
e = s["ego"]["carla"]; rear = s["ego"]["rear_axle_m"]
yaw0 = math.radians(e["yaw"])
ex, ey = e["x"] + math.cos(yaw0) * shift, e["y"] + math.sin(yaw0) * shift   # along the heading
actors = []
try:
    hero_bp = bl.find(s["ego"]["blueprint"]); hero_bp.set_attribute("role_name", "hero")
    hero = w.spawn_actor(hero_bp, carla.Transform(carla.Location(ex, ey, e["z"]), carla.Rotation(yaw=e["yaw"])))
    actors.append(hero)
    hero.apply_control(carla.VehicleControl(hand_brake=True, brake=1.0))
    for _ in range(20): w.tick()
    ht = hero.get_transform(); hy = math.radians(ht.rotation.yaw)
    for name, car in s["cars"].items():
        dx, dy = car["centre"]; ax, ay = dx + rear, -dy
        x = ht.location.x + math.cos(hy) * ax - math.sin(hy) * ay
        y = ht.location.y + math.sin(hy) * ax + math.cos(hy) * ay
        a = w.spawn_actor(bl.find(s["car_blueprint"]), carla.Transform(
            carla.Location(x, y, ht.location.z + 0.5), carla.Rotation(yaw=ht.rotation.yaw - car["yaw"])))
        actors.append(a)
    for _ in range(40): w.tick()
    lb = bl.find(top["blueprint"])
    for k, v in top["parameters"].items():
        lb.set_attribute(k, str(v))
    lb.set_attribute("rotation_frequency", str(rot))
    # top's pose in the actor frame: base_link is rear_axle_m along x from the actor's origin;
    # CARLA's y and the yaw/pitch signs are ROS's negated
    mount = carla.Transform(carla.Location(tv[0] + rear, -tv[1], tv[2]),
                            carla.Rotation(roll=math.degrees(roll), pitch=-math.degrees(pitch),
                                           yaw=-math.degrees(yaw)))
    lidar = w.spawn_actor(lb, mount, attach_to=hero); actors.append(lidar)
    clouds = []
    lidar.listen(lambda m: clouds.append(m))
    for _ in range(10): w.tick()
    clouds.clear()
    for _ in range(int(secs / 0.05)): w.tick(); time.sleep(0.002)
    time.sleep(0.5)
    lidar.stop()
    truth = {}
    inv = np.array(hero.get_transform().get_inverse_matrix())
    for a in actors[1:-1]:
        l = a.get_transform().transform(a.bounding_box.location)
        p = inv @ np.array([l.x, l.y, l.z, 1.0])
        truth[[k for k in s["cars"]][actors.index(a) - 1]] = [round(p[0] - rear, 2), round(-p[1], 2)]
finally:
    for a in actors[::-1]:
        a.destroy()
    w.apply_settings(old)

if os.path.exists(out): shutil.rmtree(out)
os.makedirs(out); json.dump(truth, open(f"{out}/truth.json", "w"), indent=1)
step = (float(top["parameters"]["upper_fov"]) - float(top["parameters"]["lower_fov"])) / (int(top["parameters"]["channels"]) - 1)
lo = float(top["parameters"]["lower_fov"])
wr = rosbag2_py.SequentialWriter()
wr.open(rosbag2_py.StorageOptions(uri=f"{out}/bag", storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
TOPIC = "/sensing/lidar/top/pointcloud_before_sync"
wr.create_topic(rosbag2_py.TopicMetadata(name=TOPIC, type="sensor_msgs/msg/PointCloud2", serialization_format="cdr"))
wr.create_topic(rosbag2_py.TopicMetadata(name="/tf_static", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"))
wr.write("/tf_static", serialize_message(TFMessage()), 1)
dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "u1"), ("r", "u1"), ("c", "<u2")])
starts = []
for k, m in enumerate(clouds):
    raw = np.frombuffer(m.raw_data, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                                     ("cos", "<f4"), ("idx", "<u4"), ("tag", "<u4")]))
    ps = np.stack([raw["x"], -raw["y"], raw["z"]], 1).astype(np.float64)      # ROS sensor frame
    el = np.degrees(np.arctan2(ps[:, 2], np.hypot(ps[:, 0], ps[:, 1])))
    b = ps @ R.T + tv                                                           # base_link
    a = np.zeros(len(raw), dt)
    a["x"], a["y"], a["z"] = b[:, 0], b[:, 1], b[:, 2]
    a["i"] = np.clip(np.rint(refl[np.minimum(raw["tag"], len(refl) - 1)] * np.abs(raw["cos"])), 0, 255)
    a["c"] = np.clip(np.rint((el - lo) / step), 0, 127)
    starts.append(float(np.degrees(np.arctan2(ps[0, 1], ps[0, 0]))) if len(ps) else 0.0)
    msg = PointCloud2(); msg.header.frame_id = "base_link"
    msg.header.stamp.sec, msg.header.stamp.nanosec = 1000 + k // 20, (k % 20) * 50_000_000
    msg.height, msg.width, msg.point_step = 1, len(a), 16
    msg.row_step = 16 * len(a); msg.is_dense = True
    msg.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in
                  (("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 2), ("return_type", 13, 2), ("channel", 14, 4))]
    msg.data = a.tobytes()
    wr.write(TOPIC, serialize_message(msg), 2 + k)
print(out, "clouds", len(clouds), "pts/cloud", np.mean([len(m.raw_data) // 24 for m in clouds]).round(0),
      "rotation", rot, "shift", shift, "first-point azimuth of scans 0-5:", np.round(starts[:6], 3), "truth", truth)
