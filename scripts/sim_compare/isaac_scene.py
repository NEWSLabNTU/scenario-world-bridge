"""The comparison scene on Isaac: the ego (aib's Tesla Model 3, the world server's PhysX ego
with its RTX lidar) and the parked cars of the scene file, over the world protocol (REQ to the
Isaac world server, as the bridge drives it). Steps the world for HOLD s of wall time (20 Hz
frames), writes the cars' box centres in the ego's base_link (rear axle centre on the ground)
to OUT, as carla_scene.py does, then despawns and releases the server. Touching OUT.stop ends
the hold early.

Run in Isaac's venv python with scensim_world_server on PYTHONPATH:
    isaac_scene.py <port> <world.yaml> <map dir> <hold s> <out truth.json> [<scene yaml>]
"""
import json, math, os, socket, sys, time

import numpy as np
import yaml
import zmq

import scensim_world_server.protocol as P

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene as scenemod  # noqa: E402

port, world_yaml, map_dir, HOLD, OUT = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), sys.argv[5]
wy = yaml.safe_load(open(world_yaml))
SCENE = scenemod.load(*sys.argv[6:7])

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 900_000)
sock.connect(f"tcp://127.0.0.1:{port}")
start_time = int(open("/proc/self/stat").read().rsplit(")", 1)[1].split()[19])
CLIENT = P.ClientId(host=socket.gethostname(), pid=os.getpid(), start_time=start_time)


def call(**kw):
    req = P.Request(client=CLIENT, **kw)
    sock.send(req.SerializeToString())
    rep = P.Reply.FromString(sock.recv())
    if not rep.ok:
        raise RuntimeError(f"{list(kw)[0]}: {rep.error}")
    return rep


def quat_yaw(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))  # w x y z


def pose(x, y, z, yaw):
    return P.make_pose((x, y, z), quat_yaw(yaw))


def rot(q):  # w x y z -> R
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


h = call(hello=P.Hello(protocol_version=P.PROTOCOL_VERSION)).hello
print("hello", h.simulator_name, h.simulator_version, "ready", h.ready, "max_dt", h.max_physics_dt, flush=True)
dt = h.max_physics_dt or 1 / 60
STEP = 0.05
sub = max(1, round(STEP / dt))
t = time.time()
o = wy.get("map_origin", [0, 0, 0])
if any(o):
    # the scene's poses are map-frame and go to the world server as they are; the bridge would
    # subtract map_origin first, which this script does not
    raise SystemExit(f"{world_yaml}: map_origin {o} is not zero; isaac_scene.py places map-frame "
                     f"poses as they are and supports only a zero map_origin")
lw = call(load_world=P.LoadWorld(world=wy["usd"], ground_paths=wy.get("ground_paths", []),
                                 map_dir=map_dir, lanelet2_map_path=f"{map_dir}/lanelet2_map.osm",
                                 map_origin=P.Vec3(x=o[0], y=o[1], z=o[2]), step_time=STEP,
                                 realtime_factor=1.0)).load_world
print(f"loaded world in {time.time() - t:.1f} s, episode {lw.episode_id}", flush=True)

BOX_C, BOX_D = tuple(SCENE["box_centre_m"]), tuple(SCENE["box_dims_m"])
EGO_X, EGO_Y, EGO_YAW = SCENE["ego_map"]


def spawn(name, kind, motion, p, role=""):
    sp = P.Spawn(name=name, kind=P.KIND[kind], subtype="car",
                 box_center=P.Vec3(x=BOX_C[0], y=BOX_C[1], z=BOX_C[2]),
                 box_dimensions=P.Vec3(x=BOX_D[0], y=BOX_D[1], z=BOX_D[2]), pose=p,
                 model="vehicle.tesla.model3", motion=P.MOTION[motion], role_name=role,
                 asset_key="tesla_model3", axles=P.Axles(front=3.005, rear=0.0),
                 performance=P.Performance(max_acceleration=3.0, max_deceleration=8.0),
                 box_declared=True)
    r = call(spawn=sp).spawn
    print("spawned", name, r.handle, r.body.model, "offset", r.body.origin_offset, flush=True)
    return r


t = time.time()
ego = spawn("ego", "EGO", "EXTERNAL", pose(EGO_X, EGO_Y, 0.3, EGO_YAW), role="ego")
print(f"ego spawn {time.time() - t:.1f} s", flush=True)


def step(npcs):
    ap = [P.EntityPose(handle=ego.handle)] + [P.EntityPose(handle=hd, pose=pp) for hd, pp in npcs.values()]
    call(apply=P.Apply(poses=ap, npc_logic_started=True))
    return call(step=P.Step(substeps=sub, dt=STEP / sub, render=True, want_states=True)).step


# settle the ego (hold brake) to read its ground height
for _ in range(40):
    s = step({})
es = {e.handle: e for e in s.states}[ego.handle]
ez = es.pose.position.z
print("ego state", es.pose.position, flush=True)
gz = ez - ego.body.origin_offset.z  # entity origin height (on the ground)

npcs, offsets = {}, {}
for name, (x, y, yaw) in SCENE["cars_map"].items():
    p = pose(x, y, gz, yaw)
    r = spawn(name, "VEHICLE", "KINEMATIC", p)
    npcs[name] = (r.handle, p)
    o = r.body.origin_offset
    offsets[name] = np.array([o.x, o.y, o.z])

for _ in range(20):
    s = step(npcs)
st = {e.handle: e for e in s.states}
# the ego's entity origin (base_link) from its body pose and the spawn's origin offset
ep = st[ego.handle].pose
eq = (ep.orientation.w, ep.orientation.x, ep.orientation.y, ep.orientation.z)
Re = rot(eq)
off = ego.body.origin_offset
e_org = np.array([ep.position.x, ep.position.y, ep.position.z]) - Re @ np.array([off.x, off.y, off.z])
truth, full = {}, {"ego_body_pose": [ep.position.x, ep.position.y, ep.position.z, *eq],
                   "ego_origin_offset": [off.x, off.y, off.z], "base_link_in_world": e_org.tolist(),
                   "cars": {}}
for name, (hd, p) in npcs.items():
    # measured: the car's state after the steps (its body pose less its origin offset, as the
    # ego's), not the pose commanded, so a world server that moves a body shows here
    mp = st[hd].pose
    mq = (mp.orientation.w, mp.orientation.x, mp.orientation.y, mp.orientation.z)
    m_org = np.array([mp.position.x, mp.position.y, mp.position.z]) - rot(mq) @ offsets[name]
    b = Re.T @ (m_org + rot(mq) @ np.array(BOX_C) - e_org)
    q = (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
    cmd = np.array([p.position.x, p.position.y, p.position.z])
    bc = Re.T @ (cmd + rot(q) @ np.array(BOX_C) - e_org)
    truth[name] = [round(float(b[0]), 2), round(float(b[1]), 2)]
    full["cars"][name] = {"commanded_origin": cmd.round(3).tolist(),
                          "measured_origin": m_org.round(3).tolist(),
                          "commanded_box_centre_base_link": [round(float(v), 3) for v in bc],
                          "measured_box_centre_base_link": [round(float(v), 3) for v in b]}
json.dump(truth, open(OUT, "w"), indent=1)
json.dump(full, open(OUT.replace(".json", "_full.json"), "w"), indent=1)
print("truth", truth, flush=True)
err = max((math.hypot(truth[k][0] - c["centre"][0], truth[k][1] - c["centre"][1])
           for k, c in SCENE["cars"].items()), default=0.0)
if err > 0.05:
    raise SystemExit(f"cars placed up to {err:.2f} m from the scene's centres")
print("SCENE-READY", flush=True)

t0 = time.time(); n = 0
stop = OUT + ".stop"
while time.time() - t0 < HOLD and not os.path.exists(stop):
    ts = time.time()
    s = step(npcs); n += 1
    time.sleep(max(0.0, STEP - (time.time() - ts)))
print(f"stepped {n} frames in {time.time() - t0:.1f} s, sim_time {s.sim_time:.2f}", flush=True)
for hd, _ in npcs.values():
    call(despawn=P.Despawn(handle=hd))
call(despawn=P.Despawn(handle=ego.handle))
call(release=P.Release())
print("SCENE-DONE", flush=True)
