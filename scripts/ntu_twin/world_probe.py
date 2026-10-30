"""Roadmap 059 N6: open the NTU twin in Isaac Sim and check what the RTX camera and lidar see.

    ISAAC_ROOT=... src/swb_isaac_world/spikes/run_spike.sh fastrtps <log> \\
        $PWD/scripts/ntu_twin/world_probe.py <map/isaac/world.yaml> <out dir> [x,y,z,yaw_deg]
    # prints one PROBE-RESULT json; writes cam_*.jpg and lidar.npy into <out dir>

The world is referenced at /World/env, as swb_isaac_world does. The pose is in the Isaac world
frame (map less map_origin), the ground height under the car; the camera sits 1.5 m above it
and the lidar 1.9 m (aib's OS1). Measured: stage open and payload load, the first frames (shader
compiles and texture streaming), the frame time after, the process's VRAM, and the lidar's
returns split into ground (within 0.3 m of the pose's ground) and above.
"""
import json
import math
import os
import subprocess
import sys
import time

import yaml

os.environ.pop("DISPLAY", None)
world_yaml, out_dir = sys.argv[1], sys.argv[2]
pose = [float(v) for v in (sys.argv[3] if len(sys.argv) > 3 else "-282.58,-45.02,11.8,0.2").split(",")]
os.makedirs(out_dir, exist_ok=True)
cfg = yaml.safe_load(open(world_yaml))

t_boot = time.perf_counter()
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})
import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

out = {"boot_s": round(time.perf_counter() - t_boot, 1), "pose": pose}


def vram_mib():
    q = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory",
                        "--format=csv,noheader,nounits"], capture_output=True, text=True).stdout
    for line in q.splitlines():
        pid, mem = (v.strip() for v in line.split(","))
        if int(pid) == os.getpid():
            return int(mem)
    total = subprocess.run(["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True).stdout.strip()
    return f"gpu total {total}"


world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 20)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(3000.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 30.0))
UsdLux.DomeLight.Define(stage, "/World/sky").CreateIntensityAttr(800.0)
out["vram_empty_mib"] = vram_mib()

t0 = time.perf_counter()
env = stage.DefinePrim("/World/env", "Xform")
env.GetReferences().AddReference(cfg["usd"])
while is_stage_loading():
    app.update()
out["reference_s"] = round(time.perf_counter() - t0, 1)
tris = meshes = 0
for p in Usd.PrimRange(env):
    if p.IsA(UsdGeom.Mesh):
        meshes += 1
        c = UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()
        tris += 0 if c is None else int(sum(n - 2 for n in c))
out["meshes"], out["triangles"] = meshes, tris
out["count_s"] = round(time.perf_counter() - t0, 1)

x, y, gz, yaw = pose
cams = {
    # name: (position, rotateXYZ); a USD camera looks down -Z, so (90, 0, yaw-90) looks along yaw.
    "dash": ((x, y, gz + 1.5), (90.0, 0.0, yaw - 90.0)),
    "high": ((x - 30 * math.cos(math.radians(yaw)), y - 30 * math.sin(math.radians(yaw)), gz + 25.0),
             (55.0, 0.0, yaw - 90.0)),
    "down": ((x + 15.0, y, gz + 60.0), (0.0, 0.0, yaw - 90.0)),
}
annots = {}
for name, (pos, rot) in cams.items():
    cam = UsdGeom.Camera.Define(stage, f"/World/cam_{name}")
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 2000.0))
    xf = UsdGeom.Xformable(cam)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*rot))
    rp = rep.create.render_product(cam.GetPath().pathString, resolution=(1280, 720))
    a = rep.AnnotatorRegistry.get_annotator("rgb")
    a.attach([rp.path])
    annots[name] = a

from isaacsim.sensors.experimental.rtx import Lidar as RtxLidar  # noqa: E402

mount = UsdGeom.Xform.Define(stage, "/World/mount")
mount.AddTranslateOp().Set(Gf.Vec3d(x, y, gz + 1.9))
mount.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw))
lidar = RtxLidar.create("/World/mount/lidar", config="OS1", variant="OS1_REV7_128ch20hz1024res")
lidar_rp = rep.create.render_product(lidar.paths[0], resolution=(128, 128))
points = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
points.attach([lidar_rp.path])

world.reset()
world.play()
first = []
for _ in range(30):
    t1 = time.perf_counter()
    world.step(render=True)
    first.append((time.perf_counter() - t1) * 1e3)
out["first_frames_ms"] = [round(v) for v in first[:10]]
out["first_30_frames_s"] = round(sum(first) / 1e3, 1)
for _ in range(60):  # let textures stream in
    world.step(render=True)
steady = []
scans = []
for _ in range(40):
    t1 = time.perf_counter()
    world.step(render=True)
    steady.append((time.perf_counter() - t1) * 1e3)
    d = points.get_data()
    p = np.asarray(d.get("data", []) if isinstance(d, dict) else [], dtype=np.float64)
    if p.ndim == 2 and len(p):
        scans.append(p[:, :3].copy())
out["frame_ms_median"] = round(float(np.median(steady)), 1)
out["frame_ms_p95"] = round(float(np.percentile(steady, 95)), 1)
out["vram_loaded_mib"] = vram_mib()

for name, a in annots.items():
    img = np.asarray(a.get_data())
    if img.ndim == 3 and img.size:
        rgb = img[..., :3].astype(np.uint8)
        Image.fromarray(rgb).save(os.path.join(out_dir, f"cam_{name}.jpg"), quality=90)
        out[f"cam_{name}_mean_rgb"] = [round(float(v), 1) for v in rgb.reshape(-1, 3).mean(0)]
    else:
        out[f"cam_{name}_mean_rgb"] = None

if scans:
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    pts = np.concatenate(scans)
    w = np.empty_like(pts)  # sensor frame -> world
    w[:, 0] = x + c * pts[:, 0] - s * pts[:, 1]
    w[:, 1] = y + s * pts[:, 0] + c * pts[:, 1]
    w[:, 2] = gz + 1.9 + pts[:, 2]
    np.save(os.path.join(out_dir, "lidar.npy"), w.astype(np.float32))
    r = np.linalg.norm(pts, axis=1)
    near = np.hypot(w[:, 0] - x, w[:, 1] - y) < 20.0
    out["lidar"] = {
        "points_per_frame": int(len(pts) / len(scans)),
        "frames_with_points": len(scans),
        "range_m_p5_p50_p95": [round(float(v), 1) for v in np.percentile(r, [5, 50, 95])],
        "ground_within_0.3m_of_pose_in_20m": round(float(
            (np.abs(w[near, 2] - gz) < 0.3).mean()), 3) if near.any() else None,
        "above_ground_0.3m_share": round(float((w[:, 2] - gz > 0.3).mean()), 3),
    }
else:
    out["lidar"] = None
print("PROBE-RESULT " + json.dumps(out), flush=True)
os._exit(0)
