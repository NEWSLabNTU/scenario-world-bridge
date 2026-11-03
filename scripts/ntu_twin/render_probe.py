"""Roadmap 059, rendering: the NTU twin's camera picture from fixed viewpoints, to compare a
world's texture and light before and after a change. Run by Isaac's Python:

    $ISAAC_ROOT/venv/bin/python scripts/ntu_twin/render_probe.py WORLD_YAML OUT_DIR \\
        --view name,x,y,yaw_deg[,height,back,pitch_deg] ... [--default-light] [--exposure]

A view is a chase camera: `back` m behind and `height` m above the ground at world (x, y)
(map less map_origin; the ground found by a ray), looking along `yaw` (deg, counter-clockwise
from +x) and `pitch` deg down (defaults 3.4, 7.0, 15: the video rig's chase camera). 1280 x 720,
80 deg across, as that camera. Writes OUT_DIR/<name>.png.

`--default-light`: the world's own lights off, and the sun and sky aib's runner and
swb_isaac_world add to a world with none (the picture before the world had its light).
`--exposure`: the camera's auto-exposure (OmniRtxCameraAutoExposureAPI_1), as aib's rig `auto_exposure`.
"""
import argparse
import math
import os
import sys

import yaml

ap = argparse.ArgumentParser()
ap.add_argument("world_yaml")
ap.add_argument("out_dir")
ap.add_argument("--view", action="append", required=True)
ap.add_argument("--default-light", action="store_true")
ap.add_argument("--exposure", action="store_true")
ap.add_argument("--warmup", type=int, default=60, help="frames rendered before each capture")
ap.add_argument("--reach", type=float, default=200.0, help="m: only the tiles this near a view are loaded")
a = ap.parse_args()
os.makedirs(a.out_dir, exist_ok=True)
os.environ.pop("DISPLAY", None)

from isaacsim import SimulationApp  # noqa: E402

extra = ["--/crashreporter/url="]
app = SimulationApp({"headless": True, "extra_args": extra})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage, is_stage_loading  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402

cfg = yaml.safe_load(open(a.world_yaml))
usd = cfg["usd"] if os.path.isabs(cfg["usd"]) else os.path.join(os.path.dirname(a.world_yaml), cfg["usd"])
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
add_reference_to_stage(usd_path=usd, prim_path="/World/env")
# Only the tiles within --reach of a view: the whole world takes ~16 GB of a shared GPU.
REACH = a.reach
origin = cfg.get("map_origin", [0.0, 0.0, 0.0])
views = [tuple(float(c) for c in v.split(",")[1:3]) for v in a.view]
tiles = stage.GetPrimAtPath("/World/env/Tiles")
kept = 0
for t in (tiles.GetChildren() if tiles else []):
    ix, iy = (int(s.replace("m", "-")) for s in t.GetName().split("_")[1:3])
    cx, cy = ix * 50 + 25 - origin[0], iy * 50 + 25 - origin[1]
    near = any(math.hypot(cx - x, cy - y) < REACH for x, y in views)
    t.SetActive(near)
    kept += near
print(f"tiles: {kept} within {REACH:.0f} m of a view", flush=True)
for _ in range(10):
    app.update()
while is_stage_loading():
    app.update()

own = [p for p in Usd.PrimRange(stage.GetPrimAtPath("/World/env")) if p.HasAPI(UsdLux.LightAPI)]
if a.default_light or not own:
    for p in own:
        p.SetActive(False)
    # aib runner.ensure_light / swb_isaac_world's SUN and SKY
    sun = UsdLux.DistantLight.Define(stage, "/World/sun")
    sun.CreateIntensityAttr(2500.0)
    sun.CreateAngleAttr(1.0)
    UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate(Gf.Vec3f(-50.0, 20.0, 30.0))
    UsdLux.DomeLight.Define(stage, "/World/sky").CreateIntensityAttr(600.0)
print(f"lights: {'default' if a.default_light or not own else [str(p.GetPath()) for p in own]}, "
      f"exposure {'auto' if a.exposure else 'fixed'}", flush=True)
world.reset()


def ground_z(x, y):
    from omni.physx import get_physx_scene_query_interface

    hit = get_physx_scene_query_interface().raycast_closest(Gf.Vec3d(x, y, 200.0), Gf.Vec3d(0, 0, -1), 400.0)
    return hit["position"][2] if hit["hit"] else 0.0


W, H, HFOV, APERTURE = 1280, 720, 80.0, 20.955
cam = UsdGeom.Camera.Define(stage, "/World/probe_cam")
cam.CreateHorizontalApertureAttr(APERTURE)
cam.CreateVerticalApertureAttr(APERTURE * H / W)
cam.CreateFocalLengthAttr(APERTURE / (2 * math.tan(math.radians(HFOV) / 2)))
cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 1000.0))
xop = UsdGeom.Xformable(cam.GetPrim()).AddTransformOp()
if a.exposure:
    # Isaac 6.1: auto-exposure is the camera's (OmniRtxCameraAutoExposureAPI_1), as aib's
    # Camera sets it with the rig's `auto_exposure`
    from pxr import Sdf

    p = cam.GetPrim()
    p.AddAppliedSchema("OmniRtxCameraAutoExposureAPI_1")
    p.CreateAttribute("omni:rtx:autoExposure:enabled", Sdf.ValueTypeNames.Bool).Set(True)
rp = rep.create.render_product(str(cam.GetPath()), resolution=(W, H))
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp.path])

for v in a.view:
    f = v.split(",")
    name, x, y, yaw = f[0], float(f[1]), float(f[2]), math.radians(float(f[3]))
    height, back, pitch = (float(f[4]), float(f[5]), math.radians(float(f[6]))) if len(f) > 4 else (3.4, 7.0, math.radians(15))
    z = ground_z(x, y)
    fwd = Gf.Vec3d(math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch))
    pos = Gf.Vec3d(x - back * math.cos(yaw), y - back * math.sin(yaw), z + height)
    right = Gf.Cross(fwd, Gf.Vec3d(0, 0, 1)).GetNormalized()
    up = Gf.Cross(right, fwd)
    # a USD camera looks down its -z with +y up: rows x = right, y = up, z = -forward
    xop.Set(Gf.Matrix4d(right[0], right[1], right[2], 0, up[0], up[1], up[2], 0,
                        -fwd[0], -fwd[1], -fwd[2], 0, pos[0], pos[1], pos[2], 1))
    for _ in range(a.warmup):
        world.render()
    d = np.asarray(rgb.get_data())
    Image.fromarray(np.ascontiguousarray(d[..., :3]).astype(np.uint8)).save(os.path.join(a.out_dir, f"{name}.png"))
    print(f"view {name}: ground {z:.2f}, mean {d[..., :3].mean():.0f}", flush=True)

app.close()
sys.exit(0)
