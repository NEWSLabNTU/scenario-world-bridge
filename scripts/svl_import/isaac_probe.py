#!/usr/bin/env python3
"""Render an imported world in Isaac Sim headless, a quick look before the world server.

    $ISAAC_ROOT/venv/bin/python scripts/svl_import/isaac_probe.py WORLD_USD OUT_DIR \\
        [--view x,y,z,tx,ty,tz ...] [--size 1280x720] [--frames 30]

Opens the stage, references WORLD_USD under /World/env, adds a sun and a sky (as the world
server does for a world without lights), and saves one RGB PNG per `--view` (camera at
x,y,z looking at tx,ty,tz, world frame) to OUT_DIR, with the load time and peak VRAM in
`probe.json`. Default views: a high view of the whole map and one at a driver's height.
"""
import argparse
import json
import os
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd")
    ap.add_argument("out")
    ap.add_argument("--view", action="append", default=[])
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--frames", type=int, default=30)
    a = ap.parse_args()
    w, h = (int(v) for v in a.size.split("x"))
    views = a.view or ["-120,-120,90,0,0,0", "0,-60,1.6,0,40,1.2"]
    t0 = time.time()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": w, "height": h})
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    from PIL import Image
    from pxr import Gf, Sdf, UsdGeom, UsdLux

    t_boot = time.time() - t0
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    env = stage.DefinePrim("/World/env", "Xform")
    env.GetReferences().AddReference(os.path.realpath(a.usd))
    sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
    sun.CreateIntensityAttr(3000)
    sun.CreateAngleAttr(0.5)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(45, 0, 30))
    sky = UsdLux.DomeLight.Define(stage, "/World/Sky")
    sky.CreateIntensityAttr(1000)
    t1 = time.time()
    for _ in range(10):
        app.update()
    t_load = time.time() - t1
    os.makedirs(a.out, exist_ok=True)
    shots = []
    for i, v in enumerate(views):
        x, y, z, tx, ty, tz = (float(s) for s in v.split(","))
        cam = rep.create.camera(position=(x, y, z), look_at=(tx, ty, tz), focal_length=18.0)
        rp = rep.create.render_product(cam, (w, h))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(rp)
        t2 = time.time()
        for _ in range(a.frames):
            app.update()
        for _ in range(3):
            rep.orchestrator.step(rt_subframes=8)
        data = rgb.get_data()
        img = np.asarray(data)[..., :3]
        path = os.path.join(a.out, f"view{i}.png")
        Image.fromarray(img.astype(np.uint8)).save(path)
        shots.append({"view": v, "file": path, "seconds": round(time.time() - t2, 1),
                      "mean": float(img.mean())})
        rgb.detach()
    res = {"boot_s": round(t_boot, 1), "load_s": round(t_load, 1), "shots": shots}
    try:
        import subprocess
        res["nvidia_smi"] = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader"],
            capture_output=True, text=True).stdout
    except Exception:
        pass
    json.dump(res, open(os.path.join(a.out, "probe.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
