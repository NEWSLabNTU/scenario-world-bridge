#!/usr/bin/env python3
"""Roadmap 062, O5: the Isaac world from world.py's meshes; run by Isaac's Python (pxr).

    $ISAAC_ROOT/venv/bin/python scripts/open_maps/world_usd.py <site.yaml> <out>

One stage in the Isaac world frame (the map frame less the site's map_origin), metres, z up,
built with 058's helpers (swb_isaac_world.course.usd) so it loads as a course does:

- the ground, textured with NLSC's photo (Terrain);
- the roads (Road), colliders;
- the paint (Lines);
- the buildings, colliders, both sides drawn;
- traffic-light poles and street lamps, simple shapes with colliders;
- trees, CARLA's, through the `carla-usd` link beside the world.

It writes the trees' surface points (`world/props_points.npy`) for world.py's pcd step, and
`isaac/world.yaml` with the map origin and the ground paths.
"""
import json
import math
import os
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src" / "swb_isaac_world"))
from swb_isaac_world.course import props as pr  # noqa: E402
from swb_isaac_world.course.usd import LABELS, LINK, QCODE, _grid_normals, _material, _mesh  # noqa: E402

ROOT = "/World"
TREES = ["Town01/Props/SM_Beech.usd", "Town02/Props/SM_Japanese_Maple_01.usd",
         "Town01/Props/SM_TreePine_1.usd"]
BUILDING_COLOUR = (0.62, 0.6, 0.57)
POLE_COLOUR = (0.35, 0.36, 0.38)


def tri_mesh(stage, path, pts, tris, material, label=None, collider=True):
    from pxr import Sdf, UsdGeom, UsdPhysics, UsdShade, Vt
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tris)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(tris.astype(np.int32).ravel()))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    lo, hi = pts.min(0), pts.max(0)
    mesh.CreateExtentAttr(Vt.Vec3fArray([tuple(lo.tolist()), tuple(hi.tolist())]))
    prim = mesh.GetPrim()
    if collider:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    if label:
        prim.CreateAttribute(QCODE, Sdf.ValueTypeNames.TokenArray).Set([label])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def pole(stage, path, x, y, height, radius, material, head=None):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    c = UsdGeom.Cylinder.Define(stage, path + "/pole")
    c.CreateHeightAttr(height)
    c.CreateRadiusAttr(radius)
    c.CreateAxisAttr("Z")
    c.AddTranslateOp().Set(Gf.Vec3d(0, 0, height / 2))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material)
    if head is not None:        # a signal head: 0.35 x 0.35 x 1.1 m box on top, facing yaw
        b = UsdGeom.Cube.Define(stage, path + "/head")
        b.CreateSizeAttr(1.0)
        b.AddTranslateOp().Set(Gf.Vec3d(0, 0, height + 0.55))
        b.AddRotateZOp().Set(math.degrees(head))
        b.AddScaleOp().Set(Gf.Vec3f(0.35, 0.35, 1.1))
        UsdShade.MaterialBindingAPI.Apply(b.GetPrim()).Bind(material)


def main():
    from pxr import Gf, Sdf, Usd, UsdGeom, Vt
    site = yaml.safe_load(open(sys.argv[1]))
    out = Path(sys.argv[2])
    work = out / "map" / "world"
    idir = out / "map" / "isaac"
    idir.mkdir(parents=True, exist_ok=True)
    m = np.load(work / "meshes.npz")
    props = json.loads((work / "props.json").read_text())
    shutil.copyfile(work / "photo.png", idir / "ground_photo.png")
    path = idir / "world.usdc"
    if path.exists():
        path.unlink()
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(stage, ROOT + "/Looks")
    ground = _material(stage, ROOT + "/Looks/GroundPhoto", texture="./ground_photo.png", roughness=0.95)
    asphalt = _material(stage, ROOT + "/Looks/Asphalt", colour=(0.16, 0.16, 0.17), roughness=0.8)
    white = _material(stage, ROOT + "/Looks/White", colour=(0.9, 0.9, 0.9), roughness=0.6)
    yellow = _material(stage, ROOT + "/Looks/Yellow", colour=(0.85, 0.66, 0.08), roughness=0.6)
    wall = _material(stage, ROOT + "/Looks/Building", colour=BUILDING_COLOUR, roughness=0.9)
    metal = _material(stage, ROOT + "/Looks/Pole", colour=POLE_COLOUR, roughness=0.5)
    shape = tuple(int(v) for v in m["terrain_shape"])
    _mesh(stage, ROOT + "/Terrain", m["terrain_points"], m["terrain_faces"], uv=m["terrain_uv"],
          normals=_grid_normals(m["terrain_points"], shape), collider=True, label=LABELS["Terrain"],
          material=ground)
    _mesh(stage, ROOT + "/Road", m["road_points"], m["road_faces"], collider=True,
          label=LABELS["Road"], material=asphalt)
    UsdGeom.Xform.Define(stage, ROOT + "/Lines")
    stage.GetPrimAtPath(ROOT + "/Lines").CreateAttribute(
        QCODE, Sdf.ValueTypeNames.TokenArray).Set([LABELS["Lines"]])
    if len(m["white_faces"]):
        _mesh(stage, ROOT + "/Lines/white", m["white_points"], m["white_faces"], material=white)
    if len(m["yellow_faces"]):
        _mesh(stage, ROOT + "/Lines/yellow", m["yellow_points"], m["yellow_faces"], material=yellow)
    if len(m["building_tris"]):
        tri_mesh(stage, ROOT + "/Buildings", m["building_points"], m["building_tris"], wall,
                 label="Q41176")
    UsdGeom.Xform.Define(stage, ROOT + "/Props")
    UsdGeom.Xform.Define(stage, ROOT + "/Props/Signals")
    for i, (x, y, yaw) in enumerate(props["signals"]):
        pole(stage, f"{ROOT}/Props/Signals/s{i}", x, y, 5.5, 0.12, metal, head=yaw)
    UsdGeom.Xform.Define(stage, ROOT + "/Props/Lamps")
    for i, (x, y) in enumerate(props["lamps"]):
        pole(stage, f"{ROOT}/Props/Lamps/l{i}", x, y, 8.0, 0.1, metal)
    # trees: CARLA's, through a link beside the world (issue 020)
    carla = os.path.realpath(os.environ.get("CARLA_USD", "/mnt/disk1/carla-usd"))
    link = idir / LINK
    if link.is_symlink() or link.exists():
        link.unlink()
    os.symlink(carla, link)
    kinds = [t for t in TREES if (link / t).exists()]
    samples = []
    if kinds and props["trees"]:
        rng = np.random.default_rng(62)
        inst = UsdGeom.PointInstancer.Define(stage, ROOT + "/Props/Trees")
        protos = []
        for k, t in enumerate(kinds):
            p = stage.DefinePrim(f"{ROOT}/Props/Trees/kind{k}", "Xform")
            p.GetReferences().AddReference(f"./{LINK}/{t}")
            protos.append(p.GetPath())
        inst.CreatePrototypesRel().SetTargets(protos)
        T = np.array(props["trees"], float)
        kind = rng.integers(0, len(kinds), len(T))
        yaw = rng.uniform(0, 2 * math.pi, len(T))
        scale = rng.uniform(0.8, 1.2, len(T))
        inst.CreateProtoIndicesAttr(Vt.IntArray([int(v) for v in kind]))
        inst.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(float(x), float(y), 0.0) for x, y in T]))
        inst.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(math.cos(a / 2), 0, 0, math.sin(a / 2)) for a in yaw]))
        inst.CreateScalesAttr(Vt.Vec3fArray([Gf.Vec3f(float(s), float(s), float(s)) for s in scale]))
        surf = [pr.sample_surface(pr.asset_surface(str(link / t)), pr.SAMPLE_DENSITY / 4, rng) for t in kinds]
        for (x, y), k, a, s in zip(T, kind, yaw, scale):
            samples.append(pr.place(surf[int(k)], x, y, 0.0, a, s))
    pts = np.concatenate(samples) if samples else np.zeros((0, 3))
    np.save(work / "props_points.npy", pts.astype(np.float32))
    stage.GetRootLayer().Save()
    ox, oy, oz = site["frame"]["map_origin"]
    (idir / "world.yaml").write_text(
        "# Generated by scripts/open_maps/world_usd.py (roadmap 062): the roads around NTU from public\n"
        "# data; the world frame is the map frame (MGRS 51RUH local) less map_origin.\n"
        f"usd: {path.resolve()}\n"
        f"map_origin: [{float(ox)}, {float(oy)}, {float(oz)}]\n"
        "ground_paths:\n"
        '  - "/World/env/Terrain"\n'
        '  - "/World/env/Road"\n'
        '  - "/World/env/Lines"\n')
    print(json.dumps({"usd": str(path), "tree_kinds": kinds, "trees": len(props["trees"]),
                      "tree_points": len(pts), "signals": len(props["signals"]), "lamps": len(props["lamps"])}))


if __name__ == "__main__":
    main()
