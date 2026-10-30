#!/usr/bin/env python3
"""An Isaac world from an SVL environment dump (unity_dump.py), in the map's frame.

Run by Isaac's Python (it has `pxr`):

    $ISAAC_ROOT/venv/bin/python scripts/svl_import/svl_to_usd.py DUMP_DIR MAP_DIR

Writes `MAP_DIR/isaac/`: `world.usdc` (one mesh per Unity mesh, referenced by every
GameObject that renders it, under /SVL/Scene), `textures/`, and `world.yaml`.

Frame. SVL places a map with its `MapOrigin` GameObject: a point's UTM coordinates are
its position in MapOrigin's local frame plus (OriginEasting, OriginNorthing), local x east,
local z north, y up. Shalun's MapOrigin turns -90 degrees about y, so the map's local east,
north and up are Unity (z, -x, y). That is a reflection of Unity's left-handed frame, so
every triangle's winding is reversed. The stage is in that local ENU frame (metres, z up);
the Lanelet2 map's `x`/`y` tags are in the same frame (svl_to_autoware.py). world.yaml's
`map_origin` is the origin's MGRS-local position, so Autoware's map frame (MGRS) less
map_origin is this stage's frame.

Labels: SimReady Q-codes on each mesh by its first material's name: asphalt Road, lane
paint, arrows and crosswalks RoadLine, sidewalks and curbs SideWalk, grass Terrain,
buildings Building. Colliders where Unity had a MeshCollider. Unity's 98 lights are not
carried over: the world server's sun and sky light a world that brings none of its own.
"""
import argparse
import json
import math
import os
import re
import shutil
import sys

import numpy as np

QCODE = "semantics:labels:wikidata_qcode"
LABELS = [  # (material name pattern, Q-code), the first match wins
    (r"Asphalt", "Q34442"),             # Road
    (r"White|Yellow|Arrow|CrossWalk|Crosswalk", "Q1392287"),  # RoadLine
    (r"SideWalk|Sidewalk|Curb", "Q177749"),  # SideWalk
    (r"Grass", "Q186131"),              # Terrain
    (r"Building", "Q41176"),            # Building
]
ROOT = "/SVL"


def unity_to_map(origin_yaw_deg):
    """The 4x4 that takes Unity world coordinates to the map's local ENU (x east, y north,
    z up): into MapOrigin's local frame (its rotation about y undone), then (x, z, y)."""
    a = math.radians(origin_yaw_deg)
    # Unity's rotation about y by a: x' = x cos a + z sin a, z' = -x sin a + z cos a
    ry = np.array([[math.cos(a), 0, math.sin(a), 0], [0, 1, 0, 0],
                   [-math.sin(a), 0, math.cos(a), 0], [0, 0, 0, 1]])
    swap = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)
    return swap @ np.linalg.inv(ry)


def mgrs_local(easting, northing):
    """The MGRS 100 km square's local coordinates of a UTM position."""
    return easting % 100_000, northing % 100_000


def label_of(material_names):
    for name in material_names:
        for pat, q in LABELS:
            if re.search(pat, name):
                return q
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dump")
    ap.add_argument("map_dir")
    ap.add_argument("--origin-yaw", type=float, default=None,
                    help="MapOrigin's rotation about Unity's y, degrees (default: from scene.json)")
    a = ap.parse_args(argv)
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    scene = json.load(open(os.path.join(a.dump, "scene.json")))
    mats = {m["name"]: m for m in json.load(open(os.path.join(a.dump, "materials.json")))}
    man = scene["manifest"]
    yaw = a.origin_yaw if a.origin_yaw is not None else scene.get("map_origin_yaw_deg", -90.0)
    T = unity_to_map(yaw)

    out = os.path.join(a.map_dir, "isaac")
    os.makedirs(os.path.join(out, "textures"), exist_ok=True)
    stage = Usd.Stage.CreateNew(os.path.join(out, "world.usdc"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(stage, ROOT + "/Looks")
    UsdGeom.Scope.Define(stage, ROOT + "/Meshes")
    UsdGeom.Xform.Define(stage, ROOT + "/Scene")

    def tex(name):
        if not name:
            return None
        src = os.path.join(a.dump, "textures", name)
        if not os.path.exists(src):
            return None
        shutil.copyfile(src, os.path.join(out, "textures", name))
        return "./textures/" + name

    looks = {}

    def look(name):
        if name in looks:
            return looks[name]
        m = mats.get(name, {"base_color": [0.8, 0.8, 0.8, 1]})
        path = f"{ROOT}/Looks/{re.sub(r'[^A-Za-z0-9_]', '_', name)}"
        mat = UsdShade.Material.Define(stage, path)
        pbr = UsdShade.Shader.Define(stage, path + "/PBR")
        pbr.CreateIdAttr("UsdPreviewSurface")
        pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
            float(1.0 - m.get("smoothness", 0.2)))
        pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(m.get("metallic", 0.0)))
        c = m.get("base_color", [1, 1, 1, 1])
        base = tex(m.get("base_map"))
        if base:
            st = UsdShade.Shader.Define(stage, path + "/st")
            st.CreateIdAttr("UsdPrimvarReader_float2")
            st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            xf = UsdShade.Shader.Define(stage, path + "/tiling")
            xf.CreateIdAttr("UsdTransform2d")
            xf.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
            xf.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*m.get("tiling", [1, 1])))
            xf.CreateInput("translation", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*m.get("offset", [0, 0])))
            xf.CreateOutput("result", Sdf.ValueTypeNames.Float2)

            def texture(prim, file, cs, out_name, out_type):
                t = UsdShade.Shader.Define(stage, path + "/" + prim)
                t.CreateIdAttr("UsdUVTexture")
                t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(file)
                t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(xf.ConnectableAPI(), "result")
                t.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
                t.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
                t.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set(cs)
                t.CreateOutput(out_name, out_type)
                return t

            t = texture("base", base, "sRGB", "rgb", Sdf.ValueTypeNames.Float3)
            t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(c[0], c[1], c[2], 1))
            pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(t.ConnectableAPI(), "rgb")
            nrm = tex(m.get("normal_map"))
            if nrm:
                n = texture("normal", nrm, "raw", "rgb", Sdf.ValueTypeNames.Float3)
                n.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2, 2, 2, 1))
                n.CreateInput("bias", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1, -1, -1, 0))
                pbr.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(n.ConnectableAPI(), "rgb")
        else:
            pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(c[0], c[1], c[2]))
        mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
        looks[name] = mat
        return mat

    # Each instance is baked into the map frame (Unity's mirror cannot be a USD transform
    # without negative scale, which PhysX's triangle colliders reject).
    n_tris = 0
    used = {}
    for k, inst in enumerate(scene["instances"]):
        npz = np.load(os.path.join(a.dump, "meshes", f"{inst['mesh']}.npz"))
        W = T @ np.array(inst["world"])
        P = npz["positions"].astype(np.float64)
        V = (W[:3, :3] @ P.T).T + W[:3, 3]
        N = None
        if "normals" in npz:
            Nm = np.linalg.inv(W[:3, :3]).T
            N = (Nm @ npz["normals"].astype(np.float64).T).T
            N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-9)
        subs = sorted((key for key in npz.files if key.startswith("sub")), key=lambda s: int(s[3:]))
        F_all, counts = [], []
        for s in subs:
            F = npz[s][:, ::-1]  # the mirror reverses winding
            F_all.append(F)
            counts.append(len(F))
        if not counts or sum(counts) == 0:
            continue
        F = np.concatenate(F_all)
        name = re.sub(r"[^A-Za-z0-9_]", "_", inst["path"].split("/")[-1]) or "mesh"
        used[name] = used.get(name, 0) + 1
        path = f"{ROOT}/Scene/{name}_{used[name]}"
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V.astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.astype(np.int32).ravel()))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr(Vt.Vec3fArray([tuple(V.min(0)), tuple(V.max(0))]))
        if N is not None:
            mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(N.astype(np.float32)))
            mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        if "uv" in npz:
            UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
                Vt.Vec2fArray.FromNumpy(npz["uv"].astype(np.float32)))
        prim = mesh.GetPrim()
        mnames = inst["materials"]
        if len(subs) == 1 or len(mnames) <= 1:
            if mnames:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(look(mnames[0]))
        else:
            start = 0
            for i, cnt in enumerate(counts):
                if cnt == 0:
                    continue
                sub = UsdGeom.Subset.CreateGeomSubset(
                    mesh, f"sub{i}", UsdGeom.Tokens.face,
                    Vt.IntArray(list(range(start, start + cnt))), UsdShade.Tokens.materialBind)
                UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(
                    look(mnames[min(i, len(mnames) - 1)]))
                start += cnt
        q = label_of(mnames)
        if q:
            prim.CreateAttribute(QCODE, Sdf.ValueTypeNames.TokenArray).Set([q])
        if inst["collider"]:
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
        n_tris += len(F)
    stage.GetRootLayer().Save()

    # the ground paths: the labelled road, road-line, sidewalk and terrain meshes, under
    # /World/env where the world server references the stage
    ground = []
    for prim in stage.Traverse():
        attr = prim.GetAttribute(QCODE)
        if attr and set(attr.Get() or []) & {"Q34442", "Q1392287", "Q177749", "Q186131"}:
            ground.append("/World/env" + str(prim.GetPath())[len(ROOT):])
    ox, oy = mgrs_local(*man["mapOrigin"])
    with open(os.path.join(out, "world.yaml"), "w") as f:
        f.write("# Generated by scripts/svl_import/svl_to_usd.py from the SVL map "
                f"{man['assetName']} ({man['assetGuid']}).\n"
                "# World frame = the map's local ENU; Autoware's map frame (MGRS local) less map_origin.\n"
                f"usd: {os.path.realpath(os.path.join(out, 'world.usdc'))}\n"
                f"map_origin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                "ground_paths:\n" + "".join(f'  - "{p}"\n' for p in ground))
    print(f"{len(scene['instances'])} instances, {n_tris:,} triangles, {len(looks)} materials, "
          f"{len(ground)} ground meshes; map_origin MGRS-local ({ox:.1f}, {oy:.1f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
