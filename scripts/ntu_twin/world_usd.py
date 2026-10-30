#!/usr/bin/env python3
"""The NTU twin's Isaac world (roadmap 059 N6, first version) from build_meshes.py's tiles.

Run by Isaac's Python (it has `pxr`):

    $ISAAC_ROOT/venv/bin/python scripts/ntu_twin/world_usd.py MESH_DIR ORTHO_DIR MAP_DIR

Writes `MAP_DIR/isaac/`: one `tiles/tile_<ix>_<iy>.usdc` per tile (its ground with the tile's
2 cm orthophoto, its buildings and its other objects with their scan colours), a
`world.usdc` that holds every tile as a payload under /NTU/Tiles, the orthophotos beside
them, and `world.yaml`.

Frame: the world frame is the map frame (MGRS 51RUH local, the Autoware maps') less
`MAP_ORIGIN`, near the site's middle, so float32 vertices keep millimetres; `world.yaml`'s
`map_origin` tells the bridge. Metres, z up.

Labels (SimReady Q-codes, as the world server's ground paths and surfaces read them): the
ground Road (Q34442) for now, buildings Building (Q41176). Colliders: triangle meshes on the
ground, the buildings and the other objects.
"""
import argparse
import glob
import json
import os
import re
import shutil
import sys

import numpy as np

MAP_ORIGIN = (52500.0, 67700.0, 0.0)
QCODE = "semantics:labels:wikidata_qcode"
LABELS = {"Ground": "Q34442", "Building": "Q41176"}
ROOT = "/NTU"


def _ply(path):
    """Vertices, triangles, vertex colours of an ASCII or binary PLY (open3d's output)."""
    import open3d as o3d  # noqa: F401  (optional; fallback below)

    m = o3d.io.read_triangle_mesh(path)
    return (np.asarray(m.vertices), np.asarray(m.triangles), np.asarray(m.vertex_colors))


def read_ply(path):
    try:
        return _ply(path)
    except ImportError:
        pass
    # minimal binary little-endian reader for open3d's PLYs: x y z (double) [nx ny nz] r g b
    with open(path, "rb") as f:
        header = []
        while True:
            line = f.readline().decode().strip()
            header.append(line)
            if line == "end_header":
                break
        nv = int(next(h for h in header if h.startswith("element vertex")).split()[-1])
        nf = int(next(h for h in header if h.startswith("element face")).split()[-1])
        props = [h.split() for h in header if h.startswith("property") and "list" not in h]
        vprops = props[:len(props)]
        types = {"double": "<f8", "float": "<f4", "uchar": "u1"}
        dt = np.dtype([(p[2], types[p[1]]) for p in vprops])
        v = np.frombuffer(f.read(nv * dt.itemsize), dtype=dt)
        fdt = np.dtype([("n", "u1"), ("i", "<i4", 3)])
        fc = np.frombuffer(f.read(nf * fdt.itemsize), dtype=fdt)
    V = np.stack([v["x"], v["y"], v["z"]], 1)
    C = np.stack([v["red"], v["green"], v["blue"]], 1) / 255.0 if "red" in v.dtype.names else None
    return V, fc["i"], C


def _vertex_colour_material(stage, path):
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    pbr = UsdShade.Shader.Define(stage, path + "/PBR")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    rd = UsdShade.Shader.Define(stage, path + "/colour")
    rd.CreateIdAttr("UsdPrimvarReader_float3")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
    rd.CreateOutput("result", Sdf.ValueTypeNames.Float3)
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rd.ConnectableAPI(), "result")
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    return mat


def _texture_material(stage, path, texture):
    from pxr import Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, path)
    pbr = UsdShade.Shader.Define(stage, path + "/PBR")
    pbr.CreateIdAttr("UsdPreviewSurface")
    pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
    pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    st = UsdShade.Shader.Define(stage, path + "/st")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tex = UsdShade.Shader.Define(stage, path + "/photo")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
    return mat


def srgb_to_linear(c):
    """The scan's colours are sRGB; a displayColor primvar is linear (RTX reads it as such)."""
    c = np.clip(c, 0.0, 1.0)
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def vertex_normals(V, F):
    """Area-weighted smooth normals, so Poisson meshes do not render faceted."""
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    n = np.zeros_like(V)
    for k in range(3):
        np.add.at(n, F[:, k], fn)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    return np.where(ln > 0, n / np.maximum(ln, 1e-12), [0.0, 0.0, 1.0])


def _mesh(stage, path, V, F, uv=None, colours=None, label=None, material=None, collider=True):
    from pxr import Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

    V = (V - np.asarray(MAP_ORIGIN)).astype(np.float32)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V))
    mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(
        vertex_normals(V.astype(np.float64), F).astype(np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), F.shape[1], np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.astype(np.int32).ravel()))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr(Vt.Vec3fArray([tuple(V.min(0).tolist()), tuple(V.max(0).tolist())]))
    pv = UsdGeom.PrimvarsAPI(mesh)
    if uv is not None:
        pv.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
            Vt.Vec2fArray.FromNumpy(uv.astype(np.float32)))
    if colours is not None:
        mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(
            Vt.Vec3fArray.FromNumpy(srgb_to_linear(colours).astype(np.float32)))
    prim = mesh.GetPrim()
    if collider:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    if label:
        prim.CreateAttribute(QCODE, Sdf.ValueTypeNames.TokenArray).Set([label])
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    return prim


def tile_stage(path, ix, iy, mesh_dir, texture):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Tile")
    stage.SetDefaultPrim(root.GetPrim())
    looks = "/Tile/Looks"
    UsdGeom.Scope.Define(stage, looks)
    tris = 0
    g = np.load(os.path.join(mesh_dir, f"ground_{ix}_{iy}.npz"))
    if len(g["faces"]):
        _mesh(stage, "/Tile/Ground", g["points"].astype(np.float64), g["faces"], uv=g["uv"],
              label=LABELS["Ground"], material=_texture_material(stage, looks + "/Ortho", texture))
        tris += len(g["faces"])
    vc = None
    for name, label in (("building", LABELS["Building"]), ("other", None)):
        p = os.path.join(mesh_dir, f"{name}_{ix}_{iy}.ply")
        if not os.path.exists(p):
            continue
        V, F, C = read_ply(p)
        if not len(F):
            continue
        vc = vc or _vertex_colour_material(stage, looks + "/ScanColour")
        _mesh(stage, f"/Tile/{name.capitalize()}", V, F, colours=C, label=label, material=vc)
        tris += len(F)
    stage.GetRootLayer().Save()
    return tris


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mesh_dir")
    ap.add_argument("ortho_dir")
    ap.add_argument("map_dir")
    a = ap.parse_args(argv)
    from pxr import Sdf, Usd, UsdGeom

    out = os.path.join(a.map_dir, "isaac")
    os.makedirs(os.path.join(out, "tiles"), exist_ok=True)
    os.makedirs(os.path.join(out, "ortho"), exist_ok=True)
    tiles = sorted({tuple(map(int, re.findall(r"-?\d+", os.path.basename(p))))
                    for p in glob.glob(os.path.join(a.mesh_dir, "ground_*.npz"))})
    stage = Usd.Stage.CreateNew(os.path.join(out, "world.usdc"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Xform.Define(stage, ROOT + "/Tiles")
    total = 0
    for ix, iy in tiles:
        src = os.path.join(a.ortho_dir, f"rgb_{ix}_{iy}.jpg")
        shutil.copyfile(src, os.path.join(out, "ortho", f"rgb_{ix}_{iy}.jpg"))
        tp = os.path.join(out, "tiles", f"tile_{ix}_{iy}.usdc")
        n = tile_stage(tp, ix, iy, a.mesh_dir, f"../ortho/rgb_{ix}_{iy}.jpg")
        total += n
        prim = stage.DefinePrim(f"{ROOT}/Tiles/t_{ix}_{iy}".replace("-", "m"), "Xform")
        prim.GetPayloads().AddPayload(Sdf.Payload(f"./tiles/tile_{ix}_{iy}.usdc"))
        print(f"tile {ix}_{iy}: {n:,} triangles", flush=True)
    stage.GetRootLayer().Save()
    with open(os.path.join(out, "world.yaml"), "w") as f:
        f.write("# Generated by scripts/ntu_twin/world_usd.py (roadmap 059): the NTU campus from the\n"
                "# vendor's MLS scan. World frame = map frame (MGRS 51RUH local) less map_origin.\n"
                f"usd: {os.path.realpath(os.path.join(out, 'world.usdc'))}\n"
                f"map_origin: [{MAP_ORIGIN[0]}, {MAP_ORIGIN[1]}, {MAP_ORIGIN[2]}]\n"
                "ground_paths:\n"
                f'  - "/World/env/Tiles"\n')
    with open(os.path.join(out, "tiles.json"), "w") as f:
        json.dump({"tiles": [f"{ix}_{iy}" for ix, iy in tiles], "triangles": total,
                   "map_origin": MAP_ORIGIN}, f)
    print(f"world: {len(tiles)} tiles, {total:,} triangles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
