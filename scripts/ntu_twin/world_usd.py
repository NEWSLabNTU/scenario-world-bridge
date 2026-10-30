#!/usr/bin/env python3
"""The NTU twin's Isaac world (roadmap 059 N6, first version) from build_meshes.py's tiles.

Run by Isaac's Python (it has `pxr`):

    $ISAAC_ROOT/venv/bin/python scripts/ntu_twin/world_usd.py MESH_DIR ORTHO_DIR MAP_DIR
        [--clear-road GROUND_DIR [--clear-margin 2.5]]

Writes `MAP_DIR/isaac/`: one `tiles/tile_<ix>_<iy>.usdc` per tile (its ground with the tile's
2 cm orthophoto, its buildings and its other objects with their scan colours), a
`world.usdc` that holds every tile as a payload under /NTU/Tiles, the orthophotos beside
them, and `world.yaml`.

Frame: the world frame is the map frame (MGRS 51RUH local, the Autoware maps') less
`MAP_ORIGIN`, near the site's middle, so float32 vertices keep millimetres; `world.yaml`'s
`map_origin` tells the bridge. Metres, z up.

Labels (SimReady Q-codes, as the world server's ground paths and surfaces read them): the
ground Road (Q34442) for now, buildings Building (Q41176). Colliders: triangle meshes on the
ground and the buildings only. The other objects (trees as Poisson blobs, the shards Poisson
leaves, parked cars baked in) are seen by camera and lidar but have no collider: a shard over
the road stopped an Autoware drive on r02 (roadmap 059 N8).

`--clear-road GROUND_DIR` drops the other objects' triangles that stand on the road: within
`--clear-margin` (CLEAR_MARGIN) of a lanelet (MAP_DIR/lanelet2_map.osm) and less than CLEAR_HEIGHT above the
ground (GROUND_DIR's DTM). On the lanes, what stands that low is a vehicle, a person or
Poisson's skirt; canopy above it stays, and so does everything off the lanes. Without it,
such pieces 0.15 m outside a 2.17 m lanelet set off Autoware's AEB on r01 (N8). The margin is
2.5 m until N2 removes the survey's transients: beside r01's lanes the scan caught people or
parked scooters that came and went (ground seen for 240 s, then 1864 points at body height in
40 s), and at 0.5 or 1.5 m their meshes still stood beside the car's path. It takes real
roadside shrubs down too.
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
SHIFT = np.array([-0.54, 0.20, 17.30])  # vendor (the DTM's frame) = map + SHIFT
CLEAR_MARGIN = 2.5   # m around the lanelets (default of --clear-margin)
CLEAR_HEIGHT = 2.5   # m above the ground
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


def lane_raster(osm, meta, shape, margin):
    """The lanelets of a Lanelet2 file (map frame) on the DTM's grid (GROUND_DIR/dtm.json's
    `meta`, cells of the vendor frame), grown by `margin` m."""
    import xml.etree.ElementTree as ET

    from scipy import ndimage

    c = meta["cell"]
    lane = np.zeros(shape, bool)
    root = ET.parse(osm).getroot()

    def xy(n):
        return [float(n.find(f"tag[@k='{k}']").get("v")) for k in ("local_x", "local_y")]

    nodes = {n.get("id"): xy(n) for n in root.findall("node")}
    ways = {w.get("id"): [nodes[nd.get("ref")] for nd in w.findall("nd")]
            for w in root.findall("way")}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        m = {x.get("role"): x.get("ref") for x in rel.findall("member")}
        poly = np.array(ways[m["left"]] + ways[m["right"]][::-1]) + SHIFT[:2]
        i0, j0 = np.floor((poly.min(0) - (meta["x0"], meta["y0"])) / c).astype(int)
        i1, j1 = np.ceil((poly.max(0) - (meta["x0"], meta["y0"])) / c).astype(int)
        jj, ii = np.mgrid[max(j0, 0):min(j1, shape[0] - 1) + 1, max(i0, 0):min(i1, shape[1] - 1) + 1]
        px = meta["x0"] + (ii + 0.5) * c
        py = meta["y0"] + (jj + 0.5) * c
        inside = np.zeros(px.shape, bool)
        for (ax, ay), (bx, by) in zip(poly, np.roll(poly, -1, axis=0)):  # even-odd rule
            cross = (ay > py) != (by > py)
            xs = ax + (py - ay) * (bx - ax) / np.where(by == ay, 1e-12, by - ay)
            inside ^= cross & (px < xs)
        lane[jj[inside], ii[inside]] = True
    return ndimage.binary_dilation(lane, iterations=int(round(margin / c)))


class RoadClear:
    """Which map-frame points stand on a lane, below CLEAR_HEIGHT: the lanelets rasterized on
    the DTM's grid and grown by CLEAR_MARGIN."""

    def __init__(self, osm, ground_dir, margin=CLEAR_MARGIN):
        self.meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
        self.dtm = np.load(os.path.join(ground_dir, "dtm.npy"))
        self.lane = lane_raster(osm, self.meta, self.dtm.shape, margin)

    def on_road(self, P):
        """Points (n, 3, map frame) that stand on a lane below CLEAR_HEIGHT."""
        c, m = self.meta["cell"], self.meta
        v = P + SHIFT
        i = np.clip(((v[:, 0] - m["x0"]) / c).astype(int), 0, m["width"] - 1)
        j = np.clip(((v[:, 1] - m["y0"]) / c).astype(int), 0, m["height"] - 1)
        return self.lane[j, i] & (v[:, 2] - self.dtm[j, i] < CLEAR_HEIGHT)


def tile_stage(path, ix, iy, mesh_dir, texture, clear=None):
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
        if clear is not None and name == "other" and len(F):
            F = F[~clear.on_road(V[F].mean(axis=1))]
        if not len(F):
            continue
        vc = vc or _vertex_colour_material(stage, looks + "/ScanColour")
        _mesh(stage, f"/Tile/{name.capitalize()}", V, F, colours=C, label=label, material=vc,
              collider=name == "building")
        tris += len(F)
    stage.GetRootLayer().Save()
    return tris


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mesh_dir")
    ap.add_argument("ortho_dir")
    ap.add_argument("map_dir")
    ap.add_argument("--clear-road", default="", metavar="GROUND_DIR",
                    help="drop other objects standing on the lanes (the DTM's dir)")
    ap.add_argument("--clear-margin", type=float, default=CLEAR_MARGIN,
                    help="m around the lanelets that --clear-road clears")
    a = ap.parse_args(argv)
    clear = RoadClear(os.path.join(a.map_dir, "lanelet2_map.osm"), a.clear_road, a.clear_margin) \
        if a.clear_road else None
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
        n = tile_stage(tp, ix, iy, a.mesh_dir, f"../ortho/rgb_{ix}_{iy}.jpg", clear)
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
