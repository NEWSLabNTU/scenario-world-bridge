#!/usr/bin/env python3
"""The NTU twin's Isaac world (roadmap 059 N6, first version) from build_meshes.py's tiles.

Run by Isaac's Python (it has `pxr`):

    $ISAAC_ROOT/venv/bin/python scripts/ntu_twin/world_usd.py MESH_DIR ORTHO_DIR MAP_DIR
        [--clear-road GROUND_DIR [--clear-margin 2.5] [--trees TREES_DIR [--tree-assets TOWN_DIR]]]

Writes `MAP_DIR/isaac/`: one `tiles/tile_<ix>_<iy>.usdc` per tile (its ground with the tile's
2 cm orthophoto, its buildings and its other objects with their scan colours), a
`world.usdc` that holds every tile as a payload under /NTU/Tiles, the orthophotos beside
them, and `world.yaml`, and the world's light: a sun and a sky dome (`sky.hdr`), see SUN_AZ_EL.
`--lights-only` writes only the light into an existing world.

Frame: the world frame is the map frame (MGRS 51RUH local, the Autoware maps') less
`MAP_ORIGIN`, near the site's middle, so float32 vertices keep millimetres; `world.yaml`'s
`map_origin` tells the bridge. Metres, z up.

Labels (SimReady Q-codes, as the world server's ground paths and surfaces read them): the
ground Road (Q34442) for now, buildings Building (Q41176). Colliders: triangle meshes on the
ground and the buildings only (with `--trees`, the trunks too). The other objects (trees as
Poisson blobs, the shards Poisson leaves, parked cars baked in) are seen by camera and lidar
but have no collider: a shard over the road stopped an Autoware drive on r02 (roadmap 059 N8).

`--trees TREES_DIR` (trees.py's output, N5; needs `--clear-road`): the other objects'
triangles on a crown and over TREE_ABOVE, or within TRUNK_REACH of a trunk below its crown,
become /Tile/Tree, labelled tree (Q10884), so they can be told apart and later replaced; and
every tree found on a trunk gets an invisible cylinder collider from the ground to its crown's
base, labelled the same. A trunk standing on a lane (--clear-road's raster) gets none.

`--tree-assets TOWN_DIR` (a town of the CARLA asset export, its Props/ and Materials/; needs
`--trees`): every tree gets one of TREE_ASSETS instead of the scan's /Tile/Tree mesh, an
instanced reference under /Tile/TreeAssets scaled to its height and crown radius: a palm where
the crown is high and narrow (PALM_BASE, PALM_RADIUS), otherwise the broadleaf whose radius to
height is nearest. None stands on a lane. The scan's other pieces over TREE_ABOVE within
ASSET_CLEAR of a crown go with the Tree mesh (they were the canopy's fringe). The assets, their MDL and textures are copied into
`isaac/trees/`, so the world does not need the export at run time.

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import site_params  # noqa: E402  (sites/<SWB_TWIN_SITE>.yaml, default ntu)

SITE = site_params.current()


MAP_ORIGIN = tuple(SITE["map_origin"])  # NTU (52500, 67700, 0)
SHIFT = np.array(SITE["shift"])  # vendor (the DTM's frame) = map + SHIFT; NTU (-0.54, 0.20, 17.30)
CLEAR_MARGIN = 2.5   # m around the lanelets (default of --clear-margin)
CLEAR_HEIGHT = 2.5   # m above the ground
TREE_ABOVE = 2.5     # m: the other objects' triangles on a crown and above this are the tree's
TRUNK_REACH = 0.4    # m beyond a trunk's radius: its triangles are the tree's too
TRUNK_RADIUS = (0.1, 0.6)  # m, the colliders' bounds
TREE_ASSETS = ["SM_Acer_02", "SM_Sassafras_01", "SM_Sassafras_02", "SM_Japanese_Maple_03", "SM_Beech"]
PALM_ASSET = "SM_CoconutPalm_02"
PALM_BASE = 0.6      # crown base over height at least this, and
PALM_RADIUS = 3.0    # crown radius at most this (m): a palm
ASSET_SPREAD = (0.6, 1.6)  # an asset's horizontal scale within these times its vertical one
ASSET_CLEAR = 2.0    # m beyond a tree's crown radius: with assets, the scan's pieces over TREE_ABOVE go too


def set_label(prim, qcode):
    """The SimReady label, under its schema (LabelsAPI) so Replicator's annotators see it too."""
    from pxr import UsdSemantics

    UsdSemantics.LabelsAPI.Apply(prim, "wikidata_qcode").CreateLabelsAttr().Set([qcode])
LABELS = {"Ground": "Q34442", "Building": "Q41176", "Tree": "Q10884"}
# The world's own light (roadmap 059, rendering): a sun and a sky dome with a clear-sky texture
# (sky.hdr, written beside world.usdc). Engineered, not measured: the sun at SUN_AZ_EL (deg,
# azimuth clockwise from north) gives a mid-morning light over Taipei; its disc is the real
# 0.53 deg (soft shadow edges); the dome's share is about a quarter of the light on the ground
# under a clear sky (sun : sky ~ 2 : 1 on level ground, tuned by eye against render_probe.py's views),
# and its colour paler than a real sky's, as a camera's white balance makes the shade look. A world with its own light
# is lit as it is by aib's runner and swb_isaac_world (their defaults go only into a world with none).
SUN_AZ_EL = (135.0, 55.0)
SUN_INTENSITY = 3000.0
SUN_ANGLE = 0.53
SKY_INTENSITY = 1000.0
SKY_SIZE = (1024, 512)  # the dome texture, latlong
# The scan's own light out of the meshes' colours, and light through the crowns (roadmap 059 N10
# 3, 4; `retouch_tile`). The scanner saw a crown's top sky-lit and its underside in shade, so its
# vertex colours run from near black to near white (linear luminance p10-p90 0.03-0.55 in a
# tile on r01); under RTX's own sun that light is counted twice. Each mesh's luminance is pulled
# toward its median, L' = median (L / median) ** GAMMA, the chroma kept.
DELIGHT_GAMMA = {"Tree": 0.45, "Other": 0.6, "Building": 0.6}
DELIGHT_RANGE = (0.01, 0.6)   # linear luminance after
# A crown is a closed Poisson shell, opaque: its shadow was solid black. A random HOLE_FRACTION of
# its vertices get opacity 0 and the material cuts out where the interpolated opacity is under
# 0.5, so the sun comes through in flecks (two shells: light passes a gap in both).
HOLE_FRACTION = 0.45
# Loose pieces: a Tree component of fewer than SHARD_FACES faces, or an Other one that small
# standing more than SHARD_ABOVE m over the ground, is a fringe of leaves Poisson left floating.
SHARD_FACES = 60
SHARD_ABOVE = 2.0
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


def _leaf_material(stage, path):
    """The scan colour with a cut-out: opacity from the `displayOpacity` primvar, cut at 0.5."""
    from pxr import Sdf, UsdShade

    mat = _vertex_colour_material(stage, path)
    pbr = UsdShade.Shader(stage.GetPrimAtPath(path + "/PBR"))
    rd = UsdShade.Shader.Define(stage, path + "/opacity")
    rd.CreateIdAttr("UsdPrimvarReader_float")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayOpacity")
    rd.CreateInput("fallback", Sdf.ValueTypeNames.Float).Set(1.0)
    rd.CreateOutput("result", Sdf.ValueTypeNames.Float)
    pbr.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(rd.ConnectableAPI(), "result")
    pbr.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.5)
    return mat


def _components(F, n):
    """Each face's connected component (faces sharing a vertex), and the components' sizes."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    k = F.shape[1]
    rows = np.repeat(np.arange(len(F)), k)
    g = coo_matrix((np.ones(len(rows), np.int8), (rows, F.ravel())), shape=(len(F), n)).tocsr()
    _, lab = connected_components(g @ g.T, directed=False)
    return lab, np.bincount(lab)


def retouch_tile(stage):
    """Roadmap 059 N10 3 and 4 on a built tile: the meshes' colours delit (DELIGHT_GAMMA), the
    loose shards dropped (SHARD_FACES), the crowns cut out (HOLE_FRACTION). Once per tile: a
    tile marked `ntu:retouched` is left as it is. Returns {mesh: (faces before, after)}."""
    from pxr import Sdf, UsdGeom, UsdShade, Vt

    root = stage.GetPrimAtPath("/Tile")
    if root.GetCustomDataByKey("ntu:retouched"):
        return {}
    ground = None
    gp = stage.GetPrimAtPath("/Tile/Ground")
    if gp:
        from scipy.spatial import cKDTree

        G = np.asarray(UsdGeom.Mesh(gp).GetPointsAttr().Get())
        ground = (cKDTree(G[:, :2]), G[:, 2])
    out = {}
    for name, gamma in DELIGHT_GAMMA.items():
        prim = stage.GetPrimAtPath(f"/Tile/{name}")
        if not prim:
            continue
        mesh = UsdGeom.Mesh(prim)
        V = np.asarray(mesh.GetPointsAttr().Get())
        F = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        n0 = len(F)
        if name in ("Tree", "Other") and len(F):
            lab, size = _components(F, len(V))
            drop = size[lab] < SHARD_FACES
            if name == "Other" and ground is not None:
                _, i = ground[0].query(V[F].mean(axis=1)[:, :2])
                low = np.full(len(size), np.inf)
                np.minimum.at(low, lab, V[F][:, :, 2].min(axis=1) - ground[1][i])
                drop &= low[lab] > SHARD_ABOVE
            F = F[~drop]
            mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(F.astype(np.int32).ravel()))
            mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
        pv = mesh.GetDisplayColorPrimvar()
        if pv and pv.Get() is not None and len(F):
            C = np.asarray(pv.Get(), np.float64)
            L = C @ [0.2126, 0.7152, 0.0722]
            used = np.unique(F)
            med = float(np.median(L[used])) or 1e-3
            L2 = np.clip(med * (np.maximum(L, 1e-4) / med) ** gamma, *DELIGHT_RANGE)
            C = np.clip(C * (L2 / np.maximum(L, 1e-4))[:, None], 0.0, 1.0)
            pv.Set(Vt.Vec3fArray.FromNumpy(C.astype(np.float32)))
        if name == "Tree" and len(F):
            # a vertex's hole from its position, so a rebuild cuts the same holes
            h = np.abs(np.sin(V[:, 0] * 12.9898 + V[:, 1] * 78.233 + V[:, 2] * 37.719) * 43758.5453) % 1.0
            op = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
                "displayOpacity", Sdf.ValueTypeNames.FloatArray, UsdGeom.Tokens.vertex)
            op.Set(Vt.FloatArray.FromNumpy((h >= HOLE_FRACTION).astype(np.float32)))
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(_leaf_material(stage, "/Tile/Looks/ScanLeaves"))
        out[name] = (n0, len(F))
    root.SetCustomDataByKey("ntu:retouched", True)
    return out


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
        set_label(prim, label)
    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    return prim


def world_yaml_text(site, map_origin):
    """world.yaml for the world written to MAP_DIR/isaac/. `usd:` is relative to the map dir,
    as aib's runner and swb's world server resolve it, so the world loads wherever the map dir
    is copied or mounted (it was an absolute path on the host that built it)."""
    return ("# Generated by scripts/ntu_twin/world_usd.py (roadmap 059), site "
            f"{site['name']}: World frame = map frame less map_origin.\n"
            "usd: isaac/world.usdc\n"
            f"map_origin: [{map_origin[0]}, {map_origin[1]}, {map_origin[2]}]\n"
            "ground_paths:\n"
            '  - "/World/env/Tiles"\n')


def lane_raster(osm, meta, shape, margin):
    """The lanelets of a Lanelet2 file (map frame) on the DTM's grid (GROUND_DIR/dtm.json's
    `meta`, cells of the vendor frame), grown by `margin` m."""
    import xml.etree.ElementTree as ET

    from scipy import ndimage

    c = meta["cell"]
    lane = np.zeros(shape, bool)
    root = ET.parse(osm).getroot()

    nodes = {i: [x, y] for i, (x, y, _) in site_params.lanelet_nodes(root, SITE).items()}
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


class Trees:
    """trees.py's output: which map-frame points are a tree's (on a crown above TREE_ABOVE, or
    around a trunk below its crown), and the trunks in a tile."""

    def __init__(self, trees_dir, ground_dir):
        d = np.load(os.path.join(trees_dir, "crowns.npz"))
        self.crowns, self.cell, self.origin = d["crowns"], float(d["cell"]), d["origin"]
        self.trees = json.load(open(os.path.join(trees_dir, "trees.json")))["trees"]
        self.meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
        self.dtm = np.load(os.path.join(ground_dir, "dtm.npy"), mmap_mode="r")
        t = [q for q in self.trees if q["kind"] == "trunk"]
        self.trunks = np.array([[q["x"], q["y"], q["z"], q["trunk_radius"], q["crown_base"]]
                                for q in t]).reshape(-1, 5)

    def in_tile(self, ix, iy):
        """The trunks (x, y, ground z, radius, crown base; map frame) whose foot is in the tile."""
        v = self.trunks[:, :2] + SHIFT[:2]
        return self.trunks[(np.floor(v[:, 0] / 50.0) == ix) & (np.floor(v[:, 1] / 50.0) == iy)]

    def all_in_tile(self, ix, iy):
        """Every tree (trees.json's records, trunk or top) whose foot is in the tile."""
        return [t for t in self.trees if np.floor((t["x"] + SHIFT[0]) / 50.0) == ix
                and np.floor((t["y"] + SHIFT[1]) / 50.0) == iy]

    def height(self, P):
        """Map-frame points' height over the DTM."""
        m = self.meta
        v = P + SHIFT
        i = np.clip(((v[:, 0] - m["x0"]) / m["cell"]).astype(int), 0, m["width"] - 1)
        j = np.clip(((v[:, 1] - m["y0"]) / m["cell"]).astype(int), 0, m["height"] - 1)
        return v[:, 2] - np.asarray(self.dtm[j, i])

    def in_canopy(self, P):
        """Points over TREE_ABOVE within ASSET_CLEAR of a tree's crown: what an asset replaces."""
        from scipy.spatial import cKDTree

        xy = np.array([[t["x"], t["y"]] for t in self.trees]); r = np.array([t["radius"] for t in self.trees])
        d, k = cKDTree(xy).query(P[:, :2])
        return (d < r[k] + ASSET_CLEAR) & (self.height(P) > TREE_ABOVE)

    def is_tree(self, P):
        h = self.height(P)
        H, W = self.crowns.shape
        ci = ((P[:, 0] - self.origin[0]) / self.cell).astype(int)
        cj = ((P[:, 1] - self.origin[1]) / self.cell).astype(int)
        ok = (ci >= 0) & (ci < W) & (cj >= 0) & (cj < H)
        crown = np.zeros(len(P), bool)
        crown[ok] = self.crowns[cj[ok], ci[ok]] > 0
        tree = crown & (h > TREE_ABOVE)
        if len(self.trunks):
            from scipy.spatial import cKDTree

            d, k = cKDTree(self.trunks[:, :2]).query(P[:, :2])
            tree |= (d < self.trunks[k, 3] + TRUNK_REACH) & (h <= self.trunks[k, 4] + 0.5)
        return tree


def trunk_colliders(stage, trunks):
    """An invisible cylinder per trunk, from the ground to its crown's base: the car hits it,
    the sensors see the scan's mesh."""
    from pxr import Gf, UsdGeom, UsdPhysics

    UsdGeom.Scope.Define(stage, "/Tile/Trunks")
    for k, (x, y, z, r, base) in enumerate(trunks):
        h = max(base, 2.0)
        cyl = UsdGeom.Cylinder.Define(stage, f"/Tile/Trunks/t{k}")
        cyl.CreateRadiusAttr(float(np.clip(r, *TRUNK_RADIUS)))
        cyl.CreateHeightAttr(float(h))
        cyl.CreateAxisAttr(UsdGeom.Tokens.z)
        o = np.array([x, y, z + h / 2]) - np.asarray(MAP_ORIGIN)
        cyl.AddTranslateOp().Set(Gf.Vec3d(*o.tolist()))
        cyl.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        set_label(cyl.GetPrim(), LABELS["Tree"])
    return len(trunks)


def _mdl_deps(path, seen):
    """An MDL module's files: itself, the modules it uses (`using .::M import`) and its textures."""
    if path in seen or not os.path.exists(path):
        return
    seen.add(path)
    d = os.path.dirname(path)
    text = open(path, errors="replace").read()
    for m in re.findall(r"using \.::(\w+) import", text):
        _mdl_deps(os.path.join(d, m + ".mdl"), seen)
    for t in re.findall(r'texture_2d\("(\./[^"]+)"', text):
        seen.add(os.path.normpath(os.path.join(d, t)))


def copy_tree_assets(town_dir, out):
    """Copy TREE_ASSETS and PALM_ASSET with their materials from a CARLA town export into `out`
    (its layout kept); returns {name: (height, crown radius)} from their bounds."""
    from pxr import Usd, UsdGeom, UsdUtils

    sizes, files = {}, set()
    for name in TREE_ASSETS + [PALM_ASSET]:
        src = os.path.join(town_dir, "Props", name + ".usd")
        layers, assets, _ = UsdUtils.ComputeAllDependencies(src)
        files.update(layer.realPath for layer in layers)
        for f in assets:
            (_mdl_deps(f, files) if f.endswith(".mdl") else files.add(f))
        st = Usd.Stage.Open(src)
        b = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
            st.GetPseudoRoot()).ComputeAlignedRange()
        lo, hi = b.GetMin(), b.GetMax()
        sizes[name] = (float(hi[2]), float(hi[0] - lo[0] + hi[1] - lo[1]) / 4)
    town = os.path.realpath(town_dir)
    for f in files:
        rel = os.path.relpath(os.path.realpath(f), town)
        os.makedirs(os.path.dirname(os.path.join(out, rel)), exist_ok=True)
        shutil.copyfile(f, os.path.join(out, rel))
    return sizes


def pick_asset(t, sizes, k):
    """The asset for tree record `t` (the k-th of its tile): a palm, or the broadleaf whose radius
    to height is nearest, the nearer two alternating."""
    if t["crown_base"] >= PALM_BASE * t["height"] and t["radius"] <= PALM_RADIUS:
        return PALM_ASSET
    ratio = t["radius"] / t["height"]
    near = sorted(TREE_ASSETS, key=lambda n: abs(sizes[n][1] / sizes[n][0] - ratio))
    return near[k % 2]


def tree_assets(stage, records, sizes, rel_dir):
    """One instanced reference per tree under /Tile/TreeAssets, scaled to its height and crown."""
    from pxr import Gf, UsdGeom

    UsdGeom.Scope.Define(stage, "/Tile/TreeAssets")
    for k, t in enumerate(records):
        name = pick_asset(t, sizes, k)
        h, r = sizes[name]
        sz = t["height"] / h
        sxy = float(np.clip(t["radius"] / r, ASSET_SPREAD[0] * sz, ASSET_SPREAD[1] * sz))
        x = UsdGeom.Xform.Define(stage, f"/Tile/TreeAssets/a{k}")
        o = np.array([t["x"], t["y"], t["z"]]) - np.asarray(MAP_ORIGIN)
        x.AddTranslateOp().Set(Gf.Vec3d(*o.tolist()))
        x.AddRotateZOp().Set((k * 137.5) % 360.0)
        x.AddScaleOp().Set(Gf.Vec3f(sxy, sxy, sz))
        p = x.GetPrim()
        p.GetReferences().AddReference(f"{rel_dir}/Props/{name}.usd")
        p.SetInstanceable(True)
        set_label(p, LABELS["Tree"])
    return len(records)


def tile_stage(path, ix, iy, mesh_dir, texture, clear=None, trees=None, assets=None):
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
        parts = [(name.capitalize(), F, label)]
        if trees is not None and name == "other" and len(F):
            c = V[F].mean(axis=1)
            t = trees.is_tree(c)
            if assets is not None:
                t |= trees.in_canopy(c)
            parts = [("Tree", F[t], LABELS["Tree"]), ("Other", F[~t], None)]
        for part, Fp, lab in parts:
            if not len(Fp) or (part == "Tree" and assets is not None):
                continue
            vc = vc or _vertex_colour_material(stage, looks + "/ScanColour")
            _mesh(stage, f"/Tile/{part}", V, Fp, colours=C, label=lab, material=vc,
                  collider=name == "building")
            tris += len(Fp)
    if trees is not None:
        trunks = trees.in_tile(ix, iy)
        if clear is not None and len(trunks):
            # a trunk on a lane is a misread (a post, a person N2 missed): no collider there
            trunks = trunks[~clear.on_road(trunks[:, :3] + [0.0, 0.0, 1.0])]
        if len(trunks):
            trunk_colliders(stage, trunks)
    if assets is not None:
        recs = trees.all_in_tile(ix, iy)
        if clear is not None and recs:
            foot = np.array([[t["x"], t["y"], t["z"] + 1.0] for t in recs])
            recs = [t for t, lane in zip(recs, clear.on_road(foot)) if not lane]
        if recs:
            tree_assets(stage, recs, assets, "../trees")
    retouch_tile(stage)
    stage.GetRootLayer().Save()
    return tris


def _write_hdr(path, rgb):
    """A float RGB image as Radiance RGBE (.hdr), uncompressed scanlines."""
    h, w, _ = rgb.shape
    m = rgb.max(axis=2)
    e = np.where(m > 1e-32, np.floor(np.log2(np.maximum(m, 1e-32))) + 1, 0)
    scale = np.where(m > 1e-32, 256.0 / np.exp2(e), 0)
    rgbe = np.zeros((h, w, 4), np.uint8)
    rgbe[..., :3] = np.clip(rgb * scale[..., None], 0, 255).astype(np.uint8)
    rgbe[..., 3] = np.where(m > 1e-32, e + 128, 0).astype(np.uint8)
    with open(path, "wb") as f:
        f.write(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n" + f"-Y {h} +X {w}\n".encode())
        f.write(rgbe.tobytes())


def sky_texture(path, size=SKY_SIZE):
    """A clear sky, latlong, linear: deep blue at the zenith to a pale horizon, a grey-brown
    ground below it (seen only past the world's edge). Azimuth-symmetric, so the dome's
    orientation about z does not matter; the sun is the distant light, not in the texture."""
    w, h = size
    el = (0.5 - (np.arange(h) + 0.5) / h) * np.pi  # +pi/2 at the top row
    zen, hor, gnd = np.array([0.32, 0.45, 0.70]), np.array([0.78, 0.82, 0.86]), np.array([0.30, 0.28, 0.25])
    t = np.clip(np.sin(np.maximum(el, 0.0)), 0, 1) ** 0.5
    rows = np.where(el[:, None] >= 0, (1 - t)[:, None] * hor + t[:, None] * zen, gnd)
    _write_hdr(path, np.repeat(rows[:, None, :], w, axis=1).astype(np.float32))


def add_lights(stage, out):
    """The world's sun and sky under ROOT/Lights (replaced if there), sky.hdr beside world.usdc."""
    import math

    from pxr import Gf, Sdf, UsdGeom, UsdLux

    sky_texture(os.path.join(out, "sky.hdr"))
    path = ROOT + "/Lights"
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    UsdGeom.Xform.Define(stage, path)
    az, el = (math.radians(v) for v in SUN_AZ_EL)
    to_sun = Gf.Vec3d(math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el))
    # a distant light shines down its -z: its z axis points at the sun
    x = Gf.Cross(Gf.Vec3d(0, 0, 1), to_sun).GetNormalized()
    y = Gf.Cross(to_sun, x)
    m = Gf.Matrix4d(x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, to_sun[0], to_sun[1], to_sun[2], 0, 0, 0, 0, 1)
    sun = UsdLux.DistantLight.Define(stage, path + "/sun")
    sun.CreateIntensityAttr(SUN_INTENSITY)
    sun.CreateAngleAttr(SUN_ANGLE)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.90))
    UsdGeom.Xformable(sun.GetPrim()).AddTransformOp().Set(m)
    sky = UsdLux.DomeLight.Define(stage, path + "/sky")
    sky.CreateIntensityAttr(SKY_INTENSITY)
    sky.CreateTextureFileAttr(Sdf.AssetPath("./sky.hdr"))
    sky.CreateTextureFormatAttr(UsdLux.Tokens.latlong)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mesh_dir")
    ap.add_argument("ortho_dir")
    ap.add_argument("map_dir")
    ap.add_argument("--clear-road", default="", metavar="GROUND_DIR",
                    help="drop other objects standing on the lanes (the DTM's dir)")
    ap.add_argument("--clear-margin", type=float, default=CLEAR_MARGIN,
                    help="m around the lanelets that --clear-road clears")
    ap.add_argument("--trees", default="", metavar="TREES_DIR",
                    help="trees.py's output: a Tree layer and trunk colliders (needs --clear-road's GROUND_DIR)")
    ap.add_argument("--tree-assets", default="", metavar="TOWN_DIR",
                    help="a CARLA asset export's town: its tree assets in place of the Tree layer (needs --trees)")
    ap.add_argument("--lights-only", action="store_true",
                    help="only (re)write the sun and sky into MAP_DIR/isaac/world.usdc (mesh_dir and ortho_dir unused)")
    ap.add_argument("--retouch", action="store_true",
                    help="only retouch the built tiles in MAP_DIR/isaac/tiles in place (N10 3, 4; once per tile)")
    a = ap.parse_args(argv)
    if a.retouch:
        from pxr import Usd

        for tp in sorted(glob.glob(os.path.join(a.map_dir, "isaac", "tiles", "tile_*.usdc"))):
            stage = Usd.Stage.Open(tp)
            r = retouch_tile(stage)
            stage.GetRootLayer().Save()
            print(os.path.basename(tp), r or "already retouched", flush=True)
        return 0
    if a.lights_only:
        from pxr import Usd

        out = os.path.join(a.map_dir, "isaac")
        stage = Usd.Stage.Open(os.path.join(out, "world.usdc"))
        add_lights(stage, out)
        stage.GetRootLayer().Save()
        print(f"lights: {out}/world.usdc, {out}/sky.hdr")
        return 0
    if a.tree_assets and not a.trees:
        ap.error("--tree-assets needs --trees")
    clear = RoadClear(os.path.join(a.map_dir, "lanelet2_map.osm"), a.clear_road, a.clear_margin) \
        if a.clear_road else None
    trees = Trees(a.trees, a.clear_road) if a.trees else None
    from pxr import Sdf, Usd, UsdGeom

    out = os.path.join(a.map_dir, "isaac")
    os.makedirs(os.path.join(out, "tiles"), exist_ok=True)
    os.makedirs(os.path.join(out, "ortho"), exist_ok=True)
    assets = copy_tree_assets(a.tree_assets, os.path.join(out, "trees")) if a.tree_assets else None
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
        n = tile_stage(tp, ix, iy, a.mesh_dir, f"../ortho/rgb_{ix}_{iy}.jpg", clear, trees, assets)
        total += n
        prim = stage.DefinePrim(f"{ROOT}/Tiles/t_{ix}_{iy}".replace("-", "m"), "Xform")
        prim.GetPayloads().AddPayload(Sdf.Payload(f"./tiles/tile_{ix}_{iy}.usdc"))
        print(f"tile {ix}_{iy}: {n:,} triangles", flush=True)
    add_lights(stage, out)
    stage.GetRootLayer().Save()
    with open(os.path.join(out, "world.yaml"), "w") as f:
        f.write(world_yaml_text(SITE, MAP_ORIGIN))
    with open(os.path.join(out, "tiles.json"), "w") as f:
        json.dump({"tiles": [f"{ix}_{iy}" for ix, iy in tiles], "triangles": total,
                   "map_origin": MAP_ORIGIN}, f)
    print(f"world: {len(tiles)} tiles, {total:,} triangles")
    if not os.path.exists(os.path.join(a.map_dir, "map_projector_info.yaml")):
        # aib's GNSS projects through the map dir's projector; without it GNSS fell back to a
        # projection about (0, 0), 34 km off, and NDT found no map to initialize on (N8)
        print(f"warning: no map_projector_info.yaml in {a.map_dir}: the ego's GNSS will be wrong",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
