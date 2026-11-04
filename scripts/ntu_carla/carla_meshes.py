#!/usr/bin/env python3
"""The NTU campus's meshes for CARLA (roadmap 072 C3-C5), per 50 m tile, from roadmap 059's.

    carla_meshes.py MESH_DIR MAP_DIR GROUND_DIR TREES_DIR OUT_DIR [--tiles ix_iy,...|--route r01]
        [--keep 0.3] [--workers 4] [--parked PARKED.npz] [--hedges HEDGES.npz]

Run with the NTU tool venv (open3d, scipy). Reads 059's `build_meshes.py` output (MESH_DIR:
ground_<t>.npz, building_<t>.ply, other_<t>.ply), the Lanelet2 map, the DTM and `trees.py`'s
output, and reuses `scripts/ntu_twin/world_usd.py`'s rules: the lane raster, the road clearing
(`RoadClear`, 0.5 m), the tree split (`Trees`), the de-lighting (DELIGHT_GAMMA), the shards
(SHARD_FACES, SHARD_ABOVE) and the crown cut-outs (HOLE_FRACTION). Writes OUT_DIR/tile_<t>.npz,
the classes CARLA's import tags by name:

  road     the ground under a lanelet: Road_Road (tag Road), the ortho's UVs
  terrain  the rest of the ground: Road_Grass (tag Terrain), the ortho's UVs
  building the scan's buildings, de-lit and decimated, vertex colours
  prop     the other objects off the road, without the trees' faces, de-lit and decimated;
           with `--parked` (`parked_vehicles.py`'s mask) without the parked vehicles' faces
           below PARKED_BELOW
  tree     the trees' faces, de-lit and decimated; alpha 0 at a hashed HOLE_FRACTION of the
           vertices, which the master material cuts out (as 059's Isaac crowns); with
           `--hedges` (`hedges.py`'s height field) the low vegetation's faces under its mask
           (trees' and props') are left out and the height field takes their place, a grid on
           the DTM's cells with alpha 1, its edge down on the ground

Vertices are in the map frame less MAP_ORIGIN (metres, z up), as the OpenDRIVE; colours are
linear RGBA in 0..1. UE 4.26 has no Nanite: the buildings, props and trees keep `--keep` of
their triangles (quadric decimation); the ground keeps its 0.25 m grid.
"""
import argparse
import json
import os
import re
import sys
import time
import xml.etree.ElementTree as ET
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ntu_twin"))
import world_usd as W  # noqa: E402

import hedges as HG  # noqa: E402

PARKED_BELOW = 2.6  # m over the DTM: a parked vehicle's faces, under the mask (parked_vehicles.TALL)
ROAD_MARGIN = 0.25  # m (one DTM cell; 0 would make lane_raster dilate without end): Road, past it Terrain


def route_tiles(osm, route, reach=30.0):
    """The tiles within `reach` m of a route's lanelets (r01: ids from 1 000 000, r02: below)."""
    root = ET.parse(osm).getroot()
    xy = {n.get("id"): (float(n.find("tag[@k='local_x']").get("v")), float(n.find("tag[@k='local_y']").get("v")))
          for n in root.iter("node")}
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.iter("way")}
    pts = []
    for r in root.iter("relation"):
        rid = int(r.get("id"))
        if (route == "r01") != (rid >= 1000000):
            continue
        for m in r.findall("member"):
            if m.get("type") == "way":
                pts += [xy[i] for i in ways[m.get("ref")]]
    P = np.array(pts) + W.SHIFT[:2]
    out = set()
    for dx in (-reach, 0, reach):
        for dy in (-reach, 0, reach):
            out |= {(int(a), int(b)) for a, b in np.floor((P + (dx, dy)) / 50.0)}
    return sorted(out)


def delight(C, gamma):
    """059's rule on linear colours: luminance toward its median, chroma kept."""
    L = C @ [0.2126, 0.7152, 0.0722]
    med = float(np.median(L)) or 1e-3
    L2 = np.clip(med * (np.maximum(L, 1e-4) / med) ** gamma, *W.DELIGHT_RANGE)
    return np.clip(C * (L2 / np.maximum(L, 1e-4))[:, None], 0.0, 1.0)


def compact(V, F, C=None):
    used = np.unique(F)
    remap = np.full(len(V), -1, np.int64)
    remap[used] = np.arange(len(used))
    return V[used], remap[F], (C[used] if C is not None else None)


def decimate(V, F, C, keep):
    import open3d as o3d

    if keep >= 1.0 or len(F) < 200:
        return V, F, C
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(F))
    m.vertex_colors = o3d.utility.Vector3dVector(C)
    m = m.simplify_quadric_decimation(max(int(len(F) * keep), 100))
    m.remove_degenerate_triangles()
    m.remove_unreferenced_vertices()
    return np.asarray(m.vertices), np.asarray(m.triangles), np.asarray(m.vertex_colors)


# After decimation: pieces this small (faces), standing this high over the ground (m), are the
# fringes the decimation left floating; they cast hard shadows on the road (072 pilot).
SHARD_FACES_AFTER = 25
SHARD_ABOVE_AFTER = 1.0


def drop_shards(V, F, kind, ground, faces=None, above=None):
    faces = W.SHARD_FACES if faces is None else faces
    above = W.SHARD_ABOVE if above is None else above
    lab, size = W._components(F, len(V))
    drop = size[lab] < faces
    if kind == "prop" and ground is not None:
        _, i = ground[0].query(V[F].mean(axis=1)[:, :2])
        low = np.full(len(size), np.inf)
        np.minimum.at(low, lab, V[F][:, :, 2].min(axis=1) - ground[1][i])
        drop &= low[lab] > above
    return F[~drop]


def holes(V):
    h = np.abs(np.sin(V[:, 0] * 12.9898 + V[:, 1] * 78.233 + V[:, 2] * 37.719) * 43758.5453) % 1.0
    return (h >= W.HOLE_FRACTION).astype(np.float32)


_ctx = {}


def _init(a):
    _ctx["road"] = W.RoadClear(a["osm"], a["ground"], ROAD_MARGIN)
    _ctx["clear"] = W.RoadClear(a["osm"], a["ground"], 0.5)
    _ctx["trees"] = W.Trees(a["trees"], a["ground"])
    _ctx["parked"] = np.load(a["parked"])["mask"] if a.get("parked") else None
    _ctx["hedges"] = dict(np.load(a["hedges"])) if a.get("hedges") else None
    _ctx["meta"] = json.load(open(os.path.join(a["ground"], "dtm.json")))
    _ctx["a"] = a


def parked(P):
    """Faces (centroids, map frame) of a parked vehicle: under the mask, below PARKED_BELOW."""
    mask, m = _ctx["parked"], _ctx["meta"]
    if mask is None or not len(P):
        return np.zeros(len(P), bool)
    v = P + W.SHIFT
    i = np.clip(((v[:, 0] - m["x0"]) / m["cell"]).astype(int), 0, m["width"] - 1)
    j = np.clip(((v[:, 1] - m["y0"]) / m["cell"]).astype(int), 0, m["height"] - 1)
    return mask[j, i] & (_ctx["trees"].height(P) < PARKED_BELOW)


def _cell(P):
    """Map-frame points' DTM cell (j, i)."""
    m = _ctx["meta"]
    v = P + W.SHIFT
    i = np.clip(((v[:, 0] - m["x0"]) / m["cell"]).astype(int), 0, m["width"] - 1)
    j = np.clip(((v[:, 1] - m["y0"]) / m["cell"]).astype(int), 0, m["height"] - 1)
    return j, i


def hedge_faces(V, F, C):
    """Of the faces F (off the road), the low vegetation under the hedges' mask."""
    hg = _ctx["hedges"]
    if hg is None or not len(F):
        return np.zeros(len(F), bool)
    veg, _, _ = HG.low_vegetation(V, F, C, _ctx["trees"])
    j, i = _cell(V[F].mean(axis=1))
    return veg & hg["mask"][j, i]


def hedge_mesh(ix, iy):
    """The hedges' height field in tile (ix, iy): a vertex per DTM cell centre, a quad between
    four of them when all four are within a cell of the mask and one is in it; the vertices
    outside the mask sit on the DTM. Map frame; colours sRGB 0..1."""
    hg, m = _ctx["hedges"], _ctx["meta"]
    if hg is None:
        return None
    n = int(round(50.0 / m["cell"]))
    i0 = int(round((ix * 50.0 - m["x0"]) / m["cell"]))
    j0 = int(round((iy * 50.0 - m["y0"]) / m["cell"]))
    if i0 + n < 0 or j0 + n < 0 or i0 >= m["width"] or j0 >= m["height"]:
        return None
    js = np.clip(np.arange(j0, j0 + n + 1), 0, m["height"] - 1)
    is_ = np.clip(np.arange(i0, i0 + n + 1), 0, m["width"] - 1)
    M = hg["mask"][np.ix_(js, is_)]
    if not M.any():
        return None
    from scipy import ndimage

    D = ndimage.binary_dilation(hg["mask"][np.ix_(np.clip(np.arange(j0 - 1, j0 + n + 2), 0, m["height"] - 1),
                                                   np.clip(np.arange(i0 - 1, i0 + n + 2), 0, m["width"] - 1))])[1:-1, 1:-1]
    q = D[:-1, :-1] & D[:-1, 1:] & D[1:, 1:] & D[1:, :-1] & (M[:-1, :-1] | M[:-1, 1:] | M[1:, 1:] | M[1:, :-1])
    if not q.any():
        return None
    dtm = np.asarray(_ctx["trees"].dtm[np.ix_(js, is_)], np.float64)
    top = np.where(M, hg["top"][np.ix_(js, is_)].astype(np.float64), 0.0)
    jj, ii = np.meshgrid(js, is_, indexing="ij")
    V = np.stack([m["x0"] + (ii + 0.5) * m["cell"], m["y0"] + (jj + 0.5) * m["cell"], dtm + top], axis=-1).reshape(-1, 3) - W.SHIFT
    C = hg["colour"][np.ix_(js, is_)].reshape(-1, 3).astype(np.float64) / 255.0
    w = n + 1
    a = np.flatnonzero(q.ravel())
    a = (a // n) * w + a % n  # the quad's lower-left vertex
    F = np.r_[np.c_[a, a + 1, a + w + 1], np.c_[a, a + w + 1, a + w]]
    return compact(V, F, C)


def one(t):
    from scipy.spatial import cKDTree

    a = _ctx["a"]
    ix, iy = t
    origin = np.array([W.MAP_ORIGIN[0], W.MAP_ORIGIN[1], 0.0])
    out, stats = {}, {}
    gp = os.path.join(a["mesh"], f"ground_{ix}_{iy}.npz")
    ground = None
    if os.path.exists(gp):
        g = np.load(gp)
        V, F, UV = g["points"].astype(np.float64), g["faces"], g["uv"]
        if len(F):
            on = _ctx["road"].on_road(V[F].mean(axis=1))
            for cls, sel in (("road", on), ("terrain", ~on)):
                if sel.any():
                    used = np.unique(F[sel])
                    remap = np.full(len(V), -1, np.int64)
                    remap[used] = np.arange(len(used))
                    out[f"{cls}_V"] = (V[used] - origin).astype(np.float32)
                    out[f"{cls}_F"] = remap[F[sel]].astype(np.int32)
                    out[f"{cls}_UV"] = UV[used].astype(np.float32)
                    stats[cls] = int(sel.sum())
            ground = (cKDTree(V[:, :2]), V[:, 2])
    for name in ("building", "other"):
        p = os.path.join(a["mesh"], f"{name}_{ix}_{iy}.ply")
        if not os.path.exists(p):
            continue
        V, F, C = W.read_ply(p)
        if not len(F):
            continue
        C = C if C is not None else np.full((len(V), 3), 0.5)
        if name == "building":
            parts = [("building", F)]
        else:
            F = F[~_ctx["clear"].on_road(V[F].mean(axis=1))]
            hf = hedge_faces(V, F, C)
            if _ctx["hedges"] is not None:
                stats["hedge_faces"] = int(hf.sum())
            F = F[~hf]
            tr = _ctx["trees"].is_tree(V[F].mean(axis=1))
            Fp = F[~tr]
            pk = parked(V[Fp].mean(axis=1))
            stats["parked"] = int(pk.sum())
            parts = [("tree", F[tr]), ("prop", Fp[~pk])]
        C = W.srgb_to_linear(C)
        for cls, Fp in parts:
            if not len(Fp):
                continue
            n0 = len(Fp)
            if cls in ("tree", "prop"):
                Fp = drop_shards(V, Fp, cls, ground)
            if not len(Fp):
                continue
            Vc, Fc, Cc = compact(V, Fp, C)
            Cc = delight(Cc, W.DELIGHT_GAMMA["Tree" if cls == "tree" else "Building" if cls == "building" else "Other"])
            Vc, Fc, Cc = decimate(Vc, Fc, Cc, a["keep"])
            if cls in ("tree", "prop") and len(Fc):
                Fc = drop_shards(Vc, Fc, "prop", ground, SHARD_FACES_AFTER, SHARD_ABOVE_AFTER)
                Vc, Fc, Cc = compact(Vc, Fc, Cc)
            if not len(Fc):
                continue
            alpha = holes(Vc) if cls == "tree" else np.ones(len(Vc), np.float32)
            out[f"{cls}_V"] = (Vc - origin).astype(np.float32)
            out[f"{cls}_F"] = Fc.astype(np.int32)
            out[f"{cls}_C"] = np.c_[Cc, alpha].astype(np.float32)
            stats[cls] = (n0, int(len(Fc)))
    h = hedge_mesh(ix, iy)
    if h is not None:
        Vh, Fh, Ch = h
        Ch = np.c_[delight(W.srgb_to_linear(Ch), W.DELIGHT_GAMMA["Tree"]), np.ones(len(Vh))]
        Vh = Vh - origin
        if "tree_V" in out:
            k = len(out["tree_V"])
            out["tree_V"] = np.r_[out["tree_V"], Vh].astype(np.float32)
            out["tree_F"] = np.r_[out["tree_F"], Fh + k].astype(np.int32)
            out["tree_C"] = np.r_[out["tree_C"], Ch].astype(np.float32)
        else:
            out["tree_V"], out["tree_F"], out["tree_C"] = Vh.astype(np.float32), Fh.astype(np.int32), Ch.astype(np.float32)
        stats["hedge"] = int(len(Fh))
    np.savez_compressed(os.path.join(a["out"], f"tile_{ix}_{iy}.npz"), **out)
    return t, stats


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mesh_dir")
    ap.add_argument("map_dir")
    ap.add_argument("ground_dir")
    ap.add_argument("trees_dir")
    ap.add_argument("out")
    ap.add_argument("--tiles", default="")
    ap.add_argument("--route", default="", help="r01 or r02: the tiles within 30 m of its lanelets")
    ap.add_argument("--keep", type=float, default=0.3)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--parked", default="", help="parked_vehicles.py's mask: leave those out")
    ap.add_argument("--hedges", default="", help="hedges.py's height field: in place of the low vegetation")
    a = ap.parse_args(argv)
    osm = os.path.join(a.map_dir, "lanelet2_map.osm")
    if a.tiles:
        tiles = [tuple(map(int, t.split("_"))) for t in a.tiles.split(",")]
    elif a.route:
        tiles = route_tiles(osm, a.route)
    else:
        tiles = sorted({(int(m.group(1)), int(m.group(2))) for f in os.listdir(a.mesh_dir)
                        if (m := re.match(r"ground_(-?\d+)_(-?\d+)\.npz$", f))})
    os.makedirs(a.out, exist_ok=True)
    ctx = {"mesh": a.mesh_dir, "osm": osm, "ground": a.ground_dir, "trees": a.trees_dir,
           "out": a.out, "keep": a.keep, "parked": a.parked,
           "hedges": a.hedges}
    t0, report = time.time(), {}
    with Pool(a.workers, initializer=_init, initargs=(ctx,)) as pool:
        for k, (t, st) in enumerate(pool.imap_unordered(one, tiles)):
            report[f"{t[0]}_{t[1]}"] = st
            print(f"{k + 1}/{len(tiles)} {t[0]}_{t[1]} {st} {time.time() - t0:.0f}s", flush=True)
    json.dump({"tiles": report, "keep": a.keep}, open(os.path.join(a.out, "tiles.json"), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
