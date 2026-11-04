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


# Spikes and bumps in the ground on and beside the lanes. 059's DTM put the road up to 6.5 m high
# into the canopy in 7 places (smooth_dtm.py's lane rule took a crown for the lowest return where
# the whole window was crown; 118 fixed it in smooth_dtm.py 2cedde5, leaving 0.4-0.9 m), and the
# ego stopped in front of the cone (the tour). Bringing single vertices down to a local percentile
# left 0.25 m steps that the ego's ground filter called obstacles and stopped for (the tour, run C,
# leg 00 at (499, 232)). So the lane's surface is opened (grey erosion then dilation over
# SPIKE_WINDOW, lane cells only): a slope or a plane comes through unchanged and a bump narrower
# than the window is cut off. A lane vertex more than SPIKE_ABOVE over the opening comes down to it,
# then the ground within SPIKE_NEAR of one that came down, if it stands over the lane beside it.
# SPIKE_ABOVE 0.08 lowered 48k road vertices in 51 tiles (median 0.10 m: crowns, humps); 0.15
# keeps those and still takes the steps the ground filter stops for.
SPIKE_WINDOW, SPIKE_ABOVE, SPIKE_NEAR = 8.0, 0.15, 2.0
ELE_K, ELE_R, ELE_OFF = 6, 6.0, 0.25  # lanelet nodes, m, m: the lane's surface from the lanelets' ele
ROUND_SIGMA, ROUND_ABOVE = 1.5, 0.05
BLEND_NEAR, BLEND_SIGMA = 1.5, 0.75  # m: round what moved, the Gaussian that blends it  # m: the opened lane's convex edges, rounded to its Gaussian


def despike(V, F, ix, iy):
    """V's lane vertices brought down to the opened lane surface. The opening runs over this tile
    and its eight neighbours' ground (as 059 built it), so a tile's edge sees the same surface as
    the tile beside it: per tile, the window stopped at the edge and left a 0.3 m ridge on the seam
    at (-289, -151), where the ego sat on its floor (the tour, run D, leg 02)."""
    from scipy import ndimage
    from scipy.spatial import cKDTree

    on = _ctx["road"].on_road(V)
    if not on.any():
        return V, 0
    V, at_ele = to_lane_ele(V, on)
    ctx = [V[on]]
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            p = os.path.join(_ctx["a"]["mesh"], f"ground_{ix + dx}_{iy + dy}.npz")
            if (dx or dy) and os.path.exists(p):
                Vn = np.load(p)["points"].astype(np.float64)
                if len(Vn):
                    on_n = _ctx["road"].on_road(Vn)
                    ctx.append(to_lane_ele(Vn, on_n)[0][on_n])
    L = np.concatenate(ctx)
    cell = _ctx["meta"]["cell"]
    lo = L[:, :2].min(axis=0)
    ij_l = np.round((L[:, :2] - lo) / cell).astype(int)
    shape = tuple(ij_l.max(axis=0) + 1)
    Z = np.full(shape, np.inf)
    np.minimum.at(Z, (ij_l[:, 0], ij_l[:, 1]), L[:, 2])
    k = int(round(SPIKE_WINDOW / cell)) | 1
    ero = ndimage.minimum_filter(Z, size=k, mode="constant", cval=np.inf)
    ero[~np.isfinite(Z)] = -np.inf
    opened = ndimage.maximum_filter(ero, size=k, mode="constant", cval=-np.inf)
    # then the opened surface's convex edges rounded (only ever lowered): a ramp's top edge left by
    # the DTM (0.4 m over 2 m at (-289, -151)) high-centred the car on it (run D, leg 02)
    lane = np.isfinite(opened)
    S = np.where(lane, opened, 0.0)
    sig = ROUND_SIGMA / cell
    w = ndimage.gaussian_filter(lane.astype(float), sig)
    sm = np.where(lane, ndimage.gaussian_filter(S, sig) / np.maximum(w, 1e-9), np.inf)
    opened = np.where(lane & (opened - sm > ROUND_ABOVE), sm, opened)
    ij = np.clip(np.round((V[:, :2] - lo) / cell).astype(int), 0, np.array(shape) - 1)
    ref = opened[ij[:, 0], ij[:, 1]]
    out = V.copy()
    lane_fix = on & np.isfinite(ref) & (V[:, 2] - ref > SPIKE_ABOVE)
    out[lane_fix, 2] = ref[lane_fix]
    lane_fix |= at_ele
    # the edges of what moved, blended: a Gaussian of the lane surface over the moved vertices and
    # BLEND_NEAR round them (hard replacement left 0.1-0.3 m steps at a patch's edge)
    if lane_fix.any():
        Zo = np.full(shape, np.nan)
        Zo[ij[on, 0], ij[on, 1]] = out[on, 2]
        lane_r = np.isfinite(Zo)
        Mv = np.zeros(shape, bool)
        Mv[ij[lane_fix, 0], ij[lane_fix, 1]] = True
        Mv = ndimage.binary_dilation(Mv, iterations=int(round(BLEND_NEAR / cell))) & lane_r
        sg = BLEND_SIGMA / cell
        wsum = ndimage.gaussian_filter(lane_r.astype(float), sg)
        zs = ndimage.gaussian_filter(np.where(lane_r, Zo, 0.0), sg) / np.maximum(wsum, 1e-9)
        sel = on & Mv[ij[:, 0], ij[:, 1]]
        # only ever lowered, as everything here
        out[sel, 2] = np.minimum(out[sel, 2], zs[ij[sel, 0], ij[sel, 1]])
        lane_fix |= sel
    n = int(lane_fix.sum())
    if n:
        # the cone's foot off the lane: down to the lowered lane beside it
        f = cKDTree(V[lane_fix, :2])
        d, i = f.query(V[:, :2], distance_upper_bound=SPIKE_NEAR)
        foot = ~on & np.isfinite(d)
        lane_z = out[np.flatnonzero(lane_fix)[np.minimum(i[foot], n - 1)], 2]
        high = V[foot, 2] - lane_z > 0.3
        idx = np.flatnonzero(foot)[high]
        out[idx, 2] = lane_z[high]
        n += len(idx)
    return out, n


_ctx = {}


def _init(a):
    _ctx["road"] = W.RoadClear(a["osm"], a["ground"], ROAD_MARGIN)
    _ctx["clear"] = W.RoadClear(a["osm"], a["ground"], 0.5)
    _ctx["trees"] = W.Trees(a["trees"], a["ground"])
    _ctx["parked"] = np.load(a["parked"])["mask"] if a.get("parked") else None
    _ctx["hedges"] = dict(np.load(a["hedges"])) if a.get("hedges") else None
    _ctx["meta"] = json.load(open(os.path.join(a["ground"], "dtm.json")))
    _ctx["a"] = a
    from scipy.spatial import cKDTree

    nodes = []
    for n in ET.parse(a["osm"]).getroot().findall("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        nodes.append((float(t["local_x"]), float(t["local_y"]), float(t.get("ele", 0))))
    nodes = np.array(nodes)
    _ctx["ele"] = (cKDTree(nodes[:, :2]), nodes[:, 2])


def lane_ele(P):
    """The lanelets' surface under map-frame points: the median ele of the ELE_K nearest lanelet
    nodes within ELE_R (NaN where there are none)."""
    tree, z = _ctx["ele"]
    d, i = tree.query(P[:, :2], k=ELE_K, distance_upper_bound=ELE_R)
    zz = np.where(np.isfinite(d), z[np.minimum(i, len(z) - 1)], np.nan)
    with np.errstate(all="ignore"):
        return np.nanmedian(zz, axis=1)


def to_lane_ele(P, on):
    """Lane points more than ELE_OFF over the lanelets' surface put on it: where the scan has no
    ground under a canopy, the DTM's fill invents the road (0.3-0.5 m over the lanelets at
    (101, 325), where the ego's ground filter stopped it: the tour, run D, leg 24)."""
    e = lane_ele(P)
    # only ever lowered: where the lanelets stand higher than the DTM (the raised deck at (-124, -91))
    # raising made 1 m steps at the deck's edge
    fix = on & np.isfinite(e) & (P[:, 2] - e > ELE_OFF)
    out = P.copy()
    out[fix, 2] = e[fix]
    return out, fix


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
    # not on a lane (the road clearing's raster, 0.5 m round the lanelets): the field's blur spread
    # the roadside hedges up to 0.75 m onto the lane, and the ego's lidar stopped it there (the tour)
    top = np.where(M & ~_ctx["clear"].lane[np.ix_(js, is_)], hg["top"][np.ix_(js, is_)].astype(np.float64), 0.0)
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
            V, stats["despiked"] = despike(V, F, ix, iy)
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
