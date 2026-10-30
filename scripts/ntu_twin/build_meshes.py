#!/usr/bin/env python3
"""The NTU twin's meshes per 50 m tile (roadmap 059 N3-N5, first version), in the map frame.

For every tile `retile_laz.py` wrote, beside `ground_ortho.py`'s DTM and orthophotos:

  ground_<ix>_<iy>.npz   the DTM's 0.25 m grid over the tile, cells kept where the ground was
                         seen within `GROUND_REACH` (no skirts over the scan's edge); UVs into
                         the tile's 2 cm orthophoto.
  building_<ix>_<iy>.ply the vendor's building class (6), screened Poisson, vertex colours.
  other_<ix>_<iy>.ply    everything else above the ground (vegetation, poles, signs, parked
                         vehicles, unclassified), the same way. Trees come out as blobs; the
                         first version keeps them so the lidar and the camera see something
                         where they stand.

All in the map frame of the Autoware maps r01/r02: MGRS 51RUH local minus the vendor-to-map
shift `SHIFT` (019: about (-0.54, +0.20, +17.30) m; it is subtracted from the vendor's
coordinates). The meshes stay in that frame; the USD stage places them.

Usage:
  build_meshes.py TILES_DIR GROUND_DIR OUT_DIR [--workers 3] [--tiles ix_iy,...]

A rerun skips the tiles that have their `stats_<ix>_<iy>.json` (written when a tile is
complete), so a stopped build resumes.
"""
import argparse
import glob
import json
import os
import re
import time
from multiprocessing import Pool

import numpy as np

SHIFT = np.array([-0.54, 0.20, 17.30])  # vendor (MGRS local) minus map frame
TILE = 50.0
GROUND_BAND = 0.12      # m, as ground_ortho.py
GROUND_REACH = 2.0      # m: a DTM cell is meshed within this of a ground cell
VOXEL = 0.08            # m, points per Poisson input voxel
DEPTH = 11              # Poisson octree depth over a 50 m tile: ~2.5 cm cells
DENSITY_CUT = 0.08      # drop the Poisson vertices of lowest support (this quantile)
MAX_TRIS = {"building": 400_000, "other": 400_000}
MIN_POINTS = 2000       # a layer with fewer points is skipped
CHUNK = 10_000_000      # points read at a time


def voxel_keys(P, ix, iy):
    """One int64 per VOXEL cube, unique within the tile (map frame, z from -1000 m)."""
    q = np.floor((P - np.array([ix * TILE - 10.0, iy * TILE - 10.0, -1000.0])) / VOXEL).astype(np.int64)
    return (q[:, 0] * 1_000_000 + q[:, 1]) * 1_000_000 + q[:, 2]


def tile_index(path):
    m = re.search(r"tile_(-?\d+)_(-?\d+)\.laz$", path)
    return int(m.group(1)), int(m.group(2))


def ground_keep(ground, cell):
    """The DTM cells to mesh, site-wide: those within GROUND_REACH of a ground cell, and every
    hole that leaves enclosed. Without the fill, a patch where something stood during the scan
    (parked cars, a stopped vehicle, scooters) more than GROUND_REACH from ground was a hole in
    the road, and a car spawned over one on r01 fell through the world (roadmap 059 N8). All
    2142 such holes on the site are enclosed (the largest under 2000 m2)."""
    from scipy import ndimage

    keep = ndimage.binary_dilation(np.asarray(ground), iterations=int(GROUND_REACH / cell))
    return ndimage.binary_fill_holes(keep)


def ground_grid(ix, iy, meta, dtm, keep_site):
    """Vertices (n, 3), triangles (m, 3) and UVs (n, 2) of the tile's DTM patch."""
    c = meta["cell"]
    i0 = int(round((ix * TILE - meta["x0"]) / c))
    j0 = int(round((iy * TILE - meta["y0"]) / c))
    n = int(round(TILE / c))
    # vertices on cell corners: n + 1 per side, heights from the cell centres around them
    z = np.asarray(dtm[max(j0 - 1, 0):j0 + n + 1, max(i0 - 1, 0):i0 + n + 1], dtype=np.float64)
    pad = ((1 if j0 == 0 else 0, 0), (1 if i0 == 0 else 0, 0))
    z = np.pad(z, pad, mode="edge")
    z = np.pad(z, ((0, n + 2 - z.shape[0]), (0, n + 2 - z.shape[1])), mode="edge")
    zc = 0.25 * (z[:-1, :-1] + z[1:, :-1] + z[:-1, 1:] + z[1:, 1:])  # (n + 1, n + 1) corners
    keep = np.asarray(keep_site[j0:j0 + n, i0:i0 + n])
    keep = np.pad(keep, ((0, n - keep.shape[0]), (0, n - keep.shape[1])))
    jj, ii = np.mgrid[0:n + 1, 0:n + 1]
    V = np.stack([ix * TILE + ii * c, iy * TILE + jj * c, zc], axis=-1).reshape(-1, 3)
    UV = np.stack([ii / n, jj / n], axis=-1).reshape(-1, 2)  # v up: row 0 of the image is north
    cj, ci = np.nonzero(keep)
    a = cj * (n + 1) + ci
    b, cc, d = a + 1, a + n + 2, a + n + 1
    F = np.concatenate([np.stack([a, b, cc], 1), np.stack([a, cc, d], 1)])
    used = np.unique(F)
    remap = np.full(len(V), -1)
    remap[used] = np.arange(len(used))
    return V[used] - SHIFT, remap[F], UV[used]


def poisson(P, C):
    import open3d as o3d

    off = P.min(0)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P - off))
    pc.colors = o3d.utility.Vector3dVector(C)
    pc = pc.voxel_down_sample(VOXEL)
    if len(pc.points) < MIN_POINTS:
        return None
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.4, max_nn=30))
    pc.orient_normals_consistent_tangent_plane(20)
    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pc, depth=DEPTH)
    d = np.asarray(dens)
    mesh.remove_vertices_by_mask(d < np.quantile(d, DENSITY_CUT))
    # Poisson closes the surface past the points: keep only what lies near them
    from scipy.spatial import cKDTree

    d, _ = cKDTree(np.asarray(pc.points)).query(np.asarray(mesh.vertices), workers=4)
    mesh.remove_vertices_by_mask(d > 3 * VOXEL)
    mesh.translate(off)
    return mesh, len(pc.points)


def write_ground(ix, iy, ground_dir, out):
    """The tile's ground mesh, ground_<ix>_<iy>.npz; its triangle count."""
    meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
    dtm = np.load(os.path.join(ground_dir, "dtm.npy"), mmap_mode="r")
    keep = np.load(os.path.join(out, "ground_keep.npy"), mmap_mode="r")
    V, F, UV = ground_grid(ix, iy, meta, dtm, keep)
    np.savez_compressed(os.path.join(out, f"ground_{ix}_{iy}.npz"), points=V.astype(np.float32),
                        faces=F.astype(np.int32), uv=UV.astype(np.float32))
    return len(F)


def _tile(args):
    path, ground_dir, out = args
    import laspy
    import open3d as o3d

    t0 = time.time()
    ix, iy = tile_index(path)
    stats = {"tile": f"{ix}_{iy}"}
    stats["ground_tris"] = write_ground(ix, iy, ground_dir, out)
    meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
    dtm = np.load(os.path.join(ground_dir, "dtm.npy"), mmap_mode="r")

    # read in chunks and keep one point per VOXEL cube per layer as we go: a dense tile is
    # ~180 M points, and holding it whole (laspy's record plus float64 copies) took ~35 GB
    keep = {"building": [], "other": []}
    counts = {"building": 0, "other": 0}
    with laspy.open(path) as reader:
        for las in reader.chunk_iterator(CHUNK):
            x, y, z = np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)
            ci = ((x - meta["x0"]) / meta["cell"]).astype(np.int64).clip(0, meta["width"] - 1)
            cj = ((y - meta["y0"]) / meta["cell"]).astype(np.int64).clip(0, meta["height"] - 1)
            above = (z - np.asarray(dtm[cj, ci])) >= GROUND_BAND
            cls = np.asarray(las.classification)
            rgb = np.stack([las.red, las.green, las.blue], 1)
            for name, sel in (("building", above & (cls == 6)), ("other", above & (cls != 6))):
                if not sel.any():
                    continue
                counts[name] += int(sel.sum())
                P = np.stack([x[sel], y[sel], z[sel]], 1) - SHIFT
                key = voxel_keys(P, ix, iy)
                k, first = np.unique(key, return_index=True)
                keep[name].append((k, P[first], rgb[sel][first]))
    for name in ("building", "other"):
        parts = keep.pop(name)
        if counts[name] < MIN_POINTS or not parts:
            continue
        # chunks overlap in voxels: one point per voxel over the whole tile
        k = np.concatenate([q[0] for q in parts])
        _, first = np.unique(k, return_index=True)
        P = np.concatenate([q[1] for q in parts])[first]
        C = np.concatenate([q[2] for q in parts])[first].astype(np.float64)
        del parts, k
        C /= 65535.0 if C.max() > 255 else 255.0
        res = poisson(P, C)
        if res is None:
            continue
        mesh, n_in = res
        if len(mesh.triangles) > MAX_TRIS[name]:
            mesh = mesh.simplify_quadric_decimation(MAX_TRIS[name])
        o3d.io.write_triangle_mesh(os.path.join(out, f"{name}_{ix}_{iy}.ply"), mesh)
        stats[f"{name}_points"] = counts[name]
        stats[f"{name}_tris"] = len(mesh.triangles)
    stats["seconds"] = round(time.time() - t0, 1)
    # written last: a tile with its stats file is complete, and a rerun skips it
    with open(os.path.join(out, f"stats_{ix}_{iy}.json"), "w") as f:
        json.dump(stats, f)
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tiles_dir")
    ap.add_argument("ground_dir")
    ap.add_argument("out")
    ap.add_argument("--workers", type=int, default=3,
                    help="each holds one tile (up to ~180 M points, ~19 GB at peak)")
    ap.add_argument("--tiles", default="")
    ap.add_argument("--ground-only", action="store_true",
                    help="rewrite only the ground meshes of the tiles already in OUT (no LAZ read)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    meta = json.load(open(os.path.join(a.ground_dir, "dtm.json")))
    np.save(os.path.join(a.out, "ground_keep.npy"),
            ground_keep(np.load(os.path.join(a.ground_dir, "ground_cells.npy")), meta["cell"]))
    if a.ground_only:
        names = sorted(glob.glob(os.path.join(a.out, "ground_*_*.npz")))
        tiles = [tuple(int(v) for v in re.findall(r"(-?\d+)_(-?\d+)\.npz$", p)[0]) for p in names]
        if a.tiles:
            tiles = [t for t in tiles if "%d_%d" % t in set(a.tiles.split(","))]
        mj = os.path.join(a.out, "meshes.json")
        index = json.load(open(mj)) if os.path.exists(mj) else {"tiles": {}}
        for k, (ix, iy) in enumerate(tiles):
            n = write_ground(ix, iy, a.ground_dir, a.out)
            index["tiles"].setdefault(f"{ix}_{iy}", {})["ground_tris"] = n
            sp = os.path.join(a.out, f"stats_{ix}_{iy}.json")
            if os.path.exists(sp):
                st = json.load(open(sp))
                st["ground_tris"] = n
                json.dump(st, open(sp, "w"))
        json.dump(index, open(mj, "w"), indent=1)
        print(f"ground: {len(tiles)} tiles rewritten")
        return
    tiles = sorted(glob.glob(os.path.join(a.tiles_dir, "tile_*.laz")))
    if a.tiles:
        want = set(a.tiles.split(","))
        tiles = [t for t in tiles if "%d_%d" % tile_index(t) in want]
    done = {}
    for t in list(tiles):
        p = os.path.join(a.out, "stats_%d_%d.json" % tile_index(t))
        if os.path.exists(p):
            st = json.load(open(p))
            done[st["tile"]] = st
            tiles.remove(t)
    print(f"meshes: {len(done)} tiles done before, {len(tiles)} to build", flush=True)
    t0 = time.time()
    with Pool(a.workers, maxtasksperchild=1) as pool:
        for k, st in enumerate(pool.imap_unordered(_tile, [(t, a.ground_dir, a.out) for t in tiles])):
            done[st["tile"]] = st
            print(f"meshes {k + 1}/{len(tiles)} {st}, {time.time() - t0:.0f}s", flush=True)
    with open(os.path.join(a.out, "meshes.json"), "w") as f:
        json.dump({"shift_vendor_minus_map": SHIFT.tolist(), "tile_m": TILE, "tiles": done}, f, indent=1)


if __name__ == "__main__":
    main()
