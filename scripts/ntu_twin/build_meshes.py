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


def tile_index(path):
    m = re.search(r"tile_(-?\d+)_(-?\d+)\.laz$", path)
    return int(m.group(1)), int(m.group(2))


def ground_grid(ix, iy, meta, dtm, ground):
    """Vertices (n, 3), triangles (m, 3) and UVs (n, 2) of the tile's DTM patch."""
    from scipy import ndimage

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
    g = np.asarray(ground[j0:j0 + n, i0:i0 + n])
    g = np.pad(g, ((0, n - g.shape[0]), (0, n - g.shape[1])))
    keep = ndimage.binary_dilation(g, iterations=int(GROUND_REACH / c))
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


def _tile(args):
    path, ground_dir, out = args
    import laspy
    import open3d as o3d

    t0 = time.time()
    ix, iy = tile_index(path)
    meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
    dtm = np.load(os.path.join(ground_dir, "dtm.npy"), mmap_mode="r")
    gcells = np.load(os.path.join(ground_dir, "ground_cells.npy"), mmap_mode="r")
    stats = {"tile": f"{ix}_{iy}"}
    V, F, UV = ground_grid(ix, iy, meta, dtm, gcells)
    np.savez_compressed(os.path.join(out, f"ground_{ix}_{iy}.npz"), points=V.astype(np.float32),
                        faces=F.astype(np.int32), uv=UV.astype(np.float32))
    stats["ground_tris"] = len(F)

    las = laspy.read(path)
    x, y, z = np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)
    ci = ((x - meta["x0"]) / meta["cell"]).astype(np.int64).clip(0, meta["width"] - 1)
    cj = ((y - meta["y0"]) / meta["cell"]).astype(np.int64).clip(0, meta["height"] - 1)
    above = (z - np.asarray(dtm[cj, ci])) >= GROUND_BAND
    cls = np.asarray(las.classification)
    rgb = np.stack([las.red, las.green, las.blue], 1).astype(np.float64)
    rgb /= 65535.0 if rgb.max() > 255 else 255.0
    for name, sel in (("building", above & (cls == 6)), ("other", above & (cls != 6))):
        if sel.sum() < MIN_POINTS:
            continue
        P = np.stack([x[sel], y[sel], z[sel]], 1) - SHIFT
        res = poisson(P, rgb[sel])
        if res is None:
            continue
        mesh, n_in = res
        if len(mesh.triangles) > MAX_TRIS[name]:
            mesh = mesh.simplify_quadric_decimation(MAX_TRIS[name])
        o3d.io.write_triangle_mesh(os.path.join(out, f"{name}_{ix}_{iy}.ply"), mesh)
        stats[f"{name}_points"] = int(sel.sum())
        stats[f"{name}_tris"] = len(mesh.triangles)
    stats["seconds"] = round(time.time() - t0, 1)
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tiles_dir")
    ap.add_argument("ground_dir")
    ap.add_argument("out")
    ap.add_argument("--workers", type=int, default=3,
                    help="each holds one tile (up to ~180 M points, ~12 GB)")
    ap.add_argument("--tiles", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    tiles = sorted(glob.glob(os.path.join(a.tiles_dir, "tile_*.laz")))
    if a.tiles:
        want = set(a.tiles.split(","))
        tiles = [t for t in tiles if "%d_%d" % tile_index(t) in want]
    done = {}
    t0 = time.time()
    with Pool(a.workers, maxtasksperchild=1) as pool:
        for k, st in enumerate(pool.imap_unordered(_tile, [(t, a.ground_dir, a.out) for t in tiles])):
            done[st["tile"]] = st
            print(f"meshes {k + 1}/{len(tiles)} {st}, {time.time() - t0:.0f}s", flush=True)
    with open(os.path.join(a.out, "meshes.json"), "w") as f:
        json.dump({"shift_vendor_minus_map": SHIFT.tolist(), "tile_m": TILE, "tiles": done}, f, indent=1)


if __name__ == "__main__":
    main()
