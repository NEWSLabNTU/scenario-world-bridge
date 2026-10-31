#!/usr/bin/env python3
"""The site's trees from the scan: trunks, crowns and heights (roadmap 059 N5).

    trees.py TILES_DIR GROUND_DIR OUT_DIR [--dynamic DYNAMIC_DIR] [--workers 2] [--tiles ix_iy,...]

The scan has no vegetation class (every point is class 0 or the vendor's building, 6), so the
trees are found from shape. Per tile, on the points above the DTM (less transients.py's moved
objects with `--dynamic`), one per VOXEL cube:

- vegetation: a point whose K nearest neighbours within RADIUS scatter in 3-D (smallest over
  largest eigenvalue above SCATTER, planarity below PLANAR) and that stands over VEG_MIN. Walls,
  floor slabs and facades are planar, poles linear; leaves and branches are neither.
- per CELL of the site's grid: the vegetation's height (canopy height model, CHM), its lowest
  point above CROWN_MIN (the crown's base), and the share of the cell's points that are
  vegetation.
- trunk candidates: in TRUNK_LO-TRUNK_HI above the ground, TRUNK_CELL occupancy blobs of
  TRUNK_AREA, each spanning more than TRUNK_SPAN in height. A walking scanner sees trunks from
  below the canopy far better than it sees the treetops above it.

Then over the whole site, so a tree on a tile's edge is one tree: canopy is a cell whose CHM is
over CANOPY_MIN and that is mostly vegetation. A tree is a trunk candidate with canopy within
TRUNK_REACH; canopy no trunk explains is seeded by a treetop (a local maximum of the smoothed
CHM, TOP_SPACING apart, more than TOP_CLEAR from every trunk). Each canopy cell goes to the
nearest seed within CROWN_REACH; a crown under MIN_CROWN is a shrub and no tree.

OUT_DIR/tile_<ix>_<iy>.npz: the tile's grids and trunk candidates (a rerun skips a tile with
one). OUT_DIR/trees.json: one record per tree, map frame (the vendor's less SHIFT, as
build_meshes.py's meshes): x, y (the trunk, or the top), ground z, height, crown base and
radius over ground, kind ("trunk" or "top") and the trunk's radius. OUT_DIR/crowns.npz: the
site's crown labels (index into trees.json, plus one; 0 is no crown) on the CELL grid, with
its origin in the map frame.

Tile 1053_1357 (beside r01): 27 trees on a trunk, 18 on a top only; heights p50 19.5 m,
crown base 4.7 m. A side view of the row along y = 67878 showed 9 of about 12 trunks found
and one lamp post taken for a trunk.
"""
import argparse
import glob
import json
import os
import re
import time
from multiprocessing import Pool

import numpy as np

SHIFT = np.array([-0.54, 0.20, 17.30])  # vendor (MGRS local) minus map frame, as build_meshes.py
TILE = 50.0
CHUNK = 10_000_000
VOXEL = 0.15           # m, one point per cube
ABOVE = 0.3            # m over the DTM: lower is ground
K, RADIUS = 20, 0.6    # neighbourhood for the eigenvalues
MIN_NEIGHBOURS = 8
SCATTER = 0.12         # smallest / largest eigenvalue
PLANAR = 0.5           # (l2 - l3) / l1
VEG_MIN = 1.0          # m over the DTM
CELL = 0.5             # m, the canopy grid
CROWN_MIN = 2.5        # m: vegetation above this is crown
TRUNK_CELL = 0.25      # m
TRUNK_LO, TRUNK_HI = 0.8, 2.5  # m over the DTM
TRUNK_AREA = (0.05, 1.0)       # m2 of a trunk's blob
TRUNK_SPAN = 1.2       # m of height a trunk's blob spans
TRUNK_HITS = 3         # points in a trunk cell
CANOPY_MIN = 2.5       # m
CANOPY_VEG = 0.5       # share of a canopy cell's points that are vegetation
TRUNK_REACH = 2.0      # m from a trunk to its canopy
TOP_SPACING = 4.0      # m between treetops
TOP_MIN = 3.0          # m: a treetop stands higher
TOP_CLEAR = 4.0        # m from a treetop to every trunk
CROWN_REACH = 8.0      # m from a seed to its crown's cells
MIN_CROWN = 3.0        # m2: smaller is a shrub
FEATURE_BLOCK = 1_000_000


def tile_index(path):
    m = re.search(r"tile_(-?\d+)_(-?\d+)\.laz$", path)
    return int(m.group(1)), int(m.group(2))


def eigen_features(P):
    """Scatter, planarity and neighbour count of every point, from its K neighbours in RADIUS."""
    from scipy.spatial import cKDTree

    tree = cKDTree(P)
    scatter = np.zeros(len(P)); planar = np.zeros(len(P)); count = np.zeros(len(P), int)
    for s in range(0, len(P), FEATURE_BLOCK):
        d, idx = tree.query(P[s:s + FEATURE_BLOCK], k=K, distance_upper_bound=RADIUS, workers=4)
        ok = np.isfinite(d)
        n = ok.sum(1)
        Q = np.where(ok[..., None], P[np.minimum(idx, len(P) - 1)], 0.0)
        mu = Q.sum(1) / np.maximum(n, 1)[:, None]
        R = np.where(ok[..., None], Q - mu[:, None], 0.0)
        C = np.einsum("nki,nkj->nij", R, R) / np.maximum(n, 1)[:, None, None]
        ev = np.maximum(np.linalg.eigvalsh(C)[:, ::-1], 1e-9)  # l1 >= l2 >= l3
        scatter[s:s + len(n)] = ev[:, 2] / ev[:, 0]
        planar[s:s + len(n)] = (ev[:, 1] - ev[:, 2]) / ev[:, 0]
        count[s:s + len(n)] = n
    return scatter, planar, count


def _tile(args):
    from transients import keep_mask
    import laspy
    from scipy import ndimage

    path, ground_dir, out, dynamic = args
    t0 = time.time()
    ix, iy = tile_index(path)
    meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
    dtm = np.load(os.path.join(ground_dir, "dtm.npy"), mmap_mode="r")
    keys, pts, hts = [], [], []
    with laspy.open(path) as reader:
        for las in reader.chunk_iterator(CHUNK):
            x, y, z = np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)
            ci = ((x - meta["x0"]) / meta["cell"]).astype(np.int64).clip(0, meta["width"] - 1)
            cj = ((y - meta["y0"]) / meta["cell"]).astype(np.int64).clip(0, meta["height"] - 1)
            h = z - np.asarray(dtm[cj, ci])
            sel = (h > ABOVE) & (np.asarray(las.classification) != 6)
            if dynamic:
                sel &= keep_mask(x, y, z, dynamic, ix, iy)
            P = np.stack([x[sel], y[sel], z[sel]], 1)
            q = np.floor(P / VOXEL).astype(np.int64)
            k = (q[:, 0] * 1_000_000 + q[:, 1]) * 1_000_000 + q[:, 2]
            k, first = np.unique(k, return_index=True)
            keys.append(k); pts.append(P[first]); hts.append(h[sel][first])
    k = np.concatenate(keys)
    _, first = np.unique(k, return_index=True)
    P = np.concatenate(pts)[first]; H = np.concatenate(hts)[first]
    del keys, pts, hts, k
    n = int(round(TILE / CELL))
    x0, y0 = ix * TILE, iy * TILE
    chm = np.zeros((n, n), np.float32); base = np.full((n, n), np.inf, np.float32)
    nveg = np.zeros((n, n), np.int32); nall = np.zeros((n, n), np.int32)
    trunks = np.zeros((0, 3))
    if len(P):
        scatter, planar, count = eigen_features(P)
        veg = (count >= MIN_NEIGHBOURS) & (scatter > SCATTER) & (planar < PLANAR) & (H > VEG_MIN)
        gi = ((P[:, 0] - x0) / CELL).astype(int).clip(0, n - 1)
        gj = ((P[:, 1] - y0) / CELL).astype(int).clip(0, n - 1)
        np.maximum.at(chm, (gj[veg], gi[veg]), H[veg])
        crown = veg & (H > CROWN_MIN)
        np.minimum.at(base, (gj[crown], gi[crown]), H[crown])
        np.add.at(nveg, (gj, gi), veg); np.add.at(nall, (gj, gi), 1)
        # trunks: small occupied blobs in the trunk band that span its height
        band = (H > TRUNK_LO) & (H < TRUNK_HI)
        m = int(round(TILE / TRUNK_CELL))
        ti = ((P[band, 0] - x0) / TRUNK_CELL).astype(int).clip(0, m - 1)
        tj = ((P[band, 1] - y0) / TRUNK_CELL).astype(int).clip(0, m - 1)
        occ = np.zeros((m, m), np.int32); np.add.at(occ, (tj, ti), 1)
        lo = np.full((m, m), np.inf); hi = np.zeros((m, m))
        np.minimum.at(lo, (tj, ti), H[band]); np.maximum.at(hi, (tj, ti), H[band])
        lab, _ = ndimage.label(occ >= TRUNK_HITS)
        found = []
        for k, sl in enumerate(ndimage.find_objects(lab), 1):
            blob = lab[sl] == k
            area = blob.sum() * TRUNK_CELL ** 2
            if TRUNK_AREA[0] <= area <= TRUNK_AREA[1] and hi[sl][blob].max() - lo[sl][blob].min() > TRUNK_SPAN:
                j, i = ndimage.center_of_mass(blob)
                found.append((x0 + (sl[1].start + i + .5) * TRUNK_CELL,
                              y0 + (sl[0].start + j + .5) * TRUNK_CELL, np.sqrt(area / np.pi)))
        trunks = np.array(found).reshape(-1, 3)
    np.savez_compressed(os.path.join(out, f"tile_{ix}_{iy}.npz"), chm=chm, base=base,
                        nveg=nveg, nall=nall, trunks=trunks)
    return {"tile": f"{ix}_{iy}", "voxels": len(P), "trunks": len(trunks),
            "seconds": round(time.time() - t0, 1)}


def site(out, ground_dir):
    """Merge the tiles' grids and split the canopy into trees: trees.json, crowns.npz."""
    from scipy import ndimage

    tiles = sorted(tuple(map(int, re.findall(r"-?\d+", os.path.basename(p))))
                   for p in glob.glob(os.path.join(out, "tile_*_*.npz")))
    i0 = min(t[0] for t in tiles); j0 = min(t[1] for t in tiles)
    n = int(round(TILE / CELL))
    W = (max(t[0] for t in tiles) - i0 + 1) * n; Hh = (max(t[1] for t in tiles) - j0 + 1) * n
    chm = np.zeros((Hh, W), np.float32); base = np.full((Hh, W), np.inf, np.float32)
    share = np.zeros((Hh, W), np.float32)
    trunks = []
    for ix, iy in tiles:
        d = np.load(os.path.join(out, f"tile_{ix}_{iy}.npz"))
        r, c = (iy - j0) * n, (ix - i0) * n
        chm[r:r + n, c:c + n] = d["chm"]; base[r:r + n, c:c + n] = d["base"]
        share[r:r + n, c:c + n] = d["nveg"] / np.maximum(d["nall"], 1)
        trunks.append(d["trunks"])
    trunks = np.concatenate(trunks)
    X0, Y0 = i0 * TILE, j0 * TILE  # vendor frame

    def cell(x, y):
        return (((y - Y0) / CELL).astype(int).clip(0, Hh - 1), ((x - X0) / CELL).astype(int).clip(0, W - 1))

    canopy = ndimage.binary_opening((chm > CANOPY_MIN) & (share > CANOPY_VEG))
    near = ndimage.binary_dilation(canopy, iterations=int(TRUNK_REACH / CELL))
    trunks = trunks[near[cell(trunks[:, 0], trunks[:, 1])]]
    sm = ndimage.gaussian_filter(np.where(canopy, chm, 0), 1.5)
    peaks = (sm == ndimage.maximum_filter(sm, size=int(TOP_SPACING / CELL))) & canopy & (sm > TOP_MIN)
    lab, npk = ndimage.label(peaks)
    tops = np.array(ndimage.center_of_mass(peaks, lab, range(1, npk + 1))).reshape(-1, 2)
    tops = np.stack([X0 + (tops[:, 1] + .5) * CELL, Y0 + (tops[:, 0] + .5) * CELL], 1)
    if len(trunks) and len(tops):
        from scipy.spatial import cKDTree

        d, _ = cKDTree(trunks[:, :2]).query(tops)
        tops = tops[d > TOP_CLEAR]
    seeds = np.concatenate([trunks[:, :2], tops])
    markers = np.zeros((Hh, W), np.int32)
    markers[cell(seeds[:, 0], seeds[:, 1])] = np.arange(1, len(seeds) + 1)
    dist, (nj, ni) = ndimage.distance_transform_edt(markers == 0, return_indices=True)
    crowns = np.where(canopy & (dist * CELL < CROWN_REACH), markers[nj, ni], 0)
    meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
    dtm = np.load(os.path.join(ground_dir, "dtm.npy"), mmap_mode="r")
    idx = np.arange(1, len(seeds) + 1)
    area = ndimage.sum(np.ones_like(crowns), crowns, idx)
    height = ndimage.maximum(chm, crowns, idx)
    fin = np.isfinite(base)
    bsum = ndimage.sum(np.where(fin, base, 0), crowns, idx); bcnt = ndimage.sum(fin, crowns, idx)
    trees, relabel = [], np.zeros(len(seeds) + 1, np.int32)
    for k, (sx, sy) in enumerate(seeds):
        if area[k] * CELL ** 2 < MIN_CROWN:
            continue
        gz = float(dtm[int((sy - meta["y0"]) / meta["cell"]), int((sx - meta["x0"]) / meta["cell"])])
        t = {"x": round(float(sx - SHIFT[0]), 3), "y": round(float(sy - SHIFT[1]), 3),
             "z": round(gz - SHIFT[2], 3), "kind": "trunk" if k < len(trunks) else "top",
             "height": round(float(height[k]), 2),
             "crown_base": round(float(bsum[k] / bcnt[k]), 2) if bcnt[k] else CROWN_MIN,
             "radius": round(float(np.sqrt(area[k] * CELL ** 2 / np.pi)), 2)}
        if k < len(trunks):
            t["trunk_radius"] = round(float(trunks[k, 2]), 3)
        trees.append(t)
        relabel[k + 1] = len(trees)
    json.dump({"frame": "map", "trees": trees}, open(os.path.join(out, "trees.json"), "w"))
    np.savez_compressed(os.path.join(out, "crowns.npz"), crowns=relabel[crowns], cell=CELL,
                        origin=np.array([X0 - SHIFT[0], Y0 - SHIFT[1]]))
    kinds = [t["kind"] for t in trees]
    return {"trees": len(trees), "on_trunk": kinds.count("trunk"), "top_only": kinds.count("top"),
            "height_p50": float(np.median([t["height"] for t in trees])) if trees else 0.0}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tiles_dir")
    ap.add_argument("ground_dir")
    ap.add_argument("out")
    ap.add_argument("--dynamic", default="", help="transients.py's output")
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--tiles", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    paths = sorted(glob.glob(os.path.join(a.tiles_dir, "tile_*_*.laz")))
    if a.tiles:
        want = set(a.tiles.split(","))
        paths = [p for p in paths if "%d_%d" % tile_index(p) in want]
    todo = [p for p in paths if not os.path.exists(os.path.join(a.out, "tile_%d_%d.npz" % tile_index(p)))]
    print(f"{len(todo)} of {len(paths)} tiles to read", flush=True)
    with Pool(a.workers) as pool:
        for k, s in enumerate(pool.imap_unordered(_tile, [(p, a.ground_dir, a.out, a.dynamic) for p in todo])):
            print(f"[{k + 1}/{len(todo)}] {json.dumps(s)}", flush=True)
    print(json.dumps(site(a.out, a.ground_dir)), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
