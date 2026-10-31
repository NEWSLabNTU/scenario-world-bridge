#!/usr/bin/env python3
"""The scan's moving objects, by free-space carving along the scanner's trajectory (roadmap 059 N2).

    transients.py TILES_DIR GROUND_DIR TRAJ_DIR OUT_DIR [--workers 3] [--tiles ix_iy,...]

The survey stood in the scan of whatever was there when the backpack passed: people walking,
parked scooters and cars that later left. Their meshes stand in the twin, and Autoware stopped
for one beside r01's lanes (N8). Telling a moved object from one the scanner could not see at
the time takes rays: each return is a ray from the scanner (the vendor's trajectory,
TRAJ_DIR/*.traj, 10 Hz, interpolated at the point's gps_time) to the point, and the space it
crosses was empty then.

In the slab LO-HI above the DTM (where people and vehicles are), on a VOXEL grid in the
vendor's frame, per WINDOW-second window: the windows a voxel was hit in, and the windows a
ray crossed it (sampled every STEP, the last SKIP before the point left out). A voxel is
dynamic when it was crossed in at least max(MIN_FREE, its hit windows) windows it was not hit
in. Porous things (shrubs, crowns, fences) are crossed and hit alike; a voxel is dropped only
where its 0.2 m column holds no static slab voxel, so they stay. Rays come from a sample of
the tile's points (at most RAYS); every slab point counts as a hit.

OUT_DIR/dynamic_<ix>_<iy>.npy: the sorted keys (`voxel_key`) of the dropped voxels;
stats_<ix>_<iy>.json. build_meshes.py and export_map.py take `--dynamic OUT_DIR` and leave
out the points in them. A rerun skips tiles with their stats file.

The trajectory files are Unix time, except sessions written as seconds of the GPS week (their
.gnsstraj gives the week). Those seconds are UTC's, not GPS time (no leap seconds): the
.gnsstraj's UtcTime column is their time of day, and against the scan a .gnsstraj fits at that
offset, not 18 s off it (the share of 0.1 s windows with a return within 3 m of the
interpolated scanner: 98 % against 49 %, tile 1047_1352). A session with no .traj
(20260507114956) falls back to its .gnsstraj, the GNSS antenna's fixes at 1 Hz: the walker
moved ~0.7 m between them (median), and SKIP keeps the interpolation's error off the points'
own voxels. Points with no trajectory sample within MAX_GAP cast no ray.
"""
import argparse
import glob
import json
import os
import re
import time
from multiprocessing import Pool

import numpy as np

VOXEL = 0.2       # m
WINDOW = 10.0     # s
LO, HI = 0.2, 3.0  # m above the DTM
STEP = 0.1        # m along a ray
SKIP = 0.3        # m before the point, not carved
MAX_RANGE = 40.0  # m: longer rays are left out
MIN_FREE = 3      # windows
RAYS = 6_000_000  # per tile
MAX_GAP = 0.6     # s to the nearest trajectory sample (a .gnsstraj is 1 Hz)
CHUNK = 10_000_000
GPS_UNIX = 315_964_800  # 1980-01-06 in Unix time
TILE = 50.0


def voxel_key(x, y, z):
    """A point's voxel on the VOXEL grid, vendor frame, as one int64."""
    ix = np.floor(np.asarray(x) / VOXEL).astype(np.int64)
    iy = np.floor(np.asarray(y) / VOXEL).astype(np.int64)
    iz = np.floor(np.asarray(z) / VOXEL).astype(np.int64) + 2048
    return (ix << 32) | (iy << 12) | iz


def keep_mask(x, y, z, dynamic_dir, ix, iy):
    """False for the points of tile (ix, iy) in its dropped voxels (vendor-frame x y z); all
    True without `dynamic_dir` or the tile's file."""
    p = os.path.join(dynamic_dir or "", f"dynamic_{ix}_{iy}.npy")
    if not dynamic_dir or not os.path.exists(p):
        return np.ones(len(x), bool)
    return ~np.isin(voxel_key(x, y, z), np.load(p))


def load_trajectory(traj_dir):
    """(n, 4): Unix time and the scanner's vendor-frame x, y, z (UTM 51N less the MGRS 51RUH
    square's corner; ellipsoidal height, as the scan's z), sorted by time."""
    from pyproj import Transformer

    utm = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)
    out = []
    for g in sorted(glob.glob(os.path.join(traj_dir, "*.gnsstraj"))):
        if os.path.exists(g[:-9] + ".traj"):
            continue
        a = np.array([[float(v) for v in line.split(",")[:5]] for line in open(g)
                      if line[:1].isdigit()])
        e, n = utm.transform(a[:, 3], a[:, 2])
        out.append(np.c_[GPS_UNIX + a[:, 0] * 604_800 + a[:, 1], e - 300_000.0,
                         n - 2_700_000.0, a[:, 4]])
    for f in sorted(glob.glob(os.path.join(traj_dir, "*.traj"))):
        a = np.loadtxt(f, delimiter=",", comments="#", usecols=range(4))
        t = a[:, 0]
        if t[0] < 1e9:  # GPS seconds of week
            g = f[:-5] + ".gnsstraj"
            week = int(next(line for line in open(g) if line[:1].isdigit()).split(",")[0])
            t = GPS_UNIX + week * 604_800 + t
        e, n = utm.transform(a[:, 1], a[:, 2])
        out.append(np.c_[t, e - 300_000.0, n - 2_700_000.0, a[:, 3]])
    T = np.concatenate(out)
    return T[np.argsort(T[:, 0])]


def _local(x, y, z, lo):
    """The voxel within its tile (8 + 8 + 12 bits); slab voxels only reach a few metres up."""
    i = np.floor((x - lo[0]) / VOXEL).astype(np.int64).clip(0, 255)
    j = np.floor((y - lo[1]) / VOXEL).astype(np.int64).clip(0, 255)
    k = np.floor(z / VOXEL).astype(np.int64).clip(0, 4095)
    return (i << 20) | (j << 12) | k


def tile_index(path):
    m = re.search(r"tile_(-?\d+)_(-?\d+)\.laz$", path)
    return int(m.group(1)), int(m.group(2))


def _height(meta, dtm, x, y, z):
    ci = ((x - meta["x0"]) / meta["cell"]).astype(np.int64).clip(0, meta["width"] - 1)
    cj = ((y - meta["y0"]) / meta["cell"]).astype(np.int64).clip(0, meta["height"] - 1)
    return z - np.asarray(dtm[cj, ci])


def _march(pos, pts, win):
    """Samples every STEP from `pos` to SKIP short of `pts`: (x, y, z, window) arrays."""
    d = pts - pos
    r = np.linalg.norm(d, axis=1)
    keep = (r > SKIP + STEP) & (r < MAX_RANGE)
    pos, d, r, win = pos[keep], d[keep], r[keep], win[keep]
    u = d / r[:, None]
    n = ((r - SKIP) / STEP).astype(np.int64)
    start = np.repeat(np.cumsum(n) - n, n)
    k = np.arange(n.sum()) - start
    ray = np.repeat(np.arange(len(n)), n)
    q = pos[ray] + u[ray] * (k * STEP)[:, None]
    return q, win[ray]


def carve(path, ground_dir, T, out):
    """Carve one tile; write its dropped voxels and stats. A (voxel, window) pair is one int64:
    the tile-local voxel above 20 bits of window since the trajectory's first."""
    import laspy

    t0 = time.time()
    ix, iy = tile_index(path)
    meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
    dtm = np.load(os.path.join(ground_dir, "dtm.npy"), mmap_mode="r")
    lo = np.array([ix * TILE, iy * TILE])  # the tile's vendor-frame corner (retile_laz.py)
    w0 = int(np.floor(T[0, 0] / WINDOW))
    hits, free = [], []
    rng = np.random.default_rng(ix * 100_000 + iy)
    with laspy.open(path) as reader:
        frac = min(1.0, RAYS / max(1, reader.header.point_count))
        for las in reader.chunk_iterator(CHUNK):
            x, y, z = np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)
            t = np.asarray(las.gps_time)
            w = np.floor(t / WINDOW).astype(np.int64) - w0
            h = _height(meta, dtm, x, y, z)
            s = (h > LO) & (h < HI)
            hits.append(np.unique((_local(x[s], y[s], z[s], lo) << 20) | w[s]))
            r = rng.random(len(x)) < frac
            k = np.searchsorted(T[:, 0], t[r]).clip(1, len(T) - 1)
            gap = np.minimum(np.abs(T[k, 0] - t[r]), np.abs(T[k - 1, 0] - t[r]))
            ok = gap < MAX_GAP
            tr = t[r][ok]
            pos = np.stack([np.interp(tr, T[:, 0], T[:, i]) for i in (1, 2, 3)], 1)
            pts = np.stack([x[r][ok], y[r][ok], z[r][ok]], 1)
            for a in range(0, len(pts), 200_000):
                q, qw = _march(pos[a:a + 200_000], pts[a:a + 200_000], w[r][ok][a:a + 200_000])
                inside = ((q[:, 0] >= lo[0]) & (q[:, 0] < lo[0] + TILE)
                          & (q[:, 1] >= lo[1]) & (q[:, 1] < lo[1] + TILE))
                q, qw = q[inside], qw[inside]
                hq = _height(meta, dtm, q[:, 0], q[:, 1], q[:, 2])
                m = (hq > LO) & (hq < HI)
                free.append(np.unique((_local(q[m, 0], q[m, 1], q[m, 2], lo) << 20) | qw[m]))
    hits = np.unique(np.concatenate(hits)) if hits else np.zeros(0, np.int64)
    free = np.unique(np.concatenate(free)) if free else np.zeros(0, np.int64)
    hv, hn = np.unique(hits >> 20, return_counts=True)
    fv, fn = np.unique(free[~np.isin(free, hits)] >> 20, return_counts=True)  # crossed, not hit
    j = np.searchsorted(fv, hv).clip(0, max(0, len(fv) - 1))
    free_only = np.where(fv[j] == hv, fn[j], 0) if len(fv) else np.zeros(len(hv), np.int64)
    dyn = free_only >= np.maximum(MIN_FREE, hn)
    col = hv >> 12  # the voxel's (i, j) in the tile
    drop = hv[dyn & ~np.isin(col, col[~dyn])]
    # back to vendor-frame keys
    i_, j_, k_ = drop >> 20, (drop >> 12) & 255, drop & 4095
    drop = voxel_key(lo[0] + (i_ + 0.5) * VOXEL, lo[1] + (j_ + 0.5) * VOXEL, (k_ + 0.5) * VOXEL)
    np.save(os.path.join(out, f"dynamic_{ix}_{iy}.npy"), np.sort(drop))
    stats = {"tile": f"{ix}_{iy}", "slab_voxels": int(len(hv)), "dynamic_voxels": int(dyn.sum()),
             "dropped_voxels": int(len(drop)), "free_pairs": int(len(free)),
             "seconds": round(time.time() - t0, 1)}
    json.dump(stats, open(os.path.join(out, f"stats_{ix}_{iy}.json"), "w"))
    return stats


def _job(args):
    path, ground_dir, T, out = args
    try:
        return carve(path, ground_dir, T, out)
    except Exception as e:  # one bad tile must not end the run
        return {"tile": os.path.basename(path), "error": repr(e)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tiles_dir")
    ap.add_argument("ground_dir")
    ap.add_argument("traj_dir")
    ap.add_argument("out")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--tiles", default="")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    T = load_trajectory(a.traj_dir)
    print(f"trajectory: {len(T):,} samples, {T[0, 0]:.0f}-{T[-1, 0]:.0f}", flush=True)
    paths = sorted(glob.glob(os.path.join(a.tiles_dir, "tile_*_*.laz")))
    if a.tiles:
        want = set(a.tiles.split(","))
        paths = [p for p in paths if "%d_%d" % tile_index(p) in want]
    paths = [p for p in paths
             if not os.path.exists(os.path.join(a.out, "stats_%d_%d.json" % tile_index(p)))]
    print(f"{len(paths)} tiles to carve", flush=True)
    with Pool(a.workers) as pool:
        for k, s in enumerate(pool.imap_unordered(_job, [(p, a.ground_dir, T, a.out) for p in paths])):
            print(f"[{k + 1}/{len(paths)}] {json.dumps(s)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
