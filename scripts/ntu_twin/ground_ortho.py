#!/usr/bin/env python3
"""The NTU twin's ground (roadmap 059 N3): a terrain model and orthophotos from the 50 m tiles.

Three passes over the tiles `retile_laz.py` wrote (MGRS 51RUH local, vendor Z datum):

  1. Per tile, the lowest point in every `CELL` cell of one site-wide grid -> zmin.npy.
  2. Over the whole site at once, a simple morphological ground filter (SMRF-like: openings
     of growing window, a point is ground when it lies within slope x window of the opened
     surface) on zmin, holes filled by nearest ground and smoothed -> dtm.npy (+ dtm.json:
     origin, cell). Objects narrower than `MAX_WINDOW` (parked cars, hedges, kiosks) drop out.
  3. Per tile, the points within `GROUND_BAND` of the DTM, averaged into `ORTHO` m pixels:
     RGB and intensity orthophotos, ortho/rgb_<ix>_<iy>.jpg and ortho/int_<ix>_<iy>.png, with
     empty pixels filled from coarser levels (a pyramid fill) and a coverage mask.

Usage:
  ground_ortho.py TILES_DIR OUT_DIR [--passes 1,2,3] [--workers 6] [--tiles ix_iy,...]
"""
import argparse
import glob
import json
import os
import re
import time
from multiprocessing import Pool

import numpy as np

CELL = 0.25          # m, the DTM grid
TILE = 50.0          # m, retile_laz.py's tiles
ORTHO = 0.02         # m per ortho pixel
GROUND_BAND = 0.12   # m above the DTM that still counts as ground
SLOPE = 0.15         # SMRF slope (rise per metre of window)
MAX_WINDOW = 16.0    # m, the largest object the filter removes
ELEV_THRESHOLD = 0.3 # m, SMRF's base height threshold


def tile_index(path):
    m = re.search(r"tile_(-?\d+)_(-?\d+)\.laz$", path)
    return int(m.group(1)), int(m.group(2))


def read_tile(path, fields):
    import laspy

    las = laspy.read(path)
    return {f: (np.asarray(getattr(las, f))) for f in fields}


def grid_of(tiles):
    ix = [tile_index(t)[0] for t in tiles]
    iy = [tile_index(t)[1] for t in tiles]
    x0, y0 = min(ix) * TILE, min(iy) * TILE
    w = int(round((max(ix) + 1 - min(ix)) * TILE / CELL))
    h = int(round((max(iy) + 1 - min(iy)) * TILE / CELL))
    return x0, y0, w, h


# -- pass 1 -------------------------------------------------------------------------------

def _zmin(args):
    path, x0, y0, w, h = args
    p = read_tile(path, ("x", "y", "z"))
    i = ((p["x"] - x0) / CELL).astype(np.int64).clip(0, w - 1)
    j = ((p["y"] - y0) / CELL).astype(np.int64).clip(0, h - 1)
    cell = j * w + i
    u, inv = np.unique(cell, return_inverse=True)
    z = np.full(len(u), np.inf)
    np.minimum.at(z, inv, p["z"])
    return u, z, len(p["x"])


def pass1(tiles, out, workers):
    x0, y0, w, h = grid_of(tiles)
    zmin = np.full(w * h, np.inf, dtype=np.float64)
    n = 0
    t0 = time.time()
    with Pool(workers) as pool:
        for k, (u, z, cnt) in enumerate(pool.imap_unordered(_zmin, [(t, x0, y0, w, h) for t in tiles])):
            zmin[u] = np.minimum(zmin[u], z)
            n += cnt
            print(f"pass1 {k + 1}/{len(tiles)} tiles, {n:,} points, {time.time() - t0:.0f}s", flush=True)
    np.save(os.path.join(out, "zmin.npy"), zmin.reshape(h, w).astype(np.float32))
    with open(os.path.join(out, "dtm.json"), "w") as f:
        json.dump({"x0": x0, "y0": y0, "cell": CELL, "width": w, "height": h, "points": n}, f)


# -- pass 2 -------------------------------------------------------------------------------

def pass2(out):
    from scipy import ndimage

    zmin = np.load(os.path.join(out, "zmin.npy")).astype(np.float64)
    empty = ~np.isfinite(zmin)
    # empty cells take their nearest neighbour's value for the openings only
    idx = ndimage.distance_transform_edt(empty, return_distances=False, return_indices=True)
    z = zmin[tuple(idx)]
    ground = np.ones_like(z, dtype=bool)
    surf = z
    win = 1.0
    while win <= MAX_WINDOW:
        r = max(1, int(round(win / CELL / 2)))
        fp = np.hypot(*np.mgrid[-r:r + 1, -r:r + 1]) <= r
        opened = ndimage.grey_opening(surf, footprint=fp)
        ground &= (z - opened) <= ELEV_THRESHOLD + SLOPE * win
        surf = opened
        print(f"pass2 window {win:g} m: ground {ground[~empty].mean():.3f}", flush=True)
        win *= 2
    ground &= ~empty
    # the DTM: ground cells, every other cell from its nearest ground cell, smoothed lightly
    idx = ndimage.distance_transform_edt(~ground, return_distances=False, return_indices=True)
    dtm = z[tuple(idx)]
    dtm = ndimage.gaussian_filter(dtm, 1.0)
    dtm[ground] = np.minimum(dtm[ground], z[ground] + 0.05)
    np.save(os.path.join(out, "dtm.npy"), dtm.astype(np.float32))
    np.save(os.path.join(out, "ground_cells.npy"), ground)
    print(f"pass2 ground cells {ground.sum():,} of {(~empty).sum():,} occupied", flush=True)


# -- pass 3 -------------------------------------------------------------------------------

def pyramid_fill(img, mask, levels=8):
    """Fill pixels where mask is False from successively coarser averages of filled ones."""
    out = img.astype(np.float32).copy()
    filled = mask.copy()
    cur_img, cur_w = out * mask[..., None], mask.astype(np.float32)
    stack = []
    for _ in range(levels):
        h, w = cur_w.shape
        h2, w2 = (h + 1) // 2, (w + 1) // 2
        pi = np.zeros((h2 * 2, w2 * 2, img.shape[2]), np.float32)
        pw = np.zeros((h2 * 2, w2 * 2), np.float32)
        pi[:h, :w], pw[:h, :w] = cur_img, cur_w
        cur_img = pi.reshape(h2, 2, w2, 2, -1).sum((1, 3))
        cur_w = pw.reshape(h2, 2, w2, 2).sum((1, 3))
        stack.append((cur_img, cur_w))
    for k, (ci, cw) in enumerate(stack):
        f = 2 ** (k + 1)
        avg = ci / np.maximum(cw, 1e-6)[..., None]
        up = np.repeat(np.repeat(avg, f, 0), f, 1)[:out.shape[0], :out.shape[1]]
        ok = np.repeat(np.repeat(cw > 0, f, 0), f, 1)[:out.shape[0], :out.shape[1]]
        take = ~filled & ok
        out[take] = up[take]
        filled |= take
    return out, filled


def _ortho(args):
    path, out, meta = args
    from PIL import Image

    ix, iy = tile_index(path)
    dtm = np.load(os.path.join(out, "dtm.npy"), mmap_mode="r")
    p = read_tile(path, ("x", "y", "z", "red", "green", "blue", "intensity"))
    ci = ((p["x"] - meta["x0"]) / CELL).astype(np.int64).clip(0, meta["width"] - 1)
    cj = ((p["y"] - meta["y0"]) / CELL).astype(np.int64).clip(0, meta["height"] - 1)
    g = (p["z"] - np.asarray(dtm[cj, ci])) < GROUND_BAND
    tx0, ty0 = ix * TILE, iy * TILE
    n = int(round(TILE / ORTHO))
    oi = ((p["x"][g] - tx0) / ORTHO).astype(np.int64).clip(0, n - 1)
    oj = ((p["y"][g] - ty0) / ORTHO).astype(np.int64).clip(0, n - 1)
    pix = (n - 1 - oj) * n + oi  # row 0 is the tile's north edge
    cnt = np.bincount(pix, minlength=n * n)
    rgb_max = max(int(p["red"].max()), 1)
    scale = 255.0 / (65535.0 if rgb_max > 255 else 255.0)
    chans = [np.bincount(pix, weights=p[c][g].astype(np.float64) * scale, minlength=n * n)
             for c in ("red", "green", "blue")]
    chans.append(np.bincount(pix, weights=p["intensity"][g].astype(np.float64), minlength=n * n))
    img = np.stack(chans, axis=1) / np.maximum(cnt, 1)[:, None]
    mask = (cnt > 0).reshape(n, n)
    img, filled = pyramid_fill(img.reshape(n, n, 4), mask)
    os.makedirs(os.path.join(out, "ortho"), exist_ok=True)
    Image.fromarray(img[..., :3].clip(0, 255).astype(np.uint8)).save(
        os.path.join(out, "ortho", f"rgb_{ix}_{iy}.jpg"), quality=92)
    Image.fromarray(img[..., 3].clip(0, 65535).astype(np.uint16)).save(
        os.path.join(out, "ortho", f"int_{ix}_{iy}.png"))
    Image.fromarray((mask * 255).astype(np.uint8)).save(
        os.path.join(out, "ortho", f"mask_{ix}_{iy}.png"))
    return ix, iy, float(mask.mean()), int(g.sum()), len(g)


def pass3(tiles, out, workers):
    meta = json.load(open(os.path.join(out, "dtm.json")))
    t0 = time.time()
    stats = {}
    with Pool(workers) as pool:
        for k, (ix, iy, cov, ng, n) in enumerate(pool.imap_unordered(_ortho, [(t, out, meta) for t in tiles])):
            stats[f"{ix}_{iy}"] = {"coverage": round(cov, 4), "ground_points": ng, "points": n}
            print(f"pass3 {k + 1}/{len(tiles)} tile {ix}_{iy}: coverage {cov:.2f}, "
                  f"{time.time() - t0:.0f}s", flush=True)
    with open(os.path.join(out, "ortho", "tiles.json"), "w") as f:
        json.dump({"ortho_m": ORTHO, "tile_m": TILE, "tiles": stats}, f, indent=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tiles_dir")
    ap.add_argument("out")
    ap.add_argument("--passes", default="1,2,3")
    ap.add_argument("--workers", type=int, default=6,
                    help="each holds one tile's points (~30-180 M points: up to ~8 GB)")
    ap.add_argument("--tiles", default="", help="only these tiles in pass 3, e.g. 1047_1350,1048_1350")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    tiles = sorted(glob.glob(os.path.join(a.tiles_dir, "tile_*.laz")))
    passes = {int(s) for s in a.passes.split(",")}
    if 1 in passes:
        pass1(tiles, a.out, a.workers)
    if 2 in passes:
        pass2(a.out)
    if 3 in passes:
        if a.tiles:
            want = set(a.tiles.split(","))
            tiles = [t for t in tiles if "%d_%d" % tile_index(t) in want]
        pass3(tiles, a.out, a.workers)


if __name__ == "__main__":
    main()
