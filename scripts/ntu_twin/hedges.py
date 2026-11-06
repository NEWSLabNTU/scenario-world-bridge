#!/usr/bin/env python3
"""Roadmap 072, map revision stage S4 (docs/design/ntu-twin-map-revisions.md): the scan's hedges
and low bushes as a smooth height field on the DTM's grid, so each world's export can put it in
place of their crumpled faces (`carla_meshes.py --hedges`).

    hedges.py MESH_DIR MAP_DIR GROUND_DIR TREES_DIR OUT.npz [--parked PARKED.npz] [--preview DIR]
        [--workers 6]

The scan's vegetation is a crumpled sheet: adjacent faces of a hedge meet at 43 deg at the
median, and neither smooth normals nor Taubin smoothing (40 iterations: 29 deg, back to 43 after
the decimation) take the faceting out. A hedge is a 2.5-D thing, though: a top over the ground.
So it is rebuilt as one, as the ground is:
- the low vegetation faces are the `other` meshes' off the road (as `carla_meshes.py` splits
  them) under LOW_TOP over the DTM that are a tree's (`Trees.is_tree`) or green (`is_green`,
  G over R and B by GREEN_RATIO; 072's parked vehicles: hedges are over 0.6 green);
- a 25 cm cell is a hedge's when it holds MIN_FACES of them, VEG_SHARE of its low faces are
  vegetation, its top is over MIN_TOP and it is further than TRUNK_CLEAR from a trunk's
  surface (a trunk's foot is a tree's face too); closed by two cells (the scan's gaps), opened
  by one, blobs under MIN_AREA dropped; never under `--parked`'s mask (`parked_vehicles.py`:
  a scooter row's greenish faces are not a hedge, and the vehicles are left out);
- the top: the cell's highest vegetation face (the nearest one's, in a gap the closing filled),
  median-filtered over 3x3 and Gaussian-smoothed (SIGMA cells) with nothing outside the mask,
  so the sides fall to the ground over a few cells, none on a lane (LANE_CLEAR); the mask is
  then where the top is over EDGE.
  The colour: the cells' mean face colour (sRGB), smoothed within the mask.
OUT.npz holds `mask` (bool, the DTM's grid, rows by y like dtm.npy), `top` (float16, m over the
DTM, 0 outside), `colour` (uint8 sRGB).
"""
import argparse
import json
import os
import sys
from multiprocessing import Pool

import cv2
import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_usd as W  # noqa: E402

LOW_TOP = 2.6  # m over the DTM: a hedge's faces are below (parked_vehicles.TALL)
MIN_TOP = 0.25  # m: a cell's vegetation must stand this high (not grass on the terrain)
GREEN_RATIO = 1.08  # G over R and B (sRGB), as parked_vehicles' green cells
MIN_FACES = 2
VEG_SHARE = 0.6
TRUNK_CLEAR = 0.4  # m from a trunk's surface
MIN_AREA = 0.75  # m2
SIGMA = 1.2  # cells
EDGE = 0.05  # m: the smoothed top's edge, where it meets the ground
# m around the lanelets with no hedge: the top's spread put 0.9 m of hedge into lanelet 1008385
# at its end, (103, 328), and the ego stopped 9 m short of tour leg 24's goal (072); 202 cells
# of the site's hedges stood on a lane. carla_meshes' road margin (ROAD_MARGIN) is the same.
LANE_CLEAR = 0.25
_ctx = {}


def is_green(Cf):
    """Faces' mean colours (sRGB, 0..1): green ones."""
    return (Cf[:, 1] > Cf[:, 0] * GREEN_RATIO) & (Cf[:, 1] > Cf[:, 2] * GREEN_RATIO)


def low_vegetation(V, F, C, trees):
    """Of the faces F (already off the road), which are low vegetation, and their heights over
    the DTM: a tree's or green, under LOW_TOP."""
    P = V[F].mean(axis=1)
    h = trees.height(P)
    Cf = C[F].mean(axis=1) if C is not None else np.full((len(F), 3), 0.5)
    return (h < LOW_TOP) & (trees.is_tree(P) | is_green(Cf)), h, Cf


def _init(a):
    _ctx["clear"] = W.RoadClear(a["osm"], a["ground"], 0.5)
    _ctx["trees"] = W.Trees(a["trees"], a["ground"])
    _ctx["meta"] = json.load(open(os.path.join(a["ground"], "dtm.json")))


def _cells(path):
    m = _ctx["meta"]
    V, F, C = W.read_ply(path)
    F = F[~_ctx["clear"].on_road(V[F].mean(axis=1))]
    veg, h, Cf = low_vegetation(V, F, C, _ctx["trees"])
    low = h < LOW_TOP
    v = V[F].mean(axis=1) + W.SHIFT
    i = ((v[:, 0] - m["x0"]) / m["cell"]).astype(int)
    j = ((v[:, 1] - m["y0"]) / m["cell"]).astype(int)
    ok = low & (i >= 0) & (i < m["width"]) & (j >= 0) & (j < m["height"])
    return i[ok], j[ok], veg[ok], h[ok].astype(np.float32), Cf[ok].astype(np.float32)


def smooth_within(x, mask, sigma):
    """Gaussian smoothing of x (H x W or H x W x k) over the mask's cells only."""
    w = ndimage.gaussian_filter(mask.astype(np.float32), sigma)
    if x.ndim == 2:
        return np.where(mask, ndimage.gaussian_filter(np.where(mask, x, 0), sigma) / np.maximum(w, 1e-6), 0)
    return np.stack([smooth_within(x[..., k], mask, sigma) for k in range(x.shape[-1])], axis=-1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for k in ("mesh_dir", "map_dir", "ground_dir", "trees_dir", "out"):
        ap.add_argument(k)
    ap.add_argument("--parked", default="", help="parked_vehicles.py's mask: no hedge there")
    ap.add_argument("--preview", default="")
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args(argv)
    osm = os.path.join(a.map_dir, "lanelet2_map.osm")
    meta = json.load(open(os.path.join(a.ground_dir, "dtm.json")))
    H, Wd, cell = meta["height"], meta["width"], meta["cell"]
    top = np.zeros((H, Wd), np.float32)
    nveg = np.zeros((H, Wd), np.int32)
    nlow = np.zeros((H, Wd), np.int32)
    col = np.zeros((H, Wd, 3), np.float32)
    files = sorted(os.path.join(a.mesh_dir, f) for f in os.listdir(a.mesh_dir) if f.startswith("other_"))
    ctx = {"osm": osm, "ground": a.ground_dir, "trees": a.trees_dir}
    with Pool(a.workers, initializer=_init, initargs=(ctx,)) as pool:
        for i, j, veg, h, Cf in pool.imap_unordered(_cells, files):
            np.add.at(nlow, (j, i), 1)
            iv, jv = i[veg], j[veg]
            np.add.at(nveg, (jv, iv), 1)
            np.maximum.at(top, (jv, iv), h[veg])
            np.add.at(col, (jv, iv), Cf[veg])
    mask = (nveg >= MIN_FACES) & (nveg >= VEG_SHARE * nlow) & (top > MIN_TOP)
    trees = W.Trees(a.trees_dir, a.ground_dir)
    if len(trees.trunks):
        # the cells within TRUNK_CLEAR of a trunk's surface
        near = np.zeros((H, Wd), np.uint8)
        v = trees.trunks[:, :2] + W.SHIFT[:2]
        for (x, y), r in zip(v, trees.trunks[:, 3]):
            cv2.circle(near, (int((x - meta["x0"]) / cell), int((y - meta["y0"]) / cell)),
                       int(np.ceil((r + TRUNK_CLEAR) / cell)), 1, -1)
    else:
        near = np.zeros((H, Wd), np.uint8)
    parked = np.load(a.parked)["mask"] if a.parked else np.zeros((H, Wd), bool)
    mask = ndimage.binary_closing(mask & ~parked, iterations=2) & (near == 0) & (nlow > 0) & ~parked
    mask = ndimage.binary_opening(mask, iterations=1)
    lab, k = ndimage.label(mask)
    area = ndimage.sum(np.ones_like(lab), lab, np.arange(1, k + 1)) * cell * cell
    mask = np.isin(lab, np.flatnonzero(area >= MIN_AREA) + 1)
    # the closing's cells take the nearest vegetation cell's top and colour
    _, (jj, ii) = ndimage.distance_transform_edt(nveg == 0, return_indices=True)
    t = ndimage.median_filter(np.where(mask, top[jj, ii], 0), size=3)
    # unnormalised at the edge: the sides fall to the ground over a few cells, not a cell's step
    t = ndimage.gaussian_filter(t, SIGMA)
    # the closing and the blur carry a hedge's top past its faces: off the lanes (LANE_CLEAR)
    t[W.lane_raster(osm, meta, (H, Wd), LANE_CLEAR)] = 0
    mask = (t > EDGE) & ~parked
    rgb = smooth_within((col / np.maximum(nveg, 1)[..., None])[jj, ii], mask, SIGMA)
    np.savez_compressed(a.out, mask=mask, top=t.astype(np.float16),
                        colour=np.clip(rgb * 255, 0, 255).astype(np.uint8))
    print(f"{mask.sum()} hedge cells ({mask.sum() * cell * cell:.0f} m2) in {int((area >= MIN_AREA).sum())} blobs, "
          f"top p50 {np.median(t[mask]):.2f} m -> {a.out}")
    if a.preview:
        os.makedirs(a.preview, exist_ok=True)
        ortho = cv2.imread(os.path.join(a.ground_dir, "site_ortho_25cm.jpg"))
        over = ortho.copy()
        mf = np.flipud(mask)
        over[mf] = (0.4 * over[mf] + 0.6 * np.array([0, 255, 255])).astype(np.uint8)
        cv2.imwrite(os.path.join(a.preview, "hedges_site.jpg"),
                    cv2.resize(np.hstack([ortho, over]), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
    return 0


if __name__ == "__main__":
    sys.exit(main())
