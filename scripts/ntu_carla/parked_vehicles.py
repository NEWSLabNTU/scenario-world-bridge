#!/usr/bin/env python3
"""Roadmap 072: where the scan holds parked vehicles, as a mask on the DTM's grid, so
`carla_meshes.py --parked` can leave them out of the CARLA map.

    parked_vehicles.py MESH_DIR MAP_DIR GROUND_DIR TREES_DIR OUT.npz [--preview DIR]

The scan's parked cars and scooter rows are coarse one-sided lumps (072: the grey planes along
the road) and were never part of the place: CARLA puts its own vehicles where the scenario
asks. They are found on the props, the scan's `other` meshes off the lanes and outside the
trees (as `carla_meshes.py` splits them):
- each 25 cm cell keeps the highest prop face over the DTM, the face count and the mean colour;
- the cells whose props top out between LOW and HIGH over the ground, with nothing taller
  next to them (a wall, a kiosk), are closed by two cells and labelled into blobs;
- a blob is a vehicle, a row of them nose to tail or a block of scooters, when its width (area
  over its long axis) is in WIDTH, its top in TOP, it is not green (hedges), it fills SOLID of
  its convex hull, it covers at least MIN_AREA and it comes within NEAR_LANE of a lane.
OUT.npz holds `mask` (bool, the DTM's grid, rows by y like dtm.npy, one cell of margin), and
`blobs` (label, area m2, length m, width m, top m, green fraction, distance to a lane m,
solidity).
`--preview` writes the orthophoto beside the mask, for a look.
"""
import argparse
import json
import os
import sys
from multiprocessing import Pool

import cv2
import numpy as np
from scipy import ndimage

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ntu_twin"))
import world_usd as W  # noqa: E402

LOW, HIGH = 0.5, 2.2  # m over the DTM: a cell's highest prop face, for a candidate cell
TALL = 2.6  # m: a cell this tall next to a candidate is a structure, not a vehicle
WIDTH = (0.9, 8.0)  # m, area over length: one car across, up to a scooter block a few rows deep
TOP = (0.9, 2.5)  # m: a blob's highest face (a mirror, a rider's helmet on a seat)
GREEN = 0.5  # at most this fraction of green cells (G 8 % over R and B): hedges are over 0.6
SOLID = 0.4  # at least this share of its convex hull: a wall's L or a frame is not a vehicle
MIN_AREA = 1.5  # m2
NEAR_LANE = 8.0  # m
_ctx = {}


def _init(a):
    _ctx["road"] = W.RoadClear(a["osm"], a["ground"], 0.5)
    _ctx["trees"] = W.Trees(a["trees"], a["ground"])
    _ctx["meta"] = json.load(open(os.path.join(a["ground"], "dtm.json")))


def _cells(path):
    m = _ctx["meta"]
    V, F, C = W.read_ply(path)
    P = V[F].mean(axis=1)
    off = ~_ctx["road"].on_road(P)
    P, Cf = P[off], (C[F[off]].mean(axis=1) if C is not None else np.full((off.sum(), 3), 0.5))
    tr = _ctx["trees"].is_tree(P)
    P, Cf = P[~tr], Cf[~tr]
    h = _ctx["trees"].height(P)
    v = P + W.SHIFT
    i = ((v[:, 0] - m["x0"]) / m["cell"]).astype(int)
    j = ((v[:, 1] - m["y0"]) / m["cell"]).astype(int)
    ok = (i >= 0) & (i < m["width"]) & (j >= 0) & (j < m["height"])
    return i[ok], j[ok], h[ok].astype(np.float32), Cf[ok].astype(np.float32)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for k in ("mesh_dir", "map_dir", "ground_dir", "trees_dir", "out"):
        ap.add_argument(k)
    ap.add_argument("--preview", default="")
    ap.add_argument("--workers", type=int, default=6)
    a = ap.parse_args(argv)
    osm = os.path.join(a.map_dir, "lanelet2_map.osm")
    meta = json.load(open(os.path.join(a.ground_dir, "dtm.json")))
    H, Wd, cell = meta["height"], meta["width"], meta["cell"]
    htop = np.full((H, Wd), -9.0, np.float32)
    n = np.zeros((H, Wd), np.int32)
    col = np.zeros((H, Wd, 3), np.float32)
    files = sorted(os.path.join(a.mesh_dir, f) for f in os.listdir(a.mesh_dir) if f.startswith("other_"))
    ctx = {"osm": osm, "ground": a.ground_dir, "trees": a.trees_dir}
    with Pool(a.workers, initializer=_init, initargs=(ctx,)) as pool:
        for i, j, h, Cf in pool.imap_unordered(_cells, files):
            np.maximum.at(htop, (j, i), h)
            np.add.at(n, (j, i), 1)
            np.add.at(col, (j, i), Cf)
    occ = n > 0
    rgb = col / np.maximum(n, 1)[..., None]
    green = (rgb[..., 1] > rgb[..., 0] * 1.08) & (rgb[..., 1] > rgb[..., 2] * 1.08)
    low = occ & (htop > LOW) & (htop < HIGH)
    low = ndimage.binary_closing(low, iterations=2) & ~ndimage.binary_dilation(occ & (htop >= TALL), iterations=1)
    lab, k = ndimage.label(low)
    idx = np.arange(1, k + 1)
    area = ndimage.sum(np.ones_like(lab), lab, idx) * cell * cell
    gfrac = ndimage.mean(green, lab, idx)
    top = ndimage.maximum(np.where(occ, htop, 0.0), lab, idx)
    lane = W.RoadClear(osm, a.ground_dir, 0.25).lane
    dmin = ndimage.minimum(ndimage.distance_transform_edt(~lane) * cell, lab, idx)
    rows = []
    for t, sl in enumerate(ndimage.find_objects(lab)):
        if area[t] < MIN_AREA:
            continue
        jj, ii = np.nonzero(lab[sl] == t + 1)
        P = np.c_[ii, jj] * cell
        P = P - P.mean(axis=0)
        _, evec = np.linalg.eigh(np.cov(P.T))
        length = max(np.ptp(P @ evec[:, 1]), cell)
        hull = cv2.convexHull(np.c_[ii, jj].astype(np.int32))
        solid = len(ii) / max(cv2.contourArea(hull) + 0.5 * cv2.arcLength(hull, True) + 1.0, 1.0)
        rows.append((t + 1, area[t], length, area[t] / length, top[t], gfrac[t], dmin[t], solid))
    B = np.array(rows).reshape(-1, 8)
    veh = ((B[:, 3] > WIDTH[0]) & (B[:, 3] < WIDTH[1]) & (B[:, 4] > TOP[0]) & (B[:, 4] < TOP[1])
           & (B[:, 5] < GREEN) & (B[:, 6] < NEAR_LANE) & (B[:, 7] > SOLID))
    mask = ndimage.binary_dilation(np.isin(lab, B[veh, 0].astype(int)), iterations=1)
    np.savez_compressed(a.out, mask=mask, blobs=B[veh].astype(np.float32))
    print(f"{veh.sum()} vehicle blobs of {len(B)} low ones, {B[veh, 1].sum():.0f} m2; mask {mask.sum()} cells -> {a.out}")
    if a.preview:
        os.makedirs(a.preview, exist_ok=True)
        ortho = cv2.imread(os.path.join(a.ground_dir, "site_ortho_25cm.jpg"))
        over = ortho.copy()
        mf = np.flipud(mask)
        over[mf] = (0.4 * over[mf] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
        cv2.imwrite(os.path.join(a.preview, "parked_site.jpg"),
                    cv2.resize(np.hstack([ortho, over]), None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
    return 0


if __name__ == "__main__":
    sys.exit(main())
