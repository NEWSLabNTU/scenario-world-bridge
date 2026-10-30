#!/usr/bin/env python3
"""Re-derive the DTM's non-ground cells from its ground cells alone (roadmap 059 N3/N8).

    smooth_dtm.py GROUND_DIR [--lanes MAP_DIR/lanelet2_map.osm]

ground_ortho.py's DTM keeps ground cells' heights and fills the rest, and that fill takes
heights from non-ground points (parked scooters, kerbs of planters, a stopped car): on r01 it
left 0.2-0.3 m bumps on the road where the scan has no points at all (most cells there are
empty), and a car at creep torque could not climb them (an Autoware drive stood still with
acceleration commanded). Only 17.8 % of the site's cells are ground.

A non-ground cell is capped at the ground's own height there plus CAP: the normalized
Gaussian average of the ground cells around it,
at sigma 0.5, 1, 2, 4, 8, 16, 32 m in turn, the first whose ground weight reaches WEIGHT, and
at 64 m with any ground at all. Capped, not replaced: replacing every non-ground cell made the
road rougher (single noisy ground cells stood out against smooth neighbours; |z - 3x3 median|
p99 on the roads 0.083 m against the raw 0.058), while the cap only takes out what stands above
the ground (the bumps, and the fill's roof heights inside building footprints, up to 26 m off).
On and beside the lanes (--lanes), where the scan has returns, the surface is the scan's
lowest return (zmin, GROUND_DIR/zmin.npy) taken as a ZMIN_MEDIAN x ZMIN_MEDIAN median, unless
that stands more than CANOPY above the window's lower quarter (a cell whose lowest return is
a crown). Along r01's last lanelet the road runs above a drop: SMRF's 16 m opening took the
raised road for an object, no cell there was ground, and the fill put the road 6 m down
(zmin was 11.84-11.89 in every cell, the raw DTM 5.8-7.3); the car braked for its own road,
slid towards the drop and NDT lost it (N8). Then narrow pits close: a cell more than PIT_DEPTH below its PIT_WINDOW x PIT_WINDOW median
is raised to that median less PIT_DEPTH. Through a drain grate at r01's road edge the scanner
saw the pit under it, ground-classified 2 m below the road; the right wheels dropped in, the
car slid sideways, and the EKF, rejecting NDT's poses, ended the drive in an MRM (N8). That
pit was wider than the window: on and beside the lanes (--lanes, LANE_MARGIN around the
lanelets), where a pit is always wrong, the closing repeats with windows of LANE_WINDOWS.
Elsewhere one window keeps kerbs and ditches. Then
the whole grid, ground cells too, takes a MEDIAN x MEDIAN median and a Gaussian of
SIGMA_CELLS: the scan's ground cells carry single-cell steps of 0.1-0.2 m (sparse points, the
SMRF band), and one stopped a car at creep torque on r01 after the cap. Kerbs round off over
about a metre. The original is kept as dtm_raw.npy (and read from it on a rerun);
dtm.npy is rewritten. Then: build_meshes.py --ground-only, world_usd.py, register_lanelets.py.
"""
import json
import os
import sys

import numpy as np
from scipy import ndimage

SIGMAS_M = (0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0)
WEIGHT = 0.05  # of a full neighbourhood of ground (17.8 % of the site is ground)
CAP = 0.05     # m above the ground-only estimate
PIT_WINDOW = 9  # cells (2.25 m)
PIT_DEPTH = 0.15  # m
LANE_WINDOWS = (9, 17, 33)  # cells
LANE_MARGIN = 3.0  # m
ZMIN_MEDIAN = 5  # cells
CANOPY = 0.3  # m above the window's 25th percentile of zmin
MEDIAN = 5     # cells (1.25 m)
SIGMA_CELLS = 1.0


def smooth(dtm, ground, cell):
    out = dtm.copy()
    g = ground.astype(np.float32)
    z = np.where(ground, dtm, 0.0).astype(np.float32)
    todo = ~ground
    for s in SIGMAS_M:
        w = ndimage.gaussian_filter(g, s / cell, mode="nearest")
        v = ndimage.gaussian_filter(z, s / cell, mode="nearest")
        last = s == SIGMAS_M[-1]  # the widest takes any ground at all: no raw fill is left
        ok = todo & (w >= (1e-6 if last else WEIGHT))
        out[ok] = v[ok] / w[ok]
        todo &= ~ok
        print(f"sigma {s:4.1f} m: {ok.sum():,} cells, {todo.sum():,} left", flush=True)
    return out, todo


def main():
    import argparse

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ground_dir")
    ap.add_argument("--lanes", default="", help="a Lanelet2 map: close pits harder on its lanes")
    a = ap.parse_args()
    d = a.ground_dir
    raw = os.path.join(d, "dtm_raw.npy")
    if not os.path.exists(raw):
        os.replace(os.path.join(d, "dtm.npy"), raw)
    dtm = np.load(raw)
    ground = np.load(os.path.join(d, "ground_cells.npy"))
    cell = json.load(open(os.path.join(d, "dtm.json")))["cell"]
    lanes = None
    if a.lanes:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from world_usd import lane_raster

        lanes = lane_raster(a.lanes, json.load(open(os.path.join(d, "dtm.json"))), dtm.shape,
                            LANE_MARGIN)
    est, left = smooth(dtm, ground, cell)  # cells still left keep the raw fill
    out = np.where(ground, dtm, np.minimum(dtm, est + CAP))
    if lanes is not None:
        zmin = np.load(os.path.join(d, "zmin.npy"))
        seen = np.isfinite(zmin)
        zf = np.where(seen, zmin, 1e6).astype(np.float64)
        zm = ndimage.median_filter(zf, ZMIN_MEDIAN)
        low = ndimage.percentile_filter(zf, 25, size=PIT_WINDOW)
        use = lanes & seen & (zm < 1e5) & (zm <= low + CANOPY)
        out = np.where(use, zm, out)
        print(f"lanes: {int(use.sum()):,} of {int(lanes.sum()):,} cells from the scan's lowest return")
    out = np.maximum(out, ndimage.median_filter(out, PIT_WINDOW) - PIT_DEPTH)
    if lanes is not None:
        for w in LANE_WINDOWS:
            out = np.where(lanes, np.maximum(out, ndimage.median_filter(out, w) - PIT_DEPTH), out)
    out = ndimage.gaussian_filter(ndimage.median_filter(out, MEDIAN), SIGMA_CELLS)
    np.save(os.path.join(d, "dtm.npy"), out.astype(dtm.dtype))
    for name, sel in (("ground", ground), ("non-ground", ~ground)):
        diff = np.abs(out - dtm)[sel]
        print(f"{name} cells changed: p50 {np.median(diff):.3f} m, p95 "
              f"{np.percentile(diff, 95):.3f} m, max {diff.max():.2f} m")
    print(f"{left.sum():,} cells without ground near")
    return 0


if __name__ == "__main__":
    sys.exit(main())
