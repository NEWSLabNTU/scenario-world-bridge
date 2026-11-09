#!/usr/bin/env python3
"""What a small car will meet on a built world's lanes, from the ground the pipeline wrote.

    world_report.py GROUND_DIR MESH_DIR LANELET2.osm [--margin 0.25] [--json OUT]

Under the lanelets (grown by `--margin`), on the DTM the ground mesh is built from:
- `lane_cells_meshed`: the share of lane cells build_meshes.py meshed (ground_keep.npy): below
  1 a car can fall through.
- `roughness_p99`: p99 of |z - its 3x3 median|, the bumps a wheel rides over. NTU reached 0.01 m
  after smooth_dtm.py; a 5 cm wheel (AutoSDV's car A twin) wants no more.
- `pit_depth_max`: the deepest cell below its 9x9 neighbourhood's median: drains and grates
  the scan saw through.
Run under the site the world was built for (SWB_TWIN_SITE): the lanelets' frame is the site's.
Exit status 1 when a lane cell is unmeshed.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_usd import lane_raster  # noqa: E402


def report(ground_dir, mesh_dir, osm, margin=0.25):
    from scipy import ndimage

    meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
    dtm = np.load(os.path.join(ground_dir, "dtm.npy"))
    keep = np.load(os.path.join(mesh_dir, "ground_keep.npy"))
    lane = lane_raster(osm, meta, dtm.shape, margin)
    rough = np.abs(dtm - ndimage.median_filter(dtm, size=3))[lane]
    pits = (ndimage.median_filter(dtm, size=9) - dtm)[lane]
    return {
        "lane_cells": int(lane.sum()),
        "lane_cells_meshed": float(keep[lane].mean()) if lane.any() else 0.0,
        "roughness_p99": float(np.percentile(rough, 99)) if lane.any() else 0.0,
        "pit_depth_max": float(pits.max()) if lane.any() else 0.0,
        "cell": meta["cell"],
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ground_dir")
    ap.add_argument("mesh_dir")
    ap.add_argument("osm")
    ap.add_argument("--margin", type=float, default=0.25)
    ap.add_argument("--json")
    a = ap.parse_args()
    r = report(a.ground_dir, a.mesh_dir, a.osm, a.margin)
    print(json.dumps(r, indent=1))
    if a.json:
        json.dump(r, open(a.json, "w"), indent=1)
    sys.exit(0 if r["lane_cells_meshed"] >= 1.0 else 1)


if __name__ == "__main__":
    main()
