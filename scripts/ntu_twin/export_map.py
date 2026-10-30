#!/usr/bin/env python3
"""The NTU twin's Autoware map dir (roadmap 059 N7, first version).

    export_map.py TILES_DIR ROUTES_DIR MAP_DIR [--voxel 0.2] [--workers 4]

Writes into MAP_DIR:

- `pointcloud_map/tile_<ix>_<iy>.pcd` and `pointcloud_map_metadata.yaml`: Autoware's divided
  point-cloud map (dynamic loading), one 50 m tile per file, x y z intensity (float32),
  thinned to one point per `--voxel` cube (the mean of its points, as PCL's VoxelGrid), from the
  full scan in the map frame: the vendor's MGRS-local coordinates less the vendor-to-map
  shift (019), the same frame and shift as build_meshes.py, so NDT's map and the world the
  RTX lidar sees are the same geometry.
- `lanelet2_map.osm`: routes r01 and r02 (ROUTES_DIR/r01, r02) merged. They do not overlap
  (r02 south-west, r01 north-east, ~90 m apart) but their ids do, so r01's are offset by
  `ID_OFFSET`. Their nodes stay where they were drawn: on the routes' own PCDs, which drift up
  to 0.3 m against the scan (019).
- `map_projector_info.yaml`: r02's (MGRS 51RUH).
"""
import argparse
import glob
import os
import re
import shutil
import time
import xml.etree.ElementTree as ET
from multiprocessing import Pool

import numpy as np

SHIFT = np.array([-0.54, 0.20, 17.30])  # build_meshes.py's
TILE = 50.0
ID_OFFSET = 1_000_000
CHUNK = 10_000_000  # points read at a time


def tile_index(path):
    m = re.search(r"tile_(-?\d+)_(-?\d+)\.laz$", path)
    return int(m.group(1)), int(m.group(2))


def _sums(k, V):
    """Unique keys and the column sums of V per key."""
    order = np.argsort(k, kind="stable")
    ks = k[order]
    starts = np.r_[0, np.nonzero(np.diff(ks))[0] + 1]
    return ks[starts], np.add.reduceat(V[order], starts)


def _pcd(args):
    path, out, voxel = args
    import laspy

    ix, iy = tile_index(path)
    # in chunks (a dense tile is ~180 M points): per voxel the sums of x y z intensity and
    # the count, merged over the chunks, then the means
    parts = []
    n_in = 0
    with laspy.open(path) as reader:
        for las in reader.chunk_iterator(CHUNK):
            P = np.stack([np.asarray(las.x), np.asarray(las.y), np.asarray(las.z)], 1) - SHIFT
            n_in += len(P)
            q = np.floor((P - np.array([ix * TILE - 10.0, iy * TILE - 10.0, -1000.0])) / voxel)
            k = (q[:, 0].astype(np.int64) * 1_000_000 + q[:, 1].astype(np.int64)) * 1_000_000 \
                + q[:, 2].astype(np.int64)
            parts.append(_sums(k, np.c_[P, np.asarray(las.intensity, np.float64), np.ones(len(P))]))
    k, S = _sums(np.concatenate([p[0] for p in parts]), np.concatenate([p[1] for p in parts]))
    M = S[:, :4] / S[:, 4:5]
    rec = np.zeros(len(M), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    rec["x"], rec["y"], rec["z"], rec["intensity"] = M[:, 0], M[:, 1], M[:, 2], M[:, 3]
    name = f"tile_{ix}_{iy}.pcd"
    with open(os.path.join(out, "pointcloud_map", name), "wb") as f:
        f.write(("# .PCD v0.7 - scripts/ntu_twin/export_map.py (roadmap 059)\nVERSION 0.7\n"
                 "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
                 f"WIDTH {len(rec)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(rec)}\n"
                 "DATA binary\n").encode())
        f.write(rec.tobytes())
    return name, ix, iy, len(rec), n_in


def pointcloud(tiles, out, voxel, workers):
    os.makedirs(os.path.join(out, "pointcloud_map"), exist_ok=True)
    meta = [f"x_resolution: {TILE}", f"y_resolution: {TILE}"]
    total = 0
    t0 = time.time()
    with Pool(workers, maxtasksperchild=1) as pool:
        for k, (name, ix, iy, n, n_in) in enumerate(pool.imap_unordered(_pcd, [(t, out, voxel) for t in tiles])):
            # the tile's lower corner in the map frame, floored: Autoware 1.5 reads the corners
            # as integers ("yaml-cpp: bad conversion" otherwise) and uses them for loading only;
            # the tiles are cut on the vendor grid, so the shift moves them under a metre
            meta.append(f"{name}: [{int(np.floor(ix * TILE - SHIFT[0]))}, {int(np.floor(iy * TILE - SHIFT[1]))}]")
            total += n
            print(f"pcd {k + 1}/{len(tiles)} {name}: {n:,} of {n_in:,} points, {time.time() - t0:.0f}s",
                  flush=True)
    with open(os.path.join(out, "pointcloud_map_metadata.yaml"), "w") as f:
        f.write("\n".join(meta) + "\n")
    print(f"pcd: {total:,} points in {len(tiles)} tiles")


def lanelets(routes, out):
    merged = ET.Element("osm", {"version": "0.6", "generator": "scripts/ntu_twin/export_map.py"})
    counts = {}
    for r, off in (("r02", 0), ("r01", ID_OFFSET)):
        root = ET.parse(os.path.join(routes, r, "lanelet2_map.osm")).getroot()
        n = 0
        for e in root:
            if e.tag not in ("node", "way", "relation"):
                continue
            e.set("id", str(int(e.get("id")) + off))
            for nd in e.findall("nd"):
                nd.set("ref", str(int(nd.get("ref")) + off))
            for m in e.findall("member"):
                m.set("ref", str(int(m.get("ref")) + off))
            merged.append(e)
            n += 1
        counts[r] = n
    order = {"node": 0, "way": 1, "relation": 2}
    merged[:] = sorted(merged, key=lambda e: order[e.tag])
    ET.ElementTree(merged).write(os.path.join(out, "lanelet2_map.osm"), encoding="UTF-8",
                                 xml_declaration=True)
    print(f"lanelets: merged {counts}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tiles_dir")
    ap.add_argument("routes_dir")
    ap.add_argument("map_dir")
    ap.add_argument("--voxel", type=float, default=0.2)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--skip-pcd", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.map_dir, exist_ok=True)
    lanelets(a.routes_dir, a.map_dir)
    shutil.copyfile(os.path.join(a.routes_dir, "r02", "map_projector_info.yaml"),
                    os.path.join(a.map_dir, "map_projector_info.yaml"))
    if not a.skip_pcd:
        pointcloud(sorted(glob.glob(os.path.join(a.tiles_dir, "tile_*.laz"))), a.map_dir, a.voxel,
                   a.workers)


if __name__ == "__main__":
    main()
