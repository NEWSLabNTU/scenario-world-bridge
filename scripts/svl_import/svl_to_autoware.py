#!/usr/bin/env python3
"""An Autoware map dir from an SVL map: its Lanelet2 map, re-projected, and a point-cloud
map sampled from its 3D scene.

    svl_to_autoware.py DUMP_DIR LANELET2_OSM MAP_DIR [--spacing 0.1]

DUMP_DIR is unity_dump.py's output, LANELET2_OSM the map asset's `hdmaps/lanelet2/*.osm`.
Writes into MAP_DIR:

- `lanelet2_map.osm`: SVL's export with every node's `lat`/`lon` recomputed from its `x`/`y`
  tags (the map's local ENU, metres from the UTM origin): SVL's own conversion is up to
  0.4 m off a proper UTM inverse. `local_x`/`local_y` are added in the MGRS 100 km square's
  frame, `ele` kept. Two Autoware conventions applied: stop signs, which SVL writes as
  `usR1-1` (the MUTCD code), become `stop_sign`, as Autoware's stop-line module looks for;
  and every road lanelet without a `speed_limit` gets `--speed-limit` (km/h).
- `map_projector_info.yaml`: MGRS, the origin's grid square.
- `pointcloud_map.pcd`: every mesh's surface sampled uniformly (one point per `--spacing`
  squared, then one per 0.1 m voxel), x y z intensity (float32) in the MGRS-local frame, so
  NDT's map and the world the RTX lidar sees are the same geometry. Intensity by material:
  lane paint high, asphalt low (a rough stand-in for real reflectivity).

The frame is svl_to_usd.py's (`unity_to_map`, `mgrs_local`); the Isaac world's
`map_origin` is the MGRS-local origin, so map less map_origin is the stage.
"""
import argparse
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from svl_to_usd import label_of, mgrs_local, unity_to_map  # noqa: E402

INTENSITY = {"Q1392287": 200.0, "Q34442": 20.0, "Q177749": 60.0, "Q186131": 40.0, "Q41176": 80.0}


def mgrs_square(zone, easting, northing, lat):
    """The MGRS grid zone and 100 km square ('51QTF') of a UTM position."""
    bands = "CDEFGHJKLMNPQRSTUVWX"
    band = bands[min(int((lat + 80) // 8), 19)]
    cols = ["ABCDEFGH", "JKLMNPQR", "STUVWXYZ"][(zone - 1) % 3]
    rows = "ABCDEFGHJKLMNPQRSTUV"
    col = cols[int(easting // 100_000) - 1]
    row = rows[(int(northing // 100_000) + (5 if zone % 2 == 0 else 0)) % 20]
    return f"{zone}{band}{col}{row}"


def sample(V, F, spacing, rng):
    """Points spread uniformly over the triangles, about one per spacing^2."""
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
    n = rng.poisson(area / spacing ** 2)
    idx = np.repeat(np.arange(len(F)), n)
    u, v = rng.random(len(idx)), rng.random(len(idx))
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return A[idx] + u[:, None] * (B - A)[idx] + v[:, None] * (C - A)[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("osm")
    ap.add_argument("map_dir")
    ap.add_argument("--spacing", type=float, default=0.1)
    ap.add_argument("--voxel", type=float, default=0.1)
    ap.add_argument("--speed-limit", type=float, default=30.0, help="km/h, for lanelets without one")
    a = ap.parse_args()
    from pyproj import Transformer

    scene = json.load(open(os.path.join(a.dump, "scene.json")))
    org = scene["map_origin"]
    zone = int(org["UTMZoneId"])
    e0, n0 = float(org["OriginEasting"]), float(org["OriginNorthing"])
    ox, oy = mgrs_local(e0, n0)
    os.makedirs(a.map_dir, exist_ok=True)

    # Lanelet2: lat/lon from the x/y tags, MGRS-local local_x/local_y
    inv = Transformer.from_crs(f"EPSG:{32600 + zone}", "EPSG:4326", always_xy=True)
    tree = ET.parse(a.osm)
    root = tree.getroot()
    n_nodes = 0
    lat0 = None
    for node in root.iter("node"):
        tags = {t.get("k"): t for t in node.findall("tag")}
        x, y = float(tags["x"].get("v")), float(tags["y"].get("v"))
        lon, lat = inv.transform(e0 + x, n0 + y)
        node.set("lat", f"{lat:.11f}")
        node.set("lon", f"{lon:.11f}")
        lat0 = lat if lat0 is None else lat0
        for k, v in (("local_x", ox + x), ("local_y", oy + y)):
            if k in tags:
                tags[k].set("v", f"{v:.4f}")
            else:
                ET.SubElement(node, "tag", {"k": k, "v": f"{v:.4f}"})
        if "ele" not in tags:
            ET.SubElement(node, "tag", {"k": "ele", "v": "0"})
        n_nodes += 1
    for way in root.iter("way"):
        for t in way.findall("tag"):
            if t.get("k") == "subtype" and t.get("v") == "usR1-1":
                t.set("v", "stop_sign")
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") == "lanelet" and tags.get("subtype") == "road" and "speed_limit" not in tags:
            ET.SubElement(rel, "tag", {"k": "speed_limit", "v": f"{a.speed_limit:g}"})
    tree.write(os.path.join(a.map_dir, "lanelet2_map.osm"), encoding="UTF-8", xml_declaration=True)
    grid = mgrs_square(zone, e0, n0, lat0)
    with open(os.path.join(a.map_dir, "map_projector_info.yaml"), "w") as f:
        f.write(f"projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: {grid}\n")

    # point cloud from the meshes
    yaw = scene.get("map_origin_yaw_deg", -90.0)
    T = unity_to_map(yaw)
    rng = np.random.default_rng(0)
    pts, inten = [], []
    for inst in scene["instances"]:
        z = np.load(os.path.join(a.dump, "meshes", f"{inst['mesh']}.npz"))
        W = T @ np.array(inst["world"])
        V = (W[:3, :3] @ z["positions"].astype(np.float64).T).T + W[:3, 3]
        F = np.concatenate([z[k] for k in z.files if k.startswith("sub")] or [np.zeros((0, 3), int)])
        if not len(F):
            continue
        P = sample(V, F, a.spacing, rng)
        pts.append(P)
        inten.append(np.full(len(P), INTENSITY.get(label_of(inst["materials"]), 50.0)))
    P = np.concatenate(pts)
    I = np.concatenate(inten)
    key = np.floor((P - P.min(0)) / a.voxel).astype(np.int64)
    k = (key[:, 0] * 100_000 + key[:, 1]) * 100_000 + key[:, 2]
    _, first = np.unique(k, return_index=True)
    P, I = P[first], I[first]
    rec = np.zeros(len(P), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    rec["x"], rec["y"], rec["z"], rec["intensity"] = P[:, 0] + ox, P[:, 1] + oy, P[:, 2], I
    with open(os.path.join(a.map_dir, "pointcloud_map.pcd"), "wb") as f:
        f.write(("# .PCD v0.7 - scripts/svl_import/svl_to_autoware.py\nVERSION 0.7\n"
                 "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
                 f"WIDTH {len(rec)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(rec)}\n"
                 "DATA binary\n").encode())
        f.write(rec.tobytes())
    print(f"lanelet2: {n_nodes} nodes re-projected; MGRS {grid}, origin local ({ox:.1f}, {oy:.1f}); "
          f"pcd: {len(rec):,} points")


if __name__ == "__main__":
    main()
