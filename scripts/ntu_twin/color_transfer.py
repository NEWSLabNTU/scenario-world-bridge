#!/usr/bin/env python3
"""Colour a precise map cloud from a coarse, coloured copy of the same scan, and write it as
the pipeline's 50 m LAZ tiles (retile_laz.py's format), for a site whose survey arrives as a
PCD in the map frame rather than as a vendor LAZ.

    color_transfer.py --site coss MAP.pcd OUT_DIR ORIGINAL.pcd [ORIGINAL.pcd ...]
        [--north-tol 0.15] [--tight-tol 0.03] [--no-refine]

The first user is COSS (AutoSDV's car A twin, AutoSDV roadmap 15). Its map PCD
(4.9 M points, local TransverseMercator, mm-precise) carries no colour, while the survey's
originals (133 M points, real RGB and intensity) are float32 TWD97 (EPSG:3826): at a
northing of 2.77e6 a float32 steps 0.25 m, at an easting of 3.05e5 0.03 m. Their geometry is
unusable; their colour is not. So:

1. The originals are taken into the map frame as the site's `survey_frame` says
   (`survey_frame()`): projected through Autoware's TransverseMercator, or, as COSS's map was
   made, TWD97 less TWD97(map origin).
2. The remaining offset (survey to map registration, datum) is refined by a translation-only
   ICP: the median nearest-neighbour displacement, iterated.
3. Each map point takes the colour and intensity of the nearest original in an anisotropic
   metric: `north_tol` along the map's y (the quantized northing, up to +-0.125 m off) and
   `tight_tol` along x and z; with `--fallback`, a point with none in that reach takes the
   nearest original's within that many metres (COSS's map holds geometry the survey did not
   see: half its points are over 0.77 m from any original); beyond it, grey.

The tiles are in the map frame, so the site's SHIFT is 0. Every point is unclassified (the
survey has no classes): build_meshes.py meshes all of it as "other".
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import yaml

TILE = 50.0
OUT_SCALE = 0.001


class MapFrame:
    """Autoware's TransverseMercator map frame about (lat0, lon0)."""

    def __init__(self, lat0, lon0, k0=0.9996):
        from pyproj import Transformer

        tm = f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k={k0} +x_0=0 +y_0=0 +ellps=WGS84 +units=m"
        self._ll = Transformer.from_crs("EPSG:4326", tm, always_xy=True)
        self._twd = Transformer.from_crs("EPSG:3826", tm, always_xy=True)

    def from_latlon(self, lat, lon):
        return self._ll.transform(np.asarray(lon), np.asarray(lat))

    def from_twd97(self, e, n):
        return self._twd.transform(np.asarray(e, float), np.asarray(n, float))

    @classmethod
    def from_projector_info(cls, path):
        info = yaml.safe_load(open(path))
        if info.get("projector_type") != "TransverseMercator":
            raise SystemExit(f"{path}: projector_type {info.get('projector_type')!r}, not TransverseMercator")
        o = info["map_origin"]
        return cls(o["latitude"], o["longitude"])


def survey_frame(site):
    """(to_map(e, n), from_map(x, y)) between the survey's TWD97 and the site's map frame, by
    the site's `survey_frame`:
    - `projected`: Autoware's TransverseMercator about the map origin (MapFrame);
    - `twd97_minus_origin`: TWD97 less TWD97(map origin), no projection. COSS's map is this:
      its PCD lines up with the survey only so (registration, roadmap 15: the projected frame
      is 0.22 deg off, the meridian convergence), and its lanelets' lat/lon are the inverse TM
      of local_x/y less the same origin, to 2 cm, so Autoware's projector agrees with it."""
    from pyproj import Transformer

    info = yaml.safe_load(open(os.path.join(site["map_dir"], "map_projector_info.yaml")))
    o = info["map_origin"]
    mode = site.get("survey_frame", "projected")
    if mode == "projected":
        f = MapFrame(o["latitude"], o["longitude"])
        back = Transformer.from_crs(f._twd.target_crs, "EPSG:3826", always_xy=True)
        return f.from_twd97, lambda x, y: back.transform(np.asarray(x, float), np.asarray(y, float))
    if mode == "twd97_minus_origin":
        e0, n0 = Transformer.from_crs("EPSG:4326", "EPSG:3826", always_xy=True).transform(o["longitude"], o["latitude"])
        return (lambda e, n: (np.asarray(e, float) - e0, np.asarray(n, float) - n0),
                lambda x, y: (np.asarray(x, float) + e0, np.asarray(y, float) + n0))
    raise SystemExit(f"site {site['name']}: survey_frame {mode!r}: projected or twd97_minus_origin")


def refine_offset(src, dst, guess=None, iters=60, radius=1.0, sample=200_000, seed=0):
    """The translation that takes `src` onto `dst`: the median nearest-neighbour displacement of
    a sample of `src`, iterated with a shrinking reach until it moves less than 1 mm.
    Point-to-point steps are short along a gently sloping surface, hence the many iterations."""
    from scipy.spatial import cKDTree

    rng = np.random.default_rng(seed)
    s = src if len(src) <= sample else src[rng.choice(len(src), sample, replace=False)]
    tree = cKDTree(dst)
    off = np.zeros(3) if guess is None else np.asarray(guess, float).copy()
    r = radius
    for _ in range(iters):
        d, k = tree.query(s - off, distance_upper_bound=r, workers=-1)
        ok = np.isfinite(d)
        if ok.sum() < 10:
            break
        step = np.median((s - off)[ok] - dst[k[ok]], axis=0)
        off += step
        r = max(0.1, r * 0.6)
        if np.abs(step).max() < 1e-3:
            break
    return off


def transfer(map_xyz, orig_xyz, orig_attr, north_tol=0.15, tight_tol=0.03, fallback=0.0):
    """`orig_attr` (rows of the originals) for each map point, and how it was matched (int8):
    1 for the nearest original with |dy| scaled by `north_tol` and |dx|, |dz| by `tight_tol`
    (the reach is 1 in that metric); 2, for the rest, for the nearest original within
    `fallback` m (a map point the survey did not see takes the colour nearby); 0 for none."""
    from scipy.spatial import cKDTree

    scale = np.array([1 / tight_tol, 1 / north_tol, 1 / tight_tol])
    d, k = cKDTree(orig_xyz * scale).query(map_xyz * scale, distance_upper_bound=1.0, workers=-1)
    matched = np.where(np.isfinite(d), 1, 0).astype(np.int8)
    out = np.zeros((len(map_xyz),) + np.shape(orig_attr)[1:], dtype=np.asarray(orig_attr).dtype)
    out[matched == 1] = np.asarray(orig_attr)[k[matched == 1]]
    rest = np.flatnonzero(matched == 0)
    if fallback > 0 and len(rest):
        d2, k2 = cKDTree(orig_xyz).query(map_xyz[rest], distance_upper_bound=fallback, workers=-1)
        hit = np.isfinite(d2)
        out[rest[hit]] = np.asarray(orig_attr)[k2[hit]]
        matched[rest[hit]] = 2
    return out, matched


# ---- PCD I/O (binary, float fields; rgb packed as float) ------------------------------------

def read_pcd(path, bbox=None):
    """xyz (float64), and rgb (uint16 0-65535) and intensity when present. `bbox` (lo, hi) in the
    file's own frame keeps only the points inside, read in chunks (the originals are ~1 GB)."""
    with open(path, "rb") as f:
        head = {}
        while True:
            line = f.readline().decode().strip()
            k, _, v = line.partition(" ")
            head[k] = v.split()
            if k == "DATA":
                break
        if head["DATA"][0] != "binary":
            raise SystemExit(f"{path}: DATA {head['DATA'][0]}, only binary is read")
        fields, sizes, types = head["FIELDS"], [int(s) for s in head["SIZE"]], head["TYPE"]
        dt = np.dtype([(n, {"F": "<f", "U": "<u", "I": "<i"}[t] + str(s)) for n, s, t in zip(fields, sizes, types)])
        n = int(head["POINTS"][0])
        parts = []
        chunk = 4_000_000
        for i in range(0, n, chunk):
            a = np.frombuffer(f.read(dt.itemsize * min(chunk, n - i)), dtype=dt)
            if bbox is not None:
                lo, hi = bbox
                m = ((a["x"] >= lo[0]) & (a["x"] <= hi[0]) & (a["y"] >= lo[1]) & (a["y"] <= hi[1]))
                a = a[m]
            parts.append(a.copy())
    a = np.concatenate(parts)
    out = {"xyz": np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)}
    if "rgb" in fields:
        c = a["rgb"].view(np.uint32) if a["rgb"].dtype.kind == "f" else a["rgb"].astype(np.uint32)
        out["rgb"] = (np.stack([(c >> 16) & 255, (c >> 8) & 255, c & 255], 1) * 257).astype(np.uint16)
    if "intensity" in fields:
        out["intensity"] = a["intensity"].astype(np.float32)
    return out


def write_tiles(xyz, rgb, intensity, out_dir):
    """retile_laz.py's tiles: tile_<floor(x/50)>_<floor(y/50)>.laz, LAS 1.4 point format 3."""
    import laspy

    os.makedirs(out_dir, exist_ok=True)
    ix = np.floor(xyz[:, 0] / TILE).astype(np.int64)
    iy = np.floor(xyz[:, 1] / TILE).astype(np.int64)
    tiles = []
    for tx, ty in sorted(set(zip(ix.tolist(), iy.tolist()))):
        m = (ix == tx) & (iy == ty)
        h = laspy.LasHeader(point_format=3, version="1.4")
        h.scales = [OUT_SCALE] * 3
        h.offsets = [tx * TILE, ty * TILE, 0.0]
        las = laspy.LasData(h)
        las.x, las.y, las.z = xyz[m, 0], xyz[m, 1], xyz[m, 2]
        las.red, las.green, las.blue = rgb[m, 0], rgb[m, 1], rgb[m, 2]
        las.intensity = np.clip(intensity[m], 0, 65535).astype(np.uint16)
        las.classification = np.ones(int(m.sum()), np.uint8)       # unclassified
        p = os.path.join(out_dir, f"tile_{tx}_{ty}.laz")
        las.write(p)
        tiles.append({"tile": f"tile_{tx}_{ty}", "points": int(m.sum())})
    return tiles


def main():
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import site_params

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--site", required=True)
    ap.add_argument("map_pcd")
    ap.add_argument("out_dir")
    ap.add_argument("originals", nargs="+")
    ap.add_argument("--north-tol", type=float, default=0.15)
    ap.add_argument("--tight-tol", type=float, default=0.03)
    ap.add_argument("--fallback", type=float, default=0.0,
                    help="m: a map point with no original in the tight reach takes the nearest within this")
    ap.add_argument("--no-refine", action="store_true")
    a = ap.parse_args()
    site = site_params.load(a.site)
    to_map, from_map = survey_frame(site)

    t0 = time.time()
    m = read_pcd(a.map_pcd)
    P = m["xyz"]
    lo, hi = P.min(0) - 2.0, P.max(0) + 2.0
    print(f"map: {len(P):,} points, {np.round(lo, 1)} .. {np.round(hi, 1)}", flush=True)
    # the map's bbox in TWD97, to read only the originals' points near it
    ex, ny = from_map(np.array([lo[0], hi[0], lo[0], hi[0]]), np.array([lo[1], lo[1], hi[1], hi[1]]))
    bbox = ((min(ex) - 5, min(ny) - 5), (max(ex) + 5, max(ny) + 5))
    O, rgb, inten = [], [], []
    for p in a.originals:
        o = read_pcd(p, bbox=bbox)
        x, y = to_map(o["xyz"][:, 0], o["xyz"][:, 1])
        O.append(np.stack([x, y, o["xyz"][:, 2]], 1))
        rgb.append(o["rgb"])
        inten.append(o.get("intensity", np.zeros(len(o["xyz"]), np.float32)))
        print(f"{os.path.basename(p)}: {len(o['xyz']):,} points near the map ({time.time() - t0:.0f} s)", flush=True)
    O, rgb, inten = np.concatenate(O), np.concatenate(rgb), np.concatenate(inten)
    off = np.zeros(3)
    if not a.no_refine:
        # a first guess from the clouds' heights (a vertical datum), then ICP on translation
        off = refine_offset(O, P, guess=np.array([0, 0, np.median(O[:, 2]) - np.median(P[:, 2])]))
    print(f"originals -> map offset {np.round(off, 3).tolist()}", flush=True)
    attrs, how = transfer(P, O - off, np.c_[rgb, inten], a.north_tol, a.tight_tol, a.fallback)
    ok = how > 0
    col = np.where(ok[:, None], attrs[:, :3], 128 * 257).astype(np.uint16)
    it = np.where(ok, attrs[:, 3], 0).astype(np.float32)
    print(f"coloured {(how == 1).mean() * 100:.1f} % of the map in the tight reach, "
          f"{(how == 2).mean() * 100:.1f} % within {a.fallback} m, {(~ok).sum():,} points grey", flush=True)
    tiles = write_tiles(P, col, it, a.out_dir)
    meta = {"source_map": os.path.abspath(a.map_pcd), "originals": [os.path.abspath(p) for p in a.originals],
            "frame": f"map ({site['name']}), survey_frame {site.get('survey_frame', 'projected')}",
            "tile_size_m": TILE, "tile_name": "tile_<floor(x/50)>_<floor(y/50)>.laz",
            "originals_minus_map_offset": off.tolist(), "north_tol": a.north_tol, "tight_tol": a.tight_tol,
            "matched_tight": float((how == 1).mean()), "matched_fallback": float((how == 2).mean()),
            "fallback_m": a.fallback, "total_points": int(len(P)), "tiles": tiles}
    with open(os.path.join(a.out_dir, "tiles.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(f"{len(tiles)} tiles in {a.out_dir} ({time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
