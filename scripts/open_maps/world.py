#!/usr/bin/env python3
"""Roadmap 062, O5: an Autoware map directory with an Isaac world, from a generated Lanelet2 map
and the site's public data. The world is believable, not a twin: the real layout and the real
ground photo, the roads and their paint as the map has them, buildings extruded from their
footprints.

    scripts/open_maps/world.py <site.yaml> <out> meshes     # the tool venv
    $ISAAC_ROOT/venv/bin/python scripts/open_maps/world_usd.py <site.yaml> <out>
    scripts/open_maps/world.py <site.yaml> <out> pcd        # the tool venv, after the USD step

Writes <out>/map/: `lanelet2_map.osm` (the generated map), `map_projector_info.yaml` (MGRS),
`traffic_lights.yaml` (the signal table),
`pointcloud_map.pcd`, `isaac/world.usdc`, `isaac/world.yaml`, and the work files in
`<out>/map/world/`.

Frames: the Autoware map frame is the site's (MGRS-local); the Isaac world is the map frame less
the site's `map_origin` (059's, so the two NTU worlds share an origin), which world.yaml names.

- **Ground:** a flat plane 3 cm under the road with NLSC's photo (z19, 0.27 m) as its texture.
  The NTU box is flat (the Taipei basin); MOI's 20 m DTM would add nothing at this scale.
- **Roads:** every road lanelet's surface, from its bounds, at z 0, a collider (labelled Road).
- **Paint** (labelled RoadLine, not colliders), from the map's line types:
  - white lines: solid, or dashed 4 m on 6 m off;
  - the yellow centre line where two directions share a line;
  - 0.4 m stop lines;
  - zebra crossings on the crosswalks.
- **Buildings:** Overture's footprints (OSM's and others'), extruded to `height`, else
  `num_floors` x 3.2 m, else 12 m; colliders.
- **Props:**
  - trees from OSM (`natural=tree`, `tree_row`), as CARLA's trees;
  - traffic-light poles beyond each signalised approach's stop line, 1 m clear of every
    road lanelet (a junction's turning paths too); lamps and trees on the road dropped;
  - street lamps at Mapillary's street lights.
- **Point cloud:** divided into 50 m tiles (`pointcloud_map/`, its metadata, and
  `pointcloud_map.pcd` a link to the directory), sampled from the same surfaces (road, ground within 30 m of a road,
  buildings within 60 m, poles, the trees' meshes from the USD step), one point per 0.2 m
  voxel, so NDT's map is the lidar's world.
"""
import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from generate import Frame  # noqa: E402
from plot_map import load  # noqa: E402

TERRAIN_STEP = 5.0
GROUND_Z = -0.03
LINE_HALF = 0.075
LINE_LIFT = 0.006
DASH, GAP = 4.0, 6.0
STOP_HALF = 0.2
ZEBRA, ZEBRA_GAP = 0.45, 0.45
FLOOR_M, DEFAULT_HEIGHT = 3.2, 12.0
VOXEL = 0.2
PROP_CLEAR = 1.0    # m, a pole, lamp or tree from the nearest road lanelet
POLE_REACH = 8.0    # m, how far beyond its stop line a signal pole may go
TILE = 50.0         # m, the divided point cloud's tiles (059's size)


def resample(pts, n):
    p = np.asarray(pts, float)
    d = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    if d[-1] < 1e-9:
        return np.repeat(p[:1], n, 0)
    t = np.linspace(0, d[-1], n)
    return np.stack([np.interp(t, d, p[:, 0]), np.interp(t, d, p[:, 1])], 1)


def strip(left, right, z=0.0):
    """Quads between two polylines of equal count: points (2n, 3), faces (n-1, 4)."""
    n = len(left)
    pts = np.concatenate([np.c_[left, np.full(n, z)], np.c_[right, np.full(n, z)]])
    i = np.arange(n - 1)
    return pts, np.stack([i, i + 1, n + i + 1, n + i], 1)


def ribbon(line, half, z):
    """A painted band along a polyline."""
    p = np.asarray(line, float)
    if len(p) < 2:
        return None
    d = np.gradient(p, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-12
    nrm = np.stack([-d[:, 1], d[:, 0]], 1)
    return strip(p + nrm * half, p - nrm * half, z)


def cut(line, start, end):
    p = np.asarray(line, float)
    d = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    keep = (d > start) & (d < end)
    a = np.array([np.interp(start, d, p[:, 0]), np.interp(start, d, p[:, 1])])
    b = np.array([np.interp(end, d, p[:, 0]), np.interp(end, d, p[:, 1])])
    return np.vstack([a, p[keep], b]), d[-1]


def merge(parts):
    pts, faces, off = [], [], 0
    for pp, ff in parts:
        pts.append(pp)
        faces.append(ff + off)
        off += len(pp)
    if not pts:
        return np.zeros((0, 3)), np.zeros((0, 4), int)
    return np.concatenate(pts), np.concatenate(faces)


def meshes(site, out):
    frame = Frame(site["frame"])
    ox, oy, _ = site["frame"]["map_origin"]
    gen = out / "generated" / "lanelet2_map.osm"
    xy, ways, L = load(str(gen))
    W = {k: (v[0] - ox, v[1] - oy) for k, v in xy.items()}      # world frame

    def wpts(ids):
        return np.array([W[i] for i in ids if i in W])
    # roads
    road = []
    for k, (t, l, r) in L.items():
        if t.get("subtype") != "road":
            continue
        lp, rp = wpts(l), wpts(r)
        n = max(2, int(math.ceil(max(np.linalg.norm(np.diff(lp, axis=0), axis=1).sum(),
                                     np.linalg.norm(np.diff(rp, axis=0), axis=1).sum()) / 2.0)) + 1)
        road.append(strip(resample(lp, n), resample(rp, n)))
    # paint
    white, yellow = [], []
    by_ends = {}
    for wid, (ids, t) in ways.items():
        if t.get("type") == "line_thin":
            by_ends.setdefault(frozenset((ids[0], ids[-1])), []).append(wid)
    done = set()
    for wid, (ids, t) in ways.items():
        typ = t.get("type")
        if typ == "line_thin":
            key = frozenset((ids[0], ids[-1]))
            if key in done:
                continue
            done.add(key)
            p = wpts(ids)
            if len(p) < 2:
                continue
            shared = len(by_ends[key]) > 1          # two directions' common line
            if t.get("subtype") == "dashed":
                total = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
                s = 1.0
                while s + DASH <= total:
                    seg, _ = cut(p, s, s + DASH)
                    rb = ribbon(seg, LINE_HALF, LINE_LIFT)
                    if rb:
                        white.append(rb)
                    s += DASH + GAP
            else:
                rb = ribbon(p, LINE_HALF, LINE_LIFT)
                if rb:
                    (yellow if shared else white).append(rb)
        elif typ == "stop_line":
            rb = ribbon(wpts(ids), STOP_HALF, LINE_LIFT)
            if rb:
                white.append(rb)
    for k, (t, l, r) in L.items():
        if t.get("subtype") != "crosswalk":
            continue
        lp, rp = wpts(l), wpts(r)
        total = np.linalg.norm(np.diff(lp, axis=0), axis=1).sum()
        s = 0.3
        while s + ZEBRA <= total:
            a, _ = cut(lp, s, s + ZEBRA)
            b, _ = cut(rp, s, s + ZEBRA)
            n = max(len(a), len(b))
            white.append(strip(resample(a, n), resample(b, n), LINE_LIFT))
            s += ZEBRA + ZEBRA_GAP
    # ground
    s_, w_, n_, e_ = site["bbox"]
    xs = [frame.xy(la, lo)[0] - ox for la, lo in ((s_, w_), (n_, w_), (s_, e_), (n_, e_))]
    ys = [frame.xy(la, lo)[1] - oy for la, lo in ((s_, w_), (n_, w_), (s_, e_), (n_, e_))]
    gx, gy = np.meshgrid(np.arange(min(xs), max(xs) + 1e-6, TERRAIN_STEP),
                         np.arange(min(ys), max(ys) + 1e-6, TERRAIN_STEP))
    meta = json.loads((out / "raw" / "nlsc_PHOTO2_z19.json").read_text())
    N = 2 ** meta["zoom"]
    lat, lon = np.vectorize(lambda x, y: frame.ll(x + ox, y + oy))(gx.ravel(), gy.ravel())
    px = (lon + 180) / 360 * N - meta["tiles"][0]
    py = (1 - np.arcsinh(np.tan(np.radians(lat))) / np.pi) / 2 * N - meta["tiles"][1]
    tw, th = meta["tiles"][2] - meta["tiles"][0] + 1, meta["tiles"][3] - meta["tiles"][1] + 1
    uv = np.stack([px / tw, 1 - py / th], 1)
    tp = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, GROUND_Z)], 1)
    nc = gx.shape[1]
    jj, ii = np.meshgrid(np.arange(gx.shape[0] - 1), np.arange(nc - 1), indexing="ij")
    a = (jj * nc + ii).ravel()
    tf = np.stack([a, a + 1, a + nc + 1, a + nc], 1)
    # buildings
    bpts, btri, heights = buildings(out, frame, ox, oy)
    # props
    props = props_of(out, site, frame, ox, oy, W, ways, L)
    work = out / "map" / "world"
    work.mkdir(parents=True, exist_ok=True)
    rp, rf = merge(road)
    wp, wf = merge(white)
    yp, yf = merge(yellow)
    np.savez_compressed(work / "meshes.npz", terrain_points=tp, terrain_faces=tf, terrain_uv=uv,
                        terrain_shape=np.array(gx.shape), road_points=rp, road_faces=rf,
                        white_points=wp, white_faces=wf, yellow_points=yp, yellow_faces=yf,
                        building_points=bpts, building_tris=btri)
    (work / "props.json").write_text(json.dumps(props))
    shutil.copyfile(out / "raw" / "nlsc_PHOTO2_z19.png", work / "photo.png")
    m = out / "map"
    shutil.copyfile(gen, m / "lanelet2_map.osm")
    signal_table(gen, m / "traffic_lights.yaml")
    f = site["frame"]
    (m / "map_projector_info.yaml").write_text(
        f"projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: {f['mgrs_grid']}\n")
    print(json.dumps({"road_quads": len(rf), "white_quads": len(wf), "yellow_quads": len(yf),
                      "terrain_vertices": len(tp), "buildings": int(len(heights)),
                      "building_triangles": len(btri), "trees": len(props["trees"]),
                      "signal_poles": len(props["signals"]), "lamps": len(props["lamps"])}))


def signal_table(osm, path):
    """The map's signal table, as the bridge reads it (swb_core map_resolver: `traffic_lights.yaml`):
    per traffic_light way its regulatory elements and an id for the world server, here the way's
    own. No positions: the world has no lamp prims to colour, and the colours reach Autoware
    without them (swb_isaac_world signals.groups). Without the table nothing publishes the
    lights' states and Autoware stops at every signal (062, the first NTU runs)."""
    import xml.etree.ElementTree as ET
    root = ET.parse(osm).getroot()
    lights = [w.get("id") for w in root.findall("way")
              if any(t.get("k") == "type" and t.get("v") == "traffic_light" for t in w.findall("tag"))]
    regs = {}
    for r in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in r.findall("tag")}
        if tags.get("subtype") != "traffic_light":
            continue
        for mem in r.findall("member"):
            if mem.get("role") == "refers":
                regs.setdefault(mem.get("ref"), []).append(int(r.get("id")))
    lines = ["# Generated by scripts/open_maps/world.py (roadmap 062) from the generated Lanelet2 map;",
             "# the world server's id for each light is its way id.", "signals:"]
    for w in lights:
        lines += [f"- way_id: {w}", "  regulatory_element_ids:"] + [f"  - {r}" for r in regs.get(w, [])] \
            + [f"  opendrive_id: '{w}'"]
    path.write_text("\n".join(lines) + "\n")


def buildings(out, frame, ox, oy):
    import duckdb
    import shapely
    from shapely import wkb
    from shapely.geometry import Polygon
    con = duckdb.connect()
    con.execute("LOAD spatial")
    rows = con.execute(f"""SELECT ST_AsWKB(geometry), height, num_floors, min_height
                           FROM '{out / "raw" / "overture_building.parquet"}'
                           WHERE NOT coalesce(is_underground, false)""").fetchall()
    pts, tris, hs = [], [], []
    for g, h, nf, minh in rows:
        geom = wkb.loads(bytes(g))
        polys = list(geom.geoms) if geom.geom_type == "MultiPolygon" else [geom]
        top = h if h else (nf * FLOOR_M if nf else DEFAULT_HEIGHT)
        base = minh or 0.0
        for poly in polys:
            if poly.geom_type != "Polygon" or poly.area == 0:
                continue

            def tr(coords):
                return [(frame.xy(la, lo)[0] - ox, frame.xy(la, lo)[1] - oy) for lo, la in coords]
            P = Polygon(tr(poly.exterior.coords), [tr(r.coords) for r in poly.interiors]).buffer(0)
            if P.is_empty or P.geom_type != "Polygon":
                continue
            hs.append(top)
            off = sum(len(p) for p in pts)
            vs, fs = [], []
            # roof
            for t in shapely.constrained_delaunay_triangles(P).geoms:
                c = list(t.exterior.coords)[:3]
                i = len(vs)
                vs += [(x, y, top) for x, y in c]
                fs.append((i, i + 1, i + 2))
            # walls
            for ring in [P.exterior] + list(P.interiors):
                c = list(ring.coords)
                for (x0, y0), (x1, y1) in zip(c, c[1:]):
                    i = len(vs)
                    vs += [(x0, y0, base), (x1, y1, base), (x1, y1, top), (x0, y0, top)]
                    fs += [(i, i + 1, i + 2), (i, i + 2, i + 3)]
            pts.append(np.array(vs))
            tris.append(np.array(fs) + off)
    return (np.concatenate(pts) if pts else np.zeros((0, 3)),
            np.concatenate(tris) if tris else np.zeros((0, 3), int), np.array(hs))


def props_of(out, site, frame, ox, oy, W, ways, L):
    d = json.loads((out / "raw" / "osm.json").read_text())
    s, w, n, e = site["bbox"]
    nodes = {el["id"]: el for el in d["elements"] if el["type"] == "node"}

    def inbox(nd):
        return s <= nd["lat"] <= n and w <= nd["lon"] <= e

    def wxy(nd):
        x, y = frame.xy(nd["lat"], nd["lon"])
        return [x - ox, y - oy]
    trees = [wxy(nd) for nd in nodes.values() if nd.get("tags", {}).get("natural") == "tree" and inbox(nd)]
    for el in d["elements"]:
        if el["type"] == "way" and el.get("tags", {}).get("natural") == "tree_row":
            p = np.array([wxy(nodes[i]) for i in el["nodes"] if i in nodes and inbox(nodes[i])])
            if len(p) >= 2:
                total = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
                for k in np.arange(0, total, 7.0):
                    seg, _ = cut(p, 0, max(k, 1e-3))
                    trees.append(seg[-1].tolist())
    # Nothing stands on the road: the stop line's end is 1 m from its own lane but can be inside
    # a junction's turning paths (the first NTU run with the merges fixed stopped for a pole
    # 4.6 m right of its lane, in the right turn onto 新生南路).
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union
    from shapely.prepared import prep
    surface = unary_union([Polygon(np.concatenate([[W[i] for i in l], [W[i] for i in r][::-1]])).buffer(0)
                           for t, l, r in L.values() if t.get("subtype") == "road"])
    near = prep(surface.buffer(PROP_CLEAR))

    def clear(q):
        return not near.contains(Point(q))
    trees = [q for q in trees if clear(q)]
    # a signal pole beyond the right end of each signalised approach's stop line, the first
    # point along it PROP_CLEAR m off every road lanelet (none within POLE_REACH m: no pole)
    signals, dropped = [], 0
    stop = {wid: ids for wid, (ids, t) in ways.items() if t.get("type") == "stop_line"}
    for wid, ids in stop.items():
        a, b = np.array(W[ids[0]]), np.array(W[ids[-1]])
        u = (b - a) / (np.linalg.norm(b - a) + 1e-9)
        p = next((b + u * k for k in np.arange(1.0, POLE_REACH, 0.25) if clear(b + u * k)), None)
        if p is None:
            dropped += 1
            continue
        # facing the approach: the stop line runs left to right across travel
        yaw = math.atan2(u[1], u[0]) + math.pi / 2
        signals.append([float(p[0]), float(p[1]), yaw])
    feats = json.loads((out / "raw" / "mapillary_features.json").read_text())
    lamps = []
    for f in feats:
        if f["object_value"] == "object--street-light":
            lo, la = f["geometry"]["coordinates"]
            if s <= la <= n and w <= lo <= e:
                x, y = frame.xy(la, lo)
                if clear([x - ox, y - oy]):
                    lamps.append([x - ox, y - oy])
    if dropped:
        print(f"{dropped} stop lines with no room for a pole within {POLE_REACH} m", file=sys.stderr)
    return {"trees": trees, "signals": signals, "lamps": lamps}


def pcd(site, out):
    """The point cloud, in the map frame."""
    from shapely.geometry import LineString
    from shapely.strtree import STRtree
    ox, oy, _ = site["frame"]["map_origin"]
    work = out / "map" / "world"
    m = np.load(work / "meshes.npz")
    props = json.loads((work / "props.json").read_text())
    rng = np.random.default_rng(62)

    def quads(pts, faces, step):
        if len(faces) == 0:
            return np.zeros((0, 3))
        tri = np.concatenate([faces[:, [0, 1, 2]], faces[:, [0, 2, 3]]])
        return tris(pts, tri, step)

    def tris(pts, tri, step):
        a, b, c = pts[tri[:, 0]], pts[tri[:, 1]], pts[tri[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        k = np.repeat(np.arange(len(a)), rng.poisson(area / (step * step)))
        u, v = rng.random(len(k)), rng.random(len(k))
        f = u + v > 1
        u[f], v[f] = 1 - u[f], 1 - v[f]
        return a[k] + u[:, None] * (b - a)[k] + v[:, None] * (c - a)[k]
    road = quads(m["road_points"], m["road_faces"], 0.25)
    # ground near the roads only
    g = quads(m["terrain_points"], m["terrain_faces"], 0.5)
    lanes = [LineString(p) for p in np.split(m["road_points"][:, :2], 1)]
    from scipy.spatial import cKDTree
    tree = cKDTree(m["road_points"][:, :2])
    dg, _ = tree.query(g[:, :2])
    g = g[dg < 30.0]
    b = tris(m["building_points"], m["building_tris"], 0.3)
    db, _ = tree.query(b[:, :2]) if len(b) else (np.zeros(0), None)
    b = b[db < 60.0]
    poles = []
    for x, y, *_ in props["signals"]:
        h = np.linspace(0, 6.0, 30)
        poles.append(np.stack([np.full(30, x), np.full(30, y), h], 1))
    for x, y in props["lamps"]:
        h = np.linspace(0, 8.0, 40)
        poles.append(np.stack([np.full(40, x), np.full(40, y), h], 1))
    parts = [road, g, b] + poles
    extra = work / "props_points.npy"
    if extra.exists():
        parts.append(np.load(extra))
    pts = np.concatenate(parts)
    key = np.floor(pts / VOXEL).astype(np.int64)
    _, first = np.unique(key, axis=0, return_index=True)
    pts = pts[np.sort(first)]
    pts[:, 0] += ox
    pts[:, 1] += oy
    pts = pts.astype(np.float32)
    write_divided(pts, out / "map")
    print(json.dumps({"pcd_points": len(pts), "road": len(road), "ground": len(g), "buildings": len(b),
                      "props": int(len(np.load(extra))) if extra.exists() else 0}))


def write_pcd(path, pts):
    with open(path, "wb") as f:
        f.write(("# .PCD v0.7 - scripts/open_maps/world.py (roadmap 062)\nVERSION 0.7\n"
                 "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                 f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\n"
                 "DATA binary\n").encode())
        f.write(pts.astype(np.float32).tobytes())


def write_divided(pts, map_dir, tile=TILE):
    """Autoware's divided point-cloud map: `pointcloud_map/` of tiles, `pointcloud_map_metadata.yaml`
    (each tile's lower corner), and `pointcloud_map.pcd` a link to the directory, so the default
    file name finds it. One 1.9 x 1.4 km file overflowed the compare-map filter's voxel grid ("Number
    of voxel overflows int32_t limit", the first NTU runs): tiles let it load only what is near."""
    d = map_dir / "pointcloud_map"
    if d.exists():
        shutil.rmtree(d)
    d.mkdir()
    key = np.floor(pts[:, :2] / tile).astype(np.int64)
    order = np.lexsort((key[:, 1], key[:, 0]))
    pts, key = pts[order], key[order]
    cut = np.flatnonzero(np.any(np.diff(key, axis=0) != 0, axis=1)) + 1
    meta = [f"x_resolution: {tile}", f"y_resolution: {tile}"]
    for a, b in zip(np.r_[0, cut], np.r_[cut, len(pts)]):
        ix, iy = key[a]
        name = f"tile_{ix}_{iy}.pcd"
        write_pcd(d / name, pts[a:b])
        # integers: Autoware's map loader reads a tile's corner as ints ("yaml-cpp: bad conversion" on 51550.00)
        meta.append(f"{name}: [{int(ix * tile)}, {int(iy * tile)}]")
    (map_dir / "pointcloud_map_metadata.yaml").write_text("\n".join(meta) + "\n")
    link = map_dir / "pointcloud_map.pcd"
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to("pointcloud_map", target_is_directory=True)
    return len(meta) - 2


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("step", choices=("meshes", "pcd"))
    a = ap.parse_args()
    site = yaml.safe_load(a.site.read_text())
    (a.out / "map").mkdir(parents=True, exist_ok=True)
    meshes(site, a.out) if a.step == "meshes" else pcd(site, a.out)


if __name__ == "__main__":
    main()
