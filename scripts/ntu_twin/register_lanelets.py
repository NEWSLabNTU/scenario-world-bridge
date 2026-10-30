#!/usr/bin/env python3
"""Re-register the merged Lanelet2 map to the scan (roadmap 059 N7).

    register_lanelets.py ROUTES_DIR MAP_DIR GROUND_DIR [--keep MESH_DIR/ground_keep.npy]
        [--min-width 2.8] [--report report.json]

The lanelets of r01 and r02 were drawn on each route's own point cloud
(ROUTES_DIR/<route>/pointcloud_map.pcd*), which drifts up to 0.3 m against the vendor scan
(019) although the mean shift is taken out. MAP_DIR/pointcloud_map/ is the scan in the map
frame (export_map.py), so the route clouds are registered to it, and the lanelets follow:

  1. per route, a rigid point-to-plane ICP of the route cloud to the scan (open3d);
  2. then a rigid ICP per anchor, a 25 m grid over the route, each on the route's points within
     30 m, kept when it converges (fitness >= 0.5); each anchor's displacement is what its
     transform does to the anchor's centre;
  3. the lanelet nodes of that route move by the anchors' displacements, Gaussian-weighted
     (sigma 30 m), the global one where no anchor is near;
  4. a node's height becomes the ground's (GROUND_DIR/dtm.npy, bilinear) where the ground is
     meshed (--keep, build_meshes.py's ground_keep.npy; without it, where the DTM is ground
     within a cell of the node); elsewhere it keeps the ICP's z.

  5. a road lanelet narrower than --min-width at a boundary vertex has that vertex pushed
     outward to the width (at most MAX_PUSH), when the boundary is the lanelet's alone (an
     outer edge, not shared with a neighbour) and the new place is on the scan's ground (a
     ground cell within a cell of it). The vendor's maps pinch to 1.97 m on roads the scan
     shows 10 m wide; Autoware's footprint of the car is 2.163 m, so path_optimizer stopped
     r01 at lanelet 1004849's 2.16 m vertex, "outside drivable area" (roadmap 059 N8).

lat/lon move with local_x/local_y (UTM 51N), so the MGRS projector and the local tags agree.
The original is kept as lanelet2_map.orig.osm (the first run's input on reruns).

Checks, written to the report: node heights against the DTM before and after, and lane paint
under the painted line strings (type line_thin: the intensity orthophoto,
GROUND_DIR/ortho/int_*.png, sampled every 0.1 m), the mean and the share of samples on the
brightest decile of their tile, before and after.
"""
import argparse
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from align_route import read_pcd  # noqa: E402

SHIFT = np.array([-0.54, 0.20, 17.30])  # vendor = map + SHIFT (build_meshes.py's)
TILE = 50.0
ID_OFFSET = 1_000_000  # export_map.py's: r01's ids
ANCHOR = 25.0
RADIUS = 30.0
SIGMA = 30.0
MAX_PUSH = 1.5  # m a boundary vertex may move outward to reach --min-width


def voxel(p, s):
    k = np.floor(p / s).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return p[idx]


def scan_near(map_dir, lo, hi):
    """The divided PCD map's points within [lo, hi] (xy), map frame."""
    out = []
    pdir = os.path.join(map_dir, "pointcloud_map")
    for name in sorted(os.listdir(pdir)):
        ix, iy = (int(v) for v in name[len("tile_"):-len(".pcd")].split("_"))
        x0, y0 = ix * TILE - SHIFT[0], iy * TILE - SHIFT[1]
        if x0 > hi[0] or y0 > hi[1] or x0 + TILE < lo[0] or y0 + TILE < lo[1]:
            continue
        out.append(read_pcd(os.path.join(pdir, name)))
    p = np.concatenate(out)
    m = np.all((p[:, :2] >= lo) & (p[:, :2] <= hi), axis=1)
    return p[m]


def icp(src, tgt, init=np.eye(4)):
    import open3d as o3d

    reg = o3d.pipelines.registration
    t = init
    res = None
    for d in (1.0, 0.4, 0.2):
        res = reg.registration_icp(src, tgt, d, t, reg.TransformationEstimationPointToPlane(),
                                   reg.ICPConvergenceCriteria(max_iteration=40))
        t = res.transformation
    return t, res.fitness, res.inlier_rmse


def pcd(p, normals=False):
    import open3d as o3d

    c = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
    if normals:
        c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=30))
    return c


def displacement_field(route_pts, map_dir):
    """(anchors (k, 2), displacements (k, 3), global transform, report) for one route."""
    src_all = voxel(route_pts, 0.3)
    lo, hi = src_all[:, :2].min(0) - 5.0, src_all[:, :2].max(0) + 5.0
    import open3d as o3d

    tgt = pcd(scan_near(map_dir, lo, hi), normals=True)
    tgt_p, tgt_n = np.asarray(tgt.points), np.asarray(tgt.normals)
    T, fit, rmse = icp(pcd(src_all), tgt)
    rep = {"points": int(len(src_all)), "global": {
        "t": [round(float(v), 3) for v in T[:3, 3]], "fitness": round(fit, 3), "rmse": round(rmse, 3),
        "yaw_deg": round(float(np.degrees(np.arctan2(T[1, 0], T[0, 0]))), 4)}}
    from scipy.spatial import cKDTree

    tree = cKDTree(src_all[:, :2])
    tgt_tree = cKDTree(tgt_p[:, :2])
    gx = np.arange(lo[0], hi[0] + ANCHOR, ANCHOR)
    gy = np.arange(lo[1], hi[1] + ANCHOR, ANCHOR)
    anchors, disp, kept, dropped = [], [], [], 0
    for x in gx:
        for y in gy:
            idx = tree.query_ball_point([x, y], RADIUS)
            if len(idx) < 3000:
                continue
            local = src_all[idx]
            # the anchor's own patch of the scan: open3d builds a KD-tree of the whole target
            # on every call, which made the route-wide target cost minutes per anchor
            ti = tgt_tree.query_ball_point([x, y], RADIUS + 3.0)
            if len(ti) < 3000:  # the route off the scan's coverage
                dropped += 1
                continue
            patch = pcd(tgt_p[ti])
            patch.normals = o3d.utility.Vector3dVector(tgt_n[ti])
            Ta, fa, ra = icp(pcd(local), patch, T)
            if fa < 0.5:
                dropped += 1
                continue
            c = np.array([x, y, np.median(local[:, 2]), 1.0])
            anchors.append((x, y))
            disp.append((Ta @ c)[:3] - c[:3])
            kept.append(ra)
    rep["anchors"] = len(anchors)
    rep["anchors_dropped"] = dropped
    if anchors:
        d = np.asarray(disp)
        h = np.hypot(d[:, 0], d[:, 1])
        rep["anchor_xy_m_p50_max"] = [round(float(np.median(h)), 3), round(float(h.max()), 3)]
        rep["anchor_z_m_p5_p50_p95"] = [round(float(v), 3) for v in np.percentile(d[:, 2], [5, 50, 95])]
        rep["anchor_rmse_p50"] = round(float(np.median(kept)), 3)
    return np.asarray(anchors).reshape(-1, 2), np.asarray(disp).reshape(-1, 3), T, rep


def move(xyz, anchors, disp, T):
    g = (T[:3, :3] @ xyz.T).T + T[:3, 3] - xyz
    if not len(anchors):
        return xyz + g
    r2 = ((xyz[:, None, :2] - anchors[None]) ** 2).sum(-1)
    w = np.exp(-r2 / (2 * SIGMA ** 2))
    ws = w.sum(1, keepdims=True)
    local = (w @ disp) / np.maximum(ws, 1e-12)
    a = np.clip(ws / 0.5, 0, 1)  # far from every anchor: the global transform
    return xyz + a * local + (1 - a) * g


class Ground:
    def __init__(self, ground_dir, keep=""):
        self.meta = json.load(open(os.path.join(ground_dir, "dtm.json")))
        self.dtm = np.load(os.path.join(ground_dir, "dtm.npy"))
        self.ground = np.load(os.path.join(ground_dir, "ground_cells.npy"))
        self.keep = np.load(keep) if keep else None
        self.dir = ground_dir
        self._int = {}

    def z(self, xy):
        """Map-frame ground height (bilinear over cell centres) and whether it is ground."""
        m, c = self.meta, self.meta["cell"]
        v = xy + SHIFT[:2]
        fx = (v[:, 0] - m["x0"]) / c - 0.5
        fy = (v[:, 1] - m["y0"]) / c - 0.5
        i0 = np.clip(np.floor(fx).astype(int), 0, m["width"] - 2)
        j0 = np.clip(np.floor(fy).astype(int), 0, m["height"] - 2)
        ax, ay = np.clip(fx - i0, 0, 1), np.clip(fy - j0, 0, 1)
        d = self.dtm
        z = ((1 - ax) * (1 - ay) * d[j0, i0] + ax * (1 - ay) * d[j0, i0 + 1]
             + (1 - ax) * ay * d[j0 + 1, i0] + ax * ay * d[j0 + 1, i0 + 1])
        if self.keep is not None:
            return z - SHIFT[2], self.keep[j0, i0] & np.isfinite(z)
        ok = np.zeros(len(xy), bool)
        for dj in (-1, 0, 1, 2):
            for di in (-1, 0, 1, 2):
                jj = np.clip(j0 + dj, 0, m["height"] - 1)
                ii = np.clip(i0 + di, 0, m["width"] - 1)
                ok |= self.ground[jj, ii]
        return z - SHIFT[2], ok & np.isfinite(z)

    def intensity(self, xy):
        """The intensity orthophoto at map xy (nan off the ortho) and the tile's 90th pct."""
        from PIL import Image

        v = xy + SHIFT[:2]
        out = np.full(len(xy), np.nan)
        hi = np.full(len(xy), np.nan)
        ix = np.floor(v[:, 0] / TILE).astype(int)
        iy = np.floor(v[:, 1] / TILE).astype(int)
        for key in set(zip(ix.tolist(), iy.tolist())):
            if key not in self._int:
                p = os.path.join(self.dir, "ortho", f"int_{key[0]}_{key[1]}.png")
                mp = os.path.join(self.dir, "ortho", f"mask_{key[0]}_{key[1]}.png")
                if not os.path.exists(p):
                    self._int[key] = None
                else:
                    a = np.asarray(Image.open(p), dtype=np.float64)
                    if os.path.exists(mp):
                        a[np.asarray(Image.open(mp)) == 0] = np.nan
                    self._int[key] = (a, np.nanpercentile(a, 90))
            t = self._int[key]
            sel = (ix == key[0]) & (iy == key[1])
            if t is None:
                continue
            a, p90 = t
            n = a.shape[0]
            oi = ((v[sel, 0] - key[0] * TILE) / 0.02).astype(int).clip(0, n - 1)
            oj = ((v[sel, 1] - key[1] * TILE) / 0.02).astype(int).clip(0, n - 1)
            out[sel] = a[n - 1 - oj, oi]
            hi[sel] = p90
        return out, hi


def paint_check(root, coords, ground):
    """Lane paint under the painted line strings: mean intensity, share on the top decile."""
    samples = []
    for w in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("type") != "line_thin":
            continue
        pts = np.array([coords[int(nd.get("ref"))][:2] for nd in w.findall("nd")])
        for a, b in zip(pts[:-1], pts[1:]):
            n = max(2, int(np.hypot(*(b - a)) / 0.1))
            samples.append(a + (b - a) * np.linspace(0, 1, n)[:, None])
    if not samples:
        return None
    s = np.concatenate(samples)
    v, p90 = ground.intensity(s)
    ok = np.isfinite(v)
    return {"samples": int(ok.sum()), "mean": round(float(v[ok].mean()), 1),
            "on_top_decile": round(float((v[ok] >= p90[ok]).mean()), 3)}


def _dist_to(p, poly):
    """Distance from p to a polyline, and the unit vector from its nearest point to p."""
    best, away = 1e18, np.zeros(2)
    for a, b in zip(poly[:-1], poly[1:]):
        d = b - a
        t = np.clip(np.dot(p - a, d) / max(np.dot(d, d), 1e-12), 0.0, 1.0)
        q = a + t * d
        r = np.linalg.norm(p - q)
        if r < best:
            best, away = r, (p - q) / max(r, 1e-12)
    return best, away


def widen(root, pos, ground, min_width):
    """Push narrow road lanelets' outer boundary vertices out to `min_width` (step 5). `pos`:
    {node id: map xyz (array, edited in place)}. Returns the report."""
    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")] for w in root.findall("way")}
    lanelets, uses = [], {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
            continue
        m = {x.get("role"): int(x.get("ref")) for x in rel.findall("member")}
        lanelets.append((int(rel.get("id")), m["left"], m["right"]))
        for w in (m["left"], m["right"]):
            uses[w] = uses.get(w, 0) + 1
    moved, blocked, shared, narrow = {}, 0, 0, set()
    for lid, lw, rw in lanelets:
        for own, other in ((lw, rw), (rw, lw)):
            opp = np.array([pos[i][:2] for i in ways[other]])
            for nid in ways[own]:
                w, away = _dist_to(pos[nid][:2], opp)
                if w >= min_width:
                    continue
                narrow.add(lid)
                if uses[own] > 1:
                    shared += 1
                    continue
                push = min(min_width - w, MAX_PUSH)
                target = pos[nid][:2] + away * push
                m, c = ground.meta, ground.meta["cell"]
                v = target + SHIFT[:2]
                i = int((v[0] - m["x0"]) / c)
                j = int((v[1] - m["y0"]) / c)
                if not ground.ground[max(j - 1, 0):j + 2, max(i - 1, 0):i + 2].any():
                    blocked += 1
                    continue
                pos[nid][:2] = target
                moved[nid] = round(float(push), 3)
    after = []
    for lid, lw, rw in lanelets:
        lb = np.array([pos[i][:2] for i in ways[lw]])
        rb = np.array([pos[i][:2] for i in ways[rw]])
        after.append(min(min(_dist_to(p, rb)[0] for p in lb), min(_dist_to(p, lb)[0] for p in rb)))
    after = np.array(after)
    return {"min_width": min_width, "lanelets_narrower_before": len(narrow),
            "vertices_moved": len(moved), "push_m_p50_max": [round(float(np.median(list(moved.values()))), 3),
                                                             round(float(max(moved.values())), 3)] if moved else None,
            "vertices_shared_skipped": shared, "vertices_off_ground_skipped": blocked,
            "lanelets_narrower_after": int((after < min_width).sum()),
            "narrowest_after_m": round(float(after.min()), 2),
            "narrower_than_car_after": int((after < 2.163).sum())}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("routes_dir")
    ap.add_argument("map_dir")
    ap.add_argument("ground_dir")
    ap.add_argument("--keep", default="", help="build_meshes.py's ground_keep.npy")
    ap.add_argument("--min-width", type=float, default=2.8,
                    help="widen road lanelets to this, m (0: leave them)")
    ap.add_argument("--report", default="")
    a = ap.parse_args(argv)

    osm = os.path.join(a.map_dir, "lanelet2_map.osm")
    orig = os.path.join(a.map_dir, "lanelet2_map.orig.osm")
    if not os.path.exists(orig):
        shutil.copyfile(osm, orig)
    tree = ET.parse(orig)
    root = tree.getroot()
    nodes = {int(n.get("id")): n for n in root.findall("node")}

    def tag(n, k):
        return next(t for t in n.findall("tag") if t.get("k") == k)

    ids = np.array(sorted(nodes))
    xyz = np.array([[float(tag(nodes[i], k).get("v")) for k in ("local_x", "local_y", "ele")]
                    for i in ids])
    ground = Ground(a.ground_dir, a.keep)
    report = {"routes": {}}
    gz, gok = ground.z(xyz[:, :2])
    dz = xyz[gok, 2] - gz[gok]
    report["node_minus_ground_before"] = [round(float(v), 3) for v in np.percentile(dz, [5, 50, 95])]
    report["paint_before"] = paint_check(root, dict(zip(ids.tolist(), xyz)), ground)

    new = xyz.copy()
    for r, is_r in (("r02", ids < ID_OFFSET), ("r01", ids >= ID_OFFSET)):
        d = os.path.join(a.routes_dir, r)
        pcd_path = next(os.path.join(d, f) for f in sorted(os.listdir(d)) if ".pcd" in f)
        anchors, disp, T, rep = displacement_field(read_pcd(pcd_path), a.map_dir)
        new[is_r] = move(xyz[is_r], anchors, disp, T)
        h = np.hypot(*(new[is_r] - xyz[is_r])[:, :2].T)
        rep["node_xy_move_m_p50_max"] = [round(float(np.median(h)), 3), round(float(h.max()), 3)]
        report["routes"][r] = rep
        print(r, json.dumps(rep), flush=True)

    if a.min_width > 0:
        pos = {int(i): new[k] for k, i in enumerate(ids)}  # views into `new`
        report["widen"] = widen(root, pos, ground, a.min_width)
        print("widen", json.dumps(report["widen"]), flush=True)
    gz, gok = ground.z(new[:, :2])
    new[gok, 2] = gz[gok]
    report["nodes"] = int(len(ids))
    report["nodes_on_ground"] = int(gok.sum())
    report["paint_after"] = paint_check(root, dict(zip(ids.tolist(), new)), ground)

    from pyproj import Transformer

    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32651", always_xy=True)
    to_ll = Transformer.from_crs("EPSG:32651", "EPSG:4326", always_xy=True)
    lon = np.array([float(nodes[i].get("lon")) for i in ids])
    lat = np.array([float(nodes[i].get("lat")) for i in ids])
    e, n = to_utm.transform(lon, lat)
    lon2, lat2 = to_ll.transform(e + new[:, 0] - xyz[:, 0], n + new[:, 1] - xyz[:, 1])
    for k, i in enumerate(ids):
        nd = nodes[i]
        nd.set("lat", f"{lat2[k]:.11f}")
        nd.set("lon", f"{lon2[k]:.11f}")
        tag(nd, "local_x").set("v", f"{new[k, 0]:.4f}")
        tag(nd, "local_y").set("v", f"{new[k, 1]:.4f}")
        tag(nd, "ele").set("v", f"{new[k, 2]:.4f}")
    tree.write(osm, encoding="UTF-8", xml_declaration=True)
    print(json.dumps({k: v for k, v in report.items() if k != "routes"}))
    if a.report:
        with open(a.report, "w") as f:
            json.dump(report, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
