#!/usr/bin/env python3
"""Measure how far a map's Lanelet2 lanes sit from CARLA's OpenDRIVE lanes (the feature-
completeness doc's "lane position" gap). Offline: CARLA's Python API parses the .xodr
without a server.

SSv2 re-canonicalises poses onto Lanelet2 while CARLA works in OpenDRIVE, so a lanelet off
its OpenDRIVE lane shows up as lane-matching failures. For points every `--step` metres along
each lanelet's centreline (the midpoints of its resampled bounds), this finds the nearest
OpenDRIVE lane centre running the same way (sampled every 0.5 m, junction connectors
included) and reports the lateral offset to it (across the lane, from a sample within
0.5 m along it), the heading difference and the width difference. A point past the end of
every matching lane (a lanelet drawn longer than its lane) is unmatched.

    scripts/lanelet_opendrive_registration.py \\
        data/carla-autoware-bridge/Town01/lanelet2_map.osm \\
        $CARLA_DIR/CarlaUE4/Content/Carla/Maps/OpenDrive/Town01.xodr [--step 1.0] [--worst 10]

Lanelet2 positions are its lat/lon, projected as Autoware projects them
(`autoware_projection.py`: the map's TransverseMercator, WGS84, k0 0.9996). Not the pack's
local_x/local_y: those were a spherical Mercator on R = 6 378 137 m, 0.7 % long in y against
CARLA (2.2 m on Town01's y = -330 road), while the point cloud map's kerbs and the driven ego
agree with the projected lat/lon; repair_lanelet_local_coords.py now rewrites them.
CARLA's frame is (x, -y).
"""

import argparse
import math
import statistics
import sys
from collections import Counter
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from autoware_projection import Projector  # noqa: E402


def parse_lanelets(path, subtypes=("road",)):
    """{lanelet id: (left [(x, y)], right [(x, y)])} in the ROS map frame, of the lanelets
    whose subtype is in `subtypes` (walkways and untyped lanelets are not OpenDRIVE driving
    lanes). A left bound drawn against the right one is reversed, as Lanelet2 does: the
    right bound gives the driving direction."""
    root = ET.parse(path).getroot()
    project = Projector.for_map(path).project
    nodes = {n.get("id"): project(float(n.get("lat")), float(n.get("lon")))
             for n in root.iter("node")}
    ways = {w.get("id"): [nodes[nd.get("ref")] for nd in w.iter("nd")] for w in root.iter("way")}
    out = {}
    for rel in root.iter("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.iter("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype") not in subtypes:
            continue
        members = {m.get("role"): m.get("ref") for m in rel.iter("member") if m.get("type") == "way"}
        if "left" in members and "right" in members:
            left, right = ways[members["left"]], ways[members["right"]]
            if math.dist(left[0], right[0]) > math.dist(left[0], right[-1]):
                left = left[::-1]
            out[rel.get("id")] = (left, right)
    return out


def resample(line, n):
    """`n` points evenly spaced by arc length along a polyline"""
    seg = [math.dist(a, b) for a, b in zip(line, line[1:])]
    total = sum(seg)
    if total == 0.0:
        return [line[0]] * n
    out, i, acc = [], 0, 0.0
    for k in range(n):
        target = total * k / (n - 1)
        while i < len(seg) - 1 and acc + seg[i] < target:
            acc += seg[i]
            i += 1
        t = 0.0 if seg[i] == 0 else min(1.0, (target - acc) / seg[i])
        (ax, ay), (bx, by) = line[i], line[i + 1]
        out.append((ax + t * (bx - ax), ay + t * (by - ay)))
    return out


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


SAMPLE_M = 0.5  # OpenDRIVE lane centres sampled this often, junction connectors included
SEARCH_M = 8.0  # a lanelet point farther than this from every matching lane centre is unmatched
SAME_WAY_DEG = 45.0  # a lane centre matches a lanelet point heading within this of it


def lane_centres(carla_map):
    """[(x, y, yaw rad, lane width, lane type)] in CARLA's frame, every SAMPLE_M along every
    driving lane and the lanes beside it (shoulders, sidewalks, ...)"""
    out, seen = [], set()
    for w in carla_map.generate_waypoints(SAMPLE_M):
        for step in ("get_left_lane", "get_right_lane"):
            n = w
            while n is not None:
                key = (n.road_id, n.section_id, n.lane_id, round(n.s, 2))
                if key in seen and n is not w:
                    break  # across the centre line a left lane's left lane is itself
                seen.add(key)
                t = n.transform
                out.append((t.location.x, t.location.y, math.radians(t.rotation.yaw),
                            n.lane_width, str(n.lane_type)))
                nxt = getattr(n, step)()
                # stop at the road's edge, or where a lane crosses to the other side
                n = nxt if nxt is not None and nxt.road_id == n.road_id else None
    return out


def measure(lanelets, centres, step):
    """one row per sampled centreline point: (lanelet, offset m, heading deg, width diff m,
    OpenDRIVE lane type), against the nearest OpenDRIVE lane centre running the same way
    (junction lanes overlap, so the nearest of all can be a crossing or oncoming one); inf
    and None when none is within SEARCH_M"""
    grid = {}
    for c in centres:
        grid.setdefault((int(c[0] // SEARCH_M), int(c[1] // SEARCH_M)), []).append(c)
    rows = []
    for lid, (left, right) in sorted(lanelets.items()):
        length = max(sum(math.dist(a, b) for a, b in zip(line, line[1:])) for line in (left, right))
        n = max(2, int(length / step) + 1)
        lp, rp = resample(left, n), resample(right, n)
        mids = [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(lp, rp)]
        for k, (x, y) in enumerate(mids):
            a, b = mids[max(0, k - 1)], mids[min(n - 1, k + 1)]
            yaw = -math.atan2(b[1] - a[1], b[0] - a[0])  # CARLA's frame
            cx, cy = x, -y
            gx, gy = int(cx // SEARCH_M), int(cy // SEARCH_M)
            best = None
            for i in (gx - 1, gx, gx + 1):
                for j in (gy - 1, gy, gy + 1):
                    for (wx, wy, wyaw, width, kind) in grid.get((i, j), ()):
                        head = math.degrees(angle_diff(yaw, wyaw))
                        if kind != "Driving":
                            head = min(head, 180.0 - head)  # a walkway has no way
                        if head > SAME_WAY_DEG:
                            continue
                        # across the lane only: a sample beside the point, not ahead of it
                        dx, dy = cx - wx, cy - wy
                        if abs(dx * math.cos(wyaw) + dy * math.sin(wyaw)) > SAMPLE_M:
                            continue
                        d = abs(-dx * math.sin(wyaw) + dy * math.cos(wyaw))
                        if best is None or d < best[0]:
                            best = (d, head, width, kind)
            if best is None:
                rows.append((lid, math.inf, math.inf, math.inf, None))
            else:
                rows.append((lid, best[0], best[1], math.dist(lp[k], rp[k]) - best[2], best[3]))
    return rows


def pct(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, int(q * (len(v) - 1) + 0.5))]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("lanelet2", type=Path)
    ap.add_argument("xodr", type=Path)
    ap.add_argument("--step", type=float, default=1.0)
    ap.add_argument("--worst", type=int, default=10)
    ap.add_argument("--subtypes", nargs="+", default=["road"],
                    help="lanelet subtypes measured (default: road)")
    a = ap.parse_args()
    import carla

    lanelets = parse_lanelets(a.lanelet2, tuple(a.subtypes))
    carla_map = carla.Map(a.xodr.stem, a.xodr.read_text())
    rows = measure(lanelets, lane_centres(carla_map), a.step)
    finite = [r for r in rows if math.isfinite(r[1])]
    print(f"{len(lanelets)} lanelets ({', '.join(a.subtypes)}), {len(rows)} points every "
          f"{a.step} m, {len(rows) - len(finite)} with no same-way OpenDRIVE lane within "
          f"{SEARCH_M} m")
    # the lane type each lanelet lies on: its points' majority
    kinds = {}
    for r in finite:
        kinds.setdefault(r[0], Counter())[r[4]] += 1
    on = {lid: c.most_common(1)[0][0] for lid, c in kinds.items()}
    print("  lanelets by the OpenDRIVE lane type they lie on: "
          + ", ".join(f"{k} {v}" for k, v in Counter(on.values()).most_common()))
    for kind in sorted(set(on.values())):
        pts = [r for r in finite if on[r[0]] == kind]
        print(f"  on {kind} ({len(pts)} points):")
        for name, col in (("offset m", 1), ("heading deg", 2), ("width diff m", 3)):
            v = [r[col] for r in pts]
            print(f"    {name:13s} p50 {pct(v, .5):.3f}  p95 {pct(v, .95):.3f}  "
                  f"p99 {pct(v, .99):.3f}  max {max(v):.3f}  mean {statistics.fmean(v):.3f}")
    worst = {}
    for lid, off, head, _, kind in finite:
        worst[lid] = max(worst.get(lid, (0, 0, kind)), (off, head, kind))
    print(f"  worst {a.worst} lanelets by offset (id: offset m, heading deg, lane type there):")
    for lid, (off, head, kind) in sorted(worst.items(), key=lambda kv: -kv[1][0])[:a.worst]:
        print(f"    {lid}: {off:.3f}, {head:.1f}, {kind}")

if __name__ == "__main__":
    main()
