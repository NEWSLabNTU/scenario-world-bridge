#!/usr/bin/env python3
"""Retag the walkways and crosswalks that pass under a raised road, so Autoware stops for neither.

The map pack's Lanelet2 maps are flat (every node at z 0), so a walkway that runs under a
bridge crosses the bridge's lanes in the map. Autoware's walkway and crosswalk modules then
stop the car on the bridge for a footpath 10 m below it: Town05's ring highway rises to 10 m
on its west side (OpenDRIVE road 37), and 13 walkway lanelets under it gave a stop-and-go every
8-15 m (swb roadmap 057, server-241's lap 12, the walkway module's "sidewalk" velocity factor).

A `walkway` or `crosswalk` lanelet is retagged when its centre line passes inside a road
lanelet at a point where an OpenDRIVE reference line within NEAR m stands more than LEVEL m
above the walkway's own road (the reference line nearest the walkway's point): one level over
the other. It gets `subtype=unknown` -- kept in the map, ignored by Autoware's walkway and
crosswalk modules and by its prediction, as repair_lanelet_subtypes.py does for strips with no
subtype -- and `swb:walkway_under_road=<its old subtype>`. A crossing at one level (an ordinary
crosswalk) is not touched. The geometry is OpenDRIVE's line and arc and its cubic elevation
profile; the map frame is OpenDRIVE's (local coordinates after repair_lanelet_local_coords.py).

Edits are text edits inside the retagged `<relation>` blocks; the rest of the file stays
byte-identical. Idempotent; the first edit keeps a `.orig` copy (shared with the other
repair_lanelet_*.py scripts).

    scripts/repair_lanelet_walkways_under_roads.py data/carla-autoware-bridge --xodr-dir <dir>

<dir> as repair_lanelet_speed_limits.py's: `<Town>.xodr` or `<Town>/main.xodr` (default
$CARLA_SRC/carla/Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive).
"""
import argparse
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

LEVEL = 3.0     # m between the two levels
NEAR = 15.0     # m from the walkway's point to a reference line of the road over it
CELL = 20.0     # m, the spatial hash
STEP = 2.0      # m between reference-line samples
ALIASES = {"Town10": "Town10HD"}


def elevation(road):
    prof = sorted((float(e.get("s")), [float(e.get(k)) for k in "abcd"])
                  for e in road.findall("elevationProfile/elevation"))

    def z(s):
        best = None
        for s0, c in prof:
            if s0 <= s + 1e-6:
                best = (s0, c)
        if best is None:
            return 0.0
        ds = s - best[0]
        a, b, c, d = best[1]
        return a + b * ds + c * ds * ds + d * ds ** 3
    return z


def samples(xodr):
    """[(x, y, z, road id)] along every road's reference line."""
    out = []
    for road in ET.parse(xodr).getroot().findall("road"):
        z = elevation(road)
        for g in road.findall("planView/geometry"):
            s0, x0, y0 = float(g.get("s")), float(g.get("x")), float(g.get("y"))
            h0, length = float(g.get("hdg")), float(g.get("length"))
            arc = g.find("arc")
            k = float(arc.get("curvature")) if arc is not None else 0.0
            n = max(1, int(length / STEP))
            for i in range(n + 1):
                u = length * i / n
                if abs(k) < 1e-9:
                    x, y = x0 + u * math.cos(h0), y0 + u * math.sin(h0)
                else:
                    h = h0 + k * u
                    x = x0 + (math.sin(h) - math.sin(h0)) / k
                    y = y0 - (math.cos(h) - math.cos(h0)) / k
                out.append((x, y, z(s0 + u), road.get("id")))
    return out


def grid(items, key):
    g = defaultdict(list)
    for it in items:
        x, y = key(it)
        g[(int(x // CELL), int(y // CELL))].append(it)
    return g


def around(g, x, y, r=1):
    cx, cy = int(x // CELL), int(y // CELL)
    for i in range(cx - r, cx + r + 1):
        for j in range(cy - r, cy + r + 1):
            yield from g.get((i, j), ())


def inside(poly, x, y):
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def along(poly, u):
    """The point a share u of the way along a polyline."""
    d = [0.0]
    for a, b in zip(poly, poly[1:]):
        d.append(d[-1] + math.dist(a, b))
    target = u * d[-1]
    for i in range(len(poly) - 1):
        if d[i + 1] >= target:
            f = (target - d[i]) / max(d[i + 1] - d[i], 1e-9)
            return (poly[i][0] + f * (poly[i + 1][0] - poly[i][0]),
                    poly[i][1] + f * (poly[i + 1][1] - poly[i][1]))
    return poly[-1]


def centre_line(left, right, step=1.0):
    """The lanelet's centre line, a point every `step` m (its bounds' nodes can be 20 m apart)."""
    length = max(sum(math.dist(a, b) for a, b in zip(w, w[1:])) for w in (left, right))
    n = max(2, int(length / step))
    out = []
    for i in range(n + 1):
        a, b = along(left, i / n), along(right, i / n)
        out.append(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
    return out


def lanelets(path):
    """(road polygons, walkways {id: (subtype, centre points)}) of a map."""
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.findall("node"):
        t = {k.get("k"): k.get("v") for k in n.findall("tag")}
        if "local_x" in t:
            nodes[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    ways = {w.get("id"): [nodes[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in nodes]
            for w in root.findall("way")}
    roads, walks = [], {}
    for rel in root.findall("relation"):
        t = {k.get("k"): k.get("v") for k in rel.findall("tag")}
        if t.get("type") != "lanelet":
            continue
        m = {mb.get("role"): mb.get("ref") for mb in rel.findall("member")}
        left, right = ways.get(m.get("left"), []), ways.get(m.get("right"), [])
        if len(left) < 2 or len(right) < 2:
            continue
        sub = t.get("subtype", "road")
        if sub == "road":
            poly = left + right[::-1]
            xs, ys = [p[0] for p in poly], [p[1] for p in poly]
            roads.append((poly, (min(xs), min(ys), max(xs), max(ys))))
        elif sub in ("walkway", "crosswalk"):
            walks[rel.get("id")] = (sub, centre_line(left, right))
    return roads, walks


def under_roads(path, xodr):
    """{relation id: subtype} of the walkways and crosswalks that pass under a raised road."""
    roads, walks = lanelets(path)
    road_grid = defaultdict(list)
    for poly, (x0, y0, x1, y1) in roads:
        for i in range(int(x0 // CELL), int(x1 // CELL) + 1):
            for j in range(int(y0 // CELL), int(y1 // CELL) + 1):
                road_grid[(i, j)].append((poly, (x0, y0, x1, y1)))
    ref = grid(samples(xodr), lambda s: (s[0], s[1]))
    out = {}
    for rid, (sub, pts) in walks.items():
        for x, y in pts:
            cell = road_grid.get((int(x // CELL), int(y // CELL)), ())
            if not any(b[0] <= x <= b[2] and b[1] <= y <= b[3] and inside(p, x, y) for p, b in cell):
                continue
            near = [(math.hypot(s[0] - x, s[1] - y), s[2]) for s in around(ref, x, y)]
            near = [n for n in near if n[0] <= NEAR]
            if not near:
                continue
            own = min(near)[1]  # the walkway's own road: the nearest reference line
            if max(z for _, z in near) - own > LEVEL:
                out[rid] = sub
                break
    return out


def retag(text, rel_id, old):
    block = re.compile(r'(<relation id="%s"[^>]*>)(.*?)(</relation>)' % re.escape(rel_id), re.S)
    m = block.search(text)
    if not m:
        return text, False
    body = m.group(2)
    sub = re.search(r'([ \t]*)<tag k="subtype" v="%s"\s*/>' % re.escape(old), body)
    if not sub:
        return text, False
    new = '%s<tag k="subtype" v="unknown"/>\n%s<tag k="swb:walkway_under_road" v="%s"/>' % (
        sub.group(1), sub.group(1), old)
    body = body[:sub.start()] + new + body[sub.end():]
    return text[:m.start(2)] + body + text[m.end(2):], True


def repair(path, xodr):
    found = under_roads(path, xodr)
    if not found:
        return {}
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    done = {}
    for rid, sub in found.items():
        text, ok = retag(text, rid, sub)
        if ok:
            done[rid] = sub
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return done


def find_xodr(xodr_dir, town):
    name = ALIASES.get(town, town)
    for p in (xodr_dir / f"{name}.xodr", xodr_dir / name / "main.xodr", xodr_dir / town / "main.xodr"):
        if p.exists():
            return p
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("root", nargs="?", default="data/carla-autoware-bridge")
    default = Path(os.environ.get("CARLA_SRC", "/mnt/disk1/carla-src"),
                   "carla/Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive")
    ap.add_argument("--xodr-dir", type=Path, default=default)
    args = ap.parse_args()
    root = Path(args.root)
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        raise SystemExit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        town = m.parent.name
        xodr = find_xodr(args.xodr_dir, town)
        if xodr is None:
            print(f"  {town}: no OpenDRIVE for it under {args.xodr_dir}; skipped")
            continue
        done = repair(m, xodr)
        total += len(done)
        kinds = sorted(set(done.values()))
        print(f"  {town}: {len(done)} lanelet(s) under a raised road retagged ({', '.join(kinds)})"
              if done else f"  {town}: no walkway under a raised road")
    print(f"{total} lanelet(s) retagged")


if __name__ == "__main__":
    main()
