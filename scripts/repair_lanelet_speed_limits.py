#!/usr/bin/env python3
"""Give each road lanelet in a Lanelet2 map its OpenDRIVE road's speed limit (`speed_limit`).

The map pack's converter writes no `speed_limit` tag, so Autoware takes lanelet2's default for
the lanelet's `location`: 50 km/h in a town, whatever the road. Town05's ring highway is a
65 mph road in its OpenDRIVE; Autoware drove it at 50 km/h (swb roadmap 057, server-241's lap
10, 2026-10-27).

Each road lanelet is matched to the OpenDRIVE road whose reference line is nearest the middle of
its centre line, heading along it (either direction: a road's left lanes run against its
reference line) within 30 deg and within MAX_OFFSET m; the lanelet gets that road's `<type>`
`<speed>` at that point, converted to km/h (lanelet2's unit for a bare number). A road with no
speed record, or a lanelet matched to none, is left as it was (the default applies). The
geometry is OpenDRIVE's line and arc, all CARLA's towns use; the map frame is OpenDRIVE's
(the local coordinates after repair_lanelet_local_coords.py, which must run first).

Edits are text insertions into the matched `<relation>` blocks, so the rest of the file stays
byte-identical. Idempotent (a lanelet with a speed_limit is not changed); the first edit keeps a
`.orig` copy (shared with the other repair_lanelet_*.py scripts, whichever runs first).

    scripts/repair_lanelet_speed_limits.py data/carla-autoware-bridge --xodr-dir <dir>

<dir> holds `<Town>.xodr` (CARLA's Content/Carla/Maps/OpenDrive) or `<Town>/main.xodr` (the Isaac
asset set); default $CARLA_SRC/Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive. The map pack's
Town10 is OpenDRIVE's Town10HD.
"""
import argparse
import math
import os
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

MAX_OFFSET = 20.0      # m from a road's reference line: the widest CARLA road's lanes
MAX_HEADING = math.radians(30)
STEP = 1.0             # m between reference-line samples
UNIT = {"mph": 1.609344, "km/h": 1.0, "kmh": 1.0, "m/s": 3.6, "ms": 3.6}
ALIASES = {"Town10": "Town10HD"}


def reference_samples(road):
    """[(x, y, heading, s)] along a road's reference line, every STEP m."""
    out = []
    for g in road.findall("planView/geometry"):
        s0, x0, y0 = float(g.get("s")), float(g.get("x")), float(g.get("y"))
        h0, length = float(g.get("hdg")), float(g.get("length"))
        arc = g.find("arc")
        k = float(arc.get("curvature")) if arc is not None else 0.0
        n = max(1, int(length / STEP))
        for i in range(n + 1):
            u = length * i / n
            if abs(k) < 1e-9:
                x, y, h = x0 + u * math.cos(h0), y0 + u * math.sin(h0), h0
            else:
                h = h0 + k * u
                x = x0 + (math.sin(h) - math.sin(h0)) / k
                y = y0 - (math.cos(h) - math.cos(h0)) / k
            out.append((x, y, h, s0 + u))
    return out


def road_speeds(road):
    """[(s, km/h)] from a road's <type> records, by s."""
    out = []
    for t in road.findall("type"):
        sp = t.find("speed")
        if sp is not None and sp.get("max") not in (None, "", "no limit", "undefined"):
            out.append((float(t.get("s", 0.0)), float(sp.get("max")) * UNIT.get(sp.get("unit", "m/s"), 3.6)))
    return sorted(out)


def speed_at(speeds, s):
    best = None
    for s0, v in speeds:
        if s0 <= s + 1e-6:
            best = v
    return best


def roads_of(xodr):
    root = ET.parse(xodr).getroot()
    return [(r.get("id"), reference_samples(r), road_speeds(r)) for r in root.findall("road")]


def lanelet_middles(path):
    """{relation id: (x, y, heading)} for each road lanelet without a speed_limit."""
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if "local_x" in tags:
            nodes[n.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    out = {}
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet" or tags.get("subtype", "road") != "road" or "speed_limit" in tags:
            continue
        m = {mb.get("role"): mb.get("ref") for mb in rel.findall("member")}
        left = [nodes[i] for i in ways.get(m.get("left"), []) if i in nodes]
        right = [nodes[i] for i in ways.get(m.get("right"), []) if i in nodes]
        if len(left) < 2 or len(right) < 2:
            continue
        a = [(l[0] + r[0]) / 2 for l, r in ((left[0], right[0]), (left[-1], right[-1]))]
        b = [(l[1] + r[1]) / 2 for l, r in ((left[0], right[0]), (left[-1], right[-1]))]
        mid_l, mid_r = left[len(left) // 2], right[len(right) // 2]
        out[rel.get("id")] = ((mid_l[0] + mid_r[0]) / 2, (mid_l[1] + mid_r[1]) / 2,
                              math.atan2(b[1] - b[0], a[1] - a[0]))
    return out


def match(middles, roads):
    """{relation id: km/h} for the lanelets whose matched road has a speed there."""
    out = {}
    for rid, (x, y, h) in middles.items():
        best = None
        for _, samples, speeds in roads:
            if not speeds:
                continue
            for sx, sy, sh, s in samples:
                d = math.hypot(sx - x, sy - y)
                if d > MAX_OFFSET or (best and d >= best[0]):
                    continue
                dh = abs(math.remainder(h - sh, math.pi))
                if dh <= MAX_HEADING:
                    best = (d, speed_at(speeds, s))
        if best and best[1]:
            out[rid] = best[1]
    return out


def add_speed(text, rel_id, kmh):
    block = re.compile(r'(<relation id="%s"[^>]*>)(.*?)(</relation>)' % re.escape(rel_id), re.S)
    m = block.search(text)
    if not m:
        return text, False
    body = m.group(2)
    tag = re.search(r'([ \t]*)<tag k="type" v="lanelet"\s*/>', body)
    if not tag:
        return text, False
    insert = '%s<tag k="speed_limit" v="%.1f"/>\n' % (tag.group(1), kmh)
    body = body[:tag.start()] + insert + body[tag.start():]
    return text[:m.start(2)] + body + text[m.end(2):], True


def repair(path, xodr):
    speeds = match(lanelet_middles(path), roads_of(xodr))
    if not speeds:
        return {}
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    done = {}
    for rel_id, kmh in speeds.items():
        text, ok = add_speed(text, rel_id, kmh)
        if ok:
            done[rel_id] = kmh
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
        kinds = sorted({round(v, 1) for v in done.values()})
        print(f"  {town}: {len(done)} lanelet(s) given a speed_limit ({', '.join(f'{k} km/h' for k in kinds)})"
              if done else f"  {town}: nothing to tag")
    print(f"{total} lanelet speed limit(s) set")


if __name__ == "__main__":
    main()
