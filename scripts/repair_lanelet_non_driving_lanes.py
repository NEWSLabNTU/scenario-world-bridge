#!/usr/bin/env python3
"""Close to cars the road lanelets that lie on an OpenDRIVE lane that is not a driving lane.

The map pack's converter turned some non-driving OpenDRIVE lanes into ordinary road lanelets.
On Town03's east straight the "innermost" lanelet (68766) is CARLA's lane 1 of road 65, a
`bidirectional` lane that is the tunnel's divider, with walls in it from y = 62 on; the lanelet
map let Autoware change lanes into it, and its AEB stopped the car at the walls over and over
(swb roadmap 074 E2, 2026-11-13). CARLA's own driving lanes never use such lanes.

Each road lanelet is matched by the middle of its centre line to the nearest OpenDRIVE road
reference line heading along it (as repair_lanelet_speed_limits.py does), the lateral offset
there is located among that lane section's lanes (laneOffset and lane widths), and a lanelet
whose lane is of a type in CLOSED gets `participant:vehicle=no`: lanelet2's traffic rules then
leave it out of every vehicle routing graph, so no route and no lane change goes into it. The
lanelet stays in the map (for its neighbours' bounds and any references to it).

Edits are text insertions into the matched `<relation>` blocks; idempotent (a lanelet with a
`participant:vehicle` tag is left as it is); the first edit keeps a `.orig` copy (shared with
the other repair_lanelet_*.py scripts). Runs after repair_lanelet_local_coords.py (OpenDRIVE's
frame).

    scripts/repair_lanelet_non_driving_lanes.py data/carla-autoware-bridge --xodr-dir <dir>
"""
import argparse
import math
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from repair_lanelet_speed_limits import MAX_HEADING, MAX_OFFSET, find_xodr, reference_samples  # noqa: E402

CLOSED = {"bidirectional", "median"}


def poly(entries, s):
    """A cubic OpenDRIVE record list [(s0, a, b, c, d)] evaluated at s (the last s0 <= s)."""
    cur = None
    for e in entries:
        if e[0] <= s + 1e-9:
            cur = e
    if cur is None:
        return 0.0
    ds = s - cur[0]
    return cur[1] + cur[2] * ds + cur[3] * ds * ds + cur[4] * ds ** 3


def _cubic(el, key):
    return (float(el.get(key)), float(el.get("a")), float(el.get("b")), float(el.get("c")), float(el.get("d")))


def road_lanes(road):
    """(laneOffset records, [(section s, {lane id: (type, width records)})])."""
    lanes = road.find("lanes")
    offsets = sorted(_cubic(o, "s") for o in lanes.findall("laneOffset"))
    sections = []
    for sec in lanes.findall("laneSection"):
        s0 = float(sec.get("s"))
        ids = {}
        for side in ("left", "right"):
            for ln in sec.findall(f"{side}/lane"):
                widths = sorted((s0 + w[0],) + w[1:] for w in (_cubic(w, "sOffset") for w in ln.findall("width")))
                ids[int(ln.get("id"))] = (ln.get("type"), widths)
        sections.append((s0, ids))
    return offsets, sorted(sections, key=lambda x: x[0])


def lane_at(lanes, s, t):
    """(lane id, type) at station s and lateral offset t (left positive), or None."""
    offsets, sections = lanes
    sec = None
    for s0, ids in sections:
        if s0 <= s + 1e-9:
            sec = ids
    if not sec:
        return None
    edge = poly(offsets, s)
    step = 1 if t >= edge else -1
    i = step
    while i in sec:
        kind, widths = sec[i]
        w = max(0.0, poly(widths, s))
        if (step > 0 and t < edge + w) or (step < 0 and t > edge - w):
            return i, kind
        edge += step * w
        i += step
    return None


def roads_of(xodr):
    root = ET.parse(xodr).getroot()
    return [(r.get("id"), reference_samples(r), road_lanes(r)) for r in root.findall("road")]


def lanelet_middles(path):
    """{relation id: (x, y, heading)} for each road lanelet with no participant:vehicle tag."""
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
        if tags.get("type") != "lanelet" or tags.get("subtype", "road") != "road" or "participant:vehicle" in tags:
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
    """{relation id: (road id, lane id, lane type)} for lanelets on a lane of a CLOSED type."""
    out = {}
    for rid, (x, y, h) in middles.items():
        best = None
        for road_id, samples, lanes in roads:
            for sx, sy, sh, s in samples:
                d = math.hypot(sx - x, sy - y)
                if d > MAX_OFFSET or (best and d >= best[0]):
                    continue
                if abs(math.remainder(h - sh, math.pi)) <= MAX_HEADING:
                    t = -(x - sx) * math.sin(sh) + (y - sy) * math.cos(sh)
                    best = (d, road_id, lanes, s, t)
        if not best:
            continue
        _, road_id, lanes, s, t = best
        lane = lane_at(lanes, s, t)
        if lane and lane[1] in CLOSED:
            out[rid] = (road_id, lane[0], lane[1])
    return out


def close(text, rel_id):
    block = re.compile(r'(<relation id="%s"[^>]*>)(.*?)(</relation>)' % re.escape(rel_id), re.S)
    m = block.search(text)
    if not m:
        return text, False
    body = m.group(2)
    tag = re.search(r'([ \t]*)<tag k="type" v="lanelet"\s*/>', body)
    if not tag:
        return text, False
    insert = '%s<tag k="participant:vehicle" v="no"/>\n' % tag.group(1)
    body = body[:tag.start()] + insert + body[tag.start():]
    return text[:m.start(2)] + body + text[m.end(2):], True


def repair(path, xodr):
    found = match(lanelet_middles(path), roads_of(xodr))
    if not found:
        return {}
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    done = {}
    for rel_id, where in found.items():
        text, ok = close(text, rel_id)
        if ok:
            done[rel_id] = where
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return done


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("root", nargs="?", default="data/carla-autoware-bridge")
    default = Path(os.environ.get("CARLA_SRC", "/mnt/disk1/carla-src"),
                   "carla/Unreal/CarlaUE4/Content/Carla/Maps/OpenDrive")
    ap.add_argument("--xodr-dir", type=Path, default=default)
    ap.add_argument("--dry-run", action="store_true", help="list what would be closed, change nothing")
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
        done = match(lanelet_middles(m), roads_of(xodr)) if args.dry_run else repair(m, xodr)
        total += len(done)
        print(f"  {town}: {len(done)} lanelet(s) on non-driving lanes closed to vehicles"
              + (": " + ", ".join(f"{k} (road {r} lane {l} {t})" for k, (r, l, t) in sorted(done.items())) if done else ""))
    print(f"{total} lanelet(s) closed{' (dry run)' if args.dry_run else ''}")


if __name__ == "__main__":
    main()
