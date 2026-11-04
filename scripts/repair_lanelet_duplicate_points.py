#!/usr/bin/env python3
"""Drop the repeated points from lanelet bounds.

The map pack's converter sometimes writes two consecutive points of a lanelet bound at the
same place, as two nodes. Lanelet2 does not mind, but Autoware's map loader resamples each
lanelet's bounds into a fine centre line, and a zero-length segment gives it NaN points
and a NaN length. In Town05 that is lanelets 94727, 94798, 94939, 95010 and 95740 (near
(-126, 77..103)); a NaN length in the lane change module's walk back over the preceding
lanelets keeps the walk from ever reaching its length, so it recursed until
behavior_path_planner's stack overflowed and its container died on SIGSEGV as soon as a
route asked for a lane change nearby (found by the crdt-tensor testbed, its round 59: the
left turn that repair_lanelet_line_types.py opened). Town05 has 240 such segments in 79
bounds, Town03 25 in 13, Town04 6, Town10 12.

The repair drops every point of a lanelet bound that lies within TOL_M of the point
before it, keeping each way's first and last node (when the last two coincide it drops
the one before the last), so the ends lanelets join at are unchanged.

Edits remove `<nd ref=.../>` lines inside the affected `<way>` blocks, so the rest of the
file stays byte-identical. Idempotent; the first edit keeps a `.orig` copy (shared with
the other repair_lanelet_*.py scripts, whichever runs first). Run it after
repair_lanelet_local_coords.py, whose coordinates it compares.

    scripts/repair_lanelet_duplicate_points.py data/carla-autoware-bridge
"""
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

TOL_M = 1e-3


def plan(path):
    """{way id: [the node ids to keep]} for every lanelet bound with a repeated point."""
    root = ET.parse(path).getroot()
    pos = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if "local_x" in tags:
            pos[n.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    bounds = {mb.get("ref") for rel in root.findall("relation") for mb in rel.findall("member")
              if mb.get("role") in ("left", "right") and mb.get("type") == "way"}
    out = {}
    for w in root.findall("way"):
        if w.get("id") not in bounds:
            continue
        ids = [nd.get("ref") for nd in w.findall("nd")]
        if len(ids) < 3 or any(i not in pos for i in ids):
            continue
        keep = [ids[0]]
        for i in ids[1:-1]:
            if math.dist(pos[i], pos[keep[-1]]) >= TOL_M:
                keep.append(i)
        if len(keep) > 1 and math.dist(pos[ids[-1]], pos[keep[-1]]) < TOL_M:
            keep.pop()
        keep.append(ids[-1])
        if len(keep) < len(ids):
            out[w.get("id")] = keep
    return out


def repair(path):
    todo = plan(path)
    if not todo:
        return 0, 0
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    dropped = 0
    for way_id, keep in todo.items():
        block = re.compile(r'(<way id="%s"[^>]*>)(.*?)(</way>)' % re.escape(way_id), re.S)
        m = block.search(text)
        if not m:
            continue
        body = m.group(2)
        wanted = list(keep)
        lines = []
        for line in body.splitlines(keepends=True):
            nd = re.search(r'<nd ref="([^"]+)"', line)
            if nd:
                if wanted and nd.group(1) == wanted[0]:
                    wanted.pop(0)
                else:
                    dropped += 1
                    continue
            lines.append(line)
        text = text[:m.start(2)] + "".join(lines) + text[m.end(2):]
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return len(todo), dropped


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        ways, dropped = repair(m)
        total += dropped
        print(f"  {m.parent.name}: {dropped} repeated point(s) dropped from {ways} bound(s)" if ways
              else f"  {m.parent.name}: no repeated point in any bound")
    print(f"{total} repeated bound point(s) dropped")


if __name__ == "__main__":
    main()
