#!/usr/bin/env python3
"""Give each lanelet a left bound that runs the way it drives.

The map pack's converter builds a road's innermost lane on one side from the centre line
the opposite lane already uses as its left bound, and references that way as it is: the
new lanelet's left bound then runs against its right bound. Lanelet2 accepts it (it
inverts the left bound on load), but Autoware's behavior_path_planner does not survive a
route through such a lanelet: given a route through Town05's lanelet 769 its container
died on a signal, with nothing logged, and the ego stood in PLANNING until the scenario
timed out (found by the crdt-tensor testbed, its round 59). These are the lanes
repair_lanelet_line_types.py makes reachable, so the two repairs go together. Town05 has
48 such lanelets, Town01 26, Town02 20, Town03 32, Town04 41 and Town10 19.

A lanelet's left bound is reversed when its ends lie nearer the right bound's opposite
ends. The repair adds a way over the same nodes in the driving order, with the same tags,
and makes it the lanelet's left bound. The nodes are the same, so successors,
predecessors and the opposite lane are untouched; only the reversed reference goes.

Edits are text insertions (one `<way>` per lanelet before `</osm>`, the lanelet's left
member pointed at it), so the rest of the file stays byte-identical. Idempotent; the
first edit keeps a `.orig` copy (shared with the other repair_lanelet_*.py scripts,
whichever runs first).

    scripts/repair_lanelet_bound_directions.py data/carla-autoware-bridge
"""
import math
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def plan(path):
    """[(lanelet id, left way id, its nodes, its tags)] for every reversed left bound."""
    root = ET.parse(path).getroot()
    pos = {}
    for n in root.findall("node"):
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        if "local_x" in tags:
            pos[n.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    wtags = {w.get("id"): {t.get("k"): t.get("v") for t in w.findall("tag")} for w in root.findall("way")}
    out = []
    for rel in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in rel.findall("tag")}
        if tags.get("type") != "lanelet":
            continue
        m = {mb.get("role"): mb.get("ref") for mb in rel.findall("member")}
        if "left" not in m or "right" not in m:
            continue
        left = [pos[i] for i in ways.get(m["left"], []) if i in pos]
        right = [pos[i] for i in ways.get(m["right"], []) if i in pos]
        if len(left) < 2 or len(right) < 2:
            continue
        same = math.dist(left[0], right[0]) + math.dist(left[-1], right[-1])
        crossed = math.dist(left[0], right[-1]) + math.dist(left[-1], right[0])
        if crossed < same:
            out.append((rel.get("id"), m["left"], ways[m["left"]], wtags.get(m["left"], {})))
    ids = [int(x) for x in [*pos, *ways, *(r.get("id") for r in root.findall("relation"))] if x.lstrip("-").isdigit()]
    return out, max(ids)


def repair(path):
    todo, top = plan(path)
    if not todo:
        return 0
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    new = []
    done = 0
    for rel_id, way_id, nodes, tags in todo:
        top += 1
        block = re.compile(r'(<relation id="%s"[^>]*>.*?<member type="way" ref=")%s(" role="left"\s*/>)'
                           % (re.escape(rel_id), re.escape(way_id)), re.S)
        text, n = block.subn(lambda mm, w=top: mm.group(1) + str(w) + mm.group(2), text, count=1)
        if not n:
            continue
        nds = "".join(f'    <nd ref="{i}"/>\n' for i in reversed(nodes))
        tg = "".join(f'    <tag k="{k}" v="{v}"/>\n' for k, v in tags.items())
        new.append(f'  <way id="{top}" version="1">\n{nds}{tg}  </way>\n')
        done += 1
    i = text.rindex("</osm>")
    text = text[:i] + "".join(new) + text[i:]
    path.write_text(text, encoding="utf-8", errors="surrogateescape")
    return done


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        n = repair(m)
        total += n
        print(f"  {m.parent.name}: {n} lanelet(s) given a left bound in their driving direction" if n
              else f"  {m.parent.name}: every left bound runs with its lanelet")
    print(f"{total} lanelet bound(s) turned")


if __name__ == "__main__":
    main()
