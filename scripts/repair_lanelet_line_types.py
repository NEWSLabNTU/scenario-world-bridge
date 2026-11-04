#!/usr/bin/env python3
"""Give the map pack's double broken lane lines a line type Lanelet2 knows.

The map pack's converter writes OpenDRIVE's `broken broken` road mark as a `line_thin` way
with `subtype=dashed_dashed`. Lanelet2's traffic rules know `dashed`, `solid`,
`solid_solid`, `dashed_solid` and `solid_dashed`; a line of any other subtype permits no
lane change, so the routing graph has no left or right neighbour across it. In Town05 those
are the 158 lines between two lanes of the same direction (Town01 has 26), so a car can
never change into its road's inner lane. At the junction near (28, -3) the west arm's
left-turn lane (769, the only one joined to the left-turn connector 116686) is reached only
from the outer lane 879, and Autoware routed a car from 879 to the north exit straight on
and round the block instead of turning left (found by the crdt-tensor testbed, its round
58). With the line as `dashed` Lanelet2 routes 879 -> 769 -> 116686 -> 30817 -> 30854.

A double broken line permits crossing from both sides, as `dashed` does. Every
`dashed_dashed` line in the pack separates two full-width road lanes (checked in Town05:
none borders a shoulder), so the change opens lane changes only between driving lanes.

Edits replace the subtype tag inside the affected `<way>` blocks, so the rest of the file
stays byte-identical. Idempotent; the first edit keeps a `.orig` copy (shared with the
other repair_lanelet_*.py scripts, whichever runs first).

    scripts/repair_lanelet_line_types.py data/carla-autoware-bridge
"""
import re
import shutil
import sys
from pathlib import Path

RETYPE = {"dashed_dashed": "dashed"}
TAG = re.compile(r'(<way id="[^"]+"[^>]*>(?:(?!</way>).)*?<tag k="subtype" v=")(%s)("\s*/>)'
                 % "|".join(map(re.escape, RETYPE)), re.S)


def repair(path):
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    new, n = TAG.subn(lambda m: m.group(1) + RETYPE[m.group(2)] + m.group(3), text)
    if not n:
        return 0
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(new, encoding="utf-8", errors="surrogateescape")
    return n


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        n = repair(m)
        total += n
        print(f"  {m.parent.name}: {n} line(s) retyped dashed_dashed -> dashed" if n
              else f"  {m.parent.name}: no line to retype")
    print(f"{total} lane line(s) retyped")


if __name__ == "__main__":
    main()
