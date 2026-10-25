#!/usr/bin/env python3
"""Set every node's `local_x` / `local_y` in a Lanelet2 map to its position in Autoware's map
frame.

Autoware and SSv2 place a node from its lat/lon, through the TransverseMercator the map's
`map_projector_info.yaml` declares (`autoware_projection.py`); neither reads the local tags
(Autoware only under a `Local` projector, SSv2 never). swb reads them: swb_core's traffic
light table, the other repair_lanelet_*.py scripts, swb_isaac_world's surface check. So they
must hold the same frame, or swb and Autoware disagree about where a thing is.

The pack's tags did not. They were CARLA's spherical Mercator on R = 6 378 137 m about
(0, 0), x = R lon, y = R ln(tan(pi/4 + lat/2)), 0.7 % long in y against the TransverseMercator
(2.2 m on Town01's y = -330 road), and 50 of Town10's nodes, on stop lines and signs, had
none. Measured on Town01 (docs/design/ssv2-feature-completeness.md, "Lane position"): the
projected lat/lon is the frame of OpenDRIVE's lanes, the point cloud map and the driven ego;
the tags are not.

The repair rewrites both tags on every node from `Projector.project(lat, lon)` (adding them
where absent, as the node's first tags). lat/lon is taken as the truth: a map that
`regenerate_light_geometry.py` or `repair_lanelet_signal_links.py` edited before they wrote
lat/lon with `Projector.unproject` has its new nodes' lat/lon wrong; rerun that script
first. Edits are text edits inside the `<node>` blocks; the rest of the file stays
byte-identical. Idempotent; the first edit keeps a `.orig` copy (shared with the other
repair_lanelet_*.py scripts, whichever runs first).

    scripts/repair_lanelet_local_coords.py data/carla-autoware-bridge
"""
import re
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from autoware_projection import Projector  # noqa: E402

NODE = re.compile(r'<node\b([^>]*?)(/>|>(.*?)</node>)', re.S)
LAT = re.compile(r'\blat="([^"]*)"')
LON = re.compile(r'\blon="([^"]*)"')


def set_local(match, projector):
    """one <node> block with local_x/local_y from its lat/lon"""
    attrs = match.group(1)
    lat, lon = LAT.search(attrs), LON.search(attrs)
    if not (lat and lon):
        return match.group(0)
    x, y = projector.project(float(lat.group(1)), float(lon.group(1)))
    values = {"local_x": "%.4f" % x, "local_y": "%.4f" % y}
    if match.group(3) is None:  # self-closing: <node .../> -> <node ...> tags </node>
        text = match.string
        line_start = text.rfind("\n", 0, match.start()) + 1
        indent = text[line_start:match.start()]
        tags = "".join(f'{indent}  <tag k="{k}" v="{v}"/>\n' for k, v in values.items())
        return f"<node{attrs}>\n{tags}{indent}</node>"
    inner = match.group(3)
    missing = []
    for k, v in values.items():
        tag = re.compile(r'<tag k="%s" v="[^"]*"/>' % k)
        if tag.search(inner):
            inner = tag.sub(f'<tag k="{k}" v="{v}"/>', inner, count=1)
        else:
            missing.append(f'<tag k="{k}" v="{v}"/>')
    if missing:
        nxt = re.match(r'\n([ \t]*)', inner)
        indent = nxt.group(1) if nxt else "    "
        inner = "\n" + "".join(f"{indent}{t}\n" for t in missing)[:-1] + inner
    return f"<node{attrs}>{inner}</node>"


def repair(path):
    """the number of nodes whose local tags changed"""
    projector = Projector.for_map(path)
    text = path.read_text(encoding="utf-8", errors="surrogateescape")
    changed = 0

    def one(m):
        nonlocal changed
        out = set_local(m, projector)
        changed += out != m.group(0)
        return out

    fixed = NODE.sub(one, text)
    if changed:
        backup = path.with_suffix(path.suffix + ".orig")
        if not backup.exists():
            shutil.copy2(path, backup)
        path.write_text(fixed, encoding="utf-8", errors="surrogateescape")
    return changed


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else "data/carla-autoware-bridge")
    maps = [root] if root.is_file() else sorted(root.rglob("lanelet2_map.osm"))
    if not maps:
        sys.exit(f"no lanelet2_map.osm under {root}")
    total = 0
    for m in maps:
        n = repair(m)
        total += n
        print(f"  {m.parent.name}: {n} node(s) given Autoware's local_x/local_y" if n
              else f"  {m.parent.name}: every node's local_x/local_y is Autoware's")
    print(f"{total} node local coordinate(s) repaired")


if __name__ == "__main__":
    main()
