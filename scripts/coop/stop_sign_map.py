#!/usr/bin/env python3
"""Roadmap 074 U3: a Town05 Lanelet2 map variant with an all-way stop at junction j979.

A study copy, not swb's default map: the source map directory is read, never written. The output
directory is what `map_path:=<dir>` takes: lanelet2_map.osm (edited), map_projector_info.yaml and
map_config.yaml and LICENSE.md (copied), pointcloud_map.pcd (a symlink to the source's file).

The junction's approach lanelets are found from the map itself: the lanelets with a turn_direction
whose centreline midpoint lies within --radius of the junction centre are the junction's, and
their predecessors are the approaches (j979: two lanes on each of four arms, eight lanelets; the
four E6 named, 5193 / 1412 / 92625 / 2826, must be among them). Each approach lanelet gets

  - a stop line way (type stop_line) across the lane, --setback m before the lanelet's end,
  - a regulatory element (type regulatory_element, subtype traffic_sign) with role `refers` the
    arm's sign way and role `ref_line` that stop line, linked from the lanelet relation
    (member type relation, role regulatory_element);

and each arm one sign way (type traffic_sign, subtype stop_sign, a 0.6 m horizontal line 1 m off
the rightmost lane's right edge, 1.5 m up, with a height tag). That is what Autoware 1.5.0's
stop_line module reads (autoware_core 1.5.0, behavior_velocity stop_line_module manager.cpp:
each lanelet's regulatoryElementsAs<lanelet::TrafficSign>(), type() == "stop_sign", its
refLines()) and what SSv2's hdmap_utils reads for NPCs' stop lines. The intersection module then
yields at the junction as without a light.

New ids start above the source map's largest id (nodes, ways and relations share lanelet2's id
space). The output is always rebuilt from the source, so a rerun writes the same files. The OSM
is edited as text: new nodes, ways and relations go in before </osm>, and the member lines into
the eight lanelet relations; nothing else in the file changes.

Usage:
  scripts/coop/stop_sign_map.py [--src <map dir>] [--out <dir>] [--plot <png>]
  (--src defaults to $CARLA_MAPS/Town05 or data/carla-autoware-bridge/Town05, --out to
  /mnt/p5plus/aeon/coop/u3_map/Town05)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CENTRE_ROS = (-127.3, -0.6)  # j979, CARLA (-127.3, 0.6)
E6_APPROACHES = {5193, 1412, 92625, 2826}  # sites.py's approach lanelet per arm (roads 49, 1, 50, 2)
TAG = "swb_u3"  # marks every element this script adds
COPY = ("map_projector_info.yaml", "map_config.yaml", "LICENSE.md")
LINK = ("pointcloud_map.pcd",)


def load(path):
    import lanelet2
    from lanelet2.io import Origin
    from lanelet2.projection import UtmProjector
    m, errs = lanelet2.io.loadRobust(path, UtmProjector(Origin(0, 0)))
    for pt in m.pointLayer:
        if "local_x" in pt.attributes:
            pt.x = float(pt.attributes["local_x"])
            pt.y = float(pt.attributes["local_y"])
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.Vehicle)
    return m, errs, lanelet2.routing.RoutingGraph(m, rules)


def junction(m, g, centre, radius):
    """The junction's lanelets (turn_direction, midpoint within radius) and their predecessors."""
    jl = []
    for ll in m.laneletLayer:
        c = ll.centerline
        mid = c[len(c) // 2]
        if "turn_direction" in ll.attributes and math.hypot(mid.x - centre[0], mid.y - centre[1]) < radius:
            jl.append(ll)
    inside = {ll.id for ll in jl}
    app = {}
    for ll in jl:
        for p in g.previous(ll):
            if p.id not in inside:
                app[p.id] = p
    return jl, [app[k] for k in sorted(app)]


def back_along(ls, s):
    """The point s metres back from the end of a linestring, (x, y, z)."""
    pts = [(p.x, p.y, p.z) for p in ls]
    for i in range(len(pts) - 1, 0, -1):
        (x0, y0, z0), (x1, y1, z1) = pts[i - 1], pts[i]
        d = math.hypot(x1 - x0, y1 - y0)
        if d >= s and d > 0:
            f = s / d
            return x1 + (x0 - x1) * f, y1 + (y0 - y1) * f, z1 + (z0 - z1) * f
        s -= d
    return pts[0]


def heading(ll):
    c = ll.centerline
    p, q = c[len(c) - 2], c[len(c) - 1]
    return math.atan2(q.y - p.y, q.x - p.x)


def plan(m, approaches, setback):
    """The stop lines per approach, the arms (grouped by end heading) and a sign per arm."""
    lines = []
    for ll in approaches:
        lines.append({"lanelet": ll.id, "heading": heading(ll),
                      "left": back_along(ll.leftBound, setback), "right": back_along(ll.rightBound, setback)})
    arms = []
    for ln in lines:
        for arm in arms:
            if abs((ln["heading"] - arm["heading"] + math.pi) % (2 * math.pi) - math.pi) < math.radians(20):
                arm["lines"].append(ln)
                break
        else:
            arms.append({"heading": ln["heading"], "lines": [ln]})
    for arm in arms:
        h = arm["heading"]
        rx, ry = math.sin(h), -math.cos(h)  # right of the driving direction (ROS frame)
        # the rightmost lane's right edge: the largest offset along the right normal
        edge = max((ln["right"] for ln in arm["lines"]), key=lambda p: p[0] * rx + p[1] * ry)
        a = (edge[0] + rx * 1.0, edge[1] + ry * 1.0, edge[2] + 1.5)
        b = (a[0] + rx * 0.6, a[1] + ry * 0.6, a[2])
        arm["sign"] = (a, b)
    return lines, arms


def latlon(x, y):
    from pyproj import Transformer
    t = latlon.t = getattr(latlon, "t", None) or Transformer.from_crs(
        "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84", "EPSG:4326", always_xy=True)
    lon, lat = t.transform(x, y)
    return lat, lon


def build_osm(text, lines, arms):
    ids = [int(i) for i in re.findall(r'<(?:node|way|relation) id="(-?\d+)"', text)]
    nxt = max(ids) + 1
    first = nxt
    nodes, ways, rels, members = [], [], [], {}
    added = {"first_id": first, "arms": []}

    def node(p):
        nonlocal nxt
        i, nxt = nxt, nxt + 1
        lat, lon = latlon(p[0], p[1])
        nodes.append(f'  <node id="{i}" visible="true" version="1" lat="{lat!r}" lon="{lon!r}">\n'
                     f'    <tag k="local_x" v="{p[0]:.4f}"/>\n    <tag k="local_y" v="{p[1]:.4f}"/>\n'
                     f'    <tag k="ele" v="{p[2]:.4f}"/>\n    <tag k="{TAG}" v="j979"/>\n  </node>\n')
        return i

    def way(nds, tags):
        nonlocal nxt
        i, nxt = nxt, nxt + 1
        body = "".join(f'    <nd ref="{n}"/>\n' for n in nds)
        body += "".join(f'    <tag k="{k}" v="{v}"/>\n' for k, v in tags)
        ways.append(f'  <way id="{i}" visible="true" version="1">\n{body}    <tag k="{TAG}" v="j979"/>\n  </way>\n')
        return i

    for arm in arms:
        a, b = arm["sign"]
        sign = way([node(a), node(b)], [("type", "traffic_sign"), ("subtype", "stop_sign"), ("height", "0.6")])
        rec = {"heading_deg": round(math.degrees(arm["heading"]), 1), "sign_way": sign, "lanes": []}
        for ln in arm["lines"]:
            stop = way([node(ln["left"]), node(ln["right"])], [("type", "stop_line")])
            reg, nxt_ = nxt, nxt + 1
            nxt = nxt_
            rels.append(f'  <relation id="{reg}" visible="true" version="1">\n'
                        f'    <member type="way" ref="{sign}" role="refers"/>\n'
                        f'    <member type="way" ref="{stop}" role="ref_line"/>\n'
                        f'    <tag k="type" v="regulatory_element"/>\n    <tag k="subtype" v="traffic_sign"/>\n'
                        f'    <tag k="{TAG}" v="j979"/>\n  </relation>\n')
            members[ln["lanelet"]] = reg
            rec["lanes"].append({"lanelet": ln["lanelet"], "stop_line_way": stop, "regulatory_element": reg,
                                 "stop_line": [[round(c, 3) for c in ln["left"]], [round(c, 3) for c in ln["right"]]]})
        added["arms"].append(rec)
    added["last_id"] = nxt - 1

    for lid, reg in members.items():
        pat = re.compile(r'(  <relation id="%d"[^>]*>\n(?:    <member [^\n]*\n)*)' % lid)
        hits = pat.findall(text)
        if len(hits) != 1:
            sys.exit(f"stop_sign_map: lanelet relation {lid} found {len(hits)} times")
        text = pat.sub(lambda mo: mo.group(1) + f'    <member type="relation" ref="{reg}" role="regulatory_element"/>\n',
                       text, count=1)
    end = text.rindex("</osm>")
    text = text[:end] + "".join(nodes) + "".join(ways) + "".join(rels) + text[end:]
    return text, added


def verify(out_osm, approaches, added, src_errs):
    """Reload the output: every approach lanelet holds one stop_sign traffic_sign whose stop line
    crosses its centreline, no other lanelet holds one, and the load errors are the source's."""
    import lanelet2
    m, errs, _ = load(out_osm)
    want = {ln["lanelet"]: ln for arm in added["arms"] for ln in arm["lanes"]}
    report = {"load_errors": len(errs), "source_load_errors": src_errs, "lanes": []}
    for ll in m.laneletLayer:
        signs = [r for r in ll.regulatoryElements if isinstance(r, lanelet2.core.TrafficSign)]
        stop = [r for r in signs if r.type() == "stop_sign"]
        if ll.id not in want:
            if any(TAG in r.attributes for r in stop):
                sys.exit(f"verify: lanelet {ll.id} got a j979 stop sign")
            continue
        if len(stop) != 1:
            sys.exit(f"verify: lanelet {ll.id} has {len(stop)} stop_sign elements")
        r = stop[0]
        lines = r.refLines()
        if r.id != want[ll.id]["regulatory_element"] or [l.id for l in lines] != [want[ll.id]["stop_line_way"]]:
            sys.exit(f"verify: lanelet {ll.id}: element {r.id}, ref lines {[l.id for l in lines]}")
        crosses = lanelet2.geometry.intersects2d(lanelet2.geometry.to2D(lines[0]), lanelet2.geometry.to2D(ll.centerline))
        d_end = lanelet2.geometry.distance(lanelet2.geometry.to2D(lines[0]), lanelet2.geometry.to2D(ll.centerline[len(ll.centerline) - 1]))
        if not crosses:
            sys.exit(f"verify: lanelet {ll.id}: stop line {lines[0].id} does not cross its centreline")
        report["lanes"].append({"lanelet": ll.id, "regulatory_element": r.id, "type": r.type(),
                                "sign_way": r.trafficSigns()[0].id, "stop_line": lines[0].id,
                                "crosses_centreline": crosses, "to_lanelet_end_m": round(d_end, 2),
                                "other_regs": [f"{x.attributes['subtype']}:{x.id}" for x in ll.regulatoryElements if x.id != r.id]})
    if len(report["lanes"]) != len(want) or len(errs) != src_errs:
        sys.exit(f"verify: {len(report['lanes'])} of {len(want)} lanelets, load errors {len(errs)} vs {src_errs}")
    return report


def plot(m, jl, approaches, out_osm, png, centre):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    m2, _, _ = load(out_osm)
    fig, ax = plt.subplots(figsize=(10, 10))
    r = 30
    for ll in m2.laneletLayer:
        c = ll.centerline
        if min(math.hypot(p.x - centre[0], p.y - centre[1]) for p in c) > r + 10:
            continue
        is_app = ll.id in {a.id for a in approaches}
        col = "tab:blue" if is_app else ("0.55" if ll.id in {j.id for j in jl} else "0.8")
        for b in (ll.leftBound, ll.rightBound):
            ax.plot([p.x for p in b], [p.y for p in b], color=col, lw=1.0 if is_app else 0.7, zorder=3 if is_app else 1)
        if ll.id in {a.id for a in approaches}:
            q = c[len(c) - 1]
            ax.annotate(str(ll.id), (q.x, q.y), fontsize=7, color="tab:blue",
                        xytext=(-40 * math.cos(heading(ll)), -40 * math.sin(heading(ll))), textcoords="offset points",
                        ha="center")
    for reg in m2.regulatoryElementLayer:
        if TAG not in reg.attributes:
            continue
        for ls in reg.refLines():
            ax.plot([p.x for p in ls], [p.y for p in ls], color="tab:red", lw=2.5)
        for s in reg.trafficSigns():
            xs = sum(p.x for p in s) / len(s)
            ys = sum(p.y for p in s) / len(s)
            ax.plot(xs, ys, marker="8", ms=13, color="tab:red", mec="white")
            ax.text(xs, ys, "STOP", fontsize=4.5, color="white", ha="center", va="center", weight="bold")
    ax.plot(*centre, "k+", ms=12)
    ax.set_xlim(centre[0] - r, centre[0] + r)
    ax.set_ylim(centre[1] - r, centre[1] + r)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m, ROS map frame = CARLA x)")
    ax.set_ylabel("y (m, ROS map frame = -CARLA y)")
    ax.set_title("Town05 j979 all-way stop: approach lanelets (blue), stop lines (red), stop signs")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png, dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    maps = os.environ.get("CARLA_MAPS") or os.path.join(REPO, "data/carla-autoware-bridge")
    ap.add_argument("--src", default=os.path.join(maps, "Town05"))
    ap.add_argument("--out", default="/mnt/p5plus/aeon/coop/u3_map/Town05")
    ap.add_argument("--radius", type=float, default=12.0, help="m: junction lanelets' midpoints within this")
    ap.add_argument("--setback", type=float, default=0.5, help="m: stop line before the approach lanelet's end")
    ap.add_argument("--plot", default=None, help="png of the junction (default <out>/../j979_stop_signs.png)")
    args = ap.parse_args()
    src, out = os.path.realpath(args.src), os.path.realpath(args.out)
    if src == out:
        sys.exit("stop_sign_map: --out is the source directory; the source map is never written")
    src_osm = os.path.join(src, "lanelet2_map.osm")

    m, errs, g = load(src_osm)
    jl, approaches = junction(m, g, CENTRE_ROS, args.radius)
    ids = {a.id for a in approaches}
    if not E6_APPROACHES <= ids:
        sys.exit(f"stop_sign_map: approaches {sorted(ids)} miss E6's {sorted(E6_APPROACHES - ids)}")
    for a in approaches:
        for r in a.regulatoryElements:
            if r.attributes["subtype"] == "traffic_sign":
                sys.exit(f"stop_sign_map: lanelet {a.id} already has traffic_sign {r.id} in the source")
    lines, arms = plan(m, approaches, args.setback)

    with open(src_osm, encoding="utf-8") as f:
        text = f.read()
    if f'k="{TAG}"' in text:
        sys.exit(f"stop_sign_map: the source already holds {TAG} elements")
    text, added = build_osm(text, lines, arms)

    os.makedirs(out, exist_ok=True)
    tmp = os.path.join(out, "lanelet2_map.osm.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
    os.replace(tmp, os.path.join(out, "lanelet2_map.osm"))
    for name in COPY:
        if os.path.exists(os.path.join(src, name)):
            shutil.copyfile(os.path.join(src, name), os.path.join(out, name))
    for name in LINK:
        dst = os.path.join(out, name)
        if os.path.lexists(dst):
            os.remove(dst)
        os.symlink(os.path.realpath(os.path.join(src, name)), dst)
    added.update({"source": src, "junction": "Town05 j979", "centre_ros": CENTRE_ROS, "setback_m": args.setback,
                  "junction_lanelets": sorted(j.id for j in jl), "approach_lanelets": sorted(ids),
                  "source_load_errors": len(errs)})
    added["verify"] = verify(os.path.join(out, "lanelet2_map.osm"), approaches, added, len(errs))
    meta = os.path.join(os.path.dirname(out), "j979_stop_signs.json")
    with open(meta, "w") as f:
        json.dump(added, f, indent=1)
    png = args.plot or os.path.join(os.path.dirname(out), "j979_stop_signs.png")
    plot(m, jl, approaches, os.path.join(out, "lanelet2_map.osm"), png, CENTRE_ROS)
    print(f"stop_sign_map: {len(arms)} arms, {len(lines)} approach lanelets {sorted(ids)}; ids "
          f"{added['first_id']}..{added['last_id']}; map {out}; {meta}; {png}")


if __name__ == "__main__":
    main()
