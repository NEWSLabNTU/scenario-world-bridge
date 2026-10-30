#!/usr/bin/env python3
"""Score a Lanelet2 map against what Autoware needs (roadmap 062; the survey's table).

    scripts/open_maps/check_map.py <lanelet2_map.osm> [--json out.json]

Plain XML, no ROS. Reports:

- size: nodes, ways, lanelets, nodes per way (crdesigner's maps are known to hang Autoware on
  node count, Autoware discussion #6019);
- connectivity: successors by shared bound endpoints, dead ends, lanelets with no
  predecessor, branch points (a lanelet with two or more successors starts a junction path);
- junction paths (the successors of a branch point): how many carry `turn_direction`;
- road lanelets with `speed_limit`;
- traffic-light regulatory elements: how many have a stop line (`ref_line`), how many lanelets
  refer to one;
- crosswalks, stop lines, right_of_way elements;
- `swb:source` on lanelets, where the generator wrote it;
- `local_x`/`local_y` on every node.
"""
import argparse
import json
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict


def tags(e):
    return {t.get("k"): t.get("v") for t in e.findall("tag")}


def check(path):
    root = ET.parse(path).getroot()
    nodes = root.findall("node")
    ways = {w.get("id"): [nd.get("ref") for nd in w.findall("nd")] for w in root.findall("way")}
    way_tags = {w.get("id"): tags(w) for w in root.findall("way")}
    lanelets, regs = {}, {}
    for r in root.findall("relation"):
        t = tags(r)
        m = [(x.get("type"), x.get("role"), x.get("ref")) for x in r.findall("member")]
        if t.get("type") == "lanelet":
            lanelets[r.get("id")] = (t, m)
        elif t.get("type") == "regulatory_element":
            regs[r.get("id")] = (t, m)
    road = {k: v for k, v in lanelets.items() if v[0].get("subtype", "road") == "road"}

    def bound(m, role):
        for typ, rl, ref in m:
            if typ == "way" and rl == role:
                return ways.get(ref, [])
        return []
    start, end = {}, {}
    for k, (t, m) in road.items():
        lb, rb = bound(m, "left"), bound(m, "right")
        if lb and rb:
            start[k] = (lb[0], rb[0])
            end[k] = (lb[-1], rb[-1])
    by_start = defaultdict(list)
    for k, s in start.items():
        by_start[s].append(k)
    succ = {k: by_start.get(e, []) for k, e in end.items()}
    pred = Counter(x for v in succ.values() for x in v)
    branch = [k for k, v in succ.items() if len(v) >= 2]
    paths = {x for k in branch for x in succ[k]}
    turn = Counter(road[x][0].get("turn_direction", "-") for x in paths)
    regs_by_sub = Counter(t.get("subtype") for t, _ in regs.values())
    tl = [(t, m) for t, m in regs.values() if t.get("subtype") == "traffic_light"]
    tl_ids = {k for k, (t, _) in regs.items() if t.get("subtype") == "traffic_light"}
    refers_tl = sum(1 for t, m in road.values()
                    if any(typ == "relation" and rl == "regulatory_element" and ref in tl_ids
                           for typ, rl, ref in m))
    stop_lines = sum(1 for t in way_tags.values() if t.get("type") == "stop_line")
    per = [len(v) for v in ways.values()] or [0]
    local = sum(1 for n in nodes if any(t.get("k") == "local_x" for t in n.findall("tag")))
    rep = {
        "nodes": len(nodes), "ways": len(ways), "lanelets": len(lanelets), "road_lanelets": len(road),
        "nodes_per_way_mean": round(sum(per) / len(per), 1), "nodes_per_way_max": max(per),
        "subtypes": dict(Counter(t.get("subtype", "road") for t, _ in lanelets.values())),
        "dead_ends": sum(1 for v in succ.values() if not v),
        "no_predecessor": sum(1 for k in start if pred[k] == 0),
        "branch_points": len(branch), "junction_paths": len(paths),
        "junction_paths_turn_direction": dict(turn),
        "speed_limit": sum(1 for t, _ in road.values() if "speed_limit" in t),
        "regulatory_elements": dict(regs_by_sub),
        "traffic_lights_with_stop_line": sum(1 for _, m in tl if any(rl == "ref_line" for _, rl, _ in m)),
        "lanelets_referring_a_traffic_light": refers_tl,
        "stop_line_ways": stop_lines,
        "swb_source": dict(Counter(t.get("swb:source", "-") for t, _ in road.values())),
        "nodes_with_local_xy": local,
    }
    return rep


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("--json")
    a = ap.parse_args()
    rep = check(a.map)
    for k, v in rep.items():
        print(f"{k:38s} {v}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(rep, f, indent=1)


if __name__ == "__main__":
    main()
