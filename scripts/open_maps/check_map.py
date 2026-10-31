#!/usr/bin/env python3
"""Score a Lanelet2 map against what Autoware needs (roadmap 062; the survey's table).

    scripts/open_maps/check_map.py <lanelet2_map.osm> [--json out.json]

Plain XML, no ROS. Reports:

- size: nodes, ways, lanelets, nodes per way (crdesigner's maps are known to hang Autoware on
  node count, Autoware discussion #6019);
- connectivity: successors by shared bound endpoints, dead ends, lanelets with no
  predecessor, stranded lanes (none from a way into the map, or none to a way out), branch points (a lanelet with two or more successors starts a junction path);
- junction paths (the successors of a branch point): how many carry `turn_direction`;
- road lanelets with `speed_limit`;
- traffic-light regulatory elements: how many have a stop line (`ref_line`), how many lanelets
  refer to one;
- crosswalks, stop lines, right_of_way elements;
- `swb:source` on lanelets, where the generator wrote it;
- `local_x`/`local_y` on every node;
- road lanelets whose left and right bounds cross: Autoware's path ends in one (062, run 7);
- junction paths by their tightest radius (under 4, 5 and 6 m): a car cannot drive one under
  about 5 m, and Autoware's path optimiser stops in it (062, the main gate).
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
    # stranded lanes: a box cuts every road that crosses its edge, so a lane that leads out of
    # it, or comes in from outside, is fine; what is not is a lane no car can reach from any
    # way into the map, or one from which no car can get out of it (the strongly connected
    # part is no measure here: it counts every lane the box cut off from coming back)
    xy = {}
    for nd in nodes:
        t = {x.get("k"): x.get("v") for x in nd.findall("tag")}
        if "local_x" in t:
            xy[nd.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    if xy:
        xs = [p[0] for p in xy.values()]
        ys = [p[1] for p in xy.values()]
        box = (min(xs) + 60, min(ys) + 60, max(xs) - 60, max(ys) - 60)

        def edge(nid):
            x, y = xy[nid]
            return not (box[0] < x < box[2] and box[1] < y < box[3])
        entries = [k for k in start if pred[k] == 0 and edge(start[k][0])]
        exits = [k for k in end if not succ[k] and edge(end[k][0])]
        # Autoware's routing also changes lanes across a dashed line: neighbours share that
        # bound way, one as its right bound, the other as its left
        role_of = defaultdict(list)
        for k, (t, m) in road.items():
            for typ, rl, ref in m:
                if typ == "way" and rl in ("left", "right") and way_tags.get(ref, {}).get("subtype") == "dashed":
                    role_of[ref].append((k, rl))
        move = {k: list(v) for k, v in succ.items()}
        for ref, lst in role_of.items():
            for a, ra in lst:
                for b, rb in lst:
                    if a != b and ra != rb and a in move:
                        move[a].append(b)
        back = defaultdict(list)
        for k, v in move.items():
            for x in v:
                back[x].append(k)

        def reach(seeds, g):
            seen, todo = set(seeds), list(seeds)
            while todo:
                v = todo.pop()
                for w in g.get(v, []):
                    if w not in seen:
                        seen.add(w)
                        todo.append(w)
            return seen
        enterable, leavable = reach(entries, move), reach(exits, back)
        stranded = {"not reachable from an entry": sum(1 for k in start if k not in enterable),
                    "cannot reach an exit": sum(1 for k in start if k not in leavable)}
    else:
        stranded = {}
    per = [len(v) for v in ways.values()] or [0]
    local = sum(1 for n in nodes if any(t.get("k") == "local_x" for t in n.findall("tag")))
    from shapely.geometry import LineString
    xy = {}
    for n in nodes:
        t = tags(n)
        if "local_x" in t:
            xy[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    crossed = 0
    from generate import min_radius
    tight = Counter()
    for t, m in road.values():
        lb, rb = bound(m, "left"), bound(m, "right")
        if len(lb) >= 2 and len(rb) >= 2 and all(i in xy for i in lb + rb):
            crossed += LineString([xy[i] for i in lb]).crosses(LineString([xy[i] for i in rb]))
            if "turn_direction" in t:
                r = min_radius([xy[i] for i in lb], [xy[i] for i in rb])
                for lim in (4, 5, 6):
                    tight[f"under_{lim}m"] += r < lim
    rep = {
        "nodes": len(nodes), "ways": len(ways), "lanelets": len(lanelets), "road_lanelets": len(road),
        "nodes_per_way_mean": round(sum(per) / len(per), 1), "nodes_per_way_max": max(per),
        "subtypes": dict(Counter(t.get("subtype", "road") for t, _ in lanelets.values())),
        "dead_ends": sum(1 for v in succ.values() if not v),
        "no_predecessor": sum(1 for k in start if pred[k] == 0),
        "branch_points": len(branch), "junction_paths": len(paths),
        "stranded": stranded,
        "junction_paths_turn_direction": dict(turn),
        "speed_limit": sum(1 for t, _ in road.values() if "speed_limit" in t),
        "regulatory_elements": dict(regs_by_sub),
        "traffic_lights_with_stop_line": sum(1 for _, m in tl if any(rl == "ref_line" for _, rl, _ in m)),
        "lanelets_referring_a_traffic_light": refers_tl,
        "stop_line_ways": stop_lines,
        "swb_source": dict(Counter(t.get("swb:source", "-") for t, _ in road.values())),
        "nodes_with_local_xy": local,
        "bounds_crossing": crossed,
        "turn_paths_radius": dict(tight),
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
