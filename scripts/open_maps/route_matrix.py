#!/usr/bin/env python3
"""Can a car get from each landmark to every other? A routing check of a generated map that does
not depend on where the box cuts the roads (a largest strongly connected part does: a lane
that leaves the box never comes back).

    scripts/open_maps/route_matrix.py <lanelet2_map.osm> <site.yaml>

Landmarks come from the site file (`landmarks: {name: {road: <OSM name>, near: [x, y]}}`, map
frame). A landmark is every lanelet of that road within 80 m of the point, both directions; a pair is routable when some lanelet
of the first routes to some lanelet of the second (Lanelet2's routing graph, vehicle rules).
Needs Lanelet2 (Autoware's install: source /opt/autoware/1.5.0/setup.bash).
"""
import argparse
import math

import lanelet2
import yaml
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map")
    ap.add_argument("site")
    a = ap.parse_args()
    site = yaml.safe_load(open(a.site))
    s, w, n, e = site["bbox"]
    m, errs = lanelet2.io.loadRobust(a.map, UtmProjector(Origin((s + n) / 2, (w + e) / 2)))
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.Vehicle)
    g = lanelet2.routing.RoutingGraph(m, rules)
    road = [ll for ll in m.laneletLayer if ll.attributes["subtype"] == "road"]

    def local(p):
        return float(p.attributes["local_x"]), float(p.attributes["local_y"])
    mids = {ll.id: tuple((p + q) / 2 for p, q in zip(local(ll.leftBound[len(ll.leftBound) // 2]),
                                                       local(ll.rightBound[len(ll.rightBound) // 2])))
            for ll in road}
    marks = {}
    for name, lm in site["landmarks"].items():
        marks[name] = [ll for ll in road if "name" in ll.attributes and ll.attributes["name"] == lm["road"]
             and math.dist(mids[ll.id], lm["near"]) < 80.0]
    names = list(marks)
    ok = total = 0
    print("from \\ to".ljust(14) + "".join(x[:6].ljust(7) for x in names))
    for i in names:
        row = []
        for j in names:
            if i == j:
                row.append("-")
                continue
            r = any(g.getRoute(x, y) is not None for x in marks[i] for y in marks[j])
            row.append("yes" if r else "NO")
            ok += r
            total += 1
        print(f"{i[:13]:14s}" + "".join(c.ljust(7) for c in row))
    print(f"routable: {ok} of {total}; load errors {len(errs)}; "
          f"lanelets per landmark {[len(v) for v in marks.values()]}")


if __name__ == "__main__":
    main()
