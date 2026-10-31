#!/usr/bin/env python3
"""Plan a drive on a generated map before writing its scenario (roadmap 062, O6): route through
points in order, as Autoware will, and list each leg's junction paths.

    source /opt/autoware/1.5.0/setup.bash
    scripts/open_maps/route_plan.py <lanelet2_map.osm> x,y,heading_deg x,y,heading_deg ...

Each point snaps to the nearest road lanelet (not a junction path) heading within 45 deg of it;
the snapped lanelet's middle (x, y, heading in rad) is what scenario.py's --start, --via and
--goal take. Per leg: its length (whole lanelets, so a little over) and its junction paths in
order, S/L/R for straight, left and right, `*` when the approach before it has a traffic light,
`[id]` that approach lanelet. A leg with NO ROUTE needs another point.
"""
import math
import sys

import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from lanelet2.io import Origin


def main():
    m, _ = lanelet2.io.loadRobust(sys.argv[1], MGRSProjector(Origin(0, 0)))
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.Vehicle)
    g = lanelet2.routing.RoutingGraph(m, rules)

    def head(ll):
        c = ll.centerline
        return math.degrees(math.atan2(c[-1].y - c[0].y, c[-1].x - c[0].x))

    def nearest(x, y, h):
        best = None
        for ll in m.laneletLayer:
            if ll.attributes["subtype"] != "road" or "turn_direction" in ll.attributes:
                continue
            d = min(math.hypot(p.x - x, p.y - y) for p in ll.centerline)
            if abs((head(ll) - h + 180) % 360 - 180) < 45 and (best is None or d < best[0]):
                best = (d, ll)
        return best[1]

    def middle(ll):
        c = ll.centerline
        i = len(c) // 2
        p, q = c[max(0, i - 1)], c[min(len(c) - 1, i + 1)]
        return round(c[i].x, 3), round(c[i].y, 3), round(math.atan2(q.y - p.y, q.x - p.x), 3)

    lls = [nearest(*map(float, a.split(","))) for a in sys.argv[2:]]
    for k, ll in enumerate(lls):
        name = ll.attributes["name"] if "name" in ll.attributes else ""
        print("point", k, ll.id, name, middle(ll))
    total = 0.0
    for a, b in zip(lls, lls[1:]):
        r = g.getRoute(a, b)
        if r is None:
            print("NO ROUTE", a.id, b.id)
            continue
        sp = r.shortestPath()
        length = sum(lanelet2.geometry.length2d(x) for x in sp)
        total += length
        ev, lit = [], False
        for x in sp:
            if "turn_direction" in x.attributes:
                ev.append(x.attributes["turn_direction"][0].upper() + ("*" if lit else ""))
                lit = False
            else:
                lit = any(e.attributes["subtype"] == "traffic_light" for e in x.regulatoryElements)
                if lit:
                    ev.append(f"[{x.id}]")
        print(f"leg {a.id}->{b.id} {length:6.0f} m  junction paths: {' '.join(ev)}")
    print("total", round(total), "m   (* = signalised)")


if __name__ == "__main__":
    main()
