"""czp-zones (step 1, offline): conflict zones from a Lanelet2 map into zones.yaml.

For every pair of road lanelets that are not one lane's sequence or neighbours (not predecessor or
successor, not left or right of each other, not splitting from one predecessor), the overlap of their
swept areas for the widest vehicle class (``--width``, with ``--margin`` on each side) is a zone. Per
zone and approach (each of the two lanelets, as a path from ``--upstream`` m before it to
``--downstream`` m after it):

- entry and exit arc length along the path where the swept areas first and last overlap;
- ``order``: the map's fixed total order over the junction's approaches -- the compass bearing of the
  arm the path comes from, seen from the junction's centroid, clockwise from north, then the lanelet
  id -- so that every car computes the same order;
- ``priority``: from the map's regulatory elements (right_of_way: right_of_way/yield; a stop_sign
  traffic_sign on every approach: all_way_stop); none of these: equal;
- ``width``: the lane's width at the zone; the zone's ``shared_width`` (the free width across both);
- ``exit_box``: the stretch after the zone on each path that must be clear (``--exit-box`` m);
- the zone's kind: cross_unsignalled (or all_way_stop), merge (one successor), left_turn (a left turn
  against an oncoming path), signalled (a traffic light on both approaches; out of the protocol's scope
  but listed).

Junctions are the connected groups of zones. A plot per junction (``--plots``).

Usage:
  czp-zones --map <lanelet2_map.osm> --out <zones.yaml> [--name Town05] [--plots <dir>] [--fixture <junction id>]
"""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import sys
from collections import defaultdict

import yaml

from .geom import area, clip_convex, convex_hull, polyline_length, polyline_project, ribbon_quads
from .model import Approach, Zone
from .zones import zone_to_dict


def load_map(path):
    import lanelet2
    from lanelet2.io import Origin
    from lanelet2.projection import UtmProjector
    import re
    head = open(path).read(4096)
    ll = re.search(r'lat="([-0-9.]+)" lon="([-0-9.]+)"', head)
    origin = Origin(float(ll.group(1)), float(ll.group(2))) if ll else Origin(0, 0)  # a projector near the map
    m, errs = lanelet2.io.loadRobust(path, UtmProjector(origin))
    if not len(m.pointLayer) and errs:
        raise RuntimeError(f"{path}: lanelet2 loaded no points: {errs[:3]}")
    for pt in m.pointLayer:
        if "local_x" in pt.attributes:
            pt.x = float(pt.attributes["local_x"])
            pt.y = float(pt.attributes["local_y"])
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.Vehicle)
    return m, lanelet2.routing.RoutingGraph(m, rules)


def attr(e, k):
    return e.attributes[k] if k in e.attributes else None


def pts(ls):
    return [(p.x, p.y) for p in ls]


def resample(p, step=1.0):
    if len(p) < 2:
        return p
    out = [p[0]]
    acc = 0.0
    for (x0, y0), (x1, y1) in zip(p[:-1], p[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg / step))
        for i in range(1, n + 1):
            out.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n))
        acc += seg
    return out


def bbox(p, pad=0.0):
    xs, ys = [a[0] for a in p], [a[1] for a in p]
    return min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad


def bb_overlap(a, b):
    return a[0] <= b[2] and b[0] <= a[2] and a[1] <= b[3] and b[1] <= a[3]


def width_of(ll):
    lb, rb = pts(ll.leftBound), pts(ll.rightBound)
    if not lb or not rb:
        return 3.5
    ws = []
    for x, y in lb[:: max(1, len(lb) // 5)]:
        ws.append(min(math.hypot(x - a, y - b) for a, b in rb))
    return sum(ws) / len(ws)


class MapZones:
    def __init__(self, path, width=2.6, margin=0.3, upstream=80.0, downstream=20.0, exit_box=7.0):
        self.path = path
        self.m, self.g = load_map(path)
        self.half = width / 2 + margin
        self.upstream, self.downstream, self.exit_len = upstream, downstream, exit_box
        self.roads = [ll for ll in self.m.laneletLayer if attr(ll, "subtype") == "road"]
        self.by_id = {ll.id: ll for ll in self.roads}
        self.cl = {ll.id: resample(pts(ll.centerline), 1.0) for ll in self.roads}
        self.quads = {i: ribbon_quads(c, self.half) for i, c in self.cl.items() if len(c) >= 2}
        self.bb = {i: bbox(c, self.half) for i, c in self.cl.items() if len(c) >= 2}

    # --- relations between lanelets ------------------------------------------------------------------
    def related(self, a, b):
        """One lane's sequence or neighbours: never a conflict zone."""
        g = self.g
        fa = {x.id for x in g.following(a)}
        pa = {x.id for x in g.previous(a)}
        if b.id in fa or b.id in pa:
            return True
        for f in (g.left, g.right, g.adjacentLeft, g.adjacentRight):
            x = f(a)
            if x is not None and x.id == b.id:
                return True
        pb = {x.id for x in g.previous(b)}
        if pa & pb:  # split from one predecessor: they diverge, nobody crosses
            return True
        return False

    def overlap(self, ia, ib):
        """The intersection points of the two swept areas, or []."""
        out = []
        for qa in self.quads[ia]:
            ba = bbox(qa)
            for qb in self.quads[ib]:
                if not bb_overlap(ba, bbox(qb)):
                    continue
                poly = clip_convex(list(qa), list(qb))
                if len(poly) >= 3 and abs(area(poly)) > 1e-3:
                    out.extend(poly)
        return out

    def path_of(self, ll):
        """The approach path: upstream predecessors (straight back) + the lanelet + some of its successor."""
        g = self.g
        up, cur, acc = [], ll, 0.0
        while acc < self.upstream:
            prev = list(g.previous(cur))
            if not prev:
                break
            cur = prev[0]
            c = self.cl.get(cur.id) or resample(pts(cur.centerline))
            up = c[:-1] + up
            acc += polyline_length(c)
        down, cur, acc = [], ll, 0.0
        nxt = None
        while acc < self.downstream:
            f = list(g.following(cur))
            if not f:
                break
            cur = f[0]
            nxt = nxt or cur.id
            c = self.cl.get(cur.id) or resample(pts(cur.centerline))
            down += c[1:]
            acc += polyline_length(c)
        return up + self.cl[ll.id] + down, polyline_length(up + self.cl[ll.id][:1]) if up else 0.0, nxt

    def regs(self, ll):
        out = set()
        for r in ll.regulatoryElements:
            st = attr(r, "subtype")
            if st == "traffic_sign":
                for prims in r.parameters.values():
                    for p in prims:
                        if hasattr(p, "attributes") and attr(p, "subtype") in ("stop_sign", "stop"):
                            out.add("stop_sign")
            out.add(st)
        return out

    def approach_regs(self, ll):
        """Regulatory elements on the lanelet or its predecessors (the stop line is usually on the arm)."""
        out = set(self.regs(ll))
        for p in self.g.previous(ll):
            out |= self.regs(p)
        return out

    def stop_line(self, ll):
        """A stop_line linestring within 4 m of where the lanelet starts (the arm's end), or None."""
        if not hasattr(self, "_stop_lines"):
            self._stop_lines = [pts(ls) for ls in self.m.lineStringLayer if attr(ls, "type") == "stop_line"]
        x, y = self.cl[ll.id][0]
        best, bd = None, 4.0
        for q in self._stop_lines:
            mx, my = (q[0][0] + q[-1][0]) / 2, (q[0][1] + q[-1][1]) / 2
            d = math.hypot(mx - x, my - y)
            if d < bd:
                best, bd = (q[0], q[-1]), d
        return best

    # --- zones ---------------------------------------------------------------------------------------
    def pairs(self):
        ids = sorted(self.quads)
        cand = defaultdict(list)
        cell = 20.0
        for i in ids:
            b = self.bb[i]
            for cx in range(int(b[0] // cell), int(b[2] // cell) + 1):
                for cy in range(int(b[1] // cell), int(b[3] // cell) + 1):
                    cand[(cx, cy)].append(i)
        seen = set()
        for lst in cand.values():
            for x in range(len(lst)):
                for y in range(x + 1, len(lst)):
                    a, b = lst[x], lst[y]
                    k = (min(a, b), max(a, b))
                    if k in seen:
                        continue
                    seen.add(k)
                    if bb_overlap(self.bb[a], self.bb[b]):
                        yield k

    def parallel(self, ia, ib, hull):
        """(same direction?, opposite?) of the two lanelets where they overlap."""
        hx = sum(p[0] for p in hull) / len(hull)
        hy = sum(p[1] for p in hull) / len(hull)
        ha = polyline_project(self.cl[ia], hx, hy)[2]
        hb = polyline_project(self.cl[ib], hx, hy)[2]
        d = abs(math.atan2(math.sin(hb - ha), math.cos(hb - ha)))
        return d < math.radians(20), d > math.radians(160)

    def build(self):
        raw = []
        junction_marked = any(attr(ll, "turn_direction") for ll in self.roads)
        for ia, ib in self.pairs():
            a, b = self.by_id[ia], self.by_id[ib]
            if self.related(a, b):
                continue
            # a map that marks its junction lanelets (turn_direction): a zone needs one of them
            if junction_marked and not (attr(a, "turn_direction") or attr(b, "turn_direction")):
                continue
            ov = self.overlap(ia, ib)
            if len(ov) < 3:
                continue
            hull = convex_hull(ov)
            if abs(area(hull)) < 0.5:
                continue
            same, opp = self.parallel(ia, ib, hull)
            fa = {x.id for x in self.g.following(a)}
            fb = {x.id for x in self.g.following(b)}
            if same and not (fa & fb):
                continue  # side by side the same way (a lane's taper or a parking strip), not a conflict zone
            if opp and width_of(a) + width_of(b) >= 2 * 2 * self.half:
                continue  # two-way lanes wide enough for both: no narrow passage on the map
            raw.append((ia, ib, hull))
        # junctions: connected groups of zones (through shared lanelets)
        parent = {}

        def find(x):
            parent.setdefault(x, x)
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        for ia, ib, _ in raw:
            parent[find(ia)] = find(ib)
        groups = defaultdict(list)
        for z in raw:
            groups[find(z[0])].append(z)
        junctions = []
        for k, zs in sorted(groups.items(), key=lambda kv: min(min(z[0], z[1]) for z in kv[1])):
            junctions.append(self.junction(zs))
        return junctions

    def junction(self, zs):
        lls = sorted({z[0] for z in zs} | {z[1] for z in zs})
        allp = [p for z in zs for p in z[2]]
        cx, cy = sum(p[0] for p in allp) / len(allp), sum(p[1] for p in allp) / len(allp)
        paths = {i: self.path_of(self.by_id[i]) for i in lls}
        # the map's order: the arm each path comes from, by compass bearing from the centroid (clockwise
        # from north), then the lanelet id
        def bearing(i):
            c = self.cl[i]
            x, y = c[0]
            return (math.degrees(math.atan2(x - cx, y - cy)) + 360.0) % 360.0
        ranked = sorted(lls, key=lambda i: (round(bearing(i) / 10.0), i))
        order = {i: n for n, i in enumerate(ranked)}
        jid = f"j{min(lls)}"
        regs = {i: self.approach_regs(self.by_id[i]) for i in lls}
        stop_all = all("stop_sign" in regs[i] for i in lls)
        zones = []
        for ia, ib, hull in zs:
            aps = []
            for i in (ia, ib):
                path, off, nxt = paths[i]
                ss = [polyline_project(path, x, y)[0] for x, y in hull]
                e_s, x_s = min(ss), max(ss)
                c = self.cl[i]
                h = math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0])
                pr = "equal"
                if "right_of_way" in regs[i]:
                    pr = self.row_priority(self.by_id[i])
                if stop_all:
                    pr = "all_way_stop"
                aps.append(Approach(id=str(i), lanelet=i, heading=h, order=order[i], priority=pr,
                                    stop_line=self.stop_line(self.by_id[i]), entry_s=e_s, exit_s=x_s,
                                    width=width_of(self.by_id[i]), centerline=tuple(path[::2]), exit_lanelet=nxt))
            kind = self.kind_of(ia, ib, regs, stop_all)
            exit_box = []
            for ap in aps:
                path = paths[ap.lanelet][0]
                s0, s1 = ap.exit_s, ap.exit_s + self.exit_len
                seg = [p for p in path if s0 <= polyline_project(path, *p)[0] <= s1]
                if len(seg) >= 2:
                    q = ribbon_quads(seg, ap.width / 2)
                    exit_box.append((ap.id, tuple(convex_hull([v for quad in q for v in quad]))))
            wa, wb = aps[0].width, aps[1].width
            zones.append(Zone(id=f"{jid}_{ia}_{ib}", kind=kind, approaches=tuple(aps), polygon=tuple(hull),
                              lanelets=(ia, ib), exit_box=tuple(exit_box), shared_width=round(max(wa, wb), 2)))
        return {"id": jid, "centroid": [round(cx, 2), round(cy, 2)], "lanelets": lls,
                "order": [int(i) for i in ranked], "stop_all": stop_all,
                "signalled": all("traffic_light" in regs[i] for i in lls), "zones": zones}

    def row_priority(self, ll):
        for r in ll.regulatoryElements:
            if attr(r, "subtype") != "right_of_way":
                continue
            for k, prims in r.parameters.items():
                if any(getattr(p, "id", None) == ll.id for p in prims):
                    return "major" if k == "right_of_way" else "minor"
        return "equal"

    def kind_of(self, ia, ib, regs, stop_all):
        a, b = self.by_id[ia], self.by_id[ib]
        fa = {x.id for x in self.g.following(a)}
        fb = {x.id for x in self.g.following(b)}
        if "traffic_light" in regs[ia] and "traffic_light" in regs[ib]:
            return "signalled"
        if fa & fb:
            return "merge"
        ta, tb = attr(a, "turn_direction"), attr(b, "turn_direction")
        ca, cb = self.cl[ia], self.cl[ib]
        ha = math.atan2(ca[-1][1] - ca[0][1], ca[-1][0] - ca[0][0])
        hb0 = math.atan2(cb[1][1] - cb[0][1], cb[1][0] - cb[0][0])
        ha0 = math.atan2(ca[1][1] - ca[0][1], ca[1][0] - ca[0][0])
        opp = abs(math.atan2(math.sin(hb0 - ha0), math.cos(hb0 - ha0))) > math.radians(135)
        if opp and "left" in (ta, tb):
            return "left_turn"
        if opp and ta is None and tb is None:
            return "narrow_passage"
        del ha
        return "all_way_stop" if stop_all else "cross_unsignalled"


def plot_junction(j, mz, path, name):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cx, cy = j["centroid"]
    fig, ax = plt.subplots(figsize=(7, 7))
    R = 45
    for i, c in mz.cl.items():
        if any(abs(x - cx) < R and abs(y - cy) < R for x, y in c):
            ax.plot([p[0] for p in c], [p[1] for p in c], color="#bbb", lw=0.8)
    colors = {"cross_unsignalled": "#d62728", "all_way_stop": "#9467bd", "merge": "#ff7f0e", "left_turn": "#1f77b4",
              "signalled": "#7f7f7f", "narrow_passage": "#2ca02c"}
    seen = set()
    for z in j["zones"]:
        poly = list(z.polygon) + [z.polygon[0]]
        ax.fill([p[0] for p in poly], [p[1] for p in poly], color=colors.get(z.kind, "k"), alpha=0.35,
                label=z.kind if z.kind not in seen else None)
        seen.add(z.kind)
        for ap in z.approaches:
            c = mz.cl[ap.lanelet]
            ax.plot([p[0] for p in c], [p[1] for p in c], color="#333", lw=1.2)
    for i in j["lanelets"]:
        c = mz.cl[i]
        ax.annotate(str(j["order"].index(i)), c[0], fontsize=8, color="#c00")
    ax.set_xlim(cx - R, cx + R)
    ax.set_ylim(cy - R, cy + R)
    ax.set_aspect("equal")
    ax.legend(loc="upper right", fontsize=7)
    ax.set_title(f"{name} {j['id']}: {len(j['zones'])} zones, {len(j['lanelets'])} paths; red: the map's order", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="")
    ap.add_argument("--width", type=float, default=2.6, help="m, the widest vehicle class")
    ap.add_argument("--margin", type=float, default=0.3, help="m on each side")
    ap.add_argument("--exit-box", type=float, default=7.0, help="m after the zone that must be clear")
    ap.add_argument("--plots", default="", help="a directory for one plot per junction")
    ap.add_argument("--fixture", default="", help="write only this junction (a small test fixture)")
    ap.add_argument("--no-centerline", action="store_true", help="omit the paths (smaller files)")
    a = ap.parse_args(argv)
    mz = MapZones(a.map, a.width, a.margin, exit_box=a.exit_box)
    js = mz.build()
    if a.fixture:
        js = [j for j in js if j["id"] == a.fixture]
    sha = hashlib.sha256(open(a.map, "rb").read()).hexdigest()
    out = {"map": os.path.abspath(a.map), "name": a.name or os.path.basename(os.path.dirname(os.path.abspath(a.map))),
           "map_sha256": sha, "tool": "czp-zones 0.1", "width": a.width, "margin": a.margin, "exit_box": a.exit_box,
           "order_rule": "arm bearing from the junction centroid, clockwise from north (10 deg bins), then lanelet id",
           "junctions": []}
    for j in js:
        zd = [zone_to_dict(z) for z in j["zones"]]
        if a.no_centerline:
            for z in zd:
                for apd in z["approaches"]:
                    apd["centerline"] = []
        out["junctions"].append({k: v for k, v in j.items() if k != "zones"} | {"zones": zd})
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=None, width=200)
    kinds = defaultdict(int)
    for j in js:
        for z in j["zones"]:
            kinds[z.kind] += 1
    print(f"{out['name']}: {len(js)} junctions, {sum(len(j['zones']) for j in js)} zones {dict(kinds)} -> {a.out}")
    if a.plots:
        os.makedirs(a.plots, exist_ok=True)
        for j in js:
            plot_junction(j, mz, os.path.join(a.plots, f"{out['name']}_{j['id']}.png"), out["name"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
