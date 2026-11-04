#!/usr/bin/env python3
"""Roadmap 072, the tour: legs that between them drive every lanelet of the NTU map.

Each leg is a route Autoware's mission planner takes as planned: lanelet2's shortest path from
the lanelet the start pose is in to the one the goal is in (the map has no lane changes, so the
path is the route). The start and the goal are put where only their own lanelet covers the
point, so a split's or a merge's overlap cannot hand Autoware a sibling lanelet.

Planning is greedy and polynomial: from every lanelet, a Dijkstra tree over the following
relation (cost: the entered lanelet's length, lanelet2's distance cost); the leg is the tree path
with the most still-uncovered length, checked against lanelet2's own `shortestPath`.

    source /opt/autoware/1.5.0/setup.bash
    python3 scripts/ntu_carla/tour_legs.py MAP.osm [--out DIR]
    python3 scripts/ntu_twin/scenario_pair.py          # the ntu_carla_tour_NN.xosc pairs

Writes scenarios/ntu_twin/ntu_twin_tour_NN.xosc (from ntu_twin_r02.xosc: start, goal, success on
reach, failure on timeout, max(600 s, 120 s + length / 1.5 m/s); no NPCs) and tour_legs.json
(each leg's lanelets, length, start and goal). Plan on the map fixed by
scripts/ntu_twin/fix_lanelets.py. Lanelet 1011236, an isolated 4 m stub left over from the r01/r02
merge, is left out (agreed with 059).
"""
import argparse
import heapq
import json
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np
import lanelet2
from lanelet2.io import Origin
from lanelet2.projection import UtmProjector

SCENARIOS = pathlib.Path(__file__).resolve().parents[2] / "scenarios/ntu_twin"
TEMPLATE = SCENARIOS / "ntu_twin_r02.xosc"
Z_LIFT = 0.4          # spawn height over the lanelet's elevation
START_IN = 2.0        # the start pose at least this far into the first lanelet
GOAL_BACK = 4.0       # the goal at least this far before the end of the last (and its footprint inside)
MIN_LEG = 25.0
MIN_START_GOAL = 12.0  # m, start to goal (SSv2's reach tolerance is 5 m)
# tesla_model3's vehicle_info: rear axle to the front edge (wheel_base + front_overhang), to the rear
# edge, and half the width (wheel_tread / 2 + side overhang)
FRONT, REAR, HALF_W = 3.005 + 0.807, 0.98, 1.667 / 2 + 0.248
SKIP = {1011236}       # isolated 4 m stub of the r01/r02 merge (059 and 072 agreed to leave it)        # shorter legs get a predecessor or a follower added


def load(osm):
    root = ET.parse(osm).getroot()
    pts = {}
    for n in root.findall("node"):
        t = {x.get("k"): x.get("v") for x in n.findall("tag")}
        pts[int(n.get("id"))] = (float(t["local_x"]), float(t["local_y"]), float(t.get("ele", 0)))
    ways = {int(w.get("id")): [int(nd.get("ref")) for nd in w.findall("nd")] for w in root.findall("way")}
    centre_way = {}
    for rel in root.findall("relation"):
        for mb in rel.findall("member"):
            if mb.get("role") == "centerline":
                centre_way[int(rel.get("id"))] = ways[int(mb.get("ref"))]
    n0 = root.find("node")
    m = lanelet2.io.load(osm, UtmProjector(Origin(float(n0.get("lat")), float(n0.get("lon")))))
    return m, pts, centre_way


def resample(a, k):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(a[:, :2], axis=0), axis=1))]
    u = np.linspace(0, s[-1], k)
    return np.array([np.interp(u, s, a[:, j]) for j in range(3)]).T


def centreline(l, pts, centre_way):
    """Autoware's centreline: the lanelet's `centerline` way where it has one, else mid-bounds."""
    if l.id in centre_way:
        # oriented as the lanelet: 23 of the map's custom centrelines run against it (roadmap 072, the tour)
        c = np.array([pts[i] for i in centre_way[l.id]])
        s, e = np.array(pts[l.leftBound[0].id]), np.array(pts[l.leftBound[-1].id])
        if np.linalg.norm(c[0, :2] - s[:2]) > np.linalg.norm(c[0, :2] - e[:2]):
            c = c[::-1]
        return c
    a = np.array([pts[p.id] for p in l.leftBound])
    b = np.array([pts[p.id] for p in l.rightBound])
    k = max(len(a), len(b), 2) * 4
    return (resample(a, k) + resample(b, k)) / 2


def pose_at(c, d):
    seg = np.linalg.norm(np.diff(c[:, :2], axis=0), axis=1)
    cum = np.r_[0, np.cumsum(seg)]
    d = min(max(d, 0.0), cum[-1])
    j = int(min(np.searchsorted(cum, d, side="right") - 1, len(seg) - 1))
    f = (d - cum[j]) / max(seg[j], 1e-6)
    p = c[j] + f * (c[j + 1] - c[j])
    h = float(np.arctan2(c[j + 1, 1] - c[j, 1], c[j + 1, 0] - c[j, 0]))
    return [float(p[0]), float(p[1]), float(p[2]), h]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("osm")
    ap.add_argument("--out", default=str(SCENARIOS), help="where the xosc files go (default scenarios/ntu_twin)")
    ap.add_argument("--min-gain", type=float, default=1.0)
    a = ap.parse_args()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    m, pts, centre_way = load(a.osm)
    tr = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                       lanelet2.traffic_rules.Participants.Vehicle)
    rg = lanelet2.routing.RoutingGraph(m, tr)
    L = {l.id: l for l in m.laneletLayer if l.id not in SKIP}
    C = {i: centreline(l, pts, centre_way) for i, l in L.items()}
    length = {i: float(np.linalg.norm(np.diff(C[i][:, :2], axis=0), axis=1).sum()) for i in L}
    succ = {i: [f.id for f in rg.following(L[i], False)] for i in L}
    pred = {i: [p.id for p in rg.previous(L[i], False)] for i in L}

    def route(s, t):
        p = rg.shortestPath(L[s], L[t], 0, False)
        return [x.id for x in p] if p else None

    # lanelet polygons in the map frame (the nodes' local_x/local_y, as Autoware's local projector
    # reads them; lanelet2's own geometry is in the UTM projector's frame, which is not that)
    poly = {i: np.array([pts[p.id][:2] for p in l.leftBound] + [pts[p.id][:2] for p in l.rightBound][::-1])
            for i, l in L.items()}
    box = {i: (q.min(axis=0), q.max(axis=0)) for i, q in poly.items()}

    def inside(q, xy):
        x, y = xy
        xs, ys = q[:, 0], q[:, 1]
        xn, yn = np.roll(xs, -1), np.roll(ys, -1)
        c = ((ys > y) != (yn > y)) & (x < (xn - xs) * (y - ys) / np.where(yn != ys, yn - ys, 1e-12) + xs)
        return bool(c.sum() % 2)

    def heading_near(j, xy):
        c = C[j]
        k = int(np.argmin(np.linalg.norm(c[:-1, :2] - xy, axis=1) + np.linalg.norm(c[1:, :2] - xy, axis=1)))
        return float(np.arctan2(c[k + 1, 1] - c[k, 1], c[k + 1, 0] - c[k, 0]))

    def others_at(i, xy, h):
        """Other lanelets covering xy whose direction there is within 45 deg of h: those Autoware's
        closest-lanelet query (a yaw constraint) could take instead of i. A crossing lanelet is not one."""
        out = []
        for j, (lo, hi) in box.items():
            if j == i or not (lo[0] <= xy[0] <= hi[0] and lo[1] <= xy[1] <= hi[1]) or not inside(poly[j], xy):
                continue
            dh = (heading_near(j, np.asarray(xy)) - h + np.pi) % (2 * np.pi) - np.pi
            if abs(dh) < np.pi / 4:
                out.append(j)
        return out

    def unique_d(i, lo, hi, step=0.5):
        """Distances along lanelet i, from lo towards hi, whose centreline point no other lanelet covers."""
        d = lo
        sgn = 1 if hi >= lo else -1
        while (d - hi) * sgn <= 0:
            p = pose_at(C[i], d)
            if not others_at(i, p[:2], p[3]):
                return d
            d += sgn * step
        return None

    def dijkstra(s):
        dist, prev = {s: 0.0}, {s: None}
        h = [(0.0, s)]
        while h:
            d, x = heapq.heappop(h)
            if d > dist[x]:
                continue
            for y in succ[x]:
                nd = d + length[y]
                if nd < dist.get(y, 1e18):
                    dist[y], prev[y] = nd, x
                    heapq.heappush(h, (nd, y))
        return prev

    trees = {s: dijkstra(s) for s in L}

    def path(s, t):
        p, x = [], t
        while x is not None:
            p.append(x)
            x = trees[s][x]
        return p[::-1]

    def footprint_ok(p, g):
        """Autoware's goal check (default_planner's check_goal_footprint_inside_lanes): the car's
        footprint at the goal inside the lanelets around it -- the goal's, those before it within
        the car's front reach, and those after (here: the leg's, and the first follower's chain)."""
        near = list(p)
        x = p[-1]
        reach = 0.0
        while succ[x] and reach < FRONT:
            x = succ[x][0]
            if x in near:
                break
            near.append(x)
            reach += length[x]
        c, s_ = math.cos(g[3]), math.sin(g[3])
        for u in np.linspace(-REAR, FRONT, 12):
            for v in np.linspace(-HALF_W, HALF_W, 5):
                q = (g[0] + u * c - v * s_, g[1] + u * s_ + v * c)
                if not any(inside(poly[j], q) for j in near):
                    return False
        return True

    def start_d(p):
        return unique_d(p[0], min(START_IN, length[p[0]] * 0.3), length[p[0]] * 0.8)

    def goal_d(p, d0=None):
        """The goal on the leg's last lanelet: GOAL_BACK before its end or further back, on a point
        of its own, the footprint inside the lanes, and MIN_START_GOAL from the start (SSv2's reach
        condition, 5 m, would otherwise end a loop at once)."""
        le = length[p[-1]]
        d0 = start_d(p) if d0 is None else d0
        st = pose_at(C[p[0]], d0) if d0 is not None else None
        lo = (d0 + 5.0) if (len(p) == 1 and d0 is not None) else le * 0.2
        d = max(le - GOAL_BACK, le * 0.7)
        while d >= lo:
            g = pose_at(C[p[-1]], d)
            if not others_at(p[-1], g[:2], g[3]) and footprint_ok(p, g) and \
                    (st is None or math.hypot(g[0] - st[0], g[1] - st[1]) >= MIN_START_GOAL):
                return d
            d -= 0.5
        return None

    def endpoints_ok(p):
        d0 = start_d(p)
        if d0 is None:
            return None
        d1 = goal_d(p, d0)
        if d1 is None:
            return None
        return d0, d1

    covered, legs, unusable = set(), [], {}
    while True:
        best = None
        for s in L:
            for t in trees[s]:
                p = path(s, t)
                gain = sum(length[x] for x in p if x not in covered and x not in unusable)
                if gain < a.min_gain:
                    continue
                cost = sum(length[x] for x in p) - gain
                key = (gain - 0.5 * cost, -len(p))
                if best is None or key > best[0]:
                    best = (key, p)
        if best is None:
            break
        p = best[1]
        # grow a short leg, or one whose start or goal has no point of its own, by a neighbour
        def fit(c, front):
            """c, cut (at its back, or at its front) to the longest part that is lanelet2's route."""
            while len(c) >= 2:
                if route(c[0], c[-1]) == c:
                    return c
                c = c[1:] if front else c[:-1]
            return None

        for _ in range(8):
            total = sum(length[x] for x in p)
            need_s, need_g = start_d(p) is None, goal_d(p) is None
            if endpoints_ok(p) and total >= MIN_LEG:
                break
            if not need_s and not need_g:
                need_s = bool(pred[p[0]])
                need_g = not need_s
            grown = False
            if need_s:
                cands = [fit([q] + p, front=False) for q in pred[p[0]] if q not in p]
                cands = [c for c in cands if c]
                cands.sort(key=lambda c: start_d(c) is None)
                if cands:
                    p, grown = cands[0], True
            if need_g:
                cands = [fit(p + [q], front=True) for q in succ[p[-1]] if q not in p]
                cands = [c for c in cands if c]
                cands.sort(key=lambda c: goal_d(c) is None)
                if cands:
                    p, grown = cands[0], True
            if not grown:
                break
        if route(p[0], p[-1]) != p:
            raise SystemExit(f"leg {p} is not lanelet2's route")
        ends = endpoints_ok(p)
        # still no start or goal: cut the leg until it has both; a lanelet cut from an end that
        # nothing else reaches is reported (a sink too short or too curved for the car's goal
        # footprint, a source with no point of its own)
        cut = []
        while ends is None and len(p) > 1:
            if start_d(p) is None:
                cut.append((p[0], "start"))
                p = p[1:]
            else:
                cut.append((p[-1], "goal"))
                p = p[:-1]
            ends = endpoints_ok(p)
        for x, why in cut:
            if x not in covered and x not in p:
                unusable.setdefault(x, why)
        if ends is None:
            unusable.setdefault(p[0], "leg")
        legs.append({"lanelets": p, "ends": ends})
        covered |= set(p)

    rows = []
    tmpl = TEMPLATE.read_text()
    for n, g in enumerate(legs):
        p = g["lanelets"]
        if g["ends"] is None:
            rows.append({"leg": n, "lanelets": p, "skip": "no start or goal point of its own"})
            continue
        d0, d1 = g["ends"]
        s, e = pose_at(C[p[0]], d0), pose_at(C[p[-1]], d1)
        drive = sum(length[x] for x in p) - d0 - (length[p[-1]] - d1)
        rows.append({"leg": n, "lanelets": p, "length": round(drive, 1), "start": s, "goal": e})
        name = f"ntu_twin_tour_{n:02d}"
        x = tmpl
        x = x.replace('x="52217.42" y="67654.98" z="12.05" h="0.0035"',
                      f'x="{s[0]:.2f}" y="{s[1]:.2f}" z="{s[2] + Z_LIFT:.2f}" h="{s[3]:.4f}"')
        x = x.replace('x="52565.0" y="67656.93" z="11.9" h="0.0052"',
                      f'x="{e[0]:.2f}" y="{e[1]:.2f}" z="{e[2]:.2f}" h="{e[3]:.4f}"')
        x = x.replace('x="52565.0" y="67656.93" z="11.9" h="0"',
                      f'x="{e[0]:.2f}" y="{e[1]:.2f}" z="{e[2]:.2f}" h="0"')
        x = x.replace('<SimulationTimeCondition value="600.0"',
                      f'<SimulationTimeCondition value="{max(600.0, 120 + drive / 1.5):.1f}"')
        x = x.replace('<Story name="ntu_twin_r02">', f'<Story name="{name}">')
        x = x.replace('description="roadmap 059 N8: route r02 on the NTU campus twin (MLS scan), lanelet 7 to 49, '
                      '347 m, no NPCs"',
                      f'description="roadmap 059 N8: tour leg {n:02d} on the NTU campus twin (MLS scan), lanelets '
                      f'{p[0]} to {p[-1]} ({len(p)} lanelets), {drive:.0f} m, no NPCs; written by '
                      f'scripts/ntu_carla/tour_legs.py"')
        assert x.count(f'{s[0]:.2f}') >= 1 and x.count(f'{e[0]:.2f}') >= 2 and name in x and "tour leg" in x, name
        (out / f"{name}.xosc").write_text(x)

    for old in sorted(out.glob("ntu_twin_tour_*.xosc")):
        if int(old.stem[-2:]) >= len(rows):
            old.unlink()
    json.dump(rows, open(out / "tour_legs.json", "w"), indent=1)
    cov = set(x for r in rows if "start" in r for x in r["lanelets"])
    tot, drv = sum(length.values()), sum(r.get("length", 0) for r in rows)
    print(f"{len(rows)} legs ({sum('skip' in r for r in rows)} skipped), {drv / 1000:.2f} km driven for "
          f"{tot / 1000:.2f} km of lanes; lanelets covered {len(cov)}/{len(L)}; "
          f"under {MIN_LEG:.0f} m {sum(r.get('length', 99) < MIN_LEG for r in rows)}")
    missing = sorted(set(L) - cov)
    if missing:
        why = {x: unusable.get(x, "?") for x in missing}
        print("not covered:", why)
    json.dump({"not_covered": {str(x): unusable.get(x, "?") for x in missing}},
              open(out / "tour_legs_missing.json", "w"), indent=1)


if __name__ == "__main__":
    main()
