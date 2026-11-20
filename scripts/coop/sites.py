#!/usr/bin/env python3
"""Roadmap 074 E6 and E7: candidate sites on CARLA's towns, offline (no CARLA server).

The OpenDRIVE is read by the carla client library alone (carla.Map(name, xodr)), the Autoware
map by lanelet2 with its local_x/local_y (ROS frame: CARLA's (x, -y)).

E6, a 4-way stop: every junction with four incoming arms about 90 deg apart. Per arm: its lanes,
the straight length behind the junction (heading within STRAIGHT_DEG of the entry) up to the
previous junction and, through junctions, as far as the road stays straight; the same after the
straight-on exit; the OpenDRIVE signals on the approach (1000001 light, 206 stop, 205 yield); and
from the lanelet2 map: the approach lanelet, its regulatory elements (traffic lights linked or not,
right_of_way, traffic_sign), the stop_line / traffic_sign / light ways near its end, the junction
lanelets' turn_direction and speed limits. Ranked: no OpenDRIVE lights first, then by the shortest
straight approach through junctions.

E7, a merge: (a) a driving lane whose width runs to zero (or from zero) within a lane section, or
a lane-count change between sections (a lane drop or add); (b) a junction where incoming lanes of
two roads run onto one outgoing lane, one straight on (< 10 deg) and the other within 60 deg, over
a connector of 30 m or more (an on-ramp or a Y merge, not a turn); (c) in the lanelet2 map: a road lanelet with two non-junction predecessors, and road
lanelets that end without a successor, classed by the CARLA lane under their end (only a driving
lane that continues in CARLA is a candidate; shoulders and sidewalks are the map's kerb strips).

Usage:
  scripts/coop/sites.py --out <dir> [--towns Town03,Town05,Town10HD] [--xodr-only Town04,Town06] [--plots]
  (--xodr-dir defaults to $CARLA_DIR/CarlaUE4/Content/Carla/Maps/OpenDrive, --lanelet-dir to
  $CARLA_MAPS or data/carla-autoware-bridge; Town10HD's lanelet dir is Town10)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

import carla

STRAIGHT_DEG = 4.0  # an approach is straight while its heading stays this close to the entry's
STEP = 1.0
MAX_WALK = 400.0
NEAR_STOP = 12.0  # m: stop_line ways this close to an approach lanelet's end
LL_DIR_NAME = {"Town10HD": "Town10"}
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def yaw_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def xy(wp):
    loc = wp.transform.location
    return loc.x, loc.y


# --- OpenDRIVE side --------------------------------------------------------------------------------
def straight_walk(wp, forward, through=False, max_len=MAX_WALK):
    """Metres along wp's lane (back or ahead) while the heading stays within STRAIGHT_DEG of wp's.
    Stops at a junction unless `through` (then takes the straightest branch). Returns (length,
    why it stopped, junctions passed)."""
    ref = wp.transform.rotation.yaw
    cur, s, passed = wp, 0.0, []
    while s < max_len:
        cands = cur.next(STEP) if forward else cur.previous(STEP)
        if not cands:
            return s, "lane end", passed
        nxt = min(cands, key=lambda w: abs(yaw_diff(w.transform.rotation.yaw, ref)))
        if abs(yaw_diff(nxt.transform.rotation.yaw, ref)) > STRAIGHT_DEG:
            return s, "bend", passed
        if nxt.is_junction:
            jid = nxt.get_junction().id
            if not through:
                return s, f"junction {jid}", passed
            if not passed or passed[-1] != jid:
                passed.append(jid)
        cur, s = nxt, s + STEP
    return s, "max", passed


def outside(wp, forward):
    """The first waypoint outside the junction, behind (or after) a connector's end."""
    for d in (0.3, 0.6, 1.0, 2.0):
        c = wp.next(d) if forward else wp.previous(d)
        c = [w for w in c if not w.is_junction]
        if c:
            return min(c, key=lambda w: abs(yaw_diff(w.transform.rotation.yaw, wp.transform.rotation.yaw)))
    return None


def signals_before(wp_in):
    """OpenDRIVE landmarks valid for the approach lane, within 40 m before the junction."""
    back = wp_in.previous(40.0)
    start = min(back, key=lambda w: abs(yaw_diff(w.transform.rotation.yaw, wp_in.transform.rotation.yaw))) if back else wp_in
    names = {"1000001": "light", "206": "stop", "205": "yield"}
    return sorted({names[lm.type] for lm in start.get_landmarks(45.0, False) if lm.type in names})


def junctions(m):
    seen = {}
    for a, b in m.get_topology():
        for w in (a, b):
            if w.is_junction:
                seen[w.get_junction().id] = w.get_junction()
    return seen


def connectors(j):
    """(entry outside, exit outside, turn deg, entry in junction) per driving connector of j."""
    out = []
    for en, ex in j.get_waypoints(carla.LaneType.Driving):
        pin, pout = outside(en, False), outside(ex, True)
        if pin is not None and pout is not None:
            out.append((pin, pout, yaw_diff(ex.transform.rotation.yaw, en.transform.rotation.yaw), en))
    return out


def junction_arms(j):
    arms, exits = {}, defaultdict(dict)
    for pin, pout, turn, _ in connectors(j):
        kind = "straight" if abs(turn) < 30 else ("right" if turn > 0 else "left")  # CARLA yaw is clockwise
        arm = arms.setdefault(pin.road_id, {"lanes": set(), "wps": {}, "turns": defaultdict(set)})
        arm["lanes"].add(pin.lane_id)
        arm["wps"][pin.lane_id] = pin
        arm["turns"][kind].add((pout.road_id, pout.lane_id))
        exits[pout.road_id][pout.lane_id] = pout
    for a in arms.values():
        a["heading"] = next(iter(a["wps"].values())).transform.rotation.yaw % 360.0
    return arms, exits


def four_way(arms):
    """Four arms whose headings are about 90 deg apart (each within 25 deg)."""
    if len(arms) != 4:
        return False
    hs = sorted(a["heading"] for a in arms.values())
    return all(abs((hs[(i + 1) % 4] - hs[i]) % 360.0 - 90.0) < 25.0 for i in range(4))


# --- lanelet2 side ---------------------------------------------------------------------------------

def parse_origin(osm):
    """(lat, lon) for lanelet2's UTM parsing projector near the map `osm` (issue 084: at (0, 0),
    zone 31, GeographicLib refuses every point more than 60 deg of longitude away): a transverse
    Mercator map's map_origin beside it ((0, 0) for CARLA's, so their frame is unchanged),
    else the first node's."""
    import re
    from pathlib import Path
    info = Path(osm).parent / "map_projector_info.yaml"
    if info.is_file():
        import yaml
        doc = yaml.safe_load(info.read_text()) or {}
        if doc.get("projector_type") == "TransverseMercator":
            o = doc.get("map_origin") or {}
            return float(o.get("latitude") or 0.0), float(o.get("longitude") or 0.0)
    with open(osm) as f:
        m = re.search(r'<node[^>]*\blat="([-0-9.]+)"[^>]*\blon="([-0-9.]+)"', f.read(1 << 16))
    return (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)


class LaneletMap:
    def __init__(self, path):
        import lanelet2
        from lanelet2.io import Origin
        from lanelet2.projection import UtmProjector
        self.ll2 = lanelet2
        self.map, _ = lanelet2.io.loadRobust(path, UtmProjector(Origin(*parse_origin(path))))
        for pt in self.map.pointLayer:
            if "local_x" in pt.attributes:
                pt.x = float(pt.attributes["local_x"])
                pt.y = float(pt.attributes["local_y"])
        rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                              lanelet2.traffic_rules.Participants.Vehicle)
        self.graph = lanelet2.routing.RoutingGraph(self.map, rules)
        self.roads = [ll for ll in self.map.laneletLayer if ll.attributes["subtype"] == "road"]
        self.special = defaultdict(list)
        for ls in self.map.lineStringLayer:
            t = self.attr(ls, "type")
            if t in ("stop_line", "traffic_sign", "traffic_light"):
                c = [(p.x, p.y) for p in ls]
                self.special[t].append((ls.id, self.attr(ls, "subtype") or "",
                                        sum(p[0] for p in c) / len(c), sum(p[1] for p in c) / len(c)))
        self.ref_by_reg = defaultdict(set)  # linestring id -> regulatory elements referencing it
        self.reg_count = Counter()
        for r in self.map.regulatoryElementLayer:
            self.reg_count[self.attr(r, "subtype")] += 1
            for prims in r.parameters.values():
                for p in prims:
                    if hasattr(p, "id"):
                        self.ref_by_reg[p.id].add(r.id)

    @staticmethod
    def attr(e, k):
        return e.attributes[k] if k in e.attributes else None

    def nearest_road(self, x_ros, y_ros, yaw_ros_deg, at_end):
        """The road lanelet whose centreline end (or start) is nearest (x, y), heading alike."""
        best, bd = None, 1e9
        for ll in self.roads:
            c = ll.centerline
            if len(c) < 2:
                continue
            p, q = (c[len(c) - 2], c[len(c) - 1]) if at_end else (c[0], c[1])
            if abs(yaw_diff(math.degrees(math.atan2(q.y - p.y, q.x - p.x)), yaw_ros_deg)) > 30:
                continue
            e = c[len(c) - 1] if at_end else c[0]
            d = math.hypot(e.x - x_ros, e.y - y_ros)
            if d < bd:
                best, bd = ll, d
        return best, bd

    def regs(self, ll):
        return [f"{self.attr(r, 'subtype')}:{r.id}" for r in ll.regulatoryElements]

    def near(self, kind, x, y, r):
        return [[i, sub, round(math.hypot(px - x, py - y), 1), sorted(self.ref_by_reg.get(i, ()))]
                for i, sub, px, py in self.special[kind] if math.hypot(px - x, py - y) < r]

    def lane_change(self, ll):
        f = lambda e: e.id if e else None  # noqa: E731
        return {"left": f(self.graph.left(ll)), "right": f(self.graph.right(ll)),
                "adjacent_left_no_change": f(self.graph.adjacentLeft(ll)),
                "adjacent_right_no_change": f(self.graph.adjacentRight(ll))}

    def speed(self, ll):
        v = self.attr(ll, "speed_limit")
        return float(v) if v else None


# --- E6 --------------------------------------------------------------------------------------------
def e6(town, m, lm):
    rows = []
    for jid, j in sorted(junctions(m).items()):
        arms, exits = junction_arms(j)
        bb = j.bounding_box
        cx, cy = bb.location.x, bb.location.y
        rec = {"town": town, "junction": jid, "centre_carla": [round(cx, 1), round(cy, 1)],
               "centre_ros": [round(cx, 1), round(-cy, 1)], "extent": [round(bb.extent.x, 1), round(bb.extent.y, 1)],
               "n_arms": len(arms), "four_way": four_way(arms), "arms": []}
        for road, a in sorted(arms.items(), key=lambda kv: kv[1]["heading"]):
            lanes = sorted(a["lanes"], key=abs)
            inner = a["wps"][lanes[0]]  # the lane next to the centre line
            ln, why, _ = straight_walk(inner, False)
            lt, why_t, passed = straight_walk(inner, False, through=True)
            out_len = out_thr = out_road = None
            for (orr, oln) in sorted(a["turns"].get("straight", ())):
                L, _, _ = straight_walk(exits[orr][oln], True)
                Lt, _, _ = straight_walk(exits[orr][oln], True, through=True)
                if out_len is None or L > out_len:
                    out_len, out_thr, out_road = L, Lt, orr
            arm = {"road": road, "lanes": lanes, "heading_carla": round(a["heading"], 1),
                   "entry_carla": [round(v, 2) for v in xy(inner)],
                   "straight_in_m": ln, "in_stops_at": why,
                   "straight_in_through_m": lt, "in_through_stops_at": why_t, "in_through_junctions": passed,
                   "straight_out_m": out_len, "straight_out_through_m": out_thr, "out_road": out_road,
                   "turns": {k: len(v) for k, v in a["turns"].items()}, "xodr_signals": signals_before(inner)}
            if lm is not None:
                x, y = xy(inner)
                ll, d = lm.nearest_road(x, -y, -a["heading"], True)
                arm["lanelet"] = ll.id if ll is not None and d < 4.0 else None
                if arm["lanelet"] is not None:
                    nexts = lm.graph.following(ll)
                    e = ll.centerline[len(ll.centerline) - 1]
                    arm.update({
                        "lanelet_regs": lm.regs(ll), "speed_limit_kmh": lm.speed(ll),
                        "junction_lanelets": {str(n.id): lm.attr(n, "turn_direction") for n in nexts},
                        "junction_lanelet_regs": sorted({r for n in nexts for r in lm.regs(n)}),
                        "stop_lines": lm.near("stop_line", e.x, e.y, NEAR_STOP),
                        "traffic_signs": lm.near("traffic_sign", e.x, e.y, 25.0),
                        "lights_near": lm.near("traffic_light", e.x, e.y, 40.0)})
            rec["arms"].append(arm)
        if rec["arms"]:
            A = rec["arms"]
            rec["min_straight_in_m"] = min(a["straight_in_m"] for a in A)
            rec["min_straight_in_through_m"] = min(a["straight_in_through_m"] for a in A)
            outs = [a["straight_out_m"] for a in A]
            rec["min_straight_out_m"] = None if None in outs else min(outs)
            rec["xodr_lights"] = sum("light" in a["xodr_signals"] for a in A)
            rec["xodr_stops"] = sum("stop" in a["xodr_signals"] for a in A)
            rec["xodr_yields"] = sum("yield" in a["xodr_signals"] for a in A)
            rec["lanelet_lights_linked"] = sum(any(r.startswith("traffic_light") for r in a.get("lanelet_regs", [])) for a in A)
            rec["lanelet_stop_lines"] = sum(bool(a.get("stop_lines")) for a in A)
            rec["lanelet_stop_signs"] = sum(any(s[1] in ("de206", "stop_sign") for s in a.get("traffic_signs", [])) for a in A)
            rec["lanelet_right_of_way"] = sum(any("right_of_way" in r for r in a.get("junction_lanelet_regs", []) + a.get("lanelet_regs", []))
                                              for a in A)
        rows.append(rec)
    return rows


# --- E7 --------------------------------------------------------------------------------------------
def xodr_lane_changes(town, xodr_text, m):
    """Driving lanes that taper to (or from) zero width inside a lane section, and lane-count
    changes between sections, outside junctions. Directions are in traffic's sense."""
    out = []
    root = ET.fromstring(xodr_text)
    for road in root.findall("road"):
        if road.get("junction") != "-1":
            continue
        rid, L = int(road.get("id")), float(road.get("length"))
        secs = road.find("lanes").findall("laneSection")
        counts = []
        for i, sec in enumerate(secs):
            s0 = float(sec.get("s"))
            s1 = float(secs[i + 1].get("s")) if i + 1 < len(secs) else L
            c = {}
            for side in ("left", "right"):
                sd = sec.find(side)
                lanes = [ln for ln in sd.findall("lane") if ln.get("type") == "driving"] if sd is not None else []
                c[side] = sorted(int(ln.get("id")) for ln in lanes)
                ids = sorted(abs(int(ln.get("id"))) for ln in lanes)
                for ln in lanes:
                    ws = ln.findall("width")
                    lid = int(ln.get("id"))
                    for k, wv in enumerate(ws):
                        r0 = s0 + float(wv.get("sOffset"))
                        r1 = s0 + float(ws[k + 1].get("sOffset")) if k + 1 < len(ws) else s1
                        a, b, cc, d = (float(wv.get(q)) for q in "abcd")
                        ds = r1 - r0
                        wa, wb = a, a + b * ds + cc * ds * ds + d * ds ** 3
                        if not ((wa >= 2.5 and wb < 0.6) or (wa < 0.6 and wb >= 2.5)):
                            continue
                        # right lanes (id < 0) run with s, left lanes against it
                        ends = (wb < 0.6) == (lid < 0)
                        s_full, s_zero = (r0, r1) if wb < 0.6 else (r1, r0)
                        # full width in traffic's sense before the taper (drop) or after it (add), in this section
                        full = (r0 - s0) if (wb < 0.6) else (s1 - r1)
                        side = ("left" if abs(lid) == ids[0] else "right" if abs(lid) == ids[-1] else "middle")
                        rec = {"town": town, "kind": "lane drop" if ends else "lane add", "road": rid, "lane": lid,
                               "driver_side": side, "driving_lanes_same_way": len(ids),
                               "taper_s": [round(r0, 1), round(r1, 1)], "taper_m": round(ds, 1),
                               "full_width_m": round(full, 1), "width_from_to": [round(wa, 2), round(wb, 2)],
                               "cubic_like_arena": abs(b) < 1e-6 and abs(abs(cc) - 3 * max(wa, wb) / ds ** 2) < 1e-4}
                        try:
                            wf = m.get_waypoint_xodr(rid, lid, min(max(s_full, 0.1), L - 0.1))
                            wz = m.get_waypoint_xodr(rid, lid, min(max(s_zero, 0.1), L - 0.1))
                            rec["full_width_carla"] = [round(v, 1) for v in xy(wf)] if wf else None
                            rec["zero_width_carla"] = [round(v, 1) for v in xy(wz)] if wz else None
                        except RuntimeError:
                            pass
                        out.append(rec)
            counts.append((float(sec.get("s")), c))
        for (sa, ca), (sb, cb) in zip(counts, counts[1:]):
            for side in ("left", "right"):
                if len(ca[side]) != len(cb[side]):
                    out.append({"town": town, "kind": "lane count change", "road": rid, "side": side, "at_s": sb,
                                "lanes_before": ca[side], "lanes_after": cb[side]})
    return out


def junction_merges(town, m):
    """Incoming lanes of two roads onto one outgoing lane: one straight on (< 10 deg), the other
    within 60 deg, over a connector of 30 m or more (a Y merge or an on-ramp, not a turn)."""
    out = []
    for jid, j in sorted(junctions(m).items()):
        by_exit = defaultdict(list)
        for c in connectors(j):
            by_exit[(c[1].road_id, c[1].lane_id)].append(c)
        for (orr, oln), conns in by_exit.items():
            gentle = [c for c in conns if abs(c[2]) < 60.0]
            if len({c[0].road_id for c in gentle}) < 2 or min(abs(c[2]) for c in gentle) > 10.0:
                continue
            if max(c[3].transform.location.distance(c[1].transform.location) for c in gentle) < 30.0:
                continue  # an ordinary intersection's right turn onto the straight-on exit
            pout = gentle[0][1]
            after, why_a, _ = straight_walk(pout, True)
            rec = {"town": town, "kind": "junction merge", "junction": jid, "out_road": orr, "out_lane": oln,
                   "exit_carla": [round(v, 1) for v in xy(pout)], "straight_after_m": after, "after_stops_at": why_a,
                   "inputs": []}
            for pin, _, turn, en in gentle:
                ln, why, _ = straight_walk(pin, False)
                rec["inputs"].append({"road": pin.road_id, "lane": pin.lane_id, "turn_deg": round(turn, 1),
                                      "entry_carla": [round(v, 1) for v in xy(pin)],
                                      "connector_chord_m": round(en.transform.location.distance(pout.transform.location), 1),
                                      "straight_before_m": ln, "before_stops_at": why})
            out.append(rec)
    return out


def lanelet_merges(town, lm, m):
    out = []
    ends = Counter()
    for ll in lm.roads:
        prev = list(lm.graph.previous(ll))
        if len(prev) >= 2 and all(lm.attr(p, "turn_direction") is None for p in prev):
            c = ll.centerline[0]
            out.append({"town": town, "kind": "lanelet merge", "lanelet": ll.id, "inputs": [p.id for p in prev],
                        "at_ros": [round(c.x, 1), round(c.y, 1)],
                        "inputs_lane_change": {str(p.id): lm.lane_change(p) for p in prev}})
        if lm.graph.following(ll):
            continue
        e = ll.centerline[len(ll.centerline) - 1]
        loc = carla.Location(e.x, -e.y, 0.0)
        w = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Any)
        if w is None or math.hypot(w.transform.location.x - loc.x, w.transform.location.y - loc.y) > 2.0:
            ends["no CARLA lane"] += 1
            continue
        cont = bool(w.next(3.0))
        ends[f"{w.lane_type} {'continues' if cont else 'ends'}"] += 1
        if w.lane_type == carla.LaneType.Driving:
            out.append({"town": town, "kind": "lanelet dead end on a CARLA driving lane", "lanelet": ll.id,
                        "at_ros": [round(e.x, 1), round(e.y, 1)], "carla_road": w.road_id, "carla_lane": w.lane_id,
                        "carla_in_junction": w.is_junction, "carla_continues": cont, "lane_change": lm.lane_change(ll)})
    return out, {k: v for k, v in sorted(ends.items())}


# --- plots -----------------------------------------------------------------------------------------
def plot_site(m, centre, half, title, path, marks=()):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    cx, cy = centre
    pts = {"drv": ([], []), "jn": ([], []), "other": ([], [])}
    for w in m.generate_waypoints(1.5):
        x, y = xy(w)
        if abs(x - cx) > half or abs(y - cy) > half:
            continue
        k = "other" if w.lane_type != carla.LaneType.Driving else ("jn" if w.is_junction else "drv")
        pts[k][0].append(x)
        pts[k][1].append(-y)
    ax.plot(*pts["other"], ".", ms=1, color="#ccc", label="other lanes")
    ax.plot(*pts["drv"], ".", ms=1.5, color="#555", label="driving")
    ax.plot(*pts["jn"], ".", ms=1.5, color="#39c", label="junction connectors")
    for label, (x, y), colour in marks:
        ax.plot(x, -y, "o", ms=6, color=colour)
        ax.annotate(label, (x, -y), fontsize=7, xytext=(3, 3), textcoords="offset points")
    ax.set_aspect("equal")
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(-cy - half, -cy + half)
    ax.set_xlabel("x (m, ROS = CARLA x)")
    ax.set_ylabel("y (m, ROS = -CARLA y)")
    ax.set_title(title, fontsize=9)
    ax.legend(fontsize=7, loc="lower right", markerscale=6)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--towns", default="Town03,Town05,Town10HD")
    ap.add_argument("--xodr-only", default="Town04,Town06", help="towns scanned for E7 on the OpenDRIVE alone")
    ap.add_argument("--xodr-dir", default=os.path.join(os.environ.get("CARLA_DIR", "/mnt/p5plus/aeon/carla/CARLA_0.9.16"),
                                                       "CarlaUE4/Content/Carla/Maps/OpenDrive"))
    ap.add_argument("--lanelet-dir", default=os.environ.get("CARLA_MAPS", os.path.join(REPO, "data/carla-autoware-bridge")))
    ap.add_argument("--plots", action="store_true")
    ap.add_argument("--v", type=float, default=10.0, help="m/s, for the v*H window (H 20 s)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    all_j, all_m, summary = [], [], {}
    towns = [(t, True) for t in args.towns.split(",") if t] + [(t, False) for t in args.xodr_only.split(",") if t]
    for town, full in towns:
        path = os.path.join(args.xodr_dir, town + ".xodr")
        if not os.path.exists(path):
            print(f"{town}: no OpenDRIVE at {path}", file=sys.stderr)
            continue
        text = open(path).read()
        m = carla.Map(town, text)
        p = os.path.join(args.lanelet_dir, LL_DIR_NAME.get(town, town), "lanelet2_map.osm")
        lm = LaneletMap(p) if os.path.exists(p) else None
        summary[town] = {"lanelet2_map": p if lm else None,
                         "regulatory_elements": dict(lm.reg_count) if lm else None,
                         "road_lanelets": len(lm.roads) if lm else None,
                         "road_lanelets_with_light": sum(any(lm.attr(r, "subtype") == "traffic_light" for r in ll.regulatoryElements)
                                                         for ll in lm.roads) if lm else None,
                         "landmarks": dict(Counter(lk.type for lk in m.get_all_landmarks()))}
        merges = xodr_lane_changes(town, text, m) + junction_merges(town, m)
        if lm:
            lmr, ends = lanelet_merges(town, lm, m)
            merges += lmr
            summary[town]["lanelet_dead_ends_by_carla_lane"] = ends
        all_m += merges
        if full:
            js = e6(town, m, lm)
            all_j += js
        if args.plots:
            for r in (js if full else []):
                if r["four_way"]:
                    marks = [(f"r{a['road']} {a['straight_in_m']:.0f}/{a['straight_in_through_m']:.0f} m "
                              f"{','.join(a['xodr_signals']) or 'none'}", a["entry_carla"], "#c50") for a in r["arms"]]
                    plot_site(m, r["centre_carla"], 90, f"{town} junction {r['junction']} (E6): arms, straight in "
                              "(to the next junction / through), OpenDRIVE signals",
                              os.path.join(args.out, f"e6_{town}_j{r['junction']}.png"), marks)
            for r in merges:
                if r["kind"] in ("lane drop", "lane add") and r.get("full_width_carla"):
                    c = [(a + b) / 2 for a, b in zip(r["full_width_carla"], r["zero_width_carla"])]
                    plot_site(m, c, 150, f"{town} road {r['road']} lane {r['lane']}: {r['kind']} over {r['taper_m']} m (E7)",
                              os.path.join(args.out, f"e7_{town}_r{r['road']}_l{r['lane']}.png"),
                              [("full width", r["full_width_carla"], "#c50"), ("zero width", r["zero_width_carla"], "#05c")])
                elif r["kind"] == "junction merge":
                    plot_site(m, r["exit_carla"], 120, f"{town} junction {r['junction']} merge onto r{r['out_road']} (E7)",
                              os.path.join(args.out, f"e7_{town}_j{r['junction']}_r{r['out_road']}.png"),
                              [(f"r{i['road']} in", i["entry_carla"], "#c50") for i in r["inputs"]]
                              + [("exit", r["exit_carla"], "#05c")])
    four = sorted([r for r in all_j if r["four_way"]], key=lambda r: (r["xodr_lights"] > 0, -r["min_straight_in_through_m"]))
    with open(os.path.join(args.out, "e6_junctions.json"), "w") as f:
        json.dump(all_j, f, indent=1)
    with open(os.path.join(args.out, "e7_merges.json"), "w") as f:
        json.dump(all_m, f, indent=1)
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1)
    cols = ["town", "junction", "centre_carla_x", "centre_carla_y", "arms_lanes", "arm_roads", "straight_in_m",
            "straight_in_through_m", "min_in_m", "min_in_through_m", "min_out_m", "xodr_lights", "xodr_stops",
            "lanelet_lights_linked", "lanelet_stop_lines", "lanelet_stop_signs", "lanelet_right_of_way",
            "approach_lanelets", "speed_kmh"]
    with open(os.path.join(args.out, "e6_four_way.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in four:
            A = r["arms"]
            j = lambda k, A=A: "/".join(str(a.get(k)) for a in A)  # noqa: E731
            w.writerow([r["town"], r["junction"], *r["centre_carla"], "/".join(str(len(a["lanes"])) for a in A), j("road"),
                        j("straight_in_m"), j("straight_in_through_m"), r["min_straight_in_m"], r["min_straight_in_through_m"],
                        r["min_straight_out_m"], r["xodr_lights"], r["xodr_stops"], r["lanelet_lights_linked"],
                        r["lanelet_stop_lines"], r["lanelet_stop_signs"], r["lanelet_right_of_way"], j("lanelet"),
                        j("speed_limit_kmh")])
    print(f"E6: four-way junctions, unsignalised (OpenDRIVE) first, by the shortest straight approach through "
          f"junctions (v*H = {args.v * 20:.0f} m at {args.v} m/s)")
    for r in four:
        A = r["arms"]
        print(f"  {r['town']:8s} j{r['junction']:<5d} centre {r['centre_carla']}  "
              f"in {'/'.join('%.0f' % a['straight_in_m'] for a in A)} m, through {'/'.join('%.0f' % a['straight_in_through_m'] for a in A)} m  "
              f"lanes {'/'.join(str(len(a['lanes'])) for a in A)}  xodr lights {r['xodr_lights']} stops {r['xodr_stops']}  "
              f"lanelet: lights linked {r['lanelet_lights_linked']}, stop lines {r['lanelet_stop_lines']}, "
              f"stop signs {r['lanelet_stop_signs']}, right_of_way {r['lanelet_right_of_way']}")
    stops = [r for r in all_j if r.get("xodr_stops")]
    print("E6: junctions with OpenDRIVE stop signs")
    for r in stops:
        print(f"  {r['town']:8s} j{r['junction']:<5d} centre {r['centre_carla']}  arms {r['n_arms']}  stops {r['xodr_stops']} "
              f"yields {r['xodr_yields']} lights {r['xodr_lights']}  lanelet stop signs {r['lanelet_stop_signs']}")
    print("E7: lane drops / adds / merges")
    kinds = Counter((r["town"], r["kind"]) for r in all_m)
    for town, _ in towns:
        print(f"  {town}: " + (", ".join(f"{k} {n}" for (t, k), n in sorted(kinds.items()) if t == town) or "none"))
    for r in all_m:
        print("   ", json.dumps({k: v for k, v in r.items() if k not in ("inputs_lane_change",)}))


if __name__ == "__main__":
    main()
