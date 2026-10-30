#!/usr/bin/env python3
"""Route B (roadmap 062, O3): OSM, Overture and Mapillary straight to an Autoware Lanelet2 map.

    scripts/open_maps/generate.py <site.yaml> <out>

Reads <out>/raw/ (fetch.py) and writes <out>/generated/: `lanelet2_map.osm`,
`map_projector_info.yaml` and `report.json`. Run with the tool venv (`data/maps/.venv`).

What it builds:

- **Roads.** Every car road in the box (OSM `highway` trunk to service), at ground level:
  underpasses and major roads raised for more than 60 m are left out (short bridges stay). Ways are chained between junctions. Each direction gets its lanes as offsets from
  the OSM centre line: two-way roads put the forward lanes right of the line and the backward
  lanes left; one-way roads are centred. Lane counts and widths come from `lanes`,
  `lanes:forward/backward` and `width`, else the road class's default.
- **Junctions.** Nodes where three or more road chains meet are clustered. Taipei's dual
  carriageways cross as four nodes 15-35 m apart, so nodes joined by a short one-way major
  segment, or both by a signal, become one junction. Each approach is cut back to a circle
  around the junction, and every allowed entry-to-exit lane pair gets a connecting lanelet
  (cubic bounds) with `turn_direction` from its change of heading. U-turns are not built.
- **Which lanes turn where**, per approach lane, first source that has it:
  1. OSM `turn:lanes`, when its lane count matches;
  2. Mapillary's painted arrows within 60 m of the stop line, voted per lane by their lateral
     place;
  3. the rule: the leftmost lane turns left, the rightmost right, all go straight.

  Then OSM turn restrictions, and Mapillary's no-left and no-right signs on the approach's
  right side, remove movements.
- **Speed limits**, per direction: OSM `maxspeed`, else Overture's `speed_limits` for the same
  OSM way, else a Mapillary speed-limit sign on the road's right side, else the rule (50 km/h
  on public roads, Taiwan's default; 30 on service roads).
- **Signals.** A junction with an OSM signal, or two or more of Mapillary's vehicle traffic
  lights, is signalised: each approach gets a stop line at its cut, a traffic-light linestring
  across the junction, and a `traffic_light` regulatory element referenced by its lanes.
- **Crosswalks** from OSM `footway=crossing` ways that cross a road.

Every lanelet records where its attributes came from: `swb:source` (geometry), `swb:speed_source`,
`swb:turn_source` (connectors), `swb:osm_way`. The frame is the site's MGRS-local frame;
`local_x`/`local_y` are on every node.
"""
import argparse
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path

import yaml
from pyproj import Transformer
from shapely.geometry import LineString, Point

CAR = {"trunk", "primary", "secondary", "tertiary", "unclassified", "residential", "service",
       "living_street", "trunk_link", "primary_link", "secondary_link", "tertiary_link"}
MAJOR = {"trunk", "primary", "secondary", "tertiary", "trunk_link", "primary_link",
         "secondary_link", "tertiary_link"}
SKIP_SERVICE = {"parking_aisle", "driveway", "drive-through", "emergency_access"}
LANES_DEFAULT = {"trunk": 2, "primary": 2, "secondary": 2}          # per direction; else 1
WIDTH_DEFAULT = {"trunk": 3.5, "primary": 3.5, "secondary": 3.5, "tertiary": 3.25}  # else 3.0
SPEED_RULE = {"service": 30}                                          # else 50 (Taiwan's default)
STEP = 4.0          # m between bound nodes on straight-ish roads
TURN_DEG = 30.0     # |heading change| below this is straight
UTURN_DEG = 150.0
ARROWS = {
    "marking--discrete--arrow--straight": {"straight"},
    "marking--discrete--arrow--left": {"left"},
    "marking--discrete--arrow--right": {"right"},
    "marking--discrete--arrow--split-left-or-straight": {"left", "straight"},
    "marking--discrete--arrow--split-right-or-straight": {"right", "straight"},
    "marking--discrete--arrow--split-left-or-right": {"left", "right"},
    "marking--discrete--arrow--split-left-or-right-or-straight": {"left", "right", "straight"},
}
DEBUG_AT = tuple(map(float, __import__('os').environ['SWB_DEBUG_AT'].split(','))) \
    if __import__('os').environ.get('SWB_DEBUG_AT') else None
STATS = Counter()   # counts from the network build, merged into report.json
BAN_LOG = []   # every sign ban, for review (report.json's bans)
NO_TURN = {"regulatory--no-left-turn--g1": "left", "regulatory--no-right-turn--g1": "right"}
TURN_TOKENS = {"left": "left", "sharp_left": "left", "slight_left": "left",
               "right": "right", "sharp_right": "right", "slight_right": "right",
               "through": "straight", "merge_to_left": "straight", "merge_to_right": "straight"}


class Frame:
    def __init__(self, f):
        self.fwd = Transformer.from_crs("EPSG:4326", f"EPSG:{f['utm_epsg']}", always_xy=True)
        self.inv = Transformer.from_crs(f"EPSG:{f['utm_epsg']}", "EPSG:4326", always_xy=True)
        self.ox, self.oy = f["mgrs_offset"]

    def xy(self, lat, lon):
        e, n = self.fwd.transform(lon, lat)
        return e - self.ox, n - self.oy

    def ll(self, x, y):
        lon, lat = self.inv.transform(x + self.ox, y + self.oy)
        return lat, lon


def num(v, default=None):
    try:
        return float(str(v).split(";")[0].strip())
    except (TypeError, ValueError):
        return default


def speed_kmh(v):
    """'50', '50 km/h', '50kmh', '30 mph', '50|50|30' (per lane: the lowest)."""
    if not v:
        return None
    vals = []
    for part in str(v).split("|"):
        m = re.match(r"\s*(\d+(?:\.\d+)?)\s*(mph)?", part)
        if m:
            vals.append(float(m.group(1)) * (1.609 if m.group(2) else 1.0))
    return min(vals) if vals else None


def heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def wrap(a):
    while a <= -math.pi:
        a += 2 * math.pi
    while a > math.pi:
        a -= 2 * math.pi
    return a


def resample(line, step=STEP):
    """Nodes every `step` m, kept closer where the line bends (no more than 5 deg between)."""
    L = line.length
    if L < 1e-6:
        return list(line.coords)
    pts = [line.interpolate(0)]
    s = 0.0
    while s < L:
        ds = step
        a = line.interpolate(s)
        b = line.interpolate(min(L, s + 1.0))
        c = line.interpolate(min(L, s + 2.0))
        bend = abs(wrap(heading((b.x, b.y), (c.x, c.y)) - heading((a.x, a.y), (b.x, b.y))))
        if bend > 1e-3:
            ds = max(1.0, min(step, math.radians(5.0) / bend))
        s = min(L, s + ds)
        pts.append(line.interpolate(s))
    return [(p.x, p.y) for p in pts]


def offset(line, d):
    """The line `d` m to its left (negative: right), same direction."""
    if abs(d) < 1e-6:
        return line
    o = line.offset_curve(d, join_style="round", quad_segs=4)
    if o.geom_type != "LineString" or o.is_empty or o.length < 0.2 * line.length:
        # degenerate on a tight bend: shift each vertex along its normal instead
        cs = list(line.coords)
        out = []
        for i, p in enumerate(cs):
            a, b = cs[max(0, i - 1)], cs[min(len(cs) - 1, i + 1)]
            h = heading(a, b)
            out.append((p[0] - d * math.sin(h), p[1] + d * math.cos(h)))
        return LineString(out)
    return o


def cubic(p0, h0, p3, h3, n):
    d = math.dist(p0, p3) * 0.4
    p1 = (p0[0] + d * math.cos(h0), p0[1] + d * math.sin(h0))
    p2 = (p3[0] - d * math.cos(h3), p3[1] - d * math.sin(h3))
    out = []
    for i in range(n + 1):
        t = i / n
        u = 1 - t
        out.append((u ** 3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t ** 3 * p3[0],
                    u ** 3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t ** 3 * p3[1]))
    return out


# ---------------------------------------------------------------------------- the writer

class Osm:
    def __init__(self, frame):
        self.frame = frame
        self.nodes, self.ways, self.rels = [], [], []
        self.next = 1

    def node(self, xy, ele=0.0):
        i = self.next
        self.next += 1
        self.nodes.append((i, xy, ele))
        return i

    def way(self, nids, tags):
        i = self.next
        self.next += 1
        self.ways.append((i, nids, tags))
        return i

    def rel(self, members, tags):
        i = self.next
        self.next += 1
        self.rels.append((i, members, tags))
        return i

    def write(self, path):
        from xml.sax.saxutils import quoteattr as q
        with open(path, "w", encoding="utf-8") as f:
            f.write("<?xml version='1.0' encoding='UTF-8'?>\n<osm version=\"0.6\" generator=\"swb open_maps\">\n")
            for i, (x, y), ele in self.nodes:
                lat, lon = self.frame.ll(x, y)
                f.write(f'  <node id="{i}" lat="{lat:.9f}" lon="{lon:.9f}">'
                        f'<tag k="local_x" v="{x:.3f}"/><tag k="local_y" v="{y:.3f}"/>'
                        f'<tag k="ele" v="{ele:.2f}"/></node>\n')
            for i, nids, tags in self.ways:
                f.write(f'  <way id="{i}">' + "".join(f'<nd ref="{n}"/>' for n in nids))
                f.write("".join(f"<tag k={q(k)} v={q(str(v))}/>" for k, v in tags.items()))
                f.write("</way>\n")
            for i, members, tags in self.rels:
                f.write(f'  <relation id="{i}">')
                f.write("".join(f'<member type="{t}" ref="{r}" role="{role}"/>' for t, r, role in members))
                f.write("".join(f"<tag k={q(k)} v={q(str(v))}/>" for k, v in tags.items()))
                f.write("</relation>\n")
            f.write("</osm>\n")


# ---------------------------------------------------------------------------- inputs

def load_osm(raw, bbox, frame, margin=0.0005):
    d = json.loads((raw / "osm.json").read_text())
    s, w, n, e = bbox
    nodes, ways, rels = {}, [], []
    for el in d["elements"]:
        if el["type"] == "node":
            nodes[el["id"]] = el
        elif el["type"] == "way":
            ways.append(el)
        else:
            rels.append(el)
    inside = {i for i, x in nodes.items()
              if s - margin <= x["lat"] <= n + margin and w - margin <= x["lon"] <= e + margin}
    xy = {i: frame.xy(nodes[i]["lat"], nodes[i]["lon"]) for i in inside}
    return nodes, ways, rels, inside, xy


def car_road(t):
    h = t.get("highway")
    if h not in CAR or t.get("area") == "yes":
        return False
    if t.get("service") in SKIP_SERVICE:
        return False
    for k in ("motor_vehicle", "motorcar", "vehicle", "access"):
        if t.get(k) == "no":
            return False
    # a bus-only way: bus or psv designated, and nothing letting cars on (羅斯福路大客車專用道
    # is a primary with bus=designated alone in 12 of its ways)
    if "designated" in (t.get("bus"), t.get("psv")) and not any(
            t.get(k) in ("yes", "designated", "destination", "permissive")
            for k in ("motor_vehicle", "motorcar", "vehicle", "access")):
        return False
    if num(t.get("layer"), 0) < 0 or t.get("tunnel") in ("yes", "building_passage"):
        return False        # underpasses (辛亥路車行地下道) lie under the roads they cross
    return True


def elevated(t, length):
    """A viaduct the map would lay over the roads below it (基隆路高架, the 建國 ramps): a major
    road above ground for more than 60 m. Short bridges over the canals (舟山路) are kept."""
    return num(t.get("layer"), 0) > 0 and h_class(t) in MAJOR and length > 60.0


def h_class(t):
    return t.get("highway")


def overture_speeds(raw):
    """OSM way id -> km/h, from Overture segments whose source is that way."""
    p = raw / "overture_segment.parquet"
    if not p.exists():
        return {}
    import duckdb
    rows = duckdb.sql(f"SELECT sources, speed_limits FROM '{p}' WHERE speed_limits IS NOT NULL").fetchall()
    out = {}
    for sources, limits in rows:
        ids = [re.match(r"w(\d+)", s.get("record_id") or "") for s in (sources or [])]
        ids = [int(m.group(1)) for m in ids if m]
        vals = []
        for lim in limits or []:
            mx = (lim or {}).get("max_speed") or {}
            if mx.get("value"):
                vals.append(mx["value"] * (1.609 if mx.get("unit") == "mph" else 1.0))
        if vals:
            for i in ids:
                out[i] = min(vals)
    return out


# ---------------------------------------------------------------------------- the network

class Chain:
    """A run of road between junctions, along the OSM way's direction."""

    def __init__(self, way, nids, pts):
        self.way, self.nids, self.pts = way, nids, pts
        t = way.get("tags", {})
        self.t = t
        self.cls = t.get("highway")
        self.name = t.get("name", "")
        ow = t.get("oneway")
        self.fwd_ok = ow != "-1"
        self.bwd_ok = not (ow in ("yes", "true", "1", "-1") or t.get("junction") == "roundabout")
        if ow == "-1":
            self.pts, self.nids = pts[::-1], nids[::-1]
            self.bwd_ok = False
            self.fwd_ok = True
        self.oneway = not self.bwd_ok
        lanes = num(t.get("lanes"))
        d = LANES_DEFAULT.get(self.cls.replace("_link", ""), 1)
        if self.oneway:
            self.nf, self.nb = int(lanes) if lanes else d, 0
        else:
            f, b = num(t.get("lanes:forward")), num(t.get("lanes:backward"))
            if f and b:
                self.nf, self.nb = int(f), int(b)
            elif lanes and lanes >= 2:
                self.nf = int(f) if f else int(lanes - (b or lanes // 2))
                self.nb = int(lanes) - self.nf
            else:
                self.nf = self.nb = d
            self.nf, self.nb = max(1, self.nf), max(1, self.nb)
        width = num(t.get("width"))
        wd = WIDTH_DEFAULT.get(self.cls.replace("_link", ""), 3.0)
        self.w = min(4.5, max(2.6, width / (self.nf + self.nb))) if width else wd
        self.width_source = "osm" if width else "rule"


def build(site, out):
    raw = out / "raw"
    frame = Frame(site["frame"])
    nodes, ways, rels, inside, xy = load_osm(raw, site["bbox"], frame)
    roads, raised = [], set()
    for w in ways:
        t = w.get("tags", {})
        if not car_road(t):
            continue
        pts = [xy[i] for i in w["nodes"] if i in xy]
        if len(pts) >= 2 and elevated(t, LineString(pts).length):
            raised.update(w["nodes"])
            continue
        roads.append(w)
    # the ramps of a viaduct left out go with it (辛亥路出口匝道 led down from nothing), and so
    # does a ramp that only led to such a ramp
    while True:
        ramps = [w for w in roads if w["tags"].get("highway", "").endswith("_link")
                 and raised & set(w["nodes"])]
        if not ramps:
            break
        for w in ramps:
            raised.update(w["nodes"])
        roads = [w for w in roads if w not in ramps]
        STATS["viaduct_ramps_dropped"] += len(ramps)
    # keep each way's run of nodes inside the box (relations pulled in ways from far away)
    pieces = []
    clipped = set()     # run ends where the box cut the way: dead ends that are not real
    for w in roads:
        run = []
        for i, nid in enumerate(w["nodes"]):
            if nid in inside:
                if not run and i > 0:
                    clipped.add(nid)
                run.append(nid)
            else:
                if run:
                    clipped.add(run[-1])
                if len(run) >= 2:
                    pieces.append((w, run))
                run = []
        if len(run) >= 2:
            pieces.append((w, run))
    use = Counter()
    for w, run in pieces:
        for nid in set(run):
            use[nid] += 1
        use[run[0]] += 1
        use[run[-1]] += 1
    chains = []
    for w, run in pieces:
        cut = [0] + [i for i in range(1, len(run) - 1) if use[run[i]] >= 2] + [len(run) - 1]
        for a, b in zip(cut, cut[1:]):
            ids = run[a:b + 1]
            pts = [xy[i] for i in ids]
            if LineString(pts).length > 0.5:
                chains.append(Chain(w, ids, pts))
    # stubs: a short chain from a junction to nowhere (新生南路's 5-lane piece that ends where
    # only the bus lanes go on, at the main gate) would be an exit into a dead end; drop it,
    # and again for what that leaves
    while True:
        ends = defaultdict(list)
        for c in chains:
            ends[c.nids[0]].append((c, 0))
            ends[c.nids[-1]].append((c, -1))
        stubs = set()
        for c in chains:
            for a, b in ((c.nids[0], c.nids[-1]), (c.nids[-1], c.nids[0])):
                if (len(ends[a]) == 1 and a not in clipped and len(ends[b]) >= 3
                        and LineString(c.pts).length < 40.0):
                    stubs.add(id(c))
        if not stubs:
            break
        chains = [c for c in chains if id(c) not in stubs]
        STATS["stub_chains_dropped"] += len(stubs)
    # junctions: nodes where three or more chain ends meet
    jnodes = {n for n, v in ends.items() if len(v) >= 3}
    signals = [xy[i] for i in inside
               if nodes[i].get("tags", {}).get("highway") == "traffic_signals"]
    feats = json.loads((raw / "mapillary_features.json").read_text()) if (raw / "mapillary_features.json").exists() else []
    for f in feats:
        lon, lat = f["geometry"]["coordinates"]
        f["xy"] = frame.xy(lat, lon)

    def near_signal(p, r=30.0):
        return any(math.dist(p, s) < r for s in signals)
    # cluster: two junction nodes join when a short chain links them and it is a one-way major
    # road (a dual carriageway's half) or both are by a signal
    parent = {n: n for n in jnodes}

    def find(n):
        while parent[n] != n:
            parent[n] = parent[parent[n]]
            n = parent[n]
        return n
    for c in chains:
        a, b = c.nids[0], c.nids[-1]
        if a in jnodes and b in jnodes and a != b:
            L = LineString(c.pts).length
            if L < 40 and ((c.oneway and c.cls in MAJOR) or (near_signal(xy[a]) and near_signal(xy[b]))):
                parent[find(a)] = find(b)
    clusters = defaultdict(list)
    for n in jnodes:
        clusters[find(n)].append(n)
    junction_of = {}
    J = []
    for root, members in clusters.items():
        pts = [xy[n] for n in members]
        cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
        spread = max(math.dist((cx, cy), p) for p in pts)
        j = {"id": len(J), "nodes": members, "c": (cx, cy), "spread": spread, "joint": False}
        for n in members:
            junction_of[n] = j
        J.append(j)
    # joints: two chains meeting end to end (OSM splits ways at every tag change); their lanes
    # are joined by short connectors, with no turn_direction
    for n, v in ends.items():
        if len(v) == 2 and n not in junction_of:
            j = {"id": len(J), "nodes": [n], "c": xy[n], "spread": 0.0, "joint": True}
            junction_of[n] = j
            J.append(j)
    # chains inside one junction (the median crossings) are dropped
    keep = []
    for c in chains:
        ja, jb = junction_of.get(c.nids[0]), junction_of.get(c.nids[-1])
        if ja is not None and ja is jb:
            continue
        keep.append(c)
    chains = keep
    for j in J:
        if j["joint"]:
            j["R"], j["signal"] = 2.0, None
            continue
        hw = [(c.nf + c.nb) * c.w / 2 for c in chains
              if junction_of.get(c.nids[0]) is j or junction_of.get(c.nids[-1]) is j]
        # each approach is cut this far from its own node in the junction, which clears the
        # widest road it crosses (a cluster's other carriageway is crossed by the connector)
        j["R"] = max(5.0, (max(hw) if hw else 3.0) + 2.0)
        j["signal"] = "osm" if any(math.dist(j["c"], s) < j["R"] + 20 for s in signals) else None
        if j["signal"] is None:
            lights = [f for f in feats if f["object_value"].startswith("object--traffic-light--general")
                      and math.dist(j["c"], f["xy"]) < j["R"] + 15]
            if len(lights) >= 2:
                j["signal"] = "mapillary"
    # cut each chain back at its junctions
    for c in chains:
        line = LineString(c.pts)
        L = line.length
        s0, s1 = 0.0, L
        ja, jb = junction_of.get(c.nids[0]), junction_of.get(c.nids[-1])
        if ja:
            a = xy[c.nids[0]]
            s0 = next((s for s in [k * 0.5 for k in range(int(L / 0.5) + 1)]
                       if math.dist(a, (line.interpolate(s).x, line.interpolate(s).y)) >= ja["R"]), L)
        if jb:
            b = xy[c.nids[-1]]
            s1 = next((L - s for s in [k * 0.5 for k in range(int(L / 0.5) + 1)]
                       if math.dist(b, (line.interpolate(L - s).x, line.interpolate(L - s).y)) >= jb["R"]), 0.0)
        if s1 - s0 < 2.0:   # too short to keep a lane: keep its middle 2 m
            m = (s0 + s1) / 2 if s1 > s0 else L / 2
            s0, s1 = max(0.0, m - 1.0), min(L, m + 1.0)
        from shapely.ops import substring
        c.line = LineString(resample(substring(line, s0, s1)))
        c.full = line
        c.ja, c.jb = ja, jb
    return frame, nodes, rels, chains, J, junction_of, feats


# ---------------------------------------------------------------------------- lanes

class Lane:
    def __init__(self, chain, direction, i, n):
        self.chain, self.dir, self.i, self.n = chain, direction, i, n
        self.left = self.right = None   # way ids, oriented along travel
        self.lid = None


def lanes_of(osm, chain, report):
    """Bound ways and lane objects for both directions of a chain."""
    w = chain.w
    line = chain.line
    edge = {"type": "line_thin", "subtype": "solid"}
    dash = {"type": "line_thin", "subtype": "dashed"}
    if chain.oneway:
        offs = [(chain.nf / 2 - k) * w for k in range(chain.nf + 1)]
    else:
        offs = [-k * w for k in range(chain.nf + 1)]
    node_ids = {}

    def line_nodes(d):
        if d not in node_ids:
            o = offset(line, d)
            node_ids[d] = [osm.node(p) for p in resample(o)]
        return node_ids[d]
    out = {}
    # forward lanes, leftmost first
    fw = []
    for k in range(chain.nf):
        dl, dr = offs[k], offs[k + 1]
        ln = Lane(chain, +1, k, chain.nf)
        tl = edge if k == 0 else dash
        tr = edge if k == chain.nf - 1 else dash
        ln.left = (line_nodes(dl), tl)
        ln.right = (line_nodes(dr), tr)
        fw.append(ln)
    out[+1] = fw
    if not chain.oneway:
        bw = []
        for k in range(chain.nb):
            dl, dr = k * w, (k + 1) * w     # left of the way; travel reversed
            ln = Lane(chain, -1, k, chain.nb)
            tl = edge if k == 0 else dash
            tr = edge if k == chain.nb - 1 else dash
            ln.left = (line_nodes(dl)[::-1], tl)
            ln.right = (line_nodes(dr)[::-1], tr)
            bw.append(ln)
        out[-1] = bw
    # shared bound ways: one way per (node list, orientation)
    made = {}
    for d, lst in out.items():
        for ln in lst:
            for side in ("left", "right"):
                nids, tags = getattr(ln, side)
                key = (nids[0], nids[-1])
                if key not in made:
                    made[key] = osm.way(nids, dict(tags))
                setattr(ln, side, (made[key], nids))
    return out


def travel_frame(chain, direction):
    """(line along travel, the lanes' left edge offset in the travel frame)."""
    if direction > 0:
        return chain.line, (chain.nf / 2 * chain.w if chain.oneway else 0.0)
    return LineString(list(chain.line.coords)[::-1]), 0.0


def approach_frame(chain, direction):
    """(the whole road along travel, the lanes' left edge offset, where the lanes end on it).
    Signs and arrows are placed against the whole road, not the cut lanes: a point beyond a
    line's end projects onto the end, and read as 0 m from it (a sign 570 m away once banned a
    turn at the main gate)."""
    full = chain.full if direction > 0 else LineString(list(chain.full.coords)[::-1])
    line, left = travel_frame(chain, direction)
    end = line.coords[-1]
    return full, left, full.project(Point(end))


def lateral(line, p):
    """(distance along, signed offset: + left) of point p from line, or None when p lies beyond
    either end (its projection clamps there)."""
    s = line.project(Point(p))
    if s <= 1e-6 or s >= line.length - 1e-6:
        return None
    a = line.interpolate(max(0.0, s - 0.5))
    b = line.interpolate(min(line.length, s + 0.5))
    h = heading((a.x, a.y), (b.x, b.y))
    q = line.interpolate(s)
    dx, dy = p[0] - q.x, p[1] - q.y
    return s, -math.sin(h) * dx + math.cos(h) * dy


def faces(f, line, s):
    """A sign faces the traffic it governs: Mapillary's `aligned_direction` (the compass bearing
    the sign faces) within 45 deg of the travel heading's reverse. Measured on this site's one-way
    roads (2026-10-30): signs on the right side peak at 150-210 deg from the travel heading."""
    ad = f.get("aligned_direction")
    if ad is None:
        return True
    a = line.interpolate(max(0.0, s - 1.0))
    b = line.interpolate(min(line.length, s + 1.0))
    compass = (90 - math.degrees(math.atan2(b.y - a.y, b.x - a.x))) % 360
    return abs((ad - compass) % 360 - 180) < 45


def generate(site, out):
    frame, nodes, rels, chains, J, junction_of, feats = build(site, out)
    raw = out / "raw"
    ov = overture_speeds(raw)
    osm = Osm(frame)
    report = Counter()
    lanes = {}
    for c in chains:
        lanes[c] = lanes_of(osm, c, report)

    # speeds per (chain, direction)
    signs = [f for f in feats if re.match(r"regulatory--maximum-speed-limit-(\d+)--", f["object_value"])]

    def speed(c, d):
        t = c.t
        v = speed_kmh(t.get("maxspeed:forward" if d > 0 else "maxspeed:backward") or t.get("maxspeed"))
        if v:
            return v, "osm"
        if c.way["id"] in ov:
            return ov[c.way["id"]], "overture"
        line, left, _ = approach_frame(c, d)
        n = c.nf if d > 0 else c.nb
        best = None
        for f in signs:
            sl = lateral(line, f["xy"])
            if sl is None:
                continue
            s, off = sl
            if left - n * c.w - 10 < off < left and faces(f, line, s):
                v = float(re.match(r"regulatory--maximum-speed-limit-(\d+)--", f["object_value"]).group(1))
                best = v if best is None else min(best, v)
        if best:
            return best, "mapillary"
        return SPEED_RULE.get(c.cls, 50.0), "rule"

    lanelet_of = {}
    for c, by_dir in lanes.items():
        for d, lst in by_dir.items():
            v, src = speed(c, d)
            report[f"speed:{src}"] += len(lst)
            for ln in lst:
                tags = {"type": "lanelet", "subtype": "road", "location": "urban", "one_way": "yes",
                        "speed_limit": f"{v:g}", "swb:source": "osm", "swb:speed_source": src,
                        "swb:width_source": c.width_source, "swb:osm_way": c.way["id"]}
                if c.name:
                    tags["name"] = c.name
                ln.speed = v
                ln.lid = osm.rel([("way", ln.left[0], "left"), ("way", ln.right[0], "right")], tags)
                lanelet_of[ln.lid] = (ln, tags)

    # junction approaches and exits
    restrictions = []
    for r in rels:
        t = r.get("tags", {})
        if t.get("type") == "restriction" and t.get("restriction"):
            m = {x["role"]: x for x in r.get("members", [])}
            if "from" in m and "to" in m and "via" in m and m["via"]["type"] == "node":
                restrictions.append((m["from"]["ref"], m["via"]["ref"], m["to"]["ref"], t["restriction"]))
    arrows = [f for f in feats if f["object_value"] in ARROWS]
    noturn = [f for f in feats if f["object_value"] in NO_TURN]
    conn_n = 0
    stats = Counter()
    for j in J:
        app, ext = [], []   # (chain, direction, lanes, end point, heading)
        for c in chains:
            for d, lst in lanes[c].items():
                coords = list(c.line.coords) if d > 0 else list(c.line.coords)[::-1]
                into = (c.jb is j) if d > 0 else (c.ja is j)
                outof = (c.ja is j) if d > 0 else (c.jb is j)
                if into:
                    app.append((c, d, lst, coords[-1], heading(coords[-2], coords[-1])))
                if outof:
                    ext.append((c, d, lst, coords[0], heading(coords[0], coords[1])))
        roads_here = len({id(a[0]) for a in app + ext})
        is_junction = roads_here >= 3 and not j["joint"]
        if is_junction:
            stats["junctions"] += 1
            if j["signal"]:
                stats[f"signalised:{j['signal']}"] += 1
        for (c, d, lst, p_in, h_in) in app:
            # movements available from this approach
            moves = defaultdict(list)   # class -> [exit]
            for e in ext:
                if e[0] is c and e[1] == -d:
                    continue        # back the way it came
                dh = math.degrees(wrap(e[4] - h_in))
                if abs(dh) > UTURN_DEG:
                    continue
                k = "straight" if abs(dh) < TURN_DEG else ("left" if dh > 0 else "right")
                moves[k].append(e)
            if not moves:
                continue
            n = len(lst)
            allowed = [None] * n
            src = [None] * n
            tl = c.t.get("turn:lanes" if c.oneway else ("turn:lanes:forward" if d > 0 else "turn:lanes:backward"))
            if tl and len(tl.split("|")) == n:
                for i, part in enumerate(tl.split("|")):
                    s = {TURN_TOKENS[x] for x in part.split(";") if x in TURN_TOKENS}
                    if s:
                        allowed[i], src[i] = s, "osm"
            if any(a is None for a in allowed):
                line, left, L_end = approach_frame(c, d)
                votes = defaultdict(Counter)
                for f in arrows:
                    sl = lateral(line, f["xy"])
                    if sl is None:
                        continue
                    s, off = sl
                    if L_end - 60 < s <= L_end + 3:
                        k = int((left - off) // c.w)
                        if 0 <= k < n:
                            for m in ARROWS[f["object_value"]]:
                                votes[k][m] += 1
                for k in range(n):
                    if allowed[k] is None and votes[k]:
                        top = max(votes[k].values())
                        allowed[k] = {m for m, v in votes[k].items() if v >= top / 2}
                        src[k] = "mapillary"
            # the rule for what is left; keep left turns left of any lane that turns left, etc.
            for k in range(n):
                if allowed[k] is None:
                    s = {"straight"}
                    if k == 0:
                        s.add("left")
                    if k == n - 1:
                        s.add("right")
                    if n == 1:
                        s = {"left", "straight", "right"}
                    allowed[k], src[k] = s, "rule"
            # restrictions
            banned = set()
            only = None
            for frm, via, to, kind in restrictions:
                if frm == c.way["id"] and via in j["nodes"]:
                    for k, es in moves.items():
                        for e in es:
                            if e[0].way["id"] == to:
                                if kind.startswith("no_"):
                                    banned.add((k, id(e[0]), e[1]))
                                elif kind.startswith("only_"):
                                    only = (k, id(e[0]), e[1])
            line, left, L_end = approach_frame(c, d)
            signed = set()
            for f in noturn:
                sl = lateral(line, f["xy"])
                if sl is None:
                    continue
                s, off = sl
                if (L_end - 50 < s <= L_end + 5 and left - n * c.w - 10 < off < left
                        and faces(f, line, s)):
                    signed.add(NO_TURN[f["object_value"]])
                    BAN_LOG.append({"road": c.name, "way": c.way["id"], "dir": d, "kind": NO_TURN[f["object_value"]],
                                    "sign": f["id"], "sign_xy": [round(v, 1) for v in f["xy"]],
                                    "before_stop_m": round(L_end - s, 1), "offset_m": round(off - left, 1),
                                    "lanes": n, "junction": j["id"], "junction_xy": [round(v) for v in j["c"]],
                                    "exits": {k: [e[0].name for e in es] for k, es in moves.items()}})
            for kind in signed:
                if kind in moves:
                    for e in moves[kind]:
                        banned.add((kind, id(e[0]), e[1]))
                    stats[f"approach_no_{kind}_by_mapillary_sign"] += 1
            open_moves = {k for k, es in moves.items()
                          if any((k, id(e[0]), e[1]) not in banned and
                                 (only is None or (k, id(e[0]), e[1]) == only) for e in es)}
            if not open_moves:
                continue
            # every lane needs somewhere to go: a lane whose movements are all missing or banned
            # keeps the nearest open one
            for k in range(n):
                allowed[k] &= open_moves
                if not allowed[k]:
                    allowed[k] = {"straight"} if "straight" in open_moves else {sorted(open_moves)[0]}
                    src[k] = (src[k] or "rule") + "+fallback"
            # a missing arrow is no evidence of a ban: a movement the junction offers and nothing
            # bans goes to the lane that would take it (left: the leftmost, right: the rightmost,
            # straight: every lane that has no other movement)
            for kind in open_moves - set().union(*allowed):
                if kind == "left":
                    targets = [0]
                elif kind == "right":
                    targets = [n - 1]
                else:
                    targets = [k for k in range(n) if not allowed[k] - {"straight"}] or list(range(n))
                for k in targets:
                    allowed[k] = allowed[k] | {kind}
                    if not (src[k] or "").endswith("+gap"):
                        src[k] = (src[k] or "rule") + "+gap"
                stats[f"movements_added:{kind}"] += 1
            for k in range(n):
                stats[f"lane_turns:{src[k]}"] += 1
            if DEBUG_AT and math.dist(j["c"], DEBUG_AT) < 40:
                print(f"J{j['id']} approach {c.name or c.way['id']} ({c.way['id']}, dir {d}, {n} lanes): "
                      f"exits {{{', '.join(f'{k}: ' + '/'.join(e[0].name or str(e[0].way['id']) for e in es) for k, es in moves.items())}}} "
                      f"banned {sorted(b[0] for b in banned)} lanes {[sorted(a) for a in allowed]} from {src}")
            for kind, es in moves.items():
                ins = [k for k in range(n) if kind in allowed[k]]
                if not ins:
                    continue
                for e in es:
                    if (kind, id(e[0]), e[1]) in banned or (only and (kind, id(e[0]), e[1]) != only):
                        continue
                    outs = e[2]
                    m = len(outs)
                    pairs = []
                    if kind == "left":
                        for q, k in enumerate(ins):
                            pairs.append((k, min(q, m - 1)))
                    else:   # right and straight align from the right edge
                        for q, k in enumerate(reversed(ins)):
                            pairs.append((k, max(0, m - 1 - q)))
                    for k, o in pairs:
                        a, b = lst[k], outs[o]
                        conn_n += 1
                        connector(osm, a, b, kind if is_junction else None,
                                  src[k], j, lanelet_of)
                        stats[f"paths:{kind}"] += 1
            if j["signal"] and is_junction:
                signal(osm, j, c, d, lst, p_in, h_in, lanelet_of)
    crosswalks(osm, nodes, rels, site, frame, chains, stats, out)
    write_out(osm, site, out, report, stats)


def connector(osm, a, b, turn, src, j, lanelet_of):
    la, ra = a.left[1], a.right[1]
    lb, rb = b.left[1], b.right[1]
    pts = {}

    def bound(n0, n1, prev0, next1):
        p0, p1 = osm_xy(osm, n0), osm_xy(osm, n1)
        h0 = heading(osm_xy(osm, prev0), p0)
        h1 = heading(p1, osm_xy(osm, next1))
        k = max(3, int(math.dist(p0, p1) / 2.0))
        mid = cubic(p0, h0, p1, h1, k)[1:-1]
        return [n0] + [osm.node(p) for p in mid] + [n1]
    left = bound(la[-1], lb[0], la[-2], lb[1])
    right = bound(ra[-1], rb[0], ra[-2], rb[1])
    wl = osm.way(left, {"type": "virtual"})
    wr = osm.way(right, {"type": "virtual"})
    v = min(a.speed, b.speed)
    tags = {"type": "lanelet", "subtype": "road", "location": "urban", "one_way": "yes",
            "speed_limit": f"{v:g}", "swb:source": "osm", "swb:speed_source": "connector"}
    if turn:
        tags["turn_direction"] = turn
        tags["swb:turn_source"] = src
    lid = osm.rel([("way", wl, "left"), ("way", wr, "right")], tags)
    lanelet_of[lid] = (None, tags)


_xy_cache = {}


def osm_xy(osm, nid):
    if not _xy_cache or len(_xy_cache) < len(osm.nodes):
        for i, p, _ in osm.nodes[len(_xy_cache):]:
            _xy_cache[i] = p
    return _xy_cache[nid]


def signal(osm, j, c, d, lst, p_in, h_in, lanelet_of):
    left_end = osm_xy(osm, lst[0].left[1][-1])
    right_end = osm_xy(osm, lst[-1].right[1][-1])
    stop = osm.way([osm.node(left_end), osm.node(right_end)], {"type": "stop_line"})
    # the light over the far side of the junction, facing the approach
    ahead = 2 * j["R"] + 2 * j["spread"]
    pl = (left_end[0] + ahead * math.cos(h_in), left_end[1] + ahead * math.sin(h_in))
    pr = (right_end[0] + ahead * math.cos(h_in), right_end[1] + ahead * math.sin(h_in))
    light = osm.way([osm.node(pl, 5.0), osm.node(pr, 5.0)],
                    {"type": "traffic_light", "subtype": "red_yellow_green", "height": "1.0"})
    reg = osm.rel([("way", light, "refers"), ("way", stop, "ref_line")],
                  {"type": "regulatory_element", "subtype": "traffic_light",
                   "swb:source": j["signal"]})
    for ln in lst:
        for i, (lid, members, tags) in enumerate(osm.rels):
            if lid == ln.lid:
                members.append(("relation", reg, "regulatory_element"))
                break


def crosswalks(osm, nodes, rels, site, frame, chains, stats, out):
    raw = out / "raw"
    d = json.loads((raw / "osm.json").read_text())
    s, w, n, e = site["bbox"]
    roads = [c.line for c in chains]
    for el in d["elements"]:
        if el["type"] != "way":
            continue
        t = el.get("tags", {})
        if t.get("footway") != "crossing" and t.get("highway") != "crossing":
            continue
        pts = [nodes[i] for i in el["nodes"] if i in nodes]
        if len(pts) < 2 or not all(s <= p["lat"] <= n and w <= p["lon"] <= e for p in pts):
            continue
        line = LineString([frame.xy(p["lat"], p["lon"]) for p in pts])
        if not any(line.intersects(r.buffer(3.0)) for r in roads):
            continue
        left = [osm.node(p) for p in offset(line, 1.5).coords]
        right = [osm.node(p) for p in offset(line, -1.5).coords]
        wl = osm.way(left, {"type": "line_thin", "subtype": "solid"})
        wr = osm.way(right, {"type": "line_thin", "subtype": "solid"})
        osm.rel([("way", wl, "left"), ("way", wr, "right")],
                {"type": "lanelet", "subtype": "crosswalk", "location": "urban", "one_way": "no",
                 "participant:pedestrian": "yes", "speed_limit": "10", "swb:source": "osm",
                 "swb:osm_way": el["id"]})
        stats["crosswalks"] += 1


def write_out(osm, site, out, report, stats):
    g = out / "generated"
    g.mkdir(parents=True, exist_ok=True)
    osm.write(g / "lanelet2_map.osm")
    f = site["frame"]
    (g / "map_projector_info.yaml").write_text(
        f"projector_type: MGRS\nvertical_datum: WGS84\nmgrs_grid: {f['mgrs_grid']}\n")
    rep = {"nodes": len(osm.nodes), "ways": len(osm.ways), "relations": len(osm.rels)}
    (g / "bans.json").write_text(json.dumps(BAN_LOG, indent=1, ensure_ascii=False))
    rep.update(report)
    rep.update(stats)
    rep.update(STATS)
    (g / "report.json").write_text(json.dumps(rep, indent=1, sort_keys=True))
    for k in sorted(rep):
        print(f"{k:34s} {rep[k]}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site", type=Path)
    ap.add_argument("out", type=Path)
    a = ap.parse_args()
    site = yaml.safe_load(a.site.read_text())
    generate(site, a.out)


if __name__ == "__main__":
    main()
