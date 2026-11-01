"""Lane widths and positions from Taipei's road pavement blocks (roadmap 062): 新工處's
臺北市道路銑鋪區塊圖資, curb to curb per direction, drawn on the city's 1/1000 topographic map
(fetch.py's `taipei_pavement.geojson`), less the curb parking spaces painted on the road
(停管處's 臺北市路邊停車格位, `taipei_parking.geojson`).

For a road piece (a chain), the road is cut across every 4 m, keeping 6 m clear of its ends,
by a line 60 m long. The cut's part on the pavement around the OSM line, less the parking
spaces, is the road its lanes may use:

- the blocks are those of the road's own name when it has blocks near (a junction's cross road
  would otherwise widen it), else every block;
- the piece of the cut that holds the OSM line, or the nearest within 3 m (OSM's line need not
  lie on the road);
- a dual carriageway mapped as two one-way ways whose halves share one block (a painted
  divide) is split halfway to the other half's line.

Each cut gives the road's left and right edge, as offsets from the OSM line; the chain takes
their medians over its cuts. The lanes then fill it: a lane is the width over the lane count
(2.6-4.0 m), and the lanes are centred on it, a two-way road's forward lanes to the right of
the backward ones. Taiwan's outer lanes are often wide (機慢車優先 space): over 4.0 m a lane,
the rest is left at the edges. A chain with fewer than two good cuts, or whose cuts disagree by
more than 2 m in width, keeps OSM's or the rule's width.
"""
import json
import math
from statistics import median

from shapely.geometry import LineString, Point, shape
from shapely.ops import unary_union
from shapely.strtree import STRtree

STEP, CLEAR, HALF = 4.0, 6.0, 30.0
W_MIN, W_MAX = 2.6, 4.0
SPREAD = 2.0


def load(raw, name, frame):
    p = raw / name
    if not p.exists():
        return []
    out = []
    for f in json.loads(p.read_text())["features"]:
        rings = [[frame.xy(lat, lon) for lon, lat in r] for r in f["geometry"]["coordinates"]]
        g = shape({"type": "Polygon", "coordinates": rings}).buffer(0)
        if not g.is_empty:
            out.append((g, f["properties"]))
    return out


def fit_all(chains, raw, frame, stats):
    pave = load(raw, "taipei_pavement.geojson", frame)
    if not pave:
        return
    park = load(raw, "taipei_parking.geojson", frame)
    pave_tree = STRtree([g for g, _ in pave])
    park_tree = STRtree([g for g, _ in park]) if park else None
    lines = [LineString(c.pts) for c in chains]
    line_tree = STRtree(lines)
    for ci, c in enumerate(chains):
        line = lines[ci]
        L = line.length
        if L < 8.0:
            continue
        stations = [L / 2] if L < 2 * CLEAR + STEP else [CLEAR + k * STEP for k in range(int((L - 2 * CLEAR) / STEP) + 1)]
        near = [pave[i] for i in pave_tree.query(line.buffer(HALF))]
        own = [g for g, p in near if c.name and p.get("ROAD") == c.name]
        road = unary_union(own or [g for g, _ in near])
        if road.is_empty:
            continue
        stats["pavement_chains_tried"] += 1
        parked = unary_union([park[i][0] for i in park_tree.query(line.buffer(HALF))]) if park_tree else None
        lefts, rights = [], []
        for s in stations:
            a = line.interpolate(max(0.0, s - 1.0))
            b = line.interpolate(min(L, s + 1.0))
            h = math.atan2(b.y - a.y, b.x - a.x)
            nx, ny = -math.sin(h), math.cos(h)
            p = line.interpolate(s)
            cut = LineString([(p.x - nx * HALF, p.y - ny * HALF), (p.x + nx * HALF, p.y + ny * HALF)])
            on = cut.intersection(road)
            if parked is not None and not parked.is_empty:
                on = on.difference(parked)
            pieces = [g for g in getattr(on, "geoms", [on]) if g.geom_type == "LineString" and not g.is_empty]
            if not pieces:
                continue
            piece = min(pieces, key=lambda g: g.distance(p))
            if piece.distance(p) > 3.0:
                continue
            offs = sorted((q[0] - p.x) * nx + (q[1] - p.y) * ny for q in piece.coords)
            lo, hi = offs[0], offs[-1]
            # the other half of a dual carriageway, inside this piece: the divide is halfway
            if c.oneway:
                for k in line_tree.query(cut):
                    o = chains[k]
                    if o is c or not o.oneway:
                        continue
                    x = cut.intersection(lines[k])
                    for q in getattr(x, "geoms", [x]):
                        if q.geom_type != "Point":
                            continue
                        off = (q.x - p.x) * nx + (q.y - p.y) * ny
                        if lo < off < hi and abs(off) > 1.0:
                            if off > 0:
                                hi = min(hi, off / 2)
                            else:
                                lo = max(lo, off / 2)
            if hi - lo > 1.5:
                lefts.append(hi)
                rights.append(lo)
        if len(lefts) < 2:
            continue
        widths = sorted(l - r for l, r in zip(lefts, rights))
        q1, q3 = widths[len(widths) // 4], widths[(3 * len(widths)) // 4]
        if q3 - q1 > SPREAD:
            stats["pavement_chains_disagreeing"] += 1
            continue
        left, right = median(lefts), median(rights)
        n = c.nf + c.nb
        w = min(W_MAX, max(W_MIN, (left - right) / n))
        mid = (left + right) / 2
        c.w = w
        c.d = mid if c.oneway else mid - (c.nb - c.nf) * w / 2
        c.width_source = "taipei"
        c.road_width = left - right
        stats["pavement_chains_fitted"] += 1
        if (left - right) / n > W_MAX:
            stats["pavement_lanes_capped_at_4m"] += 1
