"""The lanelet2 side of `simple_pilot`: load a map in the scenario's frame, route along it, and
turn the route into a path to follow (points, speed limits, curvature).

The map frame is the one Autoware and scenario_simulator_v2 compute (and the vehicle bridges'
ground truth is in): each node's lat/lon through the projector the map's
`map_projector_info.yaml` declares.

- `TransverseMercator`: WGS84, about the declared origin, `scale_factor` or 0.9996 (Autoware's
  map_loader, SSv2's lanelet_loader.cpp). The CARLA towns use it, origin (0, 0).
- `MGRS` (and no yaml, as SSv2 defaults): UTM in the zone of the map, less the 100 km grid
  square of its first node.
- `Local`: the nodes' `local_x` / `local_y` tags (Autoware reads them only then).

lanelet2's own Python projectors (UTM, local Cartesian) are not these, so the map is loaded
with any of them and every point is then moved to its position from the node's lat/lon.

Routing is lanelet2's: the vehicle's lanelet (the nearest one it may drive whose direction
matches its heading) to the goal's, through the waypoints' lanelets, shortest path, without
lane changes when there is such a path. The path is the lanelets' centre lines, resampled,
from the vehicle's projection to the goal's.

Only lanelet2 (python3 `lanelet2`, ROS 2 Humble's `ros-humble-lanelet2`) and the standard
library; no Autoware.
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path as FsPath

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
DEFAULT_K0 = 0.9996
_E2 = WGS84_F * (2 - WGS84_F)
_EP2 = _E2 / (1 - _E2)

# lanelets a car may be routed on (lanelet2's subtypes; Autoware's maps use road mostly)
DRIVABLE = ("road", "highway", "road_shoulder", "play_street", "emergency_lane", "bus_lane")
HEADING_MATCH = math.radians(60.0)  # a lanelet whose direction is further off is not "ours"
NEAR = 4.0  # m: candidate lanelets within this of the pose


class RouteError(Exception):
    """No route: the message says why (a goal off the map, an unreachable lane)."""


# --- the map frame ------------------------------------------------------------------------

def _meridian_arc(lat):
    e2, e4, e6 = _E2, _E2 ** 2, _E2 ** 3
    return WGS84_A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat
                      - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * lat)
                      + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * lat)
                      - (35 * e6 / 3072) * math.sin(6 * lat))


@dataclass(frozen=True)
class TransverseMercator:
    """Autoware's and SSv2's TransverseMercatorProjector (Snyder's series; within a millimetre
    of pyproj's tmerc over a town). The same as acb's carla_map_gen.projection (roadmap 087)."""

    lat0: float = 0.0
    lon0: float = 0.0
    k0: float = DEFAULT_K0

    def project(self, lat_deg, lon_deg):
        lat = math.radians(lat_deg)
        s, c = math.sin(lat), math.cos(lat)
        n = WGS84_A / math.sqrt(1 - _E2 * s * s)
        t = (s / c) ** 2
        cc = _EP2 * c * c
        a = math.radians(lon_deg - self.lon0) * c
        x = self.k0 * n * (a + (1 - t + cc) * a ** 3 / 6
                           + (5 - 18 * t + t * t + 72 * cc - 58 * _EP2) * a ** 5 / 120)
        y = self.k0 * (_meridian_arc(lat) - _meridian_arc(math.radians(self.lat0))
                       + n * (s / c) * (a * a / 2 + (5 - t + 9 * cc + 4 * cc * cc) * a ** 4 / 24
                                        + (61 - 58 * t + t * t + 600 * cc - 330 * _EP2)
                                        * a ** 6 / 720))
        return x, y


def read_projector_info(map_dir):
    """The map's `map_projector_info.yaml` as {projector_type, latitude, longitude,
    scale_factor}; None when there is none (SSv2 then takes MGRS)."""
    path = FsPath(map_dir) / "map_projector_info.yaml"
    if not path.is_file():
        return None
    import yaml
    doc = yaml.safe_load(path.read_text()) or {}
    origin = doc.get("map_origin") or {}
    return {"projector_type": str(doc.get("projector_type") or ""),
            "latitude": float(origin.get("latitude") or 0.0),
            "longitude": float(origin.get("longitude") or 0.0),
            "scale_factor": doc.get("scale_factor")}


def _osm_nodes(osm):
    """{node id: (lat, lon, local_x, local_y)}, the local tags None when absent."""
    nodes = {}
    for _, el in ET.iterparse(str(osm), events=("end",)):
        if el.tag != "node":
            continue
        tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
        lx, ly = tags.get("local_x"), tags.get("local_y")
        nodes[int(el.get("id"))] = (float(el.get("lat", 0.0)), float(el.get("lon", 0.0)),
                                    None if lx is None else float(lx),
                                    None if ly is None else float(ly))
        el.clear()
    return nodes


def map_frame_positions(osm, info):
    """{node id: (x, y)} in the map frame `info` (read_projector_info) declares."""
    nodes = _osm_nodes(osm)
    kind = (info or {}).get("projector_type") or "MGRS"
    if kind == "TransverseMercator":
        k0 = info.get("scale_factor")
        tm = TransverseMercator(info["latitude"], info["longitude"],
                                DEFAULT_K0 if k0 in (None, "") else float(k0))
        return {i: tm.project(lat, lon) for i, (lat, lon, _, _) in nodes.items()}
    if kind == "Local":
        missing = [i for i, n in nodes.items() if n[2] is None or n[3] is None]
        if missing:
            raise RouteError(f"{osm}: projector Local, but {len(missing)} node(s) have no "
                             "local_x/local_y")
        return {i: (n[2], n[3]) for i, n in nodes.items()}
    if kind == "MGRS":
        return _mgrs_positions(nodes)
    raise RouteError(f"{osm}: projector_type {kind!r}: TransverseMercator, MGRS or Local")


def _mgrs_positions(nodes):
    if not nodes:
        return {}
    lat0, lon0 = next(iter(nodes.values()))[:2]
    zone = int((lon0 + 180.0) // 6.0) + 1
    utm = TransverseMercator(0.0, zone * 6.0 - 183.0, 0.9996)
    north = 0.0 if lat0 >= 0 else 10000000.0
    x0, y0 = utm.project(lat0, lon0)
    x0 += 500000.0
    y0 += north
    gx, gy = math.floor(x0 / 1e5) * 1e5, math.floor(y0 / 1e5) * 1e5
    out = {}
    for i, (lat, lon, _, _) in nodes.items():
        x, y = utm.project(lat, lon)
        out[i] = (x + 500000.0 - gx, y + north - gy)
    return out


def load_map(map_path):
    """(LaneletMap, traffic rules, routing graph) of a map directory or its lanelet2_map.osm,
    points in the map frame."""
    import lanelet2
    from lanelet2.io import Origin
    from lanelet2.projection import UtmProjector

    p = FsPath(map_path)
    osm = p / "lanelet2_map.osm" if p.is_dir() else p
    if not osm.is_file():
        raise RouteError(f"no lanelet2 map at {osm}")
    info = read_projector_info(osm.parent)
    lmap, errors = lanelet2.io.loadRobust(str(osm), UtmProjector(Origin(0.0, 0.0)))
    positions = map_frame_positions(osm, info)
    for point in lmap.pointLayer:
        xy = positions.get(point.id)
        if xy is not None:
            point.x, point.y = xy
    return finish_map(lmap)


def finish_map(lmap):
    """(map, rules, graph) of a map whose points are already in the map frame."""
    import lanelet2
    rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(lmap, rules)
    return lmap, rules, graph


# --- geometry -----------------------------------------------------------------------------

def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _project_on(points, x, y):
    """(arc length of the nearest point on polyline `points`, lateral distance, heading)."""
    best = (float("inf"), 0.0, 0.0)
    s0 = 0.0
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (seg * seg)))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            best = (d, s0 + t * seg, math.atan2(dy, dx))
        s0 += seg
    d, s, h = best
    return s, d, h


def _centerline(ll):
    return [(p.x, p.y) for p in ll.centerline]


def _length(points):
    return sum(math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:]))


# --- routing ------------------------------------------------------------------------------

@dataclass
class Router:
    """Routes on one map. `Router.load(map_path)`, or `Router(lmap, rules, graph)`."""

    lmap: object
    rules: object
    graph: object
    default_speed: float = 30.0 / 3.6  # m/s, for a lanelet lanelet2 knows no limit of

    @classmethod
    def load(cls, map_path, **kw):
        return cls(*load_map(map_path), **kw)

    def drivable(self, ll):
        subtype = ll.attributes["subtype"] if "subtype" in ll.attributes else "road"
        return subtype in DRIVABLE and self.rules.canPass(ll)

    def lanelet_at(self, x, y, yaw, what="the vehicle"):
        """(lanelet, arc length on its centre line) for a pose: the drivable lanelet nearest
        to it whose direction is within HEADING_MATCH of `yaw`."""
        import lanelet2
        from lanelet2.core import BasicPoint2d
        found = []
        for dist, ll in lanelet2.geometry.findNearest(self.lmap.laneletLayer,
                                                       BasicPoint2d(x, y), 12):
            if dist > NEAR or not self.drivable(ll):
                continue
            pts = _centerline(ll)
            s, lateral, heading = _project_on(pts, x, y)
            off = abs(_wrap(heading - yaw))
            if off > HEADING_MATCH:
                continue
            found.append((dist, lateral + 2.0 * off, ll, s))
        if not found:
            raise RouteError(f"{what} at ({x:.1f}, {y:.1f}) heading {math.degrees(yaw):.0f} deg "
                             f"is on no lanelet a car may drive in that direction (within "
                             f"{NEAR:.0f} m)")
        found.sort(key=lambda f: (f[0], f[1]))
        _, _, ll, s = found[0]
        return ll, s

    def _shortest(self, a, b):
        path = self.graph.shortestPath(a, b, 0, False)
        if path is None:
            path = self.graph.shortestPath(a, b, 0, True)
        return None if path is None else list(path)

    def lanelets(self, start, start_s, goals):
        """The lanelet sequence from (start, start_s) through each (lanelet, s) of `goals`."""
        seq = [start]
        here, here_s = start, start_s
        for ll, s in goals:
            if ll.id == here.id and s >= here_s:
                here_s = s
                continue
            leg = None
            if ll.id == here.id:  # behind us on the same lanelet: around the block
                best = None
                for nxt in self.graph.following(here, True):
                    p = self._shortest(nxt, ll)
                    if p and (best is None or len(p) < len(best)):
                        best = p
                leg = best
            else:
                p = self._shortest(here, ll)
                leg = p[1:] if p else None
            if not leg:
                raise RouteError(f"no route from lanelet {here.id} to lanelet {ll.id}")
            seq += leg
            here, here_s = ll, s
        return seq

    def route(self, start_pose, points):
        """The `Path` from `start_pose` (x, y, yaw) through each of `points` ((x, y, yaw);
        the last is the goal)."""
        if not points:
            raise RouteError("no goal")
        start, s0 = self.lanelet_at(*start_pose)
        goals = [self.lanelet_at(*p, what="the goal" if i == len(points) - 1
                                 else f"waypoint {i + 1}") for i, p in enumerate(points)]
        seq = self.lanelets(start, s0, goals)
        return self.path(seq, start_pose, points[-1])

    def speed_limit(self, ll):
        """m/s: the lanelet's `speed_limit` tag (lanelet2 reads it, km/h unless a unit is
        given); `default_speed` for a lanelet without one (lanelet2 would take its urban 50
        km/h)."""
        if "speed_limit" not in ll.attributes:
            return self.default_speed
        try:
            v = float(self.rules.speedLimit(ll).speedLimit) / 3.6  # lanelet2 gives km/h
        except Exception:  # noqa: BLE001 -- a malformed tag: the default
            v = 0.0
        return v if v > 0.1 else self.default_speed

    def path(self, seq, start_pose, goal_pose):
        """The centre lines of `seq` joined, from the start's projection to the goal's. A lane
        change (a neighbour, not a successor) jumps from the middle of one to the middle of the
        next, which the controller rounds off."""
        pts, limits = [], []
        for i, ll in enumerate(seq):
            line = _centerline(ll)
            v = self.speed_limit(ll)
            change_in = i > 0 and not _follows(self.graph, seq[i - 1], ll)
            change_out = i + 1 < len(seq) and not _follows(self.graph, ll, seq[i + 1])
            if change_in or change_out:
                n = len(line)
                lo = n // 2 if change_in else 0
                hi = n // 2 + 1 if change_out else n
                line = line[lo:hi] or line[n // 2:n // 2 + 1]
            for p in line:
                if pts and math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) < 0.05:
                    continue
                pts.append(p)
                limits.append(v)
        path = Path.resample(pts, limits)
        first = _length(_centerline(seq[0]))  # the start is on the first lanelet
        s_start = path.locate(start_pose[0], start_pose[1], 0, path.index(first + 2.0) + 1)
        s_goal = path.project_last(goal_pose[0], goal_pose[1])
        if s_goal <= s_start:
            raise RouteError("the goal is behind the vehicle on its route")
        return path.trimmed(s_start, s_goal)


def _follows(graph, a, b):
    return any(f.id == b.id for f in graph.following(a, True))


# --- the path -----------------------------------------------------------------------------

@dataclass
class Path:
    """A polyline to follow: points every ~`STEP` m, their arc length, speed limit (m/s) and
    curvature (1/m, left positive). `end` is the arc length of the goal (the stop point)."""

    x: list
    y: list
    s: list
    limit: list
    curvature: list = field(default_factory=list)
    end: float = 0.0

    STEP = 1.0

    @classmethod
    def resample(cls, pts, limits, step=STEP):
        if len(pts) < 2:
            raise RouteError("the route has no length")
        xs, ys, ss, vs = [pts[0][0]], [pts[0][1]], [0.0], [limits[0]]
        s_total, carry = 0.0, 0.0
        for i in range(1, len(pts)):
            ax, ay = pts[i - 1]
            bx, by = pts[i]
            seg = math.hypot(bx - ax, by - ay)
            if seg < 1e-9:
                continue
            d = step - carry
            while d <= seg:
                t = d / seg
                xs.append(ax + t * (bx - ax))
                ys.append(ay + t * (by - ay))
                ss.append(s_total + d)
                vs.append(limits[i])
                d += step
            carry = seg - (d - step)
            s_total += seg
        if s_total - ss[-1] > 1e-3:
            xs.append(pts[-1][0])
            ys.append(pts[-1][1])
            ss.append(s_total)
            vs.append(limits[-1])
        path = cls(xs, ys, ss, vs, end=s_total)
        path.curvature = curvatures(xs, ys)
        return path

    @classmethod
    def from_points(cls, pts, limit):
        """A path straight along `pts` at one speed limit (tests, the fake world's direct
        route)."""
        return cls.resample(pts, [limit] * len(pts))

    def __len__(self):
        return len(self.s)

    def locate(self, x, y, lo, hi):
        """The arc length of (x, y)'s projection on segments lo..hi."""
        best, best_s = float("inf"), self.s[lo]
        for i in range(max(lo, 0), min(hi, len(self.s) - 1)):
            ax, ay, bx, by = self.x[i], self.y[i], self.x[i + 1], self.y[i + 1]
            dx, dy = bx - ax, by - ay
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if d < best:
                best, best_s = d, self.s[i] + t * (self.s[i + 1] - self.s[i])
        return best_s

    def project_last(self, x, y):
        """The goal's arc length: its projection on the path, searching from the end (a route
        that passes the goal's place twice stops at the second)."""
        n = len(self.s)
        best, best_i = float("inf"), n - 1
        for i in range(n - 1, -1, -1):
            d = math.hypot(self.x[i] - x, self.y[i] - y)
            if d < best - 1e-9:
                best, best_i = d, i
            elif d > best + 30.0:
                break
        return self.locate(x, y, max(best_i - 2, 0), min(best_i + 2, n - 1))

    def index(self, s):
        """The last point index at or before arc length s."""
        lo, hi = 0, len(self.s) - 1
        if s <= self.s[0]:
            return 0
        if s >= self.s[hi]:
            return hi
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self.s[mid] <= s:
                lo = mid
            else:
                hi = mid
        return lo

    def point(self, s):
        """(x, y) at arc length s; past the end, straight on along the last segment."""
        n = len(self.s)
        if s >= self.s[-1]:
            i = n - 2
            h = math.atan2(self.y[-1] - self.y[i], self.x[-1] - self.x[i])
            d = s - self.s[-1]
            return self.x[-1] + d * math.cos(h), self.y[-1] + d * math.sin(h)
        i = self.index(s)
        seg = self.s[i + 1] - self.s[i]
        t = 0.0 if seg < 1e-9 else (s - self.s[i]) / seg
        return (self.x[i] + t * (self.x[i + 1] - self.x[i]),
                self.y[i] + t * (self.y[i + 1] - self.y[i]))

    def heading(self, s):
        i = min(self.index(s), len(self.s) - 2)
        return math.atan2(self.y[i + 1] - self.y[i], self.x[i + 1] - self.x[i])

    def trimmed(self, s0, s1):
        """The part from arc length s0 to s1 (kept a little longer past s1 for the lookahead),
        re-based at s0, with `end` at s1."""
        keep = [i for i, s in enumerate(self.s) if s0 - 1e-6 <= s <= s1 + 15.0]
        x0, y0 = self.point(s0)
        xs, ys, ss, vs, ks = [x0], [y0], [0.0], [self.limit[self.index(s0)]], \
            [self.curvature[self.index(s0)]]
        for i in keep:
            if self.s[i] - s0 < 0.05:
                continue
            xs.append(self.x[i])
            ys.append(self.y[i])
            ss.append(self.s[i] - s0)
            vs.append(self.limit[i])
            ks.append(self.curvature[i])
        if len(xs) < 2:
            x1, y1 = self.point(s1)
            xs.append(x1)
            ys.append(y1)
            ss.append(s1 - s0)
            vs.append(vs[-1])
            ks.append(0.0)
        return Path(xs, ys, ss, vs, ks, end=s1 - s0)


def curvatures(xs, ys, window=3):
    """Signed curvature at each point from the circle through its neighbours `window` points
    away (smooths the centre lines' kinks)."""
    n = len(xs)
    out = [0.0] * n
    for i in range(n):
        a, b = max(i - window, 0), min(i + window, n - 1)
        if b - a < 2:
            continue
        ax, ay, bx, by, cx, cy = xs[a], ys[a], xs[i], ys[i], xs[b], ys[b]
        if i in (a, b):
            mid = (a + b) // 2
            bx, by = xs[mid], ys[mid]
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        d1 = math.hypot(bx - ax, by - ay)
        d2 = math.hypot(cx - bx, cy - by)
        d3 = math.hypot(cx - ax, cy - ay)
        if d1 * d2 * d3 < 1e-9:
            continue
        out[i] = 2.0 * cross / (d1 * d2 * d3)
    return out

