#!/usr/bin/env python3
"""Roadmap 074 U2 swb: Town03's roundabout as roundabout_game's geometry.

Town03's ring has two lanes; this uses the outer one (CARLA lane -5: roads 38, 1470, 13, 1655, 14,
1786, 10, 1206, 11, 35, 1100, 36, 37, 1624) and the entry from the south (road 1691 lane 3, then
connector 1577 lane -3, which runs into the outer lane tangentially near -50 degrees). Both are
taken from CARLA's waypoints (0.5 m), so the cars follow the lanes the map has.

The model frame is roundabout_game's: the ring centred at the origin (the outer lane's
least-squares circle), counter-clockwise, A's approach from the south heading about +y. From
CARLA's frame it is a reflection (CARLA is left-handed) and a shift: x = -x_c - cx, y = y_c - cy,
yaw = pi - yaw_c. SSv2's (ROS) frame is (x_c, -y_c), heading -yaw_c.

install(geom) points roundabout_game's geometry (a_path, ring, R_RING, PHI_JOIN and the zone:
A_IN, A_JOINED, SZ0, SZ1, S_YIELD, S_CREEP) at these lanes, so its RBRules, simulate and score
run on Town03's roundabout unchanged. Ring positions stay roundabout_game's: R_RING x the angle
about the centre (the lane's radius varies by +-0.3 m around it, so this differs from its arc
length by about 1 %).

Usage (a CARLA with Town03 loaded):
  scripts/coop/roundabout_town03.py --port 2600 --out <geom.json>
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roundabout_game as G  # noqa: E402

A_START = (-6.55, -40.04)  # CARLA: the approach, road 1691 lane 3, 40 m south of the centre
A_BACK = 40.0  # m of approach behind it (staging and parking room)
A_ROADS = {1691, 25, 1577}  # A's lanes up to the ring; then the ring's
RING_START = (-23.88, -1.35)  # CARLA: the outer lane, road 38
STEP = 0.5
SPAN = 5.0  # route.lane_kappa's smoothing


# --- frames ---------------------------------------------------------------------------------------
class Frame:
    def __init__(self, centre):
        self.cx, self.cy = centre

    def to_m(self, xc, yc):
        return -xc - self.cx, yc - self.cy

    def to_c(self, x, y):
        return -(x + self.cx), y + self.cy

    @staticmethod
    def yaw_m(yaw_c):
        return math.pi - yaw_c

    @staticmethod
    def yaw_c(yaw_m):
        return math.pi - yaw_m


# --- from CARLA -----------------------------------------------------------------------------------
def _rad(w):
    loc = w.transform.location
    return math.hypot(loc.x, loc.y)


def _ring_branch(cands, r_ref=23.5, probe=14.0):
    """At a junction the ring's own branch: the one still on the ring `probe` m on."""
    def off(n):
        q = n.next(probe)
        return max(abs(_rad(z) - r_ref) for z in q) if q else 9.0
    return min(cands, key=off) if len(cands) > 1 else cands[0]


def build(cmap):
    import carla

    # the ring: one loop of the outer lane
    wp = cmap.get_waypoint(carla.Location(x=RING_START[0], y=RING_START[1]))
    ring_c, lanes = [], set()
    start = wp.transform.location
    for i in range(2000):
        loc = wp.transform.location
        ring_c.append((loc.x, loc.y, math.radians(wp.transform.rotation.yaw), loc.z))
        lanes.add((wp.road_id, wp.lane_id))
        if i > 100 and math.hypot(loc.x - start.x, loc.y - start.y) < STEP * 0.75:
            break
        wp = _ring_branch(wp.next(STEP))
    else:
        raise RuntimeError("the outer lane did not close into a loop")
    # its circle (algebraic least squares, CARLA frame)
    n = len(ring_c)
    sx = sum(p[0] for p in ring_c) / n
    sy = sum(p[1] for p in ring_c) / n
    suu = sum((p[0] - sx) ** 2 for p in ring_c)
    svv = sum((p[1] - sy) ** 2 for p in ring_c)
    suv = sum((p[0] - sx) * (p[1] - sy) for p in ring_c)
    suuu = sum((p[0] - sx) ** 3 for p in ring_c)
    svvv = sum((p[1] - sy) ** 3 for p in ring_c)
    suvv = sum((p[0] - sx) * (p[1] - sy) ** 2 for p in ring_c)
    svuu = sum((p[1] - sy) * (p[0] - sx) ** 2 for p in ring_c)
    b1, b2 = 0.5 * (suuu + suvv), 0.5 * (svvv + svuu)
    det = suu * svv - suv * suv
    uc, vc = (b1 * svv - b2 * suv) / det, (b2 * suu - b1 * suv) / det
    xc0, yc0 = sx + uc, sy + vc
    r_fit = sum(math.hypot(p[0] - xc0, p[1] - yc0) for p in ring_c) / n
    fr = Frame((-xc0, yc0))  # the centre in the reflected frame: x_m = -x_c - cx with cx = -xc0
    # A: back along the approach, then forward along its lanes into the ring, and on around it
    wp = cmap.get_waypoint(carla.Location(x=A_START[0], y=A_START[1]))
    back = []
    s = 0.0
    while s < A_BACK:
        p = wp.previous(STEP)
        if not p:
            break
        wp = min(p, key=lambda w: abs((w.transform.rotation.yaw - wp.transform.rotation.yaw + 180) % 360 - 180))
        back.append(wp)
        s += STEP
    a_wps = list(reversed(back))
    wp = cmap.get_waypoint(carla.Location(x=A_START[0], y=A_START[1]))
    a_roads = []
    for _ in range(2000):
        a_wps.append(wp)
        a_roads.append((wp.road_id, wp.lane_id))
        mine = [c for c in wp.next(STEP) if c.road_id in A_ROADS]
        if not mine:
            break
        wp = mine[0]
    a_c = [(w.transform.location.x, w.transform.location.y, math.radians(w.transform.rotation.yaw), w.transform.location.z)
           for w in a_wps]
    # on around the ring, along the ring's own points (the lane's next() leaves it at an exit)
    phi_c = lambda p: math.atan2(p[1] - yc0, -(p[0] - xc0))  # noqa: E731 -- the model frame's angle
    phi0 = phi_c(a_c[-1])
    loop = sorted(ring_c, key=lambda p: (phi_c(p) - phi0) % (2 * math.pi))
    loop = [p for p in loop if (phi_c(p) - phi0) % (2 * math.pi) > 0.3 / r_fit]
    for k in range(2):
        a_c += loop

    def to_model(pts):
        out = []
        for xc, yc, yawc, _ in pts:
            x, y = fr.to_m(xc, yc)
            out.append((round(x, 3), round(y, 3), round(math.atan2(math.sin(fr.yaw_m(yawc)), math.cos(fr.yaw_m(yawc))), 5)))
        return out

    z = sorted(p[3] for p in ring_c)[n // 2]
    roads = []
    for r in a_roads:
        if not roads or roads[-1] != r:
            roads.append(r)
    return {"centre_carla": [round(xc0, 3), round(yc0, 3)], "frame": [fr.cx, fr.cy], "r_ring": round(r_fit, 3),
            "z": round(z, 3), "ring": to_model(ring_c), "a": to_model(a_c), "a_lanes": roads[:12],
            "ring_lanes": sorted(lanes)}


# --- polylines ------------------------------------------------------------------------------------
def _kappa(pts, s):
    """Heading change over SPAN m about each point, as route.lane_kappa smooths it."""
    out = []
    for i in range(len(pts)):
        j0 = bisect.bisect_left(s, s[i] - SPAN / 2)
        j1 = min(len(pts) - 1, bisect.bisect_right(s, s[i] + SPAN / 2) - 1)
        ds = s[j1] - s[j0]
        dy = math.atan2(math.sin(pts[j1][2] - pts[j0][2]), math.cos(pts[j1][2] - pts[j0][2]))
        out.append(dy / ds if ds > 0.5 else 0.0)
    return out


class Poly:
    """A polyline by arc length: at(s) -> (x, y, yaw, kappa), linear between points."""

    def __init__(self, pts):
        self.p = pts
        self.s = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.k = _kappa(pts, self.s)
        self.len = self.s[-1]

    def at(self, s):
        s = max(0.0, min(self.len, s))
        i = max(0, min(len(self.p) - 2, bisect.bisect_right(self.s, s) - 1))
        f = (s - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-9)
        a, b = self.p[i], self.p[i + 1]
        dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * dyaw, self.k[i] + f * (self.k[i + 1] - self.k[i])

    def project(self, s_guess, x, y, back=2.0, ahead=4.0):
        """The arc length nearest (x, y) within [s_guess - back, s_guess + ahead]."""
        i0 = max(0, bisect.bisect_left(self.s, s_guess - back) - 1)
        i1 = min(len(self.p) - 1, bisect.bisect_right(self.s, s_guess + ahead))
        best, bs = math.inf, s_guess
        for i in range(i0, i1):
            a, b = self.p[i], self.p[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            ll = dx * dx + dy * dy
            f = 0.0 if ll < 1e-12 else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / ll))
            d = (a[0] + f * dx - x) ** 2 + (a[1] + f * dy - y) ** 2
            if d < best:
                best, bs = d, self.s[i] + f * (self.s[i + 1] - self.s[i])
        return bs


class RingPoly:
    """The ring by angle: at(phi) -> (x, y, yaw, kappa), linear in phi between points."""

    def __init__(self, pts):
        rows = []
        for x, y, yaw in pts:
            rows.append((math.atan2(y, x), x, y, yaw))
        rows.sort()
        self.rows = rows
        self.phi = [r[0] for r in rows]
        sl = Poly([(r[1], r[2], r[3]) for r in rows])
        self.k = sl.k

    def at(self, phi):
        phi = math.atan2(math.sin(phi), math.cos(phi))
        n = len(self.rows)
        i = bisect.bisect_right(self.phi, phi) - 1
        a = self.rows[i % n]
        b = self.rows[(i + 1) % n]
        pa, pb = a[0], b[0]
        if pb <= pa:
            pb += 2 * math.pi
        p = phi if phi >= pa else phi + 2 * math.pi
        f = (p - pa) / max(pb - pa, 1e-9)
        dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        ka, kb = self.k[i % n], self.k[(i + 1) % n]
        return a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw, ka + f * (kb - ka)


# --- into roundabout_game -------------------------------------------------------------------------
def install(geom, g=None):
    """Point roundabout_game's geometry at Town03's lanes (geom: build()'s dict); g: the module to point
    (roundabout_game run as a script is __main__, not the imported one). Returns (a, ring, frame)."""
    G = g or globals()["G"]  # noqa: N806
    a = Poly([tuple(p) for p in geom["a"]])
    rp = RingPoly([tuple(p) for p in geom["ring"]])
    G.R_RING = geom["r_ring"]

    def ring(s):
        return rp.at(s / G.R_RING)

    def a_path(s):
        return a.at(s)

    G.ring, G.a_path = ring, a_path
    # the join: where A's lane meets the ring's (its centre within 0.3 m of the ring lane, along it)
    a_in, a_joined, sz0, sz1 = G.zone_from_path(a_path, a.len - 1.0, geom["r_ring"])
    if a_in is None or a_joined is None:
        # the lane's radius varies around the circle: A has joined once it is on the ring lane itself
        raise RuntimeError("A's lane never joins the fitted ring")
    x, y, _, _ = a_path(a_joined)
    G.PHI_JOIN = math.atan2(y, x)
    G.A_IN, G.A_JOINED, G.SZ0, G.SZ1 = a_in, a_joined, sz0, sz1
    G.S_YIELD = a_in - 1.0
    G.S_CREEP = a_in - 0.3
    return a, rp, Frame(geom["frame"])


def load(path):
    with open(path) as f:
        return json.load(f)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args()
    import carla

    c = carla.Client("localhost", a.port)
    c.set_timeout(60.0)
    m = c.get_world().get_map()
    if not m.name.endswith("Town03"):
        sys.exit(f"this CARLA has {m.name} loaded, not Town03")
    geom = build(m)
    with open(a.out, "w") as f:
        json.dump(geom, f)
    install(geom)
    rs = [math.hypot(x, y) for x, y, _ in geom["ring"]]
    print(f"ring r {geom['r_ring']:.2f} (lane {min(rs):.2f}-{max(rs):.2f}), {len(geom['ring'])} points; A {len(geom['a'])} points, "
          f"lanes {geom['a_lanes']}")
    print(f"zone: A in band at {G.A_IN:.2f}, joined {G.A_JOINED:.2f}, join at {math.degrees(G.PHI_JOIN):.1f} deg, "
          f"stretch {G.SZ0:.2f}..{G.SZ1:.2f}, yield line {G.S_YIELD:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
