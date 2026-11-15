"""Small planar geometry, standard library only (no shapely on the car's image)."""

from __future__ import annotations

import math


_CUM = {}


def _cum(pts):
    c = _CUM.get(id(pts))
    if c is None or c[0] is not pts:
        acc, out = 0.0, [0.0]
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            acc += math.hypot(x1 - x0, y1 - y0)
            out.append(acc)
        c = _CUM[id(pts)] = (pts, out)
    return c[1]


def polyline_project(pts, x, y):
    """(arc length of the nearest point on the polyline, signed lateral offset (left +), heading there).
    The nearest vertex first, then the two segments beside it (exact for polylines sampled finer than
    their curvature, as czp-zones' are)."""
    n = len(pts)
    if n < 2:
        return 0.0, 0.0, 0.0
    cum = _cum(pts)
    bi, bd = 0, math.inf
    for i in range(n):
        px, py = pts[i]
        d = (px - x) * (px - x) + (py - y) * (py - y)
        if d < bd:
            bi, bd = i, d
    best = (math.inf, 0.0, 0.0, 0.0)
    for j in (bi - 1, bi):
        if j < 0 or j + 1 >= n:
            continue
        (x0, y0), (x1, y1) = pts[j], pts[j + 1]
        dx, dy = x1 - x0, y1 - y0
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-18:
            continue
        u = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg2))
        px, py = x0 + u * dx, y0 + u * dy
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            seg = math.sqrt(seg2)
            best = (d, cum[j] + u * seg, (dx * (y - y0) - dy * (x - x0)) / seg, math.atan2(dy, dx))
    return best[1], best[2], best[3]


def polyline_length(pts):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))


def point_at(pts, s):
    """The point and heading at arc length s along the polyline (clamped)."""
    acc = 0.0
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if acc + seg >= s or (x1, y1) == tuple(pts[-1]):
            u = 0.0 if seg < 1e-9 else max(0.0, min(1.0, (s - acc) / seg))
            return (x0 + u * (x1 - x0), y0 + u * (y1 - y0)), math.atan2(y1 - y0, x1 - x0)
        acc += seg
    return tuple(pts[-1]), 0.0


def offset_polyline(pts, d):
    """The polyline shifted d to the left (per-vertex normals)."""
    out = []
    n = len(pts)
    for i in range(n):
        a = pts[max(0, i - 1)]
        b = pts[min(n - 1, i + 1)]
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        out.append((pts[i][0] - d * math.sin(h), pts[i][1] + d * math.cos(h)))
    return out


def ribbon_quads(pts, half_w):
    """The swept area of a box of half-width half_w along the polyline, as convex quads."""
    left, right = offset_polyline(pts, half_w), offset_polyline(pts, -half_w)
    return [(right[i], right[i + 1], left[i + 1], left[i]) for i in range(len(pts) - 1)]


def area(poly):
    a = 0.0
    for (x0, y0), (x1, y1) in zip(poly, list(poly[1:]) + [poly[0]]):
        a += x0 * y1 - x1 * y0
    return a / 2.0


def ccw(poly):
    return list(poly) if area(poly) >= 0 else list(reversed(poly))


def clip_convex(subject, clip):
    """Sutherland-Hodgman: subject polygon clipped by a convex clip polygon (both lists of points)."""
    clip = ccw(clip)
    out = list(subject)
    for (cx0, cy0), (cx1, cy1) in zip(clip, clip[1:] + clip[:1]):
        inp, out = out, []
        if not inp:
            break
        inside = lambda p: (cx1 - cx0) * (p[1] - cy0) - (cy1 - cy0) * (p[0] - cx0) >= -1e-12  # noqa: E731
        s = inp[-1]
        for e in inp:
            if inside(e):
                if not inside(s):
                    out.append(_isect(s, e, (cx0, cy0), (cx1, cy1)))
                out.append(e)
            elif inside(s):
                out.append(_isect(s, e, (cx0, cy0), (cx1, cy1)))
            s = e
    return out


def _isect(p, q, a, b):
    x1, y1, x2, y2 = p[0], p[1], q[0], q[1]
    x3, y3, x4, y4 = a[0], a[1], b[0], b[1]
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(den) < 1e-15:
        return q
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))


def convex_hull(points):
    pts = sorted(set((round(p[0], 6), round(p[1], 6)) for p in points))
    if len(pts) <= 2:
        return pts
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])  # noqa: E731
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < (x1 - x0) * (y - y0) / (y1 - y0 + 1e-300) + x0:
            inside = not inside
    return inside


def segment_hits_poly(p, q, poly):
    """Whether segment pq meets polygon poly (an end inside, or an edge crossing)."""
    if point_in_poly(p[0], p[1], poly) or point_in_poly(q[0], q[1], poly):
        return True
    n = len(poly)
    for i in range(n):
        if _segs_cross(p, q, poly[i], poly[(i + 1) % n]):
            return True
    return False


def _segs_cross(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < 0) and (o3 * o4 < 0)
