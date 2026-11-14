"""Roadmap 074: per-run safety, efficiency and comfort measures for a pair of cars.

Fed one truth sample at a time by the arena (arena.py, every 0.05 s tick) and by swb's scoring
(c2b.py, the agents' 0.1 s logs). Every measure is defined over time, not over samples, so the two
agree at different rates, and none depends on how long the run lasts (074 D2's lesson).

Safety (while the two cars share a lane, the follower closing on the leader)
  drac       the deceleration rate to avoid a crash, closing^2 / (2 gap): the largest, per follower
  margin     1 - drac / b_max: the smallest; below 0 the follower could not have stopped in time
  tet, tit   time-exposed and time-integrated TTC: seconds with TTC below ttc_ref, and the
             integral of (ttc_ref - TTC) over them (s^2)
  impact     the closing speed at the first contact (m/s), 0 without one
Efficiency
  stops      stops per car: speed below STOP_V for at least STOP_MIN s
  stopped    seconds per car spent in such stops
Comfort
  jerk       the largest |change of acceleration| per car, acceleration and its change both as
             slopes over WINDOW s
General risk (any path relation; docs/design/cooperative-driving.md ①), from both cars' poses
  ttc2d      the smallest time until the two bodies would touch at constant velocity, up to TAU_REF
  thw        the smallest time headway while one car follows the other (same heading, overlapping
             corridor): the gap over the follower's speed
  pet        post-encroachment time where the paths cross (headings 30-150 degrees apart): the
             smallest time between one car leaving a spot and the other reaching it
  clear      the smallest body clearance (m), 0 at contact
  v_touch    the relative speed (vector difference, m/s) when the footprints first touch
  pen        the deepest overlap of the two footprints (m), 0 without contact: a footprint is the
             spec's box (Autoware's vehicle_info), square-cornered, so a shallow overlap at a
             steep yaw need not be a physical touch
  rho        the risk term: the largest shortfall of the four against TAU_REF, H_REF, PET_REF, D_REF
  s          arc length each car covered (m) since t = 0, the run's start; s_h the same at
             t = HORIZON, the arc length route progress is scored on (lost_progress; 074 N3)
  lat_peak   the largest lateral acceleration per car (the velocity vector's slope over WINDOW,
             across the car's heading)
Latency (event times, s from the run's start; None when the event did not happen)
  t_lat      A's lateral move begins (truth: lateral > LAT_START of a lane)
  t_brake    each car's first braking (acceleration over WINDOW below -BRAKE_START)
  react_b    B's reaction to A's move: t_brake[B] - t_lat, when B brakes after A moves
  react_a    A's reaction to B's braking: t_decide - t_brake[B], when A decides after B brakes
"""

import math
from collections import deque

WINDOW = 0.5  # s
STOP_V = 0.1  # m/s
STOP_MIN = 1.0  # s
LAT_START = 0.05  # share of a lane
BRAKE_START = 0.5  # m/s^2
TTC_REF = 3.0  # s, as cutin_game.Preference.ttc_ref
TAU_REF, H_REF, PET_REF, D_REF = 3.0, 1.0, 1.5, 1.0  # s, s, s, m: the risk references (①)
A_LAT_MAX = 4.0  # m/s^2, the lateral acceleration comfort is judged against (about 0.4 g)
W_COMFORT = 0.3
W_CONTACT = 10.0
PET_SAMPLE = 0.1  # s between stored footprints for PET
HORIZON = 20.0  # s: route progress is scored over [0, HORIZON] of the run (design ①, H; 074 N3)


def lost_progress(v0, s_h, horizon=HORIZON):
    """Lost route progress over the fixed window: (v0 H - s(H)) / (v0 H), clipped to [0, 1].

    One window for every study (074 N3): a run may go on past H (until a merge is complete, or
    for a fixed time), and what happens after H counts in risk, comfort and contact, never in
    progress, so payoffs scored on runs of different lengths compare. s_h None (a run shorter
    than H, which only a contact ends, and a contact scores lost = 1 anyway) counts as no progress."""
    if s_h is None:
        return 1.0
    return min(1.0, max(0.0, (v0 * horizon - s_h) / max(v0 * horizon, 1.0)))


# --- geometry: a car as an oriented box -----------------------------------------------------------
def corners(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2, width / 2
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def _sep(p, q):
    """True if a side of p separates it from q (both convex, counter-clockwise or clockwise)."""
    n = len(p)
    for i in range(n):
        (x1, y1), (x2, y2) = p[i], p[(i + 1) % n]
        nx, ny = y2 - y1, x1 - x2
        pp = [nx * (u - x1) + ny * (v - y1) for u, v in p]
        qq = [nx * (u - x1) + ny * (v - y1) for u, v in q]
        if (max(pp) <= min(qq)) or (max(qq) <= min(pp)):
            return True
    return False


def overlap(p, q):
    return not (_sep(p, q) or _sep(q, p))


def _pt_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - u * dx, py - ay - u * dy)


def clearance(p, q):
    """The smallest distance between two convex polygons, 0 when they overlap."""
    if overlap(p, q):
        return 0.0
    d = math.inf
    for a, b in ((p, q), (q, p)):
        for px, py in a:
            for i in range(len(b)):
                (ax, ay), (bx, by) = b[i], b[(i + 1) % len(b)]
                d = min(d, _pt_seg(px, py, ax, ay, bx, by))
    return d


def separation(p, q):
    """Signed separation of two convex polygons along their best separating axis: a lower bound
    of the distance when positive, minus the penetration depth when negative."""
    best = -math.inf
    for a, b in ((p, q), (q, p)):
        for i in range(len(a)):
            (x1, y1), (x2, y2) = a[i], a[(i + 1) % len(a)]
            nx, ny = y2 - y1, x1 - x2
            n = math.hypot(nx, ny)
            nx, ny = nx / n, ny / n
            pp = [nx * u + ny * v for u, v in a]
            qq = [nx * u + ny * v for u, v in b]
            best = max(best, min(qq) - max(pp), min(pp) - max(qq))
    return best


def ttc2d(pa, pb, d_now=None, horizon=TAU_REF, step=0.1):
    """The first time two cars' bodies touch at constant velocity, inf if not within horizon.

    pa, pb: (x, y, yaw, vx, vy, length, width).
    """
    rvx, rvy = pa[3] - pb[3], pa[4] - pb[4]
    rv = math.hypot(rvx, rvy)
    if d_now is None:
        d_now = clearance(corners(*pa[:3], *pa[5:]), corners(*pb[:3], *pb[5:]))
    if d_now <= 0.0:
        return 0.0
    if rv * horizon < d_now:
        return math.inf
    def touch(t):
        return overlap(corners(pa[0] + pa[3] * t, pa[1] + pa[4] * t, pa[2], *pa[5:]),
                       corners(pb[0] + pb[3] * t, pb[1] + pb[4] * t, pb[2], *pb[5:]))
    t_prev, t = 0.0, max(step, d_now / rv)  # the bodies cannot touch before d_now / rv
    while t <= horizon + 1e-9:
        if touch(t):
            lo, hi = t_prev, t
            for _ in range(5):
                mid = (lo + hi) / 2
                lo, hi = (lo, mid) if touch(mid) else (mid, hi)
            return hi
        t_prev, t = t, t + step
    return math.inf


def headway(pf, pl):
    """Time headway of follower pf behind leader pl, None unless pf follows pl."""
    c, s = math.cos(pf[2]), math.sin(pf[2])
    dx, dy = pl[0] - pf[0], pl[1] - pf[1]
    lon, lat = dx * c + dy * s, -dx * s + dy * c
    if lon <= 0 or abs(lat) >= (pf[6] + pl[6]) / 2 or math.cos(pl[2] - pf[2]) < math.cos(math.radians(45)):
        return None
    gap = lon - (pf[5] + pl[5]) / 2
    v = pf[3] * c + pf[4] * s
    if v <= 1.0 or gap <= 0:
        return None
    return gap / v


def rho(ttc=math.inf, thw=math.inf, pet=math.inf, clear=math.inf):
    """The risk term: the largest shortfall against its reference, 0..1 (①)."""
    sf = lambda x, ref: 0.0 if x is None or not math.isfinite(x) else max(0.0, ref - x) / ref
    return max(sf(ttc, TAU_REF), sf(thw, H_REF), sf(pet, PET_REF), sf(clear, D_REF))


def utility(lost, risk, peak_lon, b_max, peak_lat=0.0, contact=False, a_lat_max=A_LAT_MAX):
    """U_i = -l_i - rho_i - c_i - 10 [contact] (docs/design/cooperative-driving.md ①)."""
    c = W_COMFORT * (min(peak_lon, b_max) / b_max) ** 2 + W_COMFORT * (min(peak_lat, a_lat_max) / a_lat_max) ** 2
    return -lost - risk - c - (W_CONTACT if contact else 0.0)


class Measures:
    def __init__(self, b_max=(9.6, 9.6), ttc_ref=TTC_REF):
        self.b_max = b_max
        self.ttc_ref = ttc_ref
        self.t_prev = None
        self.vh = (deque(), deque())  # (t, v) over WINDOW
        self.ah = (deque(), deque())  # (t, a) over WINDOW
        self.drac = [0.0, 0.0]
        self.margin = [1.0, 1.0]  # no closing: the whole braking capability is spare
        self.tet = self.tit = 0.0
        self.impact = 0.0
        self.contact = False
        self.stops = [0, 0]
        self.stopped = [0.0, 0.0]
        self._slow_since = [None, None]
        self._counted = [False, False]
        self.jerk = [0.0, 0.0]
        self.t_lat = None
        self.t_brake = [None, None]
        # general risk (push_poses)
        self.ttc2d = self.thw = self.pet = self.clear = math.inf
        self.pen = 0.0  # m, the deepest overlap of the two footprints
        self.v_touch = None  # m/s, the relative speed when the footprints first touch
        self.s = [0.0, 0.0]
        self.s_h = [None, None]  # s at t = HORIZON (t as the caller counts it, 0 at the run's start)
        self.lat_peak = [0.0, 0.0]
        self._vv = (deque(), deque())  # (t, vx, vy) over WINDOW
        self._prev_pose = None
        self._fp = ([], [])  # (t, centre x, y, yaw, corners) every PET_SAMPLE s
        self._fp_t = -1.0

    def push_poses(self, t, pa, pb):
        """Both cars' poses at t: (x, y, yaw, vx, vy, length, width), in any one plane frame.

        Feeds the general risk (2D TTC, headway, PET, clearance), arc length and lateral
        acceleration; call it with push() at the same t, or alone.
        """
        dt = 0.0 if self._prev_pose is None else t - self._prev_pose[0]
        th = HORIZON  # the window is [0, H] in the caller's time, t = 0 the run's start
        for k, p in ((0, pa), (1, pb)):
            sp = math.hypot(p[3], p[4])
            if self._prev_pose is None and t > 0:
                self.s[k] = sp * min(t, th)  # a first sample after the start (the arena's first tick)
            if self._prev_pose is not None:
                q = self._prev_pose[1 + k]
                s_prev = self.s[k]
                self.s[k] += 0.5 * (sp + math.hypot(q[3], q[4])) * dt
                if self.s_h[k] is None and t >= th - 1e-9:
                    # interpolated to H exactly, so 0.05 s ticks and 0.1 s logs agree
                    f = 1.0 if dt <= 0 else max(0.0, min(1.0, (th - self._prev_pose[0]) / dt))
                    self.s_h[k] = s_prev + f * (self.s[k] - s_prev)
            h = self._vv[k]
            h.append((t, p[3], p[4]))
            while len(h) > 1 and t - h[1][0] >= WINDOW - 1e-6:
                h.popleft()
            if t - h[0][0] >= WINDOW - 0.051:
                span = t - h[0][0]
                ax, ay = (p[3] - h[0][1]) / span, (p[4] - h[0][2]) / span
                self.lat_peak[k] = max(self.lat_peak[k], abs(-ax * math.sin(p[2]) + ay * math.cos(p[2])))
        self._prev_pose = (t, pa, pb)
        ca, cb = corners(*pa[:3], *pa[5:]), corners(*pb[:3], *pb[5:])
        reach = math.hypot(pa[5], pa[6]) / 2 + math.hypot(pb[5], pb[6]) / 2
        if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) - reach < min(self.clear, D_REF) + 1e-9:
            d = clearance(ca, cb)
        else:
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1]) - reach  # a lower bound, above D_REF
        self.clear = min(self.clear, d)
        if d <= 0.0:
            self.pen = max(self.pen, -separation(ca, cb))
            if self.v_touch is None:
                self.v_touch = math.hypot(pa[3] - pb[3], pa[4] - pb[4])
        self.ttc2d = min(self.ttc2d, ttc2d(pa, pb, d_now=d))
        for pf, pl in ((pa, pb), (pb, pa)):
            h = headway(pf, pl)
            if h is not None:
                self.thw = min(self.thw, h)
        if t - self._fp_t >= PET_SAMPLE - 1e-6:
            self._fp_t = t
            self._fp[0].append((t, pa[0], pa[1], pa[2], ca, math.hypot(pa[5], pa[6]) / 2))
            self._fp[1].append((t, pb[0], pb[1], pb[2], cb, math.hypot(pb[5], pb[6]) / 2))

    def _pet(self):
        """Post-encroachment time over the stored footprints, where the paths cross."""
        best = math.inf
        fa, fb = self._fp
        # no two headings 30 degrees apart (a follow, a merge, an oppose): no crossing to time
        ha = {round(f[3], 2) for f in fa}
        hb = {round(f[3], 2) for f in fb}
        if all(abs(math.cos(x - y)) > math.cos(math.radians(30)) for x in ha for y in hb):
            return best
        for ta, xa, ya, ha, ca, ra in fa:
            for tb, xb, yb, hb, cb, rb in fb:
                if abs(ta - tb) >= best:
                    continue
                if math.hypot(xa - xb, ya - yb) > ra + rb:
                    continue
                if abs(math.cos(ha - hb)) > math.cos(math.radians(30)):  # a follow or an oppose
                    continue
                if overlap(ca, cb):
                    best = abs(ta - tb)
        return best

    def _slope(self, h, t, x):
        h.append((t, x))
        while len(h) > 1 and t - h[1][0] >= WINDOW - 1e-6:
            h.popleft()
        if t - h[0][0] >= WINDOW - 0.051:
            return (x - h[0][1]) / (t - h[0][0])
        return None

    def push(self, t, v, lat_a=None, follower=None, gap=None, closing=None, contact=False):
        """One truth sample at t (s from the run's start).

        v: (v_A, v_B); lat_a: A's lateral progress toward B's lane (share of a lane);
        follower: 0 or 1 when the cars share a lane (the rear one), with gap (bumper to bumper, m)
        and closing (the follower's speed minus the leader's, m/s); contact: the bodies overlap.
        """
        dt = 0.0 if self.t_prev is None else t - self.t_prev
        self.t_prev = t
        for k in (0, 1):
            a = self._slope(self.vh[k], t, v[k])
            if a is not None:
                if a <= -BRAKE_START and self.t_brake[k] is None:
                    self.t_brake[k] = t
                j = self._slope(self.ah[k], t, a)
                if j is not None:
                    self.jerk[k] = max(self.jerk[k], abs(j))
            if v[k] < STOP_V:
                if self._slow_since[k] is None:
                    self._slow_since[k] = t
                    self._counted[k] = False
                elif t - self._slow_since[k] >= STOP_MIN and not self._counted[k]:
                    self._counted[k] = True
                    self.stops[k] += 1
                    self.stopped[k] += t - self._slow_since[k]
                elif self._counted[k]:
                    self.stopped[k] += dt
            else:
                self._slow_since[k] = None
        if lat_a is not None and lat_a > LAT_START and self.t_lat is None:
            self.t_lat = t
        if contact and not self.contact:
            self.contact = True
            self.impact = abs(closing) if closing is not None else abs(v[0] - v[1])
        if follower is not None and gap is not None and closing is not None and closing > 1e-3:
            if gap > 0:
                d = closing ** 2 / (2 * gap)
                self.drac[follower] = max(self.drac[follower], d)
                self.margin[follower] = min(self.margin[follower], 1 - d / self.b_max[follower])
                ttc = gap / closing
                if ttc < self.ttc_ref:
                    self.tet += dt
                    self.tit += (self.ttc_ref - ttc) * dt

    def result(self, t_decide=None):
        r2 = lambda x: round(x, 3) if x is not None and math.isfinite(x) else None
        react_b = (self.t_brake[1] - self.t_lat if self.t_lat is not None and self.t_brake[1] is not None
                   and self.t_brake[1] >= self.t_lat else None)
        react_a = (t_decide - self.t_brake[1] if t_decide is not None and self.t_brake[1] is not None
                   and t_decide >= self.t_brake[1] else None)
        gen = {}
        if self._prev_pose is not None:
            self.pet = self._pet()
            r_all = rho(self.ttc2d, self.thw, self.pet, self.clear)
            gen = {"ttc2d": r2(self.ttc2d), "thw": r2(self.thw), "pet": r2(self.pet), "clear": r2(self.clear), "pen": r2(self.pen), "v_touch": r2(self.v_touch),
                   "rho": r2(r_all), "sA": r2(self.s[0]), "sB": r2(self.s[1]),
                   "sA_h": r2(self.s_h[0]), "sB_h": r2(self.s_h[1]),
                   "latA": r2(self.lat_peak[0]), "latB": r2(self.lat_peak[1])}
        return {**gen, "dracA": r2(self.drac[0]), "dracB": r2(self.drac[1]),
                "marginA": r2(self.margin[0]), "marginB": r2(self.margin[1]),
                "tet": r2(self.tet), "tit": r2(self.tit), "impact": r2(self.impact),
                "stopsA": self.stops[0], "stopsB": self.stops[1],
                "stoppedA": r2(self.stopped[0]), "stoppedB": r2(self.stopped[1]),
                "jerkA": r2(self.jerk[0]), "jerkB": r2(self.jerk[1]),
                "t_lat": r2(self.t_lat), "t_brakeA": r2(self.t_brake[0]), "t_brakeB": r2(self.t_brake[1]),
                "react_a": r2(react_a), "react_b": r2(react_b)}


def selftest():
    # a follower closing at 5 m/s from 12 m, braking at 4 m/s^2 after 1 s: the DRAC is largest
    # when braking starts (gap 7 m: 25/14), TTC below 3 s from the start until the closing falls
    m = Measures(b_max=(9.6, 9.6))
    gap, va, vb = 12.0, 10.0, 15.0
    t = 0.0
    while t < 6.0:
        closing = vb - va
        m.push(t, (va, vb), lat_a=0.0, follower=1, gap=gap, closing=closing)
        if t > 1.0:
            vb = max(va, vb - 4.0 * 0.05)
        gap -= closing * 0.05
        t += 0.05
    r = m.result()
    assert abs(r["dracB"] - 25 / 14) < 0.1, r
    assert abs(r["marginB"] - (1 - r["dracB"] / 9.6)) < 1e-3, r
    assert r["tet"] > 0 and r["t_brakeB"] is not None and 1.0 < r["t_brakeB"] < 1.8, r
    # the same run sampled at 0.1 s gives the same braking start and jerk within a sample
    m2 = Measures()
    for k in range(60):
        tt = k * 0.1
        m2.push(tt, (5.0, 5.0 - (2.0 * (tt - 2.0) if tt > 2.0 else 0.0)))
    m3 = Measures()
    for k in range(120):
        tt = k * 0.05
        m3.push(tt, (5.0, 5.0 - (2.0 * (tt - 2.0) if tt > 2.0 else 0.0)))
    a, b = m2.result(), m3.result()
    assert abs(a["t_brakeB"] - b["t_brakeB"]) <= 0.1 and abs(a["jerkB"] - b["jerkB"]) < 1.0, (a, b)
    # a stop of 2 s counts once, with 2 s stopped
    m4 = Measures()
    for k in range(80):
        tt = k * 0.05
        m4.push(tt, (0.0 if 1.0 <= tt < 3.0 else 3.0, 3.0))
    r4 = m4.result()
    assert r4["stopsA"] == 1 and abs(r4["stoppedA"] - 2.0) < 0.1, r4
    # geometry: two 4.8 x 2.2 boxes side by side in adjacent 3.5 m lanes are 1.3 m apart; one
    # 10 m behind the other at 5 m/s closing touches after (10 - 4.8) / 5 s
    W, L5 = 2.2, 4.8
    p = lambda x, y, yaw=0.0, vx=0.0, vy=0.0: (x, y, yaw, vx, vy, L5, W)
    assert abs(clearance(corners(*p(0, 0)[:3], L5, W), corners(*p(0, 3.5)[:3], L5, W)) - 1.3) < 1e-9
    assert abs(ttc2d(p(0, 0, vx=15.0), p(10, 0, vx=10.0)) - 5.2 / 5) < 0.01
    assert ttc2d(p(0, 0, vx=10.0), p(0, 3.5, vx=10.0)) == math.inf
    # an oppose closes at the sum of the speeds; a merge closes before the lanes are shared
    assert abs(ttc2d(p(0, 0, vx=10.0), p(30, 0, yaw=math.pi, vx=-10.0)) - 25.2 / 20) < 0.01
    assert ttc2d(p(0, 1.5, vx=10.0, vy=-1.0), p(-1, -1.0, vx=10.0)) < 1.0
    assert abs(headway(p(0, 0, vx=10.0), p(15, 0, vx=10.0)) - 1.02) < 1e-9
    assert headway(p(0, 0, vx=10.0), p(15, 3.5, vx=10.0)) is None
    # a crossing: B's centre crosses A's path at t = 0, A's at t = 2; B's rear clears A's path
    # (half-width 1.1 m) at 0.35 s and A's front reaches B's at 1.65 s: PET 1.3 s
    m5 = Measures()
    for k in range(100):
        tt = k * 0.05
        m5.push_poses(tt, p(-20 + 10 * tt, 0, vx=10.0), p(0, -10 + 10 * (tt + 1.0), yaw=math.pi / 2, vy=10.0))
    r5 = m5.result()
    assert r5["clear"] > 0 and 1.25 <= r5["pet"] <= 1.45, r5
    assert abs(r5["sA"] - 10 * 4.95) < 0.1, r5
    # progress over the fixed window: 10 m/s for 30 s, sampled at 0.05 or 0.1 s, slowing to 8 m/s
    # from 10 s at 1 m/s^2: s(H) = 100 + 18 + 8 * 8, the same at both rates, whatever follows H
    for step in (0.05, 0.1):
        m7, x, t = Measures(), 0.0, 0.0
        while t < 30.0 + 1e-9:
            v = 10.0 if t < 10.0 else max(8.0, 10.0 - (t - 10.0))
            m7.push_poses(t, p(x, 0, vx=v), p(x - 50, 0, vx=10.0))
            x += v * step
            t += step
        assert abs(m7.s_h[0] - (100 + 18 + 8 * 8)) < 0.15, (step, m7.s_h)
        assert abs(lost_progress(10.0, m7.s_h[0]) - 18 / 200) < 1e-3
    assert lost_progress(10.0, None) == 1.0
    # a cosine lane change of 3.5 m in 3 s: peak lateral acceleration 3.5 pi^2 / (2 * 9) = 1.92
    m6 = Measures()
    for k in range(120):
        tt = k * 0.05
        u = min(1.0, max(0.0, (tt - 1.0) / 3.0))
        vy = 3.5 * 0.5 * math.pi / 3.0 * math.sin(math.pi * u) if 0 < u < 1 else 0.0
        m6.push_poses(tt, p(10 * tt, 3.5 * 0.5 * (1 - math.cos(math.pi * u)), math.atan2(vy, 10.0), 10.0, vy),
                      p(10 * tt - 40, 0, vx=10.0))
    r6 = m6.result()
    assert 1.6 < r6["latA"] < 2.0 and r6["latB"] < 0.05, r6
    a0, b0 = corners(0, 0, 0, L5, W), corners(4.0, 0, 0, L5, W)
    assert abs(separation(a0, b0) + 0.8) < 1e-9 and abs(separation(a0, corners(6.0, 0, 0, L5, W)) - 1.2) < 1e-9
    assert rho(ttc=1.5) == 0.5 and rho(clear=0.2) == 0.8 and rho() == 0.0
    print("measures selftest ok")


if __name__ == "__main__":
    selftest()
