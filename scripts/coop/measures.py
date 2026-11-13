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
        return {"dracA": r2(self.drac[0]), "dracB": r2(self.drac[1]),
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
    print("measures selftest ok")


if __name__ == "__main__":
    selftest()
