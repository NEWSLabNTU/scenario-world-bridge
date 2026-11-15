"""Step 5, the role: a map fact first (map_priority, then map_order), a perception fact with the tie
margin second; inside the margin the reading is ambiguous.

The perception fact is bias-corrected first. A hard threshold is unsafe for any perception error
(074 C8: a band of width 2e near the threshold where the two cars read differently); with the margin
m >= e (the error's centred 99 % quantile) on each side of every threshold, two readings within
+-e of the truth never pick opposite branches: a disagreement becomes ambiguous, and both wait.
"""

from __future__ import annotations

import math

from .model import (AHEAD_AT_END, AMBIG, FIRST, MAP_ORDER, MAP_PRIORITY, OBSTRUCTION_SIDE, PENDING, PRIORITY_RANK,
                    SECOND, STOPPED_FIRST, TIE, Agent, Role, Zone)


def right_of(zone: Zone, a_id, b_id):
    """True when approach b comes from a's right (right-hand traffic: b's heading is a's + 90 deg,
    within 45 deg)."""
    ha, hb = zone.approach(a_id).heading, zone.approach(b_id).heading
    d = math.atan2(math.sin(hb - ha), math.cos(hb - ha))
    return abs(d - math.pi / 2) < math.pi / 4


def map_priority(zone: Zone, own: Agent, other: Agent):
    ra = PRIORITY_RANK.get(zone.approach(own.approach).priority, 1)
    rb = PRIORITY_RANK.get(zone.approach(other.approach).priority, 1)
    reading = FIRST if ra < rb else SECOND if ra > rb else TIE
    return Role(MAP_PRIORITY, reading, {own.id: ra, other.id: rb})


def map_order(zone: Zone, own: Agent, other: Agent):
    oa, ob = zone.approach(own.approach).order, zone.approach(other.approach).order
    return Role(MAP_ORDER, FIRST if oa < ob else SECOND if oa > ob else TIE, {own.id: oa, other.id: ob})


def stopped_first(own_rest_t, other_rest_t, t, obs_age, other: Agent, params, far=False):
    """074 C8's reading of the rest stamps. d = (the other's stamp - bias) - own rest, bias the other's rest stamp's
    (``rest_stamp_bias``, reading - truth; the own rest is the own odometry's, not corrected):
    first if d > T + m, second if d < -(T + m), tie if |d| < T - m, else ambiguous. With no stamp of
    the other yet: first once a stamp within T + m would already be known (t - age - bias - own rest
    >= T + m) or when the other is far; pending otherwise."""
    T, m, bias = params.tie_threshold_T, params.tie_margin_m, params.rest_stamp_bias
    if own_rest_t is None:
        return Role(STOPPED_FIRST, PENDING, {})
    if other_rest_t is None:
        known = t - obs_age - bias - own_rest_t >= T + m
        return Role(STOPPED_FIRST, FIRST if (known or far) else PENDING, {"own": own_rest_t, "other": None})
    d = (other_rest_t - bias) - own_rest_t
    reading = FIRST if d > T + m else SECOND if d < -(T + m) else TIE if abs(d) < T - m else AMBIG
    conf = min(1.0, abs(abs(d) - T) / max(m, 1e-6))
    return Role(STOPPED_FIRST, reading, {"own": own_rest_t, "other": other_rest_t, "d": d}, conf)


def ahead_at_end(own_t_end, other_t_end, params):
    """Who is ahead at the end of a dropping lane, by time to the merge point, with the margin."""
    m = params.tie_margin_m + params.role_time_error_e
    d = other_t_end - own_t_end
    reading = FIRST if d > m else SECOND if d < -m else AMBIG
    return Role(AHEAD_AT_END, reading, {"own": own_t_end, "other": other_t_end, "d": d})


def obstruction_side(zone: Zone, own: Agent, other: Agent):
    oa, ob = zone.approach(own.approach).obstructed, zone.approach(other.approach).obstructed
    reading = SECOND if oa and not ob else FIRST if ob and not oa else TIE
    return Role(OBSTRUCTION_SIDE, reading, {own.id: oa, other.id: ob})


def role(fact, zone: Zone, own: Agent, other: Agent, params, **kw):
    """Step 5: the role fact named by the convention row, its reading for own against other."""
    if fact == MAP_PRIORITY:
        return map_priority(zone, own, other)
    if fact == MAP_ORDER:
        return map_order(zone, own, other)
    if fact == OBSTRUCTION_SIDE:
        return obstruction_side(zone, own, other)
    if fact == STOPPED_FIRST:
        return stopped_first(kw.get("own_rest_t"), kw.get("other_rest_t"), kw["t"], kw.get("obs_age", 0.0), other,
                             params, kw.get("far", False))
    if fact == AHEAD_AT_END:
        return ahead_at_end(kw["own_t_end"], kw["other_t_end"], params)
    raise ValueError(f"unknown role fact {fact}")
