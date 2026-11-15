"""Step 3, zone intervals: [t_in, t_out] per agent, as [earliest, latest], from its capability.

Own car: from its plan and its own capability (launch dead time, launch acceleration, length,
braking from the spec or its class), never a default car's (074 C8: a heavy car using the
Model 3's launch under-estimates its own interval about 2x). Other agents: earliest arrival at the
class's maximum acceleration up to the speed limit; latest at the class's braking bound (INF when
it can still stop before the zone). Besides the bounds, an Interval carries what the band needs:
the exit of an agent that goes now (its own go-now interval, as it would compute it: so that the two
cars' clear readings are of the same quantity, up to perception error), whether it can still stop,
and its constant-speed (nominal) entry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .model import INF, Agent


def t_cover(d, v, vmax, a, dead=0.0):
    """Time to cover d metres from speed v, after ``dead`` s, accelerating at a up to vmax."""
    if d <= 0:
        return 0.0
    v = max(v, 0.0)
    vmax = max(vmax, 0.1)
    if v >= vmax - 1e-9 or a <= 0:
        return dead + d / max(v if a <= 0 else vmax, 0.1)
    d_acc = (vmax * vmax - v * v) / (2 * a)
    if d <= d_acc:
        return dead + (-v + math.sqrt(v * v + 2 * a * d)) / a
    return dead + (vmax - v) / a + (d - d_acc) / vmax


def t_slow_cover(d, v, decel, v_floor):
    """Time to cover d metres from v, slowing at ``decel`` down to ``v_floor`` (> 0)."""
    if d <= 0:
        return 0.0
    v = max(v, 0.0)
    v_floor = max(min(v_floor, v), 0.05)
    if v <= v_floor:
        return d / v_floor
    d_slow = (v * v - v_floor * v_floor) / (2 * decel)
    if d <= d_slow:
        disc = v * v - 2 * decel * d
        return (v - math.sqrt(max(disc, 0.0))) / decel
    return (v - v_floor) / decel + (d - d_slow) / v_floor


def stop_need(s_entry, v):
    """The deceleration a stop before the zone needs (m/s^2); INF when already at or past it."""
    if s_entry <= 0.05:
        return INF if v > 0.05 else 0.0
    return v * v / (2 * s_entry)


@dataclass(frozen=True)
class ZoneInterval:
    agent: str
    t_in: tuple  # (earliest, latest)
    t_out: tuple
    proceed_out: float  # latest exit of an agent that keeps going (INF: at rest, it may not go)
    nominal_in: float  # constant-speed entry (INF at rest)
    can_stop: bool  # it can still stop before the zone at its braking bound
    reason: str = "track"
    proceed_in: float = INF  # its go-now entry (INF at rest)


def own_interval(own: Agent, params=None, at_rest_v=0.1):
    """The own car's interval if it goes now, from its own capability (fix (b) of 074 C8)."""
    c = own.cap
    dead = c.launch_dead if own.v < at_rest_v else 0.0
    a_go = c.a_launch if own.v < at_rest_v else c.a_max
    if own.cleared:
        return ZoneInterval(own.id, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0, False, "own", 0.0)
    e_in = t_cover(own.s_entry, own.v, c.v_max, c.a_max)
    l_in = t_cover(own.s_entry, own.v, c.v_max, a_go, dead)
    e_out = t_cover(own.s_exit, own.v, c.v_max, c.a_max)
    l_out = t_cover(own.s_exit, own.v, c.v_max, a_go, dead)
    return ZoneInterval(own.id, (e_in, l_in), (e_out, l_out), l_out, own.s_entry / own.v if own.v > 0.1 else INF,
                        own.s_entry > 0 and stop_need(own.s_entry, own.v) <= c.b_bound, "own", l_in)


def other_interval(ag: Agent, params, at_rest_v=0.1):
    """Another agent's interval by reachability from its track and its class's bounds."""
    c = ag.cap
    if ag.cleared:
        return ZoneInterval(ag.id, (INF, INF), (INF, INF), -INF, INF, False, "cleared")
    reason = "phantom" if ag.phantom else "track"
    dead = c.launch_dead if ag.v < at_rest_v else 0.0
    e_in = 0.0 if ag.s_entry <= 0 else t_cover(ag.s_entry, ag.v, c.v_max, c.a_max, dead)
    e_out = t_cover(ag.s_exit, ag.v, c.v_max, c.a_max, dead)
    need = stop_need(ag.s_entry, ag.v)
    can_stop = ag.s_entry > 0 and need <= c.b_bound
    if ag.s_entry <= 0:
        l_in = 0.0
    elif can_stop:
        l_in = INF
    else:  # it cannot stop: braking at its bound it still reaches the zone
        disc = ag.v * ag.v - 2 * c.b_bound * ag.s_entry
        l_in = (ag.v - math.sqrt(max(disc, 0.0))) / c.b_bound
    l_out = INF if can_stop or ag.v < at_rest_v else t_slow_cover(ag.s_exit, ag.v, c.b_bound, 0.05)
    # if it goes now: its own go-now exit (as it computes its own interval, from its class's capability),
    # so that this car's "clear for the other" and the other's "clear for me" read the same quantity
    proceed = INF if ag.v < at_rest_v else t_cover(ag.s_exit, ag.v, c.v_max, c.a_max)
    p_in = INF if ag.v < at_rest_v else (0.0 if ag.s_entry <= 0 else t_cover(ag.s_entry, ag.v, c.v_max, c.a_max))
    nominal = 0.0 if ag.s_entry <= 0 else (ag.s_entry / ag.v if ag.v > 0.1 else INF)
    return ZoneInterval(ag.id, (e_in, l_in), (e_out, l_out), proceed, nominal, can_stop, reason, p_in)


def intervals(agent: Agent, zone_id, params, own=False):
    """Step 3 (the design's signature): the agent's ZoneInterval at the zone."""
    return own_interval(agent, params) if own else other_interval(agent, params)
