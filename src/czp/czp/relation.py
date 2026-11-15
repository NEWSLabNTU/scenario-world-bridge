"""Step 2, the relation: each agent's predicted path against the car's route, per zone.

An agent on a zone's approach lanelet (or near an approach's centreline, heading alike) is in that
zone's conflict set with the relation the zone's kind gives; an agent on the own approach ahead of
the own car is a lead, not a conflict. An agent off every lane (a scooter between lanes) is matched
by path intersection: its constant-velocity path over ``horizon`` s against the zone's polygon.
``place`` sets an agent's approach and its s_entry / s_exit from its pose along that approach.
"""

from __future__ import annotations

import math
from dataclasses import replace

from .geom import polyline_project, segment_hits_poly
from .model import (ALL_WAY_STOP, CIRCULATING, CROSS, CROSS_UNSIGNALLED, LEAD, LEFT_TURN, MERGE, MERGE_LANE_DROP,
                    NARROW_PASSAGE, ONCOMING_LEFT, OPPOSE, ROUNDABOUT_ENTRY, Agent, Relation, Zone)

KIND_RELATION = {ALL_WAY_STOP: CROSS, CROSS_UNSIGNALLED: CROSS, MERGE_LANE_DROP: MERGE, NARROW_PASSAGE: OPPOSE,
                 LEFT_TURN: ONCOMING_LEFT, ROUNDABOUT_ENTRY: CIRCULATING, "cut_in": "cut_in"}
LATERAL_MATCH = 2.0  # m from an approach's centreline
HEADING_MATCH = math.radians(45)


def match_approach(zone: Zone, x, y, yaw):
    """(approach, arc length, lateral) of the zone approach this pose drives on, or (None, ...)."""
    best = (None, 0.0, math.inf)
    for ap in zone.approaches:
        if len(ap.centerline) < 2:
            continue
        s, lat, h = polyline_project(ap.centerline, x, y)
        dh = abs(math.atan2(math.sin(yaw - h), math.cos(yaw - h)))
        if abs(lat) <= LATERAL_MATCH and dh <= HEADING_MATCH and abs(lat) < best[2]:
            best = (ap, s, abs(lat))
    return best


def place(zone: Zone, ag: Agent, approach=None):
    """The agent with its approach and its s_entry / s_exit at the zone, from its pose (or unchanged)."""
    if ag.pose is None:
        return ag
    x, y, yaw = ag.pose
    if approach is None:
        ap, s, _ = match_approach(zone, x, y, yaw)
    else:
        ap = zone.approach(approach)
        s, _, _ = polyline_project(ap.centerline, x, y)
    if ap is None:
        return ag
    half = ag.cap.length / 2
    return replace(ag, approach=ap.id, s_entry=ap.entry_s - (s + half), s_exit=ap.exit_s - (s - half))


def relate(agents, own: Agent, zones, route=(), horizon=8.0):
    """Step 2. {zone id: [(Relation, agent placed at that zone)]} for the zones on the own route.
    A zone is on the route when one of its approaches' lanelets is in ``route`` (lanelet ids), or when the
    own agent already carries the approach (the arena)."""
    out = {}
    route = set(route)
    for z in zones:
        own_ap = own.approach if own.approach and any(a.id == own.approach for a in z.approaches) else None
        if own_ap is None:
            hits = [a for a in z.approaches if a.lanelet is not None and a.lanelet in route]
            if not hits:
                continue
            own_ap = hits[0].id
        rel = []
        for ag in agents:
            if ag.id == own.id:
                continue
            placed = ag
            if ag.approach is None or not any(a.id == ag.approach for a in z.approaches):
                ap = next((a for a in z.approaches if ag.lanelet is not None and a.lanelet == ag.lanelet), None)
                placed = place(z, ag, ap.id if ap is not None else None)
            if placed.approach is None:
                if ag.pose is not None and z.polygon and _path_hits(ag, z, horizon):
                    rel.append((Relation(ag.id, z.id, CROSS), _nearest_heading(z, ag)))
                continue
            if placed.approach == own_ap:
                kind = LEAD
            else:
                kind = KIND_RELATION.get(z.kind, CROSS)
            if placed.cleared:
                continue
            rel.append((Relation(ag.id, z.id, kind), placed))
        out[z.id] = rel
    return out


def conflict_set(relations):
    return [(r, a) for r, a in relations if r.kind != LEAD]


def _path_hits(ag: Agent, z: Zone, horizon):
    x, y, yaw = ag.pose
    d = max(ag.v, 1.0) * horizon
    return segment_hits_poly((x, y), (x + d * math.cos(yaw), y + d * math.sin(yaw)), z.polygon)


def _nearest_heading(z: Zone, ag: Agent):
    """An off-lane agent gets the approach whose heading is nearest its own (its map facts), placed by
    its distance to the zone polygon's nearest vertex."""
    x, y, yaw = ag.pose
    ap = min(z.approaches, key=lambda a: abs(math.atan2(math.sin(yaw - a.heading), math.cos(yaw - a.heading))))
    dist = min(math.hypot(px - x, py - y) for px, py in z.polygon)
    return replace(ag, approach=ap.id, s_entry=max(dist - ag.cap.length / 2, 0.0),
                   s_exit=dist + ag.cap.length / 2 + 4.0)
