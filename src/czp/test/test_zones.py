"""zones.yaml (czp-zones' output) as the library reads it: a Town01 T-junction fixture."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import czp  # noqa: E402
from czp.geom import point_at  # noqa: E402
from czp.model import Agent  # noqa: E402
from czp.relation import relate  # noqa: E402
from czp.zones import load_zones, zone_from_dict, zone_to_dict  # noqa: E402

FIX = os.path.join(os.path.dirname(__file__), "fixtures", "town01_j35161.yaml")


def test_load_and_order_total():
    head, zones = load_zones(FIX)
    assert head["name"] == "Town01" and zones
    for z in zones:
        orders = [a.order for a in z.approaches]
        assert len(set(orders)) == len(orders)  # never a tie in the map's order
        assert all(a.exit_s > a.entry_s for a in z.approaches)
        assert len(z.polygon) >= 3


def test_round_trip():
    _, zones = load_zones(FIX)
    z = zones[0]
    assert zone_from_dict(zone_to_dict(z)) == z


def test_relate_places_agents_on_approaches():
    """An agent on one approach's path, 20 m before the zone, is placed on it with s_entry ~ 20 m."""
    _, zones = load_zones(FIX)
    z = next(z for z in zones if z.kind == "cross_unsignalled")
    a0, a1 = z.approaches
    p = czp.load_params()
    cap = p.capability("car")
    (x, y), h = point_at(list(a0.centerline), a0.entry_s - 20.0 - cap.length / 2)
    own = Agent("ego", approach=None, lanelet=a1.lanelet, pose=None, cap=cap)
    other = Agent("o1", pose=(x, y, h), v=5.0, cap=cap)
    rel = relate([other], own, [z], route=[a1.lanelet])
    (r, placed), = rel[z.id]
    assert placed.approach == a0.id and r.kind == "cross"
    assert abs(placed.s_entry - 20.0) < 1.5, placed.s_entry
    assert not math.isinf(placed.s_exit)
