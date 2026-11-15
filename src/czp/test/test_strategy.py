"""The ROS adapter's logic (czp.strategy) on the Town01 fixture: nothing sent on a clear route (EV2's
contract), RTC manual mode and DEACTIVATE when another car holds the turn, auto mode back after."""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from czp.geom import point_at  # noqa: E402
from czp.strategy import Obj, Status, Strategy  # noqa: E402
from czp.zones import load_zones  # noqa: E402

FIX = os.path.join(os.path.dirname(__file__), "fixtures", "town01_j35161.yaml")


def setup():
    _, zones = load_zones(FIX)
    z = next(z for z in zones if z.kind == "cross_unsignalled")
    return zones, z


def pose_on(ap, s_before_entry, length=4.79):
    (x, y), h = point_at(list(ap.centerline), ap.entry_s - s_before_entry - length / 2)
    return x, y, h


def test_clear_route_sends_nothing():
    zones, z = setup()
    st = Strategy(zones)
    a0 = z.approaches[0]
    for k in range(50):
        x, y, h = pose_on(a0, 40.0 - 0.6 * k)
        acts = st.cycle(0.1 * k, (x, y, h, 6.0, 0.0), [], None, [Status("intersection", "aa" * 16)], 0.1)
        assert not acts.rtc and not acts.auto_mode and acts.velocity_limit is None
    assert st.counts == {"rtc": 0, "auto_mode": 0, "velocity_limit": 0}


def test_contested_is_stock():
    """A car that cannot or will not stop keeps the turn: stock Autoware yields to it too, nothing is sent."""
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    x, y, h = pose_on(a0, 12.0)
    ox, oy, oh = pose_on(a1, 6.0)
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), [Obj("o1", 1, ox, oy, oh, 10.0, 0.0)], None,
                    [Status("intersection", "ab" * 16)], 0.1)
    assert acts.decision is None and not acts.rtc and acts.records[0].reason == "contested"


def test_tie_takes_rtc_then_back_to_auto():
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    stat = [Status("intersection", "ab" * 16)]
    # a tie (both 20 m out at 6 m/s): the band's edge, both wait -- not what stock would do
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)], None, stat, 0.1)
    assert acts.decision is not None and acts.decision.kind in ("wait", "yield")
    assert ("intersection", False) in acts.auto_mode
    assert ("intersection", "ab" * 16, False) in acts.rtc
    # it has gone: the module goes back to auto mode, nothing else is sent
    acts = st.cycle(2.0, (x, y, h, 0.0, 0.0), [], None, stat, 0.1)
    assert ("intersection", True) in acts.auto_mode and not acts.rtc


def test_velocity_limit_without_rtc_cover():
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)], None, [], 0.1)
    assert acts.decision.kind in ("wait", "yield") and isinstance(acts.velocity_limit, float)
    assert acts.velocity_limit <= math.sqrt(2 * 3.5 * 18.0) + 1e-6


def test_held_module_seen_in_auto_gets_the_command_again():
    """078 P2: a module the node holds that reports auto mode (the watchdog restored it) is commanded again."""
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    obj = [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)]
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), obj, None, [Status("intersection", "ab" * 16)], 0.1)
    assert ("intersection", "ab" * 16, False) in acts.rtc
    acts = st.cycle(0.1, (x, y, h, 6.0, 0.0), obj, None, [Status("intersection", "ab" * 16, auto_mode=False)], 0.1)
    assert not acts.rtc  # held and manual: nothing new
    acts = st.cycle(0.2, (x, y, h, 6.0, 0.0), obj, None, [Status("intersection", "ab" * 16, auto_mode=True)], 0.1)
    assert ("intersection", "ab" * 16, False) in acts.rtc
