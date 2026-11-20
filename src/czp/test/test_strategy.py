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
    # its track is gone, unseen leaving (078 P2c ruling): the last decision holds for mode_hold; then for mode_hold
    # the module stays manual and follows its own safe (ACTIVATE here); then auto
    acts = st.cycle(2.0, (x, y, h, 0.0, 0.0), [], None, stat, 0.1)
    assert not acts.auto_mode and not acts.rtc and acts.held_dropped == ("o1",)
    acts = st.cycle(2.0 + st.p.mode_hold, (x, y, h, 0.0, 0.0), [], None, stat, 0.1)
    assert not acts.auto_mode and acts.rtc == [("intersection", "ab" * 16, True)] and not acts.held_dropped
    acts = st.cycle(2.0 + 2 * st.p.mode_hold, (x, y, h, 0.0, 0.0), [], None, stat, 0.1)
    assert ("intersection", True) in acts.auto_mode and not acts.rtc


def test_partner_seen_leaving_is_not_held():
    """078 P2c ruling: a partner seen inside the zone that then drops out of the conflict set has exited; one still
    tracked and moving off the approaches has turned away. Neither holds the decision: the mode hysteresis starts
    at once (the module's own safe)."""
    zones, z = setup()
    a0, a1 = z.approaches
    stat = [Status("intersection", "ab" * 16)]
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    for case in ("exit", "turn"):
        st = Strategy(zones)
        acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)], None, stat, 0.1)
        assert acts.decision is not None
        if case == "exit":
            ix, iy, ih = pose_on(a1, -3.0)  # its front 3 m inside
            acts = st.cycle(0.5, (x, y, h, 0.0, 0.0), [Obj("o1", 1, ix, iy, ih, 6.0, 0.0)], None, stat, 0.1)
            assert st.partners["o1"].s_entry <= 0.0
            objs = []
        else:  # tracked, moving, heading 90 degrees off its approach: placed off the zone
            objs = [Obj("o1", 1, ox, oy, oh + math.pi / 2, 6.0, 0.0)]
        acts = st.cycle(1.0, (x, y, h, 0.0, 0.0), objs, None, stat, 0.1)
        assert not acts.held_dropped and acts.rtc == [("intersection", "ab" * 16, True)], (case, acts)


def test_dropped_partner_at_rest_holds_the_wait():
    """078 P2c: the other car at rest at its line drops from the conflict set for one cycle (its track's yaw flips,
    or the track is lost). The node keeps its wait (DEACTIVATE stays), not the module's safe = true; a failed
    contract still releases at once."""
    zones, z = setup()
    a0, a1 = z.approaches
    stat = [Status("intersection", "ab" * 16, safe=True)]
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    st = Strategy(zones)
    obj = [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)]
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), obj, None, stat, 0.1)
    assert acts.decision.kind in ("wait", "yield") and ("intersection", "ab" * 16, False) in acts.rtc
    rest = [Obj("o1", 1, ox, oy, oh + math.pi / 2, 0.0, 0.0)]  # at rest, its yaw read 90 degrees off
    for k in range(1, int(st.p.mode_hold / 0.1)):
        acts = st.cycle(0.1 * k, (x, y, h, 6.0, 0.0), rest, None, stat, 0.1)
        assert acts.held_dropped == ("o1",) and not acts.rtc and not acts.auto_mode, k
        assert acts.decision.kind in ("wait", "yield")
    acts = st.cycle(0.1 * k + 0.1, (x, y, h, 6.0, 0.0), rest, None, stat, obs_age=5.0)  # the latency contract fails
    assert ("intersection", True) in acts.auto_mode and not acts.held_dropped


def test_mode_hysteresis_band_edge_flicker():
    """078 P2b: a reading that flickers between a held decision and stock-equivalent at the band's edge switches
    the mode once, not at every flip; the commands follow the module's own safe while stock-equivalent."""
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    tie = [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)]
    stat = [Status("intersection", "ab" * 16, safe=False)]
    switches = 0
    for k in range(20):  # 0.1 s per cycle: the tie, then nothing (stock), alternating every 0.3 s
        objs = tie if (k // 3) % 2 == 0 else []
        acts = st.cycle(0.1 * k, (x, y, h, 6.0, 0.0), objs, None, stat, 0.1)
        switches += len(acts.auto_mode)
    assert switches == 1 and st.manual == {"intersection"}
    # the last [] spell is a dropped partner (unseen leaving): mode_hold held, then mode_hold of hysteresis
    for k in range(20, 20 + int(2 * st.p.mode_hold / 0.1) + 2):
        acts = st.cycle(0.1 * k, (x, y, h, 6.0, 0.0), [], None, stat, 0.1)
        switches += len(acts.auto_mode)
    assert switches == 2 and not st.manual


def test_mode_hysteresis_released_by_a_contract():
    """A failed contract hands the module back to auto at once, hysteresis or not."""
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    obj = [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)]
    stat = [Status("intersection", "ab" * 16)]
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), obj, None, stat, 0.1)
    assert ("intersection", False) in acts.auto_mode
    acts = st.cycle(0.1, (x, y, h, 6.0, 0.0), obj, None, stat, obs_age=5.0)  # the latency contract fails
    assert ("intersection", True) in acts.auto_mode


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


def test_watch_from_a_clear_go_deactivates():
    """Issue 069 part 3: the probe's watch, standing on a go that reads clear (stock-equivalent) once the car moves,
    is still czp's decision: the strategy sends DEACTIVATE (the 069 grid's s13 kept ACTIVATE and overran)."""
    from czp.model import CREEP, Decision
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    stat = [Status("intersection", "ab" * 16)]
    x, y, h = pose_on(a0, 1.0)
    ox, oy, oh = pose_on(a1, 1.2)
    obj = [Obj("o1", 1, ox, oy, oh, 0.0, 0.0)]
    from czp.model import STOCK
    now = {"d": Decision(CREEP, z.id, "probe:creep", stock_equivalent=False)}
    st.proto.step = lambda t, own, confl, zone, *a, **k: (now["d"] if zone.id == z.id else
                                                          Decision(STOCK, zone.id, "no conflict"), None)
    acts = st.cycle(0.0, (x, y, h, 0.0, 0.0), obj, None, stat, 0.1)
    assert ("intersection", "ab" * 16, True) in acts.rtc and acts.velocity_limit == st.p.creep_speed
    now["d"] = Decision(CREEP, z.id, "probe:watch", stock_equivalent=False)
    acts = st.cycle(0.1, (x, y, h, 0.6, 0.5), obj, None, stat, 0.1)
    assert ("intersection", "ab" * 16, False) in acts.rtc and acts.velocity_limit == "clear"


def test_velocity_limit_types_are_autoware_s():
    """Issue 074: czp's velocity limit and its clear are published in Autoware 1.5.0's types
    (autoware_internal_planning_msgs), which its external_velocity_limit_selector subscribes to; the
    tier4_planning_msgs ones of the same name reach no node."""
    import ast
    pkg = os.path.join(os.path.dirname(__file__), "..", "czp")
    for name, wanted in (("node.py", {"VelocityLimit", "VelocityLimitClearCommand"}),
                         ("watchdog.py", {"VelocityLimitClearCommand"})):
        found = {}
        for n in ast.walk(ast.parse(open(os.path.join(pkg, name)).read())):
            if isinstance(n, ast.ImportFrom):
                for a in n.names:
                    if a.name in wanted:
                        found[a.name] = n.module
        assert found == {k: "autoware_internal_planning_msgs.msg" for k in wanted}, (name, found)
    hdr = ("/opt/autoware/1.5.0/include/autoware/external_velocity_limit_selector/"
           "external_velocity_limit_selector_node.hpp")
    if os.path.exists(hdr):
        text = open(hdr).read()
        assert "using autoware_internal_planning_msgs::msg::VelocityLimit;" in text
        assert "using autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;" in text
