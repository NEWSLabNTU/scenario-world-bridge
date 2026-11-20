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
    # issue 079: kept in the conflict set for cs_hold_s (the decision is the protocol's own), then held by P2c
    n_kept = round(st.p.cs_hold_s / 0.1)
    for k in range(1, n_kept + int(st.p.mode_hold / 0.1)):
        acts = st.cycle(0.1 * k, (x, y, h, 6.0, 0.0), rest, None, stat, 0.1)
        if k < n_kept:
            assert acts.cs_held == {z.id: ("o1",)} and not acts.held_dropped, k
        else:
            assert acts.held_dropped == ("o1",) and not acts.cs_held, k
        assert not acts.rtc and not acts.auto_mode, k
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
    # the last [] spell is a dropped partner (unseen leaving): cs_hold_s kept in the conflict set (issue 079),
    # mode_hold held, then mode_hold of hysteresis
    for k in range(20, 20 + int((st.p.cs_hold_s + 2 * st.p.mode_hold) / 0.1) + 2):
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


def test_a_slow_cycle_holds_the_stop_in_manual_mode():
    """Issue 075 (grid074 s17): the self contract failing on a live node under a stopping decision no longer switches
    the module to auto mode (where Autoware ignores RTC commands and stock, reading the other car at rest as static,
    launched): the module stays manual on DEACTIVATE, with a velocity limit that stops before the zone, and the record
    says "hold: contract". Past hold_by_contract_s the car goes to stock (auto mode). Fails on the code before 075."""
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    stat = [Status("intersection", "ab" * 16, safe=True)]
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    obj = [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)]
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), obj, None, stat, 0.1)
    assert acts.decision.kind in ("wait", "yield") and ("intersection", "ab" * 16, False) in acts.rtc
    kind = acts.decision.kind
    acts = st.cycle(0.1, (x, y, h, 6.0, 0.0), obj, None, stat, 0.1, cycle_time=0.021, self_ok=False, alive=True)
    assert not acts.auto_mode, acts.auto_mode
    assert acts.decision is not None and acts.decision.kind == kind and acts.decision.reason == "hold: contract"
    assert any(r.reason == "hold: contract" for r in acts.records)
    assert st.manual == {"intersection"} and st.sent["ab" * 16] is False  # DEACTIVATE stands
    assert isinstance(acts.velocity_limit, float) and acts.velocity_limit <= math.sqrt(2 * 3.5 * 18.0) + 1e-6
    t = 0.1
    while t < 0.1 + st.p.hold_by_contract_s - 0.1 - 1e-9:
        t = round(t + 0.1, 3)
        acts = st.cycle(t, (x, y, h, 0.0, 0.0), obj, None, stat, 0.1, cycle_time=0.021, self_ok=False, alive=True)
        assert not acts.auto_mode and acts.decision.reason == "hold: contract", t
    acts = st.cycle(0.1 + st.p.hold_by_contract_s, (x, y, h, 0.0, 0.0), obj, None, stat, 0.1, cycle_time=0.021,
                    self_ok=False, alive=True)
    assert ("intersection", True) in acts.auto_mode and acts.velocity_limit == "clear"


def test_a_slow_cycle_on_a_dead_node_goes_to_stock_at_once():
    """R9 unchanged by issue 075: the node not alive (wall time past the period) hands the stop to stock at once."""
    zones, z = setup()
    st = Strategy(zones)
    a0, a1 = z.approaches
    stat = [Status("intersection", "ab" * 16)]
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 20.0)
    obj = [Obj("o1", 1, ox, oy, oh, 6.0, 0.0)]
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), obj, None, stat, 0.1)
    assert acts.decision.kind in ("wait", "yield")
    acts = st.cycle(0.1, (x, y, h, 6.0, 0.0), obj, None, stat, 0.1, cycle_time=0.021, self_ok=False, alive=False)
    assert ("intersection", True) in acts.auto_mode and acts.decision is None


def test_the_trajectory_prefilter_keeps_the_candidates():
    """Issue 075: the trajectory test's bounding-box prefilter and its nearest-first limit return what the full test
    returned (the first max_zones of every zone on the approach that the trajectory crosses, nearest first)."""
    from czp.geom import segment_hits_poly
    zones, z = setup()
    st = Strategy(zones)
    for ap in z.approaches:
        for d in (40.0, 20.0, 5.0, 1.0):
            x, y, h = pose_on(ap, d)
            (ex, ey), _ = point_at(list(ap.centerline), ap.exit_s + 15.0)
            for traj in ([(x, y), (ex, ey)], [(x, y), (x + 0.5 * math.cos(h), y + 0.5 * math.sin(h))], None):
                full = [r for r in st.candidates(x, y, h)
                        if not traj or not r[1].polygon or
                        any(segment_hits_poly(traj[i], traj[i + 1], r[1].polygon) for i in range(len(traj) - 1))]
                for lim in (None, 1, 3):
                    got = st.candidates(x, y, h, traj, None, lim)
                    want = full if lim is None else full[:lim]
                    assert [(r[0], r[1].id) for r in got] == [(r[0], r[1].id) for r in want], (d, lim)


def test_a_stopped_partner_with_its_yaw_flipped_stays_in_the_conflict_set():
    """Issue 079 (U3 on_dt-1_stop1y_c1.5_s0): the other car at rest at its line, its track's yaw flipped by 180
    degrees for one or two messages with vx +0.1 m/s. relate() matched no approach (heading 180 degrees off), the
    8 m path along the flipped yaw missed the zone, and czp went "stock: no conflict" for 0.2 s and back. A
    stopped agent now matches its approach whatever way its yaw points; a moving one still does not."""
    from czp.model import Agent
    from czp.relation import place
    from czp.strategy import Strategy as S
    zones, z = setup()
    a0, a1 = z.approaches
    ox, oy, oh = pose_on(a1, 3.0)
    cap = S(zones).p.capability("car")
    flipped = Agent("o1", pose=(ox, oy, oh + math.pi), v=0.13, cap=cap)
    assert place(z, flipped).approach == a1.id
    assert place(z, replace_v(flipped, 6.0)).approach is None  # moving the other way: not on this approach

    x, y, h = pose_on(a0, 20.0)
    stat = [Status("intersection", "ab" * 16, safe=True)]
    st = Strategy(zones)
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), [Obj("o1", 1, ox, oy, oh, 0.0, 0.0)], None, stat, 0.1)
    assert acts.decision.kind in ("wait", "yield")
    reasons = []
    for k, (yaw, v) in enumerate([(oh, 0.0), (oh + math.pi, 0.13), (oh + math.pi, 0.12), (oh, 0.0),
                                  (oh + math.pi, 0.11), (oh, 0.0)], start=1):
        acts = st.cycle(0.1 * k, (x, y, h, 6.0, 0.0), [Obj("o1", 1, ox, oy, yaw, v, 0.0)], None, stat, 0.1)
        reasons += [r.reason for r in acts.records]
        assert acts.decision is not None and not acts.cs_held and not acts.held_dropped, k
    assert "no conflict" not in reasons


def replace_v(ag, v):
    from dataclasses import replace
    return replace(ag, v=v)


def test_a_partner_dropped_unseen_is_kept_in_the_conflict_set_for_cs_hold_s():
    """Issue 079, the backstop: a partner that drops out of the conflict set for any other reason (here its track
    is missing) is kept in it for cs_hold_s after it was last in it, so the protocol keeps its
    decision and its memory of the partner; the kept ticks have their own decisions.jsonl lines ("cs_hold")."""
    from czp.record import cs_hold_to_json
    import json
    zones, z = setup()
    a0, a1 = z.approaches
    x, y, h = pose_on(a0, 20.0)
    ox, oy, oh = pose_on(a1, 3.0)
    stat = [Status("intersection", "ab" * 16, safe=True)]
    st = Strategy(zones)
    seen = [Obj("o1", 1, ox, oy, oh, 0.0, 0.0)]
    off = []  # its track missing for those cycles
    acts = st.cycle(0.0, (x, y, h, 6.0, 0.0), seen, None, stat, 0.1)
    first = acts.decision
    assert first.kind in ("wait", "yield")
    events, records = [], []
    for k, objs in enumerate([off, off, seen, off] + [off] * 12, start=1):
        acts = st.cycle(0.1 * k, (x, y, h, 6.0, 0.0), objs, None, stat, 0.1)
        events += [json.loads(cs_hold_to_json(e)) for e in acts.cs_events]
        if k <= 12:  # the last time in the set is k = 3; kept while 0.1 k - 0.3 < cs_hold_s
            records += acts.records
            assert acts.decision.kind == first.kind and not acts.held_dropped, k
            assert acts.cs_held == ({} if k == 3 else {z.id: ("o1",)}), k
        else:
            assert not acts.cs_held and acts.held_dropped == ("o1",), k  # then P2c's hold, as before
    assert not records  # the protocol's decision never changed while the partner was kept
    assert [(round(e["t"], 1), e["kept"], e["reason"]) for e in events] == [
        (0.1, ["o1"], "cs_hold: o1 kept in the conflict set"), (0.3, [], "cs_hold: released"),
        (0.4, ["o1"], "cs_hold: o1 kept in the conflict set"), (1.3, [], "cs_hold: released")]
    assert all(e["event"] == "cs_hold" and e["decision"] is None for e in events)
