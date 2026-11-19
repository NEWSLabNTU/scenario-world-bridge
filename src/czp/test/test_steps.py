"""Unit tests: each step against hand cases from the scenario walkthroughs (cooperative-driving.md)."""

import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import czp  # noqa: E402
from czp import contracts as C  # noqa: E402
from czp.act import latch  # noqa: E402
from czp.band import band  # noqa: E402
from czp.convention import break_cycles, decide, decide_all, find_cycle  # noqa: E402
from czp.intervals import other_interval, own_interval, t_cover  # noqa: E402
from czp.memory import Debounce, Memory, debounce, update_memory  # noqa: E402
from czp.model import (AMBIG, AMBIGUOUS, BAND, CLEAR, CONTESTED, CREEP, FIRST, GO, SECOND, STOCK, TIE, WAIT,
                       YIELD, Agent, Approach, Decision, Zone)  # noqa: E402
from czp.probe import classify_response, probe  # noqa: E402
from czp.memory import ProbeState  # noqa: E402
from czp.record import to_json  # noqa: E402
from czp.role import map_order, map_priority, obstruction_side, right_of, stopped_first  # noqa: E402

P = czp.load_params()
CONV = czp.load_conventions()
# the all-way stop's readings at 074 C8's arena e (0.25 s, no bias), where m >= e holds (params.yaml's e is Autoware's)
P_C8 = P.override(source="test: 074 C8 arena e", role_time_error_e=0.25, arrival_bias=0.0,
                rest_stamp_bias=0.0)
# exact rest stamps (decide_all's walkthroughs give every car the truth): no lag to correct
P_EXACT = P.override(source="test: exact stamps", rest_stamp_bias=0.0)
CAR = P.capability("car", v_max=6.0)


def four_way(n=2):
    """A junction with n approaches: A heading east (order 0), B from A's right (north-bound, order 1), C
    west-bound (order 2), D south-bound (order 3)."""
    heads = [0.0, math.pi / 2, math.pi, -math.pi / 2]
    return Zone("j", "all_way_stop", tuple(Approach("ABCD"[i], heading=heads[i], order=i) for i in range(n)))


def car(i, s_entry=1.0, v=0.0, a=0.0, stamp=0.0, cap=CAR):
    return Agent(i, approach=i, s_entry=s_entry, s_exit=s_entry + 3.5 + 4.79, v=v, a=a, stamp=stamp, cap=cap)


# --- intervals -----------------------------------------------------------------------------------------
def test_t_cover():
    assert t_cover(0, 5, 10, 3) == 0
    assert abs(t_cover(10, 10, 10, 3) - 1.0) < 1e-9
    assert abs(t_cover(6, 0, 10, 3) - 2.0) < 1e-9  # 0.5 a t^2 = 6
    assert abs(t_cover(6, 0, 10, 3, dead=0.5) - 2.5) < 1e-9


def test_own_interval_from_own_capability():
    """074 C8: a heavy car's own interval from its own launch is about 2x the Model 3's."""
    m3 = own_interval(car("A"))
    hgv_cap = P.capability("truck", v_max=6.0)
    hgv = own_interval(Agent("A", approach="A", s_entry=1.0, s_exit=1.0 + 3.5 + hgv_cap.length, cap=hgv_cap))
    assert hgv.t_out[1] > 1.5 * m3.t_out[1]


def test_other_interval_can_stop():
    far = other_interval(car("B", s_entry=30, v=6), P)
    assert far.can_stop and math.isinf(far.t_in[1])
    near = other_interval(car("B", s_entry=2, v=10), P)
    assert not near.can_stop and near.t_in[1] < 0.5


# --- band ----------------------------------------------------------------------------------------------
def test_band_clear_band_ambiguous():
    own = car("A", s_entry=1.0)
    oiv = own_interval(own)
    far = car("B", s_entry=60, v=6, a=-1.0)
    assert band(oiv, other_interval(far, P), far, P, stopping=True).kind == CLEAR
    rest = car("B", s_entry=1.0)
    assert band(oiv, other_interval(rest, P), rest, P, rested=True).kind == BAND
    # at the band's edge the reading is ambiguous
    found = set()
    for s in [x * 0.5 for x in range(10, 80)]:
        o = car("B", s_entry=s, v=6, a=-2.0)
        found.add(band(oiv, other_interval(o, P), o, P, stopping=True).kind)
    assert AMBIGUOUS in found and CLEAR in found


def test_band_contested_zone_guard():
    """C8's zone guard at a stop: a car not braking for its line that would be in before I am out."""
    own = car("A", s_entry=1.0)
    o = car("B", s_entry=8, v=6, a=0.0)
    assert band(own_interval(own), other_interval(o, P), o, P, stopping=False, stop_law=True).kind == CONTESTED
    assert band(own_interval(own), other_interval(o, P), o, P, stopping=True, stop_law=True).kind != CONTESTED


def test_band_gap_merge():
    """078 S3: at a lane drop the merge is judged on who leads whom by a headway now, not on arrival at the
    end: a car ahead but slower is caught before the end; a car beside but faster is not ahead yet."""
    def lane(i, front_to_end, v):  # s_entry: front to the lane's end; s_exit: the rear's
        return Agent(i, approach=i, s_entry=front_to_end, s_exit=front_to_end + 4.79, v=v, cap=P.capability("car"))
    a = lane("A", 80.0, 12.0)
    b_far_behind = lane("B", 80.0 + 4.79 + 25.0, 12.0)  # A's rear 25 m ahead of B's front, same speed
    assert band(own_interval(a), other_interval(b_far_behind, P), b_far_behind, P, mode="gap", own=a).kind == CLEAR
    assert band(own_interval(a), other_interval(b_far_behind, P), b_far_behind, P, mode="gap", own=a).who == "A"
    beside_faster = lane("A", 80.0, 15.0)
    b_beside = lane("B", 80.0 - 0.2, 12.0)  # fronts level, A 3 m/s faster: still not ahead by a headway
    k = band(own_interval(beside_faster), other_interval(b_beside, P), b_beside, P, mode="gap", own=beside_faster).kind
    assert k in (BAND, AMBIGUOUS)
    b_ahead = lane("B", 80.0 - 4.79 - 25.0, 12.0)  # B's rear 25 m ahead of A's front
    r = band(own_interval(a), other_interval(b_ahead, P), b_ahead, P, mode="gap", own=a)
    assert r.kind == CLEAR and r.who == "B"


# --- role ----------------------------------------------------------------------------------------------
def test_stopped_first_readings():
    T, m = P_C8.tie_threshold_T, P_C8.tie_margin_m
    o = car("B")
    assert stopped_first(10.0, 10.0 + T + m + 0.01, 20, 0.7, o, P_C8).reading == FIRST
    assert stopped_first(10.0, 10.0 - T - m - 0.01, 20, 0.7, o, P_C8).reading == SECOND
    assert stopped_first(10.0, 10.0 + 0.1, 20, 0.7, o, P_C8).reading == TIE
    assert stopped_first(10.0, 10.0 + T, 20, 0.7, o, P_C8).reading == AMBIG  # at the threshold: ambiguous
    # no stamp yet: first only once one within T + m would be known
    assert stopped_first(10.0, None, 10.0 + 0.7 + T + m + 0.01, 0.7, o, P_C8).reading == FIRST
    assert stopped_first(10.0, None, 10.5, 0.7, o, P_C8).reading != FIRST


def test_map_facts():
    z = four_way(4)
    a, b = car("A"), car("B")
    assert right_of(z, "A", "B") and not right_of(z, "B", "A")
    assert map_order(z, a, b).reading == FIRST and map_order(z, b, a).reading == SECOND
    zp = Zone("x", "cross_unsignalled", (Approach("A", priority="major"), Approach("B", heading=math.pi / 2, order=1,
                                                                                        priority="minor")))
    assert map_priority(zp, a, b).reading == FIRST
    zn = Zone("n", "narrow_passage", (Approach("A", obstructed=True), Approach("B", heading=math.pi, order=1)))
    assert obstruction_side(zn, a, b).reading == SECOND


# --- convention ----------------------------------------------------------------------------------------
def test_four_way_two_cars_tie_right_before_left():
    """Walkthrough: two cars at a tie at an all-way stop: B (on A's right) goes, A waits."""
    z = four_way(2)
    ags = [car("A"), car("B")]
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.05}, P_EXACT, t=3.0, quiet_since=-math.inf)
    assert d["B"].kind == GO and d["A"].kind != GO


def test_four_way_ambiguous_both_wait_then_round_two():
    z = four_way(2)
    ags = [car("A"), car("B")]
    T = P.tie_threshold_T
    rests = {"A": 0.0, "B": T}  # exactly at the threshold: both read ambiguous
    d = decide_all(CONV, z, ags, rests, P, t=T + P.stop_dwell + 0.5, quiet_since=-math.inf)
    assert d["A"].kind != GO and d["B"].kind != GO
    t2 = T + P.stop_dwell + ((CAR.launch_dead + P.see_delay) if P.fresh_rest_guard else 0.0) + P.second_round_H2 + 0.1
    d = decide_all(CONV, z, ags, rests, P, t=t2, quiet_since=-math.inf)
    assert d["B"].kind == GO and d["B"].reason == "round2" and d["A"].kind != GO


def test_four_way_four_at_once_cycle_broken_by_map_order():
    """Walkthrough: four at once, right-before-left is cyclic: the map's order (A first) breaks it."""
    z = four_way(4)
    ags = [car(i) for i in "ABCD"]
    rests = {i: 0.0 for i in "ABCD"}
    d = decide_all(CONV, z, ags, rests, P_C8, t=1.5, quiet_since=-math.inf)
    assert all(x.kind != GO for x in d.values())
    fresh = (CAR.launch_dead + P_C8.see_delay) if P_C8.fresh_rest_guard else 0.0
    d = decide_all(CONV, z, ags, rests, P_C8, t=P_C8.stop_dwell + fresh + P_C8.second_round_H2 + 0.1, quiet_since=-math.inf)
    goes = [k for k, x in d.items() if x.kind == GO]
    assert goes == ["A"] and d["A"].reason == "cycle"


def test_rest_tie_binds_after_a_mutual_stop():
    """078 V4: at an unsignalled crossing both cars at rest at their lines read a tie the same way: the car on
    the right (B) goes, after the fresh-rest guard's wait; A yields; neither reads the noisy band."""
    z = Zone("x", "cross_unsignalled", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))
    ags = [car("A"), car("B")]
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.0}, P_EXACT, t=guard - 0.1, quiet_since=-math.inf)
    assert d["B"].kind != GO and d["A"].kind != GO
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.0}, P_EXACT, t=guard + 0.1, quiet_since=-math.inf)
    assert d["B"].kind == GO and d["B"].reason == "rest_tie" and d["A"].kind != GO


def test_own_rest_at_autowares_line():
    """078 P2c: the own car comes to rest at Autoware's stop line, up to own_rest_window before the zone (3.3 m at
    Town05 j979): its rest registers there; the other car's rest window stays rest_window."""
    m = update_memory(Memory(), 5.0, car("A", s_entry=3.3, v=0.0), [], P)
    assert m.own_rest_t == 5.0
    m = update_memory(Memory(), 5.0, car("A", s_entry=P.own_rest_window + 0.5, v=0.0), [], P)
    assert m.own_rest_t is None
    b = car("B", s_entry=3.3, v=0.0)
    m = update_memory(Memory(), 5.0, car("A", s_entry=1.0, v=0.0), [b], P)
    assert m.agents["B"].rest_t is None


def test_guards_take_no_rest_stamp_correction():
    """078 P2c ruling: rest_stamp_bias orders stopped_first only. rest_tie's guard (and the fresh-rest guard, the
    claim, the second round's epoch, all through rest_since) counts from the raw stamp: with the bias at 0.7 s the
    winner still waits the full dwell + launch + see_delay after the stamp, not 0.7 s less. A correction may only
    lengthen a guard: a negative bias does."""
    from czp.convention import rest_since
    z = Zone("x", "cross_unsignalled", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))
    ags = [car("A"), car("B")]
    bias = P.rest_stamp_bias
    assert bias > 0
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.0}, P, t=guard - bias + 0.1, quiet_since=-math.inf)
    assert d["B"].kind != GO and d["B"].reason == "rest_tie_wait" and d["A"].kind != GO
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.0}, P, t=guard - 0.1, quiet_since=-math.inf)
    assert d["B"].kind != GO and d["B"].reason == "rest_tie_wait"
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.0}, P, t=guard + 0.1, quiet_since=-math.inf)
    assert d["B"].kind == GO and d["B"].reason == "rest_tie" and d["A"].kind != GO
    assert rest_since(10.0, P.override(source="test", rest_stamp_bias=0.7)) == 10.0
    assert rest_since(10.0, P.override(source="test", rest_stamp_bias=-0.3)) == 10.3
    assert rest_since(None, P) is None


def test_fresh_rest_guard_takes_no_rest_stamp_correction():
    """078 P2c ruling: the fresh-rest guard of a stopped_first go counts from the raw stamp at any rest_stamp_bias."""
    from czp.convention import PairView, pair_verdict
    from czp.memory import AgentMemory
    p = P_C8.override(source="test", rest_stamp_bias=0.7)
    assert p.fresh_rest_guard
    z = Zone("x", "all_way_stop", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))
    guard = p.stop_dwell + CAR.launch_dead + p.see_delay
    own, o = car("A", s_entry=0.5, v=0.0), car("B", s_entry=0.5, v=0.0)
    for t, want in ((10.0 + guard - 0.1, "fresh"), (10.0 + guard + 0.1, "first")):
        mem = Memory(own_rest_t=5.0)
        am = AgentMemory(rest_t=10.0, rest_s=0.5, stopping=Debounce(state=True))
        pv = PairView(o, other_interval(o, p), am)
        v = pair_verdict({"first": "stopped_first", "stop": False}, z, own, own_interval(own, p), pv, mem, p, t)
        assert v.reason == want, (t, v)


def test_stopped_first_corrects_the_rest_stamp():
    """078 P2c: stopped_first reads the other's stamp less rest_stamp_bias: a stamp 0.85 s after the own rest, read
    0.7 s late, is a tie, not first."""
    o = car("B")
    p = P_C8.override(source="test", rest_stamp_bias=0.7)
    assert stopped_first(10.0, 10.85, 20, 0.2, o, p).reading == TIE
    assert stopped_first(10.0, 10.85, 20, 0.2, o, P_C8).reading == FIRST


def test_narrow_round_two_follows_the_obstruction():
    """078 V4: the narrow passage's tie rule is the obstruction side (the free side first), then the map's order."""
    from czp.convention import tie_rule
    zn = Zone("n", "narrow_passage", (Approach("A", obstructed=True), Approach("B", heading=math.pi, order=1)))
    a, b = car("A"), car("B")
    assert not tie_rule("obstruction_side", zn, a, b) and tie_rule("obstruction_side", zn, b, a)
    ze = Zone("n", "narrow_passage", (Approach("A"), Approach("B", heading=math.pi, order=1)))
    assert tie_rule("obstruction_side", ze, a, b)  # equal sides: the map's order (A first)


def test_narrow_claim_waits_for_the_other_to_yield():
    """078 V4: at the narrow passage a go that needs the other to yield (by arrival, not out before it could be
    in) waits until the other is seen at rest for the guard's time."""
    zn = Zone("n", "narrow_passage", (Approach("A", obstructed=True), Approach("B", heading=math.pi, order=1)))
    own = Agent("B", approach="B", s_entry=12.0, s_exit=12.0 + 36.0 + 4.79, v=6.0, cap=CAR)
    o = Agent("A", approach="A", s_entry=20.0, s_exit=20.0 + 36.0 + 4.79, v=6.0, cap=CAR)
    from czp.convention import PairView, pair_verdict
    from czp.memory import AgentMemory
    v = pair_verdict(CONV["narrow_passage"], zn, own, own_interval(own), PairView(o, other_interval(o, P), AgentMemory()),
                     Memory(), P, 0.0)
    assert v.kind == WAIT and v.reason == "claim"


def test_bystander_events_restart_the_quiet_epoch():
    """078 V4: a car outside the conflict set (the opposite arm at a crossing) starting or leaving is a junction
    event: the round-two clock restarts."""
    p = czp.Protocol(params=P)
    z = Zone("x", "cross_unsignalled", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))
    own = car("A")
    by_in = Agent("C", approach="C", s_entry=-1.0, s_exit=5.0, v=3.0, stamp=1.0, cap=CAR)
    for k in range(4):
        p.step(1.0 + 0.1 * k, own, [car("B", stamp=1.0 + 0.1 * k)], z, bystanders=[replace_stamp(by_in, 1.0 + 0.1 * k)])
    assert p.mem["x"].last_event >= 1.0


def replace_stamp(a, t):
    from dataclasses import replace
    return replace(a, stamp=t)


def test_exit_box_blocks_go():
    z = four_way(2)
    d = decide_all(CONV, z, [car("A"), car("B")], {"A": 0.0, "B": 0.05}, P, t=3.0, quiet_since=-math.inf,
                   exit_clear=False)
    assert all(x.kind != GO for x in d.values())


def test_stop_first_dwell():
    z = four_way(2)
    mem = Memory(own_rest_t=0.0)
    own = car("A")
    o = car("B", s_entry=80, v=6)
    from czp.convention import PairView
    from czp.memory import AgentMemory
    dec, _ = decide(CONV, z, own, own_interval(own), [PairView(o, other_interval(o, P), AgentMemory())], mem, P, t=0.5)
    assert dec.kind == WAIT and dec.reason == "stop"
    dec, _ = decide(CONV, z, own, own_interval(own), [PairView(o, other_interval(o, P), AgentMemory())], mem, P, t=1.2)
    assert dec.kind == GO and dec.stock_equivalent


def test_no_conflict_is_stock():
    z = four_way(2)
    dec, _ = decide(CONV, z, car("A"), own_interval(car("A")), [], Memory(), P, t=0.0)
    assert dec.kind == STOCK


def test_cycle_tools():
    e = {("a", "b"), ("b", "c"), ("c", "a"), ("c", "d")}
    assert set(find_cycle(e, ["a", "b", "c", "d"])) == {"a", "b", "c"}
    left, rel = break_cycles(e, ["a", "b", "c", "d"], {"a": 2, "b": 0, "c": 1, "d": 3})
    assert rel == ["b"] and find_cycle(left, ["a", "b", "c", "d"]) is None


# --- memory, probe, act --------------------------------------------------------------------------------
def test_debounce_counts_distinct_samples():
    d = Debounce()
    d = debounce(d, True, 0.1, 2)
    d = debounce(d, True, 0.1, 2)  # the same sample again
    assert not d.state
    d = debounce(d, True, 0.2, 2)
    assert d.state and d.flips == 1
    d = debounce(d, False, 0.3, 2)
    d = debounce(d, True, 0.4, 2)
    assert d.state and d.flips == 1  # one noisy sample does not flip it


def test_rest_stamp_is_the_observation_time():
    mem = update_memory(Memory(), 5.0, car("A", s_entry=1.0, v=0.0), [car("B", s_entry=1.2, v=0.0, stamp=4.3)], P)
    assert mem.agents["B"].rest_t == 4.3 and mem.own_rest_t == 5.0


def test_probe_creep_watch_go():
    row = CONV["all_way_stop"]
    dec = Decision(GO, "j", "round2")
    own = car("A", s_entry=1.0)
    d, ps = probe(dec, own, ProbeState(), P, 10.0, row)
    assert d.kind == CREEP and d.reason == "probe:creep"
    own = car("A", s_entry=1.0 - P.probe_creep_dist, v=0.0)
    d, ps = probe(dec, own, ps, P, 11.0, row)
    assert d.kind == CREEP and d.reason == "probe:watch"
    d, ps = probe(dec, own, ps, P, 11.0 + P.probe_watch, row)
    assert d.kind == GO and ps.phase == "done"
    # a hard verdict during the probe aborts it
    d, ps2 = probe(Decision(YIELD, "j", "turn"), own, ProbeState("watch", 11.0, 1.0, "round2"), P, 11.2, row)
    assert d.kind == YIELD and ps2.phase == "abort"


def test_probe_is_not_bypassed_by_another_go():
    """078 P2g: a go for another reason (the rest guard's clear) during the creep or the watch stays in the probe;
    only a car already inside goes at once."""
    row = CONV["all_way_stop"]
    own = car("A", s_entry=1.0)
    d, ps = probe(Decision(GO, "j", "last_resort"), own, ProbeState(), P, 10.0, row)
    assert d.kind == CREEP and ps.phase == "creep"
    d, ps = probe(Decision(GO, "j", "clear"), car("A", s_entry=0.8, v=0.5), ps, P, 10.3, row)
    assert d.kind == CREEP and d.reason == "probe:creep"
    own = car("A", s_entry=1.0 - P.probe_creep_dist, v=0.0)
    d, ps = probe(Decision(GO, "j", "clear"), own, ps, P, 10.6, row)
    assert d.kind == CREEP and d.reason == "probe:watch"
    d, ps = probe(Decision(GO, "j", "clear"), own, ps, P, 10.6 + P.probe_watch, row)
    assert d.kind == GO and ps.phase == "done"
    d, _ = probe(Decision(GO, "j", "inside"), car("A", s_entry=0.0, v=1.0), ProbeState("watch", 10.6, 1.0, "round2"),
                 P, 10.8, row)
    assert d.kind == GO and d.reason == "inside"
    # without a probe under way a go that is not probed passes as before
    d, ps = probe(Decision(GO, "j", "clear"), own, ProbeState(), P, 12.0, row)
    assert d.kind == GO and d.reason == "clear"


def test_classify_response():
    s = [(t / 10, -1.0 if t >= 8 else 0.0) for t in range(30)]
    assert classify_response(s, P, 0.0) == "yield"
    assert classify_response([(t / 10, 0.0) for t in range(30)], P, 0.0) == "hold"
    assert classify_response([(t / 10, 0.0) for t in range(5)], P, 0.0) == "unknown"


def test_go_latch_for_launch_dead_time():
    own = car("A")
    prev = Decision(GO, "j", "first")
    d, ch = latch(Decision(YIELD, "j", "contested"), prev, 10.0, 10.0, own, False, P, 10.2)
    assert d.kind == GO and not ch  # latched
    d, ch = latch(Decision(YIELD, "j", "turn"), prev, 10.0, 10.0, own, False, P, 10.2)
    assert d.kind == YIELD  # the turn guard is never latched over
    d, ch = latch(Decision(YIELD, "j", "contested"), prev, 10.0, 10.0, own, False, P, 10.0 + own.cap.launch_dead + 0.01)
    assert d.kind == YIELD


def test_go_from_rest_latched_until_motion():
    """078 P2c: Autoware takes about 1.4 s from ACTIVATE at rest to motion. A go from rest is held over a wait (a
    re-read band) until the car moves or go_latch_motion_timeout passes; a yield (the turn guard, contested, the
    other clear first) still releases it, and a go given on the move is held only the launch dead time."""
    rest, rolling = car("A", s_entry=1.0, v=0.0), car("A", s_entry=1.0, v=0.6)
    prev = Decision(GO, "j", "rest_tie")
    t = 10.0 + rest.cap.launch_dead + 0.4  # past the launch latch, before Autoware's 1.4 s
    d, ch = latch(Decision(WAIT, "j", "ambiguous_band"), prev, 10.0, 10.0, rest, False, P, t, go_from_rest=True)
    assert d.kind == GO and not ch and d.latched_until == 10.0 + P.go_latch_motion_timeout
    d, _ = latch(Decision(WAIT, "j", "ambiguous_band"), prev, 10.0, 10.0, rest, False, P, t, go_from_rest=False)
    assert d.kind == WAIT  # a go given on the move: the launch latch only
    for r in ("turn", "contested", "clear"):
        d, _ = latch(Decision(YIELD, "j", r), prev, 10.0, 10.0, rest, False, P, t, go_from_rest=True)
        assert d.kind == YIELD, r
    d, _ = latch(Decision(WAIT, "j", "ambiguous_band"), prev, 10.0, 10.0, rolling, False, P, t, go_from_rest=True)
    assert d.kind == WAIT  # moving (> moved_speed): the latch is done (a row with commit: on_move commits instead)
    d, _ = latch(Decision(WAIT, "j", "ambiguous_band"), prev, 10.0, 10.0, rest, False, P,
                 10.0 + P.go_latch_motion_timeout + 0.01, go_from_rest=True)
    assert d.kind == WAIT  # still at rest after the timeout: released
    d, _ = latch(Decision(WAIT, "j", "ambiguous_band"), prev, 10.0, 10.0, rest, False,
                 P.override(source="test", go_latch_launch=False), t, go_from_rest=True)
    assert d.kind == WAIT  # the latches off (C8's reference)


def test_go_from_rest_latched_through_protocol():
    """The same through Protocol.step: a go from rest stays go while the car has not moved, though the band now
    reads the other as an ambiguous arrival."""
    z = Zone("j", "cross_unsignalled", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))
    pr = czp.Protocol()
    pr.mem[z.id] = Memory(decision=Decision(GO, z.id, "clear"), decision_since=0.0, go_since=0.0, go_from_rest=True,
                          own_rest_t=-3.0)
    b = car("B", s_entry=9.0, v=6.0)  # coming, would read in the band or ambiguous: a wait without the latch
    b.approach = "B"
    dec, _ = pr.step(1.2, car("A", s_entry=1.0, v=0.0), [b], z, C.Inputs(obs_age=0.2))
    assert dec.kind == GO
    pr2 = czp.Protocol(params=P.override(source="test", go_latch_motion_timeout=0.0))
    pr2.mem[z.id] = Memory(decision=Decision(GO, z.id, "clear"), decision_since=0.0, go_since=0.0, go_from_rest=True,
                           own_rest_t=-3.0)
    dec2, _ = pr2.step(1.2, car("A", s_entry=1.0, v=0.0), [b], z, C.Inputs(obs_age=0.2))
    assert dec2.kind != GO  # without it, the re-read flips the go before the car moves
    pr.mem[z.id] = Memory(decision=Decision(GO, z.id, "clear"), decision_since=0.0, go_since=0.0, go_from_rest=True,
                          go_moved=True, own_rest_t=-3.0)
    dec3, _ = pr.step(1.2, car("A", s_entry=1.0, v=0.0), [b], z, C.Inputs(obs_age=0.2))
    assert dec3.kind != GO  # it moved under this go and was brought to rest again (its safety layer): not latched


def test_commit_on_move():
    own = car("A", s_entry=0.8, v=0.5)
    d, _ = latch(Decision(YIELD, "j", "contested"), Decision(GO, "j", "first"), 0, 0, own, True, P, 5.0,
                 CONV["all_way_stop"])
    assert d.kind == GO and d.committed
    d, _ = latch(Decision(STOCK, "j", "contract"), Decision(GO, "j", "first"), 0, 0, own, True, P, 5.0,
                 CONV["all_way_stop"])
    assert d.kind == STOCK  # a failed contract is never committed over


def test_crossing_commits_a_go_from_rest_only():
    """078 P2b: the crossing's go commits once the car moves under a go it began at rest, not a go given on the move
    far from the zone (that one commits at its commit point only)."""
    assert CONV["cross_unsignalled"].get("commit") == "on_move"
    z = Zone("j", "cross_unsignalled", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))
    pr = czp.Protocol()
    pr.mem[z.id] = Memory(decision=Decision(GO, z.id, "rest_tie"), decision_since=0.0, go_since=0.0, go_from_rest=True,
                          own_moving_since=0.5, own_rest_t=-3.0)
    b = car("B", s_entry=4.0, v=4.0)  # it would be in first: no longer a go as read
    b.approach = "B"
    dec, _ = pr.step(1.0, car("A", s_entry=1.5, v=1.2), [b], z, C.Inputs(obs_age=0.2))
    assert dec.kind == GO and dec.committed
    pr.mem[z.id] = Memory(decision=Decision(GO, z.id, "rest_tie"), decision_since=0.0, go_since=0.0, go_from_rest=True,
                          go_moved=True, own_rest_t=-3.0)
    dec, _ = pr.step(1.0, car("A", s_entry=1.5, v=0.0), [b], z, C.Inputs(obs_age=0.2))
    assert dec.kind != GO  # brought to rest (its safety layer): the commit is released
    pr.mem[z.id] = Memory(decision=Decision(GO, z.id, "first"), decision_since=0.0, go_since=0.0, go_from_rest=False,
                          own_moving_since=-10.0)
    dec, _ = pr.step(1.0, car("A", s_entry=30.0, v=8.0), [car("B", s_entry=24.0, v=8.0)], z, C.Inputs(obs_age=0.2))
    assert not (dec.kind == GO and dec.committed)


# --- contracts, record, the cycle ----------------------------------------------------------------------
def test_latency_contract_falls_back_to_stock():
    pr = czp.Protocol()
    z = four_way(2)
    dec, rec = pr.step(0.0, car("A", s_entry=20, v=6), [car("B", s_entry=20, v=6)], z, C.Inputs(obs_age=0.6))
    assert dec.kind == STOCK and rec is not None and rec.contracts["latency"]["ok"] is False


def test_record_json_line():
    pr = czp.Protocol()
    z = four_way(2)
    dec, rec = pr.step(0.0, car("A", s_entry=20, v=6), [car("B", s_entry=25, v=6)], z, C.Inputs(obs_age=0.2))
    line = json.loads(to_json(rec))
    assert line["decision"] == dec.kind and line["conflict_set"] == ["B"] and line["row"] == "all_way_stop"
    dec2, rec2 = pr.step(0.1, car("A", s_entry=19.4, v=6), [car("B", s_entry=24.4, v=6)], z, C.Inputs(obs_age=0.2))
    assert rec2 is None  # one line per decision change


def test_protocol_two_cars_tie_sequence():
    """The whole cycle, two cars stopping 0.05 s apart: B goes first, A after B has cleared."""
    z = four_way(2)
    prs = {"A": czp.Protocol(), "B": czp.Protocol()}
    st = {"A": [12.0, 6.0], "B": [12.3, 6.0]}  # s_entry, v
    went = []
    t = 0.0
    while t < 30 and len(went) < 2:
        for me, ot in (("A", "B"), ("B", "A")):
            s, v = st[me]
            os_, ov = st[ot]
            own = Agent(me, approach=me, s_entry=s, s_exit=s + 8.29, v=v, stamp=t, cap=CAR)
            oth = Agent(ot, approach=ot, s_entry=os_, s_exit=os_ + 8.29, v=ov, stamp=t, a=-2.0 if ov > 0 and os_ > 1 else 0,
                        cap=CAR)
            dec, _ = prs[me].step(t, own, [oth], z, C.Inputs(obs_age=0.0))
            if dec.kind == GO and me not in went and v == 0.0:
                went.append(me)
            if dec.kind in (GO, CREEP) and (dec.kind == GO or dec.reason == "probe:creep"):
                st[me][1] = 6.0 if dec.kind == GO else 1.0
            elif s > 1.0:
                st[me][1] = max(0.0, min(v, math.sqrt(2 * 2.0 * max(s - 1.0, 0.0))))
            else:
                st[me][1] = 0.0
        for k in st:
            st[k][0] -= st[k][1] * 0.1
            if abs(st[k][0] - 1.0) < 0.31 and st[k][1] < 6.0 and st[k][0] > 0.5:
                pass
        t += 0.1
    assert went and went[0] == "B"


def test_entry_band_bias_correction():
    """078 P2b: the entry band reads the other's arrival bias-corrected: bias is reading - truth, so with Autoware's
    -0.24 s (the track predicts the other early) its arrival is read 0.24 s later, the own slack 0.24 s larger."""
    own, o = car("A", s_entry=20.0, v=6.0), car("B", s_entry=21.0, v=6.0)
    oi, ti = own_interval(own), other_interval(o, P)
    p0 = P.override(source="test", arrival_bias=0.0)
    b0 = band(oi, ti, o, p0, mode="entry")
    b1 = band(oi, ti, o, P.override(source="test", arrival_bias=-0.24), mode="entry")
    assert abs((b1.slack_own - b0.slack_own) - 0.24) < 1e-9 and abs((b0.slack_other - b1.slack_other) - 0.24) < 1e-9


def test_crossing_commit_released_by_the_turn_guard():
    """078 P2b: the crossing's committed go gives way to the other's seen start while the car can still stop, and holds
    over it once past its commit point."""
    row = CONV["cross_unsignalled"]
    assert row.get("commit_release") == "turn"
    prev = Decision(GO, "j", "rest_tie")
    slow = car("A", s_entry=4.0, v=1.0)
    d, _ = latch(Decision(YIELD, "j", "turn"), prev, 0.0, 0.0, slow, True, P, 2.0, row)
    assert d.kind == YIELD
    d, _ = latch(Decision(WAIT, "j", "ambiguous_band"), prev, 0.0, 0.0, slow, True, P, 2.0, row)
    assert d.kind == GO and d.committed  # a re-read band is not a start: the commit holds
    fast = car("A", s_entry=3.0, v=6.0)  # 6 m/s 3 m out: past its comfortable stop
    d, _ = latch(Decision(YIELD, "j", "turn"), prev, 0.0, 0.0, fast, True, P, 2.0, row)
    assert d.kind == GO and d.committed


# --- 078 P2e: a go against a car at rest waits one guard from the later of the two rests --------------------------
def _cross():
    return Zone("x", "cross_unsignalled", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))


def _verdict(own, o, own_rest_t, other_rest_t, t, p=P):
    from czp.convention import PairView, pair_verdict
    from czp.memory import AgentMemory
    am = AgentMemory(rest_t=other_rest_t, rest_s=o.s_entry if other_rest_t is not None else None,
                     stopping=Debounce(state=other_rest_t is not None))
    return pair_verdict(CONV["cross_unsignalled"], _cross(), own, own_interval(own, p), PairView(o, other_interval(o, p), am),
                        Memory(own_rest_t=own_rest_t), p, t)


def test_rest_tie_guard_counts_from_the_later_rest():
    """078 P2e: rest_tie's winner waits the guard from the later of the two rests, not from the other's alone. P2d: A
    rested 1.5 s before B; B's go at A's rest + 3.0 s met A's restart on a 0.5 s patience after the mutual rest."""
    z = _cross()
    ags = [car("A"), car("B")]  # B comes from A's right: B wins the tie
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    late = 0.6  # B rests 0.6 s after A: within stopped_first's margin (T + m), so the tie's role binds
    rests = {"A": 0.0, "B": late}
    d = decide_all(CONV, z, ags, rests, P_EXACT, t=guard + 0.1, quiet_since=-math.inf)  # the other's rest + guard
    assert d["B"].kind != GO and d["B"].reason == "rest_tie_wait", d["B"]
    d = decide_all(CONV, z, ags, rests, P_EXACT, t=late + guard - 0.1, quiet_since=-math.inf)
    assert d["B"].kind != GO and d["B"].reason == "rest_tie_wait"
    d = decide_all(CONV, z, ags, rests, P_EXACT, t=late + guard + 0.1, quiet_since=-math.inf)
    assert d["B"].kind == GO and d["B"].reason == "rest_tie" and d["A"].kind != GO
    # the other rested later: the guard runs from its rest, as before
    d = decide_all(CONV, z, ags, {"A": late, "B": 0.0}, P_EXACT, t=late + guard - 0.1, quiet_since=-math.inf)
    assert d["B"].kind != GO and d["B"].reason == "rest_tie_wait"


def test_rest_first_keeps_the_turn():
    """078 P2e: under rest_tie, a car that came to rest clearly first (stopped_first reads the own car SECOND, beyond
    T + m) keeps the turn: the right-hand car yields to it, in round two as well. One way only: the reading makes a car
    yield, never go; the first car still yields by the tie's role, so both wait until the last resort (map order)."""
    z = _cross()
    ags = [car("A"), car("B")]
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    rests = {"A": 0.0, "B": 1.5}
    for t in (1.5 + guard + 0.1, 1.5 + guard + P.second_round_H2 + 0.1):
        d = decide_all(CONV, z, ags, rests, P_EXACT, t=t, quiet_since=-math.inf)
        assert d["B"].kind != GO and d["B"].reason == "rest_first", (t, d["B"])
        assert d["A"].kind != GO
    h3 = P.second_round_H2 + P.tie_threshold_T + P.tie_margin_m + P.role_time_error_e + P.last_resort_extra
    d = decide_all(CONV, z, ags, rests, P_EXACT, t=1.5 + guard + h3 + 0.2, quiet_since=-math.inf)
    assert d["A"].kind == GO and d["A"].reason == "last_resort" and d["B"].kind != GO
    off = dict(CONV, cross_unsignalled=dict(CONV["cross_unsignalled"], rest_first=False))
    d = decide_all(off, z, ags, rests, P_EXACT, t=1.5 + guard + 0.1, quiet_since=-math.inf)
    assert d["B"].kind == GO and d["B"].reason == "rest_tie"


def test_clear_go_from_rest_waits_the_mutual_rest_guard():
    """078 P2e: a band clear go made from rest against a car at rest at its line waits the guard from the later rest (P2d:
    clear gos 0.6-1.7 s after the other's rest, the car crawling through as the other restarted on its patience)."""
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    own, o = car("B", s_entry=0.2), car("A", s_entry=1.9)
    # A rested 0.5 s before B: within stopped_first's margin, so not rest_first (exact stamps)
    v = _verdict(own, o, 6.0, 5.5, 6.0 + guard - 0.1, P_EXACT)
    assert v.band.kind == CLEAR and v.band.who == "B"
    assert v.kind == WAIT and v.reason == "fresh_clear" and not v.hard
    v = _verdict(own, o, 6.0, 5.5, 6.0 + guard + 0.1, P_EXACT)
    assert v.kind == GO and v.reason == "clear"
    # A rested clearly first: it keeps the turn, though B, at rest at the zone's edge, reads itself clear (078 P2e arena)
    v = _verdict(car("B", s_entry=0.01), o, 8.15, 6.4, 11.2, P_EXACT)
    assert v.band.kind == CLEAR and v.band.who == "B"
    assert v.kind == YIELD and v.reason == "rest_first" and not v.hard
    # still rolling below moved_speed, not yet at rest: its rest is still to come, so it waits
    v = _verdict(car("B", s_entry=0.3, v=0.3), o, None, 5.0, 20.0)
    assert v.kind == WAIT and v.reason == "fresh_clear"


def test_clear_go_at_speed_is_not_held():
    """078 P2e: a clear go while the own car moves at or above moved_speed is not a go from rest and is not held: holding
    it made the car stop and then meet a patient rule's restart (P2d exp2: 0 to 9 and 10 contacts against t1.5p5 and
    tinfp5). The other car not at rest: no guard either."""
    o = car("A", s_entry=1.0)
    v = _verdict(car("B", s_entry=1.5, v=4.0), o, None, 5.0, 5.5)
    assert v.kind == GO and v.reason == "clear"
    far = car("A", s_entry=40.0, v=6.0)
    v = _verdict(car("B", s_entry=0.2), far, 6.0, None, 6.1)
    assert v.kind == GO and v.reason == "clear"


def test_tie_needs_the_own_car_at_rest():
    """078 P2e: a tie go against a car at rest needs the own car at rest (then the guard from the later rest) or past its
    commit point. P2d: tie:right_before_left 0.7-0.8 s after the other's rest, the own car still rolling."""
    o = car("A", s_entry=1.0)
    v = _verdict(car("B", s_entry=12.0, v=6.0), o, None, 5.0, 9.5)
    assert v.band.kind == BAND and v.role is not None and v.role.reading == TIE
    assert v.kind == WAIT and v.reason == "fresh_tie" and not v.hard
    # at rest itself, B goes by rest_tie once the guard from the later rest has passed
    v = _verdict(car("B", s_entry=0.5), o, 5.0, 5.0, 5.0 + P.stop_dwell + CAR.launch_dead + P.see_delay + 0.1)
    assert v.kind == GO and v.reason == "rest_tie"


def test_mutual_rest_wait_exemptions():
    """078 P2e: past its commit point the own car goes (it can no longer stop comfortably); the guard is off with
    fresh_rest_guard, or when the other is not at rest at its line (the all-way stop row keeps its own 074 C8 rules:
    test_four_way_two_cars_tie_right_before_left)."""
    from czp.convention import mutual_rest_wait
    from czp.memory import AgentMemory
    o = car("A", s_entry=1.0)
    am = AgentMemory(rest_t=5.0, rest_s=1.0)
    passed = car("B", s_entry=1.0, v=5.0)  # needs 12.5 m/s^2 to stop: past it
    assert mutual_rest_wait(passed, o, am, Memory(), P, 5.1, "fresh_tie", need_rest=True) is None
    rolling = car("B", s_entry=12.0, v=6.0)
    assert mutual_rest_wait(rolling, o, am, Memory(), P, 5.1, "fresh_tie", need_rest=True).kind == WAIT
    assert mutual_rest_wait(rolling, o, am, Memory(), P, 5.1, "fresh_clear") is None
    off = P.override(source="test", fresh_rest_guard=False)
    assert mutual_rest_wait(car("B", s_entry=0.2), o, am, Memory(own_rest_t=6.0), off, 6.5, "fresh_clear") is None
    moving = car("A", s_entry=1.0, v=4.0)
    assert mutual_rest_wait(car("B", s_entry=0.2), moving, AgentMemory(), Memory(own_rest_t=6.0), P, 6.5, "x") is None
    far_still = car("A", s_entry=P.rest_window + 5.0)
    assert mutual_rest_wait(car("B", s_entry=0.2), far_still, AgentMemory(still_since=1.0), Memory(own_rest_t=6.0), P, 9.5,
                            "x") is None


def test_other_at_the_zone_edge_counts_as_at_rest():
    """078 P2e: a car stopped right at the zone's edge (a never-yield car held there by its safety layer) gets no rest stamp
    while its noisy position flickers across the edge; a guard reads the stamp since which it has been still, or now while
    it is still slowing there. A go against it waits the guard from the later rest all the same."""
    from czp.convention import mutual_rest_wait, other_rest_stamp
    from czp.memory import AgentMemory
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    edge = car("A", s_entry=0.1, v=0.05)
    assert other_rest_stamp(edge, AgentMemory(rest_t=7.0), P, 9.0) == 7.0
    assert other_rest_stamp(edge, AgentMemory(still_since=7.4), P, 9.0) == 7.4
    assert other_rest_stamp(edge, AgentMemory(), P, 9.0) == 9.0
    assert other_rest_stamp(car("A", s_entry=0.1, v=1.0), AgentMemory(), P, 9.0) is None
    own = car("B", s_entry=1.0)
    am = AgentMemory(still_since=7.7)
    w = mutual_rest_wait(own, edge, am, Memory(own_rest_t=7.6), P, 7.7 + guard - 0.1, "fresh_tie", need_rest=True)
    assert w is not None and w.kind == WAIT
    assert mutual_rest_wait(own, edge, am, Memory(own_rest_t=7.6), P, 7.7 + guard + 0.1, "fresh_tie", need_rest=True) is None


def test_protocol_go_waits_for_the_later_rest():
    """078 P2e, through czp.Protocol: B (A's right) rolls up below moved_speed and comes to rest 1.5 s after A; its go comes
    the guard after its own rest (not after A's), and a restart of A within that guard (a 0.5 s patience after the mutual rest) turns it into a yield."""
    z = _cross()
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    conv = dict(CONV, cross_unsignalled=dict(CONV["cross_unsignalled"], rest_first=False))  # the guard alone
    pr = czp.Protocol(conv, P_EXACT)
    first_go = None
    for k in range(0, 120):
        t = k * 0.1
        a = car("A", s_entry=1.0, stamp=t)
        b = car("B", s_entry=1.0 if t >= 1.5 else 1.0 + (1.5 - t) * 0.3, v=0.0 if t >= 1.5 else 0.3, stamp=t)
        d, _ = pr.step(t, b, [a], z, inputs=C.Inputs(obs_age=0.0), obs_age=0.0)
        if d.kind == GO and first_go is None:
            first_go = t
    assert first_go is not None and first_go >= 1.5 + guard - 1e-6, first_go
    # A restarts 1.2 s after the mutual rest (seen moving): B never goes before A
    pr = czp.Protocol(conv, P_EXACT)
    for k in range(0, 60):
        t = k * 0.1
        moving = t >= 1.5 + 1.2
        s_a = 1.0 - (0.5 * 3.0 * (t - 2.7) ** 2 if moving else 0.0)
        a = car("A", s_entry=s_a, v=3.0 * (t - 2.7) if moving else 0.0, stamp=t)
        b = car("B", s_entry=1.0 if t >= 1.5 else 1.0 + (1.5 - t) * 0.3, v=0.0 if t >= 1.5 else 0.3, stamp=t)
        d, _ = pr.step(t, b, [a], z, inputs=C.Inputs(obs_age=0.0), obs_age=0.0)
        assert d.kind != GO, (t, d)


def test_the_self_contract_reads_the_decision_cycles_median():
    """078 P2g: the self contract's reading is the median of the last decision cycles, so one slow cycle (or the
    node's own I/O, which is not fed to it) does not flip the car to stock."""
    from czp import contracts as C
    from czp.config import load_contracts
    w = C.SelfWindow(10)
    assert w.median() == 0.0
    for x in [0.004] * 9 + [0.030]:  # one slow decision among ten
        w.add(x)
    assert w.median() == 0.004
    ok = {s.name: s.ok for s in C.check(load_contracts(), C.Inputs(cycle_time=w.median()))}
    assert ok["self"]
    for x in [0.030] * 6:  # a sustained overrun still fails the contract
        w.add(x)
    assert w.median() == 0.030 and len(w.times) == 10
    assert not {s.name: s.ok for s in C.check(load_contracts(), C.Inputs(cycle_time=w.median()))}["self"]


def test_the_node_feeds_the_self_contract_the_decision_time_only():
    """078 P2g: node.tick passes the window's median (Strategy.last_cycle, the decision) and never its whole tick
    (RTC calls, records, the log file: 078 P2f's 162 flips at a 0.018-0.026 s median)."""
    import inspect
    import pytest
    pytest.importorskip("rclpy")
    from czp import node
    src = inspect.getsource(node.InteractionStrategy.tick)
    assert "self.self_window.median()" in src and "self.self_window.add(self.strategy.last_cycle)" in src
    assert "self.cycle_times[-SELF_WINDOW:]" not in src
