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
    T, m = P.tie_threshold_T, P.tie_margin_m
    o = car("B")
    assert stopped_first(10.0, 10.0 + T + m + 0.01, 20, 0.7, o, P).reading == FIRST
    assert stopped_first(10.0, 10.0 - T - m - 0.01, 20, 0.7, o, P).reading == SECOND
    assert stopped_first(10.0, 10.0 + 0.1, 20, 0.7, o, P).reading == TIE
    assert stopped_first(10.0, 10.0 + T, 20, 0.7, o, P).reading == AMBIG  # at the threshold: ambiguous
    # no stamp yet: first only once one within T + m would be known
    assert stopped_first(10.0, None, 10.0 + 0.7 + T + m + 0.01, 0.7, o, P).reading == FIRST
    assert stopped_first(10.0, None, 10.5, 0.7, o, P).reading != FIRST


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
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.05}, P, t=3.0, quiet_since=-math.inf)
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
    d = decide_all(CONV, z, ags, rests, P, t=1.5, quiet_since=-math.inf)
    assert all(x.kind != GO for x in d.values())
    fresh = (CAR.launch_dead + P.see_delay) if P.fresh_rest_guard else 0.0
    d = decide_all(CONV, z, ags, rests, P, t=P.stop_dwell + fresh + P.second_round_H2 + 0.1, quiet_since=-math.inf)
    goes = [k for k, x in d.items() if x.kind == GO]
    assert goes == ["A"] and d["A"].reason == "cycle"


def test_rest_tie_binds_after_a_mutual_stop():
    """078 V4: at an unsignalled crossing both cars at rest at their lines read a tie the same way: the car on
    the right (B) goes, after the fresh-rest guard's wait; A yields; neither reads the noisy band."""
    z = Zone("x", "cross_unsignalled", (Approach("A", heading=0.0, order=0), Approach("B", heading=math.pi / 2, order=1)))
    ags = [car("A"), car("B")]
    guard = P.stop_dwell + CAR.launch_dead + P.see_delay
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.0}, P, t=guard - 0.1, quiet_since=-math.inf)
    assert d["B"].kind != GO and d["A"].kind != GO
    d = decide_all(CONV, z, ags, {"A": 0.0, "B": 0.0}, P, t=guard + 0.1, quiet_since=-math.inf)
    assert d["B"].kind == GO and d["B"].reason == "rest_tie" and d["A"].kind != GO


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


def test_commit_on_move():
    own = car("A", s_entry=0.8, v=0.5)
    d, _ = latch(Decision(YIELD, "j", "contested"), Decision(GO, "j", "first"), 0, 0, own, True, P, 5.0,
                 CONV["all_way_stop"])
    assert d.kind == GO and d.committed


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
