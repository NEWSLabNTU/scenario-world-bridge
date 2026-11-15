"""Property tests (design, "Tests"): for random conflict sets and map orders, the order is total; no
wait-for cycle survives; two agents never both decide go into one zone (readings within +-e);
ambiguous never yields go; go never without a clear exit box; outside a band the decision is stock's.

hypothesis is used when installed; otherwise the same properties run over seeded random cases."""

import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import czp  # noqa: E402
from czp.convention import break_cycles, decide, decide_all, find_cycle, soft_waits  # noqa: E402
from czp.convention import PairView  # noqa: E402
from czp.intervals import other_interval, own_interval  # noqa: E402
from czp.memory import AgentMemory, Debounce, Memory  # noqa: E402
from czp.model import AMBIG, GO, FIRST, SECOND, Agent, Approach, Zone  # noqa: E402
from czp.role import map_order  # noqa: E402

# the all-way stop's properties hold for m >= e (074 C8 V1): checked at C8's arena e (0.25 s, no bias), where that
# row was verified. On Autoware's tracks (e 0.55 s, 078 P2 EV4) stopped_first is unusable and the row is stock
P = czp.load_params().override(source="test: 074 C8 arena e", role_time_error_e=0.25, role_time_bias=0.0)
CONV = czp.load_conventions()
CAR = P.capability("car", v_max=8.0)
N_CASES = int(os.environ.get("CZP_PROPERTY_CASES", "400"))


def random_zone(rng, n):
    """n approaches at random distinct headings, with a random permutation as the map's order."""
    heads = sorted(rng.sample(range(0, 360, 15), n))
    order = list(range(n))
    rng.shuffle(order)
    return Zone("z", "all_way_stop", tuple(Approach(f"q{i}", heading=math.radians(heads[i]), order=order[i])
                                           for i in range(n)))


def agents_at_rest(z):
    return [Agent(a.id, approach=a.id, s_entry=1.0, s_exit=9.3, v=0.0, cap=CAR) for a in z.approaches]


def cases(seed=0):
    rng = random.Random(seed)
    for _ in range(N_CASES):
        yield rng


def test_map_order_is_total():
    for rng in cases(1):
        z = random_zone(rng, rng.randint(2, 6))
        ags = agents_at_rest(z)
        for a in ags:
            for b in ags:
                if a.id == b.id:
                    continue
                ra, rb = map_order(z, a, b).reading, map_order(z, b, a).reading
                assert {ra, rb} == {FIRST, SECOND}  # antisymmetric, never a tie
        # transitive: the "first" relation is the order of a list
        rank = sorted(ags, key=lambda a: z.approach(a.id).order)
        for i in range(len(rank)):
            for j in range(i + 1, len(rank)):
                assert map_order(z, rank[i], rank[j]).reading == FIRST


def test_no_wait_for_cycle_survives():
    for rng in cases(2):
        n = rng.randint(2, 7)
        nodes = [f"n{i}" for i in range(n)]
        edges = {(a, b) for a in nodes for b in nodes if a != b and rng.random() < 0.4}
        order = dict(zip(nodes, rng.sample(range(100), n)))
        left, _ = break_cycles(edges, nodes, order)
        assert find_cycle(left, nodes) is None
    # and on the convention's own wait-for edges among agents at rest
    for rng in cases(3):
        z = random_zone(rng, rng.randint(2, 4))
        ags = agents_at_rest(z)
        rests = {a.id: rng.choice([0.0, rng.uniform(-1.5, 1.5)]) for a in ags}
        edges, nodes = soft_waits(z, CONV["all_way_stop"], ags, rests, P)
        left, _ = break_cycles(edges, nodes, {a.id: z.approach(a.id).order for a in ags})
        assert find_cycle(left, nodes) is None


def _perceive(rng, e):
    errs = {}

    def f(i, j, stamp):
        k = (i, j)
        if k not in errs:
            errs[k] = rng.uniform(-e, e)
        return stamp + errs[k]
    return f


def test_never_two_go_into_one_zone():
    """Agents at rest at their lines, each reading the others' rest stamps within +-e, stepped in time: a
    car that decides go starts, and the others see it moving SEE s later (074 C8's model: visible motion
    0.7 s after the go, perception and decision 0.8 s); it is in the zone until it clears (CLEAR_T s after
    its go). Property: no car decides go while another's go is in progress (between its go and its
    clearing), over ties, ambiguous readings, the second round, the cycle and the last resort."""
    e = P.role_time_error_e
    SEE, CLEAR_T, DT = 1.5, 4.0, 0.05
    for rng in cases(4):
        z = random_zone(rng, rng.randint(2, 4))
        ags = agents_at_rest(z)
        base = rng.uniform(-0.5, 0.5)
        rests = {a.id: base + rng.choice([0.0, rng.uniform(-1.2, 1.2), rng.uniform(-0.3, 0.3)]) for a in ags}
        perceive = _perceive(rng, e)
        t = max(rests.values())
        went, last_event = {}, -math.inf
        while t < max(rests.values()) + 40 and len(went) < len(ags):
            live = [a for a in ags if not (a.id in went and t >= went[a.id] + CLEAR_T)]
            for k, tg in went.items():  # a start seen, or a clearing: junction events
                if abs(t - (tg + SEE)) < DT / 2 or abs(t - (tg + CLEAR_T)) < DT / 2:
                    last_event = t
            seen_moving = {k for k, tg in went.items() if t >= tg + SEE}
            waiting = [a for a in live if a.id not in went]
            d = decide_all(CONV, z, [a for a in live], {a.id: rests[a.id] for a in live}, P, t, last_event,
                           perceive=perceive, moved=seen_moving)
            for a in waiting:
                if d[a.id].kind in (GO, "stock"):  # stock: nobody left in the conflict set
                    busy = [k for k, tg in went.items() if tg <= t < tg + CLEAR_T]
                    assert not busy, (rests, round(t - max(rests.values()), 2), a.id, d[a.id].reason, busy)
                    went[a.id] = t
            t += DT
        assert len(went) == len(ags), ("not live", rests, went)


def test_ambiguous_never_go_and_exit_box():
    for rng in cases(5):
        z = random_zone(rng, 2)
        a, b = agents_at_rest(z)
        T, m = P.tie_threshold_T, P.tie_margin_m
        d = rng.uniform(T - m + 1e-3, T + m - 1e-3) * rng.choice([-1, 1])  # inside the margin: ambiguous
        mem = Memory(own_rest_t=10.0, last_event=-math.inf)
        am = AgentMemory(rest_t=10.0 + d, rest_s=1.0, stopping=Debounce(state=True))
        t = 10.0 + P.stop_dwell + rng.uniform(0.0, P.second_round_H2 - 0.05)  # before round 2
        dec, verdicts = decide(CONV, z, a, own_interval(a), [PairView(b, other_interval(b, P), am)], mem, P, t, 0.0)
        assert verdicts[0].role.reading == AMBIG and dec.kind != GO
        # go never without a clear exit box, at any time
        for tt in (t, t + 5, t + 20):
            dec, _ = decide(CONV, z, a, own_interval(a), [PairView(b, other_interval(b, P), am)], mem, P, tt, 0.0,
                            exit_clear=False)
            assert dec.kind != GO


def test_outside_band_is_stock():
    """A clear band (the other far, or already through) gives stock's decision (stock_equivalent)."""
    for rng in cases(6):
        z = Zone("x", "cross_unsignalled", (Approach("A"), Approach("B", heading=math.pi / 2, order=1)))
        v = rng.uniform(4, 12)
        cap = P.capability("car", v_max=v)
        a = Agent("A", approach="A", s_entry=rng.uniform(5, 20), s_exit=30, v=v, cap=cap)
        a.s_exit = a.s_entry + 8.3
        far = Agent("B", approach="B", s_entry=a.s_entry + v * rng.uniform(4, 10), v=v, cap=cap)
        far.s_exit = far.s_entry + 8.3
        dec, _ = decide(CONV, z, a, own_interval(a), [PairView(far, other_interval(far, P), AgentMemory())],
                        Memory(), P, 0.0)
        assert dec.kind == GO and dec.stock_equivalent
        dec, _ = decide(CONV, z, far, own_interval(far), [PairView(a, other_interval(a, P), AgentMemory())],
                        Memory(), P, 0.0)
        assert dec.stock_equivalent


try:  # the same properties under hypothesis, when it is installed
    from hypothesis import given, settings, strategies as hs

    @settings(max_examples=200, deadline=None)
    @given(hs.integers(0, 10 ** 6))
    def test_never_two_go_hypothesis(seed):
        rng = random.Random(seed)
        z = random_zone(rng, rng.randint(2, 4))
        ags = agents_at_rest(z)
        rests = {a.id: rng.uniform(-1.0, 1.0) for a in ags}
        for t in (1.5, 2.0, 3.6):  # before any last resort, a snapshot: at most one go
            d = decide_all(CONV, z, ags, rests, P, max(rests.values()) + t, -math.inf,
                           perceive=_perceive(rng, P.role_time_error_e))
            assert sum(x.kind == GO for x in d.values()) <= 1
except ImportError:
    pass
