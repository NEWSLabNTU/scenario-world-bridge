"""Step 6, the convention: the jurisdiction's table, row by the zone's relation, then role.

Per agent in the conflict set a pair verdict is formed; the decision is the most restrictive.
Verdicts are hard (a guard: the other in the zone or moved off its line, contested, clear for the
other, the exit box blocked) or soft (the role: second, the loser of a tie, ambiguous, pending).
Soft waits are what the later rounds resolve, never hard ones:

- round 2 (``wait_second_round``): once the junction has been quiet for H2 (no car seen starting,
  entering or leaving the zone; a blocked exit's release counts) and every soft agent is at rest, the
  row's tie rule decides (right-before-left: go when no soft agent is on this car's right);
- a wait-for cycle over the conflict set (four cars: right-before-left is a cycle) is broken by the
  map's total order: the cycle's lowest ``order`` goes;
- the last resort: after H3 = H2 + T + m + e + extra, the map's order alone (074 C8 (e)).

A round-2, cycle or last-resort go is marked ``probe`` so that the probe step puts the legible creep
before it (074 C8 fix (c)). ``go`` always needs the zone's exit box clear.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .band import band as band_of
from .intervals import ZoneInterval, own_interval, other_interval
from .memory import AgentMemory, Memory
from .model import (AMBIG, AMBIGUOUS, BAND, CLEAR, CONTESTED, FIRST, GO, SECOND, STOCK,
                    STOPPED_FIRST, TIE, WAIT, WAIT_SECOND_ROUND, YIELD, Agent, Band, Decision, Role, Zone)
from .role import map_order, obstruction_side, right_of, role as role_of


@dataclass(frozen=True)
class Verdict:
    other: str
    kind: str  # GO, YIELD, WAIT
    hard: bool
    reason: str
    band: Optional[Band] = None
    role: Optional[Role] = None


@dataclass(frozen=True)
class PairView:
    """What the own car knows of one agent of the conflict set this cycle."""
    other: Agent
    iv: ZoneInterval
    mem: AgentMemory
    band: Optional[Band] = None


def tie_rule(rule, zone: Zone, own: Agent, other: Agent):
    """The row's tie-break between two agents: True when own goes. Right-before-left: the car on the
    other's right goes; between approaches neither of which is on the other's right (opposite ones) it
    is undefined, and the map's order decides."""
    if rule == "right_before_left":
        if right_of(zone, other.approach, own.approach):
            return True  # own comes from the other's right
        if right_of(zone, own.approach, other.approach):
            return False
        return map_order(zone, own, other).reading == FIRST
    if rule in ("map_order", "zipper"):
        return map_order(zone, own, other).reading == FIRST
    if rule == "obstruction_side":  # the free side first, then the map's order (078 V4: the narrow passage's
        r = obstruction_side(zone, own, other)  # round two follows its first fact, else it undid the rest tie)
        return r.reading == FIRST if r.reading != TIE else map_order(zone, own, other).reading == FIRST
    raise ValueError(f"unknown tie rule {rule}")


def far_test(other: Agent, params):
    return other.s_entry > params.far_distance or (
        other.v > params.moved_speed and other.s_entry / max(other.v, 0.1) > params.far_time and other.a > -params.yield_decel)


def pair_verdict(row, zone: Zone, own: Agent, own_iv: ZoneInterval, pv: PairView, mem: Memory, params, t,
                 obs_age=0.0) -> Verdict:
    o, am = pv.other, pv.mem
    stop_law = bool(row.get("stop"))
    if o.cleared or am.cleared:
        return Verdict(o.id, GO, False, "cleared")
    if am.in_zone.state or am.moved.state or o.s_entry < -1.0:
        return Verdict(o.id, YIELD, True, "turn")  # in the zone, or seen moving off its line: it has the turn
    b = pv.band or band_of(own_iv, pv.iv, o, params, am.stopping.state, stop_law, am.rest_t is not None,
                           mode=row.get("band", "interval"), own=own)
    if (not stop_law and row.get("rest_tie", False) and mem.own_rest_t is not None and own.v < params.rest_speed
            and am.rest_t is not None and b.kind in (BAND, AMBIGUOUS)):
        # 078 V4: both at rest at their lines is a tie, read the same by both (the rest is filtered memory): the
        # row's role binds after the mutual stop (074 C7) and the priority car goes at once. Read from the
        # noisy band instead, it flickered between ambiguous (wait) and band (the tie's go), and the late go
        # raced an impatient car's restart (the arena's czp-in-B contacts at the crossing)
        b = Band(BAND, None, b.slack_own, b.slack_other)
        v = pair_verdict_role(row, zone, own, own_iv, pv, mem, params, t, obs_age, b, first_override=None)
        if v.kind != GO:
            return v
        # the winner waits as the fresh-rest guard does (dwell + the other's launch + see_delay): a car that
        # ignores the role and goes on a short patience (S2's 2 s) is seen starting first; then the probe
        if params.fresh_rest_guard and t - am.rest_t < params.stop_dwell + o.cap.launch_dead + params.see_delay:
            return Verdict(o.id, WAIT, False, "rest_tie_wait", b, v.role)
        return Verdict(v.other, v.kind, v.hard, "rest_tie", v.band, v.role)
    return pair_verdict_role(row, zone, own, own_iv, pv, mem, params, t, obs_age, b)


def pair_verdict_role(row, zone, own, own_iv, pv, mem, params, t, obs_age, b, first_override=None):
    """pair_verdict past the guards and the band: contested, clear, then the row's role."""
    o, am = pv.other, pv.mem
    if b.kind == CONTESTED:
        return Verdict(o.id, YIELD, True, "contested", b)
    v = _role_verdict(row, zone, own, own_iv, pv, mem, params, t, obs_age, b)
    if v.kind == GO and row.get("claim"):
        # 078 V4, the claim (design step 7): a go by arrival order or by the role, not out before the other could be
        # in (the interval test), needs the other to have yielded: at rest at its line for the fresh-rest guard's
        # time (dwell + its launch + see_delay), so that a restart of a car that stopped only for its safety layer,
        # or that goes on a short patience, is seen first. A car that never yields (S4's go) met this one head-on
        # in the long passage, and its safety layer's braking, read through noise, had passed for a yield
        ib = band_of(own_iv, pv.iv, o, params, am.stopping.state, False, False, mode="interval")
        if not (ib.kind == CLEAR and ib.who == own.id):
            guard = params.stop_dwell + o.cap.launch_dead + params.see_delay
            since = am.rest_t if am.rest_t is not None else am.still_since  # at its line, or at rest in a queue
            if since is None or t - since < guard:
                return Verdict(o.id, WAIT, False, "claim", b, v.role)
    return v


def _role_verdict(row, zone, own, own_iv, pv, mem, params, t, obs_age, b):
    o, am = pv.other, pv.mem
    if b.kind == CLEAR:
        if b.who == own.id:
            return Verdict(o.id, GO, False, "clear", b)
        return Verdict(o.id, YIELD, True, "clear", b)
    first = row.get("first", STOPPED_FIRST)
    if first == STOPPED_FIRST:
        r = role_of(STOPPED_FIRST, zone, own, o, params, own_rest_t=mem.own_rest_t, other_rest_t=am.rest_t, t=t,
                    obs_age=obs_age, far=far_test(o, params))
    elif first in ("map_priority", "map_order", "obstruction_side"):
        r = role_of(first, zone, own, o, params)
        if r.reading == TIE and b.kind == AMBIGUOUS:
            r = Role(r.fact, AMBIG, r.value)  # a map tie at the band's edge: the band itself is unsure
    elif first == "ahead_at_end":
        r = role_of(first, zone, own, o, params, own_t_end=own_iv.t_in[1], other_t_end=pv.iv.nominal_in)
    else:
        r = role_of("map_priority", zone, own, o, params)
    mode = row.get("band", "interval")
    if r.reading == FIRST and b.kind == AMBIGUOUS and mode in ("entry", "gap") and first != STOPPED_FIRST:
        # 078 V4: at the band's edge the other may read itself clear (its own reading of its lead is within 2e of
        # this car's): this car's role-first go would meet its go. Go only when the other's lead, as read here,
        # rules that out (< M - e); else wait (the other yields by its role or goes on its clear, and a mutual
        # wait is the second round's)
        m = params.band_margin_entry
        if b.slack_other >= m - params.role_time_error_e - 1e-6:
            return Verdict(o.id, WAIT, False, "ambiguous_band", b, r)
    if r.reading == FIRST:
        # 078 V2b: a car that has only just come to rest may be one that ignores arrival order (it starts after
        # its own stop); wait until its start would have been seen (dwell + its launch dead time + see_delay)
        if first == STOPPED_FIRST and params.fresh_rest_guard and am.rest_t is not None and \
                t - am.rest_t < params.stop_dwell + o.cap.launch_dead + params.see_delay:
            return Verdict(o.id, WAIT, False, "fresh", b, r)
        if first == STOPPED_FIRST and am.rest_t is None and o.v > params.moved_speed and params.probe_first_moving:
            # first, but a car is still coming that has not stopped: stock reads this car at rest as static, so
            # the go is made legible first (the probe), as before a round-2 go (078 V2b)
            return Verdict(o.id, GO, False, "first_probe", b, r)
        return Verdict(o.id, GO, False, "first", b, r)
    if r.reading == SECOND:
        return Verdict(o.id, YIELD, False, "second", b, r)
    if r.reading == TIE:
        tr = row.get("tie", "map_order")
        if tie_rule(tr, zone, own, o):
            return Verdict(o.id, GO, False, f"tie:{tr}", b, r)
        return Verdict(o.id, YIELD, False, f"tie:{tr}", b, r)
    return Verdict(o.id, WAIT, False, "ambiguous" if r.reading == AMBIG else "pending", b, r)


def soft_waits(zone, row, agents, rests, params):
    """The wait-for edges among the conflict set's agents at rest (i -> j: i's rule makes it wait for j),
    from the rest stamps as this car reads them, by the row's first fact and then its tie rule."""
    edges = set()
    ids = [a.id for a in agents]
    for i in agents:
        for j in agents:
            if i.id == j.id:
                continue
            ri, rj = rests.get(i.id), rests.get(j.id)
            if ri is None or rj is None:
                continue
            T, m = params.tie_threshold_T, params.tie_margin_m
            d = rj - ri
            if d < -(T + m):
                edges.add((i.id, j.id))  # j stopped first
            elif d > T + m:
                continue
            else:  # a tie or ambiguous: the second round's tie rule
                if not tie_rule(row.get("tie", "map_order"), zone, i, j):
                    edges.add((i.id, j.id))
    return edges, ids


def find_cycle(edges, nodes):
    """One wait-for cycle (a list of nodes) or None."""
    adj = {n: [b for a, b in edges if a == n] for n in nodes}
    color = {n: 0 for n in nodes}
    stack = []

    def dfs(n):
        color[n] = 1
        stack.append(n)
        for m in adj.get(n, ()):
            if color.get(m, 2) == 1:
                return stack[stack.index(m):]
            if color.get(m) == 0:
                c = dfs(m)
                if c:
                    return c
        stack.pop()
        color[n] = 2
        return None

    for n in nodes:
        if color[n] == 0:
            c = dfs(n)
            if c:
                return c
    return None


def on_cycle(edges, nodes):
    """The nodes that lie on some wait-for cycle."""
    adj = {n: {b for a, b in edges if a == n} for n in nodes}
    out = set()
    for n in nodes:
        seen, stack = set(), list(adj[n])
        while stack:
            m = stack.pop()
            if m == n:
                out.add(n)
                break
            if m in seen:
                continue
            seen.add(m)
            stack.extend(adj.get(m, ()))
    return out


def cycle_release(edges, nodes, order):
    """The one agent the map's order releases: the lowest-order node on any wait-for cycle (or None)."""
    c = on_cycle(edges, nodes)
    return min(c, key=lambda n: order[n]) if c else None


def break_cycles(edges, nodes, order):
    """Remove, for every wait-for cycle, the out-edges of its lowest node in the map's order: that agent
    goes. Returns (acyclic edges, the agents released)."""
    edges, released = set(edges), []
    while True:
        c = find_cycle(edges, nodes)
        if not c:
            return edges, released
        low = min(c, key=lambda n: order[n])
        released.append(low)
        edges = {(a, b) for a, b in edges if a != low}


def decide(conv, zone: Zone, own: Agent, own_iv: ZoneInterval, pairs, mem: Memory, params, t, obs_age=0.0,
           exit_clear=True):
    """Step 6: the own car's decision at the zone (before the probe and the latch)."""
    row_name = zone.kind
    row = conv.get(row_name) or {}
    if not pairs:
        return Decision(STOCK, zone.id, "no conflict", row_name), []
    if own.cleared or own.in_zone:
        return Decision(GO, zone.id, "inside", row_name, committed=True), []
    if row.get("stop") and own.s_entry > 0 and (mem.own_rest_t is None or t - mem.own_rest_t < params.stop_dwell):
        return Decision(WAIT, zone.id, "stop", row_name), []
    # 078 V4: a car queued behind another of the conflict set on the same approach cannot reach the zone before
    # that one (one lane): while its leader has not left, it is shadowed -- the leader's verdict stands for both
    # (a queued car never stood at its line, so the claim's rest guard and the rounds would wait for it forever)
    def shadowed(pv):
        o = pv.other
        return any(q.other.approach == o.approach and q.other.id != o.id and q.other.s_entry < o.s_entry - 1.0
                   and not (q.other.cleared or q.mem.cleared) for q in pairs)
    verdicts = [Verdict(pv.other.id, GO, False, "shadowed") if shadowed(pv) else
                pair_verdict(row, zone, own, own_iv, pv, mem, params, t, obs_age) for pv in pairs]
    hard = [v for v in verdicts if v.kind != GO and v.hard]
    soft = [v for v in verdicts if v.kind != GO and not v.hard]
    if hard:
        return Decision(YIELD, zone.id, hard[0].reason, row_name, tuple(v.other for v in hard),
                        stock_equivalent=all(v.reason in ("clear", "contested") for v in hard)), verdicts
    if not soft:
        if not exit_clear:
            return Decision(WAIT, zone.id, "exit", row_name), verdicts
        reason = verdicts[0].reason if len(verdicts) == 1 else next(
            (r for r in ("first_probe", "rest_tie") if any(v.reason == r for v in verdicts)), "first")
        return Decision(GO, zone.id, reason,
                        row_name, stock_equivalent=all(v.reason in ("clear", "cleared") for v in verdicts)), verdicts
    # soft waits only: the second round, the wait-for cycle, the last resort. These rounds are decided on
    # map facts among every agent of the conflict set at rest (not only those this car waits for), from
    # the junction's common quiet epoch (the latest rest + dwell, or the last event), so that every car
    # reads the same round and the same winner (up to e).
    by_id = {pv.other.id: pv for pv in pairs}
    soft_ids = [v.other for v in soft]

    def resting(pv):  # at rest at its line and not (filtered) moved off it; one noisy speed sample is not a start
        return pv.mem.rest_t is not None and not pv.mem.moved.state

    kind = WAIT if any(v.kind == WAIT for v in soft) else YIELD
    if mem.own_rest_t is None or not all(resting(by_id[i]) for i in soft_ids):
        return Decision(kind, zone.id, soft[0].reason, row_name, tuple(soft_ids)), verdicts
    rest_pvs = [pv for pv in pairs if resting(pv)]
    # with the fresh-rest guard a first go may come up to see_delay after the later
    # car's dwell: the second round waits that much longer, so that go is seen first
    fresh_own = (own.cap.launch_dead + params.see_delay) if params.fresh_rest_guard else 0.0
    epoch = max([mem.own_rest_t + params.stop_dwell + fresh_own, mem.last_event] +
                [pv.mem.rest_t - params.role_time_bias + params.stop_dwell +
                 ((pv.other.cap.launch_dead + params.see_delay) if params.fresh_rest_guard else 0.0) for pv in rest_pvs])
    q = t - epoch
    if q >= params.second_round_H2:
        tr = row.get("tie", "map_order")
        others = [pv.other for pv in rest_pvs]
        agents = [own] + others
        if all(tie_rule(tr, zone, own, o) for o in others):
            return _go_probe(zone, row_name, "round2", exit_clear), verdicts
        h3 = params.second_round_H2 + params.tie_threshold_T + params.tie_margin_m + params.role_time_error_e + \
            params.last_resort_extra
        # the last resort: the round-2 winner has not gone by H3 (it would have been seen): the map's order
        if q >= h3 and all(map_order(zone, own, o).reading == FIRST for o in others):
            return _go_probe(zone, row_name, "last_resort", exit_clear), verdicts
        # another car beats every one by the tie rule: it goes, this car waits
        if any(all(tie_rule(tr, zone, x, y) for y in agents if y.id != x.id) for x in others):
            return Decision(WAIT_SECOND_ROUND, zone.id, soft[0].reason, row_name, tuple(soft_ids)), verdicts
        # no unique winner: a wait-for cycle among the agents at rest, broken by the map's order
        edges = {(i.id, j.id) for i in agents for j in agents if i.id != j.id and not tie_rule(tr, zone, i, j)}
        order = {x.id: zone.approach(x.approach).order for x in agents}
        if cycle_release(edges, [x.id for x in agents], order) == own.id:
            return _go_probe(zone, row_name, "cycle", exit_clear), verdicts
        return Decision(WAIT_SECOND_ROUND, zone.id, soft[0].reason, row_name, tuple(soft_ids)), verdicts
    return Decision(kind, zone.id, soft[0].reason, row_name, tuple(soft_ids)), verdicts


def _go_probe(zone, row_name, reason, exit_clear):
    if not exit_clear:
        return Decision(WAIT, zone.id, "exit", row_name)
    return Decision(GO, zone.id, reason, row_name)


def decide_all(conv, zone: Zone, agents, rests, params, t, quiet_since, perceive=None, exit_clear=True, moved=()):
    """Every agent's decision from its own reading of a shared world (for the model checks and the
    property tests). rests: agent id -> its true rest time (None: not at rest); quiet_since: when the
    junction's last event was; perceive(i, j, stamp) -> agent i's reading of j's rest stamp (the truth
    plus i's perception error; default: the truth); moved: the agents seen moving off their lines."""
    from .memory import AgentMemory, Debounce
    out = {}
    for me in agents:
        mem = Memory(own_rest_t=rests.get(me.id), last_event=quiet_since)
        pairs = []
        for o in agents:
            if o.id == me.id:
                continue
            r = rests.get(o.id)
            if r is not None and perceive is not None:
                r = perceive(me.id, o.id, r)
            am = AgentMemory(rest_t=r, rest_s=o.s_entry if r is not None else None, moved=Debounce(state=o.id in moved),
                             stopping=Debounce(state=r is not None))
            pairs.append(PairView(o, other_interval(o, params), am))
        d, _ = decide(conv, zone, me, own_interval(me, params), pairs, mem, params, t, 0.0, exit_clear)
        out[me.id] = d
    return out
