"""What the protocol remembers between cycles, and the pure functions that advance it.

A cycle's functions read the memory and return a new one; nothing else persists. The memory holds
per agent the rest stamp (when it was first seen at rest at its line, the observation's own time),
the filtered guards (074 C8 fix (a): the "moved off its line" and "stopping for its line" tests are
debounced over ``guard_filter_n`` distinct observations), and the junction's last event (a car seen
starting, entering or leaving the zone, a blocked exit's release) from which the second round's
quiet time runs.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Optional

from .intervals import stop_need
from .model import INF, Agent, Decision


@dataclass(frozen=True)
class Debounce:
    """A boolean that changes only after n distinct samples (by stamp) in a row read the new value."""
    state: bool = False
    run: int = 0
    last_stamp: float = -INF
    flips: int = 0


def debounce(d: Debounce, raw: bool, stamp: float, n: int) -> Debounce:
    if stamp <= d.last_stamp + 1e-9:  # the same observation again (a cycle faster than perception)
        return d
    if raw == d.state:
        return replace(d, run=0, last_stamp=stamp)
    run = d.run + 1
    if run >= max(1, n):
        return Debounce(raw, 0, stamp, d.flips + 1)
    return replace(d, run=run, last_stamp=stamp)


@dataclass(frozen=True)
class AgentMemory:
    rest_t: Optional[float] = None  # the stamp of its first observation at rest at its line
    rest_s: Optional[float] = None  # its s_entry then
    moved: Debounce = Debounce()  # moved off its line since it rested (filtered)
    stopping: Debounce = Debounce()  # braking for its line (filtered)
    in_zone: bool = False
    cleared: bool = False
    seen_since: Optional[float] = None


@dataclass(frozen=True)
class ProbeState:
    phase: str = ""  # "", creep, watch, done, abort
    since: float = 0.0
    start_s: Optional[float] = None
    reason: str = ""


@dataclass
class Memory:
    agents: dict = field(default_factory=dict)  # id -> AgentMemory
    own_rest_t: Optional[float] = None
    own_rest_s: Optional[float] = None
    own_moving_since: Optional[float] = None
    last_event: float = -INF
    exit_clear: bool = True
    decision: Optional[Decision] = None
    decision_since: float = -INF
    go_since: Optional[float] = None
    probe: ProbeState = ProbeState()
    record_key: tuple = ()
    flips: int = 0


def at_rest_at_line(ag: Agent, params):
    return ag.v < params.rest_speed and 0.0 < ag.s_entry <= params.rest_window + 0.05


def is_stopping(ag: Agent, cap, params):
    """Braking for its line: the deceleration it shows is enough to stop there, and a comfortable stop
    (074 C8's ``_stopping``: a car whose safety layer only slows it is not stopping for its line)."""
    need = ag.v * ag.v / (2 * max(ag.s_entry + 2.0, 0.1))
    return ag.a < -params.yield_decel and need <= -ag.a + 1.0 and need <= cap.b_comfort + 1.0


def update_agent(m: AgentMemory, ag: Agent, t, params) -> tuple:
    """(new AgentMemory, event): event is True when the agent is seen starting, entering or leaving."""
    n = int(params.guard_filter_n)
    rest_t, rest_s = m.rest_t, m.rest_s
    if rest_t is None and at_rest_at_line(ag, params):
        rest_t, rest_s = ag.stamp, ag.s_entry
    moved_raw = False
    if rest_t is not None:
        off = rest_s - ag.s_entry
        moved_raw = (ag.v > params.moved_speed and off > params.moved_dist) or off > params.moved_dist_alone
    moved = debounce(m.moved, moved_raw, ag.stamp, n)
    stopping = debounce(m.stopping, is_stopping(ag, ag.cap, params), ag.stamp, n)
    in_zone, cleared = ag.in_zone, ag.cleared or m.cleared
    event = (moved.state and not m.moved.state) or (in_zone and not m.in_zone) or (cleared and not m.cleared)
    return AgentMemory(rest_t, rest_s, moved, stopping, in_zone, cleared,
                       m.seen_since if m.seen_since is not None else t), event


def update_memory(mem: Memory, t, own: Agent, others, params, exit_clear=True) -> Memory:
    """Advance the memory by one cycle (a new Memory; the old one is left as it was)."""
    new = Memory(**{k: getattr(mem, k) for k in mem.__dataclass_fields__})
    new.agents = dict(mem.agents)
    if own.v < params.rest_speed and 0.0 < own.s_entry <= params.rest_window + 0.05:
        if new.own_rest_t is None:
            new.own_rest_t, new.own_rest_s = t, own.s_entry
    if own.v >= params.rest_speed:
        new.own_moving_since = mem.own_moving_since if mem.own_moving_since is not None else t
    else:
        new.own_moving_since = None
    for ag in others:
        am, ev = update_agent(mem.agents.get(ag.id, AgentMemory()), ag, t, params)
        new.agents[ag.id] = am
        if ev:
            new.last_event = max(new.last_event, t)
    if exit_clear and not mem.exit_clear:
        new.last_event = max(new.last_event, t)  # a blocked exit's release is a junction event (074 C8)
    new.exit_clear = exit_clear
    return new


def quiet(mem: Memory, t, params):
    """Seconds the junction has been quiet: since the own dwell ended or the last event, whichever is later."""
    if mem.own_rest_t is None:
        return -INF
    return t - max(mem.own_rest_t + params.stop_dwell, mem.last_event)


def need_to_stop(ag: Agent):
    return stop_need(ag.s_entry, ag.v)
