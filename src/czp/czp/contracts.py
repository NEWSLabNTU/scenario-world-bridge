"""Step 9, the contracts: each a predicate over the cycle's inputs, with its fallback
(contracts.yaml). A failure with fallback ``stock`` hands the car back to stock Autoware (R9);
``phantom`` treats that agent as a phantom at its first position; ``contested`` makes that agent
contested (it keeps the turn); ``refuse_gap_creep`` refuses a gap (left turn, roundabout)."""

from __future__ import annotations

from dataclasses import dataclass, field

from .model import INF


@dataclass(frozen=True)
class ContractState:
    name: str
    ok: bool
    value: object
    fallback: str
    agent: object = None


@dataclass(frozen=True)
class Inputs:
    obs_age: float = 0.0  # s, the age of the newest objects message
    cycle_time: float = 0.0  # s, the node's last cycle
    alive: bool = True
    map_ok: bool = True
    track_age: dict = field(default_factory=dict)  # agent id -> s tracked (conflicting agents)
    accel: dict = field(default_factory=dict)  # agent id -> (observed a, class a_max, class b_max)
    reach: float = INF  # m, detection reach on the conflicting approach
    v_max: float = 0.0  # m/s on the conflicting approach (for the reach contract)


class SelfWindow:
    """The self contract's reading: the median of the last ``n`` *decision* cycles (Strategy.cycle's own time,
    ``Strategy.last_cycle``). R10's 20 ms is the decision's budget; the node's whole tick also holds the RTC
    service calls, the record publishing and the log file's writes, whose time made the median straddle the
    bound and flip the car to stock and back for single ticks (078 P2f: 162 of 805 logged decisions; 078 P2g)."""

    def __init__(self, n=10):
        self.n = n
        self.times = []

    def add(self, decision_time):
        self.times.append(float(decision_time))
        del self.times[:-self.n]

    def median(self):
        xs = sorted(self.times) or [0.0]
        return xs[len(xs) // 2]


def check(contracts, inp: Inputs):
    """All contract states, failing ones included (a list of ContractState)."""
    out = []
    c = contracts.get("latency")
    if c:
        out.append(ContractState("latency", inp.obs_age <= c["bound"], round(inp.obs_age, 3), c["fallback"]))
    c = contracts.get("self")
    if c:
        out.append(ContractState("self", inp.alive and inp.cycle_time <= c["bound"], round(inp.cycle_time, 4), c["fallback"]))
    c = contracts.get("map")
    if c:
        out.append(ContractState("map", bool(inp.map_ok), bool(inp.map_ok), c["fallback"]))
    c = contracts.get("track")
    if c:
        for aid, age in inp.track_age.items():
            out.append(ContractState("track", age >= c["bound"], round(age, 2), c["fallback"], aid))
    c = contracts.get("others")
    if c:
        for aid, (a, a_max, b_max) in inp.accel.items():
            ok = -b_max - c["slack"] <= a <= a_max + c["slack"]
            out.append(ContractState("others", ok, round(a, 2), c["fallback"], aid))
    c = contracts.get("reach")
    if c and inp.v_max > 0:
        need = inp.v_max * (c["turn_time"] + c["extra"])
        out.append(ContractState("reach", inp.reach >= need, round(inp.reach, 1), c["fallback"]))
    return out


def fallback(states):
    """(stock?, phantom ids, contested ids, refuse_gap?) from the failing contracts."""
    bad = [s for s in states if not s.ok]
    stock = any(s.fallback == "stock" for s in bad)
    phantom = {s.agent for s in bad if s.fallback == "phantom"}
    contested = {s.agent for s in bad if s.fallback == "contested"}
    refuse = any(s.fallback == "refuse_gap_creep" for s in bad)
    return stock, phantom, contested, refuse


def summary(states):
    out = {}
    for s in states:
        k = s.name if s.agent is None else f"{s.name}:{s.agent}"
        out[k] = {"ok": s.ok, "value": s.value}
    return out
