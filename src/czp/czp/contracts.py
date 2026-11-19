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
    cycle_time: float = 0.0  # s, the self contract's reading (SelfWindow.reading: the decisions' CPU-time median)
    alive: bool = True
    self_ok: object = None  # the self contract's state when the adapter keeps one (SelfWindow.ok), else cycle_time <= bound
    map_ok: bool = True
    track_age: dict = field(default_factory=dict)  # agent id -> s tracked (conflicting agents)
    accel: dict = field(default_factory=dict)  # agent id -> (observed a, class a_max, class b_max)
    reach: float = INF  # m, detection reach on the conflicting approach
    v_max: float = 0.0  # m/s on the conflicting approach (for the reach contract)


class SelfWindow:
    """The self contract's state, kept by the adapter over the last ``n`` decision cycles (Strategy.cycle's own
    time, ``Strategy.last_cycle_cpu`` and ``last_cycle``; issue 066, 078 P2g):

    - R10's ``bound`` (20 ms) judges the median of the decisions' *CPU* time (``time.thread_time``): wall time on
      a loaded host counts preemption, which is not the decision's cost (P2g's grid ran at load 52-113 and nice 19);
    - wall time is judged only against the tick ``period`` (100 ms at 10 Hz), as liveness: a window whose wall
      median exceeds the period is a node that is not alive;
    - judged on a full window only: the first cycle is cold (20-26 ms, then under 1 ms), and a window of one read
      it (25 of 32 P2g runs failed at t0 + 0.1 s);
    - hysteresis: it fails when the median exceeds the bound and recovers only once the median is below
      ``recover`` x bound and it has failed for ``hold`` s; a median at the bound flipped the car to stock and back
      every few ticks (P2g: 219 of 891 logged decision lines)."""

    def __init__(self, n=10, bound=0.020, recover=0.75, hold=1.5, period=0.1):
        self.n, self.bound, self.recover, self.hold, self.period = n, bound, recover, hold, period
        self.cpu, self.wall = [], []
        self.ok, self.alive = True, True
        self.fail_since = None

    @staticmethod
    def _median(xs):
        xs = sorted(xs) or [0.0]
        return xs[len(xs) // 2]

    @property
    def full(self):
        return len(self.cpu) >= self.n

    def reading(self):
        """The CPU-time median the contract reports (0.0 until the window is full)."""
        return self._median(self.cpu) if self.full else 0.0

    def wall_median(self):
        return self._median(self.wall) if self.full else 0.0

    def add(self, t, cpu, wall):
        self.cpu.append(float(cpu))
        self.wall.append(float(wall))
        del self.cpu[:-self.n], self.wall[:-self.n]
        if not self.full:
            return
        self.alive = self.wall_median() <= self.period
        med = self.reading()
        if self.ok and med > self.bound:
            self.ok, self.fail_since = False, t
        elif not self.ok and med < self.recover * self.bound and t - self.fail_since >= self.hold - 1e-9:
            self.ok, self.fail_since = True, None


def slow_only(states, inp):
    """The only stock fallback is the self contract's, on a live node: the decision is slow, not broken (issue 066:
    a committed go does not change hands for that)."""
    failing = {s.name for s in states if not s.ok and s.fallback == "stock"}
    return bool(inp.alive) and failing == {"self"}


def check(contracts, inp: Inputs):
    """All contract states, failing ones included (a list of ContractState)."""
    out = []
    c = contracts.get("latency")
    if c:
        out.append(ContractState("latency", inp.obs_age <= c["bound"], round(inp.obs_age, 3), c["fallback"]))
    c = contracts.get("self")
    if c:
        ok = inp.self_ok if inp.self_ok is not None else inp.cycle_time <= c["bound"]
        out.append(ContractState("self", bool(inp.alive and ok), round(inp.cycle_time, 4), c["fallback"]))
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
