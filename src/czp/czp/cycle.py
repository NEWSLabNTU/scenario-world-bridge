"""One cycle, steps 2-10, over the functions of the other modules.

``Protocol.step`` is what an adapter calls every cycle: it takes the own car and the observed agents
(each relative to the zone, or with poses for ``relate`` to place), the zone, and the contract inputs;
it returns the decision to act on and, when it changed, the record. Its only state is the
``Memory`` it returns (kept by the Protocol object between calls)."""

from __future__ import annotations

import time
from dataclasses import replace

from . import contracts as C
from .act import HOLD, latch
from .config import load_contracts, load_conventions, load_params
from .convention import PairView, decide
from .intervals import other_interval, own_interval
from .memory import AgentMemory, Memory, update_memory
from .model import CONTESTED, GO, STOCK, Agent, Band, Decision, Zone
from .probe import probe
from .record import record
from .relation import conflict_set, relate


class Protocol:
    def __init__(self, conventions=None, params=None, contracts=None, jurisdiction="generic"):
        self.conv = conventions if conventions is not None else load_conventions(jurisdiction)
        self.params = params if params is not None else load_params()
        self.contracts = contracts if contracts is not None else load_contracts()
        self.mem = {}  # zone id -> Memory
        self.cycle_times = []

    def memory(self, zone_id):
        return self.mem.setdefault(zone_id, Memory())

    def step(self, t, own: Agent, others, zone: Zone, inputs: C.Inputs = None, exit_clear=True, obs_age=None,
             placed=True, bystanders=()):
        """(decision, record or None). placed: the agents already carry their approach and s_entry / s_exit.
        bystanders: cars at the junction outside this zone's conflict set, each placed on its own zone: only their
        events (start, entry, exit) are used, for the junction's quiet epoch."""
        t0 = time.perf_counter()
        p = self.params
        row = self.conv.get(zone.kind) or {}
        inputs = inputs or C.Inputs(obs_age=obs_age or 0.0)
        # 2. relation
        if placed:
            confl = [a for a in others if a.approach is not None and a.approach != own.approach and not a.cleared]
        else:
            rel = relate(others, own, [zone]).get(zone.id, [])
            confl = [a for _, a in conflict_set(rel)]
        # 9. contracts (before the decision: a failure hands the car to stock, or marks an agent)
        states = C.check(self.contracts, inputs)
        stock, phantom, forced, _refuse = C.fallback(states)
        slow = stock and C.slow_only(states, inputs)  # only the self contract, on a live node (issue 066)
        confl = [replace(a, phantom=True) if a.id in phantom else a for a in confl]
        # memory
        mem = update_memory(self.memory(zone.id), t, own, confl, p, exit_clear, bystanders)
        # 3. intervals
        own_iv = own_interval(own, p)
        ivs = {a.id: other_interval(a, p) for a in confl}
        pairs = []
        for a in confl:
            am = mem.agents.get(a.id, AgentMemory())
            b = Band(CONTESTED, a.id) if a.id in forced else None
            pairs.append(PairView(a, ivs[a.id], am, b))
        # 4-6. band, role, convention
        age = inputs.obs_age if obs_age is None else obs_age
        if stock:
            dec, verdicts = Decision(STOCK, zone.id, "contract", zone.kind), []
        else:
            dec, verdicts = decide(self.conv, zone, own, own_iv, pairs, mem, p, t, age, exit_clear)
        # 7. probe
        dec, mem.probe = probe(dec, own, mem.probe, p, t, row, mem.own_rest_s)
        # 8. the latches and the commit point
        prev = mem.decision
        moving_from_rest = mem.own_moving_since is not None and mem.go_from_rest
        if mem.go_since is not None and own.v > p.moved_speed:
            mem.go_moved = True
        # issue 075: a stopping decision is held over a slow cycle while the conflict set (read this cycle) is not
        # empty, for at most hold_by_contract_s
        hold = slow and bool(confl) and (mem.hold_since is None or t - mem.hold_since < p.hold_by_contract_s - 1e-9)
        dec, changed = latch(dec, prev, mem.decision_since, mem.go_since, own, moving_from_rest, p, t, row,
                             go_from_rest=mem.go_from_rest and not mem.go_moved, slow=slow, hold=hold)
        if dec.reason == HOLD:
            mem.hold_since = t if mem.hold_since is None else mem.hold_since
        else:
            mem.hold_since = None
        if changed:
            mem.decision_since = t
            mem.flips += 1
        if dec.kind == GO and (prev is None or prev.kind != GO):
            mem.go_since = t
            mem.go_from_rest = own.v < p.moved_speed
            mem.go_moved = False
        elif dec.kind != GO:
            mem.go_since = None
            mem.go_from_rest = False
            mem.go_moved = False
        mem.decision = dec
        self.mem[zone.id] = mem
        # 10. record
        rec = None
        if changed:
            rec = record(t, zone.id, dec, verdicts, ivs, C.summary(states), mem.probe, conflict=[a.id for a in confl])
        self.cycle_times.append(time.perf_counter() - t0)
        return dec, rec
