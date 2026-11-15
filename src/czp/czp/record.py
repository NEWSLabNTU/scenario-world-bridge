"""Step 10, the record: one line per decision change, explainable by band, role, convention row and
signals (R7). ``to_json`` is what the ROS node publishes on /interaction_strategy/decision and logs."""

from __future__ import annotations

import json
import math

from .model import Record


def _num(x):
    if isinstance(x, float):
        return None if math.isinf(x) or math.isnan(x) else round(x, 3)
    if isinstance(x, (list, tuple)):
        return [_num(v) for v in x]
    if isinstance(x, dict):
        return {k: _num(v) for k, v in x.items()}
    return x


def record(t, zone, decision, verdicts, ivs, contracts_summary, probe_state=None, learned_share=0.0, conflict=None):
    agents = {}
    for v in verdicts:
        iv = ivs.get(v.other)
        agents[v.other] = {
            "verdict": v.kind, "hard": v.hard, "reason": v.reason,
            "band": None if v.band is None else {"kind": v.band.kind, "who": v.band.who,
                                                 "slack_own": v.band.slack_own, "slack_other": v.band.slack_other},
            "role": None if v.role is None else {"fact": v.role.fact, "reading": v.role.reading, "value": v.role.value},
            "interval": None if iv is None else {"t_in": iv.t_in, "t_out": iv.t_out, "reason": iv.reason},
        }
    first = verdicts[0] if verdicts else None
    return Record(t=t, zone=zone, conflict_set=list(conflict) if conflict is not None else [v.other for v in verdicts], agents=_num(agents),
                  band=None if first is None or first.band is None else first.band.kind,
                  role=None if first is None or first.role is None else _num({"fact": first.role.fact,
                                                                              "reading": first.role.reading}),
                  row=decision.row, probe=None if not probe_state or not probe_state.phase else
                  {"phase": probe_state.phase, "since": _num(probe_state.since), "for": probe_state.reason},
                  contracts=_num(contracts_summary), decision=decision.kind, reason=decision.reason,
                  learned_share=learned_share)


def to_json(rec: Record):
    return json.dumps(_num(rec.as_dict()), sort_keys=True, separators=(",", ":"))
