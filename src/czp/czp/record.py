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


def flip_to_json(flip):
    """Issue 075: a self-window flip (contracts.SelfWindow.add's return) as its own decisions.jsonl line, written
    whether or not a decision changes. It carries ``zone``, ``decision`` (null) and ``contracts.self`` so that the
    readers of decision lines skip or count it; ``event`` marks it."""
    d = dict(flip, zone=None, conflict_set=[], decision=None,
             reason="self_window: " + ("ok" if flip["ok"] else "failed") + ("" if flip["alive"] else ", not alive"),
             contracts={"self": {"ok": bool(flip["ok"] and flip["alive"]), "value": round(flip["cpu_median_ms"] / 1e3, 4)}})
    return json.dumps(_num(d), sort_keys=True, separators=(",", ":"))


def cs_hold_to_json(ev):
    """Issue 079: a change of the partners a zone's conflict set keeps after they dropped out of it unseen leaving
    (Strategy.cs_events) as its own decisions.jsonl line. Like a self-window flip it carries ``decision`` (null) so
    that the readers of decision lines skip or count it; ``event`` marks it and ``reason`` names the kept ids."""
    kept = list(ev["kept"])
    d = dict(ev, event="cs_hold", decision=None,
             reason="cs_hold: " + (",".join(kept) + " kept in the conflict set" if kept else "released"))
    return json.dumps(_num(d), sort_keys=True, separators=(",", ":"))
