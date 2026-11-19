"""Issue 069, part 2: the tie-break's timing variants, as experiment patches (imported by p2g_guard_pre.py when its
environment names one; czp itself is not changed).

- P069_WATCH_FROM_REST (s; variant a): the probe's watch also lasts until this long after the later of the two
  rests (the own car's and its conflict partners'). 6.5 = a 5 s patience restart + see_delay 1.5 s.
- P069_NO_PROBE_REST_TIE=1 (variant b): the rest tie's go is not probed; the priority car goes when the guard ends.

The later rest is read in czp.cycle's call of decide (its pairs carry each partner's rest stamp) and used by the
probe call right after it, in the same cycle of the same car."""
import os
from dataclasses import replace

import czp.cycle as CY
from czp.model import CREEP, GO

WATCH_FROM_REST = float(os.environ.get("P069_WATCH_FROM_REST", "0") or 0)
NO_PROBE_REST_TIE = os.environ.get("P069_NO_PROBE_REST_TIE") == "1"
_state = {"later": None}
_decide, _probe = CY.decide, CY.probe


def decide(conv, zone, own, own_iv, pairs, mem, *args, **kw):
    rests = [mem.own_rest_t] + [pv.mem.rest_t for pv in pairs]
    rests = [r for r in rests if r is not None]
    _state["later"] = max(rests) if len(rests) >= 2 else None
    return _decide(conv, zone, own, own_iv, pairs, mem, *args, **kw)


def probe(decision, own, ps, params, t, row=None, rest_s=None):
    if (NO_PROBE_REST_TIE and decision.kind == GO and decision.reason == "rest_tie"
            and ps.phase not in ("creep", "watch")):
        return decision, ps
    d, ns = _probe(decision, own, ps, params, t, row, rest_s)
    later = _state["later"]
    if (WATCH_FROM_REST and later is not None and d.kind == GO and d.reason.endswith("+probe")
            and ps.phase == "watch" and t < later + WATCH_FROM_REST - 1e-9):
        return replace(decision, kind=CREEP, reason="probe:watch"), ps
    return d, ns


if WATCH_FROM_REST or NO_PROBE_REST_TIE:
    CY.decide, CY.probe = decide, probe
