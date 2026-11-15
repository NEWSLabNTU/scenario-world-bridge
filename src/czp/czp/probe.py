"""Step 7, probe and commit.

Two probes:
- the legible probe before a round-2 go (074 C8 fix (c)): stock Autoware reads a car at rest as
  static and restarts into the round-2 go; so the car first creeps about ``probe_creep_dist`` (at
  ``creep_speed``), then holds and watches for ``probe_watch``; only if nobody moved off its line
  meanwhile does it go. A hard verdict during the probe (someone moved or entered) aborts it, and the
  car yields from where it is (still before the zone).
- the claim probe, when a go needs the other to yield: creep and classify the other's filtered
  acceleration (``classify_response``): yield is <= -yield_decel held ``yield_hold`` within the window;
  a hold steps the car down to yield before its commit point; a creep that gains nothing within
  ``creep_timeout`` steps down to yield.
"""

from __future__ import annotations

from dataclasses import replace

from .memory import ProbeState
from .model import CREEP, GO, YIELD, Decision

PROBED = ("round2", "cycle", "last_resort", "first_probe", "rest_tie")


def probe(decision: Decision, own, ps: ProbeState, params, t, row=None, rest_s=None):
    """(decision, new probe state). A probed go becomes CREEP (reason probe:creep / probe:watch) until done.
    The creep is measured from the car's rest at its line (rest_s), once per rest: a later round watches
    from where the car is."""
    row = row or {}
    if decision.kind == GO and decision.reason in PROBED and row.get("probe_round2", False):
        if ps.phase == "done":
            return decision, ps
        if ps.phase not in ("creep", "watch"):
            ps = ProbeState("creep", t, own.s_entry if rest_s is None else max(rest_s, own.s_entry), decision.reason)
        if ps.phase == "creep":
            if ps.start_s - own.s_entry >= params.probe_creep_dist - 0.05 or own.s_entry <= 0.35:
                ps = ProbeState("watch", t, ps.start_s, ps.reason)
            else:
                return replace(decision, kind=CREEP, reason="probe:creep"), ps
        if ps.phase == "watch":
            if t - ps.since >= params.probe_watch - 1e-9:
                return replace(decision, reason=decision.reason + "+probe"), ProbeState("done", t, ps.start_s, ps.reason)
            return replace(decision, kind=CREEP, reason="probe:watch"), ps
    if decision.kind != GO and ps.phase in ("creep", "watch"):
        return decision, ProbeState("abort", t, ps.start_s, decision.reason)
    if decision.kind != GO and ps.phase == "done" and own.v < params.rest_speed:
        return decision, ProbeState()  # a later round probes again
    return decision, ps


def classify_response(samples, params, t_probe):
    """The other's response to a claim probe from its observed (stamp, acceleration) samples after the
    probe began at t_probe: 'yield' (<= -yield_decel held yield_hold s, starting within the window),
    'hold' (the window passed without one), or 'unknown' (still inside the window)."""
    lo, hi = params.yield_window
    run_start = None
    for stamp, a in samples:
        if stamp < t_probe + lo - 1e-9 - params.yield_hold:
            continue
        if a <= -params.yield_decel:
            run_start = stamp if run_start is None else run_start
            if stamp - run_start >= params.yield_hold - 1e-9 and run_start <= t_probe + hi + 1e-9:
                return "yield"
        else:
            run_start = None
    last = samples[-1][0] if samples else t_probe
    return "hold" if last >= t_probe + hi + params.yield_hold else "unknown"


def claim(decision: Decision, response, own, params, t, t_probe, commit_point_passed):
    """The claim probe's step: CREEP while the response is unknown, GO on a yield, YIELD on a hold
    (before the commit point; after it the car continues) or after ``creep_timeout``."""
    if decision.kind != GO:
        return decision
    if response == "yield" or commit_point_passed:
        return replace(decision, reason=decision.reason + "+claim")
    if response == "hold" or t - t_probe >= params.creep_timeout:
        return replace(decision, kind=YIELD, reason="claim:hold")
    return replace(decision, kind=CREEP, reason="claim:creep")
