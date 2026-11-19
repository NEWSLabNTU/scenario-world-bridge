"""Step 7, probe and commit.

Two probes:
- the legible probe before a round-2 go (074 C8 fix (c)): stock Autoware reads a car at rest as
  static and restarts into the round-2 go; so the car first creeps about ``probe_creep_dist`` (at
  ``creep_speed``), then holds and watches for ``probe_watch``; only if nobody moved off its line
  meanwhile does it go. A hard verdict during the probe (someone moved or entered) aborts it, and the
  car yields from where it is (still before the zone). Any other go while the probe runs, or after it was aborted, is
  probed too (078 P2g). The creep ends while the car can still stop before the zone (``stop_margin``, issue
  069): the creep is asked to stop as soon as the zone is within the margin at the car's speed, a car resting
  within the margin at rest watches where it is, and a car that enters the zone during the creep or the watch goes
  on as ``probe_overrun``. The creep is kept wherever there is room for one: moving is what shows the go (a car
  that restarts on a mutual rest's patience sees the rest end), and dropping it from a rest within the margin at
  creep_speed (1.1 m) failed the formal check at patience 8-9 s (150 and 50 unsafe runs of 3780).
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


def stop_margin(v, params):
    """Issue 069: how far before the zone the car must be asked to stop, at speed v, to stop before it. Autoware
    keeps accelerating for about 0.4 s after a stop request, then brakes. The envelope of the creeps stopped in
    078 P2g's grid and its rerun (15 stops requested at 0.06-0.20 m/s, travel 0.04-0.25 m) is
    ``probe_stop_margin + probe_stop_time * v``; above 0.2 m/s it is extrapolated."""
    return params.probe_stop_margin + params.probe_stop_time * max(v, 0.0)


def probe(decision: Decision, own, ps: ProbeState, params, t, row=None, rest_s=None):
    """(decision, new probe state). A probed go becomes CREEP (reason probe:creep / probe:watch) until done.
    The creep is measured from the car's rest at its line (rest_s), once per rest: a later round watches
    from where the car is."""
    row = row or {}
    # (078 P2g) a probe under way or aborted is not bypassed: another go during the creep or the watch, or after an
    # abort (the rest guard's "clear", read off the other's rest) is probed; else it went at once from the line,
    # with the other's start not yet visible, as the probe was there to prevent
    if decision.kind == GO and decision.reason == "inside" and ps.phase in ("creep", "watch"):
        # issue 069: the car entered the zone during the probe (it could not stop before it): it goes on, committed,
        # and the record says so
        return replace(decision, reason="probe_overrun"), ProbeState("done", t, ps.start_s, ps.reason)
    probing = ps.phase in ("creep", "watch", "abort") and decision.reason != "inside"
    if decision.kind == GO and (decision.reason in PROBED or probing) and row.get("probe_round2", False):
        if ps.phase == "done":
            return decision, ps
        if ps.phase not in ("creep", "watch"):
            start_s = own.s_entry if rest_s is None else max(rest_s, own.s_entry)
            # issue 069: a car within the margin at rest watches where it is
            room = own.s_entry > max(0.35, stop_margin(0.0, params))
            ps = ProbeState("creep" if room else "watch", t, start_s, decision.reason)
        if ps.phase == "creep":
            if (ps.start_s - own.s_entry >= params.probe_creep_dist - 0.05
                    or own.s_entry <= max(0.35, stop_margin(own.v, params))):
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
