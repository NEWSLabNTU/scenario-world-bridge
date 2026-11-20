"""Step 8's protocol side: the commit point and the latches. (The act itself -- RTC approve or deny,
the velocity limit, the arena car's command -- is the adapter's.)

- committed: a go that has passed its commit point stays go (the last point from which the car can
  still stop before the zone); a row with ``commit: on_move`` commits a go that began from rest (the
  car below ``moved_speed``) as soon as the car moves under it, as 074 C8's rule did at the all-way
  stop, and until the car is in the zone (the crossing, 078 P2: an uncommitted go from rest, re-read as
  ambiguous once the car moves, chattered go/wait until the other car's patience won). A go given while
  the car is moving commits only at its commit point. With ``commit_release: turn`` (the crossing) the
  commit gives way to the turn guard -- the other seen moving off its line or in the zone -- while the car
  can still stop comfortably before the zone (078 P2b: committed against that, czp as the arena's B met
  gap-test and patience restarts in 1-3 of 90 runs per cell, where stock met none). The commit is the
  protocol's: it no longer retracts its own go, but Autoware's safety layer (obstacle stop, the AEB) still acts, and once that
  brings the car to rest the commit is released (it is no longer moving); a failed contract (stock) is
  never latched or committed over, except the self contract's on a live node: a committed go does not change
  hands for a slow decision (issue 066, 078 P2g: a go from rest, moving under ``commit: on_move``, went to stock
  for one tick at a 20 ms median and the other car went first);
- the go latch (074 C8 fix (a)): a go is held for the car's launch dead time, unless an agent is in
  the zone or seen moving off its line (the "turn" guard). A go begun from rest is held further, until the
  car first moves under it (above ``moved_speed``) or ``go_latch_motion_timeout`` has passed, against a wait only (a
  re-read band, ambiguous): a yield (the turn guard, contested, the other clear first) releases it (078 P2c:
  Autoware takes about 1.4 s from ACTIVATE at rest to motion, longer than the 0.5 s launch latch, so a
  band re-read flipped the go before the car moved, and ``commit: on_move`` had nothing to commit yet). Chosen over
  the own ``launch_dead`` at 1.4 s, which would also move the own interval 0.9 s later in every band reading,
  in the arena too, where the car launches in 0.5 s; the latch ends at motion, whatever the delay;
- the decision latch (R10): a decision is held ``latch_s`` unless the reason changes (a band, role or
  guard change beyond its hysteresis, which the margins and the filters provide);
- the hold by contract (issue 075): a self-contract failure on a live node (slow, not broken) does not hand a
  stopping decision -- yield, wait, wait_second_round, or the probe's creep or watch -- to stock. Stock reads the
  other car at rest as static and goes (075: the car was handed over mid-probe and launched 0.6 s before the other
  car's patience fired, a contact). The car stops instead: the decision becomes ``HOLD`` (a wait, or the yield or
  wait_second_round it was; a creep is aborted to a wait), which the adapter acts as a stop (manual mode,
  DEACTIVATE, a velocity limit that stops before the zone). It holds while the caller allows (``hold``: the conflict
  set, read this cycle, is not empty and ``hold_by_contract_s`` has not passed since the hold began); then the
  failure hands the car to stock as before. A node that is not alive, or any other failed contract, still hands the
  car to stock at once (R9).
"""

from __future__ import annotations

from dataclasses import replace

from .intervals import stop_need
from .model import CREEP, GO, STOCK, STOPPING, WAIT, WAIT_SECOND_ROUND, YIELD, Decision

HOLD = "hold: contract"  # the reason of a stopping decision held over a slow decision cycle (issue 075)


def commit_point_passed(own, cap=None):
    cap = cap or own.cap
    return own.in_zone or own.cleared or (own.v > 0.5 and stop_need(own.s_entry, own.v) > cap.b_comfort)


def holds_a_stop(prev: Decision):
    """Issue 075: a decision of czp's that stops the car (not stock's own reading): what a slow cycle holds."""
    if prev is None or prev.stock_equivalent:
        return False
    return prev.kind in STOPPING or (prev.kind == CREEP and prev.reason.startswith("probe:"))


def latch(new: Decision, prev: Decision, prev_since, go_since, own, own_moving, params, t, row=None,
          go_from_rest=False, slow=False, hold=False):
    """The decision to act on, and whether it is a new one (for the record and the latch's clock).
    own_moving: the car is moving under a go that began from rest (what ``commit: on_move`` commits).
    go_from_rest: the current go began with the car below ``moved_speed`` (the latch until motion).
    slow: the only failing contract is the self contract, on a live node (contracts.slow_only).
    hold: a stopping decision may be held over that (issue 075: the conflict set is not empty and the hold has not
    timed out; the caller's to judge)."""
    row = row or {}
    if new.kind == STOCK and new.reason == "contract":
        if slow and prev is not None and prev.kind == GO and \
                ((row.get("commit") == "on_move" and own_moving) or commit_point_passed(own)):
            return replace(prev, committed=True, reason=prev.reason if prev.committed else prev.reason + "+committed",
                           latched_until=t), False
        if slow and hold and holds_a_stop(prev):
            kind = prev.kind if prev.kind in (YIELD, WAIT_SECOND_ROUND) else WAIT  # a creep or watch: aborted to a wait
            return Decision(kind, prev.zone, HOLD, prev.row, prev.against), prev.reason != HOLD
        return new, prev is None or prev.kind != STOCK or prev.reason != new.reason  # a failed contract hands the car
        #                                                   to stock at once: no latch or commit holds over it
    if prev is not None and prev.kind == GO:
        on_move = row.get("commit") == "on_move" and own_moving
        if on_move and row.get("commit_release") == "turn" and new.reason == "turn" and not commit_point_passed(own):
            on_move = False  # the other has started or is in the zone while this car can still stop: it stops
        if new.kind != GO and (on_move or commit_point_passed(own)):
            return replace(prev, committed=True, reason=prev.reason if prev.committed else prev.reason + "+committed",
                           latched_until=t), False
        if new.kind != GO and params.go_latch_launch and go_since is not None and \
                t - go_since < own.cap.launch_dead - 1e-9 and new.reason != "turn":
            return replace(prev, latched_until=go_since + own.cap.launch_dead), False
        if new.kind in (WAIT, WAIT_SECOND_ROUND) and params.go_latch_launch and go_from_rest and go_since is not None \
                and own.v <= params.moved_speed and t - go_since < params.go_latch_motion_timeout - 1e-9:
            return replace(prev, latched_until=go_since + params.go_latch_motion_timeout), False
    if prev is not None and new.kind != prev.kind and new.reason == prev.reason and new.kind not in (STOCK,) \
            and prev.kind != CREEP and t - prev_since < params.latch_s - 1e-9:
        return replace(prev, latched_until=prev_since + params.latch_s), False
    changed = prev is None or new.kind != prev.kind or new.reason != prev.reason
    return new, changed
