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
  never latched or committed over;
- the go latch (074 C8 fix (a)): a go is held for the car's launch dead time, unless an agent is in
  the zone or seen moving off its line (the "turn" guard);
- the decision latch (R10): a decision is held ``latch_s`` unless the reason changes (a band, role or
  guard change beyond its hysteresis, which the margins and the filters provide).
"""

from __future__ import annotations

from dataclasses import replace

from .intervals import stop_need
from .model import CREEP, GO, STOCK, Decision


def commit_point_passed(own, cap=None):
    cap = cap or own.cap
    return own.in_zone or own.cleared or (own.v > 0.5 and stop_need(own.s_entry, own.v) > cap.b_comfort)


def latch(new: Decision, prev: Decision, prev_since, go_since, own, own_moving, params, t, row=None):
    """The decision to act on, and whether it is a new one (for the record and the latch's clock).
    own_moving: the car is moving under a go that began from rest (what ``commit: on_move`` commits)."""
    row = row or {}
    if new.kind == STOCK and new.reason == "contract":
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
    if prev is not None and new.kind != prev.kind and new.reason == prev.reason and new.kind not in (STOCK,) \
            and prev.kind != CREEP and t - prev_since < params.latch_s - 1e-9:
        return replace(prev, latched_until=prev_since + params.latch_s), False
    changed = prev is None or new.kind != prev.kind or new.reason != prev.reason
    return new, changed
