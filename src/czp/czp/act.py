"""Step 8's protocol side: the commit point and the latches. (The act itself -- RTC approve or deny,
the velocity limit, the arena car's command -- is the adapter's.)

- committed: a go that has passed its commit point stays go (the last point from which the car can
  still stop before the zone; a row with ``commit: on_move`` -- the all-way stop, starting from its
  line -- commits as soon as the car moves under a go, as 074 C8's rule did);
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
    """The decision to act on, and whether it is a new one (for the record and the latch's clock)."""
    row = row or {}
    if prev is not None and prev.kind == GO:
        on_move = row.get("commit") == "on_move" and own_moving
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
