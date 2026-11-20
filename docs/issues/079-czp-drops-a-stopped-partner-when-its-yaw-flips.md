# 079: czp drops a stopped partner from the conflict set when its track's yaw flips

**Severity**: medium. While the partner stood at its line, czp's decision flickered to `stock: no conflict` for single
ticks (2 flips per run in U3). Each flip restarted the protocol's decision and its latch timers, and a one-tick
handover at an all-way stop could let the ego go. **Status**: fixed, landed on main (e5b5bc93; the 076 grid had no contact).
**Found**: 2026-11-20, by 133, from 139's U3 stage 1 rerun (`u3_flicker.py`, u3-autoware e1c1be6f; the counter's
cs_gaps and track_gaps at 90b121c4).

## What happens

Run `on_dt-1_stop1y_c1.5_s0` (139, u3-autoware ebba6016): A (track feb52c43) stood at its line of the all-way stop,
about 25 m from the ego. czp dropped A from the zone's conflict set twice:

| t (s) | decision |
|---|---|
| 40.01 | `stock: no conflict` (A dropped) |
| 40.21 | `wait: stop` (A back, 0.20 s out) |
| 40.36 | `stock: no conflict` (dropped) |
| 40.61 | `wait: stop` (back, 0.26 s out) |

It was not a perception dropout. A's track on `/perception/object_recognition/objects` was continuous, with 144
sightings from 36.5 to 51.4 s and a largest gap of 0.173 s. A was present within 0.15 s of every drop. 139's dump of
the messages around the drops (`~/nas/home/u3_issue079_drop_objects_139.txt`):

- A's position was steady (x -110.62 to -110.72, y 4.254 to 4.264), its box 4.50-4.55 x 1.90-1.96 m, label CAR with
  probability 1.00;
- its yaw was about -179 deg (driving -x, towards the junction) except in four messages, at stamps 1066.8, 1067.0,
  1067.15 and 1067.25. In those it read about +0.5 deg, with vx +0.11 to +0.13 m/s where the others had -0.03 to
  -0.06;
- every drop followed a flipped message, and every return an unflipped one, one message later.

Autoware's tracker gives a stopped car's yaw with nothing to tell its direction, and it flipped it by 180 deg.

## Cause

`relation.match_approach` keeps an agent on a zone's approach only when it is within 2 m of the approach's centreline
and within 45 deg of its heading. A flipped yaw is 180 deg off, so A matched no approach. `relate` then fell back to
the off-lane test, a constant-velocity path of max(v, 1 m/s) x 8 s along the agent's own yaw. Along the flipped yaw
that path points away from the zone, so A was left out of the conflict set. With the set empty, the convention
decided `stock: no conflict`.

078 P2c's dropped-partner hold (`Strategy._dropped_hold`) already kept the last decision's commands (manual mode,
DEACTIVATE) through such a drop. The protocol itself still read an empty set. Its decision, latch and records
flipped, and its decision timers restarted when A came back.

## Fix (p2g-launch)

1. **The cause.** Below `STOPPED_SPEED` (0.5 m/s, params.yaml's `moved_speed`), an agent's yaw is no evidence of its
   direction. `place` now matches a stopped agent to an approach within 45 deg of its heading or of its reverse
   (`match_approach(..., stopped=True)`), but only before the zone (its arc length up to the approach's
   `entry_s`), where a car waits at its line. A moving agent is matched as before, so a car driving the other way
   is still not on the approach. The own car is matched as before. Inside the zone the approaches' paths cross,
   and in ddbbdde's first form a car stopped head-on on the own car's path was matched to the own approach in
   reverse, as a lead, which took it out of the conflict set. It is now the off-lane crossing agent it was before
   the fix (133's question, the same day).
2. **The backstop.** `Strategy` keeps a partner in a zone's conflict set for `cs_hold_s` (1.5 s, new in
   params.yaml) after it was last in it, as it was placed then and stamped now. This applies whatever the reason it
   dropped out, unless it was seen leaving (P2c's ruling: it was inside the zone, or it is tracked and moving).
   The protocol then keeps its decision and its memory of the partner. 139's counts from U3 stage 1 (11 czp runs
   on f090363c, before the reverse match): 21 gaps out of the conflict set, 0.097 to 1.156 s (median 0.4 s), and
   no gaps in perception's track, so every gap was czp's. The first value, 1.0 s, covered 20 of the 21; 1.5 s
   covers all (133, 2026-11-20). After `cs_hold_s`, P2c's hold still applies (`mode_hold`, 1.5 s), so a partner
   that really vanished holds the decision for `cs_hold_s + mode_hold` (3.0 s), which 133 ruled the safe side.
3. **The record.** Each change of a zone's kept partners is a line of `decisions.jsonl` of its own:
   `event: cs_hold`, `decision: null`, `kept: [ids]`, `conflict_set: [...]`, and `reason: "cs_hold: <ids> kept in
   the conflict set"` or `"cs_hold: released"`. Readers of decision lines skip or count it, as they do the
   self-window flips (issue 075).

## Tests

- `test_strategy.py::test_a_stopped_partner_with_its_yaw_flipped_stays_in_the_conflict_set`: the U3 sequence (yaw
  flipped for two messages, back, flipped, back, with vx about +0.12). It gives no `no conflict` and no kept or
  P2c-held ticks. A moving agent with the reversed yaw is still not placed. It fails on the old `relation.py`.
- `test_strategy.py::test_the_reverse_match_takes_no_head_on_car`: on a straight crossing of two-lane roads, a car
  waiting head-on at the opposite line with its yaw flipped is placed on its own approach, not the own car's (whose
  path runs 3.5 m beside it). A car stopped inside the zone on the own path, facing the own car, stays a crossing
  agent in the conflict set. The second part fails on ddbbdde.
- `test_strategy.py::test_a_partner_dropped_unseen_is_kept_in_the_conflict_set_for_cs_hold_s`: a track missing for
  ticks is kept for `cs_hold_s` after its last time in the set, and `cs_hold_s` covers the longest gap measured. The protocol's decision does not change, P2c's hold
  follows, and the `cs_hold` lines are as above. It fails on the old code.
- `test_dropped_partner_at_rest_holds_the_wait` and `test_mode_hysteresis_band_edge_flicker` count `cs_hold_s`
  before P2c's hold. czp's tests: 85 passed.
