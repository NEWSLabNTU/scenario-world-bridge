# 082: czp's watchdog clears a velocity limit czp never set, and Autoware answers with 0 m/s

**Severity**: high. The ego never moves: 7 czp runs in U3 stage 1. **Status**: fixed, landed on main (70316c66; czp's side); the selector's behaviour is an Autoware defect, for the user to report upstream; U3's reruns and
the next 078 grid check it. **Found**: 2026-11-20, by 139, in U3 stage 1 (u3-autoware; the runs on 139, copied to the
NAS by 139).

## What happens

In 7 czp runs of U3 stage 1 the ego stood at its start for the whole run. In each one, the same thing happened:
- czp's watchdog (`czp/watchdog.py`) logged "no heartbeat ... back to auto mode";
- it published a `VelocityLimitClearCommand` with sender `interaction_strategy`;
- 0.2 to 7 s later, Autoware 1.5's `external_velocity_limit_selector` published a velocity limit of 0.0 m/s with an
  empty sender;
- from then on, the ego did not move.

The clear never appeared in good runs or in the stock arm. czp had set no limit in those runs, so no limit with sender
`interaction_strategy` was in force when the clear arrived.

## Cause

The watchdog published the clear on every lost heartbeat, unconditionally. czp's strategy and node already clear only
a limit they set (`Strategy.limited`, and the node's shutdown checks it).

On the selector's side, 139 confirmed the mechanism from source (autoware_universe 0.48.0,
`external_velocity_limit_selector_node.cpp`):
- `onVelocityLimitClearCommand` (l.169-179) calls `clearVelocityLimit`;
- `clearVelocityLimit` returns early when `velocity_limit_table_` is empty (l.229-231, WARN "no velocity limit has been
  set from internal."). It does so whenever no other node holds a limit;
- the callback then publishes `hardest_limit_` unconditionally. That member is value-initialised, so it is
  `max_velocity` 0.0 with sender `''`;
- the 0 holds until the next API or internal limit arrives.

With a non-empty table, a clear from a sender that holds no limit is harmless.

**This is an Autoware-side defect, worth reporting upstream.** A clear with nothing to clear should not publish a 0
m/s limit. Whether to post it is the user's decision. Whatever the selector does, a clear without a limit is czp's to
avoid.

## Fix (p2g-launch)

The watchdog clears the limit only when the last heartbeat says the node held one. The heartbeat already carries
`limited`, which is `Strategy.limited` at the cycle's decision, before its commands. The decision lives in a pure
function, `czp.watchdog.trip(last, modules)`, which returns the modules to put back in auto mode and whether to clear.
When nothing is to be cleared, the warning says "no velocity limit held, none cleared".

The rule everywhere is now the same: a `VelocityLimitClearCommand` goes out only after a limit czp set and has not yet
cleared.

## Tests

`test_strategy.py::test_a_clear_is_sent_only_with_a_limit_in_force` covers both sides:
- **the watchdog's `trip`:** no clear when the last heartbeat held no limit, and a clear when it held one. This part
  fails on the old code, which had no such rule and always cleared;
- **the strategy:** through a crossing and its release, every `clear` it issues follows a limit not yet cleared.

czp's tests: 86 passed.
