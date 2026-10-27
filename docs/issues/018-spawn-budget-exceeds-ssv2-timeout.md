# 018: the bridge gives a Spawn 120 s, SSv2 waits 90 s for it

**Severity**: low (a loaded host only; the run fails before the ego moves, with a misleading
message). **Status**: open. **Found**: 2026-10-28, roadmap 057, Town05 lap 29 on Isaac.

## What is wrong

The bounds are meant to nest: every bound swb puts on a world request sits inside SSv2's
`SIMULATOR_RESPONSE_TIMEOUT`, 90 s (`docs/design/failure-and-frame-budget.md`, "Timeouts"), so a
slow request ends as swb's own error and not as SSv2 giving up on a simulator it calls dead.
The table there predates the world protocol. Since then the bridge gives every Spawn (and
LoadWorld, and the weather) the world server's `load_timeout_seconds`, 120 s
(`bridge_config.yaml`). The reason is that an Isaac ego's Spawn warms its sensors up, and a
walker's first render compiles its materials (40 s measured, roadmap 044). A Spawn between 90
and 120 s is within the bridge's budget and outside SSv2's.

Lap 29 had exactly that, on a host at a load average of 197. The ego's Spawn took 90.5 s
(`spawned ego 'ego' ... sensors warm after 6 render(s), 90.5 s`), and SSv2 ended the
scenario:

```
zmq_multi_client.cpp:103: No response from the simulator at tcp://localhost:5555 within 90 s
(SIMULATOR_RESPONSE_TIMEOUT). It may have crashed or hung with this request outstanding, or not be running.
```

Nothing had crashed or hung. With `SIMULATOR_RESPONSE_TIMEOUT=150`, lap 30 got past the
Spawn.

## Fix (not done)

Restore the nesting: either SSv2's response timeout for a Spawn above the bridge's
`load_timeout_seconds` plus the bridge's own overhead, or the bridge's Spawn budget below
SSv2's. The first keeps the walker and Isaac-ego cases working. Raising
`SIMULATOR_RESPONSE_TIMEOUT` for every request also delays the report of a really hung frame,
so a per-request bound in the SSv2 fork (Spawn and LoadWorld long, frames short) is the
cleaner option. The table in `failure-and-frame-budget.md` should gain the world-protocol rows
either way.
