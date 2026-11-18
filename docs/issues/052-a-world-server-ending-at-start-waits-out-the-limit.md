# 052: A world server that ends during the stack's start leaves `scensim run` waiting 30 minutes

**Severity**: medium. Nothing runs either way, but the user waits the whole readiness limit
(1800 s) and is told only "not ready". **Status**: fixed on branch `052-start-exit`.
**Found**: 2026-11-18, swb 241, a speed_ladder run (077 item 26).

## What happens

`scensim run` in speed_ladder, on a checkout whose Isaac world server could not import a module
(a new file that the symlink install did not have yet), printed after 30 minutes:

```
scensim run: error: the stack was not ready within 1800 s (starting); .../stack-launches/<id>/console.log
```

The cause was in the simulator side's console from the first minute:

```
[scensim_isaac_world_server-1] ... ImportError: cannot import name 'step_rate' from 'scensim_isaac_world'
[scensim_isaac_world_server-1] ... SimulationApp.close() was not called explicitly. Shutting down automatically
[INFO] [scensim_isaac_world_server-1]: process has finished cleanly [pid 2898683]
```

The world server was respawned and ended the same way five times in 30 minutes.

## Cause

1. **Kit exits 0 when the world server's Python raises.** Isaac's `SimulationApp` prints the
   traceback, then shuts itself down "cleanly". So ros2 launch logs `process has finished
   cleanly`, not `process has died`.
2. **The start watch looked only for crash loops** (`scensim/crashloop.py`, 077 G77).
   - It counted a node's exits only from `process has died` lines (ros2 launch) and
     `Exited ...` lines (play_launch).
   - It gave up only after `LIMIT` (5) exits of one node.
   - A clean exit was not counted at all, under either launcher: ros2 launch's `process has
     finished cleanly` and play_launch's `Exited successfully` matched neither pattern. And at Isaac's pace, a world server
     that takes 5 minutes to fail needs 25 minutes to reach five exits, close to the limit
     anyway.

## Fix

While the stack is starting, **any exit of a stack node ends the start at once**, under either
launcher. That includes exit 0, because nothing in the stack is meant to end before it is
ready. The message names:

- the node;
- how it exited (`exited 0`, `exited 1`, `exited -11 (SIGSEGV)`);
- the last lines of its output, so an `ImportError` before Kit's clean shutdown is shown, not
  the shutdown;
- its log.

**The one exception is exit 75** (`EX_TEMPFAIL`). The CARLA world server exits 75 to be
restarted when a CARLA call hung or CARLA went away, which can happen while the stack starts
CARLA. Those exits are tolerated as before, up to `LIMIT` (5).

This covers both world servers (CARLA's and Isaac's), the bridge and the relay, which are the
stack's nested sides, and the nested launches themselves.

**Checked against past stacks.** Every stack record under 241's log root (2026-11-16 to 18) was
read with the new check. Its exits before ready were: the crash loops already caught (exit 139,
the import failure's exit 0), and exit 75 of the world servers and the bridge, still tolerated.
Exits after ready do not matter, since the watch ends at ready. `scensim run` never starts a
stack with a free drive or a scenario, the two nodes that may end by design.

## Test

- **Stub launch logs**, ros2 launch's and play_launch's: a world server that finishes cleanly
  during the start ends it at once, with its last lines. Exits after the stop's interruption do
  not count. An exit 75 is tolerated up to the limit.
- **`wait_started` with a stub stack** whose world server exits 0 at start: it fails in seconds,
  naming the server.
