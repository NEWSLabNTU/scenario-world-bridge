# 042: The Isaac world server deadlocks on the GIL while PhysX cooks the road's rocks

**Severity**: high (every stack on a `road_detail.yaml` with rocks hangs at its start, seen
2 of 2 on 241; the stack never becomes ready, and nothing kills the world server).
**Status**: open (diagnosed; fix not chosen). **Found**: 2026-11-17, swb 241, roadmap 077
item 21 (`scenarios/projects/isaac_rough_road`: Town01 with the stages dataset's
`procedural/spikes/road_town/road_detail.yaml`, Autoware ego), swb 2bf9f30, swb_isaac_world
1c04b37, Isaac Sim 6.1.0.0 (Kit 110.3.0), at load 70-100.

## What happens

The world server's preload spawns its warm-up ego, tops up the rock pool, and then logs
nothing more:

```
INFO swb_isaac_world: spawned VEHICLE 'warm-up' as /World/entities/e0 (... microlino, tracked, 513 kg)
INFO swb_isaac_world: rock pool: 992 prims for 1 vehicle(s) in 20019 ms (992 in all)
[Warning] [omni.fabric.plugin] Attempting to set an invalid data source to array attribute
[Warning] [omni.physx.plugin] Collisions are supported currently only in one collision group.   (x 992)
```

The process sleeps (state `S`) and holds no lock it will release. On the first run the stack
waited all of `swb run --wait 3600`; on the repro, 300 s of silence and then a gdb dump.

## Why

From `thread apply all bt` of the world server (`/home/aeon/coop-c3/i21/repro.0344/` on 241):

- **Thread 1, the main thread, holds the GIL.** It is in the physics step that Python called
  (`_physics` binding, then `omni.physics`, `omni.physx`), inside
  `libomni.physx.cooking.plugin.so`, then `libcarb.ujitsoagent.plugin.so`, waiting in
  `carb.tasking` on a cooking task. Those are the rock pool's convex hulls, cooked through
  UJITSO, Kit's cook cache.
- **Thread `carb.tasking13` runs that task and waits for the GIL.** Its stack is the cooking
  task (`ujitsoagent`, then `omni.physx.cooking`) logging a message: `libcarb.so`, then
  `_log.cpython-312` (omni.log's Python binding), then `PyEval_AcquireThread`, then
  `take_gil`. A Python omni.log message consumer is registered, so every Kit log message is
  handed to Python, and a message from a worker thread needs the GIL.

The consumer is the world server's own `fatal_consumer` (`world_server.py`, issue 011: exit
75 on a Kit FATAL), registered before the preload. So the main thread waits for the task, the
task waits for the GIL, and the GIL is held by the main thread: a deadlock. The rocks expose
it because they are the first thing cooked on a worker thread with a message to log, about a
thousand of them in one step. The same can happen mid-drive whenever a rock beyond the pool
gets a live prim.

### Why nothing killed it (133's questions)

1. **The G70 watchdog was not running yet.** `swb_world_server.run()` creates the
   out-of-process child (`StepWatchdog`) and arms it per request and per loop pass. The
   Isaac world server calls `run()` only after the preload, so a hang during the preload's
   warm-up is not watched by it. The preload has only `startup_watchdog`, a
   `threading.Timer` of 1200 s (`STARTUP_LIMIT`) inside the process, and its callback
   needs the GIL. The deadlocked main thread holds the GIL, so the timer never runs. This is
   the in-process limit G70 was made to avoid, left in place at startup.
2. **The stack waited 3600 s because the run asked it to.** `--wait 3600` was 241's
   runner's choice: Kit took over 1170 s to start from a spinning disk before the NVMe move.
   `swb run`'s default `--wait` is 0, so a run without it refuses at once. `--ready-timeout`
   covers the pilots' readiness, and the pilot was ready, so it did not apply. The stack has
   no ready deadline of its own for its world server.

### Not a relative of 040

The parked rocks all sit at (0, 0, -1000), and that is also `bury_point(0)`. But a parked
rock has its rigid body and its collision disabled (`RockStream._park`). No despawn had
happened yet, and the blocked frames are cooking, not contact solving. Nothing overlaps.

## Fix (to choose)

- Take Python out of omni.log's path: drop the Python `fatal_consumer`, and watch for a Kit
  FATAL from outside the process (the G70 child reading the world server's log, or the
  supervisor). Or register it only where no worker thread logs, which cannot be guaranteed.
- Start the G70 child before Kit's start and the preload, with `STARTUP_LIMIT` as the
  deadline of a "startup" block, so a hang there ends the process (exit for a fresh one)
  whatever holds the GIL. The in-process `startup_watchdog` then goes.
The first removes the deadlock. The second makes any startup hang end in at most
1200 s instead of never.
