# 042: The Isaac world server deadlocks on the GIL while PhysX cooks the road's rocks

**Severity**: high (every stack on a `road_detail.yaml` with rocks hangs at its start, seen
2 of 2 on 241; the stack never becomes ready, and nothing kills the world server).
**Status**: fixed in swb_world_protocol `6f3aba5` and swb_isaac_world `5d66ba5`; verified on 241 (below). **Found**: 2026-11-17, swb 241, roadmap 077
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
   needs the GIL. The deadlocked main thread holds the GIL, so the timer never runs: a timer
   that needs the GIL cannot watch a thread that holds the GIL. This is
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

## Fix

Both guards of issue 011 move out of the world server's process, into G70's watchdog child
(swb_world_protocol `6f3aba5`, swb_isaac_world: see the status line):

- **No Python on Kit's log path (the root fix).** The Python `fatal_consumer` is gone. Kit's
  own logger, in C, writes every message to its log file (`/log/file`, tokens resolved; the
  cooking task's `[Info] [omni.physx.cooking.plugin] ConvexMeshCookingTask: polygon limit
  reached` is its last line in the hung runs). The world server tells the child that path
  (`StepWatchdog.watch_log`), and the child reads the file as it grows. At a line holding
  `] [Fatal] [` it ends the world server as a deadline does: stacks dumped, then SIGKILL, and
  the launch respawns it. Before, the process exited 75; nothing reads that code, and the
  respawn takes any exit.
- **The start watched from outside (the safety net).** The world server starts the child
  before Kit. Kit's start and the preload are one block, "Kit's start and the preload", with
  `STARTUP_LIMIT` (1200 s) as its deadline. `run(watchdog=)` then takes the same child for
  the requests' deadlines. The in-process `threading.Timer` (`startup_watchdog`) is gone.

Tests: swb_world_protocol's `test_server.py` checks that a fatal line in a watched log ends
the watch, that other lines (and a fatal line still without its newline) do not, and that a
fatal line kills a world server whose main thread holds the GIL (exit -9).
swb_isaac_world's `test_world_server.py` keeps 011's two behaviours (a Kit FATAL line ends
the world server, and so does a start past its limit). It also checks that `world_server.py`
registers no omni.log consumer and holds no timer, and that the start block is armed before
Kit is imported and closed after the preload.

### Verified

On 241, 2026-11-17, swb 5ea2554 with swb_world_protocol `6f3aba5` in both checkouts
(`src/` and swb_isaac_world's `deps/`), item 21's project, two stack starts in a row, each
running `suites/rough.yaml` (`--start-stack --stop-stack --wait 1500`). Both had hung before
the fix (2 of 2):

| start | load | rock pool | world server up | drive_to_goal |
|---|---|---|---|---|
| 1 | 53 | 992 prims, 3.9 s | 117.7 s | passed |
| 2 | 120 | 992 prims, 3.4 s | 98.2 s | passed |

Each world server logged `Kit's log .../kit_<time>.log: a FATAL line in it ends this world
server` before the preload, and `deadlines: 180 s, world load 900 s` once ready. The second
start also took the first run's `state/Town01/road_detail.yaml`, a link into the dataset
cache made before issue 043's fix, as swb's own.

Nothing regressed on a world without rocks: isaac_basic's `suites/towns.yaml` on the same
code, town01_ego_drive and town02_ego_drive on one stack (Town02 preloaded, the pilots
moved to it in 9.6 s), passed 2 of 2.
