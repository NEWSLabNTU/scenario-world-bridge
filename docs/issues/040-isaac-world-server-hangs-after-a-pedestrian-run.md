# 040: The Isaac world server hangs at the first step after a pedestrian run's world reload

**Severity**: high (on Isaac, every run after T3's `pedestrian_crossing` on the same stack
fails, and before 077 G70 the stack stayed up and hung). **Status**: fixed in swb_isaac_world
`ce2aac3` (each despawned entity buried at its own point, its colliders off); verified on 241
(below). **Found**: 2026-11-16, swb 241, walking tutorial 3's "On Isaac Sim" for
roadmap 077's release criterion (b): swb 2788710, swb_isaac_world 8d63f0b, Isaac Sim 6.1.0.0,
Town01 (isaac_first_drive's project). Reproduced on server139 the same day (swb 2e14590,
swb_isaac_world f2d7e1a, with G70's watchdog).

## What happens

On one Isaac stack, `pedestrian_crossing.xosc` (tutorial 3 step 1: the ego, a walker on a
timed path and two `static.prop.streetbarrier` obstacles) runs and ends. The next run on the
same stack, whatever it is, loads the world again (the same Town01, kept: `_load_env`'s
same-world branch) and stops at its first frame:

```
error    ..._red_light_r0  SimulationError: The simulator reported a failed frame (UpdateEntityStatus,
         UpdateTrafficLights or UpdateFrame); see its log for the cause. Ending the scenario instead of
         retrying a frame that cannot advance time.
```

The world server's last lines are the reload and one PhysX warning:

```
INFO swb_world_server: world .../assets/carla-usd/Town01/Town01_physics.usda loaded, episode 1
[Warning] [omni.physx.plugin] PhysX warning: Filtering: Resolving contacts between two kinematic
          objects is invalid. Contacts will not get resolved., FILE .../simulationcontroller/src/ScFilter...
```

and then nothing: its main thread blocks in a PhysX step for as long as it is left (6+ min,
twice), at 50-60 % CPU. The bridge logs, every minute:

```
WARN scenario_world_bridge::backend::remote: World server at tcp://127.0.0.1:5591 not ready
     (no answer within 2 s); waiting
```

and every later run ends `No response from the simulator at tcp://localhost:5555 within 300 s
(SIMULATOR_INITIALIZE_TIMEOUT)`. Before G70, nothing ended the hung world server: its start
watchdog (issue 011: exit 75 when not up in 20 min) does not fire once it was up, and the
stack does not respawn it.

Seen 3 of 3 on 241 (stacks 2026-11-16T17-38-13 and 17-58-07, each pedestrian_crossing then
red_light, then autopilot; 19-02-25 on G70's first cut, whose Step-only deadline armed only
`_step`: no deadline line and no dump in 23 min), and 1 of 1 on 139 (below). On a fresh
stack with no pedestrian run first, red_light **passed** (241, 18-20-44), and its reload for
the next run (autopilot) was fine. Earlier Isaac stacks reloaded the world between runs
without a walker or a prop (075 T6: drive_to_goal, then follow_to_goal with a lead car; 077
T2) and never hung.

## Backtrace

The world server's main thread, gdb at both of 241's first hangs (the same both times;
`/home/aeon/coop-c3/t3walk/gdb.txt`, `gdb2.txt` on 241):

```
#0  syscall () at ../sysdeps/unix/sysv/linux/x86_64/syscall.S:38       (futex wait)
#1  .. #4   kit/kernel/plugins/libcarb.tasking.plugin.so
#5  .. #7   extscache/omni.physx-110.3.2+110.3.0.lx64.r.cp312.u7f4/bin/libomni.physx.plugin.so
#8          extscache/omni.physics-110.3.2+110.3.0.lx64.r.cp312.u7f4/bin/libPhysicsUmbrella.so
#9  .. #10  omni/physics/core/bindings/_physics.cpython-312-x86_64-linux-gnu.so
#11         cfunction_call ()   (Python: the world server's step)
#12 .. #20  _PyEval_EvalFrameDefault, PyEval_EvalCode, builtin_exec, pymain_run_module, main
```

PhysX's step waits on a carb task that never completes.

139's run, with G70's watchdog (stack 2026-11-16T21-22-34_seat0, isaac_t3: isaac_first_drive
plus first_drive's pedestrian_crossing and red_light; the world server's `err.1`):

```
2026-11-16 21:31:51 ERROR swb_world_server.watchdog: step 3 (3 x 0.0166667 s at sim time 0.100 s)
    has run 180.1 s, past its deadline of 180 s: ending world server 49536 for a fresh one
Current thread 0x000073b0d21d5740 (most recent call first):
  File ".../isaacsim/core/api/physics_context/physics_context.py", line 582 in _step
  File ".../isaacsim/core/api/simulation_context/simulation_context.py", line 743 in step
  File ".../isaacsim/core/api/world/world.py", line 568 in step
  File ".../swb_isaac_world/swb_isaac_world/world.py", line 1226 in step
  File ".../swb_world_server/swb_world_server/server.py", line 306 in _step
  ...
```

The reload came at 21:28:51 (pedestrian_crossing's retry), with the same PhysX warning at the
same second (`out.1`). py-spy is not installed on 139, so there is no native dump; the Python
frames meet 241's native ones at the binding. The world server was killed (signal 9) and
respawned 15 s later.

## What the reload does (swb_isaac_world 8d63f0b to f2d7e1a)

`World._load_env` despawns every entity first. `despawn` deletes an ego at once, but any
other entity "goes under the world (kinematic, if it was tracked) for one step, and is
deleted after it (`_bury`)" (044 step 7, so the RTX lidar forgets it): its translate is set
to `(0, 0, BURY_Z)`, `BURY_Z = -1000`, the same point for every entity. The scene has
`reportKinematicKinematicPairs` and `reportKinematicStaticPairs` on. After a pedestrian run,
the walker (a kinematic box) and both barriers are buried at that one point together, so the
next step has overlapping kinematic bodies: the "two kinematic objects" warning. Earlier
reloads buried at most one entity (075 T6's lead car), so no pair. A mid-run despawn of two
entities in one frame put them at the one point too.

## The fix

`despawn` buries each non-ego entity at its own point, `motion.bury_point(i)`: a grid under
the world (`BURY_Z`), 100 m between neighbours, so no two buried bodies overlap; and it turns
every collider at or under the entity off (`physics:collisionEnabled = false`) before the
step, so a body on its way out takes part in no contact pair. The one-step bury stays, for
the RTX lidar (044 step 7). `test_motion.BuryPoints` checks the points (distinct, at least
100 m apart, under the world, stable per index) without Isaac.

## Recovery after a respawn (a second finding)

On 139's stack, after G70's kill and the respawn, the suite did not recover:

- red_light r0 had started while the server hung and ended at SSv2's 300 s
  `SIMULATOR_INITIALIZE_TIMEOUT`.
- The ego pilot (`lifetime: stack`) ended with the episode (21:33:50) and restarted
  (21:33:56), before the fresh world server was ready (21:35:33, preload 110 s, sensors warm).
  red_light's retry (21:34:53) and drive_to_goal r0 (21:45:16) then ended `pilot ego never
  became ready (600 s)`: "planning is up but perception is not -- nothing publishes
  .../clustering/clusters, .../pointcloud_map_filtered/pointcloud". The pilot's
  euclidean_cluster and voxel_based_compare_map_filter had loaded (21:36:45); those filters
  publish only on input, and when checked against world server #3 nothing published
  `/sensing/lidar/top/pointcloud_raw_ex` on the ego domain (2) and no world-server node was on
  the graph.
- World servers #2 (up 1428 s) and #3 (up 260 s, mid-preload) were killed by signal 9 with no
  watchdog line (play_launch: `signal: Some(9)`, `oom_killed: None`). The host had 19-31 GB
  available, another user's vLLM server started at about 21:49, and the kernel's OOM counter
  stood at 13 (the kernel log is not readable from the run's account): most likely OOM kills
  under other users' memory, not swb's.

Whether the lidar reaches a restarted pilot after a respawn on a host with memory headroom is
open; it is not on the release path once the hang itself is gone.

Logs (139): `/var/tmp/aeon/swb-logs/isaac_t3/stacks/2026-11-16T21-22-34_seat0/proc/simulator/
2026-11-16_21-22-35/` (`node/swb_isaac_world/{out,err}.{1,2,3}`, `play_launch.log`), runs
under `/var/tmp/aeon/swb-logs/isaac_t3/runs/`.

## Diagnosis

The split on 241 (swb b844a24; each leg a fresh stack: a cut-down pedestrian_crossing, then
drive_to_goal on the same stack, whose LoadWorld is the reload):

- **Walker only** (the ego and pedestrian_0; stack 2026-11-16T22-25-36): no hang. At the
  reload (22:30:43 `loaded, episode 1`) no "two kinematic objects" warning; the next ego spawned
  (sensors warm in 1.2 s) and the world server stepped on.
- **Barriers only** (the ego, barrier_0 and barrier_1, `static.prop.streetbarrier`; stack
  2026-11-16T22-33-18): hang. At the reload (22:38:10.209 `loaded, episode 1`) the warning
  `Filtering: Resolving contacts between two kinematic objects is invalid`, then nothing more in
  the world server's log; drive_to_goal ended `The simulator reported a failed frame`. (The leg
  was stopped 63 s into the block, before G70's 180 s deadline.)

One body buried at the shared point makes no pair; two kinematic bodies buried there together
make a kinematic-kinematic pair, PhysX's warning, and the step that never returns. The two
barriers alone are enough: the walker is not needed. Not tested: one barrier alone, and the
walker with one barrier.

## Verified

On 241 (swb b844a24, swb_isaac_world f2d7e1a with the fix applied; `test_motion` 8 passed), at
every reload no "two kinematic objects" warning, no G70 deadline line and no respawn (no `.1`
files in either stack's world-server node directory), and the next run's ego spawned and
stepped:

- **Barriers only, then drive_to_goal** (stack 2026-11-16T22-52-20, load 76): the reload at
  22:58:35.039 with no warning, the ego spawned at 22:58:39.884, and drive_to_goal **passed**:
  the exact pair that hung in 22-33-18.
- **pedestrian_crossing, red_light, drive_to_goal** (stack 2026-11-16T23-02-34, load 115): the
  walker and both barriers spawned in the first run; the reloads at 23:07:56.664 and 23:09:55
  had no warning and the ego spawned after each. The three verdicts were `error`, all
  Autoware's on a host at load 115 (INITIALIZING; `change_to_stop` not reachable); the world
  server answered throughout.

139 could not repeat it the same evening: another user's model server held 29.5 of 32 GB of
the GPU, and Isaac logged `Rendering failed` until issue 011's start watchdog ended it.
