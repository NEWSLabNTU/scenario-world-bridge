# The batch executor (roadmap 092 K2)

The batch executor runs a scenario in the compilable subset ([compilable-subset.md](
compilable-subset.md)) over N slots at once on an Isaac Lab environment, for training and
evaluating learned policies ([ml-training-interface.md](ml-training-interface.md) sections 5
and 5a). It lives in `src/scensim_batch`. This note grows with its milestones:

| Milestone | What | State |
|---|---|---|
| K2.0 | Isaac Lab v3.0.0-EA as an overlay on Isaac Sim 6.1 (section 3); its Cartpole benchmark on 139 as a 5a reference row | installed; benchmark waits for a free GPU |
| K2.1 | SSv2's default vehicle behaviour, batched in torch (section 2) | in progress |
| K2.2 | The environment: a DirectRLEnv from a compiled scenario and sample records, the ego from aib's rig, the control component through `step_batch`, resets per slot | open |
| K2.3 | The compiled storyboard (conditions, actions) and the truth export (truth-stream.md, schema 2) | open |
| K2.4 | Agreement: each "batch" row of the subset on seats and in batch, on the same sample (Town01) | open |
| K2.5 | `scensim bench`: throughput at N = 256 and 1024 against a hand-written Isaac Lab environment (the RoboRacer indoor field) | open |

## 1. Shape

- **A DirectRLEnv**, not a manager-based one: the storyboard is a state machine, not reward or
  termination terms, and the direct form gives the adapter its own per-slot reset and
  pre-physics step.
- **Compiled, or refused.** The environment is built only from a scenario that
  `scensim-sample compilable` accepts, and only when the environment meets every condition the
  verdict lists (a camera, a fog renderer, a batched model).
- **The ego** comes from aib's rig (the spec's car, the same rig seats run), and its control
  component runs through `sim_vehicle_ecu.batch.load_batch` with `require_vectorised`.
- **Truth** is the content of truth-stream.md's schema 2 as per-slot tensors. Each slot keeps
  its sample record and seed, and exports as a schema-2 file, so any slot replays on seats.
- **Rewards and terminations** are the user's. The environment gives events: contacts, the
  storyboard's end (`exitSuccess`, `exitFailure`) and route progress.

## 2. SSv2's default vehicle behaviour (K2.1)

A vehicle with no controller in SSv2 runs behavior_tree_plugin's vehicle tree
(`vehicle_entity_behavior.xml`):

```
Fallback root
  LaneChange
  FollowPolylineTrajectory
  Fallback follow_lane_sequence
    FollowLane
    Fallback: FollowFrontEntity, StopAtTrafficLight, StopAtStopLine, StopAtCrossingEntity,
              Yield, MoveBackward
```

`scensim_batch.npc` ports the part every SSv2 example's traffic uses: FollowLane and
FollowFrontEntity, on SSv2's jerk-limited speed planner.

| SSv2 | Port |
|---|---|
| `LongitudinalSpeedPlanner::getDynamicStates`, `planLinearJerk`, `forward`, `timeDerivative` | `dynamic_states`, `plan_linear_jerk`: the same operations in the same order, per element |
| `getRunningDistance` (the stop distance) | `running_distance`: the same loop, masked per vehicle until every one reaches its target |
| FollowLane: follow the front entity when it is within the stop distance + own length + 5 m | the same test, `front_entity_stop_margin` 5 m |
| FollowFrontEntity: 0 inside the stop distance, the front's speed + 2 m/s beyond stop + length + 5 m, the front's speed between, clamped to [0, target]; the target when it is not above the front's speed | the same rule (`front_entity_margin` 5 m, `speed_step` 2 m/s) |
| the front entity: within ±90° of the heading, on the path (its box within half the own width of the spline), ahead, within the horizon clamp(5 v, 20, 50) m or the stop distance; its distance is the bumper gap plus half the own length (`splineDistanceToBoundingBox`) | the same tests, on the route's centreline |
| the move: arc length advanced by the mean of the old and new speed times the step (`calculateUpdatedEntityStatus`) | the same |
| the target: the last SpeedAction's, else the route's speed limit | `set_target_speed`, else `speed_limit` |

Two differences are by construction:

- **The path is piecewise linear** through the route's centreline points, where SSv2 uses a
  Catmull-Rom spline through the same points. Poses and headings differ by the spline's
  deviation from the chords. K2.4's agreement measures it.
- **The front entity's box is projected per corner** onto the path, where SSv2 measures the
  box's distance to the spline by bisection (`distanceToSpline`). Like SSv2, a box counts as
  ahead when a corner is within half the own width of the path, or else when the path crosses
  the box (SSv2's `getCollisionPointIn2D` fallback): a truck wider than the car, centred on its
  path, is seen. A box already overlapping the bumper (a negative gap) is no front entity, in
  SSv2 too (`splineDistanceToBoundingBox` returns none); its contact is the collision's.
  Both cases are pinned by a test against the reference.

Not ported yet: the tree's StopAtTrafficLight, StopAtStopLine, StopAtCrossingEntity, Yield and
MoveBackward, FollowLane's right-of-way check, LaneChange and FollowPolylineTrajectory. The
compile step refuses a scenario whose default-behaviour vehicles would need one of them (a
route over a stop line or an active signal, a crossing, a lane change): these are the gaps,
each a later row.

SSv2's planner exceeds its own jerk limit on arrival: it caps the acceleration at
(target − v)/dt, so the step that reaches the target drops the acceleration at once (45 m/s³
in the test at a 0.05 s step). The port keeps this.

**Tests** (`src/scensim_batch/test`): SSv2's logic transcribed for one vehicle in plain floats
(`ssv2_ref.py`).

- The planner equals it bit for bit over 400 random states.
- The stop distance equals it to 1e-9.
- 64 slots of a follower behind a lead equal 64 scalar runs to 1e-6 over 300 steps.
- An entity counts only when it is ahead, in the cone and on the path.
- Vehicles in one slot see each other.

All pass in the system Python (torch 2.0) and in Isaac's (torch 2.11).

**Cost, CPU** (8 threads, 139 under load 8 to 12): 4 vehicles per slot plus the ego take
19.8 ms per step at N = 256 and 86 ms at N = 1024, about 50 k vehicle-steps per second. The
stop-distance loop and the corner projection (slots × vehicles × entities × 4 corners × route
segments) dominate. The GPU number waits for an announced bench window. A closed-form stop
distance would remove the loop: the distance from (v, a) to 0 is fixed per constraint set, so
a table over (v, a), or computing it only when a front candidate lies within the horizon,
bounds it (a gap, for K2.5).

**A route ends.** A vehicle's route is a fixed length of centreline from where Init places it
(`BatchRun(route_length=)`, 300 m by default). Past its end the vehicle holds the end pose, and
entities beyond it project onto the end, where SSv2 would continue along the following
lanelets. The world adapter's routes must cover the episode (K2.3).

## 3. Isaac Lab as an overlay (K2.0)

`scripts/isaaclab_overlay.sh <isaac-venv> <dest>` installs Isaac Lab beside an Isaac Sim pip
install without changing it.

- **The tag: v3.0.0-EA** (ae37b028, 2026-09-16), chosen on purpose. v3.0.0 GA (2026-10-19)
  pins `isaacsim==6.2.0.0`. The host's Isaac Sim is 6.1.0.0, which v3.0.0-EA pins, with torch
  2.11.0, the version that install has. Follow-up: move to GA when the Isaac Sim install moves
  to 6.2 (092's K2 row).
- **How it works.** It makes a venv of its own under `<dest>`, which sees the Isaac venv's
  packages through a `.pth` file. Isaac Lab's packages are installed editable from a checkout
  at the tag, without dependencies: isaaclab, isaaclab_physx, isaaclab_newton,
  isaaclab_contrib, isaaclab_assets, isaaclab_tasks and isaaclab_rl. Then it installs only the
  third-party packages that the Isaac venv lacks, constrained to the Isaac venv's own versions
  (27 on 139: gymnasium, rsl-rl-lib, hydra-core, onnx, transformers, open3d, and others).
- **What it leaves alone.** torch, isaacsim and warp stay the Isaac venv's. Its warp is
  1.19.0.dev, where Isaac Lab asks for 1.16.0; Isaac Sim's own wins, and the benchmark will show
  whether that matters.
- **Measured on 139** (2026-11-21):
  - the overlay is 1.9 GB, plus 132 MB of source, and / went from 90 to 88 GB free;
  - the Isaac venv's `pip freeze` is identical before and after;
  - `pip check` reports only the Isaac venv's own two pre-existing lines;
  - `isaaclab`, `isaaclab_tasks`, `isaaclab_rl` and `DirectRLEnv` import, with torch
    2.11.0+cu128 and warp 1.19.0.dev.
- **Running it** needs `OMNI_KIT_ACCEPT_EULA=YES` and the NVIDIA Vulkan ICD, as aib's
  `isaac_env.sh` sets for the same install.

