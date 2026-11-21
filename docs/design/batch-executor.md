# The batch executor (roadmap 092 K2)

The batch executor runs a scenario in the compilable subset ([compilable-subset.md](
compilable-subset.md)) over N slots at once on an Isaac Lab environment, for training and
evaluating learned policies ([ml-training-interface.md](ml-training-interface.md) sections 5
and 5a). It lives in `src/scensim_batch`. This note grows with its milestones:

| Milestone | What | State |
|---|---|---|
| K2.0 | Isaac Lab 3.0.0 installed beside Isaac Sim 6.1 (an overlay venv in /var/tmp/aeon), its Cartpole benchmark on 139 as a 5a reference row | open |
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
  box's distance to the spline by bisection. They agree for boxes aligned with the lane.

Not ported yet, so a route with them is refused by the compile step: traffic lights and stop
lines, crossing and conflicting entities, yield and right of way, lane changes, moving
backward, and trajectories. Each is a later row.

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
distance would remove the loop.
