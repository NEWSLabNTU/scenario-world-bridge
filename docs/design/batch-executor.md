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
| K2.3 | The compiled storyboard (conditions, actions) and the truth export (truth-stream.md, schema 2) (section 6) | in progress: the state machine, 10 conditions, SpeedAction and the exits, truth per slot; the rest executor pending |
| K2.4 | Agreement: each "batch" row of the subset on seats and in batch, on the same sample (Town01); each seats run records S, its frames before startNpcLogic, and the batch replays with `start_frames=S` (section 6, scenario time) | open |
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

- **The tag: v3.0.0** (e3f7d9ff), on Isaac Sim 6.2.0.0, which it pins (roadmap 095). Until
  then it was v3.0.0-EA (ae37b028) on Isaac Sim 6.1.0.0, which that tag pins; the script
  still takes it by `ISAACLAB_TAG` and `ISAACLAB_COMMIT`. 3.0.0's packages are a uv workspace
  whose members import one another (isaaclab imports isaaclab_ov): every member under
  `source/` is installed, and the dependencies are the root pyproject's and the members'.
  Watched: the isaacsim 6.2.0.0 wheel brings warp-lang 1.20.0.dev, Isaac Lab pins 1.17.0;
  the Isaac venv's is kept.
- **How it works.** It makes a venv of its own under `<dest>`, which sees the Isaac venv's
  packages through a `.pth` file. Isaac Lab's packages are installed editable from a checkout
  at the tag, without dependencies (under v3.0.0-EA: isaaclab, isaaclab_physx,
  isaaclab_newton, isaaclab_contrib, isaaclab_assets, isaaclab_tasks and isaaclab_rl; under
  v3.0.0 every workspace member, 14). Then it installs only the
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

## 5. The engine backends (K2.2): A, the PhysX vehicle; B, an articulation car

Two backends behind core.BatchRun's contract, named, not hidden (133, C):

- **A, `physx_vehicle`**: N copies of aib's PhysX vehicle, the same car, tire model and
  physics as seats (aib's `Vehicle.actuate`: the actuation from a component stepped outside
  it). It is K2.4's agreement path: a batch run on it can be checked against a seats run.
  Isaac Sim 6.1's physics tensor API has no vehicle view (articulation, rigid-body, deformable
  and contact views only), so its writes and wheel-state reads are per car: `vectorised =
  False`, and a throughput bench refuses it.
- **B, an articulation car**: a body of the spec (roadmap 083 `bodies:`, `isaaclab-articulation`,
  from `effective(spec, body)`), wheels as joints, stepped as tensors. Its tyre and contact model
  differ from PhysX's vehicle one, so its gap to A is a named body difference. It is the
  throughput path (K2.5), and the side comparable with WheeledLab (094), which also drives an
  articulation car; A is what seats fidelity costs.

**A measured** (139, 2026-11-21; contended: other users' jobs held the GPU at 96-98 % and 12.3
GB, so the timings are an upper bound, to measure again in a clean window). The smoke
(`scripts/smoke_physx_vehicle.py`): N cars on a ground plane, 150 N m on each wheel, 4 s at
240 Hz.

| N | pass | ms per step | per car | of it: actuate | PhysX step | sample |
|---|---|---|---|---|---|---|
| 1 | yes | 1.4 | 1381 µs | | | |
| 4 | yes | 4.6 | 1156 µs | | | |
| 16 | yes | 9.8 (10.4) | 612 (609) µs | 51 | 143 | 414 |
| 64 | yes | 38.1 | 596 (593) µs | 51 | 139 | 403 |

Pass means every car moved as one: the same distance to 0.08 mm (float32 at the grid's
offsets) and the same speed (3.267 m/s). Peak GPU memory, every user's: 15.4 GB of 32.
Reading the cars back (`Vehicle.sample`: a pose and a tensor-view velocity per car, four wheel
states) is two thirds of the cost. A rigid-body view over all the cars would batch the pose
and velocity; the wheel states stay per car. At about 0.6 ms per car, A is for agreement at
small N (K2.4), not for training.

**A is CPU physics by nature.** PhysX's vehicle constraints are not GPU-compatible: Isaac
Lab's direct-GPU pipeline refuses them ("non-GPU compatible PxConstraint") and the CPU velocity
setters A's reset uses. The environment (`env.py`) therefore runs A with `device="cpu"`, as
aib's runner does, with scene queries on. Engine A needs exactly two settings beyond Isaac Lab's defaults:

- `device="cpu"`: PhysX's vehicle constraints are not GPU-compatible (above);
- `enable_scene_query_support=True`: a PhysX vehicle finds the ground by raycasting each
  wheel, and Isaac Lab turns scene queries off by default. Without them the env smoke's
  steer (0.051/0.049 rad for 0.05) and wheel spin reached PhysX but every wheel was off the
  ground, tire force 0, and every ego coasted to rest on its body.

Fabric is not one of them: with scene queries on, `use_fabric` True and False gave the same
run to the last digit (egos 5-8 m/s to 7-10 m/s in 2 s, about 480 N per tire), so it stays
Isaac Lab's default. `scripts/smoke_env.py` asserts the read-back from PhysX (wheels on the
ground, front ones steered, tires pushing, egos sped up) and exits non-zero otherwise;
`--fabric on|off` overrides the setting. Only B runs on the GPU pipeline; 094's rate comparisons with stock
are B's.

**B, `articulation`** (K2.2b): the car from sim_vehicle_spec's neutral body file,
`generated/<car>/articulation[_<body>].yaml` (sim_vehicle_spec.articulation, on `effective(spec,
body)`; no simulator in it), which `scensim_batch.articulation` turns into a USD stage
(`write_usd`, pxr only) and an Isaac Lab `ArticulationCfg` (`articulation_cfg`: the joints'
drives as actuators, self-collisions off). The chassis hangs on four prismatic suspension joints;
the front corners steer on revolute joints; the wheels spin on revolute joints. The actuation is
the component's (`drive`, `brake` [N, 4], `tire_angle` [N]), as A takes it, written as joint
targets in one call per step (`vectorised = True`). `apply_joints` bypasses the component
(steer and spin targets straight to the joints: 094 P2's second measurement, and the only way
into a velocity spin drive, an RC car's).

What makes B a different car from A (and seats), each named here and in the body file's
`choices:`:

| | A (seats' PhysX vehicle) | B (articulation) |
|---|---|---|
| tyre | PhysX's tire model: slip curves, load sensitivity | a rigid cylinder in Coulomb contact, `tires.friction` per axle times the ground's; no slip curve, no load sensitivity |
| drivetrain | PhysX's, the component's torques per wheel | none: the component's torques straight on the spin joints |
| brake | PhysX's wheel brake | an explicit torque with stiction: full torque while the wheel turns, never reversing it, holding it at rest; what turns with the wheel (the wheel alone, or its share of the car when gripping) estimated from the step before |
| steer | PhysX's wheel steer angle, set | a stiff position drive per front wheel (the front tyre's largest scrub torque moves it 1 mrad), Ackermann angles from the bicycle angle |
| suspension | PhysX's sprung masses | a prismatic spring-damper per corner (the spec's ride frequency and damping), preloaded to rest at design height |
| masses | PhysX's vehicle | the chassis box plus 0.8/0.1/0.1 (rear 0.9/0.1) of `wheels.mass` per corner at the hubs; the whole car's centre of mass the spec's |
| physics | CPU only | CPU or GPU pipeline |

**B measured** (139, 2026-11-21; contended: other users held the GPU at 97 % and 11.5-14 GB, so
the timings are an upper bound). `scripts/smoke_articulation.py`, tesla_model3 (carla-0.9.16
body), ground friction 1.0, 240 Hz: 400 N m on each rear wheel for 4 s, then 1500 N m of brake
on every wheel to a stop, then a constant 0.15 rad bicycle angle at about 4 m/s for 8 s.

| device | N | straight: v after 4 s | brake: decel | steer (FL, FR): target / read back | circle radius / bicycle L/tan | ms per step |
|---|---|---|---|---|---|---|
| cpu | 1 | 4.794 m/s | 8.28 m/s² | 0.1565, 0.1440 / 0.1546, 0.1435 | 20.22 / 19.88 m | 0.62 |
| cuda:0 | 4 | 4.795 m/s (x 9.593-9.595 m) | 8.22 m/s² | 0.1565, 0.1440 / 0.1554, 0.1427 | 20.03 / 19.88 m (spread 0.03) | 45.8 |

Every check passes on both pipelines: the cars moved as one, straight (|y| < 1 mm), with the
driven wheels' torque read back from the actuators (400 N m) and rolling at v/r to 0.2 %; they
stopped; the steer joints reached their Ackermann targets to 2 mrad; and the turn is 1-2 %
wider than the bicycle's at 0.85 m/s² of lateral acceleration. The straight-line speed is the
torque's (800 N m at r 0.37 on 1760 kg and four wheels' inertia: 1.20 m/s², 4.80 m/s in 4 s).
Two defaults changed on the way, both from this smoke: the steer drive (a 10 Hz drive on the
knuckle's inertia let the outer wheel lag 0.037 rad: a 27 m circle for 20) and the brake (a
clamp to what stops the wheel alone left a slow tail near standstill: 4.75 m/s² for 9).

The GPU step is not a throughput number: N = 4 on a GPU at 97 % from other users, and the
backend reads the state back to the CPU every step (BatchRun's contract is CPU tensors). K2.5
measures B's throughput in an exclusive window, with the actuation and read-back kept on the
device.

## 6. The compiled storyboard and the truth export (K2.3)

`scensim_batch.storyboard` compiles a scenario's Story, Act, ManeuverGroup, Maneuver, Event and
Action elements into one state machine over N slots, every element's state an [N] tensor, and
ports SSv2's interpreter where it and the OpenSCENARIO text differ (its module docstring lists
each point):

- `StoryboardElement::evaluate`: an element's stop trigger first, then standby, start, running,
  end (back to standby while executions remain), complete, re-dispatched within one tick; a
  stop transition overrides and evaluates every child. An overriding Event overrides every
  event of its maneuver, itself included (its stopTransition fires), and restarts its actions.
  One difference no condition can see today: SSv2's ManeuverGroup::start sends every
  maneuver to its startTransition, complete ones included, so a group's second execution
  re-runs its (already complete) maneuvers, emitting their transitions and counting their
  executions; the batch leaves them complete. Its state-condition latches are one-way and
  set by the first execution, so nothing reads the difference (243).
- `Condition::update_condition`: each evaluation appends (time, value); the value used is the
  newest entry at least `delay` old; an edge compares it with the one before. A start trigger
  appends only while its element is in standby. The test checks the batched form against a
  line-by-line scalar port on random per-slot sequences, masks and delays.
- The rules: equalTo within machine epsilon, notEqualTo exact; standstill |v| <= epsilon; a
  lane-coordinate distance signed; a StoryboardElementStateCondition latched once seen.
- SpeedAction on SSv2's default vehicles, as `EntityBase::requestSpeedChange` does it: step
  sets target and speed; linear (cubic and sinusoidal map to linear in SSv2) sets the
  acceleration and the acceleration or deceleration limit to the rate until the speed passes
  the target, then resets the limits; a time dimension takes the rate (target - v) / T; a
  distance dimension sets only the target.
- The end: exitSuccess and exitFailure end their slot (Isaac Lab's `terminated`). A storyboard
  that completes or is stopped ends nothing, as in SSv2, where only the test runner's timeout
  ends such a run; the environment's episode length plays that role (`truncated`).

The storyboard ticks at reset (time 0) and after every step, on the state the step left.

**Scenario time** is SSv2's `SimulationClock::getCurrentScenarioTime`: A(S + k) - A(S), A the
step (`realtime_factor / frame_rate`) summed in a double from the node's start, S the frames
before `startNpcLogic`. The fork's `clock_source:=simulator` moves ROS time onto the world's
integer `sim_time_ns` and leaves scenario time as it is, so a SimulationTimeCondition on seats
reads the summed double. Which frame first passes a boundary depends on S (t > 1 at 0.05 s:
frame 20 or 21), and S differs between seats runs; `BatchRun(start_frames=S, step_time=dt)`
reproduces a given run's, and K2.4's seats comparison has to carry it from the run.

**The route goal.** Under `--ego policy` the ego's AcquirePositionAction and AssignRouteAction
(Init or story) are the policy's task, not actions to execute (133): the slot's
`obs["route"]` holds the route polyline (the world adapter's `route_through`, else the chords
through the waypoints), the goal pose, the ego's progress along it, the distance left and to
the goal, and `goal_reached` (within 1 m, latched) with its onset `goal_reached_now`. The ego's
FollowTrajectoryAction would drive the ego over the policy: refused (seats). Another entity's
routing executes, and is executor pending.

**Lane changes** (`storyboard.LaneChangeAction`, `npc.DefaultVehicles`) are SSv2's
LaneChangeAction under its FORCE policy, the only one the interpreter sets: the shape linear
is LINEAR and every other (cubic, sinusoidal, step) CUBIC; the dimension time, rate and
distance are TIME, LATERAL_VELOCITY and LONGITUDINAL_DISTANCE. At the action's start the curve
is `route::laneChangeTrajectory`'s: the pose 20 m along the lanelet (the distance for a
distance dimension), the target centreline's crossing between offsets +5 and -5 there, the
goal at that s and the target offset, a Hermite curve with tangents of half the start-to-goal
distance (CUBIC: the start heading and the target lanelet's spline tangent; LINEAR: the
chord), its length `HermiteCurve::getLength(100)` as written. The vehicle then moves at the
forced speed (length / T; length / (lateral offset / rate); the current speed), clamped by a
SpeedAction's target, along the curve at u = s / length, and past its end is put on the target
lanelet at the overshoot plus the target s, offset 0, with a route from there. The action is
accomplished as it starts, as in SSv2, whose check reads the behaviour tree's last tick, which
ran before the request. The ego's LaneChangeAction would steer the ego: refused under `--ego
policy`. The map queries are the world adapter's: `lanelet_pose(x, y, yaw)`,
`adjacent_lane(lane, direction, shift)`, `can_change_lane(from, to)`,
`along_lanelet(lane, s, distance)`, `collision_s(lane, p0, p1)` and `tangent(lane, s)`; on a
real map they must be SSv2's (its Catmull-Rom centreline spline, whose tangent is a raw
derivative, not a unit vector), which K2.4 checks on Town01.

**In Isaac Lab** (`env.py`) the step's world is made in `_get_dones`, the first hook after the
physics in DirectRLEnv.step (dones, rewards, resets, observations, in both Isaac Lab 2 and
3.0.0-EA): the last substep read back, the default vehicles moved, the storyboard ticked; so
dones, rewards and observations read the same step, and `BatchRun.reset` merges a reset slot's
observation in without touching the others'. Users override `rewards()` and `dones()`. Ten conditions, SpeedAction and the exit commands
run; every other batch row is executor pending (compilable-subset.md section 8), which the
checker and `compile` both name.

**Truth** (`scensim_batch.truth.TruthRecorder`): every step's entity states per slot, written
per slot and episode as `slot_<k>_ep<e>/truth.jsonl`, a schema-2 truth file that
`scensim_launch/scripts/gap_trace.py` reads, with `sample.json` (the slot's record: the replay
on seats) and `outcome.json` (how the episode ended, the storyboard's state, the seed). The
ego's values are the engine's (`measured`); the default vehicles' and props' are the batch's
(`derived`, `kinematic`); contacts are the ego's box overlaps at onset, impulse unknown.

Measured on CS2 at two samples (onset 60 m at 6.5 m/s^2, 90 m at 3 m/s^2, point-mass engine):
each lead brakes in the tick after it is within 1 m of its own onset, never beyond its rate,
stops at exactly 0 m/s, and gets SSv2's default deceleration limit back; the stop trigger
completes both storyboards at 30 s.

