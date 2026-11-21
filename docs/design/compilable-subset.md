# The compilable subset of OpenSCENARIO

Roadmap 092 S2. This note says which OpenSCENARIO constructs a batch executor can run per slot.
A batch executor is an Isaac Lab environment, say (092 K2), stepping N copies of a scenario at
once. Every other construct runs only on seats, which means SSv2 with a simulator. This note
replaces the draft table in [scenario-description.md](scenario-description.md) section 6. The
table is `scensim_sampler/subset.py`, and the checker is `scensim-sample compilable`.

## 1. The classes

| Class | Meaning |
|---|---|
| batch | compiles per slot |
| derived | compiles once rewritten; the refusal names the rewrite |
| conditional | compiles when the slot provides something (a camera and a renderer for weather, a batched form of a behaviour model); the verdict lists each condition |
| seats | never batches; the refusal gives the reason |
| unimplemented | SSv2 does not implement it, so it runs on neither: no seats run executes it, and no batch run of it could be checked against one (K2's acceptance) |

An element the table does not list is refused as "not in the compilable subset". The table
grows by decision, not by default. Containers (Story, Act, ManeuverGroup, Event,
ConditionGroup, Position, the dynamics and target elements) compile when their contents do.

**"batch" is a claim.** Each row stays a claim until K2's adapter runs the construct and
compares it against seats on the same sample. That comparison, row by row, is K2's acceptance.
A batch row the executor does not run yet is **executor pending** (section 8): the checker
lists it as a condition the batch does not meet, so it never says "compiles" for what the
executor refuses.

## 2. The table

**batch**

| Constructs | How |
|---|---|
| TeleportAction; WorldPosition, LanePosition, RelativeWorldPosition, RelativeObjectPosition | a per-slot reset pose; lane positions resolved on the world's map once per slot |
| SpeedAction (step, linear, cubic; absolute or relative target), SpeedProfileAction | a speed profile per slot |
| LaneChangeAction | a lateral profile per slot |
| FollowTrajectoryAction, AssignRouteAction, AcquirePositionAction | precomputed per slot on the world's map |
| RoadCondition; the world knob `world.surface` | friction as a per-slot material |
| the world knob `world.zone_layout`; a MiscObject | a prop set, instanced per slot |
| Distance, RelativeDistance, RelativeClearance, ReachPosition, SimulationTime, TimeHeadway, Speed, RelativeSpeed, Acceleration, and StandStill conditions | vectorised per slot |
| CollisionCondition | contact per slot |
| UserDefinedValueCondition `RelativeHeadingCondition(<entity>)` or `(<entity>, <lane>, <s>)`, matched by the function-call form | SSv2 computes it from the entity's pose against its lane, nothing from outside: a heading per slot |
| StoryboardElementStateCondition; any number of Stories, Acts, Events and ConditionGroups; `maximumExecutionCount`; StopTrigger | the storyboard's state machine per slot, as masks |
| ParameterCondition; a ParameterAction in Init | a parameter fixed for the slot |
| TrafficSignalStateAction, TrafficSignalControllerAction | a per-slot signal state |
| CustomCommandAction `exitSuccess`, `exitFailure`, `:` | the slot's end |
| CustomCommandAction `scensim.fault` | a tensor mask over the sensor for the duration (CS4) |

**derived**

| Construct | Rewrite |
|---|---|
| TimeToCollisionCondition | a DistanceCondition on a parameter the sampler derives (speed × time to collision). SSv2 implements it only in part, and derived values are the sampler's job, not the trigger's (CS1) |

**conditional**

| Constructs | Compiles only if |
|---|---|
| TimeOfDay, Sun, DomeFile, DomeImage; the knob `world.illumination` | the slot has a camera; costs render time |
| Fog; the knobs `world.fog`, `world.weather` | the slot has a camera, and the renderer supports it per slot |
| Precipitation | the slot has a camera or lidar model with precipitation per slot |
| Wind | a wind force model per slot |
| an ObjectController with `model = scensim.behaviour:<name>` | the model has a batched form |
| a vehicle or pedestrian with no controller (SSv2's default) | a batched model of SSv2's default behaviour: its behaviour tree follows the lane at the target speed its SpeedActions set, and stops for an entity ahead |
| any other `world.<knob>` | the world supports the knob per slot |

**unimplemented** (SSv2's support table says so; refused by name)

| Constructs |
|---|
| LaneOffsetAction (and its LaneOffsetActionDynamics, LaneOffsetTarget, AbsoluteTargetLaneOffset, RelativeTargetLaneOffset); RelativeLanePosition; TraveledDistanceCondition |

These three were batch rows until 243's D7 review (093) found LaneOffsetAction's children
refused; checking every row against SSv2's table then found all three unimplemented there. The
check is now a rule: a batch or conditional row must be a construct SSv2 runs.

**seats**

| Constructs | Why |
|---|---|
| AddEntityAction, DeleteEntityAction | a slot has a fixed set of entities |
| ParameterAction, ParameterSetAction, ParameterModifyAction after Init | a slot's parameters are fixed |
| ControllerAction, AssignControllerAction, ActivateControllerAction, OverrideControllerValueAction (and its Brake, Throttle, Clutch, Gear, ParkingBrake, SteeringWheel) after Init | they swap or override a pilot mid-run |
| VisibilityAction | it changes what an external pilot's sensors see |
| UserDefinedValueCondition naming an autopilot's state (`<entity>.currentState`, `.currentEmergencyState`, `.currentMinimumRiskManeuverState.*`, `.currentTurnIndicatorsState`) or a ROS topic (`/path`) | it reads a value from outside the scenario |
| every other CustomCommandAction (SSv2's FaultInjectionAction, V2ITrafficSignalStateAction, RequestToCooperateCommandAction, a shell command) | it runs as a process on seats |
| a controller `agent`, or a `pilot` property, on a non-ego entity | an external pilot does not run in a batch |
| more than one ego | a batch has one policy per slot |

## 3. The ego

In a batch, the ego is the policy being trained or evaluated. So an ego is accepted as the
batch's policy slot. An ego is an entity whose controller is named `Autoware` or has `isEgo =
true`, in Entities or by a ControllerAction in Init. The verdict says so plainly:

```
compiles with the ego replaced by the batch's policy; the scenario's own autopilot does not run in a batch
```

A batch result is therefore never Autoware's. `--ego seats` refuses an ego with its autopilot
instead, which answers the question of whether the scenario can run with its own autopilot in a
batch. It cannot. `pilot.<name>` parameters (CS3's map variant) go to the batch's policy as
observation inputs.

## 4. The checker

```bash
scensim-sample compilable scenario.xosc [more ...] [--ego policy|seats] [--all] [--json v.json]
```

- It reads `.xosc`, SSv2's YAML scenario format (as the same tree), and a
  ParameterValueDistribution, which it reads through to its ScenarioFile. A distribution
  changes values, not constructs.
- Its exit code is 0 when every scenario compiles and 1 when one is refused.
- For each scenario it prints one line: `compiles`, with its notes and its "compiles only if"
  conditions, or the first refusal:

```
refused: TimeToCollisionCondition at /OpenSCENARIO/Storyboard/Story[@name='main']/Act[@name='act']/ManeuverGroup[@name='dart']/Maneuver[@name='dart']/Event[@name='dart']/StartTrigger/ConditionGroup/Condition[@name='dart']/ByEntityCondition/EntityCondition/TimeToCollisionCondition (SSv2 implements it only partly; fix: write it as a DistanceCondition on a parameter the sampler derives (...))
```

- `--all` lists every refusal, not only the first.
- `--json` writes `scensim.compilable/1`: each scenario's verdict, refusals, conditions, notes
  and counts per class, plus a summary.

## 5. The case studies

Each case study is a real scenario under `src/scensim_sampler/examples/cases/`, with invented
values:

| Case | Verdict |
|---|---|
| CS1, a walker darts out (`cs1_walker_dartout.xosc`) | compiles if the walker's behaviour model has a batched form. Its trigger is written as a derived distance. The TTC form (`cs1_walker_dartout_ttc.xosc`) is refused as derived, with the rewrite named |
| CS2, the lead brakes, wet road, night | compiles if the slot has a camera (illumination) and SSv2's default behaviour has a batched model (the lead). The wet surface is a material |
| CS3, a stale map | compiles with the ego as the batch's policy, the map variant an observation input. Refused with `--ego seats` |
| CS4, a lidar drops out during a cut-in | compiles, with the fault as a mask, if SSv2's default behaviour has a batched model (the cutter) |
| CS5, fog mid-route | compiles if the slot has a camera and a renderer with fog per slot (and TimeOfDay). The fog's ramp over `fog_ramp` is a profile (gap below) |

## 6. Measured on SSv2's examples

The checker reads SSv2's own example scenarios: 82 in SSv2's YAML scenario format under
`test_runner/scenario_test_runner/scenario/`, and one `.xosc` under
`openscenario_interpreter/example/`. Its verdicts at the pinned SSv2 (2026-11-21):

| | `--ego policy` | `--ego seats` |
|---|---|---|
| compile | 58 of 83 | 41 |
| unconditionally | 18 | 12 |
| only if the slot provides more | 40 | 29 |
| refused | 25 | 42 |

Of the 40 conditional scenarios, 39 depend on a batched model of SSv2's default traffic
behaviour. The other is the all-weather example: TimeOfDay, Sun, Fog, Precipitation, Wind and
the sky dome.

The 25 refusals (`--ego policy`), by first construct:

| First construct | Scenarios |
|---|---|
| a CustomCommandAction other than the batch ones (V2I signals, FaultInjection, RequestToCooperate, the interpreter's own test commands, shell echoes) | 11 |
| UserDefinedValueCondition: the autopilot's `ego.currentMinimumRiskManeuverState.state` (3), `ego.currentState` (3), a ROS topic (2) | 8 |
| ControllerAction after Init | 2 |
| ParameterAction after Init | 2 |
| TimeToCollisionCondition (derived) | 1 |
| AddEntityAction | 1 |

`RelativeHeadingCondition(...)` compiles (a batch row), but the three examples that use it also
read `ego.currentMinimumRiskManeuverState.state` first, so the counts do not move.

Re-measured after the unimplemented class (2026-11-21): the same counts, since no SSv2 example
uses LaneOffsetAction, RelativeLanePosition or TraveledDistanceCondition (its examples run
only what it implements).

No element in the examples is unlisted. The test checks this, so a new SSv2 element shows up as
a failing test, not as a silent refusal. These counts are a measurement, not a target, and the
test does not fix them.

Re-measured with executor pending (2026-11-21, K2.3): `--ego policy` still compiles 58, but
12 unconditionally (was 18) and 46 only if the slot provides more (was 40): 33 of the 58 use a
pending row, 6 of them nothing else. By row, in how many scenarios: AcquirePositionAction 39
(mostly the ego's goal), FollowTrajectoryAction 8, LaneChangeAction 4, TrafficSignalStateAction
and TrafficSignalControllerAction 3 each, AbsoluteTargetLane, RelativeTargetLane,
RelativeClearanceCondition and AssignRouteAction 2 each, RoadCondition, SpeedProfileAction and
TimeHeadwayCondition 1 each. `--ego seats` does not move (41, 12, 29, 42).

Re-measured after the ego's route goal and LaneChangeAction (2026-11-21, K2.3): `--ego policy`
compiles 56 (the ego's FollowTrajectoryAction is now refused: it would override the policy,
2 examples), 14 unconditionally, 42 only if the slot provides more; 18 of the 56 use a pending
row: AcquirePositionAction 8 (other entities' goals), FollowTrajectoryAction 6,
TrafficSignalStateAction and TrafficSignalControllerAction 3 each, RelativeClearanceCondition
2, RoadCondition, SpeedProfileAction and TimeHeadwayCondition 1 each. `--ego seats` does not
move.

## 7. Gaps

- **Knob profiles.** CS5's fog ramps over a stretch of road. A batch needs the knob as a
  profile along time or route position (scenario-description.md CS5). The checker accepts the
  EnvironmentAction but cannot see the ramp.
- **SSv2's default behaviour.** Almost every example has an entity driven by SSv2's default
  behaviour, so a batched model of it decides most of what compiles: lane following at the
  target speed, stopping for an entity ahead, and the behaviour tree's other states. It is K2's
  largest item.
- **A fully scripted entity.** An entity whose every motion is a FollowTrajectoryAction need
  not wait on the default-behaviour model, if the trajectory follower ignores obstacles. That
  is not checked: if SSv2's follower still stops for an entity ahead, the model is needed. A
  later refinement may make such an entity batch.
- **Storyboard complexity.** No bound for now (133, 2026-11-21); D7 revisits it.

## 8. Executor pending

The "batch" rows and commands the batch executor (`scensim_batch`, K2.3's compiled storyboard)
does not run yet. A scenario using one gets the condition "`<row>`: batch executor pending",
which the executor never meets, so `scensim_batch.compile` refuses it by name.
`scensim_sampler.subset.EXECUTOR_PENDING` holds them; `scensim_batch.storyboard.IMPLEMENTED`
holds the rows it runs. A test (`scensim_batch/test/test_storyboard.py`, `Rows`) requires
every batch row and command in exactly one of the two, and every pending one named here, so
running a row means moving it from one set to the other.

| Pending | What it waits for |
|---|---|
| `AcquirePositionAction`, `AssignRouteAction` | routes on the world adapter's map, per slot |
| `FollowTrajectoryAction` | a trajectory follower per slot |
| `SpeedProfileAction` | a speed profile per slot |
| `RelativeClearanceCondition` | clearance along the route |
| `TrafficSignalStateAction`, `TrafficSignalControllerAction` | signal phases per slot |
| `RoadCondition` | friction as a per-slot material on the engine |
| `scensim.fault` | the fault as a per-slot mask on the observation |

Inside an implemented row, some attribute forms are refused by name too, as "... (batch
executor pending)" from `scensim_batch.compile`: a freespace distance, a routing algorithm, a
lane coordinate system on a world adapter without `lane_st`, a speed, acceleration or relative
speed condition with a `direction`, a continuous RelativeTargetSpeed, a SpeedAction on the ego
or on a prop, a story-time TeleportAction, an event priority `skip` (SSv2 leaves it
unimplemented), `selectTriggeringEntities`, a StoryboardElementState other than the standby,
running and complete states and the start, end and stop transitions.

