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

An element the table does not list is refused as "not in the compilable subset". The table
grows by decision, not by default. Containers (Story, Act, ManeuverGroup, Event,
ConditionGroup, Position, the dynamics and target elements) compile when their contents do.

**"batch" is a claim.** No row has run in a batch executor yet. Each row stays a claim until
K2's adapter runs the construct and compares it against seats on the same sample. That
comparison, row by row, is K2's acceptance.

## 2. The table

**batch**

| Constructs | How |
|---|---|
| TeleportAction; WorldPosition, LanePosition, RelativeWorldPosition, RelativeObjectPosition, RelativeLanePosition | a per-slot reset pose; lane positions resolved on the world's map once per slot |
| SpeedAction (step, linear, cubic; absolute or relative target), SpeedProfileAction | a speed profile per slot |
| LaneChangeAction, LaneOffsetAction | a lateral profile per slot |
| FollowTrajectoryAction, AssignRouteAction, AcquirePositionAction | precomputed per slot on the world's map |
| RoadCondition; the world knob `world.surface` | friction as a per-slot material |
| the world knob `world.zone_layout`; a MiscObject | a prop set, instanced per slot |
| Distance, RelativeDistance, RelativeClearance, ReachPosition, SimulationTime, TimeHeadway, Speed, RelativeSpeed, Acceleration, StandStill and TraveledDistance conditions | vectorised per slot |
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

No element in the examples is unlisted. The test checks this, so a new SSv2 element shows up as
a failing test, not as a silent refusal. These counts are a measurement, not a target, and the
test does not fix them.

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
