# Scenario description for risk studies, and the scenario-to-world compiler (design, in progress)

Status: design exploration for phase 093, 2026-11-20. Case studies D1 and the world interface
D2 are drafted here; D3 to D6 follow from them. Nothing here is built yet.

Inputs: phase 092's design ([ml-training-interface.md](ml-training-interface.md)) and its
studies ([survey-risk-categories.md](survey-risk-categories.md),
[survey-vehicle-risk-taxonomy.md](survey-vehicle-risk-taxonomy.md)).

## 1. Principles

1. **The scenario says what is tested; the world says where.** The scenario holds the actors,
   their behaviour, the encounter, the sampling and the verdict. The world holds the field and
   the knobs it exposes. They meet only through the world interface (section 3).
2. **OpenSCENARIO 1.3 first, extension points second, nothing private third.** A case uses
   standard constructs where they exist. Where they do not, it uses the standard's own
   extension points:
   - `UserDefinedDistribution` for sampling;
   - `CustomCommandAction` (inside `UserDefinedAction`) for faults;
   - `ObjectController` properties for behaviour models;
   - `ParameterDeclaration` for world knobs.

   A scenario stays a valid OpenSCENARIO file that other tools can read, even where they cannot
   evaluate our extensions.
3. **Parameters are encounter-relative.** Time to conflict, headway and time to collision,
   not absolute start times.
4. **SSv2 receives concrete scenarios.** The sampler (092 K1) evaluates distributions and
   hands SSv2 one concrete parameter set per run. SSv2 parses `UserDefinedDistribution` but
   does not evaluate it, which is exactly the split we want.
5. **What SSv2 supports today is the floor for seats** (its support table,
   `scenario_simulator_v2/docs/developer_guide/OpenSCENARIOSupport.md`). The compiler for the
   batch target accepts a stated subset (section 6). A construct outside a target's subset is
   refused by name, never dropped silently.

## 2. Case studies (D1)

Each case has four parts: the functional description, the logical scenario (abridged
OpenSCENARIO), one sample record, and the compiled form with its gaps.

### CS1. A walker darts out from behind a parked van (R2: VRU, occlusion)

**Functional.** The ego drives along an urban street. A van is parked at the kerb. A walker
steps out from behind it and crosses, so the ego sees the walker late.

**Logical.**

| Parameter | Meaning | Base (joint, fitted from public intersection data) | Constraint |
|---|---|---|---|
| `ego_speed` | the ego's speed at the encounter | the street's free-flow speed distribution | within the speed limit + 20 % |
| `van_offset` | the van's front relative to the conflict point | parked-car positions near crossings | the van fits the kerb |
| `walker_type` | adult, child, runner | demographic mix | |
| `walk_speed` | walking speed, conditioned on `walker_type` | walking-speed data | |
| `ttc_entry` | the ego's time to the conflict point when the walker leaves the van's shadow | the gap-acceptance model's thresholds, conditioned on `walker_type` | > 0 |
| `behaviour` | steady, hesitate at the lane edge, run | behaviour-model mix | |

```xml
<ParameterDeclarations>
  <ParameterDeclaration name="ego_speed"  parameterType="double" value="8.3"/>
  <ParameterDeclaration name="van_offset" parameterType="double" value="2.0"/>
  <ParameterDeclaration name="walk_speed" parameterType="double" value="1.4"/>
  <ParameterDeclaration name="ttc_entry"  parameterType="double" value="2.5"/>
  <ParameterDeclaration name="behaviour"  parameterType="string" value="steady"/>
</ParameterDeclarations>
<!-- the world anchor: a conflict point the world exposes (section 3) -->
<!-- Init: ego at ego_speed on lane anchor street_east; van parked at van_offset;
     walker behind the van, controlled by a behaviour model -->
<ObjectController>
  <Controller name="walker_model">
    <Properties>
      <Property name="model" value="scensim.behaviour:gap_acceptance_walker"/>
      <Property name="speed" value="$walk_speed"/>
      <Property name="style" value="$behaviour"/>
    </Properties>
  </Controller>
</ObjectController>
<!-- Start trigger: the ego's time to the conflict point falls to ttc_entry -->
```

The distribution file names one joint model for all the parameters:

```xml
<StochasticDistribution parameterName="ttc_entry">
  <UserDefinedDistribution type="scensim.sampler:joint">
    models/walker_dartout.yaml
  </UserDefinedDistribution>
</StochasticDistribution>
```

**Sample record** (092 S1, abridged):

```yaml
scenario: walker_dartout.xosc@sha256:...
category: R2.walker_crossing.dart_out
cell: {illumination: day, surface: dry, occluder: van}
values: {ego_speed: 9.1, van_offset: 1.6, walker_type: child, walk_speed: 1.9,
         ttc_entry: 1.8, behaviour: run}
seeds: {scenario: 41, world: 41, pilots: {ego: 41}}
base: models/walker_dartout.yaml@sha256:...   # dataset, version, licence in its header
proposal: importance/near_ttc_short.yaml       # purpose A; or a search id for purpose B
weight: 0.037
world: town_street@stage:sha256:...
car: tesla_model3@spec:...
```

**Compiled.**

| Target | What it becomes |
|---|---|
| seats (SSv2) | the concrete scenario. The walker runs its behaviour model as a pilot on the relay (a `command` pilot for entity `walker`), so its decision to go reads the ego's real approach |
| batch (Isaac Lab) | slot reset with poses and speeds; the van as an instanced prop; the walker's model batched over slots; the start condition vectorised (time to the conflict point, computed from speed and distance) |

**Gaps found.**

- **TimeToCollisionCondition is partial in SSv2.** It is implemented through
  `DistanceCondition`, and several of its properties are unsupported. The start trigger
  "time to the conflict point" is therefore written as a distance condition, `ego_speed ×
  ttc_entry`, which the sampler computes as a derived parameter. That is a rule for the
  subset: **derived parameters are the sampler's job, not the trigger's**.
- **Behaviour models need a home on seats.** Running them as pilots reuses the pilot
  interface and needs no SSv2 change (D3).

### CS2. The lead car brakes in a curve on a wet road at night (R1 × R5 × R4)

**Functional.** The ego follows a car into a right-hand curve on a wet road at night. The
lead brakes hard, at a heading angle to the lane, for a cause the ego cannot see.

**Logical.**

| Parameter | Meaning | Base | Constraint |
|---|---|---|---|
| `lead_speed` | | highway or arterial speeds | |
| `headway` | time gap, in seconds | headway conditioned on speed (highway data) | > 0.3 s |
| `decel` | lead's deceleration | emergency-braking decelerations | ≤ μ(`surface`) × g |
| `jerk` | how fast braking builds up | | |
| `onset_s` | where in the curve the lead brakes (arc position) | uniform over the curve, or importance-weighted toward the apex | |
| `surface` | a world knob: `dry`, `wet`, `wet_spray` | the region's wet-road frequency (meteorological records) | |
| `illumination` | a world knob: `day`, `dusk`, `night` | time-of-day exposure | |

```xml
<!-- the lead brakes when it reaches onset_s along the curve anchor -->
<SpeedAction>
  <SpeedActionDynamics dynamicsShape="linear" value="$decel" dynamicsDimension="rate"/>
  <SpeedActionTarget><AbsoluteTargetSpeed value="0"/></SpeedActionTarget>
</SpeedAction>
<!-- world knobs: ParameterDeclarations bound by the world interface -->
<ParameterDeclaration name="world.surface"      parameterType="string" value="wet"/>
<ParameterDeclaration name="world.illumination" parameterType="string" value="night"/>
```

**Compiled.**

| Target | What it becomes |
|---|---|
| seats | SSv2 runs the storyboard. The world server applies `world.surface` as a surface zone over the road (Track C), and `world.illumination` as a lighting preset |
| batch | every part compiles: poses, the speed profile, the arc-position trigger, surfaces as materials per slot. Night lighting compiles only when the slot has a camera, and then costs render time (092 section 5a) |

**Gaps found.**

- **The deceleration constraint names a world knob** (μ depends on `surface`). The sampler
  must read the world's declared μ for each knob value, so the world interface carries
  physical values with each knob, not only names.
- **The heading angle of the lead comes from the curve**, not from a parameter. The
  encounter's geometry belongs to the world anchor (the curve), which the scenario selects.

### CS3. A stale map: a lane closed in the world, open in the pilot's map (R3)

**Functional.** A lane is closed by work-zone cones. The pilot's HD map still shows it open.

**Logical.**

- A parameter `world.zone_layout` selects a prop set in the world (the cones and a closed
  lane).
- A parameter `pilot.map_variant` selects the map the pilot loads: `current` or
  `before_works`.
- The two are independent, so the cell (current map, works present) is the stale-map case.

**Compiled.**

| Target | What it becomes |
|---|---|
| seats | the testbench's `pilot_args` already substitute `{param:<name>}` into a pilot's arguments, so `map_path: maps/{param:pilot.map_variant}` gives the pilot the stale map while the world shows the works. No new mechanism is needed on seats |
| batch | refused for an Autoware pilot (it does not batch). For a batched policy, the map is an observation input, so the variant is a per-slot tensor |

**Gap found.** The world interface must keep **the world's own map** (for the
simulator: lanes, anchors) apart from **the map handed to a pilot**. Today they are the same
asset.

### CS4. A lidar drops out during a cut-in (R6 × R1)

**Functional.** A car cuts in ahead. As it does, the ego's top lidar stops for 1.5 s.

**Logical.** These are R1's cut-in parameters (lateral speed, gap, speeds), plus:

- `fault_sensor` (`lidar_top`);
- `fault_onset`, relative to the cut-in's start;
- `fault_duration`.

```xml
<UserDefinedAction>
  <CustomCommandAction type="scensim.fault">
    sensor_dropout entity=ego sensor=$fault_sensor duration=$fault_duration
  </CustomCommandAction>
</UserDefinedAction>
<!-- triggered fault_onset seconds after the cut-in's lane-change action starts -->
```

**Compiled.**

| Target | What it becomes |
|---|---|
| seats | SSv2 runs `CustomCommandAction` as a subprocess. Our `scensim.fault` command sends the fault through the relay to the ego's vehicle side, which stops publishing that sensor for the duration and records it |
| batch | the sensor's tensor is masked for the slot over the duration; a batched policy sees the dropout |

**Gaps found.**

- **A fault hook on the vehicle side**, per sensor and actuator. The fault hook is a
  specification: the fault names, the parameters, and the record.
- **SSv2 itself warns that `CustomCommandAction` is not portable.** That is acceptable here:
  the command is named `scensim.fault`, and another engine refuses it by name rather than
  misreading it.

### CS5. Fog rolls in mid-route (R8 × R4)

**Functional.** The ego drives a rural route. Fog thickens over a stretch, taking visibility
from 1000 m to 80 m. The question is whether the autopilot slows, stops, or leaves its ODD
correctly.

**Logical.** The parameters are:

- `fog_start`, an arc position on the route anchor;
- `fog_ramp`, the distance over which visibility falls;
- `visibility_min`;
- the ego's speed.

**Compiled.**

| Target | What it becomes |
|---|---|
| seats | an `EnvironmentAction` with fog (SSv2 supports `EnvironmentAction` and `Weather`), triggered at `fog_start`. The world server must ramp it over `fog_ramp`, not step it |
| batch | refused unless the slot has a camera and the renderer supports fog per slot; the reason is named |

**Gap found.** **Knobs that change during a run need a profile** (a value along time or
route position), not only a value at the start.

## 2a. Review: two fresh case studies (D7)

D7 checks the drafts against two case studies that were not used to write them. Both are real
files under `src/scensim_sampler/examples/cases/` (`cs6_*`, `cs7_*`, each with a distribution and
a joint model under `models/`). Their values are invented, and the models say so in their
headers. Each was run through `scensim-sample compilable`, `draw` (20 records) and `check`.

### CS6. A lead car's cut-out reveals a stopped car (R1)

**Functional.** On a three-lane highway the ego follows a lead car. The lead changes lane late,
and a car stopped in the ego's lane appears ahead.

**Logical.**

| Parameter | Meaning | Base | Constraint |
|---|---|---|---|
| `ego_speed` | the ego's and the lead's speed | normal, 28 m/s | within the limit + 20 % |
| `time_headway` | the ego's gap to the lead | lognormal, 1.6 s | |
| `reveal_time` | how far ahead of the stopped car the lead leaves, in seconds at its speed | lognormal, 2.0 s | the stopped car is ahead of the lead |
| `lc_time` | the lead's lane change duration | normal, 3.0 s | |
| `stopped_kind` | car, truck or motorbike | categorical | |
| `illumination` | day or night, bound to the world's knob by `scensim.knob.illumination` (gap 1) | categorical | |

`ego_headway_m`, `lead_s` and `reveal_m` are derived (`ego_speed × reveal_time` is the trigger
as a distance, CS1's rule). The lead's lane change starts on a freespace
`RelativeDistanceCondition` to the stopped car; the lead is blind (SSv2's `isBlind`), so its
default behaviour does not brake for the stopped car first; the stopped car is held at speed 0
by its Init `SpeedAction`; its `vehicleCategory` is `$stopped_kind`. Its records carry the
bound value (`illumination: night` where the cell says night), which the runner applies.

**Sample record** (abridged): `category: R1.lead_cut_out.stopped_vehicle`, `cell:
{illumination: day, stopped_kind: car}`, `values: {ego_speed: 32.97, time_headway: 1.88,
reveal_time: 1.42, lc_time: 3.05, reveal_m: 46.75, ...}`, `derived: [ego_headway_m, reveal_m,
lead_s]`, seeds for the scenario, the world and each entity.

**Compiled.**

| Target | What it becomes |
|---|---|
| seats (SSv2) | the concrete scenario; the stopped car's category picks the simulator's model (a truck's box for a truck) |
| batch | compiles if a batched form of SSv2's default traffic behaviour (the lead and the stopped car) exists, and, once the checker reads knob bindings, if the slot has a camera (illumination); the lane change is a precomputed lateral profile, the trigger a vectorised distance. A per-slot category needs per-slot assets (gap 6) |

### CS7. A cyclist overtaken at an intersection exit, low sun ahead (R2 × R4)

**Functional.** Leaving an intersection, the ego overtakes a cyclist riding near the kerb, with
the sun low ahead. In some runs the cyclist swerves out (a drain, a door) as the ego closes.

**Logical.**

| Parameter | Meaning | Base | Constraint |
|---|---|---|---|
| `ego_speed`, `cyclist_speed` | speeds | normal 10.5, lognormal 5.0 m/s | an overtake: ego faster by 1 m/s |
| `cyclist_s`, `kerb_offset` | where the cyclist rides on the exit lane | uniform 5 to 25 m, normal -1.1 m | |
| `rel_azimuth`, `sun_elevation` | the sun against the exit's heading, its height | normal 0 rad, uniform 0.03 to 0.35 rad | |
| `swerve_p`, `swerve_gap`, `swerve_time` | whether, when and how fast the cyclist swerves 0.8 m out | 30 %, lognormal 8 m, normal 1.2 s | peak lateral acceleration at most 3 m/s² |

Derived: `sun_azimuth` (the exit's heading plus `rel_azimuth`), `swerve_offset`, and the cell
values `glare` (into the sun or not) and `swerve`; `swerve_acc`, the swerve's peak lateral
acceleration, is an intermediate the constraint reads. The sun is set in Init by an
`EnvironmentAction` (`Sun`, `TimeOfDay`). The swerve is a `LaneChangeAction` into the cyclist's
own lane (`RelativeTargetLane` 0) ending at `targetLaneOffset = $swerve_offset`, over
`swerve_time`, started by a freespace `RelativeDistanceCondition` from the ego. The first draft
used a `LaneOffsetAction`, which SSv2 does not implement (gap 8); whether SSv2 accepts a lane
change into the entity's own lane is still to be seen on seats.

**Sample record** (abridged): `category: R2.cyclist.overtaken_at_exit`, `cell: {glare: low_sun,
swerve: yes}`, `values: {ego_speed: 10.06, cyclist_speed: 4.17, rel_azimuth: 0.71,
sun_elevation: 0.31, swerve_gap: 7.14, swerve_time: 1.48, ...}`; of 20 draws, 5 were into the sun,
and 4 more were rejected by the lateral acceleration bound.

**Compiled.**

| Target | What it becomes |
|---|---|
| seats (SSv2) | the concrete scenario; the world server sets the sun's azimuth and altitude (CARLA's weather parameters) |
| batch | compiles if a batched form of SSv2's default behaviour exists (the cyclist) and the slot has a camera (`TimeOfDay`, `Sun`); the swerve is a per-slot lateral profile. Glare matters only to a camera pilot |

### What the drafts lacked, and the rules proposed

D7's exit, "expressible with no construct the drafts lack", is **not met**. Both scenarios are
written, but with workarounds where the drafts had no construct. In order of consequence, each
with the rule this note now proposes:

1. **Sampled conditions did not reach the world.** A drawn knob value (`illumination`) landed in
   the record's cell, but nothing bound it to the scenario's `world.illumination`, so a record
   whose cell said night ran in daylight; CS1's example had the same flaw. *Rule:* section 3's
   binding (`scensim.knob.<name>`, the applied value read back into `result.json`) and section
   7's computed cells. CS6 now binds `illumination` that way.
2. **Latent base variables.** A natural base is often written in quantities the scenario does
   not use directly (a time headway, a reveal time), which only derived values read. The sampler
   refused a drawn variable that is not a scenario parameter, so both scenarios declare such
   variables as parameters their storyboard never reads. *Rule:* a model variable may be marked
   `latent: true`. It is drawn and recorded in the record's `values` (so a replay draws nothing),
   it may feed derived values, constraints and the cell, and it is never bound to the scenario;
   a latent variable with the name of a scenario parameter is refused.
3. **Optional entities.** "The lane the lead moves into is occupied in 20 % of runs" needs an
   entity that exists in some samples only. OpenSCENARIO's entity set is fixed, and
   `AddEntityAction` is seats only. *Rule:* an entity may carry the property `scensim.present` =
   `$<boolean parameter>`. Where false, the compiler places it nowhere: on seats it is not
   spawned (its actions and the conditions that name it are dropped, and the run's
   `resolved.yaml` says so); in a batch it is a masked slot entity, which every vectorised
   condition skips. A condition that names an absent entity is false, never an error.
4. **A stopped vehicle.** OpenSCENARIO has no parked or broken-down vehicle; a vehicle without a
   controller is SSv2's default traffic behaviour, so the checker conditioned a scenario on a
   batched traffic model for a car that never moves, and without its Init `SpeedAction` of 0 the
   car would drive off. *Rule:* a vehicle whose controller has the property `scensim.static =
   true` stands where it was placed for the whole run (speed 0, no behaviour), and is a static
   prop in a batch, with a vehicle's class and box. The checker classifies it batch; any motion
   action on it is refused.
5. **A blind entity.** SSv2's `isBlind` controller property (the entity ignores others) is what
   makes a scripted cut-out keep its speed. *Rule:* the subset lists `isBlind`. A vehicle that is
   blind and has only lane-following and scripted motion needs the lane-following part of the
   default behaviour and nothing that reads other entities (092 K2.1's `see_around` off), which
   the checker states as its condition instead of the whole traffic model.
6. **A per-sample entity class.** `vehicleCategory="$stopped_kind"` works in the sampler and in
   SSv2, but the subset did not classify it. *Rule:* a parameter in an entity's category or model
   is conditional in a batch on per-slot assets, and the checker names the parameter and its
   values. On seats it is the simulator's model for that category, chosen per run.
7. **The world's geometry in derived values.** The sun's azimuth relative to the road needs the
   exit lane's heading, a property of a lane anchor; the drafts gave the sampler knobs, not
   anchors, so CS7 uses a knob (`exit_heading`) as a stand-in. *Rule:* a world's anchors expose
   read-only attributes to derived expressions and constraints, as `anchor.<name>.<attribute>`:
   a lane's `heading` (at a given `s`, `anchor.exit_n.heading(s)`), `length` and `speed_limit`,
   a conflict point's `x` and `y`. The sampler reads them from the world's `world.yaml`, and the
   record names the world it read them from.

Two smaller points have their place elsewhere:

8. **The description needed a construct SSv2 does not implement.** CS7's first draft swerved
   with a `LaneOffsetAction`, the natural OpenSCENARIO construct, which the subset listed as
   batch. SSv2's own support table lists it as unimplemented (with
   `RelativeTargetLaneOffset`, `RelativeLanePosition` and `TraveledDistanceCondition`), so the
   scenario would not have run on seats either, and no batch run could ever have been checked
   against one. 092 S2's checker now refuses those constructs by name, as a class of their own,
   and keeps every batch row a construct SSv2 runs. *Rule:* a scenario construct counts as
   expressible only if SSv2 runs it; the drafts' examples and the case studies are checked by
   `scensim-sample compilable`, which refuses the unimplemented.
9. **Small points.** OpenSCENARIO's lane offset takes a peak lateral acceleration, not a
   duration, so a swerve time becomes a derived acceleration (the derived-parameter rule covers
   it). `Sun` is absolute, so a sun relative to the ego's heading depends on point 7.
   `scensim-sample check` on a missing file ended in a traceback (fixed in 092 S2's branch: it
   names the file).

Section 7's rule admits a derived cell attribute when its expression reads only bound knobs,
declared parameters and anchor geometry. CS7's `glare` and `swerve` are written that way: from
the applied `sun_azimuth` and `sun_elevation` against the exit's heading, and from the applied
`swerve_offset` against `kerb_offset`.

## 3. The world interface (D2)

A world declares, in a file beside it (`world.yaml`, which already exists for authored Isaac
stages in roadmap 091), what a scenario may name.

```yaml
# world.yaml (proposed additions)
anchors:
  lanes:   {street_east: lanelet:1234, street_west: lanelet:1235}
  regions: {work_zone_a: polygon:[...]}
  conflict_points: {crossing_mid: {x: 120.4, y: -33.0, lanes: [street_east, footpath_n]}}
  curves:  {right_curve_1: {lane: arterial_n, from_s: 410, to_s: 520}}
knobs:
  surface:
    values: {dry: {mu: 1.0}, wet: {mu: 0.7}, wet_spray: {mu: 0.7, spray: true}}
    default: dry
    applies: route            # the whole route, or named regions
  illumination: {values: [day, dusk, night], default: day}
  zone_layout: {values: {none: null, works_a: props/works_a.usda}, default: none}
  fog: {values: {visibility_m: [50, 1000]}, default: {visibility_m: 1000}, profile: true}
maps:
  world: maps/town_street             # what the simulator and the truth stream use
  pilot_variants: {current: maps/town_street, before_works: maps/town_street_2025}
```

Rules:

- **A knob is bound by a property, not by a parameter's name.** The scenario declares a plain
  parameter (`surface`, a name the sampler's models can use) and binds it in its `FileHeader`
  `Properties`: `<Property name="scensim.knob.surface" value="$surface"/>`. The sampler draws
  the parameter like any other; the runner reads the binding and applies the value to the
  world. `pilot.<name>` parameters reach pilots through the existing `{param:}` substitution.
- **A sampled knob value must reach the world, and the record says what was applied.** The
  runner sends each bound knob's value to the world before the run (weather and light through
  the world server's weather call, surfaces through the per-scenario surfaces). The world reads
  the applied value back, and `result.json` records it under `knobs`. A run whose applied
  value differs from its record's is an error run, excluded from rates.
- **Knobs carry physical values** (μ, visibility), so the sampler can apply constraints (CS2).
- **A knob with `profile: true`** accepts a value along time or route position (CS5).
- **The world's map and the pilot's maps are separate entries** (CS3).
- **`testbench check` refuses** a scenario that names an anchor or knob the world lacks, or a
  value outside the knob's declared values. The refusal names the scenario, the parameter and
  the world.
- **The same names on every target.** A CARLA town and an Isaac stage of the same place
  declare the same anchors and knobs, so one scenario runs on both. Where a target cannot
  honour a knob (fog on a target without fog), it refuses the knob by name.

## 4. Behaviour models (D3, first draft)

- **Naming.** A scenario names a behaviour model through `ObjectController` properties:
  `model` is an entry point, and the other properties are its parameters, which can
  themselves be sampled.
- **On seats,** a behaviour model runs as a pilot for that entity (a `command` pilot on the
  relay), using the same interface as an ego stack. It sees the truth stream and the other
  actors, and it commands its entity. Nothing changes in SSv2.
- **In a batch,** a behaviour model is batched (`step_batch` over slots, like the control
  component of 092 S3). A model without a batched form keeps the scenario on seats, with the
  reason named.
- **Shipping.** The starter models (a gap-acceptance walker, a car-follower with braking
  events, an aggressive driver) are examples, per 092 D1.

## 5. Sampling in the description (D4, first draft)

The distribution file says four things. Each lands in the sample record (CS1).

| What | Where in the file | In the sample record |
|---|---|---|
| the base | `UserDefinedDistribution type="scensim.sampler:joint"` naming a fitted model (its header names dataset, version, licence) | `base` with its hash |
| constraints | in the model file (`constraints:`), evaluated on each draw, may read world knobs' physical values | the rejection count for the batch |
| the proposal (purpose A) | `proposal:` in the suite, not in the scenario: the same scenario is used for rates and for search | `proposal`, `weight` |
| the search (purpose B) | `search:` in the suite (bandit over cells, cross-entropy, Bayesian optimisation) | `search` id and generation |

Two rules follow from this:

- **A scenario carries its realistic base.** It is part of what the scenario claims to
  describe.
- **The suite carries how the base is explored.** That is a choice of study, not of scenario.

## 6. The compilable subset (D5, first draft)

Superseded by [compilable-subset.md](compilable-subset.md) (092 S2), which gives the table
every construct SSv2 supports, a checker, and the count on SSv2's examples. The first draft:

| Construct | Batch | Note |
|---|---|---|
| Init: TeleportAction, initial speeds | yes | per-slot reset |
| SpeedAction (step, linear, cubic) | yes | |
| LaneChangeAction | yes | lateral profile |
| FollowTrajectoryAction, routes on anchors | yes | precomputed per slot |
| Distance, ReachPosition, SimulationTime and arc-position triggers | yes | vectorised |
| TimeHeadwayCondition | yes | |
| TimeToCollisionCondition | as a derived distance (CS1) | the sampler derives it |
| EnvironmentAction | only for knobs the slot's sensors see, and profiles the renderer supports | refused by name otherwise |
| ObjectController with a batched behaviour model | yes | otherwise the scenario stays on seats |
| `scensim.fault` (CustomCommandAction) | yes, as a tensor mask | other CustomCommandActions are refused |
| TrafficSignalStateAction | yes, as a per-slot state | |
| an Autoware or other per-process pilot | no | seats only |
| storyboard constructs beyond the above (nested Acts with complex ConditionGroups, ParameterSetAction mid-run) | no, for now | refused by name; D7 tests whether this is too narrow |

## 7. Coverage tagging (D6, first draft)

A scenario carries its category id, and each cell's attributes, in its `FileHeader`
`Properties`:

```xml
<FileHeader author="..." date="..." description="walker dart-out from a parked van">
  <Properties>
    <Property name="scensim.category" value="R2.walker_crossing.dart_out"/>
    <Property name="scensim.cell_attributes" value="illumination,surface,occluder"/>
  </Properties>
</FileHeader>
```

A cell's attributes are computed, never labelled. Each attribute named in
`scensim.cell_attributes` must be a bound knob, a declared scenario parameter, or a derived
value (the record's `derived:`) whose expression reads only bound knobs, declared parameters
and anchor geometry (`anchor.<name>.<attribute>`, section 2a, gap 7), so that it is computed
from what the world applied: CS7's `glare` reads the applied sun angle and the exit lane's
heading, not a drawn label. The sampler computes the record's cell from the record's values
through those bindings, and the coverage report groups by the values the world applied
(`result.json`'s `knobs`, and derived attributes recomputed from them). `testbench check` and
`scensim-sample check` refuse any other cell attribute, and a derived one whose expression
reads anything else (a latent variable, another derived value that does), so no cell can claim
a condition the world never set (found in D7: a drawn `illumination` labelled the cell while
the run stayed in daylight). The coverage report groups
records by category and cell. The category ids follow the taxonomy
([survey-vehicle-risk-taxonomy.md](survey-vehicle-risk-taxonomy.md)) as `R<family>.<category>.<subtype>`.

## 8. Open questions

1. **Behaviour models as pilots on seats.** Is the per-entity pilot's latency through the
   relay acceptable for a walker's reaction (CS1)? This needs measuring.
2. **Fog and lighting.** Do they compile on Isaac Lab per slot, or only per batch? This
   decides whether R4 trains in a batch at all.
3. **Knob profiles along the route.** Are profiles expressed in the scenario (an
   `EnvironmentAction` per step) or in the world interface (a profile value)? The draft says
   the world interface.
4. **The derived-parameter rule** (CS1). Is it enough to make TimeToCollision triggers
   portable, or should the subset demand a full TTC condition from every target?
