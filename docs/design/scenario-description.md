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
`scensim.cell_attributes` must be a bound knob or a declared scenario parameter. The sampler
computes the record's cell from the record's values through those bindings, and the coverage
report groups by the values the world applied (`result.json`'s `knobs`). `testbench check` and
`scensim-sample check` refuse a cell attribute that is neither a bound knob nor a parameter,
so no cell can claim a condition the world never set (found in D7: a drawn `illumination`
labelled the cell while the run stayed in daylight). The coverage report groups
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
