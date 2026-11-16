# Tutorial 2: Write a scenario

**Goal**: a new scenario, written from the map alone, in which the ego drives 80 m along a
Town01 road to a goal behind a slower car that it must not touch, and the run passes.

```
$ ros2 run swb run scenarios/follow_to_goal.xosc
passed   2026-11-15T11-02-13_follow_to_goal_r0  .../runs/2026-11-15T11-02-13_follow_...
```

## You will learn

- finding poses on a CARLA map in the scenario's frame, with `carla_poses`;
- the four parts of a scenario that its author writes: the map, the entities, the Init and
  the verdict;
- declaring a scenario parameter and changing it per run with `--param`;
- the rules of the bridge that a first scenario meets.

## Before you start

- [Tutorial 1](01-first-drive.md): the project `~/my_first_drive` with Town01 fetched, and
  its stack running. Every terminal: `source ~/swb.bash`.
- CARLA's Python client, installed in
  [Getting started on CARLA](../getting-started/carla.md#carla-itself).
- About 30 minutes.

The finished file ships with the workspace for comparison, in the Isaac Sim example
project:

```bash
E=$(ros2 pkg prefix --share swb_examples)/projects
ls $E/isaac_first_drive/scenarios/follow_to_goal.xosc
```

## 1. Find poses on the map

Scenarios place entities in the frame of the Autoware map. For the CARLA towns this frame
is CARLA's world with y mirrored and headings negated, a conversion that is error-prone by
hand. `carla_poses` reads the town that CARLA has loaded, without modifying it (so it is
safe beside a running stack), and prints positions already converted. It finds CARLA through
the host settings and the seat that holds the stack, so no address need be given:

```bash
ros2 run swb_launch carla_poses spawn-points
```

**What you see** (each entry is one line; wrapped here):

```
# CARLA 0.9.16 at localhost:2000, map Town01
   0  <WorldPosition x="335.49" y="-273.74" z="0.30" h="-1.57080" p="0" r="0" />
      <!-- road 19 lane -1 -->
   1  <WorldPosition x="299.40" y="-133.24" z="0.30" h="0.00000" p="0" r="0" />
      <!-- road 4 lane -1 -->
   2  <WorldPosition x="299.40" y="-129.75" z="0.30" h="-3.14159" p="0" r="0" />
      <!-- road 4 lane 1 -->
   ...
```

Spawn point 2 lies on a long straight road heading west. The lead car's start (20 m) and
the goal (80 m) are found by following its lane:

```bash
ros2 run swb_launch carla_poses ahead 2 --distance 20
ros2 run swb_launch carla_poses ahead 2 --distance 80
```

```
start  <WorldPosition x="299.40" y="-129.50" z="0.00" h="3.14115" p="0" r="0" />
end    <WorldPosition x="279.40" y="-129.49" z="0.00" h="3.14115" p="0" r="0" />
...
end    <WorldPosition x="219.40" y="-129.47" z="0.00" h="3.14115" p="0" r="0" />
```

Each command prints the lane centre at spawn point 2 (`start`, the ego's start) and the
pose the given distance along the lane (`end`): 20 m for the lead car, 80 m for the goal.
`ahead` follows the lane and, at a junction, keeps the straightest branch;
`carla_poses --help` lists the other queries, such as `at X Y`.

**Check**: the first line names `map Town01`. If it names another town, running Tutorial
1's scenario once makes the stack load Town01 into CARLA.

## 2. Write the scenario

The new scenario starts from a copy of the existing one, keeping its frame: `FileHeader`,
`ParameterDeclarations`, `CatalogLocations`, and the `Storyboard`'s `Story` / `Act` /
`ManeuverGroup` / `Maneuver` nesting with its triggers.

```bash
cd ~/my_first_drive
cp scenarios/drive_to_goal.xosc scenarios/follow_to_goal.xosc
```

Four parts remain to be written.

**The map**, by name. `$(env SWB_MAPS)` is the project's `assets/maps/`, which the stack
sets, so the same file works in any project that fetched Town01:

```xml
<RoadNetwork>
  <LogicFile filepath="$(env SWB_MAPS)/Town01" />
</RoadNetwork>
```

**The entities.** The ego stays as it is: the catalog car `tesla_model3` with the
controller `Autoware` (an entity's controller names what drives it; Tutorial 3 lists
them). A lead car is added with an empty controller name, which means that
SSv2 drives it:

```xml
<ScenarioObject name="lead">
  <Vehicle name="lead" vehicleCategory="car">
    <BoundingBox>
      <Center x="1.5" y="0.0" z="0.9" />
      <Dimensions width="2.1" height="1.8" length="4.5" />
    </BoundingBox>
    <Performance maxSpeed="20" maxAcceleration="3.0" maxDeceleration="8.0" />
    <Axles>
      <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8"
                 positionX="3.1" positionZ="0.3" />
      <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.8"
                positionX="0.0" positionZ="0.3" />
    </Axles>
    <Properties />
  </Vehicle>
  <ObjectController><Controller name=""><Properties /></Controller></ObjectController>
</ScenarioObject>
```

The bridge spawns the CARLA vehicle closest in size to the declared box; to force a body,
set `model3d="vehicle.audi.tt"` on the `Vehicle`.

**Init**: the initial actions teleport both cars, give the ego its goal and start the lead
car at a parameterized speed. The poses come from step 1; the ego's goal is the
`AcquirePositionAction`:

```xml
<Private entityRef="ego">
  <PrivateAction><TeleportAction><Position>
    <WorldPosition x="299.40" y="-129.50" z="0.30" h="3.14115" p="0" r="0" />
  </Position></TeleportAction></PrivateAction>
  <PrivateAction><RoutingAction><AcquirePositionAction><Position>
    <WorldPosition x="219.40" y="-129.47" z="0.00" h="3.14115" p="0" r="0" />
  </Position></AcquirePositionAction></RoutingAction></PrivateAction>
</Private>
<Private entityRef="lead">
  <PrivateAction><TeleportAction><Position>
    <WorldPosition x="279.40" y="-129.49" z="0.30" h="3.14115" p="0" r="0" />
  </Position></TeleportAction></PrivateAction>
  <PrivateAction><LongitudinalAction><SpeedAction>
    <SpeedActionDynamics dynamicsShape="step" value="0" dynamicsDimension="time" />
    <SpeedActionTarget><AbsoluteTargetSpeed value="$leadSpeed" /></SpeedActionTarget>
  </SpeedAction></LongitudinalAction></PrivateAction>
</Private>
```

The `z` values need not be exact: the bridge places every actor on the ground below its x
and y.

`$leadSpeed` requires a declaration. The copied file has an empty element,
`<ParameterDeclarations />`; expand it into an opening and a closing tag, and place the
declaration between them:

```xml
<ParameterDeclarations>
  <ParameterDeclaration name="leadSpeed" parameterType="double" value="4.0" />
</ParameterDeclarations>
```

**The verdict**: three events in one maneuver of the story, each ending the run.

| Event | Condition | Action |
|---|---|---|
| reached the goal | `ReachPositionCondition` on the ego, the goal pose, `tolerance="3.0"` | `exitSuccess` |
| touched the lead car | `CollisionCondition` between the ego and `lead` | `exitFailure` |
| too long | `SimulationTimeCondition` > 120 | `exitFailure` |

Each is a `UserDefinedAction` with a `CustomCommandAction type="exitSuccess"` (or
`exitFailure`) and a `StartTrigger` holding the condition. Simulation time starts when the ego is engaged, so Autoware's start does not
count against the timeout.

**Check**:

```bash
ros2 run swb project check
```

prints `project: ok`: every scenario parses, its map is fetched, and every entity driven
from outside (`Autoware` or `agent`) has a pilot.

## 3. Run it

```bash
ros2 run swb run scenarios/follow_to_goal.xosc
```

The lead car pulls away, the ego follows it at a distance, and the run ends when the ego
reaches the goal, after about 35 s of simulation time. Autoware perceives the lead car
only through its own lidar.

**What you see**: `passed   <run id>  <run directory>`.

**Check**: `echo $?` prints `0`.

## 4. Change a parameter per run

```bash
ros2 run swb run scenarios/follow_to_goal.xosc --param leadSpeed=2.0
```

**What you see**: a run id with the parameter in it,
`..._follow_to_goal_leadSpeed-2.0_r0`, and the verdict. The file itself is unchanged: SSv2
receives the value as an override of the declaration.

**Check**:

```bash
grep -A2 '"parameters"' runs/*leadSpeed-2.0*/result.json
```

shows `"leadSpeed": 2.0`.

## What happened

A scenario defines its map, its entities, their cars and which of them are driven from
outside; the project states only *who* drives each of those (`pilots:`).

A first scenario meets the following rules ([What a scenario may use](../reference/scenarios.md)
lists all of them):

- **SSv2-driven cars follow SSv2's poses**; a collision does not move them
  ([Tutorial 3](03-other-road-users.md)).
- **Uncommanded traffic lights are green.** A scenario commands the lights it depends on.
- **The ego must remain its catalog car.** The bridge refuses an ego whose simulated body
  differs from the catalog entry by more than 2 cm, so the `CatalogReference` to
  `tesla_model3` is left unchanged.
- **The ego takes goals and speed limits, not lane changes.** SSv2 refuses relative-speed
  and lane-change actions for an ego.

## Exercises

1. **Easy.** Move the goal 40 m further: `carla_poses ahead 2 --distance 120`, and put the
   new pose in both the `AcquirePositionAction` and the `ReachPositionCondition`. Check:
   the run passes, and `runs list` shows a longer duration than before.
2. **Medium.** Make the lead car stop halfway: a second `SpeedAction` to 0, in a new
   event triggered by `SimulationTimeCondition` > 10. Check: the run ends `failed` on the
   timeout, because the ego waits behind the lead car.
3. **Open.** Find the speed at which the lead car becomes too slow for the 120 s limit, by
   running `--param leadSpeed=` with smaller values. Check: the fastest failing value, and
   its `runs why` line.

## If it does not work

| Symptom | Fix |
|---|---|
| `carla_poses`: `No module named 'carla'` | `python3 -m pip install --user carla==0.9.16` ([Getting started on CARLA](../getting-started/carla.md#carla-itself)) |
| `project check`: `LogicFile ... does not name a project map` | the `LogicFile` must be `$(env SWB_MAPS)/<map>`, the fix it prints |
| `project check`: `entity ... has no pilot` | an entity with controller `Autoware` or `agent` needs a pilot in `swb.yaml` (or rename the controller to `""` for SSv2 to drive it) |
| The run fails at once: `the ego's body ... is not the car SSv2 declared` | the ego must stay the catalog car `tesla_model3` |

## Next

[Tutorial 3: other road users](03-other-road-users.md): pedestrians, obstacles, cars the
simulator drives.
