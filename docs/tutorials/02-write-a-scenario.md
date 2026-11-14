# Tutorial 2: Write a scenario

**Goal**: a scenario of your own, from nothing but the map: the ego drives 80 m down a
Town01 road to a goal, behind a slower car it must not touch, and the run passes.

```
$ ros2 run swb run scenarios/follow_to_goal.xosc
passed   2026-11-15T11-02-13_follow_to_goal_r0  .../runs/2026-11-15T11-02-13_follow_...
```

## You will learn

- to find poses on a CARLA map in the scenario's frame, with `carla_poses`;
- the four parts of a scenario you write yourself: the map, the entities, the Init, the
  verdict;
- to give a scenario a parameter and change it per run with `--param`;
- which rules of the bridge a first scenario meets.

## Before you start

- [Tutorial 1](01-first-drive.md): the project `~/my_first_drive` with Town01 fetched, and
  its stack running. Every terminal: `source ~/swb.bash`.
- CARLA's Python client (`pip install carla==0.9.16`), from
  [Getting started on CARLA](../getting-started/carla.md).
- About 30 minutes.

The finished file ships with the workspace, so you can compare against it. It lives in
the Isaac Sim example project, because Tutorial 6 runs this same file there unchanged; it
is the same scenario on CARLA:

```bash
E=$(ros2 pkg prefix --share swb_examples)/projects
ls $E/isaac_first_drive/scenarios/follow_to_goal.xosc
```

## 1. Find poses on the map

Scenarios place entities in the frame of the Autoware map. For the CARLA towns that is
CARLA's world with y flipped and headings negated, which is easy to get wrong by hand.
`carla_poses` reads the town CARLA has loaded (it only reads; it is safe beside a running
stack) and prints positions already converted, as OpenSCENARIO elements:

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

Spawn point 2 starts on a long straight road heading west. Walk along its lane for the
lead car's start (20 m) and the goal (80 m):

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

(Each line also ends with a comment naming the road and lane.) The `start` line is the lane
centre at spawn point 2: the ego's start. `ahead` follows the lane and, at a junction,
keeps the straightest branch (`--turn` takes the sharpest, `--every` prints the way);
`at X Y` snaps a point you picked elsewhere, in RViz for example, onto the nearest lane.
`--host` and `--port` reach a CARLA elsewhere.

**Check**: the first line names `map Town01`. If it names another town, run Tutorial 1's
scenario once: the stack loads Town01 into CARLA for it.

## 2. Write the scenario

Start from a copy of the scenario you already have, and keep its frame: `FileHeader`,
`ParameterDeclarations`, `CatalogLocations`, and the `Storyboard`'s `Story` / `Act` /
`ManeuverGroup` / `Maneuver` nesting with the Act's own `StartTrigger` and the empty
`StopTrigger`. OpenSCENARIO requires them and every scenario repeats them.

```bash
cd ~/my_first_drive
cp scenarios/drive_to_goal.xosc scenarios/follow_to_goal.xosc
```

What you write yourself are four parts.

**The map**, by name. `$(env SWB_MAPS)` is the project's `assets/maps/`, set by the stack
for every process, so the same file works in any project that fetched Town01:

```xml
<RoadNetwork>
  <LogicFile filepath="$(env SWB_MAPS)/Town01" />
</RoadNetwork>
```

**The entities.** Keep the ego as it is: the catalog car `tesla_model3` with the controller
`Autoware`, which the project's `pilots: ego:` drives. Add a lead car whose controller name
is empty, which means SSv2 drives it:

```xml
<ScenarioObject name="lead">
  <Vehicle name="lead" vehicleCategory="car">
    <BoundingBox>
      <Center x="1.5" y="0.0" z="0.9" />
      <Dimensions width="2.1" height="1.8" length="4.5" />
    </BoundingBox>
    <Performance maxSpeed="20" maxAcceleration="3.0" maxDeceleration="8.0" />
    <Axles> ... </Axles>
    <Properties />
  </Vehicle>
  <ObjectController><Controller name=""><Properties /></Controller></ObjectController>
</ScenarioObject>
```

The bridge spawns the CARLA vehicle closest in size to the declared box within its
`vehicleCategory`; to force a body, set `model3d="vehicle.audi.tt"` on the `Vehicle`.

**Init**: teleport both, give the ego its goal, start the lead car at a parameterized
speed. Paste the poses from step 1; the ego's goal is the `AcquirePositionAction`:

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

with `<ParameterDeclaration name="leadSpeed" parameterType="double" value="4.0" />` in
`ParameterDeclarations`. The `z` is not critical: the bridge places every actor on the
ground under its x and y.

**The verdict**: three events in one maneuver of the story, each ending the run.

| Event | Condition | Action |
|---|---|---|
| reached the goal | `ReachPositionCondition` on the ego, the goal pose, `tolerance="3.0"` | `exitSuccess` |
| touched the lead car | `CollisionCondition` between the ego and `lead` | `exitFailure` |
| too long | `SimulationTimeCondition` > 120 | `exitFailure` |

Each is a `UserDefinedAction` with a `CustomCommandAction type="exitSuccess"` (or
`exitFailure`) and a `StartTrigger` holding the condition; the shipped file has all three
written out. Simulation time starts when the ego is engaged, so the timeout does not count
Autoware's start.

**Check**:

```bash
ros2 run swb project check
```

prints `project: ok`. It parses every scenario under `scenarios/`, checks that its map is
fetched and that every entity driven from outside (`Autoware` or `agent`) has a pilot.

## 3. Run it

```bash
ros2 run swb run scenarios/follow_to_goal.xosc
```

In CARLA (or RViz) the lead car pulls away ahead, the ego follows it at a distance, and
the run ends when the ego reaches the goal, about 35 s of simulation time. Autoware has to
see the lead car with its own lidar: nothing tells it where the car is.

**What you see**: `passed   <run id>  <run directory>`.

**Check**: `echo $?` prints `0`.

## 4. Change a parameter per run

```bash
ros2 run swb run scenarios/follow_to_goal.xosc --param leadSpeed=2.0
```

**What you see**: a run id with the parameter in it,
`..._follow_to_goal_leadSpeed-2.0_r0`, and the verdict. The scenario file is not changed:
SSv2 receives the value as an override of its `ParameterDeclaration`.

**Check**:

```bash
grep -A2 '"parameters"' runs/*leadSpeed-2.0*/result.json
```

shows `"leadSpeed": 2.0`.

## What happened

A scenario owns its map, its entities, their cars and which of them are driven from
outside (controller `Autoware` or `agent`); the project only says *who* drives each of
those (`pilots:`). So the same `.xosc` runs in any project that fetched its map and
binds its entities, on CARLA or on Isaac Sim.

Rules worth knowing from [What a scenario may use](../reference/scenarios.md), the ones a
first scenario meets:

- **SSv2-driven cars go where SSv2 says.** They follow SSv2's pose every frame; a
  collision does not change their path. For vehicles that drive with physics, use the controller
  `simulator_autopilot` (CARLA's Traffic Manager) or `agent` (a pilot of the project,
  [Tutorial 4](04-several-pilots.md)).
- **Uncommanded traffic lights are green.** CARLA's own light cycling is frozen; command
  the lights your scenario depends on.
- **The ego's body is its catalog car's.** The bridge refuses an ego whose body in the
  simulator is more than 2 cm off its catalog box; keep the catalog reference as it is.
- **The ego takes goals and speed limits, not lane changes.** SSv2 refuses relative-speed
  and lane-change actions for an ego by design.

## Exercises

1. **Easy.** Move the goal 40 m further: `carla_poses ahead 2 --distance 120`, and put the
   new pose in both the `AcquirePositionAction` and the `ReachPositionCondition`. Check:
   the run passes, and `runs list` shows a longer duration than before.
2. **Medium.** Make the lead car stop halfway: a second `SpeedAction` to 0, in a new
   event triggered by `SimulationTimeCondition` > 10. Check: the run ends `failed` on the
   timeout (the ego waits behind it), and `ros2 run swb runs show follow_to_goal` shows the
   ego never reaching the goal.
3. **Open.** Find the speed at which the lead car is too slow for the 120 s limit: run
   `--param leadSpeed=` with smaller values. Check: the fastest failing value, and its
   `runs why` line.

## If it does not work

| Symptom | Fix |
|---|---|
| `carla_poses` names another map | CARLA has another town loaded: run a Town01 scenario once |
| `carla_poses`: `No module named 'carla'` | `pip install carla==0.9.16` |
| `project check`: `LogicFile ... does not name a project map` | the `LogicFile` must be `$(env SWB_MAPS)/<map>`, the fix it prints |
| `project check`: `entity ... has no pilot` | an entity with controller `Autoware` or `agent` needs a pilot in `swb.yaml` (or rename the controller to `""` for SSv2 to drive it) |
| The run fails at once: `the ego's body ... is not the car SSv2 declared` | the ego must stay the catalog car `tesla_model3` |

## Next

[Tutorial 3: other road users](03-other-road-users.md): pedestrians, obstacles, cars the
simulator drives.
