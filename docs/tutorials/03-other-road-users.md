# Tutorial 3: Other road users

**Goal**: three scenarios in your project, each with a road user of another kind: a
pedestrian crossing among obstacles, a traffic light the scenario turns red, and a car
CARLA's own driver takes to its goal. All three pass with Autoware driving the ego.

```
$ ros2 run swb runs list
RUN                                         VERDICT  SCENARIO                 DURATION
...pedestrian_crossing_r0                   passed   pedestrian_crossing.xosc ...
...red_light_r0                             passed   red_light.xosc           ...
...autopilot_r0                             passed   autopilot.xosc           ...
```

## You will learn

- that an entity's **controller name** decides who moves it, and what that changes;
- to place pedestrians and obstacles, and move a pedestrian along a timed path;
- to command a traffic light from the scenario;
- to hand a car to the simulator's own driver.

## Before you start

- [Tutorial 2](02-write-a-scenario.md): the project `~/my_first_drive` with its stack
  running on Town01, and a scenario of your own. Every terminal: `source ~/swb.bash`.
- About 40 minutes, most of it the runs.

The scenarios come with the workspace; you copy them into the project:

```bash
cd ~/my_first_drive
E=$(ros2 pkg prefix --share swb_examples)
```

Everything here is standard OpenSCENARIO. What decides how an entity moves is its
controller's name:

| Road user | In the scenario | Moved by | Physics |
|---|---|---|---|
| Pedestrian | `<Pedestrian>`, default controller | SSv2: the bridge places it each frame | off |
| Obstacle | `<MiscObject>` | stays where it is put | off |
| Car, scripted | `<Vehicle>` or a catalog car, controller `""` | SSv2, as a pose each frame | off |
| Car, driven by the simulator | controller `simulator_autopilot` | CARLA's Traffic Manager | on |
| Car with a pilot | controller `agent` | a pilot of the project ([Tutorial 4](04-several-pilots.md)) | on |

A scripted car or pedestrian goes where the scenario says, every run, but a collision
never pushes it. The last two drive with physics, react to what is around them, and SSv2
sees them where the simulator reports them.

## 1. A pedestrian among obstacles

```bash
cp $E/projects/first_drive/scenarios/pedestrian_crossing.xosc scenarios/
```

The ego drives west past two street barriers while a pedestrian crosses its lane. It
passes if the ego reaches its goal without coming within 1 m of the walker. The
pedestrian and a barrier are declared with CARLA's asset names, which the bridge
translates for each simulator:

```xml
<ScenarioObject name="pedestrian_0">
  <Pedestrian model="unspecified" mass="70.0" name="walker.pedestrian.0001"
              pedestrianCategory="pedestrian">
    <BoundingBox> ... </BoundingBox>
    <Properties />
  </Pedestrian>
</ScenarioObject>
<ScenarioObject name="barrier_0">
  <MiscObject mass="50.0" miscObjectCategory="obstacle"
              name="static.prop.streetbarrier"> ... </MiscObject>
</ScenarioObject>
```

The crossing is a `FollowTrajectoryAction`: a polyline with a time at each vertex. The
walker reaches the lane's edge at 12 s, stands on it until 40 s, then crosses:

```xml
<Shape><Polyline>
  <Vertex time="0"><Position>
    <WorldPosition x="240.0" y="-138.5" z="0.3" h="1.5708" /></Position></Vertex>
  <Vertex time="12"><Position>
    <WorldPosition x="240.0" y="-130.5" z="0.3" h="1.5708" /></Position></Vertex>
  <Vertex time="40"><Position>
    <WorldPosition x="240.0" y="-129.5" z="0.3" h="1.5708" /></Position></Vertex>
  <Vertex time="52"><Position>
    <WorldPosition x="240.0" y="-126.0" z="0.3" h="1.5708" /></Position></Vertex>
</Polyline></Shape>
```

The clearance check is a `RelativeDistanceCondition` with `freespace="true"` (between the
two bounding boxes, not their centres) that ends the run with `exitFailure`.

```bash
ros2 run swb run scenarios/pedestrian_crossing.xosc
```

**What you see**: in RViz the walker appears as a detected object, and the ego stops
short of the crossing until it is clear; then `passed   <run id>  <run directory>`.
Autoware sees the walker with its own lidar and cameras: nothing tells it where the
pedestrian is.

**Check**: `echo $?` prints `0`.

## 2. A traffic light

```bash
cp $E/projects/first_drive/scenarios/red_light.xosc scenarios/
```

CARLA's own light cycling is frozen while a scenario runs, and an uncommanded light is
green. This scenario turns the light ahead of the ego red in its Init, by the Lanelet2 id
of the signal, and green again at 150 s:

```xml
<TrafficSignalAction>
  <TrafficSignalStateAction name="43763" state="red" />
</TrafficSignalAction>
```

The world server knows which CARLA light that id is from the map's signal table, which
it writes into the project's `state/Town01/` the first time the stack loads Town01.

```bash
ros2 run swb run scenarios/red_light.xosc
```

**What you see**: the ego stops at the stop line, waits, and drives on when the light turns
green; the run passes in about three minutes.

**Check**: `ls state/Town01/` lists `traffic_lights.yaml`.

## 3. A car the simulator drives

This one comes from the scenario library of the workspace; copy it under a shorter name:

```bash
cp $E/scenarios/basic/town01_simulator_autopilot.xosc scenarios/autopilot.xosc
```

Its `LogicFile` is `$(env SWB_MAPS)/Town01`, as in every scenario of a project: the stack
sets `SWB_MAPS` to the project's `assets/maps`.

It has, beside the ego, one scripted car (`npc_route`) and one whose controller is
`simulator_autopilot`:

```xml
<ScenarioObject name="npc_autopilot">
  <CatalogReference catalogName="vehicles" entryName="tesla_model3"> ...
  </CatalogReference>
  <ObjectController>
    <Controller name="simulator_autopilot"><Properties /></Controller>
  </ObjectController>
</ScenarioObject>
```

Its `AcquirePositionAction` becomes a goal for CARLA's Traffic Manager, which plans the
lane route, brakes for vehicles ahead and red lights, and stops at the goal. An absolute
`SpeedAction` sets its desired speed.

```bash
ros2 run swb project check
ros2 run swb run scenarios/autopilot.xosc
```

**What you see**: `project: ok`, then the run passes when all three have reached their
goals and nothing has collided.

**Check**: `ros2 run swb runs list` shows the three new runs `passed`.

## What happened

The bridge turns each entity into a simulator actor by its kind and its controller. The
ego and `agent` entities are driven by pilots, outside SSv2; scripted ones are poses SSv2
computes each frame (as physics bodies tracking that pose, by default); a
`simulator_autopilot` car is the simulator's own. A `simulator_autopilot` car does not take
`LaneChangeAction`, a relative `SpeedAction` or `FollowTrajectoryAction`: its driver
decides those, and the scenario fails naming the action rather than ignoring it.

On Isaac Sim the same files run as far as the simulator can drive them: scripted
pedestrians, obstacles and cars the same way; `simulator_autopilot` is refused (Isaac has no
driver of its own), and only one entity can have a pilot (the ego). Every rule for each kind
of road user: [Writing scenarios](../design/scenario-authoring.md).

## Exercises

1. **Easy.** Make the pedestrian cross 10 s later: add 10 to every vertex `time` after the
   first. Check: the run still passes, and its duration in `runs list` is about 10 s longer.
2. **Medium.** Leave the light red: delete the `green` event from your copy of
   `red_light.xosc`. Check: the run ends `failed` on its 300 s timeout, and `runs show
   red_light` shows the ego stopped at the line.
3. **Open.** Give `npc_autopilot` a goal on the ego's own road ahead of it, so that the ego
   follows a car that brakes for lights. Check: the run passes, and the ego never comes
   within 2 m of it (add a `RelativeDistanceCondition` that fails the run).

## If it does not work

| Symptom | Fix |
|---|---|
| `project check`: `LogicFile ... does not name a project map` | a scenario from elsewhere names its map another way: set its `LogicFile` to `$(env SWB_MAPS)/Town01` |
| The light never changes; the bridge's log says `Cannot read the signal table` | the world server could not write `state/Town01/traffic_lights.yaml`: check the project directory is writable |
| `simulator_autopilot` refused | the stack is on Isaac Sim, which has no driver of its own |
| The pedestrian run fails within 1 m | Autoware did not see the walker in time: look at `runs why pedestrian_crossing` and at perception in `runs/<id>/logs/pilot-ego/` |

## Next

[Tutorial 4: several pilots](04-several-pilots.md): Autoware, CARLA's BasicAgent and a
program of your own in one run.
