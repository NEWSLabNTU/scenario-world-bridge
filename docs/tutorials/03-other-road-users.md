# Tutorial 3: Other road users

**Goal**: three scenarios in the project, each with a different kind of road user: a
pedestrian crossing among obstacles, a traffic light that the scenario turns red, and a car
that CARLA's own driver takes to its goal. All three pass with Autoware driving the ego.

```
$ ros2 run swb runs list
RUN                                         VERDICT  SCENARIO                 DURATION
...pedestrian_crossing_r0                   passed   pedestrian_crossing.xosc ...
...red_light_r0                             passed   red_light.xosc           ...
...autopilot_r0                             passed   autopilot.xosc           ...
```

## You will learn

- how an entity's **controller name** determines what moves it, and with what
  consequences;
- placing pedestrians and obstacles, and moving a pedestrian along a timed path;
- commanding a traffic light from the scenario;
- handing a car to the simulator's own driver.

## Before you start

- [Tutorial 2](02-write-a-scenario.md): the project `~/my_first_drive` with its stack
  running on Town01, and the scenario written there. Every terminal: `source ~/swb.bash`.
- About 40 minutes, most of it spent in the runs.

The scenarios ship with the workspace and are copied into the project:

```bash
cd ~/my_first_drive
E=$(ros2 pkg prefix --share swb_examples)
```

Everything here is standard OpenSCENARIO. How an entity moves is decided by the name of
its controller:

| Road user | In the scenario | Moved by | Physics |
|---|---|---|---|
| Pedestrian | `<Pedestrian>`, no controller or controller `""` | SSv2: the bridge places it each frame | off |
| Obstacle | `<MiscObject>` | stays where it is put | off |
| Car, scripted | `<Vehicle>` or a catalog car, controller `""` | SSv2, as a pose each frame | off |
| Car, driven by the simulator | controller `simulator_autopilot` | CARLA's Traffic Manager | on |
| Car with a pilot | controller `agent` | a pilot of the project ([Tutorial 4](04-several-pilots.md)) | on |

A scripted car or pedestrian follows the scenario exactly in every run, and a collision
never displaces it.

## 1. A pedestrian among obstacles

```bash
cp $E/projects/first_drive/scenarios/pedestrian_crossing.xosc scenarios/
```

The ego drives west past two street barriers while a pedestrian crosses its lane. It
passes if the ego reaches its goal without touching the walker or passing too close to
it (below). The pedestrian and a barrier
are declared with CARLA's asset names, which the bridge translates for each simulator:

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

The crossing is a `FollowTrajectoryAction`, a polyline with a time at each vertex. The
walker reaches the edge of the lane at 12 s, waits there until 40 s, then crosses:

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

The clearance check is two `RelativeDistanceCondition`s in one `ConditionGroup`, so both
must hold: `lateral` less than 0.3 and `longitudinal` less than 1.0, each with
`freespace="true"` (between the two bounding boxes, not their centres). It ends the run
with `exitFailure`.

```bash
ros2 run swb run scenarios/pedestrian_crossing.xosc
```

**What you see**: the ego stops short of the crossing until it is clear; then
`passed   <run id>  <run directory>`. With RViz enabled (`rviz: true`,
[Tutorial 1, step 4](01-first-drive.md#4-run-the-scenario)), the walker appears as a
detected object, since Autoware perceives it only with its own sensors.

**Check**: `echo $?` prints `0`.

## 2. A traffic light

```bash
cp $E/projects/first_drive/scenarios/red_light.xosc scenarios/
```

An uncommanded light is green. This scenario turns the light ahead of the ego red in its
Init, by the Lanelet2 id of the signal, and green again at 150 s:

```xml
<TrafficSignalAction>
  <TrafficSignalStateAction name="43763" state="red" />
</TrafficSignalAction>
```

The world server resolves that id to a CARLA light through the map's signal table, which
the fetched Town01 includes. For a map without one, the world server writes one into the
project's `state/<map>/`, the directory where it keeps what it derives for a map.

```bash
ros2 run swb run scenarios/red_light.xosc
```

**What you see**: the ego stops at the stop line, waits, and drives on when the light turns
green; the run passes in about three minutes.

**Check**: `ls assets/maps/Town01/carla/` lists `traffic_lights.yaml`, the signal table
through which the id was resolved.

## 3. A car the simulator drives

This scenario comes from the workspace's scenario library and is copied under a shorter
name:

```bash
cp $E/scenarios/basic/town01_simulator_autopilot.xosc scenarios/autopilot.xosc
```

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
route, brakes for vehicles ahead and for red lights, and stops at the goal. An absolute
`SpeedAction` sets its desired speed.

```bash
ros2 run swb project check
ros2 run swb run scenarios/autopilot.xosc
```

**What you see**: `project: ok`, then the run passes when all three have reached their
goals and nothing has collided.

**Check**: `ros2 run swb runs list` shows the three new runs `passed`.

## What happened

The bridge turns each entity into a simulator actor by its kind and its controller, as
the table above lists. A `simulator_autopilot` car does not take `LaneChangeAction`, a relative `SpeedAction` or `FollowTrajectoryAction`, since its
driver decides those: the scenario fails naming the action rather than ignoring it.

On Isaac Sim the same files run as far as the simulator can drive them: scripted
pedestrians, obstacles and cars the same way; `simulator_autopilot` is refused (Isaac has no
driver of its own), and only one entity can have a pilot (the ego). Every rule for each kind
of road user: [What a scenario may use](../reference/scenarios.md).

## Exercises

1. **Easy.** Make the pedestrian cross 10 s later: add 10 to every vertex `time` after the
   first. Check: the run still passes, and its duration in `runs list` is about 10 s longer.
2. **Medium.** Leave the light red: delete the `green` event from the project's copy of
   `red_light.xosc`. Check: the run ends `failed` on its 300 s timeout, and `runs show
   red_light` shows the ego stopped at the line.
3. **Open.** Give `npc_autopilot` a goal on the ego's own road ahead of it, so that the ego
   follows a car that brakes for lights. Check: the run passes, and the ego never comes
   within 2 m of it (add a `RelativeDistanceCondition` that fails the run).

## If it does not work

| Symptom | Fix |
|---|---|
| `project check`: `LogicFile ... does not name a project map` | a scenario from elsewhere names its map another way: set its `LogicFile` to `$(env SWB_MAPS)/Town01` |
| The light never changes; the bridge's log says `Cannot read the signal table` | the world server found no signal table: for a map without one it writes `state/<map>/traffic_lights.yaml`, so the project directory must be writable |
| `project check` (or the stack at start) refuses: `controller simulator_autopilot runs on CARLA only, the simulator is isaac ...` | the project is on Isaac Sim, which has no driver of its own |
| The pedestrian run fails on the clearance check | Autoware did not perceive the walker in time: `runs why <run id>` (the id from `runs list`, since a name that matches several runs is refused) and the perception logs in `runs/<id>/logs/pilot-ego/` show why |

## Next

[Tutorial 4: several pilots](04-several-pilots.md): Autoware, CARLA's BasicAgent and a
user-written program in one run.
