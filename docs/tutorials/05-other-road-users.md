# Tutorial 5: Other road users

Pedestrians, obstacles, other cars and other autonomous vehicles in a scenario, one kind at
a time, each with a starter scenario you can run as it is. Builds on
[Tutorial 1](01-first-run.md)'s stack (CARLA, the simulation side, Autoware on the ego) and
[Tutorial 2](02-prepare-a-scenario.md)'s scenario file.

Everything here is standard OpenSCENARIO. What decides how an entity moves is its
**controller's name**:

| Road user | In the scenario | Moved by | Physics | Extra terminal |
|---|---|---|---|---|
| Pedestrian | `<Pedestrian>`, default controller | SSv2: the bridge places it each frame | off | no |
| Obstacle | `<MiscObject>` | stays where it is put | off | no |
| Car, scripted | `<Vehicle>` or a catalog entry, default controller (`""`) | SSv2's behavior tree, as the pose each frame | off | no |
| Car, driven by the simulator | controller `simulator_autopilot` | the simulator's own driver (CARLA's Traffic Manager) | on | no |
| Another autonomous vehicle | controller `agent` | its own autopilot, e.g. a second Autoware | on | one vehicle side per vehicle |

A scripted car or pedestrian goes exactly where the scenario says, every run, but a
collision never pushes it. The last two are driven with physics, react to what is around
them and are seen by SSv2 where the simulator reports them. [Writing
scenarios](../design/scenario-authoring.md) has every rule; this tutorial shows each in use.

The scenarios below are installed with `swb_examples`:

```bash
S=$(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios
```

Run each one as in Tutorial 1, step 4, with its own `output_directory`.

## 1. A pedestrian crossing, and obstacles

`basic/town01_pedestrian.xosc`: the ego drives west past two street barriers while a
pedestrian crosses its lane. It passes if the ego reaches its goal without coming within 1 m
of the walker.

The pedestrian and a barrier are declared with the simulator's asset names
(`walker.pedestrian.0001`, `static.prop.streetbarrier`; the bridge's asset map translates
them for each simulator):

```xml
<ScenarioObject name="pedestrian_0">
  <Pedestrian model="unspecified" mass="70.0" name="walker.pedestrian.0001" pedestrianCategory="pedestrian">
    <BoundingBox> ... </BoundingBox>
    <Properties />
  </Pedestrian>
</ScenarioObject>
<ScenarioObject name="barrier_0">
  <MiscObject mass="50.0" miscObjectCategory="obstacle" name="static.prop.streetbarrier"> ... </MiscObject>
</ScenarioObject>
```

The crossing is a `FollowTrajectoryAction`: a polyline with a time at each vertex. Here the
walker reaches the lane's edge at 12 s, stands on it until 40 s, then crosses:

```xml
<FollowTrajectoryAction initialDistanceOffset="0">
  <TimeReference><Timing domainAbsoluteRelative="relative" offset="0" scale="1" /></TimeReference>
  <TrajectoryFollowingMode followingMode="position" />
  <TrajectoryRef><Trajectory name="cross_north" closed="false"><ParameterDeclarations />
    <Shape><Polyline>
      <Vertex time="0"><Position><WorldPosition x="240.0" y="-138.5" z="0.3" h="1.5708" /></Position></Vertex>
      <Vertex time="12"><Position><WorldPosition x="240.0" y="-130.5" z="0.3" h="1.5708" /></Position></Vertex>
      <Vertex time="40"><Position><WorldPosition x="240.0" y="-129.5" z="0.3" h="1.5708" /></Position></Vertex>
      <Vertex time="52"><Position><WorldPosition x="240.0" y="-126.0" z="0.3" h="1.5708" /></Position></Vertex>
    </Polyline></Shape>
  </Trajectory></TrajectoryRef>
</FollowTrajectoryAction>
```

The verdict's clearance check is a `RelativeDistanceCondition` with `freespace="true"`
(between the two bounding boxes, not their centres) that ends the scenario with
`exitFailure`. Run it:

```bash
ROS_DOMAIN_ID=9 ros2 launch swb_launch scenario.launch.xml \
    scenario:=$S/basic/town01_pedestrian.xosc output_directory:=/tmp/pedestrian
```

Autoware has to see the walker with its own lidar and cameras: nothing tells it where the
pedestrian is. In RViz the walker appears as a detected object, and the ego stops short of
the crossing until it is clear.

To make your own: take the poses from `carla_poses` (Tutorial 2), put the walker's start
beside the road, and give the trajectory's vertices times that bring it into the lane when
the ego is a few seconds away.

## 2. Other cars: scripted, or driven by the simulator

A scripted car is what Tutorial 2's `lead` is: a `TeleportAction` to place it and a
`SpeedAction` to move it along its lane. SSv2 computes its pose each frame, so it does
exactly the same thing every run.

A car driven by the simulator only needs its controller renamed:

```xml
<ScenarioObject name="npc_autopilot">
  <CatalogReference catalogName="vehicles" entryName="tesla_model3"> ... </CatalogReference>
  <ObjectController>
    <Controller name="simulator_autopilot"><Properties /></Controller>
  </ObjectController>
</ScenarioObject>
```

Its `AcquirePositionAction` becomes a goal for CARLA's Traffic Manager, which plans the lane
route, brakes for vehicles ahead and for red lights, and stops at the goal. An absolute
`SpeedAction` sets its desired speed. `basic/town01_simulator_autopilot.xosc` has one of each
next to the ego, on separate streets:

```bash
ROS_DOMAIN_ID=9 ros2 launch swb_launch scenario.launch.xml \
    scenario:=$S/basic/town01_simulator_autopilot.xosc output_directory:=/tmp/autopilot
```

It passes when all three have reached their goals and nothing has collided. A
`simulator_autopilot` car does not take `LaneChangeAction`, a relative `SpeedAction` or
`FollowTrajectoryAction`: its driver decides those, and the scenario fails with a message
naming the action rather than ignoring it.

## 3. A background autonomous vehicle

A vehicle whose controller is `agent` is driven by an autopilot that registers with the
simulation side's agent relay under the entity's name. Usually it is a second, complete
Autoware: the ego's Autoware perceives it with its own sensors, and it perceives the ego.

```xml
<ScenarioObject name="bg_av_1">
  <CatalogReference catalogName="vehicles" entryName="tesla_model3"> ... </CatalogReference>
  <ObjectController>
    <Controller name="agent"><Properties /></Controller>
  </ObjectController>
</ScenarioObject>
```

Give it a `TeleportAction` (where it starts) and an `AcquirePositionAction` (its goal) in
`Init`, as the ego has. Then start its vehicle side: the same Autoware vehicle side as the
ego's, in a ROS domain of its own, named after the entity:

```bash
# Terminal 3b
ROS_DOMAIN_ID=2 ros2 launch swb_launch background_av.launch.xml \
    entity:=bg_av_1 map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560
```

`background_av.launch.xml` uses lighter perception (CPU lidar clustering, traffic lights from
the simulator instead of a camera), so a second stack fits on one GPU beside CARLA and the
ego's. It is ready when the relay (Terminal 2) logs `entity 'bg_av_1': agent autoware
registered`. Then:

```bash
ROS_DOMAIN_ID=9 ros2 launch swb_launch scenario.launch.xml \
    scenario:=$S/multi_av/town01_two_av.xosc output_directory:=/tmp/two_av
```

`bg_av_1` starts 90 m ahead of the ego in the same lane and drives to a goal 25 m past the
ego's, so the ego follows a real, Autoware-driven car. The scenario passes when both have
arrived and nothing has collided.

The vehicle side is long-lived: leave Terminal 3b running and it serves every scenario that
has a `bg_av_1`. A scenario that gives `bg_av_1` a goal while no agent of that name is
registered fails at once, naming the `entity:=` to start.

## 4. More vehicles

Each further autonomous vehicle is one more `agent` entity and one more vehicle side, each in
its own ROS domain and with its own `entity:=`:

```bash
ROS_DOMAIN_ID=3 ros2 launch swb_launch background_av.launch.xml \
    entity:=bg_av_2 map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560
```

Keep `ROS_DOMAIN_ID` at 57 or below (the DDS profile's port range). Each Autoware is about 90
processes and several GB of GPU memory; on one machine with one GPU, two (the ego and one
background AV) is the comfortable limit beside CARLA. A vehicle side can run on another
machine: point its `relay:=` at the simulation host.

Scripted cars, simulator-driven cars and pedestrians cost almost nothing by comparison: a
scenario can have dozens.

## 5. On Isaac Sim

The same files run on Isaac Sim ([Getting started on Isaac Sim](../getting-started/isaac.md)),
as far as the simulator can drive them:

- **Scripted pedestrians, obstacles and cars** work the same way: the world server places
  CARLA's walkers, props and vehicles from the asset set where SSv2 says.
- **`simulator_autopilot`** needs a driver inside the simulator. CARLA has Traffic Manager;
  the Isaac world server has none (it does not declare the world protocol's `AUTOPILOT`
  capability), so the bridge refuses the spawn and the scenario fails naming what is
  missing.
- **`agent` vehicles** need a vehicle side per vehicle. On Isaac the world server carries one
  ego vehicle (aib's); a background AV's vehicle side exists for CARLA only, so multi-vehicle
  scenarios run on CARLA.

## Next

- [Tutorial 6, configure the system](06-configure.md): the simulator's and the bridge's
  configuration, the weather, your own Autoware.
- [Writing scenarios](../design/scenario-authoring.md): every rule for each kind of road user,
  traffic lights, collisions.
