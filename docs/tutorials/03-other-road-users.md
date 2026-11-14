# Tutorial 3: Other road users and more AVs

Pedestrians, obstacles, other cars and other autonomous vehicles in a scenario, one kind at
a time, each with a starter scenario you can run as it is. Builds on the stack on CARLA with
Autoware on Town01 ([Build your own stack](../build-your-own-stack.md), sections 1 to 5)
and [Tutorial 1](01-prepare-a-scenario.md)'s scenario file.

Everything here is standard OpenSCENARIO. What decides how an entity moves is its
**controller's name**:

| Road user | In the scenario | Moved by | Physics | Extra terminal |
|---|---|---|---|---|
| Pedestrian | `<Pedestrian>`, default controller | SSv2: the bridge places it each frame | off | no |
| Obstacle | `<MiscObject>` | stays where it is put | off | no |
| Car, scripted | `<Vehicle>` or a catalog entry, default controller (`""`) | SSv2's behavior tree, as the pose each frame | off | no |
| Car, driven by the simulator | controller `simulator_autopilot` | the simulator's own driver (CARLA's Traffic Manager) | on | no |
| Another autonomous vehicle | controller `agent` | its own autopilot, e.g. a second Autoware | on | one background AV launch per vehicle |

A scripted car or pedestrian goes exactly where the scenario says, every run, but a
collision never pushes it. The last two are driven with physics, react to what is around
them and are seen by SSv2 where the simulator reports them. [Writing
scenarios](../design/scenario-authoring.md) has every rule; this tutorial shows each in use.

The scenarios below are installed with `swb_examples`:

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
```

Run each one against the live stack, in the scenario domain, with its own
`output_directory`. `ros2 launch` takes the same arguments as `play_launch launch`.

## 1. A pedestrian crossing, and obstacles

`basic/town01_pedestrian.xosc`: the ego drives west past two street barriers while a
pedestrian crosses its lane. It passes if the ego reaches its goal without coming within 1 m
of the walker.

The pedestrian and a barrier are declared with the simulator's asset names
(`walker.pedestrian.0001`, `static.prop.streetbarrier`; the bridge's asset map translates
them for each simulator):

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

The crossing is a `FollowTrajectoryAction`: a polyline with a time at each vertex. Here the
walker reaches the lane's edge at 12 s, stands on it until 40 s, then crosses:

```xml
<FollowTrajectoryAction initialDistanceOffset="0">
  <TimeReference>
    <Timing domainAbsoluteRelative="relative" offset="0" scale="1" />
  </TimeReference>
  <TrajectoryFollowingMode followingMode="position" />
  <TrajectoryRef><Trajectory name="cross_north" closed="false"><ParameterDeclarations />
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
  </Trajectory></TrajectoryRef>
</FollowTrajectoryAction>
```

The verdict's clearance check is a `RelativeDistanceCondition` with `freespace="true"`
(between the two bounding boxes, not their centres) that ends the scenario with
`exitFailure`. Run it:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$S/basic/town01_pedestrian.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/pedestrian
```

Autoware has to see the walker with its own lidar and cameras: nothing tells it where the
pedestrian is. In RViz the walker appears as a detected object, and the ego stops short of
the crossing until it is clear.

To make your own: take the poses from `carla_poses` (Tutorial 1), put the walker's start
beside the road, and give the trajectory's vertices times that bring it into the lane when
the ego is a few seconds away.

## 2. Other cars: scripted, or driven by the simulator

A scripted car is what Tutorial 1's `lead` is: a `TeleportAction` to place it and a
`SpeedAction` to move it along its lane. SSv2 computes its pose each frame, so it does
exactly the same thing every run.

A car driven by the simulator only needs its controller renamed:

```xml
<ScenarioObject name="npc_autopilot">
  <CatalogReference catalogName="vehicles" entryName="tesla_model3">
    ...
  </CatalogReference>
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
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$S/basic/town01_simulator_autopilot.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/autopilot
```

It passes when all three have reached their goals and nothing has collided. A
`simulator_autopilot` car does not take `LaneChangeAction`, a relative `SpeedAction` or
`FollowTrajectoryAction`: its driver decides those, and the scenario fails with a message
naming the action rather than ignoring it.

## 3. A background autonomous vehicle

A vehicle whose controller is `agent` is driven by an autopilot that registers with the
stack's agent relay under the entity's name. Usually it is a second, complete Autoware: the
ego's Autoware perceives it with its own sensors, and it perceives the ego. Neither is an
SSv2 puppet.

`multi_av/town01_two_av.xosc` has the ego and one background AV, `bg_av_1`:

```xml
<ScenarioObject name="bg_av_1">
  <CatalogReference catalogName="vehicles" entryName="tesla_model3">
    ...
  </CatalogReference>
  <ObjectController>
    <Controller name="agent"><Properties /></Controller>
  </ObjectController>
</ScenarioObject>
```

Give it a `TeleportAction` (where it starts) and an `AcquirePositionAction` (its goal) in
`Init`, as the ego has. The bridge spawns `bg_av_1` as a physics vehicle whose CARLA role
name is the entity name, so its vehicle side finds it in CARLA.

Then start its vehicle side beside the stack: a background AV launch, in a ROS domain of
its own, named after the entity:

```bash
ROS_DOMAIN_ID=2 play_launch launch \
    --container-mode isolated \
    --composable-respawn on-crash \
    --web-addr 127.0.0.1:8083 \
    swb_launch background_av.launch.xml \
    entity:=bg_av_1 \
    map_path:=$CARLA_MAPS/Town01 \
    relay:=tcp://localhost:${AGENT_PORT:-5560}
```

`background_av.launch.xml` starts the same Autoware profile as the ego, with lidar
clustering on the CPU and traffic lights from the simulator instead of a camera, as the
ego has. What differs is the entity, the CARLA vehicle it attaches to (named after the
entity), a laser-scan occupancy grid (`occupancy_grid_map_method`), and RViz, which is off
unless `launch_rviz:=true`. The two `--container-mode` and
`--composable-respawn` flags reload an Autoware component that crashed
([User Guide](../user-guide.md#play_launch-flags)).

It is ready when the relay logs `entity 'bg_av_1': agent autoware registered`: open the
stack's web UI (`127.0.0.1:8084`), node `scenario_agent_relay`. Then:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$S/multi_av/town01_two_av.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/two_av
R=/tmp/two_av/scenario_test_runner/result.junit.xml
grep -o 'failures="[0-9]*" errors="[0-9]*"' $R | head -1
# failures="0" errors="0"
```

`bg_av_1` starts 90 m ahead of the ego in the same lane and drives to a goal 25 m past the
ego's, so the ego follows a real, Autoware-driven car. The scenario passes when both have
arrived and nothing has collided.

The background AV is long-lived: leave it running and it serves every scenario that has a
`bg_av_1`. A scenario that gives `bg_av_1` a goal while no agent of that name is registered
fails at once, naming the `entity:=` to start.

## 4. More vehicles

Each further autonomous vehicle is one more `agent` entity and one more background AV
launch, each in its own ROS domain, with its own `entity:=` and web UI port:

```bash
ROS_DOMAIN_ID=3 play_launch launch \
    --container-mode isolated \
    --composable-respawn on-crash \
    --web-addr 127.0.0.1:8086 \
    swb_launch background_av.launch.xml \
    entity:=bg_av_2 \
    map_path:=$CARLA_MAPS/Town01 \
    relay:=tcp://localhost:${AGENT_PORT:-5560}
```

- Keep every domain at 57 or below and different from the others in use
  ([ROS domains and DDS](../domains-and-dds.md)).
- Each Autoware is about 90 processes and several GB of GPU memory. On one machine with one
  GPU, two (the ego and one background AV) is the comfortable limit beside CARLA. With
  three or more, use `--container-mode observable` instead of the two flags above
  ([User Guide](../user-guide.md#play_launch-flags)).
- If the stack runs with `vehicle_physics:=<file>`, give each background AV the same file
  as `carla_physics:=<file>`.
- If CARLA or `SSV2_PORT` moved, also pass `carla_port:=`,
  `release_notify_endpoint:=tcp://127.0.0.1:<SSV2_PORT+1>` and
  `release_ack_endpoint:=tcp://127.0.0.1:<SSV2_PORT+2>`.

A background AV can run on another machine on CARLA: point `relay:=` at the stack's host,
`carla_host:=` at CARLA's, and the two release endpoints at the stack's host. Isaac Sim
carries no background AVs.

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
  ego vehicle (aib's); a background AV launch exists for CARLA only, so multi-vehicle
  scenarios run on CARLA.

## Next

- [Tutorial 4, configure the system](04-configure.md): the bridge's and the world server's
  settings, the weather, ports and domains.
- [Writing scenarios](../design/scenario-authoring.md): every rule for each kind of road user,
  traffic lights, collisions.
