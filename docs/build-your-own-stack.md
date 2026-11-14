# Build your own stack

Your scenario, a simulator, your car, your autopilot and more cars, run as one launch. This
page assumes the workspace builds and a map is in place
([Getting started on CARLA](getting-started/carla.md) or
[on Isaac Sim](getting-started/isaac.md), stages 1 and 2).

Commands use `play_launch launch <package> <file> <arguments>`. `ros2 launch` takes the same
arguments, but play_launch must still be installed: the stack starts the ego, the vehicle
agent and the scenario as play_launch processes of their own.

## 0. What you build

| Slot | What you choose | Where you choose it |
|---|---|---|
| Simulator | CARLA, Isaac Sim, or a world server of your own | `carla.launch.xml`, `isaac.launch.xml`, `stack.launch.py sim:=` |
| Scenario | an OpenSCENARIO file, or free drive without one | `scenario:=`, `spawn:=` |
| Vehicle | a shipped car, or one built from your spec | `vehicle_model:=`, `vehicle_physics:=`, `vehicle_catalogs:=` |
| Autopilot | Autoware (the default), your Autoware, your launch, your agent | `ego_launch:=`, `ego_launch_args:=` |
| More cars | background AVs, each a full Autoware (CARLA) | `background_av.launch.xml` |

The groups run in separate ROS domains:

| Group | Domain |
|---|---|
| World server, ego autopilot, vehicle agent | `EGO_ROS_DOMAIN_ID` (1) |
| Bridge, agent relay, SSv2 | `SCENARIO_ROS_DOMAIN_ID` (9) |
| Each background AV | its own (2, 3, ...) |

Why, and how to look into each: [ROS domains and DDS](domains-and-dds.md).

## 1. The environment file

Every terminal starts by sourcing one file, say `~/swb.bash`:

```bash
source /opt/autoware/1.5.0/setup.bash
source ~/scenario-world-bridge/install/setup.bash
export CARLA_MAPS=~/scenario-world-bridge/data/carla-autoware-bridge

# one DDS for every process, exported after Autoware's setup
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SWB_SHARE=$(ros2 pkg prefix --share swb_launch)
export CYCLONEDDS_URI=file://$SWB_SHARE/config/cyclonedds-localhost.xml

# the domains (these are the defaults; any two different values from 0 to 57)
export EGO_ROS_DOMAIN_ID=1
export SCENARIO_ROS_DOMAIN_ID=9
```

On CARLA, add `CARLA_HOST` and `CARLA_PORT` if CARLA is not at `localhost:2000`. On Isaac,
add `ISAAC_ROOT`, `CARLA_USD` and `SWB_VEHICLE_USD` as in the Isaac guide, stage 1.

## 2. Build

```bash
cd ~/scenario-world-bridge
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep install --from-paths src --ignore-src -y
export CMAKE_POLICY_VERSION_MINIMUM=3.5
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
source install/setup.bash
ros2 run autoware_sim_profile setup_autoware_data
pip install 'play_launch>=0.30.0'
```

Rust and colcon's Rust plugin come first (the getting-started guides, stage 1). Check the
play_launch version with `play_launch --version`.

## 3. The simulator

**CARLA** runs apart and stays up between runs:

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    ~/CARLA_0.9.16/CarlaUE4.sh \
    -RenderOffScreen \
    -nosound \
    -carla-rpc-port=2000
```

Wait until it serves its port (`ss -lnt | grep ':2000 '`), one to three minutes. Do not add
`-quality-level=Low` if your scenarios use more than one town: CARLA 0.9.16 crashes when it
changes towns at Low. Once per town, write the traffic-light table:

```bash
ros2 run swb_carla_world swb_carla_world \
    --generate-signal-table $CARLA_MAPS/Town01 \
    --ros-args -p carla_port:=2000 -p port:=5599
```

**Isaac Sim**: install it and make the town's world file as in the Isaac guide, stages 1
and 2. Its world server is then part of the stack.

**A simulator of your own**: a package with `launch/world.launch.xml` that starts a world
server speaking swb's world protocol; see `stack.launch.py`'s header and the
[world server guide](../src/swb_world_protocol/docs/world-server.md).

Once the stack runs (section 5), ask the world server how it is, without changing anything:

```bash
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5591   # CARLA
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5590   # Isaac
```

## 4. The scenario

Start from a starter and keep your scenarios in your own repository:

```bash
mkdir -p ~/my_scenarios
S=$(ros2 pkg prefix --share swb_examples)/scenarios
cp $S/basic/town01_ego_drive.xosc ~/my_scenarios/my_drive.xosc
```

- **The map**: `RoadNetwork/LogicFile` names the map directory, as
  `$(env CARLA_MAPS)/Town01`. The stack's `map_path:=` must be the same town.
- **The car**: the ego's `CatalogReference` names an entry of the vehicle catalog the
  scenario's `CatalogLocations` points at (`sim_vehicle_spec`'s, or your own).
- **Poses** are in the map frame. On CARLA, with CARLA running, `carla_poses` prints them
  as OpenSCENARIO elements:

  ```bash
  ros2 run swb_launch carla_poses spawn-points | head
  ros2 run swb_launch carla_poses ahead 2 --distance 120
  ```

  On Isaac, use RViz's **Publish Point** and read `/clicked_point` in the ego domain
  (Isaac guide, stage 5).

[Writing scenarios](design/scenario-authoring.md) lists what the bridge supports.

## 5. The stack with Autoware

On CARLA, with CARLA up:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    ego_launch_args:="rviz:=true max_velocity:=11.11"
```

On Isaac:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch isaac.launch.xml \
    world:=$CARLA_MAPS/Town01/isaac/world.yaml \
    render:=headless \
    map_path:=$CARLA_MAPS/Town01
```

Wait until the ego's Autoware can take a scenario (the first start builds TensorRT engines
and takes minutes):

```bash
ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} ros2 run swb_launch ego_ready --timeout 600
# [ego-health] ok: ...
```

Then run scenarios against the live stack, from another terminal, as many as you like:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$HOME/my_scenarios/my_drive.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/my_drive
cat /tmp/my_drive/scenario_test_runner/result.junit.xml
```

`failures="0" errors="0"` means it passed. Other ways to run:

| You want | Add to the stack's command |
|---|---|
| One scenario, then everything ends (scripts, CI) | `scenario:=<.xosc>` `exit_on_scenario_end:=true` |
| SSv2's options | `scenario_args:="output_directory:=/tmp/run initialize_duration:=300"` |
| Free drive, no scenario: the ego placed, you drive in RViz | `spawn:=<x>,<y>,<z>,<yaw>` (needs `map_path`) |
| The bridge's settings | `side_args:="config_file:=$HOME/my/bridge.yaml weather:=ClearNoon"` |
| CARLA world server's settings | `world_config:=$HOME/my/swb_carla_world.yaml` |

A scenario started with the stack begins at once, while Autoware may still be starting; if
it fails waiting for Autoware, raise `initialize_duration` as above or start scenarios
after `ego_ready`. Logs go to `$SWB_LOG_ROOT` (default `./play_log`), under `ego/`,
`agent/` and `scenario/`. Ctrl-C on the stack ends all of it; CARLA stays up.

## 6. The vehicle

**A shipped car.** Two ship: `tesla_model3` (the default) and `drift_mustang`. On CARLA,
the car's physics come from its spec only when you name its `carla_physics.yaml`; without
it CARLA's blueprint keeps its own:

```bash
G=$(ros2 pkg prefix --share sim_vehicle_spec)/generated
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    vehicle_model:=drift_mustang \
    vehicle_physics:=$G/drift_mustang/carla_physics.yaml \
    scenario_args:="vehicle_model:=drift_mustang"
```

The scenario's ego must then use the catalog entry `drift_mustang`.

**Your own car** starts from a spec (`vehicles/<car>.yaml`, format in `sim_vehicle_spec`'s
`docs/spec-format.md`). `sim_vehicle_spec.gen` writes what the bridge and CARLA need, in
your repository:

```bash
python3 -m sim_vehicle_spec.gen \
    --spec vehicles/my_car.yaml \
    --catalog catalogs/vehicles.xosc \
    --spec-hash config/spec_hash.yaml \
    --carla carla
```

| Output | Used by |
|---|---|
| `catalogs/vehicles.xosc` | the scenario's `CatalogLocations`, and the bridge: `vehicle_catalogs:=` |
| `config/spec_hash.yaml` | the vehicle agent: `spec_hash_file:=` |
| `carla/carla_physics.yaml` | CARLA's world server: `vehicle_physics:=` |
| `carla/carla_sensors.yaml` | acb's CARLA sensors: `ego_launch_args:="vehicle_config:=..."` |

Autoware also needs the car's own packages, `<car>_description` (with its
`config/spec_hash.yaml` and pedal maps) and `<car>_sensor_kit_*`, installed in a workspace
you source; `vehicle_model:=<car>` selects them. They are made from the spec in
`autoware_sim_vehicle`, and the pedal maps are measured, not generated
([vehicle side](design/vehicle-side.md#a-vehicle-from-another-repository)). With those
installed, run it on CARLA from your repository's directory (absolute paths, as the
processes do not share your working directory):

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    vehicle_model:=my_car \
    vehicle_physics:=$PWD/carla/carla_physics.yaml \
    vehicle_catalogs:=$PWD/catalogs/vehicles.xosc \
    spec_hash_file:=$PWD/config/spec_hash.yaml \
    ego_launch_args:="vehicle_config:=$PWD/carla/carla_sensors.yaml" \
    scenario_args:="vehicle_model:=my_car"
```

On Isaac the car is an aib ego config (`rig:`, `body:`); `aib.gen` writes the rig. The
default autopilot is the Tesla's Autoware, so bring your own with `ego_launch`:

```bash
python3 -m aib.gen \
    --spec vehicles/my_car.yaml \
    -o isaac/my_car.rig.yaml
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch isaac.launch.xml \
    ego:=$PWD/isaac/ego.yaml \
    world:=$CARLA_MAPS/Town01/isaac/world.yaml \
    map_path:=$CARLA_MAPS/Town01 \
    ego_launch:='<your package> <your autopilot launch>' \
    vehicle_catalogs:=$PWD/catalogs/vehicles.xosc \
    spec_hash_file:=$PWD/config/spec_hash.yaml
```

**The spec check.** Before the ego starts, the bridge compares the `spec_hash` of each part:
the world server's body (on CARLA only when `vehicle_physics` names a file), the vehicle
agent's, and the catalog entry for the ego's body. When two differ the spawn fails:

```
Spawn 'ego': the ego's parts were built from different vehicle specs (the world
server (...) 480be4fd0d00, the vehicle agent 65f1ba86a28f): rebuild and restart the
stale one
```

Make all three from one spec: the same `vehicle_model` (or `spec_hash_file`),
`vehicle_physics` and catalog entry; after changing a spec, run the generators again,
rebuild, and restart the stack. Two related refusals:

- `the ego's body on ... is not the car SSv2 declared (the vehicle catalog's), more than
  0.02 m off`: the scenario's catalog entry and the simulator's body are different cars.
- `vehicle catalogs disagree on <body>`: the bridge stops at start, because your catalog
  gives a body that the installed catalog already uses (`vehicle.tesla.model3`,
  `vehicle.ford.mustang`) another spec. Give your car a body of its own.

## 7. Your autopilot

| Autopilot | Stack arguments |
|---|---|
| Autoware 1.5.0 with swb's profile | none (the default) |
| Your Autoware build | source it instead of `/opt/autoware/1.5.0` in `~/swb.bash` |
| Your own `autoware.launch.xml` | `ego_launch_args:="autoware_launch:=<path>"` |
| Your parameters instead of the profile's | `ego_launch_args:="sim_localization:=false sim_planning:=false"` (also `sim_perception`, `sim_control`, `sim_system`) |
| Your own launch, which registers its own agent | `ego_launch:='<package> <file>'` `agent:=false` |
| No ego launch: an agent you run yourself | `ego_launch:=none` |

`ego_launch` runs in the ego domain, as a play_launch process of its own, restarted when
it exits. With `agent:=true` (the default) swb's vehicle agent runs beside it and drives it
through Autoware's API; any other autopilot needs `agent:=false` and an agent of its own.

**CARLA's BasicAgent**, no Autoware (CARLA only; `pip install carla==0.9.16 networkx
shapely` first):

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    ego_launch:=none
```

```bash
echo "carla_pythonapi: $HOME/CARLA_0.9.16/PythonAPI/carla" > ~/carla_basic.yaml
ros2 run swb_agent carla_basic \
    --relay tcp://localhost:${AGENT_PORT:-5560} \
    --entity ego \
    --config ~/carla_basic.yaml
```

**An agent of your own** (Python, any autopilot):

```bash
ros2 run swb_agent swb-agent new my-pilot
cd my-pilot
ros2 run swb_agent swb-agent check my_pilot.agent:MyPilotAgent
python3 -m my_pilot \
    --relay tcp://localhost:${AGENT_PORT:-5560} \
    --entity ego
```

The agent reaches the relay over TCP, so its domain does not matter to swb. If it drives a
ROS autopilot, run both with the environment file and in one domain, other than the
scenario's. [Writing an agent](agent-interface.md) has the hooks and the YAML adapter.

## 8. More cars

On CARLA each further autonomous vehicle is an entity whose controller is `agent` in the
scenario, and a background AV launch in a domain of its own, named after the entity:

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

- Ready when the relay logs `entity 'bg_av_1': agent autoware registered`.
- A second one: `ROS_DOMAIN_ID=3`, `entity:=bg_av_2`, another `--web-addr` port.
- The starter `multi_av/town01_two_av.xosc` has one.
- With `vehicle_physics:=` on the stack, give each the same file as `carla_physics:=`.
- If CARLA or `SSV2_PORT` moved, also pass `carla_port:=`,
  `release_notify_endpoint:=tcp://127.0.0.1:<SSV2_PORT+1>` and
  `release_ack_endpoint:=tcp://127.0.0.1:<SSV2_PORT+2>`.

Each Autoware is about 90 processes and several GB of GPU memory: on one GPU, the ego and
one background AV beside CARLA is the comfortable limit. Scripted cars and pedestrians cost
almost nothing ([Tutorial 5](tutorials/03-other-road-users.md)). Isaac carries one ego
only.

## 9. Judge

A directory of scenarios, one verdict each, merged:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 run swb_launch run_suite \
    ~/my_scenarios \
    --output ~/swb_results/2026-11-14 \
    --relay tcp://localhost:${AGENT_PORT:-5560} \
    --physical-verdict \
    -- port:=${SSV2_PORT:-5555}
```

Read `~/swb_results/2026-11-14/suite.junit.xml`; the exit status is 0 only if every test
passed. `--physical-verdict` adds CARLA's contacts as a test per scenario.
[Tutorial 3](tutorials/02-run-and-judge.md) explains both verdicts.

## 10. Two stacks on one host

Move both domains, `SSV2_PORT`, `AGENT_PORT`, `WORLD_PORT` (CARLA, and a second CARLA) or
`ISAAC_PORT`, and the `--web-addr` ports, in the second stack's environment file. The table
is in [ROS domains and DDS](domains-and-dds.md#two-stacks-on-one-host).

## 11. When it does not work

| Symptom | Look at |
|---|---|
| The stack stops at once: `stack: ... is 60: use 0-57` or `are both 9` | the domains (section 1); [ROS domains and DDS](domains-and-dds.md) |
| `ros2 topic list` is empty, `ego_ready` never says ok | the terminal's `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`, and `ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1}` |
| The ego and the scenario never start | play_launch missing or older than 0.30.0: `play_launch --version` |
| The scenario fails waiting for Autoware's state | `ego_ready` first; `map_path` must be the scenario's town |
| The scenario never engages | the relay's log (`scenario_agent_relay` in the stack's web UI) says whether the agent registered; with `ego_launch:=none`, start your agent |
| `the ego's parts were built from different vehicle specs` | section 6, the spec check |
| `Cannot read the signal table`, lights never change | section 3, `--generate-signal-table` for that town |
| `CARLA version mismatch` | CARLA and the build are different versions |
| A background AV never registers | its `entity:=` must be the scenario's entity name; its domain must be its own, at most 57 |
| The build fails | the getting-started guides' tables |
