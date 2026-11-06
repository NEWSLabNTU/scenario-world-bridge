# Tutorial 6: Configure the system

What you can change and where, each with a worked example: the bridge, the CARLA and Isaac
world servers, the weather, ports and ROS domains, and the Autoware side. Builds on
[Tutorial 1](01-first-run.md) (CARLA) or [Getting started on Isaac Sim](../getting-started/isaac.md).
The [User Guide](../user-guide.md) has the full reference tables.

## 1. Where settings live

A run has two sides, and each part reads its own settings:

| Part | Its settings | Chosen by |
|---|---|---|
| The bridge (SSv2's side) | `bridge_config.yaml` | `config_file:=` on the simulation launch |
| CARLA's world server | `swb_carla_world.yaml` | `swb_carla_world_config:=` on `carla_simulation.launch.xml` |
| Isaac's world server | launch arguments and environment | `swb_isaac_world.launch.xml` |
| The vehicle side (Autoware) | launch arguments of the Autoware profile | `ego_av.launch.xml`, `background_av.launch.xml`, aib's `isaac_simulator.launch.xml` |

For the two files, a launch argument of the same name wins over the file, and the file over
the built-in default. Each refuses a key that belongs to the other, naming where it went, so
a misplaced setting fails loudly rather than being ignored. Both log the file they loaded.

The installed defaults are the place to start a copy from:

```bash
B=$(ros2 pkg prefix scenario_world_bridge)/share/scenario_world_bridge/config/bridge_config.yaml
W=$(ros2 pkg prefix swb_carla_world)/share/swb_carla_world/config/swb_carla_world.yaml
```

Every key is commented in them.

## 2. The bridge: how NPCs move, contacts, traces

Keep a copy beside your scenarios and change what you need. For example, exact NPC poses
instead of physics-tracked ones, and contact reports in a directory of your own:

```bash
mkdir -p ~/my_scenarios && cp $B ~/my_scenarios/bridge.yaml
```

```yaml
# ~/my_scenarios/bridge.yaml (changed lines)
npc_motion: kinematic                      # tracked (default): physics bodies following SSv2
contact_report_dir: /home/me/swb_contacts  # where each run's contacts go
```

```bash
ROS_DOMAIN_ID=9 ros2 launch swb_launch carla_simulation.launch.xml \
    config_file:=$HOME/my_scenarios/bridge.yaml
```

The same file works for `isaac_simulation.launch.xml`. Check what the running bridge loaded:

```bash
ROS_DOMAIN_ID=9 ros2 param get /scenario_world_bridge config_file
```

Some keys are also launch arguments, for a one-off change without a file:
`npc_motion:=kinematic`, `weather:=ClearNoon`, `gap_trace_dir:=<dir>`,
`contact_report_dir:=<dir>`.

| Key | What it does |
|---|---|
| `npc_motion` | `tracked`: NPCs are physics bodies driven toward SSv2's pose (real velocities, walk cycles, physical contacts); `kinematic`: teleported each frame, exact but weightless |
| `blueprint_map` | SSv2 asset names to simulator models (`sample_vehicle: "vehicle.tesla.model3"`) |
| `weather` | a weather preset applied at connect and after every map load (section 4) |
| `contact_report_dir` | each ego run's contacts as JSON, for `physical_verdict` (Tutorial 3); `off` for none |
| `gap_trace_dir` | a per-frame trace of both worlds' poses, for diagnosis |
| `world.*` | the world server's endpoint and timeouts (set by the simulation launch) |

## 3. The world server

### CARLA

`swb_carla_world.yaml` is about CARLA itself. Two common changes: CARLA on another machine,
and a fixed Traffic Manager seed so that `simulator_autopilot` cars drive the same way on
every run:

```bash
cp $W ~/my_scenarios/carla_world.yaml
```

```yaml
# ~/my_scenarios/carla_world.yaml (changed lines)
carla:
  host: "192.168.1.50"     # the machine CARLA runs on
  port: 2000
traffic_manager:
  seed: 7                  # reseeded at every scenario start
spectator: follow          # CARLA's window looks straight down on the ego (off by default)
```

```bash
ROS_DOMAIN_ID=9 ros2 launch swb_launch carla_simulation.launch.xml \
    swb_carla_world_config:=$HOME/my_scenarios/carla_world.yaml
```

The host and port alone are also launch arguments: `carla_host:=192.168.1.50 carla_port:=2000`.
Other keys: `collision_monitor` (CARLA's own contact sensors, for Tutorial 3's verdict),
`carla_tick_seconds` and `substeps` (the simulation step), `map_alias` (map directory names
to CARLA town names, `Town10: Town10HD`).

**The car's physics from its spec.** CARLA's blueprint has its own mass, engine and tyres.
To give the ego the vehicle spec's instead (the same numbers Isaac's ego uses):

```bash
ROS_DOMAIN_ID=9 ros2 launch swb_launch carla_simulation.launch.xml \
    vehicle_physics:=$(ros2 pkg prefix sim_vehicle_spec)/share/sim_vehicle_spec/generated/tesla_model3/carla_physics.yaml
```

and pass the same file to the vehicle side as `carla_physics:=`, which reads it back and
checks it.

### Isaac Sim

The Isaac world server takes launch arguments and a few environment variables:

```bash
ISAAC_PORT=5590 EGO_ROS_DOMAIN_ID=1 \
ros2 launch swb_isaac_world swb_isaac_world.launch.xml \
    world:=$CARLA_MAPS/Town01/isaac/world.yaml,$CARLA_MAPS/Town02/isaac/world.yaml \
    render:=window physics_hz:=120
```

| Argument / variable | What it does |
|---|---|
| `world:=` | worlds to preload, comma separated (world files, map directories or USD files); the first stays loaded, and switching to another is fast |
| `render:=` | `auto` (a window when `DISPLAY` is set), `window` or `headless` |
| `ego:=` | `default` (aib's Model 3 with Autoware's sensors), `none` (a box), or your own ego config |
| `physics_hz:=` | the physics rate (default 60) |
| `ISAAC_PORT` | the world server's port (5590); `isaac_simulation.launch.xml isaac_port:=` must match |
| `EGO_ROS_DOMAIN_ID` | the ROS domain of the ego's sensors and commands: the vehicle side's `ROS_DOMAIN_ID` |
| `ISAAC_KIT_ARGS` | Isaac's own options, e.g. `--/renderer/multiGpu/enabled=false --/renderer/activeGpu=1` to pin a GPU |

Road surfaces (wet asphalt, gravel, mud) are per map, in `<map dir>/surfaces.yaml`
([off-road surfaces](../design/off-road-surfaces.md)).

## 4. The weather

On CARLA the weather is set through ROS services on the bridge, in the simulation side's
domain, by preset or by CARLA's parameter names:

```bash
ROS_DOMAIN_ID=9 ros2 service call /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather "{preset: WetSunset}"

# parameters by name; those not named keep their values
ROS_DOMAIN_ID=9 ros2 service call /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather "{names: [cloudiness, fog_density], values: [80.0, 30.0]}"

ROS_DOMAIN_ID=9 ros2 service call /scenario_world_bridge/get_weather swb_interfaces/srv/GetWeather
```

The world server keeps it across map loads and CARLA restarts. To start every run with a
preset, use `weather:=ClearNoon` on the simulation launch (or `weather:` in either file). The
presets and parameters are listed in [User Guide §5](../user-guide.md#5-configure-the-worlds-weather).

On Isaac a scenario's own `EnvironmentAction` (weather, time of day) sets the sky, fog and
rain; there is no weather service.

## 5. Ports and ROS domains

| What | Default | Set with |
|---|---|---|
| SSv2 to the bridge | 5555 | `ssv2_port:=` on the simulation launch |
| Agent relay | 5560 | `agent_port:=` on the simulation launch, `relay:=tcp://<host>:<port>` on vehicle sides |
| CARLA world server | 5591 | `world_port:=` on `carla_simulation.launch.xml` |
| Isaac world server | 5590 | `ISAAC_PORT`, and `isaac_port:=` on `isaac_simulation.launch.xml` |
| CARLA | 2000 | CARLA's `-carla-rpc-port`, `carla_port:=` |
| Scenario domain | 9 | `ROS_DOMAIN_ID` of the simulation side and the scenario |
| Ego domain | 1 | `ROS_DOMAIN_ID` of the ego's vehicle side (and `EGO_ROS_DOMAIN_ID` on Isaac) |
| Background AVs | 2, 3, ... | `ROS_DOMAIN_ID` of each background vehicle side |

Two copies of the system on one machine move every one of these together, except CARLA's
port if they share one CARLA (a world server refuses a second on the same CARLA; use a second
CARLA on another port). Keep every `ROS_DOMAIN_ID` at 57 or below.

## 6. The Autoware side

### Your Autoware

The vehicle sides use whatever Autoware the terminal sourced: the binary packages, or your
own build of the same release. Source yours instead of `/opt/autoware/1.5.0/setup.bash`, then
this workspace. To keep your own top-level launch file (a fork, extra nodes), name it:

```bash
ROS_DOMAIN_ID=1 ros2 launch swb_launch ego_av.launch.xml \
    map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560 \
    autoware_launch:=$HOME/autoware/src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml
```

The vehicle sides wrap it in the **Autoware profile** (`autoware_sim_profile`), which sets
the car's packages and simulation-tuned parameters. Each group can be turned off to keep
yours:

```bash
# Faster driving (planning's max_vel, m/s), CenterPoint instead of lidar clustering,
# and your own localization parameters
ROS_DOMAIN_ID=1 ros2 launch swb_launch ego_av.launch.xml \
    map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560 \
    max_velocity:=11.11 lidar_detection_model:=centerpoint sim_localization:=false
```

| Argument | Default | Effect |
|---|---|---|
| `max_velocity:=` | 4.17 (15 km/h) | Autoware's planning speed limit |
| `lidar_detection_model:=` | `clustering` | `centerpoint` uses TensorRT (builds engines on first start) |
| `sim_localization:=false` | on | your NDT/EKF parameters instead of the simulation-tuned ones |
| `sim_perception:=false` | on | your perception settings (traffic lights from a camera, not the simulator) |
| `sim_control:=false` | on | your controller parameters |
| `sim_planning:=false` | on | your planning parameters (and `max_velocity` no longer applies) |
| `sim_system:=false` | on | your system and diagnostics settings |

The same arguments work on `background_av.launch.xml` and aib's `isaac_simulator.launch.xml`.
Two fixes Autoware 1.5.0 lacks, and the TensorRT version it needs, are in
[User Guide, *Your own Autoware*](../user-guide.md#your-own-autoware).

### Another car

The car is described once, in `sim_vehicle_spec`'s `vehicles/<car>.yaml`, and everything
else is generated from it: CARLA's physics, Isaac's rig, SSv2's catalog entry, Autoware's
vehicle and sensor-kit packages. Two cars ship: `tesla_model3` (the default) and
`drift_mustang`. A vehicle side selects one with `vehicle_model:=` and `sensor_model:=`, the
scenario with its catalog entry (`entryName=`), and the bridge refuses a run where they
disagree. Adding a car: [vehicle side](../design/vehicle-side.md).

### Not Autoware at all

Any autopilot that registers with the agent relay can drive the ego or a background vehicle:
a Python class or a ROS 2 stack described in YAML. [Writing an agent](../agent-interface.md).
