# Tutorial 4: Configure the system

What you can change and where, each with a worked example: the bridge, the CARLA and Isaac
world servers, the weather, and ports and ROS domains. It builds on the stack
([Build your own stack](../build-your-own-stack.md), sections 1 to 5). The
[User Guide](../user-guide.md) has the full reference tables.

Commands use `play_launch launch`; `ros2 launch` takes the same arguments, but play_launch
must still be installed, because the stack starts its ego and scenario as play_launch
processes of their own.

Your car, your Autoware and an autopilot that is not Autoware are in
[Build your own stack](../build-your-own-stack.md), sections 6 and 7.

## 1. Where settings live

| Part | Its settings | Stack argument |
|---|---|---|
| The bridge (SSv2's side) | `bridge_config.yaml` | `side_args:="config_file:=<file>"` |
| CARLA's world server | `swb_carla_world.yaml` | `world_config:=<file>` on `carla.launch.xml` |
| Isaac's world server | launch arguments and environment | `isaac.launch.xml`'s own arguments |
| The ego's autopilot | launch arguments of the Autoware profile | `ego_launch_args:="name:=value ..."` |

For the two files, a launch argument of the same name wins over the file, and the file over
the built-in default. Each refuses a key that belongs to the other, naming where it went,
so a misplaced setting fails loudly rather than being ignored. Both log the file they
loaded.

Start a copy from the installed defaults; every key is commented in them:

```bash
B=$(ros2 pkg prefix --share scenario_world_bridge)/config/bridge_config.yaml
W=$(ros2 pkg prefix --share swb_carla_world)/config/swb_carla_world.yaml
```

## 2. The bridge: how NPCs move, contacts, traces

Keep a copy beside your scenarios and change what you need. For example, exact NPC poses
instead of physics-tracked ones, and contact reports in a directory of your own:

```bash
mkdir -p ~/my_scenarios
cp $B ~/my_scenarios/bridge.yaml
```

```yaml
# ~/my_scenarios/bridge.yaml (changed lines)
# tracked (default): physics bodies following SSv2
npc_motion: kinematic
# where each run's contacts go (an absolute path)
contact_report_dir: /tmp/swb_contacts
```

Start the stack with it:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    side_args:="config_file:=$HOME/my_scenarios/bridge.yaml"
```

The same works on `isaac.launch.xml`. Check what the running bridge loaded, from a terminal
with the environment file sourced:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} \
    ros2 param get /scenario_world_bridge config_file
```

Some keys are also simulation side arguments, for a one-off change without a file, all
inside one `side_args`: `npc_motion:=kinematic`, `weather:=ClearNoon`,
`gap_trace_dir:=<dir>`, `contact_report_dir:=<dir>`.

| Key | What it does |
|---|---|
| `npc_motion` | `tracked`: NPCs are physics bodies driven toward SSv2's pose (real velocities, walk cycles, physical contacts); `kinematic`: teleported each frame, exact but weightless |
| `blueprint_map` | SSv2 asset names to simulator models (`sample_vehicle: "vehicle.tesla.model3"`) |
| `weather` | a weather preset applied at connect and after every map load (section 4) |
| `contact_report_dir` | each ego run's contacts as JSON, for the physical verdict ([Tutorial 2](02-run-and-judge.md)); `off` for none |
| `gap_trace_dir` | a per-frame trace of both worlds' poses, for diagnosis |
| `world.*` | the world server's endpoint and timeouts (set by the stack) |

## 3. The world server

### CARLA

`swb_carla_world.yaml` is about CARLA itself. Two common changes: CARLA on another
machine, and a fixed Traffic Manager seed so that `simulator_autopilot` cars drive the same
way on every run:

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
spectator: follow          # CARLA's window looks down on the ego (off by default)
```

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    world_config:=$HOME/my_scenarios/carla_world.yaml
```

The host and port alone also come from the environment, `CARLA_HOST` and `CARLA_PORT`,
which win over the file. Other keys: `collision_monitor` (CARLA's own contact sensors, for
the physical verdict), `carla_tick_seconds` and `substeps` (the simulation step),
`map_alias` (map directory names to CARLA town names, `Town10: Town10HD`).

The world server's node runs in the ego domain. To see which file it loaded:

```bash
ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} \
    ros2 param get /swb_carla_world config_file
```

**The car's physics from its spec.** CARLA's blueprint has its own mass, engine and tyres.
To give the ego the vehicle spec's instead (the same numbers Isaac's ego uses):

```bash
G=$(ros2 pkg prefix --share sim_vehicle_spec)/generated
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    vehicle_physics:=$G/tesla_model3/carla_physics.yaml
```

The stack hands the same file to the ego's vehicle bridge, which reads it back and checks
it. Give a background AV the same file as `carla_physics:=`.

### Isaac Sim

The Isaac world server takes `isaac.launch.xml`'s arguments and a few environment variables:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch isaac.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    world:=$CARLA_MAPS/Town01/isaac/world.yaml,$CARLA_MAPS/Town02/isaac/world.yaml \
    render:=window \
    physics_hz:=120
```

| Argument / variable | What it does |
|---|---|
| `world:=` | worlds to preload, comma separated (world files, map directories or USD files); the first stays loaded, and switching to another is fast |
| `render:=` | `auto` (a window when `DISPLAY` is set), `window` or `headless` |
| `ego:=` | `default` (aib's Model 3 with Autoware's sensors), `none`, or your own aib ego config |
| `follow:=` | `true`: no UI, the viewport chases the ego; `editor`: the chase with the editor UI |
| `physics_hz:=` | the physics rate (default 60) |
| `ISAAC_PORT` | the world server's port (5590) |
| `ISAAC_KIT_ARGS` | Isaac's own options, e.g. `--/renderer/multiGpu/enabled=false --/renderer/activeGpu=1` to pin a GPU |

Road surfaces (wet asphalt, gravel, mud) are per map, in `<map dir>/surfaces.yaml`
([off-road surfaces](../design/off-road-surfaces.md)).

## 4. The weather

On CARLA the weather is set through ROS services on the bridge, in the scenario domain, by
preset or by CARLA's parameter names:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 service call \
    /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather \
    "{preset: WetSunset}"

# parameters by name; those not named keep their values
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 service call \
    /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather \
    "{names: [cloudiness, fog_density], values: [80.0, 30.0]}"

ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 service call \
    /scenario_world_bridge/get_weather \
    swb_interfaces/srv/GetWeather
```

The world server keeps it across map loads and CARLA restarts. To start every run with a
preset, add `side_args:="weather:=ClearNoon"` to the stack (or set `weather:` in the
bridge's file). The presets and parameters are listed in the
[User Guide](../user-guide.md#weather).

On Isaac a scenario's own `EnvironmentAction` (weather, time of day) sets the sky, fog and
rain; there is no weather service.

## 5. Ports and ROS domains

The stack reads these from the environment, so set them in the environment file; a launch
argument overrides one for a single run.

| What | Default | Environment | Stack argument |
|---|---|---|---|
| SSv2 to the bridge (and the next two ports) | 5555 | `SSV2_PORT` | `ssv2_port:=` |
| Agent relay | 5560 | `AGENT_PORT` | `agent_port:=` |
| CARLA world server | 5591 | `WORLD_PORT` | none |
| Isaac world server | 5590 | `ISAAC_PORT` | none |
| CARLA | localhost:2000 | `CARLA_HOST`, `CARLA_PORT` | none (or `world_config:=`) |
| Ego domain: world server, autopilot, vehicle agent | 1 | `EGO_ROS_DOMAIN_ID` | `ego_domain:=` |
| Scenario domain: bridge, relay, SSv2 | 9 | `SCENARIO_ROS_DOMAIN_ID` | `scenario_domain:=` |
| Background AVs | 2, 3, ... | `ROS_DOMAIN_ID` of each launch | |

Commands that take a port as an argument need the moved value too: `port:=` on
`scenario.launch.xml`, `relay:=` on `background_av.launch.xml`, `--relay` and `-- port:=`
on `run_suite`. Writing `${SSV2_PORT:-5555}` and `${AGENT_PORT:-5560}`, as the tutorials
do, picks them up.

Why the domains are split, the 0 to 57 limit, and two stacks on one machine:
[ROS domains and DDS](../domains-and-dds.md).

Next: [Tutorial 5, demo and recording](05-demo-and-recording.md).
