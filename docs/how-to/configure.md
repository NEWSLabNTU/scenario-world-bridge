# Configure the stack

What you can change in the parts of a stack, and where: the bridge, CARLA's world server,
Isaac's world server, and the weather. It assumes a project with a working stack
([Tutorial 1](../tutorials/01-first-drive.md)). The pilots' settings are in
[Your Autoware](your-autoware.md) and [Your autopilot](your-autopilot.md); the host's in
[Host settings](../reference/host-settings.md).

## Where settings live

| Part | Its settings | Given to the stack as |
|---|---|---|
| The bridge (SSv2's side) | `bridge_config.yaml` | `stack_args:="side_args:='...'"` |
| CARLA's world server | `swb_carla_world.yaml` | `stack_args:="sim_args:='...'"` |
| Isaac's world server | its launch arguments | `stack_args:="sim_args:='...'"` |
| The weather | ROS services on the bridge | at run time (below) |

For the two files, a launch argument of the same name wins over the file, and the file
over the built-in default. Each refuses a key that belongs to the other, naming where it
went, so a misplaced setting fails loudly. Both log the file they loaded. Start a copy in
the project from the installed defaults; every key is commented in them:

```bash
mkdir -p config
cp $(ros2 pkg prefix --share scenario_world_bridge)/config/bridge_config.yaml config/
cp $(ros2 pkg prefix --share swb_carla_world)/config/swb_carla_world.yaml config/
```

`stack_args` replaces what the stack gives that launch itself, so repeat the project's
state directory in it, as below. (Project keys for these files are planned; the
`stack_args` lines here have not been run with play_launch's own argument parser on a live
stack, 2026-11-15.)

## The bridge

For example, exact NPC poses instead of physics-tracked ones, and each run's contacts in a
directory of the project's:

```yaml
# config/bridge_config.yaml (changed lines)
npc_motion: kinematic          # tracked (default): physics bodies following SSv2
contact_report_dir: /tmp/swb_contacts
```

```bash
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml \
    stack_args:="side_args:='state_dir:=$PWD/state \
config_file:=$PWD/config/bridge_config.yaml'"
```

Check what the running bridge loaded:

```bash
eval "$(ros2 run swb seat env scenario)"
ros2 param get /scenario_world_bridge config_file
```

| Key | What it does |
|---|---|
| `npc_motion` | `tracked`: NPCs are physics bodies driven toward SSv2's pose (within 0.2 m p95, snapped back past 0.5 m or 5 degrees): real velocities, walk cycles, physical contacts; `kinematic`: teleported each frame, exact but weightless, the cheapest |
| `blueprint_map` | SSv2 asset names to simulator models (`sample_vehicle: "vehicle.tesla.model3"`) |
| `weather` | a weather preset applied at connect and after every map load |
| `contact_report_dir` | each ego run's contacts as JSON, for the `physical_verdict` judge; `off` for none |
| `gap_trace_dir` | a per-frame trace of both worlds' poses, for diagnosis |

An NPC that drives into the ego keeps pushing as long as SSv2 commands it on: make it
brake once the collision is seen.

## CARLA's world server

`swb_carla_world.yaml` is about CARLA itself. A common change: a fixed Traffic Manager
seed, so that `simulator_autopilot` cars drive the same way on every run:

```yaml
# config/swb_carla_world.yaml (changed lines)
traffic_manager:
  seed: 7                  # reseeded at every scenario start
spectator: follow          # CARLA's window looks down on the ego (off by default)
```

```bash
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml \
    stack_args:="sim_args:='state_dir:=$PWD/state \
config_file:=$PWD/config/swb_carla_world.yaml'"
```

CARLA's host and port come from the host settings (`carla:`), which win over the file.
Other keys: `collision_monitor` (CARLA's own contact sensors, for the contact check),
`carla_tick_seconds` and `substeps` (the simulation step), `map_alias` (map names to CARLA
town names, `Town10: Town10HD`), `carla.reconnect_wait_seconds` (how long a scenario waits
for a restarted CARLA, 120 s).

## Isaac's world server

On Isaac `sim_args` carries the world server's launch arguments; the stack sets `world`,
`ego` and `state_dir` itself, so repeat them:

| Argument | What it does |
|---|---|
| `render` | `auto` (a window when `DISPLAY` is set), `window` or `headless` |
| `follow` | `true`: no UI, the viewport chases the ego; `editor`: the chase with the editor UI |
| `physics_hz` | the physics rate (default 60) |
| `ego` | `default` (aib's Model 3 with Autoware's sensors), `none`, or your own aib ego config |
| `wheels_topic` | `true` (default): publish `/swb/wheels` |

Isaac's own options go in `ISAAC_KIT_ARGS` ([Getting started on Isaac
Sim](../getting-started/isaac.md)).

## The weather

The weather is the world server's, set through ROS services on the bridge, in the
scenario domain, by preset or by the simulator's parameter names:

```bash
eval "$(ros2 run swb seat env scenario)"
ros2 service call /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather "{preset: WetSunset}"
ros2 service call /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather \
    "{names: [cloudiness, fog_density], values: [80.0, 30.0]}"
ros2 service call /scenario_world_bridge/get_weather swb_interfaces/srv/GetWeather
```

Parameters not named keep their values. CARLA's presets: `ClearNoon`, `CloudyNoon`,
`WetNoon`, `WetCloudyNoon`, `SoftRainNoon`, `MidRainyNoon`, `HardRainNoon`, the same for
`Sunset` (`MidRainSunset`) and `Night` (`MidRainyNight`), and `DustStorm`; case is ignored.
Both services answer with every parameter and the preset they match; an unknown name is
refused with the valid ones. Every map load resets CARLA's weather to the town's default
(Town01's is overcast and raining); the world server keeps the weather you asked for
(`weather:` in the bridge's file, then the last `set_weather`) and applies it again after
every map load and CARLA restart.

On Isaac a scenario's own `EnvironmentAction` (weather, time of day) sets the sky, fog and
rain; there is no weather service.

## Timeouts

| Wait | Default | Where |
|---|---|---|
| One scenario (each variant) | 400 s | `ros2 run swb run --timeout` |
| A run's pilots to become ready | 600 s | `ros2 run swb run --ready-timeout` |
| The stack to become ready | none (refuse at once) | `ros2 run swb run --wait` |
| A map switch of the pilots | 180 s | fixed |
| One simulator response (frame, spawn, despawn) | 90 s | environment `SIMULATOR_RESPONSE_TIMEOUT` of the stack |
| CARLA gone at a scenario's start | 120 s | `carla.reconnect_wait_seconds` in `swb_carla_world.yaml` |
| A judge | 120 s | `timeout:` of the judge |
