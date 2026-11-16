# Configure the stack

What you can change in the parts of a stack, and where: the bridge, CARLA's world server,
Isaac's world server, and the weather. It assumes a project with a working stack
([Tutorial 1](../tutorials/01-first-drive.md)). The pilots' settings are in
[Your Autoware](your-autoware.md) and [Your autopilot](your-autopilot.md); the host's in
[Host settings](../reference/host-settings.md).

## Where settings live

| Part | Its settings | In `swb.yaml` |
|---|---|---|
| The bridge (SSv2's side) | `bridge_config.yaml` | `world:` (`npc_motion`, `weather`, `config_file`, `contact_report_dir`) |
| CARLA's world server | `swb_carla_world.yaml` | `world: {world_config_file: ...}` |
| Isaac's world server | its launch arguments | `isaac:` |
| The weather | the bridge's `weather`, ROS services | `world: {weather: ...}`, or at run time (below) |
| SSv2 | its run settings | `ssv2:`, and per suite ([Run settings](../reference/project-file.md#run-settings)) |

Every key: [The project file](../reference/project-file.md#world). The stack gives these to
the bridge and the world servers beside its own settings (the state directory, the
seat's ports, the per-run trace and contact directories), never in their place. A change in
`swb.yaml` needs the stack restarted; a run refuses a stack started from another
`swb.yaml`.

For the two files, a launch argument of the same name wins over the file, and the file
over the built-in default. Each refuses a key that belongs to the other, naming where it
went, so a misplaced setting fails loudly. Both log the file they loaded. Start a copy in
the project from the installed defaults; every key is commented in them:

```bash
mkdir -p config
cp $(ros2 pkg prefix --share scenario_world_bridge)/config/bridge_config.yaml config/
cp $(ros2 pkg prefix --share swb_carla_world)/config/swb_carla_world.yaml config/
```

`stack_args:="side_args:='...' sim_args:='...'"` on the stack still reaches the same
launches, for a try without editing `swb.yaml`: its names are merged with the stack's,
and a name the stack sets itself (`state_dir`, `gap_trace_dir`, `contact_report_dir`, the
ports, Isaac's `world`) is refused with what to give instead.

## The bridge

For example, exact NPC poses instead of physics-tracked ones, and the bridge's other
settings from the project's copy of its file:

```yaml
# swb.yaml (added)
world:
  npc_motion: kinematic          # tracked (default): physics bodies following SSv2
  config_file: config/bridge_config.yaml
```

Each run's contact reports land in its own directory (`contacts/`); give
`world: {contact_report_dir: off}` for none.

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
| `contact_report_dir` | the stack's (each run's in `contacts/`); set it in `world:` instead |
| `gap_trace_dir` | the stack's: `logging: {gap_trace: true}` puts each run's trace of both worlds' poses in its `gap_trace/`, for the `swb.gap` judges ([Judges](../reference/project-file.md#judges)) |

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

```yaml
# swb.yaml (added)
world:
  world_config_file: config/swb_carla_world.yaml
```

CARLA's host and port come from the host settings (`carla:`), which win over the file.
Other keys: `collision_monitor` (CARLA's own contact sensors, for the contact check),
`carla_tick_seconds` and `substeps` (the simulation step), `map_alias` (map names to CARLA
town names, `Town10: Town10HD`), `carla.reconnect_wait_seconds` (how long a scenario waits
for a restarted CARLA, 120 s).

## Isaac's world server

On Isaac `isaac:` in `swb.yaml` carries the world server's options; the stack sets the
world (the stack's map, then `preload`) and the state directory itself:

```yaml
# swb.yaml (added)
isaac:
  render: headless
  physics_hz: 240
  ego: config/ego_lockstep.yaml      # your own ego config
  preload: [Town05]                  # a run on Town05 moves the pilots, no restart
```

| Key | What it does |
|---|---|
| `render` | `auto` (a window when `DISPLAY` is set), `window` or `headless` |
| `follow` | `true`: no UI, the viewport chases the ego; `editor`: the chase with the editor UI |
| `physics_hz` | the physics rate (default 60) |
| `ego` | `default` (the Model 3 with Autoware's sensors), `none`, or your own ego config |
| `preload` | more maps loaded at start (each takes minutes and memory) |
| `wheels_topic` | `true` (default): publish `/swb/wheels` |

Isaac's own options go in `ISAAC_KIT_ARGS` ([Getting started on Isaac
Sim](../getting-started/isaac.md)); its GPU is the host setting `gpu:`, which the stack gives
both its CUDA and its renderer.

## The weather

The weather is the world server's. A project names it in `swb.yaml`, a preset or
parameters by the simulator's names; the stack sets it before it is ready:

```yaml
world:
  weather: {preset: WetSunset, fog_density: 30}
```

At run time it is set through ROS services on the bridge, in the scenario domain:

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
| One scenario (each variant) | 400 s, raised to a scenario's own time limit (its stop trigger on simulation time) plus its start and 60 s when that is longer | `ssv2: {global_timeout: ...}` in `swb.yaml`, a suite or a suite item; `ros2 run swb run --timeout` (never raised) |
| A run's pilots to become ready | 600 s | `ros2 run swb run --ready-timeout` |
| The stack to become ready | none (refuse at once) | `ros2 run swb run --wait` |
| A map switch of the pilots | 180 s | fixed |
| One simulator response (frame, spawn, despawn) | 90 s | environment `SIMULATOR_RESPONSE_TIMEOUT` of the stack |
| CARLA gone at a scenario's start | 120 s | `carla.reconnect_wait_seconds` in `swb_carla_world.yaml` |
| A judge | 120 s | `timeout:` of the judge |
