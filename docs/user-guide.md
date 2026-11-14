# User Guide

The reference: every launch a user runs, its main arguments and defaults, the environment,
the configuration files, and what to do when something goes wrong. To get a first run
going, start with a getting-started guide ([CARLA](getting-started/carla.md),
[Isaac Sim](getting-started/isaac.md)); to put your own scenario, car, autopilot and more
cars together, read [Build your own stack](build-your-own-stack.md).

A run has these parts:

- **The world server** drives the simulator over swb's world protocol: `swb_carla_world`
  for CARLA, `swb_isaac_world` for Isaac Sim.
- **The simulation side**: `scenario_world_bridge` (scenario_simulator_v2's adapter to the
  world server) and `scenario_agent_relay`, which serves SSv2 the ego's API and forwards it
  to each vehicle's agent over TCP.
- **The scenario**: scenario_simulator_v2 (SSv2), which runs your `.xosc` files and judges
  them.
- **A vehicle side** per autonomous vehicle: its autopilot (Autoware with this project's
  profile, sensor kit and vehicle interface) and its vehicle agent (swb_agent's
  `vehicle_agent`), which takes goals from the scenario through the relay. On CARLA it
  includes the CARLA vehicle bridge, `acb_bridge`; on Isaac the world server itself is the
  ego's vehicle interface.

The stack launch (`carla.launch.xml`, `isaac.launch.xml`) starts all of it but CARLA and
the scenario. The parts run in separate ROS domains: [ROS domains and DDS](domains-and-dds.md).
On CARLA a vehicle side may run on another host, since it reaches CARLA and the relay over
their own ports; on Isaac the ego's Autoware shares the world server's host and domain.

## play_launch

Every command here is `play_launch launch <package> <file> <arguments>`. `ros2 launch`
takes the same arguments, but play_launch (0.30.0 or newer) must be installed even then:
the stack starts the ego, its agent and the scenario as play_launch processes of their own.

```bash
sudo apt install libz3-dev ros-humble-rclcpp-components ros-humble-class-loader
pip install 'play_launch>=0.30.0'
play_launch --version
```

play_launch adds a web UI, per-node logs (`play_log/<time>/node/<name>/{out,err}`) and
restarts what a launch declares `respawn="true"`. Optional, once: `play_launch setcap`
(sudo) lets it record per-process I/O; without it each launch warns
`play_launch_io_helper lacks cap_sys_ptrace`, which is harmless.

### play_launch flags

| Flag | Where | Why |
|---|---|---|
| `--web-addr 127.0.0.1:<port>` | every launch | each launch serves a web UI, 8080 by default; two on one port collide. The docs use 8084 (the stack), 8081 (a scenario), 8083 and up (background AVs) |
| `--container-mode isolated --composable-respawn on-crash` | a background AV, an Autoware you launch yourself | reloads a composable node whose process crashed (below) |
| `--container-mode observable` | three or more Autoware stacks on one host | one process per container: fewer processes, no per-node reload |

The stack gives its ego the first pair itself. Why it matters: Autoware 1.5.0's
`behavior_path_planner` aborts now and then when a new route arrives (`failed to add guard
condition to wait set`, autoware_universe#12460). Autoware's containers declare no respawn,
so without a reload that vehicle side cannot plan again and every later scenario fails in
PLANNING. A composable node can be reloaded alone only when it is a process of its own
(`isolated`).

`isolated` costs a process per composable node. Three isolated Autoware stacks took about
47 of 48 cores on one host, enough to stall a pointcloud map load. In `observable` mode the
three cost about 9 cores and came up in about 15 s each, but a crashed composable takes its
container's other nodes with it until the stack is restarted.

## 1. Install and maps

Install and build as in a getting-started guide, stage 1
([CARLA](getting-started/carla.md), [Isaac Sim](getting-started/isaac.md)), or
[Build your own stack](build-your-own-stack.md), sections 1 and 2. Fresh-machine notes:

- The build targets CARLA 0.9.16 (the `carla-0916` feature in the workspace `Cargo.toml`);
  the bridges refuse a CARLA server of another version, naming both.
- Autoware's `data_path` must be writable (TensorRT writes each engine next to its ONNX
  file); it defaults to `~/autoware_data`. Fill it once with
  `ros2 run autoware_sim_profile setup_autoware_data`. The Autoware launch refuses to start,
  naming that command, while `data_path` is missing or read-only.
- If `traffic_simulator` fails to find a dependency's headers after that dependency built,
  delete `build/traffic_simulator`: its CMake cache remembers the failed configure.

### The map directory

The maps are fetched and repaired as in the getting-started guides, stage 2, or downloaded
already repaired ([Datasets](#datasets)). One directory
per town, in Autoware's map layout plus CARLA's files, side by side under `CARLA_MAPS`
(scenarios name their map as `$(env CARLA_MAPS)/<Town>`):

```
<map dir>/  lanelet2_map.osm  pointcloud_map.pcd  pointcloud_map_metadata.yaml
            map_config.yaml   map_projector_info.yaml
            carla/traffic_lights.yaml
            carla/map_origin.yaml       (only for a map not in CARLA's frame)
            isaac/world.yaml            (Isaac Sim)
```

`carla/traffic_lights.yaml` (which Lanelet2 signal is which CARLA light) is generated once
per map, with CARLA running and no other CARLA world server holding it:

```bash
ros2 run swb_carla_world swb_carla_world \
    --generate-signal-table $CARLA_MAPS/Town01 \
    --ros-args \
    -p carla_port:=2000 \
    -p port:=5599
```

It loads the town in CARLA, writes `<map dir>/traffic_lights.yaml` (and
`carla/traffic_lights.yaml`, a link to it) and puts CARLA back to asynchronous. The bridge
checks the table at every scenario start and refuses one that no longer matches CARLA. The
map directory is read-only at run time.

**A map not drawn in CARLA's frame** needs `carla/map_origin.yaml`: the map-frame position
of CARLA's world origin, in metres, no rotation. The CARLA towns share CARLA's origin and
need none; a surveyed map uses grid coordinates tens of kilometres from zero.

```yaml
# <map dir>/carla/map_origin.yaml
x: 52600.0
y: 67800.0
z: 0.0       # optional
```

Both bridges read it. No file means an origin of zero; a file that does not parse stops the
run rather than putting every pose hundreds of metres off.

### Where the maps come from

The CARLA towns converted for Autoware by TUM's
[carla-autoware-bridge](https://github.com/TUMFTM/Carla-Autoware-Bridge) (map pack
copyright 2024 TUM, LGPL-3.0; its `LICENSE.md` ships in each town directory). TUM's own
download link is dead; NEWSLabNTU keeps their zip unchanged (the same SHA-256) as the
`tum-2024-06` release of
[NEWSLabNTU/carla-autoware-maps](https://github.com/NEWSLabNTU/carla-autoware-maps/releases).

`scripts/download_maps.sh` downloads it, checks the checksum, installs it under
`data/carla-autoware-bridge/<Town>/` with the projector fix SSv2 needs, and repairs the
pack's defects, idempotently. Its environment: `SWB_XODR_DIR` (CARLA's
`CarlaUE4/Content/Carla/Maps/OpenDrive`, needed by the repairs that read OpenDRIVE),
`SWB_MAP_URL` (a mirror), `SWB_MAP_SOURCE` (a copy of the pack you already have). To repair
a copy installed another way, run the scripts on it in this order, the OpenDRIVE ones with
`--xodr-dir`:

| Script (`scripts/`) | Repairs | Without it |
|---|---|---|
| `repair_lanelet_traffic_lights.py` | traffic lights whose type tags are empty | Autoware and SSv2 see no signals |
| `repair_lanelet_subtypes.py` | lanelets with no `subtype` (the kerb strips) | `map_based_prediction` aborts near a pedestrian (`Could not find 1`) |
| `repair_lanelet_line_types.py` | `dashed_dashed` lines between same-direction lanes, made `dashed` | no lane change into an inner lane, no left turn from it |
| `repair_lanelet_bound_directions.py` | inner lanes that reuse the opposite lane's bound reversed | `behavior_path_planner` dies on a route through one |
| `repair_lanelet_local_coords.py` | local coordinates rewritten from lat/lon | stop lines and signs up to 2.4 m off |
| `repair_lanelet_duplicate_points.py` | bounds that repeat a point | NaN centre lines; `behavior_path_planner` crashes near a lane change |
| `repair_lanelet_turn_direction.py` | junction lanelets without `turn_direction` | the `intersection` and `blind_spot` modules never launch |
| `repair_lanelet_joins.py` | lanelets that start at fresh nodes where the predecessor ends | routing stops there (Town03) |
| `repair_lanelet_right_of_way.py` | `right_of_way` elements with no right-of-way lanelet | SSv2 refuses the map (Town05, Town10) |
| `repair_lanelet_speed_limits.py` (OpenDRIVE) | road lanelets with no `speed_limit` | every road driven at 50 km/h |
| `repair_lanelet_walkways_under_roads.py` (OpenDRIVE) | walkways under a raised road | Autoware stops on the bridge |
| `repair_lanelet_non_driving_lanes.py` (OpenDRIVE) | lanelets on lanes no car may drive | lane changes into Town03's tunnel divider |

One more repair needs CARLA holding the town and something ticking it (a running scenario):
`scripts/repair_lanelet_signal_links.py <Town>` links every signalised approach lanelet to
the light CARLA controls it by and gives it a stop line at CARLA's stop waypoints. Without
it Autoware crossed signalised junctions as unsignalised (on the pack's Town01, 28 of 36
approaches had no link to their own light). Run it for every town you drive, and restart
the stack after it: a stack started before keeps the old map.

For a town aib drives on its own (no SSv2 to command the lights),
`scripts/generate_signal_groups.py <Town> --port <CARLA port>` writes CARLA's light groups
to `<map dir>/carla/signal_groups.yaml`.

The repaired towns are also published, ready to use, as the `carla-town-autoware-maps`
dataset (below). Two of its files name paths on the machine that made them:
`<Town>/isaac/world.yaml` (`usd:`, absolute or relative to the map directory) and
`<Town>/carla/traffic_lights.yaml` (regenerate it with `--generate-signal-table`). A repair
added to `download_maps.sh` after the dataset was published reaches only maps fetched by
the script.

### Isaac Sim's assets

Isaac renders CARLA's towns, vehicles, props and walkers from a USD export of CARLA
0.9.16, the asset set, named by two environment variables: `CARLA_USD` (the set's root)
and `SWB_VEHICLE_USD` (its `vehicles/`). A map's `isaac/world.yaml` names its town's
`$CARLA_USD/<Town>/<Town>_physics.usda`; `src/swb_isaac_world/scripts/isaac_town.sh <Town>`
writes it. A scenario entity's `model3d` picks its model by CARLA blueprint id. Download,
check and use: [CARLA's assets on Isaac Sim](tutorials/06-carla-assets-on-isaac.md); how
the set is made: [Isaac assets](design/isaac-assets.md).

### Datasets

Published on Hugging Face, each downloaded with
`hf download jerry73204/<name> --repo-type dataset --local-dir <dir>` (`hf` is Hugging
Face's command, `pip install -U huggingface_hub`):

| Dataset | What | Licence |
|---|---|---|
| [carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd) | the asset set: six CARLA towns, 41 vehicles, 97 props, 52 walkers as USD (`CARLA_USD`) | CC BY 4.0 |
| [carla-town-autoware-maps](https://huggingface.co/datasets/jerry73204/carla-town-autoware-maps) | the repaired TUM towns (`CARLA_MAPS`), Town05 with a 110 km/h ring, a light Town01 Isaac world from the point cloud | LGPL-3.0 |
| [haruna-tsukuba-courses](https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses) | the Mount Haruna mountain road and Tsukuba Circuit: Isaac worlds, Autoware maps, a scenario each | ODbL, GSI |
| [isaac-road-physics-stages](https://huggingface.co/datasets/jerry73204/isaac-road-physics-stages) | Isaac test stages: 13 road surfaces, a proving ground | Apache-2.0 |
| [newslab-golfcart-models](https://huggingface.co/datasets/jerry73204/newslab-golfcart-models) | a golf cart for Isaac: body, rig, vehicle spec, catalog entry, RViz mesh | Apache-2.0 |

## 2. Scenarios

**Starter scenarios** are installed with the workspace (package `swb_examples`; each
scenario, its town and what it checks are listed in its
[README](../src/swb_examples/README.md)):

```
$(ros2 pkg prefix --share swb_examples)/scenarios/
    basic/      Town01 (ego drive, traffic light, pedestrian, rear contact,
                engage state, simulator autopilot), an ego drive in each of
                Town02, Town03, Town05 and Town10, the Town02 episode change
    multi_av/   town01_two_av.xosc: needs a background AV, entity bg_av_1
    tutorial/   town01_follow_to_goal.xosc, the worked example of Tutorial 1
    demo/       town01_tour.xosc, the demo tour, and its incidents one by one
    isaac/      town01_rear_contact.xosc with a standing ego, for Isaac Sim
    awf/        AWF ODD use cases UC-ACC-001-0001 and UC-AEB-001-0001
                (Apache-2.0), YAML with ScenarioModifiers: one run per
                parameter combination (27 and 9)
```

**The ego** is the entity whose controller is named `Autoware` or has the property
`isEgo=true`. Other vehicles are SSv2-driven by default; a controller named
`simulator_autopilot` hands one to CARLA's Traffic Manager, and `agent` to an autopilot of
its own, such as a background AV ([Writing scenarios](design/scenario-authoring.md),
[Tutorial 3](tutorials/03-other-road-users.md)). Scenario files are never modified.

**Which town the ego's Autoware loads.** Its `map_path` is fixed when it starts and must be
the town the scenario drives in. Every starter with an ego is Town01 except `basic/`'s
`town02_`, `town03_`, `town05_` and `town10_ego_drive`. With the wrong map, localization
never converges and the scenario fails waiting for the ego.

**A town change ends the vehicle side.** Loading a world starts a new episode of the
simulator, whose time starts again, and Autoware cannot follow time back. So on CARLA every
vehicle side ends at the load (`acb_bridge` exits with status 3, naming the change); a
restarted CARLA does the same. The stack restarts its ego, and the next scenario waits for
its agent; a background AV you started yourself needs restarting, or running under
`ros2 run swb_agent swb-agent supervise`. Each town change costs a stack start (about a
minute); keep a suite's scenarios on one town where you can.

## 3. Launches

### `carla.launch.xml` and `isaac.launch.xml`: the stack

The whole run but CARLA and the scenario, in one launch:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01
```

Arguments both take:

| Argument | Default | What |
|---|---|---|
| `map_path` | empty | the default autopilot's map directory |
| `ego_launch` | empty: Autoware (acb's on CARLA, aib's on Isaac) | the autopilot's launch: a path, `'<package> <file>'`, or `none` |
| `ego_launch_args` | empty | `name:=value ...` for `ego_launch` (the Autoware profile's: `rviz`, `max_velocity`, `autoware_launch`, ...) |
| `agent` | `true` | start swb's vehicle agent beside the autopilot; `false`: `ego_launch` registers its own |
| `vehicle_model` | `tesla_model3` | the ego's vehicle model (its description's `spec_hash.yaml`) |
| `spec_hash_file` | empty | the agent's spec hash file instead of `vehicle_model`'s |
| `vehicle_catalogs` | empty | extra vehicle catalogs for the bridge's spec check, comma separated |
| `side_args` | empty | `name:=value ...` for the simulation side: `config_file`, `weather`, `npc_motion`, `gap_trace_dir`, `contact_report_dir` |
| `scenario` | empty | an `.xosc` to run once the stack is up |
| `scenario_args` | empty | `name:=value ...` for `scenario.launch.xml` |
| `exit_on_scenario_end` | `false` | `true`: the whole stack ends with the scenario |
| `spawn` | empty | free drive, no scenario: the ego at `x,y,z,yaw` (map frame; needs `map_path`) |
| `vehicle` | empty: `vehicle_model` | free drive: the ego's catalog entry |
| `realtime` | `true` | free drive: pace to the wall clock |

Arguments `stack.launch.py` declares, which reach it through either shortcut:

| Argument | Default | What |
|---|---|---|
| `entity` | `ego` | the scenario entity the ego is |
| `frame_rate` | `20` | free drive: steps per simulated second |
| `ego_domain` | `EGO_ROS_DOMAIN_ID`, else 1 | the world server's and the ego's domain (0 to 57) |
| `scenario_domain` | `SCENARIO_ROS_DOMAIN_ID`, else 9 | the bridge's, relay's and SSv2's domain (0 to 57, not `ego_domain`) |
| `ssv2_port` | `SSV2_PORT`, else 5555 | SSv2's port on the bridge |
| `agent_port` | `AGENT_PORT`, else 5560 | the agent relay's TCP port |
| `log_dir` | `SWB_LOG_ROOT`, else `play_log` | where the nested launches (ego, agent, scenario) log |

`carla.launch.xml` only:

| Argument | Default | What |
|---|---|---|
| `vehicle_physics` | `none`: the blueprint's | the vehicle spec's `carla_physics.yaml` on the ego and agent vehicles |
| `world_config` | empty: the installed one | the CARLA world server's `swb_carla_world.yaml` |
| `npc_contact_sensors` | empty: the config's | `true`: contact sensors on NPC vehicles too |

`isaac.launch.xml` only:

| Argument | Default | What |
|---|---|---|
| `ego` | `default`: aib's Tesla | the world server's ego: `default`, `none`, or an aib ego config |
| `world` | `none` | worlds to preload, comma separated (world files, map directories or USDs); the first stays loaded |
| `render` | `auto` | `auto` (a window on `DISPLAY`), `window`, `headless` |
| `follow` | `false` | `true`: no UI, the viewport chases the ego; `editor`: the chase with the editor UI |
| `physics_hz` | `60` | the physics rate |

The default autopilot is Autoware with this project's profile; it opens RViz unless
`ego_launch_args:="rviz:=false"`. Pass that on a host without a display of your own.
Wait for it with `ros2 run swb_launch ego_ready` in the ego domain
([Build your own stack](build-your-own-stack.md#5-the-stack-with-autoware)).

### `stack.launch.py`: another simulator

The two shortcuts fill `sim` and `sim_args`. For a simulator of your own, a package with
`launch/world.launch.xml` that starts a world server:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch stack.launch.py \
    sim:=my_simulator \
    world_endpoint:=tcp://127.0.0.1:5600 \
    ego_launch:='my_autopilot autopilot.launch.xml'
```

| Argument | Default | What |
|---|---|---|
| `sim` | required | `carla`, `isaac`, or a package with `launch/world.launch.xml` |
| `sim_args` | empty | `name:=value ...` for the world server's launch |
| `world_endpoint` | the preset's | the world server's `tcp://host:port`; required for your own |

and every argument above. A simulator of your own has no default autopilot: give
`ego_launch`, or `none`.

### `scenario.launch.xml`: one scenario

Runs SSv2 on one scenario against a running stack, in the scenario domain, and ends with
it:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$HOME/my_scenarios/my_drive.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/my_drive
```

| Argument | Default | What |
|---|---|---|
| `scenario` | required | the `.xosc` or `.yaml` |
| `port` | `5555` | the bridge's SSv2 port (`SSV2_PORT`) |
| `output_directory` | `/tmp` | where the verdict and SSv2's files go |
| `global_timeout` | `400` | the scenario's timeout, s |
| `initialize_duration` | `120` | Autoware's start-through-engage budget, s |
| `global_real_time_factor` | `1.0` | SSv2's real-time factor |
| `frame_rate` | `20` | SSv2's frame rate |
| `vehicle_model` | `tesla_model3` | the ego's vehicle model |

The verdict is `<output_directory>/scenario_test_runner/result.junit.xml`;
`failures="0" errors="0"` means it passed. SSv2 empties that directory's subdirectories when
a run starts and the next run overwrites the verdict, so give every run you want to keep its
own `output_directory`.

### `background_av.launch.xml`: another Autoware vehicle (CARLA)

One Autoware for a scenario entity whose controller is `agent`, in a domain of its own:

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

| Argument | Default | What |
|---|---|---|
| `entity` | `bg_av_1` | the scenario entity; also its CARLA vehicle's name (`vehicle_name`) |
| `map_path` | required | the town's map directory |
| `relay` | `tcp://localhost:5560` | the stack's agent relay; `none`: local mode |
| `goal_poses_file` | empty | local mode's goal |
| `carla_physics` | `none` | the stack's `vehicle_physics` file, read back and verified |
| `carla_host`, `carla_port` | `localhost`, `2000` | CARLA |
| `release_notify_endpoint`, `release_ack_endpoint` | `tcp://127.0.0.1:5556`, `:5557` | the world server's sensor-release sockets, `SSV2_PORT` + 1 and + 2 |
| `vehicle_model`, `sensor_model` | `tesla_model3`, `tesla_model3_sensor_kit` | the car's Autoware packages |
| `launch_rviz`, `rviz_config` | `false`, Autoware's | RViz |
| `lidar_detection_model` | `clustering` | `centerpoint` uses TensorRT |

Its perception is the ego's (lidar clustering, traffic lights from the simulator) but for
a laser-scan occupancy grid (`occupancy_grid_map_method`). Other Autoware profile arguments
(`max_velocity`, `sim_*`, `gpu`, `extra_launch`, ...) pass through to the profile.

### `ego_av.launch.xml`: the ego's Autoware on its own (CARLA)

The stack's ego as a launch of its own, for a vehicle side you start and stop yourself
beside the simulation side (below), in the ego domain:

```bash
ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} play_launch launch \
    --container-mode isolated \
    --composable-respawn on-crash \
    --web-addr 127.0.0.1:8082 \
    swb_launch ego_av.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    relay:=tcp://localhost:${AGENT_PORT:-5560}
```

It takes the arguments of `background_av.launch.xml` with these differences: `entity`
defaults to `ego`, `vehicle_name` to `hero`, RViz is `launch_rviz` (default `false`), and
the steering and ground-truth options of acb are exposed (`steering_multiplier`,
`publish_ground_truth_objects`, ...). Arguments it does not declare (`max_velocity`,
`rviz_config`, `autoware_launch`, `sim_localization`, ...) still reach the Autoware profile,
but `--show-args` does not list them.

### The simulation side alone

`carla_simulation.launch.xml` and `isaac_simulation.launch.xml` are the stack's world
server and simulation side without an ego, all in one domain: for vehicle sides you start
yourself (`ego_av.launch.xml`, your own). On Isaac the world server is a launch of its own,
`swb_isaac_world swb_isaac_world.launch.xml`, started first (the Isaac guide shows it).

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8084 \
    swb_launch carla_simulation.launch.xml \
    weather:=ClearNoon
```

| Argument | Default | What |
|---|---|---|
| `config_file` | the installed `bridge_config.yaml` | the bridge's config file |
| `swb_carla_world_config` (CARLA) | the installed `swb_carla_world.yaml` | the CARLA world server's |
| `carla_host`, `carla_port` (CARLA) | the world server config's | CARLA |
| `world_port` (CARLA) | `5591` | the CARLA world server's port |
| `isaac_port` (Isaac) | `5590` | the Isaac world server's port |
| `ssv2_port` | `5555` | SSv2's port on the bridge |
| `agent_port` | `5560` | the agent relay's port |
| `entity` | `ego` | the entity the relay serves as the ego |
| `npc_motion` | the config's (`tracked`) | `tracked` or `kinematic` (section 4) |
| `weather` | the config's | a weather preset or `name:=value` fields |
| `gap_trace_dir`, `contact_report_dir` | none, the config's | traces and contact reports |
| `vehicle_catalogs` | empty | extra vehicle catalogs |
| `vehicle_physics`, `npc_contact_sensors` (CARLA) | `none`, the config's | as on `carla.launch.xml` |

`simulation.launch.xml` is the bridge and relay alone, for a world server you started:
`world_endpoint:=tcp://127.0.0.1:<port>` and the arguments above.

### Environment

The stack reads these; put them in the environment file
([Build your own stack](build-your-own-stack.md#1-the-environment-file)).

| Variable | Default | What |
|---|---|---|
| `EGO_ROS_DOMAIN_ID` | 1 | the world server's and the ego's domain |
| `SCENARIO_ROS_DOMAIN_ID` | 9 | the bridge's, relay's and SSv2's domain |
| `SSV2_PORT` | 5555 | SSv2's port on the bridge; the world server also uses the next two |
| `AGENT_PORT` | 5560 | the agent relay's port |
| `WORLD_PORT` | 5591 | the CARLA world server's port |
| `ISAAC_PORT` | 5590 | the Isaac world server's port |
| `CARLA_HOST`, `CARLA_PORT` | the world server config's (`localhost`, 2000) | CARLA |
| `SWB_LOG_ROOT` | `./play_log` | the nested launches' logs |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` (set by the launches) | must be Cyclone in every terminal |
| `CYCLONEDDS_URI` | swb_launch's localhost profile | the DDS profile; export it after Autoware's setup |
| `CARLA_MAPS` | none | where the scenarios find `$(env CARLA_MAPS)/<Town>` |
| `ISAAC_ROOT`, `CARLA_USD`, `SWB_VEHICLE_USD` | none | Isaac Sim and its assets (Isaac guide, stage 1; [Isaac Sim's assets](#isaac-sims-assets)) |
| `ISAAC_KIT_ARGS` | none | Isaac's own options, e.g. a GPU |
| `DISPLAY` | the terminal's | RViz, CARLA's or Isaac's window |
| `SIMULATOR_RESPONSE_TIMEOUT`, `SIMULATOR_INITIALIZE_TIMEOUT` | 90, 300 | SSv2's waits on the bridge, s ([Timeouts](#timeouts)) |

## 4. Running

**When each part is ready.**

- CARLA serves its RPC port: `ss -lnt | grep ':2000 '` lists it, one to three minutes after
  a cold headless start.
- The bridge logs `ZMQ server ready` and the relay `Relaying entity 'ego'`, in their
  per-node logs (the stack's web UI, or `play_log/.../node/<name>/`). The world server waits
  for CARLA by itself, and the bridge binds the SSv2 port only once its world server is
  ready.
- The ego is ready when `ego_ready` says ok, and the relay logs
  `entity 'ego': agent autoware registered` and then `ego: agent up`. About a minute on a
  warm host; the first start builds TensorRT engines and takes several minutes (every
  start does when your TensorRT is not 10.8.0: [Your own Autoware](#your-own-autoware)).

**Is a world server up?** One probe, which changes nothing; it prints the simulator, what
it offers, its owner and whether it is ready:

```bash
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5591   # CARLA
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5590   # Isaac
```

One bridge drives a world server at a time, and one CARLA world server drives a CARLA.

**RViz** shows the ego, its route and the objects it perceives while a scenario runs.
Between scenarios the ego is parked and its readouts keep their last values; that is not a
frozen RViz. Turn it off on a headless host, and on every Autoware after the first: each
costs CPU and GPU memory.

### Configuration files

Two processes, two files; each refuses the other's keys, naming where they went, and logs
the file it loaded:

- **The bridge**, `bridge_config.yaml`: the world server's endpoint and timeouts, the asset
  map (`blueprint_map`), the agent relay, `npc_motion`, `weather`, gap traces and contact
  reports. On the stack: `side_args:="config_file:=<file>"`.
- **The CARLA world server**, `swb_carla_world.yaml`: CARLA's host and port, blueprints and
  `map_alias`, ticking, the Traffic Manager (`traffic_manager`), the collision sensors, the
  sensor-release channel, the spectator. On the stack: `world_config:=<file>`. Its ROS
  parameter `vehicle_physics` (`vehicle_physics:=` on the stack) names the ego's physics, a
  vehicle spec's `carla_physics.yaml`; `none`, the default, keeps the blueprint's.

The installed defaults, commented, are under
`$(ros2 pkg prefix --share scenario_world_bridge)/config/` and
`$(ros2 pkg prefix --share swb_carla_world)/config/`.
[Tutorial 4](tutorials/04-configure.md) works through both.

### Many scenarios: `run_suite`

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 run swb_launch run_suite \
    $S/basic \
    --output ~/swb_results/basic \
    --relay tcp://localhost:${AGENT_PORT:-5560} \
    -- port:=${SSV2_PORT:-5555} global_timeout:=900
```

| Option | Default | What |
|---|---|---|
| scenarios | required | files (`.xosc`, `.yaml`) or directories (sorted, recursive) |
| `--output` | required | the results directory |
| `--domain` | `ROS_DOMAIN_ID`, else 9 | the scenario domain |
| `--relay` | `tcp://localhost:5560` | the relay to ask before each scenario |
| `--no-preflight` | off | skip that check |
| `--timeout-per-scenario S` | 0: none | cap one file's wall-clock time, all its variants |
| `--physical-verdict` | off | add CARLA's contacts as a testcase (below) |
| `--restart POLICY` | `on-reset-failure` when possible | restart the ego's agent between scenarios ([Writing an agent](agent-interface.md#when-a-reset-fails-restart)) |
| `-- <args>` | none | passed to every `scenario.launch.xml` |

Each scenario runs to its end, one after another, through `scenario.launch.xml` (with
play_launch, or `ros2 launch` without it), with its own
`output_directory:=<output>/<scenario name>/`. Before each, a pre-flight asks the relay
whether every entity the scenario needs (the ego, and each `agent` entity) has a registered
agent, and fails the scenario at once if not.

SSv2 already ends every variant at `global_timeout`, and one YAML file can expand to many
variants (`awf/` UC-ACC is 27, over 2000 s), so there is no per-file cap by default. In CI
set one above the suite's longest file.

Each scenario prints one line (`PASS`/`FAIL`, testcases passed, time, the first failure).
The results:

```
<output>/suite.junit.xml                every testcase of every scenario
                                        (classname = scenario name)
<output>/<scenario>/scenario_test_runner/   its result.junit.xml, SSv2's files
<output>/<scenario>/launch.log, play_log/   the launch's output
```

The exit status is 0 only if every testcase passed.

### CARLA's contacts in the verdict: `--physical-verdict`

SSv2 decides collisions in its own geometry: declared boxes, in 2D, no static world. When
the ego hits a curb, a pole, or an NPC whose CARLA body is bigger than its declared box, the
run can still pass. The bridge writes what CARLA's collision sensor saw for every ego run
to `~/.ros/swb/contacts/` (`contact_report_dir` moves it, `off` disables it). With
`--physical-verdict`, `run_suite` adds a `carla_contact` testcase to each scenario:

| `carla_contact` | When |
|---|---|
| pass | no contact, or only the contacts the scenario declares |
| failure | CARLA saw the ego touch something else; the message names it, with the SSv2 frame and the impulse |
| error | no report for the run (reports off), or CARLA restarted mid-run |

A scenario meant to make contact declares with whom (a comma-separated list of entity
names; `static` allows the static world). SSv2 ignores the parameter:

```xml
<ParameterDeclaration name="csbExpectedContacts" parameterType="string"
                      value="npc_rear" />
```

`--min-impulse N` ignores contacts below N N s. For one scenario, judge it after it ends:

```bash
ros2 run swb_launch physical_verdict \
    <output>/scenario_test_runner/result.junit.xml \
    --scenario <scenario file>
```

It judges the newest report; `--since` and `--until` choose a window.

The bridge gives each NPC and prop the CARLA body closest in size to its declared box,
within its `vehicleCategory`. Where CARLA has nothing close (a 12 m truck: its longest is
8.5 m), the testcase's `system-out` lists the body and its difference; that never fails it.
To force a body, set the entity's `model3d` to a CARLA blueprint name, or map it in
`blueprint_map`.

### How NPCs move: `npc_motion`

SSv2 computes every NPC's pose; the bridge decides how the simulator's body follows it
(`side_args:="npc_motion:=..."`, or `npc_motion:` in the bridge's file):

| `npc_motion` | What CARLA has | Choose it for |
|---|---|---|
| `tracked` (default) | physics bodies driven toward SSv2's pose each frame (within 0.2 m p95, snapped back past 0.5 m or 5 degrees): real velocities, walking animation, a hit NPC moves, an NPC that rams the ego pushes with finite force | realistic contacts and sensors |
| `kinematic` | physics off, teleported each frame: exact, but 0 m/s, gliding walkers, no pushing | exact poses, the cheapest tick |

Props and vehicles driven by `simulator_autopilot` or an `agent` are not affected. An NPC
that drives into the ego keeps pushing as long as SSv2 commands it on: make it brake once
the collision is seen, as `basic/town01_rear_contact.xosc` does.

## Your own Autoware

The default autopilot launches the **Autoware profile**, `autoware_sim_profile`'s
`autoware.launch.xml` (package `autoware_sim_vehicle`), around a stock `autoware.launch.xml`
(yours with `autoware_launch:=<path>`). On CARLA, acb's `carla_simulator.launch.xml` is the
profile plus `acb_bridge`; on Isaac, aib's `isaac_simulator.launch.xml` is the profile
alone. It selects the car's packages (`vehicle_model:=tesla_model3`,
`sensor_model:=tesla_model3_sensor_kit`) and simulation settings. Give the arguments through
`ego_launch_args:="..."` on the stack, or directly on the vehicle side's launch:

| Argument | Default | Effect |
|---|---|---|
| `autoware_launch:=<path>` | Autoware's | your own top-level `autoware.launch.xml` |
| `max_velocity:=<m/s>` | 4.17 (15 km/h) | Autoware's planning speed limit (`sim_planning`) |
| `lidar_detection_model:=` | `clustering` | `centerpoint` uses TensorRT |
| `rviz:=false` | `true` | no RViz |
| `gpu:=<index>` | empty: inherit | `CUDA_VISIBLE_DEVICES` for this Autoware |
| `extra_launch:=<file>` | none | a launch file of yours beside Autoware, in its domain |
| `sim_localization:=false` | on | your NDT/EKF/pose-initializer parameters |
| `sim_perception:=false` | on | your perception (traffic lights from a camera, not the simulator) |
| `sim_system:=false` | on | your component-state topics, MRM parameters and diagnostic graph |
| `sim_control:=false` | on | upstream's MPC parameters (the profile turns off steering-offset learning) |
| `sim_planning:=false` | on | your planning parameters; `max_velocity` no longer applies |
| `sim_clock_guard:=false` | on | no clock guard (it ends the stack when `/clock` goes back) |
| `launch_deprecated_api:=false` | on | no deprecated API (the velocity limit a scenario's `SpeedAction` sets; `awf/` fails without it) |

A launch ignores an argument no file declares, so an old name (`carla_localization`,
`carla_control`, ...) or the old car (`vehicle_model:=acb_vehicle`) is dropped without a
word. Check your arguments first; it exits 2 naming each problem and its replacement:

```bash
ros2 run autoware_sim_profile check_launch_args \
    max_velocity:=11.11 \
    sim_localization:=false
```

The lidar detector defaults to `clustering` because it passes on CARLA: the `awf/`
standing-pedestrian use case went 9/9 with it and 2/9 with `centerpoint` (2026-10-10, same
host and build).

### Fixes Autoware 1.5.0 lacks

Three fixes, in four package directories from two NEWSLabNTU forks, built from source over
Autoware 1.5.0 into a workspace of their own:

| Fork, branch | Package directory | Fixes |
|---|---|---|
| `autoware_universe`, `csb/pose-instability-empty-clip-0.48.0` | `planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module` | `blind_spot` never stops for a cyclist inside a right turn (autoware_universe#13509) |
| same | `localization/autoware_pose_instability_detector` | the node dies on a twist stamped at the previous odometry's time (`negative time point`) |
| `autoware_core`, `csb/ndt-step-reversal-1.5.0` | `localization/autoware_ndt_scan_matcher`, `localization/autoware_localization_util` | NDT's two-point cycle runs to the iteration limit, and its warning is an emergency stop |

The exact commits are in `scripts/autoware_overlay.bash` in the source tree. To build them
by hand, with Autoware sourced:

```bash
mkdir -p ~/autoware_overlay/src
cd ~/autoware_overlay/src
git clone --filter=blob:none --sparse \
    -b csb/pose-instability-empty-clip-0.48.0 \
    https://github.com/NEWSLabNTU/autoware_universe.git
git -C autoware_universe sparse-checkout set \
    planning/behavior_velocity_planner/autoware_behavior_velocity_blind_spot_module \
    localization/autoware_pose_instability_detector
git clone --filter=blob:none --sparse \
    -b csb/ndt-step-reversal-1.5.0 \
    https://github.com/NEWSLabNTU/autoware_core.git
git -C autoware_core sparse-checkout set \
    localization/autoware_ndt_scan_matcher \
    localization/autoware_localization_util
cd ~/autoware_overlay
colcon build \
    --allow-overriding $(colcon list -n) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

Then source `~/autoware_overlay/install/local_setup.bash` right after Autoware's setup, in
the environment file. Without it the ego runs stock Autoware, with the right-turn bug and
an emergency stop now and then.

### TensorRT 10.8.0

Autoware 1.5.0 was compiled against TensorRT **10.8.0** and rebuilds any cached engine that
another version made. If the first `libnvinfer.so.10` your loader finds is another version
(CUDA 12.8's 10.9 under `/usr/local/cuda`, say), CenterPoint rebuilds its engines at every
start: minutes, with `Plan was created with a different version of TensorRT` in its log.
Fetch NVIDIA's `tensorrt-cu12-libs==10.8.0.43` wheel from `https://pypi.nvidia.com`, unzip
its `tensorrt_libs/` directory anywhere, and put that directory first on `LD_LIBRARY_PATH`
in the terminal that starts the stack, after sourcing the setups.

## Another autopilot

The vehicle side does not have to be Autoware. Anything that registers with the relay as
the entity's agent drives it: a Python class (`swb_agent.Agent`) in your own project, or a
ROS 2 autopilot described in a YAML file (`ros2 run swb_agent ros_adapter --config
agent.yaml`). Start one with `ros2 run swb_agent swb-agent new <name>` and test it with
`ros2 run swb_agent swb-agent check` before running scenarios:
[Writing an agent](agent-interface.md).

## Another simulator

Every simulator is a world server behind the world protocol (`swb_world.proto` in
`swb_world_protocol`; [design](design/simulator-interface-split.md)); the bridge needs
nothing new for one. A port is one process that owns the simulator and implements the
protocol:

- **Python**: subclass `swb_world_server.world.World` (`load`, `spawn`, `despawn`, `step`;
  `set_signals` and the rest with a capability), set `simulator_name`, `capabilities` and
  `max_physics_dt`, and serve it with `swb_world_server.server.bind` and `run`. The Isaac
  world server (`swb_isaac_world/swb_isaac_world/world.py`) is the worked example.
- **Rust**: implement `swb_core::world_server::World` and serve it with `world_server::bind`
  and `serve`, as `swb_carla_world` does for CARLA.
- **Anything else**: answer the `.proto`'s `Request` with its `Reply` on a ZMQ REP socket.

Declare only what the simulator does (`Capability`): the bridge never sends a request the
server did not declare, and answers SSv2 with an honest failure naming the missing
capability. A map tells the server which world it means in
`<map dir>/<simulator name>/world.yaml` (`world`, `ground_paths`, `map_origin`).

Check the server before a scenario (a server declaring `SIGNALS` must answer `GetSignals`
with what its world shows: the check sets one lamp of the map's `traffic_lights.yaml` and
reads it back; it takes the server over, so run it with no bridge attached):

```bash
ros2 run swb_world_check swb_world_check conformance \
    tcp://127.0.0.1:<port> \
    --map <map dir>
```

Then run it in the stack with `stack.launch.py sim:=<your package>` (section 3), and the
starter scenarios against it. The vehicle under test needs a vehicle bridge of its own
(sensors, `/vehicle/status/*`, `/control/command/*`, `/clock`), as acb and aib are.

## Timeouts

A broken setup should fail in a minute or two, naming the cause, and a healthy run should
never sit in a fixed wait. These are the waits on the scenario path:

| Wait | Default | Where to tune | Why this value |
|---|---|---|---|
| Vehicle agent registered | checked before each scenario | `run_suite --relay`, `--no-preflight` | an unregistered agent can only time out |
| Init through engage | 120 s | `scenario.launch.xml initialize_duration:=` | start to engage measured 52 s at worst (median about 20 s, 33 starts) |
| First call to the ego's Autoware | follows `initialize_duration` | | the relay offers it exactly while an agent is registered |
| Any other ADAPI service to appear | 30 s | SSv2 fork | the relay's services appear in a second or two |
| Velocity limit retries | 5 (15 to 30 s) | SSv2 fork | a refusal means the vehicle side lacks the API |
| Route retries | 5 | SSv2 fork | a refused route stays refused |
| Clear route, engage, enable control, RTC retries | 10 | SSv2 fork | rides out a transient refusal, fails a persistent one in about a minute |
| One simulator response (frame, spawn, despawn) | 90 s | env `SIMULATOR_RESPONSE_TIMEOUT` | a dead bridge is reported in 1.5 min |
| Simulator `Initialize` | 300 s | env `SIMULATOR_INITIALIZE_TIMEOUT` | covers the CARLA wait below plus a town load |
| CARLA gone at `Initialize` | 120 s | `swb_carla_world.yaml` `carla.reconnect_wait_seconds` | a restarted CARLA serves its map in 25 to 51 s on a quiet host; raise it where CARLA restarts slowly |
| One scenario (variant) | 400 s | `scenario.launch.xml global_timeout:=` | longest variant measured 190 s |
| One scenario file, all variants | none | `run_suite --timeout-per-scenario` | SSv2 bounds each variant |

Also short, and fixed: the agent's own waits (10 s for fresh scans, 5 localization
attempts 2 s apart, 30 s to converge on the teleported pose, 2 s for a command to take
effect), the relay's 10 s command timeout, and the bridge's 120 s RPC timeout for a CARLA
town load (measured 44 to 71 s).

## Weather

The weather is the world server's, set through ROS services on the bridge, in the scenario
domain (`swb_interfaces`): a preset, or parameters by name, in the world server's own
names. On CARLA these are CARLA's presets and `WeatherParameters` fields
(`swb_carla_world`'s README lists them); Isaac's world server offers no weather service (a
scenario's own weather reaches it through SSv2).

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 service call \
    /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather \
    "{preset: ClearNoon}"
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 service call \
    /scenario_world_bridge/get_weather \
    swb_interfaces/srv/GetWeather

# parameters instead of a preset; those not named keep their current values
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} ros2 service call \
    /scenario_world_bridge/set_weather \
    swb_interfaces/srv/SetWeather \
    "{names: [cloudiness, sun_altitude_angle], values: [30.0, 60.0]}"
```

CARLA's presets: `ClearNoon`, `CloudyNoon`, `WetNoon`, `WetCloudyNoon`, `SoftRainNoon`,
`MidRainyNoon`, `HardRainNoon`, the same for `Sunset` (`MidRainSunset`) and `Night`
(`MidRainyNight`), and `DustStorm`; case is ignored. Both services answer with every
parameter and the preset they match. An unknown preset or parameter is refused with the
valid names, and an empty request is refused.

Every map load resets CARLA's weather to the town's default; CARLA's Town01 default is
overcast and raining. The world server keeps the weather you asked for (`weather:` in the
bridge's file, or `side_args:="weather:=ClearNoon"` on the stack, then the last
`set_weather`) and applies it again after every map load and CARLA restart. Empty (the
default) leaves CARLA's own.

## Troubleshooting

- **Nothing ever connects**: the relay never sees the vehicle's agent, `ros2 topic list` is
  empty, a scenario waits for a simulator that is running. DDS discovery is failing.
  swb_launch's launches run every process on Cyclone with the unicast-on-loopback profile
  it installs (`share/swb_launch/config/cyclonedds-localhost.xml`), unless the environment
  already sets `RMW_IMPLEMENTATION` or `CYCLONEDDS_URI`. acb's `carla_simulator.launch.xml`
  and aib's `isaac_simulator.launch.xml`, started directly, set nothing: they run on your
  terminal's settings. Source the environment file in every terminal, so all of them export
  the same two. The profile keeps all participants on one host and cannot use a domain
  above 57 ([ROS domains and DDS](domains-and-dds.md)).
- **The vehicle side never localizes; the pointcloud map never arrives**: it runs on another
  RMW than Cyclone, or on Cyclone without the profile. Under Fast DDS the pointcloud map,
  one large transient-local message, never reached its subscribers on a shared host, so NDT
  never initialised. An Autoware install that exports another RMW in its `setup.bash`
  overrides an earlier export, so set both after sourcing it. Large messages also need
  kernel socket buffers bigger than the stock 208 kB: install the repository's
  `config/99-cyclonedds-max.conf` (Autoware's DDS settings):

  ```bash
  sudo install -m 644 config/99-cyclonedds-max.conf /etc/sysctl.d/
  sudo sysctl -p /etc/sysctl.d/99-cyclonedds-max.conf
  ```

  Without it a point cloud fragments into more datagrams than a socket holds, which shows
  as `RcvbufErrors` in `netstat -su` and publishers that stall.
- **Several Autoware stacks on one host**: start them one at a time, each after the previous
  one is ready, in `--container-mode observable`, with RViz on one at most. Three starting
  together in `isolated` mode saturated a 48-core host: the pointcloud map load stalled and
  CARLA's switch to Town05 outran the bridge's 120 s timeout. A map load that times out can
  leave CARLA holding the town without its traffic-light actors: load it again and
  regenerate the town's signal table (section 1). On a host with several GPUs, `gpu:=<index>`
  pins an Autoware's CUDA nodes to one card.
- **CARLA dies**: the running scenario fails within about 30 s; restart CARLA; the next
  scenario waits for it (up to `carla.reconnect_wait_seconds`, 120 s) and runs.
- **CARLA just started**: it opens its RPC port before its first map is loaded; the world
  server waits until CARLA reports a map ("CARLA is still loading its first map" in its
  log) before sending it anything.
- **The bridge dies**: the launch restarts it; the running scenario fails within 90 s
  (`SIMULATOR_RESPONSE_TIMEOUT`), and the next one cleans up after it.
- **A scenario waits for the ego, or fails with "is not ready" or "agent relay"**: the ego
  is not up, or its agent has not registered with the relay (wrong `relay:=` or port). The
  relay logs every agent that registers; a single scenario proceeds as soon as one does and
  fails after `initialize_duration` (120 s) naming the relay; `run_suite`'s pre-flight
  fails the scenario at once.
- **A scenario fails in PLANNING or WAITING_FOR_ENGAGE** ("Simulator waited for the
  Autoware state to transition to WAITING_FOR_ENGAGE, but time is up"): the ego is up but
  cannot plan or engage. Most often a node in it has exited: its play_launch web UI shows
  it stopped, and `play_log/.../node/<name>/err` says why. `map_based_prediction` aborting
  with `Could not find 1` is a map without lanelet subtypes (section 1);
  `behavior_path_planner` aborting is an Autoware bug that `--composable-respawn on-crash`
  recovers from. Otherwise check that `map_path` is the scenario's town (section 2).
- **"Requested the service /api/autoware/set/velocity_limit 5 times"**: the ego lacks
  Autoware's deprecated API, which serves the velocity limit a scenario's absolute
  `SpeedAction` sets (every `awf/` scenario). The profile starts it
  (`launch_deprecated_api:=true`, the default); a launch passing `false` loses it.
- **`pose_instability_detector` exits** (`cannot store a negative time point in
  rclcpp::Time`): stock Autoware's node does not survive the ego being teleported to a new
  scenario's start. It only feeds a localization diagnostic; driving and verdicts are
  unaffected. The fix is in [Fixes Autoware 1.5.0 lacks](#fixes-autoware-150-lacks).
- **Without a scenario**: free drive puts the ego at a pose and steps the world, with no
  SSv2; drive it by setting a goal in RViz:

  ```bash
  play_launch launch --web-addr 127.0.0.1:8084 \
      swb_launch carla.launch.xml \
      map_path:=$CARLA_MAPS/Town01 \
      spawn:=299.4,-129.5,0.3,3.14
  ```

  With `scenario:=` set, `spawn:=` is ignored. Ctrl-C ends it.
