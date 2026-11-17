# Troubleshooting

Symptoms seen in testing, by where they show, with their cause and fix. Start with the
run's own account: `scensim runs why <run id>` names the first warning or error and
the process group it came from; `runs show <run id>` gives the whole timeline.

## The stack does not start

| Symptom | Cause, fix |
|---|---|
| `scensim: the stack is refused (3): ...` | the message names the problem and its fix: assets not fetched (`scensim assets fetch`), a scenario that does not parse, an entity without a pilot, a pilot kind the simulator cannot run |
| `warning  GPU <n> (... GB in use): ... GB free, isaac needs about 9.8` (or `... % busy`) when `scensim run` starts a stack | other processes hold the seat's GPU; the stack still starts and may run slowly or run out of memory. `nvidia-smi` names them; another card: the host setting `gpu:` |
| `warning  the workspace is off its pins: <submodule> at <commit> (pinned <commit>)` | a submodule, or a component's `deps/`, is checked out at another commit than its pin; a build from it runs other code (an older vehicle spec, for example, and the ego's spawn is refused): `git submodule update --init --recursive`, then rebuild |
| `python3 cannot import shapely, networkx, which CARLA's agents need` | `sudo apt install python3-shapely python3-networkx`, or `python3 -m pip install --user shapely networkx` |
| `play_launch <version> (...) is older than 0.30.0, which the stack needs` | the `play_launch` first on `PATH` is too old: `python3 -m pip install --user 'play_launch>=0.30.0'`, or put the newer one first on `PATH` |
| RViz dies at start, `could not connect to display` | `DISPLAY` is not set: export it, or `rviz: false` on the pilot; `project check` warns |
| `Isaac message bindings: <package> is not built` (or `is stale`) | Isaac's Python lacks an Autoware message package the world server publishes, or it was built from an older Autoware: `scensim isaac msgs` |
| `WARN dds-env-split: /simulator sets ROS_DOMAIN_ID=<n> where the rest of the launch has <m>` in the stack's console | expected: the world server runs on the ego pilot's domain, on purpose. A stack you started by hand with `play_launch` from a terminal on another domain or DDS profile warns for `/bridge` and `CYCLONEDDS_URI` too; `eval "$(scensim seat env scenario)"` first, or let `scensim run --start-stack` start it |
| `... seat <n> is taken: <user> (pid ...)` | another stack holds the seat: take another (`scensim seat take auto`) or wait |
| `... pilots ... but seat <n> has 7 pilot domains` | more pilots than the seat has domains: fewer pilots, or a seat table with a larger `stride` |
| The stack and every run start nothing | play_launch missing or older than 0.30.0: `play_launch --version` |
| `Address already in use` on the web UI | `--web-addr` is another stack's: use your seat's web port (`scensim seat show`) |

## A run is refused (exit 3)

| Symptom | Cause, fix |
|---|---|
| `no stack of project <name> on any seat` | the stack is not up, or was started from another directory: start it in the project |
| `the stack ... was started from another project state: ... differ; restart the stack` | `swb.yaml`, `scensim.lock` or the pilots changed since the stack started: restart it |
| `the stack ... is not ready (... waiting for ego); try --wait 600` | a pilot is still starting: wait for the stack's `ready` line, or add `--wait` |
| `stacks of this project on seats 0, 1: give --seat <n>` | two stacks of the project are up: name one |
| `map <name> is not one of the stack's maps` | add it to `maps:` in `swb.yaml` (or a scenario naming it under `scenarios/`), restart the stack |
| `the scenario is on map ..., the stack's Isaac world on ...` | an Isaac stack serves one world: restart it with the `map:=` the message names |
| `entity ... has no pilot in the stack` | the scenario has an `agent` entity the stack started no pilot for: add it to `pilots:`, restart |
| `refused: ... budget` | the log disk cannot hold the runs at `logging.budget`: prune runs, or capture less |

## A run ends `error` (exit 2)

| Symptom | Cause, fix |
|---|---|
| `runs why` names the simulator group; CARLA is not serving | start CARLA; a restarted CARLA is waited for up to 120 s at a scenario's start |
| The bridge logs `CARLA version mismatch` | CARLA and the build are different versions |
| A pilot never became ready within `--ready-timeout` | its log under the stack's `proc/pilot-<entity>/` says why; a user's first Autoware start builds TensorRT engines into `~/autoware_data` (minutes; kept for later starts): run again, or give `--ready-timeout 1200` |
| An Autoware pilot never becomes ready, every time; its log has `undefined symbol: _ZTVN18diagnostic_updater7UpdaterE` | the installed Autoware 1.5.0 is not the published binary packages (a local rebuild) and one of its libraries fails to load; install Autoware 1.5.0's published packages |
| The ego is up but never engages; the relay's log has no `registered` for it | the pilot's agent did not register: its log, and for `kind: none` your own agent's `--relay` |
| `Requested the service /api/autoware/set/velocity_limit 5 times` | the ego lacks Autoware's deprecated API, which serves a scenario's absolute `SpeedAction` (every AWF use case): keep `launch_deprecated_api` on ([Your Autoware](../how-to/your-autoware.md)) |
| A scenario fails in PLANNING or WAITING_FOR_ENGAGE | a node of the ego's Autoware exited: its log in the stack's web UI says why. `map_based_prediction` aborting with `Could not find 1` is a map without lanelet subtypes; `behavior_path_planner` aborting now and then (`failed to add guard condition to wait set`) is an Autoware bug the pilot's respawn recovers from |
| `the ego's parts were built from different vehicle specs` | the world server, the vehicle agent and the catalog name different specs: [Your car, the spec check](../how-to/your-car.md#the-spec-check) |

## Seeing nothing from your own terminal

| Symptom | Cause, fix |
|---|---|
| `ros2 topic list` is empty although the stack runs | the terminal is not on the stack's domain and DDS: `eval "$(scensim seat env 0)"`. The argument names a domain of your seat: `scenario` (the bridge, the relay, SSv2) or `0`, `1`, ... (the first, second, ... pilot, in the order they start, the ego first) |
| Still empty | the `ros2` daemon kept other DDS settings: `ros2 daemon stop`, or add `--no-daemon` |
| The vehicle side never localizes; the point cloud map never arrives | the kernel's socket buffers are too small for a point cloud: the `sysctl` step of [Install](../getting-started/install.md); `RcvbufErrors` in `netstat -su` says so |
| A domain above 57 fails with `failed to create domain` | the DDS profile's ports end at domain 57: a seat table within 0 to 57 ([Several stacks on one host](../how-to/several-stacks.md)) |

## CARLA

| Symptom | Cause, fix |
|---|---|
| CARLA takes minutes to answer, then freezes in synchronous mode | it came up on another GPU or a software renderer: start it with `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json` |
| CARLA crashes when a scenario changes towns | it runs at `-quality-level=Low`, where CARLA 0.9.16 segfaults on a town change: keep the default (Epic) |
| A second CARLA on a held port crashes at start | the first still holds it: stop it by its pid (SIGKILL if it ignores SIGTERM) |
| `SSv2 wrote no JUnit: The simulation has timed out. Forcibly inactivate.` | SSv2 ran out of time before the storyboard ended and stopped itself without a verdict: the ego never reached its goal, or a trigger never fired. The run's `timeline.jsonl` and the pilot's log under `logs/` say where it stood; a longer scenario needs a larger `timeout` in its suite |
| `this host's CARLA has no town <name>` | the project's map is a town that must be installed into the CARLA binary (a map package), and this host's CARLA does not have it: install the package into CARLA, or, if your packages live elsewhere, list their directories as `carla: {map_packages: [<dir>]}` in `~/.config/scensim/host.yaml` |
| `nothing serves CARLA on localhost:2000 and carla.root is not set` | the stack cannot start CARLA without knowing where it is. Set `carla: {root: <CARLA_0.9.16 directory>}` in `~/.config/scensim/host.yaml` (or the project's `.scensim/local.yaml`) and the stack starts it; or start CARLA yourself; or pass `simulator_start:=external` to wait for one |
| CARLA dies mid-run | the run fails within about 30 s. A CARLA the stack started (`carla.root`) is started again by the stack; one you started, restart yourself. The next run waits for it and runs |
| The stack stops: CARLA did not serve its RPC within `carla.ready_timeout` | the CARLA it started is slow on this host, or failed: read its log, `proc/simulator/carla.log` in the stack's directory; raise `carla.ready_timeout` |
| The ego hits a chair, table or cone of the town that a car knocked into its lane (a contact with a `static.prop.mesh`) | the town's own props were simulated: the world server's `map_props` is `live`, or its log has `Map props: could not freeze`. With `map_props: frozen` (the default since 2026-11-16) it logs `Map props: froze N of N static.prop.mesh actor(s)` at each world load and they stay put; a run's `result.json` names the setting (`provenance.world_server.map_props`). The contact still counts |
| The light never changes; `Cannot read the signal table` | the world server could not write the map's signal table into `state/<map>/`: the project directory must be writable |

## Isaac Sim

| Symptom | Cause, fix |
|---|---|
| `assets fetch` of the USD set stops with `HTTP Error 429`, then waits and retries | Hugging Face limits anonymous downloads; `hf auth login` with a free account's token raises the limit |
| The world server never says ready | the first load of a town compiles its materials (up to a quarter of an hour): `scensim_world_check hello <url> --wait 1800` waits for it |
| `not up after 1200 s`; its Kit log ends at `omni.client`, 100 % CPU | a Kit cache link points at a deleted directory: [Getting started on Isaac Sim](../getting-started/isaac.md#if-it-does-not-work) |
| It dies or hangs out of memory | it needs the GPU and about 17.5 GB of RAM to itself; on two GPUs pick one with the host setting `gpu:` |
| The ego never leaves `INITIALIZING`; the localizer's log ends at `Call align server` with no answer | the host is too busy for the first alignment in time (seen at a load of 70 to 83 on 24 cores, the align taking 30 s): run when the load is lower, or give `scensim run --ready-timeout` more |
| The ego never leaves `INITIALIZING`; `ndt_scan_matcher` logs `No InputSource`, the world server `sensors not ready after N warm-up frames` | on a two-GPU host the renderer is on another card than CUDA, so the sensors never publish: name the card with the host setting `gpu:` and take any `--/renderer/activeGpu` out of `ISAAC_KIT_ARGS` |
| It is gone for minutes after dying | the stack respawns it and it loads the world again |
| A first run ends `error` after SSv2 logs `change_to_stop` 30 times, while the world server logs the ego's spawn with `sensors warm after ... 60 s` or more | the ego's sensors took longer on their first spawn than SSv2 waits for its vehicle (about 90 s); a second spawn on the same world server is fast. A suite's `retry: {on: error, times: 1}` passes it |
| `... SCENSIM_CARLA_USD not set` | the world server was started outside a project's stack |

## Autoware

| Symptom | Cause, fix |
|---|---|
| CenterPoint takes minutes at every start; `Plan was created with a different version of TensorRT` | the loader finds another TensorRT than 10.8.0: [Your Autoware, TensorRT](../how-to/your-autoware.md#tensorrt-1080) |
| `pose_instability_detector` exits (`cannot store a negative time point in rclcpp::Time`) | stock Autoware's node does not survive the ego's teleport to a new start; it feeds only a diagnostic. The fix: [Your Autoware, fixes](../how-to/your-autoware.md#fixes-autoware-150-lacks) |
| The Autoware launch refuses to start, naming `setup_autoware_data` | Autoware's data directory is missing or read-only: `ros2 run autoware_sim_profile setup_autoware_data` |
| An argument of the profile has no effect | an old or misspelled name, which a launch ignores: `ros2 run autoware_sim_profile check_launch_args ...` |
| `voxel_based_compare_map_filter` logs `map voxel grid is not feasible, number of voxels overflows int32` | the map's point cloud is one file too large in extent for the filter's voxel grid, so map points are never taken out of the obstacle cloud and the ego may stop for terrain it sees. Tile it: a `pointcloud_map/` directory of tiles with its `pointcloud_map_metadata.yaml`, which Autoware then loads dynamically |
