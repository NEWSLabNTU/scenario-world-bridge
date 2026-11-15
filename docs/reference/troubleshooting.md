# Troubleshooting

Symptoms seen in testing, by where they show, with their cause and fix. Start with the
run's own account: `ros2 run swb runs why <run id>` names the first warning or error and
the process group it came from; `runs show <run id>` gives the whole timeline.

## The stack does not start

| Symptom | Cause, fix |
|---|---|
| `swb: the stack is refused (3): ...` | the message names the problem and its fix: assets not fetched (`ros2 run swb assets fetch`), a scenario that does not parse, an entity without a pilot, a pilot kind the simulator cannot run |
| `warning  GPU <n> (... GB in use): ... GB free, isaac needs about 9.8` (or `... % busy`) when `swb run` starts a stack | other processes hold the seat's GPU; the stack still starts and may run slowly or run out of memory. `nvidia-smi` names them; another card: the host setting `gpu:` |
| `play_launch <version> (...) is older than 0.30.0, which the stack needs` | the `play_launch` first on `PATH` is too old: `python3 -m pip install --user 'play_launch>=0.30.0'`, or put the newer one first on `PATH` |
| `Isaac message bindings: <package> is not built` (or `is stale`) | Isaac's Python lacks an Autoware message package the world server publishes, or it was built from an older Autoware: `ros2 run swb isaac msgs` |
| `WARN dds-env-split: /simulator sets ROS_DOMAIN_ID=<n> where the rest of the launch has <m>` in the stack's console | expected: the world server runs on the ego pilot's domain, on purpose. A stack you started by hand with `play_launch` from a terminal on another domain or DDS profile warns for `/bridge` and `CYCLONEDDS_URI` too; `eval "$(ros2 run swb seat env scenario)"` first, or let `swb run --start-stack` start it |
| `... seat <n> is taken: <user> (pid ...)` | another stack holds the seat: take another (`ros2 run swb seat take auto`) or wait |
| `... pilots ... but seat <n> has 7 pilot domains` | more pilots than the seat has domains: fewer pilots, or a seat table with a larger `stride` |
| The stack and every run start nothing | play_launch missing or older than 0.30.0: `play_launch --version` |
| `Address already in use` on the web UI | `--web-addr` is another stack's: use your seat's web port (`ros2 run swb seat show`) |

## A run is refused (exit 3)

| Symptom | Cause, fix |
|---|---|
| `no stack of project <name> on any seat` | the stack is not up, or was started from another directory: start it in the project |
| `the stack ... was started from another project state: ... differ; restart the stack` | `swb.yaml`, `swb.lock` or the pilots changed since the stack started: restart it |
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
| The ego is up but never engages; the relay's log has no `registered` for it | the pilot's agent did not register: its log, and for `kind: none` your own agent's `--relay` |
| `Requested the service /api/autoware/set/velocity_limit 5 times` | the ego lacks Autoware's deprecated API, which serves a scenario's absolute `SpeedAction` (every AWF use case): keep `launch_deprecated_api` on ([Your Autoware](../how-to/your-autoware.md)) |
| A scenario fails in PLANNING or WAITING_FOR_ENGAGE | a node of the ego's Autoware exited: its log in the stack's web UI says why. `map_based_prediction` aborting with `Could not find 1` is a map without lanelet subtypes; `behavior_path_planner` aborting now and then (`failed to add guard condition to wait set`) is an Autoware bug the pilot's respawn recovers from |
| `the ego's parts were built from different vehicle specs` | the world server, the vehicle agent and the catalog name different specs: [Your car, the spec check](../how-to/your-car.md#the-spec-check) |

## Seeing nothing from your own terminal

| Symptom | Cause, fix |
|---|---|
| `ros2 topic list` is empty although the stack runs | the terminal is not on the stack's domain and DDS: `eval "$(ros2 run swb seat env 0)"`. The argument names a domain of your seat: `scenario` (the bridge, the relay, SSv2) or `0`, `1`, ... (the first, second, ... pilot, in the order they start, the ego first) |
| Still empty | the `ros2` daemon kept other DDS settings: `ros2 daemon stop`, or add `--no-daemon` |
| The vehicle side never localizes; the point cloud map never arrives | the kernel's socket buffers are too small for a point cloud: the `sysctl` step of [Install](../getting-started/install.md); `RcvbufErrors` in `netstat -su` says so |
| A domain above 57 fails with `failed to create domain` | the DDS profile's ports end at domain 57: a seat table within 0 to 57 ([Several stacks on one host](../how-to/several-stacks.md)) |

## CARLA

| Symptom | Cause, fix |
|---|---|
| CARLA takes minutes to answer, then freezes in synchronous mode | it came up on another GPU or a software renderer: start it with `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json` |
| CARLA crashes when a scenario changes towns | it runs at `-quality-level=Low`, where CARLA 0.9.16 segfaults on a town change: keep the default (Epic) |
| A second CARLA on a held port crashes at start | the first still holds it: stop it by its pid (SIGKILL if it ignores SIGTERM) |
| CARLA dies mid-run | the run fails within about 30 s. A CARLA the stack started (`carla.root`) is started again by the stack; one you started, restart yourself. The next run waits for it and runs |
| The stack stops: CARLA did not serve its RPC within `carla.ready_timeout` | the CARLA it started is slow on this host, or failed: read its log, `proc/simulator/carla.log` in the stack's directory; raise `carla.ready_timeout` |
| The light never changes; `Cannot read the signal table` | the world server could not write the map's signal table into `state/<map>/`: the project directory must be writable |

## Isaac Sim

| Symptom | Cause, fix |
|---|---|
| The world server never says ready | the first load of a town compiles its materials (up to a quarter of an hour): `swb_world_check hello <url> --wait 1800` waits for it |
| `not up after 1200 s`; its Kit log ends at `omni.client`, 100 % CPU | a Kit cache link points at a deleted directory: [Getting started on Isaac Sim](../getting-started/isaac.md#if-it-does-not-work) |
| It dies or hangs out of memory | it needs the GPU and about 17.5 GB of RAM to itself; on two GPUs pin one (`ISAAC_KIT_ARGS`) |
| It is gone for minutes after dying | the stack respawns it and it loads the world again |
| `... SWB_CARLA_USD not set` | the world server was started outside a project's stack |

## Autoware

| Symptom | Cause, fix |
|---|---|
| CenterPoint takes minutes at every start; `Plan was created with a different version of TensorRT` | the loader finds another TensorRT than 10.8.0: [Your Autoware, TensorRT](../how-to/your-autoware.md#tensorrt-1080) |
| `pose_instability_detector` exits (`cannot store a negative time point in rclcpp::Time`) | stock Autoware's node does not survive the ego's teleport to a new start; it feeds only a diagnostic. The fix: [Your Autoware, fixes](../how-to/your-autoware.md#fixes-autoware-150-lacks) |
| The Autoware launch refuses to start, naming `setup_autoware_data` | Autoware's data directory is missing or read-only: `ros2 run autoware_sim_profile setup_autoware_data` |
| An argument of the profile has no effect | an old or misspelled name, which a launch ignores: `ros2 run autoware_sim_profile check_launch_args ...` |
