# Commands

Every command a user types, in the `swb` package unless named otherwise; `ros2 run swb
<command> --help` prints the same. Every command finds its project as `--project <dir>`
(or `project:=` on the stack), else the working directory or the nearest parent holding a
`swb.yaml`; `pkg://<package>[/<dir>]` names a project a ROS package installs.

## Exit codes

`run`, `project check` and `assets check`:

| Code | Means |
|---|---|
| 0 | success: every run as expected, no problem found |
| 1 | a scenario failed, or a check found a problem |
| 2 | an infrastructure error: a run the stack could not judge, Hugging Face unreachable |
| 3 | refused: no stack or the wrong one, a missing asset, bad input |

## `ros2 run swb project`

| Command | Does |
|---|---|
| `project new <dir>` | writes a starter project into a new (or empty) directory; `--template carla-first-drive` (default), `multi-pilot` or `isaac` |
| `project new <dir> --from <project>` | copies a project (a directory or `pkg://<package>/<dir>`) without `assets/`, `state/`, `runs/`, `.swb/` |
| `project check [<dir>]` | validates `swb.yaml`, the lock, every scenario and suite, the pilots against the scenarios' entities, the cars; prints `project: ok` or one `problem: <file>: <what> (fix: ...)` per finding |

## `ros2 run swb assets`

| Command | Does |
|---|---|
| `assets fetch` | resolves each dataset to its commit (`revision:`, else the newest tag, else `main`; a dataset already locked keeps its commit), downloads what `take:` names into the cache, verifies each file against the dataset's manifest, writes `swb.lock`, links `assets/` |
| `assets fetch --update` | resolves every dataset again and rewrites the lock |
| `assets list` | the known datasets and their sizes, and what this project has fetched |
| `assets check` | re-hashes every linked file against `swb.lock`; prints `assets: ok` |

## `ros2 run swb seat`

| Command | Does |
|---|---|
| `seat show` | every seat of the host's table, its numbers and its holder |
| `seat take <n>` / `seat take auto` | records the seat as yours (`seat:` in `~/.config/swb/host.yaml`) |
| `seat release` | sets your seat back to `auto` |
| `seat env scenario` / `seat env <k>` | prints `export ROS_DOMAIN_ID=... RMW_IMPLEMENTATION=... CYCLONEDDS_URI=...` for the seat's scenario domain or its k-th pilot domain (from 0); use as `eval "$(ros2 run swb seat env scenario)"` |

## The stack: `swb_launch swb.launch.xml`

```bash
play_launch launch --web-addr 127.0.0.1:<seat web port> \
    swb_launch swb.launch.xml [name:=value ...]
```

| Argument | Default | |
|---|---|---|
| `project` | the working directory's | a project directory or `pkg://<package>` |
| `seat` | the host settings' `seat:` | a seat number or `auto` |
| `simulator` | the project's `simulator:` | `carla` or `isaac` |
| `map` | CARLA: the first run's map; Isaac: the first map of the project's | the map the stack's pilots start on |
| `logs` | the host settings' `logs:` | the log root |
| `gpu` | the seat's | the GPU index (`CUDA_VISIBLE_DEVICES`) |
| `stack_args` | empty | `name:=value ...` for the lower-level `stack.launch.py`: `side_args` and `sim_args` are merged by name with what the stack and `swb.yaml` give (yours win; a name the stack sets itself is refused), the rest (`vehicle_catalogs`, ...) after swb's own |

`ego_launch`, `ego_launch_args`, `agent` and `map_path` are refused by name, given directly
or in `stack_args`: who drives each entity is the project's pilots in `swb.yaml` ([Project
file](project-file.md)), and the map is `map:=`.

play_launch binds its web UI and opens its own log directory (`./play_log/`) before the
launch file is read, so `--web-addr` (the seat's web port, `seat show`) and `--log-dir`
are its command line's. `ros2 launch swb_launch swb.launch.xml` takes the same arguments,
but play_launch must be installed: the stack runs its pilots and each run's SSv2 as
play_launch processes of their own. Ctrl-C (SIGINT) stops the stack and everything it
started.

## `ros2 run swb run`

```bash
ros2 run swb run <scenario.xosc | scenario.yaml | directory | suite.yaml> [options]
```

| Option | |
|---|---|
| `--param NAME=VALUE` | a scenario parameter (repeatable; with a suite, one more grid axis) |
| `--repeat N` | repetitions (a suite's own `repetitions:` otherwise) |
| `--shard i/n` | every n-th run from the i-th (1-based), for several hosts |
| `--resume` | a suite: skip the runs that have a result |
| `--capture LEVEL` | record at `result`, `standard`, `full` or `debug` and keep it whatever the verdict |
| `--timeout S` | SSv2's `global_timeout` (400 s) |
| `--wait S` | wait up to S for the stack to become ready (default: refuse at once) |
| `--start-stack` | start the project's stack when none is up (and leave it up) |
| `--stop-stack` | as `--start-stack`, then stop the stack it started after the runs |
| `--stack-timeout S` | how long a started stack may take to become ready (default `--wait`, else 1800) |
| `--ready-timeout S` | how long a run waits for its pilots' readiness (600 s) |
| `--seat N`, `--project DIR`, `--logs DIR` | instead of the host settings' and the working directory's |

It prints one line per run, `<verdict> <run id>  <run directory>[  judges: ...][  reason]`,
and a suite's `summary: <summary.csv>`. A run on another map than the stack's pilots
first prints `map <name>  (pilots restarted from <old> in <s> s)`. From Python,
`swb.api.run_one(project, scenario, overrides)` runs one and returns its result.

## `ros2 run swb runs`

| Command | Does |
|---|---|
| `runs list` | every run: id, verdict, scenario, duration |
| `runs show <id>` | a run's verdict, reason, scenario, duration, then its timeline |
| `runs why <id>` | a run's verdict, reason, and the first WARN or ERROR in any process's log, with its group |
| `runs summary [<dir>]` | writes `summary.csv` and a merged `junit.xml` for a directory of runs |
| `runs prune [--older-than D] [--verdict V] [--keep-last N] [--to LEVEL] [--dry-run]` | removes runs, or reduces what they keep to a level |

`<id>` is a run id, a unique part of one (`drive_to_goal`), or a run directory. `--root
<dir>` reads another runs directory than the project's.

## Other tools

| Command | Does |
|---|---|
| `ros2 run swb_launch carla_poses spawn-points \| ahead <i> --distance <m> \| at <x> <y>` | poses on the town CARLA has loaded, in the scenario's frame, as `<WorldPosition>` elements |
| `ros2 run swb_launch ego_ready --timeout S` | exits 0 once an Autoware in this domain can take a route |
| `ros2 run swb_launch physical_verdict <junit> --scenario <file>` | CARLA's contacts for a run as a JUnit testcase (a judge) |
| `ros2 run swb_launch chase_view` | a chase camera window on CARLA's ego |
| `ros2 run swb_world_check swb_world_check hello <url> [--wait S]` | asks a world server what it is and whether it is ready |
| `ros2 run swb_world_check swb_world_check conformance <url> --map <dir>` | checks a world server against the world protocol (no bridge attached) |
| `ros2 run swb_agent swb-agent new <name>` / `check <module:Class>` / `supervise` | the agent SDK ([The agent SDK](agent-sdk.md)) |
| `ros2 run swb_isaac_world make_world <map dir>` | a map's Isaac world file from the CARLA USD set |
| `ros2 run swb_isaac_world surfaces check <file>`, `road_detail check <file>` | checks a surface map or a road detail |
| `ros2 run autoware_sim_profile check_launch_args <name:=value ...>` | checks arguments for the Autoware profile |
| `ros2 run autoware_sim_profile setup_autoware_data` | once per machine: Autoware's writable data directory |
