# Commands

Every command a user types, in the `scensim` package unless named otherwise;
`scensim <command> --help` prints the same. A command names its testbench by its package
name (`scensim run my_testbench`), found in the sourced workspaces
([The testbench](testbench.md)). The older form, a project directory, remains: `--project
<dir>` (or `project:=` on the stack), else the working directory or the nearest parent
holding a `testbench.yaml`; `pkg://<package>[/<dir>]` names a project a ROS package
installs. Such a
project is read-only: `project check` reads it, and the commands that write (`assets
fetch`, `run`, `drive`, the stack) refuse it, naming the `project new --from` line that
copies it; the copy holds its lock, assets, state and runs.

## Exit codes

`run`, `testbench check`, `project check` and `assets check`:

| Code | Means |
|---|---|
| 0 | success: every run as expected, no problem found |
| 1 | a scenario failed, or a check found a problem |
| 2 | an infrastructure error: a run the stack could not judge, Hugging Face unreachable |
| 3 | refused: no stack or the wrong one, a missing asset, bad input |

`project check` and `assets check` give 3 only for what leaves the project unusable on this
host as it stands: an asset or map not fetched, a project file or asset entry that cannot be
read, a car that is not installed or whose body is off its catalog box. Anything else they
find in what you wrote is 1: a scenario whose car is not its pilot's, an entity without a
pilot, a suite key it does not know. The stack refuses to start (3) on either.

## `scensim testbench`

| Command | Does |
|---|---|
| `testbench new <name>` | writes a testbench package into `./<name>` (run it in a workspace's `src/`); `--template carla-first-drive` (default), `multi-pilot` or `isaac`; a name that is not a valid ROS package name is refused |
| `testbench new <name> --from <project>` | copies a project or testbench (a directory or `pkg://<package>/<dir>`) and adds the package manifest and a default suite |
| `testbench list` | the testbenches of the sourced workspaces, with their suites |
| `testbench check <name>` | the build's checks (the package's own findings) and this host's, printed as `problem (this host): ...`; ends `testbench: ok` |

## `scensim stack`

| Command | Does |
|---|---|
| `stack <name>` | starts the testbench's stack, detached, and returns once it is up: `stack    <name> on seat <n>: started; stop it with: scensim stack <name> --stop` (`already up` when one was); its console log is named in the line before |
| `stack <name> --foreground` | runs the stack in this terminal: `stack    <name> on seat <n> in the foreground (web UI on port <p>); Ctrl-C stops it`; Ctrl-C stops it and exits 130. Refused when the stack is already up, naming `--stop` |
| `stack <name> --stop` | stops it, and everything it started |
| `stack --list` | the stacks running on this host |

`scensim run <name>` starts the stack itself when none is up and leaves it up. The ROS
form of the same stack is the launch below with `testbench:=<name>`.

<!-- TODO(exit run): confirm the stack commands' lines against the release. -->

## `scensim project`

The commands for a project directory, the form before testbenches.

| Command | Does |
|---|---|
| `project new <dir>` | writes a starter project into a new (or empty) directory; `--template carla-first-drive` (default), `multi-pilot` or `isaac` |
| `project new <dir> --from <project>` | copies a project (a directory or `pkg://<package>/<dir>`) without `assets/`, `state/`, `runs/`, `.scensim/` |
| `project check [<dir>]` | validates `testbench.yaml`, the lock, every scenario and suite, the pilots against the scenarios' entities, the cars; prints the launcher it resolved (`launcher: auto -> play_launch 0.30.0`; under `ros2 launch` with an Autoware pilot, a note that a crashed component ends its container), then `project: ok` or one `problem: <file>: <what> (fix: ...)` per finding |

## `scensim assets`

| Command | Does |
|---|---|
| `assets fetch` | resolves each dataset to its commit (`revision:`, else the newest tag, else `main`; a dataset already locked keeps its commit), downloads what `take:` names into the cache, verifies each file against the dataset's manifest, writes `scensim.lock`, links `assets/` |
| `assets fetch --update` | resolves every dataset again and rewrites the lock |
| `assets fetch --verify` | also hashes every file already in the cache again |
| `assets list` | the known datasets and their sizes, and what this project has fetched |
| `assets check` | checks every linked file against `scensim.lock`, hashing a file only when its size or times changed since it was last hashed; prints `assets: ok` |
| `assets check --hash` | hashes every file, changed or not |

## `scensim seat`

| Command | Does |
|---|---|
| `seat show` | every seat of the host's table, its numbers and its holder, then the launcher line |
| `seat take <n>` / `seat take auto` | records the seat as yours (`seat:` in `~/.config/scensim/host.yaml`) |
| `seat release` | sets your seat back to `auto` |
| `seat env scenario` / `seat env <k>` | prints `export ROS_DOMAIN_ID=... RMW_IMPLEMENTATION=... CYCLONEDDS_URI=...` for the seat's scenario domain or its k-th pilot domain (from 0); use as `eval "$(scensim seat env scenario)"` |

## `scensim isaac`

| Command | Does |
|---|---|
| `isaac msgs` | builds, into `<isaac.root>/msgs`, every Autoware message package the Isaac world server needs that is missing there or was built from another Autoware (the install is the host setting `autoware.setup`'s) |
| `isaac msgs <package> ...` | rebuilds those packages |
| `isaac msgs --check` | lists what is missing or stale, as `project check` does for an Isaac project; exits 0 when nothing is |

## The stack: `scensim_launch scensim.launch.xml`

```bash
ros2 launch scensim_launch scensim.launch.xml [name:=value ...]
# or, with play_launch (recommended, see Launchers below):
play_launch launch --web-addr 127.0.0.1:<seat web port> \
    scensim_launch scensim.launch.xml [name:=value ...]
```

| Argument | Default | |
|---|---|---|
| `testbench` | none | a testbench by name, from the sourced workspaces; refused together with `project` |
| `project` | the working directory's | a project directory or `pkg://<package>` |
| `seat` | the host settings' `seat:` | a seat number or `auto` |
| `simulator` | the project's `simulator:` | `carla` or `isaac` |
| `map` | CARLA: the first run's map; Isaac: the first map of the project's | the map the stack's pilots start on |
| `logs` | the host settings' `logs:` | the log root |
| `gpu` | the seat's | the GPU index (`CUDA_VISIBLE_DEVICES`) |
| `launcher` | the host settings' `launcher:` | `auto`, `play_launch` or `ros2`: what starts the stack's own launches |
| `stack_args` | empty | `name:=value ...` for the lower-level `stack.launch.py`: `side_args` and `sim_args` are merged by name with what the stack and `swb.yaml` give (yours win; a name the stack sets itself is refused), the rest (`vehicle_catalogs`, ...) after scensim's own |

`ego_launch`, `ego_launch_args`, `agent` and `map_path` are refused by name, given directly
or in `stack_args`: who drives each entity is the testbench's pilots in `testbench.yaml` ([Testbench
file](project-file.md)), and the map is `map:=`.

Both launchers take the same arguments. play_launch binds its web UI and opens its own log
directory (`./play_log/`) before the launch file is read, so `--web-addr` (the seat's web
port, `seat show`) and `--log-dir` are its command line's. The stack runs its world server, its pilots and each run's SSv2 as launches of their own,
each with the host setting `launcher` ([Host settings](host-settings.md)): play_launch
when it is installed at 0.30 or later, else `ros2 launch`; `launcher:=` overrides it for
one stack. Ctrl-C (SIGINT) stops the stack and everything it started.

### Launchers

The stack runs under ROS 2's own `ros2 launch` or under
[play_launch](https://pypi.org/project/play-launch/) 0.30 or later; the verdicts, the
results and the run's logs are the same under both, and each run's `result.json` names the
launcher in its provenance. play_launch is recommended for the following:

| | `ros2 launch` | play_launch |
|---|---|---|
| Logs | each process's output in the launch's `console.log`, led by `[<name>-<n>]`, and rcl's per-process files | a directory per process (`node/<name>/out`, `err`) |
| Web interface | none | every process, its state and its log, at the seat's web port |
| A crashed Autoware component | ends its whole container; scensim ends the run `error` and starts the pilots again | restarted alone; scensim ends the run `error` and starts the pilots again |
| Stop | Ctrl-C; scensim stops each launch it started by its process group | the same |

`runs show`, `runs why` and the timeline read either layout
([Results and logs](results-and-logs.md#the-timeline)). The host setting `launcher:`
chooses which one starts the stack's own launches (the world server, the pilots, each run's
SSv2); `project check` and `seat show` print the choice.

## `scensim run`

```bash
scensim run <testbench>[/<suite> | /<path>] [options]
scensim run <scenario.xosc | scenario.yaml | directory | suite.yaml> [options] # project
```

`<testbench>` runs its `default:` suite; `<testbench>/<suite>` runs `suites/<suite>.yaml`
(looked up before any path); `<testbench>/<path>` runs a file of the testbench, such as
`my_testbench/scenarios/x.xosc` or `my_testbench/suites/x.yaml`. A testbench's stack is
started when none is up.

| Option | |
|---|---|
| `--param NAME=VALUE` | a scenario parameter (repeatable; with a suite, one more grid axis) |
| `--repeat N` | repetitions (a suite's own `repetitions:` otherwise) |
| `--shard i/n` | every n-th run from the i-th (1-based), for several hosts |
| `--resume` | a suite: skip the runs that have a result |
| `--capture LEVEL` | record at `result`, `standard`, `full` or `debug` and keep it whatever the verdict |
| `--timeout S` | SSv2's `global_timeout` (400 s) |
| `--wait S` | wait up to S for the stack to become ready (default: refuse at once) |
| `--start-stack` | start the project's stack when none is up (and leave it up); implied for a testbench |
| `--stop-stack` | as `--start-stack`, then stop the stack it started after the runs |
| `--stack-timeout S` | how long a started stack may take to become ready (default `--wait`, else 1800) |
| `--ready-timeout S` | how long a run waits for its pilots' readiness (600 s) |
| `--seat N`, `--project DIR`, `--logs DIR` | instead of the host settings' and the working directory's |

It prints one line per run, `<verdict> <run id>  <run directory>[  judges: ...][  reason]`,
and a suite's `summary: <summary.csv>`. A run on another map than the stack's pilots
first prints `map <name>  (pilots restarted from <old> in <s> s)`. From Python,
`scensim.api.run_one(project, scenario, overrides)` runs one and returns its result.

## `scensim drive`

Drives one pilot on the project's stack without a scenario: the entity is spawned at a
pose on a map, the world steps, and the pilot is sent a goal.

```bash
scensim drive --spawn 190.8,-130.1,0.3,3.14159 --goal 120,-129.8,0,3.14159 \
    --map Town01 [--entity ego] [--timeout 600] [--start-stack] [--stop-stack]
```

| Option | |
|---|---|
| `--spawn x,y,z,yaw` | where the entity starts, in the map frame (yaw in rad, counter-clockwise from east); `x,y,yaw` puts it at z 0 |
| `--goal x,y,z,yaw` | the pilot is sent it once it is up on the episode, and drives there; the run passes when the pilot reports it arrived, fails at `--timeout` |
| no `--goal` | the world steps until Ctrl-C or `--timeout` while you drive the pilot by hand (an RViz goal); the run passes |
| `--map` | one of the project's maps; default the stack's |
| `--realtime false` | step as fast as the world goes instead of at wall-clock pace |

Any pilot kind takes the goal (a `stack` lifetime pilot; `run` pilots are refused). The
run has a run directory as `scensim run`'s (`resolved.yaml`, `result.json`,
`timeline.jsonl`, `logs/free_drive/`), takes the stack's turn the same way, and exits with
the same codes.

## `scensim runs`

| Command | Does |
|---|---|
| `runs list [<testbench>]` | a testbench's runs (in a testbench's or project's directory, its own): id, verdict, scenario, duration, workspace |
| `runs show <id>` | a run's verdict, reason, scenario, duration, then its timeline |
| `runs why <id>` | a run's verdict, reason, and the first WARN or ERROR in any process's log, with its group |
| `runs summary [<dir>]` | writes `summary.csv` and a merged `junit.xml` for a directory of runs |
| `runs prune [--older-than D] [--verdict V] [--keep-last N] [--to LEVEL] [--dry-run]` | removes runs, or reduces what they keep to a level |

`<id>` is a run id, a unique part of one (`drive_to_goal`), or a run directory; outside a
testbench's directory `show` and `why` look across every testbench's runs, and
`--testbench <name>` names one for any `runs` command. `--root <dir>` reads
another runs directory. A testbench's runs are in `<logs>/<testbench>/runs/`.

## Other tools

| Command | Does |
|---|---|
| `ros2 run scensim_launch carla_poses spawn-points \| ahead <i> --distance <m> \| at <x> <y>` | poses on the town CARLA has loaded, in the scenario's frame, as `<WorldPosition>` elements; CARLA is the host settings' `carla.host` and `carla.port` (your seat's own CARLA when the seat table gives one), or `--host` and `--port`, before or after the subcommand |
| `ros2 run scensim_launch ego_ready --timeout S` | exits 0 once an Autoware in this domain can take a route |
| `ros2 run scensim_launch physical_verdict <junit> --scenario <file>` | CARLA's contacts for a run as a JUnit testcase (a judge) |
| `ros2 run scensim_launch chase_view` | a chase camera window on CARLA's ego |
| `ros2 run scensim_world_check scensim_world_check hello <url> [--wait S]` | asks a world server what it is and whether it is ready |
| `ros2 run scensim_world_check scensim_world_check conformance <url> --map <dir>` | checks a world server against the world protocol (no bridge attached) |
| `ros2 run scensim_agent scensim-agent new <name>` / `check <module:Class>` / `supervise` | the agent SDK ([The agent SDK](agent-sdk.md)) |
| `ros2 run scensim_isaac_world make_world <map dir>` | a map's Isaac world file from the CARLA USD set |
| `ros2 run scensim_isaac_world surfaces check <file>`, `road_detail check <file>` | checks a surface map or a road detail |
| `ros2 run autoware_sim_profile check_launch_args <name:=value ...>` | checks arguments for the Autoware profile |
| `ros2 run autoware_sim_profile setup_autoware_data` | once per machine: Autoware's writable data directory |
