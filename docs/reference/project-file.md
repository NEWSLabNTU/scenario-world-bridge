# The testbench file

Every key of `testbench.yaml`, of a suite file and of `scensim.lock`. A key not listed is refused by
name (`scensim testbench check <testbench>`). Why a testbench owns what it does:
[How it works](../explanation/how-it-works.md).

## `testbench.yaml`

```yaml
simulator: carla                        # carla | isaac | none (world.server)
assets:                                 # what to fetch (below)
  - dataset: carla-town-autoware-maps
    take: [Town01, Town05]
  - path: ../shared/my_map              # a directory of your own, linked as is
    as: my_map
maps: [Town01, Town05]                  # the maps the stack serves (default below)
pilots:                                 # entity name (or fnmatch pattern) -> pilot
  ego:     {kind: autoware, vehicle: tesla_model3}
  bg_av_*: {kind: autoware}
  cut_in:  {kind: command, run: [python3, -m, my_policies.cut_in], lifetime: run}
vehicles: [vehicles/my_car.yaml]        # own cars: specs (sim_vehicle_spec format)
ground: {}                              # Isaac: per map, surface files
logging: {capture: standard, keep: {passed: result, failed: full, error: full}}
judges: []                              # programs over each finished run
world: {}                               # weather, NPC motion, the bridge's config
carla: {}                               # CARLA: its quality and arguments
isaac: {}                               # Isaac: the world server's options
ssv2: {}                                # SSv2's run settings for every run
```

`simulator`, `assets` and `pilots` are required; the rest default as shown.

| Key | Holds |
|---|---|
| `simulator` | `carla`, `isaac`, or `none` with a world server of your own ([World](#world)); `simulator:=` on the stack overrides it |
| `assets` | a list of datasets and paths ([Assets](#assets)) |
| `maps` | map names under `assets/maps/`; default: every map the testbench's scenarios name. Give it when runs use maps the testbench's own scenarios do not, or to fix their order |
| `pilots` | entity name or pattern to a pilot ([Pilots](#pilots)) |
| `vehicles` | spec files of your own cars: the stack makes each car's catalog entry, physics, sensors and frames from its spec when it starts, into `state/vehicles/`; a scenario names the catalog as `$(env SCENSIM_VEHICLES)` ([Your car](../how-to/your-car.md), [Supported and not supported](limits.md#testbenches)) |
| `ground` | Isaac: `{<map>: {surfaces: <file>, road_detail: <file>, surface_maps: {<name>: <file>}}}`, paths in the testbench; a scenario names a surface map with `scensim.surface_map` ([Road physics](road-physics.md)) |
| `logging` | what runs record and keep ([Logging](#logging)) |
| `judges` | programs run over each finished run ([Judges](#judges)) |
| `world` | the simulation side's settings: weather, NPC motion, contact reports, config files ([World](#world)) |
| `carla` | CARLA: the quality and arguments the stack starts it with ([CARLA](#carla)) |
| `isaac` | Isaac: the world server's options ([Isaac](#isaac)) |
| `ssv2` | SSv2's run settings, the default for every run ([Run settings](#run-settings)) |

### Assets

| Entry key | |
|---|---|
| `dataset` | a Hugging Face dataset's name, in the `jerry73204` namespace unless `repo:` says otherwise |
| `repo` | `<namespace>/<name>`, for a dataset elsewhere |
| `take` | the subpaths to fetch (globs relative to the dataset's root); default: all of it |
| `revision` | a tag or commit; default: the newest tag, else `main` |
| `path` | a directory of your own, relative to the testbench, linked as it is |
| `as` | the name it is linked under (default: the directory's name) |

`assets fetch` links a map (a directory holding `lanelet2_map.osm`) under
`assets/maps/<map>/`, the CARLA USD set under `assets/carla-usd/`, and the rest of a
dataset under `assets/<dataset>/`. Scenarios name maps as `$(env SCENSIM_MAPS)/<map>`; a world
file's `usd:` may use `${SCENSIM_CARLA_USD}`; the stack sets both for every process. The
published datasets: [Maps and assets](maps-and-assets.md#datasets).

### Pilots

Each key is a scenario entity name, or an `fnmatch` pattern (`bg_av_*`); the scenario's
entities with the controller `Autoware` or `agent` (or the property `isEgo`) must each
match one.

| Key | Kinds | |
|---|---|---|
| `kind` | all | `autoware`, `carla_basic` (CARLA only), `simple`, `launch`, `command`, `none` |
| `vehicle` | all | the car's spec name (`tesla_model3`); must agree with the entity's catalog entry |
| `args` | all | launch arguments for the pilot's launch (Autoware's profile, your launch); substitutions apply |
| `lifetime` | all | `stack` (default): started with the stack, serves every run; `run`: started afresh for each run |
| `ready` | all | a command that exits 0 when the pilot is ready; default `autoware`: `ego_ready`, the others: started is ready |
| `inputs` | all | what the pilot uses, recorded in results: `sensors`, `map`, `ground_truth` |
| `rviz` | all | `true`: open RViz for this pilot |
| `component` | the ego, on Isaac | the car's own control component in place of its onboard computers: `{entry_point, params, truth, workspace}` ([Your car's control component](#your-cars-control-component)) |
| `autoware_launch` | `autoware` | another `autoware.launch.xml` for the profile to include |
| `overlays` | `autoware`, `launch` | ROS workspaces of your own the pilot sources, in order, after this workspace's installs and before the TensorRT runtime: a list of `setup.bash` files, each relative to the testbench or `{workspace:<name>}/<path>` under a directory the host settings name (`workspaces:`, [Host settings](host-settings.md)) |
| `launch` | `launch` | the driver: `"<package> <file>"` or a path |
| `agent` | `launch` | `own`: the launch registers the entity itself; default scensim's vehicle agent |
| `run` | `command` | the program's words (not a shell), relative paths the testbench's |

An `autoware` pilot's `args` (and a suite's `pilot_args` for it) are checked by the
Autoware profile's own argument check when `testbench check` runs, and so before the stack
starts: a renamed argument (`carla_control`, now `sim_control`), a value outside its
choices (`ndt:` is `autoware`, `cuda` or `none`) and a pair that cannot work together
(`ndt: none` with a diagnostic graph that judges NDT) are refused with the profile's message.
A value holding a substitution is known per run only and is not checked.

### Your car's control component

On Isaac, the ego's car may run your own model of its control component (its speed controller,
motor and steering servo) in place of the onboard computers scensim ships. It is a Python class in
a package of yours, declared as an entry point in the group `scensim.vehicle_components`, and
the Isaac world server steps it at every physics step.

```yaml
pilots:
  ego:
    kind: launch
    vehicle: tesla_model3
    overlays: ["{workspace:racer}/install/setup.bash"]
    component:
      entry_point: my_racer.vesc_servo
      params: {erpm_gain: 4614.0}
      truth: {speed: {latency_s: 0.02, noise_std: 0.01}}
```

| Key | |
|---|---|
| `entry_point` | its name in the entry point group `scensim.vehicle_components`; `ideal` and `ev` are the shipped onboard computers |
| `params` | a mapping handed to it as is |
| `truth` | for each ground-truth input it declares, how it is delivered: `latency_s` (the value that many seconds ago) and `noise_std` (Gaussian noise, seeded by the run); default none and none |
| `workspace` | the `setup.bash` of the workspace that holds its package, as an `overlays` entry; default the pilot's `overlays` |

The world server runs Isaac's own Python, so the component must be pure Python, using the
standard library and numpy. A compiled model goes through an FMU instead. scensim hands Isaac only
the workspace's Python package directories, never the rest of its environment.

`testbench check`:
- refuses an entry point that no package in the workspace declares;
- loads and times the component in Isaac's Python;
- refuses it when it does not load, or when it takes commands the vehicle interface does not
  give;
- refuses a `truth` setting for an input it does not declare;
- warns when its mean time per step is over 60 / `isaac.physics_hz` ms (0.1 ms at 600 Hz). The
  run then goes ahead, only slower.

Each run's `provenance.components` records the entry point, `params`, `truth`, and the package
found: its name, version, module file and that file's sha256.

`overlays` is how a driver uses packages built in a workspace of its own, as
autoware-drift's `drift.launch.xml` includes its `drift_controller`:

```yaml
pilots:
  ego:
    kind: autoware
    vehicle: drift_mustang
    overlays: ["{workspace:autoware-drift}/install/setup.bash"]
    args:
      extra_launch: "{workspace:autoware-drift}/install/drift_launch/share/\
        drift_launch/launch/drift.launch.xml"
```

`testbench check` refuses a workspace name the host settings do not have and a setup file that
is not there, naming the key to set. An absolute or `~` path works too, but it is a host path
in a testbench file, and `testbench check` warns about it: name the directory in the host
settings instead, so the testbench reads the same on every host. The stack's `stack.json` (each
pilot's `overlays`) and every run's `result.json` (`provenance.overlays`) record each setup
file's path and, when its workspace is a git checkout, the commit.

Substitutions in `args` and `run`: `{seed}`, `{run_dir}`, `{run_id}`, `{map_path}` (the
run's map directory, the one a launch pilot gets as `map_path`), `{param:<name>}`,
`{workspace:<name>}` (a directory the host settings' `workspaces:` names; an unknown name is
refused). A pilot
using `{param:...}` or `{map_path}` must be `lifetime: run`. Every pilot's launch receives the
contract `entity`, `relay`, `map_path`, `vehicle`, `seed`, `run_dir`, `sim`,
`use_sim_time:=true`; a `command` program gets `--relay --entity --seed [--run-dir]`. On
Isaac only one pilot has a vehicle side (the ego). [Your autopilot as a
pilot](../how-to/your-autopilot.md).

### Logging

| Key | |
|---|---|
| `capture` | what a run records: `result` (result, resolved inputs, JUnit, timeline), `standard` (+ each process's log for the run, SSv2's logs), `full` (+ a rosbag of the key topics per pilot, host samples), `debug` (+ every topic, the scenario domain's too). Default `standard` |
| `keep` | per verdict, the level kept once the verdict is known: `{passed: result, failed: full, error: full}` (default); a verdict it does not name keeps everything. A run that failed or ended in error, a run a judge found `failed` or `error`, and a run with a contact of the ego keep everything, whatever `keep` says. The files the judges read (their `reads`) are kept wherever the logs are, `standard` and up |
| `budget` | `{per_run: 2GB, total: 200GB}`: refuse a run or suite that cannot fit on the log disk |
| `topics` | the topics `full` records, instead of the key topics |
| `gap_trace` | `true`: the bridge records its gap trace (both worlds, frame by frame) for every run, into the run's `gap_trace/`; default `false`. `testbench.yaml` only: the stack records it for every run or for none. A judge with `needs: [gap_trace]` turns it on too |

`scensim run --capture <level>` records one run at that level and keeps it whatever
the verdict. What each level keeps: [Results and logs](results-and-logs.md).

### Judges

```yaml
judges:
  - name: comfort                     # writes judges/comfort.json
    run: [python3, -m, my_judges.comfort, "{run_dir}"]
    version: "1"
    on: [passed, failed]              # default; error runs are not judged
    timeout: 120                      # s, default
    needs: []                         # gap_trace, context: what the run must record
    reads: []                         # the run's files it reads, kept with the logs
```

A judge runs after each run whose verdict is in `on`. It is
given the run directory as its last argument, unless `{run_dir}` or `{scenario}` (the run's
scenario file) stands in `run`, and as `SCENSIM_RUN_DIR`. It may print one JSON object, whose
`verdict` (`passed` or `failed`) is the judge's; without one, its exit code is (0 passed,
1 failed) and its last line the message. Any other exit, JSON that is not an object, or a
timeout is `error`. scensim writes the record, with the judge's name, version, exit code and
duration, to `judges/<name>.json`, beside the run's verdict, never in its place.

`needs` turns on what a judge reads that a run records only on request: `gap_trace`, the
bridge's gap trace (for the whole stack), and `context`, a log of SSv2's storyboard state
changes for each run, `context.jsonl` (one JSON object per line: `sim`, `element`, `from`,
`to`).

`reads` names the files of the run directory the judge reads that no level keeps by name, such
as a log a `command` pilot writes into the run directory: paths or globs relative to it
(`[coop_a.jsonl, out/*.csv]`). A run kept at `standard`, `full` or `debug` keeps them; one kept
at `result` does not. Without it such a file is kept only at `debug`, or when the run keeps
everything.

Two judges come with scensim, both over the run's gap trace:

```yaml
judges:
  - name: gap                         # gap_oracle on each trace of the run
    run: [python3, -m, scensim.gap, oracle]
    needs: [gap_trace]
  - name: incident                    # incident_judge, a demo incident's row
    run: [python3, -m, scensim.gap, incident, --incident, auto,
          --context, context.jsonl]
    needs: [gap_trace, context]
```

`oracle` runs `gap_oracle` with the run's JUnit and writes its findings beside each trace
(`gap_trace/<trace>.findings.json`). Its verdict is `failed` when the two worlds disagree on
the outcome (a G4 class such as `PASS+contact`: SSv2 passed while the ego hit something it
could not see), else `passed`; `--fail-on never` keeps the findings as data only, and
`gap_oracle`'s own options (`--pos 0.3`, ...) pass through. `incident` runs
`incident_judge`, which also reads the storyboard log (`--context`, relative to the run
directory). `--incident auto` takes the incident's number from the scenario's name
(`town01_i3_red_light.xosc` is incident 3), so one judge serves a suite of incidents; a number
works too. No trace, no storyboard log, or an incident without a row in `incident_judge`'s
table (i8 has none yet) is `error`, never `passed`.

## World

```yaml
world:
  weather: ClearNoon                  # a preset, or parameters:
  # weather: {preset: ClearNoon, cloudiness: 30, sun_altitude_angle: 60}
  npc_motion: tracked                 # tracked | kinematic
  contact_report_dir: off             # unset: each run's in its contacts/
  config_file: config/bridge.yaml     # the bridge's settings file
  world_config_file: config/carla.yaml   # CARLA: the world server's settings file
  # server: [my_world, my_world_server]   # your world server, not the stock one
```

| Key | |
|---|---|
| `weather` | a preset by the world server's name (CARLA's: `ClearNoon`, `WetSunset`, ...), kept across map loads; or a mapping of parameters by name, with an optional `preset` under them. Parameters are set once the bridge answers, before the stack is ready; a name the world server does not know fails the stack |
| `npc_motion` | how SSv2-driven NPCs move: `tracked` (default) or `kinematic` |
| `contact_report_dir` | unset: each run's contact reports in its `contacts/`; `off`: none; a directory: yours, not cut per run |
| `config_file` | the bridge's settings file instead of the installed one, relative to the testbench |
| `world_config_file` | CARLA: the world server's settings file instead of the installed one; refused beside `server` |
| `server` | a world server of your own, `[<package>, <executable>, <args>...]` as `ros2 run` takes them, started in place of the stock one ([Your world server](../how-to/your-world-server.md)) |

These go to the bridge and the world server beside scensim's own settings, never in their
place. What scensim sets itself is refused here by name, with what to give instead: the state
directory, the gap trace directory (`logging.gap_trace`), the world server and its ports.

### `simulator: none`

For a simulator scensim does not ship, set `simulator: none` and give `world.server`. The
stack then starts:
- the bridge and the agent relay, as always;
- your world server, on the seat's world port (`--port`);
- the pilots.

It starts no simulator; your server starts and owns its own. It also skips what belongs to a
stock simulator:
- no CARLA is decided or started, and `carla:` is refused;
- no Isaac, world or Isaac ego is given, and `isaac:` is refused;
- no stock vehicle side runs, so pilots of kind `autoware` and `carla_basic` are refused: use
  `launch` or `command` with a vehicle side of your own;
- no CARLA physics or other simulator arguments are passed.

Maps are still needed: the scenarios' maps are fetched, and SSv2 and the bridge read their
Lanelet2 map. `none` without `world.server` is refused. `stack.json` records
`simulator: none`, with `simulator_start` set to `world_server` and `carla` set to null.

## CARLA

```yaml
carla:
  quality: epic                       # epic | low
  args: [-benchmark, -fps=20]         # more CARLA arguments, after scensim's own
```

How the stack starts CARLA, when it starts it (`carla.root` set and nothing serving on the
port, or `carla.start: start`; see [Host settings](host-settings.md)).

| Key | |
|---|---|
| `quality` | CARLA's `-quality-level`: `epic` (default) or `low`. The quality changes what sensors see (level of detail, foliage), so `scripts/t3.py compare` refuses to compare results run at another quality or args than the reference's, and names both |
| `args` | more command-line arguments for CARLA, after scensim's own: device or renderer choices (`-graphicsadapter=1`, `-vulkan`), `-benchmark`, `-fps=`, and so on |

The same two keys in the host's `carla` block override the testbench's, each key whole: an
`args` there replaces the testbench's list instead of adding to it. The host's block is in
`.scensim/local.yaml` first, then `~/.config/scensim/host.yaml`. `root`, `host`, `port`,
`start` and `ready_timeout` are host settings, not testbench keys.

scensim sets some flags itself and refuses them in `args` by name, naming the key that sets each:
- `-carla-rpc-port`, `-carla-port`, `-carla-streaming-port` and `-carla-secondary-port` follow
  `carla.port`, the seat's port;
- `-quality-level` is `quality`;
- `-RenderOffScreen` and `-nosound` are always given.

An entry of the form `NAME=value` is refused as an environment variable. scensim pins
`VK_ICD_FILENAMES` to the NVIDIA card itself when it is unset.

`quality: low` is refused with CARLA 0.9.x when the stack's maps span more than one town: at
Low, 0.9.16 segfaults on any town change. It is allowed with CARLA 1.0, and on a single town. A
CARLA whose version the stack cannot tell is read as 0.9.x.

With a CARLA the stack does not start, quality and args do not apply. That is `carla.start:
external`, an `auto` that finds one already serving, or a remote host. The stack then warns,
and records them as not applied.

`stack.json` records the effective settings in `carla.launch`: `quality`, `args`, `from` (each
key's source: `testbench`, `host` or `default`), `applied`, and `why` when not applied. It
records the command line in `carla.argv`, and `VK_ICD_FILENAMES` in `carla.env` when the stack
pinned it. Every run's `result.json` records them as `provenance.carla` ([Results and
logs](results-and-logs.md)). `carla:` is refused on an Isaac testbench.

## Isaac

```yaml
isaac:
  ego: config/ego_lockstep.yaml       # an ego config of yours; default | none
  render: headless                    # auto | window | headless
  follow: false                       # true | false | editor
  physics_hz: 240                     # the physics rate, Hz (default 60)
  preload: [Town05]                   # more maps the world server loads at start
  wheels_topic: true                  # /scensim/wheels on the ego's domain
```

| Key | |
|---|---|
| `ego` | the ego's config for the world server: `default` (the Tesla rig), `none` (a kinematic box), or a file of the testbench. Default: `default` when a pilot has a vehicle side, else `none` |
| `render` | `auto` (a window when a display is set), `window`, `headless` |
| `follow` | with a window: `true` chases the ego, `editor` keeps Isaac's editor around the chase |
| `physics_hz` | physics steps per simulated second; each SSv2 step is split into steps no longer than one over this |
| `preload` | maps the world server loads at start beside the stack's first; a run on one of them moves the pilots there instead of being refused. Each must be fetched |
| `wheels_topic` | `false` turns `/scensim/wheels` off |

`isaac:` is refused on a CARLA testbench. The world (`world`) and the state directory are
scensim's: give `maps:` and `map:=` instead.

## Run settings

```yaml
ssv2:
  global_timeout: 400                 # s of wall clock per scenario (per variant)
  frame_rate: 20                      # frames per simulated second (100: 10 ms)
  global_real_time_factor: 1.0        # simulated seconds per real second
  initialize_duration: 120            # s from the scenario's start to the ego's engage
  managed_ego: true                   # false: the scenario's ego is handed to no pilot
```

`ssv2:` in `testbench.yaml` is every run's default; a suite's `ssv2:` replaces the keys it names
for its runs, a suite item's (`scenarios: [{path: ..., ssv2: {...}}]`) for that item's, and
`scensim run --timeout` replaces `global_timeout` for one command. With
`managed_ego: false` SSv2 does not hand its ego to a pilot: the ego needs no pilot, the run
waits for none, and the scenario must not use autonomy actions (`RoutingAction` and the
like). A standing ego on Isaac is the usual case.

## Suites

A suite is a YAML file in the testbench, given to `scensim run`:

```yaml
scenarios: [scenarios/cut_in.xosc, scenarios/awf/]   # files and directories
parameters:                                          # a grid over ParameterDeclarations
  gap_m: [6, 10, 14]
  policy: [yield, push]
cases:                                               # parameter sets, each run as it is
  - {speed: 10, lead_gap: 5}
  - {speed: 20, lead_gap: 8}
distribution: scenarios/cut_in_dist.xosc             # or a ParameterValueDistribution
repetitions: 5
seed: 1000                                           # run k gets seed 1000 + k
expect: {awf/known_issue.xosc: failed}
retry: {on: error, times: 2}
pilot_args: {cut_in: {policy: "{param:policy}"}}
logging: {capture: full, keep: {passed: result}}
ssv2: {global_timeout: 600}
```

| Key | |
|---|---|
| `scenarios` | files and directories (every scenario in them, sorted); an item may be a mapping `{path: <file or directory>, ssv2: {...}}`, run settings for its runs |
| `parameters` | name to a list of values; the runs are the grid's every combination |
| `cases` | a list of parameter sets, each a mapping name to one value, run as it is; with `parameters`, each case times the grid. A name is in one of them, not both |
| `distribution` | an OpenSCENARIO `ParameterValueDistribution` file, passed to SSv2 as it is, one run per repetition (this SSv2 does not expand it yet) |
| `repetitions` | runs per combination (default 1); run ids end `_r0`, `_r1`, ... |
| `seed` | run k gets `seed + k` (`run --seeds A-B` overrides it: seeds A to B, one run each) |
| `expect` | scenario (relative to the testbench or `scenarios/`) to the verdict expected; the suite exits 0 when runs end as expected |
| `retry` | `{on: error, times: N}`: run again a run the stack could not judge |
| `pilot_args` | per entity, `args` for `lifetime: run` pilots, substitutions applied |
| `logging` | as in `testbench.yaml`, for this suite's runs (not `gap_trace`, which is the stack's) |
| `ssv2` | run settings for this suite's runs, over `testbench.yaml`'s ([Run settings](#run-settings)) |

A run id is `<time>_<scenario>[_<name>-<value>...]_r<k>`.

## `scensim.lock`

Written by `assets fetch`; commit it. Runs read the lock, never the tag.

```yaml
version: 1
datasets:
  carla-town-autoware-maps:
    repo: jerry73204/carla-town-autoware-maps
    commit: 6b1bd8b9dd315ae36bb401fc137a09583e67d53d
    take: [Town01]
    files: {Town01/lanelet2_map.osm: "sha256:...", ...}
paths:
  my_map: {path: ../shared/my_map}
```
