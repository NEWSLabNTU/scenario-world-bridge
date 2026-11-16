# The project file

Every key of `swb.yaml`, of a suite file and of `swb.lock`. A key not listed is refused by
name (`ros2 run swb project check`). Why a project owns what it does:
[How it works](../explanation/how-it-works.md).

## `swb.yaml`

```yaml
simulator: carla                        # carla | isaac
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
isaac: {}                               # Isaac: the world server's options
ssv2: {}                                # SSv2's run settings for every run
```

`simulator`, `assets` and `pilots` are required; the rest default as shown.

| Key | Holds |
|---|---|
| `simulator` | `carla` or `isaac`; `simulator:=` on the stack overrides it |
| `assets` | a list of datasets and paths ([Assets](#assets)) |
| `maps` | map names under `assets/maps/`; default: every map the project's scenarios name. Give it when runs use maps the project's own scenarios do not, or to fix their order |
| `pilots` | entity name or pattern to a pilot ([Pilots](#pilots)) |
| `vehicles` | spec files of your own cars: the stack makes each car's catalog entry, physics, sensors and frames from its spec when it starts, into `state/vehicles/`; a scenario names the catalog as `$(env SWB_VEHICLES)` ([Your car](../how-to/your-car.md), [Supported and not supported](limits.md#projects)) |
| `ground` | Isaac: `{<map>: {surfaces: <file>, road_detail: <file>}}`, paths in the project ([Road physics](road-physics.md)) |
| `logging` | what runs record and keep ([Logging](#logging)) |
| `judges` | programs run over each finished run ([Judges](#judges)) |
| `world` | the simulation side's settings: weather, NPC motion, contact reports, config files ([World](#world)) |
| `isaac` | Isaac: the world server's options ([Isaac](#isaac)) |
| `ssv2` | SSv2's run settings, the default for every run ([Run settings](#run-settings)) |

### Assets

| Entry key | |
|---|---|
| `dataset` | a Hugging Face dataset's name, in the `jerry73204` namespace unless `repo:` says otherwise |
| `repo` | `<namespace>/<name>`, for a dataset elsewhere |
| `take` | the subpaths to fetch (globs relative to the dataset's root); default: all of it |
| `revision` | a tag or commit; default: the newest tag, else `main` |
| `path` | a directory of your own, relative to the project, linked as it is |
| `as` | the name it is linked under (default: the directory's name) |

`assets fetch` links a map (a directory holding `lanelet2_map.osm`) under
`assets/maps/<map>/`, the CARLA USD set under `assets/carla-usd/`, and the rest of a
dataset under `assets/<dataset>/`. Scenarios name maps as `$(env SWB_MAPS)/<map>`; a world
file's `usd:` may use `${SWB_CARLA_USD}`; the stack sets both for every process. The
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
| `autoware_launch` | `autoware` | another `autoware.launch.xml` for the profile to include |
| `launch` | `launch` | the driver: `"<package> <file>"` or a path |
| `agent` | `launch` | `own`: the launch registers the entity itself; default swb's vehicle agent |
| `run` | `command` | the program's words (not a shell), relative paths the project's |

An `autoware` pilot's `args` (and a suite's `pilot_args` for it) are checked by the
Autoware profile's own argument check when `project check` runs, and so before the stack
starts: a renamed argument (`carla_control`, now `sim_control`), a value outside its
choices (`ndt:` is `autoware`, `cuda` or `none`) and a pair that cannot work together
(`ndt: none` with a diagnostic graph that judges NDT) are refused with the profile's message.
A value holding a substitution is known per run only and is not checked.

Substitutions in `args` and `run`: `{seed}`, `{run_dir}`, `{run_id}`, `{param:<name>}`. A
pilot using `{param:...}` must be `lifetime: run`. Every pilot's launch receives the
contract `entity`, `relay`, `map_path`, `vehicle`, `seed`, `run_dir`, `sim`,
`use_sim_time:=true`; a `command` program gets `--relay --entity --seed [--run-dir]`. On
Isaac only one pilot has a vehicle side (the ego). [Your autopilot as a
pilot](../how-to/your-autopilot.md).

### Logging

| Key | |
|---|---|
| `capture` | what a run records: `result` (result, resolved inputs, JUnit, timeline), `standard` (+ each process's log for the run, SSv2's logs), `full` (+ a rosbag of the key topics per pilot, host samples), `debug` (+ every topic, the scenario domain's too). Default `standard` |
| `keep` | per verdict, the level kept once the verdict is known: `{passed: result, failed: full, error: full}` (default) |
| `budget` | `{per_run: 2GB, total: 200GB}`: refuse a run or suite that cannot fit on the log disk |
| `topics` | the topics `full` records, instead of the key topics |
| `gap_trace` | `true`: the bridge records its gap trace (both worlds, frame by frame) for every run, into the run's `gap_trace/`; default `false`. `swb.yaml` only: the stack records it for every run or for none. A judge with `needs: [gap_trace]` turns it on too |

`ros2 run swb run --capture <level>` records one run at that level and keeps it whatever
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
```

A judge runs in the project directory after each run whose verdict is in `on`. It is
given the run directory as its last argument, unless `{run_dir}` or `{scenario}` (the run's
scenario file) stands in `run`, and as `SWB_RUN_DIR`. It may print one JSON object, whose
`verdict` (`passed` or `failed`) is the judge's; without one, its exit code is (0 passed,
1 failed) and its last line the message. Any other exit, JSON that is not an object, or a
timeout is `error`. swb writes the record, with the judge's name, version, exit code and
duration, to `judges/<name>.json`, beside the run's verdict, never in its place.

`needs` turns on what a judge reads that a run records only on request: `gap_trace`, the
bridge's gap trace (for the whole stack), and `context`, a log of SSv2's storyboard state
changes for each run, `context.jsonl` (one JSON object per line: `sim`, `element`, `from`,
`to`).

Two judges come with swb, both over the run's gap trace:

```yaml
judges:
  - name: gap                         # gap_oracle on each trace of the run
    run: [python3, -m, swb.gap, oracle]
    needs: [gap_trace]
  - name: incident                    # incident_judge, a demo incident's row
    run: [python3, -m, swb.gap, incident, --incident, auto, --context, context.jsonl]
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
table (i7 and i8 have none yet) is `error`, never `passed`.

## World

```yaml
world:
  weather: ClearNoon                  # a preset, or parameters:
  # weather: {preset: ClearNoon, cloudiness: 30, sun_altitude_angle: 60}
  npc_motion: tracked                 # tracked | kinematic
  contact_report_dir: off             # unset: each run's in its contacts/
  config_file: config/bridge.yaml     # the bridge's settings file
  world_config_file: config/carla.yaml   # CARLA: the world server's settings file
```

| Key | |
|---|---|
| `weather` | a preset by the world server's name (CARLA's: `ClearNoon`, `WetSunset`, ...), kept across map loads; or a mapping of parameters by name, with an optional `preset` under them. Parameters are set once the bridge answers, before the stack is ready; a name the world server does not know fails the stack |
| `npc_motion` | how SSv2-driven NPCs move: `tracked` (default) or `kinematic` |
| `contact_report_dir` | unset: each run's contact reports in its `contacts/`; `off`: none; a directory: yours, not cut per run |
| `config_file` | the bridge's settings file instead of the installed one, relative to the project |
| `world_config_file` | CARLA: the world server's settings file instead of the installed one |

These go to the bridge and the world server beside swb's own settings, never in their
place. What swb sets itself is refused here by name, with what to give instead: the state
directory, the gap trace directory (`logging.gap_trace`), the world server and its ports.

## Isaac

```yaml
isaac:
  ego: config/ego_lockstep.yaml       # an ego config of yours; default | none
  render: headless                    # auto | window | headless
  follow: false                       # true | false | editor
  physics_hz: 240                     # the physics rate, Hz (default 60)
  preload: [Town05]                   # more maps the world server loads at start
  wheels_topic: true                  # /swb/wheels on the ego's domain
```

| Key | |
|---|---|
| `ego` | the ego's config for the world server: `default` (the Tesla rig), `none` (a kinematic box), or a file of the project. Default: `default` when a pilot has a vehicle side, else `none` |
| `render` | `auto` (a window when a display is set), `window`, `headless` |
| `follow` | with a window: `true` chases the ego, `editor` keeps Isaac's editor around the chase |
| `physics_hz` | physics steps per simulated second; each SSv2 step is split into steps no longer than one over this |
| `preload` | maps the world server loads at start beside the stack's first; a run on one of them moves the pilots there instead of being refused. Each must be fetched |
| `wheels_topic` | `false` turns `/swb/wheels` off |

`isaac:` is refused on a CARLA project. The world (`world`) and the state directory are
swb's: give `maps:` and `map:=` instead.

## Run settings

```yaml
ssv2:
  global_timeout: 400                 # s of wall clock per scenario (per variant)
  frame_rate: 20                      # frames per simulated second (100: 10 ms)
  global_real_time_factor: 1.0        # simulated seconds per real second
  initialize_duration: 120            # s from the scenario's start to the ego's engage
  managed_ego: true                   # false: the scenario's ego is handed to no pilot
```

`ssv2:` in `swb.yaml` is every run's default; a suite's `ssv2:` replaces the keys it names
for its runs, a suite item's (`scenarios: [{path: ..., ssv2: {...}}]`) for that item's, and
`ros2 run swb run --timeout` replaces `global_timeout` for one command. With
`managed_ego: false` SSv2 does not hand its ego to a pilot: the ego needs no pilot, the run
waits for none, and the scenario must not use autonomy actions (`RoutingAction` and the
like). A standing ego on Isaac is the usual case.

## Suites

A suite is a YAML file in the project, given to `ros2 run swb run`:

```yaml
scenarios: [scenarios/cut_in.xosc, scenarios/awf/]   # files and directories
parameters:                                          # a grid over ParameterDeclarations
  gap_m: [6, 10, 14]
  policy: [yield, push]
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
| `distribution` | an OpenSCENARIO `ParameterValueDistribution` file, passed to SSv2 as it is, one run per repetition (this SSv2 does not expand it yet) |
| `repetitions` | runs per combination (default 1); run ids end `_r0`, `_r1`, ... |
| `seed` | run k gets `seed + k` |
| `expect` | scenario (relative to the project or `scenarios/`) to the verdict expected; the suite exits 0 when runs end as expected |
| `retry` | `{on: error, times: N}`: run again a run the stack could not judge |
| `pilot_args` | per entity, `args` for `lifetime: run` pilots, substitutions applied |
| `logging` | as in `swb.yaml`, for this suite's runs (not `gap_trace`, which is the stack's) |
| `ssv2` | run settings for this suite's runs, over `swb.yaml`'s ([Run settings](#run-settings)) |

A run id is `<time>_<scenario>[_<name>-<value>...]_r<k>`.

## `swb.lock`

Written by `assets fetch`; commit it. Runs read the lock, never the tag.

```yaml
version: 1
datasets:
  carla-town-autoware-maps:
    repo: jerry73204/carla-town-autoware-maps
    commit: e94113508882f9dbd4e2e70b81d645d7f290bf1a
    take: [Town01]
    files: {Town01/lanelet2_map.osm: "sha256:...", ...}
paths:
  my_map: {path: ../shared/my_map}
```
