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
```

`simulator`, `assets` and `pilots` are required; the rest default as shown.

| Key | Holds |
|---|---|
| `simulator` | `carla` or `isaac`; `simulator:=` on the stack overrides it |
| `assets` | a list of datasets and paths ([Assets](#assets)) |
| `maps` | map names under `assets/maps/`; default: every map the project's scenarios name. Give it when runs use maps the project's own scenarios do not, or to fix their order |
| `pilots` | entity name or pattern to a pilot ([Pilots](#pilots)) |
| `vehicles` | spec files of your own cars; checked to exist, not yet loaded by the stack ([Your car](../how-to/your-car.md)) |
| `ground` | Isaac: `{<map>: {surfaces: <file>, road_detail: <file>}}`, paths in the project ([Road physics](road-physics.md)) |
| `logging` | what runs record and keep ([Logging](#logging)) |
| `judges` | programs run over each finished run ([Judges](#judges)) |

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
| `kind` | all | `autoware`, `carla_basic` (CARLA only), `launch`, `command`, `none` |
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
```

A judge runs in the project directory after each run whose verdict is in `on`. It is
given the run directory as its last argument, unless `{run_dir}` or `{scenario}` (the run's
scenario file) stands in `run`, and as `SWB_RUN_DIR`. It may print one JSON object, whose
`verdict` (`passed` or `failed`) is the judge's; without one, its exit code is (0 passed,
1 failed) and its last line the message. Any other exit, JSON that is not an object, or a
timeout is `error`. swb writes the record, with the judge's name, version, exit code and
duration, to `judges/<name>.json`, beside the run's verdict, never in its place.

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
```

| Key | |
|---|---|
| `scenarios` | files and directories (every scenario in them, sorted) |
| `parameters` | name to a list of values; the runs are the grid's every combination |
| `distribution` | an OpenSCENARIO `ParameterValueDistribution` file, passed to SSv2 as it is, one run per repetition (this SSv2 does not expand it yet) |
| `repetitions` | runs per combination (default 1); run ids end `_r0`, `_r1`, ... |
| `seed` | run k gets `seed + k` |
| `expect` | scenario (relative to the project or `scenarios/`) to the verdict expected; the suite exits 0 when runs end as expected |
| `retry` | `{on: error, times: N}`: run again a run the stack could not judge |
| `pilot_args` | per entity, `args` for `lifetime: run` pilots, substitutions applied |
| `logging` | as in `swb.yaml`, for this suite's runs |

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
