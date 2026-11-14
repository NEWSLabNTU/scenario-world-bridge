# Projects, Stacks, Runs and Pilots

How a user drives scenario-world-bridge (swb) without knowing its insides: a **project**
names what a use case needs; a long-lived **stack** brings up the simulator side and the
**pilots**; **runs** and **suites** go against it and leave **results** and **logs**; a
**seat** keeps stacks on one host apart. The phase that builds this is
[roadmap 076](../roadmap/076-assets-interface-and-docs-by-reader.md); the use-case study
behind it is in [076-study/](../roadmap/076-study/).

## Words

| Word | Means |
|---|---|
| project | a directory (a git repository or a ROS package) holding one use case |
| stack | the long-lived processes of a project on one seat: world server, bridge, relay, pilots |
| run | one scenario (one parameter set, one repetition) against a live stack, ending with a result |
| suite | many runs: a list, a directory, a parameter grid or distribution, repetitions |
| pilot | what drives one scenario entity: a vehicle side swb starts, plus a driver |
| seat | one stack's share of a host: ROS domains, ports, web port, GPU |

## Ownership

| Fact | Owner |
|---|---|
| map, entities, their cars, which entities an external pilot drives | the scenario (`LogicFile`, `CatalogReference`, `Controller`) |
| which pilot drives an entity name | the project file, `pilots:` |
| which assets, at which versions | the project file `assets:`, pinned by `swb.lock` |
| the simulator | the project file `simulator:`, overridable per stack |
| seats, GPUs, install paths, log and cache folders | the host settings (files, never exports) |

## Commands

All in the ament_python package `swb`; the stack is a launch file in `swb_launch`.

| Command | Does |
|---|---|
| `ros2 run swb project new <dir> [--template <name>]` | writes a starter project |
| `ros2 run swb project check [<dir>]` | validates the project against its scenarios and assets |
| `ros2 run swb assets fetch\|list\|check` | assets into the cache, the lock, links into `assets/` |
| `ros2 run swb seat take <n>\|auto`, `show`, `release`, `env <group>` | seats |
| `play_launch launch swb_launch swb.launch.xml [project:=] [seat:=] ...` | the stack |
| `ros2 run swb run <scenario.xosc\|suite.yaml> [--repeat N] [--shard i/n] [--resume] [--logs L]` | runs and suites against the stack |
| `ros2 run swb runs list\|show\|why\|prune\|summary` | results and logs |

Exit codes (`run`, `project check`, `assets check`): 0 success, 1 a scenario failed or a
check found a problem, 2 an infrastructure error, 3 refused (wrong stack, missing asset,
bad input).

Every command finds its project as `project:=`/`--project`, else the working directory or the
nearest parent with `swb.yaml` (`swb.config.find_project`). `pkg://<package>` names a ROS
package's share directory that installs a `swb.yaml`.

## The project file

```yaml
# swb.yaml
simulator: carla                        # carla | isaac | <a simulator package>
assets:                                 # see "Assets"
  - dataset: carla-town-autoware-maps
    take: [Town01, Town05]
  - path: ../shared/my_map              # a directory of your own, linked as is
    as: my_map
maps: [Town01, Town05]                  # the maps the stack serves; default: every map
                                        # the project's scenarios name
pilots:                                 # entity name (or fnmatch pattern) -> pilot
  ego:     {kind: autoware, vehicle: tesla_model3}
  bg_av_*: {kind: autoware}
  cut_in:  {kind: command, run: [python3, -m, my_policies.cut_in, --policy, "{param:policy}"]}
vehicles: [vehicles/my_car.yaml]        # own cars: specs (sim_vehicle_spec format)
ground: {}                              # Isaac: per map, surface classes (see "Ground")
logging: {capture: standard, keep: {passed: result, failed: full, error: full}}
judges: []                              # post-run programs over result.json
```

Keys not listed are refused by name (`project check`). `simulator`, `pilots` and `assets` are
required; the rest default as shown. `maps` is a list of map names under `assets/maps/`;
give it when runs name maps the project's own scenarios do not (a scenario run by path), or
to fix the order; without it the stack serves the maps its scenarios name.

`project check` (and the stack before it starts):

1. every asset is fetched and matches `swb.lock`;
2. every scenario under `scenarios/` (and every suite's) parses; its `LogicFile` names
   `$(env SWB_MAPS)/<map>` with `<map>` fetched; every map of `maps:` is fetched;
3. every entity whose controller is `agent`/`Autoware` (SSv2's external control) matches a
   pilot; every pilot kind is valid for the simulator (`carla_basic`: CARLA only);
4. every car named by a scenario or pilot is in a catalog the stack loads, and its spec hash
   agrees;
5. simulator limits (Isaac: one ego, no background AV).

## Assets

Datasets are Hugging Face datasets (the default namespace `jerry73204`, `repo:` for another).
`fetch`:

1. resolves each dataset to its commit (`revision:` a tag or commit; default the newest tag),
2. downloads only the `take:` subpaths (globs relative to the dataset root) into the cache,
   `<cache>/datasets/<repo>/<commit>/`, read-only once complete,
3. verifies each file against the dataset's `MANIFEST.sha256` when it has one,
4. writes `swb.lock`,
5. links `assets/` from the cache: maps under `assets/maps/<map>/` (a map directory is one
   holding `lanelet2_map.osm`), the CARLA USD set under `assets/carla-usd/`, cars under
   `assets/vehicles/`.

```yaml
# swb.lock (written by fetch; committed)
version: 1
datasets:
  carla-town-autoware-maps:
    repo: jerry73204/carla-town-autoware-maps
    commit: 3f9c2a1e8b...
    take: [Town01, Town05]
    files: {Town01/lanelet2_map.osm: "sha256:7a41...", ...}
paths:
  my_map: {path: ../shared/my_map}
```

Runs read the lock, never the tag. `check` re-hashes the linked files against it. A dataset
without huggingface_hub installed is fetched over plain HTTPS
(`https://huggingface.co/datasets/<repo>/resolve/<commit>/<file>`).

Nothing writes into the cache. What the stack writes for a map (a CARLA signal table, Isaac
ruts, a project's surface map) goes to `<project>/state/<map>/`, and the stack points the
world server at it.

Inside the data, names not paths: scenarios name maps `$(env SWB_MAPS)/<map>`; an Isaac
`world.yaml`'s `usd:` may use `${SWB_CARLA_USD}`; the stack sets both for every process it
starts (`SWB_MAPS=<project>/assets/maps`, `SWB_CARLA_USD=<project>/assets/carla-usd`).

### Ground

`ground:` names, per map, the project's own Isaac surface files:

```yaml
ground:
  Town01:
    surfaces: ground/town01.yaml      # the surface map (mud, gravel, wet asphalt)
    road_detail: ground/rough.yaml    # optional: rocks and ruts on the road
```

The Isaac world server reads `<state>/<map>/surfaces.yaml` and `road_detail.yaml` over the
fetched map directory's. When a stack starts, swb links each named file there; a link it made
that the project no longer names is removed, and a file a user put there by hand is refused,
never replaced. `project check` reports a named file that is missing.

`ros2 run swb assets prune [--dry-run]` removes the cache's commits that no project's
`swb.lock` names (fetch records each project in `<cache>/projects.json`); the cache is
read-only, so a plain `rm -r` fails.

## Host settings

Read by `swb.config.host_settings`, highest first: arguments; `<project>/.swb/local.yaml`
(git-ignored); `~/.config/swb/host.yaml`; `/etc/swb/host.yaml`; defaults.

```yaml
# ~/.config/swb/host.yaml
seat: 2                                  # or auto
gpu: 0                                   # overrides the seat table's
logs: ~/swb-logs                         # on a fast disk
cache: ~/.cache/swb
carla: {root: ~/CARLA_0.9.16, host: localhost, port: 2000}
isaac: {root: ~/isaac-sim}
autoware: {setup: /opt/autoware/1.5.0/setup.bash}
seats:                                   # the host's table (often in /etc/swb/host.yaml)
  domains: {base: 1, stride: 8}
  ports: {base: 5555, stride: 100}
  web: {base: 8080, stride: 10}
  count: 7
  gpus: [0, 0, 0, 1, 1, 1, 1]
```

The shell's environment file only sources ROS, Autoware and the workspace.

## Seats

Seat `n` of a table:

| Resource | Value |
|---|---|
| scenario domain | `domains.base + domains.stride * n` |
| pilot domains | the next `domains.stride - 1`, in the order pilots start |
| SSv2 port | `ports.base + ports.stride * n`; its sensor-release sockets +1, +2 |
| relay port | SSv2 port + 5 |
| world server port | SSv2 port + 36 |
| web UI | `web.base + web.stride * n` (+0 stack, +1 the run's scenario) |
| GPU | `gpus[n]` (or `gpu:`) |

Every domain must be 0-57 (the localhost Cyclone profile's DomainGain); a table that breaks
that, or overlaps itself, is refused when read. `seat take` takes an exclusive lock file
`<seat_locks>/seat-<n>.lock` (flock, holding user, pid, project, time; a dead holder's lock is
reclaimed) and writes `seat:` to the user's host.yaml; the stack holds the lock while up. `seat
env <group>` prints `export ROS_DOMAIN_ID=.. RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=..` for `scenario` or a pilot entity, for ROS's own tools.

## The stack

`play_launch launch swb_launch swb.launch.xml` (it is `stack.launch.py` with the project's
settings applied):

1. finds the project, reads host settings, takes the seat;
2. runs `project check`; refuses (3) on a problem;
3. sets `SWB_MAPS`, `SWB_CARLA_USD`, the DDS profile and the seat's numbers;
4. starts the simulator side (on CARLA, CARLA itself when `carla.root` is set and nothing
   serves `carla.port`; `simulator_start: external` never starts it), the bridge and the relay
   in the scenario domain;
5. starts pilots: `lifetime: stack` pilots for the stack's map (below), `lifetime: run` ones
   per run; each in its own domain;
6. writes `<logs>/<project name>/stacks/<stack id>/stack.json`: the fingerprint (hashes of
   swb.yaml and swb.lock, the pilots, the workspace's build commits), the seat, the domains
   and ports, the pids, the maps, and `state: ready` once every stack pilot is ready; and
   links `<logs>/<project name>/seat-<n>-current` to it.

### Maps and the keeper

A stack serves several maps: `maps:` of swb.yaml, else every map its scenarios name (each
must be fetched, or the stack is refused). Its `lifetime: stack` pilots run on one of them at
a time, `map` in stack.json; Autoware loads the map's lanelet2 and point cloud at start, so
another map means a pilot restart with another `map_path`.

- **When the pilots start.** On CARLA, for the first run's map (the stack is ready with
  `map: null` and no pilot running), unless `map:=` names one. On Isaac the world server
  loads its world (`world:=`) at start, so the first map is the stack's for its lifetime, and
  a run on another is refused, naming the `map:=` to restart with.
- **Who runs them.** The keeper (`python3 -m swb.stack keep`), not the launch: a launch cannot
  restart a process with other arguments. Each pilot is a nested play_launch of its kind's
  launch (`swb_launch/launch/pilots/`) in its own session, with the stack's contract
  (`map_path` and, on CARLA, the state dir's signal table, of the current map), logging under
  `proc/pilot-<entity>/`, respawned 5 s after it ends, as the launch did (an episode end: a
  world load ends `acb_bridge`, exit 3); it dies with the keeper (`PR_SET_PDEATHSIG`), and
  the keeper stops every pilot on its way out (SIGINT, SIGTERM, SIGKILL to its group).
- **The switch.** A runner whose scenario is on another map writes `switch-request.json`
  (`{id, map}`, atomically) into the stack's directory. The keeper polls it every 0.1 s; on a
  new id it records `switch: {id, map, state: switching}`, stops the pilots, sets `map`,
  `map_path` and the pilots' contract in stack.json, starts them again and answers
  `state: done` (or `failed` with `error`). The runner waits for that answer (180 s), then for
  its pilots' readiness as every run does; the world server loads the town itself on SSv2's
  Initialize (which ends the CARLA vehicle stacks once more; the keeper respawns them, and
  SSv2 waits for the ego's agent). A file, not a socket: the keeper already polls, a request
  survives a slow keeper, and stack.json stays the one record a run reads.
- **What is refused (3)**: a map that is not fetched, one not among the stack's maps, another
  map on Isaac. A switch the keeper could not do, or did not answer, is the run's `error`.

A run on the stack's current map restarts nothing.

## Runs

`ros2 run swb run <x>`:

1. finds the project and seat; reads `seat-<n>-current/stack.json`; refuses (3) when there is
   none, when its fingerprint differs from the project's (naming what differs), or when it is
   not ready;
2. expands `<x>` into run specs (a scenario, or a suite: below), each with an id
   `<time>_<scenario>[_<params>]_r<k>`;
3. per run: moves the stack's pilots to the scenario's map when they are on another (above),
   creates `runs/<id>/`, writes `resolved.yaml`, starts `lifetime: run` pilots and
   waits for every pilot's readiness, notes each stack log's size (the slice start), starts
   SSv2 (`scenario.launch.xml` as a nested play_launch in the scenario domain, with the
   project's `SWB_MAPS`), waits for it to end, notes the sizes again, cuts the slices, builds
   the timeline, writes `result.json`, applies `keep`, prints one line;
4. a suite writes `summary.csv` and a merged JUnit; `--resume` skips run ids with a
   `result.json`; `--shard i/n` takes every n-th spec from the i-th; then the runs are ordered
   by map (the stack's current map first, then by first appearance, stable within a map), so
   a suite over five towns switches at most five times. Sharding comes first, so that every
   host takes the same share.

`swb.api.run_one(project, scenario, overrides) -> RunResult` does step 3 from Python.

### Suites

```yaml
# suites/c3.yaml
scenarios: [scenarios/cut_in.xosc, scenarios/awf/]   # files, directories
parameters:                                          # a grid over ParameterDeclarations
  gap_m: [6, 10, 14]
  policy: [yield, push]
distribution: scenarios/cut_in_dist.xosc             # or SSv2's ParameterValueDistribution
repetitions: 5
seed: 1000                                           # run k gets seed 1000 + k
expect: {awf/known_issue.xosc: failed}
retry: {on: error, times: 2}
pilot_args: {cut_in: {policy: "{param:policy}"}}
logging: {capture: full, keep: {passed: result}}
```

Pilots receive per-run values by substitution in their `args`/`run`: `{run_dir}`, `{seed}`,
`{run_id}`, `{param:<name>}`.

## Pilots

A pilot = **vehicle side** + **driver** (+ the agent registering the entity with the relay).

| Simulator | Vehicle side (swb starts) |
|---|---|
| carla | acb: `acb_bridge.launch.xml` (sensors, vehicle interface, physics check) in the pilot's domain |
| isaac | the car's rig inside the world server (aib), its topics in the pilot's domain |

The driver sees Autoware's vehicle interface (`/control/command/*` in, `/vehicle/status/*`
out) and the sensor topics in its domain, whatever the simulator.

| kind | Driver | Agent |
|---|---|---|
| `autoware` | `autoware_sim_profile autoware.launch.xml` (or `autoware_launch:`) | swb's vehicle agent |
| `carla_basic` | CARLA's BasicAgent; no vehicle side | `swb_agent carla_basic` |
| `simple` | `simple_pilot` (pure pursuit, speed PID) | swb's vehicle agent |
| `launch` | `launch: <pkg> <file>` | swb's vehicle agent unless `agent: own` |
| `command` | `run: [argv...]`, given `--relay --entity --seed --run-dir` | the program itself |
| `none` | started by the user | the user's |

Common keys: `vehicle`, `args` (launch arguments; substitutions apply), `lifetime: stack|run`
(default `stack`), `ready` (default: the kind's check; `autoware`: `ego_ready`), `inputs`
(`sensors`, `map`, `ground_truth`; recorded in results), `rviz` (bool).

The launch contract every `launch` pilot receives: `entity`, `relay`, `map_path`, `vehicle`,
`seed`, `run_dir`, `sim`, `use_sim_time:=true`.

## Results

`runs/<id>/result.json`, schema `swb.result/1`:

```json
{
  "schema": "swb.result/1",
  "run_id": "2026-11-14T10-21-07_cut_in_gap6_yield_r2",
  "verdict": "passed | failed | error",
  "reason": "first cause, one line",
  "scenario": {"file": "...", "sha256": "...", "parameters": {}, "repetition": 2, "seed": 1002},
  "ssv2": {"junit": "junit.xml", "failures": 0, "errors": 0, "message": "..."},
  "events": [{"t_sim": 18.4, "t_wall": "...", "type": "collision.vehicle", "entity": "ego", "other": "cut_in"}],
  "metrics": {"duration_s": 24.0, "route_completion": 0.62},
  "pilots": {"ego": {"kind": "autoware", "vehicle": "tesla_model3", "inputs": ["sensors"], "version": "..."}},
  "provenance": {"swb": "<commit>", "submodules": {}, "simulator": "carla 0.9.16",
                 "swb_lock": "sha256:...", "spec_hash": {"ego": "..."}, "host": "...", "gpu": "...",
                 "stack_id": "..."}
}
```

Event types (fixed names): `collision.vehicle`, `collision.pedestrian`, `collision.static`,
`red_light`, `off_lane`, `stuck`, `route_complete`, `pilot.restart`, `stack.error`. The verdict
is `error` when the run cannot be judged for the stack's fault (a world server or bridge
error, a pilot that never became ready, SSv2 ending on an infrastructure error), `failed` when
SSv2 failed the scenario, `passed` otherwise.

**Judges** are versioned programs over a finished run, listed in `swb.yaml`:

```yaml
judges:
  - name: comfort                     # writes judges/comfort.json
    run: [python3, -m, my_judges.comfort]
    version: "1"
    on: [passed, failed]              # the default; error runs are not judged
    timeout: 120                      # s
```

A judge gets the run directory as its last argument (and `SWB_RUN_DIR`), runs in the project
directory and prints one JSON object. swb adds `name`, `version`, `exit_code`, `duration_s`
and a `verdict`: the object's own `verdict` if it has one, else `passed` for exit 0 and
`failed` for exit 1; any other exit, no JSON object or a timeout is `error`, with the reason
in `message`. A judge's verdict is recorded beside the run's (the run line, `summary.csv`'s
`judges` column, a `judge` event in the timeline), never in place of it: the run's verdict
stays SSv2's, and the scoring stays with the judges.

## Logs

```
<logs>/<project name>/
  stacks/<stack id>/stack.json  events.jsonl  proc/<group>/<process>.log
  runs/<run id>/   result.json resolved.yaml junit.xml timeline.jsonl
                   logs/<group>/<process>.log   record/   host.json
```

`<project>/runs` links to `<logs>/<project name>/runs`. Groups: `simulator`, `bridge`,
`relay`, `pilot-<entity>`, `scenario`, `judge`.

A **slice** is the byte range of a stack process's log between the run's start and end.
**timeline.jsonl**: one JSON object per line, `{t_wall, t_sim, group, event, ...}`, ordered by
wall time: run start/end, pilot ready, SSv2 milestones, judge events, and every log line at
WARN or above in any slice.

Levels: `result` (result, resolved, JUnit, timeline), `standard` (+ slices, SSv2's logs),
`full` (+ a rosbag of the key topics, `host.json` samples), `debug` (+ every topic, DEBUG on
`logging.debug:` groups). `capture` records; `keep` prunes per verdict after it is known;
At `full` a `ros2 bag record` per pilot domain (pilots sharing a domain share one) records
the key topics (ego state, control command, vehicle status, detected objects, trajectory,
`/swb/wheels`; `logging.topics` replaces the list) from the moment every pilot is ready until
SSv2 ends, into `record/<entity>/`; at `debug` every topic, the scenario domain's too
(`record/scenario/`). `ros2 run swb run --capture <level>` records one run at that level and
keeps it whatever the verdict. `budget` (`{per_run: 2GB, total: 200GB}`) refuses a run or suite that cannot fit: `per_run`
times the runs to go must be free on the log disk and, with `total`, fit beside what the runs
directory already holds. `runs why <id>` prints the first WARN/ERROR with its
group.

## Examples

The working examples live in `src/swb_examples/projects/`: `first_drive` (CARLA, one Autoware
ego), `multi_pilot` (Autoware, `carla_basic`, a `command` agent), `safety_suite` (AWF scenarios
across towns), `isaac_first_drive`, `road_physics` (Isaac surfaces), and the golf cart twin as
a package project in its own repository.
