# Phase 076: One Entry Launch, Assets by Name, and Docs by Reader

A user names what they want (a simulator, a world, a scenario, an autopilot) and swb fetches
the data, finds it and runs it. No absolute paths to edit, no symlinks to make, no repository
scripts, no internal knowledge. The docs are organised by what the reader is doing, each fact
lives in one place, and every command in them has run.

**Status**: in progress, 2026-11-14: design settled (A, and the design doc); building steps 3-7.
Old paths retire in the last step (user, 2026-11-14).

**Source**: user, 2026-11-14: "I would fix the gaps on swb and assets sides for the best of user
experience. That is, the swb will have proper interface with work with these assets without
internal knowledge. Otoh, review all tutorials and docs. Have a proper organization. Avoid dup
content. I would serve for various readers: adaptors for their project, newbies, educators,
etc." Then: "Docs are markdown on GitHub. Use ros2 commands. I suggest to have a entry launch
file. I don't have specific course. You can explore the tutorial formulation".

**Follows**: 075 (the docs on the stack, the datasets published) and its "Gaps found writing
tutorials 6 and 7". 070's code items (per-version CARLA launcher, `simple_pilot`, releases)
stay in 070.

## Decisions (user, 2026-11-14)

1. Docs stay markdown on GitHub.
2. Every user command is a ROS one (`ros2 run`, `ros2 launch` / `play_launch launch`, colcon).
   No separate CLI.
3. One entry launch file.
4. No specific course: the tutorials are designed for self-study and reuse in any class.
5. The datasets stay under `jerry73204` on Hugging Face.
6. The entry launch is `swb.launch.xml`.
7. **A project directory, not a global data directory** (user: "I prefer to have a organized
   working dir placing required assets. In this way, it makes it easier for users to manage
   assets for a certain project and sharing. We can change the work dir so we can quickly
   swap to a new scenario/simulator/actor combinations").
8. **One interface for every autopilot, several kinds in one run** (user: "the autopilot can be
   Autoware or something else. Have a unified interface. It should also support multi-agent
   running different autopilot kinds"): pilots, A5.
9. **CARLA first**: the tutorials start on CARLA; Isaac Sim comes in later chapters.
10. **The second design is accepted** (user, 2026-11-14): "ok to have a long-lived stack ... your
    build is ok". Seats are kept in config files, not environment exports.
11. **The seat table is the user's**: "the seat table should be configuable on users. the actual
    assignment vary on hosts". swb ships a default table; `host.yaml` replaces it per host
    (A6).
12. **Build the new interface and working examples first; retire the old paths in the last
    step** (user, 2026-11-14).
13. Ruling (mine, 2026-11-14, cheap to change before step 3 lands): the commands live in a new
    ament_python package **`swb`**, so they read `ros2 run swb project|assets|seat|run|runs`;
    the entry launch stays `swb_launch swb.launch.xml`. Why: one short name for every user
    command, and a Python package for the libraries they share (config, seats, results), which
    swb_launch (ament_cmake, launch files and scripts) is not.

## Use-case study (2026-11-14)

Before anything retires, the first design (a project file and one entry launch per run) was
walked through real work and outside practice. Reports in [076-study/](076-study/):

- [usecases-internal.md](076-study/usecases-internal.md): the AWF safety suites and gap
  pipeline, the Haruna drift campaign, the cooperation study's batches, the golf cart twin in
  its own repository, two stacks on one host.
- [personas.md](076-study/personas.md): a newcomer, an adopter with its own autopilot, an
  educator with 20 seats, a researcher running sweeps, an Isaac road-physics user, a scenario
  author; plus the design's internal consistency.
- [related-work.md](076-study/related-work.md): CARLA Leaderboard and ScenarioRunner,
  Autoware Evaluator and driving_log_replayer, esmini and OpenSCENARIO's parameter
  distributions, Foretify, RoadRunner Scenario, nuPlan, MetaDrive, Waymax, UN R157, Euro NCAP,
  ISO 34502, Safety Pool; what a third-party evaluation platform needs.

What all three found, and what changes:

| Finding | Seen in | Change |
|---|---|---|
| Every run restarted the stack (CARLA 1-3 min, Autoware up to 11 min, Isaac about 7 min); every study kept its stack up and drove many runs from its own script | all studies; researcher, scenario author | the **stack** is long-lived; **runs** attach to it (A3, A4) |
| Map, cars and who drives were named twice (the scenario and the project file), so mismatches fail late; one study's suite spans five towns | AWF suites; scenario author | the **scenario owns** map, entities, cars and controllers; the project binds pilots to entity names (A2) |
| A non-Autoware autopilot still needed swb's insides (acb's sensors and vehicle interface live in the Autoware launch) | adopter; drift campaign | a pilot is a **vehicle side** swb starts plus a **driver** (A5) |
| Domains in a committed project file collide when a project is copied; ports never moved | educator; two stacks per host | host settings become a **seat** (A6) |
| Signal tables, ruts and surface maps were written into shared cached assets | Isaac user | a **read-only cache**, a lock file, project state apart (A7) |
| Nothing for batches; each study wrote its own driver (`c3_run.sh`, sweeps, the gap search) | cooperation, drift, gap search; researcher | **suites** and a Python **run primitive** (A4) |
| Verdicts mix the autopilot's failures with the stack's; no provenance; no machine-readable result | cooperation retries; evaluation platforms | a versioned **result** per run with provenance (A8) |
| `$(env PWD)` goes stale; play_launch writes `play_log/` and binds :8080 before the launch file is read | tested with play_launch 0.20 and ros2 launch | resolve the project with `os.getcwd()`; logs and web port from the seat (A3, A6) |

## A. The interface (second design, after the study)

### A1. Words

| Word | Means |
|---|---|
| **project** | a directory (often a git repository or a ROS package) holding one use case: scenarios, suites, pilot bindings, own cars, the assets it needs |
| **stack** | the long-lived processes of a project on one seat: the simulator's world server, the bridge, the relay, the pilots |
| **run** | one scenario (one parameter set, one repetition) against a live stack; it ends with a result |
| **suite** | many runs: a list, a directory, an OpenSCENARIO parameter distribution, repetitions |
| **pilot** | what drives one scenario entity: a vehicle side plus a driver |
| **seat** | one stack's share of a host: its domains, ports, web port, GPU, log root |

### A2. The project, and who owns what

```
my_project/
  swb.yaml        # the project file
  swb.lock        # written by `assets fetch`: resolved dataset commits, file checksums
  scenarios/      # .xosc files (own map, entities, cars, controllers)
  suites/         # suite files (A4)
  vehicles/       # own cars: specs, catalogs (optional)
  pilots/         # own pilot launches or programs (optional)
  assets/         # links into the read-only cache (A7); not committed
  state/          # what the stack writes per project: signal tables, ruts; not committed
  runs/           # results (A8); not committed (or a link to the seat's runs root)
```

Ownership, so nothing is said twice:

| Fact | Owner |
|---|---|
| map, entities, their cars, which entities an external pilot drives (`Controller` named `agent`) | the scenario |
| which pilot drives an entity name (`ego`, `bg_av_1`, `cut_in`) | the project file, `pilots:` |
| which assets exist and their versions | the project file, `assets:`; pinned in `swb.lock` |
| the simulator | the project file (`simulator:`), overridable per stack |
| domains, ports, GPU, log root | the host: the seat (A6) |

```yaml
# swb.yaml
simulator: carla
assets:
  - carla-town-autoware-maps: [Town01, Town05]   # Hugging Face datasets by name
  - path: ../shared/my_map                       # or a directory of your own
pilots:
  ego:     {kind: autoware, vehicle: tesla_model3}
  bg_av_*: {kind: autoware}                      # entity names may use a pattern
  cut_in:  {kind: command, run: [python3, -m, my_policies.cut_in]}
vehicles: [vehicles/my_car.yaml]                 # own cars: catalog entries and specs
```

`ros2 run swb project new <dir> [--template carla-first-drive|isaac|multi-pilot]`
writes a starter. `project check` cross-checks every scenario against the project before
anything starts: its map is fetched, every `agent`-controlled entity has a pilot, each car's
spec matches, the simulator supports what the scenario uses (Isaac: one ego). A project can be
a ROS package (`project:=pkg://golfcart_twin`), so an external repository ships one.

### A3. The stack

```bash
cd my_project
play_launch launch swb_launch swb.launch.xml           # the stack: stays up
```

- Starts the world server (and CARLA itself, `simulator_start: auto`, or uses one running,
  `external`), the bridge, the relay, and the pilots the project binds.
- The project is the launch's working directory (`os.getcwd()`, not `$(env PWD)`), or
  `project:=<dir|pkg://...>`.
- Pilots start lazily, for the entities the coming run has, and restart on a map change (a
  world load ends a vehicle side by design).
- It records a **fingerprint** (the project file and lock, the pilots, the build's commits);
  runs refuse a stack whose fingerprint differs from their project's, saying what differs.
- `end_with:=run` makes it one-shot (scripts, CI): start, one run or suite, stop.

### A4. Runs, suites, and the primitive under them

```bash
ros2 run swb run scenarios/cut_in.xosc                    # one run
ros2 run swb run scenarios/cut_in.xosc --repeat 5
ros2 run swb run suites/nightly.yaml --shard 1/3 --resume
```

- `run` attaches to this project's stack on this seat, waits for each pilot's readiness, runs
  SSv2, writes the result (A8), and prints one line: verdict, run directory.
- A suite file lists scenarios or directories, OpenSCENARIO `ParameterValueDistribution` files
  (SSv2 expands them; swb adds no sweep syntax), `repetitions`, seeds, `expect` per scenario,
  retries for infrastructure errors only, and per-run pilot parameters.
- Pilots get per-run values by substitution: `{run_dir}`, `{seed}`, `{param:<name>}`.
- `swb_launch.api.run_one(project, scenario, overrides) -> RunResult` is the same in Python,
  for ask-and-tell searches (the gap search, the cooperation study's oracle).
- Exit codes: 0 all passed, 1 a scenario failed, 2 an infrastructure error, 3 refused.

### A5. Pilots: a vehicle side plus a driver

| Part | Who starts it | What it is |
|---|---|---|
| vehicle side | swb, per simulator | CARLA: acb (sensors, vehicle interface, the car's physics check) in the pilot's domain. Isaac: the car's rig inside the world server |
| driver | the pilot's kind | Autoware, CARLA's BasicAgent, `simple_pilot`, or yours |
| agent | swb or the driver | the relay protocol for that entity; swb's vehicle agent for ROS autopilots on Autoware's interface |

The driver sees Autoware's vehicle interface and sensor topics in its own domain, whatever the
simulator; a non-Autoware autopilot needs nothing from swb's insides.

| `kind` | Driver |
|---|---|
| `autoware` | Autoware 1.5 with swb's profile (or `autoware_launch:` of your own) |
| `carla_basic` | CARLA's BasicAgent (no vehicle side; CARLA only) |
| `simple` | pure pursuit and a speed PID on the lanelet map |
| `launch` | your launch: given `entity`, `map_path`, `vehicle`, `seed`, `run_dir`, the relay |
| `command` | your program (an SDK agent): `--relay --entity --seed --run-dir` |
| `container` | an image by digest, the same contract (later) |
| `none` | started by you |

Each pilot declares `lifetime: stack|run` (the cooperation study needed a fresh Autoware per
run), its `ready` check, and its inputs (`sensors`, `map`, `ground_truth`), recorded in the
result so different autopilots stay comparable.

### A6. Seats: host settings in one number, kept in a config file

A **seat** is one stack's share of a host: the scenario domain, one domain per pilot, SSv2's
port and its +1/+2, the relay's, the world server's, the web port, the GPU and the log root,
all derived from one number by a table. swb ships a default table; a host's `host.yaml`
(or the host-wide `/etc/swb/host.yaml` an administrator writes) replaces it, since what is free
differs per host:

```yaml
seats:
  domains: {base: 1, stride: 8}      # seat n: scenario 1+8n, pilots 2+8n .. 7+8n
  ports: {base: 5555, stride: 100}   # SSv2, +1, +2, relay +5, world server +36
  web: {base: 8080, stride: 10}
  count: 7
  gpus: [0, 0, 0, 1, 1, 1, 1]        # seat -> GPU
```
 A lock per seat names its holder; `seat: auto`
takes the first free one.

The seat and the other host settings live in **config files, not environment exports**
(user, 2026-11-14: "I prefer to turn it to a config instead of env exports. Users may forget
to export env in new terminals"). Every swb command reads them, so a new terminal needs
nothing exported:

| File | Scope | Holds |
|---|---|---|
| `~/.config/swb/host.yaml` | this user on this host | `seat`, `gpu`, CARLA's install and address, Isaac's root, Autoware's setup, the TensorRT runtime, the log root |
| `<project>/.swb/local.yaml` (git-ignored) | this checkout on this host | overrides, e.g. a second checkout's own seat |
| launch arguments | one stack or run | `seat:=`, `gpu:=`, ... |

Precedence: argument, then the checkout's file, then the user's file, then the defaults.
`ros2 run swb seat take <n>|auto` writes the user's file and takes the lock;
`seat show` lists every seat on the host and its holder; `seat release` frees it. The locks
live in a host-wide directory every user can write (`/tmp/swb-seats/`).

ROS's own tools still read `ROS_DOMAIN_ID` from the environment. For them:
`ros2 run swb seat env <pilot|scenario>` prints the three exports (domain, RMW, DDS
profile) for one group, for `eval` in a terminal that wants to look in; the docs show it once,
on the domains page.

The environment file shrinks to what only the shell can do: sourcing ROS, Autoware and the
workspace.

### A7. Assets: read-only cache, lock, project state

- `ros2 run swb assets fetch|list|check` reads `swb.yaml`, writes `swb.lock` (each
  dataset's resolved Hugging Face commit and every file's checksum), and links the files from
  a read-only cache (`~/.cache/swb/`) into `assets/`. Runs read the lock, not the tag.
- `assets:` takes Hugging Face datasets, `path:` directories and `generate:` recipes (the
  Haruna course from its YAML, a town's Isaac world).
- What the stack writes for a map (signal tables, ruts, surface maps) goes to `state/`, never
  into the cache. Surfaces are a project key (`ground:`), not an edit of a shared map.
- Names, not paths, inside the data (unchanged from the first design): `usd:` expands the
  project's CARLA set; scenarios name maps `$(env SWB_MAPS)/<map>`.

### A8. Results

Each run writes `runs/<id>/`:

- `resolved.yaml`: everything the run used, after overrides and substitutions.
- `result.json`, versioned schema: verdict (`passed`, `failed`, `error` for the stack, never
  blamed on the autopilot), SSv2's outcome, fixed-name events and metrics (collision kinds, red
  light, off lane, stuck, route completion, time, distance), and provenance (submodule pins
  and binary hashes, the simulator's version, `swb.lock`, each car's `spec_hash`, each pilot's
  version or image digest, host and GPU).
- SSv2's JUnit, the logs, a recording when asked.
- `runs/latest` and a suite's `summary.csv` / merged JUnit. Scoring stays outside: judges are
  named, versioned programs over `result.json` (`judges:` in the project), and a third-party
  evaluator reads the same files.

Runs are not bit-identical (Autoware and the simulators' GPU physics are not deterministic):
swb promises the same inputs, records them, and a suite's repetitions measure the spread.

### A8b. Logs per run

User, 2026-11-14: "Think about logging per run as well. User may control the logging verbosity
in the case they want to save the logs from many repeated runs. Also, the proper logging
structure helps diagnosing the issues."

The stack outlives its runs, so its processes (world server, bridge, relay, pilots) log
continuously into the stack's folder, and each run gets a **slice**: the window between the
run's start and end markers, cut from every process's log, beside the run's own processes
(SSv2, a per-run pilot). Logs go under the log folder of `host.yaml` (a fast disk: a busy
spinning disk stalls nodes that log from real-time loops), linked as the project's `runs/`.

```
<logs>/<project>/
  stacks/<stack id>/          # one per stack start
    stack.json                # fingerprint: project, lock, pilots, build, seat, host
    events.jsonl              # the stack's own events (starts, restarts, map loads)
    proc/<group>/<process>.log
  runs/<run id>/
    result.json  resolved.yaml  junit.xml
    timeline.jsonl            # one ordered event stream for the run (below)
    logs/<group>/<process>.log   # this run's slice, or SSv2's own logs
    record/                   # a rosbag, when the level asks
    host.json                 # load, GPU memory, disk at start and end
```

**The timeline** is the diagnosis tool: one JSON line per event, with wall time and simulation
time, from every group, in order: run start, pilot ready, scenario initialised, engage, goal
sent, each SSv2 condition, each judge event (collision, red light, off lane, stuck), every
WARN or ERROR any process logged in the window (with its group and process), restarts, run
end. Groups: `simulator`, `bridge`, `relay`, `pilot:<entity>`, `scenario`, `judge`.

**Levels**, set in `swb.yaml` (`logging:`), per suite, or per run (`--logs <level>`):

| Level | Keeps | About per run |
|---|---|---|
| `result` | `result.json`, `resolved.yaml`, JUnit, the timeline | kilobytes |
| `standard` (default) | + each process's slice and SSv2's logs | megabytes |
| `full` | + a rosbag of the key topics (ego state, control, detected objects, `/swb/wheels`), `host.json` samples | hundreds of megabytes |
| `debug` | + every topic, DEBUG level on the groups named in `debug:` | gigabytes |

**Keep by verdict.** A run is captured at the highest level any outcome asks for, and pruned
when its verdict is known, so a failure is never missing the logs that explain it:

```yaml
logging:
  capture: full                 # what is recorded while the run goes
  keep: {passed: result, failed: full, error: full}
  levels: {pilot:ego: warn, bridge: info}    # ROS log levels per group
  budget: {per_run: 2GB, total: 200GB}       # refuse to start a suite that cannot fit
```

`ros2 run swb runs list|show|why|prune`:
- `show <run>`: the timeline, readable;
- `why <run>`: the first WARN/ERROR and the group it came from, which is also how the verdict
  tells `error` (the stack's fault) from `failed`;
- `prune --keep-last 20 --verdict passed`: retention by count, age or verdict.

The suite's `summary.csv` names each run's folder, so a table row leads to its logs.

### A9. Fewer manual steps, telemetry, datasets

Kept from the first design: the signal table written on first load (into `state/`),
`make_world`, the surfaces map with a lanelet map, the rough road in the package, `/swb/wheels`,
`record:=`, the datasets re-uploaded with relative paths and tagged.

## B. The docs

### B1. Four kinds of page

| Kind | For | Rule |
|---|---|---|
| Tutorials | learning by doing | one path, run as written, expected output at each step |
| How-to guides | a task in your project | starts from a working stack, ends at your goal |
| Reference | looking up | complete, generated where possible (launch arguments) |
| Explanation | understanding | why, not steps |

### B2. Entry by reader (README and `docs/README.md`)

- **New to swb**: README, then the getting-started path, then the tutorials in order.
- **Using swb in your project**: the how-to guides (your car, your autopilot, your simulator,
  more cars, CI suites, several stacks on one host) and the reference.
- **Teaching**: a page for educators: what a seat needs, many seats on one host (domains and
  ports per seat), the tutorials as a lab sequence with exercises and their checks, grading
  with `run_suite`'s JUnit.
- **Developing swb**: `docs/development.md`, `docs/design/`.

### B3. One source per fact

The install, the build, the environment, the domain table and each launch's arguments live
in one page each; every other page links there. Tutorials do not repeat setup: each starts
with "Before you start" linking the page that gets you there.

### B4. The tutorial format

Each tutorial:

1. **Goal**, one sentence, and a picture of the end state.
2. **You will learn**: three to five outcomes, as a reader can check them.
3. **Before you start**: links to the pages that get you there; time; hardware (GPU memory).
4. **Steps**, each: the command, what you see (the line to look for), and **Check**: a
   command whose output proves the step worked.
5. **What happened**: a short explanation, linking the explanation pages.
6. **Exercises**: two or three, graded easy to open-ended, each with a check (a JUnit
   verdict, a topic value, a file), so a teacher can use them as labs.
7. **If it does not work**: the two or three failures seen in testing, with their fix.
8. **Next**.

### B5. The tutorial sequence

CARLA first; Isaac Sim from tutorial 6. Each tutorial works in a project directory.

| # | Tutorial | Simulator | Builds on |
|---|---|---|---|
| 1 | First drive: a starter project, `assets fetch`, the stack, one run, its result | CARLA | getting started |
| 2 | Write a scenario: poses, the ego's goal, a pass condition | CARLA | 1 |
| 3 | Other road users: pedestrians, scripted cars | CARLA | 2 |
| 4 | Several pilots: Autoware, CARLA's BasicAgent and your own agent in one run | CARLA | 3 |
| 5 | Suites: a parameter distribution, repetitions, the results table | CARLA | 2 |
| 6 | The same project on Isaac Sim, on CARLA's assets in USD | Isaac | 1 |
| 7 | Road physics: surfaces, mud, a rough road, watching the wheels | Isaac | 6 |

How-to guides: your car; your autopilot as a pilot; your Autoware; your simulator; several
stacks on one host; recording and demos; configuration (bridge, world server, weather);
sharing a project.

### B6. Checks

- `just test` gains a docs check: no `just` in user docs, code lines at most 88 columns, no
  banned characters, no broken relative links.
- Every command in tutorials and how-to guides runs once on a quiet host before it ships
  (CARLA and Isaac), and three fresh readers (a newcomer, an adopter, an educator), each with
  only the docs, reach their goal.

## Steps

1. [x] First design; use-case study (076-study/); second design (A), 2026-11-14.
2. [x] The user's word on the second design (decisions 10-13), 2026-11-14. Design doc:
   [projects-and-pilots.md](../design/projects-and-pilots.md); package `src/swb` scaffold and
   `swb.config` (8cb4025).
3. [ ] A2, A7: project file and lock, `project new|check`, `assets fetch|list|check` with the
   read-only cache and `state/`; tests.
4. [ ] A6: seats (the table, the lock, `seat show`); the stack's processes take their numbers.
5. [ ] A5: the vehicle side split from the driver; the pilot contract; kinds `autoware`,
   `carla_basic`, `launch`, `command`, `none` (`simple` with 070 D, `container` later).
6. [ ] A3, A4: `swb.launch.xml` as the stack (fingerprint, lazy pilots, `end_with:=run`);
   `ros2 run swb run` for runs and suites; `run_one`.
7. [ ] A8, A8b: `result.json` schema with provenance, `resolved.yaml`, summaries, judges; per-run
   log slices, the timeline, levels, keep by verdict, budgets, `runs show|why|prune`.
8. [ ] A9: signal table on first load, `make_world`, the surfaces map, the rough road in the
   package, `/swb/wheels`, `record:=`, the datasets re-uploaded and tagged.
9. [ ] Port the studies onto it as the proof: an AWF suite, a cooperation batch sharded on two
   hosts, a Haruna drift run with its own pilot, the golf cart twin as a package project.
10. [ ] Retire what the design replaces (`ego_launch`, `background_av.launch.xml` on the user
   path, `CARLA_MAPS`/`CARLA_USD` for users), by name.
11. [ ] B: the docs reorganised (B1-B5).
12. [ ] B6: the docs check; live runs; fresh readers.

## Progress and rulings

- Step 4 (seats): 1bec606. Step 7 library (results, slices, timeline, `runs`): 3ea15a9.
- Step 5 (pilots): `swb_launch/launch/pilots/` (vehicle side, autoware, carla_basic, launch,
  command) and `stack.launch.py pilots:=`, 5e23d1c-equivalent on main; static checks only (no
  simulator started). Rulings on its open questions:
  - **Isaac vehicle side**: the rig lives in swb_isaac_world and publishes on the ego domain
    only, so on Isaac one pilot with a vehicle side, on the ego domain (Isaac carries one ego
    today). Per-entity rigs are future work, recorded here, not built in 076.
  - **CARLA role name**: `hero` for entity `ego`, else the entity name, overridable
    (`vehicle_name`). Kept; the runner checks it against the scenario's ego when it can.
  - **Agent lifetime**: the vehicle agent lives in the autoware pilot's launch, so an episode
    end restarts it with Autoware. Accepted: it registers again in seconds.
  - `autoware_launch` swaps the profile's Autoware launch; replacing the whole driver is the
    `launch` kind.
- Step 6 (the stack entry and the runner), first part: `swb_launch swb.launch.xml` (a thin
  wrapper of `swb.launch.py`, its logic in `swb.stack`), `ros2 run swb run`, `swb.api.run_one`;
  tests without a simulator (a real keeper process, a stub SSv2 writing a JUnit). Not yet:
  lazy pilots and their restart on a map change, `end_with:=run`, `simulator_start` (CARLA
  is external), rosbag capture. Decisions:
  - **The keeper holds the seat.** play_launch may parse a launch file in a process that does
    not live as long as the stack, so the launch refuses a taken seat at parse and a required
    `python3 -m swb.stack keep` process takes the flock, writes its pid into `stack.json`,
    links `seat-<n>-current`, watches stack pilots' readiness and marks the stack
    `stopped` when it ends. A run trusts a `stack.json` only while its keeper holds the seat.
  - **Every stack process logs under `proc/`.** `stack.launch.py nest_side:=true` runs the
    world server and the simulation side as nested play_launches (`proc/simulator/`,
    `proc/bridge/`, bridge and relay together), as the pilots already were
    (`proc/pilot-<entity>/`). The web UI and log directory of the user's own play_launch are
    its command line's (`--web-addr` the seat's web port, printed at start).
  - **Seat numbers are arguments.** `stack.launch.py` gained `world_port`, `carla_host`,
    `carla_port`; swb.launch.py passes every domain and port, never the environment.
  - **One map per stack** (superseded by the next step's map switch, below). The stack's pilots
    loaded the first map the project's scenarios name; a run on another map was refused.
  - **Parameters reach SSv2 as `override_parameters`** (its preprocessor writes them into the
    scenario's ParameterDeclarations; `scenario.launch.xml` passes it on). A
    `distribution:` file goes to SSv2 as it is, one run per repetition; this SSv2's
    preprocessor does not expand ParameterValueDistribution yet (a TODO upstream).
  - **Run ids** `<time>_<scenario>[_<name>-<value>...]_r<k>`, k from 0, seed `seed + k`.
    A suite's runs are linked by their id without the time from
    `<logs>/<project>/suites/<suite>[_shard-i-of-n]/`, which holds `summary.csv` and the merged
    JUnit; `--resume` skips the links with a result. `--shard i/n` is 1-based.
  - A pilot with `{param:...}` must be `lifetime: run`; the stack refuses it otherwise.
- Step 7 remainder (judges, budget), 2026-11-15: `swb.judges` and `swb.budget`. Ruling: **a
  judge's verdict sits beside the run's, never replaces it** (the run's verdict is SSv2's;
  "scoring stays outside"); a certification gate reads `judges/` or `summary.csv`'s `judges`
  column. Retries are not budgeted (they replace a run that `keep` prunes).
- Step 6, map changes (C3, use case 1): a stack serves several maps (`maps:` in swb.yaml, new,
  else the scenarios' maps). Decisions:
  - **The keeper runs the stack pilots**, not stack.launch.py (a launch cannot restart a
    process with other arguments): nested play_launches it respawns 5 s after they end, as
    the launch did.
  - **Lazy on CARLA**: the pilots start for the first run's map unless `map:=`; on Isaac the
    world is the stack's (`world:=` at start), so another map there is refused.
  - **The control channel is a request file**, `switch-request.json` in the stack directory,
    answered in stack.json's `switch`; the run then waits for its pilots' readiness as every
    run does. A map not fetched, or not among the stack's, is refused (3); a switch that fails
    is the run's `error`.
  - **Suites are ordered by map** after sharding: the stack's map first, then by first
    appearance, stable within a map.
  - Not yet: loading the new town before the pilots restart (the world server loads it on
    SSv2's Initialize, so CARLA's vehicle stacks start once on the old town and once more on
    the new).
- Step 9 preparation (2026-11-15): the working examples in `src/swb_examples/projects/`
  (`first_drive`, `multi_pilot`, `safety_suite`, `isaac_first_drive`, `road_physics`),
  installed to swb_examples' share and copied with `project new <dir> --from
  pkg://swb_examples/projects/<name>` (`find_project` takes `pkg://<package>/<dir>`).
  swb_agent's command line takes `--seed` and `--run-dir` (`Agent.seed`, `Agent.run_dir`),
  which every `command` pilot is given. Each example fetches from the real datasets (the
  USD set cut to two files) and `project check`s clean; a test does the same against the
  fake Hub. None has run on a live stack. Rulings:
  - The examples ship without `swb.lock` (a lock in a read-only share pins nothing for the
    user); the copy's first fetch writes it, and the copy commits it.
  - `ground:` is per map, `{<map>: {surfaces: <file>, road_detail: <file>}}`, paths in the
    project. The stack links them into `state/<map>/` at start (`swb.ground`), where the
    Isaac world server reads them over the map directory's (`state_dir`, de38b28).
  - `judges:` entries are `{name, run}`; `swb run` runs them since 48475ce.
  - Found: the published `carla-town-autoware-maps` `Town01/isaac/world.yaml` names its
    stage `/mnt/wd/aeon/carla-usd/...`, not `${SWB_CARLA_USD}/...`; both Isaac examples
    need the re-upload of step 8 before they run on another host. No dataset has a tag
    yet, so fetches resolve `main`.
- Step 8 datasets, 2026-11-15: every public dataset tagged `v2026.11` (the newest tag is what
  `assets fetch` resolves). `carla-town-autoware-maps`: the Isaac world files name their stage
  `${SWB_CARLA_USD}/<Town>/<Town>_physics.usda`, the signal tables' `lanelet2_map:` is
  relative, the READMEs carry no host names or paths. `haruna-tsukuba-courses`: the scenarios'
  LogicFile is `$(env SWB_MAPS)/<course>`. `carla-0.9.16-isaac-usd`: the export logs' build
  user and paths replaced, MANIFEST updated. `isaac-road-physics-stages`: README without host
  paths. Checked: a copy of `first_drive` fetches the tagged commit and `project check`s clean.
  Older commits in the datasets' history still hold the host paths (not squashed). Not done:
  a tesla_model3 ego variant of the Haruna scenario.
- `ground:` (A9): `swb.ground` links a project's per-map surface files into `state/<map>/` at
  stack start (35bf522). Judges take `{run_dir}`/`{scenario}` and plain programs (exit code),
  so `physical_verdict` is one (67550bc). `capture: full|debug` records rosbags per pilot
  domain, `run --capture` (623deec).
- First live run (2026-11-15, CARLA 0.9.16 at Epic, play_launch 0.34, the `first_drive` copy):
  - fixed (5616e87): the stack's bridge and relay ran on a sourced Autoware's DDS profile, the
    runs on swb's, so SSv2 never found the relay's ADAPI (the run's `error` said so). The
    stack now passes its profile as `dds_config`, never the environment's.
  - fixed (5616e87): the autoware pilot's `vehicle` (the car) collided with the Autoware
    profile's `vehicle` switch; play_launch's parser refused the pilot. RViz defaults off.
  - friction: the stack's fingerprint names the build's commit, so a commit of the same tree
    after the stack started refuses every run ("restart the stack"). Right for users, costly
    for developers; a tree hash would keep it right and spare that restart.
  - friction: play_launch warns `dds-env-split` for the stack's nested sides, whose domains
    differ on purpose.

## Acceptance

- From a clean clone and build, with only the environment file sourced: `project new`,
  `assets fetch`, the stack, then `ros2 run swb run` reach a passed scenario on CARLA
  with an Autoware ego; a second `run` reuses the stack.
- One run with three pilots of different kinds (Autoware, `carla_basic`, an SDK agent) passes.
- A suite with a parameter distribution and repetitions, sharded over two hosts, merges into
  one table; a stack failure shows as `error`, not `failed`.
- The same project with `simulator: isaac` passes on Isaac; Haruna and the surfaces world drive
  there.
- A project copied to another host (`swb.yaml`, `swb.lock`, scenarios) runs unchanged after
  `assets fetch`; two seats on one host run side by side.
- The four studies of step 9 run on the design without a study-specific driver script.
- No user doc tells the reader to edit a path, make a link or run a repository script; each
  fact in B3's list appears in one page; the docs check passes; every tutorial ran as written;
  three fresh readers reach their goal.
