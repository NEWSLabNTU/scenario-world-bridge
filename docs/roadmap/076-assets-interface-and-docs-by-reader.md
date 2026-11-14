# Phase 076: One Entry Launch, Assets by Name, and Docs by Reader

A user names what they want (a simulator, a world, a scenario, an autopilot) and swb fetches
the data, finds it and runs it. No absolute paths to edit, no symlinks to make, no repository
scripts, no internal knowledge. The docs are organised by what the reader is doing, each fact
lives in one place, and every command in them has run.

**Status**: planned, 2026-11-14; second design after a use-case study the same day (A). Nothing
retires before this design is settled (user, 2026-11-14).

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

`ros2 run swb_launch project new <dir> [--template carla-first-drive|isaac|multi-pilot]`
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
ros2 run swb_launch run scenarios/cut_in.xosc                    # one run
ros2 run swb_launch run scenarios/cut_in.xosc --repeat 5
ros2 run swb_launch run suites/nightly.yaml --shard 1/3 --resume
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

### A6. Seats: host settings in one number

`SWB_SEAT=<n>` (a host setting, never in the project) derives the scenario domain, one
domain per pilot, SSv2's port and its +1/+2, the relay's, the world server's, the web port,
the GPU (`SWB_GPU`) and the log root, from one table. A lock per seat names its holder;
`seat:=auto` takes the first free one. `ros2 run swb_launch seat show` prints a seat's
numbers and who holds it. Two projects, two students or two checkouts on one host each take
a seat; nothing else moves by hand.

### A7. Assets: read-only cache, lock, project state

- `ros2 run swb_launch assets fetch|list|check` reads `swb.yaml`, writes `swb.lock` (each
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
2. [ ] The user's word on the second design.
3. [ ] A2, A7: project file and lock, `project new|check`, `assets fetch|list|check` with the
   read-only cache and `state/`; tests.
4. [ ] A6: seats (the table, the lock, `seat show`); the stack's processes take their numbers.
5. [ ] A5: the vehicle side split from the driver; the pilot contract; kinds `autoware`,
   `carla_basic`, `launch`, `command`, `none` (`simple` with 070 D, `container` later).
6. [ ] A3, A4: `swb.launch.xml` as the stack (fingerprint, lazy pilots, `end_with:=run`);
   `ros2 run swb_launch run` for runs and suites; `run_one`.
7. [ ] A8: `result.json` schema with provenance, `resolved.yaml`, summaries, judges.
8. [ ] A9: signal table on first load, `make_world`, the surfaces map, the rough road in the
   package, `/swb/wheels`, `record:=`, the datasets re-uploaded and tagged.
9. [ ] Port the studies onto it as the proof: an AWF suite, a cooperation batch sharded on two
   hosts, a Haruna drift run with its own pilot, the golf cart twin as a package project.
10. [ ] Retire what the design replaces (`ego_launch`, `background_av.launch.xml` on the user
   path, `CARLA_MAPS`/`CARLA_USD` for users), by name.
11. [ ] B: the docs reorganised (B1-B5).
12. [ ] B6: the docs check; live runs; fresh readers.

## Acceptance

- From a clean clone and build, with only the environment file sourced: `project new`,
  `assets fetch`, the stack, then `ros2 run swb_launch run` reach a passed scenario on CARLA
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
