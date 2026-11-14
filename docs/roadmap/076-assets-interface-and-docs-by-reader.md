# Phase 076: One Entry Launch, Assets by Name, and Docs by Reader

A user names what they want (a simulator, a world, a scenario, an autopilot) and swb fetches
the data, finds it and runs it. No absolute paths to edit, no symlinks to make, no repository
scripts, no internal knowledge. The docs are organised by what the reader is doing, each fact
lives in one place, and every command in them has run.

**Status**: planned, 2026-11-14.

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

## A. The interface

### A1. The project directory

A **project** is a directory that holds one use case: which simulator, which world, which
scenarios, which cars and which pilots drive them, and the assets they need. It is what a user
keeps in git and shares; swapping to another combination is pointing at another project (or
overriding one line on the command line).

```
my_project/
  swb.yaml           # the project file (A2): simulator, world, pilots, assets
  scenarios/         # the project's .xosc files
  vehicles/          # the project's own cars (specs, catalogs), if any
  pilots/            # the project's own pilot configs or launches, if any
  assets/            # filled by `assets fetch` (A3); not committed
    maps/Town01/       an Autoware map dir (lanelet2, pcd, signal table, isaac/world.yaml)
    carla-usd/         only when the project runs Isaac
  runs/              # each run's logs, verdicts and recordings (A6); not committed
```

No global data directory: a project names what it needs and gets only that. Large sets are
downloaded once into a cache (`~/.cache/swb/`, keyed by dataset and version) and linked into
each project's `assets/`, so ten projects on Town01 hold one copy.

Inside a project, scenarios name maps through `$(env SWB_MAPS)/<map>`, which the entry launch
sets to the project's `assets/maps`. `CARLA_MAPS` and `CARLA_USD` are retired from the user
path by name (the no-shims rule: a launch that finds them says what replaces them); the
developer recipes keep working through the same mechanism.

### A2. The project file

```yaml
# swb.yaml
simulator: carla                 # carla | isaac | <your simulator package>
world: Town01                    # a map under assets/maps
assets:                          # what `assets fetch` brings in, pinned
  - carla-town-autoware-maps@v2026.11: [Town01]
scenario: scenarios/cut_in.xosc  # the default scenario; empty: free drive
pilots:                          # one per scenario entity a pilot drives
  ego:
    kind: autoware
    vehicle: tesla_model3
  bg_av_1:
    kind: autoware
  cut_in_car:
    kind: carla_basic
  my_car:
    kind: launch
    launch: my_pilot_pkg my_pilot.launch.xml
    args: {gain: 0.8}
domains: {scenario: 9, pilots: 1}  # pilots take 1, 2, 3, ... in order
```

`ros2 run swb_launch project new <dir>` writes a starter project (A2's file, one scenario, a
README); `project check` validates the file, the assets and the scenarios' entities against
the pilots before anything starts.

### A3. `ros2 run swb_launch assets`

```bash
ros2 run swb_launch assets fetch          # what swb.yaml lists, into ./assets
ros2 run swb_launch assets list           # the datasets that exist, and what is fetched
ros2 run swb_launch assets check          # every fetched set against its manifest
```

- Pinned revisions of the Hugging Face datasets (huggingface_hub when present, else plain
  HTTPS), resumable, through the shared cache.
- Wires sets together inside the project: a course's props reach `assets/carla-usd/`; no file
  names an absolute path (A4).
- `fetch` is idempotent and prints what it changed; `check` exits 1 on a missing or changed
  file.

### A4. Names instead of paths

- A world file may name the CARLA set relatively (`usd:` expands `${SWB_CARLA_USD}`, set by the
  entry launch to the project's `assets/carla-usd`).
- Signal tables name their osm relative to their map dir.
- Shipped scenarios name maps as `$(env SWB_MAPS)/<map>`; the course scenarios get a
  `tesla_model3` variant beside the drift one.
- The datasets are re-uploaded with these fixes and tagged (A8).

### A5. Pilots: one interface for any autopilot

A **pilot** drives one scenario entity: an autopilot plus the agent that connects it to the
relay. The relay and the agent protocol are already the per-entity interface; a pilot is the
launch that starts both, with one contract:

| Argument | Meaning |
|---|---|
| `entity` | the scenario entity it drives |
| `relay` | the relay's `tcp://host:port` |
| `sim` | `carla` or `isaac`, for pilots that need the simulator's side (sensors, vehicle bridge) |
| `map_path` | the map directory |
| `vehicle` | the car (its spec and catalog entry) |
| `sim_endpoint` | the simulator's address (CARLA host:port, the sensor release sockets) |

The pilot runs in a ROS domain the entry launch gives it (one per pilot), as a respawned
play_launch of its own, so pilots of different kinds never share topics.

| `kind` | What runs | Simulators |
|---|---|---|
| `autoware` | Autoware with swb's vehicle agent (acb on CARLA, aib on Isaac) | both |
| `carla_basic` | CARLA's BasicAgent through swb's adapter | CARLA |
| `simple` | `simple_pilot`: pure pursuit and a speed PID on the lanelet map (070 D) | both |
| `launch` | your launch, given the contract's arguments | yours |
| `command` | a program of yours (an SDK agent), given `--relay --entity` | any |
| `none` | nothing: you start a pilot yourself | any |

The stack's `ego_launch`, `agent` and `background_av.launch.xml` become the `autoware` kind
and retire from the user path.

### A6. The entry launch

`swb_launch swb.launch.xml`, the one file a user starts from:

```bash
cd my_project
play_launch launch swb_launch swb.launch.xml
```

It reads `./swb.yaml` (or `project:=<dir>`). Any line can be overridden for one run:

| Argument | Overrides |
|---|---|
| `project` | the project directory (default: the current one) |
| `simulator`, `world`, `scenario` | the file's |
| `pilots` | per entity, e.g. `pilots:="ego=carla_basic bg_av_1=none"` |
| `end_with_scenario` | the stack ends with the scenario (default `true` with a scenario) |
| `record` | a rosbag of the run (A7) |
| `rviz` | RViz on the ego's pilot |

It checks the project (A2's `project check`), refuses what is not fetched with the `assets
fetch` line that fixes it, assigns domains (checked 0-57), sets `SWB_MAPS` and
`SWB_CARLA_USD`, starts the simulator side, one pilot per entry, and the scenario, and
writes the run's logs and verdict to `runs/<time>/`. `carla.launch.xml`, `isaac.launch.xml`
and `stack.launch.py` stay as the long-lived and scripted forms underneath.

### A7. Fewer manual steps

- The CARLA world server writes a town's traffic-light table on first load when it is
  missing (today a user runs `--generate-signal-table`).
- A town's Isaac world: `ros2 run swb_isaac_world make_world <map dir>` (today
  `isaac_town.sh`); the fetched maps already carry theirs.
- The 13-surface ground gets a lanelet2 map, a point cloud and a scenario, so the stack drives
  it like any map (`world:=surfaces`).
- The rough road (roughness, potholes, washboard, rocks) moves from `spikes/` into the
  package as a world option: `road_detail.yaml` beside `surfaces.yaml`, documented keys,
  example files shipped.

### A8. Telemetry and recording

- A ROS topic for what the road does to the car, per wheel: surface class, friction, slip,
  sinkage (Isaac), `/swb/wheels`, so a run can be plotted and judged.
- `record:=true` on the entry launch (and `isaac.launch.xml`): a rosbag of the ego's key
  topics and `/swb/wheels`, in the run's `runs/<time>/`.

### A9. The datasets, versioned

Re-uploaded with A3's fixes, a tag per release (`v2026.11`), one card layout (what, use with
swb, how it was made, licence, citation). The project file pins a tag.

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
| 1 | First drive: a starter project, `assets fetch`, the entry launch, the verdict | CARLA | getting started |
| 2 | Write a scenario: poses, the ego's goal, a pass condition | CARLA | 1 |
| 3 | Other road users: pedestrians, scripted cars | CARLA | 2 |
| 4 | Several pilots: Autoware, CARLA's BasicAgent and your own agent in one run | CARLA | 3 |
| 5 | Run a suite and judge it | CARLA | 2 |
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

1. [ ] This doc; 075's open items pointed here.
2. [ ] A1-A3: the project directory and file, `project new|check`, `assets fetch|list|check`
   with the shared cache; tests.
3. [ ] A4, A9: names instead of paths in swb and the datasets; datasets re-uploaded and tagged.
4. [ ] A5: the pilot contract; `autoware`, `carla_basic`, `launch`, `command`, `none` kinds;
   `simple` with 070 D.
5. [ ] A6: the entry launch with tests (project resolution, overrides, refusals, domains,
   several pilots).
6. [ ] A7, A8: signal table on first load, `make_world`, the surfaces map, the rough road in
   the package, `/swb/wheels`, `record:=`.
7. [ ] B: the docs reorganised (B1-B5).
8. [ ] B6: the docs check; live runs; fresh readers.

## Acceptance

- From a clean clone and build, with only the environment file sourced: `project new`,
  `assets fetch` and `play_launch launch swb_launch swb.launch.xml` in that project reach a
  passed scenario on CARLA with an Autoware ego.
- One run with three pilots of different kinds (Autoware, `carla_basic`, an SDK agent) passes.
- The same project with `simulator: isaac` and `carla-0.9.16-isaac-usd` fetched passes on Isaac;
  the Haruna and surfaces worlds drive there.
- A project copied to another host (its `swb.yaml` and scenarios only) runs after
  `assets fetch`, unchanged.
- No user doc tells the reader to edit a path, make a link or run a repository script.
- Each fact in B3's list appears in one page.
- The docs check passes; every tutorial ran as written; three fresh readers reach their goal.
