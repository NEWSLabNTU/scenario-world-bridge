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

## A. The interface

### A1. The data directory

One root for everything swb downloads, `SWB_DATA` (default `~/.local/share/swb`):

```
$SWB_DATA/
  carla-usd/            CARLA 0.9.16's towns and actors in USD (Isaac only)
  maps/                 one directory per map, an Autoware map dir:
    Town01/ ... Town10/   lanelet2, pcd, carla/traffic_lights.yaml, isaac/world.yaml
    haruna/ tsukuba/      the courses
    surfaces/             the 13-surface ground, with a lanelet2 map (A4)
  vehicles/             extra cars (the golf cart)
```

`CARLA_MAPS` keeps its meaning (the maps directory every scenario names in `LogicFile`) and
defaults to `$SWB_DATA/maps`; `CARLA_USD` defaults to `$SWB_DATA/carla-usd`. A user who
exports neither gets both from `SWB_DATA`.

### A2. `ros2 run swb_launch assets`

```bash
ros2 run swb_launch assets list            # what exists, what is fetched, sizes
ros2 run swb_launch assets fetch carla-maps
ros2 run swb_launch assets fetch carla-usd courses road-physics golfcart
ros2 run swb_launch assets check           # every fetched set against its manifest
```

- Downloads the pinned revision of each Hugging Face dataset (huggingface_hub when present,
  else plain HTTPS from the Hub's resolve URLs), resumable.
- Places each set under `$SWB_DATA` as A1, checks its manifest, and wires the parts: a
  course's props reach `carla-usd/` without a hand-made link; no file names an absolute path.
- `fetch` is idempotent and prints what it changed; `check` exits 1 on a missing or changed
  file.
- Replaces `scripts/download_maps.sh` for users (it stays for building the maps).

### A3. Names instead of paths

- A world file may name the CARLA set relatively: `usd:` expands `${CARLA_USD}` (and `~`), so
  `maps/Town01/isaac/world.yaml` says `usd: ${CARLA_USD}/Town01/Town01_physics.usda`.
- The signal table names its osm relative to its map dir.
- Scenarios name maps as `$(env CARLA_MAPS)/<map>`; the course scenarios get that, and a
  `tesla_model3` variant next to the drift one.
- The datasets are re-uploaded with these fixes and tagged (A6).

### A4. Fewer manual steps

- The CARLA world server writes a town's traffic-light table on first load when it is
  missing (today a user runs `--generate-signal-table`).
- A town's Isaac world: `ros2 run swb_isaac_world make_world <map dir>` (today
  `isaac_town.sh`); the fetched maps already carry theirs.
- The 13-surface ground gets a lanelet2 map, a point cloud and a scenario, so the stack drives
  it like any map (`world:=surfaces`).
- The rough road (roughness, potholes, washboard, rocks) moves from `spikes/` into the
  package as a world option: `road_detail.yaml` beside `surfaces.yaml`, documented keys,
  example files shipped.

### A5. The entry launch

`swb_launch swb.launch.xml`, the one file a user starts from:

```bash
play_launch launch swb_launch swb.launch.xml \
    sim:=carla \
    world:=Town01 \
    scenario:=town01_ego_drive \
    autopilot:=autoware
```

| Argument | Values | Default |
|---|---|---|
| `sim` | `carla`, `isaac`, or a simulator package | `carla` |
| `world` | a map name under `CARLA_MAPS`, or a map dir path | the scenario's map |
| `scenario` | a name (swb_examples, a fetched set) or a `.xosc` path; empty: none | empty |
| `autopilot` | `autoware`, `none`, or `<package> <launch>` | `autoware` |
| `vehicle` | a car by name | `tesla_model3` |
| `spawn` | free drive pose, when no scenario | empty |
| `end_with_scenario` | the stack ends with the scenario | `true` with a scenario |
| `record` | a rosbag of the run | `false` |
| `rviz` | RViz on the ego | `true` |

It resolves names (A3), refuses what is not fetched with the `assets fetch` line that fixes
it, sets `CARLA_MAPS`/`CARLA_USD` for the processes it starts, checks domains, and includes
`carla.launch.xml` / `isaac.launch.xml` / `stack.launch.py` (which stay, for the long-lived
and scripted forms). `world` without `scenario` is free drive at the map's first spawn point.

### A6. Telemetry and recording

- A ROS topic for what the road does to the car, per wheel: surface class, friction, slip,
  sinkage (Isaac), `/swb/wheels`, so a run can be plotted and judged.
- `record:=true` on the entry launch (and `isaac.launch.xml`): a rosbag of the ego's key
  topics and `/swb/wheels`.

### A7. The datasets, versioned

Re-uploaded with A3's fixes, a tag per release (`v2026.11`), one card layout (what, use with
swb, how it was made, licence, citation). `assets` pins a tag.

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

| # | Tutorial | Simulator | Builds on |
|---|---|---|---|
| 1 | First drive: the entry launch, a scenario, its verdict | CARLA | getting started |
| 2 | Write a scenario: poses, the ego's goal, a pass condition | CARLA | 1 |
| 3 | Other road users: pedestrians, cars, another AV | CARLA | 2 |
| 4 | Run a suite and judge it | either | 2 |
| 5 | Isaac Sim: the same scenario on CARLA's assets in USD | Isaac | 1 |
| 6 | Road physics: surfaces, mud, a rough road, watching the wheels | Isaac | 5 |
| 7 | Your autopilot: replace Autoware with your own agent | either | 1 |

How-to guides: your car; your Autoware; your simulator; background AVs at scale; several
stacks on one host; recording and demos; configuration (bridge, world server, weather).

### B6. Checks

- `just test` gains a docs check: no `just` in user docs, code lines at most 88 columns, no
  banned characters, no broken relative links.
- Every command in tutorials and how-to guides runs once on a quiet host before it ships
  (CARLA and Isaac), and three fresh readers (a newcomer, an adopter, an educator), each with
  only the docs, reach their goal.

## Steps

1. [ ] This doc; 075's open items pointed here.
2. [ ] A1, A2: `SWB_DATA`, `ros2 run swb_launch assets` with tests.
3. [ ] A3, A7: names instead of paths in swb and the datasets; datasets re-uploaded and tagged.
4. [ ] A5: the entry launch with tests (name resolution, refusals, domains).
5. [ ] A4, A6: signal table on first load, `make_world`, the surfaces map, the rough road in
   the package, `/swb/wheels`, `record:=`.
6. [ ] B: the docs reorganised (B1-B5).
7. [ ] B6: the docs check; live runs; fresh readers.

## Acceptance

- From a clean clone and build, with nothing exported but the environment file:
  `ros2 run swb_launch assets fetch carla-maps` and one `swb.launch.xml` command reach a passed
  scenario on CARLA; the same with `fetch carla-usd` and `sim:=isaac`; `world:=haruna` and
  `world:=surfaces` drive on Isaac.
- No user doc tells the reader to edit a path, make a link or run a repository script.
- Each fact in B3's list appears in one page.
- The docs check passes; every tutorial ran as written; three fresh readers reach their goal.
