# Tutorial 1: First drive

**Goal**: Autoware drives a Tesla Model 3 70 m down a road in CARLA's Town01 to a goal, and
you read the verdict of the run. At the end you have this:

```
~/my_first_drive/
  swb.yaml  swb.lock  README.md  .gitignore
  scenarios/drive_to_goal.xosc
  assets/maps/Town01 -> (the map, in swb's cache)
  runs/ -> (one directory per run, under the log root)

$ ros2 run swb runs list
RUN                                   VERDICT  SCENARIO           DURATION
2026-11-15T10-21-07_drive_to_goal_r0  passed   drive_to_goal.xosc 41.2s
```

## You will learn

- to make a project with `ros2 run swb project new` and say what is in it;
- to fetch a project's assets by name and check them;
- to bring up a stack, and tell when it is ready;
- to run a scenario against the stack and read its result and its timeline.

## Before you start

- [Install](../getting-started/install.md) and
  [Getting started on CARLA](../getting-started/carla.md): the workspace built, the
  environment file, CARLA serving its port.
- Every terminal: `source ~/swb.bash`.
- About 30 minutes. The first Autoware start on a machine builds TensorRT engines and can
  take several minutes; later starts take about one.
- An NVIDIA GPU with room for CARLA (about 2 GB on Town01) and Autoware, about 0.1 GB of
  disk for the map.

## 1. Make a project

```bash
ros2 run swb project new ~/my_first_drive
cd ~/my_first_drive
cat swb.yaml
```

**What you see**: `wrote /home/<you>/my_first_drive (carla-first-drive); next: cd
~/my_first_drive && ros2 run swb assets fetch`, then the project file:

```yaml
# my_first_drive: one Autoware ego on CARLA's Town01.
simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
```

Three facts: the simulator, the assets (Town01 of the `carla-town-autoware-maps` dataset),
and who drives the scenario entity named `ego` (Autoware, on a Tesla Model 3). The
scenario, `scenarios/drive_to_goal.xosc`, names its map as `$(env SWB_MAPS)/Town01` and
its ego as the catalog car `tesla_model3` with the controller `Autoware`.

**Check**:

```bash
ros2 run swb project check
echo $?
```

It lists two problems, both with the same fix, and exits 3 (refused):

```
problem: swb.lock: missing: the assets were never fetched (fix: ros2 run swb assets ...
problem: scenarios/drive_to_goal.xosc: map Town01 is not fetched (assets/maps/Town0...
```

## 2. Fetch the map

```bash
ros2 run swb assets fetch
```

**What you see**: the dataset resolved to a commit, its files downloaded (about 0.1 GB),
the map linked into the project, and the lock written:

```
carla-town-autoware-maps: e94113508882: 9 file(s), 9 downloaded
linked assets/maps/Town01 -> .../jerry73204/carla-town-autoware-maps/e94.../Town01
swb.lock written
```

`swb.lock` pins the dataset's commit and every file's checksum: commit it with the
project, and every later fetch, on any machine, gets the same files. The files live in
swb's cache (`cache:` of the host settings), read-only and shared by all your projects;
`assets/` holds only links.

**Check**:

```bash
ros2 run swb project check
ros2 run swb assets check
```

print `project: ok` and `assets: ok`.

## 3. Bring up the stack

In a terminal of its own, in the project directory:

```bash
cd ~/my_first_drive
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml \
    map:=Town01
```

The stack checks the project, takes a seat, starts CARLA's world server, the bridge, the
agent relay and, because of `map:=Town01`, the ego's pilot: Autoware on Town01 with CARLA's
vehicle bridge. Leave it running; every run goes against it. `8080` is seat 0's web port:
on a host with other users, take yours from `ros2 run swb seat show`.

**What you see**, in order (paths shortened):

```
swb: project .../my_first_drive on seat 0 (carla, maps Town01; pilots on Town01); ...
swb stack 2026-11-15T10-15-42_seat0: seat 0; scenario domain 1, SSv2 5555, ...
swb stack: pilot ego ready on Town01
swb stack 2026-11-15T10-15-42_seat0: ready
```

The third line comes when Autoware can take a scenario: a minute, or several on the first
start. The web UI at `http://127.0.0.1:8080` lists every process with its log.

**Check**, in a second terminal:

```bash
source ~/swb.bash
ros2 run swb seat show
```

Your seat's line ends `held by <you> (pid ..., project my_first_drive, ...)  <- yours`.

## 4. Run the scenario

In the second terminal, in the project:

```bash
cd ~/my_first_drive
ros2 run swb run scenarios/drive_to_goal.xosc
```

The run starts SSv2 on the scenario, which teleports the ego to its start, gives Autoware
the goal and engages it. In CARLA's window (or RViz, if the pilot has `rviz: true`) the
car drives west and stops at the goal.

**What you see**, after about a minute: one line, the verdict, the run id and the run
directory.

```
passed   2026-11-15T10-21-07_drive_to_goal_r0  .../my_first_drive/runs/2026-11-15T10-...
```

**Check**:

```bash
echo $?
```

prints `0`: the run passed. A failed run exits 1, a run the stack could not judge 2, a
refused one 3.

## 5. Read the result

```bash
ros2 run swb runs list
ros2 run swb runs show drive_to_goal
```

**What you see**: `runs list` has one row, `passed`. `runs show` prints the run's header
(`run`, `verdict`, `scenario`, `duration`) and then its timeline: one row per event, with
the wall time, the simulation time, the process group and the event (the run's start, the
pilot's readiness, SSv2's milestones, every warning in any process's log).

The run's directory holds the record:

| File | Holds |
|---|---|
| `result.json` | the verdict, its reason, the scenario's hash, the pilots, the versions that ran |
| `resolved.yaml` | everything the run was given, after defaults |
| `junit.xml` | SSv2's verdict as JUnit |
| `timeline.jsonl` | the timeline above |
| `logs/<group>/` | each process's log for the time of the run |

**Check**:

```bash
grep '"verdict"' runs/*drive_to_goal*/result.json
```

prints `"verdict": "passed",`.

## 6. Run it again

```bash
ros2 run swb run scenarios/drive_to_goal.xosc
```

**What you see**: a second `passed` line with a new run id. Nothing restarted: the stack,
Autoware included, serves one run after another.

**Check**: `ros2 run swb runs list` has two rows.

When you are done, stop the stack with Ctrl-C in its terminal: it ends every process it
started and frees the seat. CARLA stays up.

## What happened

- The **project** holds what this use case needs, by name: the simulator, the dataset and
  the part of it to take, the pilot of each entity. Nothing in it is a path on your
  machine, so it runs unchanged on another one after `assets fetch`.
- The **stack** is long-lived: the world server, the bridge, the relay and the pilots,
  each process in the ROS domain of your seat. Runs come and go; a run whose scenario is
  on another map moves the pilots there first.
- A **run** is one scenario against the stack. SSv2 judges it; swb records the verdict
  beside the logs of every process, cut to the time of the run.

[How it works](../explanation/how-it-works.md) explains the parts;
[Verdicts](../explanation/verdicts.md) what `passed`, `failed` and `error` mean.

## Exercises

1. **Easy.** Run the scenario three times in one command: `ros2 run swb run
   scenarios/drive_to_goal.xosc --repeat 3`. Check: `ros2 run swb runs list` shows three
   new rows, ending `_r0`, `_r1`, `_r2`.
2. **Medium.** Make the run fail on purpose: in a copy of the scenario,
   `scenarios/too_slow.xosc`, lower the timeout (`SimulationTimeCondition value="180"`) to
   `5`. Check: the run line starts with `failed`, the command exits 1, and `ros2 run swb
   runs why too_slow` names the cause.
3. **Open.** Copy the shipped example with three scenarios (`ros2 run swb project new
   ~/first_drive --from pkg://swb_examples/projects/first_drive`), fetch its map, and run
   its suite, `suites/all.yaml`, against a stack started there. Check: three `passed`
   lines and a `summary:` line naming `summary.csv`.

## If it does not work

| Symptom | Fix |
|---|---|
| The stack stops at once: `swb: the stack is refused (3): ...` | the message names the problem and its fix; most often the assets were not fetched (step 2) |
| `swb run: refused: no stack of project my_first_drive on any seat` | the stack is not up, or was started in another directory: start it in the project (step 3) |
| `swb run: refused: the stack ... is not ready (... waiting for ego); try --wait 600` | Autoware is still starting: wait for `pilot ego ready`, or add `--wait 600` |
| The run ends `error` and `runs why` names CARLA | CARLA is not serving its port (`ss -lnt` lists no `:2000`), or is another version |
| A `ros2 topic list` of your own shows nothing | the terminal is not on the stack's domain and DDS: `eval "$(ros2 run swb seat env 0)"` first ([ROS domains and DDS](../explanation/domains-and-dds.md)) |

More: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Tutorial 2: write a scenario](02-write-a-scenario.md), on your own road, in this project.
