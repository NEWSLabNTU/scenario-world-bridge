# Tutorial 1: First drive

**Goal**: Autoware drives a Tesla Model 3 70 m along a road in CARLA's Town01 to a goal,
and the verdict of the run is read back. The tutorial ends with this state:

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

- creating a project with `ros2 run swb project new`, and reading its contents;
- fetching a project's assets by name and verifying them;
- bringing up a stack and recognising when it is ready;
- running a scenario, observing it, and reading its result and timeline.

Terms are defined at first use; the [glossary](../reference/glossary.md) collects them.

## Before you start

- [Install](../getting-started/install.md) and
  [Getting started on CARLA](../getting-started/carla.md): the workspace built, the
  environment file, and CARLA either named in the host settings (`carla.root`, so that the
  stack starts it, which is recommended here) or already serving its port.
- ROS 2 Humble is the fixed requirement; the simulator and the autopilot are choices.
  This tutorial uses CARLA 0.9.16, the version swb is built for, and Autoware: a built
  Autoware workspace whose `setup.bash` the host setting `autoware.setup` names. swb
  provides the car's launch files and configuration, tested with Autoware 1.5.0.
- Every terminal: `source ~/swb.bash`.
- Network access to Hugging Face for step 2.
- About 30 minutes once the install is done.
- An NVIDIA GPU with room for CARLA and Autoware (together about 3 to 4.5 GB of video
  memory on one host), and about 0.1 GB of disk for the map.

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

A **project** is a directory that names everything a use case needs. Its file states
three things: the simulator, the assets (Town01 of the `carla-town-autoware-maps` dataset),
and the **pilot** of the scenario entity named `ego`, that is, what drives it (Autoware, on
a Tesla Model 3). The **ego** is the vehicle under test. The scenario,
`scenarios/drive_to_goal.xosc`, names its map as `$(env SWB_MAPS)/Town01` and its ego as
the catalog car `tesla_model3`.

**Check**:

```bash
ros2 run swb project check
echo $?
```

It reports two problems with a common fix and exits 3, meaning refused, because the assets
are missing ([Exit codes](../reference/commands.md#exit-codes) lists the others):

```
problem: swb.lock: missing: the assets were never fetched (fix: ros2 run swb assets ...
problem: scenarios/drive_to_goal.xosc: map Town01 is not fetched (assets/maps/Town0...
[ros2run]: Process exited with failure 3
```

The last line is `ros2 run`'s own, printed after any command that does not exit 0.

## 2. Fetch the map

```bash
ros2 run swb assets fetch
```

The datasets are public, so no Hugging Face account is needed, only network access.

**What you see**: the dataset resolved to a commit, its files downloaded (about 0.1 GB),
the map linked into the project, and the lock written:

```
carla-town-autoware-maps: e94113508882: 9 file(s), 9 downloaded
linked assets/maps/Town01 -> .../jerry73204/carla-town-autoware-maps/e94.../Town01
swb.lock written
```

Without a network the fetch prints `error: <url>: <reason>` and exits 2 (a proxy requires
`https_proxy`); a later attempt keeps the files already downloaded.

`swb.lock` pins the dataset's commit and the checksum of every file. Committed with the
project, it guarantees that every later fetch, on any machine, obtains the same files. The
files themselves reside in swb's cache, shared by all projects; `assets/` holds only
links.

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
    swb_launch swb.launch.xml
```

The **stack** is the long-lived set of processes that every run uses. It checks the
project and takes a **seat**, one stack's share of the host (its ROS domains and ports),
so that it never interferes with another user's stack. It then starts CARLA if
`carla.root` is set and nothing answers on the port, then CARLA's world server, the bridge
and the agent relay, and it stays up for all later runs.
`8080` is seat 0's web port; on a shared host, `ros2 run swb seat show` gives the port of
the seat actually taken.

The ego's pilot (Autoware with CARLA's vehicle bridge) starts when the first run needs it,
on that run's map; `map:=Town01` on the command line starts it at once.

**What you see**, in order (paths shortened; the two CARLA lines only when the stack
starts CARLA):

```
swb: project .../my_first_drive on seat 0 (carla, maps Town01; pilots on the first ...
swb stack 2026-11-15T10-15-42_seat0: seat 0; scenario domain 1, SSv2 5555, ...
swb stack: CARLA started on localhost:2000 (pid ..., log ...); waiting for its RPC, ...
swb stack: CARLA serves RPC (0.9.16)
swb stack 2026-11-15T10-15-42_seat0: ready
```

A CARLA started by the stack takes one to three minutes to serve its port. The web UI at
`http://127.0.0.1:8080` lists every process with its log; on a remote machine, the port is
forwarded first (`ssh -L 8080:127.0.0.1:8080 <host>`).

**Check**, in a second terminal:

```bash
source ~/swb.bash
ros2 run swb seat show
```

The seat's line ends `held by <you> (pid ..., project my_first_drive, ...)  <- yours`.

## 4. Run the scenario

In the second terminal, in the project:

```bash
cd ~/my_first_drive
ros2 run swb run scenarios/drive_to_goal.xosc
```

A **run** is one execution of a scenario against the stack. This one asks the stack to
start the pilots on Town01 and waits until Autoware can accept a route. SSv2
(scenario_simulator_v2, the OpenSCENARIO interpreter that executes and judges the scenario)
then teleports the ego to its start, gives Autoware the goal and engages it. The car drives
west and stops at the goal.

**The pilot's first start.** With the default lidar detector, `clustering`, the pilot is
ready within seconds. With `lidar_detection_model: centerpoint`
([Your Autoware](../how-to/your-autoware.md)), the first start on a machine builds TensorRT
engines into `~/autoware_data`, which takes minutes. During that time the run prints
nothing while the pilot's logs grow under
`~/.local/state/swb/logs/my_first_drive/seat-0-current/proc/pilot-ego/` (and in the web
UI). This is not a hang: the run waits up to 600 s for the pilot (`--ready-timeout`), and
later starts load the stored engines.

**Observing the drive.** A CARLA started by the stack renders without a window. On a
machine with a display, the pilot can show RViz (`rviz: true` in its entry in `swb.yaml`,
then a restart of the stack), `ros2 run swb_launch chase_view` opens a chase camera on the
ego (it requires `pygame`), or CARLA can be started by hand with its own window
([Getting started on CARLA](../getting-started/carla.md#or-start-it-yourself)). Without a
display, the timeline and logs of step 5 record what happened.

**What you see**: the map line on the first run, then, after about a minute of driving,
one line: the verdict, the run id and the run directory.

```
map      Town01  (pilots restarted from none in ... s)
passed   2026-11-15T10-21-07_drive_to_goal_r0  .../my_first_drive/runs/2026-11-15T10-...
```

**Check**:

```bash
echo $?
```

prints `0`: the run passed. A failed scenario exits 1. An error, that is, a run the stack
could not judge because a component broke, exits 2 (`ros2 run swb runs why` names the
cause). A refused run, in which nothing was executed, exits 3.

## 5. Read the result

```bash
ros2 run swb runs list
ros2 run swb runs show drive_to_goal
```

**What you see**: `runs list` has one row, `passed`. `runs show` prints the run's header
(`run`, `verdict`, `scenario`, `duration`) and then its timeline: one row per event, such
as SSv2's milestones and every warning in any process's log.

The run's directory holds `result.json` (the verdict, its reason, the pilots and the
versions that ran), `timeline.jsonl`, SSv2's `junit.xml` and, for a run that did not pass,
each process's log under `logs/`; [Results and logs](../reference/results-and-logs.md)
describes every file.

**Check**:

```bash
grep '"verdict"' runs/*drive_to_goal*/result.json
```

prints `"verdict": "passed",`.

## 6. Run it again

```bash
ros2 run swb run scenarios/drive_to_goal.xosc
```

**What you see**: a second `passed` line with a new run id, and no map line. Nothing was
restarted: the stack, Autoware included, serves one run after another.

**Check**: `ros2 run swb runs list` has two rows.

To finish, stop the stack with Ctrl-C in its terminal. It ends every process it started
(CARLA too, if it started CARLA) and frees the seat; a CARLA started by hand stays up.

## What happened

The project contains no path of the local machine, so it runs unchanged on another one
after `assets fetch`. The stack outlives the runs, and a run on another map first moves
the pilots there. SSv2 judges each run; swb records the verdict beside the logs of every
process. [How it works](../explanation/how-it-works.md) explains the parts, and
[Verdicts](../explanation/verdicts.md) the meaning of `passed`, `failed` and `error`.

## Exercises

1. **Easy.** Run the scenario three times with one command: `ros2 run swb run
   scenarios/drive_to_goal.xosc --repeat 3`. Check: `ros2 run swb runs list` shows three
   new rows, ending `_r0`, `_r1`, `_r2`.
2. **Medium.** Make the run fail deliberately: in a copy of the scenario,
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
| `swb: the stack is refused (3): ...` | the message names the problem and its fix; most often the assets were not fetched (step 2) |
| The stack stops with `CARLA ... ready_timeout` | the CARLA it started did not serve its port in 300 s: read the `carla.log` the message names, or raise `carla.ready_timeout` |
| `swb run: refused: no stack of project my_first_drive on any seat` | the stack is not up, or was started in another directory (step 3) |
| The run ends `error`: `pilot ego never became ready (600 s)` | a slow first start (step 4): run again, or give `--ready-timeout 1200` |
| `ros2 topic list` in another terminal shows nothing | that terminal is not on the pilot's ROS domain: `eval "$(ros2 run swb seat env 0)"` first |

More, CARLA's own failures among them: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Tutorial 2: write a scenario](02-write-a-scenario.md), on a road of one's choosing, in this
project.
