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
  stack starts it, which is recommended here) or already serving its port, and Autoware
  (tested with 1.5.0), as Install describes.
- Every terminal: `source ~/swb.bash`.
- Network access to Hugging Face for step 2, about 0.1 GB of disk for the map, and an
  NVIDIA GPU with room for CARLA and Autoware
  ([Install](../getting-started/install.md#time-and-space)).
- About 30 minutes once the install is done.

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
`tesla_model3`, an entry of the vehicle catalog that ships with swb (an OpenSCENARIO
`CatalogReference`).

**Check**:

```bash
ros2 run swb project check
echo $?
```

It reports two problems, both caused by the missing assets, and exits 3 (refused):

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

The datasets are public: no Hugging Face account is needed.

**What you see**: the dataset resolved to a commit, its files downloaded, the map linked
into the project, and the lock written:

```
carla-town-autoware-maps: e94113508882: 9 file(s), 9 downloaded
linked assets/maps/Town01 -> .../jerry73204/carla-town-autoware-maps/e94.../Town01
swb.lock written
```

`swb.lock` pins the dataset's commit and the checksum of every file, so that every later
fetch, on any machine, obtains the same files; the files reside in swb's cache, shared by
all projects, and `assets/` holds only links.

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
ros2 launch swb_launch swb.launch.xml
```

The **stack** is the long-lived set of processes that every run uses. It checks the
project and takes a **seat**, one stack's share of the host (its ROS domains and ports). It then starts CARLA if
`carla.root` is set and nothing answers on the port, and three processes: the **world
server**, the only process that drives CARLA; the **bridge**, which SSv2 addresses as its
simulator; and the **agent relay**, through which SSv2 reaches each pilot
([How it works](../explanation/how-it-works.md)). The stack stays up for all later runs.

The ego's pilot (Autoware, with the **vehicle bridge** that carries the car's sensors and
controls between CARLA and Autoware) starts when the first run needs it, on that run's
map; appending `map:=Town01` to the launch command starts it at once.

**What you see**, among `ros2 launch`'s own lines on the processes it starts (paths
shortened; the two CARLA lines only when the stack starts CARLA):

```
swb: project .../my_first_drive on seat 0 (carla, maps Town01; pilots on the first ...
[stack_keeper-1] swb stack 2026-11-15T10-15-42_seat0: seat 0; scenario domain 1, ...
[stack_keeper-1] swb stack: CARLA started on localhost:2000 (pid ..., log ...)
[stack_keeper-1] swb stack: CARLA serves RPC (0.9.16)
[stack_keeper-1] swb stack: the bridge serves SSv2 (its world server is up)
[stack_keeper-1] swb stack 2026-11-15T10-15-42_seat0: ready
```

`ros2 launch` leads each line of a process with the process's name; under play_launch the
same lines appear without it.

Each process of the stack writes its log under the log root
(`<logs>/my_first_drive/seat-<n>-current/proc/`). The recommended launcher,
[play_launch](https://pypi.org/project/play-launch/), starts the same stack with a log
directory per process and a web interface that lists every process with its state and log:

```bash
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml
```

where 8080 is seat 0's web port (`seat show` names another seat's). The tutorials use
`ros2 launch`; either serves every step ([Launchers](../reference/commands.md#launchers)).

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

With the default lidar detector the pilot is ready within seconds; a non-default one may
first build TensorRT engines for several minutes
([Your Autoware](../how-to/your-autoware.md)), during which the pilot's logs grow under
`<logs>/my_first_drive/seat-<n>-current/proc/pilot-ego/` (`logs:` of the host settings).

**Observing the drive.** A CARLA started by the stack has no window. RViz (`ego: {kind:
autoware, vehicle: tesla_model3, rviz: true}` in `swb.yaml`, then a restart of the stack),
`ros2 run swb_launch chase_view` (a chase camera, which requires `pygame`) or a CARLA
[started by hand](../getting-started/carla.md#or-start-it-yourself) with its window shows
the drive; without a display, the timeline of step 5 records it.

<p align="center">
  <img src="../media/t1_drive_to_goal.webp" alt="The Tesla Model 3 on a Town01 street seen from behind, beside RViz's speed bar (11 km/h, 4.8 m to the goal) and Autoware's state panel: routing set, localization initialized, motion moving, control autonomous" width="720"><br>
  <em>The last frame of a passing run: the ego is 4.8 m from its goal. SSv2 ends the run
  when the ego reaches the goal, so the car is still moving. Routing set, localization
  initialized and autonomous control are the states to look for.</em>
</p>

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

prints `0`: the run passed ([Exit codes](../reference/commands.md#exit-codes): 1 failed,
2 error, 3 refused).

## 5. Read the result

```bash
ros2 run swb runs list
ros2 run swb runs show drive_to_goal
```

**What you see**: `runs list` has one row, `passed`. `runs show` prints the run's header
(`run`, `verdict`, `scenario`, `duration`) and its timeline: one row per event, such as
SSv2's milestones and every warning in any process's log.

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
after `assets fetch`. SSv2 judges each run; swb records the verdict beside the logs of every
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
   ~/first_drive --from pkg://swb_examples/projects/first_drive`) and run its suite,
   `suites/all.yaml`, as [Tutorial 5](05-suites.md) describes. Check: three `passed`
   lines and a `summary:` line naming `summary.csv`.

## If it does not work

| Symptom | Fix |
|---|---|
| `swb: the stack is refused (3): ...` | the message names the problem and its fix; most often the assets were not fetched (step 2) |
| `assets fetch` prints `error: <url>: <reason>` and exits 2 | no network, or a proxy that requires `https_proxy`; a later attempt keeps the files already downloaded |
| `swb run: refused: no stack of project my_first_drive on any seat` | the stack is not up, or was started in another directory (step 3) |
| The run ends `error`: `pilot ego never became ready (600 s)` | a slow first start (step 4): run again, or give `--ready-timeout 1200` |
| `ros2 topic list` in another terminal shows nothing | that terminal is not on the pilot's ROS domain: `eval "$(ros2 run swb seat env 0)"` first (0 is the index of the seat's first pilot domain, the ego's, not a seat number) |

More, CARLA's own failures among them: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Tutorial 2: write a scenario](02-write-a-scenario.md): a scenario written from the map,
in this project.
