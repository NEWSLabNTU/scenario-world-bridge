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
- to run a scenario against the stack, watch it, and read its result and its timeline.

The words this tutorial uses (project, stack, pilot, seat, run, ego, SSv2, ...) are in the
[glossary](../reference/glossary.md).

## Before you start

- [Install](../getting-started/install.md) and
  [Getting started on CARLA](../getting-started/carla.md): the workspace built, the
  environment file, and CARLA either named in your host settings (`carla.root`, so the stack
  starts it: recommended for this tutorial) or serving its port.
- Every terminal: `source ~/swb.bash`.
- Network access to Hugging Face for step 2.
- About 30 minutes once the install is done. With the CenterPoint detector, the first
  Autoware start on a machine takes several minutes longer (step 4 says why).
- An NVIDIA GPU with room for CARLA and Autoware (on one host they held about 2.0 to 2.5 GB
  and 1 to 2 GB of video memory), and about 0.1 GB of disk for the map.

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

It lists two problems, both with the same fix, and exits 3 (refused: the assets are
missing). A problem in what you wrote, such as a scenario whose car is not its pilot's,
exits 1 instead ([Exit codes](../reference/commands.md#exit-codes)):

```
problem: swb.lock: missing: the assets were never fetched (fix: ros2 run swb assets ...
problem: scenarios/drive_to_goal.xosc: map Town01 is not fetched (assets/maps/Town0...
[ros2run]: Process exited with failure 3
```

The last line is `ros2 run`'s own, after every command that exits with a code other than 0.

## 2. Fetch the map

```bash
ros2 run swb assets fetch
```

The datasets are public: no Hugging Face account or login is needed, only network access.

**What you see**: the dataset resolved to a commit, its files downloaded (about 0.1 GB),
the map linked into the project, and the lock written:

```
carla-town-autoware-maps: e94113508882: 9 file(s), 9 downloaded
linked assets/maps/Town01 -> .../jerry73204/carla-town-autoware-maps/e94.../Town01
swb.lock written
```

Without a network the fetch retries a few times, then prints `error: <url>: <reason>` and
exits 2; run it again once the network is back. Files already downloaded are kept.

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
    swb_launch swb.launch.xml
```

The stack checks the project and takes a **seat**: one stack's share of the host (its ROS
domains, its ports and its web UI port, all from one number), so that it never meets
another user's stack. It starts CARLA when `carla.root` is set and nothing answers on the
port, then CARLA's world server, the bridge and the agent relay. Leave it running; every
run goes against it. `8080` is seat 0's web port: on a host with other users, take yours
from `ros2 run swb seat show`.

The ego's pilot, Autoware with CARLA's vehicle bridge, starts when the first run needs it,
on that run's map. To start it at once on a map, add `map:=Town01` to the command; nothing
else changes.

**What you see**, in order (paths shortened; the two CARLA lines only when the stack
starts CARLA):

```
swb: project .../my_first_drive on seat 0 (carla, maps Town01; pilots on the first ...
swb stack 2026-11-15T10-15-42_seat0: seat 0; scenario domain 1, SSv2 5555, ...
swb stack: CARLA started on localhost:2000 (pid ..., log ...); waiting for its RPC, ...
swb stack: CARLA serves RPC (0.9.16)
swb stack 2026-11-15T10-15-42_seat0: ready
```

A CARLA the stack starts takes one to three minutes to serve its port. The web UI at
`http://127.0.0.1:8080` lists every process with its log. On a remote machine, forward the
port first (`ssh -L 8080:127.0.0.1:8080 <host>`) and open the same address locally.

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

The run asks the stack to start the pilots on Town01, waits until Autoware can take a
route, then starts SSv2 on the scenario, which teleports the ego to its start, gives
Autoware the goal and engages it. The car drives west and stops at the goal.

**The pilot's first start.** With the default lidar detector, `clustering`, there is
nothing to build: on one host (2026-11-16) the pilot was ready 10 s after the run asked
for it. With `lidar_detection_model: centerpoint`
([Your Autoware](../how-to/your-autoware.md)) the first start on a machine builds TensorRT
engines for its neural networks and writes them to `~/autoware_data`, for minutes; later
starts load them. Meanwhile the run prints nothing, the machine is busy on every core, and
the pilot's logs grow under
`~/.local/state/swb/logs/my_first_drive/seat-0-current/proc/pilot-ego/` (and in the web
UI). That is not a hang: the run waits up to 600 s for the pilot (`--ready-timeout`).
Later runs against the same stack wait for nothing.

**Watching the drive.** CARLA started by the stack runs off-screen, without a window. On a
machine with a display:

- give the pilot RViz, `ego: {kind: autoware, vehicle: tesla_model3, rviz: true}` in
  `swb.yaml`, and restart the stack: Autoware's RViz shows the car, its route, the lidar and
  what it detects;
- or open a chase camera on the ego, `ros2 run swb_launch chase_view` (a window; it needs
  `pygame`);
- or start CARLA yourself without `-RenderOffScreen` for CARLA's own window
  ([Getting started on CARLA](../getting-started/carla.md#or-start-it-yourself)).

Without a display, the timeline and logs of step 5 tell what happened.

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

prints `0`: the run passed. The other codes: 1, the scenario failed (its conditions say
the car did not do what it should); 2, an error, a run the stack could not judge because
something in it broke (CARLA gone, a pilot that never became ready; `ros2 run swb runs why`
says what); 3, refused, nothing was run (no stack, a missing asset, a bad argument).

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
| `contacts/` | what CARLA's contact sensor saw, for the `physical_verdict` judge ([Tutorial 5](05-suites.md)) |
| `logs/<group>/` | each process's log for the time of the run; by default kept for a run that did not pass, not for a passed one (`logging.keep`, [Tutorial 5](05-suites.md)) |

**Check**:

```bash
grep '"verdict"' runs/*drive_to_goal*/result.json
```

prints `"verdict": "passed",`.

## 6. Run it again

```bash
ros2 run swb run scenarios/drive_to_goal.xosc
```

**What you see**: a second `passed` line with a new run id, and no map line. Nothing
restarted: the stack, Autoware included, serves one run after another.

**Check**: `ros2 run swb runs list` has two rows.

When you are done, stop the stack with Ctrl-C in its terminal: it ends every process it
started (CARLA too, if it started CARLA) and frees the seat. A CARLA you started yourself
stays up.

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
| `assets fetch` ends `error: ...` and exits 2 | Hugging Face did not answer: check the network (a proxy needs `https_proxy`), then fetch again |
| The stack stops with `CARLA ... ready_timeout` | the CARLA it started did not serve its port in 300 s: read the `carla.log` the message names; a first start on a new host can take longer, raise `carla.ready_timeout` |
| `swb run: refused: no stack of project my_first_drive on any seat` | the stack is not up, or was started in another directory: start it in the project (step 3) |
| The first run prints nothing for minutes | Autoware's first start builds TensorRT engines (step 4); `runs why` after the run if it ends `error` |
| The run ends `error`: `pilot ego never became ready (600 s)` | a slow first start: run again (the engines built so far are kept), or give `--ready-timeout 1200` |
| The run ends `error` and `runs why` names CARLA | CARLA is not serving its port (`ss -lnt` lists no `:2000`), or is another version |
| A `ros2 topic list` of your own shows nothing | the terminal is not on the pilot's ROS domain: `eval "$(ros2 run swb seat env 0)"` first. The argument is a domain of your seat: `scenario` for the bridge, relay and SSv2, or `0`, `1`, ... for the first, second, ... pilot ([ROS domains and DDS](../explanation/domains-and-dds.md)) |

More: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Tutorial 2: write a scenario](02-write-a-scenario.md), on your own road, in this project.
