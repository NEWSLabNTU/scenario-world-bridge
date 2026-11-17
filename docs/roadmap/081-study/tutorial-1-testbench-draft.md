# Draft: Tutorial 1 on a testbench workspace

*A paper draft of `docs/tutorials/01-first-drive.md` rewritten for
[081](../081-testbench-packages.md)'s testbenches, by 243 on 2026-11-17, before 081 is
built. Nothing here is implemented; the outputs are what the design implies, not what a
program printed. Choices the design leaves open are made here and listed in
[questions.md](questions.md).*

---

# Tutorial 1: First drive

**Goal**: Autoware drives a Tesla Model 3 70 m along a road in CARLA's Town01 to a goal,
and the verdict of the run is read back. The tutorial ends with this state:

```
~/my_ws/
  src/my_first_drive/        the testbench: package.xml, testbench.yaml, scensim.lock,
                             scenarios/drive_to_goal.xosc
  install/                   what colcon built

$ scensim runs list my_first_drive
RUN                                   VERDICT  SCENARIO            DURATION  WORKSPACE
2026-11-18T10-21-07_drive_to_goal_r0  passed   drive_to_goal.xosc  41.2s     ~/my_ws
```

## You will learn

- making a **testbench**, the package that holds a use case, in a colcon workspace;
- building the workspace, and editing a testbench without rebuilding it;
- fetching a testbench's assets by name;
- running the testbench, and reading the result.

## Before you start

- [Install](../../getting-started/install.md) and
  [Getting started on CARLA](../../getting-started/carla.md): scensim installed, the
  environment file written, CARLA named in the host settings (`carla.root`), and Autoware
  1.5.0 installed.
- Every terminal: `source ~/scensim.bash`.
- About 30 minutes, and 0.1 GB of disk for the map.

## 1. Make a workspace and a testbench

Your work lives in a colcon workspace, as any ROS work does. A **testbench** is a package
in it that names everything one use case needs: the simulator, the assets, the scenarios,
and what drives each vehicle.

```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
scensim testbench new my_first_drive
cat my_first_drive/package.xml my_first_drive/testbench.yaml
```

**What you see**: `wrote src/my_first_drive (carla-first-drive)`, then the two files that
make it a testbench. The package manifest declares the build type and what the testbench
uses:

```xml
<package format="3">
  <name>my_first_drive</name>
  <version>0.1.0</version>
  <description>One Autoware ego on CARLA's Town01.</description>
  <maintainer email="ada@example.org">Ada</maintainer>
  <license>TODO: License declaration</license>
  <exec_depend>scensim</exec_depend>
  <export><build_type>scensim_testbench</build_type></export>
</package>
```

and `testbench.yaml` states three things:

```yaml
simulator: carla
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
default: first                    # what `scensim run my_first_drive` runs: suites/first.yaml
```

the simulator; the **assets**, Town01 of a public dataset of maps; and the **pilot** of
the scenario entity named `ego`, that is, what drives it: Autoware, on a Tesla Model 3.
The **ego** is the vehicle under test. `default:` names the **suite** that runs when the
testbench is run by name: here `suites/first.yaml`, which holds the one scenario,
`scenarios/drive_to_goal.xosc`. The maintainer comes from your git settings (`git config
user.name`, `user.email`); set the licence before you share the testbench.

## 2. Build the workspace

```bash
cd ~/my_ws
colcon build --symlink-install
source install/setup.bash
```

A testbench compiles nothing: the build installs its files and checks it, with the same
rules as `scensim testbench check`. `--symlink-install` links the installed files to your
sources, so an edit to a scenario or to `testbench.yaml` takes effect at once; a rebuild is
needed only when a package is added or removed.

**What you see**: `Summary: 1 package finished`. A testbench with a problem fails its
build, naming the file, the problem and the fix.

## 3. Fetch the map

```bash
scensim assets fetch my_first_drive
```

**What you see**:

```
carla-town-autoware-maps: e94113508882: 9 file(s), 9 downloaded
src/my_first_drive/scensim.lock written
```

The map goes to scensim's cache, shared by every testbench of yours; the testbench only
names it. `scensim.lock` records the dataset's exact commit and each file's checksum, so
that any machine fetches the same files. It is written to your source directory: commit it
with the testbench.

Without `--symlink-install` the build installs copies, and `assets fetch` needs
`--source src/my_first_drive` to know where to write the lock.

**Check**: `scensim testbench check my_first_drive` ends `testbench: ok`.

## 4. Run the testbench

```bash
scensim run my_first_drive
```

scensim first brings up the testbench's **stack**, the long-lived set of processes every
run uses. It takes a **seat**, one stack's share of the host (its ROS domains and ports),
starts CARLA if nothing answers on its port, then the **world server** that drives CARLA,
the **bridge** that SSv2 addresses as its simulator, and the **agent relay** through which
SSv2 reaches each pilot. The stack stays up for later runs, which then start at once.

Then the **run**, one execution of a scenario against the stack: SSv2
(scenario_simulator_v2, the OpenSCENARIO interpreter that executes and judges the
scenario) places the ego, gives Autoware the goal and engages it; the car drives west and
stops at the goal. Autoware is ready within about 40 s on its first start.

**What you see**:

```
stack    my_first_drive on seat 0: started (CARLA, Town01); left up, `scensim stack ...
map      Town01  (pilots restarted from none in 6.3 s)
passed   2026-11-18T10-21-07_drive_to_goal_r0  .../my_first_drive/runs/2026-11-18T10-...
summary: .../my_first_drive/suites/first/summary.csv
```

**Check**: `echo $?` prints `0`.

## 5. Read the result

```bash
scensim runs list my_first_drive
scensim runs show 2026-11-18T10-21-07_drive_to_goal_r0
```

`runs list` has one row, `passed`, with the workspace it came from; `runs show` prints the run's verdict, duration and
timeline. The run's directory, under the log root, holds `result.json` and every
process's log.

## 6. Change the scenario, without a rebuild

Open `src/my_first_drive/scenarios/drive_to_goal.xosc` and halve the goal's distance. Run
it again (step 4). The new goal applies at once, since the build linked the file, and the
stack is already up.

When you are done, stop the stack: `scensim stack my_first_drive --stop`.

## What happened

You built a workspace whose one package is a testbench, fetched what it names, and ran one
of its scenarios on a stack that scensim started for it. `scensim stack my_first_drive`
starts that stack in a terminal of its own, where its log scrolls by, for when you want
to watch it. Further testbenches and **support packages** (your own pilot,
car or world server) go in the same `src/`, side by side.

## Exercises

1. **Easy.** Run the one scenario twice, by its path:
   `scensim run my_first_drive/scenarios/drive_to_goal.xosc --repeat 2`. Check: two new
   rows in `runs list`.
2. **Medium.** Add a second testbench, `scensim testbench new other`, build, and list both
   with `scensim testbench list`.
3. **Open.** Put the workspace in git, with `scensim.lock`, and clone it on another
   machine. Check: `scensim assets fetch my_first_drive` there downloads the same commit.

## If it does not work

| Symptom | Fix |
|---|---|
| `colcon build`: `unknown build type scensim_testbench` | the terminal did not source `~/scensim.bash` before building: the build type comes with scensim's `setup.bash` |
| `scensim run`: `no testbench my_first_drive` | the terminal did not source `~/my_ws/install/setup.bash`, or the workspace was not rebuilt after the testbench was added |
| `testbench.yaml`: `swb.yaml is renamed testbench.yaml` | a project from an older scensim: rename the file |
| The run ends `error`: `pilot ego never became ready (600 s)` | a slow first start: run again, or give `--ready-timeout 1200` |

## Next

[Tutorial 2: Write a scenario](../../tutorials/02-write-a-scenario.md).
