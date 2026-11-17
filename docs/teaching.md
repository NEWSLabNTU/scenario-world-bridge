# Teaching with scensim

For a course or a lab on automated driving: what a lab host needs, many students sharing
a few hosts, the tutorials as a lab sequence, a sample schedule, and submitting and
grading with suites and judges. The tutorials are written for self-study, so a student can
also do them alone; this page is for the person who sets up the room. The words it uses
are in the [glossary](reference/glossary.md).

## What a student needs

A seat on a GPU host (below), the workspace sourced (`source ~/scensim.bash`), and a testbench
directory of their own. Nothing in a student's testbench is a path on the host, so the same
assignment repository works on every lab machine and on a student's own.

## Setting up a lab host (administrator)

These steps need sudo, or write to directories students cannot; do them once per host.

1. The machine-wide part of [Install](getting-started/install.md#once-per-machine-needs-sudo):
   the apt packages, `sudo rosdep init`, and the kernel socket buffers (`sysctl`).
2. Clone and build the workspace once, in a directory every student can read, for
   example `/opt/scensim`, following [Install, From source](getting-started/install.md#from-source)
   (including `rosdep update`, then `rosdep install`, which asks for sudo).
3. Unpack CARLA 0.9.16 where every student can read it, for example `/opt/CARLA_0.9.16`
   ([Getting started on CARLA](getting-started/carla.md)).
4. Write the host's settings for every user, `/etc/scensim/host.yaml`:

```yaml
# /etc/scensim/host.yaml
seats:
  domains: {base: 1, stride: 8}
  ports: {base: 5555, stride: 100}
  web: {base: 8080, stride: 10}
  carla: {base: 2000, stride: 10}   # seat n: its own CARLA on 2000 + 10 n
  count: 4
  gpus: [0, 0, 1, 1]                # seat -> GPU
seat_locks: /tmp/scensim-seats      # every user can write here
carla:
  root: /opt/CARLA_0.9.16           # each seat's stack starts its own CARLA from here
```

With `seats.carla` and `carla.root`, every seat's stack starts its own CARLA on its own
port when it starts and stops it when it stops: students never start CARLA by hand, and
never meet another seat's CARLA. `scensim seat show` prints each seat's CARLA port.

## What each student does once

Each student, in their own account:

```bash
python3 -m pip install --user 'play_launch>=0.30.0'   # Install, "Your tools"
cat > ~/scensim.bash <<'EOF'
source /opt/autoware/1.5.0/setup.bash
source /opt/scensim/install/setup.bash
export PATH=$HOME/.local/bin:$PATH
EOF
source ~/scensim.bash
ros2 run autoware_sim_profile setup_autoware_data      # ~/autoware_data, per user
scensim seat show
```

`setup_autoware_data` gives each student a writable `~/autoware_data`, where Autoware's
TensorRT engines are built on the student's first Autoware start and kept. That first start
is the slow one ([Failures in a lab](#failures-in-a-lab)): have every student do it in the
first session, one after another.

Each student's datasets go to their own cache (`~/.cache/scensim` by default). A cache is one
user's: a fetch into a cache another user owns fails on its permissions, so do not point
several users at one `cache:`. Town01's map is about 0.1 GB, so a per-student cache is
cheap on CARLA; the USD set for Isaac Sim is several GB per student. Each student's runs
go to their `logs:` (default `~/.local/state/scensim/logs`, on the home disk; point it at a
fast disk if home is slow or has a quota).

## Capacity

Measured on one 32-core host (a single host, other users' jobs beside it), not a sizing
guarantee:

| Part | GPU memory | Other |
|---|---|---|
| CARLA 0.9.16, Epic quality, Town01 | about 2.0 to 2.5 GB | about 2.5 CPU cores, 2.4 GB of RAM |
| one Autoware 1.5.0, default `clustering` lidar detector | about 1 to 2 GB | about 90 processes |
| Autoware with `lidar_detection_model: centerpoint` | not measured | builds its TensorRT engines on its first start: 3 to 11 minutes |
| the Isaac Sim world server, Town01 | an RTX card with 24 GB to itself | about 17.5 GB of RAM |

The four-pilot run of [Tutorial 4](tutorials/04-several-pilots.md) (two Autowares, one
BasicAgent, one program, one CARLA) ran on that host beside other users' jobs. Three
Autowares starting at once saturated a 48-core host: start stacks one after another.

So one seat with an Autoware ego costs roughly 3.5 to 4.5 GB of video memory. Before the
course, measure your own hosts with one seat, then two, then more, while a run of Tutorial 1
drives:

```bash
nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv -l 5
uptime          # the load average, against the core count
free -g
```

The number of seats a host carries is the number at which a run of Tutorial 1 still passes
and the load stays under the core count. Set `count:` to that; the default table has 7
seats, the most its domains allow.

**A lighter lab.** For labs about scenarios rather than Autoware, give the ego CARLA's
BasicAgent instead, which costs almost nothing beside CARLA:

```yaml
pilots:
  ego: {kind: carla_basic}
```

The same scenarios run, and perception is not tested (BasicAgent knows where everything
is). This variant has not been run through every tutorial.

## Twenty students, two hosts

Two hosts with, say, four seats each serve eight stacks at a time, not twenty. Students
share seats in turns:

- **Book time slots per seat.** A shared sheet: host, seat, time slot, student. In a slot
  the student takes that seat (`scensim seat take 2`, written to their own host
  settings) and starts their stack; at the end they stop it with Ctrl-C, which frees the
  seat and its CARLA.
- **Or first come, first served.** Leave every student on `seat: auto`: a stack takes the
  first free seat, and is refused when none is free, naming who holds each.
  `scensim seat show` lists the holders.
- **Pair up** for the Autoware labs (1 to 5): two students, one seat, one testbench.
- **Keep the slow work outside the slots.** Writing scenarios (Tutorial 2, step 1 needs
  CARLA for `carla_poses`, the rest does not), the agent check of Tutorial 4
  (`scensim-agent check`, no simulator), and reading results need no seat.

A seat is held by a lock the kernel drops when the stack's process ends, however it ends:
a crashed stack frees its seat at once.

## The tutorials as labs

Each tutorial has a goal, steps with a check each, and graded exercises with their own
checks: a verdict, a value on a topic, a file. Every check is a command whose output a
student can paste.

| Lab | Tutorial | A student shows |
|---|---|---|
| 1 | [First drive](tutorials/01-first-drive.md) | a `passed` run, and `runs show` read aloud: what each timeline row is |
| 2 | [Write a scenario](tutorials/02-write-a-scenario.md) | their own scenario passing, and a `--param` that makes it fail |
| 3 | [Other road users](tutorials/03-other-road-users.md) | a pedestrian or traffic-light scenario of their own, passing |
| 4 | [Several pilots](tutorials/04-several-pilots.md) | a pilot of their own on the SDK passing `scensim-agent check`, then a run with it |
| 5 | [Suites](tutorials/05-suites.md) | a suite over a parameter grid with repetitions, and its `summary.csv` |
| 6 | [Isaac Sim](tutorials/06-carla-assets-on-isaac.md) | the lab 2 scenario passing on Isaac Sim, unchanged |
| 7 | [Road physics](tutorials/07-road-physics.md) | a ground that makes the car stall, shown on `/scensim/wheels` |

Labs 1 to 5 run on CARLA; 6 and 7 need an Isaac seat. Tutorial 4 runs two Autowares in one
stack: it needs a seat with room for both (32 GB of RAM or more, and the GPU memory of two).

## A six-week schedule

One session a week, the lab hosts set up and every student's `~/scensim.bash` written before
week 1.

| Week | In the session | Due the next week |
|---|---|---|
| 1 | Lab 1. Every student's first Autoware start, one after another (the TensorRT build) | Lab 1 exercises 1 and 2 |
| 2 | Lab 2 | a scenario of their own, passing |
| 3 | Lab 3 | Lab 3 exercise 2; the assignment handed out |
| 4 | Lab 4 (in pairs, one seat with room for two Autowares) | a pilot passing `scensim-agent check` |
| 5 | Lab 5: suites and judges, the assignment's grading suite run on their own seat | the assignment, submitted |
| 6 | Labs 6 and 7 on Isaac seats; grading runs | |

**Without an Isaac seat** (no 24 GB RTX GPU free), week 6 stays on CARLA: Tutorial 5's
exercise 3 (the Autoware Foundation's ACC and AEB use cases, about 45 minutes, a good
demonstration of a nightly suite and an expected failure), or a second assignment round,
with Labs 6 and 7 shown once by the instructor on one Isaac host.

## Assignments as testbenches

An assignment is a testbench: a git repository with `package.xml`, `testbench.yaml`,
`scensim.lock`, the scenarios and suites the students start from, and the judges you grade
with. Students copy it into a workspace and a repository of their own:

```bash
mkdir -p ~/ws/src && cd ~/ws/src
scensim testbench new assignment1 --from /path/to/assignment1
cd assignment1 && git init && git add -A && git commit -m "start"
cd ~/ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch assignment1
```

The lock pins every asset's commit and checksum, so every student fetches the same files.
A run records the scenario's hash, the lock's hash and the workspace's commits in its
`result.json`, so a submitted result says what it ran.

**Submitting** is pushing that repository (to the course's git server) and handing in its
URL and commit. What belongs in it: `package.xml`, `testbench.yaml`, `scensim.lock`,
`scenarios/`, `suites/`, `pilots/`. A host's assets, state and runs are kept outside the
package, so they never enter it: the grader runs the work again rather than trusting the
student's runs.

## Grading with suites and judges

A grading suite runs the student's work over a fixed set of scenarios, parameters and
seeds; a judge scores each run. A judge is any program, run after each run with the run
directory, that prints one JSON object:

```yaml
# testbench.yaml of the assignment
judges:
  - name: grade
    run: [python3, judges/grade.py]
    version: "2026-fall"
```

A rubric as a judge, for example 6 points for a passed run, 2 for no collision and 2 for
reaching the goal within 60 s:

```python
# judges/grade.py: the rubric, from the run's result.json
import json, pathlib, sys
run = pathlib.Path(sys.argv[-1])
r = json.loads((run / "result.json").read_text())
passed = r["verdict"] == "passed"
collided = any(e["type"].startswith("collision.") for e in r.get("events", []))
duration = r.get("metrics", {}).get("duration_s")
in_time = passed and duration is not None and duration <= 60
points = 6.0 * passed + 2.0 * (not collided) + 2.0 * in_time
print(json.dumps({"verdict": "passed" if points >= 6 else "failed", "points": points}))
```

**The grader's seat.** Keep one seat on one host for grading only, off the booking sheet
(seat 3 of the table above). For each submission:

```bash
mkdir -p ~/grading/alice/src && cd ~/grading/alice/src
git clone <student repository> assignment1 && cd assignment1
git checkout <submitted commit>
cp -r ~/ws/src/assignment1/judges .              # your copies, not theirs
cp ~/ws/src/assignment1/suites/grading.yaml suites/
git diff --stat <assignment start commit> -- testbench.yaml   # what they changed
cd ~/grading/alice && colcon build --symlink-install && source install/setup.bash
scensim assets fetch assignment1
scensim testbench check assignment1
scensim run assignment1/grading --seat 3 --stop-stack
```

`--stop-stack` starts the testbench's stack on the grading seat, runs the suite, and stops
the stack, so one submission never sees another's stack. Copy your own judges and grading
suite over the student's, and read the `testbench.yaml` diff: the judges a run uses are named in
`testbench.yaml`. `summary.csv` has one row per run, with SSv2's verdict and each judge's in its
`judges` column; each judge's full output is `<logs>/assignment1/runs/<id>/judges/grade.json`. The run's own
verdict stays SSv2's: a judge adds to it, never replaces it
([Verdicts](explanation/verdicts.md)), so a grading rule can change and be applied again to
old runs. The exit code of `scensim run` (0, 1, 2, 3) tells a script whether every run passed,
one failed, the setup broke, or the run was refused; grade a run that ended `error` again,
not as a fail.

**Time.** A grading run costs the stack's start (Autoware's start, about a minute once its
engines are built) plus, per run, SSv2's start and the drive: Tutorial 1's scenario drives
for about a minute. Time one submission's suite before planning the rest. On several hosts,
`--shard i/n` splits one grading suite between them, and `--resume` continues one that was
cut short ([Run suites in CI](how-to/run-in-ci.md)).

## Expected results (instructor key)

What a correct exercise shows. **Run-dependent** marks the ones whose exact output varies
between runs or hosts: grade the method and the evidence, not a value.

| Exercise | Expected |
|---|---|
| T1 ex1 | three new `passed` rows ending `_r0`, `_r1`, `_r2` |
| T1 ex2 | a `failed` line, exit code 1; `runs why too_slow` names the simulation-time condition |
| T1 ex3 | three `passed` lines and a `summary:` line naming `summary.csv` |
| T2 ex1 | `passed`; a longer duration in `runs list` than the 80 m version |
| T2 ex2 | `failed` on the timeout; `runs show` has no arrival at the goal |
| T2 ex3 | **run-dependent**: the fastest failing `leadSpeed` lies between 0.5 (fails, Tutorial 5) and 2.0 (passes); the exact value moves between runs. Ask for the values tried and the `runs why` line |
| T3 ex1 | `passed`, about 10 s longer than before |
| T3 ex2 | `failed` on the 300 s timeout; the ego stopped at the stop line in `runs show` |
| T3 ex3 | open: `passed` with the student's `RelativeDistanceCondition` present; check the condition's sign and distance in the file |
| T4 ex1 | `pilot bg_av_2 ready` on the stack console; a third Autoware domain in `stack.json`. Needs room for three Autowares |
| T4 ex2 | `cut_in.json` distances between 30 and 35 m |
| T4 ex3 | `scensim-agent check` passes; a run ends `passed` or `failed`, not `error` |
| T5 ex1 | only the two new runs run; the others print `skipped` |
| T5 ex2 | `failed (expected failed)`, exit code 0 |
| T5 ex3 | two `passed` lines; the ACC JUnit with 27 testcases, the AEB one with 9 |
| T6 ex1 | **run-dependent evidence**: `passed`, and no `no CARLA body` line. An absent line proves little on its own: also ask for the world server's log lines naming the walker's model |
| T6 ex2 | a `passed` run on Town02 |
| T6 ex3 | the `usd:` line as the tutorial shows; the stack with `map:=my_town01` reaches `ready` |
| T7 ex1 | **run-dependent**: `failed` on the 180 s timeout, rear-wheel slip near 1 on `/scensim/wheels`. After a stall in mud Autoware may not drive again in the next run: restart the stack before the next exercise |
| T7 ex2 | `1 corridor(s)` and 250 m from `road_detail check`; the stack starts again |
| T7 ex3 | **run-dependent**: `ruts: set worn from ...` in the world server's log; the sinkage "differs" from a fresh run's, with no fixed amount. Ask for both values |

## Failures in a lab

| What students see | Cause, what to do |
|---|---|
| `seat <n> is taken: <user> (pid ...)` | a live stack holds it (a lock is freed when its process ends, so it is never stale). `scensim seat show` names the user and pid: their slot, or a stack left running; the administrator stops it by pid |
| the stack fails with `Address already in use` on a free seat | child processes of a stack killed by hand still hold the seat's ports: find them (`ps -eo pid,ppid,user,etimes,comm`, parent 1) and stop them by pid |
| the first Autoware start of every student takes minutes | each student's TensorRT engines are built once, into their `~/autoware_data`. Twenty students starting at once load every core and each build slows the others: have them start one after another in week 1. With `centerpoint` each build takes 3 to 11 minutes |
| CARLA crashes mid-run | the run fails within about 30 s; a CARLA the stack started is started again by the stack, and the next run waits for it |
| CARLA crashes when a scenario changes towns | it ran at `-quality-level=Low`; the stack starts CARLA at Epic. Only a CARLA started by hand can be Low |
| two seats fight over one CARLA | a CARLA started by hand on another seat's port, or no `seats.carla` in the table: let the stacks start CARLA (`carla.root`, `seats.carla`) and nobody start one by hand |
| runs refused with `budget`, or a full disk | the log root is full: `scensim runs prune --older-than 7`, or a lower `logging.capture`; check `df -h` on each user's `logs:` |
| a run refused: `the stack ... was started from another project state` | `testbench.yaml`, `scensim.lock` or the pilots changed after the stack started: restart the stack |
| out of GPU memory, a pilot crashing at start | too many seats on one GPU: `nvidia-smi`, then fewer seats per GPU (`gpus:`) |

More symptoms: [Troubleshooting](reference/troubleshooting.md).

## Working on a lab host from a laptop

Students usually reach the hosts over SSH. The stack's web UI (every process and its log)
is on the seat's web port, `8080 + 10 n` for seat `n`: forward it and open it locally.

```bash
ssh -L 8080:127.0.0.1:8080 student@labhost      # seat 0; seat 2 is 8100
```

Then `http://127.0.0.1:8080` in the laptop's browser. RViz (a pilot's `rviz: true`) and
`chase_view` open windows on the host's display, so they need one: a desktop session on the
host, or a VNC server there. Without a display, `runs show`, `runs why` and the web UI tell
what happened in a run.
