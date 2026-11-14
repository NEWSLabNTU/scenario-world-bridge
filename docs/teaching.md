# Teaching with swb

For a course or a lab on automated driving: what a lab host needs, many students on one
host, the tutorials as a lab sequence, and grading with suites and judges. The tutorials
are written for self-study, so a student can also do them alone; this page is for the
person who sets up the room.

## What a student needs

A seat on a GPU host (below), the workspace sourced (`source ~/swb.bash`, from
[Install](getting-started/install.md)), and a project directory of their own. Nothing in
a student's project is a path on the host, so the same assignment repository works on
every lab machine and on a student's own.

## The lab host

Install and build the workspace once, in a directory every student can read (for example
`/opt/scenario-world-bridge`, [Install](getting-started/install.md)); each student's
`~/swb.bash` sources it. Then the host's settings, for every user, in
`/etc/swb/host.yaml`:

```yaml
# /etc/swb/host.yaml
seats:
  domains: {base: 1, stride: 8}
  ports: {base: 5555, stride: 100}
  web: {base: 8080, stride: 10}
  count: 4
  gpus: [0, 0, 1, 1]                # seat -> GPU
seat_locks: /tmp/swb-seats          # every user can write here
```

Each student takes a seat once, which writes it to their own `~/.config/swb/host.yaml`:

```bash
ros2 run swb seat show
ros2 run swb seat take 2
```

A stack holds its seat while it is up, and a second stack on a held seat is refused,
naming the holder; `seat show` lists who holds what. Seats keep students' ROS domains,
ports, web UIs and GPUs apart ([Several stacks on one host](how-to/several-stacks.md)).

**One CARLA per seat.** A CARLA serves one stack. Start one per seat, each on its own port
(`-carla-rpc-port=2000`, `3000`, `4000`, ...: CARLA uses its port and the two after it),
and put each seat's port in the student's host settings, `carla: {port: 3000}`. A shared
host's CARLAs are best started by the administrator, so that students never restart one
another's.

**What a seat costs.** CARLA holds about 2.4 GB of RAM and 2 GB of video memory on
Town01; each Autoware about 90 processes and several GB of video memory. One GPU carries
one or two seats with an Autoware each; an Isaac Sim seat needs a 24 GB GPU to itself.
Start the stacks one after another: three Autowares starting together saturated a 48-core
host.

**A lighter lab.** For labs about scenarios rather than Autoware, give the ego CARLA's
BasicAgent instead, which costs almost nothing beside CARLA:

```yaml
pilots:
  ego: {kind: carla_basic}
```

The tutorials are written with Autoware; with `carla_basic` the same scenarios run, and
perception is not tested (BasicAgent knows where everything is). This variant has not been
run through every tutorial.

**Disk.** Each student's runs go to their `logs:` (default
`~/.local/state/swb/logs`, on the home disk; point it at a fast disk if home is slow), and
their datasets to their `cache:`. Town01's map is about 0.1 GB, so a per-student cache is
cheap on CARLA; the USD set for Isaac is several GB per student.

## The tutorials as labs

Each tutorial has a goal, steps with a check each, and graded exercises with their own
checks: a verdict, a value on a topic, a file. Every check is a command whose output a
student can paste.

| Lab | Tutorial | A student shows |
|---|---|---|
| 1 | [First drive](tutorials/01-first-drive.md) | a `passed` run, and `runs show` read aloud: what each timeline row is |
| 2 | [Write a scenario](tutorials/02-write-a-scenario.md) | their own scenario passing, and a `--param` that makes it fail |
| 3 | [Other road users](tutorials/03-other-road-users.md) | a pedestrian or traffic-light scenario of their own, passing |
| 4 | [Several pilots](tutorials/04-several-pilots.md) | a pilot of their own on the SDK passing `swb-agent check`, then a run with it |
| 5 | [Suites](tutorials/05-suites.md) | a suite over a parameter grid with repetitions, and its `summary.csv` |
| 6 | [Isaac Sim](tutorials/06-carla-assets-on-isaac.md) | the lab 2 scenario passing on Isaac Sim, unchanged |
| 7 | [Road physics](tutorials/07-road-physics.md) | a ground that makes the car stall, shown on `/swb/wheels` |

Labs 1 to 5 run on CARLA; 6 and 7 need an Isaac seat.

## Assignments as projects

An assignment is a project: a repository with `swb.yaml`, `swb.lock`, the scenarios and
suites the students start from, and the judges you grade with. Students copy it:

```bash
ros2 run swb project new ~/assignment1 --from /path/to/assignment1
cd ~/assignment1
ros2 run swb assets fetch
```

The lock pins every asset's commit and checksum, so every student fetches the same files.
A run records the scenario's hash, the lock's hash and the workspace's commits in its
`result.json`, so a submitted result says what it ran.

## Grading with suites and judges

A grading suite runs the student's work over a fixed set of scenarios, parameters and
seeds; a judge scores each run. A judge is any program, run after each run with the run
directory, that prints one JSON object:

```yaml
# swb.yaml of the assignment
judges:
  - name: grade
    run: [python3, judges/grade.py]
    version: "2026-fall"
```

```python
# judges/grade.py: one point for a passed run; add your own criteria from result.json
import json, sys, pathlib
run = pathlib.Path(sys.argv[-1])
result = json.loads((run / "result.json").read_text())
points = 1.0 if result["verdict"] == "passed" else 0.0
print(json.dumps({"verdict": "passed" if points else "failed", "points": points}))
```

Then, in the student's project with the student's stack up (or on a grading host):

```bash
ros2 run swb run suites/grading.yaml
ros2 run swb runs summary
```

`summary.csv` has one row per run, with SSv2's verdict and each judge's in its `judges`
column; the merged `junit.xml` reads into any CI or grading tool; each judge's full output
is `runs/<id>/judges/grade.json`. The run's own verdict stays SSv2's: a judge adds to it,
never replaces it ([Verdicts](explanation/verdicts.md)), so a grading rule can change
and be applied again to old runs. The exit code of `swb run` (0, 1, 2, 3) tells a script
whether every run passed, one failed, the setup broke, or the run was refused.

On several hosts, `--shard i/n` splits one grading suite between them, and `--resume`
continues one that was cut short ([Run suites in CI](how-to/run-in-ci.md)).
