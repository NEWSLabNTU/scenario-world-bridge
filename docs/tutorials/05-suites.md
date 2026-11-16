# Tutorial 5: Suites

**Goal**: one command that runs your Tutorial 2 scenario over a grid of parameters, twice
each, judges every run with CARLA's own contact check beside SSv2's verdict, and leaves
one table of results.

```
$ ros2 run swb run suites/follow.yaml
passed   ..._follow_to_goal_leadSpeed-2.0_r0  ...  judges: physical_verdict=passed
...
summary: .../suites/follow/summary.csv
```

## You will learn

- to write a suite: scenarios, a parameter grid, repetitions, seeds, expectations,
  retries;
- to read a suite's `summary.csv` and merged JUnit;
- to add a judge, and why its verdict sits beside the run's;
- to split a suite between hosts, resume it, and keep its logs small.

## Before you start

- [Tutorial 2](02-write-a-scenario.md): the project `~/my_first_drive` with
  `scenarios/follow_to_goal.xosc` and its `leadSpeed` parameter, the stack running on
  Town01. Every terminal: `source ~/swb.bash`.
- About an hour: six runs of about a minute each, twice.

## 1. Write a suite

A suite is a YAML file anywhere in the project. Create `suites/follow.yaml`:

```yaml
# suites/follow.yaml
scenarios: [scenarios/follow_to_goal.xosc]
parameters:                  # a grid over the scenario's ParameterDeclarations
  leadSpeed: [2.0, 4.0, 6.0]
repetitions: 2
seed: 1000                   # run k gets seed 1000 + k
retry: {on: error, times: 1} # a run the stack could not judge is run once more
```

Three speeds times two repetitions: six runs. `scenarios:` takes files and directories
(every scenario in them); a pilot of `lifetime: run` receives `{seed}` and
`{param:<name>}` in its arguments.

**Check**: `ros2 run swb project check` prints `project: ok`; it parses every suite under
`suites/` too.

## 2. Run it

```bash
ros2 run swb run suites/follow.yaml
```

**What you see**: one line per run, in order, then the summary:

```
passed   2026-11-15T15-01-10_follow_to_goal_leadSpeed-2.0_r0  .../runs/...
passed   2026-11-15T15-02-21_follow_to_goal_leadSpeed-2.0_r1  .../runs/...
...
summary: .../my_first_drive/suites/follow/summary.csv
```

The command exits 0 only when every run ended as expected (passed, unless the suite
`expect`s otherwise), 1 when one failed, 2 when one could not be judged.

**Check**:

```bash
ros2 run swb runs summary
column -s, -t < runs/summary.csv | cut -c1-120
```

`runs summary` writes `summary.csv` and a merged `junit.xml` for the whole `runs/`
directory and prints their paths; the table has one row per run, with the columns
`run_id`, `scenario`, `params`, `repetition`, `seed`, `verdict`, `reason`, `duration`,
`judges` and `path`.

## 3. Add a judge

SSv2 judges collisions in its own world: the declared boxes, in 2D, nothing static. The
ego can clip a pole, a curb, or a car whose CARLA body is larger than its declared box,
and SSv2 still passes the run. The bridge records what CARLA's contact sensor saw in every
run; the `physical_verdict` judge reads it. Add it to `swb.yaml`:

```yaml
judges:
  - name: physical_verdict
    run: [ros2, run, swb_launch, physical_verdict, "{run_dir}/junit.xml",
          --scenario, "{scenario}"]
```

A judge is a program run after each passed or failed run, in the project directory, with
the run directory as its last argument; it prints one JSON object, and its exit code (0
passed, 1 failed) or the object's own `verdict` is the judge's verdict.

`swb.yaml` changed, so restart the stack (Ctrl-C, then the command of Tutorial 1, step 3):
a run refuses a stack started from another project state. Then:

```bash
ros2 run swb run suites/follow.yaml
```

**What you see**: each run line ends with `judges: physical_verdict=passed` (or
`failed`, naming what CARLA saw the ego touch).

**Check**:

```bash
cat runs/*leadSpeed-4.0_r0/judges/physical_verdict.json
```

shows the judge's verdict, version, exit code and duration.

## 4. Shard, resume, prune

On two hosts with the same project and lock, each takes half the suite:

```bash
ros2 run swb run suites/follow.yaml --shard 1/2    # on the first host
ros2 run swb run suites/follow.yaml --shard 2/2    # on the second
```

`--shard i/n` takes every n-th run from the i-th, so the two together cover the suite
once. A suite that stopped half way picks up where it was with `--resume`, which skips
the runs that have a result:

```bash
ros2 run swb run suites/follow.yaml --resume
```

**What you see**: `skipped  <run>  (resume: passed in ...)` for each run already done.

Runs keep what `logging.keep` says per verdict (in `swb.yaml` or the suite): the default
keeps a passed run's result only, and everything of a failed one. Old runs go with `runs prune`:

```bash
ros2 run swb runs prune --older-than 7 --dry-run
ros2 run swb runs prune --older-than 0 --dry-run
```

**Check**: runs made today are not older than 7 days, so the first says `nothing to
prune`; the second lists every run as `would remove <run>` and removes nothing (`runs
list` is unchanged).

## What happened

A suite expands into run specs: every scenario times every parameter combination times
every repetition, each with its own id and seed. They run one after another against the
same stack, ordered by map (so a suite over several towns moves the pilots once per town),
and each leaves a run directory as in Tutorial 1; the suite links them under
`<logs>/<project>/suites/<suite>/` with its table. The verdict of each run stays SSv2's;
a judge adds its own beside it, never in its place, so a scoring rule can change without
rewriting history. [Verdicts](../explanation/verdicts.md) explains `passed`, `failed` and
`error` and the contact check; the
[project file reference](../reference/project-file.md#suites) lists every suite key.

## Exercises

1. **Easy.** Add `leadSpeed: 1.0` to the grid and run with `--resume`. Check: only the two
   new runs run; the others print `skipped`.
2. **Medium.** Expect a failure: a second suite, `suites/too_slow.yaml`, runs the scenario
   with `leadSpeed: [0.5]` (80 m at 0.5 m/s outlasts the 120 s limit) and
   `expect: {follow_to_goal.xosc: failed}`. Check: the line reads
   `failed (expected failed)` and the command exits 0.
3. **Open.** Run the Autoware Foundation's use cases: copy
   `pkg://swb_examples/projects/safety_suite`, fetch, start its stack and run
   `suites/nightly.yaml` (about 45 minutes; 61 on a loaded host with one run retried,
   2026-11-16). Check: two `passed` lines, the ACC use case's JUnit with 28 testcases and
   the AEB one's with 10 (27 and 9 variants, plus the `carla_contact` testcase
   `physical_verdict` adds), each judged by `physical_verdict`.

## If it does not work

| Symptom | Fix |
|---|---|
| `swb run: refused: suites/follow.yaml: unknown key ...` | a suite takes only the keys of the [reference](../reference/project-file.md#suites) |
| `refused: the stack ... was started from another project state` | `swb.yaml` changed: restart the stack |
| A judge's verdict is `error` | it printed no JSON object, timed out (120 s by default) or exited with another code: its message is in `judges/<name>.json` |
| `refused: ... budget` | the log disk cannot hold the suite at `logging.budget`: prune runs, or lower the capture level |

## Next

[Tutorial 6: the same project on Isaac Sim](06-carla-assets-on-isaac.md), or
[Teaching with swb](../teaching.md) if you are setting up a lab.
