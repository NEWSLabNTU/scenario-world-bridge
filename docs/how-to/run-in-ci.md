# Run suites in CI

Run a testbench's suites unattended (a nightly job, a merge check) and hand the verdicts to
the CI system. It assumes a host with the workspace built, CARLA installed (named in the
host settings as `carla.root`, so the stack starts it, or already serving), and the testbench
checked out ([Start a testbench](start-a-project.md)); suites are
[Tutorial 5](../tutorials/05-suites.md).

## A job

```bash
#!/usr/bin/env bash
# nightly.sh: the project's nightly suite against a stack this job starts and stops
set -u
source ~/scensim.bash
cd "$PROJECT"
scensim assets fetch || exit 3
scensim project check || exit 3

setsid ros2 launch scensim_launch scensim.launch.xml map:=Town01 > stack.log 2>&1 &
STACK=$!

scensim run suites/nightly.yaml --wait 1200 | tee run.out
CODE=${PIPESTATUS[0]}
# the first cause of each run that did not pass
awk '$1 == "failed" || $1 == "error" {print $2}' run.out \
    | xargs -r -n1 scensim runs why

kill -INT -$STACK            # the whole stack, and only it
wait $STACK
scensim runs summary
exit $CODE
```

`--wait 1200` lets the run wait up to 20 minutes for the stack to become ready, instead of
refusing at once. `setsid` gives the stack a process group of its own, so the job stops
exactly what it started. A stack that is already up and from the same testbench state
serves the job as it is; one from another state is refused (restart it). `map:=Town01`
starts the pilots before the first run, so `--wait` covers Autoware's start too.

**Headless.** A CI host needs no display: the CARLA the stack starts runs off-screen
(`-RenderOffScreen`), and nothing else opens a window unless a pilot asks for RViz
(`rviz: true`; leave it out in CI).

## In a CI system

The job needs the host's GPU, CARLA and the built workspace, so it runs on a machine of
your own: a self-hosted runner. For GitHub Actions, for example:

```yaml
# .github/workflows/nightly.yml in the project's repository
on:
  schedule: [{cron: "0 2 * * *"}]
jobs:
  nightly:
    runs-on: [self-hosted, gpu]
    timeout-minutes: 240
    steps:
      - uses: actions/checkout@v4
      - run: PROJECT=$PWD bash nightly.sh
      - uses: actions/upload-artifact@v4
        if: always()
        with:
          name: results
          path: runs/*/result.json
```

A GitLab CI job is the same script under a `tags:` that picks the GPU runner. The
runner's user needs its own host settings (`~/.config/scensim/host.yaml`) and a seat no
person books.

## What the CI system reads

| Exit code | Means |
|---|---|
| 0 | every run ended as expected (passed, or what the suite `expect`s) |
| 1 | a scenario failed |
| 2 | a run could not be judged for the stack's fault (`error`): a world server or bridge error, a pilot that never became ready |
| 3 | refused: no stack, a stack of another testbench state, a missing asset, bad input |

Each suite writes `summary.csv` and a merged `junit.xml` under
`<logs>/<project>/suites/<suite>/`, printed on its last line (`summary: <path>`); `ros2 run
scensim runs summary [<dir>]` writes the same for any directory of runs. Any JUnit viewer reads
the merged file: one test suite per run, SSv2's testcases in it, and an `error` testcase
when the stack failed a run SSv2 did not see fail. A judge's verdict is in `summary.csv`'s
`judges` column and in each run's `judges/`, never in place of the run's
([Verdicts](../explanation/verdicts.md)).

## Two hosts, one suite

With the same testbench and `scensim.lock` on both:

```bash
scensim run suites/nightly.yaml --shard 1/2     # host A
scensim run suites/nightly.yaml --shard 2/2     # host B
```

Each writes its own `suites/nightly_shard-1-of-2/` table. Copy both runs directories to one
place and run `scensim runs summary <dir>` there for one table. A job that was cut
short continues with `--resume`, which skips the runs that already have a result.

## Keep the disk

`logging` in `testbench.yaml` (or the suite) sets what a run records (`capture`) and what it
keeps once its verdict is known (`keep`), and `budget` refuses a suite the log disk cannot
hold:

```yaml
logging:
  capture: standard
  keep: {passed: result, failed: full, error: full}
  budget: {per_run: 2GB, total: 200GB}
```

Old runs go with `scensim runs prune --older-than 14`, or shrink with
`--to result`. The levels: [Results and logs](../reference/results-and-logs.md).
