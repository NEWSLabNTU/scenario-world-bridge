# Results and logs

What a run leaves, where, and what each file holds. The verdicts are explained in
[Verdicts](../explanation/verdicts.md); the commands that read them are `scensim runs
list|show|why|summary|prune` ([Commands](commands.md#scensim-runs)).

## Where

```
<logs>/<project name>/
  stacks/<stack id>/   stack.json  events.jsonl  proc/<group>/...   one per stack
  seat-<n>-current  -> stacks/<stack id>          the live stack on seat n
  runs/<run id>/       one per run (below)
  suites/<suite>[_shard-i-of-n]/   links to its runs, summary.csv, rollouts.csv, ...
  suites/rollouts-<scenario>/      the same, for a scenario run more than once
<project>/runs  -> <logs>/<project name>/runs
```

`<logs>` is the host settings' `logs:` (default `~/.local/state/scensim/logs`); keep it on a
fast disk, since every process of a stack writes there. A stack id is
`<time>_seat<n>`; a run id `<time>_<scenario>[_<name>-<value>...]_r<k>`.

`stack.json` is the stack's record: its fingerprint (hashes of `testbench.yaml` and `scensim.lock`,
the pilots, the workspace's build commits), seat, domains and ports, processes, maps, its
current map, each pilot's entity, kind, domain and readiness, and `state` (`ready` once
every stack pilot is). A run refuses a stack whose fingerprint is not the testbench's.

## A run's directory

| File | Level | Holds |
|---|---|---|
| `result.json` | result | the verdict and its first cause, the scenario, events, metrics, pilots, provenance (below) |
| `resolved.yaml` | result | everything the run was given, after defaults |
| `junit.xml` | result | SSv2's verdict as JUnit |
| `timeline.jsonl` | result | one event per line, by wall time (below) |
| `judges/<name>.json` | result | each judge's record |
| `contacts/` | result | the bridge's contact report for each ego run of the scenario (CARLA's collision sensor), unless `world.contact_report_dir` is set |
| `gap_trace/` | standard | with `logging.gap_trace: true`: the bridge's gap trace for each episode of the run (SSv2's world and the simulator's, frame by frame), and the gap judges' findings beside it |
| `context.jsonl` | standard | with a judge that `needs: [context]`: SSv2's storyboard state changes, one per line (`sim`, `element`, `from`, `to`) |
| `logs/<group>/` | standard | every stack process's log for the time of the run (a slice), SSv2's own logs (`logs/ssv2/`), `lifetime: run` pilots' logs |
| `record/<entity>/` | full | a rosbag (sqlite3, not mcap) of the pilot's key topics, from the moment every pilot is ready until SSv2 ends |
| `host.json` | full | samples of the host's load |
| `record/scenario/` | debug | every topic of the scenario domain (and every topic of each pilot domain in `record/<entity>/`) |

Groups: `simulator`, `bridge`, `relay`, `pilot-<entity>`, `scenario`, `judge`. The key
topics at `full`: the ego's state, the control command, the vehicle status, the detected
objects, the trajectory, and `/scensim/wheels` on Isaac; `logging.topics` replaces the list.
Any other file in the run directory (a pilot's or a judge's own output) is kept at `debug`, or
at `standard` and up when a judge names it in `reads`.

A run's `capture` sets what it records; `keep` reduces it per verdict once the verdict is
known ([The testbench file](project-file.md#logging)). A run that failed or ended in error, a
run a judge found `failed` or `error`, and a run with a contact of the ego in `contacts/` keep
everything, whatever `keep` says.

## `result.json`

Schema `swb.result/1`:

```json
{
  "schema": "swb.result/1",
  "run_id": "2026-11-14T10-21-07_cut_in_gap6_yield_r2",
  "verdict": "passed | failed | error",
  "reason": "first cause, one line",
  "scenario": {"file": "...", "sha256": "...", "parameters": {}, "repetition": 2,
               "seed": 1002},
  "ssv2": {"junit": "junit.xml", "failures": 0, "errors": 0, "message": "..."},
  "events": [{"t_sim": 18.4, "t_wall": "...", "type": "collision.vehicle",
              "entity": "ego", "other": "cut_in"}],
  "metrics": {"duration_s": 24.0, "route_completion": 0.62},
  "pilots": {"ego": {"kind": "autoware", "vehicle": "tesla_model3",
                     "inputs": ["sensors"], "version": "..."}},
  "provenance": {"scensim": "<commit>", "submodules": {}, "simulator": "carla",
                 "swb_lock": "sha256:...", "spec_hash": {"ego": "..."},
                 "host": "...", "gpu": "...", "stack_id": "...",
                 "launcher": "play_launch | ros2",
                 "carla": {"owner": "started", "version": "0.9.16", "argv": ["..."],
                           "quality": "epic", "args": [], "applied": true}}
}
```

`provenance.swb` is the commit the install was built from (`+dirty` when the checkout had
uncommitted changes then) and `provenance.built` when; `provenance.checkout` is the
checkout's commit at run time, present only when it differs. An install built without the
workspace's build recipe (a plain `colcon build`) records the checkout's commit at run time
as `scensim`.

`provenance.launcher` is the launcher the stack ran its launches with (the host setting
`launcher`, resolved), so runs under play_launch and `ros2 launch` can be told apart.

`provenance.carla` is the CARLA the run ran on (CARLA only). It holds:
- `owner`: `started` by the stack, or `external`;
- `version`;
- `argv`: the command line it was started with;
- `quality` and `args`: the effective [`carla:` settings](project-file.md#carla);
- `applied`: false when the stack did not start that CARLA, so they did not apply.

Event types: `collision.vehicle`, `collision.pedestrian`, `collision.static`, `red_light`,
`off_lane`, `stuck`, `route_complete`, `pilot.restart`, `stack.error`.

Metrics: `duration_s` (from SSv2's start to its end), and the bridge's frame budget for the
run. The bridge reports its time per frame every 200 frames (10 s at 20 Hz); a run's
metrics are those reports':

| Metric | |
|---|---|
| `frame_windows`, `frame_count` | the reports, and the frames they cover |
| `frame_p50_ms` | the bridge's own time per frame: the median of the reports' medians, weighted by frames |
| `frame_p95_ms`, `frame_max_ms` | the largest report's 95th percentile and maximum: never below the run's own |
| `tick_p50_ms`, `tick_p95_ms`, `tick_max_ms` | the same for the simulator's step |
| `frames_over_budget`, `ticks_over_budget` | frames over 25 ms of the bridge's time, steps over 100 ms |

The last frames of a run, fewer than 200, are in no report, so a run shorter than one
report has no frame metrics (they are absent, not zero).

## The timeline

`timeline.jsonl`: one JSON object per line, `{t_wall, t_sim, group, event, ...}`, ordered
by wall time: the run's start and end, each pilot's readiness, SSv2's milestones, judge
events, and every log line at WARN or above in any slice. `scensim runs show <id>`
prints it as a table; `runs why <id>` prints its first WARN or ERROR and its group.

A slice holds what the launcher wrote, and the two launchers lay it out differently.
play_launch writes `<time>/node/<name>/out` and `err` per process (`load_node/<name>/` for
a component in a process of its own), and its own lines to `console.log`. `ros2 launch`
writes everything a process prints (its standard output and error) to `console.log`,
each line led by `[<name>-<n>]`; beside it are rcl's `<executable>_<pid>_<ms>.log` files
and its own `<time>-<host>-<pid>/launch.log`, which hold copies of the console's lines.
The timeline reads each line once: from play_launch's node files and console, and from
the `ros2 launch` console, taking rcl's files and `launch.log` only where no `console.log` is beside
them. A process has the same name under either launcher: without the `-<n>`, the
`_<pid>_<ms>` or a `.py`, and a component in a `component_container` takes its node's
name, as play_launch names its process.

## A suite's table

`summary.csv` has one row per run: `run_id`, `scenario`, `params`, `repetition`, `seed`,
`verdict`, `reason`, `duration`, `judges`, `path`, `overload` (a run of `run --seats` during
which the host was overloaded, and why; empty otherwise), `seats` (how many seats were taking
work when the run started; 1 for a run without `--seats`). The merged `junit.xml` holds one test
suite per run with SSv2's testcases, plus a testcase standing for an `error` (or a
failure) SSv2 did not report.

`rollouts.csv` groups the runs per scenario and parameter set: `scenario`, `params`, `runs`,
`passed`, `failed`, `error`, `pass_rate`, `seeds` (as ranges), `duration_mean`,
`duration_sd`, `duration_min`, `duration_max`, `overloaded` (how many), `seats` (the seat
counts its runs ran at). Simulated durations grow with the seats sharing a host (a few
seconds per run at two seats, more in each seat's first run), so compare them only at one
seat count; a group whose runs ran at different counts lists them all (`1 2`), and the
summary prints a note naming it. `rollout_metrics.csv`
has one row per group and numeric metric of `result.json`: `metric`, `n`, `mean`, `sd`,
`min`, `max`. A scenario run more than once (`--repeat`, `--seeds`, a `--param` list) gets
the same files under `suites/rollouts-<scenario>/`.
