# safety_suite

**Goal**: run the Autoware Foundation's ODD use cases against Autoware on CARLA every
night, and tell a new failure from a known one.

| File | Holds |
|---|---|
| `scenarios/awf/UC-ACC-001-0001_Preceding_vehicle_partial_deceleration.yaml` | a car 30 m ahead slows at 1.5 m/s^2 to a lower speed; the ego must match it without a collision and without braking harder than 1.5 m/s^2. 27 variants (ego speed x car speed x its target) |
| `scenarios/awf/UC-AEB-001-0001_Standing-pedestrian.yaml` | a pedestrian stands in the lane ahead; the ego must stop short, braking no harder than 6 m/s^2. 9 variants (ego speed x distance) |
| `suites/nightly.yaml` | both, with Autoware's default lidar detector (clustering); nothing expected to fail |
| `suites/centerpoint.yaml` | both with the CenterPoint detector |

Both use cases come from
[autowarefoundation/odd-usecase-scenario](https://github.com/autowarefoundation/odd-usecase-scenario)
(Apache License 2.0), adapted to this map: each file's header lists the changes. They are
TIER IV YAML scenarios: SSv2 expands each into its variants within one run, one testcase
per variant. Both are written for Town01.

What `testbench.yaml` sets:

- `pilots.ego.lifetime: run`: Autoware starts afresh for each use case, so a suite can give
  it other arguments (`pilot_args`, as `centerpoint.yaml` does). A run lasts 11 to 35
  minutes, so the restart costs little.
- `logging.keep`: a passed run keeps its result only; a failed or errored one keeps its
  logs and its recording, for `scensim runs why`.
- `judges`: `physical_verdict` joins CARLA's own contact reports into the run's JUnit, so an
  ego that touched something SSv2 did not see fails. `scensim run` runs it after each
  passed or failed run; its verdict is in `judges/physical_verdict.json` in the run directory, on the
  run's line and in the suite's `summary.csv`, beside the scenario's own.

## Hardware

One NVIDIA GPU that runs CARLA 0.9.16 beside Autoware 1.5.0; CenterPoint needs TensorRT
and builds its engines on its first start (several minutes). About 0.1 GB of disk for the
map, and a few MB per kept run.

## Commands

Copy the example into a workspace as a testbench, build it, fetch its assets, check it:

```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
scensim testbench new safety_suite --from pkg://scensim_examples/projects/safety_suite
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch safety_suite
scensim testbench check safety_suite
```

CARLA 0.9.16 must be serving on the host settings' `carla.host` and `carla.port`
(default `localhost:2000`), or `carla.root` must name its install so that the stack
starts it. The ego starts with each run.

```bash
scensim run safety_suite                      # the default suite, nightly
scensim run safety_suite/centerpoint
scensim runs summary --testbench safety_suite
```

On two hosts with the same testbench and lock, `--shard 1/2` and `--shard 2/2` split a suite
between them.

## Expected result

`nightly.yaml`: two lines `passed  <run id>  <run directory>` and exit status 0, in about
45 minutes (measured on 2026-10-10: ACC 27/27 in 2098 s, AEB 9/9 in 663 s, CARLA at Epic
quality, 32 cores, one RTX GPU). `centerpoint.yaml`: the same two `passed` lines, in about
46 minutes (measured on 2026-11-16: ACC 27/27 in 2126 s, AEB 9/9 in 631 s); without the
TensorRT 10.8 runtime each Autoware start first rebuilds CenterPoint's engines.

What each host makes for the testbench (the fetched assets' links, `state/`, the runs)
is kept outside the package, under the log root; nothing of it is committed.
