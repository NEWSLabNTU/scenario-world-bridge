# safety_suite

**Goal**: run the Autoware Foundation's ODD use cases against Autoware on CARLA every
night, and tell a new failure from a known one.

| File | Holds |
|---|---|
| `scenarios/awf/UC-ACC-001-0001_Preceding_vehicle_partial_deceleration.yaml` | a car 30 m ahead slows at 1.5 m/s^2 to a lower speed; the ego must match it without a collision and without braking harder than 1.5 m/s^2. 27 variants (ego speed x car speed x its target) |
| `scenarios/awf/UC-AEB-001-0001_Standing-pedestrian.yaml` | a pedestrian stands in the lane ahead; the ego must stop short, braking no harder than 6 m/s^2. 9 variants (ego speed x distance) |
| `suites/nightly.yaml` | both, with Autoware's default lidar detector (clustering); nothing expected to fail |
| `suites/centerpoint.yaml` | both with the CenterPoint detector; the AEB run is expected to fail (a known issue: 2 of its 9 variants pass) |

Both use cases come from
[autowarefoundation/odd-usecase-scenario](https://github.com/autowarefoundation/odd-usecase-scenario)
(Apache License 2.0), adapted to this map: each file's header lists the changes. They are
TIER IV YAML scenarios: SSv2 expands each into its variants within one run, one testcase
per variant. Both are written for Town01.

What `swb.yaml` sets:

- `pilots.ego.lifetime: run`: Autoware starts afresh for each use case, so a suite can give
  it other arguments (`pilot_args`, as `centerpoint.yaml` does). A run lasts 11 to 35
  minutes, so the restart costs little.
- `logging.keep`: a passed run keeps its result only; a failed or errored one keeps its
  logs and its recording, for `ros2 run swb runs why`.
- `judges`: `physical_verdict` joins CARLA's own contact reports into the run's JUnit, so an
  ego that touched something SSv2 did not see fails. `ros2 run swb run` runs it after each
  passed or failed run; its verdict is in `runs/<id>/judges/physical_verdict.json`, on the
  run's line and in the suite's `summary.csv`, beside the scenario's own.

## Hardware

One NVIDIA GPU that runs CARLA 0.9.16 beside Autoware 1.5.0; CenterPoint needs TensorRT
and builds its engines on its first start (several minutes). About 0.1 GB of disk for the
map, and a few MB per kept run.

## Commands

```bash
ros2 run swb project new ~/safety_suite --from pkg://swb_examples/projects/safety_suite
cd ~/safety_suite
ros2 run swb assets fetch
ros2 run swb project check
```

With CARLA 0.9.16 serving, bring up the stack in one terminal (the ego starts with each run):

```bash
play_launch launch swb_launch swb.launch.xml
```

Then, in a second terminal in the project:

```bash
ros2 run swb run suites/nightly.yaml
ros2 run swb run suites/centerpoint.yaml
ros2 run swb runs summary
```

On two hosts with the same project and lock, `--shard 1/2` and `--shard 2/2` split a suite
between them.

## Expected result

`nightly.yaml`: two lines `passed  <run id>  <run directory>` and exit status 0, in about
45 minutes (measured on 2026-10-10: ACC 27/27 in 2098 s, AEB 9/9 in 663 s, CARLA at Epic
quality, 32 cores, one RTX GPU). `centerpoint.yaml`: ACC `passed`, AEB
`failed (expected failed)`, exit status 0. When the AEB failure is fixed, that line reads
`passed (expected failed)` and the suite exits 1: drop the expectation then.

`assets/`, `state/` and `runs/` are made on each host and are not committed.
