# 065: A pilot is counted ready while its composables are still constructing

**Severity**: medium. A run starts on an Autoware that has not finished loading, and a later
failure (a composable that never loads) is not seen by 064's check. **Status**: open.
**Found**: 2026-11-19, swb 243, roadmap 083 step 6 (T3 on CARLA 1.0, then a live check of
safety_suite on CARLA 0.9.16).

## What happens

`scensim run` on safety_suite (an `autoware` pilot with `lifetime: run`) on host 243, whose
host settings had no `autoware.tensorrt_lib`:

```
15:33:51  run.start, pilot.start ego
15:34:01  pilot.ready ego                      (10 s after the start)
15:34:24  WARN container:/perception/traffic_light_recognition/camera6/traffic_light_node_container:
            control channel LOAD_FAILED for
            'composable:/perception/traffic_light_recognition/camera6/classification/...'
...       Component 'autoware::traffic_light::TrafficLightFineDetectorNode' constructing for
            240s (pid 1877703, alive)
```

The fine detectors were building their TensorRT engines (the host's TensorRT, not 10.8.0, so
every start rebuilds them; see CLAUDE.md, "Autoware's TensorRT is 10.8.0"). The run was
already under way: SSv2 had spawned the ego and was driving the scenario against an Autoware
that was still starting.

## Cause

Readiness asks whether the pilot's topics are up (`ego_ready`) and, since 064, whether its
launch reports a composable that did not load (`timeline.missing_composables`). Both can be
true long before the launch has finished:

- `ego_ready`'s topics come from nodes that start in seconds;
- `missing_composables` finds a `LOAD_FAILED` only once it is written, and before the
  `STARTUP_SUMMARY` line nothing says the load is complete. A composable that is still
  constructing is neither loaded nor failed.

So 064's check runs, finds nothing missing, and the pilot is ready; the `LOAD_FAILED` that
comes 23 s later is not read again for that run.

## Fix (proposed)

Readiness waits for the launch's composable-load summary: under play_launch, the
`STARTUP_SUMMARY` line (its `composable_shortfall` then decides, as 064 reads it); under
`ros2 launch`, which writes no summary, as now. Until the summary, the pilot is not ready and
the keeper and `scensim run` say what it waits for ("waiting for its launch's composables:
N of M loaded"). The ready timeout (`--ready-timeout`, default 600 s) bounds the wait; a TRT
build longer than that fails the readiness with that reason rather than a run that drives a
half-started Autoware.

## Test (proposed)

`test_timeline.py`: a console with `LOAD_FAILED` lines and no summary yet is "not complete";
with the summary, complete, its shortfall deciding. `test_run.py`: a pilot whose console has
no summary is not ready until one appears.
