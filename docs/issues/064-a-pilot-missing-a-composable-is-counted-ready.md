# 064: A pilot with a composable that never loaded was counted ready

**Severity**: high. Every run on the stack timed out waiting for an engage that could not
come, and nothing named the cause. **Status**: fixed on main. Since issue 088, the stack keeper also restarts such a start (a LOAD_FAILED can be transient: here a GPU shared with another job), at most twice per pilot, then leaves it not ready, naming the composable, as before; the check reads the console from the pilot's last start only. **Found**: 2026-11-19, swb 133,
roadmap 088 (seats per host): a seat's Autoware started beside another user's GPU job.

## What happens

The ego's play_launch said:

```
WARN container:/pointcloud_container: control channel LOAD_FAILED for
  'composable:/perception/occupancy_grid_map/occupancy_grid_map_node': component_node failed:
  Exception loading 'autoware::occupancy_grid_map::PointcloudBasedOccupancyGridMapNode':
  cudaErrorMemoryAllocation (2)
WARN Startup complete with failures: ... composable 126/127 loaded
```

The stack counted the pilot ready all the same. Every run then started, and each one timed
out waiting for the ego to engage.

## Cause

Readiness (`ego_ready`) asks three things:

1. the ADAPI's operation-mode topic has a publisher;
2. planning publishes;
3. object recognition's inputs have publishers.

The occupancy grid map is none of these, and its container kept running. `ego_ready` also
asks play_launch's web API for crashed composables, but two things kept that from helping:

- a pilot's nested play_launch runs with `--disable-web-ui`, and the stack does not pass
  `--web-port`, so the question went to a port that does not answer;
- a composable that fails to load is in the state `LoadFailed`, not `Failed`.

The run's crash watch (`run.CrashWatch`) reads only the console written after the watch
starts. A stack pilot's start comes before that.

## Fix

`timeline.missing_composables(console)` names each composable that a pilot's launch declared
and did not load, with the reason. It reads the launch's current instance, after its last
start (a respawned pilot appends to the same console):

- under play_launch, the `LOAD_FAILED` lines. Once the `STARTUP_SUMMARY` line is written,
  its `composable_shortfall` decides, so a composable that a retry then loaded is not
  missing;
- under `ros2 launch`, launch_ros's "Failed to load node '<name>' ... in container" lines.

`scensim run` checks it before each pilot's readiness check. A pilot with a missing
composable ends the wait at once as not ready, with exit error, and the message names the
composable, the reason, and the console. The keeper does not mark that pilot ready. It says
why, once, and records a `pilot.missing` event.

## Test

`test_timeline.py`:

- 088's lines, before and after the summary;
- a retry that loaded the composable;
- a respawned pilot whose new start is clean;
- the `ros2 launch` form.

`test_run.py`: `wait_pilots_ready` with every topic up and one `LOAD_FAILED` returns
"pilot ego is not ready: its launch did not load
/perception/occupancy_grid_map/occupancy_grid_map_node (... cudaErrorMemoryAllocation)". On
a clean summary it returns ready.

On the real 088 console, `missing_composables` names occupancy_grid_map_node with
`cudaErrorMemoryAllocation`.
