# 088: An Autoware start hangs, not fails, when a load_node response is lost

**Severity**: low so far. It was seen once, and the rerun passed. A stack that never becomes ready waits
out the whole ready timeout (1800 s under T3) before anything says why. **Status**: open. **Found**:
2026-11-20, by 241, in the T3 Isaac suites for issue 086 (`isaac_rough_road` rough, its first
start).

## What happens

The `isaac_rough_road` stack started at 23:53:11. The world server was ready in 42 s and the bridge
served SSv2. The ego's vehicle agent reached INITIALIZING at 23:53:35, as in the suites before it.
After that nothing happened. There was no `pilot.ready` event, and the run printed nothing for 15
minutes. `ego_ready`, run by hand in the ego's domain, said:

```
[ego-health] not ready: the ADAPI is up but planning is not -- no publisher on
/planning/scenario_planning/lane_driving/behavior_planning/path. A composable died inside its
container ...
```

No composable had died. The ego's console (`proc/pilot-ego/console.log`) shows the planning
container `behavior_planning_container` (component_container_mt-53) loading a composable. Its reply
to the launch's `load_node` request then failed:

```
[component_container_mt-53] [WARN] [1795190012.902180763] [planning.scenario_planning.lane_driving.
behavior_planning.behavior_planning_container.rclcpp]: failed to send response to
/planning/scenario_planning/lane_driving/behavior_planning/behavior_planning_container/_container/
load_node (timeout): error not set
```

launch_ros (Humble) loads a container's composables one request at a time, and waits for each
response with no time limit. It never got this one, so the composables after it in that sequence were
never requested. The path above has no publisher, so planning never came up. When the run was
interrupted at 00:08, the launch logged the wait it had been in all along:

```
[WARNING] [launch_ros.actions.load_composable_nodes]: Abandoning wait for the '/planning/
scenario_planning/lane_driving/behavior_planning/behavior_planning_container/_container/load_node'
service response, due to shutdown.
```

The rerun of the same suite, on the same build, became ready in the usual time and passed.

## Why nothing caught it

scensim's missing-composable check (issue 064, `timeline.missing_composables`) reads the console for a
load that *failed*: play_launch's `LOAD_FAILED`, or launch_ros's `Failed to load node`. Here no load
failed; the launch was still waiting for one. The stack keeper therefore kept polling `ego_ready`,
which said "not ready" each time, until the ready timeout. Its own message also blames a crashed
composable, which was wrong here.

## Options

1. The stack keeper treats a pilot whose console logged `failed to send response to .../load_node`
   while it is still not ready, a minute or so later, as a failed start. It names the container,
   then restarts the pilot (`lifetime: stack` already restarts a pilot that exits).
2. `ego_ready`'s hint names this case, not only a crashed composable.
3. Under play_launch (0.30 or later, not this host's 0.18.3) composables load through play_launch's
   own loader. Whether it has the same wait is not known.

## Evidence

- The stack's directory: `/var/tmp/aeon/isaac-ex-021/logs/isaac_rough_road/stacks/2026-11-20T23-53-11_seat0/`
  on 241.
- A copy is on the NAS, under logs `2026 SSv2 on CARLA/2026-11-21 086 aib tensor velocities
  (241)/088-rough-start/`: `pilot-ego-console.log`, `events.jsonl` and `stack.json`.
- The T3 log, which shows the run interrupted (`scensim run: interrupted`): `run.log`, in the folder
  above that.
