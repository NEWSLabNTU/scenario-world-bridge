# 088: An Autoware start hangs, not fails, when a load_node response is lost

**Severity**: low so far. It was seen once, and the rerun passed. A stack that never becomes ready waits
out the whole ready timeout (1800 s under T3) before anything says why. **Status**: fixed (scensim), under both launchers: `ros2 launch`, where it happened, and play_launch. Upstream, play_launch 0.41.2 (on PyPI) fixes it for play_launch (#0103): a composable with no reply, ComponentEvent or ListNodes answer within `composable_node_loading.load_total_budget_secs` (600 s by default) fails as `container-busy: unresponsive for 600s while loading <name>`, logged LOAD_FAILED and named in STARTUP_SUMMARY, and nothing is resent. scensim's guard stays as the faster backstop, and the minimum play_launch version is unchanged. **Found**:
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

## Fix

241's start ran under `ros2 launch`: its `stack.json` says `"launcher": "ros2"`, and the console is
launch_ros's. play_launch, which the recipes use, can stall the same way (below), so both are
covered.

**Under `ros2 launch`**, the pilot's console has both sides of each load. The container logs every
load it takes (`Instantiate class: ...`, with its time), and launch_ros logs every reply it got
(`Loaded node '<n>' in container '<c>'`, or `Failed to load node`). `timeline.stalled_loads(console)`
names each container with more loads taken than answered, in two cases:

- its reply was lost: the container logged `failed to send response to <c>/_container/load_node`,
  and `LOST_GRACE` (30 s) has passed;
- or a load stays unanswered for `LOAD_STALL` (600 s), timed from the container's last load, while
  the container's process is idle. A constructor still building answers only when it ends, and a
  CenterPoint TensorRT build took 3-11 minutes on a loaded host before 10.8's cache (065 measured
  up to 240 s). Such a build burns a core, so the container's CPU time over 2 s (from `/proc`, its
  pid from the launch's `process started with pid` line) tells a busy container from a blocked one.

Which load is unanswered is not known. A container's lines reach the console late, after the
replies to later loads: in 241's console, glog_component's reply was the lost one, logged before
behavior_path_planner's load, which was answered. So the message gives the counts and the classes
loaded there, not a guess.

**Under play_launch** (0.35.1 on 139), composables load through play_launch's own container, and it
logs no reply per load. With the container frozen (SIGSTOP), two endings were seen live:

- frozen before its first load: play_launch wrote a STARTUP_SUMMARY at 30 s naming all three
  composables missing, which is 064's check;
- frozen after it was up: `LoadNode service call timed out after 30s`, then `LoadNode response lost
  for composable:<n> -- deferring to ComponentEvent`, then `Startup: ... composable 0/3 loaded (3
  pending)` every 10 s for 12 minutes, with no LOAD_FAILED and no summary, its `busy_budget=600s`
  notwithstanding. Nothing caught that before.

`stalled_loads` reads that second ending: its loads still pending with the loaded count unchanged
`PL_LOST_GRACE` (120 s, beyond play_launch's own 60 s retry) after a lost reply, or `LOAD_STALL`
without one. Once play_launch says `Startup complete` or writes its summary, this check stays out
and 064's check takes over. A lost load that play_launch recovers by resending (seen on a T3 start
on 139) is not a stall.

**What the result does:**

- **A stack pilot**: the stack keeper restarts it (`Pilots.restart`, event `pilot.stalled`, naming
  the containers), as it would a pilot that ended. It does the same for a start that 064's check
  finds with composables missing, since a frozen play_launch start ends there. At most
  `LOAD_RESTARTS` (2) restarts per pilot, so a load that can never succeed (064's CUDA OOM) does not
  restart for ever. After those the pilot stays not ready with the reason, as before, said once.
  The count starts again once the pilot is ready. The checks read the console only from the
  pilot's last start (its size then): a restarted pilot's console keeps the old launch's lines.
- **A run's pilot** (`lifetime: run`): nothing restarts it. The run ends `error` at once ("its
  launch is stuck loading composables into ...").
- **`ego_ready`'s hint**: when play_launch reports no crashed composable, it now also names this
  case.

Under play_launch the 600 s bound has no CPU guard: play_launch's container is its own, and its
console names no pid per container. A build that keeps play_launch's loaded count unchanged for
600 s, with no lost reply, would be restarted (at most twice). Not seen, and not measured: whether
play_launch's own `busy_budget` (600 s) fails such a build first is not known.

The named limit: a container frozen before it logs taking the load, under `ros2 launch`. The
console then shows nothing outstanding. Under play_launch, the same freeze ends in its summary.

No change to launch_ros or play_launch.

## Test

- `test_timeline.py`, `ros2 launch`: the lost reply (named after the grace, not within it), the
  long unanswered load, every load answered, a stall before the pilot's restart and one in an
  earlier launch instance (neither is the current start's).
- `test_timeline.py`, play_launch: lost replies with the loads pending (named after 120 s, not at
  119 s), loads pending 600 s without one, a recovered or completed start (no stall), a stall
  before the restart.
- `test_timeline.py`, the CPU guard: a container busy past `LOAD_STALL` is not stalled, an idle one
  is, and a lost reply is named either way.
- `test_stack.py`: a `ros2 launch` start with a lost reply is restarted and its next start is
  ready. A play_launch start summarised with composables missing is restarted once, and the old
  summary is not read again. A start that never loads is restarted twice, then left not ready,
  said once.
- `test_run.py`: a run's pilot stuck that way ends the run `error` well inside the ready timeout,
  naming the container.
- On 241's console (NAS copy): the planning container is named 30 s after the lost reply, as
  `1 of 2 loads answered; loaded there: BehaviorPathPlannerNode, GlogComponent`, and nothing at 10
  s. Two healthy Autoware starts on 139 (`ros2 launch`: 107 loads taken, 127 replies) and one T3
  play_launch start with a lost and resent load are not stalled, even with every threshold expired.
- Live, `ros2 launch` (139, 2026-11-21): rclcpp_components' container with composition's Talker,
  Listener and Server. For this test only, an `LD_PRELOAD` interposer made the container's first
  `rcl_send_response` return RCL_RET_TIMEOUT. The container logged 241's line (`failed to send
  response to /planning/stall_container/_container/load_node (timeout): error not set`), the launch
  waited, and Listener and Server were never requested. `stalled_loads` named the container, and
  `missing_composables` named nothing. Without the interposer, all 3 loaded and nothing was named.
  As a stack pilot, the first start was restarted 7 s after it began (grace 5 s in the test), and
  the second was ready at 10.6 s.
- Live, play_launch 0.35.1: the same launch, its container frozen within play_launch's warmup. On
  its own, it waited 12 minutes as above, and `stalled_loads` names the container at 125 s and not
  at 119 s. As a stack pilot (with play_launch's summary at 30 s that time), the keeper restarted it
  at 34 s and the second start was ready at 39 s.
- After each live run, nothing was left: no `component_container*` or `play_launch` process, the
  SIGSTOPped containers included (the keeper's stop takes STOP_GRACE + 10 s on them, then SIGKILL).
