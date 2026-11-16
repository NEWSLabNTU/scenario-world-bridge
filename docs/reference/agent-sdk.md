# The agent SDK

An **agent** connects an autopilot to the scenarios run on scenario-world-bridge. The
scenario simulator (SSv2) never talks to your autopilot directly: it talks to the **agent
relay** on the simulation side, and the relay talks to the agent registered under the
scenario entity's name (`ego`, `bg_av_1`, ...), over plain TCP. The agent turns the
relay's commands (reset, drive to a goal, keep under a speed limit, stop) into whatever
your autopilot understands, and reports back what the autopilot is doing.

Every agent is a Python class, `swb_agent.Agent`, in your own project. Nothing in this
workspace has to change, and the agent can run on another host than the simulator: it only
needs to reach the relay's port (default 5560).

```
 simulation side                                    vehicle side (any host)
 SSv2 ── scenario-world-bridge ── CARLA              your autopilot
   └──── agent relay :5560 ◀──── TCP ────▶ your agent (swb_agent.Agent)
```

Every pilot of a project is such an agent: `autoware` pilots run swb's vehicle agent,
`carla_basic` the SDK's BasicAgent adapter, `simple` the SDK's `simple_pilot`, `command`
pilots a program of yours on this SDK. Which kind to use, and how the stack starts it: [Your autopilot as a
pilot](../how-to/your-autopilot.md). This page is the SDK itself.

The commands below assume the environment file of [Install](../getting-started/install.md)
is sourced, so `ros2 run swb_agent ...` finds the SDK's commands. (The bare `swb-agent` is
not on `PATH`: colcon installs it under `lib/swb_agent`.) Examples use seat 0's relay,
`tcp://localhost:5560`; `ros2 run swb seat show` gives yours.

**Domains and DDS for ROS agents.** The agent reaches the relay over TCP, so its ROS
domain does not matter to swb. A ROS agent (`ros_adapter`, `AutowareAgent`, your node) must
however share a domain, the RMW and `CYCLONEDDS_URI` with the autopilot it drives: as a
pilot it runs in the pilot's domain; by hand, `eval "$(ros2 run swb seat env <k>)"` first.
Why: [ROS domains and DDS](../explanation/domains-and-dds.md).

## Three ways to write one

| | When | You write |
|---|---|---|
| **Your own class** | any autopilot: a ROS stack, a learned policy, a simulator API, anything in Python | a subclass of `swb_agent.Agent` |
| **`ros_adapter` + YAML** | a ROS 2 autopilot whose steps are single topics, services and states | `agent.yaml` only |
| **A subclass of ours** | the YAML covers most of it but one step needs code | a `RosAdapter` subclass overriding that hook |

All three are `swb_agent.Agent` subclasses, and `ros2 run swb_agent swb-agent check` tests
any of them.

## Your own class

Start from a skeleton that already passes the conformance check:

```bash
ros2 run swb_agent swb-agent new my-pilot
cd my-pilot
ros2 run swb_agent swb-agent check my_pilot.agent:MyPilotAgent
```

`my_pilot/agent.py`:

```python
import swb_agent

class MyPilotAgent(swb_agent.Agent):
    def on_reset(self, ep):             # a scenario starts: forget the last one
        self.autopilot.relocalize(ep.pose)

    def on_goal(self, goal, waypoints): # drive to goal through waypoints
        self.autopilot.drive_to(goal, waypoints)
        self.report(swb_agent.DRIVING)

    def on_speed_limit(self, mps):      # None: no limit
        self.autopilot.max_speed = mps

    def on_stop(self):                  # stop, and hold until the next goal
        self.autopilot.stop()

if __name__ == "__main__":
    swb_agent.main(MyPilotAgent)
```

As a pilot, `swb.yaml` names it (`{kind: command, run: [python3, -m, my_pilot]}`) and swb
gives it `--relay`, `--entity`, `--seed` and `--run-dir`. By hand, for an entity whose
pilot is `kind: none`:

```bash
python3 -m my_pilot \
    --relay tcp://localhost:5560 \
    --entity ego \
    --config agent.yaml
```

On CARLA the agent may run on another host: give `--relay` the stack's host.

### Hooks

Override the ones your autopilot can honour. A hook returns normally for success and
raises `swb_agent.Failed("why")` to fail the command; a hook you do not override is
answered "unsupported", and a scenario that needs it fails saying so. Hooks run one at a
time, on one thread; do not block in one for longer than its command allows (below).

| Hook | Called when | Must | Time allowed |
|---|---|---|---|
| `on_reset(ep)` | a scenario starts (SSv2 initializes the entity) | drop goal, plan and controller state; re-localize at `ep.pose`; return once ready for a goal | the relay's `reset_timeout` (60 s) |
| `on_goal(goal, waypoints)` | the scenario assigns a route | start planning and driving | 9 s |
| `on_speed_limit(mps)` | a speed limit is set or lifted (`None`) | apply it | 2.5 s |
| `on_stop()` | the scenario disengages the vehicle | stop, hold until the next goal | 2.5 s |
| `on_clear_goal()` | the route is cleared | drop the goal (default: `on_stop`) | 2.5 s |
| `on_episode_end(ep, result)` | the verdict is reported (`result["passed"]`, `["failures"]`) | nothing; for logs, bags, learning | 2.5 s |
| `on_restart()` | a restart is asked for (capability `restart`) | restart the autopilot in place | |
| `on_cooperate(module, command, uuid)`, `on_cooperate_auto(module, enable)` | the scenario drives Autoware's RTC | | 2.5 s |
| `close()` | the agent shuts down | release what it holds | |

Poses are dictionaries `{x, y, z, qx, qy, qz, qw}` in the map frame (the same frame as the
scenario's Lanelet2 map). `ep` (`swb_agent.Episode`) carries `pose` and, from the relay,
an `episode` id; `scenario`, `map` and `parameters` are empty (the relay does not fill
them).

`self.config` is the `--config` file (JSON or YAML) as a dict, `self.entity` the entity
name, `self.seed` and `self.run_dir` the run's seed and directory (`--seed`, `--run-dir`;
None when not given), `self.log` a logger.

### Reporting state

The scenario reads your autopilot's state from what the agent reports. Call
`self.report(...)` from any thread whenever something changes; only the fields you pass
change:

```python
self.report(swb_agent.ARRIVED)                       # phase
self.report(pose={"x": ..., "y": ..., ...})          # the autopilot's own pose estimate
self.report(fault=swb_agent.MINIMAL_RISK_MANEUVER, fault_behavior="EMERGENCY_STOP")
self.report(detail="waiting for localization")       # free text, in the relay's log
```

| Phase | Means | Set by |
|---|---|---|
| `INITIALIZING` | not ready (re-localizing) | the SDK while `on_reset` runs |
| `IDLE` | ready for a goal | the SDK after `on_reset`, or you |
| `PLANNING` | goal received, no route yet | the SDK after `on_goal`, or you |
| `READY` | route set, about to drive | you |
| `DRIVING` | driving | **you** |
| `ARRIVED` | at the goal | **you** |
| `STOPPED` | stopped by command | the SDK after `on_stop` |

The SDK sets a phase after a hook only if the hook reported none. Reporting `ARRIVED` is
how a scenario's goal condition is met, so an agent that never reports it never finishes a
scenario that waits for arrival.

### What the SDK does for you

Registration with the relay, reconnecting when it goes away (every 1-5 s), state pushes at
5 Hz (which double as heartbeats), one command at a time with the state the command
produced sent before its answer, ignoring the scenario's retries of a command already done,
and talking to an older relay (protocol v1) when that is what it finds.

## `ros_adapter`: a ROS 2 autopilot from YAML

When each step of your autopilot is one topic or one service, no Python is needed:

```bash
ros2 run swb_agent ros_adapter --config agent.yaml
```

```yaml
entity: ego
relay: tcp://localhost:5560
goal:        {publish: /planning/goal, type: geometry_msgs/msg/PoseStamped, from: goal}
speed_limit: {publish: /planning/max_velocity, type: std_msgs/msg/Float32, from: mps,
              "null": 1000.0}
stop:        {call: /autopilot/stop, type: std_srvs/srv/Trigger}
reset:       [{call: /autopilot/reset, type: std_srvs/srv/Trigger},
              {publish: /initialpose, type: geometry_msgs/msg/PoseWithCovarianceStamped,
               from: pose},
              {wait_phase: IDLE, timeout: 30}]
phase:       {subscribe: /autopilot/state, type: std_msgs/msg/String,
              map: {idle: IDLE, driving: DRIVING, arrived: ARRIVED, stopped: STOPPED}}
pose:        {subscribe: /localization/pose, type: geometry_msgs/msg/PoseStamped}
```

Each hook (`reset`, `goal`, `speed_limit`, `stop`, `clear_goal`, `episode_end`, `restart`)
is one action or a list run in order. A hook not in the file is unsupported; `reset` is
only offered to the relay when it is mapped.

| Action | Does |
|---|---|
| `publish: TOPIC, type: T` | publishes one message (reliable, transient local) |
| `call: SERVICE, type: T` | calls the service; fails if the response has a `success` field that is false; `timeout` (5 s) |
| `wait_phase: PHASE, timeout: S` | waits until the `phase` subscription reports PHASE (needs `phase:`) |
| `sleep: S` | waits |

Common keys: `fields: {...}` static values, as `ros2 topic pub` takes them; `from: pose |
goal | mps` the episode's start pose, the goal or the speed limit; `field: a.b` where it
goes (`name[]` appends to a sequence, e.g. `field: pose[]` for
`autoware_adapi_v1_msgs/srv/InitializeLocalization`); `"null": value` what to send when a
limit is lifted (without it, nothing is sent). `phase.field` names the subscribed message's
field (default `data`); `pose` takes `PoseStamped`, `PoseWithCovarianceStamped` or
`Odometry`.

### One step in code

Subclass `swb_agent.ros_adapter.RosAdapter`, override the hook, and run the configured
actions with your values:

```python
from swb_agent.ros_adapter import RosAdapter

class GoalAhead(RosAdapter):
    def on_goal(self, goal, waypoints):
        goal = {**goal, "x": goal["x"] + 10.0}           # your logic
        self.run_hook("goal", goal=goal, waypoints=waypoints)
```

Any class can get a ROS node the same way: mix in `swb_agent.ros.RosNode` and call
`self.start_node("name")`; `self.node` is spun on its own thread and stopped in `close()`.

## Autoware: `AutowareAgent`

The stack's default ego runs `swb_agent.adapters.autoware.AutowareAgent` (through
`vehicle_agent.launch.xml`):
Autoware's ADAPI as an agent. At a reset it stops, drops the route, waits for Autoware's
services, then re-localizes at the scenario's pose (two fresh LiDAR scans, initialize
localization, wait until the estimate agrees within 1 m / 0.2 rad); on a goal it sets the
route and engages once Autoware offers autonomous mode. On its own, beside an Autoware you
started yourself in its domain:

```bash
ros2 run swb_agent autoware_agent \
    --relay tcp://localhost:5560 \
    --entity ego \
    --config agent.yaml
```

`agent.yaml` takes `use_sim_time` (true for a simulator) and the bounds: `ready_timeout`
(60 s), `scan_wait` (10), `scans_required` (2), `converge_timeout` (30),
`initialize_attempts` (5), `engage_retry` (3), `settle` (2). An initialize
found unavailable (Autoware's services restarting mid-reset, e.g. a healer restarting a hung
ADAPI container) is not an attempt: the agent waits for the services, up to
`ready_timeout`. `swb_launch vehicle_agent.launch.xml` takes the same file as
`agent_config:=agent.yaml`, and an `autoware` pilot as `args: {agent_config: ...}`. Each step is a method to override in a subclass:

```python
from swb_agent.adapters.autoware import AutowareAgent

class TrustTeleport(AutowareAgent):
    # e.g. a simulator that places the vehicle exactly
    def relocalize(self, pose):
        ...  # also: wait_ready(timeout), set_route(...), engage()
```

## The BasicAgent adapter: `carla_basic`

The SDK ships a complete agent that needs no autopilot at all: `swb_agent.adapters.carla_basic`
drives the scenario's vehicle with CARLA's own `BasicAgent` (PythonAPI `agents` package).
It is what the `carla_basic` pilot runs ([Tutorial 4](../tutorials/04-several-pilots.md)),
and a worked example of an agent. It needs CARLA's Python API: the `carla` package and the
PythonAPI's `agents` directory with its requirements (`pip install carla==0.9.16 networkx
shapely`).

By hand, as entity `ego` (any host that reaches CARLA and the relay), with a
`carla_basic.yaml`:

```yaml
# where `agents/` is
carla_pythonapi: /path/to/CARLA_0.9.16/PythonAPI/carla
# when the scenario sets no limit
target_speed_kmh: 30
```

```bash
ros2 run swb_agent carla_basic \
    --relay tcp://localhost:5560 \
    --entity ego \
    --config carla_basic.yaml
```

The agent logs each command: `reset` (it finds the vehicle the bridge spawned, by its CARLA
`role_name`, `hero` for the ego, else the entity's name), `set_goal` (BasicAgent plans
through the route), `ARRIVED`, `stop`.

How it behaves, so you can write your own the same way (the module
`swb_agent.adapters.carla_basic`):

- **It never touches the simulation beyond its vehicle.** The bridge ticks CARLA in
  synchronous mode; the agent never ticks, never changes settings, never spawns or destroys
  anything, and applies control only to its own vehicle. It follows the world's ticks
  (`world.on_tick`), and the tick callback only wakes its control thread: a synchronous
  server waits for its subscribers, so a slow callback would slow the whole simulation.
- **It does not outlive the server.** When CARLA stops answering, it exits with status 3; a
  CARLA client left connected to a dead server can hang the next one on the same port. Run
  it under `ros2 run swb_agent swb-agent supervise` or anything that starts it again.
- **Reset** finds the vehicle again (a new actor every scenario), drops the route and
  brakes; **goal** plans and drives; **speed limit** sets BasicAgent's target speed;
  **stop** brakes and holds; `ARRIVED` when BasicAgent's route is done.
- **Checked without CARLA**: with a `fake.yaml` that holds `world: fake`,

  ```bash
  ros2 run swb_agent swb-agent check \
      swb_agent.adapters.carla_basic:CarlaBasicAgent \
      --config fake.yaml
  ```

  runs the same class over a kinematic stand-in.

Config keys: `carla_host` (localhost), `carla_port` (2000), `role_name` (hero),
`carla_pythonapi` (default `<carla.root>/PythonAPI/carla` from the host settings), `target_speed_kmh` (30),
`ignore_traffic_lights` / `ignore_vehicles` (false: BasicAgent's own hazard stops),
`find_timeout` (30 s for the vehicle to appear at a reset), `world` (`carla` | `fake`).

## The simple autopilot: `simple_pilot`

`swb_agent.adapters.simple_pilot` is a small autopilot of its own, and the SDK's example of
one that drives through the [vehicle interface](../how-to/your-autopilot.md#the-vehicle-interface)
instead of a simulator's API. It needs no Autoware and runs on CARLA and Isaac Sim alike.
It is what the `simple` pilot runs (the `simple_drive` example project).

- **Route.** On a goal it loads the scenario's Lanelet2 map (`map_path`, its
  `lanelet2_map.osm` placed by the map's `map_projector_info.yaml`, as Autoware and the
  scenario simulator place it) and routes from the car's lane to the goal's, through the
  waypoints, along the lanes (lanelet2's routing, lane changes only where no route keeps
  the lane). A goal on no lane, or against its direction, fails `set_goal` with the reason.
- **Drive.** Pure pursuit steers the road wheels toward a point ahead on the lanes' centre
  line; a speed PID holds the lowest of the lane's speed limit, the scenario's limit,
  `max_speed_kmh` and what each curve ahead allows, and brakes to stop at the goal.
- **Publish.** `/control/command/control_cmd` (steering, acceleration, target speed), gear
  `DRIVE`, indicators and hazards `NO_COMMAND`, at `rate_hz` (30) of simulation time.
- **Pose.** The car's true pose, `odom_topic`: CARLA's `/carla/ground_truth/odom`, Isaac's
  `/ground_truth/odom` (the stack turns it on for a `simple` pilot). It does not localize.
- **It ignores traffic.** No traffic lights, no other road users. For a car that obeys the
  road, use Autoware.

By hand, in the vehicle side's ROS domain, with a `simple_pilot.yaml`:

```yaml
map_path: /path/to/maps/Town01
sim: carla
max_speed_kmh: 30
```

```bash
ros2 run swb_agent simple_pilot \
    --relay tcp://localhost:5560 \
    --entity ego \
    --config simple_pilot.yaml
```

On CARLA the vehicle bridge attaches only once the car's description is published (see
[What CARLA's vehicle bridge waits for](../how-to/your-autopilot.md#what-carlas-vehicle-bridge-waits-for));
the `simple` pilot's launch starts it.

Config keys: `map_path`, `sim` (`carla` | `isaac`), `odom_topic` (by `sim`), `vehicle`
(tesla_model3: its spec gives `wheel_base` and `max_steer_angle`, which the config may set
itself), `rate_hz` (30), `max_speed_kmh` (50), `default_speed_limit_kmh` (30, for a lane
without one), `lateral_accel` (2.0 m/s^2 in curves), `accel_max` (1.5), `decel` (2.0),
`brake_max` (5.0), `kp` (0.8), `ki` (0.15), `lookahead_min` (3.5 m), `lookahead_time`
(0.6 s), `lookahead_max` (20 m), `arrive_tolerance` (1.0 m), `pose_timeout` (45 s at a
reset), `route` (`auto` | `lanelet` | `direct`), `world` (`ros` | `fake`).

With `world: fake` (and no map: straight lines between the points) it drives a kinematic
car without ROS, so `ros2 run swb_agent swb-agent check
swb_agent.adapters.simple_pilot:SimplePilotAgent --config fake.yaml` checks it as above.

## Checking an agent

`ros2 run swb_agent swb-agent check` runs your class against a fake relay (no CARLA, no
SSv2, no autopilot beyond what your class starts) through a scripted episode: register, reset, goal, speed
limit, stop, episode end, a second reset, a dropped connection. It prints each rule with
what it saw and exits non-zero on a failure:

```
$ ros2 run swb_agent swb-agent check my_pilot.agent:MyPilotAgent
  PASS  registers within 5 s
  ...
  FAIL  reset: IDLE before the reply -- phase INITIALIZING
  ...
FAIL: 14/16 rules
```

`--config agent.yaml` passes a config, `--reset-timeout S` (30) bounds the reset, `-v`
shows the SDK's log. A file works as well as a module:
`ros2 run swb_agent swb-agent check agent.py:MyAgent`.
Passing the check means the agent follows the protocol; whether it drives well is the
scenarios' to say.

## Between scenarios: reset

At the start of every scenario the agent's `on_reset` runs, with the pose the scenario
places the vehicle at. The scenario simulator waits while the agent reports `INITIALIZING`,
and goes on once it reports `IDLE`, so a reset may take as long as the autopilot needs to
re-localize, up to the relay's `reset_timeout` (default 60 s). A reset that fails or runs
out of time fails that scenario.

### When a reset fails: restart

A reset is the cheap way between scenarios; when it fails, the autopilot can be restarted.
A pilot the stack starts is respawned when it exits. An agent you start yourself (`kind:
none`) needs a supervisor, or must restart itself (`on_restart`, capability `restart`).

### `swb-agent supervise`

```bash
ros2 run swb_agent swb-agent supervise \
    --entity ego \
    --autopilot "play_launch launch my_stack bringup.launch.py" \
    --agent "python3 -m my_pilot --relay tcp://localhost:5560 --entity ego" \
    --relay tcp://localhost:5560 \
    --grace 10
```

The supervisor starts the autopilot and the agent (`--autopilot` is optional: an agent that
is its own autopilot, such as the CARLA reference agent, needs only `--agent`) in one
process group of its own, and registers with the relay as the entity's supervisor. On a
restart it stops the whole group (SIGTERM, then SIGKILL after `--grace` seconds, by
process group, never by name) and starts it again; it answers the relay once the new
group runs, and the restart is ready when the agent registers. It also starts the group
again when the agent exits on its own (at most every `--respawn-delay`, 10 s), and stops
it when the supervisor itself is stopped. Run it where the autopilot runs.

## Compatibility

The protocol is versioned. Agents written with this SDK speak v2 and fall back to v1 on an
older relay; the relay serves v1 agents unchanged, giving them the v1 sequence (`stop`,
`clear_goal`, `teleported(pose)`) instead of `reset`. An
agent without `on_reset` is treated the same way by the SDK. The protocol itself, for
writing an agent in another language, is in the developer documentation:
[the agent protocol](../design/user-workflow.md#the-agent-protocol-i3).
