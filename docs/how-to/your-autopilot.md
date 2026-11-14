# Your autopilot as a pilot

Drive a scenario entity with an autopilot that is not swb's Autoware profile: a launch
file of yours, a program, a ROS 2 stack, or something you start yourself. It assumes a
project with a working stack ([Tutorial 1](../tutorials/01-first-drive.md)) and the
pilot kinds of [Tutorial 4](../tutorials/04-several-pilots.md).

The entity is the scenario's: a controller named `Autoware` (the ego) or `agent`. The
project binds a pilot to its name in `swb.yaml`.

## Which kind

| Your autopilot | Kind | It gets |
|---|---|---|
| Autoware, with your launch or parameters | `autoware` | [Your Autoware](your-autoware.md) |
| a ROS 2 launch that serves Autoware's API (`/api/routing/...`, `/api/operation_mode/...`) | `launch` | swb's vehicle agent drives it through that API |
| a ROS 2 launch with its own topics | `launch`, `agent: own` | your launch also starts an agent (below) |
| a program on swb's agent SDK (Python) | `command` | `--relay --entity --seed --run-dir` |
| started by you, anywhere | `none` | nothing: your agent registers with the relay itself |

Every kind but `none` gets the vehicle side on CARLA (the car's sensors and vehicle
interface, `/control/command/*` in and `/vehicle/status/*` out, in the pilot's own ROS
domain); on Isaac the vehicle side is the world server's. `command` gets it with
`args: {vehicle_side: "true"}`.

## A launch file

```yaml
pilots:
  ego:
    kind: launch
    launch: my_autopilot bringup.launch.xml    # "<package> <file>", or a path
    args: {max_speed: "8.0"}                   # your launch's arguments
```

Your launch runs in the pilot's domain and receives the pilot contract as launch
arguments, then `args`:

| Argument | Is |
|---|---|
| `entity` | the scenario entity it drives |
| `relay` | the agent relay, `tcp://host:port` |
| `map_path` | the map directory of the stack's current map |
| `vehicle` | the car's spec name |
| `seed`, `run_dir` | the run's seed and directory (`lifetime: run` pilots) |
| `sim` | `carla` or `isaac` |
| `use_sim_time` | `true` |

Declare the ones you use. With `agent: own` your launch registers the entity itself
(an agent from the SDK, as below); otherwise swb's vehicle agent does, and drives your
stack through Autoware's API.

## A program

```yaml
pilots:
  cut_in:
    kind: command
    run: [python3, pilots/cut_in.py, --trigger-m, "40"]
    lifetime: run
```

swb runs the words of `run` (relative paths are the project's; not a shell) with
`--relay <url> --entity <name> --seed <n>` and, per run, `--run-dir <dir>` appended, and
starts it again when it ends. Substitutions apply in `run` and `args`: `{seed}`,
`{run_dir}`, `{run_id}`, `{param:<name>}` (a scenario or suite parameter; the pilot must
then be `lifetime: run`).

The program is an agent: a subclass of `swb_agent.Agent`, which turns the relay's
commands (reset, drive to a goal, keep under a speed limit, stop) into your autopilot's
and reports its state. Start from a skeleton that already passes the conformance check:

```bash
ros2 run swb_agent swb-agent new my-pilot
cd my-pilot
ros2 run swb_agent swb-agent check my_pilot.agent:MyPilotAgent
```

The hooks, the state to report and the checks: [The agent SDK](../reference/agent-sdk.md).

## A ROS 2 autopilot without code

When each step of your autopilot is one topic or one service, an `agent.yaml` maps them
and `ros_adapter` is the agent. As a `command` pilot in the autopilot's domain:

```yaml
pilots:
  ego:
    kind: command
    run: [ros2, run, swb_agent, ros_adapter, --config, agents/my_autopilot.yaml]
    args: {vehicle_side: "true"}
```

The YAML's keys: [The agent SDK, `ros_adapter`](../reference/agent-sdk.md#ros_adapter-a-ros-2-autopilot-from-yaml).

## Your own process

`kind: none` starts nothing: the stack waits for nothing either, and the run waits for
the entity's agent to register with the relay. Run your agent against the seat's relay
port (`ros2 run swb seat show`, `relay 5560` on seat 0), on this host or another:

```bash
python3 -m my_pilot --relay tcp://localhost:5560 --entity ego
```

An agent reaches the relay over TCP, so its ROS domain does not matter to swb. A ROS
agent must share a domain and the DDS with the autopilot it drives: `eval "$(ros2 run swb
seat env 0)"` puts a terminal on the first pilot's.

## Readiness

A run waits for each pilot's readiness before SSv2 starts. `autoware`'s check is
`ego_ready` (Autoware can take a route); the other kinds are ready when started, unless
`ready:` names a command that exits 0 when they are:

```yaml
    ready: [ros2, run, my_autopilot, is_ready]
```
