# Your autopilot as a pilot

Drive a scenario entity with an autopilot that is not swb's Autoware profile: a launch
file of yours, a program, a ROS 2 stack, or something you start yourself. It assumes a
project with a working stack ([Tutorial 1](../tutorials/01-first-drive.md)) and the
pilot kinds of [Tutorial 4](../tutorials/04-several-pilots.md).

An autopilot meets swb at two places:

- **the scenario side**: an **agent** takes the scenario's commands (reset, drive to this
  goal, keep under this speed, stop) from the relay and reports the vehicle's state;
- **the vehicle side**: the car in the simulator, its sensors and its **vehicle
  interface**. These are Autoware's messages, whatever the autopilot: a non-Autoware
  autopilot drives the car by publishing Autoware's control messages and reads its state
  from Autoware's status messages ([below](#the-vehicle-interface)).

The entity is the scenario's: a controller named `Autoware` (the ego) or `agent`. The
project binds a pilot to its name in `swb.yaml`.

## Which kind

| Your autopilot | Kind | The scenario side | The vehicle side |
|---|---|---|---|
| Autoware, with your launch or parameters | `autoware` | swb's vehicle agent | started ([Your Autoware](your-autoware.md)) |
| a ROS 2 launch that serves Autoware's API (`/api/routing/...`, `/api/operation_mode/...`) | `launch` | swb's vehicle agent, through that API | started |
| a ROS 2 launch with its own way to take a goal | `launch`, `agent: own` | an agent your launch starts (below) | started |
| a program on swb's agent SDK (Python) | `command` | the program is the agent | only with `args: {vehicle_side: "true"}` |
| started by you, anywhere | `none` | your agent registers with the relay itself | none |

"Started" means: on CARLA, the pilot gets CARLA's vehicle bridge in its own ROS domain;
on Isaac Sim, the vehicle side is the world server's and the ego's pilot runs in its
domain. **`agent: own`** means that swb starts no agent for the entity: your launch file
must start one (a program on the agent SDK, or `ros_adapter`, below) that registers the
entity with the relay given as the `relay` launch argument, or the run waits for it and
ends `error`.

## The vehicle interface

What the vehicle side offers in the pilot's ROS domain. All topics are absolute names
(`/control/...`), reliable QoS unless named, stamped in simulation time. Message packages:
`autoware_control_msgs`, `autoware_vehicle_msgs`, `tier4_vehicle_msgs`,
`autoware_perception_msgs`, all in Autoware 1.5.0.

### Your autopilot publishes

| Topic | Type | Used |
|---|---|---|
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | `lateral.steering_tire_angle` (rad, the road wheels' angle) and `longitudinal.acceleration` (m/s^2; turned into throttle or brake through the car's pedal maps on CARLA). `longitudinal.velocity`: on CARLA only its sign, and only until the first gear command (below 0 is reverse); Isaac reads it too. Publish at a steady rate: with no command for 1 s of simulation time, CARLA's vehicle bridge brakes to a stop and turns the hazard lights on, until commands return |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | the gear (`DRIVE`, `REVERSE`, `PARK`, ...); `NONE` keeps the last |
| `/control/command/turn_indicators_cmd` | `autoware_vehicle_msgs/msg/TurnIndicatorsCommand` | `NO_COMMAND` keeps the last |
| `/control/command/hazard_lights_cmd` | `autoware_vehicle_msgs/msg/HazardLightsCommand` | `NO_COMMAND` keeps the last |
| `/control/command/emergency_cmd` | `tier4_vehicle_msgs/msg/VehicleEmergencyStamped` | CARLA only, and ignored unless the vehicle bridge runs with `honor_emergency_cmd:=true` |

The service `/control/control_mode_request` (`autoware_vehicle_msgs/srv/ControlModeCommand`)
accepts `AUTONOMOUS` and refuses `MANUAL`.

### The vehicle publishes

Once per simulation frame (SSv2's frame rate, 20 frames per simulated second by default):

| Topic | Type | Frame |
|---|---|---|
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | `base_link` (longitudinal, lateral velocity, heading rate) |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | the road wheels' angle |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | |
| `/vehicle/status/turn_indicators_status` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` | |
| `/vehicle/status/hazard_lights_status` | `autoware_vehicle_msgs/msg/HazardLightsReport` | |
| `/vehicle/status/actuation_status` | `tier4_vehicle_msgs/msg/ActuationStatusStamped` | CARLA only: the applied throttle, brake, steer |
| `/clock` | `rosgraph_msgs/msg/Clock` | simulation time: run your nodes with `use_sim_time:=true` |

The sensors, each in its own frame, with sensor-data QoS on CARLA:

| Topic | Type |
|---|---|
| `/sensing/lidar/<name>/pointcloud_before_sync` | `sensor_msgs/msg/PointCloud2` |
| `/sensing/camera/<name>/image_raw`, `.../camera_info` | `sensor_msgs/msg/Image`, `CameraInfo` |
| `/sensing/imu/<name>/imu_raw` | `sensor_msgs/msg/Imu` |
| `/sensing/gnss/<name>/nav_sat_fix` | `sensor_msgs/msg/NavSatFix` |

The names come from the car's spec. For the Tesla Model 3 they are `top` (lidar),
`imu_link`, `gnss_link` and `camera6` (a camera, off by default on CARLA); on Isaac the
defaults are `/sensing/lidar/top/pointcloud_before_sync`, `/sensing/imu/imu_link/imu_raw`
and `/sensing/gnss/gnss_link/nav_sat_fix`.

Also, when the map has a signal table: the traffic lights' state,
`/perception/traffic_light_recognition/external/traffic_signals`
(`autoware_perception_msgs/msg/TrafficLightGroupArray`), as a vehicle-to-infrastructure
feed. On CARLA, always: the true pose, `/carla/ground_truth/odom` (`nav_msgs/msg/Odometry`,
`map` to `base_link`) and `/carla/ground_truth/tf`, for an autopilot that does not localize
itself or for checking one that does. On Isaac a ground-truth pose is off unless the ego's
world-server config turns it on. The vehicle side does not publish `/tf` for the car:
localization is the autopilot's.

### What CARLA's vehicle bridge waits for

On CARLA the vehicle bridge does not attach to the car until something in the pilot's
domain publishes the car's description on `/robot_description` (latched, as
`robot_state_publisher` does). It places the sensors at the frames of that description,
read back from `/tf_static`. Autoware's launch does this; a launch of your own must too.
For a shipped car, Autoware's own vehicle model file does it:

```xml
<node pkg="robot_state_publisher" exec="robot_state_publisher">
  <param name="use_sim_time" value="true"/>
  <param name="robot_description" value="$(command 'xacro
      $(find-pkg-share tier4_vehicle_launch)/urdf/vehicle.xacro
      vehicle_model:=tesla_model3 sensor_model:=tesla_model3_sensor_kit')"/>
</node>
```

Until it arrives, the vehicle bridge logs `Still waiting for Autoware...` every 5 s. On
Isaac the rig is the world server's and waits for nothing.

No shipped example drives a car through this interface with a non-Autoware autopilot yet
([Supported and not supported](../reference/limits.md#pilots-and-autopilots)); the
contract above is the vehicle bridges' own.

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
| `map_path` | the map directory of the stack's current map (`lanelet2_map.osm`, `pointcloud_map.pcd`) |
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
agent must share a domain and the DDS settings with the autopilot it drives: `eval
"$(ros2 run swb seat env 0)"` puts a terminal on the first pilot's domain (`1` for the
second, and so on; `scenario` for the bridge's).

## Readiness

A run waits for each pilot's readiness before SSv2 starts. `autoware`'s check is
`ego_ready` (Autoware can take a route); the other kinds are ready when started, unless
`ready:` names a command that exits 0 when they are:

```yaml
    ready: [ros2, run, my_autopilot, is_ready]
```
