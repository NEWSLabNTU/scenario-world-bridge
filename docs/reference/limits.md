# Supported and not supported

Everything swb does not do, or does only in part, on one page. The other pages link here
instead of repeating it. "Tested" means run end to end on a live stack; "builds" means it
compiles and the unit tests pass, nothing more.

## Platform

| | Supported | Not supported |
|---|---|---|
| OS and ROS | Ubuntu 22.04 with ROS 2 Humble | other Ubuntu releases, other ROS 2 distributions, Windows, macOS |
| Autoware | the 1.5.0 release: the binary packages or your own build of it | any other release |
| CARLA | 0.9.16, packaged release (tested) | 0.9.14 and 0.9.15 build (`CARLA_VERSION=0.9.15` in front of `colcon build`, with that release's CARLA and `carla` wheel) but are untested; 0.10 is not supported. A bridge refuses a CARLA server of another version than it was built for |
| Isaac Sim | 6.1, pip-installed | other releases |
| GPU | NVIDIA | other vendors (Autoware's perception and both simulators need CUDA or NVIDIA's Vulkan driver) |
| Python | the agent SDK, judges and pilot programs in Python 3.10 (Humble's) | an agent in another language needs the agent protocol itself, which only the developer documentation describes |

## Running CARLA

- **Quality level.** At `-quality-level=Low` CARLA 0.9.16 crashes when a scenario changes
  towns (a CARLA bug). Keep the default, Epic, unless every run stays on the town CARLA
  started with.
- **One stack per CARLA.** A CARLA serves one stack; two stacks on one host need two
  CARLAs on different ports ([Several stacks on one host](../how-to/several-stacks.md)).

## Scenarios

What the bridge does with a scenario file, on either simulator unless named:

- **Sensors are the simulator's.** SSv2's sensor requests are declined; the pilot's vehicle side publishes the simulator's
  cameras and lidar instead.
- **The ego takes goals and speed limits.** SSv2 refuses lane-change and relative-speed
  actions for an ego, by design.
- **Uncommanded traffic lights are green.** CARLA's own light cycling is frozen at the start
  of a run; a light the scenario does not command stays green.
- **SSv2-driven road users go where SSv2 says.** A collision does not change their path;
  for a vehicle that reacts to traffic with physics, use `simulator_autopilot` or `agent`.
- **`simulator_autopilot` cars (CARLA's Traffic Manager)** and **`agent` cars (a pilot of
  the project)** do not take `LaneChangeAction`, a relative `SpeedAction` or
  `FollowTrajectoryAction`; the run fails naming the action. A controller cannot be
  switched to or from either during a run, and a `TeleportAction` after the start is
  ignored for a `simulator_autopilot` car (the simulator owns its pose).
- **Isaac Sim** runs the same files as far as it can drive them: `simulator_autopilot` is
  refused (Isaac has no driver of its own), only one entity can have a pilot (the ego), the
  `carla_basic` pilot is CARLA only, and an Isaac stack serves one world: a scenario on
  another map needs the stack restarted with that `map:=`.
- **Road physics (Isaac).** A scenario sets the road's state (wetness, a zone's surface
  class); it cannot choose the surface map. Autoware on the rough gravel road is not
  validated. On CARLA a `RoadCondition` only scales tire friction; zones, soil, ruts and
  the rough road are Isaac's. What the ground model leaves out:
  [Road physics, Limits](road-physics.md#limits).

The rules in full, with what fails and how: [What a scenario may use](scenarios.md).

## Projects

- **`vehicles:` is checked, not loaded.** `project check` refuses a listed spec file that
  does not exist, but the stack does not load it: give the bridge your catalog and the
  world server your physics through `stack_args` ([Your car](../how-to/your-car.md)).
- **No project keys for the bridge's and the world servers' config files.** They go through
  `stack_args` ([Configure the stack](../how-to/configure.md)). The `stack_args` lines in
  those pages have not been run through play_launch's own argument parser on a live stack.
- **`logging.levels` and `logging.debug` are refused.** Per-process ROS log levels are not
  built; give a pilot its own (for example `args: {log_level: warn}`) where its launch takes
  one. `capture: debug` records every topic.
- **A car of your own needs measured pedal maps.** Nothing generates them from the spec.
  The `<car>_description` and `<car>_sensor_kit_*` packages are Autoware's; a car for a
  non-Autoware autopilot on CARLA needs only the spec's outputs and a URDF
  ([Your car](../how-to/your-car.md)).
- **A third simulator** has a world server protocol, but the project stack
  (`swb.launch.xml`) starts only `carla` and `isaac`; another simulator runs without
  projects, seats and `swb run` ([Your simulator](../how-to/your-simulator.md)).

## Pilots and autopilots

- **The vehicle interface is Autoware's.** Every autopilot, Autoware or not, drives the car
  through Autoware's control and status messages
  ([Your autopilot, the vehicle interface](../how-to/your-autopilot.md#the-vehicle-interface)).
- **No shipped example** of a `launch` pilot. The shipped non-Autoware autopilot on the
  vehicle interface is `simple_pilot` (the `simple` kind, the `simple_drive` example); the
  other non-Autoware pilots (`carla_basic`, the tutorial's `cut_in`) drive CARLA's vehicle
  directly, without a vehicle side.
- **The `simple` pilot ignores traffic.** It follows the lanes, their speed limits and the
  curves to the goal, and stops there; it does not stop for traffic lights, vehicles or
  pedestrians. It drives on the simulator's true pose (it does not localize), changes lanes
  only where the route cannot keep its lane, and never reverses.
- **An agent's episode** carries the pose and an episode id; its `scenario`, `map` and
  `parameters` are empty ([The agent SDK](agent-sdk.md)).
- **The `carla_basic` ego variant** of the tutorials (BasicAgent instead of Autoware) runs
  the same scenarios but has not been run through every tutorial.

## Several users and hosts

- **The asset cache is one user's.** A fetch changes the modes of the cache's directories
  and records the project in the cache, so it needs to own them: a cache another user
  fetched into cannot be fetched into. Give every user their own `cache:` (the default,
  `~/.cache/swb`).
- **Two hosts, one suite** (`--shard i/n`) leave two run directories; one results table
  comes from copying both into one place and running `ros2 run swb runs summary <dir>`
  there ([Run suites in CI](../how-to/run-in-ci.md)).
- **Isaac Sim** needs an RTX GPU with 24 GB to itself: one Isaac stack per GPU.
