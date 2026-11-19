# Connect your Autoware to a testbench

Drive a testbench's car with your own Autoware. scensim installs the simulation side; the
car's Autoware packages are source that you build in your Autoware workspace, against your
Autoware. This page imports and builds them, picks how Autoware is configured, and points a
testbench's `autoware` pilot at it. It assumes scensim is installed ([Install](../getting-started/install.md))
and a testbench runs without Autoware ([Tutorial 1](../tutorials/01-first-drive.md)).

## The two sides

| Side | What | Where it comes from |
|---|---|---|
| simulation | the scenario runner, the bridge, the world servers, the vehicle bridges, the cars' descriptions (`tesla_model3_description`, `tesla_model3_sensor_kit_description`) | `scensim install` (pip), or a build from source |
| Autoware | the car's Autoware packages: its vehicle launch (`tesla_model3_launch`), its sensor kit's launch (`tesla_model3_sensor_kit_launch`) and `individual_params`; optionally scensim's Autoware profile (`autoware_sim_profile`) | source, at the commit of your scensim release, built in your Autoware workspace |

The Autoware packages need only your Autoware to build and run: they work with Autoware's
binary packages, a source build, or your own distribution of it. scensim never downloads or
sources an Autoware: yours is the one in your terminal.

## Import and build

With your Autoware sourced, in a workspace of yours (here `~/autoware_side`). Every scensim
release has its packages' list, `autoware-side.repos`, beside its other files on the
release page (use your release's version):

```bash
source /opt/autoware/1.5.0/setup.bash        # your Autoware
mkdir -p ~/autoware_side/src && cd ~/autoware_side
wget https://github.com/NEWSLabNTU/scensim/releases/download/v0.2.0/autoware-side.repos
vcs import src < autoware-side.repos
colcon build --packages-up-to tesla_model3_launch tesla_model3_sensor_kit_launch \
    individual_params
```

Add `autoware_sim_profile` to the list for the profile ([Two ways to configure
Autoware](#two-ways-to-configure-autoware)). The packages build with CMake and xacro only, in
seconds. With scensim built from source, the same packages are in its workspace already
(`src/autoware_sim_vehicle`); build them there instead.

Rebuild after you move to another Autoware: a workspace remembers the Autoware it was built
over and sources it again.

## The environment file

Every terminal that runs scensim sources, in this order: ROS, your Autoware, scensim, then
the workspace above.

```bash
# ~/scensim.bash
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
source ~/.local/share/scensim/current/sim/setup.bash
source ~/autoware_side/install/setup.bash
```

An `autoware` pilot runs in the stack's environment, so it finds your Autoware and these
packages from here. A pilot can also name a workspace of its own with `overlays:`
([Your autopilot](your-autopilot.md)).

## The testbench

```yaml
# testbench.yaml
simulator: carla
pilots:
  ego:
    kind: autoware
    vehicle: tesla_model3        # Autoware's vehicle_model
```

| Setting | Default | What it chooses |
|---|---|---|
| `vehicle` | `tesla_model3` | the car, and Autoware's `vehicle_model`: `<vehicle>_description` (simulation side) and `<vehicle>_launch` (yours) |
| `args: {sensor_model: ...}` | `<vehicle>_sensor_kit` | Autoware's `sensor_model`: `<sensor_model>_description` and `<sensor_model>_launch` |
| `autoware_profile` | `stock` | how Autoware is configured: `stock` or `scensim` (below) |
| `autoware_launch` | your Autoware's | the `autoware.launch.xml` to start, for a fork of autoware_launch |

`tesla_model3` is the car on CARLA 0.9.16 and on Isaac Sim alike: the same vehicle model and
sensor kit, the simulator's vehicle bridge publishing the same sensors. CARLA 1.0, from a
source build of scensim, has its own body: scensim's stack picks its vehicle model,
`tesla_model3_carla_1_0`, and the workspace needs `tesla_model3_carla_1_0_launch` (add it to
the `colcon build` list; `autoware-side.repos` has it).

## Two ways to configure Autoware

**`autoware_profile: stock`** (the default) starts your `autoware.launch.xml` as your
Autoware ships it. scensim passes only what a simulator needs:

| Argument | Value | Why |
|---|---|---|
| `map_path`, `vehicle_model`, `sensor_model` | the testbench's map and car | |
| `use_sim_time` | `true` | the simulator's clock |
| `launch_vehicle_interface` | `false` | the simulator's vehicle bridge is the vehicle interface |
| `launch_sensing_driver` | `false` | the bridge publishes the sensors; there are no drivers |
| `system_run_mode` | `logging_simulation` | the pose initializer does not wait for a stopped car's IMU check |
| `traffic_light_recognition/use_high_accuracy_detection` | `false` | that detector's node does not stay up on a simulator's signal heads |
| `scenario_simulation` | `true` | Autoware's own switch for a scenario simulator: it starts the API that serves `/api/autoware/set/velocity_limit`, which a scenario's speed limit needs (1.5.0: the deprecated API's adaptor; 2.x: the evaluation adapter) |
| `rviz` | the pilot's `rviz:` | |

**Your Autoware must serve the speed limit.** A scenario that sets the ego's speed (an
OpenSCENARIO `SpeedAction`) reaches Autoware as a call to `/api/autoware/set/velocity_limit`.
Autoware serves that service only when it is launched with `scenario_simulation:=true`, which
the stock pilot passes. If you point `autoware_launch:` at a launch of your own, keep that
argument working; without it the call finds no service and the scenario errors after five
tries ("Requested the service /api/autoware/set/velocity_limit 5 times, but was not
successful"). What the switch adds, measured by resolving `autoware.launch.xml` with and
without it (nothing is removed or reconfigured in either release):

| Autoware | What `scenario_simulation:=true` adds |
|---|---|
| 1.5.0 | the deprecated tier4 API, 60 nodes: the internal and external API adaptors (`autoware_iv_internal_api_adaptor`, `autoware_iv_external_api_adaptor`, the velocity limit among them), the AWAPI adapter and its relays (`awapi_awiv_adapter`), `tier4_autoware_api_extension` and `tier4_deprecated_api_adapter`. They also give the agent the aggregated RTC and the emergency flag (`/api/external/get/emergency`), as with scensim's profile. Their relays forward `/api/autoware/set/traffic_signals`, `intersection_states` and `crosswalk_states` to Autoware only when something publishes them; scensim does not. |
| 2.1.0 | the evaluation adapter, 3 nodes: its container, `VelocityLimit` (the velocity limit) and `AutowareEngage` (`/api/external/set/engage`, which an external engage turns into an operation-mode change) |

Everything else is your Autoware's own: its planning, control and perception parameters,
its lidar detector and its ML models (`data_path`, by default `~/autoware_data` up to
Autoware 1.9.0 and `~/autoware_data/ml_models` from 2.0.0).

**`autoware_profile: scensim`** starts scensim's Autoware profile around your
`autoware.launch.xml`: the settings scensim's own scenarios are tested with. Import
`autoware_sim_profile` too. It adds:

- traffic signals from the simulator (the vehicle bridge's), with camera recognition off;
  the stock car has no traffic-light camera;
- a lidar detector you choose, `clustering` by default (CPU), which is what passes on CARLA;
  stock Autoware uses CenterPoint, which needs its models and a GPU;
- localization, control (no steering-offset learning) and planning parameters for a
  simulated car, and a speed limit, `max_velocity`;
- no GNSS initialization of a managed car: scensim's agent initializes it at the
  scenario's pose, and a second initialization would queue a second NDT alignment;
- a clock guard that ends the pilot when the simulator's clock goes back.

### How stock drives against the profile

Tutorial 1 (`drive_to_goal.xosc` on Town01) with Autoware 1.5.0's binary packages, both ways,
on the same build and host, one after the other (2026-11-19):

| | `stock` | `scensim` |
|---|---|---|
| Result | passed, 43.6 s | passed, 41.4 s |
| From the stack's start to the pilot ready | about 68 s | about 58 s |
| Emergency stops | 34, each released within a moment: stock's diagnostic graph reads the localization and routing states as stale and requests one | none |
| Lidar detector | CenterPoint (GPU) | `clustering` (CPU) |
| GPU memory beside CARLA | CenterPoint and the point cloud's CUDA preprocessing: they failed to start with about 4.8 GB free on the GPU, and ran with about 7.7 GB free | none needed |

**A known stock behaviour:** stock Autoware's diagnostic graph reads the localization and
routing states as stale under a simulator and requests brief emergency stops, 34 in this
run, each released within a moment. The profile replaces that graph and requests none. So
a stock Autoware drives the tutorial, a little slower and with emergency stops that a
scenario with tighter timing may not survive, and it needs a GPU with a few GB free once
CARLA is up. Passing `args: {lidar_detection_model: clustering}` to a stock pilot reaches
Autoware 1.5.0's perception and moves the detector to the CPU, but the point cloud's CUDA
preprocessing still needs the GPU.

The shipped examples, both ways, one scenario each, on CARLA 0.9.16 with Autoware 1.5.0
(2026-11-19):

| Project, scenario | `stock` | `scensim` |
|---|---|---|
| `safety_suite`, UC-AEB-001-0001 (9 variants) | passed 9 of 9 (a first try ended at the 6th variant on the planner abort below) | passed |
| `multi_pilot`, `two_av` (two Autowares) | passed | passed |
| template `multi-pilot`, `three_pilots` | passed | passed |
| `first_drive`, `pedestrian_crossing` | failed 3 of 3: the ego came within 1 m of the pedestrian in its path | passed |

**Stock does not stop clear of a crossing pedestrian.** In `pedestrian_crossing`, stock
Autoware 1.5.0 drove on into the space of the pedestrian crossing in front of it: with the
scenario's clearance check moved out of the way, it came within 0.02 m of the walker in one
run and touched the walker's box in the scenario runner in the other. CARLA recorded no
contact in any run, as CARLA's walker is narrower than its box. scensim's profile stops more
than 1 m short and passes. So `first_drive` names `autoware_profile: scensim`, and Tutorial 3
asks for it; why stock reacts late is not studied yet.

**A rare planner abort, in either configuration.** Autoware 1.5.0's behavior_path_planner
can abort when a new route arrives (`failed to add guard condition to wait set: guard
condition implementation is invalid`): resetting its modules recreates the goal and start
planners' callback groups while the executor still waits on the old ones
(autoware_universe#12460, rclcpp#2163). scensim measured it once in about 64 route changes.
The run then ends with an error naming the pilot's container, and a retry passes.

Its arguments are in [Your Autoware](your-autoware.md). The profile runs Autoware 1.5.0,
2.0.0 and 2.1.0 and refuses another release by name. With `stock`, any Autoware the car's
packages support runs.

## Check

`scensim testbench check` checks every `autoware` pilot against the terminal's Autoware
before a run, and every run checks it again:

- a car's Autoware package not found (or `autoware_sim_profile`, for `scensim`) is refused,
  naming the package and the import above;
- an Autoware outside a package's supported releases is refused, naming both;
- a supported Autoware, or ROS, the packages were not tested on is a warning; the run goes
  on.

scensim reads your Autoware's release from its install path (`/opt/autoware/2.1.0`) or,
for a build of your own, from `autoware_launch`'s version.

## Supported Autoware releases

| Autoware | ROS 2 | Car packages | How it is tested |
|---|---|---|---|
| 1.5.0 | Humble | supported | live: the tutorials and scensim's scenario suites; every release builds and resolves both configurations in CI |
| 1.7.1 | Humble | supported, untested (a warning) | none yet |
| 1.9.0 | Humble | supported, untested (a warning) | none yet |
| 2.0.0 | Humble | supported | live: a scenario drive with the profile |
| 2.1.0 | Humble | supported | live: Tutorial 1; every release builds and resolves both configurations in CI |
| 2.1.0 | Jazzy | supported, untested (a warning) | every release builds and resolves both configurations in CI; no live run yet |

"Resolves" means Autoware's launch is followed to its last node with the car's packages,
without running it: every package, argument, parameter file and component it names is found.

## For maintainers

Each release's workflow builds the Autoware packages from `autoware-side.repos` over
NEWSLab's Autoware binaries (the CPU build of each release) and the release's simulation
tarball, then resolves both configurations: `scripts/release/check_autoware_side.sh`, about
ten minutes per Autoware release. It runs on a lab host the same way, in a `ros:<distro>`
container:

```bash
python3 scripts/release/autoware_side_repos.py -o /tmp/autoware-side.repos
LR=https://github.com/NEWSLabNTU/autoware-localrepo/releases/download
docker run --rm --init -v "$PWD:/src:ro" -v /tmp:/w ros:humble \
    /src/scripts/release/check_autoware_side.sh 2.1.0 \
    $LR/2.1.0-1/autoware-localrepo-2-1-0_2.1.0-1ubuntu2204cpu_all.deb \
    /w/scensim-sim-<version>-humble-x86_64.tar.zst /w/autoware-side.repos /w/out
```

A live run is what moves a row of the table to tested. Before marking one:

1. Install that Autoware's binaries and its ML models on the host.
2. Build the Autoware packages over it, as above, from the release's `autoware-side.repos`.
3. Run Tutorial 1 with `autoware_profile: stock`, then with `scensim`; both must pass.
4. Add the release and ROS to `tested` in each Autoware package's `package.xml`
   (`<export><scensim_autoware supported="..." tested="..."/></export>`, generated by
   `autoware_gen`), widen `supported` for a new release, and update this table.

When a new Autoware release changes something the car's packages use (the sensor kit's
entry point, `vehicle_info`, the messages of the vehicle interface), keep one source: put a
small difference inside the package, keyed on the release, and give a release a package of
its own only where whole files differ.
