# scenario-world-bridge

Test [Autoware](https://autowarefoundation.github.io/autoware-documentation/main/) on
[OpenSCENARIO](https://www.asam.net/standards/detail/openscenario/) scenarios in
[CARLA](https://carla.org/) or [Isaac Sim](https://developer.nvidia.com/isaac/sim), with
Autoware driving on **simulated sensors**: cameras and lidar feed its full perception,
localization, planning and control, while
[scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2) (SSv2) runs the
scenario and gives the verdict.

<p align="center">
  <img src="docs/media/hero_carla_vs_isaac.webp" alt="The same Autoware drives the same NTU campus route in CARLA (left) and Isaac Sim (right)" width="840"><br>
  <em>One Autoware, one vehicle spec, two simulators: the same NTU campus route in CARLA and Isaac Sim (5x).</em>
</p>

<!-- Each image row and its caption row sit in their own <tbody>: GitHub stripes every
     second row of a tbody, so both are a first row and share one colour. -->
<table>
  <tbody><tr>
    <td width="33%" valign="bottom"><img src="docs/media/demo_tour.webp" alt="One CARLA drive with six incidents: a pedestrian, a braking lead car, a red light, a broken-down car, crossing traffic and a cyclist"></td>
    <td width="33%" valign="bottom"><img src="docs/media/two_av.webp" alt="Two Autoware stacks drive their own vehicles in one CARLA scenario"></td>
    <td width="33%" valign="bottom"><img src="docs/media/awf_acc.webp" alt="Autoware follows a decelerating vehicle in the AWF adaptive cruise scenario on CARLA, with RViz inset"></td>
  </tr></tbody>
  <tbody><tr>
    <td valign="top"><b>Six incidents, one drive</b><br><sub>CARLA, Town01: pedestrian, braking car, red light, blocked lane, crossing traffic, cyclist; all pass (3x)</sub></td>
    <td valign="top"><b>Two Autoware vehicles</b><br><sub>CARLA, Town01, one scenario (6x)</sub></td>
    <td valign="top"><b>AWF adaptive cruise</b><br><sub>CARLA, UC-ACC-001; RViz inset (3x)</sub></td>
  </tr></tbody>
  <tbody><tr>
    <td width="33%" valign="bottom"><img src="docs/media/isaac_town02.webp" alt="An SSv2 scenario on Isaac Sim: Autoware turns left on Town02 and drives to its goal, with RViz inset"></td>
    <td width="33%" valign="bottom"><img src="docs/media/haruna_drift.webp" alt="A Mustang drifts through a 10.8 m hairpin on the Mount Haruna road in Isaac Sim"></td>
    <td width="33%" valign="bottom"><img src="docs/media/gravel.webp" alt="A car at 60 km/h on a gravel road with ruts, potholes and loose rocks in Isaac Sim"></td>
  </tr></tbody>
  <tbody><tr>
    <td valign="top"><b>The same scenarios on Isaac Sim</b><br><sub>Isaac Sim, Town02; RViz inset (2.5x)</sub></td>
    <td valign="top"><b>Hairpin drift on Mount Haruna</b><br><sub>Isaac Sim, sideslip up to 22 degrees</sub></td>
    <td valign="top"><b>Gravel and rocks at 60 km/h</b><br><sub>Isaac Sim, surfaces</sub></td>
  </tr></tbody>
</table>

## Why

SSv2 normally runs Autoware against a simulator that hands it the ground truth: perfect
detections and exact positions, so perception is never tested. Here CARLA or Isaac Sim is
SSv2's simulator instead, and Autoware has to *see* the pedestrian, the car ahead and the
traffic light. For someone who already writes SSv2 scenarios:

- **The same `.xosc` files run.** The ego comes from the vehicle catalog, and the map is named
  as `$(env CARLA_MAPS)/Town01`; the [starter scenarios](src/swb_examples/scenarios/) show both.
- **The sensors are the simulator's.** SSv2's sensor requests are declined; Autoware gets the
  simulator's cameras and lidar.
- **The verdict is SSv2's** (a JUnit file), optionally with CARLA's own collision check beside it.
- **Free drive too.** Without a scenario, put the car on the map and drive it from RViz.

## How it works

<p align="center">
  <img src="docs/figures/architecture.svg" alt="SSv2 drives the bridge and the agent relay; the bridge drives a world server for CARLA or Isaac Sim; Autoware connects to the simulator through a vehicle bridge" width="820"><br>
  <em>A scenario run. The two teal lines are the boundaries every simulator implements.</em>
</p>

- **Scenario side.** SSv2 runs the scenario. The **bridge** turns its requests into one
  simulator-neutral protocol. The **agent relay** passes each Autoware-driven vehicle its
  route and engage.
- **World server**, one per simulator (`swb_carla_world`, `swb_isaac_world`). It loads the map,
  moves the other road users, sets the traffic lights and steps time.
- **Vehicle side.** Your Autoware, connected to the simulator by a **vehicle bridge**
  ([acb](https://github.com/NEWSLabNTU/autoware_carla_bridge) for CARLA,
  [aib](https://github.com/NEWSLabNTU/autoware_isaac_bridge) for Isaac Sim). The vehicle bridge
  publishes the sensors, the vehicle status and the clock, and drives the car from Autoware's
  commands. It runs in a ROS domain of its own; on CARLA it may run on another host.

The car is described once, in a vehicle spec. The simulators' physics and sensors, SSv2's
catalog and Autoware's vehicle packages are generated from it, and a run whose parts came
from different specs is refused ([vehicle side](docs/design/vehicle-side.md)).

## Requirements

| | |
|---|---|
| OS, ROS | Ubuntu 22.04, [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) |
| Autoware | [1.5.0](https://autowarefoundation.github.io/autoware-documentation/main/installation/): the binary packages in `/opt/autoware/1.5.0`, or your own build of that release |
| Simulator | [CARLA 0.9.16](https://carla.readthedocs.io/en/0.9.16/start_quickstart/) (packaged release), or Isaac Sim 6.1 (pip install) |
| Launcher | [play_launch](https://pypi.org/project/play-launch/) 0.30.0 or newer |
| Build tools | Rust ([rustup](https://rustup.rs/)) and colcon's Rust plugin, `colcon-cargo-ros2` |
| Hardware | an NVIDIA GPU that runs the simulator beside Autoware. With Isaac Sim, plan for its world server alone holding about 17.5 GB of RAM on Town01 |
| Maps | an Autoware map (Lanelet2 + point cloud) per CARLA town: one script fetches and repairs them |

SSv2 is bundled (a fork, as a submodule). Isaac Sim also needs the CARLA towns and vehicles
as USD (the asset set). Ask NEWSLab for it.

## Quick start on CARLA

Commands use `play_launch launch`. `ros2 launch` takes the same arguments, but play_launch
must still be installed: the stack starts the autopilot, the vehicle agent and the scenario
as play_launch processes of their own.

**Install and build** (Rust and the tools first, then the workspace):

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
sudo apt install libz3-dev ros-humble-rclcpp-components ros-humble-class-loader
pip install colcon-cargo-ros2 'play_launch>=0.30.0'

git clone --recurse-submodules \
    https://github.com/NEWSLabNTU/scenario-world-bridge.git
cd scenario-world-bridge
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep install --from-paths src --ignore-src -y
export CMAKE_POLICY_VERSION_MINIMUM=3.5
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
source install/setup.bash
ros2 run autoware_sim_profile setup_autoware_data
```

**The environment file.** Every terminal starts by sourcing it, say `~/swb.bash`:

```bash
source /opt/autoware/1.5.0/setup.bash
source ~/scenario-world-bridge/install/setup.bash
export CARLA_MAPS=~/scenario-world-bridge/data/carla-autoware-bridge

# one DDS for every process, exported after Autoware's setup
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SWB_SHARE=$(ros2 pkg prefix --share swb_launch)
export CYCLONEDDS_URI=file://$SWB_SHARE/config/cyclonedds-localhost.xml

# the domains (these are the defaults; any two different values from 0 to 57)
export EGO_ROS_DOMAIN_ID=1
export SCENARIO_ROS_DOMAIN_ID=9
```

**The maps**, read from CARLA's OpenDRIVE files (here CARLA is in `~/CARLA_0.9.16`):

```bash
source ~/swb.bash
cd ~/scenario-world-bridge
SWB_XODR_DIR=~/CARLA_0.9.16/CarlaUE4/Content/Carla/Maps/OpenDrive \
    scripts/download_maps.sh
```

**CARLA**, in a terminal of its own; it stays up between runs. It serves its port after one
to three minutes (`ss -lnt | grep ':2000 '`):

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    ~/CARLA_0.9.16/CarlaUE4.sh \
    -RenderOffScreen \
    -nosound \
    -carla-rpc-port=2000
```

Once per town, with CARLA up, write the traffic-light table:

```bash
ros2 run swb_carla_world swb_carla_world \
    --generate-signal-table $CARLA_MAPS/Town01 \
    --ros-args -p carla_port:=2000 -p port:=5599
```

**One scenario, start to verdict.** The stack starts CARLA's world server, the bridge, the
agent relay, Autoware on CARLA and the scenario, and ends when the scenario ends:

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
ARGS="output_directory:=/tmp/quick_start initialize_duration:=600 global_timeout:=900"
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    scenario:=$S/basic/town01_ego_drive.xosc \
    scenario_args:="$ARGS" \
    exit_on_scenario_end:=true
cat /tmp/quick_start/scenario_test_runner/result.junit.xml
```

`failures="0" errors="0"` in the JUnit file means it passed. The first start builds
Autoware's TensorRT engines, which takes minutes; the longer `initialize_duration` and
`global_timeout` give it that time. The web UI at `http://127.0.0.1:8084` shows every
process and its log.

[Getting started on CARLA](docs/getting-started/carla.md) walks through the same steps with
what to expect at each, and keeps the stack up between scenarios.

**On Isaac Sim** the same scenarios run with `isaac.launch.xml`, once Isaac Sim, its message
bindings and the asset set are installed:
[Getting started on Isaac Sim](docs/getting-started/isaac.md).

**Without a scenario runner.** acb and aib also run Autoware on CARLA or Isaac Sim with no
SSv2 at all, and check the car itself (topics, frames, then acceleration, braking and
steering against the spec) with no Autoware: see their READMEs.

## Make it yours

| You want to | Start at |
|---|---|
| write your own scenario | [Prepare a scenario](docs/tutorials/01-prepare-a-scenario.md), [Writing scenarios](docs/design/scenario-authoring.md) |
| run many scenarios and judge them | [Run and judge](docs/tutorials/02-run-and-judge.md) |
| add pedestrians, obstacles, other cars, more Autoware vehicles | [Other road users and more AVs](docs/tutorials/03-other-road-users.md), [Build your own stack: more cars](docs/build-your-own-stack.md#8-more-cars) |
| test your own car: its spec, catalog entry and Autoware packages | [Build your own stack: the vehicle](docs/build-your-own-stack.md#6-the-vehicle) |
| run your own Autoware, or another autopilot | [Build your own stack: your autopilot](docs/build-your-own-stack.md#7-your-autopilot), [Writing an agent](docs/agent-interface.md) |
| change the weather, the bridge's or the world server's settings | [Configure the system](docs/tutorials/04-configure.md) |
| add another simulator | [Build your own stack: the simulator](docs/build-your-own-stack.md#3-the-simulator), [User Guide: another simulator](docs/user-guide.md#another-simulator) |
| run the parts in other ROS domains, or two stacks on one host | [ROS domains and DDS](docs/domains-and-dds.md) |

## Learn more

- **Getting started**, from install to your own scenario:
  [CARLA](docs/getting-started/carla.md), [Isaac Sim](docs/getting-started/isaac.md).
- **[Build your own stack](docs/build-your-own-stack.md)**: your scenario, simulator, car,
  autopilot and more cars, in one launch.
- **[ROS domains and DDS](docs/domains-and-dds.md)**: which process runs in which domain,
  and how to look into each.
- **Tutorials:**
  1. [prepare a scenario](docs/tutorials/01-prepare-a-scenario.md);
  2. [run and judge](docs/tutorials/02-run-and-judge.md): suites, verdicts, result files;
  3. [other road users and more AVs](docs/tutorials/03-other-road-users.md);
  4. [configure the system](docs/tutorials/04-configure.md);
  5. [a demo run and recording](docs/tutorials/05-demo-and-recording.md).
- **Reference:**
  - the [User Guide](docs/user-guide.md): launches, their arguments, the environment,
    troubleshooting;
  - [Writing an agent](docs/agent-interface.md),
    [Writing scenarios](docs/design/scenario-authoring.md);
  - [all documentation](docs/README.md).

**Other CARLA versions.** CARLA 0.9.16 is tested. 0.9.14 and 0.9.15 build (set
`CARLA_VERSION=0.9.15` in front of `colcon build`, with that release's CARLA, maps and
`carla` wheel) but are untested; the bridges refuse a server of another version than they
were built for. 0.10 is not supported yet.

Developing scenario-world-bridge itself: the [Developer Guide](docs/development.md).

## Related projects

- [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2): Autoware's
  scenario testing framework.
- [AWSIM](https://github.com/tier4/AWSIM): a Unity-based SSv2 simulator.
- [carla-rust](https://github.com/jerry73204/carla-rust): Rust bindings for CARLA's C++ API.
- [play_launch](https://pypi.org/project/play-launch/): `ros2 launch` with per-node logs and a
  web UI.

## License

[Apache 2.0](LICENSE).
