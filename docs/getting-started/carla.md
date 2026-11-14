# Getting started on CARLA

From nothing to a scenario of your own, driven first by CARLA's own simple autopilot and
then by Autoware. Five stages; each ends with something you can see working.

| Stage | You get | Time |
|---|---|---|
| [1. Install](#1-install) | the workspace built, the environment file written | ~30 min, mostly the first build |
| [2. A map](#2-a-map) | Town01 for Autoware and the scenarios | ~10 min |
| [3. First run](#3-first-run-carlas-simple-autopilot) | a starter scenario passing, no Autoware | ~10 min |
| [4. Your scenario](#4-your-scenario) | your own goal on your own road | ~15 min |
| [5. Autoware](#5-autoware) | the same scenario driven by Autoware | ~20 min |

[Getting started on Isaac Sim](isaac.md) follows the same stages.

Commands use `play_launch launch <package> <file> <arguments>`. `ros2 launch` takes the same
arguments, but play_launch must still be installed: the stack starts the autopilot, the
vehicle agent and the scenario as play_launch processes of their own.

## 1. Install

You need:

- **Ubuntu 22.04 and ROS 2 Humble** ([install](https://docs.ros.org/en/humble/Installation.html)).
- **CARLA 0.9.16**, the packaged release
  ([quick start](https://carla.readthedocs.io/en/0.9.16/start_quickstart/)), and an NVIDIA
  GPU. Below, `~/CARLA_0.9.16` is where you unpacked it.
- **Autoware 1.5.0**, e.g. the binary packages under `/opt/autoware/1.5.0`
  ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/)).
  The build uses its `setup.bash`; Autoware drives from stage 5 on.
- **Rust**, colcon's Rust plugin, CARLA's Python API and
  [play_launch](https://pypi.org/project/play-launch/) 0.30.0 or newer, which rosdep cannot
  install:

  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  sudo apt install libz3-dev ros-humble-rclcpp-components ros-humble-class-loader
  pip install colcon-cargo-ros2 carla==0.9.16 networkx shapely
  pip install 'play_launch>=0.30.0'
  play_launch --version
  ```

Then build, the ROS way:

```bash
git clone --recurse-submodules \
    https://github.com/NEWSLabNTU/scenario-world-bridge.git
cd scenario-world-bridge
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep install --from-paths src --ignore-src -y   # apt packages: needs sudo
export CMAKE_POLICY_VERSION_MINIMUM=3.5            # CMake 4 and the vendored embree
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
```

Once per machine, give Autoware a writable data directory (TensorRT writes its engines
there):

```bash
source install/setup.bash
ros2 run autoware_sim_profile setup_autoware_data
```

**The environment file.** Every terminal from here on starts by sourcing one file, say
`~/swb.bash`:

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

The DDS lines put every process, and every `ros2` command you type, on the same transport:
without them a terminal falls back to Fast DDS and sees nothing the stack publishes. The
domains keep the scenario's side and the ego's Autoware apart;
[ROS domains and DDS](../domains-and-dds.md) says why. Add `CARLA_HOST` and `CARLA_PORT`
if CARLA is not at `localhost:2000`.

## 2. A map

Autoware and the scenarios read a map directory per CARLA town: a Lanelet2 map and a point
cloud, in CARLA's coordinates. This project uses the towns converted by TUM's
[CARLA-Autoware-Bridge](https://github.com/TUMFTM/Carla-Autoware-Bridge) project
(Town01, 02, 03, 05 and 10; LGPL-3.0). TUM's own download link no longer works, so we keep
their pack, unchanged, as a release of
[NEWSLabNTU/carla-autoware-maps](https://github.com/NEWSLabNTU/carla-autoware-maps/releases).

One command downloads it, checks its checksum, unpacks it into `data/carla-autoware-bridge/`
and repairs what Autoware and the scenario simulator need fixed (traffic-light tags, missing
lanelet types, turn directions, speed limits from CARLA's own road data, ...). The repairs
read CARLA's OpenDRIVE files, which the CARLA release ships:

```bash
source ~/swb.bash
cd ~/scenario-world-bridge
SWB_XODR_DIR=~/CARLA_0.9.16/CarlaUE4/Content/Carla/Maps/OpenDrive \
    scripts/download_maps.sh
ls $CARLA_MAPS
# Town01  Town02  Town03  Town05  Town10
```

Run it again whenever you pull: a repair added later reaches maps you already have.

One file per town is CARLA's, not TUM's: which Lanelet2 traffic light is which CARLA light.
Generate it once per town with CARLA running (start CARLA as in stage 3, Terminal 1, then):

```bash
ros2 run swb_carla_world swb_carla_world \
    --generate-signal-table $CARLA_MAPS/Town01 \
    --ros-args -p carla_port:=2000 -p port:=5599
ls $CARLA_MAPS/Town01/carla
# traffic_lights.yaml
```

It loads Town01 in CARLA, writes the table and exits. CARLA calls Town10 `Town10HD`; the
directory stays `Town10`.

## 3. First run: CARLA's simple autopilot

Four terminals, each with `source ~/swb.bash` first. The ego is driven by CARLA's own
`BasicAgent`: it follows the route to the scenario's goal and stops for traffic lights and
vehicles. No Autoware yet.

**Terminal 1: CARLA**, headless. It stays up between runs:

```bash
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    ~/CARLA_0.9.16/CarlaUE4.sh \
    -RenderOffScreen \
    -nosound \
    -carla-rpc-port=2000
```

Ready when it serves its port, after one to three minutes: `ss -lnt | grep ':2000 '`. The
`VK_ICD_FILENAMES` line makes CARLA render on the NVIDIA card on a laptop or a host with
two GPUs.

**Terminal 2: the stack**, without an autopilot of its own (`ego_launch:=none`): CARLA's
world server, the bridge and the agent relay, each in its domain:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    ego_launch:=none
```

Leave it running; scenarios come and go. Its web UI, `http://127.0.0.1:8084`, lists every
process with its log. Ask the world server whether it is up, without changing anything:

```bash
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5591
```

**Terminal 3: the ego's driver**, CARLA's BasicAgent:

```bash
echo "carla_pythonapi: $HOME/CARLA_0.9.16/PythonAPI/carla" > ~/carla_basic.yaml
ros2 run swb_agent carla_basic \
    --relay tcp://localhost:${AGENT_PORT:-5560} \
    --entity ego \
    --config ~/carla_basic.yaml
```

The relay (`scenario_agent_relay` in the web UI) reports `agent ... registered`.

**Terminal 4: a starter scenario.** The launch ends with the scenario:

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$S/basic/town01_ego_drive.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/first_run
cat /tmp/first_run/scenario_test_runner/result.junit.xml
```

The bridge spawns the ego (its log: `Spawned ego 'ego'`), the agent gets its goal
(Terminal 3: `set_goal`, later `ARRIVED`), and after about a minute the launch ends.
`failures="0" errors="0"` in the JUnit file means it passed; a failed run carries a
`<failure message="...">` with the reason.

Keep Terminals 1 to 3 up for stage 4.

## 4. Your scenario

Copy the starter and move its goal. `carla_poses` reads the town CARLA has loaded and
prints poses already in the map's frame (CARLA's y and headings are flipped), as
OpenSCENARIO elements:

```bash
mkdir -p ~/my_scenarios
S=$(ros2 pkg prefix --share swb_examples)/scenarios
cp $S/basic/town01_ego_drive.xosc ~/my_scenarios/my_drive.xosc
ros2 run swb_launch carla_poses spawn-points | head
ros2 run swb_launch carla_poses ahead 2 --distance 120
```

In `my_drive.xosc`, put the `start` line's `<WorldPosition>` in the ego's `TeleportAction`
and the `end` line's in its `AcquirePositionAction` and in the `ReachPositionCondition`
that ends the scenario. Then run it as in stage 3, Terminal 4, with
`scenario:=$HOME/my_scenarios/my_drive.xosc` and an `output_directory` of its own.

[Prepare a scenario](../tutorials/01-prepare-a-scenario.md) goes further: another car on
the road, a speed limit, what to check. [Writing scenarios](../design/scenario-authoring.md)
lists what the bridge supports.

## 5. Autoware

Same scenario, Autoware driving. Stop the BasicAgent (Ctrl-C in Terminal 3) and the stack
(Ctrl-C in Terminal 2); CARLA stays up. Then start the stack again with its default
autopilot, Autoware on CARLA, and its vehicle agent:

```bash
# Terminal 2
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    ego_launch_args:="rviz:=true"
```

`map_path` must be the town the scenario drives in. Autoware is about 90 processes and an
RViz window. Wait until this exits 0:

```bash
# Terminal 3
ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} ros2 run swb_launch ego_ready --timeout 600
# [ego-health] ok: ...
```

The first start on a machine builds Autoware's TensorRT engines and can take several
minutes. Then run your scenario exactly as in stage 3, Terminal 4. In RViz the route
appears and the ego drives to your goal.

For a script or CI, one command does it all and ends with the scenario:

```bash
ARGS="output_directory:=/tmp/my_drive initialize_duration:=600 global_timeout:=900"
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch carla.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    scenario:=$HOME/my_scenarios/my_drive.xosc \
    scenario_args:="$ARGS" \
    exit_on_scenario_end:=true
cat /tmp/my_drive/scenario_test_runner/result.junit.xml
```

The scenario starts at once, while Autoware is still coming up, so `ARGS` gives Autoware
ten minutes to be ready (`initialize_duration`) and the whole run fifteen
(`global_timeout`).

[Build your own stack](../build-your-own-stack.md) shows the stack's other slots: your car,
your autopilot, more cars.

## When it does not work

| Symptom | Look at |
|---|---|
| The build fails at `embree_vendor`: `Compatibility with CMake < 3.5 has been removed from CMake` | a CMake 4 (pip or Kitware) ahead of Ubuntu's: `export CMAKE_POLICY_VERSION_MINIMUM=3.5` before `colcon build`, as stage 1 does |
| The build fails in `traffic_simulator`: `-Werror=maybe-uninitialized` in boost's `rescale_policy.hpp` | a GCC newer than Ubuntu 22.04's 11: add `-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized` to `--cmake-args` |
| `scripts/download_maps.sh`: `checksum mismatch` | the download was cut short; delete `data/carla-autoware-bridge.zip` and run it again |
| A repair prints `no OpenDRIVE for it ... skipped` | `SWB_XODR_DIR` does not point at the CARLA release's `CarlaUE4/Content/Carla/Maps/OpenDrive` |
| The bridge logs `Cannot read the signal table`, or the scenario's traffic lights never change | stage 2's `--generate-signal-table` for that town |
| The stack stops at once: `stack: ... use 0-57` or `are both 9` | the domains in the environment file; [ROS domains and DDS](../domains-and-dds.md) |
| The ego and the scenario never start | play_launch missing or older than 0.30.0: `play_launch --version` |
| The scenario fails with `... waited for the Autoware state ...` | `ego_ready` must say ok before the scenario starts; `map_path` must be the scenario's town |
| The bridge logs `CARLA version mismatch` | CARLA and the build are different versions (README, *Other CARLA versions*) |
| `ros2 topic list` is empty, `ego_ready` never says ok | the terminal did not source `~/swb.bash` (its `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`), or the wrong `ROS_DOMAIN_ID` |

Stop with Ctrl-C on the stack: it ends everything it started. Leave CARLA up between runs;
it takes minutes to start.

## Next

- [Run and judge](../tutorials/02-run-and-judge.md): suites of scenarios, CARLA's collision
  check beside SSv2's verdict.
- [Other road users and more AVs](../tutorials/03-other-road-users.md): pedestrians, other
  cars, a second Autoware vehicle.
- [Build your own stack](../build-your-own-stack.md): your car, your autopilot, more cars.
- [Writing an agent](../agent-interface.md): drive with your own autopilot.

## Appendix: running the parts in separate terminals

The stack starts its parts in one launch. You can also start them one by one, for example
to restart Autoware alone or to run it on another host. CARLA runs as in stage 3,
Terminal 1; every terminal sources `~/swb.bash`.

**The simulation side**: CARLA's world server, the bridge and the agent relay, in the
scenario domain:

```bash
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8084 \
    swb_launch carla_simulation.launch.xml
```

Ready when the bridge logs `ZMQ server ready` and the relay `agents register on
tcp://0.0.0.0:5560`.

**The ego**: CARLA's BasicAgent as in stage 3, Terminal 3, or Autoware with its vehicle
agent, in the ego domain:

```bash
ROS_DOMAIN_ID=${EGO_ROS_DOMAIN_ID:-1} play_launch launch \
    --container-mode isolated \
    --composable-respawn on-crash \
    --web-addr 127.0.0.1:8082 \
    swb_launch ego_av.launch.xml \
    map_path:=$CARLA_MAPS/Town01 \
    relay:=tcp://localhost:${AGENT_PORT:-5560} \
    launch_rviz:=true
```

Wait for `ego_ready` as in stage 5. `ego_av.launch.xml` is not quite the stack's ego: it
uses the laser-scan occupancy grid where the stack uses the point-cloud one.

**The scenario**: as in stage 3, Terminal 4.

Stop the ego first, then the simulation side.
