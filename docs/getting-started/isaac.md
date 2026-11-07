# Getting started on Isaac Sim

From nothing to Autoware driving a scenario of your own in
[Isaac Sim](https://developer.nvidia.com/isaac/sim) 6.1, on CARLA's Town01 rebuilt for Isaac.
Same five stages as [Getting started on CARLA](carla.md), and the same scenario files: a
scenario names its map, not its simulator.

| Stage | You get | Time |
|---|---|---|
| [1. Install](#1-install) | Isaac Sim and the workspace built | ~1 h, mostly downloads |
| [2. Assets and a map](#2-assets-and-a-map) | Town01 as an Isaac world, beside its Autoware map | ~20 min, plus the copy |
| [3. The world server](#3-the-world-server) | Isaac running Town01, checked against the world protocol | ~15 min the first time |
| [4. Autoware](#4-autoware) | a starter scenario driven by Autoware, passing | ~20 min |
| [5. Your scenario](#5-your-scenario) | your own goal on your own road | ~15 min |

On Isaac the ego's vehicle runs inside the Isaac world server: its PhysX body, sensors and
vehicle interface come from [aib](https://github.com/NEWSLabNTU/autoware_isaac_bridge), so
there is no separate vehicle bridge to start. Autoware connects to it over ROS.

## 1. Install

You need:

- **Ubuntu 22.04 and ROS 2 Humble** ([install](https://docs.ros.org/en/humble/Installation.html)).
- **An NVIDIA RTX GPU with 24 GB**, a recent driver (tested: RTX 4090, driver 615), and
  **64 GB of RAM or more**: the world server holds about 17.5 GB with Town01, Autoware another
  10-20 GB. Isaac Sim's own [requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
  apply.
- **About 50 GB** of disk for Isaac Sim and its caches, and **about 25 GB** for the asset set
  (stage 2). Put both on a fast disk.
- **Autoware 1.5.0**, e.g. the binary packages under `/opt/autoware/1.5.0`
  ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/)).
- **Rust**, colcon's Rust plugin, and [uv](https://docs.astral.sh/uv/) (Isaac's Python):

  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  curl -LsSf https://astral.sh/uv/install.sh | sh
  pip install colcon-cargo-ros2
  ```

Build the workspace, the ROS way, exactly as for CARLA:

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/scenario-world-bridge.git
cd scenario-world-bridge
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep install --from-paths src --ignore-src -y      # apt packages: needs sudo
export CMAKE_POLICY_VERSION_MINIMUM=3.5               # CMake 4 and the vendored embree
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release
source install/setup.bash
ros2 run autoware_sim_profile setup_autoware_data     # once per machine
```

Then Isaac Sim itself, pip-installed into a Python 3.12 environment of its own (it does not
touch the system's Python), and Autoware's messages built for that Python. `ISAAC_ROOT` is
where it goes:

```bash
export ISAAC_ROOT=~/isaac-sim
src/swb_isaac_world/scripts/install_isaac.sh          # ends with: isaacsim 6.1.0.0, INSTALL-DONE
src/swb_isaac_world/scripts/build_msgs.sh             # into $ISAAC_ROOT/msgs
```

**Every terminal from here on** starts with the same lines. Put them in a file, say
`~/swb_isaac.bash`, and `source` it:

```bash
source /opt/autoware/1.5.0/setup.bash
source ~/scenario-world-bridge/install/setup.bash        # your clone
export CARLA_MAPS=~/scenario-world-bridge/data/carla-autoware-bridge
export ISAAC_ROOT=~/isaac-sim
export CARLA_USD=~/carla-usd                              # stage 2
export SWB_VEHICLE_USD=$CARLA_USD/vehicles
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(ros2 pkg prefix swb_launch)/share/swb_launch/config/cyclonedds-localhost.xml
```

The last two put every process on the same DDS. Without them a terminal falls back to Fast
DDS and sees nothing the others publish.

## 2. Assets and a map

**The asset set.** Isaac renders CARLA's content: its towns, vehicles, props and walkers,
exported from CARLA's editor to USD. The set is about 24 GB, one directory. Request it from
NEWSLab ([NEWSLabNTU](https://github.com/NEWSLabNTU)); building it yourself takes a CARLA
editor build (about 150 GB and several hours,
[Isaac assets](../design/isaac-assets.md#making-a-set)).

NEWSLab members: copy it from the lab NAS, `autoveh/dataset/2026-10-23_carla-0.9.16-isaac-usd/`.

Put it at `$CARLA_USD` and check the copy against its manifest:

```bash
(cd src/swb_isaac_world && python3 -m swb_isaac_world.asset_set check $CARLA_USD)
```

It lists every missing or changed file and exits non-zero if there is one.

**The Autoware map.** As for CARLA: TUM's converted towns, downloaded and repaired by one
script. Its repairs read CARLA's OpenDRIVE files, which the asset set carries
(`$CARLA_USD/<Town>/main.xodr`), so no CARLA install is needed:

```bash
SWB_XODR_DIR=$CARLA_USD scripts/download_maps.sh
ls $CARLA_MAPS
# Town01  Town02  Town03  Town05  Town10
```

**The world file.** One command per town writes the world's collider layer and
`<map>/isaac/world.yaml`, which tells the world server which USD world goes with that map:

```bash
src/swb_isaac_world/scripts/isaac_town.sh Town01
ls $CARLA_MAPS/Town01/isaac
# world.yaml
```

With the town's export present in the asset set it does not run CARLA's editor. The world
file names the layer by its absolute path on this machine, so run it again after moving the
asset set.

## 3. The world server

Three terminals. The world server is long-lived: it loads the world once and serves every
scenario after it.

```bash
# Terminal 1: the Isaac world server, Town01 preloaded, the ego's ROS domain 1
EGO_ROS_DOMAIN_ID=1 ros2 launch swb_isaac_world swb_isaac_world.launch.xml \
    world:=$CARLA_MAPS/Town01/isaac/world.yaml render:=headless
```

It answers nothing until the world is loaded and rendered once. The first time on a machine
that can take a quarter of an hour (Isaac compiles the town's materials); later starts are
faster.
Ready when it logs:

```
swb_world_server: isaac world server ready on tcp://127.0.0.1:5590 (Isaac Sim 6.1.0.0 ...)
```

Ask it from another terminal, without changing anything:

```bash
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5590 --wait 1800
```

It prints the simulator, what it offers (signals, contacts, road conditions, ...) and
whether it is ready. Then check it against the world protocol, once, before any scenario:
the check loads the map, spawns and moves bodies, sets a traffic light and reads it back.

```bash
ros2 run swb_world_check swb_world_check conformance tcp://127.0.0.1:5590 \
    --map $CARLA_MAPS/Town01
```

On a machine with two GPUs, keep the world server on one: `ISAAC_KIT_ARGS="--/renderer/multiGpu/enabled=false
--/renderer/activeGpu=0"` in front of Terminal 1's command. `render:=window` instead of
`headless` opens Isaac's window on your `DISPLAY`.

```bash
# Terminal 2: the simulation side -- the bridge driving the world server, the agent relay
ROS_DOMAIN_ID=9 ros2 launch swb_launch isaac_simulation.launch.xml
```

Ready when the bridge logs `ZMQ server ready` and the relay `agents register on
tcp://0.0.0.0:5560`. Leave it running too.

## 4. Autoware

```bash
# Terminal 3: Autoware for the Isaac ego, its vehicle agent registered with the relay
ROS_DOMAIN_ID=1 ros2 launch aib_launch isaac_simulator.launch.xml \
    map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560 rviz:=true \
    extra_launch:=$(ros2 pkg prefix swb_launch)/share/swb_launch/launch/vehicle_agent.launch.xml
```

`ROS_DOMAIN_ID=1` here must be Terminal 1's `EGO_ROS_DOMAIN_ID`: the world server publishes
the ego's sensors, status and `/clock` there and takes Autoware's commands from it. Two
domains in all, and no other command fixes either number:

| Domain | Who must share it |
|---|---|
| the ego's (1) | Terminal 1's `EGO_ROS_DOMAIN_ID`, Terminal 3, `ego_ready`, the `/clicked_point` echo |
| the scenario's (9) | Terminals 2 and 4 (the relay's log names it) |
`extra_launch` adds the vehicle agent, which takes the scenario's goals from the relay and
engages Autoware.

About 90 processes and an RViz window. Wait until this exits 0:

```bash
ROS_DOMAIN_ID=1 ros2 run swb_launch ego_ready --timeout 600
# [ego-health] ok: ...
```

The first start builds Autoware's TensorRT engines and can take several minutes; it does so
at every start unless your TensorRT is the one Autoware was built with (10.8.0;
[User Guide](../user-guide.md#4-run)).

Then a starter scenario:

```bash
# Terminal 4: one scenario; the launch ends with the scenario
ROS_DOMAIN_ID=9 ros2 launch swb_launch scenario.launch.xml \
    scenario:=$(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/basic/town01_ego_drive.xosc \
    output_directory:=/tmp/isaac_first
cat /tmp/isaac_first/scenario_test_runner/result.junit.xml
```

The bridge spawns the ego in Isaac, the agent sets the route and engages, and the ego drives
about 70 m west. `failures="0" errors="0"` means it passed. The same file runs unchanged on
CARLA.

## 5. Your scenario

Copy the starter and move its goal:

```bash
mkdir -p ~/my_scenarios
cp $(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/basic/town01_ego_drive.xosc \
    ~/my_scenarios/my_drive.xosc
```

Positions are in the map's frame, the same on Isaac and CARLA. To find one, use RViz
(Terminal 3): **Publish Point** on a lane publishes its x and y on `/clicked_point`. The topic
exists only from the first click, so start the echo first; it waits for the click:

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /clicked_point --once
# then Publish Point, and click the lane
```

The heading `h` is in radians, counter-clockwise from +x (east): a westbound lane is 3.14.
In `my_drive.xosc`, put the goal in the ego's `AcquirePositionAction` and in the
`ReachPositionCondition` that ends the scenario, then run it as in stage 4, Terminal 4,
with `scenario:=$HOME/my_scenarios/my_drive.xosc`. With CARLA at hand, `carla_poses` prints
poses directly (Getting started on CARLA, stage 4).

[Tutorial 2](../tutorials/02-prepare-a-scenario.md) goes further (another car, what to
check); [Tutorial 5](../tutorials/05-other-road-users.md) adds pedestrians and other
vehicles.

## When it does not work

| Symptom | Look at |
|---|---|
| The build fails at `embree_vendor`: `Compatibility with CMake < 3.5 has been removed from CMake` | a CMake 4 (pip or Kitware) ahead of Ubuntu's: `export CMAKE_POLICY_VERSION_MINIMUM=3.5` before `colcon build`, as stage 1 does |
| The build fails in `traffic_simulator`: `-Werror=maybe-uninitialized` in boost's `rescale_policy.hpp` | a GCC newer than Ubuntu 22.04's 11: add `-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized` to `--cmake-args` |
| Every process prints `'NetworkInterfaceAddress': deprecated element`; the world server logs a DEBUG traceback, `No module named 'em'` | both harmless |
| `conformance` fails `a lamp shows the colour it was set to ... no lamp for OpenDRIVE id(s) 362` | a world without traffic lights (a point-cloud world); the SimReady world has them |
| `install_isaac.sh`: `uv: command not found` | install uv first (stage 1) |
| The world server never says ready | the first preload compiles materials: wait, and watch its log; `hello --wait` shows whether it answers |
| The world server is `not up after 1200 s`, its Kit log ends at `omni.client` and it runs at 100 % CPU | a Kit cache link (`~/.cache/ov`, `~/.local/share/ov`, `~/.nvidia-omniverse`) points at a deleted directory, e.g. after reinstalling Isaac; create its target (`mkdir -p "$(readlink -m ~/.cache/ov)"`), or run `install_isaac.sh` again, which does |
| The world server dies or hangs with an out-of-memory error | it needs the GPU (24 GB) and about 17.5 GB of RAM to itself; close other GPU work, and on two GPUs pin one (`ISAAC_KIT_ARGS`) |
| It restarts after dying and is unavailable for minutes | the launch respawns it and it preloads the world again; wait for the ready line |
| `isaac_town.sh`: `no Autoware map for Town01` | `CARLA_MAPS` is unset or stage 2's maps are missing |
| The ego never moves, `ego_ready` times out | Terminal 3's `ROS_DOMAIN_ID` differs from Terminal 1's `EGO_ROS_DOMAIN_ID`, or a terminal lacks the `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI` lines |
| Nothing ever connects | a `ROS_DOMAIN_ID` above 57 (the DDS profile's port range); [User Guide §6](../user-guide.md#6-when-things-go-wrong) |

Stop with Ctrl-C: Terminal 3, then 2. Leave the world server up between runs; it takes
minutes to load a world.

**A second checkout on the same machine** moves its ports and both domains, each in every
command that names it. For example, ports +10 and domains 11 and 19:

```bash
ISAAC_PORT=5600 EGO_ROS_DOMAIN_ID=11 ros2 launch swb_isaac_world swb_isaac_world.launch.xml ...  # T1
ROS_DOMAIN_ID=19 ros2 launch swb_launch isaac_simulation.launch.xml \
    isaac_port:=5600 ssv2_port:=5565 agent_port:=5570                                     # T2
ROS_DOMAIN_ID=11 ros2 launch aib_launch isaac_simulator.launch.xml ... relay:=tcp://localhost:5570  # T3
ROS_DOMAIN_ID=19 ros2 launch swb_launch scenario.launch.xml ... port:=5565                 # T4
```

## Next

- [Tutorial 3, run and judge](../tutorials/03-run-and-judge.md): suites of scenarios.
- [Tutorial 5, other road users](../tutorials/05-other-road-users.md): pedestrians, other
  cars, other autonomous vehicles.
- [Tutorial 6, configure the system](../tutorials/06-configure.md): the world server's and the
  bridge's configuration, your own Autoware.
