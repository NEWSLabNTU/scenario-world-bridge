# Getting started on CARLA

From nothing to a scenario of your own, driven first by CARLA's own simple autopilot and
then by Autoware. Five stages; each ends with something you can see working.

| Stage | You get | Time |
|---|---|---|
| [1. Install](#1-install) | the workspace built | ~30 min, mostly the first build |
| [2. A map](#2-a-map) | Town01 for Autoware and the scenarios | ~10 min |
| [3. First run](#3-first-run-carlas-simple-autopilot) | a starter scenario passing, no Autoware | ~10 min |
| [4. Your scenario](#4-your-scenario) | your own goal on your own road | ~15 min |
| [5. Autoware](#5-autoware) | the same scenario driven by Autoware | ~20 min |

[Getting started on Isaac Sim](isaac.md) follows the same stages.

## 1. Install

You need:

- **Ubuntu 22.04 and ROS 2 Humble** ([install](https://docs.ros.org/en/humble/Installation.html)).
- **CARLA 0.9.16**, the packaged release ([quick start](https://carla.readthedocs.io/en/0.9.16/start_quickstart/)),
  and an NVIDIA GPU. Below, `~/CARLA_0.9.16` is where you unpacked it.
- **Autoware 1.5.0** for stage 5 only, e.g. the binary packages under `/opt/autoware/1.5.0`
  ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/)).
  The build below uses its `setup.bash`.
- **Rust**, colcon's Rust plugin and CARLA's Python API, which rosdep cannot install:

  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  pip install colcon-cargo-ros2 carla==0.9.16 networkx shapely
  ```

Then build, the ROS way:

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/scenario-world-bridge.git
cd scenario-world-bridge
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep install --from-paths src --ignore-src -y      # apt packages: needs sudo
export CMAKE_POLICY_VERSION_MINIMUM=3.5               # CMake 4 and the vendored embree
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release
```

Once per machine, give Autoware a writable data directory (TensorRT writes its engines
there):

```bash
source install/setup.bash
ros2 run autoware_sim_profile setup_autoware_data
```

**Every terminal from here on** starts with the same lines. Put them in a file, say
`~/swb.bash`, and `source` it:

```bash
source /opt/autoware/1.5.0/setup.bash
source ~/scenario-world-bridge/install/setup.bash        # your clone
export CARLA_MAPS=~/scenario-world-bridge/data/carla-autoware-bridge
```

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
cd ~/scenario-world-bridge
SWB_XODR_DIR=~/CARLA_0.9.16/CarlaUE4/Content/Carla/Maps/OpenDrive scripts/download_maps.sh
ls $CARLA_MAPS
# Town01  Town02  Town03  Town05  Town10
```

Run it again whenever you pull: a repair added later reaches maps you already have.

One file per town is CARLA's, not TUM's: which Lanelet2 traffic light is which CARLA light.
Generate it once per town with CARLA running (start CARLA as in stage 3, step 1, then):

```bash
ros2 run swb_carla_world swb_carla_world --generate-signal-table $CARLA_MAPS/Town01 \
    --ros-args -p carla_port:=2000 -p port:=5599
ls $CARLA_MAPS/Town01/carla
# traffic_lights.yaml
```

It loads Town01 in CARLA, writes the table and exits. CARLA calls Town10 `Town10HD`; the
directory stays `Town10`.

## 3. First run: CARLA's simple autopilot

Four terminals. The ego is driven by CARLA's own `BasicAgent`: it follows the route to the
scenario's goal and stops for traffic lights and vehicles. No Autoware.

```bash
# Terminal 1: CARLA, headless
~/CARLA_0.9.16/CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000
```

Ready when it serves its port, after one to three minutes: `ss -lnt | grep ':2000 '`. On a
laptop or a host with two GPUs, put `VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json`
in front so CARLA renders on the NVIDIA card.

```bash
# Terminal 2: the simulation side -- CARLA's world server, the bridge, the agent relay
ROS_DOMAIN_ID=9 ros2 launch swb_launch carla_simulation.launch.xml
```

Ready when the bridge logs `ZMQ server ready` and the relay `agents register on
tcp://0.0.0.0:5560`. Leave it running; scenarios come and go.

```bash
# Terminal 3: the ego's driver, CARLA's BasicAgent
echo "carla_pythonapi: $HOME/CARLA_0.9.16/PythonAPI/carla" > ~/carla_basic.yaml
ros2 run swb_agent carla_basic --relay tcp://localhost:5560 --entity ego \
    --config ~/carla_basic.yaml
```

Terminal 2's relay reports `agent ... registered`.

```bash
# Terminal 4: a starter scenario; the launch ends with the scenario
ROS_DOMAIN_ID=9 ros2 launch swb_launch scenario.launch.xml \
    scenario:=$(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/basic/town01_ego_drive.xosc \
    output_directory:=/tmp/first_run
cat /tmp/first_run/scenario_test_runner/result.junit.xml
```

The bridge spawns the ego (Terminal 2: `Spawned ego 'ego'`), the agent gets its goal
(Terminal 3: `set_goal`, later `ARRIVED`), and after about a minute the launch ends.
`failures="0" errors="0"` in the JUnit file means it passed; a failed run carries a
`<failure message="...">` with the reason.

Keep Terminals 1-3 up for stage 4.

## 4. Your scenario

Copy the starter and move its goal. `carla_poses` reads the town CARLA has loaded and
prints poses already in the map's frame (CARLA's y and headings are flipped), as
OpenSCENARIO elements:

```bash
mkdir -p ~/my_scenarios
cp $(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/basic/town01_ego_drive.xosc \
    ~/my_scenarios/my_drive.xosc
ros2 run swb_launch carla_poses spawn-points | head
ros2 run swb_launch carla_poses ahead 2 --distance 120
```

In `my_drive.xosc`, put the `start` line's `<WorldPosition>` in the ego's `TeleportAction`
and the `end` line's in its `AcquirePositionAction` and in the `ReachPositionCondition`
that ends the scenario. Then run it as in stage 3, Terminal 4, with
`scenario:=$HOME/my_scenarios/my_drive.xosc`.

[Tutorial 2](../tutorials/02-prepare-a-scenario.md) goes further: another car on the road,
a speed limit, what to check. [Writing scenarios](../design/scenario-authoring.md) lists what
the bridge supports.

## 5. Autoware

Same scenario, Autoware driving. Stop the BasicAgent (Ctrl-C in Terminal 3); Terminals 1
and 2 stay.

```bash
# Terminal 3: Autoware on CARLA, its vehicle agent registered with the relay
ROS_DOMAIN_ID=1 ros2 launch swb_launch ego_av.launch.xml \
    map_path:=$CARLA_MAPS/Town01 relay:=tcp://localhost:5560 launch_rviz:=true
```

About 90 processes and an RViz window. The relay soon says `ego: agent up`, but Autoware
is still starting then. Wait until this exits 0:

```bash
ROS_DOMAIN_ID=1 ros2 run swb_launch ego_ready --timeout 600
# [ego-health] ok: ...
```

The first start on a machine builds Autoware's TensorRT engines and can take several
minutes. Then run your scenario exactly as before (Terminal 4). In RViz the route appears
and the ego drives to your goal.

`map_path` must be the town the scenario drives in.

## When it does not work

| Symptom | Look at |
|---|---|
| The build fails at `embree_vendor`: `Compatibility with CMake < 3.5 has been removed from CMake` | a CMake 4 (pip or Kitware) ahead of Ubuntu's: `export CMAKE_POLICY_VERSION_MINIMUM=3.5` before `colcon build`, as stage 1 does |
| The build fails in `traffic_simulator`: `-Werror=maybe-uninitialized` in boost's `rescale_policy.hpp` | a GCC newer than Ubuntu 22.04's 11: add `-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized` to `--cmake-args` |
| `scripts/download_maps.sh`: `checksum mismatch` | the download was cut short; delete `data/carla-autoware-bridge.zip` and run it again |
| A repair prints `no OpenDRIVE for it ... skipped` | `SWB_XODR_DIR` does not point at the CARLA release's `CarlaUE4/Content/Carla/Maps/OpenDrive` |
| The bridge logs `Cannot read the signal table`, or the scenario's traffic lights never change | stage 2's `--generate-signal-table` for that town |
| Terminal 4 fails with `... waited for the Autoware state ...` | `ego_ready` must say ok before the scenario starts; `map_path` must be the scenario's town |
| The bridge logs `CARLA version mismatch` | CARLA and the build are different versions (README, *Other CARLA versions*) |
| Nothing ever connects | a terminal whose environment sets another `RMW_IMPLEMENTATION` or `CYCLONEDDS_URI` than the others, or a `ROS_DOMAIN_ID` above 57; [User Guide §6](../user-guide.md#6-when-things-go-wrong) |

Stop with Ctrl-C: Terminal 3 first, then 2. Leave CARLA up between runs; it takes minutes
to start.

## Next

- [Tutorial 3, run and judge](../tutorials/03-run-and-judge.md): suites of scenarios, CARLA's
  collision check beside SSv2's verdict.
- [Tutorial 4, demo run](../tutorials/04-demo-run.md): two Autoware vehicles, a screencast.
- [Writing an agent](../agent-interface.md): drive with your own autopilot.
- [play_launch](https://pypi.org/project/play-launch/) runs the same launches with a log per
  node and a web UI, worth it for Autoware's ~90 processes:
  `play_launch launch --web-addr 127.0.0.1:8082 swb_launch ego_av.launch.xml ...`.
