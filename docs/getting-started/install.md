# Install

The one page for installing and building scenario-world-bridge (swb), writing the
environment file and the host settings file. Every other page starts from here. The
simulator itself comes next: [CARLA](carla.md) or [Isaac Sim](isaac.md).

About 30 minutes, most of it the first build.

## What you need

- **Ubuntu 22.04 and ROS 2 Humble** ([install](https://docs.ros.org/en/humble/Installation.html)).
- **Autoware 1.5.0**, for example the binary packages under `/opt/autoware/1.5.0`
  ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/)).
  The build uses its `setup.bash`.
- **An NVIDIA GPU** that runs the simulator beside Autoware (CARLA holds about 2 GB of video
  memory on Town01; Isaac Sim needs an RTX card with 24 GB).
- **Rust**, colcon's Rust plugin and [play_launch](https://pypi.org/project/play-launch/)
  0.30.0 or newer, which rosdep cannot install:

  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  sudo apt install libz3-dev ros-humble-rclcpp-components ros-humble-class-loader
  pip install colcon-cargo-ros2 'play_launch>=0.30.0'
  play_launch --version
  ```

## Build

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

Once per machine, let DDS carry point clouds: the default kernel socket buffers (208 kB)
are too small, and a point cloud then never arrives. This installs Autoware's settings:

```bash
sudo install -m 644 config/99-cyclonedds-max.conf /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-cyclonedds-max.conf
```

## The environment file

Every terminal sources one file, say `~/swb.bash`. It sources ROS, Autoware and the
workspace, and nothing else:

```bash
# ~/swb.bash
source /opt/autoware/1.5.0/setup.bash
source ~/scenario-world-bridge/install/setup.bash
```

The ROS domains, the DDS and the ports are not exported here: the stack sets them for
every process it starts, from your seat (below). A terminal of your own that should see
the stack's topics asks for them: `eval "$(ros2 run swb seat env scenario)"`
([ROS domains and DDS](../explanation/domains-and-dds.md)).

## The host settings file

What differs between machines lives in `~/.config/swb/host.yaml`, never in a project and
never in an export. Every key is optional; these are the defaults you are most likely to
change:

```yaml
# ~/.config/swb/host.yaml
logs: ~/.local/state/swb/logs    # every run's logs: put it on a fast disk (an SSD)
cache: ~/.cache/swb              # the downloaded datasets, shared by all your projects
carla: {host: localhost, port: 2000}
```

A project can override a key for one checkout in `<project>/.swb/local.yaml`, which git
ignores. Every key, and the seat table an administrator sets in `/etc/swb/host.yaml`:
[Host settings](../reference/host-settings.md).

## Check

```bash
source ~/swb.bash
ros2 run swb seat show
```

You see the host's seats, each with its ROS domains and ports, and which is yours:

```
seats (defaults); locks in /tmp/swb-seats
seat 0: scenario domain 1, pilot domains 2-8, SSv2 5555 (+5556, 5557), relay 5560, ...
...
your seat: auto (the first free one when the stack starts)
```

A seat is one stack's share of the host: [Several stacks on one host](../how-to/several-stacks.md).

## If it does not work

| Symptom | Fix |
|---|---|
| `embree_vendor`: `Compatibility with CMake < 3.5 has been removed from CMake` | a CMake 4 (pip or Kitware) ahead of Ubuntu's: `export CMAKE_POLICY_VERSION_MINIMUM=3.5` before `colcon build`, as above |
| `traffic_simulator`: `-Werror=maybe-uninitialized` in boost's `rescale_policy.hpp` | a GCC newer than Ubuntu 22.04's 11: add `-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized` to `--cmake-args` |
| `traffic_simulator` cannot find a dependency's headers after that dependency built | delete `build/traffic_simulator`: its CMake cache remembers the failed configure |
| `error: option --editable not recognized` | a pip-installed setuptools shadows Ubuntu's: `pip uninstall -y setuptools` |
| `ros2 run swb ...`: `Package 'swb' not found` | the terminal did not source `~/swb.bash` |

More symptoms: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Getting started on CARLA](carla.md), or [on Isaac Sim](isaac.md).
