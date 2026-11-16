# Install

The one page for installing and building scenario-world-bridge (swb), writing the
environment file and the host settings file. Every other page starts from here. The
simulator itself comes next: [CARLA](carla.md) or [Isaac Sim](isaac.md). The words used
here (seat, stack, pilot, ...) are in the [glossary](../reference/glossary.md).

## Time and space

Plan on several hours for a machine that has nothing yet, and much less when ROS 2 and
Autoware are already installed. What takes the time:

| Step | Time | Disk |
|---|---|---|
| ROS 2 Humble and Autoware 1.5.0's binary packages | as their own install guides say; mostly downloads | Autoware's `/opt/autoware/1.5.0` measured 4.8 GB, plus its apt dependencies |
| Cloning this workspace with its submodules | a few minutes on a fast connection | negligible beside the build |
| The first `colcon build` | the longest wait of the install: Rust, SSv2 and the bridges compile from source. A fresh Release build took about 7 to 8 minutes on 48 cores (2026-11-16); it depends on the cores | a fresh Release build measured 11 GB in `build/` and 1 GB in `install/`, 14 GB with the clone (2026-11-16) |
| CARLA 0.9.16 ([next page](carla.md)) | the download | one unpacked CARLA 0.9.16 measured 30 GB |
| A project's map | seconds | about 0.1 GB for Town01 |

Memory and GPU, for the first drive (CARLA and one Autoware on one host):

- **GPU**: an NVIDIA card with a driver that runs CARLA's Vulkan renderer and Autoware 1.5.0's
  CUDA and TensorRT. Measured on one host: CARLA on Town01 held about 2.0 to 2.5 GB of
  video memory, one Autoware (the default `clustering` lidar detector) about 1 to 2 GB.
- **RAM**: CARLA held about 2.4 GB on Town01; Autoware is about 90 processes. The
  two-Autoware tutorial asks for 32 GB; less has not been measured.
- **CPU**: CARLA used about 2.5 cores on Town01; an Autoware starting up loads every core
  for a while.

Isaac Sim needs much more (an RTX card with 24 GB, 64 GB of RAM): [Isaac Sim](isaac.md).

## What you need

- **Ubuntu 22.04 and ROS 2 Humble** ([install](https://docs.ros.org/en/humble/Installation.html)),
  the fixed requirement.
- **An Autoware workspace**, built, or Autoware's binary packages: the build compiles against
  Autoware's interface messages whichever autopilot later drives, and the Autoware pilot
  runs from it. Its `setup.bash` is sourced by the build and named in the host settings
  (`autoware.setup`, below). swb's launch files and configuration for the car are tested with
  Autoware 1.5.0
  ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/));
  the commands below use its binary packages under `/opt/autoware/1.5.0` as the example.
- **Network access** to GitHub (the clone), crates.io (Rust crates, during the build),
  PyPI (two tools below) and Hugging Face (the maps, later).

## Once per machine (needs sudo)

On a shared host an administrator does this once; other users omit it.

```bash
sudo apt install python3-pip python3-rosdep libz3-dev \
    ros-humble-rclcpp-components ros-humble-class-loader
sudo rosdep init          # skip if it says the sources list already exists
```

The socket buffers for DDS need sudo too, and the clone: they are set at the end of
[Build](#build).

## Your tools

Rust, from [rustup](https://rustup.rs/). The installer adds Rust to new terminals only;
load it into this one:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source ~/.cargo/env
cargo --version
```

colcon's Rust plugin and [play_launch](https://pypi.org/project/play-launch/) 0.30.0 or
newer, which rosdep cannot install. Both are installed for the user into Humble's Python (the
system `python3`, 3.10), not into a virtual environment: `colcon` and `ros2` run on that
Python and must find them.

```bash
python3 -m pip install --user colcon-cargo-ros2==0.5.4 xmlschema 'play_launch>=0.30.0'
export PATH=$HOME/.local/bin:$PATH    # where pip --user puts play_launch
play_launch --version
```

pip must not also have installed a newer setuptools for the user: one in `~/.local`
shadows Ubuntu's and breaks a later `colcon build --symlink-install`
(`error: option --editable not recognized`):

```bash
python3 -c 'import setuptools; print(setuptools.__version__, setuptools.__file__)'
```

The path should be under `/usr/lib/python3/dist-packages`. If it is under `~/.local`, run
`python3 -m pip uninstall -y setuptools`.

## Build

The commands below clone into `~/scenario-world-bridge`. Any directory works; use yours
wherever this page and the environment file say `~/scenario-world-bridge`.

```bash
git clone --recurse-submodules \
    https://github.com/NEWSLabNTU/scenario-world-bridge.git \
    ~/scenario-world-bridge
cd ~/scenario-world-bridge
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep update
rosdep install --from-paths $(colcon list --base-paths src -p) --ignore-src -y \
    --skip-keys python-yamale-pip                    # apt packages: asks for sudo
python3 -m pip install --user yamale
```

rosdep reads the packages colcon builds (`colcon list`) and installs apt packages, so it
asks for sudo; on a shared host the administrator runs it once from one clone. The key
`python-yamale-pip` (SSv2's test runner) resolves only from a rosdep source most hosts
lack, so it is skipped and installed with pip. Among others, rosdep installs pygame,
lanelet2, shapely and networkx.

Without sudo, `rosdep install` stops at the first apt package it cannot install. The build
does not need those packages; the few that runs need are installed for the user instead:

```bash
python3 -m pip install --user shapely networkx pygame
```

shapely and networkx are needed by CARLA's own driving agents, which the `carla_basic`
pilot runs, in the same Python (`project check` refuses a `carla_basic` pilot without
them); pygame only by `carla_manual_control`'s window. lanelet2's Python bindings are not needed by the build or
the tutorials. `rosdep check --from-paths $(colcon list --base-paths src -p) --ignore-src`
lists what is still missing.

Two settings for a toolchain newer than Ubuntu 22.04's (a CMake 4, or a GCC newer than
11), set before the build; neither affects Ubuntu 22.04's own:

```bash
# a CMake 4 (from pip or Kitware) ahead of Ubuntu's: the vendored embree needs this
export CMAKE_POLICY_VERSION_MINIMUM=3.5
```

and, only when `g++ --version` reports a version newer than 11, add
`-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized` to `--cmake-args` below (boost's headers
warn under newer GCCs, and SSv2 builds with warnings as errors). Then:

```bash
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    --cargo-args --release
```

On a machine with less than 64 GB of memory, cap the parallel compiles, or the build can run
out of memory part way: add `--parallel-workers 4` after `colcon build` and set
`export CARGO_BUILD_JOBS=4` first (each compile of SSv2 or a Rust crate takes 2 to 4 GB).

`-DBUILD_TESTING=OFF` leaves out the packages' own tests, which need test-only packages
(`ros-humble-replay-testing` for SSv2's test runner) that only a full `rosdep install`
brings. colcon prints `Summary: <n> packages finished` at the end. A package that failed is
named in `Summary: ... failed`; its log is under `log/latest_build/<package>/`.

Once per user, give Autoware a writable data directory, `~/autoware_data` (TensorRT writes
its engines there, so they are built once and kept):

```bash
source install/setup.bash
ros2 run autoware_sim_profile setup_autoware_data
```

Once per machine (sudo): enlarge the kernel's socket buffers, so that DDS can deliver point
clouds; at the default 208 kB a point cloud never arrives. From the workspace's directory:

```bash
cd ~/scenario-world-bridge            # your clone, wherever you put it
sudo install -m 644 config/99-cyclonedds-max.conf /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-cyclonedds-max.conf
```

## The environment file

Every terminal sources one file, here `~/swb.bash`. It sources Autoware (which sources ROS)
and the workspace, and nothing else; the second line names the clone's path:

```bash
# ~/swb.bash
source /opt/autoware/1.5.0/setup.bash
source ~/scenario-world-bridge/install/setup.bash    # your clone's install/
```

The ROS domains, the DDS settings and the ports are not exported here: the stack sets them
for every process it starts, from the seat (below). Another terminal that is to see
the stack's topics asks for them with `ros2 run swb seat env`
([ROS domains and DDS](../explanation/domains-and-dds.md)).

## The host settings file

What differs between machines lives in `~/.config/swb/host.yaml`, never in a project and
never in an export. Every key is optional; these are the defaults most often changed:

```yaml
# ~/.config/swb/host.yaml
logs: ~/.local/state/swb/logs    # every run's logs: put it on a fast disk (an SSD)
cache: ~/.cache/swb              # the downloaded datasets, shared by all your projects
carla: {host: localhost, port: 2000}
autoware: {setup: /opt/autoware/1.5.0/setup.bash}   # the default
```

A project can override a key for one checkout in `<project>/.swb/local.yaml`, which git
ignores. Every key, and the seat table an administrator sets in `/etc/swb/host.yaml`:
[Host settings](../reference/host-settings.md).

## Autoware's fixes

This step applies to Autoware 1.5.0, its binary packages in particular; another Autoware
workspace omits it. Autoware 1.5.0 lacks three fixes this workspace carries: a right turn on
which Autoware's blind-spot check stops the car, an oscillation in NDT localization that
triggers an occasional emergency stop, and a node that aborts on a timestamp a simulator's
clock produces. They are
built once, in the checkout, after the build (a few minutes):

```bash
ros2 run swb autoware overlay
```

Without them a stack runs stock Autoware and its log says so; the tutorials still pass, but
the ego may stop where it should not. With CenterPoint as the lidar detector
(`lidar_detection_model: centerpoint`), Autoware 1.5.0's binary packages also need the
TensorRT version they were compiled against, 10.8.0, without which the detector's engines
are rebuilt at every start: `ros2 run swb autoware tensorrt` fetches it (3 GB). The default
detector needs neither. `ros2 run swb autoware where` prints where each is placed.

## Check

A **seat** is one stack's share of the host: a set of ROS domains, ports and a web UI port,
all derived from one number, so that two stacks on one machine (two users, or two projects)
never meet. Your stack takes a free seat when it starts. List them:

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

More on seats: [Several stacks on one host](../how-to/several-stacks.md).

## If it does not work

| Symptom | Fix |
|---|---|
| `cargo: command not found` during the build | the terminal predates rustup: `source ~/.cargo/env` |
| `play_launch: command not found` | `~/.local/bin` is not on `PATH`: add `export PATH=$HOME/.local/bin:$PATH` to `~/.bashrc` |
| `ERROR: your rosdep installation has not been initialized yet` | `sudo rosdep init`, then `rosdep update` |
| `embree_vendor`: `Compatibility with CMake < 3.5 has been removed from CMake` | a CMake 4 ahead of Ubuntu's: `export CMAKE_POLICY_VERSION_MINIMUM=3.5` before `colcon build`, as above |
| `traffic_simulator`: `-Werror=maybe-uninitialized` in boost's `rescale_policy.hpp` | a GCC newer than Ubuntu 22.04's 11: add `-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized` to `--cmake-args` |
| `traffic_simulator` cannot find a dependency's headers after that dependency built | delete `build/traffic_simulator`: its CMake cache remembers the failed configure |
| `error: option --editable not recognized` | a pip-installed setuptools shadows Ubuntu's: `python3 -m pip uninstall -y setuptools` |
| `git pull --rebase` refuses: `Cargo.lock` modified | the build adds `[[patch.unused]]` entries to the lock files; discard them first ([Developer Guide](../development.md#set-up)) |
| `ros2 run swb ...`: `Package 'swb' not found` | the terminal did not source `~/swb.bash`, or its path is not your clone's |

More symptoms: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Getting started on CARLA](carla.md), or [on Isaac Sim](isaac.md).
