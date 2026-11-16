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
| Cloning this workspace with its submodules | minutes, on a good connection | small beside the build |
| The first `colcon build` | the longest wait of the install: Rust, SSv2 and the bridges compile from source. A fresh Release build took about 7 to 8 minutes on 48 cores (2026-11-16); it depends on the cores | a fresh Release build measured 11 GB in `build/` and 1 GB in `install/`, 14 GB with the clone (2026-11-16). A long-used development checkout grows: one measured 66 GB in `build/`, 16 GB in `target/` and 2 GB in `install/`; budget 100 GB free for that |
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

- **Ubuntu 22.04 and ROS 2 Humble** ([install](https://docs.ros.org/en/humble/Installation.html)).
- **Autoware 1.5.0**, for example the binary packages under `/opt/autoware/1.5.0`
  ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/)).
  The build and every terminal source its `setup.bash`.
- **Network access** to GitHub (the clone), crates.io (Rust crates, during the build),
  PyPI (two tools below) and Hugging Face (the maps, later).

## Once per machine (needs sudo)

On a shared host an administrator does this once; every user can skip it.

```bash
sudo apt install python3-pip python3-rosdep libz3-dev \
    ros-humble-rclcpp-components ros-humble-class-loader
sudo rosdep init          # skip if it says the sources list already exists
```

Let DDS carry point clouds: the default kernel socket buffers (208 kB) are too small, and
a point cloud then never arrives. After the clone below, from the workspace's directory:

```bash
cd ~/scenario-world-bridge            # your clone, wherever you put it
sudo install -m 644 config/99-cyclonedds-max.conf /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-cyclonedds-max.conf
```

The `rosdep install` step of the build also installs apt packages and asks for sudo; on a
shared host the administrator runs it once from one clone.

## Your tools

Rust, from [rustup](https://rustup.rs/). The installer adds Rust to new terminals only;
load it into this one:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source ~/.cargo/env
cargo --version
```

colcon's Rust plugin and [play_launch](https://pypi.org/project/play-launch/) 0.30.0 or
newer, which rosdep cannot install. Install them for your user into Humble's Python (the
system `python3`, 3.10), not into a virtual environment: `colcon` and `ros2` run on that
Python and must find them.

```bash
python3 -m pip install --user colcon-cargo-ros2 'play_launch>=0.30.0'
export PATH=$HOME/.local/bin:$PATH    # where pip --user puts play_launch
play_launch --version
```

CARLA's own driving agents, which the `carla_basic` pilot runs, need two more Python
modules in the same Python; `project check` refuses a `carla_basic` pilot without them:

```bash
sudo apt install python3-shapely python3-networkx
```

(or `python3 -m pip install --user shapely networkx` without sudo).

Check that pip did not install a newer setuptools for your user as well: one in `~/.local`
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

rosdep reads the packages colcon builds (`colcon list`), not the whole `src/`: its crawl
would walk into the components' own `deps/` copies and stop on their templates. One key,
`python-yamale-pip` (SSv2's test runner), resolves only from a rosdep source most hosts do
not have, so it is skipped and installed with pip. Among others, rosdep installs
pygame, lanelet2, shapely and networkx.

Two settings for compilers newer than Ubuntu 22.04's own, set before the build. Neither
harms an Ubuntu 22.04 toolchain:

```bash
# a CMake 4 (from pip or Kitware) ahead of Ubuntu's: the vendored embree needs this
export CMAKE_POLICY_VERSION_MINIMUM=3.5
```

and, only if your `g++ --version` is newer than 11, add
`-DCMAKE_CXX_FLAGS=-Wno-error=maybe-uninitialized` to `--cmake-args` below (boost's headers
warn under newer GCCs, and SSv2 builds with warnings as errors). Then:

```bash
colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --cargo-args --release
```

colcon prints `Summary: <n> packages finished` at the end. A package that failed is named
in `Summary: ... failed`; its log is under `log/latest_build/<package>/`.

Once per user, give Autoware a writable data directory, `~/autoware_data` (TensorRT writes
its engines there, so they are built once and kept):

```bash
source install/setup.bash
ros2 run autoware_sim_profile setup_autoware_data
```

## The environment file

Every terminal sources one file, say `~/swb.bash`. It sources Autoware (which sources ROS)
and the workspace, and nothing else. Write the path of your clone:

```bash
# ~/swb.bash
source /opt/autoware/1.5.0/setup.bash
source ~/scenario-world-bridge/install/setup.bash    # your clone's install/
```

The ROS domains, the DDS settings and the ports are not exported here: the stack sets them
for every process it starts, from your seat (below). A terminal of your own that should see
the stack's topics asks for them with `ros2 run swb seat env`
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
| `ros2 run swb ...`: `Package 'swb' not found` | the terminal did not source `~/swb.bash`, or its path is not your clone's |

More symptoms: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Getting started on CARLA](carla.md), or [on Isaac Sim](isaac.md).
