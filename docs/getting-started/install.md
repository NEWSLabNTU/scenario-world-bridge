# Install

This page installs scensim for one user, writes the environment file and the host
settings file, and checks the result. Every other page starts from here. The simulator
comes next: [CARLA](carla.md) or [Isaac Sim](isaac.md). The words used here (seat, stack,
pilot, testbench) are defined in the [glossary](../reference/glossary.md).

scensim installs neither a simulator nor an autopilot. Both are installed separately, and
this page assumes that they are, or will be.

There are two ways to install scensim. The released build is installed with pip and needs
no compiler; it is the way described first. A build from source is for work on scensim
itself, or for a platform without a release; it is described in
[From source](#from-source).

<!-- TODO(exit run): every output below is from the first development build
(0.0.0-dev, 2026-11-17); replace version, sizes and lines with the release's. -->

## What you need

| | Tested with |
|---|---|
| Ubuntu and ROS 2 | Ubuntu 22.04 with ROS 2 Humble from apt ([install](https://docs.ros.org/en/humble/Installation.html)) |
| An autopilot | Autoware 1.5.0, from its binary packages under `/opt/autoware/1.5.0` or a workspace of your own ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/)) |
| A simulator | CARLA 0.9.16 ([next page](carla.md)), or Isaac Sim 6.1 ([Isaac Sim](isaac.md)) |
| A GPU | an NVIDIA card with room for both: on one host CARLA on Town01 held about 2.0 to 2.5 GB of video memory, one Autoware about 1 to 2 GB |
| Network | PyPI and GitHub (the install), Hugging Face (the maps, later) |

Memory and processors, for the first drive: CARLA held about 2.4 GB of RAM and 2.5 cores on
Town01; Autoware is about 90 processes and loads every core while it starts. Isaac Sim
needs considerably more (an RTX card with 24 GB, 64 GB of RAM).

Ubuntu 24.04 with ROS 2 Jazzy has no release yet: see [Ubuntu 24.04](#ubuntu-2404).

## Install with pip

scensim is installed for your user, into the Python that ROS runs on:

```bash
python3 -m pip install --user scensim
export PATH=$HOME/.local/bin:$PATH    # where pip --user puts commands; also ~/.bashrc
scensim install
```

pip installs a small front end. It chooses the build for your ROS release from the
Python version (3.10 is Humble). `scensim install` downloads that build, checks its size
and checksum, and unpacks it under `~/.local/share/scensim/`. When an Autoware is
sourced, it also downloads the vehicle side built for that Autoware release. If this step
is omitted, the first command that needs scensim performs it and says so; `scensim
--version` never downloads anything.

**What you see**, ending with the lines to run next:

```
downloading scensim 0.0.0-dev for ROS 2 humble, 49.5 MB
  5.2 MB of 49.5 MB
  ...
  sha256 ok; unpacking to <home>/0.0.0-dev-humble/sim
installed in <home>/0.0.0-dev-humble (current)
missing system packages: run once, as an administrator:
  sudo rosdep install --from-paths <home>/current/sim/share --ignore-src -y
then, in every terminal (after ROS and your Autoware):
  source <home>/current/sim/setup.bash
```

scensim prints the full path where this page writes `<home>` for
`~/.local/share/scensim`. The first development build measured 49.5 MB to download and 212 MB unpacked.

## Once per machine (needs sudo)

The rosdep line printed above installs the system packages scensim's processes load. On a
shared host an administrator runs it once, and other users omit it. The first two lines
are needed only where rosdep was never initialised:

```bash
sudo rosdep init          # skip if it says the sources list already exists
rosdep update
sudo rosdep install --from-paths ~/.local/share/scensim/current/sim/share \
    --ignore-src -y
```

The kernel's socket buffers must also be enlarged, so that DDS can deliver point clouds;
at the default 208 kB a point cloud never arrives:

<!-- TODO(exit run): the installed path of 99-cyclonedds-max.conf in the release. -->

```bash
sudo install -m 644 99-cyclonedds-max.conf /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-cyclonedds-max.conf
```

These are the only steps of the install that need root.

## The environment file

Every terminal sources one file, here `~/scensim.bash`, in this order: ROS, the autopilot,
then scensim. Autoware comes before scensim because scensim's vehicle side is built for
it.

```bash
# ~/scensim.bash
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash          # when the autopilot is Autoware
source ~/.local/share/scensim/current/sim/setup.bash
source ~/.local/share/scensim/current/vehicle-autoware1.5.0/setup.bash   # vehicle side
```

A workspace of your own ([Tutorial 1](../tutorials/01-first-drive.md)) is built over these
and sourced after them. The ROS domains, the DDS settings and the ports are not exported
here: the stack sets them for every process it starts, from its seat.

Once per user, give Autoware a writable data directory, `~/autoware_data`, where TensorRT
keeps the engines it builds, so that they are built once:

```bash
source ~/scensim.bash
ros2 run autoware_sim_profile setup_autoware_data
```

## The host settings file

What differs between machines lives in `~/.config/scensim/host.yaml`, never in a
testbench and never in an exported variable. Every key is optional; these are the ones
most often changed:

```yaml
# ~/.config/scensim/host.yaml
carla: {root: ~/CARLA_0.9.16}     # started when nothing serves CARLA's port
logs: ~/.local/state/scensim/logs # every run's logs: a fast disk (an SSD)
cache: ~/.cache/scensim           # downloaded datasets, shared by all testbenches
autoware: {setup: /opt/autoware/1.5.0/setup.bash}
```

Every key, and the seat table an administrator sets in `/etc/scensim/host.yaml`, is
described in [Host settings](../reference/host-settings.md).

## Autoware's fixes

This step applies to Autoware 1.5.0's binary packages. They lack three fixes that
scensim's scenarios rely on: a right turn on which Autoware's blind-spot check stops the
car, an oscillation in NDT localization that triggers an occasional emergency stop, and a
node that aborts on a timestamp a simulator's clock produces. They are built once, in a
few minutes:

```bash
scensim autoware overlay
```

Without them a stack runs stock Autoware and its log says so. With CenterPoint as the
lidar detector, Autoware 1.5.0 also needs TensorRT 10.8.0, without which the detector's
engines are rebuilt at every start: `scensim autoware tensorrt` fetches it (3 GB). The
default detector needs neither. `scensim autoware where` prints where each is placed.

<!-- TODO(exit run): confirm the three autoware commands from the release. -->

## Recommended: play_launch

scensim starts its processes with a launcher. Without anything more that is `ros2
launch`, which every ROS 2 installation has, and every page here works with it.
[play_launch](https://pypi.org/project/play-launch/) 0.30.0 or newer is recommended: it
keeps a log per node, shows the stack in a web interface, and restarts a crashed Autoware
component alone.

```bash
python3 -m pip install --user 'play_launch>=0.30.0'
```

scensim uses it when it is on `PATH` (the host setting `launcher: auto`, the default).

## Check

```bash
source ~/scensim.bash
scensim --version
scensim seat show
```

`scensim --version` names the front end's version and the installed build:

```
scensim 0.0.0-dev (pip front end, /usr/bin/python3)
installed for ROS humble: <home>/0.0.0-dev-humble
```

A **seat** is one stack's share of the host: a set of ROS domains and ports derived from
one number, so that two stacks on one machine never meet. `seat show` lists the host's
seats and ends with the launcher the stack will use, for example `launcher: auto -> ros2
(play_launch not on PATH)`. More on seats: [Several stacks on one
host](../how-to/several-stacks.md).

## Ubuntu 24.04

Ubuntu 24.04 refuses `pip install` outside a virtual environment
(`error: externally-managed-environment`, PEP 668). Either of two forms works. The first
keeps every command on these pages unchanged:

```bash
python3 -m pip install --user --break-system-packages scensim
```

The second is a virtual environment that sees the system's packages, so that ROS's Python
remains visible:

```bash
python3 -m venv --system-site-packages ~/.venvs/scensim
source ~/.venvs/scensim/bin/activate
pip install scensim
```

The build for ROS 2 Jazzy is not released yet. Until it is, pip installs the front end
alone, and its first command refuses with `no prebuilt scensim for ROS jazzy yet; build
from source`, followed by a link to [From source](#from-source).

## Updating, and several versions

```bash
python3 -m pip install --user --upgrade scensim
scensim install
```

The new version is unpacked beside the old one, and `current` is pointed at it. Rebuild
your workspaces afterwards (`colcon build` in each): `scensim run` refuses a testbench
built over another version of scensim, naming the rebuild.

## From source

A build from source needs Rust, colcon's Rust plugin, and two tools the steps use (`pip`
for rosdep's Python keys, `vcs` for message packages):

```bash
sudo apt install python3-pip python3-vcstool python3-rosdep
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source ~/.cargo/env
python3 -m pip install --user colcon-cargo-ros2
git clone https://github.com/NEWSLabNTU/scensim.git ~/scensim
cd ~/scensim
```

scensim has two sides. The **simulation side** (the scenario runner, the bridge, the world
servers) needs only ROS 2; the **vehicle side** (the CARLA and Isaac Sim vehicle bridges
and the Autoware launches) needs Autoware. Fetch the submodules of the side, or sides, the
host runs. Neither form needs `--skip-keys`.

On a host with Autoware, both sides:

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep install --from-paths src --ignore-src -y
```

On a host without Autoware, the simulation side alone. The message packages Autoware
would otherwise provide come from `sim.repos`, at the versions Autoware 1.5.0 uses:

```bash
git submodule update --init --recursive src/scenario_simulator_v2 src/color_names \
    src/scensim_carla_world src/scensim_isaac_world src/scensim_world_protocol \
    src/sim_vehicle_spec src/sim_vehicle_ecu
vcs import src < sim.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
```

The Isaac Sim world server's vehicle library comes with `src/scensim_isaac_world`, which
the recursive update fetches. Do not import `sim.repos` on a host with Autoware: the
workspace would then hold a second copy of messages that Autoware's programs already load.

Then, on either host:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release
```

colcon prints `Summary: <n> packages finished` at the end; a package that failed is named,
and its log is under `log/latest_build/<package>/`. On a clean Humble container the
simulation side measured about 15 minutes of preparation (the tools, the message
packages and rosdep) and 14 minutes of build on 12 cores (2026-11-17). With less than 64 GB of memory, add `--parallel-workers 4` and set
`export CARGO_BUILD_JOBS=4` first.

In the environment file, `~/scensim/install/setup.bash` then takes the place of the
installed scensim's two `setup.bash` lines. The [Developer Guide](../development.md) has the
rest.

## If it does not work

| Symptom | Fix |
|---|---|
| `scensim: command not found` | `~/.local/bin` is not on `PATH`: add `export PATH=$HOME/.local/bin:$PATH` to `~/.bashrc` |
| `scensim is installed for humble, but ROS_DISTRO is jazzy (python ...)` | the terminal sourced another ROS, or pip ran in another Python than ROS's (a conda or pyenv one): source the ROS the install is for, and install with `/usr/bin/python3 -m pip` |
| `download cut` or `sha256 mismatch` during `scensim install` | the download was interrupted or damaged: run the command again; the partial file is removed |
| `version skew: the scensim front end is ...` | the front end and its release package differ: `python3 -m pip install --user --upgrade scensim scensim-ros-humble` |
| `ERROR: your rosdep installation has not been initialized yet` | `sudo rosdep init`, then `rosdep update` |
| `pip: pip is not installed` from rosdep | `sudo apt install python3-pip`, then the rosdep line again |
| `Cannot locate rosdep definition for [tier4_rtc_msgs]` (or another `tier4_` or `autoware_` message) | from source on a host without Autoware: `vcs import src < sim.repos`; with Autoware: source its `setup.bash` before rosdep |
| `cargo: command not found` during the build | the terminal predates rustup: `source ~/.cargo/env` |
| `error: option --editable not recognized` | a pip-installed setuptools shadows Ubuntu's: `python3 -m pip uninstall -y setuptools` |

More symptoms: [Troubleshooting](../reference/troubleshooting.md).

## Next

[Getting started on CARLA](carla.md), or [on Isaac Sim](isaac.md).
