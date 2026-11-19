# Install

This page installs scensim for one user, writes the environment file and the host
settings file, and checks the result. Every other page starts from here. The simulator
comes next: [CARLA](carla.md) or [Isaac Sim](isaac.md). The words used here (seat, stack,
pilot, testbench) are defined in the [glossary](../reference/glossary.md).

scensim installs neither a simulator nor an autopilot. Both are installed separately, and
this page assumes that they are, or will be.

There are two ways to install scensim. The released build is installed with pip and needs
no compiler; it is the way described first. It installs the simulation side: the scenario
runner, the bridge, the world servers and the vehicle bridges. To drive a car with Autoware,
you build the car's Autoware packages in your own Autoware workspace, against your Autoware
([Connect your Autoware](../how-to/connect-autoware.md)). A build from source
([From source](#from-source)) is for work on scensim itself, and for a platform without a
release.

<!-- TODO(exit run): every output below is from the first development build
(0.0.0-dev, 2026-11-17); replace version, sizes and lines with the release's. -->

## What you need

| | Tested with |
|---|---|
| Ubuntu and ROS 2 | Ubuntu 22.04 with ROS 2 Humble from apt ([install](https://docs.ros.org/en/humble/Installation.html)), or Ubuntu 24.04 with ROS 2 Jazzy from apt ([install](https://docs.ros.org/en/jazzy/Installation.html)) for the simulation side only |
| An autopilot | Autoware, for example 1.5.0 from its binary packages under `/opt/autoware/1.5.0`, or a workspace of your own; the releases the car's packages support are in [Connect your Autoware](../how-to/connect-autoware.md#supported-autoware-releases) ([install](https://autowarefoundation.github.io/autoware-documentation/main/installation/)) |
| A simulator | CARLA 0.9.16 ([next page](carla.md)), or Isaac Sim 6.1 ([Isaac Sim](isaac.md)) |
| A GPU | an NVIDIA card with room for both: on one host CARLA on Town01 held about 2.0 to 2.5 GB of video memory, one Autoware about 1 to 2 GB |
| Network | PyPI and GitHub (the install), Hugging Face (the maps, later) |

Memory and processors, for the first drive: CARLA held about 2.4 GB of RAM and 2.5 cores on
Town01; Autoware is about 90 processes and loads every core while it starts. Isaac Sim
needs considerably more (an RTX card with 24 GB, 64 GB of RAM).

On Ubuntu 24.04 with ROS 2 Jazzy, an Autoware ego needs an Autoware built for Jazzy (2.1.0
has one); the car's packages are not tested there yet, and scensim says so. See
[Ubuntu 24.04](#ubuntu-2404).

## Three commands

On Ubuntu with ROS 2 from apt and, for an Autoware ego, your Autoware installed:

```bash
python3 -m pip install --user scensim
~/.local/bin/scensim setup      # one sudo prompt
source ~/scensim.bash           # in every terminal
```

`scensim setup` says what it will do before it does it:

- downloads the release for your ROS (as `scensim install`, below);
- as root, behind one password prompt, installs what is missing among rosdep, vcstool,
  colcon and pip, initialises and updates rosdep, installs the release's system packages, and
  enlarges the kernel's socket buffers. A step already done is skipped; with nothing left,
  there is no prompt. `--no-sudo` prints the script for an administrator instead;
- with an Autoware (the one your terminal sourced, else the only one under `/opt/autoware`,
  else `--autoware <setup.bash>`), imports and builds the car's Autoware packages and
  scensim's profile in `~/autoware_side` ([Connect your
  Autoware](../how-to/connect-autoware.md)), and gives Autoware its data directory;
- writes [`~/scensim.bash`](#the-environment-file) and [`host.yaml`](#the-host-settings-file)
  from what it found. A file of yours that differs is kept, and the difference is printed;
  `--force` replaces it and keeps the old one as `.bak`;
- ends with a check that names each fix. `scensim setup --check` runs the check alone.

Run it again at any time: a second run changes nothing. Then fetch CARLA if you have none
(about 8 GB, unpacked to `~/CARLA_0.9.16`), and go on with [Tutorial 1](../tutorials/01-first-drive.md):

```bash
scensim carla fetch 0.9.16
```

The sections below are the same steps by hand, for a host where you prefer to run them
yourself.

## Install with pip

scensim is installed for your user, into the Python that ROS runs on:

```bash
python3 -m pip install --user scensim
export PATH=$HOME/.local/bin:$PATH    # where pip --user puts commands; also ~/.bashrc
scensim install
```

pip installs a small front end. It chooses the build for your ROS release from the
Python version (3.10 is Humble, 3.12 is Jazzy). `scensim install` downloads that build, checks its size
and checksum, and unpacks it under `~/.local/share/scensim/`. It is the same with or without an Autoware
sourced: the car's Autoware packages are never downloaded, you build them
([Connect your Autoware](../how-to/connect-autoware.md)). scensim 0.1.0 refused every
command in a terminal with an Autoware sourced; 0.2.0 does not. If this step is omitted, the first command that needs scensim performs it and says so; `scensim
--version` never downloads anything.

**What you see**, ending with the lines to run next:

```
downloading scensim 0.0.0-dev for ROS 2 humble, 49.5 MB
  5.2 MB of 49.5 MB
  ...
  sha256 ok; unpacking to <home>/0.0.0-dev-humble/sim
installed in <home>/0.0.0-dev-humble (current)
missing system packages: run once, the second as an administrator:
  rosdep update
  sudo ROS_HOME=<user home>/.ros rosdep install --rosdistro humble \
      --from-paths <home>/current/sim/share --ignore-src -y
then, in every terminal (after ROS, and your Autoware if you use one):
  source <home>/current/sim/setup.bash
an Autoware pilot needs the car's Autoware packages in your Autoware workspace: <guide>
```

scensim prints the full path where this page writes `<home>` for
`~/.local/share/scensim`, and the link to [Connect your Autoware](../how-to/connect-autoware.md) for `<guide>`. The first development build measured 49.5 MB to download and 212 MB unpacked.

## Once per machine (needs sudo)

The rosdep lines printed above install the system packages scensim's processes load. On a
shared host an administrator runs them once, and other users omit them. `rosdep update`
refreshes rosdep's index first: with an old one a key can fail to resolve. The first line
is needed only where rosdep was never initialised:

```bash
sudo rosdep init          # skip if it says the sources list already exists
rosdep update
sudo ROS_HOME=$HOME/.ros rosdep install --rosdistro $ROS_DISTRO \
    --from-paths ~/.local/share/scensim/current/sim/share --ignore-src -y
```

`ROS_HOME` points root's rosdep at the index your `rosdep update` wrote, and `--rosdistro`
names your ROS: sudo gives root its own home, where no index is (`your rosdep installation
has not been initialized yet`), and drops `ROS_DISTRO` (`Cannot locate rosdep definition for
[rclcpp]`).

On Ubuntu 24.04 the printed line starts with `PIP_BREAK_SYSTEM_PACKAGES=1
PIP_IGNORE_INSTALLED=1`; keep it as printed (see [Ubuntu 24.04](#ubuntu-2404)).

On Ubuntu 22.04 and 24.04 this installs a brotli older than 1.2, with which huggingface_hub
cannot download the assets, so `scensim assets fetch` takes them over plain HTTPS instead.
`python3 -m pip install --user 'brotli>=1.2'` lets huggingface_hub download them itself.

The kernel's socket buffers must also be enlarged, so that DDS can deliver point clouds;
at the default 208 kB a point cloud never arrives:

<!-- TODO(exit run): the installed path of 99-cyclonedds-max.conf in the release. -->

```bash
sudo install -m 644 99-cyclonedds-max.conf /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-cyclonedds-max.conf
```

These are the only steps of the install that need root.

## The environment file

Every terminal sources one file, here `~/scensim.bash`, in this order: ROS, your Autoware
(when a pilot is Autoware), scensim, then the workspace with the car's Autoware packages.

```bash
# ~/scensim.bash
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash                 # when the autopilot is Autoware
source ~/.local/share/scensim/current/sim/setup.bash
source ~/autoware_side/install/setup.bash             # the car's Autoware packages
```

The last line is the workspace of [Connect your Autoware](../how-to/connect-autoware.md#import-and-build).

A workspace of your own ([Tutorial 1](../tutorials/01-first-drive.md)) is built over these
and sourced after them. The ROS domains, the DDS settings and the ports are not exported
here: the stack sets them for every process it starts, from its seat.

With scensim's Autoware profile (`autoware_profile: scensim`), once per user, give Autoware
a writable data directory, `~/autoware_data`, where TensorRT keeps the engines it builds, so that they are
built once:

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

## NEWSLab's Autoware 1.5.0 binaries

If you use NEWSLab's Autoware 1.5.0 binary packages with scensim's Autoware profile, see the
[testbed page](../design/autoware-testbed.md) for the fixes and the TensorRT runtime
scensim's own testbed adds. Your own Autoware needs neither.

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

Once `~/scensim.bash` is sourced, the installed build's own `scensim` comes first on
`PATH`; it hands `--version`, `-V` and `install` to the front end, so these answer the same
in every terminal.

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

From scensim 0.2.0, pip installs the build for ROS 2 Jazzy there. An Autoware ego needs an
Autoware built for Jazzy (Autoware 1.5.0 is built for Humble only; 2.1.0 has a Jazzy build).
The car's Autoware packages build and resolve against Autoware 2.1.0 on Jazzy, but no live
run has tested them there yet: `testbench check` and every run say so, and run anyway.
scensim 0.1.0 has no Jazzy build: on Ubuntu 24.04 its commands say so and point to Ubuntu
22.04.

The rosdep line `scensim install` prints on Ubuntu 24.04 is

```bash
sudo ROS_HOME=$HOME/.ros PIP_BREAK_SYSTEM_PACKAGES=1 PIP_IGNORE_INSTALLED=1 \
    rosdep install --rosdistro $ROS_DISTRO \
    --from-paths ~/.local/share/scensim/current/sim/share --ignore-src -y
```

Two of the keys are Python packages that rosdep installs with pip. The first setting lets
pip install them beside Ubuntu's own packages. The second lets them bring newer copies of
packages Ubuntu installed (typing_extensions, idna), which pip cannot remove.

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

On Ubuntu 24.04 with Jazzy, build the simulation side only, and three things differ:

- the CARLA client library compiles with libclang 14, not Ubuntu 24.04's default 18:
  `sudo apt install libclang-14-dev` and `export LIBCLANG_PATH=/usr/lib/llvm-14/lib`;
- pip needs `--break-system-packages`:
  `python3 -m pip install --user --break-system-packages colcon-cargo-ros2`;
- rosdep needs the two settings of [Ubuntu 24.04](#ubuntu-2404):
  `sudo ROS_HOME=$HOME/.ros PIP_BREAK_SYSTEM_PACKAGES=1 PIP_IGNORE_INSTALLED=1 rosdep install ...`, and
  `/opt/ros/jazzy` takes the place of `/opt/ros/humble` below.

scensim has two sides. The **simulation side** (the scenario runner, the bridge, the world
servers) needs only ROS 2; the **vehicle side** (the CARLA and Isaac Sim vehicle bridges
and the Autoware launches) needs Autoware. Fetch the submodules of the side, or sides, the
host runs. Neither form needs `--skip-keys`.

On a host with Autoware, both sides:

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash
rosdep update
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
rosdep update
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

The build has its own `scensim` command, so the pip front end is not needed beside it. In
the environment file, `~/scensim/install/setup.bash` then takes the place of the installed
scensim's `setup.bash` line, after Autoware's ([The environment file](#the-environment-file)). The [Developer Guide](../development.md) has the
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
