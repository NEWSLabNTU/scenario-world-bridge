# Draft: the install page under 082

*A paper draft of `docs/getting-started/install.md` as [082](../082-distribution-and-pypi-release.md)
designs it, written by 243 on 2026-11-17 before 082 is built, to test the user's path. Nothing
here is implemented. Commands, file names and messages follow 080's and 082's names; where
the design leaves a choice open, the draft picks one and [questions.md](questions.md) names
it.*

---

# Install

This page installs scensim for one user, writes the environment file and the host settings,
and checks the result. The simulator comes next: [CARLA](../../getting-started/carla.md) or [Isaac Sim](../../getting-started/isaac.md).
The words used here (seat, stack, pilot, testbench) are in the
[glossary](../../reference/glossary.md).

scensim brings neither a simulator nor an autopilot. Both are yours to install, and this
page assumes they are.

## What you need

| | Tested with |
|---|---|
| Ubuntu and ROS 2 | Ubuntu 22.04 with ROS 2 Humble from apt ([install](https://docs.ros.org/en/humble/Installation.html)) |
| An autopilot | Autoware 1.5.0, NEWSLab's binary packages (which carry the fixes scensim's scenarios rely on), or a build of your own |
| A simulator | CARLA 0.9.16 ([next page](../../getting-started/carla.md)), or Isaac Sim 6.1 |
| A GPU | an NVIDIA card with room for the simulator and the autopilot: CARLA on Town01 held about 2 to 2.5 GB of video memory, one Autoware about 1 to 2 GB |

Ubuntu 24.04 with ROS 2 Jazzy is the next release's; see [Ubuntu 24.04](#ubuntu-2404).

## Install with pip

scensim is installed for your user, into the Python that ROS runs on:

```bash
python3 -m pip install --user scensim
export PATH=$HOME/.local/bin:$PATH      # where pip --user puts commands (also in ~/.bashrc)
scensim install
```

pip installs a small front end; it chooses the package for your ROS release from the
Python version (3.10 is Humble). `scensim install` then downloads the built scensim for
that release, about 65 MB, checks its checksum and unpacks it under
`~/.local/share/scensim/`. If you omit this step, the first command that needs scensim does
the same and says so, with the size; `scensim --version` never downloads.

**What you see**, ending with two lines to run:

```
scensim 0.2.0 for ROS 2 humble (x86_64): downloaded 65.1 MB, sha256 ok
installed in ~/.local/share/scensim/0.2.0-humble (current)
missing system packages: run once, as an administrator:
  sudo rosdep install --from-paths ~/.local/share/scensim/current/share --ignore-src -y
then, in every terminal: source ~/.local/share/scensim/current/setup.bash
```

## Once per machine (needs sudo)

The rosdep line above installs the system packages scensim's processes load (47 apt
packages on a bare Humble). It is the only step that needs root; on a shared host an
administrator runs it once, and other users omit it. The first two lines are needed only
where rosdep was never initialised:

```bash
sudo rosdep init          # skip if it says the sources list already exists
rosdep update
sudo rosdep install --from-paths ~/.local/share/scensim/current/share --ignore-src -y
```

## The environment file

Every terminal sources one file, here `~/scensim.bash`, in this order: ROS, your autopilot,
then scensim. scensim chooses the vehicle side it downloads from the Autoware it finds
sourced, so Autoware comes first.

```bash
# ~/scensim.bash
source /opt/ros/humble/setup.bash
source /opt/autoware/1.5.0/setup.bash               # your autopilot, when it is Autoware
source ~/.local/share/scensim/current/setup.bash
```

A workspace of your own (Tutorial 1) is built over these and sourced after them.

## The host settings file

What differs between machines lives in `~/.config/scensim/host.yaml`. Every key is
optional; these are the ones most often set:

```yaml
# ~/.config/scensim/host.yaml
carla: {root: ~/CARLA_0.9.16}    # the stack starts CARLA when nothing serves its port
logs: ~/.local/state/scensim/logs   # every run's logs: put it on a fast disk
autoware: {setup: /opt/autoware/1.5.0/setup.bash}
```

Every key: [Host settings](../../reference/host-settings.md).

## Recommended: play_launch

scensim starts its processes with `ros2 launch`, which every ROS 2 has.
[play_launch](https://pypi.org/project/play-launch/) 0.30.0 or newer is recommended: a log
per node, a web interface, and a crashed Autoware component restarted alone.

```bash
python3 -m pip install --user 'play_launch>=0.30.0'
```

## Check

```bash
source ~/scensim.bash
scensim --version
scensim seat show
```

`scensim --version` names the version, the ROS release and the install's directory; `seat
show` lists the host's seats and ends with the launcher it will use. A terminal that has
not sourced the environment file is refused, naming the `source` line.

## Ubuntu 24.04

Ubuntu 24.04 refuses `pip install` outside a virtual environment (PEP 668). Either form
works; the first keeps the commands of this page unchanged:

```bash
python3 -m pip install --user --break-system-packages scensim
```

or a virtual environment that sees the system's packages, so that ROS's Python remains
visible (colcon finds the testbench build type through scensim's `setup.bash`, whichever
form installed the front end):

```bash
python3 -m venv --system-site-packages ~/.venvs/scensim
source ~/.venvs/scensim/bin/activate
pip install scensim
```

The build for ROS 2 Jazzy is not released yet. Until it is, pip installs the front end
alone, and its first command says `no prebuilt scensim for ROS jazzy yet; build from
source`, with a link to [From source](#from-source).

## Updating, and several versions

`python3 -m pip install --user --upgrade scensim`, then `scensim install`, puts the new
version beside the old and points `current` at it. Rebuild your workspaces afterwards
(`colcon build` in each): `scensim run` refuses a testbench built over another version,
naming the rebuild.

## From source

For working on scensim itself, or on a platform without a release. Install Rust
([rustup](https://rustup.rs/)) and colcon's Rust plugin, and the two tools the steps below
use (`pip` for rosdep's Python keys, `vcs` for the message packages):

```bash
sudo apt install python3-pip python3-vcstool
python3 -m pip install --user colcon-cargo-ros2
git clone https://github.com/NEWSLabNTU/scensim.git ~/scensim
cd ~/scensim
```

scensim has two sides. The simulation side (the scenario runner, the bridge, the world
servers) needs only ROS 2; the vehicle side (the CARLA and Isaac Sim vehicle bridges and the
Autoware launches) needs Autoware. Fetch the submodules of the side, or sides, the host runs.

On a host with Autoware, both sides:

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash && source /opt/autoware/1.5.0/setup.bash
rosdep install --from-paths src --ignore-src -y
```

On a host without Autoware, the simulation side alone. The message packages Autoware would
provide come from `sim.repos`, at the versions Autoware 1.5.0 uses:

```bash
git submodule update --init --recursive src/scenario_simulator_v2 src/color_names \
    src/scensim_carla_world src/scensim_isaac_world src/scensim_world_protocol \
    src/sim_vehicle_spec src/sim_vehicle_ecu
vcs import src < sim.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
```

Do not import `sim.repos` on a host with Autoware: the workspace would then hold a second
copy of messages Autoware's programs already load. Then, on either host:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release
```

In the environment file, `~/scensim/install/setup.bash` then takes the place of
`~/.local/share/scensim/current/setup.bash`. A source build is an underlay exactly like the
installed one; the [Developer Guide](../../development.md) has the rest.

## If it does not work

| Symptom | Fix |
|---|---|
| `scensim: command not found` | `~/.local/bin` is not on `PATH`: add `export PATH=$HOME/.local/bin:$PATH` to `~/.bashrc` |
| `scensim is installed for humble, but ROS_DISTRO is jazzy (python /usr/bin/python3.12)` | the terminal sourced another ROS, or pip ran in another Python than ROS's (a conda or pyenv one): source the ROS the install is for, and install with `/usr/bin/python3 -m pip` |
| `sha256 mismatch` during `scensim install` | the download was cut: run it again; a partial file is not kept |
| `ERROR: your rosdep installation has not been initialized yet` | `sudo rosdep init`, then `rosdep update` |
| `pip: pip is not installed` from rosdep | `sudo apt install python3-pip`, then the rosdep line again |
| `Cannot locate rosdep definition for [tier4_rtc_msgs]` (or another `tier4_`, `autoware_` message) | a host without Autoware: `vcs import src < sim.repos`; with Autoware: source its setup.bash before rosdep |
| `error: externally-managed-environment` | Ubuntu 24.04: use one of the forms in [Ubuntu 24.04](#ubuntu-2404) |
| `no prebuilt scensim for ROS jazzy yet` | no release for your ROS yet: [From source](#from-source) |

## Next

[Getting started on CARLA](../../getting-started/carla.md), then [Tutorial 1](../../tutorials/01-first-drive.md).
