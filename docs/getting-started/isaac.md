# Getting started on Isaac Sim

Isaac Sim 6.1 installed and found by swb. About an hour, mostly downloads. Do
[Install](install.md) first. The Isaac tutorials (6 and 7) start from here; the CARLA
tutorials before them are worth doing first, since everything but the simulator works the
same.

On Isaac the ego's vehicle runs inside the Isaac world server: its PhysX body, sensors and
vehicle interface come from [aib](https://github.com/NEWSLabNTU/autoware_isaac_bridge), so
there is no separate vehicle bridge. Autoware connects to it over ROS, on the same host.

## What you need

- **An NVIDIA RTX GPU with 24 GB**, a recent driver (tested: RTX 4090, driver 615), and
  **64 GB of RAM or more**: the world server holds about 17.5 GB with Town01, Autoware
  another 10 to 20 GB. Isaac Sim's own
  [requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
  apply.
- **About 50 GB** of disk for Isaac Sim and its caches, and several GB in swb's cache for
  the USD towns and cars a project fetches. Put both on a fast disk.
- [uv](https://docs.astral.sh/uv/), which installs Isaac's own Python:

  ```bash
  curl -LsSf https://astral.sh/uv/install.sh | sh
  ```

## Install Isaac Sim

Isaac Sim is pip-installed into a Python 3.12 environment of its own (it does not touch the
system's Python), with Autoware's messages built for that Python. Choose where it goes and
run the two installers from the workspace:

```bash
cd ~/scenario-world-bridge
export ISAAC_ROOT=~/isaac-sim
src/swb_isaac_world/scripts/install_isaac.sh     # ends: isaacsim 6.1.0.0, INSTALL-DONE
src/swb_isaac_world/scripts/build_msgs.sh        # into $ISAAC_ROOT/msgs
```

## Tell swb where it is

The Isaac world server reads Isaac's install from `ISAAC_ROOT`. Add it to the environment
file, `~/swb.bash`, after the two `source` lines:

```bash
export ISAAC_ROOT=~/isaac-sim
```

(`isaac: {root: ...}` in `~/.config/swb/host.yaml` is the place meant for it; the world
server does not read that key yet.)

On a machine with two GPUs, keep the world server on one by adding
`export ISAAC_KIT_ARGS="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0"` as
well.

**Check**:

```bash
source ~/swb.bash
ls $ISAAC_ROOT/msgs
```

lists the message packages `build_msgs.sh` built.

## If it does not work

| Symptom | Fix |
|---|---|
| `install_isaac.sh`: `uv: command not found` | install uv first (above) |
| The world server is `not up after 1200 s`, its Kit log ends at `omni.client` and it runs at 100 % CPU | a Kit cache link (`~/.cache/ov`, `~/.local/share/ov`, `~/.nvidia-omniverse`) points at a deleted directory, for example after reinstalling Isaac: create its target (`mkdir -p "$(readlink -m ~/.cache/ov)"`), or run `install_isaac.sh` again, which does |
| Every process prints `'NetworkInterfaceAddress': deprecated element`; the world server logs a DEBUG traceback, `No module named 'em'` | both harmless |

## Next

[Tutorial 6: the same project on Isaac Sim](../tutorials/06-carla-assets-on-isaac.md).
