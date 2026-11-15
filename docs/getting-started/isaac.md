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
system's Python). Choose where it goes and run the installer from the workspace:

```bash
cd ~/scenario-world-bridge                        # your clone
export ISAAC_ROOT=~/isaac-sim
src/swb_isaac_world/scripts/install_isaac.sh     # ends: isaacsim 6.1.0.0, INSTALL-DONE
```

## Tell swb where it is

Tell swb where Isaac is installed, once per host, in `~/.config/swb/host.yaml`:

```yaml
isaac:
  root: ~/isaac-sim
```

The stack hands it to the Isaac world server (as `ISAAC_ROOT`), so no terminal needs an
export.

Then build Autoware's messages for Isaac's Python, into `<isaac root>/msgs`:

```bash
source ~/swb.bash
ros2 run swb isaac msgs        # builds every package that is missing or stale
```

Run it again after Autoware is updated: `project check` and the stack refuse an Isaac project
while a message package the world server needs is missing or was built from another
Autoware, naming the package.

On a machine with two GPUs, keep the world server on one by adding
`export ISAAC_KIT_ARGS="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=0"` as
well.

**Check**:

```bash
ros2 run swb isaac msgs --check
```

prints nothing and exits 0 when every message package is built.

## If it does not work

| Symptom | Fix |
|---|---|
| `install_isaac.sh`: `uv: command not found` | install uv first (above) |
| `project check`: `Isaac message bindings: <package> is not built` (or `is stale`) | `ros2 run swb isaac msgs` |
| The world server is `not up after 1200 s`, its Kit log ends at `omni.client` and it runs at 100 % CPU | a Kit cache link (`~/.cache/ov`, `~/.local/share/ov`, `~/.nvidia-omniverse`) points at a deleted directory, for example after reinstalling Isaac: create its target (`mkdir -p "$(readlink -m ~/.cache/ov)"`), or run `install_isaac.sh` again, which does |
| Every process prints `'NetworkInterfaceAddress': deprecated element`; the world server logs a DEBUG traceback, `No module named 'em'` | both harmless |

## Next

[Tutorial 6: the same project on Isaac Sim](../tutorials/06-carla-assets-on-isaac.md).
