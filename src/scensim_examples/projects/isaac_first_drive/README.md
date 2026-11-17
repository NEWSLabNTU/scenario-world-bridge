# isaac_first_drive

**Goal**: drive one Autoware ego on Isaac Sim, in CARLA's Town01 loaded from CARLA's own
assets in USD.

| File | Holds |
|---|---|
| `testbench.yaml` | Isaac Sim; Town01's Autoware map and Isaac world file; the town and the cars in USD; the pilot of `ego` |
| `scenarios/drive_to_goal.xosc` | the ego drives 70 m west to a goal |
| `scenarios/follow_to_goal.xosc` | the ego drives 80 m to a goal behind a slower car the scenario moves (4 m/s); touching it fails |

On Isaac the world server is the ego's simulator: a PhysX Tesla Model 3 with its own
sensors and vehicle interface, which Autoware drives as on CARLA. Isaac carries one
externally driven vehicle per scenario; traffic lights, CARLA's autopilot and `agent`
vehicles are CARLA only.

## Hardware

An NVIDIA RTX GPU with 24 GB, and 64 GB of RAM or more: the world server holds about
17.5 GB with Town01, Autoware another 10 to 20 GB. Isaac Sim installed (the host settings'
`isaac.root`). Disk: about 0.1 GB for the map, and several GB for the USD town and cars, in
the cache (`cache:` of `~/.config/scensim/host.yaml`; put it on a fast disk).

## Commands

Copy the example into a workspace as a testbench, build it, fetch its assets, check it:

```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
scensim testbench new isaac_first_drive \
    --from pkg://scensim_examples/projects/isaac_first_drive
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch isaac_first_drive
scensim testbench check isaac_first_drive
```

The first fetch downloads Town01 and the vehicles from the 25.5 GB
`carla-0.9.16-isaac-usd` dataset (only what `take:` names) and writes `scensim.lock`; commit
it. Isaac Sim 6.1 is named by the host settings' `isaac.root`. Then:

```bash
scensim run isaac_first_drive/scenarios/drive_to_goal.xosc
scensim run isaac_first_drive/scenarios/follow_to_goal.xosc
```

The first run brings up the stack: the world server loads Town01 (several minutes on a cold
host), then Autoware starts. The stack stays up for the second run.

## Expected result

`testbench check` ends `testbench: ok`; each run prints `passed  <run id>  <run directory>`.

What each host makes for the testbench (the fetched assets' links, `state/`, the runs)
is kept outside the package, under the log root; nothing of it is committed.
