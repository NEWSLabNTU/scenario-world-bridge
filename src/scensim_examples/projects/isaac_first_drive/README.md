# isaac_first_drive

**Goal**: drive one Autoware ego on Isaac Sim, in CARLA's Town01 loaded from CARLA's own
assets in USD.

| File | Holds |
|---|---|
| `swb.yaml` | Isaac Sim; Town01's Autoware map and Isaac world file; the town and the cars in USD; the pilot of `ego` |
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

```bash
ros2 run scensim project new ~/isaac_first_drive \
    --from pkg://scensim_examples/projects/isaac_first_drive
cd ~/isaac_first_drive
ros2 run scensim assets fetch
ros2 run scensim project check
```

The first fetch downloads Town01 and the vehicles from the 25.5 GB
`carla-0.9.16-isaac-usd` dataset (only what `take:` names) and writes `scensim.lock`; commit
it. Bring up the stack in one terminal:

```bash
play_launch launch scensim_launch scensim.launch.xml
```

The world server loads Town01 (several minutes on a cold host), then Autoware starts. In a
second terminal in the project:

```bash
ros2 run scensim run scenarios/drive_to_goal.xosc
ros2 run scensim run scenarios/follow_to_goal.xosc
```

## Expected result

`project check` prints `project: ok`; each run prints `passed  <run id>  <run directory>`.

`assets/`, `state/` and `runs/` are made on each host and are not committed.
