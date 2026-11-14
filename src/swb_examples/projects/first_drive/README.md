# first_drive

**Goal**: drive one Autoware ego through three scenarios on CARLA's Town01 and read their
verdicts.

| File | Holds |
|---|---|
| `swb.yaml` | CARLA, the Town01 map, and the pilot of the entity `ego` (Autoware on a Tesla Model 3) |
| `scenarios/drive_to_goal.xosc` | the ego drives 70 m west to a goal: the smoke test |
| `scenarios/red_light.xosc` | the scenario turns a light red ahead of the ego; it must stop at the line and go on green |
| `scenarios/pedestrian_crossing.xosc` | the ego passes street barriers and stops for a pedestrian crossing its lane, then drives on; within 1 m of the pedestrian fails |
| `suites/all.yaml` | the three, one run each, in one table |

## Hardware

One NVIDIA GPU that runs CARLA 0.9.16 beside Autoware 1.5.0 (CARLA holds about 2 GB of
video memory on Town01 at Epic quality), and about 0.1 GB of disk for the map.

## Commands

Copy the example, fetch its map, check it:

```bash
ros2 run swb project new ~/first_drive --from pkg://swb_examples/projects/first_drive
cd ~/first_drive
ros2 run swb assets fetch
ros2 run swb project check
```

`assets fetch` downloads Town01's Autoware map (lanelet2 and point cloud) from the
`carla-town-autoware-maps` dataset and writes `swb.lock`; commit the lock. `project check`
prints `project: ok`.

CARLA 0.9.16 must be serving on the host settings' `carla.host` and `carla.port` (default
`localhost:2000`). Bring up the stack in one terminal and leave it running:

```bash
play_launch launch swb_launch swb.launch.xml
```

It is ready when Autoware is (a few minutes on a cold host). In a second terminal, in the
project directory, run a scenario, then all three:

```bash
ros2 run swb run scenarios/drive_to_goal.xosc
ros2 run swb run suites/all.yaml
ros2 run swb runs list
```

## Expected result

Each run prints one line, `passed  <run id>  <run directory>`, and exits 0. On the
development host (32 cores, one RTX GPU, CARLA at Epic quality, the stack already up) the
three take about 45 s, 190 s and 130 s, most of it SSv2's start and the ego's engage.
A failed run prints `failed` and its first cause; `ros2 run swb runs why <run id>` names the
first warning or error and the process it came from.

`assets/`, `state/` and `runs/` are made on each host and are not committed.
