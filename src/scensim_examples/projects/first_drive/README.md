# first_drive

**Goal**: drive one Autoware ego through three scenarios on CARLA's Town01 and read their
verdicts.

| File | Holds |
|---|---|
| `testbench.yaml` | CARLA, the Town01 map, and the pilot of the entity `ego` (Autoware on a Tesla Model 3, with scensim's Autoware profile: stock Autoware 1.5.0 fails `pedestrian_crossing`) |
| `scenarios/drive_to_goal.xosc` | the ego drives 70 m west to a goal: the smoke test |
| `scenarios/red_light.xosc` | the scenario turns a light red ahead of the ego; it must stop at the line and go on green |
| `scenarios/pedestrian_crossing.xosc` | the ego passes street barriers and stops for a pedestrian crossing its lane, then drives on; within 1 m of the pedestrian fails |
| `suites/all.yaml` | the three, one run each, in one table |

## Hardware

One NVIDIA GPU that runs CARLA 0.9.16 beside Autoware 1.5.0 (CARLA holds about 2 GB of
video memory on Town01 at Epic quality), and about 0.1 GB of disk for the map.

## Commands

Copy the example into a workspace as a testbench, build it, fetch its assets, check it:

```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
scensim testbench new first_drive --from pkg://scensim_examples/projects/first_drive
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch first_drive
scensim testbench check first_drive
```

`assets fetch` downloads Town01's Autoware map (lanelet2 and point cloud) from the
`carla-town-autoware-maps` dataset and writes `scensim.lock` into `src/first_drive/`; commit
the lock. `testbench check` ends `testbench: ok`.

CARLA 0.9.16 must be serving on the host settings' `carla.host` and `carla.port`
(default `localhost:2000`), or `carla.root` must name its install so that the stack
starts it. Run a scenario, then the default suite (all three), and list the runs:

```bash
scensim run first_drive/scenarios/drive_to_goal.xosc
scensim run first_drive
scensim runs list first_drive
```

The first run brings up the testbench's stack and leaves it up for the next ones; it is
ready when Autoware is (a few minutes on a cold host). `scensim stack first_drive --stop`
ends it.

## Expected result

Each run prints one line, `passed  <run id>  <run directory>`, and exits 0. On the
development host (32 cores, one RTX GPU, CARLA at Epic quality, the stack already up) the
three take about 45 s, 190 s and 130 s, most of it SSv2's start and the ego's engage.
A failed run prints `failed` and its first cause; `scensim runs why <run id>` names the
first warning or error and the process it came from.

What each host makes for the testbench (the fetched assets' links, `state/`, the runs)
is kept outside the package, under the log root; nothing of it is committed.
