# simple_drive

**Goal**: drive the ego to its goals on CARLA's Town01 with `simple_pilot`, a small
autopilot that needs no Autoware.

| File | Holds |
|---|---|
| `testbench.yaml` | CARLA, the Town01 map, and the pilot of the entity `ego`: kind `simple` on a Tesla Model 3 |
| `scenarios/drive_to_goal.xosc` | the ego drives 70 m west to a goal |
| `scenarios/turn_to_goal.xosc` | the ego drives 100 m west, turns left at the corner and stops 35 m down the next road |
| `suites/all.yaml` | the two, one run each, in one table |

`simple_pilot` routes along the map's lanes from where the car is to the scenario's goal,
steers by pure pursuit, holds the lanes' speed limits (and slows for curves), and stops at
the goal. It reads the car's true pose from the simulator and drives through the same
vehicle interface Autoware does.

**It ignores traffic**: it does not stop for red lights, other vehicles or pedestrians.
Use it for scenarios that test the road, the map or your scenario itself; for a car that
obeys the road, use the `autoware` pilot (the `first_drive` example).

## Hardware

One NVIDIA GPU that runs CARLA 0.9.16 (about 2 GB of video memory on Town01 at Epic
quality), and about 0.1 GB of disk for the map. No Autoware install and no GPU for it.

## Commands

Copy the example into a workspace as a testbench, build it, fetch its assets, check it:

```bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src
scensim testbench new simple_drive --from pkg://scensim_examples/projects/simple_drive
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim assets fetch simple_drive
scensim testbench check simple_drive
```

CARLA 0.9.16 must be serving on the host settings' `carla.host` and `carla.port`
(default `localhost:2000`), or `carla.root` must name its install so that the stack
starts it. Then:

```bash
scensim run simple_drive/scenarios/drive_to_goal.xosc
scensim run simple_drive                      # the default suite, both scenarios
```

The first run brings up the stack, ready in seconds once CARLA is: the pilot is ready when
its agent has registered with the relay.

## Expected result

Each run prints one line, `passed  <run id>  <run directory>`, and exits 0.

## On Isaac Sim

The same pilot drives on Isaac: set `simulator: isaac` and add the USD assets, as the
`isaac_first_drive` example does. There the car's pose comes from the Isaac world server's
ground truth, which the stack turns on for a `simple` pilot.

What each host makes for the testbench (the fetched assets' links, `state/`, the runs)
is kept outside the package, under the log root; nothing of it is committed.
