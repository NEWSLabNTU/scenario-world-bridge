# simple_drive

**Goal**: drive the ego to its goals on CARLA's Town01 with `simple_pilot`, a small
autopilot that needs no Autoware.

| File | Holds |
|---|---|
| `swb.yaml` | CARLA, the Town01 map, and the pilot of the entity `ego`: kind `simple` on a Tesla Model 3 |
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

Copy the example, fetch its map, check it:

```bash
ros2 run swb project new ~/simple_drive --from pkg://swb_examples/projects/simple_drive
cd ~/simple_drive
ros2 run swb assets fetch
ros2 run swb project check
```

CARLA 0.9.16 must be serving on the host settings' `carla.host` and `carla.port` (default
`localhost:2000`). Bring up the stack in one terminal and leave it running:

```bash
play_launch launch swb_launch swb.launch.xml
```

It is ready in seconds once CARLA is: the pilot is ready when its agent has registered
with the relay. In a second terminal, in the project directory:

```bash
ros2 run swb run scenarios/drive_to_goal.xosc
ros2 run swb run suites/all.yaml
```

## Expected result

Each run prints one line, `passed  <run id>  <run directory>`, and exits 0.

## On Isaac Sim

The same pilot drives on Isaac: set `simulator: isaac` and add the USD assets, as the
`isaac_first_drive` example does. There the car's pose comes from the Isaac world server's
ground truth, which the stack turns on for a `simple` pilot.

`assets/`, `state/` and `runs/` are made on each host and are not committed.
