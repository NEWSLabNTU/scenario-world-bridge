# multi_pilot

**Goal**: one run in which four vehicles are driven by four different pilots, one of them
a program of your own, on CARLA's Town01.

| Entity | Pilot (`swb.yaml`) | Does |
|---|---|---|
| `ego` | `autoware` | comes down the north arm of a T junction, turns right, drives west to its goal |
| `lead` | `carla_basic`: CARLA's BasicAgent | drives west ahead of the ego |
| `bg_av_1` | `autoware`, a second Autoware in its own ROS domain (pattern `bg_av_*`) | drives east in the oncoming lane |
| `cut_in` | `command`: `pilots/cut_in.py`, an agent on the swb_agent SDK | waits at the south arm's stop line, then turns into the ego's road just ahead of it |

| File | Holds |
|---|---|
| `scenarios/four_pilots.xosc` | the four, each with controller `agent` or `Autoware`; success when the ego and `bg_av_1` reach their goals, failure when the ego touches any of the others or after 300 s |
| `scenarios/two_av.xosc` | the ego behind `bg_av_1` in one lane: two Autowares only |
| `suites/cut_in_seeds.yaml` | the cut-in five times, seeds 1000 to 1004 |
| `pilots/cut_in.py` | the `command` pilot |

## Your own pilot

`cut_in.py` subclasses the SDK's `carla_basic` adapter and adds a policy: it holds the
goal the scenario gives it until the ego is within its trigger distance, then goes. swb
starts it per run (`lifetime: run`) as

```
python3 <project>/pilots/cut_in.py --trigger-m 40 \
    --relay <url> --entity cut_in --seed <n> --run-dir <dir>
```

The SDK reads `--relay`, `--entity`, `--seed` and `--run-dir` (`self.entity`, `self.seed`,
`self.run_dir`); the trigger distance is 40 m plus up to 5 m drawn from the seed, and what
it chose goes to `runs/<id>/logs/pilot-cut_in/cut_in.json`. It finds CARLA through the
project's host settings (`carla: {root, host, port}`). Check it without a simulator:

```bash
echo '{"world": "fake"}' > /tmp/fake.json
ros2 run swb_agent swb-agent check pilots/cut_in.py:CutIn --config /tmp/fake.json
```

## Hardware

Two Autoware stacks and CARLA on one host: one NVIDIA GPU with room for both Autowares'
perception beside CARLA, and 32 GB of RAM or more. About 0.1 GB of disk for the map.

## Commands

```bash
ros2 run swb project new ~/multi_pilot --from pkg://swb_examples/projects/multi_pilot
cd ~/multi_pilot
ros2 run swb assets fetch
ros2 run swb project check
```

With CARLA 0.9.16 serving (`carla.host`, `carla.port` of the host settings), start the
stack in one terminal: it starts both Autowares and the BasicAgent, each in its own domain.

```bash
play_launch launch swb_launch swb.launch.xml
```

Then, in a second terminal in the project:

```bash
ros2 run swb run scenarios/four_pilots.xosc
ros2 run swb run suites/cut_in_seeds.yaml
```

## Expected result

`project check` prints `project: ok`. A run prints `passed  <run id>  <run directory>`
when the ego yields to the car cutting in and both Autowares reach their goals; a run where
the ego hits it prints `failed`, and `ros2 run swb runs show <run id>` gives the timeline.
The suite writes `summary.csv` with one row per seed.

Not yet run end to end on a live stack: the junction, the poses and the trigger distance
are taken from Town01's lanelet map, not from a drive.

`assets/`, `state/` and `runs/` are made on each host and are not committed.
