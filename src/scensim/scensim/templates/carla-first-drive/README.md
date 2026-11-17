# @PROJECT@

A scenario-world-bridge project: one Autoware ego drives a Tesla Model 3 to a goal on
CARLA's Town01.

| File | Holds |
|---|---|
| `swb.yaml` | the simulator, the assets, and who drives each entity |
| `scensim.lock` | the assets' exact versions, written by `assets fetch`; commit it |
| `scenarios/drive_to_goal.xosc` | the scenario: the map, the ego, its goal, the pass condition |

Fetch the map, check the project, bring up the stack and run the scenario:

```bash
ros2 run scensim assets fetch
ros2 run scensim project check
play_launch launch scensim_launch scensim.launch.xml
ros2 run scensim run scenarios/drive_to_goal.xosc
```

`assets/`, `state/` and `runs/` are made on each host and are not committed.
