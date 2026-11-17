# @PROJECT@

A scenario-world-bridge project with three pilots of different kinds in one scenario on
CARLA's Town01: Autoware drives the ego, CARLA's BasicAgent the car ahead, and your own
program (`pilots/follower.py`, an agent on the scensim_agent SDK) the car behind.

| File | Holds |
|---|---|
| `testbench.yaml` | the simulator, the assets, and the pilot of each entity |
| `scensim.lock` | the assets' exact versions, written by `assets fetch`; commit it |
| `scenarios/three_pilots.xosc` | the scenario: three entities with controller `agent` or `Autoware` |
| `pilots/follower.py` | the `command` pilot; fill in its hooks |

```bash
scensim assets fetch
scensim project check
play_launch launch scensim_launch scensim.launch.xml
scensim run scenarios/three_pilots.xosc
```

`assets/`, `state/` and `runs/` are made on each host and are not committed.
