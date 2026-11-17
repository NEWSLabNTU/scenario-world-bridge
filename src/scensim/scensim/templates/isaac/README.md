# @PROJECT@

A scensim project on Isaac Sim: one Autoware ego drives a Tesla Model 3 to a
goal in CARLA's Town01, loaded from CARLA's assets in USD. Isaac drives one ego per
scenario.

| File | Holds |
|---|---|
| `testbench.yaml` | the simulator, the assets, and who drives each entity |
| `scensim.lock` | the assets' exact versions, written by `assets fetch`; commit it |
| `scenarios/drive_to_goal.xosc` | the scenario: the map, the ego, its goal, the pass condition |

The USD town and vehicles are several GB; fetch them to a cache on a fast disk (`cache:` in
`~/.config/scensim/host.yaml`).

```bash
scensim assets fetch
scensim project check
play_launch launch scensim_launch scensim.launch.xml
scensim run scenarios/drive_to_goal.xosc
```

`assets/`, `state/` and `runs/` are made on each host and are not committed.
