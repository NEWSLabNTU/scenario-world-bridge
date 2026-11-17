# Glossary

The words these pages use, in the order you meet them.

| Word | Means |
|---|---|
| **scensim** | scensim, this workspace: the commands `ros2 run scensim ...`, the stack, the bridge |
| **SSv2** | [scenario_simulator_v2](https://github.com/tier4/scenario_simulator_v2), TIER IV's scenario runner. It reads a scenario file, moves the scripted road users, sends the ego its goal and decides whether the run passed. scensim bundles it |
| **scenario file** | an OpenSCENARIO file (`.xosc`), or TIER IV's YAML form of one: the map, the road users, what happens and when, and the pass or fail conditions. The same file runs on CARLA and on Isaac Sim |
| **ego** | the vehicle under test: the scenario entity whose controller is `Autoware` (or that has `isEgo=true`). Autoware drives it by default |
| **catalog** | the vehicle catalog that ships with scensim: the cars a scenario names by `CatalogReference` (such as `tesla_model3`), each with its body and dimensions |
| **entity** | one road user of a scenario: a car, a pedestrian, an obstacle, named in the scenario file |
| **project** | a directory with a `swb.yaml`: the simulator, the assets (maps, worlds, cars) by dataset name, and the pilot of each driven entity. It holds no path of your machine, so it can be a git repository others clone |
| **pilot** | what drives one entity: a vehicle side (the car's sensors and its vehicle interface in the simulator), a driver (Autoware, CARLA's BasicAgent, a launch file or a program of yours) and the agent that registers the entity with the relay. `pilots:` in `swb.yaml` binds entities to pilots |
| **agent** | the small program that connects a pilot's driver to the scenario: it takes the scenario's commands (go to this goal, keep under this speed, stop) and reports the vehicle's state. scensim's own vehicle agent serves Autoware; you can write one with the agent SDK |
| **relay** | the agent relay: the process between SSv2 and the agents. SSv2 talks to it as it would to Autoware; each agent registers with it over TCP for its entity |
| **world server** | the one process that drives the simulator (CARLA's or Isaac Sim's). The bridge sends it every spawn, move and step |
| **bridge** | the process SSv2 talks to as its simulator. It turns SSv2's requests into requests to the world server |
| **stack** | the long-lived part, started once with `play_launch launch ... scensim_launch scensim.launch.xml`: the world server, the bridge, the relay and the pilots. Runs go against it; it keeps running between them |
| **run** | one scenario against the stack, started with `ros2 run scensim run <file>`. It starts SSv2 on the scenario, waits for the verdict and leaves a run directory with the result and every process's log |
| **verdict** | the result of a run: `passed`, `failed` (the scenario's conditions say so) or `error` (the stack could not judge it). See [Verdicts](../explanation/verdicts.md) |
| **judge** | a program of yours that scensim runs after each run on its run directory. It prints a verdict and any scores of its own beside SSv2's, never in place of it |
| **suite** | a file that lists scenarios, parameter values and repetitions, run as one batch with one results table |
| **seat** | one stack's share of a host: its ROS domains, its ports, its web UI port, its GPU (and, when the host's table says so, its CARLA port), all derived from one number. Two stacks on one host take two seats so they never meet. See [Several stacks on one host](../how-to/several-stacks.md) |
| **ROS domain** | a ROS 2 `ROS_DOMAIN_ID`: processes on different domains do not see each other. A seat has a scenario domain (bridge, relay, SSv2) and one domain per pilot |
| **host settings** | what differs between machines (where CARLA is, the log root, the cache, the seat table), in `~/.config/scensim/host.yaml` and `/etc/scensim/host.yaml`, never in a project. See [Host settings](host-settings.md) |
| **dataset** | a public Hugging Face dataset of maps, USD worlds or cars, fetched by name into a project with `ros2 run scensim assets fetch` |
| **lock** | `scensim.lock`: the exact dataset commits and file checksums a project fetched, so every machine gets the same files |
| **vehicle spec** | the one file that describes a car; its physics, sensors, catalog entry and Autoware packages are made from it |
| **vehicle interface** | the topics an autopilot drives a car through: commands in on `/control/command/*`, the car's state out on `/vehicle/status/*`. These are Autoware's messages ([Your autopilot](../how-to/your-autopilot.md#the-vehicle-interface)) |
