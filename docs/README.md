# Documentation

## Using scenario-world-bridge

- [README](../README.md) -- what it is, prerequisites, setup, quick start
- [Getting started on CARLA](getting-started/carla.md) -- start here: install, the TUM maps, a first run, your scenario, Autoware
- [Getting started on Isaac Sim](getting-started/isaac.md) -- the same on Isaac Sim: install, the asset set, the world server, Autoware, your scenario
- [Build your own stack](build-your-own-stack.md) -- your scenario, simulator, car, autopilot and more cars, in one launch
- [ROS domains and DDS](domains-and-dds.md) -- which process lives in which domain, values 0-57, Cyclone, looking into each domain
- Tutorials, in order:
  1. [First run](tutorials/01-first-run.md) -- both sides up, a starter scenario, its verdict
  2. [Prepare a scenario](tutorials/02-prepare-a-scenario.md) -- poses on the map, a `.xosc` of your own
  3. [Run and judge](tutorials/03-run-and-judge.md) -- suites, CARLA's contacts in the verdict, results
  4. [Demo run](tutorials/04-demo-run.md) -- two Autoware vehicles, recording a screencast
  5. [Other road users](tutorials/05-other-road-users.md) -- pedestrians, obstacles, other cars, background AVs
  6. [Configure the system](tutorials/06-configure.md) -- bridge and world server settings, weather, ports, your Autoware
  7. [The whole stack in one launch](tutorials/07-the-stack.md) -- simulator, bridge, autopilot and scenario or free drive; your car, scenarios and simulator
- [User Guide](user-guide.md) -- the reference: install, maps, scenario directories, running,
  the CARLA world, troubleshooting
- [Writing an agent](agent-interface.md) -- drive scenarios with your own autopilot: the
  `swb_agent.Agent` class, `ros_adapter` from YAML, `swb-agent check` / `new`
- [Writing scenarios](design/scenario-authoring.md) -- what a scenario may use on CARLA, and how
- [User workflow and the vehicle agent](design/user-workflow.md) -- the two sides, swapping
  CARLA or Autoware

## Developing scenario-world-bridge

- [Developer Guide](development.md) -- build, test, layout, how a scenario runs, conventions,
  and the index of design documents ([design/](design/))

## Roadmap

- [Roadmap](roadmap/README.md) -- one document per phase: problem, design, steps, measured results
