# Documentation

## Using scenario-world-bridge

Read in this order:

- [README](../README.md): what it is, requirements, a quick start on CARLA
- Getting started, from install to Autoware driving your own scenario:
  - [on CARLA](getting-started/carla.md): install, the TUM maps, a first run, your
    scenario, Autoware
  - [on Isaac Sim](getting-started/isaac.md): install, the asset set, the stack, a first
    scenario, your scenario
- [Build your own stack](build-your-own-stack.md): your scenario, simulator, car, autopilot
  and more cars, in one launch
- [ROS domains and DDS](domains-and-dds.md): which process runs in which domain, values
  0-57, Cyclone, looking into each domain, two stacks on one host
- Tutorials, in order:
  1. [Prepare a scenario](tutorials/01-prepare-a-scenario.md): poses on the map, a `.xosc`
     of your own
  2. [Run and judge](tutorials/02-run-and-judge.md): suites, CARLA's contacts in the
     verdict, results
  3. [Other road users and more AVs](tutorials/03-other-road-users.md): pedestrians,
     obstacles, other cars, background AVs
  4. [Configure the system](tutorials/04-configure.md): bridge and world server settings,
     weather, ports, domains
  5. [A demo run and recording](tutorials/05-demo-and-recording.md): a longer drive,
     recording a screencast

## Reference

- [User Guide](user-guide.md): the launches, their arguments and defaults, the environment,
  troubleshooting
- [Writing an agent](agent-interface.md): drive scenarios with your own autopilot, the
  `swb_agent.Agent` class, `ros_adapter` from YAML
- [Writing scenarios](design/scenario-authoring.md): what a scenario may use, and how

## Developing scenario-world-bridge

- [Developer Guide](development.md): build, test, layout, how a scenario runs, conventions,
  and the index of design documents ([design/](design/))
- [User workflow and the vehicle agent](design/user-workflow.md): the design of the two
  sides

## Roadmap

- [Roadmap](roadmap/README.md): one document per phase: problem, design, steps, measured
  results
