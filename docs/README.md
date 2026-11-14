# Documentation

## Using scenario-world-bridge

Read in this order:

- [README](../README.md): what it is, requirements, a quick start on CARLA
- Getting started, from install to Autoware driving your own scenario:
  - [on CARLA](getting-started/carla.md): install, the TUM maps, a first run, your
    scenario, Autoware
  - [on Isaac Sim](getting-started/isaac.md): install, the asset set from Hugging Face,
    the stack, a first scenario, your scenario
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
  6. [CARLA's assets on Isaac Sim](tutorials/06-carla-assets-on-isaac.md): the asset set
     from Hugging Face, a town's Isaac world, CARLA's vehicles, walkers and props in a
     scenario, a light Town01, the Mount Haruna course
  7. [Road physics: surfaces, mud and rocks](tutorials/07-road-physics.md)

## Datasets

Published on Hugging Face (`hf download jerry73204/<name> --repo-type dataset`):

- [carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd):
  CARLA 0.9.16's towns, 41 vehicles, 97 props and 52 walkers as USD (CC BY 4.0)
- [carla-town-autoware-maps](https://huggingface.co/datasets/jerry73204/carla-town-autoware-maps):
  TUM's CARLA towns as Autoware maps, with our repairs (LGPL-3.0)
- [haruna-tsukuba-courses](https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses):
  Mount Haruna and Tsukuba Circuit, Isaac worlds and Autoware maps (ODbL, GSI)
- [isaac-road-physics-stages](https://huggingface.co/datasets/jerry73204/isaac-road-physics-stages):
  road-surface and proving-ground test stages for Isaac Sim (Apache-2.0)
- [newslab-golfcart-models](https://huggingface.co/datasets/jerry73204/newslab-golfcart-models):
  a golf cart's Isaac body, rig, spec and RViz mesh (Apache-2.0)

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
