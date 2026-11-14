# Documentation

## By reader

| You are | Start with | Then |
|---|---|---|
| **New to swb** | [Install](getting-started/install.md), [Getting started on CARLA](getting-started/carla.md) | the [tutorials](#tutorials) in order |
| **Adopting swb** for your project | [Start a project of your own](how-to/start-a-project.md) | [your car](how-to/your-car.md), [your autopilot](how-to/your-autopilot.md), [your Autoware](how-to/your-autoware.md), the [reference](#reference) |
| **Teaching** | [Teaching with swb](teaching.md) | [several stacks on one host](how-to/several-stacks.md), the tutorials as labs |
| **Writing scenarios** | [Tutorial 2](tutorials/02-write-a-scenario.md), [Tutorial 3](tutorials/03-other-road-users.md) | [Writing scenarios](design/scenario-authoring.md): what the bridge supports |
| **Integrating an autopilot** | [Your autopilot as a pilot](how-to/your-autopilot.md) | [the agent SDK](reference/agent-sdk.md), [Tutorial 4](tutorials/04-several-pilots.md) |
| **Developing swb** | the [Developer Guide](development.md) | [design/](design/) |

## Getting started

- [Install](getting-started/install.md): the workspace, the environment file, the host
  settings file
- [On CARLA](getting-started/carla.md): CARLA 0.9.16, running and found
- [On Isaac Sim](getting-started/isaac.md): Isaac Sim 6.1, installed and found

## Tutorials

Learning by doing: one path each, every step with a check, exercises at the end. CARLA
first, Isaac Sim from tutorial 6.

1. [First drive](tutorials/01-first-drive.md): a project, its map, the stack, one run, its
   result
2. [Write a scenario](tutorials/02-write-a-scenario.md): poses, the ego's goal, a pass
   condition, a parameter
3. [Other road users](tutorials/03-other-road-users.md): pedestrians, obstacles, a traffic
   light, a car the simulator drives
4. [Several pilots](tutorials/04-several-pilots.md): Autoware, CARLA's BasicAgent and your
   own agent in one run
5. [Suites](tutorials/05-suites.md): a parameter grid, repetitions, a judge, the results
   table
6. [The same project on Isaac Sim](tutorials/06-carla-assets-on-isaac.md): CARLA's assets
   in USD, models in a scenario, the Mount Haruna course
7. [Road physics](tutorials/07-road-physics.md): mud, gravel and a wet road, watching the
   wheels

## How-to guides

A task in your own project, starting from a working stack:

- [Start a project of your own](how-to/start-a-project.md): templates, your maps, sharing
- [Your car](how-to/your-car.md): a shipped car, or one from your vehicle spec
- [Your autopilot as a pilot](how-to/your-autopilot.md): a launch file, a program, your
  own process
- [Your Autoware](how-to/your-autoware.md): arguments, your build, fixes, TensorRT
- [Your simulator](how-to/your-simulator.md): a world server for another simulator
- [Several stacks on one host](how-to/several-stacks.md): seats
- [Configure the stack](how-to/configure.md): the bridge, the world servers, the weather,
  timeouts
- [Run suites in CI](how-to/run-in-ci.md): exit codes, JUnit, two hosts, the disk
- [Record a demo](how-to/record-a-demo.md): windows, a screencast, rosbags

## Reference

- [Commands](reference/commands.md): every `ros2 run swb` command, the stack's arguments,
  exit codes, the other tools
- [The project file](reference/project-file.md): `swb.yaml`, suites, `swb.lock`
- [Host settings](reference/host-settings.md): `host.yaml`, `local.yaml`, the seat table
- [Results and logs](reference/results-and-logs.md): a run's directory, `result.json`, the
  timeline, capture levels
- [Maps and assets](reference/maps-and-assets.md): a map directory, the datasets
- [The agent SDK](reference/agent-sdk.md): `swb_agent.Agent`, `ros_adapter`, checking an
  agent
- [Road physics on Isaac Sim](reference/road-physics.md): surface classes, zones, the rough
  road, `/swb/wheels`
- [Troubleshooting](reference/troubleshooting.md): symptoms, causes, fixes
- [Writing scenarios](design/scenario-authoring.md): what a scenario may use, and how

## Explanation

- [How it works](explanation/how-it-works.md): the parts of a run; project, stack, run
- [ROS domains and DDS](explanation/domains-and-dds.md): why several domains, which
  process lives where
- [Verdicts](explanation/verdicts.md): passed, failed, error; CARLA's contacts; judges

## Datasets

Public, on Hugging Face under `jerry73204`; a project fetches them by name
([Maps and assets](reference/maps-and-assets.md#datasets)):
[carla-town-autoware-maps](https://huggingface.co/datasets/jerry73204/carla-town-autoware-maps),
[carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd),
[haruna-tsukuba-courses](https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses),
[isaac-road-physics-stages](https://huggingface.co/datasets/jerry73204/isaac-road-physics-stages),
[newslab-golfcart-models](https://huggingface.co/datasets/jerry73204/newslab-golfcart-models).
