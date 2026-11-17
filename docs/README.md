# Documentation

New words (testbench, stack, pilot, seat, run, judge, ...): the [glossary](reference/glossary.md).
What scensim does not do: [Supported and not supported](reference/limits.md).

## By reader

Every reader begins with [Install](getting-started/install.md). After installation, each
continues from the point that matches their role:

- **A new user** continues with [Getting started on CARLA](getting-started/carla.md) (or
  [on Isaac Sim](getting-started/isaac.md)), then the [tutorials](#tutorials) in order.
- **A team adopting scensim**, with its own car, autopilot and maps, continues with
  [Start a testbench of your own](how-to/start-a-project.md), then
  [your car](how-to/your-car.md), [your autopilot](how-to/your-autopilot.md),
  [your Autoware](how-to/your-autoware.md) and the [reference](#reference).
- **A scenario author** continues with [Tutorial 2](tutorials/02-write-a-scenario.md) and
  [Tutorial 3](tutorials/03-other-road-users.md), then
  [What a scenario may use](reference/scenarios.md).
- **An autopilot integrator** continues with
  [Your autopilot as a pilot](how-to/your-autopilot.md), then
  [the agent SDK](reference/agent-sdk.md) and [Tutorial 4](tutorials/04-several-pilots.md).
- **An instructor** continues with [Teaching with scensim](teaching.md), then
  [several stacks on one host](how-to/several-stacks.md); the tutorials serve as laboratory
  sessions.
- **A developer of scensim itself** continues with the [Developer Guide](development.md) and the
  design documents in [design/](design/).

The sections below list every page, in the order a new user reads them.

## Getting started

- [Install](getting-started/install.md): the workspace, the environment file, the host
  settings file
- [On CARLA](getting-started/carla.md): CARLA 0.9.16, running and found
- [On Isaac Sim](getting-started/isaac.md): Isaac Sim 6.1, installed and found

## Tutorials

Learning by doing: one path each, every step with a check, exercises at the end. CARLA
first, Isaac Sim from tutorial 6.

1. [First drive](tutorials/01-first-drive.md): a testbench, its map, the stack, one run, its
   result
2. [Write a scenario](tutorials/02-write-a-scenario.md): poses, the ego's goal, a pass
   condition, a parameter
3. [Other road users](tutorials/03-other-road-users.md): pedestrians, obstacles, a traffic
   light, a car the simulator drives
4. [Several pilots](tutorials/04-several-pilots.md): Autoware, CARLA's BasicAgent and your
   own agent in one run
5. [Suites](tutorials/05-suites.md): a parameter grid, repetitions, a judge, the results
   table
6. [The same testbench on Isaac Sim](tutorials/06-carla-assets-on-isaac.md): CARLA's assets
   in USD, models in a scenario
7. [Road physics](tutorials/07-road-physics.md): mud, gravel and a wet road, watching the
   wheels

## How-to guides

A task in your own testbench, starting from a working stack:

- [Start a testbench of your own](how-to/start-a-project.md): templates, your maps, sharing
- [Your car](how-to/your-car.md): a shipped car, or one from your vehicle spec
- [Your autopilot as a pilot](how-to/your-autopilot.md): a launch file, a program, your
  own process
- [Your Autoware](how-to/your-autoware.md): arguments, your build, fixes, TensorRT
- [Your simulator](how-to/your-simulator.md): a world server for another simulator
- [Your world server in a testbench](how-to/your-world-server.md): run a testbench on a
  world server from a package of your workspace
- [Several stacks on one host](how-to/several-stacks.md): seats
- [Configure the stack](how-to/configure.md): the bridge, the world servers, the weather,
  timeouts
- [Run suites in CI](how-to/run-in-ci.md): exit codes, JUnit, two hosts, the disk
- [Record a demo](how-to/record-a-demo.md): windows, a screencast, rosbags
- [Drive a course on Isaac Sim](how-to/drive-a-course-on-isaac.md): the Mount Haruna road
  from another dataset

## Reference

- [Commands](reference/commands.md): every `ros2 run scensim` command, the stack's arguments,
  exit codes, the other tools
- [The testbench](reference/testbench.md): the package, its build type, its rules
- [The testbench file](reference/project-file.md): `testbench.yaml`, suites, `scensim.lock`
- [Host settings](reference/host-settings.md): `host.yaml`, `local.yaml`, the seat table
- [Results and logs](reference/results-and-logs.md): a run's directory, `result.json`, the
  timeline, capture levels
- [Maps and assets](reference/maps-and-assets.md): a map directory, the datasets
- [The agent SDK](reference/agent-sdk.md): `scensim_agent.Agent`, `ros_adapter`, checking an
  agent
- [Road physics on Isaac Sim](reference/road-physics.md): surface classes, zones, the rough
  road, `/scensim/wheels`
- [What a scenario may use](reference/scenarios.md): the ego, road users the simulator or a
  pilot drives, traffic lights, collisions
- [Supported and not supported](reference/limits.md): every limit, on one page
- [Glossary](reference/glossary.md): the words these pages use
- [Troubleshooting](reference/troubleshooting.md): symptoms, causes, fixes

## Explanation

- [How it works](explanation/how-it-works.md): the parts of a run; testbench, stack, run
- [ROS domains and DDS](explanation/domains-and-dds.md): why several domains, which
  process lives where
- [Verdicts](explanation/verdicts.md): passed, failed, error; CARLA's contacts; judges

## Datasets

Public, on Hugging Face under `jerry73204`; a testbench fetches them by name
([Maps and assets](reference/maps-and-assets.md#datasets)):
[carla-town-autoware-maps](https://huggingface.co/datasets/jerry73204/carla-town-autoware-maps),
[carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd),
[haruna-tsukuba-courses](https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses),
[isaac-road-physics-stages](https://huggingface.co/datasets/jerry73204/isaac-road-physics-stages),
[newslab-golfcart-models](https://huggingface.co/datasets/jerry73204/newslab-golfcart-models).
