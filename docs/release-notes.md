# Release notes

## v0.2.0 (draft, not released)

<!-- Draft. Before the release: the heading becomes "## v0.2.0" (release.yml's preflight
refuses a version without that exact heading, so this draft cannot ship by accident), the
open items below are settled or removed, and the "Tested with" row for Autoware 2.1.0 names
what was run. -->

scensim 0.2.0 adds Ubuntu 24.04 with ROS 2 Jazzy for the simulation side, and Autoware
2.1.0 as an autopilot next to 1.5.0. The code of this release was tested at scensim commit
a25e3aa.

### Requirements

Ubuntu 22.04 with ROS 2 Humble, as in 0.1.0, or Ubuntu 24.04 with ROS 2 Jazzy for the
simulation side only. [Install](getting-started/install.md) lists both.

### Tested with

| Component | Version |
|---|---|
| ROS 2 | Humble (Ubuntu 22.04, Python 3.10); Jazzy (Ubuntu 24.04, Python 3.12), the simulation side only |
| CARLA | 0.9.16, the packaged release |
| Isaac Sim | 6.1.0, pip install |
| Autoware | 1.5.0, binary packages; 2.1.0 (to be filled in: what was run on it) |
| scenario_simulator_v2 | bundled (a fork, as a submodule) |

Before the release, the release workflow built and checked both distributions on the
tested commit (run 44296930995): the simulation side's tarball for each, its link check
in a clean ROS image, the wheels for Python 3.10 and 3.12, and the install from those
wheels in a clean image, on Humble and on Jazzy.

### What is new

- **Ubuntu 24.04 with ROS 2 Jazzy, the simulation side.** On Python 3.12,
  `pip install scensim` brings the Jazzy release package, `scensim-ros-jazzy`, and
  `scensim install` downloads the simulation side built for Jazzy: the scenario runner,
  the bridge, the world servers and the agent SDK. The release now publishes one
  simulation tarball per ROS distribution, `scensim-sim-0.2.0-humble-x86_64.tar.zst` and
  `scensim-sim-0.2.0-jazzy-x86_64.tar.zst`, under one `SHA256SUMS`.
- **The vehicle side stays on Humble.** Autoware 1.5.0 is built for ROS 2 Humble only, so on
  Jazzy, with an Autoware sourced or with `scensim install --autoware 1.5.0`, scensim
  refuses before it downloads anything and says why. Use pilots that are not Autoware there,
  or run the Autoware ego from a Humble install.
- **rosdep on Ubuntu 24.04.** The rosdep line that `scensim install` prints starts with
  `PIP_BREAK_SYSTEM_PACKAGES=1 PIP_IGNORE_INSTALLED=1` on Python 3.11 and newer: without
  them, Ubuntu 24.04 refuses rosdep's two pip packages (PEP 668), and huggingface_hub
  cannot replace Debian's older typing_extensions and idna. Keep the line as printed.
- **Autoware 2.1.0.** The car's Autoware profile runs Autoware 2.1.0 (2.0.0 with
  autoware_universe 0.53.2), with a copy of map_based_prediction's parameter file that
  adds the three parameters 0.53.2 requires and autoware_launch 0.53.0 lacks.
- **An autoware pilot follows the host's Autoware.** A stack sets each autoware pilot's
  release from the host's `autoware.setup` (the version-named directory above
  `setup.bash`). A pilot whose arguments name another release is refused, naming both,
  as is a release the profile does not have. The fixes scensim carries for Autoware
  (its overlay) are for 1.5.0: over another release they are not sourced, and scensim says
  so.
- **The agent on Autoware 2.1.0.** Autoware 2.1.0 no longer has TIER IV's
  tier4_ad_api_adaptor, which served the agent's RTC and emergency endpoints. The agent
  now finds which API the running Autoware serves: the adaptor's RTC (1.5.0), else each
  planning module's own cooperate_commands service, else none. Faults then come from
  `/api/fail_safe/mrm_state` alone. The agent logs the path it took once Autoware's core
  services answer, and again only when it changes.
- **A source build on Jazzy.** The CARLA client library compiles with libclang 14, not
  Ubuntu 24.04's default 18; `scripts/release/build_sim.sh` uses
  `/usr/lib/llvm-14/lib` when it is installed. [Install](getting-started/install.md) lists
  the three differences from a Humble build.

### Fixed since 0.1.0

- On Ubuntu 24.04, scensim 0.1.0's `pip install scensim` installed `scensim-ros-jazzy`
  0.1.0, a placeholder whose commands refuse and point to Ubuntu 22.04. 0.2.0 replaces it
  with the Jazzy release.

<!-- Open items for the user and 133 before the release:
1. The acb rotation fix (acb b0428a3: a car on a slope was published pitched and rolled
   the wrong way in the ego odometry) is on acb main but scensim main still pins acb
   f93c64a. It belongs here only once the pin moves; its note would go under "Fixed since
   0.1.0". The same commit adds acb's CARLA 1.0 build (roadmap 083), which is not a 0.2.0
   feature unless decided otherwise.
2. Issues 046 to 050 and the PEP 440 version check in the front end (546b258) are in the
   v0.1.0 tag (d90ac38), so they are not listed as changes in 0.2.0.
3. Autoware 2.1.0's vehicle side has no release tarball (release.yml builds the vehicle side
   for Autoware 1.5.0 only): from a release install, 2.1.0 is a source build. Say so here,
   or add a vehicle tarball for 2.1.0 to the release.
4. Whether to keep "The code of this release was tested at scensim commit a25e3aa" or move
   it to the release commit.
-->

## v0.1.0

The first release of scenario-world-bridge (swb). It evaluates an autopilot on
OpenSCENARIO scenarios in CARLA or Isaac Sim, with the autopilot driving on the simulator's
own sensors and scenario_simulator_v2 (SSv2) judging the run. The code of this release
was tested at swb commit 36e4da4; the commits after it change only the documentation and
the maintainers' test tooling.

### Requirements

Ubuntu 22.04 and ROS 2 Humble, Autoware's interface messages, Rust and colcon's Rust
plugin: [Install](getting-started/install.md) lists them. Only ROS 2's own `ros2 launch` is
required to start a stack. [play_launch](https://pypi.org/project/play-launch/) 0.30.0 or
newer is recommended: it adds a log directory per process, a web interface listing every
process, and restarts a crashed Autoware component alone, where `ros2 launch` ends its
container.

### Tested with

| Component | Version |
|---|---|
| CARLA | 0.9.16, the packaged release |
| Isaac Sim | 6.1.0, pip install |
| Autoware | 1.5.0, binary packages |
| scenario_simulator_v2 | bundled (a fork, as a submodule) |
| GPU | NVIDIA; Isaac Sim tested on an RTX 4090 (24 GB) with 64 GB of RAM |

Before the release, 29 suites covering both simulators, the pilot kinds and the
examples' studies were run on the tested commit and their results kept as references.
During development, on a fresh clone without play_launch (swb c94b12c, an earlier commit),
tutorials 1 to 5 were followed as written: 31 runs, all passed. On 2026-11-17, with the
tested code and Autoware's published 1.5.0 packages, Tutorial 1's drive and Tutorial 5's
suite passed again as written under `ros2 launch`: 7 runs, all passed at the first attempt.

### What it does

- **Projects.** A project is a directory with a `swb.yaml` naming its simulator, maps,
  cars, pilots and suites. `project new` writes one from a template (`carla-first-drive`,
  `multi-pilot`, `isaac`) or copies an example; `project check` validates it, naming a fix
  for each problem.
- **Assets from Hugging Face.** `assets fetch` downloads what a project takes from the
  public datasets, verifies each file against its manifest and records the exact dataset
  commits in `swb.lock`, so a project fetches the same files on every host.
- **Pilots.** Any entity of a scenario can be driven: by Autoware (the car's launch files
  and configuration are provided), CARLA's BasicAgent, a simple pilot on the ground truth, a
  launch file of one's own, or a Python program written with the agent SDK. Several pilots
  can drive in one scenario.
- **A long-lived stack.** `ros2 launch swb_launch swb.launch.xml` brings up the simulator
  side and the pilots once; runs then go against it. Seats let several users or checkouts
  run stacks on one host without sharing ports or ROS domains.
- **Runs and suites.** `swb run` runs a scenario, a directory or a suite (a grid of
  parameters with repetitions, shards for several hosts, resume, retry on error). Each run
  leaves a directory with `result.json` (the verdict, every judge's verdict and the
  provenance: commits, launcher, simulator), a timeline and the logs; `--capture` keeps a
  rosbag of the key topics. `runs list`, `show`, `why`, `summary` (CSV and JUnit) and
  `prune` read and manage them. The exit code tells a pass, a failure, an infrastructure
  error and a refusal apart, for CI.
- **Verdicts.** SSv2's verdict, CARLA's own contact check, and judges of one's own, side by
  side in each result.
- **The same scenarios on Isaac Sim**, with CARLA's towns, cars and walkers as USD.
- **Road physics on Isaac Sim.** Ground given by the project per map: a surface class per
  semantic label and per zone, a soil model with sinkage and ruts that persist between runs,
  a rough-road corridor, and the road's state set from a scenario with `RoadCondition`.
  Each wheel's surface, slip and sinkage is published on `/swb/wheels`.
- **Documentation.** Seven tutorials from a first drive to road physics, how-to guides
  (your car, your autopilot, your Autoware, CI, several stacks), and a reference for every
  command, file and setting.

### Not in this release

- Video recording tools: a screencast, a chase view and captions for demo videos, an RViz
  configuration for recording, and recording on Isaac Sim. A run's rosbag (`--capture`) is
  available.
- Ground chosen per scenario rather than per map, and a judge computed from a run's
  rosbag.
- A gate run before a batch, merging one suite's results from two hosts in one command,
  per-pilot ROS parameter overrides, and running a project's assets and car without a
  stack (for tools that drive the simulator directly).
- CARLA 0.10.
- A car of one's own on Isaac Sim (on CARLA it is supported), and the Mount Haruna and
  Tsukuba courses under Autoware on Isaac Sim.
- The speeds up to which a camera-only Autoware drives on Isaac Sim have not been
  measured.

### Known limits

- **Isaac Sim: a slow first spawn.** A run's first ego on Isaac can take longer to spawn
  (about 70 s with a camera and a lidar) than scenario_simulator_v2 waits for it, and the
  run then ends `error`. A suite's retry on error passes on the second spawn, which is
  fast. The first spawn of a walker on a freshly started Isaac world server can also take
  more than two minutes.
- **Isaac Sim: the frame rate must divide the physics rate.** On Isaac, scenario_simulator_v2's
  frame rate (`ssv2: {frame_rate: ...}` in `swb.yaml`, default 20) must divide the physics
  rate (`isaac: {physics_hz: ...}`, default 60) exactly: with the default physics, 60, 30,
  20, 15, 12 or 10 Hz. At another frame rate, for example 50 or 100 Hz, the ego receives
  its commands and stays in drive, but does not move. This was seen with one car; the
  others are expected to behave the same. A frame rate of 50 or 100 Hz needs
  `isaac: {physics_hz: ...}` set to a multiple of it, for example 100.
- **Isaac Sim: back to the map a stack started on.** An Isaac stack whose pilots have moved
  to a preloaded map (`isaac.preload`) refuses a later scenario on the map it started on,
  although that world is still loaded: `swb run` reports "the stack's Isaac world on
  Town02". Run suites on different Isaac maps each on a stack of its own: stop the stack
  between them, or run each suite with `ros2 run swb run <suite> --stop-stack`, which
  starts a stack for it and stops it afterwards.
- **Isaac Sim: contact with the cyclist in the i6 demo.** On Isaac, the i6 scenario's
  judges may report contact with the cyclist while scenario_simulator_v2 passes the run:
  the ego, approaching at low speed before it yields, comes within about 0.2 m of the
  cyclist's bounding box, and Isaac's cyclist body touches it. On CARLA the same scenario
  passes every judge, with about 0.56 m of clearance.
- **Isaac Sim on a VNC display.** Isaac Sim's window renderer crashed at start on the
  TurboVNC displays tested, and the world server opens a window whenever `DISPLAY` is set.
  On a VNC display, set `isaac: {render: headless}` in the project's `swb.yaml`.
- **play_launch and a refused world server.** Under play_launch, a world server refused for
  a usage error that the launch file cannot check in advance is restarted every 10 s until
  the stack is stopped, since play_launch does not run the launch file's exit handlers
  (play_launch issue #0090); `ros2 launch` ends the launch instead. Two related play_launch
  issues are open: #0091 (`--parser python` ignores a shutdown handler) and #0092 (the Rust
  parser does not apply `SetEnvironmentVariable` to the launch file's environment).
- **A crash loop outside `swb run`.** `swb run` ends a stack's start when a node exits five
  times before the stack is ready, naming the node. A stack started directly, with
  `ros2 launch swb_launch swb.launch.xml` or play_launch, has no such limit: a node that
  fails at every start is restarted until the stack is stopped.

Everything else swb does not do is on one page:
[Supported and not supported](reference/limits.md).

### For a checkout from before this release

Rebuild both workspaces from a clean shell after updating. The developer recipes that ran a
scenario now run it in a project, and each has a variant keeping the earlier behaviour: the
[Developer Guide](development.md) lists both.
