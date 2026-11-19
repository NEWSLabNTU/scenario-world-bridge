# Release notes

## v0.2.0

scensim 0.2.0 drives a car with your own Autoware: the car's Autoware packages are source
you build against it, and an `autoware` pilot starts it stock by default. It adds Ubuntu
24.04 with ROS 2 Jazzy, Autoware 2.1.0, and the studies as testbench packages. The code of
this release is the commit tagged `v0.2.0`.

### Requirements

Ubuntu 22.04 with ROS 2 Humble, as in 0.1.0, or Ubuntu 24.04 with ROS 2 Jazzy.
[Install](getting-started/install.md) lists both. For an Autoware ego, your Autoware, with
the car's Autoware packages built in your workspace
([Connect your Autoware](how-to/connect-autoware.md)).

### Tested with

| Component | Version |
|---|---|
| ROS 2 | Humble (Ubuntu 22.04, Python 3.10); Jazzy (Ubuntu 24.04, Python 3.12) |
| CARLA | 0.9.16, the packaged release |
| Isaac Sim | 6.1.0, pip install |
| Autoware | 1.5.0 and 2.1.0, binary packages (live, below) |
| scenario_simulator_v2 | bundled (a fork, as a submodule) |

The release workflow built and checked the commit it publishes:
- the simulation side's tarball for Humble and for Jazzy, each with its link check in a
  clean ROS image;
- the wheels for Python 3.10 and 3.12, installed in a clean image of each, with a first run;
- the car's Autoware packages for each of Autoware 1.5.0 and 2.1.0 on Humble, and 2.1.0 on
  Jazzy: NEWSLab's Autoware binaries installed, the packages built from
  `autoware-side.repos` over them, and Autoware's launch followed to its last node, stock and
  with scensim's profile.

Live runs before the release (scensim built from source):
- **Autoware 1.5.0**, Tutorial 1 both ways on one build (2026-11-19): stock passed in
  43.6 s, scensim's profile in 41.4 s. Stock requested 34 brief emergency stops and needed a
  GPU with a few GB free beside CARLA ([how stock drives against the
  profile](how-to/connect-autoware.md#how-stock-drives-against-the-profile)).
- **Autoware 2.1.0**, Tutorial 1 with the profile (2026-11-18): passed; the pilot ran 2.1.0's
  own packages only, and the agent took RTC from the planning modules. CenterPoint was not
  run on 2.1.0.
- **CARLA 1.0**, Tutorial 1 from a source build (2026-11-18, Autoware 1.5.0): passed. This
  release's binaries are for CARLA 0.9.16 only (below).
- **The shipped examples**, both ways on Autoware 1.5.0 (2026-11-19): `safety_suite`,
  `multi_pilot` and the `multi-pilot` template passed stock and with the profile.
  `first_drive`'s `pedestrian_crossing` passed with the profile and failed stock 3 of 3, so
  `first_drive` names the profile (below). The Isaac examples were not run live.

### What is new

- **Your Autoware drives the car.** A release has no vehicle download. The car's Autoware
  packages (its vehicle and sensor kit launches, `individual_params`) are source:
  `autoware-side.repos`, attached to each release, imports them into your Autoware
  workspace, and they build against your Autoware, from binary packages, a source build or
  your own distribution ([Connect your Autoware](how-to/connect-autoware.md)). With an
  Autoware sourced, the pip front end installs and runs the simulation side as without one.
  `scensim install --autoware` is gone, and refused by name.
- **Stock Autoware by default, scensim's profile on request.** An `autoware` pilot starts your
  `autoware.launch.xml` as your Autoware ships it, told only what a simulator needs (the
  map, the car, sim time, no vehicle interface and no sensor drivers). `autoware_profile:
  scensim` starts scensim's Autoware profile around it instead, the settings scensim's own
  scenarios are tested with: traffic signals from the simulator, the `clustering` lidar
  detector, parameters for a simulated car. The profile is an optional package in the same
  `autoware-side.repos`.
- **A speed limit reaches stock Autoware.** Stock Autoware serves
  `/api/autoware/set/velocity_limit` only under its `scenario_simulation` switch, so the
  stock pilot passes `scenario_simulation:=true`: on 1.5.0 it adds the deprecated API, on
  2.1.0 the evaluation adapter, and removes nothing. A launch of your own needs it too, or a
  scenario's speed limit is never applied ([Connect your
  Autoware](how-to/connect-autoware.md)). scensim's profile on Autoware 2.0.0 and 2.1.0
  passes it as well.
- **The car's Autoware packages are checked against your Autoware.** Each declares the
  Autoware releases it supports and those it was tested on. `testbench check` and every run
  refuse a missing package or an Autoware outside the supported releases, naming both, and
  warn about a supported one that is not tested: Autoware 1.5.0, 2.0.0 and 2.1.0 on Humble are
  tested; 1.7.1 and 1.9.0, and 2.1.0 on Jazzy, are supported but not tested yet.
- **Ubuntu 24.04 with ROS 2 Jazzy.** On Python 3.12, `pip install scensim` brings the Jazzy
  release package, `scensim-ros-jazzy`, and `scensim install` downloads the simulation side
  built for Jazzy. The release publishes one simulation tarball per ROS distribution,
  `scensim-sim-0.2.0-humble-x86_64.tar.zst` and `scensim-sim-0.2.0-jazzy-x86_64.tar.zst`,
  under one `SHA256SUMS`. An Autoware ego on Jazzy needs an Autoware built for Jazzy
  (Autoware 2.1.0 has one); the car's Autoware packages build and resolve against it, and
  scensim warns that no live run has tested them there yet.
- **rosdep on Ubuntu 24.04.** The rosdep line that `scensim install` prints starts with
  `PIP_BREAK_SYSTEM_PACKAGES=1 PIP_IGNORE_INSTALLED=1` on Python 3.11 and newer: without
  them, Ubuntu 24.04 refuses rosdep's two pip packages (PEP 668), and huggingface_hub
  cannot replace Debian's older typing_extensions and idna. Keep the line as printed.
- **Autoware 2.1.0.** scensim's profile runs Autoware 2.1.0 (2.0.0 with autoware_universe
  0.53.2) with 2.1.0's own perception parameters, and follows the host's Autoware: a stack
  sets each `autoware_profile: scensim` pilot's release from the host's `autoware.setup`,
  and refuses a pilot that names another release, or a release the profile does not have.
  The fixes scensim's testbed carries for Autoware 1.5.0 are not sourced over another
  release, and scensim says so.
- **The agent on Autoware 2.1.0.** Autoware 2.1.0 no longer has TIER IV's
  tier4_ad_api_adaptor, which served the agent's RTC and emergency endpoints. The agent
  now finds which API the running Autoware serves: the adaptor's RTC (1.5.0), else each
  planning module's own cooperate_commands service, else none. Faults then come from
  `/api/fail_safe/mrm_state` alone.
- **A workspace built against another Autoware is refused.** An autoware pilot whose
  workspaces were built with a different Autoware release than the host's `autoware.setup`
  would run a mix of the two. scensim refuses such a pilot before the run, naming the
  workspace's `setup.bash`, both releases and the fix.
- **The studies as testbench packages.** scensim's own studies are testbenches now, built
  with colcon like yours, and their shared scenarios (the bench and gap scenarios) are a
  package of the release, `scensim_study_scenarios`, next to `coop_pilots`. A testbench names
  another package's scenario as `pkg://<package>/<path>`.
- **The car per simulator (from source).** The vehicle spec describes the real car and gives
  each simulator its own body: CARLA 0.9.16's (also Isaac Sim's) and CARLA 1.0's, with its
  own physics on Chaos and its own Autoware vehicle model, `tesla_model3_carla_1_0`, in the
  same `autoware-side.repos`. CARLA 1.0 runs from a source build; this release's binaries and
  its tests are for CARLA 0.9.16.
  On CARLA 1.0:
  - a pilot that drives through CARLA's Python API gets the server's own client: the stack
    unpacks this Python's wheel from `<carla.root>/PythonAPI/carla/dist` and puts it first on
    the pilots' `PYTHONPATH`. Without a wheel for this Python under `carla.root` (or without
    `carla.root`), a `carla_basic` pilot is refused, naming
    both releases and the wheel to install, as 0.9.16's client fails on a 1.0 server;
  - the AWF use cases (`safety_suite`, `scensim_examples/scenarios/awf`, `isaac_demo`) take
    the ego from the testbench's vehicle catalog, so they run on either body;
  - the `coop_pilots` drive the server's body, with 1.0's physics and measured pedal maps,
    and read its wheel positions.
- **Isaac Sim's lidar intensity.** The Isaac lidar returns an intensity from the spec's
  table (reflectance by the surface's SimReady class, times the cosine of incidence), where
  it returned none.
- **A source build on Jazzy.** The CARLA client library compiles with libclang 14, not
  Ubuntu 24.04's default 18; `scripts/release/build_sim.sh` uses `/usr/lib/llvm-14/lib` when
  it is installed. [Install](getting-started/install.md) lists the three differences from a
  Humble build.

### Known issues

- **Stock Autoware 1.5.0 and a crossing pedestrian.** In `first_drive`'s
  `pedestrian_crossing`, stock Autoware drove into the space of the pedestrian in front of
  it; scensim's profile stops more than 1 m short. `first_drive` and Tutorial 3 use the
  profile.
- **Autoware 1.5.0's planner can abort on a new route**, about once in 64 route changes, in
  either configuration (autoware_universe#12460); the run ends with an error and a retry
  passes.
- **A pilot is counted ready while its composables still start** (issue 065), for example
  while TensorRT builds an engine, so a run can start on a half-started Autoware. The
  TensorRT 10.8.0 runtime of `autoware.tensorrt_lib` avoids the rebuild.

### Fixed since 0.1.0

- With an Autoware sourced, scensim 0.1.0's pip front end refused every command, looking for
  a vehicle download the release did not have. 0.2.0 runs as without one.
- On Ubuntu 24.04, 0.1.0's `pip install scensim` installed `scensim-ros-jazzy` 0.1.0, a
  placeholder whose commands refuse and point to Ubuntu 22.04. 0.2.0 replaces it with the
  Jazzy release.
- **The ego's orientation on a slope (CARLA).** The CARLA vehicle bridge published the ego's
  odometry with its pitch and roll mirrored: a car on a slope was reported pitched and rolled
  the wrong way. The bridge now converts CARLA's rotation as LibCarla itself does.
- **A stack whose node ends during its start** (issue 052). A node that exited before the
  stack was ready, with exit 0 as Isaac Sim's world server does after a Python error, left
  `scensim run` waiting out its whole start limit. Now any exit before ready ends the start at
  once, naming the node, how it exited, its last log lines and its log.
- **The agent relay on Autoware 2.1.0** (issue 053). 2.1.0's planning modules publish
  non-finite RTC distances, which reach the relay as null, and the relay died and restarted
  on them. A null distance is now an unknown one.
- **A suite's `pkg://` scenario** (issue 054). `testbench check` and `colcon build` accepted a
  suite naming another package's scenario, and `scensim run` then refused it.
- **`scensim testbench new --from`** (issues 055 and 061). The copy could fail `testbench
  check` without saying why; it now rewrites paths that leave the source and says what is
  left to do. A project that still had `swb.yaml` is copied too, its renames said.
- **A scenario from a fetched dataset** (issue 056) was reported as a problem of the package,
  so a testbench whose scenarios come with a dataset could not be built; it is now the host's
  finding, checked where the dataset is fetched.
- **An Isaac ego config's rig** (issue 057) may name a package (`pkg://`), and a rig outside
  the testbench is checked instead of passing unchecked.
- **The old-name hint** (issue 058) is said once, not on every command, and Tutorial 1's lock
  line names the lock from the current directory.
- **A crash dump's path with a space** (issue 059) is taken whole.
- **A signal table generated beside its map** (issue 060) is in the datasets' form, so it
  joins a map dataset as it is.
- **The installed `scensim` and the first python on `PATH`** (issue 062). On a host where a
  uv or conda python came before the system one, every command failed with `No module named
  'yaml'`; `scensim` now runs its own console script.

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
