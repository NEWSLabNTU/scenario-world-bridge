# Developer Guide

For working on scensim itself: building, testing, running against a live
stack, and finding your way around the code and the design documents. Users start at the
[README](../README.md) and the [tutorials](tutorials/).

## Set up

On top of the user prerequisites (ROS 2 Humble, Autoware, CARLA 0.9.16, maps):

```bash
git clone --recurse-submodules https://github.com/NEWSLabNTU/scensim.git
cd scensim
just setup      # rustup, colcon-cargo-ros2, submodules, rosdep, and the dev tools
just build      # the user's colcon build plus --symlink-install and a dev-release profile
```

An unreleased colcon-cargo-ros2 (a commit of its repository rather than PyPI's wheel)
does not install from git with Ubuntu 22.04's pip 22: the plugin builds with maturin, and
pip's isolated build environment does not put `maturin` on `PATH`
(`FileNotFoundError: ... 'maturin'`). Install maturin first and build without isolation:

```bash
python3 -m pip install --user maturin
python3 -m pip install --user --no-build-isolation \
    "git+https://github.com/jerry73204/colcon-cargo-ros2@<commit>#subdirectory=packages/colcon-cargo-ros2"
```

[`just`](https://github.com/casey/just) wraps the developer workflow; `just` alone lists the
recipes. The ones you will use most:

| Recipe | What it does |
|---|---|
| `just build` | colcon build of everything under `src/` (both workspaces' packages, see below) |
| `just check` | `cargo +nightly fmt --check` and clippy with `-D warnings` |
| `just test` | Rust tests (cargo-nextest), the Python tests, the pin check (`scripts/check_pins.py`) and the time check (`scripts/test_sim_time.py`) |
| `just ci` | build + check + test |
| `just format` | format the Rust code |
| `just carla-start` / `carla-status` / `carla-stop` / `carla-health` | CARLA as a background service, and whether it is fit to run a scenario |
| `just run` | the whole stack (`scensim_launch carla.launch.xml`, `sim:=isaac`: `isaac.launch.xml`): world server + bridge + agent relay + the ego (Autoware and its agent, nested, respawned; `ego_launch:=none` for none), play_launch, web UI on 8084. On CARLA every car of the ego's model, the agents' too, runs the spec's physics (`vehicle_physics:=spec`, the default; a path for another file; `none` for the blueprints' own) |
| `just world-check` | the conformance check against the running world server (`sim:=isaac` for Isaac's) |
| `just ego-av` | the vehicle side for the ego, web UI on 8082 (`just ego-av rviz:=true` for RViz; any acb_launch argument as `name:=value`). It serves one scenario: `just scenario-direct` refuses it, and a `just bg-av` stack, once they have run one (roadmap 077, G64); restart it for the next |
| `just build-autoware-overlay` | Autoware fixes 1.5.0 lacks, built from a pinned fork commit into `third_party/autoware_overlay/`; `ego-av` / `bg-av` source it by default (`autoware_overlay:=false`: off; roadmap 042) |
| `just setup-tensorrt` | the TensorRT 10.8.0 runtime Autoware 1.5.0 was compiled against, from NVIDIA's pip wheel into `AUTOWARE_TENSORRT_LIB` (default `third_party/tensorrt/lib`); `ego-av` / `ego-isaac` / `bg-av` put it first on `LD_LIBRARY_PATH`, so CenterPoint's engines survive a restart |
| `just scenario <file>` | one scenario on the project workflow (`ros2 run scensim run <file> --start-stack`) in the project holding it (the nearest `swb.yaml` above it, links not followed), else `scenarios/projects/dev`; the stack stays up. `project:=`, `global_timeout:=`; result under the host's logs (`ros2 run scensim runs show <id>`) |
| `just scenario-direct <file>` | the stack-based path: one scenario (web UI on 8081) against `just run`'s stack, which it starts when it is not up; verdict in `<output_directory>/scenario_test_runner/` (`output_directory:=`, default `/tmp`; `sim:=`, `worlds:=`) |
| `just two-av` | two Autoware stacks, one scenario, on the project workflow (`scenarios/projects/two_av`, its suite `two_av`); `just two-av-direct` is the stack-based path, where `record:=true` records a screencast |
| `just demo-tour` | the demo tour (one Town01 drive through six incidents) on the project workflow (`scenarios/projects/demo`, its suite `tour`, every incident judged); `just demo-tour-direct` is the stack-based path, with the windows, the captions and `record:=true` |
| `just prune-logs [days:=7]` | move runs older than that out of the log root into the archive on `/mnt/disk1` (never deletes) |
| `just e2e [<file>]` | one scenario on the project workflow, as `just scenario` finds its project, starting the stack for it and stopping it after (`ros2 run scensim run <file> --stop-stack`); `just e2e-direct` is the stack-based path (`sim:=`, a map, `keep_stacks:=`, `output_directory:=`) |
| `just acceptance <file> <runs>` | repeat a scenario and summarise |

The `just` recipes use [play_launch](https://pypi.org/project/play-launch/) (per-node logs
under `play_log/<kind>/<time>/`, a web UI per stack); everything they start is also startable
with plain `ros2 launch`, as the README shows.

Project settings (map, rviz, vehicle and launch choices, scenario output) are recipe
arguments, `name:=value` as for a launch file; host settings (ports, displays, install and
data paths such as `CARLA_MAPS` and `AUTOWARE_TENSORRT_LIB`, the log root `SCENSIM_LOG_ROOT`) come from the environment, made
permanent in `.env` or `.envrc`. The recipes refuse to run while one of the environment
variables that used to carry a project setting is set, naming the argument that replaced it
(`scripts/just_lib.bash`).

These recipes are the developers' path, beside a project's (`scensim.launch.xml`, `ros2 run scensim
run`). Three things live only here (roadmap 076 step 10): `ego_launch` and `ego_launch_args`
of `just run` (`carla.launch.xml`, `isaac.launch.xml`, `stack.launch.py`), which a project
replaces with pilots in `swb.yaml` and `scensim.launch.xml` refuses by name; `just bg-av`
(`background_av.launch.xml`), which a project replaces with a pilot `bg_av_*: {kind:
autoware}`; and `CARLA_MAPS`, the developers' map directory. Every recipe exports `SCENSIM_MAPS`
as `CARLA_MAPS` too, so scensim_examples' scenarios, which name their maps `$(env
SCENSIM_MAPS)/<Town>` as a project's do, run under `just scenario` unchanged; the repository's
own `scenarios/` still name `$(env CARLA_MAPS)`. `CARLA_USD` stays the host setting of the
USD tooling (`just isaac-assets`), and the Isaac world server reads it when `SCENSIM_CARLA_USD`,
which a project's stack sets, is not set.

Notes on a checkout:

- A long-used development checkout grows: one measured 66 GB in `build/`, 16 GB in
  `target/` and 2 GB in `install/`; allow 100 GB of free space for such a checkout.
- rosdep is given the packages colcon builds (`colcon list`), not the whole `src/`: its
  crawl would walk into the components' own `deps/` copies and stop on their templates.
- The build adds `[[patch.unused]]` entries (the message crates it generates) to five lock
  files: the repository root's `Cargo.lock` and those of `src/autoware_carla_bridge`,
  `src/scensim_carla_world`, `src/scensim_world_protocol` and `third_party/cuda_ndt_matcher`.
  These entries belong to the build, not to the source: before a `git pull --rebase`,
  discard them with

  ```bash
  git checkout -- Cargo.lock
  for d in src/autoware_carla_bridge src/scensim_carla_world src/scensim_world_protocol \
           third_party/cuda_ndt_matcher; do
      git -C "$d" checkout -- Cargo.lock
  done
  ```

### Rules that cost hours when broken

- **Always `just build`**, never a bare `colcon build`: it adds `--symlink-install` and
  checks for the setuptools that breaks editable Python installs.
- **Stop every stack before building.** A running bridge or `acb_bridge` makes the install
  step fail with `Text file busy` -- after compiling everything.
- **Rebuild acb's workspace, then this one, from a clean environment.** Both hold an
  `acb_bridge` (`just build` here compiles the acb packages too), and which one a stack
  executes depends on the shell acb was built in: built with this repo's `install/` sourced,
  acb's setup chains it as an underlay and acb's copy wins. Ask `ros2 pkg prefix acb_bridge`
  rather than assume; after `git pull` moves a submodule, rebuild both.
- **Build the Autoware overlay once** (`just build-autoware-overlay`, again when its pin in
  `scripts/autoware_overlay.bash` moves). Without it `just ego-av` warns and runs stock
  Autoware, whose `blind_spot` never yields on a right turn (autoware_universe#13509).
- **Fetch Autoware's TensorRT once** (`just setup-tensorrt`). Without it the stack runs
  on the host's TensorRT. If that is not 10.8.0, Autoware rejects every engine it cached
  and CenterPoint rebuilds at each start, which takes minutes.
- **Restart the simulation side after a build.** It is long-lived and keeps running the old
  binary.
- **The CARLA version is a Cargo feature** (`carla-0916` in both workspaces' `Cargo.toml`);
  `CARLA_VERSION` in the environment overrides it. A stray `CARLA_VERSION=0.10.0` in your
  shell builds the wrong API and reports correct code as a missing field.
- **Never destroy a vehicle with sensors still attached** (destroy children first): an
  orphaned IMU segfaults CARLA.
- **Kill processes by pid**, never `pkill -f <pattern>` -- the pattern matches your own
  shell -- and reap what a stopped stack leaves (`ps` for `ppid == 1`).

[`CLAUDE.md`](../CLAUDE.md) keeps the long form of these, with the incident behind each.

## Repository layout

```
src/
  scenario_world_bridge/   Rust: the SSv2 backend; knows no simulator
    src/main.rs              entry point, the world server connection, shutdown
    src/zmq_server.rs        ZMQ REP socket, protobuf dispatch
    src/coordinator.rs       the SSv2 handlers -> the SimBackend seam
    src/backend/remote/      RemoteBackend: a world server over the world protocol; the
                             ego's spawn and spec_hash checks (spec_check.rs)
    src/world_services.rs    the ROS services (weather)
    config/                  bridge_config.yaml
  scensim_world_protocol/      submodule: scensim_world.proto and SSv2's protos, scensim_core (world
                           client and server shell, sim_time, config, NPC tracking, gap
                           trace, contacts), scensim_world_server (Python shell), scensim_world_check
  scensim_carla_world/     submodule: the CARLA world server (carla-rust)
  scensim_isaac_world/     submodule: the Isaac world server (Isaac's Python)
  sim_vehicle_spec/        submodule: the car's spec and what is generated from it
  autoware_sim_vehicle/    submodule: the car for Autoware, the profile, vehicle_check
  autoware_carla_bridge/   submodule: acb, the CARLA vehicle bridge
  autoware_isaac_bridge/   submodule: aib, the Isaac vehicle library
  scenario_simulator_v2/   submodule: our SSv2 fork
  scensim_launch/          launch files (simulation, scenario, ego, background AV); run_suite,
                           physical_verdict
  scensim_examples/        starter scenarios: basic/, multi_av/, awf/ (installed)
  scensim_interfaces/      ROS services (weather)
  scensim_agent/       the agent SDK, `scensim-agent supervise`, the vehicle agent
  scenario_agent_relay/    the scenario's side of the vehicle agent protocol
  scensim_study_scenarios/ the studies' shared bench/ and gap/ scenarios (installed)
  coop_pilots/             scripts/coop installed as data, for the coop studies
scenarios/                 the studies as testbenches (projects/), T3's manifest and references
scripts/                   bring-up (carla_up.sh, isaac_up.sh), checks, measurement and media tools
docs/                      user docs, design docs, roadmap
```

NEWSLab's testbenches on Taiwanese sites (the NTU campus twin, the roads around NTU) are not
in this repository: they are in the private repository NEWSLabNTU/2026-taiwan-testbenches,
testbench packages over the installed scensim.

Each submodule is a repository that builds and tests on its own against its `deps/`
submodules (roadmap 051); `just test` here checks that their `deps/` pins match this
workspace's.

## How a scenario runs

SSv2's interpreter steps the scenario and, every frame, sends the simulator requests over
ZMQ + Protobuf (`simulation_interface`): `Initialize`, spawns, `UpdateFrame`,
`UpdateEntityStatus`, traffic light updates, despawns. `zmq_server.rs` decodes each and the
`Coordinator` turns it into world protocol requests to the world server, which makes the
simulator calls -- NPCs are moved to SSv2's poses, the ego is driven by the simulator's
physics under Autoware's control, and the simulator advances one fixed step per frame. The ego's goals reach Autoware through the agent relay, not ROS, so the two
sides need no shared domain. Details: [architecture](design/architecture.md),
[protocol](design/ssv2-protocol.md), [time and ticking](design/time-and-ticking.md).

## Testing

- **Unit tests** (`just test`): the coordinator's policies, coordinate conversion, the
  blueprint chooser, the contact report, `run_suite` and `physical_verdict`. Pure logic is
  kept apart from the CARLA calls so it can be tested without a server.
- **Live runs**: `just scenario <file>` (the project workflow, its stack started for it),
  or `just run` (with its ego), then `just scenario-direct <file>` or `ros2 run scensim_launch
  run_suite <dir> --physical-verdict`. The starter scenarios are the regression set; `src/scensim_study_scenarios/gap/` holds reproducers for fixed SSv2-vs-CARLA disagreements.
- Before trusting a surprising measurement, check the running binaries against the source
  (`ls -l install/.../acb_bridge`, `/proc/<pid>/cmdline`) and the host's load (`uptime`):
  stale builds and other users' jobs have both looked like regressions here.

## Design documents

| Document | About |
|---|---|
| [Architecture](design/architecture.md) | design principles, the responsibility split, the main flows |
| [Simulator interface](design/simulator-interface-split.md) | the world protocol, one world server per simulator |
| [The vehicle side](design/vehicle-side.md) | one spec per car, the I5 contract, Autoware with or without SSv2 |
| [SSv2 protocol](design/ssv2-protocol.md) | the ZMQ messages the bridge implements |
| [SSv2 launch configuration](design/ssv2-launch-configuration.md) | how SSv2 is launched against CARLA |
| [Time and ticking](design/time-and-ticking.md) | who ticks CARLA, `/clock`, episodes and vehicle stacks |
| [Failure handling and the frame budget](design/failure-and-frame-budget.md) | what fails a frame, what retries |
| [User workflow and the vehicle agent](design/user-workflow.md) | the two sides, the agent protocol |
| [Writing scenarios](design/scenario-authoring.md) | what scenarios may use, and how |
| [SSv2 feature completeness](design/ssv2-feature-completeness.md) | which SSv2 features work on CARLA |
| [Multi-instance architecture](design/multi-instance-architecture.md) | several stacks on one host |
| [CARLA server tuning](design/carla-server-tuning.md) | quality, GPU, startup |
| [Fork policy](design/fork-policy.md) | what lives in our SSv2 fork and why |
| [Gap campaign](design/gap-campaign.md) | finding SSv2-vs-CARLA verdict disagreements |
| [Two Autoware installs on one host](design/duplicate-autoware-installs.md) | keeping a source build and the binary packages apart |
| [Survey: Isaac Sim and digital twins](design/survey-isaac-sim-and-digital-twin.md) | other simulators, real-site twins (roadmap 019) |

The [roadmap](roadmap/README.md) has one document per phase: the problem, the design, the
steps and the measured results.

## Conventions

- **Docs before code**: a phase's roadmap document states the design before it is built,
  and records the results after.
- **Submodules**: push a submodule commit to its remote *before* committing the new pin
  here; a pin to an unpushed commit breaks every clone. SSv2 and Autoware patches live on
  our forks (NEWSLabNTU), not upstream.
- **Commits** are small and topical (`feat(022): ...`, `docs(023): ...`), each building and
  passing `just check`.
