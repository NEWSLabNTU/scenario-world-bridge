# 077: earlier work on the project workflow, and a test matrix across autopilots and simulators

## Goal

User, 2026-11-15: "For your open items, we should have extensive tests on Autoware and other
autopilots and in different simulators. Make sure the previous work can migrate to our new
workflow without breakage."

Two outcomes:

1. **Migration**: every earlier study, scenario set, demo and tool a user or a study runs
   (roadmaps 017 to 075) runs as a project on the 076 workflow (`swb.yaml`, pilots, suites,
   `ros2 run swb run`), and reaches the result it reached before, or the difference is
   explained. Nothing that worked stops working; the developer recipes keep working until
   their project form is proven, then they call it.
2. **A test matrix**: autopilot kinds x simulators x scenario classes, in tiers, so a change
   to any of them shows what it broke: no-simulator tests on every commit, conformance
   checks, and live runs with recorded reference results.

The open items of 076 ("Open after 076's build") are part of the matrix: the Isaac live run,
own cars (`vehicles:`), a non-Autoware autopilot on the vehicle interface, two SSv2 clients,
the map-switch preload.

## Inputs

- The inventory: [077-study/inventory.md](077-study/inventory.md) (every runnable item, how it
  runs today, what the project workflow lacks for it, its pass criterion and reference result).
- 076's acceptance runs (2026-11-15): first_drive, multi_pilot, the sharded suite, the AWF
  suite, all on CARLA 0.9.16.

## The matrix

Axes:

- **Autopilots**: Autoware 1.5.0 through the profile, and its variants that earlier work
  used (detection clustering or CenterPoint, CPU or CUDA NDT, vision-only localization);
  Autoware 2.0.0 through the same profile (`autoware_release:=2.0.0`, autoware_sim_vehicle
  32a9d78/7ce07d5; T1 `test_pilot_parse` case `autoware_2.0.0`, T2 below);
  SDK agents (`command` pilots: the cut-in agent, the cooperation agents); CARLA's BasicAgent
  (`carla_basic`); a user's own launch (`launch`, with `agent: own`); a non-Autoware autopilot
  on the vehicle interface (a minimal reference autopilot, new); no pilot (`none`, the
  scenario drives the entity).
- **Simulators**: CARLA 0.9.16; Isaac Sim 6.x through swb_isaac_world; CARLA 0.10 where earlier
  work ran on it (the 0.10 profile).
- **Scenario classes**: drive to goal, traffic lights, pedestrians and NPCs, cut-in and
  cooperation, distributions (TIER IV ScenarioModifiers, ParameterValueDistribution),
  multi-AV, town changes, Isaac surfaces and road physics, the NTU twin and course worlds.

Tiers:

| Tier | What | Where | When |
|---|---|---|---|
| T0 | unit and integration tests without a simulator (src/swb, swb_launch, the bridge, swb_agent, the relay) | any host | every commit (`just test`) |
| T1 | conformance without a simulator: `project check` over every project, `swb-agent check` over every SDK agent, `swb_world_check conformance` over each world server, launch parse (`play_launch dump`) of every pilot kind | any host | every commit |
| T2 | live smoke: one short scenario per (simulator, pilot kind) cell | a GPU host per simulator | before a merge that touches the stack, the pilots or a world server |
| T3 | live regression: the migrated studies' suites, each against its reference result | GPU hosts, Isaac hosts, two hosts for sharding | nightly or on demand |

Every live run records its `result.json`; a T3 suite keeps a reference table
(`expect:` per scenario and recorded metrics) and reports a regression as a failed expectation.

## Steps

1. [x] Inventory (077-study/inventory.md); this doc refined from it.
2. [x] T0/T1 in `just test`: project check over every project in the repository, swb-agent
   check over every SDK agent, launch parse of every pilot kind and of swb.launch.xml.
3. [~] The gaps the inventory names, built in the project workflow (each before the items it
   blocks), with tests.
4. [ ] Migration, item by item: a project (in `src/swb_examples/projects/` for user-facing
   ones, `scenarios/projects/` for studies), its suite and expectations, run live, compared
   with its reference result; the developer recipe then calls the project.
5. [~] A reference non-Autoware autopilot on the vehicle interface (a small ROS 2 node driving
   through the documented topics), as a `launch` pilot; T2 on CARLA and Isaac. Built
   2026-11-15 as 070 D's `simple_pilot`, its own pilot kind `simple` rather than a `launch`
   pilot (the driver is its own agent); T0/T1 in place, T2 open.
6. [ ] T2 matrix live, every cell, on CARLA (this host) and Isaac (an Isaac host).
7. [ ] T3 suites live; references recorded.
8. [ ] The developer recipes on the project workflow; what remains of the old paths for
   developers only, documented. Plan (2026-11-16): `just scenario <xosc>` runs `ros2 run swb
   run` on the project holding the scenario (the nearest `swb.yaml` above it), else on
   `scenarios/projects/dev` (Town01 to Town10, the default Autoware ego, physical_verdict),
   `project:=` naming another; the old path stays as `just scenario-direct`, for developers.
   `demo-tour` runs the demo project's tour suite (item 11), `two-av` item 7's project,
   `acceptance` and `e2e` their own, each switched once its item is migrated, and announced
   to the team first (other sessions' tools call `just scenario`).

## The release cut (2026-11-16)

The user asked for the campaign to end in a release (through 133, 2026-11-16). **Release
ready** means: (a) install.md's commands build from a fresh clone on two clean hosts, one for
CARLA and one for Isaac; (b) tutorials 1 to 7 pass for a fresh reader; (c) every ready
inventory item is migrated, with a T3 reference that compares clean on the tag commit; (d) no open gap
gives a wrong verdict (the class of G42, G54, G55) or blocks (a) to (c); (e) check_pins,
check_docs and the tests pass; (f) the release notes list what is deferred, below.

Main is not frozen (the user, through 133, 2026-11-16). The T3 references are recorded once,
on the release candidate (`scripts/t3.py record`). Right before the user tags, the T3 suites
run on the exact commit to be tagged and `scripts/t3.py compare` checks them against the
references; any changed verdict or judge verdict is reported to the user. A change landed
after the references that alters a result is resolved before the tag: either that suite's
reference is recorded again on the tag commit, or the change is held out of the release.
The references are recorded on a quiet GPU (below half its memory and load: `t3.py
record` refuses otherwise and keeps the GPU's state in the reference): on 133, with another
user's training at 98 %, two runs failed on near-miss box overlaps with the ego slow or
standing (i6 r0, a tour's i2), the kind perception lag under load makes. A run that fails
so on a quiet GPU is a real margin of Autoware's and is written up, not recorded as passed.

Must-have, in order: the fresh-clone build (241's three package failures); G55 and the
vehicle_physics default (118), then items 1 to 6 and 8 to 10 again on the spec's physics;
G34, G35, G53; 075 step 8's Isaac Town01 steps (241); step 4's ready items (CARLA 7, 11, 32,
34, 36, 37, 52 to 54; Isaac 16 to 18, 20, 21, 24 to 26, 51, 55; 56); step 6's Isaac plain
reruns; step 7's T3 references; step 8's developer recipes.

**Deferred, for the release notes:**
- recording and its tools (G4): screencast, chase view, captions, the RViz config, per-leg
  videos, Isaac's `record:=`; items 7 and 11 run without them;
- ground per scenario (G7) and the acceptance judge from a bag (G11): items 19, 50;
- a gate before a batch (G16), two hosts sharing one suite (G17; the copy-then-summarise
  step has no command), per-pilot ROS parameter overrides (G18), harness mode (G19): items
  31, 33, 35, 38 to 40, 43, 49, 57;
- CARLA 0.10 (G20, a branch): item 58;
- item 15 (upstream SSv2), item 44 (no lanelet map);
- the private-asset items (22, 23, 27, 28, 30, 59): the user checks those with their own
  configuration;
- 076's Haruna drift, the golf cart twin and Isaac own cars.
- item 24, Haruna under Autoware on Isaac (swb 139): localization passes 058 C9, the run
  stops for good before the first hairpin (below); Tsukuba not run yet.

## Progress and rulings

- Inventory (2026-11-15): 73 items, 24 gaps (G1 to G24), [077-study/inventory.md](077-study/inventory.md).
- T0/T1 (e9ecdb5): `just test` runs src/swb's tests (it never did); every pilot kind's launch
  is resolved by `play_launch dump` (test_pilot_parse.py).
- G1 (the autoware pilot's environment): an autoware pilot runs under
  `swb/autoware_env.bash`, which sources what `just ego-av` does in the same order (Autoware,
  the 042 overlay, CUDA NDT when built, acb's and this workspace's installs, TensorRT 10.8.0
  first on `LD_LIBRARY_PATH`) and warns the same way for a missing piece. Paths are host
  settings (`autoware.setup`, `overlay`, `use_overlay`, `cuda_ndt`, `tensorrt_lib`), the
  workspace's `third_party/` builds by default; stack.json records them. Ruling: the 076
  acceptance runs on this host ran stock Autoware either way (none of the three is built
  here), so their verdicts stand; a host with the overlay built ran the developer path
  fixed and the project path stock until this change.
- Batches: A (G2, G3, G6, G10, G12, G23: run and stack settings), D (G9 town preload, G21 two
  SSv2 clients) and the `simple` kind (G22) are being built; B (G5, G14, G15: own cars, free
  drive, package projects), C (G4: recording), E (G16 to G19: gate, two hosts, parameter
  overrides, harness mode) next; G8 and G13 (asset sources, CARLA map packages) need
  decisions.
- G20 decided (user, 2026-11-15): CARLA 0.10 is in the matrix. Today it lives only on the
  068 branch (068 step 8), and the workspaces pin the 0.9.16 API (`carla-0916`), so its
  cells need the 0.10 profile brought to main as a build of its own first; until then
  they report not built rather than pass.
- Private assets (user, 2026-11-15): the user docs (README, tutorials, how-to, reference)
  never mention the private NTU campus and Shalun maps; the roadmaps, scenarios and scripts
  about them stay. The matrix fetches public assets only; the user checks the work on the
  private maps with a configuration of their own, outside the repository.
- Isaac on the project workflow, live (2026-11-15, Town10HD, Autoware 1.5.0): the keeper
  marked the stack ready with its pilots while the Isaac world server was still loading
  (over 20 min on a host whose spinning disk, where Isaac is installed, other users kept
  80 % busy; the world server's own 1200 s limit restarted it once), and the first run failed
  at SSv2's SIMULATOR_INITIALIZE_TIMEOUT. Fixed (`0300af9`): the stack is ready only once the
  bridge serves SSv2, which it does only after its world server is up (`world.ready`). Then
  `town10_ego_drive` passed on Isaac and on CARLA; the README's hero is that pair.
- G9 town preload and G21 two SSv2 clients (2026-11-15), step 3. Tests without a simulator
  (the bridge's against a fake backend and an in-process world server, over real ZMQ
  sockets; the keeper's with a stand-in command). Not run live. Rulings:
  - **`/scenario_world_bridge/load_map`** (`swb_interfaces/srv/LoadMap`: `map_path`, a map
    directory or its lanelet2 file, absolute; `success`, `message`): handed to the bridge's
    world thread as `set_weather` is, and answered after its `LoadWorld`. The load is
    `Initialize`'s without the episode (no gap trace file, no frame count), at the last
    Initialize's step and realtime factor, and it forgets the bridge's entities. A new
    `BridgeBackend` trait in the bridge (over `swb_core`'s `SimBackend`) carries it, so the
    world protocol and `swb_world_protocol` are unchanged.
  - **Refused while a scenario runs**: entities spawned and the session's SSv2 asked within
    5 s. A scenario starting (Initialized, nothing spawned yet) is not protected; the keeper
    calls only between runs, after the runner stopped SSv2.
  - **The keeper calls it between the pilots' stop and their start** (`python3 -m
    swb.load_map`, rclpy, in the scenario domain with the stack's DDS settings; 10 s for the
    service, 120 s for the answer, inside the runner's 180 s for the switch), on CARLA only and
    only when something serves the seat's SSv2 port. The answer is the switch's `preload`
    (`loaded`, `failed` with `error`, `skipped`) and a `map.preload` event. A failure is not
    the switch's: the town then loads at SSv2's Initialize, one extra episode, as before.
  - **One SSv2 at a time, by connection, not by a client id in Initialize.**
    `InitializeRequest` carries nothing that names the client (`initialize_ros_time` is a
    clock reading), and no other request carries anything. SSv2 opens one REQ socket per
    scenario (`traffic_simulator::API`, per interpreter activation) and Initializes on it
    first, so the bridge's socket became a ROUTER (same request-reply exchange; libzmq gives
    each connection a routing id) and the connection whose Initialize was accepted last holds
    the session. Any other connection's requests are refused, naming the session (client,
    map, entities, last request); its Initialize too while the session is live (entities, and
    a request within 5 s). After a scenario ends (SSv2 despawns everything), never spawned
    (an ego refused at its spawn), or its SSv2 died silent for 5 s, the next Initialize takes
    over. No SSv2 change. Left open: a stray whose Initialize lands between the first one's
    Initialize and first spawn takes the session over, and the first one's requests are
    then refused (one of the two fails, with the reason, instead of both interleaving).
  - To check live: a `town02_episode_change`-style switch on a stack (the keeper's log line
    `town for Town02: loaded`, acb_bridge starting once on Town02, the bridge's `Map 'Town02'
    is already loaded` at Initialize, no `episode changed` warning), and two
    `scenario_test_runner`s against one bridge (the second's error names the first).
- A CARLA the stack starts now ends with it (2026-11-15). Seen live: stopping a simple_drive
  stack left its CARLA serving port 2000, because `CarlaUE4.sh` runs the engine as its own
  child (not `exec`) and nothing signalled it when the keeper died. The keeper now runs the
  engine binary itself (`CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping CarlaUE4 ...`, the
  script only where the binary is missing) with a parent-death signal (SIGTERM), so it ends
  even when play_launch SIGKILLs the keeper. Test: a stand-in CARLA outliving a SIGKILLed
  keeper fails test_simulator.py.
- G1 regression, found live (2026-11-15): Autoware's `setup.bash` sets its own
  `CYCLONEDDS_URI`, so an autoware pilot under the prelude ran on Autoware's DDS profile, not
  the stack's. `ego_ready` (on the stack's) saw nothing on `/api/operation_mode/state` and
  both scenarios of a map-switch suite ended "pilot ego never became ready (600 s)". The
  prelude now restores the stack's `CYCLONEDDS_URI`, `RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID`
  after every setup, and a readiness timeout now names the check's last reason.


Step 3, gaps G2, G3, G6, G10, G12, G23 (2026-11-15; tests without a simulator, live checks
open):

- **G2, gap trace per run.** The bridge reads `gap_trace_dir` (and `contact_report_dir`)
  once, at start, and has no service to move them, so the ruling is **per stack**: the stack
  points both at `stacks/<id>/episodes/`, and each run moves the files that appeared during
  it (one per `Initialize`, closed when SSv2 despawns everything at the end) into
  `runs/<id>/gap_trace/` and `contacts/` (`swb.episodes`). On with `logging.gap_trace: true`
  (swb.yaml only) or a judge's `needs: [gap_trace]`. `python3 -m swb.gap oracle|incident`
  runs gap_oracle and incident_judge as judges over the run directory.
- **G3, SSv2 run settings.** `ssv2:` (`global_timeout`, `frame_rate`,
  `global_real_time_factor`, `initialize_duration`, `managed_ego`) in swb.yaml, a suite, a
  suite item (`{path, ssv2}`), `--timeout`/`run_one`; each layer replaces the keys it names.
  `managed_ego: false` hands SSv2's ego to no pilot (check and run).
- **G10, world config.** `world:` (`weather` preset or parameters, `npc_motion`,
  `contact_report_dir`, `config_file`, `world_config_file`) merged by name with swb's side
  and sim arguments; swb's own names are refused, in swb.yaml and in `stack_args`, which now
  merges instead of replacing. Weather parameters: the keeper's `set_weather` before ready.
- **G6, Isaac options.** `isaac:` (`ego`, `render`, `follow`, `physics_hz`, `preload`,
  `wheels_topic`) to the Isaac world server beside swb's `world`/`ego`/`state_dir`; a
  preloaded map is one the stack may switch its pilots to.
- **G12, profile arguments.** `project check` (so the plan) runs the profile's own
  `check_launch_args` `problems()` over each autoware pilot's `args` and the suites'
  `pilot_args`; the table stays the profile's.
- **G23, bench metrics.** `frame_*`/`tick_*` metrics from the bridge's 200-frame "Frame
  budget" windows in the run's slice; the whole-run line belongs to the next run's slice
  and is left out.

Open, live: a gap trace through a real bridge into a run (CARLA), weather parameters
through `set_weather`, an Isaac switch to a preloaded world, `managed_ego: false` on
Isaac's rear contact (inventory row 18).
- Live on CARLA 0.9.16, Autoware 2.0.0 ego (2026-11-15, newslab-server139): the ego in
  `ghcr.io/autowarefoundation/autoware:universe-humble-2.0.0` (acb_bridge and the launch
  packages built there; `--net=host`), the stack on the host. T1: `test_pilot_parse` in the
  image, 6 passed, 2 skipped (the 1.5.0 cases). T2 `basic/town01_ego_drive`: passed, the ego
  engaged and reached its goal; 64/64 nodes, 54/54 composables, clustering and the laserscan
  grid. Open: play_launch 0.35.0's Rust parser cannot evaluate 2.0.0's `agnocast_env.launch.py`
  (`EnvironmentVariable` has no `perform`); 2.0.0 stacks resolve and run with `--parser python`.
- Live on CARLA 0.9.16, Autoware 1.5.0 ego (2026-11-15):
  - **G9 town preload**: a suite of drive_to_goal (Town01) then town02_ego_drive (Town02) on
    one stack passed 2/2. The keeper's `~/load_map` loaded Town02 in 2.7 s between the
    pilots' stop and start; CARLA's episode changed once, there, and SSv2's Initialize then
    found the town loaded (no second episode); acb_bridge started once on Town02, on that
    episode. The first map of a stack is `skipped` (no bridge serves the SSv2 port yet) and
    loads at Initialize, as before.
  - **A started CARLA ends with its stack**: after each `--stop-stack` no CARLA was left.
  - **G2, G10, G23**: a run with `logging.gap_trace: true`, a gap judge and
    `world: {weather: {preset: WetNoon, cloudiness: 30}}` passed (`judges: gap=passed`, one
    trace, G4 agree, 805 frames). The trace and contact report moved into the run;
    `weather.applied` came before `stack.ready`; frame metrics in result.json (4 windows,
    frame p95 4.1 ms, tick p95 36 ms). Under `keep: {passed: result}` the trace is pruned with
    the logs, as documented (`gap_trace/` is a `standard` file); the judge's verdict stays.
  - Still open: the keeper is SIGKILLed by play_launch about 5 s after SIGTERM, inside the
    pilots' stop grace, so stack.json ends at `stopping` rather than `stopped` (pilots and
    CARLA still end, by their parent-death signal).
- A started CARLA left alive once more (2026-11-15): its keeper's SIGTERM reached it by the
  parent-death signal, but UE's graceful exit hung with the RPC port open and unanswered,
  and the keeper (SIGKILLed by play_launch) never escalated; the next stack refused port
  2000 as "not CARLA's RPC server". Each started CARLA now has a reaper (`python3 -m
  swb.simulator reap`, its own session, so play_launch's stop does not reach it) that, when
  the keeper dies, sends SIGTERM, waits 20 s and sends SIGKILL, through a pidfd.
- T3, the AWF safety suite against its 076 reference (2026-11-15, CARLA 0.9.16 started by the
  stack, Autoware 1.5.0 clustering, `lifetime: run`): UC-ACC-001 27/27 passed (2454 s; the
  reference 2409 s), UC-AEB-001 9/9 passed (578 s; 721 s), `physical_verdict` passed on both.
  First suite on the ROUTER socket (G21), the autoware prelude (G1) and gaps A. After
  `--stop-stack` no CARLA, reaper or port was left.
- Frictions of 076's Isaac acceptance on a second host (swb 241, 2026-11-15), new gaps:
  - **G25, Isaac message bindings.** `project check` and the stack's readiness should import
    in Isaac's Python every message package the world server publishes and name a missing
    one; the fix (`just isaac-msgs`) has no `ros2 run swb` form. The blocker of that run.
  - **G26, play_launch version.** The host had 0.18.3; the workflow needs a newer one
    (0.35.0 was installed apart). `project check` should name the floor.
  - **G27, GPU choice.** host.yaml's `gpu` pointed at a card another user's job held (7.4 GB,
    85 %); nothing warned. The plan could report the card's memory and load.
  - **G28, DDS warnings.** The stack console warns `dds-env-split` for `/bridge` (domain 1)
    and `/simulator` (2) against the launch's unset 0: true and harmless, alarming to a user.
  - **G29, provenance.** result.json reads the checkout's HEAD at run time (it moved during
    the runs), not the installed build's commit.
  - **G30, web port.** The stack took web 8080 although the checkout's `.env` set
    `SWB_WEB_PORT` 8180: the project workflow reads host.yaml's seat, not `.env`, so the
    migration note must say so, or the seat must follow it.
  - Also seen: `assets fetch` took 215 min for 19 GB (the Isaac USD set, 11840 files);
    play_launch warns io_helper lacks `cap_sys_ptrace`.
- Since 2026-11-15 swb 133 manager assigns the campaign's jobs; this doc stays this session's.
- **G25 built (2026-11-16).** `project check` of an Isaac project, and so the stack before it
  starts, checks `<isaac.root>/msgs` (else the ISAAC_ROOT the world server inherits) against
  the packages aib's installed `build_msgs.sh` builds by default (its own list, one source):
  a package not built, lacking a message or service module the Autoware install has, or with
  a type-support library older than the install's is refused by name, fix `ros2 run swb
  isaac msgs`, which runs the build script for just those (`--check` lists them). Tests
  without Isaac (test_isaac.py, a fake install; one test reads aib's real script). Live on
  this host: the check named `autoware_perception_msgs is not built` (the same gap as 241's,
  which town10_ego_drive had not hit), `isaac msgs` built it, `--check` and `project check`
  then passed. User docs: getting-started/isaac.md, commands, troubleshooting.
- **T2 on CARLA 0.9.16, every pilot kind (2026-11-16)**, each a copy of an example project with
  one pilot changed, `ros2 run swb run <xosc> --start-stack --stop-stack` (the stack starting
  CARLA), seat 0, Autoware 1.5.0, build e00448a; each run's result.json is in the project's
  run directory. GPU shared with three other users' jobs (25 to 29 GB of 32 in use), load 25
  to 60, so no timing.

  | Pilot kind | Project, scenario | Verdict |
  |---|---|---|
  | `simple` | simple_drive, drive_to_goal | passed |
  | `carla_basic` (ego) | simple_drive with the ego `carla_basic`, drive_to_goal | passed |
  | `launch` (`agent: own`) | simple_drive with the ego `launch: swb_launch simple.launch.py` | passed |
  | `command` | multi_pilot four_pilots: ego and `lead` `carla_basic`, `bg_av_1` `simple`, `cut_in` the SDK program | passed |
  | `none` | simple_drive with the ego `none`, simple.launch.py started by hand against the seat's relay | passed |
  | `autoware` | first_drive, drive_to_goal | passed (second try) |

  Found on the way: four_pilots with a `simple` ego ended `failed` on its own collision
  condition (`ego_hits_cut_in`): `simple` ignores traffic, as documented, so that scenario
  needs a pilot that brakes. The first `autoware` run ended `error` at the ego's spawn, the
  bridge naming the cause ("the ego's parts were built from different vehicle specs (the
  vehicle agent 480be4fd0d00, the catalog 01b4d3f52735)"): a non-recursive `git submodule
  update` had left acb's `deps/` on older specs. `git submodule update --init --recursive`
  and both workspaces rebuilt, it passed. The spec guard did its job; the fix is the one
  CLAUDE.md gives.
- **G26 built (2026-11-16).** `project check` (so the stack before it starts) runs the
  `play_launch` first on `PATH` and refuses one older than 0.30.0 (the floor install.md
  names), or none, naming the version, its path and the pip line; a version that does not
  parse is reported, not refused. Live here: `~/.local/bin/play_launch` 0.20.0 refused, the
  0.34.0 venv first on `PATH` passed.
- **G30 built (2026-11-16).** A seat whose lock is free but whose ports something already
  listens on (a developer stack on a checkout's own `SWB_WEB_PORT`, which the project
  workflow does not read) is skipped by `auto` and refused when named, naming each port in
  use: the runner checks the web UI port too before it starts play_launch, the stack's plan
  the ports it binds later (SSv2, its two release sockets, the relay, the world server),
  since play_launch holds the web port by then. `seat show` and `seat env` resolve as before.
  Ruling: no reading of `.env` (host settings stay in files, decision of 076); the seat
  table is the one way to move a stack's numbers. Docs: several-stacks.md.
- **G31 (new, 2026-11-16, from this host's T2).** A `git submodule update` without
  `--recursive` left acb's `deps/` on older commits than acb's own pins, and the build from
  them gave a vehicle agent of another spec; the bridge refused the ego's spawn, a minute
  into the run. The stack could catch it at its start: the workspace's nested submodules
  checked out off their pins (`git submodule status --recursive`, a `+` line), as
  check_pins does for the pins themselves, named with the fix.
- G30 live (2026-11-16): a listener on 8080 (seat 0's web UI), then `swb run --start-stack`
  on simple_drive: the stack started on seat 1 and passed.
- **G28 built (2026-11-16).** play_launch's DDS check compares each node with play_launch's
  own environment, which was the terminal's (Autoware's Cyclone profile, domain 0): every
  stack warned for `/bridge`, `/simulator`, `/stack_keeper`. `swb run --start-stack` now
  starts play_launch on the seat's scenario domain and the stack's Cyclone profile, as its
  nodes are. Live: three warnings became one, `/simulator` on the ego's domain, which is
  meant; troubleshooting says so, and how to start a stack by hand without the others.
- **G29 built (2026-11-16).** `just build` stamps the install (`install/swb_build.json`: the
  checkout's commit, `+dirty` when its tracked tree had changes, the submodule pins, the
  time). A run's `provenance.swb` is that build, `provenance.built` its time, and
  `provenance.checkout` the checkout's commit at run time when it moved since. An install
  without a stamp (a plain `colcon build`) records the checkout, as before.
- G29 live (2026-11-16): `just build` stamped 6d31f4a, and a simple_drive run's provenance
  named it, with its build time.
- **G27 built (2026-11-16).** When `swb run` starts a stack it reads the seat's GPU
  (nvidia-smi) and warns, without refusing, when the card has less memory free than the
  simulator needs (about 2.5 GB for CARLA, 10 GB for Isaac) or is at least 80 % busy,
  naming both numbers and the `gpu:` setting. Ruling: a warning, since other users' jobs
  come and go. Live here: `GPU 0 (27.2 of 31.8 GB in use): 4.7 GB free, isaac needs about
  9.8`, nothing for CARLA.
- **G31 built (2026-11-16).** When `swb run` starts a stack it lists the workspace's
  submodules, nested ones too (`git submodule status --recursive`), checked out off the
  commit their parent pins, and warns with both commits and the fix (`git submodule update
  --init --recursive`, then rebuild). Ruling: a warning, not a refusal, since a developer
  may move a submodule on purpose; and it reads the checkout now, not what was built. Live:
  acb's `deps/sim_vehicle_spec` put back on e7d8656 was named against its pin 9b7e48d.
- **G14 built (2026-11-16): `ros2 run swb drive`.** A run without a scenario on the
  project's stack: swb_free_drive (073 S5) in SSv2's place initializes the map, spawns the
  entity from its catalog entry at `--spawn` and steps the world; once the entity's pilot is
  up on the episode (the relay's query: a phase past INITIALIZING, with a pose), `--goal` is
  sent as a commander's `set_goal` (route, control and engage for every kind); ARRIVED
  passes, `--timeout` fails, a free drive that ends is an error. Without a goal it steps
  until Ctrl-C or the timeout (driven by hand). Its run directory is a run's; the steps go
  to `timeline.jsonl`, `result.json` keeps the schema's events (`route_complete`). Rulings:
  `lifetime: run` pilots are refused (nothing starts them outside a run); one goal per
  drive (item 47's pose lists are a sequence of drives, or a scenario). Live on CARLA
  0.9.16: drive_to_goal's spawn and goal, the `simple` pilot arrived 17 s after the goal
  (run 2026-11-16T01-04-27), Autoware 1.5.0 was up 22 s after the spawn and arrived 27 s
  after the goal (01-05-48). Migrates item 46 (acb `just sim` + `just autoware`); 48 (the
  same on Isaac) waits for the Isaac cells' GPU room.
- **G15 built (2026-11-16).** (a) The host setting `dds.profile`: a Cyclone XML that takes
  the place of swb's localhost profile for the stack, its processes (`dds_config`), the
  runner's play_launch and `seat env`. Live: a stack with the setting recorded it in
  stack.json's env and its run passed. (b) Ruling on "a package project's state goes
  where": a project in an installed package's share is read-only; `project check` reads it,
  `assets fetch`, `run`, `drive` and the stack refuse it, naming the `project new <dir>
  --from pkg://...` that copies it, and the copy holds its lock, assets, state and runs.
  Live: the installed simple_drive refused that way. A package ships its project, a user
  copies it once.
- **G5 built (2026-11-16): `vehicles:` is loaded, on CARLA.** At start the stack generates
  each listed spec (`sim_vehicle_spec.gen`) into `state/vehicles/<name>/` (catalog entry,
  spec_hash, CARLA physics and sensors) with a URDF of its frames from the spec, and one
  catalog of the installed cars and the project's (`state/vehicles/catalogs/`), which
  `SWB_VEHICLES` names for every run (the installed catalog's directory without own cars)
  and the bridge reads (`vehicle_catalogs`). The ego's own physics go to CARLA's world
  server (`vehicle_physics`); a pilot of an own car gets `vehicle_config`, `spec_hash_file`,
  `autoware_spec_hash_file`, `vehicle_maps_dir` and (simple) its `spec` and the URDF.
  `project check` refuses an installed car's name or body, an `autoware` pilot of an own car
  without its Autoware packages, and an own car on Isaac. Fixes found on the way: the
  simple, launch and command pilots' includes of the vehicle side named only `sim:=`, and
  play_launch's parser scopes an include to what it names, so nothing else reached the
  vehicle side (they forward every argument now; T1 case
  `test_an_own_car_simple_pilot_resolves_without_the_cars_autoware_packages`); the vehicle
  side's `vehicle_maps_dir` default resolves `<car>_description` in both branches of its
  `eval`. Rulings: pedal maps from `vehicles/<name>_maps/` beside the spec, else the Tesla
  Model 3's, recorded in stack.json; SSv2's `vehicle_model` stays its default for an own car
  without `<car>_description` (SSv2 reads only that package's parameter files and does not
  launch Autoware here); Isaac's rig from the spec is later. Live on CARLA 0.9.16: a
  Lincoln MKZ 2020 body with its measured geometry (the Tesla's other sections),
  `simple` pilot, drive_to_goal passed (run 2026-11-16T01-38-11); the world server, acb and
  the catalog all named spec e12f66a91e53, and acb verified the applied physics.
- Regression after G5's pilot-launch change (2026-11-16, CARLA): simple and launch passed;
  command (four_pilots) passed on the second try. The first ended `error` at SSv2's
  timeout: both carla_basic agents' CARLA calls timed out (10 s) at 01:46:44 under the host's
  load (load 18 to 20, GPU 90 % busy with other users' jobs), the relay dropped them as
  silent and they registered again, and the ego's agent never had its goal again.
- **G32 (new, 2026-11-16).** An agent that registers again within a scenario (it went
  silent, or restarted) has lost its goal: the relay or the agent should hand the
  scenario's last `set_goal` (and speed limit) back to a re-registered agent of a live
  episode, or the run should end `error` naming the drop, not wait for SSv2's timeout.
- **G32 built (2026-11-16).** The relay keeps, per entity, the goal and the speed limit its
  agent last accepted within the episode (forgotten at a reset, a teleport, an episode end,
  a clear or a stop). An agent that registers again (it went silent, or restarted) is
  handed both back once it reports a state. When it refuses, reports no state within 10 s,
  or registers again while driving with no goal on record, the entity is `lost`, and a
  commander's query carries the reason. `swb run` asks the relay every 2 s while SSv2 runs
  and ends the run `error` naming the pilot and the reason, stopping SSv2 rather than
  waiting for its timeout. T0: scenario_agent_relay/test/test_replay.py (an agent killed
  mid-episode and registered again: handed back, refused, silent, after a stop), and
  test_run.py (wait_ssv2, and a run ended `error` by a lost goal). Not yet seen live (it
  needs a CARLA stall like the one that showed it).
- **Item 48 migrated (2026-11-16): `swb drive` on Isaac Sim 6.1**, Town10HD, the spawn and
  goal of town10_ego_drive. The `simple` pilot: up 1.5 s after the spawn, arrived 30 s
  after the goal (run 2026-11-16T02-24-09). Autoware 1.5.0: up 5.5 s after the spawn,
  arrived 52 s after the goal (02-28-37), the stack started and stopped by the drive. The
  world server came up in about a minute from the NVMe copy of Isaac. Two fixes on the way:
  - `drive` sent a pilot left up by an earlier run its goal at once, on its old pose, before
    the spawn; it planned from there and never arrived (the first try failed at 600 s). The
    goal now waits for the pilot's pose at the spawn (3 m).
  - On Isaac the world server's ground truth was on for `kind: simple` only, so a `launch`
    pilot running simple_pilot had no pose and the run ended at SSv2's Autoware wait. A
    pilot that says `inputs: [ground_truth]` turns it on too. Live: the launch cell on
    Town10HD then passed (02-26-10). (241 holds the Isaac cells; this one was rerun here
    only to check the fix.)
- **Step 4 (2026-11-16): items 46, 47 and 48 migrated** onto `ros2 run swb drive`. 47:
  `--poses <file>` reads acb's pilot poses (`initial_pose`, `goal_pose`) as the spawn and
  the goal; live on CARLA 0.9.16 with Autoware 1.5.0 and `autoware_sim_pilot`'s
  example_poses.yaml (Town01), up 23 s after the spawn, arrived 99 s after the goal (run
  2026-11-16T02-35-06). The rest of step 4 is laid out for handing out by host in
  [the inventory, section 4](077-study/inventory.md#4-step-4-hand-out-2026-11-16): each
  item's host class (CARLA, CARLA + CUDA, two Autoware, Isaac, two hosts, private assets),
  its gaps still open, and whether it can be migrated now.
- G32, a false alarm found live (2026-11-16): between suites on one stack the keeper
  restarted the ego's pilot for a map switch; its last phase was DRIVING (the previous run
  had ended there), so the relay marked it lost, and the next run (town02_ego_drive) ended
  `error` at its first poll, its retry too. The relay now stamps when an entity went lost,
  and the runner counts only a loss at or after its SSv2 started (a reset clears it in any
  case). Test: test_a_goal_lost_before_the_run_began_is_not_this_runs.
- **Step 4, items 1-3 migrated (2026-11-16)**: `scenarios/projects/basic/` (Autoware 1.5.0
  ego, the five towns, the `physical_verdict` judge), its suites naming the starters where
  they live (`src/swb_examples/scenarios/basic/`). Live on CARLA 0.9.16 (started by the
  stack), one stack per command:
  - `suites/town01.yaml` (item 1): 6 of 6 passed, `physical_verdict` passed on each
    (rear_contact declares its contact), 02:43 to 02:52; the reference 6 of 6 (034).
  - `suites/towns.yaml` (item 2): town02, town03, town05, town10 passed, 4 of 4; the keeper
    loaded each town between runs (11 to 13 s each), no extra episode; the reference 020.
  - `suites/episode_change.yaml` (item 3): passed (03-06-30), the town load inside the run.
- **Step 4, items 5 and 6 (2026-11-16).** Item 6: `scenarios/projects/basic`'s
  `suites/tutorial.yaml`, town01_follow_to_goal passed, `physical_verdict` passed. Item 5:
  safety_suite's `centerpoint.yaml` (a copy), Autoware 1.5.0 with
  `lidar_detection_model: centerpoint`, `lifetime: run`: UC-ACC-001-0001 passed (27
  variants, 2126 s), and UC-AEB-001-0001, expected to fail (2 of 9 with centerpoint_tiny,
  roadmap 020), passed all 9 (631 s): `passed (expected failed)`, exit 1. Since 2026-11-15
  `centerpoint` is the full model (roadmap 074 S5). The expectation is dropped and the
  README says so. This host has no TensorRT 10.8 runtime (`just setup-tensorrt`), so each
  Autoware start rebuilt the engines.
- **241's Isaac T2 cells (2026-11-16, origin/main 979ef22, Isaac Sim 6.1, Town01,
  drive_to_goal)**: autoware passed (the first run `error`, below), simple, launch
  (`agent: own`), command (simple_pilot, `vehicle_side: true`) and none (a scenario SSv2
  drives itself) passed; launch and command with `isaac: {ego: <aib's ego.yaml with
  ground_truth>}`, which the ground truth fix (3b95116) makes unnecessary: 241 reruns
  them as a user writes them once it is on origin. Report on the NAS, "2026-11-16 swb 077
  T2 Isaac cells".
- New gaps (2026-11-16):
  - **G33, the storyboard log per run.** incident_judge needs a log of SSv2's storyboard
    state changes (`/simulation/context`); nothing records it in a run (033 recorded it
    from scratch scripts). Blocks items 8 to 11.
  - **G34**: a `kind: none` pilot on an entity whose controller is Autoware is not refused;
    the run fails after 119 s naming the vehicle side instead.
  - **G35**: `project check` re-hashes every fetched file (25 GB for the Isaac set) on each
    call; it should compare sizes and times against the lock and leave hashing to `assets
    check`.
  - **G36**: the Isaac world server finds car bodies under `SWB_VEHICLE_USD` (default a host
    path, `/mnt/disk1/carla-usd/vehicles`), which the stack does not set: on a host without
    that path every car, the ego and the NPCs, is a box, so the sensors see boxes.
  - **G37**: the first Isaac run on a host can end `error` at the ego's spawn. 241's logs: the
    preload ego's sensors were not ready after 60 warm-up frames; the ego's spawn at 01:59:33
    was followed by two minutes of MDL shader compilation (about 380 lines of
    rtx.neuraylib/omni.rtx.materials, Town01's materials on the sensors' first renders), and
    the bridge's 120 s spawn wait ran out (exit 75). The rerun, shaders cached, spawned in
    0.2 s.
  - **G38**: CARLA's agents (carla_basic, and any pilot using them) import shapely and
    networkx; a host without them crash-loops the pilot with ModuleNotFoundError (241's step
    9 shard; seen here too).
- **G38 built (2026-11-16).** `project check` (so the stack) refuses a `carla_basic` pilot
  when the `python3` first on `PATH` cannot import shapely or networkx, naming the pilots
  and the apt and pip fixes; install.md lists them. Ruling: `carla_basic` only, since a
  `command` or `launch` pilot's imports are its own.
- **G37 built (2026-11-16), in swb_isaac_world.** The preload ego now warms its sensors for
  up to 1200 frames, not 60 (`PRELOAD_WARM_FRAMES`), so a cold host compiles the map's
  shaders before the bridge's spawn wait starts. The commit is on swb_isaac_world; the pin
  here moves once that commit is on origin. A cold host has to confirm it (241's).
- **G36 built (2026-11-16).** On Isaac the stack sets `SWB_VEHICLE_USD` to the project's
  asset set (`assets/carla-usd/vehicles`) when that set has its `vehicles.json`, so the ego
  and the NPCs are their CARLA bodies wherever the project is. This changes what the sensors
  see, not only the picture: the NPCs' bodies come from the same catalogue, so it stays ahead
  of G33. Live: a `simple` drive on Town10 passed with the variable naming the project's set,
  and the world server logged no box stand-in.
- **G33 built (2026-11-16).** A judge may `need: [context]`: the runner then records SSv2's
  storyboard state changes (`/simulation/context`, as changes only) into the run's
  `context.jsonl` (swb.context), kept at `standard`. `python3 -m swb.gap incident --incident
  auto` takes the incident's number from the scenario's name, so one judge serves a suite of
  them; i7 and i8 have no row in incident_judge's table and record `error` there. The demo
  incidents are a study project, `scenarios/projects/demo` (suites for items 8, 9, 10).
- **G39 (found and built 2026-11-16).** A stack on CARLA with `carla.start: auto` and no
  `carla.root` used whatever served the port; with nothing there it waited out its 1800 s
  readiness, the world server's "cannot read the CARLA server version" its only word (item
  8's first attempt, a project without `.swb/local.yaml`). Now refused at once, naming the
  three ways out; `simulator_start:=external` still waits.
- **075 step 8, CARLA guides (swb 118, 2026-11-16).** Every command of README, install.md,
  carla.md and tutorials 01 to 05 run once on origin/main 11a2407 by a fresh reader (own HOME,
  fresh clone). All run end to end (68 packages in 8 min; T1 to T5 pass). Report on the NAS,
  "2026-11-16 075 step 8 CARLA guides". Its findings, as gaps:
  - **G40**: install.md's `rosdep install --from-paths src --ignore-src -y` fails on
    `python-yamale-pip` (scenario_test_runner) and installs nothing; needs `-r` (then it adds
    pygame, lanelet2, shapely, networkx) and `pip install --user yamale`.
  - **G41**: under play_launch 0.35 the stack's progress lines (seat, CARLA, pilot ready,
    ready) reach only the keeper's log, not the terminal: a reader cannot tell when the stack
    is up.
  - **G42**: the `physical_verdict` judge in T5 and safety_suite fails every clean run ("no
    contact report from the bridge in the run window"); `--contacts {run_dir}/contacts`
    passes.
  - **G43**: play_launch writes `play_log/` into the project; the generated `.gitignore`
    does not list it.
  - **G44**: `swb-agent new` prints three check commands that fail; `ros2 run swb_agent
    swb-agent check <file>:<Class>` works.
  - **G45**: the stack prints "CARLA serves RPC (None)", not its version, and "web UI: pass
    --web-addr" when it was passed.
  - **G46**, doc corrections: T3's `ls state/Town01/` (the dataset ships the signal table);
    T1 step 5's kept files for a passed run; T4's cut-in jitter (plus or minus 5 m, not plus
    up to 5); T2's elided `<Axles>`; T5's prune dry run (`--older-than 0`); T5 exercise 3's
    counts (28 and 10, with the judge's testcase) and time (about 61 min); install.md's
    interactive rustup (`-s -- -y`) and the first build time (about 8 min on 48 cores); T1
    step 4's first start (CenterPoint's minutes; clustering ready in 10 s); 075 step 8's page
    list (the current set is the one above).
  - Minor: a failing `ros2 run swb` also prints ros2run's "Process exited with failure N".
- **G42 built (2026-11-16): a false pass as well as a false failure.** physical_verdict read
  the bridge's default report directory (`~/.ros/swb/contacts`), never the run's. On a fresh
  host it found nothing and failed every clean run (118's finding); on ours it judged the
  newest report there, from 2026-11-14, so every `physical_verdict` pass recorded by the
  basic and safety_suite runs since is a verdict on some other run, not on the run it sits
  beside. As a swb judge (`SWB_RUN_DIR` set) without `--contacts` it now reads the run's own
  `contacts/` and judges every report in it. No other judge reads a shared path (gap_oracle
  and incident_judge get the run's files from swb.gap; run_suite, which also defaults to it,
  is not a judge). Re-judged (2026-11-16) with the fixed judge on each run's own reports,
  the records left as they were: 16 of the 18 passes recorded here hold (basic's 12; the
  safety use cases, now judged over all 27 and 9 variants, not the newest alone). Two
  safety_suite runs of 2026-11-15 (02:23 UC-ACC, 03:03 UC-AEB) kept no reports of their own:
  their passes were a stale report's, and they now read `error`; that suite's later runs
  (05:59, 06:41) pass. So no live rerun is needed.
- **G40 built (2026-11-16).** install.md's rosdep line is the justfile's (`--from-paths
  $(colcon list --base-paths src -p) --skip-keys python-yamale-pip`, then pip's yamale);
  `rosdep install --simulate` exits 0 with it. rustup runs with `-s -- -y`.
- **G41 built (2026-11-16), with G45's two lines.** The keeper writes its lines to its
  launcher's stdout as well (`/proc/<parent>/fd/1`, unless that is its own), so `play_launch
  launch ... swb.launch.xml` shows seat, CARLA, pilots and ready in its terminal, and
  `--start-stack`'s console log has them. The first line names the web UI the launcher was
  given (`--web-addr`), the hint only when none was. "CARLA serves RPC (None)": the probe's
  reading of CARLA's version, checked on the next live CARLA.
- **G47 built (2026-11-16): the wait was kept, the message misled.** 118's log shows the
  check started at 07:48:11 and the next command at 08:18:59: `hello --wait 1800` waited the
  full 30 min for a world server that kept dying on that host's GPU, then printed only its
  last try's "no answer within 2 s". A failed wait now says how long it waited in all
  ("... (the last of N tries over T s)"); swb_world_protocol 0349578.
- **G48 built (2026-11-16).** A run without SSv2's JUnit gives SSv2's own word for it: its
  timeout line (`SSv2 wrote no JUnit: The simulation has timed out. Forcibly inactivate.`),
  else the last ERROR or FATAL line of its test runner or interpreter, read from the run's
  `logs/scenario/`; `runs why` shows it with the reason. Checked on c_command's four_pilots
  run of 2026-11-16 01:46, which said only "(None)". A troubleshooting row says what the
  timeout means.
- **G43 built (swb 118, 2026-11-16, cd744de).** The generated and example `.gitignore`s list
  `runs` (no slash: a link is not matched by `runs/`) and `play_log/`; `project new --from`
  no longer copies `play_log/`.
- **G44 built (swb 118, 2c71830).** `load_class` imports a module from the working directory
  (ros2 run put the entry point's directory first on `sys.path`); the skeleton's hints, its
  README and agent-sdk.md say `ros2 run swb_agent swb-agent`. A test runs the printed forms
  with nothing added to `sys.path`.
- **G50 closed as a doc gap (swb 118, eb38b13).** The code was right: 1 is a problem, 3 a
  refusal, and a car mismatch is a problem. commands.md now lists which findings refuse
  (asset or map not fetched, an unreadable file or entry, a car not installed or off its
  box); T1 and T6 step 6 say which they meet.
- **G52 (found and built 2026-11-16, item 8).** `logging.keep` followed the run's verdict
  only: a run SSv2 passed and a judge failed kept `result` (no logs, no gap trace, no
  storyboard log), so i1's judged failure could not be looked into. A judge's `failed` or
  `error` now keeps what that verdict keeps, when that is more.
- **G45's version line built (2026-11-16).** "CARLA serves RPC (None)": the probe called
  `version` with no arguments, and CARLA refused it ("invalid number of arguments"); its
  error reply was taken as CARLA's, without a version. LibCarla sends a Metadata argument
  first; the probe now does, and reads the version out of CARLA's Response wrapper
  (`[[1, "0.9.15"]]`, measured on a CARLA 0.9.15). The tests' fake server answered with a
  bare string, which is why they never saw it.
- **Item 8, first pass (2026-11-16, demo project, clustering, CARLA 0.9.16 Epic).** SSv2
  passed i1 to i5. incident_judge passed i2, i4, i5. i3 failed only on "--stop-line not
  given": `--incident auto` had no stop line; the scenario now declares it
  (`csbIncidentStopLine`, from the site file, 325.73 -330.61 yaw 0) and swb.gap reads it.
  i1 failed "the ego never stood behind the standing lead": the lead braked at its fallback
  brake point with the ego 107 m behind at 4.17 m/s (033's reference: the brake fired with
  the ego 13.4 to 13.6 m behind at 3.3 m/s, stop gap 4.8 to 5.3 m), and the ego was still at
  4.17 m/s at the closest gap, 19 m, when the scenario ended. Treated as a possible safety
  regression (133): rerun with full capture, and against the old developer recipe.
- **G49 closed (swb 118, 2026-11-16): nothing to change.** Only CARLA's agents import
  shapely and networkx, through `carla_basic`, which is a CARLA-only pilot; install.md, which
  isaac.md has a reader do first, installs both since G38. No Isaac step needs them.
- **G46 built (swb 118, 2026-11-16, 39df738).** T1 to T5, install.md and 075 step 8 as 118's
  pass found them: T3 checks `assets/maps/Town01/carla/`; T1's kept files, first start and
  ros2run line; T2's axles in full; T4's jitter (give or take 5 m) and exercise 2; T5's
  `--older-than 0` and exercise 3's counts and time; install.md's build time and size (7 to
  8 min on 48 cores, 11 GB build and 1 GB install); 075 step 8's page set.
- **G51 built (swb 118, 2026-11-16, 68b477c).** isaac.md's install time (6 min) and where
  `ISAAC_KIT_ARGS` goes (the environment file); T6's set size (19 GB, hours); T7's soil-zone
  log lines. T6 step 6's cause: haruna_down's own limit is 600 s of simulation time and SSv2's
  `global_timeout` defaults to 400 s, so the run was cut off before the scenario decided; the
  page runs it with `--timeout 700`. Open: the course's project should carry it itself.
- **G53 (2026-11-16, from G51).** A scenario whose own time limit (a stop trigger on
  simulation time, haruna_down's 600 s) exceeds the timeout the run gives SSv2 (`global_
  timeout`, 400 s by default) is cut off as `error` before it decides. `run` should give SSv2
  the scenario's limit plus a margin when it is larger, or `project check` name the mismatch;
  until then T6 says `--timeout 700`. After items 8 to 10.
- **G32's false loss (2026-11-16, item 8).** i3's rerun ended `error` 13 s in: "its agent
  registered again while DRIVING, and the relay holds no goal to hand back". The relay had
  kept i1's pilot's last phase (DRIVING: i1 ends with the ego moving), and i3's fresh
  `lifetime: run` pilot registered after SSv2 started. With no goal standing, a new agent
  is a new start: an agent restarted mid-goal has its goal standing and takes the replay
  path. That branch now logs and marks nothing.
- **Item 8's i1, explained (2026-11-16).** The ego held 4.17 m/s in every incident: the
  profile's default `max_velocity` (upstream's `max_vel`). 62eae3c (2026-10-20) re-timed all
  six templates for the demo tour's ego (`max_velocity` 11.11, CenterPoint, the 042
  overlay): i1's lead cruises at 8 m/s and brakes at an 18 m gap, which a 4.17 m/s ego never
  closes, so the lead brakes at its fallback point 107 m ahead. 034's 5 of 5 (2026-10-16)
  predates it. Not a regression: inventory rows 8 to 10 named the wrong pilot. The demo
  project's ego takes item 11's arguments.
- **The fresh-clone build (2026-11-16, 241's 075 step 8 pass; a release must-have).** On
  241, install.md's `colcon build` failed three packages from a fresh clone at 85465d6:
  carla_manual_control and carla_pcd_gen ("no matching package named
  autoware_adapi_v1_msgs ... required by acb_bridge", the plugin's patches written per
  package), and scenario_test_runner (`find_package(replay_testing)` under BUILD_TESTING).
  Here a fresh clone at 05a2214, the same colcon-cargo-ros2 0.5.3 and install.md's command
  built acb_bridge, carla_manual_control and carla_pcd_gen (the plugin wrote one workspace
  `src/autoware_carla_bridge/.cargo/config.toml` with every patch), 60 packages before the
  build was stopped for the host's memory. 241 found the cause: colcon-cargo-ros2 0.5.3
  reads Cargo.toml with `tomllib`, else `tomli`, and with neither (Python 3.10, a fresh
  HOME) returns nothing and says nothing, so every crate is taken for a standalone one, with
  only its own patches; this host's `~/.local` had tomli. install.md and `just setup` install
  it. The plugin's fix (the user's repo, approved and released 2026-11-16): colcon-cargo-ros2
  **0.5.4** declares `tomli>=1.1; python_version < '3.11'` and raises naming the file when a
  Cargo.toml cannot be read (an absent one still reads as none). Checked on 241's case, a
  Python 3.10 environment without tomli and a fresh clone: 0.5.3 failed as on 241; 0.5.4
  without tomli stopped at once ("... has neither: pip install tomli"); 0.5.4 installed
  normally pulled tomli and built acb's three crates. install.md and `just setup` pin 0.5.4.
  install.md now builds with `-DBUILD_TESTING=OFF` (as `just build` does; replay_testing
  comes only with a full rosdep install) and says how to drop the `[[patch.unused]]` lock
  entries a plain build leaves before a `git pull --rebase`.
- **G55 (found 2026-11-16, swb 118, item 14): an installed car runs CARLA's blueprint
  physics.** On a project stack the tesla_model3 ran with acb's "carla_physics none": the
  stack sets `vehicle_physics` only for an own car, so the spec's tires and mass reach CARLA
  only through `just run`. Every project result to date ran that way. 118 takes it with the
  `just run` default (carla.launch.xml); items 1 to 6 and 8 to 10 run again after it.
- **G54 (found and built 2026-11-16): the ego drove blind.** On the tour's arguments i1
  collided: the gap trace shows the ego at 11.11 m/s from 48 s to the impact at 57.05 s
  without braking, behind a lead at 8 m/s and through its stop; the lead's "10.35 m/s^2"
  was the impact. lidar_centerpoint was still building its TensorRT engine all through the
  scenario (this host had no Autoware 10.8 runtime, so every Autoware start rebuilt it, and
  `lifetime: run` started Autoware for each run), while `ego_ready` counted the pilot ready
  on the ADAPI and planning alone. Measured on a stack brought up alone: the detector's
  output (`.../detection/centerpoint/objects`) had a subscriber and no publisher until the
  engine was built, 2.3 min after the pilot was called ready; on a ready stack only debug
  topics and the traffic light pipeline's absent camera6 stay unpublished. `ego_ready` now
  holds an object recognition topic (debug excepted) with a subscriber and no publisher as
  not ready, naming it; `project check` and the stack warn when a CenterPoint pilot has no
  `autoware.tensorrt_lib` with a libnvinfer.so.10; install.md and the how-to name the key.
  Audits: 133's 078 runs (the 10.8 runtime set) and 118's 074 runs (N5: 172 starts on cached
  engines; S5d: two first-start builds, tracking non-empty throughout) were not affected.
- **Items 8 to 10 migrated (2026-11-16, `scenarios/projects/demo`).** The ego is the demo
  tour's (`max_velocity` 11.11, CenterPoint, `use_multi_channel_tracker_merger: false`, the
  042 overlay, built on this host for it), `lifetime: stack`, on Autoware's TensorRT 10.8;
  judges `incident` (`--incident auto`, needs gap_trace and context) and physical_verdict.
  - Item 8, suite incidents: 5 of 5 passed SSv2 and both judges. i1 against 033's route C
    reference: brake with the ego 32.2 m behind at 8.85 m/s (7.6 to 8.2 m/s, 22.6 to 45.7 m),
    the lead's peak 6.0 m/s^2 (6.0), stop gap 5.19 m (4.9 to 5.6), stood 5.7 s (5.6 to 6.2).
  - Item 9, suite i6_centerpoint: 3 of 3 passed both judges, the ego at 0 m/s near the
    cyclist each time (042: 3 of 3 yielding).
  - Item 10, suite cut_in_rear, the first live runs (025 only rendered them): i8 passed SSv2
    and physical_verdict. i7 hit the cutting car twice: the first run ended `error` at SSv2's
    acceleration bound (the ego at -17.4 m/s^2 on a 198 N s touch), the retry passed SSv2 and
    failed physical_verdict (2289 N s, 25 frames). In the trace `i7_cut` pulls out from rest
    24 m ahead of the ego at 11.11 m/s, and the ego brakes from an 8 m gap. A scenario
    question for 025 (timed for this ego, or too tight for it), not a migration fault;
    incident_judge has no rows for i7 and i8 and records `error` for both, as designed.
- **G13 built (2026-11-16; the user's ruling, through 133: CARLA map packages are a host
  setting, like `CARLA_DIR`).** `carla.map_packages`, the directories this host's CARLA towns
  are installed in (default the install's `CarlaUE4/Content`); `project check` refuses a
  CARLA project map whose town (after the world server's `map_alias`, Town10 to Town10HD) has
  no `.umap` there, naming the key. A CARLA with no root and no packages set is not checked.
  On this host: Town01 to Town10 found, NTU not (its package is not installed here).
- **G8 built (2026-11-16; the user's ruling, through 133: public assets from Hugging Face,
  private ones from the NAS or a local path; user docs name only the public datasets).** A
  host names the datasets it keeps a copy of, `datasets: {<name>: <directory>}` in its host
  settings; `assets fetch` links such a dataset from there with the project's `take:`, its
  file list in the project's git-ignored `state/assets.local.json`, and `swb.lock` keeps only
  the Hub's entries, so a project naming `- dataset: ntu-campus-maps` reads the same on a
  host with the NAS copy and on one fetching it with a token. `assets check` refuses a copy
  that is not there. host-settings.md documents the key without naming a private dataset.
- **075 step 9 (2026-11-16; the user's ruling, through 133: public only what is built from
  public assets).** Already so: carla-town-autoware-maps, carla-0.9.16-isaac-usd,
  isaac-road-physics-stages, haruna-tsukuba-courses and newslab-golfcart-models are public;
  shalun-map and ntu-campus-maps are private. Nothing flipped. Open for the user: whether
  the golf cart's models count as private NTU material.
- **Item 12 migrated (swb 118, 2026-11-16, 918cf60).** `scenarios/projects/gap`, suite
  `gap`: entity_bodies and motorbike_body passed twice (oracle 0/0/0), npc_shoulder passed
  twice (0/1/0, 20 spawn infos, as 032), truck_pass failed twice as expected (G1 1 ego and
  G3 1, fail_collision: 024's positive control); physical_verdict passed all 8.
- **075 step 8, Isaac guides (swb 118, 2026-11-16).** isaac.md, T6 and T7 by a fresh reader
  on origin/main 75e00ab: the install (Isaac 6.1.0.0, 6 min) and the message build work as
  written; Haruna ran. That host's driver (615.71.09) loses the GPU on every Town01 render
  (ERROR_DEVICE_LOST, 22 world-server crashes in 30 min), so T6 steps 3 to 5 and T7 steps 3
  to 5 were blocked by the host, not the pages. Report on the NAS, "2026-11-16 075 step 8
  Isaac guides". Its findings, as gaps:
  - **G47**: `swb_world_check hello ... --wait 1800` gives up after 2 s ("no answer within
    2 s") while the world server starts; T6 step 3's check relies on the wait.
  - **G48**: a run that ends because SSv2 timed out ("The simulation has timed out.
    Forcibly inactivate" in its log) is `error ... SSv2 wrote no JUnit (None)`; `runs why`
    does not say it timed out (T6 step 6, haruna_down with tesla_model3, 7 min).
  - **G49**: isaac.md alone installs neither shapely nor networkx (G38's modules); the reader
    had them from carla.md.
  - **G50**: `project check` exits 1 on a car mismatch, where T1 calls 3 "refused".
  - **G51**, doc corrections: isaac.md's check `ls $ISAAC_ROOT/msgs` in a new terminal (the
    page has just said no export is needed: `ls ~/isaac-sim/msgs`); where the two-GPU
    `ISAAC_KIT_ARGS` export goes; the install takes 6 min, the hour is T6's fetch; T6 step
    2's set is 19 GB (11840 files, 2 h 28 min here), not "several GB"; T6 step 6 says
    nothing of what to expect on the drift course with a Model 3; T7 step 3's quoted log
    line is stale (now `soil zone mud_road: cut (...) out of 8 mesh(es)` and `32 tiles of
    1.6 m at z 0.00`).
- **The reruns on the spec's physics (2026-11-16, after G55 at fa539b5).** Every car of
  the ego's model now runs its spec's physics on a project stack, with acb verifying it; the
  results before G55 ran on CARLA's blueprint. Rerun on this host (a76643f to d5c9843):
  - items 1 to 4, `scenarios/projects/basic`: town01 6 of 6, towns 4 of 4 (Town02, 03, 05,
    10), episode_change 1 of 1, tutorial 1 of 1, all with physical_verdict passed;
  - item 8, incidents i1 to i5: 5 of 5, SSv2 and both judges;
  - item 9, i6: 3 of 3, both judges;
  - item 10: i8 passed SSv2 and physical_verdict; i7 ended `error` on both tries (SSv2's
    acceleration bound, the ego at -15.8 and -18.8 m/s^2), each on a contact with `i7_cut`
    (727 N s at peak on the retry): as on the blueprint, so the physics does not decide it.
    133's subagent checks whether the cut is avoidable at all and proposes a re-timing or a
    stated expected outcome; the two runs are on the NAS, "2026-11-16 077 i7 spec physics";
  - item 11, suite tour: passed SSv2, incident 6 of 6, physical_verdict passed, no contact,
    583.5 s wall (033's take_1: 6 of 6, no contact).
  Items 5 and 6 (Isaac) are 241's.
- **Item 11 migrated (2026-11-16, d5c9843).** The demo project's suite `tour` runs
  `town01_tour` with `global_timeout` 1200 on the project's ego, ClearNoon (`world.weather`,
  as 033). `swb.gap incident --incident auto` on a scenario whose name carries no incident
  number judges every incident its storyboard log names, one incident_judge each
  (`<trace>.incident<N>.json`), passing when all pass; the tour declares i3's stop line
  (`csbIncidentStopLine`). `just demo-tour` runs it (133's subagent, 33b3f95); the recording
  stays with `just demo-tour-direct` (G4).
- **Item 7 migrated (swb 118, 2026-11-16, 7c277ee).** `scenarios/projects/two_av`: the ego and
  `bg_av_1` as Autoware pilots on the Model 3, `lifetime: stack`, judges physical_verdict and
  gap. Live on fa539b5: passed, both judges, no ego contact, gap oracle 0/0/0, as 050's
  "after"; both acb_bridges verified the spec's physics. `just two-av` runs it; the old path
  is `just two-av-direct`. NAS: "2026-11-16 077 item 7 two_av".
- **Item 32 migrated (swb 118, 2026-11-16, a9be59a).** `scenarios/projects/c2b`: `command`
  pilots (`pilots/cutin_pilot.py --pair`, the cell read from the scenario's parameters), six
  per-cell scenarios with the start on the straight, judge `c2b_score` against 074's arena
  run (`reference/arena.jsonl`) and physical_verdict; suite 6 cells x 2 pairs x 2. Live: 24
  of 24 runs agree on the outcome, payoffs inside the arena's interval 46 of 48 (the
  reference's own 42 of 48); the two misses are uB at 10/0 with the eq pair, as 074's
  lane-frame score. A developer project: it reaches into `scripts/coop`. NAS: "2026-11-16
  077 item 32 c2b".
- **Item 34 migrated (swb 118, 2026-11-16, 1adab31).** `scenarios/projects/e2`: the two E2
  scenarios, Autoware on clustering at `max_velocity` 10, capture full (/clock,
  kinematic_state, control_cmd) and the gap trace, judge `e2_speed` (lane change and lead
  brake within 1 s of E2's). Two passes on the spec's physics, every run passed: lane change
  4.6 and 4.5 to 4.55 s (E2 4.05 to 4.10, 4.80 to 4.85), lead brake 2.3 to 2.35 and 2.55 to
  2.7 s (E2 2.3, 2.7). Its findings: capture full writes sqlite3 bags, which tools reading
  only mcap do not open (results-and-logs.md now says so); with a `lifetime: stack` pilot a
  run's bag can start while the ego still rolls from the run before, so a judge anchored on
  first motion is fooled (projects-and-pilots.md now says so). NAS: "2026-11-16 077 item 34
  e2".
- **G34, G35, G53 built (133's subagent, 2026-11-16).** G34 (2a6c4a9): `project check`
  refuses an Autoware-controlled entity bound to `kind: none`, naming the fix. G35
  (a76643f): a cached file is hashed only when its stat key changed since
  `<cache>/verified/<repo>/<commit>.json`, in fetch, check, project check and the stack's
  start; `assets check --hash` and `assets fetch --verify` hash all; on NVMe the 25 GB USD
  set's check went from 41.5 s to 1.4 s. G53 (1206644): a scenario's own simulation-time
  limit raises `global_timeout` to (limit + initialize_duration) / rtf + 60 when larger,
  recorded as `ssv2.timeout_raised`; `--timeout` is never raised. **A behaviour change:**
  236 scenarios with a 600 s limit of their own now run to 780 s and 18 with 300 s to 480 s
  under the default 400 s, so runs that ended `error` at 400 s reach their own verdict.
  Suites at 1200 s are not affected.
- **G56 (found 2026-11-16, swb 241, 075 step 8 T6 step 4; a release must-have): Isaac's
  sensors silent on a two-GPU host.** drive_to_goal ended `error` at `INITIALIZING` in 4 of 4
  runs on two stacks: ndt_scan_matcher "No InputSource", the world server "sensors not ready
  after 1200 warm-up frames" (the G37 code ran; cold caches were ruled out). The host
  setting `gpu: 1` gave the world server `CUDA_VISIBLE_DEVICES=1`, while isaac.md's own
  two-GPU line set `--/renderer/activeGpu=0`; Vulkan does not honour CUDA_VISIBLE_DEVICES,
  so the renderer, and with it the RTX lidar, ran on another card than CUDA. With
  `activeGpu=1` the run passed (13:20, 2 min 31 s), the sensors warm after 3 renders. The
  stack now appends `--/renderer/multiGpu/enabled=false --/renderer/activeGpu=<gpu>` to
  `ISAAC_KIT_ARGS` on Isaac whenever the host names a GPU; isaac.md's export is gone, and
  troubleshooting names the symptom.
- **Step 8 (2026-11-16).** `just scenario` (72c7561) and `just e2e` (65e2020) run on the
  project workflow in the project holding the scenario, else `scenarios/projects/dev`; the
  stack-based recipes are `scenario-direct` and `e2e-direct`, and every caller under
  `scripts/` moved to them. `two-av` (7c277ee) and `demo-tour` (33b3f95) run their
  projects. The project is found from the scenario's path as given, links not followed (a
  linked scenario runs in the project holding the link). `just acceptance` stays: it has no
  project form until the acceptance judge (G11) exists, which needs acceptance.py's checks as
  a judge over a run, /diagnostics, the route and GNSS recorded on the judge's `needs`, the
  gap trace's ego pose as ground truth, node deaths from the pilot's log, and Isaac's batch
  scoring.
- **076 step 9's shards ran on the blueprint (241's audit, 2026-11-16).** Shard 2 of 2
  (10:00, 979ef22) logged "carla_physics none: the hero runs its blueprint's own physics",
  and bg_av_1, lead and cut_in were Model 3s without the spec's physics; shard 1 (this
  host, 2026-11-15) predates G55 too. Both run again on G55: shard 2 on 241, shard 1 here.
- **Item 36 migrated (swb 118, 2026-11-16, b9b7c10).** `scenarios/projects/s5`: B is Autoware
  on full CenterPoint, `lifetime: run`, `max_velocity` 8; A is `coop_a`, a `command` pilot
  (oncoming_agent) at 10 m/s; six gaps (5.5 to 9 s), each twice; judge `s5_score` against the
  reference batch. Live: 12 of 12 decisions as the reference, the 7 s split included (one go
  with a contact, which physical_verdict fails as the reference's did, and one yield). Run 5
  once hung CARLA inside `world.tick()` ("no stream available" in carla.log; the world
  server exited 75 as designed); a fresh CARLA did not reproduce it, so it stays the known
  stream hang. NAS: "2026-11-16 077 item 36 s5" (the hang's carla.log under `hang/`).
- **G57 (found 2026-11-16, swb 118, item 36; built b74b76b): the next run's agent was handed
  the last run's goal.** The relay forgets an entity's standing goal at an episode boundary
  (reset, teleported, episode_end). run_suite sent episode_end after each scenario; swb run
  did not, and a `lifetime: run` pilot is stopped with its run, so the goal stood and the next
  run's agent got it about 50 ms after registering, before its teleport. swb run now sends
  episode_end with the run's verdict to every pilot's entity, and the relay drops the
  standing commands on episode_end even with no agent registered. 118 verified it live: two
  runs on one stack, the second agent's first command its teleport.
- **The spawn reset (found 2026-11-16 by 133's subagent in i7; built 83da98a, swb_carla_world
  f669bca, acb 47f443a).** Under the spec's physics a spawn of a car of the ego's blueprint
  zeroed the moving ego's speed for about 0.4 s: CARLA shares the blueprint's tyre object with
  the car applied last, and every physics apply rebuilds the car at rest. Every apply now hands
  that object to a short-lived decoy of the blueprint (role `swb_entity:~tire_decoy`, under the
  town, destroyed before any tick) and restores the car's velocity in the same tick; the
  re-apply after a spawn is a logged backstop. Measured: 0 re-applies in 8 runs, i7 holding
  11.03 m/s at the spawn. The reruns on the spec's physics above ran before it and are not
  references. Known, after the release: a road friction change mid-run still rebuilds each
  car's physics (wheels and engine at 0 rpm).
- **i7 (133's subagent, 2026-11-16).** i7 was written on 2026-10-26, after 62eae3c re-timed
  i1 to i6, and never ran live before item 10: no build of the ego ever avoided `i7_cut`. At
  the 18.3 m bumper gap the time to contact at the pull-out is about 1.8 s, which Autoware
  1.5.0 (-2.5 m/s^2, jerk -1.5) cannot stop in. Re-timed to a 28 m gap on i7's own site, with
  an incident_judge row (no contact, minimum gap at least 2 m): 3 of 3 without contact on 133,
  minimum gap 5.9 to 6.7 m, Autoware braking at 4.2 to 4.4 m/s^2.
- **G58 (found 2026-11-16, swb 118; a release must-have): the tour's verdict was chance.**
  Town01's own level props (actors 130 to 173, `static.prop.mesh` with physics: plastic
  chairs of 0.5 kg, tables of 5 and 648 kg, cones of 24 kg) are knocked about by the NPC
  `g_close_1` in every tour, and the ego sometimes hits the debris (blueprint 2 of 2, the
  spec's physics before the decoy 0 of 2, after it 1 of 2: 53 N s at x 214.75 on the y -195
  straight). G55's tour A/B is withdrawn; G55 stands. Ruling: the CARLA world server turns
  physics off on every `static.prop.mesh` it did not spawn, after each world load and reload,
  a world server setting keeping them live; the judges go on counting every contact. Built by
  133's subagent; each tour (demo, set050_tour) then needs 3 tours without an ego contact.
- **075 step 8 on Isaac Town01 (swb 241, 2026-11-16).** T6 steps 1 to 5 and T7 steps 1 to 5 by
  a fresh reader in a sandboxed HOME, a fresh clone through 036d927, work at origin with the
  fixes that landed during the run (0.5.4, BUILD_TESTING off, xmlschema, G25, `hello --wait`,
  G36, G41, G56: the stack set `--/renderer/activeGpu=1` and the sensors warmed in 3 to 7
  renders, cold cache or warm). G37 on a cold cache: no hang; its "not ready after 1200
  frames" was G56. Page fixes (2637e2c): T7 step 4's ruts are saved at the next world load;
  T6 step 5 says where a model's box goes; troubleshooting has the busy host's alignment.
  G35 on the same spinning disk at 036d927: a fresh project's fetch 5 s and check 1 to 2 s on
  19 GB, a stack start and stop with a run 1 min 49 s (the launch parse alone was 14 min);
  the first pass after G35 hashes once to write its record. pygame and lanelet2's Python
  bindings are missing on that shared host and not needed by T6 or T7. NAS: "2026-11-16 075
  step 8 Isaac Town01".
- **Isaac T2 cells (swb 241, 2026-11-16, 036d927).** All five pass on the plain-user setup
  (`inputs: [ground_truth]`, no `isaac.ego`): autoware, simple, none (an SSv2-driven
  scenario), launch with its own agent, command; each stack stopped itself. 241 commits them
  as `scenarios/projects/isaac_t2`.
- **Step 7, the T3 tooling (2026-11-16).** `scripts/t3.py list|run|record|compare` over
  `scenarios/t3.yaml`; a reference is `scenarios/t3-references/<project>/<suite>.csv`, each
  run's verdict and judges' verdicts, with the commit and host. `--part i/n` gives each host
  whole projects, pinned per project in the manifest (`part: k, parts: n`): 1 swb 118
  (two_av, set050_tour, s5, safety_suite; about 125 min), 2 swb 241 (basic, gap, bench,
  set050, e2; 110), 3 swb 243 (demo, c2b, multi_pilot; 96), with 133 the fallback for 2 and
  3. Dry runs on this host's results compare clean. No reference is recorded before the RC.
- **Release criterion (a) on the Isaac host (swb 241, 2026-11-16).** A fresh clone of
  d5cbd4f in an empty HOME, install.md as written: rustup's cargo 1.100.0; the pip line put in
  colcon-cargo-ros2 0.5.4 with tomli 2.5.0 as its dependency (nothing by hand), xmlschema
  4.3.2 and play_launch 0.35.1; setuptools 59.6.0 (the system's); `rosdep install` not run (no
  sudo). The page's `colcon build ... -DBUILD_TESTING=OFF --cargo-args --release`: 68 packages
  finished, none failed or aborted, 47 min 12 s at a load of 55 to 80; one workspace
  `src/autoware_carla_bridge/.cargo/config.toml`, none per crate. Open for (a): the CARLA
  host's clean build.
- **The reruns on the fixed physics (133's subagent on 133, 2026-11-16, d92e51a: the spawn
  fix, G55, G58, i7's re-time, row 8; both workspaces rebuilt clean; GPU 0 97 to 99 % busy
  from another user's process throughout).** basic town01 6 of 6, physical_verdict on each;
  i1 to i5 5 of 5, both judges; i6 3 of 3 SSv2, r0's incident judge failed on SSv2's
  contact with the cyclist (box gap -0.15 m at 2.8 m/s, the ego yielding; no CARLA contact,
  physical passed), r1 and r2 all passed; cut_in_rear 3 times: i7 3 of 3 and i8 3 of 3,
  both judges (row 8 live); the tour, a first batch 2 of 3 (r0: incident 5 of 6 and physical
  failed, the ego standing about 1 m over i2's crossing, box gap -0.36 m, 0 N s; no prop near)
  and a second 3 of 3, so G58's bar of three clean tours is met; shard 1 of 2 3 of 3. The two
  near misses fit the busy GPU; the references are recorded on a quiet one (above). G58:
  `map_props: frozen|live` (default frozen, swb_carla_world 8b90053, 38b2d06), "Map props:
  froze 115 of 115" on Town01, `provenance.world_server.map_props` in result.json; 56
  samples over three tours, no prop moved more than 1 cm or 0.5 degrees.
- **G59 (found 2026-11-16, swb 118; built here): diagnostics.csv grew without bound.** Each
  nested play_launch of a stack (the simulation side, each pilot) recorded the ego's
  /diagnostics in its own `diagnostics.csv`, 0.75 GB an hour each (1.5 GB in 2 h, 10.4 M
  rows, 4 M of them topic_state_monitor); nothing reads them (the timeline skips them). The
  stack's nested launches, its pilots' and the run's scenario launch now pass
  `--disable-diagnostics` (in play_launch since 0.30, the floor). Open: a stack's own
  `stacks/<id>/proc/` logs are pruned by nothing (`runs prune` and `keep:` cover runs), and
  play_launch itself could cap or deduplicate nested recording (the user's repository).
- **Release criterion (a) on the CARLA host (swb 118, 2026-11-16): met on both hosts.** A
  fresh clone of 63b5233 in an empty HOME, install.md, carla.md and T1 as written: 68 packages,
  none failed, 6 min 39 s (peak RSS 3.0 GB); build/ 11 GB and install/ 1.0 GB, 13 GB with the
  clone; colcon-cargo-ros2 0.5.4, tomli met by Ubuntu's python3-tomli 1.2.2; one generated
  config per Cargo workspace root; T1's drive_to_goal passed in 49.3 s, the stack starting
  CARLA itself. Without sudo `rosdep install` stops on python3-pygame (and lacks lanelet2,
  shapely, networkx): install.md now says what to install for the user and what is optional.
  Seen again at a stack's stop: stack_keeper not out within 5 s and SIGKILLed (known). NAS:
  "2026-11-16 077 criterion a fresh clone".
- **Item 24, Haruna (swb 139, 2026-11-16; deferred).** `courses`, haruna_down with
  drift_mustang under Autoware (`max_velocity` 8) on Isaac: the nvtl judge passed 058 C9 (p05
  3.13, least 3.08, median 3.16 over 6399 samples, 262 m), but both tries ended `error`: the
  ego stopped for good at (82.1, 431.4), 18.8 m before H1, about 80 s in, and SSv2 timed out
  at 780 s (G53's raise). The trajectory's zero-velocity point jumped from 85 m ahead to 31.7 m
  at t = 80.6 s and settled about 1 m behind the car; no planning node logged and every
  planning validator stayed OK. A candidate, not proven: voxel_based_compare_map_filter in
  ERROR all run ("number of voxels overflows int32"; Haruna's map is one 15.8 M-point PCD of
  907 x 1216 x 370 m, untiled, so dynamic loading is off), so map points are never removed
  from the obstacle cloud. Next, outside the release: tile the PCD (`pointcloud_map/` and its
  metadata) or raise the filter's leaf size, and rerun with velocity_factors and
  steering_factors recorded (now in the project's `logging.topics`).
- **Item 52, `just run` (swb 118, 2026-11-16, 2788710).** Kept as the developer path and
  checked on main, both workspaces rebuilt clean, its own web port, world server port and
  CARLA: the world server loaded tesla_model3's generated physics and acb_bridge verified it
  (G55); "Map props: froze 115 of 115" (G58); no diagnostics.csv under play_log/bridge or
  play_log/ego (G59); `just scenario-direct` town01_ego_drive passed; stopping its play_launch
  by process group left nothing in 12 s, every port free. G57 is swb run's (scenario-direct
  sends no episode_end). Isaac's half is 241's (Town01's render crashes on 118's host). NAS:
  "2026-11-16 077 item 52 just run".
- **The G58 follow-up on a quiet GPU (swb 118, e4d9ae9).** set050_tour's tour 3 of 3, no ego
  or prop contact, the GPU at most 5.8 GB and 47 %. `i6_cyclist`'s box overlapped the ego's by
  1.8, 0.3 and 1.2 cm in SSv2's and CARLA's geometry with no CARLA contact (set050_tour has no
  incident judge, so row 6 did not see it): a real margin of Autoware's at i6, not load. Ruled:
  i6 is re-timed for at least 0.5 m, as i7 was, and the near-zero margin is written up as an
  Autoware finding. NAS: "2026-11-16 077 G58 tour reruns".
- **Item 53, the separate ego and background stacks (swb 118, 2026-11-16, 2788710).** `just
  ego-av` and `bg-av`, fresh: ego_ready answered, the G1 environment loaded (the overlay's
  blind_spot, pose_instability and NDT, TensorRT 10.8), ego_drive and two_av passed. G1 stands.
  **G64**: an ego-av reused for a second scenario failed it 3 times of 3 (once a collision
  with bg_av_1 that is a wrong verdict, then PLANNING timeouts; behavior_path_planner "ego pose
  is beyond goal" 70 times after an OK reset): nothing resets a separate ego between
  scenarios. Ruled: for the release `scenario-direct` refuses an ego stack that already served
  a scenario (a marker on the play_launch's pid and start time; 118 builds it), and CLAUDE.md
  says a separate ego-av serves one scenario (5de256d); after it, the reset is diagnosed and
  fixed. G59 extended: these recipes and the other developer launches record no /diagnostics
  either (41e4485). NAS: "2026-11-16 077 item 53 ego-av bg-av".
- **Item 54, `just carla-start` (swb 118, 2026-11-16).** Town01 at Epic offscreen and Town02
  at Low each served RPC in 16 s; `just carla-stop` left no process and no port. The gap
  stands: the project path sets neither the boot map, the render mode nor extra arguments,
  and runs Epic only. NAS: "2026-11-16 077 item 54 carla-start".
- **G60 built (swb 139, 13c9333; reviewed here).** A pilot's own ROS workspaces:
  `overlays: [<setup.bash>, ...]` on `autoware` and `launch` pilots, sourced after this
  workspace's installs and before the TensorRT runtime; entries project-relative or
  `{workspace:<name>}/...` under the host's `workspaces: {<name>: <dir>}` (as G8's
  `datasets:`), also a substitution in `args` and `run`; `project check` refuses an unknown
  workspace and a missing file and warns on a host path; stack.json and provenance record
  each setup file and its workspace's commit. Live: item 25's drift_controller ran from
  autoware-drift's install, its commit recorded.
- **Item 25, the Town05 ring drift lap (swb 139, 76843b3): deferred, needs a quiet GPU.**
  Two runs ended `error`: a first Step of 18.4 s past the bridge's 10 s, then 128 s simulated
  in 820 s wall (0.16x real time, another user's job at 97 % of the GPU), so the scenario never
  left its 300 s initialization. **G61** (after the release): `global_timeout` and G53's raise
  use the configured real-time factor, not the achieved one; a run far below it should warn.
- **G62 built (da06fd0): Autoware's fixes without just.** `ros2 run swb autoware
  overlay|tensorrt|where` runs the workspace's own scripts into the directories an autoware
  pilot reads; install.md's step and the stack's warning name it. Both apply to Autoware 1.5.0.
- **Release criterion (b), tutorials 2 to 5 on CARLA (133's subagent, 2026-11-16, d06ed00).**
  A fresh clone in a sandbox HOME, install.md and carla.md as written: 68 packages, none failed;
  22 runs, none failed or errored, on a GPU 98 % busy from another user. Its findings went into
  the tutorials' editing pass (5e2da37: a formal register, about 1000 prose words each, prose
  1855/1259/1093/1177/935/1647/1101 to 1533/1154/1057/1095/962/1474/1150) and G62; **G63**:
  `carla_poses` ignores the host settings' CARLA port (133's subagent). The docs now state ROS 2
  Humble as the requirement and the simulator and autopilot as choices (5e89b3c); the build
  still compiles against Autoware's interface messages, so an Autoware install remains a
  build requirement (after the release: vendor or split them).
- **`just test` on any host (9b385fc).** The scripts/ tests run without pytest plugins: ROS's
  launch_testing hook imported scripts/test_phase1.py, a live ZMQ client, and failed where
  pyzmq or protobuf differed (swb 139; verified there).
- **G64 (b) built (swb 118, ea0a188; reviewed here).** `scenario-direct` refuses a separate
  ego-av, or a bg-av whose entity the scenario names, that already served a scenario: a
  marker per user under /tmp keyed on the stack's first-started play_launch's pid and start
  time, written as the scenario starts (an interrupted run counts); stale markers go when
  checked; scripts/test_ego_used.py (6 tests). Live: a fresh ego passed and was marked, the
  next scenario-direct and a two-av-direct on it were refused at once, and after a restart
  the next passed. Open, after the release: two-av-direct starts its bg-av before the
  refusal; `just run`'s own ego is not marked and not measured (050's and the coop batches ran
  against it for weeks); (a) the reset itself: the route was set from x 190.8 to a goal at x
  120, both at yaw pi, then "ego pose is beyond goal", so which pose behavior_path_planner held
  after the reset is the question.
- **G65 (found 2026-11-16, swb 139, item 26; a known limit for the release).** On Isaac a
  run's first ego spawn can take longer than SSv2 waits for its vehicle: speed_ladder's rung 10
  (a quiet GPU) ended `error` after SSv2's `change_to_stop` 30 times (about 90 s), while the
  world server spawned the Mustang with zed_left and lidar in 68.2 s ("sensors warm after 4
  render(s)"). The world itself was loaded at the stack's start, and the stack's readiness was
  right; the spawn happens inside SSv2's initialize, so swb cannot wait for it beforehand. The
  preload's warm-up ego never warmed this rig ("sensors not ready after 1200 warm-up
  frames"), and ego_ready passed for the lifetime: run pilot before the run's ego existed.
  A suite's retry on error passes on the warm second spawn; troubleshooting names it. After
  the release: profile the spawn in swb_isaac_world and make the warm-up cover a camera rig.
  The "GNSS pose has not arrived" lines were automatic_pose_initializer's, not the cause.
- **241's walk of T3's and T4's Isaac claims (2026-11-16, 2788710).** carla_basic is refused
  on Isaac and "only one entity can have a pilot" holds, both at `project check`. Its findings:
  - **G66 (built, 0f36b3f):** `project check` passed `simulator_autopilot` on Isaac, and the
    run ended in SSv2's `Spawn entity ... resulted in failure`, the reason only in the
    bridge's log; it is now refused at check, as carla_basic is.
  - **G67 (118; built 331fb94):** `runs why` showed SSv2's SemanticError and an unrelated play_launch WARN
    (`cap_sys_ptrace`) as `first`. Ruling: a refusal or failure line of swb's own processes
    (the bridge's or a world server's `Cannot spawn ...`, `refuses`, `failed`) in the run's
    slices outranks SSv2's generic error, and play_launch's capability warnings (`getcap`,
    `cap_sys_ptrace`) are never `first`; a test from 241's run's slices.
  - **G68 (118; built 331fb94):** a stack refusal raised while play_launch parses swb.launch.xml arrives
    wrapped ("Rust parser error ... Python error ... RuntimeError: swb: the stack is refused
    (3): ...") with play_launch's misleading `--parser python` hint. Ruling: `run
    --start-stack` plans the stack in its own process before starting play_launch and prints a
    refusal plainly (exit 3); swb.launch.py also prints the refusal on its own line to stderr
    before raising, so a reader starting play_launch directly sees it first.
  - **G69 (118; built 331fb94):** the timeline stamps a world server's log lines 28800 s late on a UTC+8 host:
    their timestamps carry no zone and were read as UTC. Ruling: a log line's naive timestamp
    is the host's local time when read; a test under `TZ=Asia/Taipei`.
  - **G70 (a release must-have in part):** the Isaac world server hung at the first step after
    the world reload that follows pedestrian_crossing (2 of 2): its main thread blocked in a
    PhysX step for 6 minutes after "Resolving contacts between two kinematic objects is
    invalid"; the bridge only waited. (ii) **built (118, 8fe78c3: swb_world_protocol e141f27,
    swb_carla_world c50745a, swb_isaac_world 15c1922):** every step runs under a 180 s
    deadline in swb_world_server; past it the world server logs the step, dumps every thread's
    stack and exits 75, and a step that holds the GIL meets faulthandler's backstop 30 s later
    (exit 1); the supervisor respawns it and the run ends `error`. (i) 241 writes the issue and
    takes a time-boxed diagnosis; (iii) without a fix by the RC, T3's Isaac paragraph states
    the limit. pedestrian_crossing's first walker render took 75 s inside frame 2: G65's
    warm-up should cover walkers too.
- **G67 to G69 built (swb 118, 331fb94; reviewed here).** `runs why` names the first refusal
  or failure of swb's own processes ahead of SSv2's generic error and never shows
  play_launch's capability notes (on 241's run: "Cannot spawn 'npc_autopilot'
  (simulator_autopilot): the isaac world server does not offer AUTOPILOT"; the run's slices
  are a test fixture); `run --start-stack` plans the stack in its own process first, so a
  refusal (a refused CARLA, G39, among them) is printed plainly with exit 3 and nothing starts;
  a log stamp without a zone is the host's local time.
- **i6 re-pathed (133's subagent, 2026-11-16, 096df33; the ruling of the quiet-GPU tour).**
  i6's margin was zero by construction, and Autoware's stop comes late. The cyclist rode 1.47 m
  right of the ego lane's centre, so a 0.6 m bicycle passed the 2.16 m Model 3 with 0.10-0.15 m
  to its side (042's 0.13-0.15 m). Autoware 1.5.0 (CenterPoint, the overlay's blind_spot fix)
  does not stop the ego before a cyclist coming up on the inside reaches it: it slows through
  the pass and stops once the cyclist is by, so the stand of 1.2-2.2 s is a yield to a cyclist
  already beside or ahead of it, and under load the stop comes later still. In 133's run
  2026-11-16T15-32-24 r0 the ego was at 3.0 m/s and turning in (yaw -1.61 against the lane's
  -1.57) as the cyclist passed its front, and stopped only 2 m past the turn's entry: box gap
  -0.15 m, an SSv2 contact; 118's quiet-GPU tours overlapped by 0.3-1.8 cm the same way. The
  side clearance is the only margin, and on Town01 the lane caps it: 4 m wide, with
  blind_spot's attention area ending at the lane's right bound (x 332.72) and taking an
  object by its centre. Re-pathed 0.40 m toward the kerb (096df33; timing, speed, trigger and
  row 6 unchanged; i1-i5, i7, i8 regenerate byte-identically), the cyclist passes with
  0.54-0.57 m and the ego still yields: 3 of 3 i6_centerpoint and 3 of 3 tours (6 of 6
  incidents, no contact), on 133 with GPU 0 97-99 % busy from another user. Open: a turn-in as
  late as r0's would still leave about 0.3 m; the cyclist's centre is 9 cm inside the attention
  area (it yielded 12 of 12); the margin on a quiet GPU (118 measures it).
- **G63 built (133's subagent, 14026d8):** carla_poses finds CARLA from the host settings and
  the seat (`seat: <n>`, or `auto`: the one seat whose stack you hold, else an error naming
  `seat take`), the project from the working directory; `--host` and `--port` go before or after
  the subcommand. 11 tests; commands.md and T2 say so.
- **i6 on a quiet GPU (swb 118, 2026-11-16, baeba03 with 096df33, a clean rebuild; NAS "2026-11-16
  077 i6 quiet GPU check").** 6 of 6 passed: i6 alone 0.56, 0.57 and 0.55 m least box gap,
  the tour 0.53, 0.55 and 0.52 m; no contact in CARLA or SSv2, ego_yielded in all six,
  incident_judge and physical_verdict passed, and the tour's other incidents without contact.
  GPU 0 held only 118's processes (at most 5.9 GB, 64 %); the CPU load was 11-159 on 48 cores
  from others' builds. The ego is still moving at the closest point (1.7-3.3 m/s; in the tour
  1.7-2.6 m/s, then a stand of 1.4-1.5 s) and stops while the cyclist is near, so the
  finding's "stops once the cyclist is by" holds; the tour is slower and slightly closer. The
  margin above the 0.5 m bar is 2-7 cm. The bar is this roadmap's target for the scenario, not
  a judge's threshold: row 6 fails on contact only, about 0.5 m away, and stays so. The
  quiet-GPU point of the re-path entry is closed; the late turn-in and the attention area's
  edge stay open as written there.
- **G72 (found 2026-11-16 by swb 118 during 079; numbered by 133, G71 unused; 118 builds it,
  reviewed here).** swb_agent's vehicle_agent never exits on SIGINT: under play_launch it is
  killed after 5 s ("did not exit within 5s, sending SIGKILL"), under `ros2 launch` it dies
  with -11 (3 of 3), and stand-alone it aborts on the way out ("terminate called without an
  active exception"): its step thread keeps calling into a shut-down rclpy context. Ruling: the
  agent stops on the context's shutdown, bounds its close steps, joins its spin threads, and a
  SIGINT test requires exit 0 and no abort. A commit of its own, apart from 079, so the release
  carries it. **Built (634ca72, reviewed here):** rclpy's own signal handlers are off and SIGINT
  and SIGTERM set the agent's stop; the close (agent, executor, spin thread, node, context) runs
  in order on a thread within 3 s in all, each join bounded at 1 s, and a step past the budget is
  named before os._exit; an exception out of the run is logged once, exits 1 and takes the same
  close. Tests run the agent as a real subprocess on SIGINT, SIGTERM and a raising run: exit 0
  (1 for the raise), one traceback at most, no "terminate called". swb_agent 173 passed.
- **G70 (ii) verified live (swb 139, 2026-11-16, 2e14590 with b844a24).** The hang reproduced as
  on 241 (the world reload after pedestrian_crossing at 21:28:51, PhysX's "Resolving contacts
  between two kinematic objects is invalid", step 3 hung). The watchdog child ended it:
  "step 3 (3 x 0.0166667 s at sim time 0.100 s) has run 180.1 s, past its deadline of 180 s:
  ending world server 49536 for a fresh one"; faulthandler put the main thread in
  isaacsim.core.api physics_context.py:582 `_step` under swb_isaac_world world.py:1226. The
  server was SIGKILLed, respawned in 10 s and ready 3.5 min later (Town01 preloaded in 110 s).
  Every run ended, none hung. Recovery was not shown and is inconclusive, a known limit for the
  release: red_light r0, already waiting, ran out SSv2's 300 s initialize; the ego pilot
  (lifetime: stack) restarted with the episode at 21:33:56, before the respawned world server
  was ready (21:35:33), and never became ready ("planning is up but perception is not": no
  lidar publisher on the ego domain), so red_light's retry and drive_to_goal r0 ended `error`.
  World servers #2 and #3 were then SIGKILLed with no watchdog line just after another user's
  vLLM server started on the host (free memory 19-31 GB against Isaac's 17.5 GB): most likely
  the OOM killer, the kernel log being unreadable from that account. Open: rerun the recovery
  half on a host with memory headroom (does the respawned world server's ego lidar reach the
  restarted pilot, and does the run after a hang pass?).
  Until those are fixed, a stack whose Isaac world server was ended by the deadline is
  restarted before further runs. Open: (i) the PhysX hang itself, now 139's root fix (issue
  040: the shared bury point in `_load_env`), with its live repro; with it in, recovery after a
  respawn (the pilot's perception, the silent exit) is post-release and recorded by 139 as a
  finding of its own. (iii), T3's Isaac note on the limit, is drafted and goes in only if the
  root fix misses the RC.
  pedestrian_crossing's first walker render (19.8 s, then a 16.2 s step past the bridge's
  10 s) belongs to G65's walker warm-up.
- **G70 (ii) confirmed again, and recovered (swb 241, 2026-11-16, its NVMe Isaac).** The hang
  reproduced 4 of 4; the watchdog ended the world server at 180.1 s in step 2 after the reload
  (physics_context.py:582 `_step` under world.py:1226), which was respawned and ready in
  1 min 44 s; after two errors, red_light's retry and drive_to_goal passed. Recovery after a
  respawn: recovered on 241, not on 139 (there the pilot restarted before the world server was
  ready, and memory ran short); not systematic, and the rerun with headroom stays open. For
  G65: the walker's first render took 122 s, past the bridge's 120 s Spawn limit, so the first
  pedestrian run on a fresh Isaac stack can end `error`; a retry passes. 241's Isaac stack now
  starts in 2.5 min.
