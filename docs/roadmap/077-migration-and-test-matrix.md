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
   developers only, documented.

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
