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

1. [ ] Inventory (077-study/inventory.md); this doc refined from it.
2. [ ] T0/T1 in `just test`: project check over every project in the repository, swb-agent
   check over every SDK agent, launch parse of every pilot kind and of swb.launch.xml.
3. [ ] The gaps the inventory names, built in the project workflow (each before the items it
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
  overrides, harness mode) next; G8, G13 and G20 (asset sources, CARLA map packages, CARLA
  0.10) need decisions.
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
