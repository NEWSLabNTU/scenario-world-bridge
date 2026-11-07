# Phase 073: One Launch for the Whole Stack, Simulator and Use Case as Slots

A person starts swb (the simulator, the bridge and relay, their autopilot, and optionally an
SSv2 scenario) with one `play_launch` command. swb stays a framework: it names no car, no
scenario and no use case, and the simulator is a slot like the rest. CARLA and Isaac Sim are
officially supported, so swb ships shortcuts for them; any other simulator comes from the
user's own support package.

**Status**: done, 2026-11-08 (S1-S9). User direction: "I'd like to see if we can have a package
to launch the whole SSv2 + simulator stack"; "Note that scenario file is per use case. We
don't save the scenario file in the swb project"; "The swb supports generic simulator. The
simulator is switchable, and we allow user custom simulator via their support package"; "We
can have shortcut launch files or args in swb, since the CARLA and Isaac Sim are officially
supported."

The first user is the golf cart's Isaac twin (NEWSLabNTU/2026-golf-cart, roadmap 10), whose
car already lives in its own repository (efee145: `vehicle_catalogs`, `spec_hash_file`, aib's
relative paths and `aib.gen --spec`).

## Shape

### `swb_launch stack.launch.py` (generic)

```bash
play_launch launch swb_launch stack.launch.py \
    sim:=isaac|carla|<package> sim_args:="..." \
    ego_launch:=<the user's autopilot launch> ego_launch_args:="..." \
    vehicle_catalogs:=<catalog.xosc> spec_hash_file:=<spec_hash.yaml> \
    [scenario:=<path to .xosc>] [spawn:=x,y,z,yaw]
```

| Group | What | ROS domain |
|---|---|---|
| sim | the world server `sim:=` names | ego |
| simulation side | the bridge (`vehicle_catalogs`) and the relay, simulator-agnostic | scenario |
| ego | `ego_launch`, swb's vehicle agent (`spec_hash_file`); on CARLA also acb, the vehicle bridge | ego |
| run | SSv2 with `scenario:=`, else the free-drive driver at `spawn:=` | scenario |

- **`sim:=`**:
  - `isaac` and `carla` are swb's world servers (`swb_isaac_world`, `swb_carla_world`).
  - Any other value is a package of the user's that installs `launch/world.launch.xml`. That
    launch starts a world server speaking `swb_world.proto` (the Python shell is
    `swb_world_server`; `swb_world_check conformance` checks it).
  - The package also declares whether the ego's vehicle interface and sensors come from the
    world server (as on Isaac) or from a vehicle bridge in the ego group (as acb on CARLA).
- **`scenario:=`** is a path. Scenarios belong to the use case's repository. swb_examples
  stays as starters, not as any use case's scenarios.
- **Switching scenarios**: SSv2 runs one scenario per run. Run the next one against the live
  stack with `play_launch launch swb_launch scenario.launch.xml scenario:=<path>`, without
  reloading the simulator.
- **Free drive (new)**: with no scenario, a small swb driver speaks SSv2's protocol to the
  bridge. It spawns the ego at `spawn:=`, advances time and does nothing else, and the
  autopilot is driven by hand (RViz). So the no-scenario case runs on every simulator
  through the same stack. aib's stand-alone runner (`aib_launch isaac_vehicle.launch.xml`)
  stays as Isaac's lighter path.

### Shortcuts for the supported simulators

The arguments are promoted, so nobody writes `sim_args` for them:

```bash
play_launch launch swb_launch isaac.launch.xml ego:=<ego.yaml> world:=<world.yaml> ego_launch:=... [scenario:=...]
play_launch launch swb_launch carla.launch.xml map:=Town01 vehicle_model:=... ego_launch:=... [scenario:=...]
```

Each includes `stack.launch.py` with `sim:=isaac` or `sim:=carla`.

### A use case's own launch (the golf cart's, outside swb)

`golfcart_twin twin.launch.xml` includes `swb_launch isaac.launch.xml` with its own ego
config, world, `twin_autoware.launch.xml`, catalog and spec hash file. `route:=N` becomes
`spawn:=`; `scenario:=<name>` becomes its `scenarios/<name>.xosc`:

```bash
play_launch launch golfcart_twin twin.launch.xml route:=1
play_launch launch golfcart_twin twin.launch.xml scenario:=route1
```

## Work items

| # | Item | Notes |
|---|---|---|
| S1 | play_launch: a `<group>`'s `<set_env ROS_DOMAIN_ID>` stays in the group | **done 2026-11-08**: play_launch 0.30.0 scopes it as stock `ros2 launch` does (two groups at 21 and 22, the outer process at the shell's 5, both tools) |
| S2 | SSv2 inside a larger launch | **checked 2026-11-08**: `scenario_test_runner` is `on_exit=ShutdownOnce`. An included launch's Shutdown ends the whole launch, in stock `ros2 launch` and in play_launch alike, so the stack would die with the scenario. A nested launch does not: the run group starts the scenario as its own `play_launch launch --disable-web-ui ... scenario.launch.xml` process, which ends alone while the stack lives on (both tools). `exit_on_scenario_end:=true` makes that process required, for scripted runs |
| S3 | `stack.launch.py` with the groups, `sim:=` presets and the package hook | **done 2026-11-08** (Python: it parses `sim_args`/`ego_launch_args` and picks presets). The ego and the vehicle agent run as nested play_launch processes, respawned: an autopilot stack ends its own launch at an episode end (acb_bridge is required), which must not end the stack. Smoke test on 118 (Isaac, the golf cart's ego config and Autoware, no scenario): world server, ego and agent in domain 11, bridge and relay in 19; the bridge took the world server (28 s, the twin preloaded), the agent registered with the relay, the cart's Autoware came up (43 nodes); killing the ego's play_launch respawned it; SIGINT to the stack left nothing behind |
| S4 | Shortcuts `isaac.launch.xml`, `carla.launch.xml` | **done 2026-11-08**. Isaac's run in S3's test; CARLA's composed (world server, bridge, relay) with no CARLA up. A scenario through the stack waits for S8's (the golf cart's) |
| S5 | Free-drive driver | **done 2026-11-08**: `swb_free_drive` (scenario_world_bridge's second binary) speaks SSv2's protocol to the bridge: Initialize on `map_path`, the ego spawned from its catalog entry (`vehicle`, the first of `vehicle_catalogs`) at `spawn`, then UpdateEntityStatus and UpdateFrame per step. Synchronous; paced to the wall clock by default, `realtime:=false` as fast as it goes; respawned while the world server loads. On 118, the golf cart on Isaac (twin.launch.py route:=1): the spec check matched all three (world server, agent, catalog: 40ffc561f31c), the world stepped at 0.58x, and the cart's Autoware drove route 1 by ADAPI to 0.72 m from the goal |
| S6 | The ego group restarts after a world reload or a simulator restart | **done with S3**: the ego and agent are nested play_launch processes with respawn; killing the ego's brought it back (S3's test) |
| S7 | Docs | **done 2026-11-08**: tutorial 7 (`docs/tutorials/07-the-stack.md`): the launches and their arguments, scenario or free drive, synchronous time, your car (`sim_vehicle_spec.gen --spec`, `aib.gen --spec`), your simulator (the package contract: a world server on the protocol, `launch/world.launch.xml`, `sim_args`, `world_endpoint`, `swb_world_check conformance`, where the ego's vehicle side comes from) |
| S9 | `just run` and `just scenario` on `stack.launch.py` (user, 2026-11-08) | **done 2026-11-08**. `just run [sim:=carla|isaac]` is one play_launch of `carla.launch.xml`/`isaac.launch.xml` (web +4, logs `bridge/`, nested `ego/`, `agent/`) in `just ego-av`'s environment (Autoware, fix overlay, cuda_ndt, acb, TensorRT 10.8, Cyclone) and with its defaults for the default ego (hero, physics verify, the release sockets at SSV2_PORT+1/+2, no RViz, no localization seed); the ego is in by default, `ego_launch:=none` leaves it out; a name the shortcut does not declare is refused; the simulation side's `weather:=`, `config_file:=`, `gap_trace_dir:=`, ... go through the new `side_args` (stack.launch.py and both shortcuts). `scripts/carla_up.sh`/`isaac_up.sh` are thin over `scripts/stack_up.bash`: reuse the stack on +4, else start `just run` (setsid) on the scenario's map (`ego_launch:=none` for a scenario without ego, or when a separate `just ego-av` is up on +2: two-av, demo-tour, run_050_set start it so), replace one on another map or without the needed ego, wait for the bridge, relay, `ego_ready` and the agent. `isaac-world`, `ego-av`, `ego-isaac` stay. Live on 118 (CARLA 2010, Epic): `just scenario basic/town01_ego_drive.xosc` with nothing up started the stack (ego ready and its agent registered within ~1 min of the start), the ego drove (agent READY -> DRIVING), PASS; a second `just scenario` reused the stack, PASS; SIGTERM to the stack's process group took the nested ego and agent (their own group) down in ~12 s, no orphans. Isaac not run here (Town01 crashes this host's GPU). The stack's ego ignores `EGO_RMW_IMPLEMENTATION` (the recipe warns) and has no web UI, so `ego_ready` cannot reload a crashed composable through it (`--composable-respawn on-crash` remains) |
| S8 | Golf cart (its repository) | **done 2026-11-08**: `golfcart_twin twin.launch.py` (route:= free drive, scenario:= a scenario), `catalogs/vehicles.xosc` and `isaac/spec_hash.yaml` generated by `sim_vehicle_spec.gen --spec` (e7d8656), `scenarios/route1..4.xosc` from its routes. route1 through the stack with exit_on_scenario_end:=true: PASS (junit failures 0, errors 0), and the stack ended with it. A first try failed only on the scenario's declared maxSpeed 2.5 against the cart's 2.51 m/s; the scenarios declare 3.0 (the sim's motor holds 2.5) |

## Synchronous time

Every way of running steps the simulator synchronously:
- **With a scenario:** SSv2 steps the world frame by frame through the bridge. CARLA is in
  synchronous mode and Isaac's world server is stepped. The pace is
  `global_real_time_factor` (scenario_args).
- **Free drive (S5):** the same lockstep, paced to the wall clock by default, as fast as
  possible with `realtime:=false`.
- **aib's stand-alone runner:** it steps its world itself; `realtime` paces it.

## Decisions

- `just run` and `just scenario` move onto `stack.launch.py` (S9; user, 2026-11-08).
- Free drive runs at real time by default, and stays synchronous (user, 2026-11-08: "Ensure
  we have ways to run in synchronous mode").

## play_launch against stock ros2 launch

- `RegisterEventHandler(OnProcessExit(target_action, on_exit=[EmitEvent(Shutdown())]))` is
  ignored by play_launch 0.30.0: stock `ros2 launch` ends 2 s into the repro, play_launch
  runs on. Filed with NEWSLabNTU/play_launch and fixed there (user rule). The stack does not
  depend on it: `exit_on_scenario_end` puts `on_exit=Shutdown()` on the scenario's process,
  which both honour.
