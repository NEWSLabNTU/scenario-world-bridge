# Phase 073: One Launch for the Whole Stack, Simulator and Use Case as Slots

A person starts swb (the simulator, the bridge and relay, their autopilot, and optionally an
SSv2 scenario) with one `play_launch` command. swb stays a framework: it names no car, no
scenario and no use case, and the simulator is a slot like the rest. CARLA and Isaac Sim are
officially supported, so swb ships shortcuts for them; any other simulator comes from the
user's own support package.

**Status**: planned, 2026-11-08. User direction: "I'd like to see if we can have a package
to launch the whole SSv2 + simulator stack"; "Note that scenario file is per use case. We
don't save the scenario file in the swb project"; "The swb supports generic simulator. The
simulator is switchable, and we allow user custom simulator via their support package"; "We
can have shortcut launch files or args in swb, since the CARLA and Isaac Sim are officially
supported."

The first user is the golf cart's Isaac twin (NEWSLabNTU/2026-golf-cart, roadmap 10), whose
car already lives in its own repository (efee145: `vehicle_catalogs`, `spec_hash_file`, aib's
relative paths and `aib.gen --spec`).

## Shape

### `swb_launch stack.launch.xml` (generic)

```bash
play_launch launch swb_launch stack.launch.xml \
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

Each includes `stack.launch.xml` with `sim:=isaac` or `sim:=carla`.

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
| S3 | `stack.launch.xml` with the groups, `sim:=` presets and the package hook | |
| S4 | Shortcuts `isaac.launch.xml`, `carla.launch.xml` | |
| S5 | Free-drive driver | SSv2's ZMQ protocol (initialize, spawn ego, update frames); the bridge needs no change |
| S6 | The ego group restarts after a world reload or a simulator restart | today `swb-agent supervise` does this for `just` recipes |
| S7 | Docs: tutorial on the stack; "a simulator of your own" (the package contract) | |
| S8 | Golf cart (its repository): `twin.launch.xml`, its catalog (generated from its spec), spec hash file, `scenarios/` | phase C of its roadmap 10 |

## Open

- Whether `just run` and `just scenario` move onto `stack.launch.xml` (one path) or stay.
- The free drive's time: real time by default, like the stand-alone runner.
