# Tutorial 7: The whole stack in one launch, with your car, scenarios and simulator

One `play_launch` command starts everything a run needs:
- a simulator's world server;
- the bridge and the agent relay;
- your autopilot with swb's vehicle agent;
- an SSv2 scenario, or free drive without one.

swb names no car, scenario or use case here: each is a slot you fill from your own
repository. The design is [roadmap 073](../roadmap/073-stack-launch.md). `just run` and
`just scenario` start this same launch ([Tutorial 1](01-first-run.md)).

## 1. The launches

```bash
play_launch launch swb_launch isaac.launch.xml [isaac arguments] [common arguments]
play_launch launch swb_launch carla.launch.xml [carla arguments] [common arguments]
play_launch launch swb_launch stack.launch.py sim:=<package> sim_args:="..." world_endpoint:=tcp://... [common arguments]
```

`isaac.launch.xml` and `carla.launch.xml` are the shortcuts for the two supported
simulators. Each fills `stack.launch.py`'s `sim` and `sim_args` from arguments of its own.

| Isaac (`isaac.launch.xml`) | |
|---|---|
| `ego` | the world server's ego: `default` (aib's Tesla), `none`, or an aib ego config of yours |
| `world` | worlds to preload, comma separated (a map's `isaac/world.yaml`, a map directory, a USD) |
| `render`, `follow`, `physics_hz` | the window (`auto`, `window`, `headless`), a chase view without the editor UI, the physics rate |

| CARLA (`carla.launch.xml`) | |
|---|---|
| `vehicle_physics` | the spec's `carla_physics.yaml` on the ego, or `none` |
| `npc_contact_sensors` | contact sensors on NPCs |

CARLA itself runs apart and long-lived (`just carla-start`). Its host and port come from
`CARLA_HOST` and `CARLA_PORT`.

| Common | |
|---|---|
| `map_path` | the default autopilot's map directory, and free drive's |
| `ego_launch` | your autopilot's launch: a path, or `'<package> <file>'`. Empty: the simulator's Autoware profile. `none`: no ego |
| `ego_launch_args` | `'a:=1 b:=2'` for it |
| `agent`, `vehicle_model`, `spec_hash_file` | swb's vehicle agent beside it (`agent:=false` if your launch has its own), and how it names the car's spec |
| `vehicle_catalogs` | your car's SSv2 catalog, for the bridge's spec check |
| `scenario`, `scenario_args`, `exit_on_scenario_end` | an `.xosc` to run (a path), arguments for SSv2's `scenario.launch.xml`, and whether everything ends with it |
| `spawn`, `vehicle`, `realtime` | free drive: the ego at `x,y,z,yaw` in the map frame, its catalog entry, and whether to pace it to the wall clock |

Host settings stay in the environment, as everywhere in swb:
- the domains: `EGO_ROS_DOMAIN_ID` for the simulator and the ego, `SCENARIO_ROS_DOMAIN_ID`
  for the bridge, the relay and SSv2;
- the ports: `SSV2_PORT`, `AGENT_PORT`, `ISAAC_PORT`, `WORLD_PORT`;
- `SWB_LOG_ROOT`, and the DDS settings.

The ego, the vehicle agent and the scenario run as their own play_launch processes inside the
stack, with logs under `$SWB_LOG_ROOT/{ego,agent,scenario}`. An autopilot stack ends with its
episode (a world load, a simulator restart), and SSv2 ends with its scenario. Neither takes
the stack down: the ego and the agent are started again.

## 2. A scenario, or free drive

- **A scenario:** `scenario:=<path to .xosc>`. SSv2 places the ego, routes it, and judges it.
  The stack stays up afterwards. Run the next scenario against it from another terminal:
  ```bash
  ROS_DOMAIN_ID=$SCENARIO_ROS_DOMAIN_ID play_launch launch --web-addr 127.0.0.1:8081 \
      swb_launch scenario.launch.xml scenario:=<path> port:=$SSV2_PORT
  ```
  Or end everything with it: `exit_on_scenario_end:=true` (scripted runs).
- **Free drive:** `spawn:=x,y,z,yaw map_path:=<map>`. `swb_free_drive` takes SSv2's place: it
  initializes the world on the map, spawns the ego from its catalog entry, and steps time.
  You drive the autopilot by hand: a goal and engage in RViz, or Autoware's API.

Time is synchronous in both: the world advances one step per frame, CARLA in synchronous mode
and Isaac's world server stepped. A scenario's pace is SSv2's (`global_real_time_factor` in
`scenario_args`). Free drive is paced to the wall clock, and `realtime:=false` runs as fast
as the stack keeps up.

Scenarios belong to your use case and live in your repository; swb ships starters only
(`swb_examples`).

## 3. Your car

A car kept in your repository, beside your Autoware, is plugged in by files you generate from
its spec with swb's tools ([vehicle side](../design/vehicle-side.md#a-vehicle-from-another-repository)):

```bash
python3 -m sim_vehicle_spec.gen --spec vehicle/<car>.yaml \
    --catalog catalogs/vehicles.xosc --spec-hash isaac/spec_hash.yaml     # the SSv2 catalog, the spec_hash
python3 -m aib.gen --spec vehicle/<car>.yaml -o isaac/<car>.rig.yaml     # Isaac's rig
```

Then, with an aib ego config `isaac/ego.yaml` (`rig:`, `body:`), run:

```bash
play_launch launch swb_launch isaac.launch.xml \
    ego:=isaac/ego.yaml world:=<map>/isaac/world.yaml map_path:=<map> \
    ego_launch:='<your package> <your autopilot launch>' \
    vehicle_catalogs:=catalogs/vehicles.xosc spec_hash_file:=isaac/spec_hash.yaml \
    vehicle:=<car> scenario:=scenarios/<one>.xosc
```

The bridge refuses an ego whose world server, vehicle agent and catalog name different specs.
A use case wraps this in a launch file of its own. The golf cart's, in NEWSLabNTU/2026-golf-cart,
is `golfcart_twin twin.launch.py route:=1` for free drive and `scenario:=route1` for a scenario.

## 4. Your simulator

A simulator swb does not ship comes from a package of yours. It provides:

1. **A world server** speaking swb's world protocol (`swb_world.proto`). Write it on the
   Python shell (`swb_world_server`) or the Rust one (`swb_core::world_server`).
   [world-server.md](../../src/swb_world_protocol/docs/world-server.md) has a complete Python
   example. Check it with:
   ```bash
   swb_world_check conformance tcp://127.0.0.1:<port> --map <map dir>
   ```
2. **`launch/world.launch.xml`**, installed in your package's share, starting that server. It
   takes whatever arguments your server needs; the user passes them as `sim_args:="a:=1 b:=2"`.
3. **The ego's vehicle side.** Where the ego's sensors and vehicle interface (Autoware's
   `/vehicle/status/*` and `/control/command/*`) come from:
   - from the world server itself, on `EGO_ROS_DOMAIN_ID`, as Isaac's does;
   - or from a vehicle bridge your autopilot launch starts, as acb does on CARLA.

   Your package's docs say which, and the user's `ego_launch` follows.

Then:

```bash
play_launch launch swb_launch stack.launch.py sim:=<your package> \
    world_endpoint:=tcp://127.0.0.1:<port> sim_args:="..." \
    ego_launch:=<autopilot launch> map_path:=<map> scenario:=<.xosc>
```

The bridge, the relay, the agent, SSv2 and free drive work unchanged: they speak only the
protocol.
