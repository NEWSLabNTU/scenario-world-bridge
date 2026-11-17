# Your simulator

Make another simulator a backend: every simulator is a **world server** behind one
protocol, `scensim_world.proto` in `scensim_world_protocol`, and the bridge needs nothing new for
one (for developers, the [design](../design/simulator-interface-split.md)). CARLA's world server
(`scensim_carla_world`, Rust) and Isaac Sim's (`scensim_isaac_world`, Python) are the worked
examples.

## A world server

A world server is one process that owns the simulator and answers the protocol's
`Request` with its `Reply` on a ZMQ REP socket:

- **Python**: subclass `scensim_world_server.world.World` (`load`, `spawn`, `despawn`, `step`;
  `set_signals` and the rest with a capability), set `simulator_name`, `capabilities` and
  `max_physics_dt`, and serve it with `scensim_world_server.server.bind` and `run`
  (`scensim_isaac_world/scensim_isaac_world/world.py`).
- **Rust**: implement `swb_core::world_server::World` and serve it with
  `world_server::bind` and `serve`, as `scensim_carla_world` does.
- **Anything else**: the `.proto` and a ZMQ REP socket.

Declare only what the simulator does (`Capability`): the bridge never sends a request the
server did not declare, and answers SSv2 with a failure naming the missing capability. A
map tells the server which world it means in `<map dir>/<simulator name>/world.yaml`
(`world`, `ground_paths`, `map_origin`).

## Check it

With the server running and no bridge attached (the check takes the server over):

```bash
ros2 run scensim_world_check scensim_world_check conformance \
    tcp://127.0.0.1:<port> \
    --map <map dir>
```

It loads the map, spawns and moves bodies, and, for a server declaring `SIGNALS`, sets one
lamp of the map's `traffic_lights.yaml` and reads it back. `scensim_world_check hello <url>`
asks a running server what it is without changing anything.

## Run it in a stack

A ROS package with `launch/world.launch.xml`, which starts the world server on the
`world_endpoint` it is given, is a simulator to the lower-level stack launch:

```bash
ros2 launch scensim_launch stack.launch.py \
    sim:=my_simulator
```

It starts no autopilot of its own. Who drives each entity is `pilots:=`, the same mapping as
a project's `pilots:` in YAML or JSON, e.g. `pilots:='{ego: {kind: launch, launch: my_pkg
drive.launch.xml}}'` ([Project file](../reference/project-file.md)).

The project's stack, `scensim.launch.xml`, starts `carla` and `isaac` only
([Supported and not supported](../reference/limits.md#projects)), so a third simulator
runs without projects, seats and `scensim run`: its scenarios go to `scenario.launch.xml`
directly. The vehicle under test needs a vehicle side of its own: the car's sensors and
the vehicle interface CARLA's and Isaac's offer
([Your autopilot, the vehicle interface](your-autopilot.md#the-vehicle-interface)),
`/clock` included.
