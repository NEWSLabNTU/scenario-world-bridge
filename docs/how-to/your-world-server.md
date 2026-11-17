# Your world server in a testbench

> Draft. `testbench check --live` and the stack's start of your server
> land with `scensim run`'s testbench support; until then, run the conformance check by hand
> (below).

Run your testbench on a world server of your own: a fork of CARLA's with a sensor it lacks, a
simulator scensim does not ship, or a fake world for CI. The server lives in a package of your
workspace, beside the testbench, and the testbench names it:

```yaml
# src/my_testbench/testbench.yaml
simulator: carla
world:
  server: [my_world, my_world_server, --quality, low]   # <package>, <executable>, its arguments
```

```xml
<!-- src/my_testbench/package.xml -->
<exec_depend>my_world</exec_depend>
```

scensim starts `ros2 run my_world my_world_server --quality low --port <port>` in place of
the stock server, in the ROS domain of the ego (`EGO_ROS_DOMAIN_ID`, default 1, so its own
ROS nodes, if any, meet the vehicle side there), restarts it if it ends, and drives it over the
world protocol on `<port>`, as it drives CARLA's and Isaac's. The other `world:` keys
(`config_file`, `world_config_file`) configure the stock CARLA server and are refused beside
`server`: give your server its settings as its arguments.

## Another simulator: `simulator: none`

With `simulator: carla` scensim still starts CARLA, for a server that drives it (a fork of
CARLA's). For a simulator scensim does not ship, say `none`: scensim starts no simulator, and
your server starts and owns its own.

```yaml
simulator: none
world:
  server: [my_world, my_world_server]
pilots:
  ego: {kind: launch, launch: my_vehicle drive.launch.xml}   # your vehicle side and autopilot
```

`none` needs `world.server`. A pilot of kind `autoware` or `carla_basic` drives through
CARLA's or Isaac's vehicle side, which `none` does not start, so it is refused: use `launch`
or `command` with a vehicle side of your own, or `none`.

## What the server must do

- **Take `--port <port>`** (scensim appends it after your arguments; it is also in
  `SCENSIM_WORLD_PORT`) and bind the world protocol's socket on `127.0.0.1:<port>` at once,
  before any long start. Do not put `--port` in `server:` yourself: the check refuses it.
- **Answer `Hello` while it starts**, ready or not, so scensim sees it come up. The protocol's
  shells do this for you.
- **End with its parent** and on SIGINT or SIGTERM: `lifetime.die_with_parent()` in Python.
- **Pass the conformance check** (below).

[Your simulator](your-simulator.md) shows how to write one: subclass
`scensim_world_server.world.World` in Python, or implement `scensim_core`'s `World` in Rust.

## A minimal one

An `ament_python` package whose console script serves the protocol shell's smallest world.
Its `package.xml` declares the shell and what it imports, so `rosdep install` brings them:

```xml
<!-- my_world/package.xml -->
<?xml version="1.0"?>
<package format="3">
  <name>my_world</name>
  <version>0.1.0</version>
  <description>My world server</description>
  <maintainer email="me@example.com">me</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>scensim_world_server</exec_depend>
  <exec_depend>python3-zmq</exec_depend>
  <exec_depend>python3-protobuf</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`ament_python` also wants an empty marker file, `resource/my_world`, which `setup.py`
installs (below).

```python
# my_world/my_world/server.py
import argparse

from scensim_world_server import lifetime
from scensim_world_server.server import bind, run
from scensim_world_server.world import World


class MyWorld(World):
    simulator_name = "my_world"
    capabilities = ()          # core only
    max_physics_dt = 0.0

    def __init__(self):
        self.poses = {}

    def load(self, world, ground_paths, signals, request=None):
        self.poses.clear()

    def spawn(self, handle, name, kind, subtype, center, dims, pose, model="",
              motion="KINEMATIC"):
        self.poses[handle] = pose

    def despawn(self, handle):
        del self.poses[handle]

    def step(self, drives, substeps, dt, render, sim_time):
        for handle, drive in drives.items():
            if drive.pose is not None:
                self.poses[handle] = drive.pose
        return [(h, p, None) for h, p in self.poses.items()], []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, required=True)
    args = ap.parse_args()
    lifetime.die_with_parent()
    run(MyWorld(), bind(args.port))
```

```python
# my_world/setup.py
from setuptools import setup

setup(name="my_world", version="0.1.0", packages=["my_world"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/my_world"]),
                  ("share/my_world", ["package.xml"])],
      entry_points={"console_scripts": ["my_world_server = my_world.server:main"]})
```

```ini
# my_world/setup.cfg: without it `ros2 run` finds no executable
[develop]
script_dir=$base/lib/my_world
[install]
install_scripts=$base/lib/my_world
```

## Check it

```bash
colcon build --packages-up-to my_testbench
source install/setup.bash
ros2 run my_world my_world_server --port 5599 &
ros2 run scensim_world_check scensim_world_check conformance tcp://127.0.0.1:5599
#   ... 20 passed, 0 failed
kill %1          # the server: stop it when the check is done
```

`scensim testbench check my_testbench --live` will do the same: start your server as the
stack would, wait for its `Hello`, run the conformance check, stop it, and report each failed
check as a problem of `world.server`.

The build checks `world.server`'s form only (a package, an executable, no `--port`); that
the package exists and its server conforms is checked live.
