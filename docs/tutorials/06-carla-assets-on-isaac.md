# Tutorial 6: The same project on Isaac Sim

**Goal**: Autoware drives the Tutorial 2 scenarios on Isaac Sim, in CARLA's Town01 loaded
from CARLA's own assets in USD (Universal Scene Description, Isaac Sim's scene format),
with a scenario's cars and props rendered in CARLA's models; then on another world, the
Mount Haruna mountain road.

<p align="center">
  <img src="../media/isaac_town02.webp" alt="An SSv2 scenario on Isaac Sim: Autoware turns left on CARLA's Town02 and drives to its goal, with RViz inset" width="640">
</p>

## You will learn

- that a scenario names its map, not its simulator, so that the same `.xosc` runs on
  CARLA and on Isaac Sim;
- what CARLA's asset set in USD contains, and how a project takes only the part it needs;
- how a map's Isaac world file names its stage, and how to make one with `make_world`;
- rendering scenario entities in CARLA's models with `model3d`;
- adding a world from another dataset to a project.

## Before you start

- [Getting started on Isaac Sim](../getting-started/isaac.md): Isaac Sim installed (6.1
  is the tested version) and `isaac.root` in `~/.config/swb/host.yaml`.
- [Tutorial 1](01-first-drive.md) and [Tutorial 2](02-write-a-scenario.md), on CARLA:
  everything except the simulator works the same way here.
- An NVIDIA RTX GPU with 24 GB and 64 GB of RAM. The world server holds about 17.5 GB of
  RAM with Town01 and Autoware another 10 to 20 GB, so no CARLA stack should run beside
  it.
- About 19 GB of disk in swb's cache (`cache:` of the host settings), on a fast disk: a
  town's first load reads 1 to 2 GB of small files.
- Several hours the first time: the USD download took 2 h 28 min on one host
  (2026-11-16), and a town's first load compiles its materials (up to a quarter of an
  hour).

## 1. Copy the Isaac example

```bash
ros2 run swb project new ~/isaac_first_drive \
    --from pkg://swb_examples/projects/isaac_first_drive
cd ~/isaac_first_drive
cat swb.yaml
```

**What you see**: the project file of Tutorial 1 with `simulator: isaac` and a second
dataset:

```yaml
simulator: isaac
assets:
  - dataset: carla-town-autoware-maps
    take: [Town01]               # the Autoware map and Town01's Isaac world file
  - dataset: carla-0.9.16-isaac-usd
    take: [Town01, vehicles]     # the town and the cars in USD: 19 GB of the 25.5
pilots:
  ego: {kind: autoware, vehicle: tesla_model3}
```

`carla-0.9.16-isaac-usd` is CARLA 0.9.16's content exported to USD: six towns and
CARLA's vehicles, props and walkers, under CC BY 4.0 (any use must credit CARLA). `take:`
fetches only Town01 and `vehicles/`, which holds the vehicles, props and walkers.

**Check**: the scenarios are Tutorial 2's, unchanged:

```bash
grep LogicFile scenarios/*.xosc
```

Both name `$(env SWB_MAPS)/Town01`.

## 2. Fetch

```bash
ros2 run swb assets fetch
```

**What you see**: one line per dataset with its commit and file count, the links, and the
lock. Running the command again resumes an interrupted download, since files already in
the cache are kept. Hugging Face limits anonymous downloads (`HTTP Error 429`, then a wait and a
retry); `hf auth login` with a free account's token raises the limit.

```
linked assets/carla-usd -> .../datasets/jerry73204/carla-0.9.16-isaac-usd/<commit>
linked assets/maps/Town01 -> .../jerry73204/carla-town-autoware-maps/<commit>/Town01
```

The map's Isaac world file names its **stage**, the USD scene to load, through a variable
rather than a fixed path:

```bash
grep '^usd:' assets/maps/Town01/isaac/world.yaml
```

```
usd: ${SWB_CARLA_USD}/Town01/Town01_physics.usda
```

The stack sets `SWB_CARLA_USD` to the project's `assets/carla-usd`, so the world file
works in any project, on any host. `Town01_physics.usda` is a layer over CARLA's export
that gives the road meshes their colliders and surface labels.

**Check**: `ros2 run swb assets check` prints `assets: ok`, and `ros2 run swb project
check` prints `project: ok`. The first `assets check` hashes the whole USD set, which takes
from a few minutes to a quarter of an hour; later checks hash only files that changed.

## 3. Bring up the stack

```bash
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml
```

On Isaac the world server is part of the stack and loads its world at start: the first
map that the project's scenarios name, Town01, which it keeps for the stack's whole life.
Autoware starts after it.

**What you see**: the stack's lines as in Tutorial 1, then, after the world has loaded
(minutes; up to a quarter of an hour the first time), `swb stack: pilot ego ready on
Town01` and `swb stack ...: ready`.

**Check**, from a second terminal; the probe changes nothing:

```bash
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5591 --wait 1800
```

prints the simulator (`isaac`), what it offers (signals, contacts, road conditions, ...)
and that it is ready. `5591` is seat 0's world server port (`ros2 run swb seat show`).

## 4. Run the same scenarios

```bash
ros2 run swb run scenarios/drive_to_goal.xosc
ros2 run swb run scenarios/follow_to_goal.xosc
```

The ego drives west as on CARLA, in the second scenario behind a slower car.

**What you see**: two `passed` lines.

**Check**: `grep '"simulator"' runs/*follow_to_goal*/result.json` shows `isaac`: the
same file, the other simulator.

## 5. CARLA's models in a scenario

A scenario names a CARLA model in an entity's `model3d` attribute, by its CARLA blueprint
id. On Isaac the world server dresses the entity in that model from the asset set; on
CARLA the same id spawns the same blueprint. In a copy of `follow_to_goal.xosc`,
`scenarios/follow_the_van.xosc`, give the lead car a model:

```xml
<Vehicle name="lead" vehicleCategory="car" model3d="vehicle.mercedes.sprinter">
```

Then set its `BoundingBox` to the model's size, `Dimensions` and `Center`, from the
vehicle's row of the list below, because the body for physics, contacts and the lidar
remains the box. The list gives the ids the set holds, with each vehicle's size:

```bash
less assets/carla-usd/vehicles/vehicles.md
```

| Entity | `model3d` | Without it |
|---|---|---|
| `Vehicle` | `vehicle.<make>.<model>` | the four-wheeled car closest to its bounding box; a `bicycle` or `motorbike` gets the two-wheeler closest in length |
| `Pedestrian` | `walker.pedestrian.0001` to `0052` | a walker picked from the entity's name |
| `MiscObject` | `static.prop.<name>`, or a `vehicle.*` id | `static.prop.streetbarrier` |

An id that the set does not hold yields a plain box and a warning in the world server's
log (`no CARLA body for ...`). The ego is the exception: its body is always its own
car's.

```bash
ros2 run swb run scenarios/follow_the_van.xosc
```

**What you see**: in Isaac's window (or a recording) a Sprinter van leads; the run passes.

**Check**: `ros2 run swb runs why follow_the_van` shows no `no CARLA body` warning.

## 6. Another world: Mount Haruna

Worlds need not be CARLA towns. The `haruna-tsukuba-courses` dataset (ODbL and GSI data,
see its notice) holds two courses built from open map and elevation data: the mountain
road on Mount Haruna, Japan, and Tsukuba Circuit. Each is a map directory with its
Autoware map, Isaac world and a scenario. Add it to `swb.yaml`:

```yaml
  - dataset: haruna-tsukuba-courses
    take: [haruna]
```

The course's guardrails and trees are CARLA props from `assets/carla-usd`, which the
project already has. Fetch the dataset and copy the course's scenario into the project:

```bash
ros2 run swb assets fetch
cp assets/maps/haruna/scenarios/haruna_down.xosc scenarios/
chmod u+w scenarios/haruna_down.xosc
```

Its ego is the catalog car `drift_mustang`: set its `entryName` to `tesla_model3`, the
car of the project's pilot (otherwise `project check` names the mismatch and exits 1).

An Isaac stack serves one world for its whole life, so it is stopped (Ctrl-C) and started
again on the course:

```bash
play_launch launch --web-addr 127.0.0.1:8080 \
    swb_launch swb.launch.xml \
    map:=haruna
```

**What you see**: the stack's lines with `pilots on haruna`, and in time `pilot ego ready
on haruna`.

**Check**:

```bash
ros2 run swb run scenarios/haruna_down.xosc
```

ends with a verdict line; a run on Town01 against this stack is refused, naming the
`map:=` to restart with. The course is long (its scenario allows 600 s of simulation
time), and it was made for a drift controller: how far the Model 3 under Autoware gets
down it has not been measured.

## What happened

A scenario names its map (`$(env SWB_MAPS)/<map>`), never its simulator, so the project
differed only by one line (`simulator: isaac`) and one dataset. The map directory is the same
Autoware map as on CARLA plus `isaac/world.yaml`, which names the world's USD stage. On
Isaac the ego is a PhysX car inside the world server, built from the same vehicle spec as
CARLA's, so there is no separate vehicle bridge, and the world server carries one ego.
[Isaac assets](../design/isaac-assets.md) describes how the asset set is made.

## Exercises

1. **Easy.** Add a pedestrian in a CARLA model to `follow_the_van.xosc`, standing on the
   pavement (`walker.pedestrian.0012`). Check: the run passes, and the world server's log
   has no `no CARLA body` line for it.
2. **Medium.** Drive Town02: take `Town02` from both datasets, copy
   `drive_to_goal.xosc` to `scenarios/town02_drive.xosc` with the `LogicFile` on Town02 and
   poses from Town02 (on CARLA, `carla_poses`), and restart the stack with `map:=Town02`.
   Check: the run on Town02 passes.
3. **Open.** An Isaac world for a new map: copy `assets/maps/Town01` (with
   `cp -rL`) to `maps/my_town01`, add it to `swb.yaml` as
   `{path: maps/my_town01, as: my_town01}`, remove its `isaac/`, then make its world file
   from the asset set, in Isaac's Python:

   ```bash
   SWB_CARLA_USD=$PWD/assets/carla-usd \
       ros2 run swb_isaac_world make_world maps/my_town01 --town Town01
   ```

   Check: `grep '^usd:' maps/my_town01/isaac/world.yaml` prints
   `usd: ${SWB_CARLA_USD}/Town01/Town01_physics.usda`, and a stack with `map:=my_town01`
   becomes ready.

## If it does not work

| Symptom | Fix |
|---|---|
| The world server never says ready | the first load compiles materials: wait, and watch its log in the stack's web UI; `hello --wait` shows whether it answers |
| The world server dies or hangs out of memory | it needs the GPU and about 17.5 GB of RAM to itself: close other GPU work; on two GPUs pick one with the host setting `gpu:` |
| `refused: the scenario is on map ..., the stack's Isaac world on ...` | the Isaac stack serves one world: restart it with the `map:=` the message names |

More: [Troubleshooting, Isaac Sim](../reference/troubleshooting.md#isaac-sim).

## Next

[Tutorial 7: road physics](07-road-physics.md): mud, gravel and a wet road under the
wheels.
