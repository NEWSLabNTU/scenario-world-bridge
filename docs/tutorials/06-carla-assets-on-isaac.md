# Tutorial 6: CARLA's assets on Isaac Sim

On Isaac Sim the stack renders CARLA's own content: its towns, its vehicles, its props and
its walkers, exported from CARLA 0.9.16's editor to USD. This tutorial gets that set,
checks it, turns a town into an Isaac world beside its Autoware map, runs the stack on it,
and dresses a scenario's cars, pedestrians and obstacles in CARLA's models. It ends with two
other worlds: a light Town01 for smaller GPUs, and the Mount Haruna mountain road.

It assumes Isaac Sim is installed and the workspace built
([Getting started on Isaac Sim](../getting-started/isaac.md), stage 1), and that every
terminal sources the environment file, `~/swb.bash`.

Commands use `play_launch launch`; `ros2 launch` takes the same arguments, but play_launch
must still be installed, because the stack starts its ego and scenario as play_launch
processes of their own.

## 1. What the set is

One directory, about 25 GB, CARLA 0.9.16's content as SimReady USD:

| Part | Where | Count |
|---|---|---|
| Towns | `Town01`, `Town02`, `Town03`, `Town05`, `Town10HD`, `Town07_Opt` | 6 |
| Vehicles | `vehicles/vehicle.<make>.<model>/`, indexed by `vehicles/vehicles.json` | 41 |
| Props | `vehicles/props/static.prop.<name>/`, indexed by `props.json` | 97 |
| Walkers | `vehicles/walkers/<id>/`, indexed by `walkers.json` | 52 |

Each town holds its USD stage and a physics layer over it, `<Town>_physics.usda`, which
gives the road meshes their colliders and surfaces: that layer is the file the world
server loads. Every path inside the set is relative, so it works from any directory.

Two environment variables name it, and nothing else does:

| Variable | Points at |
|---|---|
| `CARLA_USD` | the set's root (the towns) |
| `SWB_VEHICLE_USD` | its `vehicles/` directory (vehicles, props, walkers) |

Without `SWB_VEHICLE_USD` the world server still runs, but every vehicle, pedestrian and
obstacle is a plain box.

## 2. Download it

The set is published on Hugging Face as
[carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd)
(CC BY 4.0: credit CARLA when you use it). Install Hugging Face's `hf` command, with either
of:

```bash
curl -LsSf https://hf.co/cli/install.sh | bash
pip install -U huggingface_hub
```

Then download it to a fast disk (a town's first load reads 1 to 2 GB of small files):

```bash
hf download jerry73204/carla-0.9.16-isaac-usd \
    --repo-type dataset \
    --local-dir ~/carla-usd
```

The download resumes where it stopped if you run the same command again. The set is about
21 000 files, and Hugging Face rate-limits anonymous downloads (`HTTP Error 429`, after
which `hf` waits and retries by itself); `hf auth login` with a free account's token
raises the limit. To fetch only part of it, add an `--include` pattern, for example
`--include "Town01/*"`.

Name it in `~/swb.bash`, then source the file again:

```bash
export CARLA_USD=~/carla-usd
export SWB_VEHICLE_USD=$CARLA_USD/vehicles
```

**Building it yourself** instead takes a CARLA 0.9.16 editor build from source with
NVIDIA's SimReady exporter (Unreal Engine 4.26 and CARLA, about 150 GB and several hours
to build), then a few hours of export. The scripts are in `src/swb_isaac_world/scripts/`:
`isaac_town.sh <Town> export:=true` per town, `isaac_vehicles.sh` for the vehicles, then
the same with `props:=true`, `walkers:=true` and `riders:=true`, and
`dump_vehicle_physics.py` against a running CARLA for the vehicles' physics.
[Isaac assets](../design/isaac-assets.md#making-a-set) has the details.

## 3. Check the copy

The set carries `MANIFEST.sha256`, every file's checksum in `sha256sum`'s format. Check
the copy against it:

```bash
cd $CARLA_USD
sha256sum --check --quiet MANIFEST.sha256
echo $?
# 0
```

It prints only the files that are missing or differ, and exits 0 when there are none. The
manifest covers every town, `Town07_Opt` included; the dataset's own `README.md` and
`NOTICE.md` are not in it.

The workspace has the same check, which also lists files the manifest does not name:

```bash
cd ~/scenario-world-bridge/src/swb_isaac_world
python3 -m swb_isaac_world.asset_set check $CARLA_USD
```

On a Hugging Face copy it lists `README.md`, `NOTICE.md` and the download's own `.cache/`
as not in the manifest, which is expected. A `missing:` or `changed:` line is a damaged
copy: run `hf download` again.

## 4. A town's Isaac world

The world server finds the USD world for a map in the map's directory, in
`<map dir>/isaac/world.yaml`:

```yaml
usd: /data/carla-usd/Town02/Town02_physics.usda
map_origin: [0.0, 0.0, 0.0]
ground_paths:
  - "/World/env/..."
```

`usd` is the world (an absolute path, or one relative to the map directory),
`map_origin` where the world's origin sits in the map frame (zero for CARLA's towns, which
are already in it), and `ground_paths` the prims a car rests on, whose contacts are not
reported as collisions.

The Autoware maps come from one of two places, both with the same repairs (Getting
started on Isaac Sim, stage 2):

- `scripts/download_maps.sh`, as in the guide, which writes no world file;
- the [carla-town-autoware-maps](https://huggingface.co/datasets/jerry73204/carla-town-autoware-maps)
  dataset (LGPL-3.0), which ships an `isaac/world.yaml` per town:

  ```bash
  hf download jerry73204/carla-town-autoware-maps \
      --repo-type dataset \
      --local-dir ~/carla-maps
  ```

  Then set `CARLA_MAPS=~/carla-maps` in `~/swb.bash`.

**Write the world file** for a town, either way. This reads the town's physics layer from
the asset set and writes `$CARLA_MAPS/<Town>/isaac/world.yaml`; with the town in the set it
does not need CARLA's editor:

```bash
cd ~/scenario-world-bridge
src/swb_isaac_world/scripts/isaac_town.sh Town02
cat $CARLA_MAPS/Town02/isaac/world.yaml
```

The town is CARLA's name (`Town01`, `Town02`, `Town03`, `Town05`, `Town10HD`); the map
directory for `Town10HD` is `Town10`, which the script finds by itself. It names the layer
by its absolute path, so run it again after moving the asset set.

**Or fix the downloaded one by hand.** The world files in carla-town-autoware-maps name the
layer by a path on the machine that made them. Point `usd` at your copy:

```bash
T=Town02
sed -i "s|^usd: .*|usd: $CARLA_USD/$T/${T}_physics.usda|" \
    $CARLA_MAPS/$T/isaac/world.yaml
```

For `Town10` the layer is `$CARLA_USD/Town10HD/Town10HD_physics.usda`. `Town07_Opt` is in
the asset set but has no Autoware map, so Autoware cannot drive it.

The same dataset's `<Town>/carla/traffic_lights.yaml` matters on CARLA only. Its
`lanelet2_map:` line also names the building machine's path: on CARLA, point it at the
town's `lanelet2_map.osm`, or regenerate the table (Getting started on CARLA, stage 2).

## 5. Run the stack on it

`world` preloads the town before the world server answers, and `map_path` is Autoware's
map; they must be the same town:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch isaac.launch.xml \
    world:=$CARLA_MAPS/Town02/isaac/world.yaml \
    render:=headless \
    map_path:=$CARLA_MAPS/Town02
```

`world` also takes the map directory itself (`world:=$CARLA_MAPS/Town02`), and several
worlds, comma separated: all are loaded and rendered once, and the first stays loaded. A
town's first load compiles its materials and can take a quarter of an hour; wait for the
world server as in the guide, stage 3:

```bash
ros2 run swb_world_check swb_world_check hello tcp://127.0.0.1:5590 --wait 1800
```

Then run a scenario on that town, in the scenario domain:

```bash
S=$(ros2 pkg prefix --share swb_examples)/scenarios
ROS_DOMAIN_ID=${SCENARIO_ROS_DOMAIN_ID:-9} play_launch launch \
    --web-addr 127.0.0.1:8081 \
    swb_launch scenario.launch.xml \
    scenario:=$S/basic/town02_ego_drive.xosc \
    port:=${SSV2_PORT:-5555} \
    output_directory:=/tmp/isaac_town02
```

The starters `town01_` to `town10_ego_drive.xosc` each name their map as
`$(env CARLA_MAPS)/<Town>`, so they find whichever maps `CARLA_MAPS` points at.

## 6. CARLA's actors in a scenario

A scenario names a CARLA model in an entity's `model3d` attribute, by its CARLA blueprint
id. The bridge passes it to the world server, which dresses the entity in that model; on
CARLA the same id spawns the same blueprint, so the scenario runs unchanged on both.

```xml
<ScenarioObject name="parked_van">
  <Vehicle name="parked_van" vehicleCategory="car"
           model3d="vehicle.mercedes.sprinter"> ... </Vehicle>
</ScenarioObject>
<ScenarioObject name="walker_0">
  <Pedestrian name="walker_0" model="unspecified" mass="70.0"
              pedestrianCategory="pedestrian"
              model3d="walker.pedestrian.0012"> ... </Pedestrian>
</ScenarioObject>
<ScenarioObject name="barrel_0">
  <MiscObject name="barrel_0" mass="50.0" miscObjectCategory="obstacle"
              model3d="static.prop.barrel"> ... </MiscObject>
</ScenarioObject>
```

| Entity | `model3d` | Without it |
|---|---|---|
| `Vehicle` | `vehicle.<make>.<model>` | the four-wheeled car closest to its bounding box; a `bicycle` or `motorbike` category gets the two-wheeler closest in length |
| `Pedestrian` | `walker.pedestrian.0001` to `0052` | a walker picked from the entity's name |
| `MiscObject` | `static.prop.<name>`, or a `vehicle.*` id | `static.prop.streetbarrier` |

The model is only the look. Each entity's body for physics, contacts and the lidar is
still its bounding box, so keep the box the size of the model. Walkers play CARLA's walk
and idle animations at the speed the scenario moves them, and two-wheelers carry a seated
rider.

The ids the set holds, with each vehicle's size and wheels:

```bash
less $SWB_VEHICLE_USD/vehicles.md
python3 -c 'import json, sys; print(*(e["id"] for e in json.load(open(sys.argv[1]))))' \
    $SWB_VEHICLE_USD/props/props.json
```

A pedestrian's CARLA id maps to an exported walker of the same blueprint, or of the same
gender and age when that one was not exported. A `Vehicle` or `MiscObject` id the set does
not hold gives a box and a warning in the world server's log (`no CARLA body for ...`).
The ego is the exception: its body is its own car's
([Build your own stack](../build-your-own-stack.md#6-the-vehicle)).

`basic/town01_pedestrian.xosc` and `demo/town01_tour.xosc` in swb_examples are worked
examples; [Other road users and more AVs](03-other-road-users.md) shows how to move them.

## 7. A light Town01 for smaller GPUs

The full town needs a 24 GB GPU. carla-town-autoware-maps also ships a Town01 built from
the map's own point cloud: every 0.4 m voxel a rendered cube, on a flat ground, about
10 MB. Autoware localizes against the same point cloud, so it drives there as in the full
town. It has no traffic lights and no textures.

Swap it into the Town01 map directory, keeping the full world's file aside:

```bash
cd $CARLA_MAPS/Town01
mv isaac isaac.full
cp -r $CARLA_MAPS/town01_pcd_isaac_world/Town01/isaac isaac
```

Its `world.yaml` names `isaac/world.usd` relative to the map directory, so it needs no
editing. Run the stack as in section 5 with `Town01`; vehicles and pedestrians still wear
CARLA's models when `SWB_VEHICLE_USD` is set. To go back, `rm -r isaac` and
`mv isaac.full isaac`.

## 8. Another world: Mount Haruna

Worlds need not be CARLA towns. The
[haruna-tsukuba-courses](https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses)
dataset (ODbL and GSI data, see its notice) has two courses built from open map and
elevation data: the mountain road on Mount Haruna, Japan, with eight hairpins, and Tsukuba
Circuit. Each is a complete map directory: the Autoware map, `surfaces.yaml`, the Isaac
world in `isaac/`, and a scenario in `scenarios/`.

```bash
hf download jerry73204/haruna-tsukuba-courses \
    --repo-type dataset \
    --local-dir ~/courses
```

The courses' guardrails and trees are CARLA props, which each world finds through a link
named `carla-usd` in its `isaac/` directory:

```bash
ln -s $CARLA_USD ~/courses/haruna/isaac/carla-usd
ln -s $CARLA_USD ~/courses/tsukuba/isaac/carla-usd
```

Their world files name the stage relative to the course directory, so the courses can
live anywhere. Run the stack on Haruna:

```bash
play_launch launch --web-addr 127.0.0.1:8084 \
    swb_launch isaac.launch.xml \
    world:=$HOME/courses/haruna \
    render:=headless \
    map_path:=$HOME/courses/haruna
```

The course's scenario, `scenarios/haruna_down.xosc`, needs two edits before it runs:

- its `LogicFile filepath` names the building machine's directory: set it to
  `$(env HOME)/courses/haruna`, the substitution the starters use for `CARLA_MAPS`;
- its ego is the catalog entry `drift_mustang`, made for a drift controller that is not
  part of this project. To drive it with the default car, set `entryName` to
  `tesla_model3`.

Then run it as in section 5, with `scenario:=$HOME/courses/haruna/scenarios/haruna_down.xosc`.
Tsukuba works the same way, with `tsukuba/scenarios/tsukuba_lap.xosc`.

## Where next

- [Road physics: surfaces, mud and rocks](07-road-physics.md): what the roads are made of
  on Isaac.
- [Isaac assets](../design/isaac-assets.md): the set's layout, and how it is made.
- [Build your own stack](../build-your-own-stack.md): your car and your autopilot on Isaac.
