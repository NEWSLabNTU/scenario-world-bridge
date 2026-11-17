# Maps and assets

What a map directory holds, the datasets a project can fetch, and CARLA's assets in USD.
How a project takes them: [The project file](project-file.md#assets).

## A map directory

One directory per map, in Autoware's map layout plus each simulator's files:

```
<map>/  lanelet2_map.osm  pointcloud_map.pcd  pointcloud_map_metadata.yaml
        map_config.yaml   map_projector_info.yaml
        carla/map_origin.yaml       (only for a map not in CARLA's frame)
        isaac/world.yaml            (Isaac Sim: the world for this map)
        surfaces.yaml               (Isaac Sim: optional, the label defaults)
```

A fetched map is read-only. What the stack writes for a map goes to the project's
`state/<map>/`:

| In `state/<map>/` | Written by | What |
|---|---|---|
| `traffic_lights.yaml`, `carla/traffic_lights.yaml` | CARLA's world server, the first time it loads the map (and again when the map's lanelet2 changes) | which Lanelet2 signal is which CARLA light |
| `ruts/runs/<time>.npz` | Isaac's world server, at each scenario's end | the soil's ruts |
| `surfaces.yaml`, `road_detail.yaml` | you ([Road physics](road-physics.md)) | the project's ground for this map, used over the map's |

**A map not drawn in CARLA's frame** needs `carla/map_origin.yaml`: the map-frame position
of CARLA's world origin, in metres, no rotation. CARLA's towns share CARLA's origin and need
none; a surveyed map uses grid coordinates tens of kilometres from zero.

```yaml
# <map>/carla/map_origin.yaml
x: 52600.0
y: 67800.0
z: 0.0       # optional
```

Both vehicle bridges read it. No file means an origin of zero; a file that does not parse
stops the run rather than putting every pose hundreds of metres off.

**The Isaac world file** names the world's USD stage, where its origin sits in the map
frame, and the prims a car rests on (whose contacts are not collisions):

```yaml
# <map>/isaac/world.yaml
usd: ${SCENSIM_CARLA_USD}/Town01/Town01_physics.usda
map_origin: [0.0, 0.0, 0.0]
ground_paths:
  - "/World/env/..."
```

`usd:` may name the stage through `${SCENSIM_CARLA_USD}` (the project's `assets/carla-usd`, set
by the stack), relative to the map directory, or absolutely. `ros2 run scensim_isaac_world
make_world <map dir> [--town <Town>] [--usd <stage>]` writes it for a CARLA town from the
USD set; it runs in Isaac's Python.

## Datasets

Public, on Hugging Face under `jerry73204`; `scensim assets list` prints them with
their sizes.

| Dataset | Holds | Licence |
|---|---|---|
| [carla-town-autoware-maps](https://huggingface.co/datasets/jerry73204/carla-town-autoware-maps) | Town01, 02, 03, 05 and 10 as Autoware maps, each with its Isaac world file; `town05_ring_110kmh/` (Town05's ring road at 110 km/h); `town01_pcd_isaac_world/` (a light Town01 world built from the point cloud, about 10 MB) | LGPL-3.0 |
| [carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd) | CARLA 0.9.16's content as SimReady USD, 25.5 GB: six towns, `vehicles/` (41 vehicles, `props/` 97, `walkers/` 52, indexed by `vehicles.json`, `props.json`, `walkers.json`, `vehicles.md`) | CC BY 4.0 |
| [haruna-tsukuba-courses](https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses) | the Mount Haruna mountain road and Tsukuba Circuit: Autoware maps, Isaac worlds, a scenario each; their props come from the USD set | ODbL, GSI |
| [isaac-road-physics-stages](https://huggingface.co/datasets/jerry73204/isaac-road-physics-stages) | the flat test ground with 13 surface strips and its surface map, the vehicle check's proving ground, `procedural/` (the rough road for Town01, worn ruts) | Apache-2.0 |
| [newslab-golfcart-models](https://huggingface.co/datasets/jerry73204/newslab-golfcart-models) | a golf cart for Isaac Sim: body, rig, vehicle spec, catalog entry, RViz mesh | Apache-2.0 |

A project takes only what it names (`take:`): Town01 of the maps is about 0.1 GB; Town01
and `vehicles/` of the USD set several GB. Each dataset is verified file by file against
its manifest when it has one.

**The CARLA towns' maps** are TUM's
[CARLA-Autoware-Bridge](https://github.com/TUMFTM/Carla-Autoware-Bridge) conversions (map
pack copyright 2024 TUM, LGPL-3.0), with repairs Autoware and SSv2 need: traffic-light type
tags, lanelet subtypes and turn directions, line types, bound directions, local
coordinates, duplicate points, joins, right-of-way elements, speed limits and walkways from
CARLA's OpenDRIVE, and each signalised approach linked to its CARLA light. CARLA calls
Town10 `Town10HD`; the map directory is `Town10`.

**CARLA's assets in USD.** Isaac renders CARLA's towns, vehicles, props and walkers from
the USD set; a scenario entity's `model3d` picks its model by CARLA blueprint id
([Tutorial 6](../tutorials/06-carla-assets-on-isaac.md)). For developers, how the set is made
from a CARLA editor build: [Isaac assets](../design/isaac-assets.md).
