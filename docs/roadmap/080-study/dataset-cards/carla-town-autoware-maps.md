---
license: lgpl-3.0
pretty_name: CARLA towns as Autoware maps, repaired for scensim
tags: [carla, autoware, lanelet2, hd-map, autonomous-driving, scensim]
---

# CARLA towns as Autoware maps, with scensim's repairs

Autoware maps (lanelet2 and point cloud) of CARLA 0.9.16's Town01, Town02, Town03, Town05 and
Town10. They start from TUM's carla-autoware-bridge map pack, with the repairs
[scensim](https://github.com/NEWSLabNTU/scensim) applies on top:
traffic-light geometry and tags, speed limits, routing fixes. TUM's original osm is kept beside
each one as `lanelet2_map.osm.orig`.

| Path | Content |
|---|---|
| `<Town>/lanelet2_map.osm` | the repaired lanelet2 map |
| `<Town>/pointcloud_map.pcd`, `map_config.yaml`, `map_projector_info.yaml`, `LICENSE.md` | TUM's, unchanged |
| `<Town>/carla/traffic_lights.yaml` (Town01, Town02) | the signal table the CARLA vehicle bridge reads |
| `<Town>/isaac/world.yaml` | the town's Isaac Sim world, in the Autoware map frame |
| `town01_pcd_isaac_world/` | a light Isaac Sim world of Town01 made from its point cloud (0.4 m voxels and a flat ground, 10 MB, self-contained): replace `Town01/isaac/` with its `Town01/isaac/` on a GPU where the full CARLA USD town is too heavy |
| `town05_ring_110kmh/` | Town05 with its ring road's speed limit at 110 km/h, for high-speed runs |

## Use

```bash
hf download jerry73204/carla-town-autoware-maps \
    --repo-type dataset \
    --local-dir ~/carla-maps
export CARLA_MAPS=~/carla-maps
```

The Isaac world files (`<Town>/isaac/world.yaml`) name their stage as
`${SCENSIM_CARLA_USD}/<Town>/<Town>_physics.usda`: set `SCENSIM_CARLA_USD` to where
[carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd) is
(a scensim project sets it to its own copy). The signal tables (`<Town>/carla/traffic_lights.yaml`)
name their lanelet map relative to the town's directory. No file names a path of the machine
that made it.

## Licence

Full text: `NOTICE.md`, `LICENSE.md`, `COPYING.GPL-3`.


TUM's carla-autoware-bridge maps: LGPL-3.0, © 2024 Gemb Kaljavesi, TUM (`LICENSE.md` in each
town). The repairs are under the same licence. The towns themselves: CARLA (code MIT, assets
CC-BY 4.0).
