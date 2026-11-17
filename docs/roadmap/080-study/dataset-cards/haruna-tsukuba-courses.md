---
license: odbl
pretty_name: Mount Haruna and Tsukuba driving courses from open data
tags: [isaac-sim, usd, autoware, lanelet2, openstreetmap, racing, drifting, scensim]
---

# Mount Haruna and Tsukuba: driving courses from open data

Two driving courses built from open data by the course generator of
[scensim](https://github.com/NEWSLabNTU/scensim)'s Isaac Sim
world server (`scensim_isaac_world.course`, 2026-10-30):

- `haruna/`: the mountain road (touge) on Mount Haruna, Gunma, Japan, downhill section.
- `tsukuba/`: Tsukuba Circuit, a closed course.

Each course holds:

| Path | Content |
|---|---|
| `lanelet2_map.osm`, `pointcloud_map.pcd`, `map_projector_info.yaml`, `map_config.yaml` | an Autoware map (local transverse Mercator) |
| `surfaces.yaml` | the road surface classes |
| `isaac/world.usdc`, `isaac/world.yaml` | the Isaac Sim world: terrain, road, lines, props |
| `scenarios/` | an OpenSCENARIO file on the course |
| `course/` | the generator's intermediate data (centreline, meshes, drift points) |
| `raw/` | the OpenStreetMap data (`osm.json`, `road.json`) and the list of GSI tile URLs the generator fetched (`jobs.txt`); the GSI tiles themselves are not included |

## Use

The props (pines, barriers) come from the CARLA asset set. Link it in each course before
loading the world:

```bash
hf download jerry73204/haruna-tsukuba-courses \
    --repo-type dataset \
    --local-dir ~/courses
ln -s ~/carla-usd ~/courses/haruna/isaac/carla-usd
ln -s ~/carla-usd ~/courses/tsukuba/isaac/carla-usd
```

`~/carla-usd` is [carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd).
`isaac/world.yaml` names the stage relative to the course directory, so the course can live
anywhere.

The scenarios (`<course>/scenarios/*.xosc`) name their map as `$(env SCENSIM_MAPS)/<course>`:
set `SCENSIM_MAPS` to the directory holding the courses (`~/courses` above; a scensim project sets
it to its own `assets/maps`).

## How it was made

Both courses come from one generator, `scensim_isaac_world.course` in
[NEWSLabNTU/scensim_isaac_world](https://github.com/NEWSLabNTU/scensim_isaac_world) (Apache-2.0),
at commit `163f6e5` (2026-10-30). Each course is described by one YAML file in that repository,
`scensim_isaac_world/courses/haruna.yaml` and `tsukuba.yaml`: the map origin, the road (an
Overpass query, a bounding box, and the section's start and end), the lane layout, the margin
around the section, the GSI layers and zoom levels, the speed limit and the props.

Haruna is Gunma prefectural road 33 (渋川松井田線) from Lake Haruna towards Ikaho: 1.55 km
downhill, about 900 m down to 810 m, eight hairpins of 9 to 20 m radius.

| Step | What it does |
|---|---|
| `fetch` | Downloads the road's ways from OpenStreetMap (Overpass API), and GSI's 5 m DEM tiles (`dem5a`, zoom 15) and aerial photo tiles (`seamlessphoto`, zoom 18) covering the section plus a 300 m margin. Cached under `raw/`, so a rerun is offline. |
| `centreline` | Chains the road's ways, cuts the section, resamples it at 0.5 m and smooths it (OSM's nodes are coarse in hairpins). Heights come from the DEM, smoothed along the road so the profile has no steps. Two lanes of 3.25 m, shoulders of 0.75 m. |
| `geometry` | The terrain: a grid mesh from the DEM, cut or filled to the road's profile under the road and blended back to the DEM over 8 m, with the aerial photo mosaic as its texture. The road: a ribbon along the centreline in strips at most 0.5 m wide, with a yellow centre line and white edge lines on top. |
| `usd` (Isaac Sim's Python) | One USD stage in the map frame: terrain, road and lines with colliders, guardrails where the ground drops away and outside the hairpins, trees where the photo is forest (props from CARLA's assets). Writes `isaac/world.usdc` and `isaac/world.yaml`. |
| `maps` | Autoware's maps: `lanelet2_map.osm` (one lanelet per lane per stretch, both directions, local coordinates on every point) and `pointcloud_map.pcd`, sampled from the same meshes the simulated lidar sees and voxelised at 0.2 m, so the localization map and the simulated world agree. |
| `scenario` | An OpenSCENARIO run down the course, the surface map, and drift points at each hairpin with its radius and turn angle. |

To rebuild Haruna you need Python 3 with numpy, scipy, pyproj, Pillow and PyYAML, Isaac Sim
6.1 (its Python has `pxr`, for the `usd` step), and CARLA's USD assets from [carla-0.9.16-isaac-usd](https://huggingface.co/datasets/jerry73204/carla-0.9.16-isaac-usd):

```bash
git clone https://github.com/NEWSLabNTU/scensim_isaac_world.git
cd scensim_isaac_world
git checkout 163f6e5
export CARLA_USD=~/carla-usd
C=scensim_isaac_world/courses/haruna.yaml
OUT=~/courses/haruna
python3 -m scensim_isaac_world.course $C $OUT fetch centreline geometry
$ISAAC_ROOT/venv/bin/python -m scensim_isaac_world.course.usd $C $OUT
python3 -m scensim_isaac_world.course $C $OUT maps scenario
```

`tsukuba.yaml` the same way. The `fetch` step downloads only the tiles covering the section;
please keep it that way and do not mirror GSI's tile server. The design and its measurements
are in scensim's
[roadmap 058](https://github.com/NEWSLabNTU/scensim/blob/main/docs/roadmap/058-touge-course-from-open-data.md).

## Sources and licences

Full text: `NOTICE.md`.


- Roads: © OpenStreetMap contributors, ODbL 1.0.
- Terrain (5 m DEM, `dem5a`) and ground texture (`seamlessphoto`): made by processing
  tiles of the Geospatial Information Authority of Japan (GSI). 出典：国土地理院の基盤地図情報
  数値標高モデル及びシームレス空中写真を加工して作成. The tiles are not redistributed here.
- Props: CARLA's assets, CC-BY 4.0.

## Citation

If you use these courses, please cite the dataset and credit its sources:

```bibtex
@misc{newslab2026harunatsukuba,
  title        = {Mount Haruna and Tsukuba: Driving Courses from Open Data},
  author       = {{NEWSLab, National Taiwan University}},
  year         = {2026},
  howpublished = {Hugging Face dataset},
  url          = {https://huggingface.co/datasets/jerry73204/haruna-tsukuba-courses}
}
```

- Map data © OpenStreetMap contributors, ODbL 1.0.
- 出典：国土地理院の基盤地図情報数値標高モデル（5mメッシュ）及びシームレス空中写真を加工して作成
  (made by processing GSI's 5 m DEM and seamless aerial photo).
- Props: CARLA (Dosovitskiy et al., "CARLA: An Open Urban Driving Simulator", CoRL 2017),
  CC BY 4.0.
