# Phase 064: SVL's Shalun Map in Autoware, CARLA and Isaac Sim

LG's SVL (LGSVL) simulator shipped a map of Taiwan CAR Lab, the automated-vehicle test field
at Shalun, Tainan: a 3D scene of about 200 x 140 m, with Lanelet2, OpenDRIVE, Apollo and
Autoware vector maps of its roads. SVL's asset store shut down in 2022. This phase brings the
map into our three stacks: its lanes to Autoware and CARLA, its 3D model to Isaac and CARLA,
all in one frame.

**Status**: in progress, 2026-10-30. The conversion is done on 241. Not yet tried:
- loading the world in Isaac (241 has no free GPU);
- CARLA's `make import` (it needs the UE 4.26 editor build on newslab-server139);
- an Autoware run on it.

**Source**: user direction 2026-10-30: "the discontinued LGSVL provided online map assets.
One of them is in Taiwan. Let's see if we can get that map", then "how we can import the
Shalun map to Autoware, CARLA and Isaac Sim for both 3D model and lane map".

## Where the map comes from

SORA-SVL (github.com/YuqiHuai/SORA-SVL), a local replacement for SVL's cloud, published
SVL's whole asset library on Google Drive (folder `10KX_VKcahLyV40mGoUWhrfjtE02IqOuq`):
- `maps.zip` (8.5 GB, 94 assets);
- `hdmaps.zip` (264 MB);
- `preview.zip`;
- `vehicles.zip`;
- `plugins.zip`;
- `geojson.json`, the store's lane geometry with each map's origin.

Shalun is asset `fce86ba3-a4d3-4a56-9b18-97975168cbc8`. The only map with an origin in
Taiwan is lat 23.000, lon 120.227. The local copy is `/mnt/toshiba/aeon/svl-assets/` on 241,
with Shalun under `Shalun/`.

The asset is a zip:
- `manifest.json`: format `com.svlsimulator.5`, `mapOrigin` (215743.6, 2546179.0);
- the HD maps: `lanelet2/Shalun.osm`, `opendrive/Shalun.xodr`, `apollo50/base_map.bin` and
  `autoware/AutowareVectorMap.zip`;
- a preview image, showing the "Taiwan CAR Lab" sign;
- Unity 2020.3 asset bundles: `_environment_main_linux` (8.7 MB) and
  `_environment_textures` (31 MB).

The scene holds:
- 1262 renderers over 1110 meshes;
- 35 HDRP materials and 56 textures;
- 1018 mesh colliders;
- 98 lights.

The lanes hold:
- 263 lanelets;
- 28 traffic lights;
- 23 stop signs;
- 28 crosswalks;
- curbs.

## The frame

SVL places a map on the earth with its `MapOrigin` GameObject:
- `OriginEasting`/`OriginNorthing` = 215743.6, 2546179.0, in UTM zone 51;
- AltitudeOffset is 27.85 m.

A point's UTM position is its position in MapOrigin's local frame plus that origin: local x
is east, local z north. Shalun's MapOrigin turns -90 degrees about Unity's y.

So the map's local ENU frame is Unity (z, -x, y) after undoing that turn. That is a mirror
of Unity's left-handed frame, so every triangle's winding reverses.

Measured agreement:
- **Lanelet2:** the export's `x`/`y` tags are in this frame. They match a UTM inverse of its
  `lat`/`lon` to within 0.4 m; SVL's own conversion is that far off.
- **OpenDRIVE:** SVL wrote it turned 90 degrees: map (east, north) = (y, -x) of the file's.
  This was found by matching geometry starts to lanelet nodes: median 0.00 m under that
  transform, 1.3-2.4 m under the seven others.
- **The 3D scene:** the asphalt and the lanelets coincide in a top-down plot.

All outputs use this frame:
- **Autoware:** MGRS 51QTF, whose local frame puts the origin at (15743.6, 46179.0).
- **Isaac:** the world frame is the map's local ENU, and `world.yaml`'s `map_origin` is
  (15743.6, 46179.0, 0).
- **CARLA:** its world is the OpenDRIVE frame with y negated, as for every OpenDRIVE map.

## Tools (`scripts/svl_import/`)

- `unity_dump.py` (UnityPy 1.25) reads the bundles into plain files:
  - one npz per mesh;
  - `materials.json`;
  - PNG textures;
  - `scene.json`, which holds every renderer's world matrix, the lights, and MapOrigin's
    origin and yaw.

  Unity's built-in meshes (cube, cylinder and so on) are not in the bundle and get stand-ins.
  Shalun uses 23 cylinders.
- `svl_to_usd.py` (Isaac's Python) writes `isaac/world.usdc`:
  - every instance baked into the map frame: 125 750 triangles;
  - UsdPreviewSurface materials with the base-colour map and its tiling, plus normal maps;
  - colliders where Unity had a MeshCollider;
  - SimReady Q-codes by material: asphalt Road; paint, arrows and crosswalks RoadLine;
    sidewalks and curbs SideWalk; grass Terrain; buildings Building.

  It also writes `world.yaml`, whose 226 ground paths are the labelled ground meshes. Unity's
  lights are left out: the world server's sun and sky light the scene.
- `svl_to_autoware.py` writes the Autoware map dir:
  - `lanelet2_map.osm`: lat/lon recomputed from the x/y tags, with MGRS `local_x`/`local_y`.
    Stop signs `usR1-1` become `stop_sign`, Autoware's subtype. 235 road lanelets get
    `speed_limit` 30 km/h.
  - `map_projector_info.yaml`: MGRS 51QTF.
  - `pointcloud_map.pcd`: every mesh sampled at 0.1 m and thinned to 0.1 m voxels, 7.4 M
    points. Intensity is set by material: paint 200, asphalt 20.

  The lanelets load in Lanelet2 with 0 errors: 263 lanelets, 51 regulatory elements.
- `svl_to_carla.py` (Python 3.11 with pip `bpy` 4.2) writes a CARLA import package:
  - **The OpenDRIVE:** turned into the map frame, and its signal controllers rebuilt (below).
  - **The FBX:** built in Blender in the map frame and exported with Blender's defaults. UE
    imports it in cm with y mirrored, which is CARLA's convention. Meshes are merged and
    named as CARLA's import tags them: Road_Road, Road_Marking, Road_Sidewalk, Road_Curb,
    Road_Grass, Building_*, Prop_*.

    Traffic lights and stop signs are left out of the FBX, because CARLA spawns its own from
    the OpenDRIVE signals. SVL wrote CARLA's signal types, 1000001 and 206.
  - **The package JSON** for `make import`.

### SVL's OpenDRIVE crashes CARLA

`carla.Map(name, xodr)` segfaults on SVL's file, the original as much as the turned one.
gdb places it in `MapBuilder::SolveControllerAndJuntionReferences` (MapBuilder.cpp:941).

The cause: SVL's 16 controllers name signal ids that do not exist (`"174"`; the signals are
`signal_5` and so on), and CARLA dereferences them unchecked. Keeping one junction at a time
showed it: the 8 junctions with controllers crash, and the 9 without do not.

`fix_controllers` drops them and gives each traffic light its own controller, listed under
the junction at its road's end: 28 controllers. The world server sets light states anyway.

After the fix, CARLA 0.9.16's client parses the file:
- 373 topology edges;
- 28 traffic lights and 23 stop signs as landmarks;
- 0.5 m waypoints a median 0.10 m from the Lanelet2 centrelines.

The 95th percentile is 3.4 m: lanes in the OpenDRIVE that have no lanelet (to check).

## Outputs (on 241: `/mnt/toshiba/aeon/svl-assets/Shalun/`)

| Dir | What |
|---|---|
| `dump/` | unity_dump.py's meshes, textures, materials and scene |
| `map/` | the Autoware map dir: `lanelet2_map.osm`, `map_projector_info.yaml`, `pointcloud_map.pcd` |
| `map/isaac/` | `world.usdc`, `textures/`, `world.yaml` |
| `carla/` | `Shalun.json`, plus `Shalun/Shalun.xodr`, `Shalun.fbx` and its textures, for CARLA's `Import/` |
| `topdown.png` | the scene by class with the lanelets over it, the frame check |

## Next

1. **Isaac:** on a host with a free RTX GPU, `just isaac-world world:=<map dir>`. Check:
   - the textures, the tiling, and the normal maps' sign;
   - colliders under the wheels;
   - the RTX lidar's returns;
   - the Q-code ground paths.
2. **CARLA:** on newslab-server139, copy `carla/` into CARLA_SRC's `Import/` and run
   `make import`, then `make package`. Check:
   - that the FBX lands on the OpenDRIVE roads (the y mirror);
   - CARLA's traffic lights at the 28 signals;
   - spawn points.
3. **Autoware:** `map_path:=<map dir>`. Check:
   - NDT on the sampled PCD against the RTX lidar in Isaac;
   - routing across the junctions;
   - the stop-sign and traffic-light modules on the converted regulatory elements.
4. **SSv2:** an OpenSCENARIO on the Lanelet2 map.
5. **Open:** the OpenDRIVE lanes without a lanelet. Also the PCD's intensity is a stand-in;
   SimReady non-visual materials would make the RTX lidar's intensity match.
