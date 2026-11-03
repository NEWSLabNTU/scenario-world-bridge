# Phase 072: The NTU Campus as a CARLA 0.9.16 Map, Built from the Scan

The NTU campus as a CARLA map package that any stock CARLA 0.9.16 binary can install: its
roads in OpenDRIVE, its ground, buildings and trees as meshes, built straight from the
vendor's MLS scan and the Autoware maps. It is not a conversion of 059's Isaac world. 059's
tools are reused as building blocks, and their outputs are shaped for CARLA from the start.

**Status**: in progress, 2026-11-03. The tools are on 133 (below). The pilot, r01's corridor, is
imported, packaged and installed into a stock 0.9.16, and its camera compared with Isaac's.

Set up on 133 (2026-11-03; user direction: "We run things on our host"; files shared through
the NAS):
- **The binary:** CARLA 0.9.16's official release, from the NAS (`autoveh/apps/carla-simulator/
  CARLA_0.9.16.tar.gz`), in `/mnt/p5plus/aeon/carla/CARLA_0.9.16` (19 GB; `CARLA_DIR` in
  133's `.env`).
- **The editor build:** 139's (built 2026-10-21 for roadmap 043; never used for `make
  import`), staged by swb 139 at the NAS's `home/carla-src-0.9.16-139/`: two zstd tarballs,
  39 GB, without `.git`, the DDC, the content tarball, packman, and the Mac and Windows
  binaries. It is unpacked in `/mnt/p5plus/aeon/carla-src` (UE 106 GB in all, engine 77 GB,
  CARLA 29 GB), at our own path, not 139's `/mnt/wd/aeon/carla-src` (user choice; rebuild
  if it gives trouble).
  - `env.sh` there sets `UE4_ROOT` and `CARLA_ROOT`.
  - `~/.config/Epic/UnrealEngine/Install.ini` maps the engine's GUID to the new path.
  - A headless boot (`UE4Editor-Cmd CarlaUE4.uproject -nullrhi`) took 70 s and needed no
    rebuild: the Carla, CarlaTools and CarlaExporter plugins mount.
  - 139's path stays in CARLA's dependency configs (cmake files, `libsqlite3.la`). That
    matters only for a LibCarla rebuild, and they are text, so it can be rewritten.
  - Untested here: `make import` and `make package`.
- **Room:** 059's Isaac Sim, the CARLA vehicle USDs and the run logs were deleted. The Isaac
  worlds and drive bags were moved to the NAS (`autoveh/dataset/2026-10-30 NTU twin
  map/archive-133-2026-11-03/`, checksummed before the local copies went). `/mnt/p5plus` has
  14 GB left, tight for an import's cook.

**Source**: user direction 2026-11-03: "Shift our focus to build NTU map in CARLA 0.9.16
instead", "I'd like to learn the map creation process on CARLA. Check the information included
in a map package and how it's distributed. Also, see how to approach realistic rendering in
CARLA like what we did in Isaac Sim", then "Instead of converting from Isaac Sim map to CARLA, I
prefer to build CARLA map from NTU dataset and reuse tools learned from Isaac map build."

**Depends on**: [059](059-ntu-twin-on-isaac.md), for the data, the frame and the tools.
[064](064-svl-shalun-map-import.md), the CARLA import, package and install chain, done
end to end for Shalun on 241. [019](019-ntu-campus-twin.md), the first NTU-in-CARLA plan
(never imported).
**Relates to**: [062](062-ntu-roads-from-open-data.md) (the roads around the campus, the
city's signal data). 059's Isaac world is swb 118's since 2026-11-03.

## The pilot: r01's corridor (2026-11-03)

The whole chain ran on 133: from 059's data to a package that a stock 0.9.16 binary
installs and loads. Tools are in `scripts/ntu_carla/`; outputs are in
`/mnt/p5plus/aeon/ntu-twin/carla/pilot/`.

1. **Road network** (`lanelet_to_xodr.py`, written for this; no converter).
   - Each road lanelet becomes a one-lane road whose reference line is its centreline. That
     is the kept centreline where 059 widened a curve.
   - The lane's width follows the bounds, and the elevation follows the centreline's z.
   - Lanelets in a split or a merge become connecting roads, in 5 junctions.
   - The geoReference is a transverse Mercator at MAP_ORIGIN (25.01767, 121.53819).
   - r01 has 155 lanelets. CARLA's client parsed them into 197 topology edges and 4073 1 m
     waypoints. On the full map (285 lanelets), the waypoints sit on the lanelet centrelines
     (to within the 1 m sample spacing) and their z is within 7 mm.
   - r01 and r02 have no lights; 48 roads start and 55 end at the map's edge.
2. **Meshes** (`carla_meshes.py`): 72 tiles within 30 m of r01, in 56 s.
   - It reuses 059's `world_usd.py`: the lane raster, road clearing, tree split, the de-light
     and shard rules, and the crown cut-outs.
   - The ground is split under the lanelets into Road (0.34 M triangles) and Terrain (3.4 M),
     both on the ortho.
   - Objects are decimated to 30 %: props 2.4 M and trees 4.5 M triangles.
   - Trees carry their cut-outs as vertex alpha. No scan buildings fall in r01's tiles; its
     walls are props.
   - `lane_raster` with margin 0 grows without end (scipy's `iterations=0`), so the road
     margin is one cell (0.25 m).
3. **Package** (`carla_package.py`, bpy 4.2, a Python 3.11 venv at `~/.venvs/bpy42`).
   - One FBX of 317 MB with an object per tile and class (`Road_Road_<t>`, `Road_Grass_<t>`,
     `Prop_<t>`, `Tree_<t>`), the 72 ortho textures, the OpenDRIVE, and the JSON with
     `use_carla_materials: false`.
4. **Import** (`Import.sh --package NTU`): 13 min, 15 GB RAM, 678 MB of content. Three
   things needed fixing on 133:
   - `Import.py` imports `carla`. A source build makes it in its PythonAPI step; here the
     release's cp310 wheel is unpacked on `PYTHONPATH` (`/mnt/p5plus/aeon/carla/pyapi310`).
   - 0.9.16's MDL converter plugin asserts without its SDK, which lives in NVIDIA's packman
     cache (left out of the copy). `Plugins/Converters/get_dependencies.sh` with
     `PM_PACKAGES_ROOT` (now in `env.sh`) fetched it.
   - CARLA's `Import.py` does not set UE's `VertexColorImportOption`, and the FBX's vertex
     colours were dropped: props and trees rendered white. 133's copy of `Import.py` now
     sets it to `Replace` (`Import.py.orig` beside it).

   The import sorts meshes into `Static/Road` (37) and `Static/Terrain` (194, props and
   trees too: it has no other folder). It gives every mesh complex-as-simple collision.
5. **Material** (`ue_materials.py`, run in the editor's Python): one master material,
   `M_NTU_ScanColour`, on the props and trees. BaseColor is the vertex colour, OpacityMask
   its alpha (masked, two-sided).
6. **Package and install.**
   - `Package.sh --packages=NTU`: 7.7 min, giving `Dist/NTU_-dirty.tar.gz` of 1.04 GB. The
     tag is empty because `.git` was not copied.
   - The tarball holds `CarlaUE4/Content/NTU/` (`Maps/NTU/NTU.umap`, `OpenDrive/NTU.xodr`,
     `TM/NTU.bin`, `Config/NTU.Package.json`, the cooked meshes and textures), the CARLA
     assets the map references, and the tagged materials. It has no pedestrian navmesh.
   - `ImportAssets.sh` put it into the 0.9.16 binary in 10 s.
   - `/mnt/p5plus/aeon/carla/pilot_build.sh` runs import, material, package and install in
     one go (19 min).
7. **Running it.**
   - The binary (port 2400, offscreen, Epic) loads `NTU`: 31 spawn points, 2084 waypoints at
     2 m.
   - A Model 3 settles on the road at z 12.05.
   - With a 1280x720 camera and a 64-channel lidar (34 k points a scan), a synchronous
     frame takes 12 ms (p95 15 ms). CARLA holds 4.3 GB of VRAM.
8. **Picture** (`carla_probe.py`: 059's four r01 views, histogram auto-exposure, sun at 55
   deg): `pilot/carla_vs_isaac.jpg`.
   - It is in the same class as Isaac's after N10. Under the canopy the road is dappled and
     brighter.
   - Faults:
     - The start view is overexposed.
     - Turn 2's wall is in shade: CARLA's sun azimuth is not 059's convention.
     - Decimation leaves larger floating shards, with hard shadows on the road.

9. **Proof run** (`carla_drive_video.py`, 2026-11-03): CARLA's own BasicAgent drove r01 on the
   pilot package in the stock binary, planning on our OpenDRIVE. Autoware on CARLA is C11.
   - It started at r01's start pose and arrived 3.2 m from the goal: 343 m in 64 s of sim
     time at 20 km/h.
   - The video (`ntu_carla_r01_agent.mp4`, 67 s) is a chase camera, the front camera and the
     orthophoto with the track and the agent's route. It is on the NAS under
     `autoveh/dataset/2026-10-30 NTU twin map/video/`.
   - The pilot package itself is in the NAS's `home/carla-0.9.16-ntu-pilot/`, with a README.

Next:
- the sun's azimuth matched to Isaac's;
- exposure compensation;
- shards dropped after decimation as well as before;
- walls and buildings in their own folder (semantic tag Building, not Terrain);
- then the whole site, and Autoware on it.

## The whole site, and Autoware on it (2026-11-03)

1. **Fixes from the pilot.**
   - Sun: CARLA's `sun_azimuth_angle` is the map azimuth + 90, so Isaac's 135 is CARLA's
     225. At 225 turn 2's wall is lit as in Isaac.
   - Exposure: the RGB camera's `exposure_compensation`. On the pilot, -1 brought the start
     view from overexposed to a mean of 111. On the whole site -1 leaves the road under the
     canopy too dark (mean 35). Item 6 below settles it.
   - Shards: `carla_meshes.py` drops small floating components after decimation too (fewer
     than 25 faces and more than 1 m above the ground), not only before it.
   - Semantic folders: `ue_materials.py` moves `Tree_*` to `Static/Vegetation`,
     `Building_*` to `Static/Building` and `Prop_*` to `Static/Static`, the folders CARLA's
     tagger reads (330 meshes on the site).
2. **Build-host fixes on 133.**
   - `Util/BuildTools/Import.py` (CARLA's source tree) is patched to import with
     `"VertexColorImportOption": "Replace"`; without it UE drops the vertex colours and props
     and trees render white. The original is kept beside it as `Import.py.orig`.
   - `Import.py` imports `carla`: `PYTHONPATH` points at the unpacked cp310 wheel.
   - The swb tools' `carla` module is the 0.9.16 cp310 wheel, `pip install --user --no-deps`.
3. **The whole site** (`/mnt/p5plus/aeon/carla/site_build.sh`).
   - 167 tiles, about 26 M triangles after decimation; the FBX is 764 MB.
   - Import 23 min, material and folders 7 min, package and install the rest: 44 min in all.
   - The package is 2.2 GB (`Dist/NTU_*.tar.gz`). The town is `NTU`, as the pilot's.
   - Still visible: grey lumps and planes along the road (item 6: the scan's own parked cars
     and scooters, not decimation), and the orthophoto's blur on the ground.
4. **Autoware on it, through the bridge.**
   - The map dir is `$SWB_NTU_TWIN/carla/NTU`: links to 059's Lanelet2 map, point cloud and
     projector, plus `carla/map_origin.yaml` (52500, 67700, 0). The bridge takes the town
     from the directory name.
   - `scenarios/ntu_twin/ntu_carla_r01.xosc` is 059's r01 scenario with that map dir.
   - `CARLA_PORT=2400 just scenario scenarios/ntu_twin/ntu_carla_r01.xosc` passed three
     times that day (junit 0 failures): the ego stack localized on the scan's point cloud and drove
     r01's 340 m in about 95 s of sim time.
   - One run logged an MRM for 0.1 s about 47 s in; not examined.
5. **Proof video** (`carla_watch_video.py`). A second CARLA client that never ticks: it waits
   for the ego (`role_name` `hero`), attaches a chase and a front camera, and records each
   tick the bridge drives, until the ego is despawned.
   - `ntu_carla_r01_autoware.mp4`, 95 s, on the NAS under
     `autoveh/dataset/2026-10-30 NTU twin map/video/`.
   - The scenario ends at SSv2's reach condition, 8.6 m before the marked goal.

6. **Exposure and the grey planes** (2026-11-04).
   - Exposure. CARLA's histogram auto-exposure is clamped to `exposure_min_bright` 10 ..
     `exposure_max_bright` 12 (EV100). Every r01 view of this map wanted less than 10, so all
     four sat on the floor and `exposure_compensation` only moved them together: lowering the
     floor to 8 raised all four, and at 7 the views came off it. With the floor at 7 the
     metering's own target is bright (start view mean 162), so compensation brings it down.
     Mean brightness of the four probe views (start, turn 2, canopy, palms):

     | camera | start | turn 2 | canopy | palms |
     |---|---|---|---|---|
     | ev -0.5, floor 10 (the first video) | 123 | 87 | 45 | 94 |
     | ev 0, floor 7 | 162 | 144 | 135 | 160 |
     | **ev -1.5, floor 7** | 124 | 103 | 89 | 122 |

     `carla_probe.py` and `carla_watch_video.py` default to ev -1.5, floor 7. The start view's
     washed-out road is the orthophoto's own sunlight, not exposure. The ego's cameras are the
     car's (`sim_vehicle_spec`), not the map's, and are unchanged.
   - Grey planes. Not decimation: the scan's `other` meshes have no edge over 2 m in the tiles
     that show them, open3d's quadric output has no triangle off the source surface, and
     pyfqmr and meshoptimizer give the same shapes at the same count. CARLA's semantic camera
     tags them Static (our props). They are the scan's parked cars, scooter rows and hedges:
     coarse one-sided lumps, white or grey where the vehicles are, which 059's Isaac render
     shows too (`pilot/carla_vs_isaac.jpg`, the canopy view), only softer.
   - Smooth shading. `carla_package.py` now smooth-shades every mesh (Blender's default is
     flat; buildings keep edges over 60 deg sharp), and UE's recomputed normals honour the
     FBX's smoothing groups. Rebuilt and compared at the same views: no visible change. The
     faceting is in the geometry (noisy triangle orientations) and its hard cascaded shadows.
   - The rebuild's first import was killed by the OOM killer at 30.7 GB (the ego stack held
     11 GB beside it), and `Import.sh` still exited 0: a 239 MB package went into CARLA.
     `site_rebuild.sh` now refuses to package or install unless the material step found 300+
     meshes and the package is over 1 GB. Stop the ego stack before an import on 133.
   - r01 again on the rebuilt map with the new exposure: passed (four of four now),
     `ntu_carla_r01_autoware_v2.mp4` on the NAS beside the first.

Next: the parked vehicles (drop them from the props, or put CARLA's vehicle meshes in their
place), the hedges' faceting (smoothing the geometry, not the normals), r02, and the site
package on the NAS.

## How a CARLA map is made and shipped

The facts this phase builds on. Sources: CARLA 0.9.16's `Docs/tuto_M_*.md`,
`Docs/large_map_import.md`, `Util/BuildTools/Package.sh`, and 064.

- **Inputs.** `<Map>.xodr` and `<Map>.fbx`, with the same name. The OpenDRIVE is mandatory:
  waypoints, spawn points, the Traffic Manager's lanes and the traffic lights all come from
  it, not from the meshes.
- **Mesh names.** These set the semantic tags: `Road_Road`, `Road_Marking`, `Road_Sidewalk`,
  `Road_Curb`, `Road_Grass`, `Building_*`, `Prop_*`. Lights and signs stay out of the FBX;
  CARLA spawns its own from the OpenDRIVE signals.
- **The FBX frame.** Blender's default export is imported by UE in centimetres with y
  mirrored, which is CARLA's convention (064, verified by ray cast).
- **The import package.** Copy `Import/<Package>/` into CARLA's source tree. It holds
  `<Package>.json` (`maps`: `name`, `source`, `xodr`, `use_carla_materials`; `props`),
  `<Map>/<Map>.fbx`, `<Map>.xodr` and the textures. A large map adds tiles named
  `<Map>_Tile_<x>_<y>.fbx`, plus `tile_size` (2 km by default) and `tiles`.
- **Import.** `make import` (`Util/BuildTools/Import.sh --package <P>`) runs in a CARLA
  source build, i.e. the UE 4.26 editor. It writes `Unreal/CarlaUE4/Content/<P>/`. Editor
  work comes after it: materials, the road painter, props, the default weather, the
  pedestrian navmesh.
- **Package.** `make package ARGS="--packages=<P>"` cooks `Dist/<P>_<tag>.tar.gz`. It holds
  `CarlaUE4/Content/<P>/` (the cooked map), the `<Map>.xodr` and `<Map>.bin` (navigation) at
  their content paths, the package config, and the tagged materials for semantic
  segmentation. Shalun took 305 s and made 251 MB.
- **Install.** Put the tarball into a stock binary's `Import/` and run `./ImportAssets.sh`.
  No UE build is needed on the user's side (064). Then `config.py --map <Map>` or the
  bridge's `load_world`.
- **Without a source build.** CARLA's Docker cook needs about 4 h and 600-700 GB to build its
  image once. 139 (`/mnt/wd/aeon/carla-src`) and 241 (`/mnt/toshiba/aeon/carla-src`) have
  editor builds already. 133 has no CARLA, and only about 38 GB free on its NVMe.
- **Two traps.** A malformed OpenDRIVE can segfault the map builder (Shalun's signal
  controllers did), so check every `.xodr` with `carla.Map(name, xodr)` on the client before
  importing it. In 0.10 the package has no `Import/`, and custom maps need a source build.

## The pipeline: 059's tools, CARLA's outputs

The data stays 059's: the 167 re-tiled LAZ tiles (NAS `tiles_50m_mgrs/`, 184 GB), the
transients removed (N2), the DTM, the trees, and the merged, re-registered Lanelet2. The
frame stays 059's map frame (MGRS 51RUH local). CARLA's world is the OpenDRIVE frame with y
negated, and the OpenDRIVE is written about `MAP_ORIGIN` (52500, 67700), so float32 keeps
millimetres. Its geoReference gives CARLA's GNSS the real latitude and longitude.

| Step | Reused from 059 | New or changed for CARLA |
|---|---|---|
| C1 Ground model | `ground_ortho.py` passes 1-2, `smooth_dtm.py` (`dtm.npy`) | none |
| C2 Ground texture | `ground_ortho.py` pass 3, `ortho_refill.py` (no block paint) | per-class texture sets if the road is split from the rest (C3) |
| C3 Ground mesh | `build_meshes.py`'s ground grid | split by class and named: `Road_Road` under the lanelets (the lane raster of `world_usd.py`'s `RoadClear`), `Road_Sidewalk` and `Road_Grass` from the scan's colour and height; a coarser grid away from the roads; UVs onto each tile's ortho |
| C4 Buildings and objects | `build_meshes.py`, `transients.py`, the road clearing, the colours delit (`retouch_tile`'s rule) | decimated to a triangle budget (UE 4.26 has no Nanite: 70 M triangles will not do), named `Building_*` and `Prop_*`, collision on buildings only, a master material that reads vertex colour |
| C5 Trees | `trees.py` (trunks, crowns, heights) | UE foliage with cut-out leaves at the trunks (CARLA's vegetation), or the scan's crowns with a masked material; chosen by camera and by NDT (CARLA tree assets wrecked NDT in 059's n5a) |
| C6 Road network | `register_lanelets.py` (z, widened curves, kept centrelines) | Lanelet2 to OpenDRIVE, checked with `carla.Map` and `scripts/lanelet_opendrive_registration.py` (offsets between lanelet and lane centres); signals later (r01 and r02 have none; 062 has the city's) |
| C7 FBX and package | `svl_to_carla.py`'s FBX writer (pip `bpy`) and package JSON | per-class meshes over all tiles, in the map frame less `MAP_ORIGIN` |
| C8 Import, package, install | 064's commands | on 139; then `ImportAssets.sh` into the 0.9.16 binary that will run it |
| C9 Autoware map dir | `export_map.py` (the scan's PCD tiles, projector), `register_lanelets.py`'s Lanelet2 | the signal table from the OpenDRIVE, once there are lights |
| C10 Picture | `render_probe.py`'s fixed views, N10's findings | a CARLA probe at the same views; the RGB camera's `exposure_mode=histogram`; a default weather (sun, clouds) for the map |
| C11 Drive | 059 N8's routes and goals | Autoware on acb's ego on r01 and r02: NDT on the scan's PCD against CARLA's raycast lidar on the decimated meshes |

The CARLA-only tools go in `scripts/ntu_carla/` and import what they need from
`scripts/ntu_twin/`. A change to an `ntu_twin` file is told to swb 118, which owns 059's
Isaac world.

## Rendering: what carries over from 059 N10

- **Ground refill and mesh de-lighting.** These change the assets, so they carry over as they
  are.
- **Sun, sky and exposure.** These are CARLA's weather (sun angle, clouds, fog, wetness) and
  the RGB camera's attributes: histogram auto-exposure, exposure compensation, bloom, lens
  flare, gamma.
- **Light through the crowns.** This is UE's masked foliage material. Shadows come from
  cascaded shadow maps plus distance-field and screen-space AO. There is no ray tracing:
  UE 4.26 ray tracing needs DX12, and CARLA on Linux renders with Vulkan.
- **A neural layer.** 0.9.16 renders NVIDIA NuRec scenes (USDZ) through an external gRPC
  container, as cameras next to CARLA's own. Our own reconstruction needs the vendor's
  posed photos, the same blocker as 059's stage B.
- **Out:**
  - CARLA 0.10's Lumen and Nanite: no custom-map import in its package, Chaos vehicles,
    about twice the VRAM.
  - Post-render translation (Cosmos-Transfer, about 65 GB).

Expected: a clean simulation picture about as good as 059's after N10, with better trees,
and lit without ray tracing.

## Work items

1. **Pilot: r01's corridor** (about 30 tiles).
   - C3 to C7 for those tiles, and C6 for r01's lanelets.
   - Import, package and install on 139.
   - A camera at 059's four r01 probe views, beside Isaac's pictures.
   - Measure: the import's time and memory, the package's size, and the frame time with the
     ego's sensors.
2. **The triangle budget.** Decimation per class against the frame time and the lidar's
   NDT score. Keep the road's ground exact; let buildings and objects go first.
3. **Trees.** Foliage assets against the scan's crowns, on camera and on NDT.
4. **The whole site, as one standard map.** At 1.1 x 0.65 km it fits under one 2 km tile, so
   no large-map streaming is needed unless memory says otherwise.
5. **Autoware on CARLA.**
   - r01 and r02 driven, with the signal table.
   - A scenario through the bridge: `load_world` on the new town, the map dir named in
     the scenario.
6. **Distribution.**
   - The tarball and the Autoware map dir on the NAS, with a README (install, frame,
     version).
   - Check the install on a clean 0.9.16 binary on a host that never built it.

## Risks

- **Vertex colours.** UE's FBX import keeps them, but CARLA's tag-based import assigns its
  own materials. We need a master material that reads vertex colour, made once in the editor
  and shipped in the package. Unverified.
- **Lanelet2 to OpenDRIVE.** At junctions and lane splits the converted lanes may not match
  the lanelets. SSv2 matches poses on Lanelet2 while CARLA drives OpenDRIVE, so measure the
  offsets.
- **Import memory.** UE's import of many large meshes can run out of memory
  (`--batch-size`).
- **Where it runs.**
  - 133 has the data but no CARLA and little disk.
  - 139 has the editor; its `/home` and `/mnt/wd` are spinning disks.
  - Running on 133 needs the 0.9.16 binary (about 20 GB) on its NVMe.
