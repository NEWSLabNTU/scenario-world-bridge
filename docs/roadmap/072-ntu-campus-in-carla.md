# Phase 072: The NTU Campus as a CARLA 0.9.16 Map, Built from the Scan

The NTU campus as a CARLA map package that any stock CARLA 0.9.16 binary can install: its
roads in OpenDRIVE, its ground, buildings and trees as meshes, built straight from the
vendor's MLS scan and the Autoware maps. It is not a conversion of 059's Isaac world. 059's
tools are reused as building blocks, and their outputs are shaped for CARLA from the start.

**Status**: planned, 2026-11-03.

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
