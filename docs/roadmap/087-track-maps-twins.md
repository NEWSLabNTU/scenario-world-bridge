# 087: Track D: Maps and Digital Twins

The places scenarios run in: the NTU campus (scan, Isaac, CARLA), its roads from public data and its off-road field, SVL's Shalun map, maps for any scenario, and the CARLA 1.0 towns' Autoware maps.

**Status**: consolidated 2026-11-18 from 019, 059, 072, 062, 040, 064, 031, 083.

**Research rule** (user, 2026-11-18): a research track does not wait for scensim releases.
Its work runs on current scensim main, and it moves to each new scensim release when that
release is published (rebuild, rerun the track's reference runs), as a deliberate step.

**Consolidated** 2026-11-18 (user: "Have A-D"): the phases below keep their own documents as
history and detail; this document is the track's plan and status from now on, and new work
is planned here.

## Parts

| phase | what | status | left |
|---|---|---|---|
| [019](019-ntu-campus-twin.md) | NTU campus twin from the scan | see doc | (feeds 059 and 072) |
| [059](059-ntu-twin-on-isaac.md) | the NTU twin on Isaac Sim | in progress, on 133 (/mnt/p5plus/aeon/ntu-twin) | stalled since 2026-10-30 |
| [072](072-ntu-campus-in-carla.md) | the NTU campus as a CARLA map | in progress: r01's corridor in a stock 0.9.16 | retarget to CARLA 1.0 content packs |
| [062](062-ntu-roads-from-open-data.md) | roads around NTU from public data, audited | in progress: O1-O3, O5 done; O6 drives pass on Isaac | O4's paint audit |
| [040](040-ntu-offroad-field.md) | NTU off-road field | planned |  |
| [064](064-svl-shalun-map-import.md) | SVL's Shalun map in Autoware, CARLA and Isaac | in progress: CARLA imported, packaged, driven |  |
| [031](031-maps-for-scenarios.md) | maps for any scenario | deferred 2026-10-13 |  |
| [083](083-carla-1-0.md) | the CARLA 1.0 towns' Autoware maps | new: Town01_Opt's road file equals 0.9.16's; CARLA 1.0 ships an Autoware map generator | regenerate or check every _Opt town |

## Folded in

- CARLA 1.0's content packs install maps without rebuilding the package, and its map generator writes lanelet2 with stop signs and light links: 072 and 064's CARLA side should target 1.0, not 0.9.16.
- The published map dataset's token fix and cards wait for the refactor (user, 2026-11-17).

## Next: the CARLA 1.0 map generator study

What it takes to produce a CARLA 1.0 map and its Autoware map (lanelet2, point cloud,
`map_projector_info`) in one frame, from our pipeline. Planned 2026-11-19 (241; manager's calls the same day).

**What exists.**
- **OpenDRIVE to lanelet2:** acb's `carla_map_gen` (crdesigner 0.8.5 plus its postprocess) and the
  `scripts/repair_lanelet_*` fixes (light links, right of way, speed limits, turn directions, and more),
  with `lanelet_opendrive_registration.py` to measure the fit.
- **The point cloud:** acb's `carla_pcd_gen` (lidars at the spawn points), `densify_pcd_along_lanes.py`
  (lidars along every lane), and sampling from meshes (`open_maps/world.py`).
- **CARLA 1.0's own Autoware map tools** (`PythonAPI/examples/av_stacks/autoware/map_tools`) do the
  same: crdesigner 0.8.5, stop signs and lights from the live actors, and a noise-free 64-beam lidar
  along the waypoints. They overlap ours; the stop signs are their one addition.

**What CARLA 1.0 needs.**
- **A road-only map:** OpenDRIVE standalone mode (`generate_opendrive_world`) builds one from an
  `.xodr` on the packaged server, with no editor.
- **A map with buildings:** `Util/Tools/Import.py`, `carla-pack` and the Digital Twin Tool all need a
  CARLA UE 5.8 source build with the editor, which no host has (about 200 GB). The content pack it
  makes installs into the packaged server of the same release.

**Gaps.**
- Our CARLA-client tools are on 0.9.16's API.
- Only 072's `lanelet_to_xodr` writes an `.xodr` from our own data, and it carries no signals; open_maps
  writes none.
- The towns' frame (Local, or TM at 0,0) differs from the real places' (MGRS).
- `autoware_lanelet2_map_validator` has not been run on our maps, nor point-cloud coverage against the
  meshes.

**Steps.**
1. **S1, Town01_Opt, no editor** (host 241, or 139 after 084e).
   - Port the CARLA-client point-cloud tools in acb under its CARLA version feature, so 0.9.16 keeps
     working.
   - Generate Town01_Opt's Autoware map with CARLA's map_tools and with ours.
   - Compare both with `carla-town-autoware-maps`' Town01: registration offsets, the validator, and the
     point cloud against the meshes.
   - Drive Autoware on each. This decides 083's decision 2 (replace or feed), and whether the `_Opt`
     towns need new maps.
2. **S2, an OpenDRIVE of our own, no editor.**
   - A private map's `.xodr` through `generate_opendrive_world` on packaged 1.0, then the same
     generation, checks and drive. This proves the road-only path end to end.
   - Private data: its results go to the NAS (and `2026-taiwan-testbenches` if needed), never into
     this repository.
3. **S3, the editor build** (UE 5.8 plus CARLA `ue58-dev`), only if S1 and S2 show the need, and the
   user's call with their numbers.
   - Import with `Import.py`, package with `carla-pack`, install into the packaged server, drive.
   - Then 072's campus as a content pack.
4. **S4, one tool:** 031's `map_build`, from an `.xodr` (and meshes) to a CARLA pack and the Autoware
   map directory, with the gates (validator, registration, point-cloud coverage, a drive), per CARLA
   version.

Ahead of 059's Isaac twin (manager, 2026-11-19).

**S1 and S2 results** (241, 2026-11-19; NAS `logs/2026 SSv2 on CARLA/2026-11-19 087 S1 CARLA 1.0 map generator
Town01_Opt (241)/`; S2's report is private).

- **S3, the editor build, is not needed so far.** OpenDRIVE standalone mode on the packaged server builds a
  road-only map from an `.xodr` in about 2 s, with its traffic lights and stop signs as actors.
- **Town01_Opt:**
  - its `.xodr` is byte-identical to 0.9.16's `Town01`;
  - acb's `carla_map_gen` puts lanelets on the OpenDRIVE lanes within 0.7 cm (p95), against 16 cm for our current
    Town01 map;
  - acb's `carla_pcd_gen` runs on 1.0 as built with `--features carla-10`.
- **The validator** (`autoware_lanelet2_map_validator`, from autoware_tools 0.2.0; 0.3.0 no longer has it): no
  generator passes it as generated.
  - Our current map has 36 errors (a light's ref_line is not a stop_line).
  - `carla_map_gen`, raw, has 126; CARLA's map_tools have 108 (local coordinates, light facing).
- **CARLA's map_tools have two defects.**
  - On an asynchronous server the point-cloud tool's `--tick` does nothing, so each lidar measurement is one server
    frame: 170k points for Town01_Opt instead of 3.9 M with the world set synchronous at 0.1 s.
  - The tools write `projector_type: Local`, which SSv2 rejects.
- **`carla_map_gen` is town-only.** It writes TransverseMercator at 0,0 and ignores the `.xodr`'s georeference, and
  a real place's georeference can carry a false origin that Autoware's TransverseMercator cannot hold.
- **Next (manager, 2026-11-19).**
  - In acb: the lanelet repairs move into its map tools, and `carla_map_gen` carries the georeference (MGRS, or an
    origin shift) and runs the repairs by default. The target is 0 validator errors on Town01_Opt.
  - Then an Autoware drive on Town01_Opt with each map, on a host with a scensim stack on 1.0.

**Town projectors at CARLA's scale** (241, 2026-11-19, as its own commit, landed after 243's T3 CARLA 1.0
comparison). CARLA georeferences its towns as `+proj=tmerc +lat_0=0 +lon_0=0 +k=1`. Our towns'
`map_projector_info.yaml` named no `scale_factor`, so Autoware and SSv2 took 0.9996, and the map was 0.04 % small
against CARLA: 0.13 m at 330 m, Town01's lanelets 0.159 m (p95) off their OpenDRIVE lanes. `download_maps.sh` now
declares `scale_factor: 1.0` on every TransverseMercator town at 0,0 that names none (p95 0.007 m on Town01, 0.005 m on
Town05), and `gen_npc_benchmark.py` reads the map's scale. **Every T3 reference recorded before this commit was on the
0.9996 maps**: positions near a town's far edge move by up to 0.13 m.

