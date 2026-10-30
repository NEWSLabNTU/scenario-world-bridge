# Phase 059: The NTU Campus Twin on Isaac Sim, from the Coloured Point Cloud

The NTU campus as an Isaac world in which a driven car's cameras and lidar see the real site.
The aim is evaluating autopilots in a digital twin under scenarios: Autoware, and
camera-driven stacks such as Autoware Foundation's Vision Pilot or a VLM agent. The camera
image is what those stacks consume, so realism of the rendering is the requirement, not only
geometry. The first version starts from the vendor's coloured MLS point cloud alone.

**Status**: in progress. Moved from newslab-241 to newslab-server133 on 2026-10-30 (241's GPUs
are held by another user's vLLM; the handover is `HANDOVER.md` in the NAS's
`autoveh/dataset/2026-10-30 NTU twin map/`). On 133 the work lives on NVMe in
`/mnt/p5plus/aeon/ntu-twin/` (tool venv `.venv`: laspy + lazrs, open3d 0.20, pyproj, scipy,
matplotlib), Isaac Sim 6.1 in `/mnt/p5plus/aeon/isaac-sim/`. 241's copy was
`/mnt/toshiba/aeon/ntu-map/`.

Progress:

- N1 done: the full LAZ re-tiled by `scripts/ntu_twin/retile_laz.py` into 167 tiles of 50 m
  (`tiles_50m_mgrs/`, 175 GB), 5 523 612 787 points (the true count, as in 019), 70 min with
  8 workers on the spinning disk.
- N3, first pass (`scripts/ntu_twin/ground_ortho.py`, `ground/`): a site-wide zmin grid at
  0.25 m (23 min), an SMRF-like ground filter (windows 1-16 m: 2.29 M of 3.11 M occupied
  cells are ground) and a DTM; RGB and intensity orthophotos at 2 cm per tile with a
  pyramid fill and a coverage mask (167 tiles, 23 min, 270 MB). Lane lines, arrows, curbs,
  paving and manholes are sharp. Parked scooters and cars are baked into the ground where
  they stood; holes under trees and hedges are filled from coarser levels.
- N4, N5 (first version), N6: `scripts/ntu_twin/build_meshes.py` meshed all 167 tiles
  (`meshes/`, 2.5 GB): the DTM grid per tile as the ground, UVs into its orthophoto;
  screened Poisson (0.08 m voxels, depth 11, at most 400 k triangles per layer) on the
  building class and on everything else above the ground, trees as blobs. Two workers, about
  3 h in all. A tile is read in 10 M-point chunks and thinned to one point per voxel as it
  is read: reading the densest tiles (~180 M points) whole took ~35 GB per worker and, with
  other work on the host, ran it out of memory twice; chunked, the densest peaks at 10.8 GB.
  A rerun skips tiles with a `stats_<ix>_<iy>.json`. `scripts/ntu_twin/world_usd.py` (Isaac's
  Python) wrote `map/isaac/`: one payload per tile, 358 meshes, 70.5 M triangles, 1.3 GB, the
  world frame the map frame less `map_origin` (52 500, 67 700, 0), bounds about 1.1 km x
  0.65 km. Not yet opened in Isaac (no free GPU).
- N7, first version: `scripts/ntu_twin/export_map.py` wrote `map/`: r01 and r02's Lanelet2
  merged (r01's ids offset by 1 000 000; 285 lanelets load), r02's projector info (MGRS
  51RUH), and the divided point-cloud map, 167 PCD tiles of 50 m (0.2 m voxel means, shift
  applied, 39.1 M points, 597 MB) with `pointcloud_map_metadata.yaml`. Lanelet nodes over
  ground cells sit a median 0.16 m above the ground mesh (5th-95th percentile -0.12 to
  +0.87 m): the routes' own PCDs drift against the scan (019); re-registering the lanelets is
  still open.
- N6 checked in Isaac (133, 2026-10-30; `scripts/ntu_twin/world_probe.py`, headless, RTX 5090,
  spawned on r02's lanelet 7 at world (-282.58, -45.02), ground 11.8 m). Isaac boots in 11-13 s,
  and all 358 meshes (70.5 M triangles) are on the stage within 5 s. The first frame takes
  4.5 s cold and 85 ms warm. VRAM is 13.1 GB with three 1280x720 cameras and an OS1 lidar.
  Frames take 13.7 ms median (17.2 ms p95) with camera and lidar.
  The orthophotos show; nothing renders black. The lidar hits the meshes: 111 k points per
  frame, range 3.3 / 12.1 / 51.4 m (p5 / p50 / p95), 59 % of returns more than 0.3 m above the
  ground. The first USD wrote the scan's sRGB colours into displayColor, which RTX reads as
  linear, so trees and trunks rendered near-white. The meshes also had no normals and rendered
  faceted. `world_usd.py` now converts the colours to linear and writes area-weighted vertex
  normals; the rebuild takes 22 s.
  Still visible: crown tops are pale and blue (the MLS scan colours the canopy from below, so
  edge points take sky pixels), Poisson shards float over the road, parked cars are meshed
  (N2), the ortho's baked shadows double RTX's own, and lidar intensity is flat (no
  non-visual materials).
- N7 lanelets re-registered (133, 2026-10-30; `scripts/ntu_twin/register_lanelets.py`, 3.5 min).
  Each route's own point cloud is registered to the scan (the divided PCD map) by point-to-plane
  ICP, globally and then per anchor on a 25 m grid, and the lanelet nodes follow a
  Gaussian-weighted field of the anchors' displacements.
  - r02: 137 anchors (37 dropped), rmse 0.10 m; anchor xy corrections p50 0.15 m, max 0.72 m.
  - r01: 193 anchors (64 dropped), rmse 0.11 m; anchor xy corrections p50 0.12 m, max 0.43 m.
  - Nodes moved 0.13 m (r02) and 0.12 m (r01) at p50, 0.35 m at most.
  - Heights: 2717 of 2733 nodes now sit on the meshed ground; they were a median 0.16 m
    above it (p95 0.87 m).
  - lat/lon move with local_x/local_y. The original is kept as `lanelet2_map.orig.osm`.
  - The lane-paint check (intensity ortho under the line strings) does not confirm the xy
    correction: 3.0 % of samples on the brightest decile before, 2.2 % after. Both are below
    chance, so these boundaries are mostly road edges, not paint, and the check does not fit
    these maps. The xy evidence is the ICP residual alone.
  - Traffic lights, stop lines and crosswalks are still to add.
- N8, Autoware on the Isaac ego (133, 2026-10-30): aib's `just isaac-ego` on the world and
  `just autoware map_path:=<map> pointcloud_map_file:=pointcloud_map`, both on ROS domain 41
  with Cyclone on localhost. The pose is initialized and the route set through the AD API
  (`/api/localization/initialize` with a map pose, `set_route_points`, `change_to_autonomous`).
  Truth comes from aib's GNSS, inverted (`scripts/ntu_twin/ndt_track.py`). Its along-track
  error includes the fix's 10 Hz sampling, about 0.15 m at 3 m/s.
  - r02, lanelet 7 to lanelet 49: 348 m, arrived. NDT error while moving p50 0.16 m, p95
    0.29 m, max 0.39 m; across-track p95 0.11 m. NVTL never below 3.08. AEB braked four or
    five times on the way (on the raw lidar: Poisson shards and low crowns), and each time
    the drive went on.
  - r01, from 10 m into lanelet 1004948 towards 1004936: 204 m driven. NDT error while
    moving p50 0.08 m, p95 0.25 m; NVTL never below 2.87. The drive stopped at (52675.3,
    67886.2) on an MRM, `control_validator: control_validation_latency`. The host was at
    load 41-47 on 32 cores with another GPU job (6.5 GB) beside Isaac, and the GPU was at
    98 %. Rerun it on a quiet host.
  - At standstill there the car crept sideways about 1.5 cm/s. A 1 m pit lies 1-3 m to its
    north-east (10.95 m against 11.9 m), a drain or an artefact; not looked at yet.
  - The world needed four fixes on the way, each found by a drive:
    1. The "other" meshes (Poisson shards, tree blobs, parked cars) had triangle colliders.
       A shard over the road held the car on r02 with acceleration commanded. Now only the
       ground and the buildings collide (`world_usd.py`).
    2. The ground mesh had holes wherever a non-ground patch lay more than 2 m from ground:
       2142 enclosed holes, the largest under 2000 m2. `build_meshes.py` now fills every
       enclosed hole (`ground_keep`), and `--ground-only` rewrites only the ground without
       the LAZ tiles.
    3. The DTM's fill took heights from non-ground points (parked scooters, planters), and
       only 17.8 % of the site's cells are ground. On r01, 0.2-0.3 m bumps in cells with no
       points at all held the car at creep torque. `scripts/ntu_twin/smooth_dtm.py` caps
       each non-ground cell at the ground-only estimate (multi-scale normalized Gaussian)
       plus 5 cm, then takes a 5x5 median and a 1-cell Gaussian over the whole grid. Road
       roughness (|z - 3x3 median|, p99 within 3 m of lanelet nodes) went from 0.058 m to
       0.010 m. The raw DTM is kept as `dtm_raw.npy`. The ortho was made with the raw DTM,
       which changes nothing on the ground cells.
    4. Autoware 1.5's PCD loader reads the metadata's tile corners as integers, so
       `export_map.py` writes them floored (yaml-cpp "bad conversion" otherwise).
  - Open:
    - aib's GNSS projects about (0, 0) (`aib.frames.lat_lon`), not the map's MGRS
      projector. `autoware_sim_pilot`'s `auto_drive` always re-initializes from GNSS, so it
      put the ego at (33696, 99955) and cannot drive this map until aib's GNSS follows the
      map's projector and `map_origin`.
    - Spawning on r01's first lanelet exactly (world (314.25, 105.19), yaw 1.5773) drops the
      car through the world within a second. The same place with yaw 0 or pi holds, as do
      3 m south, 10 m north, and r02's spot at yaw pi/2, and a box dropped there rests.
      There are only ground triangles within the car. Not explained.
    - The ego spawned at r02's lanelet centre came to rest 0.8 m south of it. Not explained.

**Source**: user direction 2026-10-29: "load our colorful NTU map into Isaac Sim ... drive
vehicles on the map and the camera and lidar sensors receive realistic data as though it runs
in real world. We also need lane info ... evaluate the autopilot in the NTU map as the digital
twin under various scenarios with perception stack like VisionPilot or VLMs. That's why I
required realistic rendering." Then: "suppose that we only start with colored point cloud"
(the user is asking the vendor for the photos, which would enable stage B below).

**Research**: [survey-ntu-twin-rendering](../design/survey-ntu-twin-rendering.md) (2026-10-29, web research), which updates the
2026-10-10 survey ([survey-isaac-sim-and-digital-twin](../design/survey-isaac-sim-and-digital-twin.md)).

**Depends on**: [019](019-ntu-campus-twin.md) (data inventory, the LAZ truncation, the frame
chain and the measured shift, `scripts/ntu_twin/`), [037](037-isaac-world-server.md) and
[038](038-isaac-vehicle-bridge.md) (the Isaac world and ego with RTX sensors),
[056](056-scenario-surface-maps.md) (surfaces). Supersedes [040](040-ntu-offroad-field.md)'s
wait for a vendor mesh: we build the mesh ourselves.

## Data

| What | Where (local copy) | Notes |
|---|---|---|
| Vendor MLS scan | `ntu-map/2026-05-15 NTU pointcloud map/台大las.laz` (51 GB) | ~5.52 G points, RGB, GPS time, ASPRS classes, EPSG:3826. LAS 1.2: the header count saturates at 2^32 - 1, so readers that trust it lose ~22 % (019). Only this file is complete |
| Vendor derived tiles | `tiles_50m/`, `tiles_100m/`, `downsample/` | Truncated at 4.29 G points (019). Kept for reference, not used |
| Trajectory and poses | `軌跡姿態資料.zip` | `.traj`, `.gnsstraj`, `.imu` per session; one `.imglist`: 2231 posed images (2026-05-07, backpack platform, EPSG:3826), images not delivered |
| Autoware maps | `ntu-map/2026-04_ntu_map/r01`, `r02` | Lanelet2 (155 and 130 road lanelets, thin solid lines only, no regulatory elements) and PCD (x y z intensity, MGRS 51RUH) |
| OpenDRIVE | `ntu-map/2026-04_ntu_map/converted_map/r01.xodr`, `r02.xodr` | Converted from the Lanelet2 maps |

Frame chain (019): EPSG:3826 to EPSG:32651, easting and northing mod 100 000 (MGRS 51RUH
local), then the vendor-to-map shift, measured about (-0.54, +0.20, +17.30) m for both routes
(per-route maps drift up to 0.3 m against the scan).

## Approach

What the research found (details and sources in the survey):

- **A textured mesh is the one representation both RTX camera and RTX lidar reliably see.**
  Isaac Sim 6.x renders 3D Gaussian splats (`ParticleField3DGaussianSplat`) for cameras, but
  no NVIDIA document says RTX lidar hits them; plan on lidar seeing meshes only.
- **Without images, realism comes from the scan's own colour.** The ground as a heightfield
  with an orthophoto from point RGB at 1-2 cm is the strongest surface we have; buildings
  meshed per tile; trees, half the points, are the hard case.
- **NuRec** now renders lidar in its own renderer and trains camera plus lidar, but wants
  more than 24 GB (48 recommended), does not list the RTX 5090, and needs images. Not for
  stage A.

Stages:

- **A (this phase): the geometry twin from the point cloud.** Mesh, orthophoto, materials,
  lanes; RTX camera and lidar on it; Autoware localizes and drives; Vision Pilot and a VLM
  agent see it.
- **B (when the photos arrive): a gaussian camera layer.** 3DGUT or gsplat per 100-200 m
  segment, initialised from the scan, exported as ParticleField, the stage A mesh kept
  camera-invisible for lidar, collision and shadows.
- **C (optional): NuRec on one corridor; Cosmos-Transfer offline.**

## Work items (stage A)

**N1. The full scan, tiled.** Re-tile the complete LAZ with `scripts/ntu_twin/retile_laz.py`
into 50 m MGRS-local tiles (as 019 did on another host). Check the summed count against
5 523 612 787.

**N2. Classes.** Re-classify ground (CSF or SMRF) and split the 40 % unclassified points:
poles, signs, fences, parked cars, bikes, people. Remove the dynamic ones (parked cars and
people are frozen ghosts to both sensors), using class, GPS-time consistency between
passes, or a segmenter.

**N3. The ground.** A heightfield mesh (DTM at 5-10 cm, tiled) and an RGB orthophoto at
2 cm, plus an intensity orthophoto; lane paint as separate prims from intensity, so RTX lidar
sees its reflectivity.

**N4. Buildings and structures.** Poisson or TSDF per tile, decimated, colour baked to
texture; poles and signs as their own meshes.

**N5. Vegetation.** First version: trees as coarse proxy meshes for lidar and collision, the
camera seeing point-initialised splats or textured crowns; then trees replaced by assets at
detected trunks. Compare both on camera.

**N6. The USD world.** Tiled payloads in the map frame, materials by class (SimReady
non-visual materials for lidar intensity), colliders on ground and buildings, an
`isaac/world.yaml` for swb_isaac_world, and `surfaces.yaml`.

**N7. Maps.** One site map dir: `pointcloud_map.pcd` from the tiles (0.2 m, shift applied,
so NDT and the world agree by construction), r01 and r02's Lanelet2 merged and re-registered
to it, with traffic lights, stop lines and crosswalks added where the scenarios need them.

**N8. Drive it.** Autoware on the Isaac ego: NDT on the RTX lidar (NVTL along r01 and r02), a
route driven; SSv2 scenarios with NPCs from the catalog.

**N9. Camera stacks.** Vision Pilot and a VLM agent on the ego's camera, open loop first on
recorded drives, then closed loop. Record what they get wrong that they do not get wrong on
real footage, if we can record some.

## Risks

- Trees look fake to a VLM; baked shadows in the scan's colour double RTX's own.
- Texture budget (~1.8 G texels at 2 cm) needs tiles and streaming.
- RTX lidar has crashed on multi-GPU setups: pin Isaac to one GPU.
- GPU: both RTX 5090s on 241 are held by another user's vLLM as of 2026-10-29, so the work
  moved to 133. On 133 another user runs a CUDA MPS daemon (`/tmp/nvidia-mps`), and our CUDA
  fails with 805 (MPS_CONNECTION_FAILED) unless `CUDA_MPS_PIPE_DIRECTORY` points at an empty
  private directory. That bypasses MPS; it is set in 133's `.env`.
- Disk on 133: `/home` is a full spinning disk; everything lives on `/mnt/p5plus` (NVMe).
- RAM on 133 is 60 GB: run build_meshes with at most 2 workers (10.8 GB peak each).
