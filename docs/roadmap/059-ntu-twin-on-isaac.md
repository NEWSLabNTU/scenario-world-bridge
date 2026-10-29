# Phase 059: The NTU Campus Twin on Isaac Sim, from the Coloured Point Cloud

The NTU campus as an Isaac world in which a driven car's cameras and lidar see the real site.
The aim is evaluating autopilots in a digital twin under scenarios: Autoware, and
camera-driven stacks such as Autoware Foundation's Vision Pilot or a VLM agent. The camera
image is what those stacks consume, so realism of the rendering is the requirement, not only
geometry. The first version starts from the vendor's coloured MLS point cloud alone.

**Status**: in progress, 2026-10-29. Local copy in `/mnt/toshiba/aeon/ntu-map/` (tool venv
`.venv`: laspy + lazrs, open3d, pyproj, scipy, matplotlib).

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
- N4, prototype: Poisson on the vendor's building class (6) of one tile (0.08 m voxels,
  depth 11, decimated to 300 k triangles, 23 s): facades with windows, patchy roofs.

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
- GPU: both RTX 5090s on 241 are held by another user's vLLM as of 2026-10-29; offline
  steps N1-N5 and N7 do not need them.
- Disk: the copy is on a spinning disk (`/mnt/toshiba`); the NVMe volumes have no directory
  we can write to.
