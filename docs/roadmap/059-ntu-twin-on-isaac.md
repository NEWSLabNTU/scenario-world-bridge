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
  - GNSS (aib 8cf6a9a): aib's fix was projected about (0, 0), and `auto_drive`, which always
    re-initializes from GNSS, put the ego at (33696, 99955). Now `just isaac-ego
    world:=<map>/isaac/world.yaml` projects the fix as the map does: MGRS 51RUH about
    `map_origin`, pure Python, within 0.4 mm of the Lanelet2 maps' own lat/lon. Autoware's
    `gnss_poser` lands 1.2 m from the spawn (the antenna lever arm, turned without a
    heading). `auto_drive`, unmodified, re-initialized at (52814.23, 67815.23), 4 cm from the
    spawn, set the route and drove.
  - r01 again (auto_drive, with lockstep): 203 m, NDT error while moving p50 0.10 m, p95
    0.28 m; NVTL never below 2.89. It stopped at (52675.9, 67885.9), the place the first r01
    drive stopped. Recording the diagnostics showed the order: AEB's emergency brake first,
    then the MRM, then a single 0.050 s `control_validation_latency` sample. The latency
    error is the MRM's own stop command arriving a sim tick late (`nominal_latency` is
    0.01 s; /clock steps 50 ms). It is a consequence, not the cause; in 106 samples while
    driving the latency was exactly 0.
  - The cause is the world. The r01 lanelet there (1004849) is 2.17 m wide for a 1.9 m car,
    while the paved ground is 10 m wide. Pieces of the "other" mesh (vegetation, Poisson
    geometry reaching to 5 cm above the road) stand 0.15 m beyond its north boundary, and
    AEB brakes on them. N2/N5 (objects and trees) and a wider lanelet, if the road has one,
    are the fixes.
  - Lockstep (aib's `lockstep` in the ego config: each frame waits for NDT's pose of the
    last scan) does not change this. It keeps the stack in sync, but NDT skips scans: about
    24 of every ~150 frames waited out the 1 s timeout, a mean of 150-210 ms a frame, so the
    world ran about 4x slower than the wall clock.
  - r01's stop at (52676, 67886), taken apart over five more drives (2026-10-30 evening):
    1. Standing "other" geometry beside the 2.17 m lanelet (AEB). Fixed:
       `world_usd.py --clear-road` drops other triangles within 0.5 m of a lanelet and below
       2.5 m (40 k triangles).
    2. A drain pit at the road edge, ground-classified 2 m below the road (the scan saw
       through the grate). Fixed: `smooth_dtm.py` closes narrow pits, harder on and within
       3 m of the lanes (`--lanes`). Lane cells more than 0.1 m below their surroundings went
       from 4328 to 305.
    3. control_validator's 0.01 s latency bound, which a 50 ms /clock step can never meet. It
       fired first in one run. Fixed: autoware_sim_vehicle e488ab0, `nominal_latency` 0.1 s in
       the profile.
    4. A process leak of mine: `auto_drive` runs as `auto_drive` under `ros2`, so cleanup
       that matched `python3` left every pilot running. Three pilots re-initialized from GNSS
       in one drive (a 7.7 m jump, "Duplicated nodes detected [/auto_drive]"), and r01j's
       stall at initialization was probably the same. The N8 runs now stop through a helper
       that matches a process's own argv only.
    5. `obstacle_stop` stopped the car for a perception object (UNKNOWN, existence 0.999) at
       (52668.71, 67885.47), z 13.28, 1.3 m above the ground. The world had an object there: the
       scan (tile 1053_1357, 2026-10-31) holds 6960 points at 0.3-1.72 m above the ground,
       centred 2.5 m north of the detection, about 1.4 x 0.8 m, and the "other" mesh follows
       them; nothing between 0.3 and 6 m within 1.5 m of the detection itself (canopy above).
       Whether it is a transient is not settled. It is in both survey days (165 points on the first
       day, 6795 about 50 h later); in the long second pass the ground within 3 m returned
       7900 points from +120 to +240 s with 7 of the object's, then 6348 of the object's at
       +320 s. A two-wheeler parked on both days, or occlusion early in the pass, fit better
       than a passer-by. (An earlier note here called it people or scooters that came and went;
       that was the same evidence read too far.) Worked around:
       `world_usd.py --clear-road --clear-margin 2.5` (the default), which drops it and any such
       object within 2.5 m of a lanelet, real roadside shrubs too.
       For N2: the survey's two days are of unequal density here (0.5 M against 24.8 M points
       in 40 x 40 m), so cross-day consistency has little to compare, and within a pass a voxel's
       share of 10 s windows does not separate static from moving (static 0.25 m voxels are hit
       in a minority of windows too). Telling occlusion from absence needs rays: the scanner's
       trajectory (the vendor's, or estimated from the scan lines' gps_time), then free-space
       carving.
    6. Then the car stopped at the same place with nothing around it: path_optimizer
       "outside drivable area". Lanelet 1004849 is 2.16-2.17 m wide and the footprint 2.163 m
       (mirrors and margin). `register_lanelets.py` now widens (`--min-width 2.8`): an
       outer, unshared boundary vertex moves out, at most 1.5 m, where the DTM has ground
       beside it. On r01+r02, 67 lanelets were narrower than 2.8 m; 320 vertices moved (p50
       0.13 m, max 0.80 m), 183 shared ones stayed, and the narrowest is now 2.63 m.
    7. Along r01's last lanelet the road runs above a drop. SMRF's 16 m opening took the
       raised road for an object, no cell there was ground, and the fill put the road 6 m
       down (the scan's lowest return 11.84-11.89 m, the raw DTM 5.8-7.3 m): the car braked for
       its own road and slid towards the drop. `smooth_dtm.py --lanes` now takes the lanes'
       surface from the lowest return (5x5 median, unless a crown: more than 0.3 m above the
       window's lower quarter): 548 k of 586 k lane cells; lane dips went from 4382 to 122.
    8. Stopped and braked, the car slid sideways, 2.2 m in about 2 minutes on a ~4 % cross
       slope, until the EKF rejected NDT ("mahalanobis distance of pose topic is large") and
       the MRM followed. PhysX's sticky tyre: parked at r01's stop, 60 s, lateral damping 20
       crept 89.6 cm (1.50 cm/s), 200 crept 0.1 cm. Fixed in the spec (sim_vehicle_spec
       3773783, lateral as longitudinal, 200), regenerated in autoware_sim_vehicle and aib.
  - r01 end to end (2026-10-30, all of the above, auto_drive): "ARRIVED at goal after
    99.6s", 349 m. NDT error while moving p50 0.22 m, across-track p95 0.08 m; NVTL never below
    2.89; no AEB, no MRM.
  - Both routes again on the rebuilt world, with lateral sticky tyre damping 200 (2026-10-31,
    `/mnt/p5plus/aeon/ntu-twin/n8/drive.sh`, auto_drive from GNSS):
    - r01: arrived after 96.3 s, 349 m. NDT error while moving p50 0.23 m, p95 0.42 m;
      across-track p95 0.08 m; NVTL never below 2.87; no AEB, no MRM.
    - r02 (lanelet 7 to 49, world (-282.58, -45.02), yaw 0.0035): arrived after 126.6 s,
      347 m. NDT error p50 0.17 m, p95 0.29 m; across-track p95 0.10 m; NVTL never below
      3.08; no AEB (four before the road clearing), no MRM.
  - SSv2 on the twin: `scenarios/ntu_twin/ntu_twin_r01.xosc` (r01, no NPCs; the map as
    `$(env SWB_NTU_TWIN)/map`, a host setting), written by 062's `scripts/open_maps/scenario.py`,
    which now takes a start's and goal's z (the twin's ground is ~12 m up; the arrival check
    at z 0 would never have fired). Passed on 133 (2026-10-31, `just scenario
    scenarios/ntu_twin/ntu_twin_r01.xosc sim:=isaac`): engaged to "Passed" in 99 s, as the
    stand-alone drive. SSv2 refuses a map dir without a `*.pcd` ("The map_path must contain
    at least one *.pcd file"), so `export_map.py` now writes `pointcloud_map.pcd`, a link to
    the tile directory, as 062's maps have; Autoware's default file name then loads the tiles.
    Not checked: whether the world server's GNSS (on the map's projector since
    swb_isaac_world c0ca899) is right; the agent initializes from the scenario's pose.
    Passed again on 2026-11-02 on the current world (n5v3) and the map whose widened lanelets
    keep their centrelines, with the overlay: launch to "Passed" in 107 s, no AEB.
  - A proof video (2026-11-02): `render_video.py` (133's `ntu-twin/logs/`, not in the repo)
    renders a drive's bag as the front camera (brightened: no auto-exposure), the orthophoto
    with the track and Autoware's planned trajectory, and speed, steering and fail-safe state.
    The first is r01 from 09:04 (`r01_autoware_ntu_twin.mp4`, 104 s). The second
    (`r01_autoware_ntu_twin_chase.mp4`, 14:06, the current map) adds a chase camera 7 m behind
    and 3.4 m up (a local copy of the rig, `tesla_model3_chase.rig.yaml`), and the operation
    mode: arrived after 96 s, no AEB. The car shows as its four wheels: the Isaac ego has no
    visual body. Isaac with lidar and two cameras needs about 15 GB of the shared GPU, so the
    run waited for other users' jobs (`video_when_free.sh`). A camera without `k` makes
    CameraInfo's assertion stop the runner after one frame with no traceback: the runner's
    `finally` closes the app before Python prints it.
  - Rendering, first pass (N10; 133, 2026-11-03). The videos showed block paint on the ground,
    no light through the trees, a white sky with black shade, doubled shadows, and the car as
    four wheels. Causes and what changed:
    - Block paint: the orthophoto is 2 cm, but the scan puts a point in only 9-57 % of a route
      tile's pixels (9 % at r01's start); pass 3's pyramid fill turned the gaps into flat
      squares 0.1-1 m wide. `scripts/ntu_twin/ortho_refill.py` refills every tile from its own
      covered pixels (the coverage mask): a Gaussian-weighted mean (normalized convolution) at
      the finest of nine scales (0.75-192 px) whose kernel holds about 4 points, blended between
      scales, the eight neighbours padded in so tile edges match. 167 tiles in 223 s on 6
      workers, into `ground/ortho_smooth/`; road markings stay sharp, sparse areas go smooth.
      `map/isaac/ortho/` now holds them (the textures are files, the tiles unchanged); the world
      before is `map/isaac_n5v3/`.
    - Light: the world had none, so aib's runner and the world server added a sun and a plain
      dome (white). `world_usd.py` now writes the world's own (`--lights-only` adds them to a
      built world): a 0.53 deg sun at azimuth 135, elevation 55 (3000) and a dome with a
      clear-sky texture (`sky.hdr`, pale blue to a pale horizon, 1000), tuned by eye with
      `scripts/ntu_twin/render_probe.py` (fixed chase views along r01, before and after).
      Both runners light a world with its own light as it is.
    - Exposure: on Isaac 6.1 auto-exposure is the camera prim's
      (`OmniRtxCameraAutoExposureAPI_1`); `--/rtx/post/histogram/enabled` is mapped to it but
      not applied ("No setting was found"). aib b82ab1a: a camera's `auto_exposure: true` (rig
      or ego config) applies it; off by default. swb_isaac_world 1efd88e: the recorder's cameras
      follow the ego's camera6. aib also fills in a hand-written camera's missing `k` and logs
      the loop's exception before closing Isaac.
    - Mean picture brightness (0-255) in render_probe's four r01 views, before and after the
      three: start 73 to 114, under the canopy 7 to 25, turn 2 35 to 98, the palms 63 to 116.
      Under the canopy it is still dark: that is the trees (N10 3).
    - The car: the world server dresses the ego in its CARLA body (`vehicle_assets`) when
      `SWB_VEHICLE_USD` names the asset set; 133 had none, so every ego there was a box or four
      wheels. The set's `vehicles/` (17 GB) is now in `/mnt/p5plus/aeon/carla-usd/vehicles`,
      named in 133's `.env`. aib's stand-alone runner stays without a body (dressing is the
      world server's). A recorded r01 (the world server's recorder: dash, wheel, chase, course
      cameras) ran out of GPU memory beside another user's 16 GB job; it is queued for when the
      GPU frees (`logs/wait_gpu.sh`, `logs/rec_scenario.sh`).
    - Trees and the scan's baked light (N10 3, 4): `world_usd.py`'s `retouch_tile` (run by each
      tile build, `--retouch` on a built world, once per tile; 43 s for the 167). The ground
      texture holds little of the survey's sun (it looks overcast; its dark blotches are sparse
      points under trees), so the baked light is the meshes': a crown's vertex colours ran from
      near black to near white (linear luminance p10-p90 0.03-0.55 in a tile on r01), the scanner
      having seen its top sky-lit and its underside in shade. Each mesh's luminance is pulled
      toward its median (L' = median (L / median) ^ gamma: 0.45 for trees, 0.6 for the rest),
      its chroma kept. Crowns get a cut-out: 45 % of their vertices (by a hash of the position, so
      a rebuild cuts the same) get opacity 0 and the material cuts where the interpolated
      opacity is under 0.5. Tree components under 60 faces, and Other ones that small standing
      2 m over the ground, are dropped (about 3 % of the tree faces). Lidar intensity did not
      help with the ground: it varies with range and angle (road dark, pavement bright).
      Under the canopy the road is dappled now, mean brightness 25 to 44; walls stay blotchy
      and some larger shards stay. Tried on a copy (`ntu-twin/n10test/`); `map/isaac` gets it
      once r01 and r02 have driven on the copy with NVTL at 2.3 or over and no AEB (the lidar
      may see through the cut-outs too): queued behind the GPU with the recorded run
      (`logs/n10_chain.sh`).
    - On the way: the 14:06 video drive's Isaac ran on for 20 h holding 14.5 GB of the GPU:
      `n8_stop.sh` matched `ego_ntu.yaml`, not `cam/ego_ntu_video.yaml`; it matches both now.
      `render_probe.py` loads only the tiles within 200 m of its views: the whole world takes
      about 16 GB of VRAM.
  - N10's checks and the recorded run (118, 2026-11-03; handed over from 133, whose one GPU
    stayed full). The retouched world (crowns cut out at 0.45) on 118's GPU 1
    (`ISAAC_KIT_ARGS="--/renderer/multiGpu/enabled=false --/renderer/activeGpu=1"`), with Isaac,
    Autoware (the overlay) and `auto_drive` on domain 11:

    | drive | result | time | NVTL min | before (133, untouched crowns) | MRM 2 | AEB |
    |---|---|---|---|---|---|---|
    | r01 | arrived | 96 s | 2.92 | 2.88-2.90 | 1, at start-up only | 0 |
    | r02 | arrived | 127 s | 3.10 | 3.08 | 0 | 0 |

    The lidar does not see through the cut-outs enough to matter, so the retouched world is
    `map/isaac` (the untouched one beside it, `map/isaac_n5v3`). AEB is now counted from
    `/diagnostics` at ERROR named `emergency_braking`: the earlier count matched the word `AEB`,
    which that diagnostic never prints, so it read 0 whatever happened.
    Then r01 as the SSv2 scenario on the world server with its recorder
    (`cam/ego_ntu_rec.yaml`: camera6 on, auto-exposed): passed, 101 s, and the world server's
    log says `ego wears vehicle.tesla.model3`. The video (chase and dash side by side, 20 fps)
    is on the NAS, `autoveh/dataset/2026-10-30 NTU twin map/video/n10_r01_chase_dash.mp4`.
    It shows the car in its body and sun coming through the crowns in patches. Left: walls
    stay blotchy, larger shards stand near r01's start, and the chase camera over-exposes the
    road there for the first seconds.
    On the way: `just isaac-world ego:=` with a path relative to the checkout crash-looped the
    world server (73 respawns in 20 min, `FileNotFoundError`): the node runs from another
    directory, so give an absolute path.
  - N8's scenarios with NPCs (118, 2026-11-04): two variants of r01 on the retouched world,
    each on the world server with its recorder (Isaac on GPU 0; GPU 1 held by another user's
    vLLM). The world server dresses a declared car in the nearest CARLA body and a pedestrian
    as a CARLA walker. Positions come from r01's driven track (`n8/drive_n10a.csv`).
    - `scenarios/ntu_twin/ntu_twin_r01_lead.xosc`: a car 25 m ahead in the ego's lane drives
      r01 at 2.5 m/s and is deleted 45 m before the goal. Passed, 135 s: the ego closed up and
      held 2.5 m/s behind it from the first turn to the last, then went on at 4.3 m/s.
    - `scenarios/ntu_twin/ntu_twin_r01_pedestrian.xosc`: 160 m along r01, when the ego is 40 m
      before the crossing, a pedestrian walks south at 1.2 m/s into the ego's lane, stands in
      it for 10 s, then walks on across both lanes. Passed: the pedestrian was in the lane
      with the ego 30 m away at 4.2 m/s. The ego braked from about 15 m (4-5 s later; about
      1 m/s^2), stopped with its front about 4.8 m from the pedestrian, and drove on once the
      pedestrian had cleared the lane.
    - The first version moved the pedestrian with a timed `FollowTrajectoryAction` (as
      town01_pedestrian does). SSv2 started the event on time, but the pedestrian moved only
      0.1 m in the next 6 s. It reached the lane about 45 s late, after the ego had passed,
      and the scenario still "passed". A watcher on `/simulation/entity/status` and
      `/simulation/context` in the scenario domain showed it (`n8_npc_118/ssv2_watch.py` on
      the NAS). Not chased further: SSv2's `WalkStraightAction` with speed steps is
      deterministic here. A "passed" with NPCs needs a check that they did their part.
    - Videos (chase and dash, 20 fps) on the NAS: `video/n8_r01_lead.mp4`,
      `video/n8_r01_pedestrian.mp4`.
  - N2, the survey's moving objects (133, 2026-10-31; `scripts/ntu_twin/transients.py`, free-space
    carving along the vendor's trajectory, method in its docstring). 167 tiles in 2.1 h of
    worker time: of 5.47 M slab voxels (0.2 m, 0.2-3 m above the DTM) 1.16 M were crossed by
    rays in more windows than they were hit, and 578 k (10.6 %) were dropped, those whose
    column holds no static voxel. 45 tiles cast no ray (no trajectory sample within 40 m or
    0.6 s); nothing is dropped there. Tile 1053_1357: 0.39 % of the points dropped, r01's
    object among them (16644 of its 19689 points), its walls and shrubs kept.
    - Timing: the `.traj` files are Unix time; the sessions written as seconds of the GPS week
      fit the scan at UTC, not GPS time (98 % of 0.1 s windows with a return within 3 m of
      the scanner, against 49 % 18 s off). In the tiles checked, the scan holds no points
      from the May-4 sessions.
    - `export_map.py --dynamic` and `build_meshes.py --dynamic` leave the dropped voxels'
      points out: the PCD map went from 39.1 M to 38.6 M points (the old one, kept as
      `map/pointcloud_map.pre_n2`, deleted 2026-11-02), the meshes were rebuilt into `meshes_n2/` (3.4 h, two
      workers), and the world from them into `n2w05/` and `n2w25/` (road cleared 0.5 and
      2.5 m).
  - N5, trees, first step (133, 2026-10-31; `scripts/ntu_twin/trees.py`, method in its
    docstring). The scan has no vegetation class, so trees are found from shape: vegetation
    is what scatters in 3-D (neither planar nor linear), and a tree is a trunk (a small blob
    0.8-2.5 m above the ground spanning that height) with canopy over it, or a treetop where
    canopy has no trunk. Crowns are split over the whole site, so a tree on a tile's edge is
    one tree. 2907 trees, 2210 on a trunk, 697 on a top only; height p10/p50/p90 9.4 / 15.9 /
    21.0 m, crown base 3.6 / 6.3 / 11.4 m, crown radius 1.3 / 2.6 / 4.0 m (55 min, two
    workers). Checked by eye on one row by r01 (tile 1053_1357): 9 of about 12 trunks found,
    one lamp post taken for a trunk; thick or merged trunks are missed.
    - `world_usd.py --trees` splits the other objects' mesh into /Tile/Tree (labelled tree,
      Q10884) and /Tile/Other, and gives every trunk an invisible cylinder collider from the
      ground to its crown's base; none on a lane. Site-wide: 33.5 M of 51.8 M other triangles
      are Tree, 2178 trunk colliders (32 trunks on lanes left out). World: `n5w05/` (meshes_n2,
      road cleared 0.5 m).
    - Camera (133, 2026-11-01; `world_probe.py` at world (180, 185) on r01's tree row): the
      dash camera sees 36 % tree, 36 % ground, 26 % unlabelled (hedges, low objects), the
      label image matching the RGB. The semantic annotator read no label until world_usd.py
      applied LabelsAPI over the bare `semantics:labels:wikidata_qcode`; the world server reads
      the attribute either way. The trees of 35-43 m are real: those columns are dense to 10 m,
      empty from 10 to 20 m and hold a canopy layer at 20-45 m (2.15 M points over 15 m in
      30 x 20 m), and the high camera, 37 m up, is inside it.
    - Thick trunks: trees.py takes trunk blobs up to 4 m2; one over 1 m2 (a thick trunk, or
      two merged) must also be round (long axis at most 2x the short) and at most half
      scattering, which keeps walls' ends and bushes out (the scatter test alone dropped 7 real
      trunks wrapped in foliage). Tiles 1052/1053_1357: 31 -> 36 and 30 -> 32 trunks, none lost;
      two thick trunks in 1053's row are still missed. Site: 2965 trees, 2348 on a trunk (2210
      before), `trees_v2/`, world `n5v2/`, the site's default `map/isaac/` since 2026-11-02
      (the one before it is `map/isaac_n5w05/`, the first `map/isaac_n1/`).
    - Tree assets: `world_usd.py --tree-assets TOWN_DIR` puts a CARLA tree (Acer, Sassafras,
      Japanese maple, Beech; a coconut palm where the crown is high and narrow) at every tree,
      scaled to its height and crown, in place of the scan's Tree mesh and its fringe, none on
      a lane; the assets travel with the world (`isaac/trees/`). World `n5a/`, 32.3 M
      triangles. On camera far better than the scan's blobs: leaves, a canopy, dappled shade.
    - Drives (r01, r02; all arrived, no AEB, the MRM counted is the startup one):

      | world | route | error p50/p95 | NVTL p50 | NVTL < 2.3 |
      |---|---|---|---|---|
      | n5v2 (scan trees) | r01 | 0.16 / 0.37 m | 3.13 | 0 s |
      | n5v2 (scan trees) | r02 | 0.13 / 0.25 m | 3.16 | 0 s |
      | n5a (assets) | r01 | 0.17 / 0.38 m | 2.47 | 11.7 of 93 s |
      | n5a (assets) | r02 | 0.15 / 0.26 m | 2.36 | 28.5 of 123 s |

      The assets are not the scan's trees, and NDT matches the lidar against the scan's PCD:
      along the tree-lined stretches its score falls under 2.3, Autoware's convergence
      threshold. Driving stays on the scan's trees until the lidar can see those while the
      camera sees the assets, or the PCD is made from what the world holds.
      n5v2r01 had no stop at r01's turn (7 of 9 drives on the N2 worlds stop there).
  - Drives on the N2 and N5 worlds (133, 2026-11-01; `n8/drive.sh` with `WORLD=`, Autoware on
    the N2 PCD map, one at a time). Every drive arrived, no AEB, no MRM while driving:

    | world | route | time | error p50/p95 | across p95 | NVTL min |
    |---|---|---|---|---|---|
    | n2w25 (2.5 m) | r01 | 99.8 s | 0.22 / 0.42 m | 0.07 m | 2.88 |
    | n2w05 (0.5 m) | r01 | 96.6 s | 0.22 / 0.42 m | 0.08 m | 2.88 |
    | n5w05 (trees) | r01 | 96.7 s | 0.22 / 0.42 m | 0.08 m | 2.89 |
    | n2w25 (2.5 m) | r02 | 126.6 s | 0.17 / 0.29 m | 0.10 m | 3.08 |
    | n2w05 (0.5 m) | r02 | 126.8 s | 0.13 / 0.25 m | 0.10 m | 3.10 |
    | n5w05 (trees) | r02 | 126.8 s | 0.13 / 0.25 m | 0.10 m | 3.10 |

    With the moving objects gone, a 0.5 m road margin is enough; the trees and their trunk
    colliders change nothing on these routes. The MRM counted on two runs is the startup one,
    before localization.
    - A world directory needs the map's `map_projector_info.yaml` next to its `lanelet2_map.osm`
      (aib reads it two levels above `world.yaml`). Without it aib's GNSS falls back to a
      transverse Mercator at (0, 0), auto_drive's initial pose is 34 km off, and NDT never
      converges ("No InputTarget"); the first two 0.5 m drives failed so. `world_usd.py` now
      warns when the map directory has none.
  - The 1 s stop, and docs/issues 025-027 (133, 2026-11-02):
    - r01's stop at its left turn ((52806, 67885), 7 of 9 drives on the N2 worlds) was
      path_optimizer's `enable_outside_drivable_area_stop`: the turn lanelet 1007361 (R about
      10 m) was 2.80 m wide, and on that radius the car's footprint sweeps about 2.8 m. In every
      drive that stopped, the front-right corner came to rest exactly on the edge; in those that
      did not, it crossed it by 0.14-0.18 m just after the turn. No planning factor names it.
      `register_lanelets.py --widen-only` now widens a curve's outer edge by the car's
      offtracking plus CURVE_MARGIN (0.3 m), carries it CAR_REACH (12 m) past the curve,
      densifies long outer segments so there are vertices to push, moves a node shared along
      one road edge, and checks the new place against the meshed surface (a road where cars
      stood all through the scan has no ground return; r01's second turn had none). A first
      version left that second turn as it was: one drive stopped there with the corner 0.3 m
      out and stayed 8 minutes, path_optimizer putting the stop at the car every cycle.
      Final map: 2379 vertices moved (p50 0.54 m, max 1.5 m), 4764 added, none narrower than
      2.80 m; on every recorded track both turns' corners stay inside. The map before is
      `lanelet2_map.before_curves.osm`.
    - 025 (falls through at r01's first lanelet): a hole in the ground mesh under the car,
      vehicles having stood there all through the scan; `ground_keep` fills enclosed holes
      only. `build_meshes.py --lanes` meshes every cell under a lanelet (plus 1 m): the car
      now rests 0.03 m from the spawn there. World `n5v3/` (meshes_n3, the widened lanelets),
      the default `map/isaac/` since 2026-11-02 (the one before it is `map/isaac_n5v2/`).
    - 026 (0.8 m off at r02's spawn): not reproduced; 0.000 m on flat ground, 0.003 m along at
      r02 after the sticky tyre's damping fix.
    - 027 (pose_instability_detector's negative time): an empty twist range read when a twist
      is stamped at the previous odometry's time; fixed in the Autoware overlay. The NTU
      drives run on the overlay with `AUTOWARE_SETUP=<overlay>/install/setup.bash` for aib's
      `just autoware` (stock otherwise: neither NDT's step fix nor blind_spot's); since
      2026-11-02 `n8_env.sh` sets it by default (`NTU_AUTOWARE_SETUP` names another setup,
      `/opt/autoware/1.5.0/setup.bash` for stock).
    - Drives on the final n5v3 with the overlay: r01 6 of 6 (w01a-c, a1-a3) and r02 (w02)
      arrived, none stopping before the goal; so did o01 and o02 on the first widening.
  - Camera and lidar on a drive (133, 2026-11-02): r01 on n5v3 with the front camera on
    (`camera6`, 1280x720 rgb8, 10 Hz; aib `sensors: {camera6: {enabled: true}}`) and a rosbag of
    `/sensing/*`, TF, localization, perception, planning and control (mcap, zstd; 9.3 GB for
    the 136 s drive, `bags/r01_cam_lit`). Every stamp-carrying topic is monotonic; by stamp the
    raw lidar runs at 19.5 Hz (112 k points a scan, ranges 5-81 m), the merged cloud 18.5, IMU,
    odometry and vehicle status 20, GNSS 10, the camera 9.6 (1305 images against 1349
    camera_info: about 3 % of the 2.7 MB images lost on the way to the recorder). TF reaches
    the camera's optical frame from base_link, and the merged cloud projected through it and
    camera_info's K lies on the road and the palm trunks in the image.
    - The first such drive's 1310 images were all zero: the NTU world holds no light, and aib's
      stand-alone runner, unlike swb_isaac_world, added none. aib 38b3a94 adds the world
      server's default sun and sky where a world brings no light.
    - Under the dense canopy west of the first turn the picture is nearly black (93 % of pixels
      under 20): no auto-exposure.
    - `ros2 bag record -e REGEX TOPICS...` in Humble filters the listed topics by the regex
      rather than adding to them; give one regex naming everything.
  - Open:
    - On the first n5v3 map, AEB fired once in 6 r01 drives (w01c), 0.3 s just past the first turn:
      a static object 7.25 m ahead against an RSS distance of 7.28 m, the car not slowing. Kept
      at first (decided 2026-11-02), fixed the same day (below). The object is a hedge or fence along the westbound lane's
      north edge: the scan holds a band 1.0-1.7 m tall from x 52786 to 52800 at y 67887.9-67888.1,
      seen over 40 minutes, about 0.4 m past the lane's original edge, meshed as Other just
      outside the road's 0.5 m clearing. Coming out of the 10 m turn the car's front-right
      corner reaches y 67887.9, within 0-0.2 m of it, on the old map and the widened one alike;
      the widening moved the exit 0.2 m north, and AEB's MPC path sweeps the hedge during the
      turn on every drive (its debug markers), crossing the RSS distance once. Neither way out
      fits: keeping widened edges 0.8 m from the scan's objects (tried) put the corner 0.34-0.41
      m outside the lane again, the 1 s stop's condition; and the road is about 6 m between
      objects on both sides (0.0-0.5 m past the eastbound lane's south edge too), so the shared
      centre line cannot move south without taking the eastbound lane under 2.8 m.
    - Fixed (2026-11-02): the widening had moved the path too. A widened lanelet's centreline is
      the centre of its new bounds, so the exit lanelet's ran 0.3 m north (y 67886.35 against
      67886.06) and the car with it. `register_lanelets.py` step 6 now gives each lanelet the
      widening changed (154) a custom centerline, the centre of its bounds before widening
      (Lanelet2's role "centerline"; Autoware's map loader, `use_waypoints: true`, makes it the
      waypoints the planners follow). The bounds and so the drivable area stay as widened; the
      world is unchanged. The car's front-right corner now peaks at y 67887.58 (was
      67887.86-67887.88 in all six earlier drives), the mesh starting at 67887.75; AEB's MPC path
      meets the fence at 7.3-9.1 m ahead at the closest (2.1-2.2 m before). r01 x3 (k1-k3) and
      r02 (k4) arrived on n5v3 with the overlay, no AEB, no MRM while driving. On the way: 23
      lanelets have a left bound drawn against the right one, which Lanelet2 inverts on load;
      the centreline does the same, or it folds.
    - Every emergency stop while driving, over all 43 N8 drives (checked 2026-11-02): the MRM
      at start-up on most of them is the topic monitors before the first messages (cleared
      before localization, never engaged). Mid-drive: r01h-r01o (2026-10-30) are the r01 stop
      taken apart above; r01p's AEB (2026-10-30 21:23, 0.3 s at turn 2's exit, (52577.5,
      67871), not slowing) predates the road clearing's 2.5 m margin and the widening; r02r
      (2026-10-31) was spawned facing north (yaw 1.5708) on an eastbound lane, so start_planner
      chose a backward pull-out, reversed 2.8 m turning to 46 deg and stayed on a backward path
      for 9 minutes; each of its 14 MRMs is planning_validator's "sudden shift in trajectory".
      r02s and w02 spawn at yaw 0.0035 and arrive, no backward path. On n5v3 only w01c's AEB
      remained, gone since widened lanelets keep their centrelines (above).

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
route driven; SSv2 scenarios with NPCs from the catalog. Done 2026-11-04: r01 behind a slower
car and r01 with a pedestrian crossing, both passed on 118.

**N9. Camera stacks.** Vision Pilot and a VLM agent on the ego's camera, open loop first on
recorded drives, then closed loop. Record what they get wrong that they do not get wrong on
real footage, if we can record some.

**N10. Camera realism.** What stage A can do for the picture without photos, measured on fixed
views (`render_probe.py`) before and after each step:
1. The ground texture without blocks: a density-adaptive refill of the orthophoto
   (`ortho_refill.py`). Done 2026-11-03.
2. Light and exposure: the world's own sun and sky dome, the camera's auto-exposure. Done
   2026-11-03.
3. Trees: crowns with gaps (a cut-out pattern in their material) so the sun comes through in
   patches, the loose shards gone; check NDT on r01 and r02, since the lidar may see through the
   gaps too. Done 2026-11-03; NDT passes on r01 and r02 (NVTL min 2.92, 3.10), now the
   default world. Not tried: CARLA tree assets seen by
   cameras only, the scan's crowns by the lidar only (whether Isaac can hide a prim from one
   sensor and not the other).
4. The scan's baked light: the meshes' colours pulled toward their median brightness. Done
   2026-11-03 with 3. Partial: walls stay blotchy.
5. The car in its body in recorded videos: the world server with the CARLA asset set
   (`SWB_VEHICLE_USD`). Done 2026-11-03: r01 recorded on 118, the video on the NAS.

These make a clean simulation picture, not a photograph: one colour per scan point, leaves as
blobs. Photo-like pictures are stage B's gaussian layer, which needs the vendor's 2231 photos.

## Parity with 072 (the NTU campus in CARLA)

User direction 2026-11-04: the two twins, this one on Isaac (swb 118) and
[072](072-ntu-campus-in-carla.md)'s on CARLA 0.9.16 (swb 133), get the same features and
comparable outcomes. Agreed between the two sessions on 2026-11-04. 072 keeps the same list in its
own words.

| | what both twins provide | owner | Isaac (059) | CARLA (072) |
|---|---|---|---|---|
| P1 | The whole site (167 tiles) in the map frame, origin (52500, 67700, 0). One Autoware map dir: 059's Lanelet2, `pointcloud_map.pcd` and projector | both | have | have (`carla/NTU` links to 059's) |
| P2 | The ego: `tesla_model3` from `sim_vehicle_spec`, the same sensor kit, Autoware profile and overlay. The lidar is each simulator's own (RTX, CARLA raycast), so NDT's input differs | both | have | have |
| P3 | One scenario set. `ntu_twin_<case>.xosc` is the source and `scripts/ntu_twin/scenario_pair.py` writes `ntu_carla_<case>.xosc` from it (only the map dir, the header and the Story name differ; `--check` finds drift): r01, r02, r01_lead, r01_pedestrian. NPC scenarios to come: a cut-in, oncoming traffic at a turn, and a signalised junction once P4 is in | 118 | r01 and the two NPC cases pass; r02 not yet run as a scenario | r01 5/5 and r02 pass; NPC cases untested (walker blueprint, WalkStraightAction, DeleteEntity on swb_carla_world) |
| P4 | Traffic lights, stop lines and crosswalks where scenarios need them: Lanelet2 (agreed before either session edits it) and the matching OpenDRIVE signals | Lanelet2 both, OpenDRIVE 133 | none yet | none yet |
| P5 | The same picture targets: sun at elevation 55 deg, azimuth 135 (CARLA's 225, which is the map azimuth + 90), the same default weather, exposure judged on the same four r01 probe views (`render_probe.py` / `carla_probe.py`: start, canopy, turn 2, palms). Any asset fix is adopted by both: ortho refill, de-lighting, shards, parked vehicles removed (072's `scripts/ntu_carla/parked_vehicles.py`), low vegetation as a smooth height field (072's `hedges.py`, in test) | both | have, except parked vehicles and hedges | have, exposure ev -1.5 and min_bright 7 |
| P6 | The same recording per run: chase and front views at 20 fps; a side-by-side mp4, 1600 px wide; `state.jsonl` of ego x, y, z and speed (map minus origin). The mounts are the rig's, in base_link (the rear axle): chase (-7.0, 0, 3.4), pitch 15 deg down, 80 deg; front = camera6 (0.9, 0, 2.0), pitch 0.86 deg down, yaw 2.09 deg right, 74.5 deg, 1280 x 720. CARLA mounts on the actor's origin, 1.386 m ahead of the rear axle on the model3 (swb_isaac_world reads the axle off the CARLA asset; its height against base_link is to check) | both | have (swb_isaac_world's recorder) | 133 adds state.jsonl, the 1600 px video and the base_link offset |
| P7 | The same outcome measures: junit result, time to goal, NDT NVTL min and mean along r01 and r02 (`ndt_track.py`), MRM and emergency count, speed behind the lead, stop distance and minimum gap to the pedestrian, and the NPC check (`scripts/ntu_twin/ssv2_watch.py`: SSv2's view of each entity and event, simulator-agnostic) | 118 the scripts, each its runs | have | 133 adds NVTL on CARLA runs |
| P8 | Camera stacks (N9): Vision Pilot and a VLM agent, open loop on P6 recordings first, one harness for both | 118 first | not started | later |
| P9 | Distribution on the NAS: each simulator's package with a README (install, frame, versions), and one comparison page, CARLA beside Isaac (probe views, P7 numbers) | both | world on the NAS | site package pending |

The fixed probe views (P5), map minus origin, yaw in degrees counter-clockwise from +x, as
`render_probe.py --view` / `carla_probe.py --view` take them: `start,314.3,118.3,91`,
`turn2,135.2,186.4,180`, `canopy,270.5,186.1,-180`, `palms,77.6,165.4,-88`. Each is a chase camera
7 m back along the yaw and 3.4 m over the ground (a ray cast at x, y), pitch 15 deg down, 80 deg
across, 1280 x 720 (the scripts' defaults). The views N10's numbers were measured on.

## Risks

- Trees look fake to a VLM; baked shadows in the scan's colour double RTX's own (seen in
  the first videos, 2026-11-02; N10).
- Texture budget (~1.8 G texels at 2 cm) needs tiles and streaming.
- RTX lidar has crashed on multi-GPU setups: pin Isaac to one GPU.
- GPU: both RTX 5090s on 241 are held by another user's vLLM as of 2026-10-29, so the work
  moved to 133. On 133 another user runs a CUDA MPS daemon (`/tmp/nvidia-mps`), and our CUDA
  fails with 805 (MPS_CONNECTION_FAILED) unless `CUDA_MPS_PIPE_DIRECTORY` points at an empty
  private directory. That bypasses MPS; it is set in 133's `.env`.
- Disk on 133: `/home` is a full spinning disk; everything lives on `/mnt/p5plus` (NVMe).
- RAM on 133 is 60 GB: run build_meshes with at most 2 workers (10.8 GB peak each).
