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

7. **Parked vehicles left out** (2026-11-04, `parked_vehicles.py`, `carla_meshes.py --parked`).
   CARLA puts its own vehicles where a scenario asks, so the scan's parked cars and scooter
   rows go, not replaced.
   - A 25 cm raster on the DTM's grid of the prop faces (off the lanes, outside the trees): per
     cell the highest face over the DTM, the face count, the mean colour. Low cells (top 0.5 to
     2.2 m, not next to a cell of 2.6 m or more: a wall, a kiosk) are closed by two cells and
     labelled.
   - A blob is a vehicle, a row of them nose to tail or a scooter block when its width (area
     over its long axis) is 0.9 to 8 m, its top 0.9 to 2.5 m, under half its cells green, it
     fills over 0.4 of its convex hull, covers 1.5 m2 or more and comes within 8 m of a lane.
     Green fraction is bimodal (vehicles under 0.3, hedges over 0.6). The first rule (width
     under 2.8 m, green under 0.35) found 161 blobs and missed the main avenue's scooter blocks,
     several rows deep (3 m wide). The solidity test drops an L-shaped wall frame.
   - Site: 260 vehicle blobs of 1946 low ones, 2860 m2, 63776 cells (one cell of margin).
     `carla_meshes.py` drops the prop faces under the mask below 2.6 m: 272585 source faces,
     props after decimation 4.48 M -> 4.40 M triangles. The ground under them is the DTM's
     terrain, which was always there.
   - Probe views before and after (`carla/site/parked_before_after.jpg`): at the canopy view the
     scooter rows and cars in the north bays and the big grey block by the right road edge are
     gone, terrain under them, walls and kerbs intact. At turn 2 a narrow olive row along the
     north kerb is gone too (the orthophoto shows a light row there, most likely scooters; not
     confirmed). The start and palms views barely change. Left in, the misses: smaller lumps
     further along the canopy view's right edge and two dark ones on its left, shards by the
     trees at the start view. Under 0.9 m, over 2.5 m, or merged with a hedge or a wall is a
     blob the rule does not take.
   - The rebuild's material step died of ENOSPC on `/mnt/p5plus` (UE's engine
     DerivedDataCache had grown to 58 GB, 49 GB of it stale from earlier imports of this map;
     cleared) and left half-moved assets, so the import was redone. `site_rebuild.sh` now
     checks for 30 GB free and stops on the editor's or the packager's exit code.
   - Lidar and NDT: the point cloud NDT matches against (059's) still holds the parked cars;
     CARLA's lidar no longer sees them. r01 on the new map passed (junit 0 failures, five of
     five on this map now): NDT logged no warning while driving, only the usual "No
     InputTarget" before the map loaded and a 0.1 s emergency stop at start-up, before
     engagement. So no sign on r01 that the missing cars disturb NDT (its scores were not
     recorded); a route along the scooter blocks may differ.
   - `ntu_carla_r01_autoware_v3.mp4` (98 s, 344 m driven) on the NAS beside v2.

8. **r02** (2026-11-04, `scenarios/ntu_twin/ntu_carla_r02.xosc`). 059 N8's second route,
   lanelet 7 to 49 eastward, on the map of item 7: start (52217.42, 67654.98), yaw 0.0035,
   goal (52565.0, 67656.93), as N8's `r02_east.yaml`; z is Isaac's spawn height plus 0.2 m, as
   r01's file. `just scenario scenarios/ntu_twin/ntu_carla_r02.xosc` with `CARLA_PORT=2400` on
   the ego stack r01 had left up: passed (junit 0 failures), 343 m in 128 s, as on Isaac (127 s).
   No NDT warning and no MRM during the run (the stack's only emergency stop is the one at its
   start-up, before r01). `ntu_carla_r02_autoware.mp4` on the NAS beside r01's.

9. **Hedges as a height field** (2026-11-04, `hedges.py`, `carla_meshes.py --hedges`).
   - Smoothing the geometry does not take the faceting out. Adjacent faces of the scan's
     vegetation meet at 43 deg at the median (p90 80). Taubin smoothing (lambda 0.5, mu -0.53,
     boundaries held) took that to 29 deg after 40 iterations, and the decimation put it back to
     43. The surface is a crumpled sheet, not a noisy smooth one.
   - A hedge is a top over the ground, so it is rebuilt as one, on the DTM's 25 cm grid.
     - Low vegetation faces: off the road, under 2.6 m, and a tree's or green (G over R and B
       by 8 %).
     - Hedge cells: 2+ such faces, 60 % of the cell's low faces, a top over 0.25 m, 0.4 m
       clear of a trunk, not under the parked-vehicle mask. Closed by two cells, opened by
       one, blobs under 0.75 m2 dropped.
     - The top: median 3x3 and an unnormalised Gaussian (sigma 1.2 cells), so the sides fall
       to the ground over a few cells; the edge is where it drops under 5 cm.
     - Site: 244617 cells, 15289 m2 in 1573 blobs, top p50 0.56 m.
   - `carla_meshes.py --hedges` drops the low vegetation faces under the mask (2.08 M source
     faces) and adds the field as tree-class geometry (Vegetation tag, alpha 1, no cut-outs):
     523k triangles. Props after decimation 4.40 M -> 3.81 M triangles.
   - The first build left a flat, light slab on the north kerb at turn 2. It was the scooter
     row item 7 removes, whose greenish faces passed as vegetation; hence the parked-mask
     exclusion.
   - Probe views (`carla/site/hedges_before_after.jpg`):
     - At turn 2 and palms the roadside hedges on the left are smooth mounds now, not
       crumpled lumps. Start and canopy barely change.
     - Left as they were: the bushes over 2.6 m or around a palm's trunk (palms, right side),
       which are crowns, and the dark non-green lumps at the canopy view.
   - `site_rebuild.sh` now counts free space after clearing its own last output (Content/NTU,
     Dist/NTU_*). The import, materials (339 meshes) and package took 41 min.

10. **The shared scenarios on the hedge map** (2026-11-04; 059's parity list, P3 and P7).
    - The four NTU cases are generated pairs now: `scripts/ntu_twin/scenario_pair.py` writes
      `ntu_carla_<case>.xosc` from `ntu_twin_<case>.xosc`. Only the LogicFile differs; edit the
      `ntu_twin_` file.
    - Run one after another on CARLA 2400, one ego stack, with the recorder and 059's
      `ssv2_watch.py` (domain 9, this workspace's install sourced for
      `openscenario_interpreter_msgs`):

      | case | junit | driven | NPC check |
      |---|---|---|---|
      | r01 | 0 failures | 344 m | - |
      | r02 | 0 failures | 343 m | - |
      | r01_lead | 0 failures | 344 m | ego behind the lead at 2.44 m/s (lead 2.50), 15 m back; `lead_gone` complete |
      | r01_pedestrian | 0 failures | 344 m | `cross`, `stand_in_the_lane`, `walk_on` complete in order; ego stopped 8.9 m from the walker (SSv2 entity positions); closest 8.1 m, after it walked on |

    - Over the four runs the stack logged no MRM. NDT's only warnings came at its start-up,
      before the map loaded. NVTL was not recorded.
    - CARLA needed no change to the NPC files: the walker blueprint, WalkStraightAction with
      SpeedAction, and DeleteEntityAction all worked, and Isaac's z values sit on CARLA's
      ground.
    - Videos: `carla/site/runs_h1/*.mp4`.

11. **Tour of every lane** (2026-11-04; user: "make a tour in r01 and r02 and visit every
    paths. Fix the issues along the way, such as ghost mesh and point, mrm triggers").
    Autoware drove every lanelet of 059's merged r01 + r02 map (285 road lanelets, 6.6 km of
    lanes) on CARLA 2400, one leg per scenario on one ego stack. Isaac ran the same legs
    (059, swb 118).
    - **The legs** (`scripts/ntu_carla/tour_legs.py`, `scenarios/ntu_twin/ntu_twin_tour_NN.xosc`,
      CARLA pairs by `scenario_pair.py`, `tour_legs.json`). The planner is greedy and polynomial:
      a Dijkstra tree from every lanelet, and each leg is the tree path with the most uncovered
      length. Each leg is checked to be lanelet2's own shortest path, which is Autoware's route
      (the map has no lane changes). It puts the start and the goal where no same-direction
      lanelet covers the point. The goal follows `default_planner`'s check, the model3 footprint
      inside the goal's lanelets and their followers. No centreline point more than 20 m before
      the goal comes within 7 m of it, since SSv2's 5 m reach condition fired on a loop or across
      the road. At the scan's edge the start moves in until the PCD surrounds it (6 of 8
      directions at 10-30 m). A second pass gives each lanelet the greedy pass missed a short leg
      of its own. Result: 114 legs, 9.85 km, all 284 lanelets except 1011236, the isolated 4 m
      stub left by the r01/r02 merge (agreed with 059). The first sets had planner bugs that a
      run found: dcb0138's leg 02 "passed" at once (a loop), 03's goal footprint stuck out of a
      curved sink, and 37c0dd9's leg 00 passed after 13 m from the opposite lane.
    - **Lanelet2 fixes** (`scripts/ntu_twin/fix_lanelets.py`, agreed with 059; idempotent, md5
      in and out; the map is 24aa4578 -> e12bfc42 -> c2854a7c after 059's step D):
      - A: 23 custom centrelines ran against their lanelet. lanelet2 flips both bounds of these
        lanelets on load, and Autoware keeps a custom centreline as stored, so its path ran
        backwards. A reverses the 23 ways. Leg 25, which starts on one of them (1008389), passes.
      - B: 5 joins touched but were not linked (on different nodes, under 0.4 m apart). 3855 and
        3719 could not be reached, and 2132 was isolated. B links them.
      - C: B's node replacement left 5 ways with a node twice in a row. SSv2 failed r02 next to
        3855 ("catmull_rom_spline.cpp:331: failed to calculate curve index"). C drops the repeat.
        The first output, 97f10f1a, was withdrawn the same hour.
      - D (059): two node eles near r01's start had come from a DTM spike.
    - **Ghost geometry: the ground.** 059's DTM put the road 2-6.5 m up into the canopy in 7
      places, where no return reaches the road and the lowest return is a crown. Leg 00 stood
      306 s before the cone at (489, 232) (`carla_tour/issues/` on the NAS). Two fixes:
      - 059 fixed `smooth_dtm.py` (2cedde5), then added a lane-ele rule (6012df6): a lane cell
        more than 0.25 m over the lanelets' surface comes down to it, except where the scan's
        lowest return backs it. That gives ground_n12 (407b9203), CARLA's and Isaac's ground
        since 18:00.
      - `carla_meshes.py` keeps a guard that only ever lowers. It runs a grey opening of the
        lane surface over 8 m, on the tile and its neighbours, and brings down lane vertices
        more than 0.15 m above it. Then it rounds convex edges that stand more than 0.05 m over
        a Gaussian with sigma 1.5 m, and blends the moved patch. A percentile version left
        0.25 m steps that the ground filter called obstacles (run C, leg 00, 450 s). A per-tile
        opening left a 0.3 m ridge on a seam that high-centred the car (run D, leg 02). A
        lane-ele step in carla_meshes (deb22af) dug r01's road 0.45 m under the scan at
        (131, 182); it is now behind `--lane-ele`, off, since n12 does it right.
      - Under the canopy the fill also invented rises 0.3-0.75 m over the lanelets, and the
        ground filter stopped the car there (D 24 at (101, 325), 71/73/74 at (-289, -150)).
        n12's lane-ele rule takes them down.
    - **Ghost geometry: the hedges.** The hedge height field's blur reached 0.6 m into lanelet
      1008289 at (64, 210), and the ego stopped for a route-obstacle (run B, leg 04). The field
      now stays off the lanes (the road clearing's raster). D's leg 04 passes.
    - **Ghost points** (`tour_lidar_diff.py`: the ego's obstacle points farther than 0.5 m from
      the PCD, in the map frame). In-lane hotspots in runs B and C were only the cone and the
      hedge above. The PCD itself showed no ghost points on the lanes. It still holds the scan's
      parked cars, which CARLA leaves out (item 7). NVTL along the tour shows no harm from that:
      the median of the per-leg minima is 3.04, and the minima stay above 2.5 except at the two
      scan corners and on the ramp.
    - **MRM** (`tour_monitor.py`: MRM, hazard faults, velocity factors, planning walls).
      - The 0.2 s PULL_OVER mid-drive (r01, (229, 186)) was NDT's pose buffer running dry. The
        EKF publishes at the 20 Hz sim clock, one pose per 20 Hz scan, so the buffer held
        exactly 2. One late pose made the scan fail to interpolate, a scan_matching_status
        WARN, and the MRM. /clock has no bursts (steady 20 Hz, `tour_clock.py`). Fixed in the
        overlay: autoware_core 879b7ce5 (autoware_localization_util), `SmartPoseBuffer::pop_old`
        keeps the newest pose older than the scan; docs/issues/035. On the user's decision there
        is no hysteresis mask. In tour D (new overlay) no leg logged an MRM while moving.
      - The other MRMs come from cars standing for minutes on a ghost: EKF Mahalanobis WARNs
        (run C, 314 flickers in leg 00) and AEB on the ramp (27, 81, below).
    - **Results, run D** (ff5da50 legs, overlay 879b7ce5, meshes_n11 + the opening guard; full
      table below): 100 of 114 legs pass, plus r01 and r02. 8.0 km driven. No MRM while moving.
      NVTL minimum per leg: median 3.04, lowest 1.81 (81, on the ramp).
      The 14 that failed, by cause:

      | legs | stop | cause | status |
      |---|---|---|---|
      | 02, 74 | (-289, -153), at the start | the fill's rise at the SW scan edge (dtm_raw steps 13.21 -> 12.26): car high-centred on its edge | pass in E (n12) |
      | 71, 73 | (-289, -150) | the same rise, approached from the north | pass in E |
      | 24 | (93, 321) route-obstacle | the fill's rise over the lanelets at (101, 325), 0.3-0.5 m | still stops in E, below |
      | 40 | error: no localization at the start | spawned at lanelet ele 9.47 in the ramp trench at (470, 165), under n11's road (10.4) | still fails in E, below |
      | 27, 81 | (469-472, 150-155) AEB, MRM | the ramp down at (469, 155-165): trench walls 0.6-1.3 m nearer the lane than the PCD's; the road a metre over the lanelets | still stops in E, below |
      | 58, 65, 89, 110 | inside a tight connector, at the start, no velocity factor | path_optimizer's "outside drivable area" stop: the car's swept footprint leaves the lanelets' drivable area in 83-84 deg turns of 10-14 m (1007306, 1011371, 1011115, 1007796) | 89 passes in E; 58, 65, 110 pass with the profile change (runs F, G, below) |
      | 03 | (81, 312), no velocity factor | the hairpin 1003951 -> 1004230, about 5.5 m across; the model3 turns no tighter than 3.6 m (rear axle) / 4.8 m (outer front corner) | not drivable by this car; recorded in tour_legs_missing.json |
      | 98 | error: Autoware stayed in PLANNING | [63, 2063, 206], goal at the SW scan corner (-289, -164) | pass in E |

      Isaac's tour (059, 37c0dd9 legs, old ground) stopped at 03, 58 and 65 in the same places,
      which puts those stops on the map or the planner, not the simulator.
    - **Reruns of the failed legs** (E: map c2854a7c, ground_n12, cd55754; F and G: the same with
      the profile change below):

      | leg | E (n12, c2854a7c) | F (expansion 0.3) | G (expansion 0.15, committed) |
      |---|---|---|---|
      | 02, 71, 73, 74, 89, 98 | pass | - | - |
      | 03 | 154 s at (81.2, 312.4), path_optimizer outside drivable area | - | - |
      | 24 | 121 of 130 m, then 180 s at (95.5, 323.4), obstacle_stop | - | - |
      | 27 | 60 of 72 m, then 180 s at (468.9, 152.9), obstacle_stop | - | - |
      | 40 | error at the start, no localization | - | - |
      | 58 | 5 of 26 m, outside drivable area | pass | pass, 21 s |
      | 65 | 3 of 53 m, outside drivable area | pass | pass, 24 s |
      | 81 | 35 of 47 m, then 181 s at (470.2, 152.1), obstacle_stop | - | - |
      | 110 | 3 of 22 m, outside drivable area; MRM after 150 s standing | pass | pass, 19 s |

      With D, E and G that is **109 of 114 legs**, plus r01 and r02, and no MRM while moving.
      Every lanelet is on a leg except 1011236. Not driven: the hairpin join 1003951 -> 1004230
      (03), and the stretches 24, 27, 40 and 81 stopped short of (below).
    - **The profile change** (autoware_sim_vehicle a9bd75c; the user chose option (a) and
      left (b) off; it applies to every scenario, not only this map). The profile carries its
      own copy of `drivable_area_expansion.param.yaml` with `ego.extra_width` and
      `ego.extra_front_overhang` at 0.15 m (upstream 0.0), passed as
      `drivable_area_expansion_param_path`. 0.3 passed too; 0.15 is the smallest tried that
      passes 58, 65 and 110. Town01's regression set (`basic/town01_*`), before and after:
      the same 5 of 6 pass; `town01_traffic_light` fails on its 300 s timeout both times.
      acb's deps pin moved with it (102b596); aib's is 059's to move.
    - **What remains**:
      - 03: the hairpin 1003951 -> 1004230 is about 5.5 m across; the model3 turns no tighter
        than 3.6 m (rear axle) / 4.8 m (outer front corner). Not drivable by this car; listed in
        `tour_legs_missing.json`. Isaac stopped there too.
      - 24: a 0.7 m object at (102.9, 328.2), about 1 m from the goal at the scan's north edge.
      - 27, 81: obstacle_stop on the ramp at (469-470, 152-153), the trench walls nearer the lane
        than in the PCD.
      - 40: NDT does not converge at its spawn in the ramp trench (yaw off by 0.40 rad).
    - Tools: `tour_monitor.py`, `tour_lidar_diff.py`, `tour_clock.py`, `tour_table.py` in
      `scripts/ntu_carla/`. Runs: `/mnt/p5plus/aeon/ntu-twin/tour/runs_{B,C,D,E,F,G}` (videos,
      state.jsonl, monitor.jsonl). Issue stills and videos: NAS `dataset/2026-10-30 NTU twin
      map/carla_tour/issues/`.
    - Run D, every leg (driven / planned m, time s, MRM, NVTL min):

      <details><summary>116 rows</summary>

      | leg | result | m | s | MRM | NVTL min |
      |---|---|---|---|---|---|
      | r01 | pass | 344 / - | 107 | 0 | 2.85 |
      | r02 | pass | 343 / - | 128 | 0 | 3.07 |
      | 00 | pass | 1158 / 1167.3 | 327 | 0 | 2.9 |
      | 01 | pass | 391 / 396.5 | 147 | 0 | 3.05 |
      | 02 | no junit | 5 / 436.3 | 397 | 1 | 2.61 |
      | 03 | no junit | 0 / 466.5 | None | 0 | 3.1 |
      | 04 | pass | 145 / 150.7 | 51 | 0 | 2.97 |
      | 05 | pass | 143 / 148.2 | 49 | 0 | 2.93 |
      | 06 | pass | 99 / 105.2 | 48 | 0 | 3.0 |
      | 07 | pass | 95 / 101.3 | 44 | 0 | 2.98 |
      | 08 | pass | 143 / 147.9 | 59 | 0 | 3.04 |
      | 09 | pass | 99 / 104.6 | 46 | 0 | 3.04 |
      | 10 | pass | 154 / 159.2 | 47 | 0 | 2.69 |
      | 11 | pass | 164 / 168.4 | 67 | 0 | 2.64 |
      | 12 | pass | 119 / 124.8 | 49 | 0 | 2.7 |
      | 13 | pass | 116 / 122.0 | 46 | 0 | 2.88 |
      | 14 | pass | 62 / 67.0 | 30 | 0 | 2.89 |
      | 15 | pass | 54 / 58.5 | 27 | 0 | 3.07 |
      | 16 | pass | 54 / 58.5 | 28 | 0 | 3.08 |
      | 17 | pass | 206 / 211.1 | 75 | 0 | 2.79 |
      | 18 | pass | 97 / 102.9 | 44 | 0 | 2.8 |
      | 19 | pass | 49 / 53.9 | 26 | 0 | 3.05 |
      | 20 | pass | 48 / 53.2 | 24 | 0 | 3.04 |
      | 21 | pass | 48 / 53.2 | 25 | 0 | 3.03 |
      | 22 | pass | 96 / 101.7 | 42 | 0 | 3.04 |
      | 23 | pass | 78 / 82.6 | 30 | 0 | 2.73 |
      | 24 | no junit | 118 / 130.3 | 197 | 0 | 3.0 |
      | 25 | pass | 122 / 128.1 | 53 | 0 | 2.98 |
      | 26 | pass | 30 / 35.1 | 20 | 0 | 3.0 |
      | 27 | no junit | 62 / 71.7 | 188 | 2 | 2.91 |
      | 28 | pass | 25 / 30.2 | 18 | 0 | 3.04 |
      | 29 | pass | 28 / 32.6 | 18 | 0 | 3.03 |
      | 30 | pass | 62 / 67.0 | 31 | 0 | 3.05 |
      | 31 | pass | 90 / 95.6 | 43 | 0 | 3.05 |
      | 32 | pass | 22 / 27.1 | 18 | 0 | 2.96 |
      | 33 | pass | 46 / 50.5 | 25 | 0 | 2.98 |
      | 34 | pass | 68 / 72.5 | 32 | 0 | 3.03 |
      | 35 | pass | 19 / 23.8 | 15 | 0 | 2.99 |
      | 36 | pass | 19 / 23.9 | 18 | 0 | 2.92 |
      | 37 | pass | 21 / 25.7 | 17 | 0 | 3.08 |
      | 38 | pass | 18 / 22.8 | 14 | 0 | 3.15 |
      | 39 | pass | 68 / 74.0 | 29 | 0 | 3.07 |
      | 40 | error | 0 / 62.1 | 44 | 0 | 3.07 |
      | 41 | pass | 60 / 65.1 | 32 | 0 | 2.63 |
      | 42 | pass | 26 / 31.2 | 23 | 0 | 2.63 |
      | 43 | pass | 48 / 53.3 | 24 | 0 | 3.01 |
      | 44 | pass | 25 / 29.3 | 19 | 0 | 3.06 |
      | 45 | pass | 48 / 53.2 | 25 | 0 | 3.03 |
      | 46 | pass | 44 / 48.7 | 25 | 0 | 3.13 |
      | 47 | pass | 33 / 38.3 | 25 | 0 | 3.13 |
      | 48 | pass | 61 / 66.5 | 30 | 0 | 3.06 |
      | 49 | pass | 35 / 39.9 | 18 | 0 | 3.02 |
      | 50 | pass | 49 / 53.6 | 28 | 0 | 3.02 |
      | 51 | pass | 24 / 29.0 | 17 | 0 | 3.13 |
      | 52 | pass | 49 / 53.7 | 31 | 0 | 3.13 |
      | 53 | pass | 84 / 89.0 | 38 | 0 | 3.05 |
      | 54 | pass | 31 / 36.0 | 21 | 0 | 3.09 |
      | 55 | pass | 24 / 28.6 | 18 | 0 | 3.02 |
      | 56 | pass | 104 / 109.5 | 36 | 0 | 2.66 |
      | 57 | pass | 93 / 98.2 | 41 | 0 | 3.15 |
      | 58 | no junit | 5 / 26.1 | 164 | 0 | 3.13 |
      | 59 | pass | 40 / 44.5 | 22 | 0 | 2.79 |
      | 60 | pass | 33 / 37.4 | 20 | 0 | 2.79 |
      | 61 | pass | 87 / 92.4 | 31 | 0 | 2.91 |
      | 62 | pass | 27 / 31.6 | 18 | 0 | 2.88 |
      | 63 | pass | 33 / 37.7 | 20 | 0 | 2.96 |
      | 64 | pass | 44 / 49.9 | 26 | 0 | 3.13 |
      | 65 | no junit | 4 / 52.9 | 162 | 0 | 2.93 |
      | 66 | pass | 33 / 38.0 | 21 | 0 | 3.13 |
      | 67 | pass | 47 / 51.4 | 26 | 0 | 3.13 |
      | 68 | pass | 52 / 57.2 | 28 | 0 | 3.05 |
      | 69 | pass | 52 / 57.2 | 28 | 0 | 3.06 |
      | 70 | pass | 28 / 32.7 | 22 | 0 | 2.98 |
      | 71 | no junit | 101 / 115.4 | 212 | 0 | 2.55 |
      | 72 | pass | 64 / 69.9 | 32 | 0 | 3.15 |
      | 73 | no junit | 80 / 93.4 | 199 | 0 | 2.58 |
      | 74 | no junit | 5 / 108.7 | 159 | 0 | 2.6 |
      | 75 | pass | 51 / 57.0 | 28 | 0 | 3.04 |
      | 76 | pass | 48 / 53.0 | 28 | 0 | 2.91 |
      | 77 | pass | 70 / 75.7 | 35 | 0 | 2.97 |
      | 78 | pass | 47 / 53.0 | 27 | 0 | 3.09 |
      | 79 | pass | 16 / 21.1 | 15 | 0 | 3.09 |
      | 80 | pass | 47 / 52.8 | 26 | 0 | 3.08 |
      | 81 | no junit | 33 / 47.3 | 229 | 31 | 1.81 |
      | 82 | pass | 209 / 213.8 | 59 | 0 | 2.92 |
      | 83 | pass | 44 / 48.5 | 22 | 0 | 2.98 |
      | 84 | pass | 37 / 42.0 | 23 | 0 | 3.0 |
      | 85 | pass | 68 / 73.8 | 35 | 0 | 3.06 |
      | 86 | pass | 19 / 23.7 | 16 | 0 | 3.1 |
      | 87 | pass | 47 / 52.2 | 28 | 0 | 3.03 |
      | 88 | pass | 18 / 22.7 | 16 | 0 | 3.1 |
      | 89 | no junit | 1 / 24.7 | 154 | 0 | 3.02 |
      | 90 | pass | 21 / 25.4 | 18 | 0 | 3.1 |
      | 91 | pass | 68 / 74.0 | 35 | 0 | 3.11 |
      | 92 | pass | 25 / 30.0 | 21 | 0 | 3.07 |
      | 93 | pass | 74 / 79.5 | 37 | 0 | 3.0 |
      | 94 | pass | 79 / 84.4 | 38 | 0 | 3.05 |
      | 95 | pass | 24 / 29.1 | 15 | 0 | 3.05 |
      | 96 | pass | 53 / 59.0 | 29 | 0 | 3.1 |
      | 97 | pass | 18 / 22.5 | 15 | 0 | 3.1 |
      | 98 | error | 0 / 110.1 | 120 | 0 | 3.13 |
      | 99 | pass | 18 / 23.2 | 15 | 0 | 3.17 |
      | 100 | pass | 97 / 101.5 | 43 | 0 | 3.15 |
      | 101 | pass | 55 / 60.4 | 29 | 0 | 3.11 |
      | 102 | pass | 38 / 43.3 | 22 | 0 | 3.11 |
      | 103 | pass | 87 / 91.5 | 37 | 0 | 3.11 |
      | 104 | pass | 119 / 123.5 | 38 | 0 | 3.08 |
      | 105 | pass | 30 / 34.8 | 18 | 0 | 3.01 |
      | 106 | pass | 19 / 24.2 | 14 | 0 | 3.03 |
      | 107 | pass | 28 / 33.7 | 17 | 0 | 3.11 |
      | 108 | pass | 36 / 41.2 | 18 | 0 | 3.08 |
      | 109 | pass | 132 / 137.2 | 41 | 0 | 3.07 |
      | 110 | no junit | 3 / 22.5 | 160 | 0 | 3.1 |
      | 111 | pass | 33 / 37.5 | 17 | 0 | 3.05 |
      | 112 | pass | 120 / 125.2 | 41 | 0 | 3.06 |
      | 113 | pass | 37 / 42.3 | 17 | 0 | 2.92 |

      </details>

Next: the four tour stops left (24's object at the goal, the ramp at (469, 153) for 27 and 81,
NDT at 40's spawn in the ramp trench); the hairpin at 03 needs a wider join in the Lanelet2
(agreed with 059 first) or stays off the car's routes. Then the site package on the NAS, and
the rest of the parity list with 059 (P4 signals; P6's recorder mounts are in,
`carla_watch_video.py --mounts`).

## Parity with 059 (the NTU twin on Isaac)

The user asked on 2026-11-04 for the two twins to ship the same features with comparable
outcomes: [059](059-ntu-twin-on-isaac.md)'s on Isaac (swb 118) and this one on CARLA 0.9.16
(swb 133). The two sessions agreed the list that day. 059 keeps the same list under its own
"Parity with 072". Status as of 2026-11-04:

| | what both twins provide | owner | CARLA (072) | Isaac (059) |
|---|---|---|---|---|
| P1 | The whole site (167 tiles) in the map frame, origin (52500, 67700, 0). One Autoware map dir: 059's Lanelet2, `pointcloud_map.pcd` and projector | both | have (`carla/NTU` links to 059's) | have |
| P2 | The ego: `tesla_model3` from `sim_vehicle_spec`, the same sensor kit, Autoware profile and overlay. Each simulator has its own lidar (CARLA raycast, RTX), so NDT's input differs | both | have | have |
| P3 | One scenario set. `ntu_twin_<case>.xosc` is the source, and `scripts/ntu_twin/scenario_pair.py` writes `ntu_carla_<case>.xosc` from it (only the map dir, the header and the Story name differ; `--check` finds drift). Cases: r01, r02, r01_lead, r01_pedestrian. NPC cases to come: a cut-in, oncoming traffic at a turn, a signalised junction once P4 is in. Positions that need moving for one simulator are changed in the `ntu_twin_` source | 118 | all four pass (item 10) | r01 and the two NPC cases pass; r02 to run |
| P4 | Traffic lights, stop lines and crosswalks where scenarios need them: Lanelet2 (agreed before either session edits it) and the matching OpenDRIVE signals | Lanelet2 both, OpenDRIVE 133 | none yet | none yet |
| P5 | The same picture targets: sun at elevation 55 deg, azimuth 135 (CARLA's 225 = map azimuth + 90), the same default weather, exposure judged on the four fixed probe views below. An asset fix is adopted by both: ortho refill, de-lighting, shards, parked vehicles left out (`parked_vehicles.py`, item 7), low vegetation as a height field (`hedges.py`, item 9) | both | have; exposure ev -1.5, min_bright 7 | have, except parked vehicles and hedges (adopting) |
| P6 | The same recording per run: chase and front views at 20 fps, a side-by-side mp4 1600 px wide, and `state.jsonl` (ego x, y, z and speed, map minus origin). Mounts in base_link (the rear axle): chase (-7.0, 0, 3.4), pitch 15 deg down, 80 deg; front = camera6 (0.9, 0, 2.0), pitch 0.86 deg down, yaw 2.09 deg right, 74.5 deg, 1280 x 720. On CARLA those sit on the actor's origin, 1.386 m ahead of the model3's rear axle: chase x -8.386, front x -0.486 (height to check) | both | to do: mounts, `state.jsonl`, 1600 px video (`carla_watch_video.py`) | have |
| P7 | The same outcome measures: junit result, time to goal, NDT NVTL min and mean along r01 and r02 (`ndt_track.py`), MRM and emergency count, speed behind the lead, stop distance and minimum gap to the pedestrian, and the NPC check (`scripts/ntu_twin/ssv2_watch.py`: SSv2's view of each entity and event, simulator-agnostic) | 118 the scripts, each its own runs | have, except NVTL | have |
| P8 | Camera stacks (059 N9): Vision Pilot and a VLM agent, open loop on P6 recordings first, one harness for both | 118 first | later | not started |
| P9 | Distribution on the NAS: each simulator's package with a README (install, frame, versions), and one comparison page, CARLA beside Isaac (probe views, P7 numbers) | both | site package pending | world on the NAS |

The fixed probe views (P5): map minus origin, yaw in degrees counter-clockwise from +x, as
`carla_probe.py --view` and `render_probe.py --view` take them:

- `start,314.3,118.3,91`
- `turn2,135.2,186.4,180`
- `canopy,270.5,186.1,-180`
- `palms,77.6,165.4,-88`

Each is a chase camera 7 m back along the yaw and 3.4 m over the ground (a ray cast at x, y),
pitch 15 deg down, 80 deg across, 1280 x 720.

Split:
- 118: the scenario template (P3), the watcher and metrics scripts (P7), the Isaac side.
- 133: the OpenDRIVE signals (P4's CARLA half), CARLA's exposure, and the CARLA recorder
  matching P6.
- Neither session changes `scripts/ntu_twin/` functions without telling the other.

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
