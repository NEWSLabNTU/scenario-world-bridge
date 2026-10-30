# Survey: Autoware and Isaac Maps from Public Data

This study asks whether a real place can be made into a driving world from public data alone:
OpenStreetMap, aerial or satellite imagery, and public street-level photos. Three outputs are
needed:

- an Autoware Lanelet2 map with turn directions, intersection paths and speed limits;
- a point-cloud map that NDT can localize against;
- an Isaac Sim world.

It generalises roadmap [058](../roadmap/058-touge-course-from-open-data.md), which built one
mountain road from OSM, Japan's GSI 5 m DEM and the GSI aerial photo. That road has no
junctions, no signals and one lane each way. This study looks at towns, where the hard parts
are the junctions.

**Status**: study, 2026-10-30 (user direction: "find way to generate Autoware lanelet2 and pcd
maps, and Isaac Sim map ... OSM and satellite image sources, and public street view images.
The lanelet should have turning direction, intersection path configuration and best if there
is speed limit").

## What Autoware needs from the Lanelet2 map

The CARLA map pack taught this the hard way. Each item below is a repair script in `scripts/`,
written because a map lacked it.

| Need | What Autoware does with it | Repaired in the CARLA maps by |
|---|---|---|
| `turn_direction` (`straight`/`left`/`right`) on every junction lanelet | Without it, the `intersection` and `blind_spot` modules don't launch, and the turn signal never blinks. | `repair_lanelet_turn_direction.py` |
| Junction connectivity: one lanelet per entry-to-exit path, chained to its predecessor and successor lanelets | Routing and the intersection module's attention lanes depend on it. | `repair_lanelet_joins.py` |
| `speed_limit` on every road lanelet | Without it, Autoware falls back to its default and drives a highway at 50 km/h (057, laps 10-13). | `repair_lanelet_speed_limits.py` |
| Traffic-light regulatory elements, each with its stop line and the lanelets it controls | Without the right links, the planner treats a signalised junction as unsignalised. | `repair_lanelet_traffic_lights.py`, `_signal_links.py`, `_far_traffic_lights.py` |
| `right_of_way` elements that name a lanelet with right of way | If any element lacks one, SSv2's loader refuses the whole map. | `repair_lanelet_right_of_way.py` |
| Crosswalks and walkways on their real level | The car stops for footpaths under a bridge (057, lap 12). | `repair_lanelet_walkways_under_roads.py`, `_subtypes.py` |
| `local_x`/`local_y` on every point, and `map_projector_info.yaml` | Autoware needs one map frame shared with the simulator. | `repair_lanelet_local_coords.py` |

A generator should produce all of these directly. The repair scripts then act as its checks
and should find nothing to fix.

## The sources

| What | Source | What it gives | Terms |
|---|---|---|---|
| Road graph and lane attributes | OpenStreetMap (Overpass) | centre lines, `lanes`, `lanes:forward/backward`, `turn:lanes`, `maxspeed`, `oneway`, `width`, turn-restriction relations, `highway=traffic_signals`, crossings, `building`/`building:levels` | ODbL: attribution, and share-alike for a derived database |
| The same, filled in | Overture Maps transportation | segments with `speed_limits` (optionally per part of a segment), `lanes`, `prohibited_transitions`, `road_surface`; mostly from OSM, with TomTom data added since 2024-09 | CDLA-Permissive / ODbL, depending on the layer |
| Aerial imagery (Japan) | GSI `seamlessphoto` | about 0.5 m per pixel at z18 (058 used it) | GSI terms, with attribution |
| Aerial imagery (Taiwan) | NLSC | The open WMTS (e-Map, OGDL 1.0) is a general map, not a photo. The orthophotos, including those from airborne LiDAR, are supplied under NLSC's fee standards. The open layers' zoom limits need checking. | OGDL 1.0 for the open layers |
| Satellite imagery | Sentinel-2 (10 m), or commercial | Too coarse for lane markings; only good for land cover. | open / commercial |
| Terrain (Japan) | GSI `dem5a` | 5 m laser DEM | GSI |
| Terrain (Taiwan) | MOI, on data.gov.tw | 20 m DTM and DSM, 2025 edition; 100 m DEM. A 1 m LiDAR DEM exists but is not open. | OGDL 1.0 |
| Buildings (Taiwan) | NLSC 3D building models (3dmaps.nlsc.gov.tw) | LOD1-LOD2, as 3D Tiles and I3S; the 2022 edition was announced for open download in 2023 | to confirm |
| Buildings, anywhere | OSM / Overture buildings | footprints with heights or level counts, where mapped | ODbL / CDLA |
| Street-level photos | Mapillary | Images (CC-BY-SA). Per-image detections, plus map features triangulated from several images: about 1,500 traffic-sign classes (speed limits such as `regulatory--maximum-speed-limit-50--g1`) and points such as street lamps. Coverage is uneven; bbox queries return at most 2,000 features per page. | CC-BY-SA 4.0 |
| Street-level photos | KartaView | Images; less coverage in Taiwan than Mapillary, to check | CC-BY-SA |

**Not usable.** Google Maps/Earth imagery, Street View and Photorealistic 3D Tiles: their terms
forbid deriving maps from them or caching their content. Bing imagery may be used to trace OSM
edits, but not for a derived product of our own.

## Lanelet2 generation, three routes

**A. Off-the-shelf chain:** OSM → SUMO `netconvert` → OpenDRIVE → CommonRoad Scenario Designer
(`crdesigner`) → Lanelet2 → our repair scripts.

- `netconvert --osm.turn-lanes` builds a junction's lane-to-lane connections from `turn:lanes`.
  Without that option it guesses them and loads only turn restrictions. Where some lanes are
  marked and others aren't, the unmarked ones become through lanes.
- `--opendrive-output` writes OpenDRIVE 1.4. A change in lane count starts a new road (it does
  not model widenings). Signals and objects carry little information.
- CARLA's own OSM import (`carla.Osm2Odr`) and the `osm-to-xodr` package are this same
  netconvert step.
- crdesigner converts OpenDRIVE to Lanelet2 and has an Autoware option (elevation on). Autoware
  discussion #6019 reports that its maps carry far too many nodes per way, enough that Autoware
  hangs. The output would need decimating.
- **Strengths:** the whole chain exists today, and the junction connections come from a tool
  that 20 years of traffic simulation has exercised.
- **Weaknesses:**
  - Lane geometry is a guess from the centre line, the lane count and a default width.
  - Junction shapes are netconvert's.
  - Three format hops, each with its own losses.
  - `turn_direction`, the stop lines and the traffic-light links would still come from our
    repair scripts.

**B. Our own generator, extending 058's `course/`:** OSM (and Overture) → Lanelet2 in one step.

- **Lane bounds:** each lane is a fixed offset from the OSM centre line, using `lanes`,
  `lanes:forward/backward` and `width`, or else a country default (Taiwan: 3.5 m on arterials,
  3.0-3.25 m on local roads).
- **Junction paths:** the OSM junction node is replaced by a junction area. Each approach is
  cut back to the area's edge, and each allowed entry→exit pair gets a connecting lanelet.
  - Its bounds are clothoids or cubic curves between the cut lanes.
  - Which pairs are allowed comes from `turn:lanes`, turn-restriction relations and Overture's
    `prohibited_transitions`. Where none exist, the default is: rightmost lane turns right,
    leftmost lane turns left, all lanes may go straight.
- **`turn_direction`:** set from the heading change across each connecting lanelet (as
  `repair_lanelet_turn_direction.py` does).
- **`speed_limit`:** taken in this order:
  1. OSM `maxspeed`;
  2. Overture `speed_limits`;
  3. a Mapillary speed-limit sign on that road;
  4. the legal default for the road class and country (Taiwan: 50 km/h urban unless posted).

  Every value records which of these it came from.
- **Traffic lights:** each `highway=traffic_signals` node becomes one regulatory element per
  approach, with a stop line where the approach is cut back. The light poles are placed at
  Mapillary's triangulated traffic lights where they exist.
- **Crosswalks:** taken from `footway=crossing` ways and `crossing` nodes.
- **Map frame:** the local transverse Mercator frame that 058 already uses, with
  `local_x`/`local_y` on every point.
- **Strengths:**
  - Lanelet2 is the only format involved.
  - Every Autoware need above is produced by construction.
  - Node density is under our control.
  - The output, Isaac's road meshes and the PCD all share one geometry.
- **Weakness:** the junction builder is the work, and it is where netconvert spent years.

**C. Geometry corrected from imagery:** routes A and B place lanes from OSM's centre lines, so a
lane can be a metre or more from the real road. Imagery can correct that.

- Lane markings segmented from an aerial photo can pull the lane bounds onto the painted lines.
  Published models for this:
  - SkyScapes and Aerial LaneNet (DLR);
  - a 2024 benchmark of twelve models, in which transformers did best;
  - an aerial lane-level dataset of about 80 km with a baseline (arXiv 2312.13449).
- SIO-Mapper (arXiv 2504.09882, 2025) builds lane-level HD maps from satellite images and OSM
  together, with no site visit, and is the closest published work to this study.
- Turn arrows painted in a lane, read from the aerial photo or from Mapillary's detections, can
  fill in `turn:lanes` where OSM lacks it.
- C needs imagery at about 0.3 m per pixel or finer. In Taiwan that means NLSC's orthophotos
  (fee-based), not the open layers. C refines the output of A or B; it does not replace them.

**Recommendation:**

1. Start with A as the baseline. It costs a day and shows how far existing tools get.
2. Build B as the product.
3. Add C where the measured error says it is needed.

## The point-cloud map

No open lidar exists for most places: NLSC's LiDAR point clouds are fee-based, and Japan's are
published only for some areas. For simulation the PCD is therefore sampled from the Isaac
world's own meshes, as 058 does: the road, terrain, kerbs, buildings, poles and trees, voxelised
at 0.2 m. NDT's map then matches exactly what the simulated lidar sees.

Two caveats:

- The map is good for simulation only. NDT on a real car needs a real survey (the vendor's MLS,
  as in 059).
- A world made of extruded boxes gives NDT less structure than a real street. The PCD should
  carry building facades, poles and kerbs, not just the ground, or NDT will slide along straight
  roads.

## The Isaac world

This reuses 058's pipeline and adds a town's features:

- **Terrain:** DEM plus an aerial-photo texture, as 058 does. Taiwan's 20 m DTM is coarse; in a
  flat town, the road profile decides the result more than the DEM does.
- **Roads:** ribbons from the Lanelet2 bounds, so the meshes and the map cannot disagree.
  - Lane lines from the bound types.
  - Kerbs and sidewalks from OSM `sidewalk=*` and the road's width.
  - Arrows and stop lines from the generated map.
  - Labelled the way SimReady labels CARLA's roads, so surface classes (054/056) work unchanged.
- **Buildings:**
  - In Taiwan, NLSC's LOD2 models (3D Tiles converted to USD) where their terms allow it.
  - Elsewhere, OSM/Overture footprints extruded to their height.
  - Facades get generic materials first. Facade textures from street photos come later and are
    research (see below).
- **Props:**
  - Traffic lights and signs placed from OSM and Mapillary's triangulated features, using the
    CARLA Isaac set's assets.
  - Trees from OSM `natural=tree`, tree rows and land use.
  - Street lamps from Mapillary's points layer.
- **Photoreal surroundings from street photos:** 3D Gaussian splatting from sparse street-level
  views is an active area: SGD (a diffusion prior for sparse views), Street Gaussians, HUGSIM.
  None of these reconstructs from crowdsourced Mapillary sequences. Isaac renders splats only
  through recent Omniverse extensions, and lidar does not see them. This is out of scope for a
  first version.

The result looks like a believable town, with the real layout and the real ground texture
beneath it. It is not a twin of the real streets. Where that realism matters, 059 (the vendor's
coloured MLS) is the route.

## How to tell whether it worked

The NTU campus and the streets around it are the proposed test area, because 059 already holds
ground truth for them: a 5.5-billion-point coloured MLS survey and its 2 cm orthophoto, which
shows every lane line, arrow and kerb (`/mnt/toshiba/aeon/ntu-map/` on 241). Measurements:

- **Lane geometry:** each generated lane bound's distance to the painted line extracted from the
  2 cm orthophoto (median and 95th percentile), for A, B and B+C.
- **Junctions:**
  - every entry→exit path that the paint and signs allow exists, and no other does;
  - `turn_direction` is right on every path;
  - stop lines are within 1 m of the painted ones.
- **Speed limits:** each lanelet's limit against the posted signs, from Mapillary and the MLS
  colour.
- **Autoware:**
  - the map loads in Autoware and in SSv2;
  - routes exist between every pair of approach lanelets;
  - the intersection module launches at each junction;
  - with the world built, Autoware drives a loop through every junction type (signalised,
    unsignalised, T, roundabout if there is one).
- **NDT:** NVTL along the loop with the generated PCD in the generated world. Also, as a stretch
  goal, the generated PCD against 059's real MLS: does the open-data map localize a scan of the
  real place?

## Open questions

- **Region:** the NTU area (it has ground truth), Haruna (it already exists), or somewhere
  without ground truth.
- **Purpose:** simulation only (the PCD from meshes is enough), or maps a real car could also
  use (real geometry and a real PCD needed). This decides how much of route C is required.
- **NLSC data:** whether its fee-based orthophoto and 1 m DEM can be obtained through NTU. They
  would make route C possible in Taiwan.
- **Licences:** ODbL share-alike applies to a Lanelet2 map derived from OSM if it is published.
  Mapillary-derived features are CC-BY-SA.

## Sources

- SUMO, OSM import (`--osm.turn-lanes`): https://sumo.dlr.de/docs/Networks/Import/OpenStreetMap.html
- SUMO export formats (OpenDRIVE 1.4): https://sumo.dlr.de/wiki/Networks/Export
- osm-to-xodr: https://pypi.org/project/osm-to-xodr/
- CommonRoad Scenario Designer, OSM conversion: https://commonroad-scenario-designer.readthedocs.io/en/latest/details/osm/
- Autoware discussion #6019, OpenDRIVE to Lanelet2: https://github.com/orgs/autowarefoundation/discussions/6019
- FlexMap Fusion (TUM; OSM conflation into Autoware maps): https://arxiv.org/abs/2404.10879
- Overture transportation segments: https://docs.overturemaps.org/schema/reference/transportation/segment/
- Mapillary API, map features: https://blog.mapillary.com/update/2020/08/28/map-data-mapillary-api.html
- Taiwan DTM 20 m: https://data.gov.tw/dataset/169808 ; DEM 100 m: https://data.gov.tw/en/datasets/165890
- NLSC 3D building models (2023 release note): https://landoffice.chcg.gov.tw/files/內政部國土測繪中心%20111%20年度三維建物模型更新及精進%20作業內容及執行成果重點說明_127_1120531.pdf
- Lane markings from aerial images, benchmark: https://arxiv.org/abs/2410.05717 ; dataset and baseline: https://arxiv.org/abs/2312.13449
- SIO-Mapper: https://arxiv.org/abs/2504.09882
- SGD, street view synthesis with Gaussian splatting: https://arxiv.org/abs/2403.20079 ; HUGSIM: https://arxiv.org/abs/2412.01718
