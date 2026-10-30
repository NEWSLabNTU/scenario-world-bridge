# Phase 062: The Roads around NTU from Public Data, Audited

This phase builds an Autoware Lanelet2 map, a point-cloud map and an Isaac world for the roads
around NTU from public data alone. It is the first site of the study
[`survey-open-data-maps.md`](../design/survey-open-data-maps.md).

The map is used in simulation first and moves to a real car later. Every attribute records
where it came from. An audit compares the map with ground truth: 059's survey now, and the
car's own drives later.

**Status**: planned, 2026-10-30.

**Source**: user direction 2026-10-30: "Find a place somewhere in Taipei where lane information
is complete. Start from simulation and move to real later. We can have a refinement and audit
process in real car deployment." Decision (user, same day): the site around NTU.

No place in Taipei has complete lane tags in OSM (the survey's measurement: `turn:lanes` is on
2 % of signal approaches citywide, and no junction is fully tagged). So completeness is
something this phase has to produce. The site was chosen because it can be audited.

**Depends on**: [058](058-touge-course-from-open-data.md) (the open-data generator,
`swb_isaac_world/course/`, its frames and its PCD from meshes),
[059](059-ntu-twin-on-isaac.md) (the vendor's MLS and its 2 cm orthophoto, the audit's ground
truth), [019](019-ntu-campus-twin.md) (the frame chain: EPSG:3826 to MGRS 51RUH).

## The site

The box is 25.012-25.024 N, 121.530-121.548 E: the NTU campus and the arterials around it.

- **Arterials:**
  - 羅斯福路四段, with its centre bus lanes (羅斯福路大客車專用道);
  - 新生南路三段;
  - 辛亥路二段 and 三段, with the ramps of the 建國 viaduct;
  - 基隆路四段, with its viaduct;
  - 復興南路二段;
  - the signalised junctions between them.
- **The campus's own roads** (椰林大道, 舟山路, ...): no signals, two lanes, low speed, and a
  first target for a real car.
- **OSM coverage**, as measured in the survey: lanes on 95 % of major-road length, speed limits
  on 55 %, turn lanes on 28 % of signal approaches; 59 signal nodes and 189 crossings in the
  box.
- **Ground truth:** 059's MLS covers 25.0147-25.0207 N, 121.5329-121.5440 E. That is the campus
  and its edges on 新生南路 and 羅斯福路. The arterials beyond it, such as 辛亥路's north side
  and 基隆路, have no ground truth until the car drives them.

**Taiwan's rules:** traffic drives on the right. Scooters have lanes, two-stage left-turn boxes
and waiting zones. Those are mapped as lanelet subtypes or areas that Autoware ignores for a car,
but they are kept, so the car does not stop inside them. A speed limit applies only where a
sign posts it; otherwise the urban default is 50 km/h.

## Rules for the data

- **Public sources only.** OSM, Overture, Mapillary, NLSC's open layers, MOI's DTM, NLSC's 3D
  buildings (if their terms allow). 059's vendor data is **never an input**. It is used only to
  audit, and that audit rehearses the one a real car will do.
- **Provenance on everything.** Every lanelet attribute and every regulatory element carries
  `swb:source` (`osm`, `overture`, `mapillary`, `imagery`, `rule`, `audit`), plus the OSM way or
  feature id it came from. A `rule` value is a default chosen by a stated rule, such as "the
  rightmost lane turns right"; it is never presented as observed.
- **Corrections are patches.** An audit finding, from 059 now or a drive later, becomes a patch
  file with `swb:source=audit`, the evidence (where and what), and a date. The patch is applied
  over the generated map, so a regeneration from newer OSM keeps it. Where the licence fits, a
  correction also goes back to OSM as an edit.

## Work items

**O1. Fetch.** The following, cached under `data/maps/ntu/raw/` (ignored by git):

- OSM by Overpass, every way and node in the box with all its tags;
- Overture's transportation and buildings themes for the box (GeoParquet over DuckDB);
- Mapillary:
  - images and their poses;
  - map features: traffic signs (speed limits, turn-only signs) and points (traffic lights,
    lamps);
  - detections of painted arrows;
  - it needs a Mapillary client token, a host setting;
- NLSC's open WMTS layers that cover the box, at their deepest zoom (their resolution is to be
  measured);
- MOI's 20 m DTM;
- NLSC's 3D buildings if they can be downloaded.

**O2. Baseline (route A).** OSM goes through `netconvert --osm.turn-lanes` to OpenDRIVE, then
through `crdesigner` to Lanelet2, then through the `repair_lanelet_*` scripts. The result is
loaded in Autoware's map loader and in SSv2's. Recorded: what loads, the node count, the
junctions it gets wrong. This is the yardstick for O3.

**O3. Generator (route B).** OSM and Overture straight to Lanelet2, extending `course/` from one
road to a network:

- **Lanes:** each lane's bounds are offset from the OSM centre line by the lane-count tags and
  widths. Dual carriageways, which Taipei's arterials are, come in as two one-way ways; the
  median between them is measured from the two centre lines.
- **Junctions:** each is cut back to a junction area. Every allowed entry→exit pair gets a
  connecting lanelet, and `turn_direction` is set from the heading change.
- **Allowed pairs:** from `turn:lanes`, turn-restriction relations and Overture's
  `prohibited_transitions`, then Mapillary's turn signs and arrows, then the rule. Each pair
  says which of these decided it.
- **Speed limits:** OSM, then Overture, then a Mapillary sign on the road, then the rule.
- **Signals, stop lines, crosswalks:** taken from the signal nodes and the crossings.
- **Bus and scooter lanes:** kept as their own lanelets.
- **Frame:** `local_x`/`local_y` in 059's frame (MGRS 51RUH, origin 52 500, 67 700), so the
  audit compares like with like and the real car's maps share it.
- **Checks:** the repair scripts and Autoware's map validator run on the output and should find
  nothing.

**O4. Audit against 059 (simulation).**

- **Lane lines:** the painted lines are extracted from the 2 cm orthophoto. For each generated
  bound, its offset to the nearest line (median, 95th percentile).
- **Junctions:** the paths that the arrows and signs allow, against the generated set. Each
  stop line against the painted one.
- **Speed limits:** each limit against the posted signs.
- **Output:** an audit report per lanelet and per junction, and the corrections as patches
  (`swb:source=audit`).
- **Question to answer:** how far public data alone gets, by attribute, before the audit
  touches it.

**O5. The world and the PCD.**

- **Terrain:** the DTM, flattened to the road profile, with the open orthophoto as its texture.
- **Roads:** ribbons from the lanelet bounds, with markings drawn from the bound types and the
  arrows.
- **Buildings:** NLSC's or OSM's.
- **Props:** signs and signals at their OSM and Mapillary positions, trees from OSM.
- **PCD:** sampled from the meshes, as in 058.
- Labelled as SimReady towns are, so 054 and 056's surfaces work.
- This world looks realistic but is not a twin. 059's world is the twin, and both share one map
  frame.

**O6. Simulation validation** (needs a GPU with about 8 GB free; 243 or 241 when free):

- the world loads;
- NDT holds along a loop through every junction type in the box (NVTL at least 2.5);
- Autoware routes and drives the loop with SSv2: turn signals at every tagged turn, the
  intersection module at every junction, stops at red lights.

Run twice: the generated map alone, and with the audit's patches applied, so the patches show
what they are worth.

**O7. Real car (design only in this phase).** The refinement and audit loop for deployment:

- the car drives with the patched map;
- its lidar and cameras are compared with the map: lane lines, arrows, signs, stop lines,
  NDT's per-scan score;
- each discrepancy becomes a candidate patch with its evidence (a scan, an image, a pose);
- a person approves it, and the map is regenerated.

The PCD for the real car is the car's own (or 059's on campus), not the meshes' samples. What
transfers from simulation is the Lanelet2 map, its provenance and the audit tooling.

## Steps

1. O1 and O2 (offline, no GPU).
2. O3, with O2 to beat: every junction in the box routed, `turn_direction` on every path, a
   speed limit on every lanelet, each with its source.
3. O4: the audit report and the first patches.
4. O5, then O6 when a GPU is free.
5. O7 written up with the car's sensors and the lab's deployment plan.

## Risks

- **OSM's centre lines on dual carriageways and viaducts.** The two directions of 羅斯福路 and
  the 基隆路 viaduct overlap in plan. The levels must be kept apart, as 057's walkways under the
  ring were.
- **Mapillary's coverage in the box is unknown.** If arrows are not detected, `turn:lanes` falls
  back to the rule, and the audit measures how wrong the rule is.
- **The open orthophoto may be too coarse** to show lane lines. Then route C is not possible from
  public data in Taiwan, and the audit's patches carry more of the work.
- **Scooter-heavy traffic** is not something SSv2's scenarios or Autoware's planner model well.
  It is out of scope for the map; noted for the scenarios.
