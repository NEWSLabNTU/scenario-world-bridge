# Phase 062: The Roads around NTU from Public Data, Audited

This phase builds an Autoware Lanelet2 map, a point-cloud map and an Isaac world for the roads
around NTU from public data alone. It is the first site of the study
[`survey-open-data-maps.md`](../design/survey-open-data-maps.md).

The map is used in simulation first and moves to a real car later. Every attribute records
where it came from. An audit compares the map with ground truth: 059's survey now, and the
car's own drives later.

**Status**: in progress, 2026-10-31: O1-O3 done (first version); O4's geometry audit run, its paint and junction audits open (newslab-118-server, offline).

Progress:

- **O1, fetched** (`scripts/open_maps/fetch.py scripts/open_maps/sites/ntu.yaml data/maps/ntu`;
  the tool venv is `data/maps/.venv` (uv): duckdb, eclipse-sumo, commonroad-scenario-designer,
  pyproj, shapely, requests, pyyaml, pillow).
  - **OSM:** 17,413 ways, 119,346 nodes, 510 relations (base 2026-10-30).
  - **Overture 2026-10-21.0:** 3,147 segments, 5,421 connectors, 1,812 buildings.
  - **Mapillary:** 17,196 map features and 40,791 images in 792 sequences, 1,064 of them
    panoramas, 7,099 from 2025-26. Among the features:
    - painted arrows: 580 straight, 116 left, 49 right, 77 split right/straight, 52 split
      left/straight;
    - 246 stop lines, 457 zebra crossings;
    - about 650 traffic lights (vehicle heads);
    - 132 no-left-turn and 65 no-right-turn signs, 23 no-U-turn signs;
    - 46 speed-limit-50, 37 speed-limit-40, and a few 20/30/60 signs.

    The API's `layers` parameter does not filter: both layers returned the same set, now
    deduplicated. The images endpoint rejects a large box, so it is fetched in tiles split at
    the 2,000 cap.
  - **NLSC orthophoto (WMTS `PHOTO2`):** real down to z19, 0.27 m per pixel (z20 is z19
    upsampled), one 7,168 x 5,120 mosaic with an "NLSC 2024" watermark. Lane lines and arrows
    are visible where trees do not shade them, so route C is possible from public data here.
  - Not fetched yet: MOI's 20 m DTM, NLSC's 3D buildings.
- **O2, the baseline** (`scripts/open_maps/baseline.py`, scored by
  `scripts/open_maps/check_map.py`). Three fixes were needed to get any output at all:
  1. crdesigner 0.8.5's CLI does not start (a click error), so the script calls its Python
     API.
  2. It parses signal ids as integers, and netconvert names them `joinedS_...` even with
     `--numerical-ids`, so they are renumbered.
  3. It ignores OpenDRIVE's header `<offset>`, so the first map landed at 24.8 S, 114.6 E. The
     shift now goes into the georeference, a transverse Mercator with the false easting and
     northing of 059's frame.

  The result (`data/maps/ntu/baseline/`):

  | measure | baseline |
  |---|---|
  | road lanelets | 1,297 |
  | nodes / per way | 310,952 / 107 mean, 2,125 max (97 MB: Autoware discussion #6019's problem) |
  | branch points / junction paths | 221 / 600 |
  | `turn_direction` on junction paths | 0 of 600 |
  | `speed_limit` | 0 of 1,297 |
  | traffic-light elements | 382, each with a stop line |
  | stop-line ways, crosswalks | 0, 0 |
  | dead ends / no predecessor | 71 / 52 |
  | random lanelet pairs routable (Lanelet2) | 80 of 200 |
  | `local_x`/`local_y` | crdesigner's are pseudo-Mercator, not the map frame |

  - It loads in Lanelet2 with no errors and routes.
  - netconvert applied OSM's turn lanes where they agree with its lane counts, and logged
    where they do not: "Ignoring turn sign information for 4 lanes on edge ... with 3 driving
    lanes". It dropped five turn restrictions whose direction it could not determine.
  - Bus and cycle lanes are discarded as "unknown compound" types.
  - Everything Autoware needs beyond the geometry would still have to come from the repair
    scripts. That is the bar O3 has to clear.
- **O3, first version** (`scripts/open_maps/generate.py`; `plot_map.py` draws a map for review).

  **Where it lives.** It is in `scripts/open_maps/`, not in 058's `course/`: lanelet generation
  has nothing simulator-specific, and keeping it beside the fetch and the checks saves a
  submodule round-trip.

  **The frame.** It is MGRS 51RUH local, 059's and the vendor maps' (019). `map_origin` is the
  Isaac world's alone, and the baseline now uses the same frame.

  **What changed on the way:**
  - **Joints.** OSM splits ways at every tag change. Two chains meeting end to end needed short
    connectors (no `turn_direction`); without them almost nothing routed.
  - **Cut-back.** Measuring it from a dual-carriageway cluster's centre gave the main gate a
    55 m circle that swallowed its approaches. Each approach is now cut from its own node in
    the junction.
  - **Mapillary's `aligned_direction`** is the bearing a sign faces. Signs beside this site's
    one-way roads peak at 150-210 deg from the direction of travel. A sign now governs an
    approach only if it faces it, is on its right side, and is within the last 50 m.
  - **Painted arrows assign movements to lanes but never remove one.** A lane voted
    "straight" by a few arrows had lost its left turn, and the largest strongly connected part
    of the lane graph shrank to 5 %. Only OSM restrictions and no-turn signs ban a movement. A
    movement the junction offers that no lane holds goes to the leftmost or rightmost lane
    (`+gap`).
  - **Bridges.** Short ones over the canals stay (舟山路 had been cut in two). Underpasses, and
    major roads raised for more than 60 m, are left out.

  **Arterial connectivity** (same day, user direction "Fix the arterial connectivity"). The
  arterials looked cut off from the campus. Three causes, and one wrong measure:

  - **Projection.** A sign or arrow beyond the end of an approach projected onto its end and
    read as 0 m before the stop line. A sign 570 m away banned a turn at the main gate, and
    arrows from far away voted lanes. Both are now placed against the whole uncut road, and a
    point beyond either end is rejected. Approaches losing a turn to a sign went from 237 to
    16, and lanes assigned by arrows from 837 to 170 (`bans.json` lists each ban with its sign
    and distance).
  - **Stubs.** Short chains from a junction to nowhere were kept as 2 m pieces inside the
    junction, and became exits into dead ends. For example, 新生南路's five-lane piece at the
    main gate ends where only the bus lanes go on. 64 were dropped, repeatedly, until none were
    left.
  - **Bus-only ways and ramps.** 羅斯福路大客車專用道 carries `bus=designated` alone on 12 of
    its ways, so it read as a car road. A bus or psv `designated` with nothing letting cars on
    is now left out. The ramps of the viaducts already left out go too: 27 of them, which had
    led from nothing.
  - **The measure.** The largest strongly connected part counts every lane the box cut off
    from coming back: a lane upstream of where campus traffic joins 羅斯福路 is reachable only
    by a loop around blocks, and that loop leaves the box. `check_map.py` now reports
    *stranded* lanes instead: lanes no car can reach from any way into the map, or that lead
    to no way out. Like Autoware's routing, it counts lane changes across dashed lines.
    `route_matrix.py` routes between landmarks with Lanelet2 itself; the landmarks are in the
    site file.

  **The result,** against the baseline:

  | measure | baseline (A) | generated (B) |
  |---|---|---|
  | road lanelets / crosswalks | 1,297 / 0 | 3,674 / 67 |
  | nodes, per way (mean, max) | 310,952 (107, 2,125), 97 MB | 60,360 (10, 132) |
  | junction paths with `turn_direction` | 0 of 600 | all 1,618 (left 494, right 514, straight 610) |
  | `speed_limit` | 0 | all |
  | traffic lights (stop line, lanelets referring) | 382 (382, 382) | 145 (145, 257) |
  | lanes not reachable from a way in / with no way out | 329 / 156 | 320 / 332 (9 %) |
  | landmark pairs routable (Lanelet2, 8 landmarks) | 0 of 56 (wrong frame then) | 56 of 56 |

  - **Stranded lanes on the arterials:** 羅斯福路 3 of 148, 辛亥路 8 of 194, 新生南路 0, and
    汀州路, 椰林大道, 和平東路 and 復興南路 0-1. Those counted under 基隆路 are its 巷 alleys.
  - **The rest** are mostly OSM's own: two-way roads that dead-end (east 舟山路 ends at its
    pedestrian zone, and the map builds no U-turns), alleys cut into short pieces, and
    two-way service roads meeting a one-way road that leads in.
  - **Loading.** The map loads in Autoware's Lanelet2 extension with the MGRS projector, with
    no errors. Every point lands on its `local_x`/`local_y`, and the traffic lights load as
    `AutowareTrafficLight`.
  - **Junctions:** 341, of which 50 are signalised (35 from OSM's signal nodes, 15 from
    Mapillary's lights).
  - **Where each approach lane's movements came from:**
    - the rule: about 1,110 lanes;
    - Mapillary's arrows: 176;
    - OSM `turn:lanes`: 39;
    - movements nothing held were added (`+gap`).
  - **Signs:** 16 approaches lost a turn to a Mapillary no-turn sign.
  - **Speed limits, by lane:**
    - the rule: 1,018;
    - OSM: 371;
    - Mapillary signs: 42;
    - Overture: 25.

  **Open:**
  - Whether the bans and the arrows' lanes are right is O4's question: the audit against 059.
  - 2 m pieces where alleys are cut very short still give some tiny lanelets.
- **O4, the audit, first part** (2026-10-31).

  **The data.** 059's handover was copied from the NAS to
  `/mnt/renegade/aeon/ntu-map-audit/` (`ground/ortho/`: 167 tiles at 2 cm in RGB, intensity
  and mask; the DTM; `map/`: the vendor's merged Lanelet2). Its frames:
  - **The orthos** are in the vendor's frame V (EPSG:3826 to EPSG:32651, modulo 100 km).
  - **059's map frame** is M = V - (-0.54, +0.20, +17.30).

  The generated map (true MGRS from WGS84) is taken as V. The audit agrees: against the
  vendor's lanelets the generated lanes sit 0.16 m to the right on average, within the vendor's
  own 0.3 m drift.

  The tools: `overlay.py` (a map over the ortho), `audit_vendor.py` (lane centres against the
  vendor's lanelets), `audit_sections.py` and `audit_lines.py` (lane lines against the paint).

  **Against the vendor's lanelets (5.7 km of campus routes):**

  | | generated (B) | baseline (A, recomputed in the MGRS frame) |
  |---|---|---|
  | vendor route with a lane within 5 m, same heading | 68.5 % | 26.8 % |
  | lateral offset of lane centres: median, 90th percentile | 0.97 m, 2.44 m | 0.53 m, 3.02 m |
  | within 0.5 m / 1.0 m | 32 % / 52 % | 45 % / 81 % |

  **What the offsets measure.** The vendor's lanelets are corridors as wide as the road
  (median 4.0 m, 90th percentile 7.1 m). The generated lanes follow OSM's lane counts and
  widths (median 3.0 m). On 椰林大道 (15.5 m, two lanes) the vendor has one 7 m corridor each
  way and the generated map one 4.5 m lane each way along the centre line: about 1.4 m between
  centres, with both on the road. In the overlays the generated lanes lie on the real roads
  across the campus.

  **What it misses.** The 31 % of the vendor's routes with no generated lane are mostly the
  parking aisles and driveways that the generator leaves out (`service=parking_aisle`,
  `driveway`). The vendor's car drives them.

  **Against the paint** (新生南路 northbound, the one arterial stretch inside the survey with
  its paint observed):
  - The four lanes are 3.5 m wide from the class default; the paint puts them at about 3.2 m.
  - So the outer lines drift about 1 m from the paint across the carriageway, while the median
    side holds.
  - Public data has no lane widths here. OSM `width` is on two ways in the box. NLSC's 0.27 m
    orthophoto shows the lane lines and could set them (route C).

  **The automated paint audits are not conclusive yet:**
  - The paint barely shows in 059's intensity (it falls with range from the scanner).
  - In RGB, a peak detector finds paint on the arterials, but also leaves and brick joints on
    the campus roads, which have little paint.
  - The survey covers the arterials only along the campus side.

  **Not audited yet:** the junction paths and the 16 sign bans against the painted arrows and
  the signs. This needs arrow detection in the orthos, or a check by eye at each signalised
  junction inside the survey.
- **Route C, tried** (2026-10-31, user direction "Try it"; `scripts/open_maps/imagery.py`, off
  unless `SWB_IMAGERY=1`).

  **The method.** On the major roads, the brightness across the road in NLSC's open 0.27 m
  photo is averaged in 40 m windows, less its 1.5 m running median. The lane model (width
  2.7-3.8 m, shift +-1.5 m off the OSM line) is fitted to the narrow bright bands.

  **Why it is weak.** At 0.27 m a 15 cm line covers half a pixel and stands 5-10 grey levels
  over the asphalt. Without the detrending, 10 of 188 road pieces fitted; with it, 98 (median
  width 3.10 m).

  **Why it is not trusted.** Neighbouring pieces of one road disagree: 新生南路's way 222745574
  gives widths 2.7-3.8 m and shifts -1.3 to +1.4 m. On 新生南路 northbound, where 059's paint
  puts the median-side line on OSM's line (shift 0), the fit says +0.9 to +1.1 m. 059's survey
  shows arterial paint in only 4 windows of 20 m, the same with the fit as without, too few to
  validate it.

  **So it stays off.** What could make it work:
  - a lane-marking segmentation model on the photo instead of a band detector (SkyScapes /
    Aerial LaneNet; the survey's references);
  - NLSC's fee-based orthophotos at a finer pixel;
  - Mapillary's street-level images, which see the lines from 2 m up.

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
