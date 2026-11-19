# 068: A share of Town03's lanelets sit about 1.3 m off their OpenDRIVE lanes

**Severity**: medium. On those lanes the ego's planned path, a scenario's lanelet poses and CARLA's
road disagree by more than a metre, and SSv2's lane matching sees it. **Status**: open; cause found, fix proposed (2026-11-20).
**Found**: 2026-11-19, swb 241, checking the candidate revision of `jerry73204/carla-town-autoware-maps`
(roadmap 087; issue 067).

## What happens

`scripts/lanelet_opendrive_registration.py` measures each road lanelet against CARLA 0.9.16's OpenDRIVE.
On the Driving lanes, offset in metres:

| map | Town01 | Town02 | Town03 | Town05 | Town10 |
|---|---|---|---|---|---|
| e9411350 (projector scale 0.9996) | 0.159 | 0.123 | 1.446 | 0.106 | 0.060 |
| scale 1.0, repaired (PR #1) | 0.007 | 0.012 | **1.365** | 0.005 | 0.007 |

The scale factor brings every other town under 0.012 m (p95). Town03 still has a p95 of 1.365 m,
with a p50 of 0.057 m, so most of its lanes register and a share (5-10 %) sit about 1.3 m off. Its
other numbers:
- 2520 of 26943 sample points have no lane running the same way within 8 m;
- heading p99 28.5 degrees;
- the worst lanelets are 10-17 m off (132088, 116638, 102222, 116659, 100838 on Driving lanes).

The Bidirectional (p95 1.345 m) and Shoulder (1.407 m) lanes show the same band.

## Cause

Diagnosed 2026-11-20 (swb 243). The bad points are not scattered: they are whole roads translated
sideways, in the source pack's conversion, not the projector and not a lane width.

- **Whole roads, both directions together.** Each point is matched to its OpenDRIVE road, section
  and lane (as `lanelet_opendrive_registration.py` matches, plus the ids). On the roads that are
  off, every lane is off by the same amount. The opposite lanes run the other way, so their offset
  has the other sign in the lane frame and the same direction in the world. The spacing between
  the two directions is right: road 19's lanes -1 and 1 sit 3.48 m apart (OpenDRIVE 3.5), and
  road 1's lanes -1 and 2 sit 6.95 m apart (7.0). Lane widths agree within 0.02 m.
- **Mostly along y, toward y = 0.** The displacement is almost all in CARLA's y. It is about
  +1.3 m on the roads near y = -200, -1.3 m near y = +200, and about 0.9 m near |y| = 137. Roads
  elsewhere at the same |y| register, so it is not a scale (a fit of a y scale leaves a p50 residual
  of 0.33 m).
- **It builds up through curves and stays.** Along road 67 (lane -1), the offset is 0.00 m on the
  straight north-south part (s 0 to 60). It grows through the road's two arcs (curvature -0.0176,
  then -0.0094; s 58 to 173), and is a constant 1.32 m on the 137 m straight after them. The
  OpenDRIVE reference line itself is continuous: each geometry's computed end meets the next one's
  start within 1 mm.
- **It is in the source pack, not our repairs**, as the table above already shows: it is there at
  both scales. CARLA 0.9.15 and 0.9.16 ship the same Town03.xodr (md5 d9b14876), so it is not a
  change between those releases. How the pack's converter drew these roads cannot be told from
  the outputs; an older Town03 or its curve handling are the candidates.
- **33.8 % of the Driving points are over 0.5 m**, not the 5 to 10 % the p50 and p95 suggested:
  4890 of 14478, in 82 roads. 1848 of them are on junction connectors.

## Fix (proposed, not applied)

Regenerating Town03's Lanelet2 map from CARLA 0.9.16's OpenDRIVE with acb's `carla_map_gen`
removes the band. That is crdesigner 0.8.5 plus the 14 repairs, run by default, at acb 6c0325f; the
same tool gave 087's v2 Town01, 0.007 m p95. The candidate is on host 243 in
`/var/tmp/aeon/i068-town03/`, with its provenance. The same script against the same OpenDRIVE:

| Town03, Lanelet2 against OpenDRIVE | Driving p50 | Driving p95 | Driving p99 | Bidirectional p95 | Shoulder p95 | points with no same-way lane |
|---|---|---|---|---|---|---|
| v2026.11b (the pack, repaired) | 0.057 m | 1.365 m | 1.390 m | 1.345 m | 1.407 m | 2520 of 26943 |
| regenerated | 0.000 m | 0.013 m | 0.055 m | 0.007 m | 0.002 m | 4 of 23957 |

It keeps what the pack has: 73 crosswalks (the pack 72), 94 traffic-light regulatory elements (38)
with 156 light ways, 94 stop lines (45), and 255 walkway lanelets (191). Lanelet ids change. The
Town03 scenarios (s5, c2b, `town03_ego_drive`) place entities by WorldPosition only, so no
scenario names an id.

Before it replaces the pack's Town03:
1. `autoware_lanelet2_map_validator` (241, 2026-11-20; `-p transverse_mercator --lat 0 --lon 0`):

   | check | v2026.11b Town03 | candidate |
   |---|---|---|
   | crosswalk.missing_regulatory_elements | 72 | 73 |
   | stop_line.missing_regulatory_elements | 7 | 0 |
   | traffic_light.missing_referrers | 38 | 0 |
   | total errors | 117 | 73 |

   The one check left is known and dataset-wide: Town03, Town05 and Town10 have crosswalk lanelets
   (72, 66, 16) and no town has a crosswalk regulatory element (Town01 and Town02 have no
   crosswalks). Autoware 1.5's crosswalk module finds crosswalks through the routing graph and
   stops 3.5 m before one without an element, so driving does not need them. The repair is
   [071](071-crosswalks-have-no-regulatory-elements.md), for the next dataset revision. The
   validator reads a file by its extension: a `.osm.orig` loads as an empty map and reports
   clean, so validate a `.osm` copy.
2. A drive check with the candidate as a local map (`{path: /var/tmp/aeon/i068-town03, as:
   Town03}` in a copy of the projects): `town03_ego_drive`, then s5 and c2b against their
   references, on 0.9.16.
3. Then a dataset revision with Town03's new lanelet2 map. Its point cloud stays, as it is CARLA's
   own geometry. Publishing it is the user's call.

Not blocking the dataset's revision with the scale factor (swb 133's ruling).
