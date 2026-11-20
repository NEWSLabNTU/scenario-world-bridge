# 084: The simple pilot loads no lanelet on a map far from 0 deg longitude (Haruna)

**Severity**: medium. The simple pilot's `route: lanelet`, its default, fails on every map whose
points lie more than 60 deg of longitude from 3 deg E: Haruna and Tsukuba (Japan, 138-140 deg E)
and any real-world map east of about 63 deg E or west of about 57 deg W. CARLA's maps lie near
0 deg and are not affected.
**Status**: fixed 2026-11-20 (118): the full haruna_down with `kind: simple` passed (176.4 s).
**Found**: 2026-11-20, swb 118, roadmap 091 W5's live check. haruna_down (isaac_first_drive)
with `kind: simple`.

## What happens

At haruna_down's start, (134.8, 197.7) heading 162 deg, the simple pilot refuses SSv2's route:

    command set_goal -> FAILED: no route: the vehicle at (134.8, 197.7) heading 162 deg is on
    no lanelet a car may drive in that direction (within 4 m)

SSv2 gives up after 5 `set_route_points` requests (`AutowareError: Requested the service
"/api/routing/set_route_points" 5 times, but was not successful`), and the run ends in error.
Autoware drives the same scenario: it loads the map with the projector of
`map_projector_info.yaml`.

## Cause

`scensim_agent/adapters/simple_route.py`'s `load_map` reads the OSM with
`lanelet2.io.loadRobust(osm, UtmProjector(Origin(0.0, 0.0)))`, then moves every point of the
loaded map to its map-frame position (`map_frame_positions`, from the map's own projector). The
origin (0, 0) puts lanelet2's UTM projector in zone 31, centred on 3 deg E. GeographicLib refuses
a longitude more than 60 deg from a zone's centre, so on Haruna every node fails to parse:

    Error parsing primitive 1: Longitude 138.905d more than 60d from center of UTM zone 31

All 2295 nodes fail (6196 errors), and `loadRobust` returns a map with an empty point layer.
The 32 lanelets keep their bounds, but every bound point is at (0, 0).
- **The pilot's step that moves the points** walks the point layer, so it moves nothing.
- **Every centre line has length 0.** lanelet2's `findNearest` gives every lanelet at 239 m
  from the start (the distance to (0, 0)), more than `NEAR` (4 m).

So no lanelet is near the start, in either heading. Neither the map's lanelet direction (the
lanelets near the start are one-way, and Autoware drives them) nor the pilot's 4 m and 60 deg
search is at fault.

Checked on the Haruna map of haruna-tsukuba-courses fff4cb0b:
- **The pilot's projection is right on its own:** `map_frame_positions` puts node 1 at
  (154.62, 192.65), against its `local_x`, `local_y` of (154.68, 192.72).
- **The same file loads cleanly near its own origin:** `loadRobust` with
  `UtmProjector(Origin(36.494, 138.903))`, the map's projector origin, gives 0 errors and
  2295 points.

## Fix

In `simple_route.load_map`:
- **The parsing origin is near the map.** A transverse Mercator map uses its `map_origin`
  from `map_projector_info.yaml`; CARLA's is (0, 0), so their parse is unchanged. Any other
  map uses its first node (`parse_origin`).
- **A parse error refuses the map.** The refusal names the count and the first error, as do
  lanelet2's own exceptions. Every map in the cache loads with 0 errors at its own origin:
  Town01 to Town10, town05_ring_110kmh, and the coop maps (Town03, Town04, Town05). Haruna
  and Tsukuba had 6196 and 8674 errors at (0, 0).
- **The map is made again from its lanelets after its points move to the map frame.** The
  layers' spatial index, which `findNearest` uses, was built from the parsed positions, and
  moving a point does not update it. At (0, 0) the parsed positions of CARLA's maps nearly
  agreed with the map frame, which is why the stale index went unnoticed there. With a parsing
  origin near the map, Town01's routing test failed until the map was rebuilt.

The tests:
- the synthetic map, placed at Haruna's latitude and longitude, routes; at (0, 0) lanelet2
  refuses it;
- a map with a node lanelet2 cannot parse is refused, naming the count and the first error.

The probe at haruna_down's start now finds lanelet 2000001 under the car, heading 164 deg, and
the pilot picks it. Live on 118, the whole of isaac_first_drive's haruna_down with the
installed Model 3 and `kind: simple` on its lanelet route passed in 176.4 s. The pilot took
SSv2's goal at the first request.

The same parse at (0, 0) was in two coop scripts: `scripts/coop/sites.py` and
`scripts/coop/stop_sign_map.py`. They now use the same rule, so CARLA's maps keep their frame.
Two other loads use projectors without the zone limit and were left as they are:
- `scripts/coop/e2_lane_change.py` (LocalCartesian);
- `scripts/open_maps/route_plan.py` (MGRS).

`czp/zones_tool.py` and `scripts/open_maps/route_matrix.py` already take their origin from the
map.

## Evidence

- NAS logs `2026 SSv2 on CARLA/2026-11-20 091 W5 own car on Isaac (118)/runs/isaac_owncar/
  2026-11-20T19-40-47_haruna_owncar_r0` (the run, the pilot's log).
- NAS logs `2026 SSv2 on CARLA/2026-11-20 issue 084 haruna_down with the simple pilot (118)/`:
  the passing run after the fix, and the probes (the pilot's lookup at the pose, the projection
  and the loaded points, lanelet2's errors at both origins, every cached map's errors).
