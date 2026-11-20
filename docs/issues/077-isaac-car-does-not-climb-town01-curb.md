# 077: The Isaac car does not climb Town01's curb: isaac_verge never reaches the sidewalk

**Severity**: medium (056 step 5's scenario fails on Isaac; a car on Isaac cannot leave the
road over a curb). **Status**: open; cause confirmed 2026-11-20 (118); the fix is roadmap 086's (below).
**Found**: 2026-11-20, swb 241, `isaac_verge` (main 7d5bb9d) on Isaac Town01, 2 of 2 runs.

## What happens

`town01_verge.xosc` has scensim's `simple` pilot drive Town01's straight at y -130 westward,
turn right at x 184 and stop on the walkway lanelet beside the road, (175.5, -125.2), 0.1 m
above the road. On CARLA (118) it passes: 26 m, 19 s, no contact. On Isaac both runs fail on
the scenario's 90 s limit:

- the right wheels reach the curb line at y -127.4 and stay there; the world server's surface
  log flips them between `asphalt` and `gravel` (the walkway's class in the scenario) many
  times a second at the same point: the wheels sit on the curb's edge;
- the car slides west along the curb at 0.05 to 0.07 m/s and never reaches the walkway.

## Cause (confirmed 2026-11-20, 118)

PhysX's raycast wheel query cannot take Town01's curb. Town01 has no road detail, so the road
layer's tyre contact (066 R5) is not there, and the curb meets PhysX's query alone.

- **The curb is a sharp step.** Town01's collision mesh at x 182-190 has a vertical face at
  y -127.419 up to 0.053 m, rounded to the 0.103 m top within 13 mm. Its lidar point cloud
  shows a soft edge, but that is the beam's footprint, not the collider.
- **The raycast does not climb it.** In a headless test of aib's vehicle against that profile
  (flat ground, straight at the curb, a held speed), the raycast fails in most cases:
  - 10 of 15 at 15 to 38 degrees off the curb line at 1 to 3 m/s;
  - 8 of 16 at 45 to 90 degrees at 1 to 4 m/s.

  In a failure, the near front wheel's ray lands on the face or its round-over, and PhysX
  reports that face as the ground plane (normal z 0 to 0.25). The tyre's force on it pushes
  the wheel back and sideways. The car then yaws parallel to the curb and creeps along it or
  stops. That is what isaac_verge shows: the front-right wheel's hit stays on y -127.4 from
  x 184.1 for 52 s while the car moves west.
- **The sweep query climbs every one of those cases.** But issue 029 found that it throws the
  car off 10 cm cleats at 10 to 20 m/s, so it was not chosen (066 R0).
- **The query type cannot change while the car runs.** PhysX reads it only when it creates
  the vehicle; set 1 s into a run, the run stays the raycast's.
- **No workaround in the scenario.** This road has no curb cut, and no approach angle is
  dependable: only 60 degrees climbed at every speed tried, while 45 and 75 failed at some.

## Fix (roadmap 086)

1. **The sweep, rechecked** (066 R1's check, pulled forward from C6; 2-3 days):
   - Rerun 029's fixtures (`spikes/road_fixtures.py`, `ROAD_QUERY=sweep`) at 10 and 20 m/s,
     with PhysX's rigid travel set beyond the rig's, as R2 sets it on the road layer's
     corridors.
   - Rerun aib's slope matrix and 055's manoeuvre suite.

   If the sweep no longer throws the car, aib switches to it, which takes curbs on every map.
2. **Otherwise, the road layer owns the curb** (C5's lip, its re-entry fixtures at 5, 10 and
   15 cm, plus this one).
   - Town01 needs a road record whose sections carry the curb as a lip.
   - PhysX's query stops seeing the sidewalk collider, so the tyre contact gives the step and
     the walkway's height.

Until then `isaac_verge` fails on Isaac and the release notes keep 077 as a known issue.

## Evidence

NAS logs `2026 SSv2 on CARLA/2026-11-20 077 curb climb, raycast and sweep (118)/` (the test,
per-step traces, Town01's mesh section).


NAS logs `2026 SSv2 on CARLA/2026-11-20 086 C4 isaac_verge on Isaac Town01 (241)/`.
