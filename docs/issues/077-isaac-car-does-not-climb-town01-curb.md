# 077: The Isaac car does not climb Town01's curb: isaac_verge never reaches the sidewalk

**Severity**: medium (056 step 5's scenario fails on Isaac; a car on Isaac cannot leave the
road over a curb). **Status**: open; diagnosis is roadmap 086's (Track C), after 0.2.1.
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

## Likely cause (not confirmed)

The PhysX vehicle's wheels query the ground with one raycast each per step (roadmap 066 R0,
issue 029: a raycast barely feels a short step). Meeting the curb at the route's shallow
angle, the wheel's ray lands alternately on the road and on the curb top, and the car does not
climb. No contact or suspension data was read yet.

## Next

Read each wheel's contact and suspension (jounce, load, hit point and plane) at the curb, then
either fix the climb in 086 C5/C6 (the road layer's tyre contact, 066 R5, which feels steps
the raycast misses) or move the scenario's exit to a curb cut.

## Evidence

NAS logs `2026 SSv2 on CARLA/2026-11-20 086 C4 isaac_verge on Isaac Town01 (241)/`.
