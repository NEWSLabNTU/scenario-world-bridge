# 026: the Isaac ego spawned at r02's lanelet centre comes to rest 0.8 m south of it

**Severity**: low (localization initializes from GNSS or the given pose and follows the
car; it starts a drive 0.8 m off the lane centre). **Status**: open, not explained.
**Found**: 2026-10-30, roadmap 059 N8 on newslab-server133 (aib's runner).

## What is wrong

Spawned at r02's start, on the lanelet's centreline at yaw pi/2 (facing north), the car
settles with its base_link 0.8 m south of the spawn point, i.e. 0.8 m back along its
heading, not sideways.

## Candidates, none checked

- aib places the rig's body origin, not base_link (the rear axle), at the spawn pose; the
  tesla_model3 wheelbase is 2.875 m, so not a whole body length, but an offset between the
  rig's prim origin and base_link would show as a constant along-heading error.
- The car rolled back on a grade before the sticky tyre held it (lateral damping was 20 then;
  the longitudinal was already 200).

## Open

Spawn on flat ground (manoeuvre pad) at a known pose and compare base_link with the request;
then at r02 with sim_vehicle_spec 3773783.
