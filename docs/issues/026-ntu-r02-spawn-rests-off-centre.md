# 026: the Isaac ego spawned at r02's lanelet centre comes to rest 0.8 m south of it

**Severity**: low (localization initializes from GNSS or the given pose and follows the
car; it starts a drive 0.8 m off the lane centre). **Status**: closed, not reproduced
(2026-11-02, aib f298a93, sim_vehicle_spec 3773783).
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

## Retried (2026-11-02, newslab-server133)

The ego alone (aib's runner, no Autoware), its rest pose from the GNSS fix (exact in aib) less
the antenna's mount, 40 s after the spawn:

| spawn | rest offset along / across the heading |
|---|---|
| flat ground plane, (0, 0), yaw 0 | 0.000 / 0.000 m |
| flat ground plane, (0, 0), yaw pi/2 | 0.000 / 0.000 m |
| r02's drive spawn on the NTU twin, world (-282.58, -45.02), yaw 0.0035 | +0.003 / -0.123 m |

aib puts base_link exactly at the requested pose (the rig's origin is base_link, its wheels
placed so that it rests on the ground), so the first candidate is out; the 0.8 m back along the
heading came with lateral damping 20, before sim_vehicle_spec 3773783 made it 200 like the
longitudinal, which is the roll-back the second candidate describes. The 0.12 m across is the
car settling on the road's cross slope. The probe: a NavSatFix listener converting through the
MGRS projector, as `scripts/ntu_twin/ndt_track.py` does.
