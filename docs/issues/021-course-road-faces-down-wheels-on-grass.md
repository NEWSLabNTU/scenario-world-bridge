# 021: a course's road faces point down, so the wheels drive on the grass under it

**Severity**: high (every run on a generated course drove on grass physics: mu 0.49 instead of
asphalt's 1.12; the hairpin drift attempts understeered into the guardrails). **Status**: fixed
(swb_isaac_world c0fcd45); Haruna's world rebuilt on 243. **Found**: 2026-10-29, roadmap 058 C9a, the first
hairpin drift attempts on Haruna's H1 (`spikes/course_probe.py DRIFT=H1`).

## What is wrong

The recorder's wheels reported `class: grass, mu: 0.491` on 3142 of 3164 wheel samples of a
run that never left the road, and no sample said asphalt. The labels are right (`/Course/Road`
carries Q34442, Road, mapped to asphalt by `surfaces.yaml`), and the terrain is 8 cm below
the road at every road vertex (p5-p95 -0.10 to -0.06 m), never above it.

The road's faces are wound clockwise seen from above: 0 % of the road's 3054 quads and 0 % of
the centre line's have an upward normal, against 100 % of the terrain's. PhysX's scene
queries, which the vehicle's suspension rays are, do not hit a triangle from behind, so each
wheel's ray went through the road and landed on the terrain 8 cm under it: the wheels took
the terrain's label (grass) and the car rode on the terrain, the road ribbon floating
around its tires. The renderer draws both sides, so nothing looked wrong.

The drift attempts on H1 then pushed wide at full lock with the rears spinning (slip ratio
6-7) and scraped the outside guardrail: on grass the front axle saturated first.

## Fix

`swb_isaac_world.course.usd` turns every face of a horizontal ribbon (the road, its lines;
the terrain too) to face up before it writes the mesh, whatever the geometry step's winding,
so a course's ground is hit from above. The geometry step's output is unchanged. Rebuild a
course's USD (`python -m swb_isaac_world.course.usd <course.yaml> <map dir>` in Isaac's venv);
the maps and the point cloud do not depend on the winding.
