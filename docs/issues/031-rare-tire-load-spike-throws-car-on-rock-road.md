# 031: On the rock road, a tire's load once jumped to 252 kN in one step and threw the car

**Severity**: high for roadmap 066 R4: a car thrown off the road ends a run, and a scenario
cannot tell it from a planner fault. It is rare. **Status**: open, not reproduced. The spike
now records the rocks under a tire whenever its load passes 30 kN (`ROCK_DIAG=1`).
**Found**: 2026-11-02 on newslab-server-241, `swb_isaac_world/spikes/rock_world.py`
(swb_isaac_world 55c4b3b, aib 645949e with its skid plate), Isaac Sim 6.1, host load near 290
on 24 cores.

## What happened

The world-path rock spike drove 800 m of the forest corridor at 40, 60 and 80 km/h. At
60 km/h, 50.6 m from the start, the front right tire's load from the tire contact
(`road_contact`) went from 4.6 kN to 252 kN in one PhysX step (1/60 s). The car's speed fell
from 16.5 to 11.3 m/s in that step. The body rose about 1.9 m and rolled, and the car stopped
on its side at 61.6 m:

| x (m) | speed (m/s) | front left | front right | rear left | rear right (kN) |
|---|---|---|---|---|---|
| 50.38 | 16.55 | 4.7 | 4.6 | 4.3 | 4.3 |
| 50.57 | 11.26 | 4.9 | **251.7** | 4.0 | 5.9 |
| 50.73 | 9.29 | 3.1 | 141.1 | 7.4 | 3.2 |
| 50.88 | 8.86 | 0.4 | 0 | 46.5 | 0 |

PhysX reported no contact of the body or the skid plate with anything at that moment. The
skid plate had last touched a rock at 44.9 m, at an impulse of 64 N s. So the force came from
our tire contact, not from a PhysX collision.

## How often

In seven runs of the same road, that was the only one. The other six had no tire over
30 kN:
- the same code with the spike's own plate, at 60 km/h;
- 200 m at 60 km/h with aib's plate;
- 800 m at 60, 40, 80 and 60 km/h with aib's plate.

The runs are not deterministic: the tile worker and the prim stream depend on timing, and the
prim pool hands out prims in a different order each run. So the rocks the plate pushes differ
from run to run.

## What it is not, so far

- **The skid plate.** It cannot load a tire through the tire contact. The plate change only
  made the two runs diverge earlier.
- **A climb.** The tire contact's load follows the height the tire stands on. Climbing over a
  rock raises it over several steps. A jump of 50x the static load in one step means the
  surface under the tire jumped by tens of centimetres in one step.

## Suspects

- A rock's pose, as the tire contact reads it, jumping into the tire's footprint. Candidates:
  a prim from the pool (a28cf19) reporting its previous rock's pose for one read after it
  wakes; or a rock just woken overlapping the tire.
- The grid index (55c4b3b) misfiling a rock that moved across a cell. The unit test covers a
  move, not a rock read mid-substep.
- A rock lifted by the plate and still marked as resting on the road (`seen`).

## Next

Run the spike with `ROCK_DIAG=1` until the spike recurs, and read which rock, where, how high
and whether awake. Then fix it and add a guard: a rock top that rises faster than any real
rock can, for example.
