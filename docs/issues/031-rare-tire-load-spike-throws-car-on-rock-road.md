# 031: A kicked rock frozen in mid-air: a tire stood on it at 252 kN and the car was thrown

**Severity**: high for roadmap 066 R4: a car thrown off the road ends a run, and a scenario
cannot tell it from a planner fault. **Status**: fixed in swb_isaac_world fac4e1a
(`rock_layer`). The spike records the rocks under a tire whenever its load passes 30 kN
(`ROCK_DIAG=1`), which is how it was found.
**Found**: 2026-11-02 on newslab-server-241, `swb_isaac_world/spikes/rock_world.py`
(swb_isaac_world 55c4b3b, aib 8fe6dd9 with its skid plate), Isaac Sim 6.1, host load near 290
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

It looked rare: one run in seven threw the car, and the other six had no tire over 30 kN:
- the same code with the spike's own plate, at 60 km/h;
- 200 m at 60 km/h with aib's plate;
- 800 m at 60, 40, 80 and 60 km/h with aib's plate.

The runs are not fully deterministic: the tile worker and the prim stream depend on timing,
and the prim pool hands out prims in a different order each run. But the failure came back on
a quiet host with the same speeds (40, 60, 80 km/h): stopped at 61.56 m after 113 frames, the
same to the centimetre. It needs the 40 km/h run before it.

## What it is not, so far

- **The skid plate.** It cannot load a tire through the tire contact. The plate change only
  made the two runs diverge earlier.
- **A climb.** The tire contact's load follows the height the tire stands on. Climbing over a
  rock raises it over several steps. A jump of 50x the static load in one step means the
  surface under the tire jumped by tens of centimetres in one step.

## Why

With `ROCK_DIAG=1` the same run printed the rock under the tire:

```
DIAG x=50.57 tire 1 load 251728 at (53.39, -0.80) road -0.065
DIAG   rock 1 4.9 kg r 0.12 pos [53.989, -0.816, 0.73] bottom +0.715 top +0.884 seen True awake False
```

A 4.9 kg loose rock was asleep, a static collider, 0.72 m over the road. The 40 km/h run had
kicked it into the air. The rock stream wakes a rock only while it is in a car's path (from
3 m behind the car to 0.6 s ahead) and put it back to sleep as soon as it left. It slept
where PhysX had it at that moment, which was still in flight. A static collider does not
fall, so it hung there.

The tire contact asked whether a rock rests on the road (`seen`) only for awake rocks, so it
took this one as ground. On the next pass the front tire met a surface 0.88 m up: 252 kN in
one step.

## Fix

- A rock leaving the path sleeps only once it has come to rest: PhysX has it within 1 cm of
  the previous frame's pose (0.2 m/s at 20 Hz). It stays awake while it flies or rolls, and
  then sleeps where it lies (`RockStream._settled`, with a unit test).
- The tire contact checks asleep rocks for resting on the road too, so a rock frozen off the
  road for any other reason holds no tire up either.

The failing run (40, 60, 80 km/h, 800 m) now drives clean, with no tire over 30 kN. One more
rock is awake at the median (7 against 6 at 40 km/h).

A rock 32 m behind the car leaves the window and is written back where PhysX has it, at rest
or not. A rock kicked by a tire lands within a second, and the car needs about two seconds to
put 32 m between them at 60 km/h, so that is left as it is.
