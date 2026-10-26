# 012: on soil the car grips more than the measured tire, and on a snow layer's base hardly at all

**Severity**: medium (soft mud and loose sand let a car drive that should stall; on fresh
snow a car barely moves or stops). **Status**: fixed (swb_isaac_world: `soil.grip`, used by the surface layer). **Found**: 2026-10-27, roadmap 053 step
3 (mud and snow fitted to measurements, then driven).

## What is wrong

A wheel on a soil zone gets the soil's traction limit over its load as PhysX's friction
(`SurfaceLayer`, `WheelSoil.mu`): the shear the soil would take if every node under the tire
were sheared to its strength, `sum (c + sigma tan phi) dA`. The soil's resistance is applied
separately as a force. Two things follow.

**More grip than the tire measured.** A tire mobilises only part of that limit: the
measurements give its gross traction, the drawbar pull at the peak plus the towed
resistance, and that is below the limit, by most where the soil is cohesive (Janosi's shear
needs displacement, a cohesive soil carries shear without load) and at low speed. On the
fitted mud_soft (Wismer 1966, cone index 25) the measured tire's gross traction is 0.26 of
the load at 20 % slip (pull 0.05, towed resistance 0.21), so a rear-drive car stalls: its
driven wheels pull less than the free ones resist. In the car, the limit gave the rear
wheels a friction of 0.38-0.50 of the load while moving and 0.75-0.92 at a standstill, and
it launched to 2.1 m/s in 3 s (`play_log/soil_drive/2026-10-27_053-mud_soft.log`).

Per class at a Model 3 tire and 4.4 kN (the grid at 1 m/s, steady):

| class | limit / load | gross traction at the class's peak slip (grid) | measured gross (pull + towed) |
|---|---|---|---|
| sand | 0.294 | 0.242 | 0.265 (He 2020 medium) |
| sand_loose | 0.262 | 0.203 | 0.245 (He 2020 low) |
| mud_wet | 0.356 | 0.331 | 0.29 (Wismer 1966, CI 46) |
| mud_soft | 0.380 | 0.327 | 0.26 (Wismer 1966, CI 25) |

**Almost no grip on a snow layer's base.** 053 gave fresh snow a `sinkage_limit` (the layer
compacts no further, the base carries the rest). Bottomed there, the soil carries a small
share of the load, the limit is that share's, and over the whole load the friction fell to
0.02 on the rear wheels: the car launched to 0.34 m/s in 3 s and took 77 m to stop from
9.5 m/s (`play_log/soil_drive/2026-10-27_053-snow_loose.log`).

## Fix

- The soil's friction is held at the class's `mu`, which for a fitted class is the measured
  tire's gross traction (the pull at its peak plus its towed resistance): sand 0.265,
  sand_loose 0.245, mud_wet 0.29, mud_soft 0.26. Below that the limit still rules (a wheel
  on soil that cannot take the shear: shallow sinkage, little load).
- On a layer with a `sinkage_limit`, the load the base carries grips at the class's `mu`
  too: the limit is the soil's plus the base's share times `mu`, then held at `mu`.

Checked on the car (`spikes/soil_drive.py`, `SOIL_SPIKE_ONLY=mud`, each class in the lane;
`play_log/soil_drive/2026-10-27_012-<class>.log`):

| class | coast, m/s^2 | holding 10 m/s asks, m/s^2 | from rest, 3 s full throttle | stop from 15 m/s |
|---|---|---|---|---|
| sand | 0.34 | 0.36 | 1.73 m/s (2.44 before) | 26.7 m |
| sand_loose | 0.90 | 0.96 | **0.11 m/s: stalls** | 24.0 m |
| mud_wet | 0.62 | 0.67 | 1.07 m/s | 23.9 m |
| mud_soft | 1.58 | 2.89, held to 6.6 m/s | **0.07 m/s: stalls** (2.13 before) | 5.9 m from 7.6 |
| snow_loose | 0.27 | 0.27 | 1.87 m/s (0.34 before) | 33.3 m (77 m from 9.5 before) |

The two grounds a rear-drive car should not start on now stop it; the others drive.
