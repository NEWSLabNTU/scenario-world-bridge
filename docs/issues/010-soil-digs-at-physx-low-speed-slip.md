# 010: below town speeds the soil reads PhysX's slip as near-total spin and digs the car in

**Severity**: medium. **Status**: fixed (swb_isaac_world, `soil.tire_slip`). **Found**: 2026-10-26, roadmap 054 step 6
(`spikes/low_speed.py`, the low-speed and edge checks).

## What is wrong

On a soil zone the grid (and the closed-form wheel) deepens a driven wheel's sinkage with
Lyasko's slip sinkage, static sinkage times `(1 + |i|) / (1 - 0.5 |i|)` (`soil.slip_factor`),
and takes `i` from `soil.slip_ratio`, `(r omega - v) / max(|r omega|, |v|)`: the wheel's real
slip. Its docstring calls that "PhysX-style". It is not PhysX's slip, which is
`(r omega - v) / (|v| + 4 m/s)` on the rig (issue 009).

PhysX's tire builds its force over its own slip: a driven wheel on mud needs a PhysX slip of
0.1-0.17 for the drive a creeping car asks, whatever the speed. With the 4 m/s in the
denominator that is 0.4-0.7 m/s of wheel overspeed at any low speed. The soil reads that
overspeed as real slip, 0.8-0.9 at 0.1 m/s, 0.46 at 1 m/s, 0.2 at 7.5 m/s, and digs in
accordingly. At town speeds the two slips nearly agree; at a creep the soil sees a wheel
spinning in place where the tire model sees a wheel pulling normally.

So a car on mud below about 3 m/s digs its own driven wheels in, the deeper sinkage raises
their resistance, the drive needs more slip, and the car bogs down.

## How it was confirmed

`spikes/low_speed.py` on manoeuvres' proving ground, mud_wet (`play_log/low_speed/
2026-10-26_mud_wet`), the Model 3 (rear drive), speed held by a P driver:

| case | asked | reached in mud |
|---|---|---|
| into the mud from asphalt at 0.5 / 1 / 2 / 3 m/s | that speed | 0.05 / 0.07 / 1.35 / 2.45 m/s, falling |
| from rest in the mud (ramp 0.25 m/s^2, 1 m/s^2, 3 m/s^2) | 3-4 m/s | 0.04-0.14 m/s after 9-80 s |
| stop in the mud, held 10 s, then 1 m/s^2 | 3 m/s | 0.105 m/s after 65 s |

The same cases on dry asphalt reach their speed. The rear wheels sink from 4.3 to 7.5-8.8 cm
as they start to pull, their resistance rising from about 500 to 850-950 N each; traction
control's limit is not what holds it (it hardly acts at the 1 m/s^2 request).

The grid alone, one wheel at 4300 N rolled for 4 m (no Isaac): free rolling, its sinkage
(2.9 cm) and resistance (390 N) do not depend on speed from 0.1 to 7.5 m/s. At the PhysX slip
the tire needs, they do:

| speed, m/s | PhysX slip | real slip | sinkage | resistance |
|---|---|---|---|---|
| 0.1 | 0.17 | 0.87 | 6.9 cm | 787 N |
| 1.0 | 0.17 | 0.46 | 4.7 cm | 594 N |
| 3.0 | 0.17 | 0.28 | 3.9 cm | 522 N |
| 7.5 | 0.17 | 0.21 | 3.6 cm | 478 N |

With the fix, the same cases with a PI driver (as Autoware's longitudinal controller; the
drive unit is feedforward only), before (`SLIP=real`, `2026-10-26_real-mud_wet`) and after
(`2026-10-26_010b-mud_wet`):

| case | before | after |
|---|---|---|
| into the mud at 0.5 m/s | 0.06-0.27 m/s | 0.33-0.51 m/s |
| a 15 deg turn held at 0.5 m/s | 0.06-0.09 m/s | 0.28-0.51 m/s |
| start after the stop, most speed | 0.11 m/s | 0.89 m/s |
| the rear wheels' rut | 9.0 cm | 6.2 cm |

Mud_wet stays hard to start in (roadmap 054 step 6): on fresh soil a wheel's compaction
resistance nearly equals what a driven wheel's traction can give. On dirt every case runs as
on asphalt.

## Fix

The soil's slip sinkage reads `soil.tire_slip`, the real slip with its denominator held at
PhysX's minimum, `(r omega - v) / max(|r omega|, |v|, 4 m/s)`: `SoilGrid.wheel` and
`wong_reece` take the slip as an argument, and the surface layer passes this one (the floor is
the rig's `min_active_slip_denominator`). Above 4 m/s it is the real slip, as before and as
047 S5 fitted `slip_sinkage` against CRM (imposed real slip, which the calibration spikes and
tests still pass). At a creep it reads the tire's overspeed over 4 m/s, close to PhysX's own
slip, so the soil digs as much as the tire model is slipping. A wheel spinning in place still
reads 1 once its rim passes 4 m/s. The shear (Janosi's displacement) stays on the real
displacement.

Tried first: PhysX's slip itself (`wheel.slip`). It fixed the creep, but at road speeds it
reads below the real slip (by v / (v + 4): 0.7 at 10 m/s), so a braking wheel dug in less and
the mud stop took 16.42 m instead of 15.72 (`play_log/manoeuvres/2026-10-26_010-ecu-off`).

The 4 m/s in PhysX's denominator is itself the reason a creeping tire needs that overspeed
(it keeps PhysX's tire stable at 60 Hz). Lowering it would touch every ground and the brake
module's thresholds, and is not this fix.
