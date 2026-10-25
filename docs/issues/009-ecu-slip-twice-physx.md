# 009: the brake module's slip reads up to twice PhysX's at low speed, so ABS and TC act early

**Severity**: medium. **Status**: fixed (`sim_vehicle_ecu` `8742540`). **Found**: 2026-10-25,
roadmap 054 step 4 (the mud circle starving under traction control).

## What is wrong

Since roadmap 055 the brake module (`sim_vehicle_ecu.estimator`) estimates each wheel's slip
itself, as `(omega r - v) / max(|v|, est_slip_floor)` with the floor at 4 m/s, and its
docstring calls that "PhysX's own active slip denominator". It is not. PhysX 5 (the non-legacy
tire, `VhTireFunctions.cpp`, `computeLongitudionalSlip`) adds the minimum denominator to the
speed:

```
lngSlip = (wheelSpeed - longSpeed) / (longSpeedAbs + minActiveLongSlipDenominator);
```

and the passive one alike, both 4 m/s on the rig (aib `params.py`). So the estimate is
`(|v| + 4) / max(|v|, 4)` times PhysX's slip: 2.0 at 4 m/s, 1.9 at 4.5, 1.4 at 10, 1.2 at
20 m/s.

ABS's and traction control's thresholds (release 0.25, reapply 0.15) were tuned in aib against
PhysX's own slip, before 055. Read on the doubled estimate they act at about half the slip they
were tuned for at town speeds. On dry asphalt it hardly showed (the tire peaks at 0.24 of
PhysX's slip, and ordinary driving stays near 0.01). With 054's T1, mud's tire is soft: a
modest drive force needs 0.10-0.13 of PhysX's slip, which the estimate reads as 0.2-0.27, so
traction control cut the motor to 0.15-0.33 on the mud circle and the car slowed from 4.5 to
0.1 m/s against a driver asking for 3 m/s^2.

## How it was confirmed

The manoeuvre suite logs both PhysX's slip and the estimate per frame
(`play_log/manoeuvres/2026-10-25_t1-ecu-off/mud_circle.jsonl`). Driving straight on mud at
4.49 m/s the rear wheels read 0.103 (PhysX) against 0.197 (estimate); at 4.53 m/s before T1,
0.025 against 0.049; on asphalt at 5 m/s, 0.002 against 0.004. The ratio is the same at every
slip, so it is the denominator, not a wheel radius: `(4.49 + 4) / 4.49 = 1.89`.

## Fix

The estimator takes PhysX's definition, `(omega r - v) / (|v| + est_slip_offset)` (the field
renamed from `est_slip_floor`, since it is no longer a floor), so the thresholds mean what they
were tuned to mean. The 054 suite with it (`play_log/manoeuvres/2026-10-25_009-ecu-{off,esc}`):
the mud circle holds 1.50 m/s^2 (ESC off; 0.28 before) and speeds up to 24 km/h, traction
control working the rear wheels at 0.22-0.28 of PhysX's slip (mud's peak); the mud stop takes
15.71 m at 3.65 m/s^2 (17.96 m, 3.15 before); dry and wet unchanged. Table in roadmap 054,
steps 4b-4d.
