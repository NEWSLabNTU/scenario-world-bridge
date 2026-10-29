# 023: Isaac applies the tyre forces at the wheel centre, so a car transfers a third of its load

**Severity**: medium (every aib car in Isaac: under braking, acceleration and cornering its wheel
loads move about a third as much as a real car's, so it keeps more grip at the limit, pitches and
rolls less, and an oversteer or a drift starts later than it would). **Status**: open.
**Found**: 2026-10-29, autoware-drift's race_planner model check against Isaac's manoeuvre suite
(`play_log/model_check/dry2/` on 243, drift_mustang with issue 022's fix).

## What is wrong

A tyre's force acts on the car at the contact patch, at the ground, so it makes a moment about
the CoM with the CoM's whole height h: across the car the wheel loads move by m ay h / t in all
(shared between the axles by their roll stiffness), and front to rear by m ax h / L. Whatever
the suspension's geometry, the totals are those. Isaac's drift_mustang (h 0.48 m, t 1.586 m, L
2.886 m, m 1450 kg) moves a third of them, on the lateral and longitudinal sweeps:

| | Isaac | m h / t / 2 per axle, 2 m h / L | ratio |
|---|---|---|---|
| sideways, per axle, N per m/s^2 | 70.1 | 219.4 | 0.32 |
| front to rear, N per m/s^2 | 168.1 | 482.3 | 0.35 |

The ratios are (h - R) / h = (0.48 - 0.32) / 0.48 = 0.33, R the wheel radius.

## Why

aib's `aib/isaac/vehicle.py` (the wheel's `PhysxVehicleSuspensionComplianceAPI`) sets both
`TireForceAppPoint` and `SuspensionForceAppPoint` to (0, 0, r - frame_z) in the suspension frame:
the wheel's centre. PhysX applies the tyre's force to the body there, so its arm about the CoM
is h - R, not h. (Game set-ups often put it high on purpose, to keep cars from rolling over.)

## Effect

race_planner's model with a real car's transfer saturated its rear first on the 40 m ramp: 2.6
deg more sideslip than Isaac at 0.94 g, 0.6 at 0.68 g. With the arm h - R
(`race_planner.car.Car.in_isaac()`) it is within 0.7 deg to 0.94 g and the steering within
0.1 deg.

## Fix (not done)

Put the tyre force's application point at the contact patch (the wheel's bottom, R below its
centre: z = -frame_z in the suspension frame), keeping the suspension force where it is, and check the car stays
stable over the manoeuvre suite (a low application point lets the tyre's force roll the body
more: about 2.7 deg at 1 g on drift_mustang's 2 Hz springs, no anti-roll bars). Every Isaac run's
numbers move with it, the drift law's included.
