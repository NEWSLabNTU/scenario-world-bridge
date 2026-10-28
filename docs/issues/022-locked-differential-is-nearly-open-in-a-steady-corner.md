# 022: the "locked" rear differential is nearly open in a steady corner

**Severity**: medium (every Isaac run of a car whose spec says `differential: {type: locked}`,
today drift_mustang: its steady cornering, power oversteer and drift are those of an almost open
axle, so a planner or controller tuned in Isaac learns the wrong car). **Status**: open.
**Found**: 2026-10-29, autoware-drift's race_planner model check against Isaac's manoeuvre suite
(`spikes/manoeuvres.py`, `EGO_RIG=drift_mustang.rig.yaml PADS=dry`; the run is
`play_log/model_check/dry1/` on 243).

## What is wrong

A spool holds both rear wheels at one speed, so in a corner of radius R the inner and outer
wheels' slips differ by the track over the radius, t/R: 0.040 on the suite's 40 m circle. Isaac's
drift_mustang shows a tenth of that, at every speed of the ramp:

| CoM speed (m/s) | rear-left slip | rear-right slip | difference | a spool's |
|---|---|---|---|---|
| 6.5 | 0.009 | 0.005 | 0.004 | 0.040 |
| 11.0 | 0.013 | 0.006 | 0.007 | 0.040 |
| 15.4 | 0.023 | 0.013 | 0.010 | 0.040 |
| 19.5 | 0.055 | 0.037 | 0.018 | 0.040 |

And the car steers like an open axle: on the circle Isaac needs 4.20-4.30 deg of tyre angle from
6.5 to 16 m/s; race_planner's two-track model (PhysX's tyre from the spec, swb's asphalt mu 1.12)
needs 4.13-4.17 with an open differential and 4.55-4.69 with a spool. The steady 5 deg step at
40 km/h gives Isaac 18.64 deg/s of yaw rate; the model 19.23 open, 17.33 spooled.

## Why

`sim_vehicle_ecu`'s `EvPowertrain.differential` (powertrain.py) moves, for a locked axle, the
torque that would bring the two wheels to one speed within the step "their inertia only: the
tires act in the physics": I (w_left - w_right) / (2 dt), I = m_wheel R^2 / 2. With 22 kg wheels
at R 0.32 and dt 1/60 s that is 34 N m per rad/s of difference. Holding the spool's difference
against the tyres takes far more: on the 40 m circle at 11 m/s the wheels' speeds differ by
r t / R_wheel = 1.36 rad/s, 46 N m from the formula, while the tyres' scrub (C_kappa = 35.6 kN per
unit slip, slip difference 0.040) asks for about 230 N m. The axle settles where the two balance,
at about a tenth of the spool's slip difference, as measured. (The mechanism is this estimate; the
ECU's own logs of `diff_transfer` would confirm it.)

## Fix (not done)

The coupling has to hold against the tyre's longitudinal stiffness, not only the wheels'
inertia: for example the transfer from an implicit step that includes the tyres' stiffness
(C_kappa R^2 / v per wheel) beside I / dt, or PhysX's own constraint for wheels at one speed
(vehicle2's tank differential, which ties a track's wheels together). Either changes every
locked-axle run's numbers; the drift law's H1 settings (roadmap 058) were found on today's axle.
