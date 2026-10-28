# 022: the "locked" rear differential is nearly open in a steady corner

**Severity**: medium (every Isaac run of a car whose spec says `differential: {type: locked}`,
today drift_mustang: its steady cornering, power oversteer and drift are those of an almost open
axle, so a planner or controller tuned in Isaac learns the wrong car). **Status**: fixed
(sim_vehicle_ecu d68637a; Isaac runs it once aib's `deps/sim_vehicle_ecu` pin, and
swb_isaac_world's of aib, move to it -- measured below with that checkout moved by hand).
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

## Fix

sim_vehicle_ecu d68637a. The coupling is a stiff shaft: the inertia term above is its damping,
and the wheels' speed difference integrated (their angle between them) is its spring, I / (4 dt^2)
N m per rad, held within the axle's grip (an LSD: within its clutch's limit). The spring winds up
to the scrub torque, so in a steady corner the wheels turn at one speed. Why not the tyres'
stiffness in an implicit step, or the tyres' forces: the ECU is called once per physics step
(1/60 s; PhysX's vehicle substeps 8 within it), and the wheel with its tyre settles in a few ms
(I v / (C_kappa R^2)), so within the ECU's step the tyre takes any torque; a proportional term
alone can lock at most half way before it oscillates, and last step's tyre forces are a step
late, which makes them the same integrator at its stability limit. I / (4 dt^2) is critically
damped with the tyres carrying nothing (both sliding) and settles a 40 m corner's scrub in about
0.5 s. Unit test: two wheels on a linear tyre in that corner hold 1.00 of a spool's slip
difference at 6.5-19.5 m/s (0.08-0.10 before).

Measured on 243, the same run as dry1 (`play_log/model_check/dry2/`, the fix in
swb_isaac_world's nested `deps/.../sim_vehicle_ecu`). PhysX's slip divides by v + 4 m/s (issue
009), so a spool's rear slip difference on the 40 m circle is (t/R) v / (v + 4), not t/R: 0.024
at 6.5 m/s, 0.029 at 11, 0.032 at 15.4, 0.033 at 19.5. Isaac now holds that:

| CoM speed (m/s) | rear-left slip | rear-right slip | difference (dry1) | a spool's |
|---|---|---|---|---|
| 6-7 | 0.019-0.020 | -0.005 | 0.024-0.025 (0.004) | 0.024-0.026 |
| 11 | 0.025 | -0.005 | 0.029 (0.007) | 0.029 |
| 15 | 0.033 | 0.001 | 0.032 (0.010) | 0.032 |
| 19 | 0.057 | 0.023 | 0.033 (0.016) | 0.033 |

Steady (frame-to-frame jumps of the difference 0.0001 or less, its spread 0.0005), and the car
now steers as a spooled one: 4.42-4.60 deg of tyre angle on the circle from 6 to 20 m/s (dry1
4.19-4.31; race_planner's spooled model 4.55-4.69). The 5 deg step at 40 km/h: yaw_rate_steady
17.56 deg/s (dry1 18.64; the model's 17.33 spooled, 19.23 open), understeer 0.23 deg/g (0.12).
Every manoeuvre completes as before; power_over still spins (peak yaw rate 190 deg/s).

Open: the drift law's H1 settings (roadmap 058) were found on the nearly open axle. And the
coupling is a 60 Hz loop, not a constraint: in a two-wheel model with a tyre that falls past its
peak, an axle torque between the tyres' sliding and peak capacity (one wheel on the falling
side) makes it hunt where a real spool holds; the dry suite did not show it. PhysX's own
constraint (vehicle2's tank differential) would be exact.
