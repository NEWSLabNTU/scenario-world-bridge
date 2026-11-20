# The RoboRacer example (roadmap 091)

A 1/10-scale RoboRacer (the F1TENTH class) as an own car: the worked example of roadmap 091,
which a user's indoor twin starts from. Today it holds the car; the field (W13 to W17) and the
stack (W18 to W20) join it as their work items land.

- `vehicles/roboracer.yaml`: the car's spec (sim_vehicle_spec, version 2). Its numbers are the
  class's published values (f1tenth_gym's vehicle parameters), not a measured car. The planar
  lidar comes with W19.
- `tools/smallcar.py`: open-loop tests of the car on Isaac at a chosen physics rate, no ROS:
  rest, launch, cruise at 3 m/s, braking from 3 m/s, full lock at 1.5 and 4 m/s, and in their
  own lanes 10 mm cleats, a push on a wall and a cone hit; the design note's M5 and M6, and
  steering ramps that trace the tyre's lateral curve.
- `tools/tyre_compare.py`, `tools/single_track.py`: PhysX's tyre against identified F1TENTH
  tyres, by a steady-state single-track model (W8).

## The refit of the full-size settings (W6)

The library's defaults are the Model 3's. Each one the small car changes, and why:

| setting | Model 3 | RoboRacer | why |
|---|---|---|---|
| PhysX's slip denominators (active, passive, lateral) | 4, 4, 1 m/s | 0.5, 0.5, 0.25 m/s | PhysX floors each slip's denominator at these speeds; at 4 m/s a car that drives at 0.5 to 8 m/s sees almost no slip |
| the brake module's slip offset (`est_slip_offset`) | 4 m/s | 0.5 m/s | now read from the spec's `min_active_slip_denominator` (sim_vehicle_ecu 4f6cfd5); only the `ev` powertrain and components use the estimate |
| sticky-tyre threshold speed | 0.2 m/s | 0.05 m/s | scaled with the car's speeds |
| substep threshold speed | 5 m/s | 1 m/s | the same |
| suspension frequency | 1.6 Hz | 5 Hz | the static sag is g / (2 pi f)^2: 9.9 mm on 30 mm of travel. At 3 Hz it was 27.6 mm, and the car rested on its bump stop (see below) |
| bump stop engagement | 0.06 m | 0.015 m | at most half the real travel (aib 0a69293); 0.06 m is twice the car's whole travel, and aib refused the car with the sweep |
| wheel inertia | 20 kg wheels | 0.1 kg, damping 1e-4 N m s | the class's wheels; the damping scaled by the wheels' inertia |
| brakes | 3000 N m, hydraulic | 1 N m, 0.02 s, no ABS | a VESC brakes electrically |

## Measured on Isaac (118, 2026-11-20)

`tools/smallcar.py` with the ideal powertrain on a flat ground plane, 0.05 s frames, at 60, 120,
240 and 480 Hz (3, 6, 12 and 24 physics steps a frame). With the 5 Hz suspension every rate
agrees within about 1 %:

| test | 60 Hz | 120 Hz | 240 Hz | 480 Hz |
|---|---|---|---|---|
| 0 to 5 m/s, s | 0.633 | 0.633 | 0.633 | 0.633 |
| stop from 3 m/s at -6 m/s^2, m | 0.670 | 0.708 | 0.725 | 0.734 |
| full lock at 1.5 m/s, radius m (kinematic 0.88) | 0.877 | 0.886 | 0.887 | 0.887 |
| full lock at 4 m/s, radius m | 0.956 | 0.955 | 0.956 | 0.956 |
| full lock at 4 m/s, lateral acceleration m/s^2 | 8.10 | 8.14 | 8.16 | 8.17 |

The stopping distance differs by the speed the braking starts at (2.87 m/s at 60 Hz, 2.98 at
480 Hz), not by the deceleration (6.0 to 6.2 m/s^2 at every rate). At rest no rate drifts. The
sweep wheel query gives the raycast's numbers at 480 Hz.

With the first draft's 3 Hz suspension the rates did not agree: at full lock and 4 m/s the car
understeered at 60 and 120 Hz (radius 0.95 to 0.98 m) and spun at 240 and 480 Hz (radius 0.5 m,
its yaw rate swinging by 1 rad/s), and at launch its wheels slipped 1.6 to 2.0. The car sat on
its suspension's limit, and PhysX resolved that limit differently at each rate. A small car's
suspension must leave it room: check that the static sag is well short of the travel.

The runs are on the NAS under `2026 SSv2 on CARLA/2026-11-20 091 W6 RoboRacer refit (118)`.

## The physics rate (W7): 240 Hz

A testbench with this car sets `physics_hz: 240`, four physics steps per 60 Hz frame. Below
200 Hz the contacts are wrong. Above 240 Hz the results move by a few percent at most, and the
cost keeps growing.

Measured on Isaac (118, 2026-11-20) with `tools/smallcar.py`, 0.05 s frames, TGS (Isaac's
default solver) at 60 to 600 Hz and PGS at 120 and 600 Hz:

| | 60 Hz | 120 Hz | 240 Hz | 300 Hz | 600 Hz |
|---|---|---|---|---|---|
| launch, braking, full lock (W6's table) | agree | agree | agree | agree | agree |
| pushing on a wall: rear wheel slip | 0.16 | 0.33 | 0.015 | 0.021 | 0.021 |
| 10 mm cleats at 3 m/s: lowest speed, m/s | 2.39 | 2.38 | 2.34 | 2.33 | 2.33 |
| the same: peak pitch rate, rad/s | 0.97 | 1.64 | 2.02 | 2.10 | 2.28 |
| the cone, 0.1 kg at 2 m/s: carried, m | 0.48 | 0.73 | 3.91 | 0.57 | 3.91 |
| cruise: acceleration noise (per step), m/s^2 | 0.003 | 0.006 | 0.021 | 0.022 | 0.160 |
| cost: wall-clock s per simulated s, one car | 0.09 | 0.17 | 0.35 | 0.43 | 0.84 |

- **The wall.** The car pushes about 15 N into the wall, and its tyres hold about 38 N, so its
  wheels should not turn. At 120 Hz and below they slip at 0.16 to 0.33 while the car stands
  still. From 200 Hz they hold (slip within 0.035 either way).
- **The cleats.** The speed the cleats take off settles from 240 Hz. The peak pitch rate keeps
  rising to 600 Hz. It is the peak of a signal sampled once a step, so a coarser step misses
  part of it.
- **The cone** is not jitter. Each rate gives the same result on every run (repeated at 240 and
  300 Hz). Whether the cone tips over (0.5 to 0.7 m) or rides on the bumper upright (3.9 m)
  turns on a tipping threshold, and the rate only moves the car across it. The cone's friction
  and the bumper's shape decide it, which is W9's.
- **The acceleration noise.** aib takes the body's acceleration from the second difference of
  PhysX's float32 positions over one step, so the noise grows about as the square of the rate.
  It is 0.02 m/s^2 at 240 Hz, against an IMU's own 0.02 to 0.05, and 0.16 at 600 Hz.
  Differencing over 1/60 s brings 600 Hz down to 0.027. The IMU and the brake module's
  acceleration sensor both read it (reported to 133).
- **TGS and PGS** agree on the driving tests and the cleats to 3 significant figures. Against
  the wall both are wrong at 120 Hz (slip 0.33 with TGS, -0.12 with PGS), and both hold at
  600 Hz, PGS more steadily (acceleration noise 1e-5 m/s^2, against TGS's 0.04). PGS costs 2 %
  more. Isaac's TGS is kept.
- **The cost** is about 1.4 ms per physics step for one car, headless, the ECU and the vehicle
  in Python included: linear in the rate.

## The tyre model (W8): keep PhysX's, with front and rear tyres apart

No real RoboRacer has been measured yet (W22). The tyres compared here are the Pacejka tyres
ForzaETH identified on their F1TENTH cars: NUC6 and NUC7, 3.75 kg like this one, from
On-Track-SysID's `models/<car>/<car>_pacejka.txt`, read as F_y = mu F_z D sin(C atan(B a -
E (B a - atan(B a)))) with mu 0.9514. The repository does not label the order or the
convention. [B, C, D, E] follows the paper's formula and their race stack's labelled files.

**PhysX's tyre on this spec.** From the steering ramps and M6 at 240 Hz, F_y / F_z rises at
4.4 per rad and flattens to about 1.1 by 0.28 rad, the same on both axles.

**The identified tyres** are stiffer, and stiffer at the rear than at the front:

| | front, per rad | rear, per rad | peak, front and rear |
|---|---|---|---|
| this spec in PhysX | 4.4 | 4.4 | 1.1 and 1.1 |
| NUC6 | 7.7 | 17.0 | 0.83 and 0.92 |
| NUC7 | 11.5 | 22.7 | 0.82 and 0.71 |

So those cars understeer, with gradients of 0.016 and 0.014 rad per m/s^2. This car is almost
neutral, at 0.0015. The single-track model (`tools/single_track.py`, no load transfer, as
ForzaETH's) reproduces Isaac's M5 radii within 0.5 to 4 %. `tools/tyre_compare.py` fits
PhysX's curve, its slope and peak scaled, to each reference's M5 and M6:

| fit | NUC6: M5 radius, worst | NUC7: M5 radius, worst |
|---|---|---|
| this spec as it is | 19 % | 13 % |
| one stiffness and friction for all four tyres (what spec v2 holds) | 27 % | 23 % |
| a stiffness and friction per axle | 7 % (rms 3 %) | 5 % (rms 2 %) |

Run in Isaac, the per-axle fits confirm it. aib's per-wheel stiffness and friction were set
after the car was made (`smallcar.py --axle`). Isaac's M5 radii then come within 4.5 % of
NUC6's and 1.8 % of NUC7's, against 18.5 % and 14 % with the spec's tyres. M6 at 0.2 rad comes
within 4 to 5 %. At 0.1 rad Isaac's grip limit is 14 % below the reference's. That compares
Isaac's transient speed ramp, with load transfer, to a steady model without it.

The decision, for the three options roadmap 091 names:
- **Keep PhysX's tyre model.** Its curve's shape fits these tyres' M5 and M6 once each axle has
  its own stiffness and friction.
- **Tune it per axle.** Spec v2's single `tires:` section cannot hold these cars. A front and
  rear section (or an axle override) in the spec, applied by aib's rig, is what the fit needs.
  aib already sets them per wheel at run time (Track C).
- **No tyre forces from the component.** Nothing in M5 and M6 calls for them.

The limits:
- **Past the peak.** These Pacejka tyres fall off steeply past their peak (the rear's C is 2.6
  and 4.3), and PhysX's curve stays flat. A car driven beyond the grip limit (a slide, a drift)
  is where the two will part. M6 only finds the limit.
- **Other cars' tyres.** The references are tyres identified on other cars and floors, read from
  a third party's repository and not checked against its paper, which prints none. Real logs of this car (W22, or W23's stand-in) are
  what the per-axle values are finally fitted to (W10).
