# The RoboRacer example (roadmap 091)

A 1/10-scale RoboRacer (the F1TENTH class) as an own car: the worked example of roadmap 091,
which a user's indoor twin starts from. Today it holds the car; the field (W13 to W17) and the
stack (W18 to W20) join it as their work items land.

- `vehicles/roboracer.yaml`: the car's spec (sim_vehicle_spec, version 2). Its numbers are the
  class's published values (f1tenth_gym's vehicle parameters), not a measured car. The planar
  lidar comes with W19.
- `tools/smallcar.py`: open-loop tests of the car on Isaac at a chosen physics rate, no ROS:
  rest, launch, cruise at 3 m/s, braking from 3 m/s, full lock at 1.5 and 4 m/s.

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
