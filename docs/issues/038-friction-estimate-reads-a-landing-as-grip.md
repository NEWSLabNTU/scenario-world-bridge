# 038: The brake module's friction estimate reads a landing as grip

**Severity**: medium (the estimate feeds the ESC's yaw-rate cap and the slip targets; a
wrong one lasts about a second). **Status**: partly fixed: sim_vehicle_ecu `15827bf` (steps 1
and 2 below; rough-road detection open). **Found**: 2026-11-07, in the 066
road-conditions tour recording (`play_log/rock_world/tour50-edges-edges/out/record/
2026-11-06_14-47-26-tour-50kmh`, `state.jsonl`), Isaac Sim 6.1, sim_vehicle_ecu as pinned
by swb_isaac_world `f364bc8`.

## What happens

On the tour's rocky section (class E, 3 % rock cover, 50 km/h) the car jumps: at x 571-576
its body rises 0.26 m and all four wheels are off the ground for about 0.3 s. The rear
wheels, still driven, spin up (estimated slip 0.5-0.7) and traction control cuts the drive,
as a real car's would. Then the car lands, with a longitudinal acceleration of -17 m/s^2
(frames 949 and 959). The friction estimate (`ecu.mu` in the record) goes:

| frame | x, m | event | mu estimated |
|---|---|---|---|
| 944 | 568 | on the ground, driving | 0.58 |
| 949 | 572 | landing impact, -15.8 m/s^2 | 1.0 |
| 958 | 577 | second landing | **4.68** |
| 959-961 | 578 | -17.2 m/s^2 | 2.99, 2.99, 1.96 |
| 967 | 582 | after the rear left spun again | **0.27** |

The ground is the class's design friction, 1.0, throughout. The other sections (class B, D,
potholes, washboard, water bars) keep the estimate between 0.25 and 1.0, the low values again
just after a driven wheel spun on a bump.

## Why

`sim_vehicle_ecu.estimator.Estimator._friction`: under traction control alone the car is
taken to be at its driven axle's limit, and the friction reached is the longitudinal
acceleration over the driven axle's share of the load:

```python
reached = abs(sensors.accel_x) / G / self._driven_load(sensors.accel_x)
```

`abs()` takes a deceleration as drive. A landing's -17 m/s^2 is the body meeting the ground
through its suspension, not the tires pushing on the road, and over the rear's share of the
load (about half) it reads as a friction of 3.5, plus the margin. Nothing bounds the estimate
above, so it stays past any real road's friction until the time constant brings it back.

The low side (0.27) is the same rule in the other direction: a driven wheel spinning in the
air puts traction control to work while the car hardly accelerates, so the friction "reached"
is near zero and the estimate falls toward its floor. On a smooth road a spinning wheel does
mean low friction; on a rough one it often means a wheel off the ground.

## The real fact

A real brake module reads the same sensors (wheel speeds, the yaw-rate and acceleration
sensor) and meets the same problem. Production ones handle it with:
- a longitudinal acceleration counted as drive only when it is forward (traction control's
  limit is the drive's);
- the estimate held between the lowest and highest friction a road can have (about 0.05 for
  ice, about 1.2 for dry asphalt with a performance tire);
- a rough-road detection: wheel-speed oscillation and the vertical shake raise the slip
  thresholds and freeze the friction estimate, so a wheel flaring over a bump is not read as
  ice.

## Fix

In sim_vehicle_ecu:
1. Done (`15827bf`): under traction control alone only the drive's forward push counts; a
   deceleration holds the estimate.
2. Done (`15827bf`): the estimate bounded above, `est_mu_max` (default 1.2).
3. Open: rough-road detection (the wheel speeds' oscillation over a short window) freezing the
   estimate: a separate step, measured on the tour's six sections and on 055's suite.

Check: the tour's rocky section keeps the estimate within 0.8-1.2; 055's manoeuvre suite
unchanged within its spread.

The Isaac world server imports sim_vehicle_ecu through aib
(`swb_isaac_world/deps/autoware_isaac_bridge/deps/sim_vehicle_ecu`), so the fix reaches Isaac
runs only when aib's pin and then swb_isaac_world's pin of aib move.
