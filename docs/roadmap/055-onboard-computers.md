# Phase 055: The Car's Onboard Computers

Move the Isaac ego's onboard control out of its vehicle bridge into a repository of its own,
`sim_vehicle_ecu`, where a real car has it: below the drive-by-wire interface, on the car's
own sensors. The drive unit's controller, the brake module (brake distribution, ABS, traction
control, ESC) and the power steering become one simulator-free library that the Isaac host
steps every physics substep beside PhysX. aib keeps what a vehicle bridge does: Autoware's
command in, status and sensors out, and the PhysX vehicle.

**Status**: done, 2026-10-25 (the repository waits to be published on GitHub; see Progress).

**Source**: user direction 2026-10-25, after 054 step 3: "Aib is the bridge. I don't think
it's a proper place to do control adjustment. The Autoware focus on planning and high level
control. It's not a home to ESC too. The esc seems to live in on board computer in practice.
Find the proper place to run ESC to reflect real world fact", then "I would have a new repo
sim_vehicle_ecu", then (on the diagram) "Good to me. Update or create phase docs and start
implementation until the refactor is complete. Record a proof run video afterwards".

**Fixes**: [issue 007](../issues/007-onboard-control-in-the-bridge.md). **Depends on**:
[051](051-target-shape.md) (the repositories and the spec), [054](054-sliding-on-soft-ground.md)
steps 2 and 3 (the manoeuvre suite, the ESC). **Blocks**: 054 step 4 (T1 changes the tire and
powertrain path this phase moves).

Not in this phase: CARLA (its car takes throttle, brake and steering, not per-wheel torque, so
it keeps its own model and acb's pedal maps); enabling ESC on the shipped Model 3 (still the
user's decision, 054); a rate for the onboard computers other than the physics substep.

## Design

![The ego's control path on CARLA and Isaac Sim](../design/vehicle-ecu.svg)

Source: `docs/design/vehicle-ecu.typ` (`typst compile` it to the SVG). On a real car the
layers are:

| layer | real car | here, on Isaac |
|---|---|---|
| planning, high-level control | Autoware: acceleration and tire angle | Autoware |
| vehicle interface | the drive-by-wire adapter | aib (`EgoRos`): the command becomes the by-wire request |
| onboard computers | drive unit controller (VCU), brake module (EBD, ABS, TC, ESC), power steering (EPS) | **`sim_vehicle_ecu`** |
| actuators | the motor and inverter, the brake hydraulics, the steering motor | `sim_vehicle_ecu` too: PhysX's drive-none vehicle has none, and they are simulator-free |
| the car's body, tires, ground | physics | PhysX, the surface layer |

**The interface** (`sim_vehicle_ecu`):

```python
ecu = Ecu(Calibration.from_spec(spec_doc, overrides))
act = ecu.step(Request(acceleration, velocity, tire_angle, gear),
               Sensors(wheel_speeds, yaw_rate, accel_x, accel_y), dt)
act.drive, act.brake   # per wheel, N m (FL, FR, RL, RR)
act.tire_angle         # the steering actuator's bicycle angle, rad
```

The inputs are the request and what the car's own sensors read: four wheel speeds, the yaw
rate and the two accelerations (the yaw-rate and acceleration sensor of a brake module).
Nothing else: no ground friction, no true velocity, no PhysX slip. What the controllers need
beyond that, the onboard computers estimate, as a real brake module does:

- **Reference speed**: predicted from the longitudinal acceleration, corrected toward the
  wheels: all four when free rolling, the undriven ones under drive, the fastest under
  braking; trusted less while ABS or TC is working a wheel.
- **Wheel slip**: each wheel's speed against the reference, over `max(|v|, slip floor)`, the
  floor at PhysX's own active slip denominator (4 m/s), which the ABS and TC thresholds were
  tuned against.
- **Sideslip**: the kinematic integral (`beta' = a_y / v - r`) with a washout.
- **Friction**: the acceleration the car reaches when it is at its limit (ABS or TC working,
  a sideslip, or turning less than the steering asks), recovering slowly toward the design
  value when it is not.

The ESC's target yaw rate is then capped by the estimated friction, not the ground's.

**Calibration** comes straight from the spec (`powertrain`, `brakes` with `abs` and `esc`,
`powertrain.traction_control`, `steering`, and the geometry and mass the feedforward needs):
`Calibration.from_spec` reads a loaded spec's document, so the repository needs no generator
and no generated file. aib passes the spec its rig names and checks its `spec_hash` against
the rig's; an ego config's `ecu:` mapping overrides calibration fields by name for
experiments, as `vehicle:` does the PhysX fields.

**What stays in aib**: the PhysX vehicle (geometry, inertia, tires, suspension), the
mechanical parts the ECU does not run (the Ackermann linkage, the brake's torque capacity,
the steering's mechanical stop), the road loads on the body (they are the world, not the
car's estimate of it), the sensors and the vehicle interface. `VehicleParams` holds the PhysX
vehicle only.

**Rate**: every physics substep (1/60 s), the rate the controllers had in aib. A real brake
module runs at 100 Hz or more; that needs more PhysX substeps and is not this phase.

**Repository**: `NEWSLabNTU/sim_vehicle_ecu`, pure Python 3 (standard library only, so Isaac's
Python 3.12 and the system's both import it), a submodule of aib under `deps/` and of this
repository under `src/`, pinned and checked like the others (`scripts/check_pins.py`).

## Steps

1. **Docs**: this phase, issue 007, the design note in
   [design/vehicle-side.md](../design/vehicle-side.md) and the diagram.
2. **`sim_vehicle_ecu`**: the repository; the controllers and actuators moved from aib with
   their tests (the same numbers: each controller still takes the speed and slips as
   arguments, so aib's powertrain and ESC tests port unchanged in substance); the estimator;
   `Ecu`; `Calibration.from_spec`.
3. **aib**: `VehicleParams` down to the PhysX vehicle; the rig regenerated; `Vehicle` builds
   the sensor readings and steps the `Ecu`; `EgoConfig.ecu`; `aib.steptest`; the tests;
   `deps/sim_vehicle_ecu` on every path Isaac's Python imports from.
4. **swb_isaac_world**: the aib pin; the spikes (`manoeuvres.py`'s `RIG=` becomes `ECU=`); the
   recorder writes the onboard computers' state.
5. **This repository**: the `src/sim_vehicle_ecu` submodule, the pins, CLAUDE.md's table.
6. **Measure**: the move alone, then the estimator, on the 054 manoeuvre suite; aib's
   `vehicle-check` on Isaac; `town01_mud.xosc` with Autoware on Isaac, recorded, and its video.

## Acceptance

- aib holds no powertrain, ABS, TC, ESC or steering actuator code, and `VehicleParams` no
  field of theirs.
- `sim_vehicle_ecu` imports nothing but the standard library; `Ecu.step` takes a `Request`
  and `Sensors` and nothing else.
- **The move is exact**: with a test estimator that returns PhysX's own speed, slips, sideslip
  and friction (spike only, never shipped), the manoeuvre suite gives 054's numbers.
- **The estimator is measured**: the suite again with it, ESC off and on, every change from
  054's numbers in the table below and explained; the dry sine with dwell still passes FMVSS
  126 with ESC on.
- aib's `just vehicle-check` passes on Isaac.
- `town01_mud.xosc` passes with Autoware on Isaac, and its video is recorded.

## Progress

**Steps 1-5, 2026-10-25.** `sim_vehicle_ecu` holds the VCU (`powertrain`: `ideal` and `ev`,
moved verbatim), the slip modulator ABS and TC share (`brake`), the ESC (`esc`), the actuators
(`actuators`), the estimator (`estimator`) and `Ecu`; 79 unit tests, the standard library
only. aib lost `powertrain.py`, `esc.py` and their tests, `VehicleParams` its ECU fields and
`control.py` its steering actuator; `Rig.load` loads the spec the rig names, checks its
`spec_hash` and builds `Rig.ecu` from it, and `Vehicle` builds the sensor readings and steps
the `Ecu` every substep (55 tests, `just test-isaac` OK). swb_isaac_world's spikes take
`rig.ecu`; `manoeuvres.py` takes `ECU=` (calibration overrides) and `ESTIMATOR=truth`, and the
recorder writes the onboard computers' state (`row["ecu"]`, each wheel's `slip_est`).

**The move is exact.** With `ESTIMATOR=truth` (a spike-only estimator returning PhysX's
speed, slips, sideslip and the ground's friction), all 19 manoeuvre cases give 054's numbers,
ESC off and on: the largest difference over every summary field is 0
(`play_log/manoeuvres/2026-10-25_055-truth-{off,esc}`).

**The estimator, measured** (`2026-10-25_055-ecu-{off,esc}`, against 054's truth runs):

| case | truth, ESC off | estimator, ESC off | truth, ESC on | estimator, ESC on |
|---|---|---|---|---|
| dry sine with dwell, yaw ratio at 1 s | 0.433 | 0.432 | 0.0 | 0.0 (passes FMVSS 126) |
| dry sine with dwell, lateral at 1.07 s | 2.77 m | 2.77 m | 2.69 m | 2.43 m (limit 1.83 m) |
| wet sine with dwell, yaw ratio at 1 s | 0.918 | 0.918 | 0.0 | 0.0 |
| mud straight-line brake, distance | 16.1 m | 16.89 m | 16.1 m | 16.89 m |
| mud straight-line brake, mean deceleration | 3.52 | 3.54 | 3.52 | 3.54 m/s^2 |
| split-mu brake, heading | -7.65 deg | -7.72 deg | -7.66 deg | -7.68 deg |
| mud steer step, steady sideslip | -10.79 deg | -10.27 deg | -2.55 deg | -5.6 deg |
| mud brake in a turn, distance | 15.05 m | 16.35 m | 15.0 m | 16.25 m |
| mud lane change, path error | 3.49 m | 3.39 m | 0.19 m | 0.21 m |
| dry circle, lateral acceleration held | 8.41 | 8.41 | 7.26 | 8.43 m/s^2 |
| wet brake, ABS active share | 0.76 | 0.67 | 0.76 | 0.67 |

The car does what it did where the estimates are easy (free rolling, driving, the dry and wet
sines, the lane change). Three changes, each the estimator's, each one a real brake module has
too:

- **Mud braking is 0.8 m longer.** Under a full ABS stop all four wheels slip, so no wheel
  reads the ground speed; the reference is the deceleration's prediction pulled toward the
  fastest wheel, and it runs about 0.2 m/s under the truth (9.45 against 9.59 m/s, 4.51
  against 4.72). The estimated slips are then larger than PhysX's, ABS releases a little
  sooner and holds a little less (the ABS share on the wet brake drops from 0.76 to 0.67).
  The same bias makes the brake in a turn 1.3 m longer.
- **The ESC holds less of the mud step's sideslip** (-5.6 deg against -2.55 deg). Its
  friction estimate is good -- it drops from 1.0 to 0.20-0.24 within 0.5 s, the ground's being
  0.23 -- but the kinematic sideslip integral lags the truth (-4.0 deg estimated against
  -6.0 deg at t = 20.05 s), so the sideslip term starts later and the ESC acts mostly through
  the motor cut (its brake on the one wheel stays at 3-6 N m). Still a quarter of the
  uncontrolled sideslip's angle less, at half the ESC-on truth's benefit.
- **The dry circle holds the full 8.43 m/s^2 with ESC on.** On PhysX's truth the ESC capped
  the target yaw rate at 0.85 of the ground's friction and cut the circle to 7.26; the
  estimated friction stays near the acceleration reached plus its margin, so the cap is
  higher and the ESC intervenes less. That is what a real one does on a dry road: it does
  not know the road is dry until the car finds the limit.

Circles driven past the limit diverge between runs (they did on the truth too); they are not
in the table.

**aib's vehicle-check passes on Isaac** (`play_log/vehicle_check/2026-10-25_055`): the
contract, then the acceleration and brake steps (+2.0 tracked to 1.998 m/s^2, -4.0 to -3.70)
and the six steering steps (the yaw rate within 1.2 % of the bicycle's). Getting there found
issue 008: once the host's loopback carried multicast, Autoware's `setup.bash` swapped the
recipe's Cyclone profile for its own and the check waited for an ego it could not discover;
fixed in aib's recipe.

**The proof run: `town01_mud.xosc` with Autoware on Isaac, PASS, recorded**
(`play_log/record/2026-10-25_13-10-11-ego/run.mp4`; the world server's `record:=`,
`scripts/record_video.py`, whose overlay now shows the onboard computers' estimates: speed,
friction, the ABS and TC factors and each wheel's estimated slip beside PhysX's). The car
crosses the 12.8 m of wet mud at 13-14 km/h as in 052's run, Autoware asking up to
0.6 m/s^2 against the soil's drag. The estimated speed stays within 0.034 m/s of PhysX's over
the whole run. On the mud the estimated rear slip reads about twice PhysX's (+0.04 against
+0.02): under drive the reference is the undriven front wheels, and the soil's drag slows them
a little below the ground speed, so the rear wheels look to slip more than they do -- the
error a real brake module makes on soft ground, and far under TC's 0.25. Neither ABS nor TC
acts and the friction estimate stays at 1.0: Autoware uses about a fifth of the mud's grip
(052), so the car never finds the limit, and the estimate has no reason to move.

A first attempt passed on asphalt only: the surface map is copied into the map directory for a
run (`scenarios/surfaces/town01_mud.yaml` to `<map>/surfaces.yaml`, 047 S6) and was not.
The recorder showed no ground class on any wheel; with the map in place the next run found it
without a world server restart.

**Publishing.** Every commit above is local until `NEWSLabNTU/sim_vehicle_ecu` exists on
GitHub; then they go up in dependency order (sim_vehicle_ecu, aib, swb_isaac_world, this
repository) and `scripts/check_pins.py` checks the pins.

## Open questions

- Whether the ESC's friction estimate is good enough on the split-mu pad, where the two sides
  differ by four times; a real ESC has the same problem.
- The onboard computers' own rate (100 Hz or more) against the substep.
