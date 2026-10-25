# 007: The car's onboard control lives in its vehicle bridge, and its ESC reads the ground

**Severity**: medium. **Status**: fixed by roadmap
[055](../roadmap/055-onboard-computers.md). **Found**: 2026-10-25, review of roadmap 054
step 3 (user: "Aib is the bridge. I don't think it's a proper place to do control adjustment.
... The esc seems to live in on board computer in practice").

## What is wrong

On Isaac, everything between Autoware's command and PhysX's wheel torques is in aib
(autoware_isaac_bridge), the Isaac vehicle bridge:

| aib module | what it does | where it lives on a real car |
|---|---|---|
| `aib.powertrain` (`ev`) | acceleration request to motor torque, regen blended with the friction brakes, brake distribution (EBD), ABS, traction control | the drive unit's controller (VCU) and the brake module |
| `aib.esc` | stability control: one wheel's brake against a yaw the tires cannot hold | the brake module (ABS, TC and ESC are one unit) |
| `aib.control.steer` | the steering actuator's lag and rate limit | the power steering (EPS) |
| `aib.powertrain` lags | motor torque lag, brake dead time and pressure build-up | the actuators themselves |

A vehicle bridge's job is the one a drive-by-wire interface has on a real car: Autoware's
command in, status and sensors out. The car's onboard computers sit below it, and Autoware
never sees them. Having them in aib has three costs:

1. **The layering is not the car's.** The vehicle's own behaviour (the same Model 3 on any
   simulator that takes wheel torques) is mixed into the code that speaks ROS to Autoware, and
   its parameters are mixed into the PhysX rig (`VehicleParams` holds the tire stiffness and
   the ESC gain side by side).
2. **The controllers read the simulator's ground truth.** A real brake module sees its own
   sensors: four wheel speeds, a yaw rate and acceleration sensor, the steering angle. aib's
   read what PhysX knows:
   - `Esc.step` takes the ground's friction under each wheel
     (`Vehicle.friction_values`, the surface layer's value) and the body's true lateral speed
     (`state.velocity[1]`). A real ESC estimates both from what it measures, which is the hard
     part of an ESC, and the part that decides how it behaves on mud.
   - `EvPowertrain.torques` takes the body's true speed and PhysX's longitudinal slip per
     wheel. A real ABS has no slip sensor: it computes slip from the wheel speeds against a
     reference speed it estimates, and that estimate is what a locked or spinning wheel
     corrupts.
3. **It cannot be shared.** Another simulator, or a test bench without Isaac, would have to
   import a ROS bridge to get the car's brakes.

## How it was confirmed

Reading the code at aib `1d49569`: `aib/isaac/vehicle.py` `Vehicle.apply` passes
`state.velocity[0]`, `state.angular_velocity[2]`, `state.velocity[1]`, the mean of
`self.friction_values` and `self.wheel_states` (PhysX's `TIRE_LONGITUDINAL_SLIP`) to `Esc.step`
and `powertrain.torques`. The roadmap 054 step 3 results with ESC on (the sine with dwell
passing, the mud lane change held) were therefore obtained by an ESC that knew the mud's
friction exactly.

## Fix

Roadmap 055: the onboard computers move to a repository of their own, `sim_vehicle_ecu`, pure
Python with no simulator and no ROS. Its input is the by-wire request and the car's own sensor
readings, its output the wheel torques and the tire angle. It estimates the reference speed,
the wheel slips, the sideslip and the friction itself. aib keeps the vehicle interface, the
sensors and the PhysX vehicle; the Isaac host steps the onboard computers every physics
substep, next to PhysX.
