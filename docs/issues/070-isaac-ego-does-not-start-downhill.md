# 070: The Isaac ego does not start from rest on a downhill under Autoware

**Severity**: high (the how-to's Haruna course cannot be driven on Isaac). **Status**: open.
**Found**: 2026-11-20, swb 118, the 0.2.1 live check of the Isaac examples
(`drive-a-course-on-isaac.md`, `haruna_down` on `isaac_first_drive`).

## What happens

The ego starts at Haruna's course start (x 134.81, y 197.67, a 3.1 deg downhill, about 5.4 %),
Autoware localizes, the agent engages and Autoware's state is Driving, in autonomous mode,
with no emergency stop (scensim's profile). The car never moves: at simulation time 408 s it
stood 0.07 m from its spawn, and the run ended at the scenario's 600 s limit.

What Autoware asked for, read from the ego's domain while it stood (2026-11-20 06:10):

| Topic | Value |
|---|---|
| `/vehicle/status/velocity_status` | 0.0 m/s |
| `/planning/.../behavior_planning/path` | 16.67 m/s (the lanelets' 60 km/h) |
| `/planning/scenario_planning/trajectory` | 0.25 m/s (the velocity smoother's engage velocity, held while the car stands) |
| `/control/command/control_cmd` | velocity 0.25 m/s, acceleration -0.028 m/s^2, gear drive |
| `/vehicle/status/control_mode` | autonomous |

With slope compensation, Autoware's longitudinal controller expects gravity to start the car
downhill and asks for a slight deceleration. On a real car the brakes it asks for (about
4 N m per wheel) do not hold 5 % of slope (about 80 N m per wheel), and the car rolls off.

## Likely cause

The Isaac car is a PhysX vehicle with its tyres in sticky-tyre mode below 0.2 m/s
(`aib.params`: `sticky_tire_threshold_speed` 0.2, `..._time` 1.0, damping 200), and
`sim_vehicle_ecu.actuators` notes that PhysX keeps a slow wheel in that hold while any brake
torque at all is applied. A negative acceleration request is brake torque in every
powertrain model (never the motor), so the car is held at rest by the tyres, not by the
brakes, and gravity cannot start it. Not confirmed by reading the brake torque the ECU
applied; that is the next measurement.

Not seen on CARLA (its vehicle physics has no sticky-tyre hold), nor on Isaac's flat towns.

## What it needs

The physics owner's call (roadmap 055, sim_vehicle_ecu and aib): a standing car with a
forward target velocity and a brake below what the slope needs should roll as a real one
does, for example by leaving sticky-tyre mode when the gravity force along the slope exceeds
the brake's hold, or by releasing a brake request below the slope's load at standstill.

## Evidence

NAS `logs/2026 SSv2 on CARLA/2026-11-20 0.2.1 Isaac examples live (118)/`: the runs, the
topic readings, and the stock-Autoware run before it, which also stood still and requested
609 brief emergency stops (stock's known stale-diagnostics behaviour; none with the profile).
