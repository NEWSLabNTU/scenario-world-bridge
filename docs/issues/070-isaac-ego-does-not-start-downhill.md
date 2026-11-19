# 070: The Isaac ego does not start from rest on a downhill under Autoware

**Severity**: high (the how-to's Haruna course cannot be driven on Isaac). **Status**: open (cause found).
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

## Cause (confirmed 2026-11-20, a PhysX test on 118)

PhysX's sticky-tyre state holds the car, not its brakes. The Isaac car's tyres enter that
state below 0.2 m/s for 1 s (`sticky_tire_threshold_speed`, `..._time`; damping 200, raised
in csb roadmap 059 N8 so a parked car does not creep across a camber), and PhysX leaves it
only when a wheel gets drive torque. Autoware's -0.028 m/s^2 is a brake request of about
4.7 N m per wheel, below the onboard computers' `brake_floor` (50 N m), so it is released to
0, and the drive stays exactly 0: nothing ever ends the sticky state, and its damping holds
the car against the 5.4 % grade. With a request of 0 m/s^2, the road-load feedforward gives
a small positive drive as soon as the car creeps, and it rolls.

The test (aib's vehicle and onboard computers as `aib.steptest` builds them, gravity tilted
3.1 deg), 20 s per case:

| Case | Result |
|---|---|
| Autoware's request (-0.028 m/s^2, 0.25 m/s, D) | held: 0.015 m |
| the same, `brake_floor` 0 (4.7 N m kept) | held: 0.015 m |
| the same, sticky damping 20 | held: 0.18 m |
| the same, sticky threshold speed 0 (sticky off) | rolls: 9.7 m/s (open loop) |
| 0 m/s^2, 0.25 m/s | rolls |

## Proposed fix (waiting for a decision)

In gear, a target velocity over 0.01 m/s, no drive torque and every brake below
`brake_floor`: a release torque of 1 N m on each driven wheel (about 0.003 m/s^2), so PhysX
leaves its sticky state when the brakes are released, as a real car is free to roll. Tested
as a patch in the test script: the downhill start rolls (2.32 m/s at 5 s); on the flat, the
same request creeps 0.016 m/s, which Autoware's controller corrects; a target of 0 (a stop,
a parked car) still holds on the hold brake, so the camber creep 059 N8 fixed stays fixed;
uphill is unchanged. 5 N m creeps 0.079 m/s on the flat. Proposed in aib's `Vehicle.apply`
(sticky tyres are PhysX's), as a rig field.

## Evidence

NAS `logs/2026 SSv2 on CARLA/2026-11-20 070 downhill start PhysX test (118)/`: the test
script, every case and the frame traces. NAS `logs/2026 SSv2 on CARLA/2026-11-20 0.2.1
Isaac examples live (118)/`: the runs, the
topic readings, and the stock-Autoware run before it, which also stood still and requested
609 brief emergency stops (stock's known stale-diagnostics behaviour; none with the profile).
