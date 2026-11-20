# 091: A RoboRacer Indoor Twin on Isaac Sim

**Status**: opened 2026-11-20 (user). Design agreed in conversation; step 1, the design note, done 2026-11-20 (approved by 133): [roboracer-indoor-twin.md](../design/roboracer-indoor-twin.md); the reference stack for step 5 is f1tenth_system's `humble-devel` (133); a component sees the car's own sensors and, when it declares them, ground truth the user also measured on the real car (the user). Step 2 in progress.

## Goal

A user with a RoboRacer (a 1/10 or 1/5 scale racing car) and a real indoor field builds a
twin of both on Isaac Sim, and runs their RoboRacer stack against it through scensim:

- **the field** looks and behaves like the real one: floor materials, friction, walls, cones
  and other objects;
- **the car** drives like the real one, from a vehicle model the user fits to measurements.

Success: the user's stack drives the twin with the same commands it sends the real car, and
lap times, speeds and stopping distances fall within the spread of the real car's, measured
the same way.

## The two decisions (user, 2026-11-20)

### The vehicle is a black box the user owns

The car's simulated control component (its speed controller, motor and steering servo) is
part of the user's RoboRacer stack, not of scensim. The user models it as a black box from
measurements on the real car (step responses, coast-down, braking, steering rate) and keeps
it in their own workspace. scensim provides the physics and a place to plug the component in.

- **What goes in**: the commands the real car's controller takes, typically speed or duty and a
  steering angle (`ackermann_msgs` on `/drive` in most RoboRacer stacks).
- **What comes out**: what PhysX needs every physics substep: a torque per wheel and the
  steering angle.
- **Where it runs**: inside the physics loop, as `sim_vehicle_ecu` does for the shipped cars
  (aib steps it every substep), not as a ROS node. A node would act a step late and break
  lockstep.
- **How it is found**: by name, from the user's workspace (a Python entry point, or an FMU
  through the powertrain slot `fmu` that `sim_vehicle_ecu` already reserves). scensim's
  repositories never hold a user's model.

A word clash to keep out of the docs: on a RoboRacer "ESC" is the electronic speed controller
(often a VESC). In scensim, `esc` is the brake module's stability control. The user's ESC is,
in scensim's terms, the powertrain.

### Detailed attributes live in the USD stage

Isaac Sim's editor is where users set a floor's material and friction and each object's
properties; that convenience is why they work there. So for an indoor field:

- **the USD stage is the source of truth** for per-object detail. scensim loads it and never
  overwrites what was authored in it;
- **the scenario file** stays about the scenario: entities, routes, start conditions, events.
  It does not set attributes per object;
- **Track C's records** (the surface file and the road record, roadmap 086) remain the source
  for generated outdoor maps and for anything CARLA must share. On an authored indoor stage
  they are optional overrides, and a run records which applied;
- **reproducibility**: every run records the hash of the USD it loaded (and of each layer it
  composed), so a result names the exact stage it ran on.

## What exists and what is missing

| | Today | Missing |
|---|---|---|
| the car's description | a spec (`vehicles/<car>.yaml`) in the user's testbench; generators make the catalog entry and the Isaac rig (`aib.gen --spec`) | own cars on Isaac: `testbench check` refuses them; small-scale cars untested (mass, wheelbase, timestep) |
| the car's control component | `sim_vehicle_ecu`: `ideal` and `ev` powertrains, chosen by name | a plug-in point for the user's model; `fmu` reserved, not built |
| the control stack | pilot kinds `launch` and `command` run the user's own stack | the vehicle interface speaks Autoware's topics; RoboRacer stacks use `ackermann_msgs` |
| the field | a map directory with its Isaac world, as the Haruna and Tsukuba courses have | loading an authored indoor stage as a map; recording its hash; the rule that authored attributes win |
| sensors | the spec's sensors, generated into the rig | RoboRacer's (a 2D lidar, an IMU, a depth camera, wheel odometry) checked against the real ones |

## Steps

1. **Design note** (no code): the plug-in interface (signature, units, the substep contract,
   reset on spawn, how it is named in `testbench.yaml`); USD as source (which attributes scensim
   reads, which it may set, the precedence with Track C's records, the hash); the vehicle
   interface for an `ackermann_msgs` stack; the measurement protocol (which runs on the real
   car, which quantities, the acceptance bands). Reviewed before step 2.
2. **The plug-in point** in `sim_vehicle_ecu` and aib: the user's control component loaded by
   name from their workspace and stepped every substep; `ideal` and `ev` move behind the same
   interface. Exit: a sample component in a test workspace, outside scensim's repositories,
   drives a shipped car identically to `ideal`.
3. **Own small cars on Isaac**: lift `testbench check`'s refusal; a RoboRacer spec as the worked
   example (geometry, mass, tyres from measurements); the physics timestep and solver settings
   a 1/10 car needs. Exit: the example car holds a straight line, turns at full lock and
   stops, steady at the chosen timestep.
4. **An authored indoor stage as a map**: a map directory whose world is the user's USD;
   authored attributes kept; Track C overrides applied only where given; the hash in the run's
   results. Exit: an attribute changed in Isaac's editor shows in the next run's behaviour and
   its recorded hash, with nothing else edited.
5. **The `ackermann_msgs` vehicle interface**: the topics a RoboRacer stack expects, offered by
   the vehicle side. Exit: an unmodified open RoboRacer stack drives the example car.
6. **The guide**: "A RoboRacer indoor twin", from the measurements to a run, with the
   fitting harness (measured responses against the simulated component's).

## Not in scope

- Shipping a RoboRacer control model: the user owns it.
- CARLA for the indoor field: Isaac only.
- Building the user's field: they author it; scensim loads it.

## Open questions

- Which open RoboRacer stack is the reference for step 5? Answered: f1tenth_system,
  `humble-devel` (133, 2026-11-20).
- The physics rate: does a 1/10 car need more than 100 Hz substeps, and what does that cost?
- Does a user component need ground truth? Answered (the user, 2026-11-20): the car's own
  sensors by default; ground truth only as declared, and only what the user also measured on
  the real car, delivered with its latency and noise (design note 2.4).
