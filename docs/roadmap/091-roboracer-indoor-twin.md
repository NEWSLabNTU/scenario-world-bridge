# 091: A RoboRacer Indoor Twin on Isaac Sim

**Status**: opened 2026-11-20 (user). Design agreed in conversation; step 1, the design note, done 2026-11-20 (approved by 133): [roboracer-indoor-twin.md](../design/roboracer-indoor-twin.md); the reference stack for step 5 is f1tenth_system's `humble-devel` (133); a component sees the car's own sensors and, when it declares them, ground truth the user also measured on the real car (the user). Step 2 in progress (241); work items W1 to W23 below, registered 2026-11-20 (user).

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
   stops, steady at the chosen timestep. Open risks this step must answer (133's study, the
   user's approval, 2026-11-20):
   - PhysX's tyre model against foam and soft RC tyres;
   - jitter of a light car (forums suggest steps of 5-10 ms or smaller);
   - collisions with walls and cones: restitution and friction measured, not assumed.
3b. **Fitting**: a tool that fits the car's spec and the user's component from logged
   manoeuvres. It is scored by the long-horizon trajectory error from matched starts, not by
   one-step error. It starts with on-track identification (arXiv 2411.17508 fits a Pacejka tyre
   from on-track F1TENTH data in under a minute); fitting through a differentiable simulator is
   an option later. Identification runs come first (arXiv 2606.22062: a few identification
   rollouts close most of the gap). Randomisation, where wanted, only around the fit, in a band
   no wider than the real runs' own spread. Exit: the example car fitted from its logs passes
   M1 to M6's bands (validation level b, below).
4. **An authored indoor stage as a map**: a map directory whose world is the user's USD;
   authored attributes kept; Track C overrides applied only where given; the hash in the run's
   results. The stage is layered:
   - a visual layer from a scan (Isaac's NuRec Gaussian splats, or a mesh), with the raw scan
     kept beside it as evidence;
   - a physics layer of colliders authored in Isaac. It is required, since splats have no
     collision;
   - measured attributes (tilt-test friction, object masses) stored on the prims with their
     source;
   - `provenance.stage` hashes every layer.

   Exit: an attribute changed in Isaac's editor shows in the next run's behaviour and its
   recorded hash, with nothing else edited.
5. **The `ackermann_msgs` vehicle interface**: the topics a RoboRacer stack expects, offered by
   the vehicle side. Exit: an unmodified open RoboRacer stack drives the example car.
6. **The guide**: "A RoboRacer indoor twin", the design note's procedure (section 7: the map twin F1-F5, the vehicle twin V1-V4, joining them J1-J3), from the measurements to a run, with the
   fitting harness (measured responses against the simulated component's). It validates at
   three levels:
   - (a) the component alone: logged commands replayed against the logged VESC telemetry;
   - (b) the car: M1 to M6's bands;
   - (c) laps with the user's stack: M7.

   Two rules apply throughout:
   - **Session conditions**: battery voltage, floor state and tyre age, recorded with every run
     and every real log. The component may declare battery voltage as an input; a learned VESC
     model in the style of ANYmal's actuator net (arXiv 1901.08652) needs it.
   - **Friction measured two ways that must agree**: an inclined-plane tilt test of the tyre on
     the floor (ASTM G219 style) and M6's grip limit.

## Work items

The procedure the user follows is the design note's section 7 (map twin F1 to F5, vehicle
twin V1 to V4, joining them J1 to J3). These are the work items that give every procedure
step its tool or interface, and that close the gaps found so far (2026-11-20). Owner "open"
means not yet assigned; 133 assigns.

| # | work item | for | step | owner | state |
|---|---|---|---|---|---|
| W1 | the component interface in `sim_vehicle_ecu`, `Ecu` behind it, `ideal` and `ev` registered, declared truth (`TruthFeed`) | V3 | 2 | 241 | done 2026-11-20 (scensim 1b596d7: sim_vehicle_ecu 7bdcb8e, sim_vehicle_spec f96adf5, aib 7cf7905) |
| W2 | aib's `Vehicle.apply` calling the component; the Isaac exit check (a sample component from an outside workspace, byte-identical to `ideal`) | V3 | 2 | 241 | done 2026-11-20 (scensim 1b596d7: sim_vehicle_ecu 7bdcb8e, sim_vehicle_spec f96adf5, aib 7cf7905) |
| W3 | the testbench side: the pilot's `component:` key, the overlay workspaces on the world server's path, `testbench check`'s import and cost checks, the truth settings, the component's provenance | V3, J2 | 2b | 241 | done 2026-11-20 (scensim 749d5dc, pins 9300a5d: aib db5bf97, scensim_isaac_world 0715624; Isaac run with a workspace component passed) |
| W4 | the `fmu` component: an FMI 2.0 unit wrapped as one shipped component, for compiled models; its dependency decided | V3 | 2 | open | |
| W5 | own cars on Isaac: lift `testbench check`'s refusal; the rig made from a user's spec in the testbench | V1 | 3 | 118 | in progress |
| W6 | a small-car example spec (1/10 RoboRacer class) and its refit of the full-size settings: PhysX slip denominators, `est_slip_offset`, sticky-tyre thresholds, suspension frequency, wheel inertia | V1, V4 | 3 | 118 | |
| W7 | the physics rate and solver for a light car: jitter tests at 100 to 600 Hz, the cost of each, the chosen default | V4 | 3 | 118 | |
| W8 | the tyre-model risk: PhysX's tyre against foam and soft RC tyres, measured on M5 and M6; a decision to keep it, tune it, or take tyre forces from the component | V4 | 3 | 118 | |
| W9 | collisions with walls and cones: restitution and friction read from the authored materials, checked by drop and push tests | F3 | 3, 4 | 118 | |
| W10 | the fitting tool: long-horizon trajectory error from matched starts; on-track tyre identification first; fits the spec's tyre and suspension values and a component's exposed `params`; randomisation bands from the real spread | V3, V4 | 3b | open | |
| W11 | the validation harnesses: (a) component replay against logged VESC telemetry, (b) M1 to M6 against 5.3's bands, (c) lap comparison for M7; one report per twin release | V3, V4, J1 | 3b, 6 | open | |
| W12 | session conditions: a schema (battery voltage, floor state, tyre age) recorded with each run and each real log; battery voltage declarable as a component input | V2, J2, J3 | 2b, 6 | open | |
| W13 | the authored stage as a map (`authored: true`): the session-layer rule, the material and `scensim:surface` readers, Track C overrides only where given | F2, F3 | 4 | 243 | in progress |
| W14 | the layered stage: visual layer (NuRec splats or a mesh), evidence layer (raw scan), required physics layer; `scensim:source` on measured attributes | F2, F3 | 4 | 243 | |
| W15 | `provenance.stage`: the sha256 of every composed layer and resolved asset, the overrides, the records; an unresolved dependency refuses the run | F5, J2 | 4 | 243 | |
| W16 | the map check: units and axes, no PhysX vehicle prim, every layer resolved and hashed | F5 | 4 | 243 | |
| W17 | the minimal lanelet2 loop generator, from the field's centreline or the occupancy map | F4 | 4 | 243 | |
| W18 | the `ackermann` and `vesc` vehicle-interface profiles, mirroring f1tenth_system `humble-devel` (`vesc_driver_node` topics, `/sensors/core` from the component's telemetry, the servo echo); `jazzy-devel`'s names checked | J1 | 5 | 241 | in progress: the names checked 2026-11-20, both branches agree ([design 6](../design/roboracer-indoor-twin.md)) |
| W19 | the RoboRacer sensors: a planar lidar profile (UST-10LX class) checked against real scans at known poses; the IMU; wheel odometry by `vesc_to_odom`'s kinematics | F5, J1 | 5 | open | |
| W20 | the end-to-end exit: an unmodified f1tenth_system drives the example car in an example field | J1 | 5 | open | |
| W21 | the guide, "A RoboRacer indoor twin": the procedure, the measurement protocol (tilt test, drop and push tests, M1 to M7), friction two ways, where Isaac's editor shows `dynamicFriction` and what it means here | all | 6 | open | |
| W22 | real data for the worked example: a RoboRacer and an indoor field to log M1 to M7 and scan F1. A lab team runs RoboRacers, but we have no access for now (the user, 2026-11-20): features first, real data later | V2, F1 | 3b to 6 | the user | deferred |
| W23 | the synthetic stand-in while W22 waits: "real" logs made by a deliberately perturbed copy of the twin (tyre, mass, component parameters, latency, noise); the fitting tool must recover the perturbation, and the harnesses must pass and fail where they should. Kept afterwards as a regression test | V2, V3, V4 | 3b, 6 | open | |

Order: W3, then W5 to W8 (the car on Isaac) beside W13 to W17 (the field), then W10 to W12
(fitting and validation), then W18 to W20 (the stack), then W21. W22 gates the real-data checks
of 3b to 6; until then W23 stands in for them.

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
