# Phase 057: Drifting on Town05's Highway

A fast rear-wheel-drive car on Town05's closed ring highway, in Isaac Sim through
swb: Autoware drives the laps at 100 km/h, and at each turn's waypoint a drift controller
takes the car into a drift, holds it through the bend, recovers and hands back. The
question is whether the drift repeats, turn after turn, lap after lap.

**Status**: in progress, 2026-10-27: the car is in every repository (pins 6225c1a) and drifts on Isaac's proving ground; Town05 and Autoware to come.

**Source**: user direction 2026-10-27: "see if we can make a drift on a very speedy car ...
a CARLA town with a long and closed road with turns and without traffic signals ...
accelerate a car to 100 km/h speed and run the drift sequence when it arrives to a
waypoint. See if it can repeat drifts on each turn along the closed high way", with an
Autoware project like AutoSDV and 2026-golf-cart, public at `jerry73204/autoware-drift`.
Decisions (user, same day): regime A below; Autoware drives between drifts; the track is
Town05's ring (the NAS's Isaac set, `2026-10-23_carla-0.9.16-isaac-usd`, has Town01, 02, 03,
05 and 10HD and no Town04; Town05 is also in the map pack).

**Depends on**: [051](051-target-shape.md) (one car, one spec), [054](054-sliding-on-soft-ground.md)
(each surface's own tire curve), [055](055-onboard-computers.md) (`sim_vehicle_ecu`: ABS, TC
and ESC, each off per spec).

## The track

Town05's ring highway (parsed from the set's `main.xodr`): roads 37 (815 m), 38 (300 m), 12,
34 (276 m), 35 and 36 close into a loop through junctions 2328 and 2014, which are on-ramp
merges: about 1.5 km a lap, three lanes each way, no signal on any of it. Its bends are 97 to
101 m in radius: 100 km/h takes 0.79 g there, at the edge of dry grip, so every bend is a
drift from 100 km/h. Not taken: Town04's figure-eight (2.9 km, arcs of 147-199 m plus 54 and
74 m bends), which has no Isaac export.

## The physics, and the regime chosen

On asphalt with no surface map the Isaac world server runs its reference class: mu 1.12 (the
rig's measured tire peak), a sliding share of 0.8 past the peak over combined slip, a 0.5 m
relaxation length (054 T1, `surface_classes.yaml`); csb 241 physics, 2026-10-27.

Published autonomous drifting runs at 8-15 m/s on 13-22 m radii (Goh/Gerdes' MARTY, TUM's
AMG GT, Velenis' expert data). A steady drift with both axles saturated needs a lateral
acceleration of about mu g: at 100 km/h on dry asphalt that is a radius of about 79 m, and
Town04's wide arcs take 100 km/h on grip (0.4-0.5 g); Town05's 100 m bends do not (0.79 g). So:

**Regime A (chosen): enter at 100 km/h, drift at the bend's speed.** At each turn-in
waypoint: a trail-brake or lift pulse (~0.3 s, ~-0.2 g) to load the front, then turn in and
power over; the controller takes over once sideslip passes ~10 deg, holds the drift at the
speed the radius allows (about 110 km/h on Town05's 100 m bends at mu 1, less as the drift
scrubs), ramps the sideslip target
to zero over ~1 s, and gives the car back to Autoware, which accelerates to 100 km/h for the
next turn. Not taken: B (holding 100 km/h sideways on the wide arcs: 400+ kW at heavy
wheelspin, beyond any published work) and C (wet drift zones).

## The pieces

1. **Track in swb.** Town05 is in the map pack and the Isaac set already; what is new is a
   route around the ring and the drift waypoints, one per bend.
2. **The car**, `drift_mustang` in `sim_vehicle_spec` on `vehicle.ford.mustang`: ~1600 kg,
   50:50, 300-450 kW RWD, a locked rear axle (`powertrain.differential`, which csb 241 is
   adding to `sim_vehicle_ecu`: open, locked, clutch LSD; open by default), 58 deg lock
   (`max_steer_angle` 1.01 rad, the bicycle angle) and no rack limit, rear grip ~10 % under
   the front, ABS, TC and ESC off (`brakes.abs.enabled`, `powertrain.traction_control.enabled`,
   no `brakes.esc`).
   **Rate:** physics runs at 60 Hz (8 PhysX substeps), and the ego takes Autoware's command
   once per world frame (SSv2's 50 ms step): a 100 Hz controller is sampled at 20 Hz. csb 241
   is making the physics step a world-server setting; the runs use a 10 ms SSv2 step and
   1/240 s physics.
3. **`jerry73204/autoware-drift`**, an Autoware 1.5.0 project in the golf cart's shape: the
   main launch, the vehicle and sensor kit packages (generated from the spec, as
   `autoware_sim_vehicle` does), and `drift_controller`: a state machine (cruise, initiate,
   hold, exit) that takes the control command near each drift waypoint and returns it.
   Hold: steering on sideslip and yaw-rate error about r* = V/R, throttle a PI on rear
   wheel speed (slip target 0.2-0.4), 100 Hz (the Goh/Gerdes structure; their model
   inversion if path error matters).
4. **The scenario**: laps of the highway through swb on Isaac, drift waypoints at each turn,
   and a per-drift result: peak and held sideslip, time held, path error, recovered or spun.

## Steps

1. [x] **Docs.** This plan (2026-10-27).
2. [ ] **Track.** A route around Town05's ring; Autoware drives a lap at 100 km/h on the
   Model 3 (no drift): localization holds, the route closes.
3. [x] **Car.** `drift_mustang` (sim_vehicle_spec e2a1d46, reviewed by csb 241), the ECU's
   locked axle (sim_vehicle_ecu 142c9a6, csb 241), its car packages and pedal maps (measured
   on CARLA 2010, physics_hash 933597f5), its aib rig; every component in lockstep (6225c1a).
   Still to do: `vehicle_check`, and the tire curves on the Mustang (`sweep_lat`/`sweep_long`, csb 241).
4. [ ] **Project.** `autoware-drift`: launcher, car packages, `drift_controller` tested
   offline against a bicycle model with a saturating rear axle (done: b78527d; the launch to
   come).
5. [ ] **One drift.** On a flat pad in Isaac (`spikes/manoeuvres.py`, a power-over case on
   its constant-radius circle), then on one of Town05's bends: initiate, hold, exit.
   **Pad, 2026-10-27 (newslab-118-server; the proving ground renders nothing, so issue 005
   does not apply; 60 Hz physics, 50 ms frames):** the manoeuvre spike's `drift` case, 100 km/h
   into a 100 m left bend. First: -18.3 deg held 4.85 s, but the speed ran from 100 to 126
   km/h and the line out to 128 m (power-on drift gains speed). With the drive taken back
   past the hold's entry speed (autoware-drift b78527d): -15.2 deg held 6 s on 97.7 m at 109
   km/h, 113 deg of heading, rear slip 0.17/0.15, then exit and hand back at 98 km/h. To do:
   240 Hz physics and 10 ms frames (csb 241), the sideslip back to its 20 deg target, a
   Town05 bend.
6. [ ] **Laps.** Every turn, several laps: the repeat rate and what ends a drift that fails.

## Risks

- Nothing published drifts at this speed; the rear axle's power and the road's grip decide
  whether a drift holds or dies on the wider arcs.
- PhysX's tire caps the combined force with no lateral fall-off past the peak, and its
  differentials are open or fixed-ratio: the drift may stay on rails or spin abruptly.
- Substepping at high slip; rollover with large steering at speed (Cai et al. in CARLA).
- Three lanes are ~10.5 m wide: a 20 deg drift at speed needs the margin.
- Rendering: SimReady towns lose newslab-118-server's GPU (issue 005, the driver); the car's
  lidar renders. The Isaac runs go to a host that renders (server-241, driver 610.57.04).

## Sources

Goh, Goel & Gerdes 2019 (JDSMC); Voser, Hindiyeh & Gerdes 2010; Werner et al. (TUM) 2026
(arXiv 2606.27914); Velenis et al. 2011 (CEP) and the trail-braking paper (SAE 2008); Cai et
al. 2020 (arXiv 2001.01377, drifting in CARLA); PhysX 5.4 vehicle docs.
