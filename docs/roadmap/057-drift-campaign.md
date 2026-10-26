# Phase 057: Drifting on Town04's Highway

A fast rear-wheel-drive car on Town04's closed figure-eight highway, in Isaac Sim through
swb: Autoware drives the laps at 100 km/h, and at each turn's waypoint a drift controller
takes the car into a drift, holds it through the bend, recovers and hands back. The
question is whether the drift repeats, turn after turn, lap after lap.

**Status**: planned, 2026-10-27.

**Source**: user direction 2026-10-27: "see if we can make a drift on a very speedy car ...
a CARLA town with a long and closed road with turns and without traffic signals ...
accelerate a car to 100 km/h speed and run the drift sequence when it arrives to a
waypoint. See if it can repeat drifts on each turn along the closed high way", with an
Autoware project like AutoSDV and 2026-golf-cart, public at `jerry73204/autoware-drift`.
Decisions (user, same day): regime A below; Autoware drives between drifts; the Town04
Isaac asset already exists (on the NAS, `~/nas/autoveh/dataset/`; csb 241 physics, who
authors the physics engine, knows it).

**Depends on**: [051](051-target-shape.md) (one car, one spec), [054](054-sliding-on-soft-ground.md)
(each surface's own tire curve), [055](055-onboard-computers.md) (`sim_vehicle_ecu`: ABS, TC
and ESC, each off per spec).

## The track

Town04's highway, four lanes each way, no signals on it (parsed from CARLA 0.9.16's
`Town04.xodr`): 16 roads of 2.48 km plus 8 junction connectors, about 2.9 km a lap on the
reference line. Its arcs: radius 147, 160, 180 and 199 m, and two tight ones, 74 m (116 m
long) and 54 m (33 m long). A Lanelet2 map and point cloud of Town04 were built by hand for
csb-seps on 2026-10-14 ([031](031-maps-for-scenarios.md), "Built by hand once").

## The physics, and the regime chosen

Published autonomous drifting runs at 8-15 m/s on 13-22 m radii (Goh/Gerdes' MARTY, TUM's
AMG GT, Velenis' expert data). A steady drift with both axles saturated needs a lateral
acceleration of about mu g: at 100 km/h on dry asphalt that is a radius of about 79 m, and
Town04's wide arcs take 100 km/h on grip (0.4-0.5 g). So:

**Regime A (chosen): enter at 100 km/h, drift at the bend's speed.** At each turn-in
waypoint: a trail-brake or lift pulse (~0.3 s, ~-0.2 g) to load the front, then turn in and
power over; the controller takes over once sideslip passes ~10 deg, holds the drift at the
speed the radius allows (about 75 km/h on the 54 and 74 m bends), ramps the sideslip target
to zero over ~1 s, and gives the car back to Autoware, which accelerates to 100 km/h for the
next turn. Not taken: B (holding 100 km/h sideways on the wide arcs: 400+ kW at heavy
wheelspin, beyond any published work) and C (wet drift zones).

## The pieces

1. **Track in swb.** Town04 in the map pack: the Lanelet2 map and point cloud, the signal
   table (the highway has none; the town has 43), the repairs, the Isaac world from the
   existing asset, a route along the highway loop.
2. **The car**, `drift_mustang` in `sim_vehicle_spec` on `vehicle.ford.mustang`: ~1600 kg,
   50:50, 300-450 kW RWD, a locked rear axle (PhysX has no limited-slip differential: in
   `sim_vehicle_ecu`'s powertrain, per-wheel drive torque), 58 deg lock and a fast rack, rear
   grip ~10 % under the front with a fall-off past the peak (054's per-surface curve), ABS,
   TC and ESC off, physics at 500 Hz or more substeps.
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

1. [ ] **Docs.** This plan (2026-10-27).
2. [ ] **Track.** Town04 into the map pack and the Isaac world; Autoware drives a lap at
   100 km/h on the Model 3 (no drift): localization holds, the route closes.
3. [ ] **Car.** The drift spec and the ECU's locked axle; `vehicle_check` and step
   responses at speed: grip limit, power-over at turn-in on a flat pad.
4. [ ] **Project.** `autoware-drift`: launcher, car packages, `drift_controller` tested
   offline against a bicycle model with a saturating rear axle.
5. [ ] **One drift.** On a flat pad in Isaac, then on the 74 m bend: initiate, hold, exit.
6. [ ] **Laps.** Every turn, several laps: the repeat rate and what ends a drift that fails.

## Risks

- Nothing published drifts at this speed; the rear axle's power and the road's grip decide
  whether a drift holds or dies on the wider arcs.
- PhysX's tire caps the combined force with no lateral fall-off past the peak, and its
  differentials are open or fixed-ratio: the drift may stay on rails or spin abruptly.
- Substepping at high slip; rollover with large steering at speed (Cai et al. in CARLA).
- Four lanes are ~14 m wide: a 20 deg drift at speed needs the margin.
- Rendering: SimReady towns lose this host's GPU (issue 005); the car's lidar renders.

## Sources

Goh, Goel & Gerdes 2019 (JDSMC); Voser, Hindiyeh & Gerdes 2010; Werner et al. (TUM) 2026
(arXiv 2606.27914); Velenis et al. 2011 (CEP) and the trail-braking paper (SAE 2008); Cai et
al. 2020 (arXiv 2001.01377, drifting in CARLA); PhysX 5.4 vehicle docs.
