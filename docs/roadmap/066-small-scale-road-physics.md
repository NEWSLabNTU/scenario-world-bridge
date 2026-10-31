# Phase 066: Small-Scale Road Physics -- Rocks, Roughness and the Corner

The Isaac ego should feel the road as a real car does: a rock lifts the wheel and can throw
the car, a pothole drops the wheel and hits it on the far edge, washboard makes the wheels hop
and lose grip, a crest unloads the car and the landing loads it, and a boulder can hit the
underbody. 047 and 054 gave the ground its friction, soil and sideways forces; this phase gives
it shape below the terrain data's resolution, and gives the car the parts of a real corner that
respond to that shape. It comes before the rally stage (planned as 067), which needs it.

**Status**: in progress, 2026-10-31. R0 done: PhysX queries the ground once per 60 Hz step,
a raycast barely feels a 10 cm step, a sweep is thrown off it by the rigid travel limit, and a
rock under a wheel is not pushed ([issue 029](../issues/029-physx-wheel-contact-on-steps.md)).
The plan now takes the vertical tire contact over from PhysX ("Where R0 leaves the plan").
R5's first slice works on the fixtures: our contact model at 60 Hz matches the 4 kHz
reference's peaks within 4 %, the same in all five start offsets, with no throw, at
0.83 ms per PhysX step for one car.

**Source**: user, 2026-10-31: "Let's see if we can build realistic off road or rally env to
evaluate autopilot"; "I'd to have realistic road physics"; "I would finish missing parts before
building the rally road. Let's see how to simulate small scale objects and physics like rocks
... The vehicle dynamics should respect the road configuration. For example, a vehicle hitting
a rock may jump"; then, on the open questions: "I prefer the answer that mirrors real world
fact. Also, we need to trade off the computation resources to achieve live simulation."

**Survey**: `reports/Rocks and road roughness for rally.md` (notes in
`research_notes/Rocks and road roughness for rally/`), building on the two earlier reports
(`Localized deformable terrain for rally.md`, `Real time tire soil lateral dynamics.md`).

**Depends on**: [047](047-off-road-surfaces.md) (surface and soil layers),
[052](052-isaac-physics-completeness.md) (frame-time measurements),
[054](054-sliding-on-soft-ground.md) (T1/T2 tire forces), [055](055-onboard-computers.md)
(ABS, TC, ESC on the car's own sensors). **Feeds**: the rally stage (067),
[040](040-ntu-offroad-field.md), [041](041-competition-director.md).

## Where it starts (2026-10-31)

Checked in the Isaac 6.1 install (omni.usd.schema.physx 110.3.2) and aib's
`src/aib/aib/isaac/vehicle.py`:

| | today |
|---|---|
| wheel-ground contact | one raycast down each wheel's centre (`suspensionLineQueryType = raycast`), one plane per wheel; no wheel collision shape against the ground |
| suspension | PhysX linear spring and damper per corner, sprung-mass model; 1.6 Hz, damping ratio 0.45, 0.2 m travel, about 0.10 m of it taken by static sag (spec, all "assumed") |
| wheel | 20 kg, spin only: no vertical wheel mass, no tire vertical stiffness, so no wheel hop |
| limits | the travel limit is rigid; no bump stop; one damper rate both ways; no anti-roll bar (Isaac 6.1 exposes none) |
| chassis | a box collider whose bottom is at wheel radius + 0.05 m, about 0.42 m: nothing on a road ever touches it |
| rates | 3 PhysX steps of 1/60 s per 50 ms frame; the vehicle integrates 8 substeps per PhysX step |
| available, unused | `sweep` query; `limitSuspensionExpansionVelocity`; per-wheel state (`SUSPENSION_JOUNCE`, `SUSPENSION_FORCE`, `GROUND_PLANE`, `GROUND_ACTOR`, `GROUND_HIT_POSITION`); aib's per-step `PhysxForceAPI` force |

So a rock already launches the car, for the wrong reason: the ray finds a 10 cm rock about
0.25 m late, the jounce jumps in one step and the damper answers with a spike of 36 kN or more
(the survey's estimate from aib's tuning), and above about 10 cm the rigid limit takes the rest.

## The rule: what a real car does, within the live budget

Every choice below is answered by the real-world fact first. Where the fact costs more than a
live run can carry, the cheaper model must be shown equivalent where it is used (beyond the
reach of any vehicle, or below what the measurement can tell apart), or it is written down as
an approximation with its error measured.

**The budget.** A frame is 50 ms of simulated time and must take at most 50 ms. One ego on
soil takes 29.2 ms p50 / 35.1 ms p95 today (052 step 5, 241's RTX 5090); two egos 45.5 / 56.0.
This phase may add at most **5 ms p50 per ego per frame** (one ego stays under 35 ms p50 and
45 ms p95, two at about real time at p50 as today), and nothing for a vehicle on smooth road
with no rock in reach. Fidelity follows the vehicle, as 047's soil follows the zone: the ego
and physics NPCs within reach of rough features get the full corner; kinematic NPCs, which
SSv2 places, get none; far features are static geometry, which no one can tell from dynamic
ones when nothing touches them.

### The five open questions, answered

1. **How often the ground is sampled.** A tire touches the road continuously, and its force on
   an obstacle starts rising when the front of the contact patch meets it, ahead of the hub.
   So: the ground under each wheel is sampled at least once per contact length (about 0.15 m)
   at the vehicle's speed, and the force follows the tire's envelope, not a step. At 60 Hz a car
   at 20 m/s moves 0.33 m between PhysX steps, so R0 first measures whether PhysX queries once
   per PhysX step or per vehicle substep. The cheapest arrangement meeting the spacing wins:
   our own per-substep sampling of the height function (R5, about 0.12 ms for 32 wheels in
   numpy, measured), or a higher PhysX rate, whose cost is the whole scene's and is measured
   in R1 before it is chosen.
2. **Whether a rock under a wheel is pushed.** Yes: momentum is conserved, so a stone under a
   wheel takes the equal and opposite suspension and tire force at the contact, and rolls,
   slides or stays according to its own mass and friction. If R0 confirms PhysX applies
   nothing to a hit dynamic actor (the survey found no statement that it does), we apply that
   force ourselves from `GROUND_ACTOR` and `GROUND_HIT_POSITION`. Only rocks within reach of a
   vehicle are dynamic (an activation radius, about 80 m); beyond it a rock is static, which
   nothing can distinguish because nothing touches it. Their state is written back when they
   leave the radius.
3. **Heightfield or mesh, GPU or CPU.** No real-world counterpart; the fact that matters is
   that there is one road. Collision, the render mesh and the lidar sample the same height
   function and agree within 5 mm. Between representations that meet that, the cheaper one
   (frame time and memory, several vehicles) is chosen by measurement.
4. **Whether the wheels feel a different surface from the rendered one.** No. There is one
   road, and enveloping is the tire's property, not the road's. Collision is the true
   geometry; the tire model does the enveloping (R5's tandem cam, live). The survey's offline
   pre-filtered "effective road" is not used: it is speed-independent and has no impact
   dynamics, and the live cam costs about 0.12 ms.
5. **Ray or sweep.** A tire has width and a round profile, and a rock off the wheel's centre
   line still lifts it. So the wheel query is the tire's shape (`sweep`), and the cam samples
   across the width as well as along it (Schmeitz: 5 cams along, 6 across, for a passenger
   tire). R1 measures the sweep's cost with one, two and four vehicles against the budget.

Two more follow from the same rule:

- **Wheel hop.** A real corner has 40-45 kg of unsprung mass on a tire spring, with a hop
  mode at 10-15 Hz that washboard excites. PhysX's corner has none. A virtual unsprung mass
  per corner (R6) is in the plan, not optional; the vehicle's 480 Hz substep rate resolves
  15 Hz.
- **The car's own numbers.** Ground clearance, travel, bump stop and damper curves follow the
  real Model 3 where published (Tesla gives about 140 mm of ground clearance, to be confirmed
  against the spec), and are marked assumed where not. The chassis collider moves to the real
  underbody height with a skid plate shape.

## Stages

Each stage is measured on R0's fixture course before the next starts, with its frame-time cost
against the budget.

- [x] **R0. Instrument and fixtures** (done 2026-10-31, newslab-server-241; swb_isaac_world
  `spikes/fixture_course.py`, `spikes/road_fixtures.py`, rig from sim_vehicle_spec 3773783).
  Nine lanes: 5, 10 and 15 cm cleats 0.2 m long with vertical edges, an 8 cm pothole 0.5 m long
  under the left wheels, 30 m of washboard (0.7 m pitch, 20 mm peak to peak), a 0.5 m kicker, a
  20 cm boulder and a loose 10 kg rock. Every wheel is logged each PhysX step: on ground, load,
  hit point, plane, jounce. The reference is a quarter car with a 42 kg wheel on a 230 kN/m
  tire spring that lifts off, the tire's geometric envelope and a progressive bump stop, driven
  over the same profile at 4 kHz. Five variants: raycast and sweep at 60 Hz, sweep with
  `limitSuspensionExpansionVelocity`, raycast and sweep at 240 Hz (2 vehicle substeps, so the
  vehicle still integrates at 480 Hz); plus sweep with 0.3 m of travel. Logs and traces in
  `play_log/road_fixtures/`.

  | front-left wheel | reference | raycast 60 | sweep 60 | sweep 240 |
  |---|---|---|---|---|
  | 10 cm cleat, 10 m/s: peak load, body rise | 20.2 kN, 7.8 cm | 4.4 kN, 1.6 cm | 18.4 kN, 27 cm, pitch 23 deg | 26.0 kN, 4.1 cm |
  | 10 cm cleat, 20 m/s, 5 start offsets: missed / thrown | 0 / 0 | 2 / 0 | 0 / 2-3 (to 3.9 m, 90 deg) | 0 / 1 (0.41 m) |
  | 15 cm cleat, 10 m/s | 26.6 kN, 20 cm | 8.4 kN, 3.8 cm | 12.6 kN, 70 cm | 31.5 kN, 61 cm |
  | 20 cm boulder, 10 m/s | 29.9 kN, 34 cm | 5.8 kN, 1.1 m * | 14.1 kN, 35 cm | 30.6 kN, 25 cm |
  | washboard, 20 m/s: load min-max | 1.4-8.2 kN | 0.0-10.5 kN | 0.0-11.6 kN | 0.0-11.5 kN |
  | where the load first rises, from the cleat's face | -0.24 m | +2.8 m | -0.06 m | -0.18 m |

  \* the rear wheels' launch, not the front's.

  - **Question 1:** once per PhysX step. Raycast missed the 0.2 m cleat at 20 m/s in 2 of 5
    offsets at 60 Hz and in none at 240 Hz.
  - **Question 2:** no. The rock was the wheel's reported ground and moved 2 mm.
  - **Question 5:** the sweep cost the same as the raycast, 0.42-0.45 ms per PhysX step with one
    car. It meets a vertical face ahead of the wheel, reports a plane tilted 40-50 deg and a
    jounce at full travel, and the rigid limit lifts the body at 10-11 m/s in one step. More
    travel did not stop it, and the outcome depends on what ran before.
  - **The expansion limit** let wheels unload as the reference does, but diverged on the
    boulder.
  - **240 Hz** is four times the PhysX steps per frame for the whole scene, the soil layer
    included.

  Issue 029.

### Where R0 leaves the plan

The real-world rule cannot be met inside PhysX's suspension query at 60 Hz:
- the raycast misses short obstacles and barely answers the ones it finds;
- the sweep answers violently;
- 240 Hz with the sweep comes close, but it quadruples the physics of the whole scene, still
  throws the car now and then, and has no wheel hop.

So the vertical contact moves to our side, which is how BeamNG, rFactor 2 and Assetto Corsa
get rocks right:
- **PhysX's query sees the smooth base road:** the road before R3's small-scale detail, and no
  rocks. PhysX keeps the tire's friction and slip and the body's integration.
- **The tire model acts on the one height function (R5's tandem cam) at the vehicle substep
  rate,** about 0.12 ms for 32 wheels. The real road is still the only road the car feels,
  whatever PhysX queries, so question 4 stands. Under each wheel it computes:
  - the envelope against the detail and the rocks;
  - the tire's spring and lift-off;
  - the bump stop (R2);
  - later, the wheel's mass (R6).
- **The resulting contact forces go to the body at the contact** and, with the opposite sign,
  to a dynamic rock (question 2). The load each wheel carries reaches PhysX's tire forces
  through its per-wheel friction, as T1 already does.

R1 shrinks to checks: the expansion limit stays off, and the raycast stays. On the base road a
raycast and a sweep are the same and the raycast is cheaper. R5 moves up to follow R0, and R2
and R6 build on it. R5 must show on the same fixtures that the reference's shape comes back at
60 Hz, within the 5 ms budget.
- [ ] **R1. PhysX settings** (1-2 days). `sweep`; `limitSuspensionExpansionVelocity` on; the
  travel and sag reviewed against the real car; chassis collider at real clearance. Measured:
  the cleat force rises ahead of the hub; the pothole unloads the wheel; sweep cost with 1, 2
  and 4 vehicles; a 120/180 Hz PhysX rate's cost, for question 1. Risk: sweep initial-overlap
  and fall-through artefacts; the expansion limit unsettling ABS, TC and ESC (re-run 055's
  manoeuvre suite).
- [x] **R2. The corner's forces** (1-1.5 weeks, a few vector operations per wheel). A
  progressive bump stop, a digressive damper with separate bump and rebound, anti-roll bars,
  applied as forces from the jounce each substep; PhysX's rigid travel set beyond the bump
  stop so it is never reached on the fixtures. Measured: crest landing peak spread over 10-30 ms
  (3-4 g at 0.5 m drop), no rigid-limit hits, the roll gradient. Risk: PhysX's tire load does
  not see our forces; fold them into the per-wheel friction as T1 does.

  **Done in `road_contact`** (2026-11-01, newslab-server-241; swb_isaac_world, Isaac at 60 Hz,
  `ROAD_TIRE=ours ROAD_TRAVEL=0.30`, logs `play_log/road_fixtures/r2_{ours,physx}`). The R5
  corner already had the bump stop, so R2 adds the rest:
  - **The damper**: digressive, with more rebound than bump. At low speed the two average
    the rig's c, at 3 : 1 rebound to bump. Above a 0.13 m/s knee the slope falls to 0.4 of
    the low-speed slope (the valve's blow-off). Tesla publishes no curves; a passenger car's
    rebound is 2-3 times its bump. `rebound_ratio`, `damper_knee` and `damper_high` on
    `Corner` are all assumed.
  - **Anti-roll bars**: one per axle, as a force on each wheel from the jounce difference.
    They give half the suspension's roll stiffness, 60 % of it at the front (assumed: Tesla
    publishes no rates). The car has both bars, since aftermarket kits replace both.
    `anti_roll_bars()` turns these shares into rates: 26.7 kN/m front, 17.8 kN/m rear.
  - **PhysX's travel at 0.30 m.** Its frame is raised 0.1 m, so the droop is the real one and
    its rigid limit lies 0.1 m beyond our bump stop. The corner keeps the real 0.2 m (the
    jounce is counted from full droop either way). `px_limit` reports any inner step where
    PhysX's predicted jounce reaches its limit.

  | | PhysX alone (0.2 m) | ours |
  |---|---|---|
  | kicker, 15 m/s: steps at the rigid limit | 2 | **0** |
  | kicker: body acceleration, 60 Hz | 93 m/s^2 (485 at 240 Hz, where the limit's impulse shows) | 111 m/s^2 |
  | kicker: the four suspension forces' peak, half width | -- | 10.7 g, 8.3 ms |
  | kicker: wheel airborne, body rise | 0.28 s, 0.59 m | 0.48 s, 0.66 m |
  | steady circle, 8-13 m/s: roll gradient | 3.80 deg/g | **2.60 deg/g** |

  | front-left peak load, ours / 4 kHz reference with the same damper | |
  |---|---|
  | 10 cm cleat, 10 m/s | 18.5 / 21.8 kN |
  | 10 cm cleat, 20 m/s, 5 offsets | 51-53 / 60.5 kN |
  | 15 cm cleat, 10 m/s | 64.5 / 61.1 kN |
  | boulder, 10 m/s | 66.4 / 58.8 kN |

  Cost: 0.77 ms p50 (0.82 p95) per PhysX step, bars included.

  - **The rigid limit is gone** from every fixture, the kicker included. The landing is
    carried by the bump stop and the tires.
  - **The landing is harder and shorter than the 3-4 g sizing target.** That target assumed
    0.15 m of stroke. The rig has 0.10 m from its sag to full bump, from the spec's 0.2 m
    travel and 1.6 Hz, both assumed. The car also leaves the 0.5 m kicker 0.16 m higher
    still, so it bottoms:
    - the front and then the rear land in peaks of 7-11 g, each 8 ms wide at half height;
    - the whole landing takes about 50 ms;
    - the same corner as a 4 kHz quarter car peaks at 18 g, so the whole car's peak is not
      the model's artefact.

    A real road car bottoms its bump stops as hard on this jump. So what remains is the car's
    travel, not the model: the spec's travel and frequency need real Model 3 values before
    the rally stage.
  - **The roll gradient** is 2.6 deg/g with the bars and 3.8 without. That is the estimate
    from the rates (2.55 and 3.68, the tires adding 0.7). It sits at the firm end of a
    passenger car's range; no measured Model 3 figure was found.
  - **The digressive damper** lets the wheel follow the road more freely in bump:
    - the 10 cm cleat at 10 m/s lifts the corner 5.1 cm instead of 6.7;
    - on washboard the wheel load's minimum falls from 1.5 kN to about 0.5 kN (offline),
      nearer to the grip loss R6 is about;
    - offline (`contact_offline.py`, PhysX's travel 0.30), every fixture is within 3 % of the
      reference's peak with the same damper. The exception is the 10 cm cleat at 10 m/s,
      18 % low on the rim strike's brief peak.
  - **Braking over the cleat** now gives +1.5 m/s^2; PhysX alone gives +4.2. It is the tire
    rolling down the cleat's far edge, whose slope pushes forward, as a real one does. The
    stop is 10.2 m (PhysX alone 9.3). The flat, washboard and pothole stops are unchanged.
- [ ] **R3. One road, small scale** (1.5-2 weeks). A seeded height function per road tile:
  ISO 8608 2D roughness shaped like measured gravel (classes C-E, the corrugation break near
  2 cycles/m), crown or level section, shoulders, ditches, ruts with a centre windrow,
  potholes weighted to sags, washboard in braking and acceleration zones, water bars by grade.
  Collision tiles at 5 cm (about 32 MB/km for a 20 m corridor), the render mesh and the lidar
  from the same samples (question 3), streamed with the vehicles. Measured: generated PSD and
  golden-car IRI on target; ISO 2631 weighted RMS in plausible bands at 30-60 km/h; frame time
  over 2 km of rough road.
- [ ] **R4. Rocks** (1.5-2 weeks). Static boulders (drivable, chassis-colliding); dynamic
  1-50 kg rocks as convex hulls (64 vertices or fewer, GPU contact), speculative CCD, TGS, in
  an activation radius per vehicle with write-back; the reaction force (question 2); gravel
  stays a surface class. Placement by a size distribution (Golombek-Rapp form) along edges,
  ruts and ditches. Measured: a 15 cm rock at 40 km/h lifts the wheel and pitches the body
  with no damper spike; a kicked stone rolls away; high-centering on a ridge; frame time with
  10^4 static and 10^2-10^3 awake rocks. Risk: GPU buffers overflow silently (watch the
  simulation statistics).
- [ ] **R5. Tire enveloping, live** (1-2 weeks, about 0.12 ms for 32 wheels). The tandem cam,
  5 x 6 per tire, on the height function each substep: effective height, slope and rolling
  radius, rim-strike flag. Feeds the corner (R2, R6) and question 1's sampling. Measured: cleat
  force shape, no braking kick at a step, edge strike tens of kN at 40 km/h. Risk: the cams'
  known weakness on Belgian block and corrugation; R6's washboard test checks it.

  **First slice done** (2026-10-31, newslab-server-241; swb_isaac_world
  `swb_isaac_world/road_contact.py` (0a73ce0), `spikes/road_fixtures.py` with `ROAD_TIRE=ours`,
  `spikes/contact_offline.py`). PhysX's query sees the base road: every lane flat but the
  kicker, which is large-scale and stays PhysX's. Each PhysX step, `road_contact.TireContact`
  takes the body's pose and does the following for each wheel:
  - **The envelope:** 5 x 6 cams over the patch on the true profile, along the wheel's path
    over the coming step. Each cam is a rigid disc of the tire's radius, 1 cm samples. The
    effective height is the cams' mean, the slope the front cams less the rear.
  - **The corner:** a 42 kg wheel on a 230 kN/m tire spring that lifts off, under the rig's
    spring and damper, a progressive bump stop over the last 3 cm and a rebound stop.
    Integrated 8 times per step (480 Hz), with the attachment moving at its velocity from the
    step's start.
  - **The force on the body,** at the wheel's centre, for the next step: our suspension force
    less what PhysX's spring and damper give on the base road. PhysX's jounce is geometric
    (travel + wheel centre z - attachment z), so it is known from the pose and the base road
    alone. In the linear range the difference does not depend on the body's motion at all, so
    the 60 Hz coupling stays stable.
  - **The effective plane's push back** along the heading, at the contact.
  - **The tire's load** as each wheel's friction (load over PhysX's own plus the wheel's
    weight), since PhysX's tire saturates at mu times its own load.

  On flat road the body sits within 1.3 mm of PhysX alone, and the predicted PhysX load is
  within 0.3 % of the one PhysX reports. Isaac at 60 Hz against the 4 kHz quarter car on the
  same effective road (`reference_cam`), front-left:

  | | peak load | wheel airborne | corner rise |
  |---|---|---|---|
  | 5 cm cleat, 10 m/s | 12.7 kN / 12.7 | 0.033 s / 0.038 | 2.4 cm / 2.5 |
  | 10 cm cleat, 10 m/s | 19.5 kN / 19.5 | 0.10 s / 0.12 | 6.6 cm / 7.6 |
  | 15 cm cleat, 10 m/s | 25.8 kN / 25.8 | 0.15 s / 0.29 | 11.5 cm / 19.5 |
  | 10 cm cleat, 20 m/s, 5 offsets | 22.6-23.0 kN / 23.5 | 0.083-0.10 s / 0.088 | 4.9 cm / 5.0, every offset |
  | pothole, 10 m/s | 10.1 kN / 10.3 | 0.017 s / 0.024 | 0.1 cm / 0.3 |
  | washboard, 10 m/s | 7.2 kN / 7.2 (min 1.8 / 1.4) | 0 / 0 | 1.6 cm / 1.6 |
  | washboard, 20 m/s | 7.7 kN / 8.1 (min 2.2 / 1.6) | 0 / 0 | 1.1 cm / 1.6 |
  | boulder, 10 m/s | 25.8 kN / 26.6 | 0.13 s / 0.34 | 9.2 cm / 23.3 |

  - **Peak loads** are within 4 %. Offline, the same model under a stand-in for PhysX's body
    (`contact_offline.py`) is within 3 % on every fixture, wheel airborne time included.
  - **The 15 cm cleat and the boulder** rise less and fly shorter in the whole car than in the
    quarter car. The other three corners and the body's pitch and roll carry part of the
    impulse; the quarter car has none of them. The boulder is also under the left wheels only.
  - **The kicker's landing** is still PhysX's rigid travel limit: 207 m/s^2 at the body (93
    with PhysX alone, whose 60 Hz trace hides the limit's impulse; 485 at 240 Hz). That is R2's.
  - **The 20 m/s cases** are still accelerating at the fixture (17.7 m/s), so their static
    front load is 4.0 kN, not 4.45.
  - **Cost:** 0.83 ms p50 (0.91 p95) per PhysX step, PhysX's own step being 0.53. That is
    2.5 ms per 50 ms frame for one ego, inside the 5 ms budget. The envelope takes 0.2 ms; the
    rest is numpy's per-call overhead in the 8 inner steps, the same for 4 wheels as for 32.
    So more egos should go in one call, and a compiled inner loop is the next lever.

  **Second slice** (2026-10-31; swb_isaac_world 59ce660):
  - **The rim strike.** Past 80 mm of tire deflection (three quarters of a 235/45 R18's
    106 mm section), the rim meets the road through the tire, and the rim's stiffness adds,
    10 times the tire's. Each step reports how far past (`rim_strike`, m). It engages on the
    10 cm cleat at 10 m/s and harder ones; the reference has the same law.
  - **Inner steps vectorised.** Everything not depending on our wheel is computed for all 8
    inner steps at once. Offline, 0.31 ms per step for 4 wheels and 0.86 ms for 32; in Isaac,
    0.73 ms p50 per step with the host's calls, or 2.2 ms per frame for one ego.
  - **Steering:** a front wheel's cams follow its steer angle.
  - **Braking from 3 m before the fixture** at -6 m/s^2 through aib's onboard computers.

  Isaac at 60 Hz, front-left, against the quarter car with the rim (`reference_cam`):

  | | peak load | wheel airborne | corner rise |
  |---|---|---|---|
  | 10 cm cleat, 10 m/s | 27.9 kN / 30.9 | 0.10 s / 0.12 | 6.7 cm / 7.9 |
  | 10 cm cleat, 20 m/s, 5 offsets | 53-56 kN / 63 | 0.10-0.12 s / 0.13 | 5.9-6.1 cm / 8.0 |
  | 15 cm cleat, 10 m/s | 69.8 kN / 67.5 | 0.18 s / 0.36 | 13.4 cm / 26.2 |
  | boulder, 10 m/s | 60.1 kN / 66.3 | 0.15 s / 0.42 | 11.2 cm / 31.5 |

  - **Rim-strike peaks** are brief, and 480 Hz samples them 10-15 % low. Offline the same
    model is within 3-10 % of the reference's peaks and 10-15 % of its rise; 16 inner steps
    gain only 1-5 %, so 8 stay.
  - **The cleat at 20 m/s** reaches tens of kN and strikes the rim, as R5's acceptance asks.
    The R0 survey's figures put such strikes in the same range: 37.7 kN for a Golf hitting a
    block at 64 km/h, and wheel force transducers ranged to 40-50 kN.
  - **The whole car again rises less and flies shorter** on the tall fixtures than the quarter
    car does (the R5 first slice's reason).

  Braking, while faster than 1 m/s, PhysX alone against ours:

  | | stop | deceleration min / max | largest step-to-step change |
  |---|---|---|---|
  | flat, from 15 m/s | 19.9 m / 19.9 | -6.0 / +0.2 and -6.1 / +0.1 | 0.9 / 0.9 m/s^2 |
  | 10 cm cleat, from 10 m/s | 9.3 m / 10.5 | -15.7 / **+4.2** and -15.5 / 0.0 | 16.9 / 8.9 |
  | pothole, from 10 m/s | 9.5 m / 9.5 | -9.2 / 0.0 and -11.8 / 0.0 | 3.8 / 4.9 |
  | washboard, from 15 m/s | 18.8 m / 19.6 | -11.6 / +0.2 and -6.9 / +0.1 | 7.9 / 1.8 |

  - **PhysX alone kicks the car forward** at +4.2 m/s^2 while braking over the cleat, when the
    raycast pops the wheel onto it. Ours has no forward kick: the cleat only pushes back, at
    1.6 g with the brakes.
  - **Over the cleat, ours stops 1.1 m longer.** The front-left wheel is off the road for
    0.067 s, and with no load the brake locks it (slip -0.68) until it lands, as a real wheel
    would.
  - **On washboard, PhysX's raycast** throws the deceleration about (-11.6 m/s^2 peaks). Ours
    stays smooth.
  - **Not reproduced: washboard's loss of grip.** Our load stays above 1.5 kN on this 20 mm
    corrugation. The friction follows the load linearly, so the mean grip is the flat road's,
    and the stop is the same. A real tire's friction falls with load, so a swinging load grips
    less on average. That load sensitivity and deeper corrugation are R6's washboard test.

  Left for R5:
  - **The effective rolling radius.** PhysX's wheel radius is fixed, and the ABS reads PhysX's
    wheel speed, so this needs a channel PhysX does not have. It is deferred until a run shows
    the ECU needs it.
  - **The model in the world server.** It needs a height function for the detail, which is
    R3's road tiles. It goes in with R3, in the surface layer's per-step hook after the soil's
    friction. With no detail, the model would still change the smooth-road corner (wheel mass,
    tire spring), so 055's suite must be rerun when it is switched on.

- [ ] **R6. Wheel hop** (2-3 weeks). A virtual unsprung mass per corner on a tire spring at the
  vehicle substep rate, between R5's effective road and the sprung body; PhysX's spring
  nulled and its tire load taken from ours. Measured: a 10-15 Hz peak in the chassis and wheel
  load spectra; grip loss on washboard rising with speed; 055's suite unchanged on smooth road.
  Risk: the tire load PhysX uses and the ECU's slip estimators.
- [ ] **R7. Sensors and Autoware** (1 week). Motion BVH on for rough-road runs (its render
  cost measured against the budget); the lidar mount following the chassis' vibration;
  `scan_ground_filter` on the fixtures (false obstacles over washboard and ruts, rocks passed
  as ground); NDT on rough tiles. Measured: false-stop rate, rock and pothole detection range,
  NDT iterations.

## Acceptance

- [ ] Every fixture's trace has the real shape: cleat force rising ahead of the hub, the
  pothole's unload then far-edge peak within 0.5 s, the washboard's 10-15 Hz wheel-hop peak,
  the landing spread over 10-30 ms; numbers within ±30 % of the R0 reference.
- [ ] A rock above the bump-stop height throws the car, by contact and corner forces, with no
  rigid-limit impulse; a boulder taller than the clearance hits the skid plate.
- [ ] Rocks under wheels move by the reaction force; rocks out of reach cost what static
  geometry costs.
- [ ] Collision, render and lidar agree on the road within 5 mm.
- [ ] One ego on R3's rough road with rocks: frame p50 at most 5 ms over today's (29.2 ms),
  p95 under 45 ms; two egos keep real time at p50.
- [ ] 055's manoeuvre suite and `town01_ego_drive` on smooth road unchanged within their spread.

## Not in this phase

The rally stage itself (067); dust and multi-echo lidar (broken in Isaac 6.1); tire damage and
punctures; DEM gravel; CARLA (no channel for any of this).
