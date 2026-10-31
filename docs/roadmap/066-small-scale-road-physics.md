# Phase 066: Small-Scale Road Physics -- Rocks, Roughness and the Corner

The Isaac ego should feel the road as a real car does: a rock lifts the wheel and can throw
the car, a pothole drops the wheel and hits it on the far edge, washboard makes the wheels hop
and lose grip, a crest unloads the car and the landing loads it, and a boulder can hit the
underbody. 047 and 054 gave the ground its friction, soil and sideways forces; this phase gives
it shape below the terrain data's resolution, and gives the car the parts of a real corner that
respond to that shape. It comes before the rally stage (planned as 067), which needs it.

**Status**: planned, 2026-10-31.

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

- [ ] **R0. Instrument and fixtures** (2-3 days, no runtime cost). Log per wheel and step:
  jounce, suspension force, ground plane, ground actor, hit position. A fixture course: 5, 10
  and 15 cm cleats, a 0.5 m pothole 8 cm deep, 30 m of washboard (0.5-1 m pitch, 20 mm), a
  0.5 m crest, a 20 cm boulder. A numpy reference: quarter-car and pitch-plane car with the
  spec's values plus a 40-45 kg wheel on a tire spring (236.7 kN/m class), the golden car for
  IRI. Answers: query rate (question 1), reaction to a dynamic actor (question 2). Baseline
  traces of today's car.
- [ ] **R1. PhysX settings** (1-2 days). `sweep`; `limitSuspensionExpansionVelocity` on; the
  travel and sag reviewed against the real car; chassis collider at real clearance. Measured:
  the cleat force rises ahead of the hub; the pothole unloads the wheel; sweep cost with 1, 2
  and 4 vehicles; a 120/180 Hz PhysX rate's cost, for question 1. Risk: sweep initial-overlap
  and fall-through artefacts; the expansion limit unsettling ABS, TC and ESC (re-run 055's
  manoeuvre suite).
- [ ] **R2. The corner's forces** (1-1.5 weeks, a few vector operations per wheel). A
  progressive bump stop, a digressive damper with separate bump and rebound, anti-roll bars,
  applied as forces from the jounce each substep; PhysX's rigid travel set beyond the bump
  stop so it is never reached on the fixtures. Measured: crest landing peak spread over 10-30 ms
  (3-4 g at 0.5 m drop), no rigid-limit hits, the roll gradient. Risk: PhysX's tire load does
  not see our forces; fold them into the per-wheel friction as T1 does.
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
