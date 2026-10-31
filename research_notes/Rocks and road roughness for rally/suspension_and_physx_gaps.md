# Suspension dynamics for rough roads, impacts and jumps, and what PhysX 5 vehicle2 (Isaac Sim 6.1) lacks

Scope note: 14 web calls plus a read of this repo's own vehicle code. Sources are the PhysX 5 docs (primary), the Omniverse schema reference, NVIDIA forums, Chrono and BeamNG docs, and some community or forum figures, each flagged where used. Tire enveloping and road PSD are out of scope here.

Local context (read from the repo, not the web):
- The spec gives `suspension.frequency: 1.6` Hz, `damping_ratio: 0.45`, `travel: 0.2` m and `wheels.mass: 20.0` kg. The spec's own provenance marks all of these as "assumed" ("a sedan's ride"; "a wheel and tire of this size"). File: `src/sim_vehicle_spec/vehicles/tesla_model3.yaml` lines 61, 116-119, 238, 259.
- aib (`src/autoware_isaac_bridge/src/aib/aib/isaac/vehicle.py`, lines 133-232) sets up the car as follows:
  - Vehicle settings: `PhysxVehicleAPI` with `suspensionLineQueryType = raycast` (not sweep), and `Low/HighForwardSpeedSubStepCount = physx_substeps` (default 8, `params.py:59`) with `subStepThresholdLongitudinalSpeed` 5.0.
  - Per wheel: `PhysxVehicleSuspensionAPI` with `sprungMass`, `springStrength`, `springDamperRate` and `travelDistance`. `PhysxVehicleSuspensionComplianceAPI` has empty toe and camber graphs, and constant force application points at wheel-centre height.
  - Not used: no anti-roll bar API and no bump stop.
  - Chassis collider: a box whose bottom sits at max(box bottom, r + 0.05 m).
  - Gravity: disabled on the rigid body, because the vehicle simulation applies it.

## 1. The PhysX 5 vehicle2 suspension model: what it does at the limits, and what it does not model

### Takeaway
PhysX vehicle2 treats each corner as a massless, linear spring-damper along a ray or sweep. The wheel's vertical position is not a state with its own mass. Each step it is placed geometrically on the ground found by the road query, so **there are no unsprung-mass vertical dynamics, no tire vertical stiffness, and so no wheel hop**. The two travel ends are handled by geometry. At full compression, a PhysX constraint, optionally with restitution, pushes the chassis up. There is no bump-stop spring curve. There is one linear damper coefficient, with no separate bump and rebound damping. Anti-roll bars and toe/camber/force-point compliance graphs exist. True kinematics (wishbone geometry, bushings) do not.

### Cited Findings
**Spring and damper**
- `PxVehicleSuspensionForceParams` has three fields: `stiffness`, `damping` and `sprungMass`. `sprungMass` is "the mass that is supported by the suspension spring". The guide recommends that "the collection of sprung masses matches the mass and center of mass of the rigid body", and `PxVehicleComputeSprungMasses` derives them — [PhysX 5.4.1 Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html); [PxVehicleSuspensionForceParams 5.6.1](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/_api_build/structPxVehicleSuspensionForceParams.html)
- The guide's tuning formulas:
  - `stiffness = naturalFrequency^2 * sprungMass`
  - `damping = dampingRatio * 2 * sqrt(stiffness * sprungMass)`
  - "The springs of a typical family car have natural frequency somewhere between 5 and 10". The guide calls this oscillations per second, but the formula needs rad/s: 5-10 rad/s is 0.8-1.6 Hz, which matches real cars (see the inference below).
  - Suggested starting damping ratio is about 0.8-1.2.
  - Source: [PhysX 5.4.1 Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- There is one `damping` scalar per suspension, so damping is linear and symmetric in bump and rebound. The force params expose no separate compression and rebound coefficients and no velocity-dependent curve. A legacy variant, `PxVehicleSuspensionForceLegacyParams`, also exists — [PxVehicleSuspensionForceParams](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/_api_build/structPxVehicleSuspensionForceParams.html); [Legacy params](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/_api_build/structPxVehicleSuspensionForceLegacyParams.html)
- High natural frequencies "can result in the car responding very strongly and quickly to the bump, with the wheel possibly even leaving the road for a short while" — [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)

**Travel and the road query**
- `PxVehicleSuspensionParams` holds `suspensionAttachment`, `suspensionTravelDir`, `suspensionTravelDist` and `wheelAttachment`. The road query starts at maximum compression and casts along the travel direction to maximum droop. `suspensionJounceCalculationType` chooses raycast or sweep, and the guide says sweep "generally produces better results but at increased computational cost" — [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- Jounce is measured from maximum droop and clamped to [0, `suspensionTravelDist`]. A separate `separation` field is 0 when the travel can place the wheel on the ground. It is negative when the wheel penetrates the ground even at maximum compression, and the docs say "this penetration may be resolved by using a constraint that simulates the effect of a bump stop" — [PxVehicleSuspensionState 5.5.1](https://nvidia-omniverse.github.io/PhysX/physx/5.5.1/_api_build/structPxVehicleSuspensionState.html)
- In USD, `PhysxVehicleSuspensionAPI.travelDistance` replaces the deprecated `maxCompression`/`maxDroop` — [omni_usd_schema_physics PhysxVehicleSuspensionAPI](https://docs.omniverse.nvidia.com/kit/docs/omni_usd_schema_physics/latest/class_physx_schema_physx_vehicle_suspension_a_p_i-members.html)

**Placing the wheel, and limiting extension** (this is why there is no wheel hop)
- `PxVehicleSuspensionStateCalculationParams::limitSuspensionExpansionVelocity`: by default, "if a hit with the ground is detected, the suspension jounce will be set such that the wheel is placed on the ground", which "can result in large changes to jounce within a single simulation frame".
- When the flag is enabled, "the suspension expansion speed will be limited to what can be achieved given the time step, suspension stiffness etc.", with the trade-off that "the wheel might loose contact with the ground more easily".
- The guide adds that this flag "places physical limits on the rate at which the suspension spring elongates", that "wheels spend more time not in contact with the ground", and that then "the tire cannot deliver longitudinal or lateral force".
- Sources: [PxVehicleSuspensionStateCalculationParams 5.5.0](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/_api_build/structPxVehicleSuspensionStateCalculationParams.html); [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)

**Wheel mass**
- `PxVehicleWheelParams::mass` is "the combined mass of the wheel and the tire in kg". The guide gives it no vertical role. It appears with the wheel's moment of inertia and spin dynamics. No unsprung-body or tire-vertical-stiffness parameter appears anywhere in the vehicle2 suspension or wheel params I read — [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)

**Full compression: the suspension-limit constraint**
- With `PxVehiclePhysXConstraintComponent`, the wheel is clamped at maximum compression, and a custom constraint on the PhysX actor pushes the vehicle up over the next simulate step.
- Without a PhysX actor, "the wheel will remain below the ground plane".
- The guide's mitigations: substeps or small timesteps reduce breaches, and "a custom bump-stop spring" can add a nonlinear force near maximum compression. That spring is the user's job, not built in.
- Source: [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- `PxVehiclePhysXSuspensionLimitConstraintParams` has two members:
  - `restitution` sets a target velocity through PhysX's `Px1DConstraintFlag::eRESTITUTION`. 0.0 disables the restitution model.
  - `directionForSuspensionLimitConstraint` is `eROAD_GEOMETRY_NORMAL`, `eSUSPENSION` or `eNONE`. `eSUSPENSION` keeps wheels above ground better when the car is on its side, but costs "an unnaturally strong torque that can lead to unwanted self-righting behaviour". `eROAD_GEOMETRY_NORMAL` "is a good choice to avoid self-righting behaviour". `eNONE` applies no extra impulse.
  - Source: [PxVehiclePhysXSuspensionLimitConstraintParams](https://nvidia-omniverse.github.io/PhysX/physx/5.5.0/_api_build/structPxVehiclePhysXSuspensionLimitConstraintParams.html)
- An NVIDIA engineer described a PhysX-vehicle vertical velocity jump as "the instantaneous bounce off the suspension limit that is enforced by the suspension limit constraint", and suspected the spring was "too stiff or underdamped" and "too stiff for the timestep". The thread later traced that user's jitter to rendering (TAA), and another user attributed the bounce to the chassis hitting the plane — [NVIDIA forum, "vertical dir of vehicle is unstable"](https://forums.developer.nvidia.com/t/how-to-solve-the-vertical-dir-of-vehicle-is-unstable/54165)
- When a wheel is held up by rigid-body contact instead of the suspension, handling suffers, and the resulting large suspension force can flip the car — [NVIDIA forum, wheel-on-wheel interaction](https://forums.developer.nvidia.com/t/how-to-handle-wheel-on-wheel-interaction-with-physx-vehicles/41424)
- Isaac Sim users report vehicles bouncing on drop-in at spawn, and NVIDIA's advice is to spawn at ground level — [NVIDIA forum, Vehicles Wizard standalone](https://forums.developer.nvidia.com/t/vehicles-wizard-assets-usage-in-standalone-env/318996)

**Anti-roll bars and compliance**
- `PxVehicleAntiRollBar` has `wheel0`, `wheel1` and `stiffness`. Its torque is proportional to the difference in spring displacement between the two wheels — [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- `PxVehicleSuspensionComplianceParams` maps normalised compression (0 = max elongation, 1 = max compression) to four quantities through graphs of 0-3 points (0 = none, 1 = constant, 2 = linear, 3 = two-segment piecewise):
  - toe angle
  - camber angle
  - suspension force application point
  - tire force application point
  - Source: [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- For a 4-wheeled car, the guide puts the suspension force application point "somewhere above the wheel center but below the center of mass of the rigid body" — [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)

### Inferences
- **Natural frequency units:** the doc's "5 to 10" only gives realistic cars when read as rad/s (ω): 0.8-1.6 Hz. Our spec's 1.6 Hz is ω = 10.05 rad/s, the top of PhysX's "family car" band. Do not feed Hz into `stiffness = ω² m`.
- **No wheel hop, by construction.**
  - With the flag off, the wheel snaps to the ground each step: no wheel mass ever has to be accelerated down into a pothole, and nothing oscillates against tire stiffness.
  - With `limitSuspensionExpansionVelocity` on, a wheel can drop into a pothole only as fast as the spring can push the wheel down. That is a crude stand-in for unsprung inertia, and arguably the right default for potholes and washboard, because a car that never loses contact on washboard is the unrealistic one.
  - The oscillation at 10-15 Hz still does not exist.
  - Washboard grip loss would therefore come only from expansion-limit lift-off, not from a resonant tire-load variation.
- **Full compression is a rigid stop, not a bump stop.**
  - A rock taller than the remaining travel produces a constraint impulse sized to resolve the penetration within about one step.
  - Pitch and jump will happen, but the impulse duration is about one simulation step, not the 10-30 ms of a real elastomer or hydraulic bump stop.
  - `restitution` > 0 adds bounce. Set it to 0 to avoid artificial rebound.
  - `eROAD_GEOMETRY_NORMAL` is the less self-righting choice.
- **Neither hit nor landing is resolved in time.** Under the default "place wheel on ground" behaviour, a 5 cm step at 20 m/s is taken in one step: the spring sees an instantaneous jounce step, and the damper sees a huge jounce velocity of about 0.05 m in 1/1440 s, roughly 70 m/s. That gives a damper-force spike. With linear damping, which no real car has (real dampers blow off or digress at high shaft speed), that spike is unrealistically large.
  - So a realistic impact response needs three things: (a) the expansion-limit flag, (b) a velocity-saturating damper added ourselves, and (c) a progressive bump stop added ourselves.
- **Single linear damper.** Real dampers have about 2-3x more rebound than bump force (a common rule of thumb; no source found in this pass). Without asymmetry, a car landing a crest rebounds as hard as it compresses.
- **Configuration gaps in aib today:**
  - Raycast, not sweep. A single ray under the wheel centre meets a rock narrower than the tire only when the ray hits it, and a sweep at least gives the tire's profile.
  - No anti-roll bars.
  - Constant compliance graphs.
  - Whether Isaac 6.1's USD exposes `limitSuspensionExpansionVelocity` and the suspension-limit restitution and direction is not confirmed from docs (see Gaps).

### Gaps
- I could not fetch the Omniverse attribute reference for `PhysxVehicleAPI` (HTTP 403), so I could not confirm the USD spellings in Isaac Sim 6.1. I expect a `limitSuspensionExpansionVelocity` attribute on `PhysxVehicleAPI` (present in recent omni.physx) and suspension-limit settings, but this is unverified. Check `dir(PhysxSchema.PhysxVehicleAPI)` in Isaac's Python.
- I found no NVIDIA statement that says outright "the wheel has no vertical DOF". The conclusion is inferred from the documented "placed on the ground" jounce rule and from the absence of any unsprung vertical parameter.
- I found no source for real Model 3 bump/rebound damping curves.

## 2. Real-world values: sedan versus rally car

### Takeaway
Passenger cars ride at about 1-1.6 Hz with damping ratio about 0.2-0.5. Wheel hop sits at about 10-15 Hz with about 35-55 kg unsprung mass per corner. Rally2/R5 gravel cars run about 250-300 mm of travel with hydraulic bump stops tuned for landings. No Model 3 specific ride numbers are published.

### Cited Findings
- **Tesla Model 3:** no published unsprung mass or ride frequency. Tesla engineers chose a vertical frequency that feels like "a brisk walk or slow run" (qualitative only) — [Autocar, secret tech behind the Model 3](https://www.autocar.co.uk/node/88109); [Autocar Pro](https://www.autocarpro.in/news-international/revealed-tesla-model-3-secret-technology-41076)
- **Ride frequency formula:** f = (1/2π)·√(wheel rate / corner sprung mass), where sprung mass is corner mass minus unsprung mass — [HP Academy, suspension frequency](https://hpacademy.com/courses/suspension-tuning-and-optimization/foundation-concepts-suspension-frequency); [Penske, ride frequency](https://www.penskeshocks.com/blog/natural-frequency-ride-frequency-cpm-in-race-suspension)
- **Unsprung mode, measured example:** a 53.8 kg unsprung mass gives an uncoupled mode of 10.5 Hz, and the FE model computes 13.6 Hz — [Scribd "Dynamics Tire Wheel" (FE study; secondary copy)](https://www.scribd.com/document/578936503/Dynamics-Tire-Wheel)
- **Typical 2-DOF quarter-car parameters:** sprung mass 240 kg, unsprung mass 36 kg — [Anadolu U. quarter-car active suspension paper](https://avesis.anadolu.edu.tr/yayin/3c3d16c0-3e93-4ff2-bcd1-781832e2048d/aspects-of-achievable-performance-for-quarter-car-active-suspensions)
- **Wheel-hop damping:** "tire damping by itself has little influence on the wheel-hop vibration since this mode is mainly damped by the shock absorber" — [same source](https://avesis.anadolu.edu.tr/yayin/3c3d16c0-3e93-4ff2-bcd1-781832e2048d/aspects-of-achievable-performance-for-quarter-car-active-suspensions)
- **Higher unsprung mass hurts ride comfort** (in-wheel motor study) — [Atlantis Press PEEE-15](https://www.atlantis-press.com/proceedings/peee-15/24839)
- **Rally travel**
  - GR Yaris Rally2: +40 mm of suspension travel over the road car — [Toyota GR](https://www.toyota.com.au/gazoo-racing/journal/gr-yaris-rally2)
  - VW Polo R5: about 280 mm front and rear. This is an unverified Reddit claim — [r/WRC](https://cal1.lr.ggtyler.dev/r/WRC/comments/1fkwbq9/hyundai_focusing_on_making_its_rally2_a_tarmac)
  - Generic gravel rally travel ">280 mm" / "sometimes exceeding 300 mm" — [engineerfix](https://engineerfix.com/what-is-a-rally-car-engineering-safety-and-classes/)
  - Hyundai: the damper is longer on gravel than on tarmac, with dedicated set-ups for each — [Hyundai Motorsport](https://www.hyundaimotorgroup.com/en/story/CONT0000000000002514)
- **Rally jumps and landings (Škoda)**
  - Brynildsen flew 39 m at Colin's Crest in a Fabia R5. Long jumps are airborne for more than 1 s.
  - Landing on one wheel is "especially critical for half-shafts, differentials".
  - Dampers are set stiffer "especially for fast compression" on jump-heavy stages.
  - On short jumps the wheels do not fully extend, so "a hydraulic bump stop in the shock absorber absorbs part of the landing energy".
  - Source: [Škoda Storyboard, How to fly a rally car](https://www.skoda-storyboard.com/en/lifestyle/motorsport/how-to-fly-a-rally-car/)
- **Damper adjustment:** dampers commonly adjust for rebound and for (fast/slow) compression separately — [WRC glossary](https://www.wrc.com/en/misc/a-z)

### Inferences
- Our spec's 20 kg "wheel mass" covers wheel and tire only, which is right for PhysX's spin inertia. A real corner's unsprung mass (wheel, tire, hub, brake, knuckle, part of the arms and half-shaft) is roughly 35-55 kg, in line with the 36 kg and 53.8 kg examples above. Any unsprung model we add should use roughly 40-45 kg per corner, not 20 kg.
- Wheel-hop frequency ≈ (1/2π)·√((k_s + k_t)/m_u):
  - Inputs: at 1.6 Hz on about 400 kg sprung per corner, k_s ≈ 40 kN/m. A passenger tire's vertical stiffness is k_t ≈ 200-300 kN/m (a common textbook range, not sourced in this pass).
  - Result with m_u = 45 kg: about 11-13 Hz, consistent with the cited 10.5-13.6 Hz.
- Damping ratio 0.45 matches a sedan, comfort-leaning. Rally cars run higher and asymmetric damping.
- Our 0.2 m of total travel is plausible for a sedan but generous: road cars usually have roughly 150-200 mm wheel travel (no source found). Rally is 250-300 mm.

### Gaps
- I found no source for Model 3 spring rates, damper curves, bump-stop free length or progressive rate, or measured unsprung mass.
- I found no published bump-stop force-deflection curve. In general these are elastomeric, highly progressive, and engage over the last 20-40 mm of travel (unsourced).

## 3. Does 60 Hz × 3 substeps (180 Hz) resolve wheel hop and impacts? What other simulators use

### Takeaway
180 Hz samples a 10-15 Hz mode at 12-18 points per cycle, which is enough for a correctly integrated stiff mode. A 10-30 ms impact gets only 2-5 scene steps, though, and PhysX has no wheel-hop mode to resolve anyway. Simulators that do model unsprung dynamics run at 333-2000 Hz. aib's vehicle-internal substeps (8 per scene step, about 1440 Hz for the suspension and tires) cover the spring and tire update, but not the rigid-body/constraint solve the impacts go through.

### Cited Findings
- **PhysX stability criterion:** alpha = sqrt(sprungMass/stiffness)/timestep, recommended above 5.0. On jitter: "substeps may be applied to the stiffest parts of the vehicle's update" — [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- **Physics rates of other simulators** (a community compiled list; treat as approximate):
  - rFactor 2: about 400 Hz
  - rFactor Pro: 800 Hz
  - Assetto Corsa: 333 Hz (possibly raised later, unconfirmed)
  - BeamNG: 2000 Hz
  - Forza: 360 Hz
  - Source: [overtake.gg thread](https://www.overtake.gg/threads/what-rate-do-physics-run-at-1000hz-500.159380/); [LFS forum](https://lfs.net/forum/post/939046)
- **Richard Burns Rally:** the NGP 7.0 update moved RBR to a "fixed physics update rate of 720 Hz", from about 150 Hz before (community changelog quote) — [io-tech forum](https://bbs.io-tech.fi/threads/ajopeliketju.27082/post-11836285)
- **BeamNG's 2 kHz** comes only from a promotional blog ("updates at a rate of 2,000 times per second") — [boxthislap](https://boxthislap.org/beamng-drive-the-definitive-soft-body-physics-sandbox-arrives-on-ps5/). BeamNG's own physics page gives no Hz figure — [beamng.com physics](https://beamng.com/game/about/physics)
- **Chrono multibody step sizes:** a user with a double-wishbone Chrono vehicle (TMeasy tire, NSC contact) found 2 ms workable and saw larger steps destabilise the suspension joints (anecdotal) — [Chrono Google Group](https://groups.google.com/g/projectchrono/c/RUJXATe5MRA/m/MPA9LUGRAAAJ)
- **Bushings and real time:** rubber bushes add high-frequency modes that force small steps. Real-time multibody suspensions use L-stable or generalized-α integrators to damp them — [JSME 2020](https://www.jstage.jst.go.jp/article/transjsme/86/881/86_19-00234/_article/-char/en); [Multibody Syst Dyn 2024](https://link.springer.com/10.1007/s11044-024-09984-2); [Chalmers/Volvo elasto-kinematic RT](https://research.chalmers.se/publication/251960/file/251960_Fulltext.pdf)

### Inferences
- **Ride mode (1.6 Hz):**
  - At 180 Hz: alpha = 1/(2π·1.6·(1/180)) ≈ 17.9.
  - At 1440 Hz (vehicle substeps): alpha ≈ 143.
  - Both are far above 5.
- **Explicit integration of a 45 kg unsprung mass on k_s + k_t ≈ 290 kN/m:**
  - ω ≈ 80 rad/s (12.8 Hz).
  - At 180 Hz: ω·dt ≈ 0.45, which is stable for semi-implicit Euler (limit 2) but loses about 5-10 % amplitude and phase per cycle.
  - At 1440 Hz: ω·dt ≈ 0.06, which is accurate.
  - So an unsprung-mass model we add should run at about 1 kHz (our own substep loop), or at 180 Hz with an implicit update.
- **Impact durations:**
  - A 10-30 ms rock strike at 180 Hz spans 1.8-5.4 steps.
  - A rigid limit constraint resolves it in about one step (5.6 ms), so peak chassis acceleration will be about 2-5x too high and too short.
  - Raising the scene to 360-720 Hz (as rFactor 2 and RBR do), or adding a compliant bump stop that spreads the load over its stroke, fixes this.

### Gaps
- I found no official Kunos or Studio 397 statement of their current rates (community figures only), and no KT Racing (WRC games) figure.
- I did not confirm whether omni.physx's vehicle substeps also resubstep the suspension-limit constraint. Probably not: the constraint is solved in the scene's rigid-body solve.

## 4. Ways to fill the gaps: our own forces, articulated unsprung bodies, Chrono, BeamNG-style

### Takeaway
The cheapest high-value fix keeps the PhysX vehicle and adds our own per-corner "virtual unsprung" model in Python:
- a progressive bump stop
- digressive, asymmetric damping
- optional wheel-hop states

It applies the difference as forces through `PhysxForceAPI` or apply_force at the suspension points, and drives the tire load through the jounce. Full fidelity (real wheel hop, kinematics, compliance) needs a multibody vehicle: Isaac articulation with prismatic or revolute suspension joints, or Chrono::Vehicle as co-simulation. That costs a rewrite of the tire contact and of the PhysX vehicle's drive and tire model.

### Cited Findings
- The PhysX guide itself suggests "a custom bump-stop spring" adding a nonlinear force near max compression — [PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- vehicle2 is modular: users can replace components. For example `PxVehiclePhysXConstraintComponent` is optional, and suspension state and force functions are free functions in `PxVehicleSuspensionFunctions.h`, such as `PxVehicleSuspensionStateUpdate` with the raycast/sweep and expansion-limit options — [PxVehicleSuspensionFunctions 5.4.1](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/_api_build/function_PxVehicleSuspensionFunctions_8h_1ac500ac525f7e0f04d166c4dc22be1676.html). Isaac/omni.physx exposes the USD schema, not the C++ component pipeline (inferred: no Python hook into component order was found).
- With `PhysxVehicleWheelControllerAPI` on the wheel attachments, wheel drive, brake and steer can be set directly, bypassing PhysX's drive/differential/brake/steer APIs. aib already does this for sim_vehicle_ecu — [omni.physx.vehicle USD representation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/ux/source/omni.physx.vehicle/docs/usd_representation.html); aib `vehicle.py:226`
- **Chrono::Vehicle** provides double-wishbone templates, including a reduced one with distance constraints, and sedan models (`Sedan_DoubleWishbone`) with bushings and full multibody suspensions — [Chrono DoubleWishbone](https://api.projectchrono.org/6.0.0/classchrono_1_1vehicle_1_1_double_wishbone.html); [Sedan_DoubleWishbone](https://api.chrono.projectchrono.org/classchrono_1_1vehicle_1_1sedan_1_1_sedan___double_wishbone.html); [Chrono vehicle overview](https://api.chrono.projectchrono.org/vehicle_overview.html)
- **BeamNG's node/beam approach**
  - Massive nodes and massless spring-damper beams.
  - Typical values: suspension spring `beamSpring` 40000 / `beamDamp` 0; damper `beamSpring` 0 / `beamDamp` 4500; structural arms `beamSpring` 8,000,000 / `beamDamp` 125.
  - "overly stiff beams on light nodes can make a structure vibrate or explode", which is why it runs at a high rate.
  - Source: [beamng.com physics](https://beamng.com/game/about/physics); [JBeam physics theory](https://wiki.beamng.com/JBeam_Physics_Theory)

### Inferences (design options, with costs)
1. **Keep PhysX vehicle2 and tune it (hours)**
   - Enable `limitSuspensionExpansionVelocity`, which gives pothole wheel drop and lift-off on washboard.
   - Switch `suspensionLineQueryType` to sweep, so rocks narrower than the ray spacing still register.
   - Set suspension-limit restitution to 0 and direction to `eROAD_GEOMETRY_NORMAL`, if Isaac exposes them.
   - Add `PhysxVehicleAntiRollBarAPI` (or the equivalent USD; unverified name).
   - Raise the scene rate to 360-720 Hz for impact events.
2. **Virtual bump stop and damper, as forces (about a day)**
   - Each substep, read jounce and jounce speed from the wheel state (omni.physx exposes wheel or suspension state queries; aib already reads `wheel_states`).
   - Compute F_extra = F_bumpstop(x_bs) + (F_damper_real(v) − c_linear·v).
   - Apply equal and opposite at the suspension attachment on the chassis. There is no unsprung body, so only the chassis receives it; the ground side is implicit.
   - Set the PhysX travel slightly longer than the real travel, so the rigid limit is the last resort and the elastomer does the work.
   - Limitation: the tire load PhysX uses for grip still comes from its own spring force. The extra force does not reach the tire's normal load unless it is also folded into the friction or `PhysxVehicleTireAPI` scaling we already write per wheel.
3. **Virtual unsprung mass in Python (a few days)**
   - Per corner, integrate z_u with m_u ≈ 45 kg, the tire as a spring k_t against the road height from the raycast hit, and the suspension spring and damper against the chassis, at about 1 kHz in our substep loop.
   - Apply the resulting suspension force to the chassis.
   - Feed the tire load (k_t·penetration, clipped ≥ 0) into the grip: friction or stiffness scaling per wheel, which aib already writes (`friction_values`, `stiffness_values`).
   - This gives true wheel hop and washboard grip loss.
   - To stop PhysX's own spring double-counting, its spring must be nulled (very soft spring, travel used only for geometry). That fights the PhysX tire model's dependence on suspension force for normal load, so it is a significant risk; the ECU's slip estimators see the result.
4. **Articulated vehicle in Isaac (weeks)**
   - Build the chassis plus four wheel bodies (about 45 kg) on prismatic suspension joints with drive stiffness and damping, or full wishbones as revolute chains.
   - Wheels collide as cylinders or convex hulls, with PhysX rigid contact and friction: no slip-based tire model, and poor tire forces unless a custom tire force is applied.
   - Isaac's wheeled-robot tooling targets differential-drive robots, not car tires.
   - This loses `PhysxVehicleTireAPI`, sticky-tire handling and everything aib calibrated.
5. **Chrono::Vehicle co-simulation (weeks to months)**
   - A multibody sedan with bushings and a TMeasy or Pac tire, at about 1-2 ms steps.
   - Isaac becomes rendering and sensors only, with the pose pushed each step, like SSv2 puppeteering.
   - Most faithful, and the biggest integration cost: a terrain/height map must be shared, and Chrono becomes the physics owner.

### Gaps
- I did not verify omni.physx's Python API for per-wheel suspension jounce and velocity, or the exact anti-roll bar USD API name in Isaac 6.1.
- I did not measure the cost or CPU timing of options 3-5.

## 5. Chassis-ground strikes and high-centering

### Takeaway
In PhysX vehicle2, underbody strikes happen only through ordinary rigid-body contact of whatever collision shape the chassis carries. The wheels do not collide with the world: they use scene queries. aib's chassis box bottom sits at max(box bottom, r + 0.05 m), and is filtered so "only the wheels touch" the ground in normal driving. Whether it collides with rocks depends on the collision groups. Realistic high-centering needs the underbody collider at real ground clearance (Model 3 about 140 mm; not sourced here), plus a friction material for the skid-plate contact.

### Cited Findings
- The wheel's collision geometry can be any shape ("does not have to be a Mesh, a Cylinder or other type will work too"). Wheel-attachment prims follow the wheel motion — [omni.physx.vehicle USD representation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/ux/source/omni.physx.vehicle/docs/usd_representation.html)
- Rigid-body contact supporting a wheel (instead of the suspension) degrades handling and can create huge suspension forces that flip the car — [NVIDIA forum](https://forums.developer.nvidia.com/t/how-to-handle-wheel-on-wheel-interaction-with-physx-vehicles/41424)
- One forum reading of a bounce event blamed "the chassis hitting the plane" rather than the suspension limit — [NVIDIA forum](https://forums.developer.nvidia.com/t/how-to-solve-the-vertical-dir-of-vehicle-is-unstable/54165)
- aib's chassis collider: a box, `bottom = max(cz - dz/2, r + 0.05)`, in the `chassis` collision group; wheel queries use `groundQuery` — `src/autoware_isaac_bridge/src/aib/aib/isaac/vehicle.py:240-252` (local)

### Inferences
- With r ≈ 0.34 m, the box bottom is at ≥ 0.39 m above the ground at rest, more than twice a real sedan's clearance, so rocks will essentially never strike the underbody.
- For rock strikes, replace the box with a convex-hull underbody, or add a thin "skid plate" box at the real clearance with low restitution and a steel-on-rock friction of about 0.3-0.5 (unsourced). It must collide with the rock and terrain groups.
- Expect PhysX contact to produce an impulsive, effectively rigid strike: no crush or deformation. A small `contactOffset`/`restOffset` and the scene rate govern how sharp it is.

### Gaps
- I found no source for the Model 3's ground clearance or underbody geometry, or for skid-plate friction coefficients.

## 6. Landing loads, jumps and pothole events: published measurements

### Takeaway
Quantitative open data is scarce. SAE pothole papers exist, but their load numbers are behind paywalls. Wheel-force-transducer traces show the shape: a load drop as the wheel falls in, then a large spike on the exit edge. Rally sources give qualitative landing practice (hydraulic bump stops, fast-compression damping) and jump lengths, not loads.

### Cited Findings
- Wang & Basch, SAE 2007-01-1647, "Effects of Braking on Suspension Loads in Potholes": speed, front corner weight and suspension stiffness drive the pothole loads, and braking changes them. Numbers are not in the abstract — [SAE Mobilus](https://saemobilus.sae.org/papers/effects-braking-suspension-loads-potholes-2007-01-1647)
- SAE 2024-28-0262 covers pothole and curb impact forces on wheel rims and a flange redesign — [SAE Mobilus](https://saemobilus.sae.org/papers/study-impact-forces-wheel-rim-pothole-impacts-improvement-flange-design-eliminate-wheel-bend-2024-28-0262)
- A wheel-force-transducer pothole trace: vertical force drops during the free fall into the hole, then a large spike when the wheel strikes the far edge, then it settles — [Michigan Scientific durability case study](https://www.michsci.com/durability-testing-case-study/)
- In a bicycle pothole study, peak horizontal force tracks vertical wheel drop (an analogue only) — [SAE 2024-01-2755](https://saemobilus.sae.org/papers/testing-analysis-riding-a-road-bicycle-artificial-pothole-2024-01-2755)
- Instrumented bump stops for measuring vertical suspension forces exist as patents — [US 7192041](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7192041); [US 7032912](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7032912)
- Rally jumps: 39 m at Colin's Crest; short jumps under 1 s airborne; hydraulic bump stops absorb landing energy on short jumps; one-wheel landings are the drivetrain's worst case — [Škoda Storyboard](https://www.skoda-storyboard.com/en/lifestyle/motorsport/how-to-fly-a-rally-car/)

### Inferences
- Validation targets we can derive without paywalled data:
  - Pothole wheel drop: the wheel should start losing load immediately, with load near zero while airborne. This needs `limitSuspensionExpansionVelocity` or an unsprung model, because a snapped wheel never unloads.
  - Exit-edge spike: it should hit the bump stop at moderate speed.
  - Landing from a crest: free fall from height h gives a vertical touchdown speed of v = √(2gh). From 0.5 m that is 3.1 m/s, a damper shaft speed far above any linear-damper calibration range. Real dampers digress there and bump stops take over, which is why the linear PhysX damper over-predicts the first landing spike unless capped.
- Energy check for a landing: a 1760 kg car at 3.1 m/s carries about 8.5 kJ vertically. Over 0.15 m of remaining stroke, the average force is about 57 kN, or about 3.3 g on top of 1 g. A plausible bump-stop sizing target, not a measured value.

### Gaps
- I found no open published peak loads (in N or g) for passenger-car pothole strikes or bump-stop engagement. The SAE full texts are paywalled.
- I found no published rally landing g-loads or bump-stop curves (teams keep them confidential).
- I found no ISO/SAE standard proving-ground event definitions (for example Belgian blocks or potholes) in open sources during this pass.
