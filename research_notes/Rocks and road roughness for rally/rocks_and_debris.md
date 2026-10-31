# Rocks, stones and loose debris on off-road/rally roads as physical objects (PhysX 5 / Isaac Sim 6.1, real time)

Scope note: notes for a PhysX 5 vehicle ego (raycast/sweep wheels) plus NPC cars, 60 Hz x 3 substeps, km-long worlds. Tire enveloping and road roughness spectra are left to other researchers. Each item marked as a sourced finding cites its source. Items under "Inferences" are my reasoning, not sourced. About 17 tool calls were used, so several areas (game-specific internals, ROAMS, TartanDrive/RELLIS) stay thin and are listed under Gaps.

## 1. Static vs kinematic vs dynamic rocks: size/mass classes and thresholds

### Takeaway
No source gives a standard size threshold. Engines and games keep most rocks static, because dynamic props cost a lot. Rover simulators also treat rocks as rigid collidable obstacles on a deformable or rigid ground. A three-class split (static boulders, a few dynamic movable rocks near vehicles, gravel as a surface property) fits the sourced practice, but the thresholds below are my own inference.

### Cited Findings
- BeamNG: a team programmer said "with dynamic objects the performance impact will be considerably higher, so we've intentionally reduced the amount of dynamic objects in official stages". The quote is from a BeamNG forum post in the search results. The exact post URL is uncertain; it is one of [BeamNG forum post 1866850](https://beamng.com/posts/1866850) or [post 1863715](https://beamng.com/posts/1863715).
- BeamNG players report that light props (barriers, signs) have no collision, which they attribute to performance; rock props are not cheap node/beam objects either. A 2013 thread estimated ~4-5 nodes per rock against ~300 nodes per car ("roughly 60 rocks for the price of a car"). A 2023 suggestion asked for a separate hard-body system because soft-body rocks feel "unrealistically bouncy". Both are community posts, not official — [Dynamic Hard-Body Physics For Props](https://beamng.com/threads/dynamic-hard-body-physics-for-props.91433); ['Dynamically Static' Objects](https://beamng.com/threads/dynamically-static-objects.13826/)
- A player proposal (not a confirmed feature) is "dynamically static" props: static until hit, dynamic while moving, then static again once at rest — [BeamNG forum](https://beamng.com/threads/dynamically-static-objects.13826/)
- Chrono VIPER digital twin (POLAR-Sim): terrain is deformable SCM and rocks are collidable rigid obstacles. "when encountering rocks too large to run over, the rover's wheels just slipped around and circumvented the obstacles". No real-time factor is given — [POLAR-Sim, arXiv 2309.12397](https://arxiv.org/pdf/2309.12397)
- Sim2Dust / Space Robotics Bench (Isaac Lab): Perlin-displaced terrain mesh, with "procedural boulders and rocks scattered across the terrain using Poisson disk sampling" — [Sim2Dust, arXiv 2508.11503](https://arxiv.org/pdf/2508.11503)
- PhysX kinematic actors push dynamic bodies but are not pushed back, and can crush dynamics into static geometry. There is no kinematic-static collision by default. Kinematics sleep unless given a target — [PhysX 5.4.1 Rigid Body Dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html)
- Static actors cannot use an SDF triangle mesh. Dynamic triangle meshes are supported only with an SDF (and only on the GPU pipeline in 5.3) — [PhysX 5.6.1 Rigid Body Collision](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/RigidBodyCollision.html); [PhysX 5.3.1 Rigid Body Collision](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/docs/RigidBodyCollision.html)
- For geometry that will never move, Omniverse suggests a static triangle-mesh collider (or a bounding cube) as the workaround when a convex hull cannot cook for GPU — [Omni Physics performance guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/physics-performance.html)

### Inferences
- **Classes I propose (unsourced; tune by test):**
  - **Static (embedded boulders, outcrops):** rocks whose mass is a large fraction of the car's, or which are partly buried. Example: a basalt boulder 0.6 m across is about 0.6^3 x 0.52 x 2700 ≈ 300 kg, so a hit should deflect or launch the car, not move the rock. Use a static triangle mesh or, better for contact quality, a static convex hull (≤64 vertices/faces for GPU, see section 2).
  - **Dynamic (1-50 kg, roughly 0.09-0.33 m across at 2.6-2.7 t/m^3):** the band where a car should visibly shove, roll or kick a rock. These need convex hulls, CCD (section 2) and an activation region (section 3).
  - **Not geometry (gravel, under about 3-5 cm, i.e. well below the wheel query's resolution):** a friction/rolling-resistance surface property plus cosmetic particles (sections 4 and 6).
- **Relation to tire radius and clearance (geometry, not sourced):**
  - A rigid wheel of radius r meets a step of height h at contact angle cos θ = 1 − h/r. A ~0.33 m-radius car tire meets a 10 cm stone at about 46°, and a stone with h ≥ r is effectively a wall.
  - Rocks taller than the ground clearance (about 0.14-0.2 m for a sedan, ~0.25 m for a rally-spec car) can strike the underbody, so they must collide with the chassis shapes.
  - Practical rule: anything with h > ~0.3 r matters to wheel dynamics. Anything with h > clearance also needs chassis collision. Anything with h < ~0.1 r can stay surface roughness or gravel.
- Kinematic rocks are rarely useful here. A rock does not move by itself, and a kinematic one would crush small dynamic stones against static ground.

### Gaps
- I found no published game or simulator rule (BeamNG, SnowRunner, Dirt Rally, Forza, GT, Arma) that states a rock size or mass threshold between static and dynamic.
- I found no source that relates rock class to tire radius or ground clearance. The thresholds above are geometric inference.

## 2. PhysX 5 specifics: colliders, offsets, CCD, solver, mass ratio, sleeping, broadphase, instancing

### Takeaway
Use GPU-cookable convex hulls (≤64 vertices/polygons) for dynamic rocks, and static convex or triangle meshes for boulders. Avoid SDF for rocks unless they are concave and dynamic: one user measured SDF about 20x slower than convex. Prefer speculative CCD (cheap, GPU-friendly) over sweep CCD, which runs on the CPU even in a GPU scene. Use TGS, which handles high mass ratios better. The default GPU buffers are sized for about 10^4 bodies, and NVIDIA has tested more than 10^5 internally. Large counts are feasible only if most rocks sleep or are not dynamic at all.

### Cited Findings
**Colliders**
- Collider cost, cheapest first: primitives (box, sphere, capsule, plane), then convex meshes, then cylinders. "Use the simplest collider" — [Isaac Sim 6.1 Performance Optimization Handbook](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html)
- On GPU, "convex hulls with more than 64 vertices or polygons" fall back to CPU contact generation. There is also a 32-vertices-per-face limit. Cooking with `eCOMPUTE_CONVEX` respects the 64 limit, and `buildGPUData` must be set — [PhysX 5.4.0 GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GPURigidBodies.html)
- High-aspect-ratio (long, thin) convex meshes can fail GPU cooking and fall back to CPU, which "can significantly impact performance". The workaround is a bounding cube, or a static triangle mesh for non-dynamic geometry — [Omni Physics performance guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/physics-performance.html); [Isaac Sim performance handbook search summary](https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html)
- GPU contact generation requires `PxSceneFlag::eENABLE_PCM`. Pairs that use contact modification run on the CPU — [PhysX 5.4.0 GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GPURigidBodies.html)
- SDF cost:
  - The triangle count drives cost and the SDF resolution mostly drives memory; "Convexes will always be faster".
  - A user reported SDF colliders running "like 20x slower than using convex hull" — [NVIDIA forum: SDF Mesh Collision Performance](https://forums.developer.nvidia.com/t/sdf-mesh-collision-performance/270825)
  - SDF colliders generate many contacts. Around 250 resolution is fine in most cases. Thin features under about twice the grid spacing are lost — [Omni Physics Collision Behavior Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/collision_guide.html)
  - Sparse and dense SDFs perform almost identically — [PhysX 5.3.1 Rigid Body Collision](https://nvidia-omniverse.github.io/PhysX/physx/5.3.1/docs/RigidBodyCollision.html)

**Contact and rest offset**
- The default `contactOffset` is 0.02 m, so the contact distance is 4 cm. `contactOffset` must exceed `restOffset`.
- Use `restOffset` 0 if collision shapes match graphics, and slightly positive to smooth sliding on triangle meshes.
- Keep the gap minimal but jitter-free. Larger time steps need a larger gap.
- An oversized `contactOffset` raises contact count and lowers performance.
- Source: [PhysX 5.4.1 Advanced Collision Detection](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/AdvancedCollisionDetection.html)

**CCD**
- **Sweep CCD:**
  - Needs the scene flag, the pair flag (`eDETECT_CCD_CONTACT` in the filter shader) and the body flag. It activates only above the CCD velocity threshold.
  - `ccdMaxPasses` defaults to 1; the minimum advance coefficient defaults to 0.15. Leftover time is dropped, which can stutter, and "paper-thin/tiny objects should be avoided if possible".
  - Cost rises with speed and with "many fast objects near each other".
  - Source: [PhysX 5.4.1 Advanced Collision Detection](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/AdvancedCollisionDetection.html)
- **Speculative CCD:**
  - Per-body flag (`eENABLE_SPECULATIVE_CCD`) that inflates contact offsets by motion. It catches angular misses that sweep CCD cannot.
  - Cheaper but less robust, and works best with PCM. Allowed on kinematics.
  - Can be combined with sweep CCD since PhysX 5.
  - Source: [PhysX 5.4.1 Advanced Collision Detection](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/AdvancedCollisionDetection.html)
- **Raycast CCD (extensions):** cheap, but approximate for dynamic-vs-dynamic, single-shape actors only, and "not production-ready" — [same](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/AdvancedCollisionDetection.html)
- "Features such as joint projection, CCD and triggers are not GPU accelerated and are still processed on the CPU" — [PhysX 5.4.0 GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GPURigidBodies.html)

**Solver and mass ratio**
- Defaults are 4 position and 1 velocity iterations. If you need more than 30, "reconsider the setup".
- TGS (PhysX 5.1+) is "generally the recommended solver", with better convergence and "better handling of high-mass ratios". Each TGS iteration is slightly slower than a PGS iteration. The solver type is fixed per scene at creation.
- Dominance groups (0-31; pair values 1:1, 1:0, 0:1) and contact-modification mass scaling can bias a contact, for example to let the car ignore a stone's reaction.
- Source: [PhysX 5.4.1 Rigid Body Dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html)
- Omniverse's example fixes 16 position and 1 velocity iterations. PGS is faster per iteration, while TGS is more stable with fewer iterations — [Omni Physics performance guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/physics-performance.html)
- `maxDepenetrationVelocity` is the knob for violent separation. Lowering it "makes the collision response a bit slower and can help to avoid overshooting" — [Omni Physics Collision Behavior Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/collision_guide.html); [PhysX 5.4.1 Rigid Body Dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html)

**Sleeping**
- A body sleeps when its kinetic energy stays below the sleep threshold long enough for the wake counter to run out. Any force or velocity set wakes it. Sleeping saves the work of simulating idle bodies, and the active-actors list reduces transform sync cost — [PhysX 5.4.1 Rigid Body Dynamics](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/RigidBodyDynamics.html)
- Sleeping bodies are removed from the solver, which "drastically reduces their CPU load". Waking must propagate through touching bodies, or you get overlaps — [Box2D: Simulation islands](https://box2d.org/posts/2023/10/simulation-islands/)

**GPU scale and broadphase**
- GPU rigid bodies give "extremely large performance advantages over CPU rigid bodies in scenes with several thousand active rigid bodies" — [PhysX GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GPURigidBodies.html)
- GPU memory buffers cannot grow and must be preallocated (`PxGpuDynamicsMemoryConfig`). The defaults suit about 10,000 rigid bodies. Overflow silently drops contacts, constraints or pairs; read `PxSimulationStatistics` to size them. An out-of-memory error marks the scene corrupted, and it must be recreated — [PhysX 5.4.0 GPU Rigid Bodies](https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/GPURigidBodies.html)
- The GPU broadphase is enabled separately (`PxBroadPhaseType::eGPU`) from GPU dynamics (`eENABLE_GPU_DYNAMICS`). The broadphase has a per-frame new-pair limit, and pairs beyond it are dropped with an error — [PhysX GPU Rigid Bodies (5.1.2)](https://nvidia-omniverse.github.io/PhysX/physx/5.1.2/docs/GPURigidBodies.html)
- An NVIDIA engineer said internal body-count limits were lifted and "we've tested scenes with over 100,000 bodies internally". No frame time was given — [NVIDIA forum: Max number of rigidbodies?](https://forums.developer.nvidia.com/t/max-number-of-rigidbodies/50157)
- For a few bodies, CPU with the MBP broadphase is likely better; for large scenes, GPU — [Omni Physics performance guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/physics-performance.html)
- GPU physics uses one GPU only, and the CPU PhysX default is 8 threads. If Python callbacks are not needed, use asynchronous updates and Fabric instead of writing USD transforms — [Isaac Sim 6.1 handbook](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html); [Omni Physics performance guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/physics-performance.html)
- A user reported Isaac Sim 4.2 becoming unresponsive with a USD of 400 rigid bodies. It is unresolved and likely a setup issue — [NVIDIA forum](https://forums.developer.nvidia.com/t/the-system-crashes-when-performing-simulation-with-400-rigid-bodies-in-isaac-sim/312141)

**Instancing**
- In a PointInstancer whose prototype has a rigid body, each instance becomes a separate PhysX body: one example had "one mesh prim on the stage, but over 800 physx bodies on the GPU".
- Per-instance rigid body properties are not accessible through USD; the workaround is several prototypes with different properties.
- `raycast_closest` returns `protoIndex` for instancer hits.
- Source: [NVIDIA forum: Scene Query with PointInstancer Bodies](https://forums.developer.nvidia.com/t/omnigraph-physx-rfe-scene-query-with-pointinstancer-bodies/238101); [Can I run PhysX rigid body sims on PointInstancer instances?](https://forums.developer.nvidia.com/t/can-i-run-physx-rigid-body-sims-on-usdgeompointinstancer-instances/175150)
- Scenegraph instancing shares meshes and cuts memory, but instanced children cannot be modified individually — [Isaac Sim 6.1 handbook](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html)

### Inferences
- **Mass ratio:** 1800 kg / 5 kg = 360:1. Wheels are scene queries, not bodies (section 3), so the hard pairs are chassis-vs-stone and NPC-chassis-vs-stone only. Small stones should not collide with chassis shapes at all, or should use a dominance group so the car is unaffected (1:0). Stones of 20-50 kg should collide fully, under TGS, with speculative CCD.
- **Tunneling check:** at 30 m/s and 180 Hz (60 x 3), a body moves 0.167 m per substep, more than the 0.09-0.2 m size of a 1-10 kg stone. Discrete contact with the 4 cm default contact distance will miss such impacts.
  - Speculative CCD on the stones (or on the car) covers this, and runs on GPU.
  - Sweep CCD would push the whole CCD stage to the CPU.
- **Body-count budget:** with ~10^4 bodies the default buffers hold; beyond that, raise the buffers.
  - For 10^5 rocks over km of road, nearly all must be static (static shapes do not cost solver time, only broadphase and memory) or sleeping.
  - The awake set should be only the rocks near vehicles, perhaps 10^2-10^3.
  - Use speculative CCD and convex ≤64 vertices so nothing falls back to CPU.
- **contactOffset for stones:** around 1-2 cm, and restOffset 0.
- **PointInstancer:** good for thousands of identical static or dynamic rocks, given a few prototypes (sizes and shapes) with fixed properties. Spawning and despawning individual instances, and per-instance control, are clumsy, so an activation scheme may prefer pooled regular prims.

### Gaps
- I found no measured PhysX 5 frame-time curve against body count (10^3-10^5) for convex rocks, on any GPU.
- I found no source for default sleep thresholds or wake-counter values in PhysX 5 / omni.physx (I believe the wake counter is 0.4 s, unverified).
- I did not research aggregates (`PxAggregate`) and their GPU broadphase interaction.
- I found no documentation on whether omni.physx PointInstancer bodies support speculative CCD, or on which per-prototype attributes apply.

## 3. Interaction with PhysX vehicle wheels (queries, dynamic stones, reaction forces, chassis strikes)

### Takeaway
PhysX vehicle wheels are scene queries (raycast or cylinder sweep) along the suspension direction. Whatever the query filter lets them hit, dynamic actors included, becomes the road plane. The docs describe suspension and tire forces as applied to the vehicle's own actor only. I found no statement that the SDK applies the equal and opposite force to a hit dynamic actor, so a stone under a wheel is not pushed unless you add that force yourself. Non-drivable things touch the car only through rigid-body contact of the wheel and chassis shapes. That is how chassis strikes and high-centering happen.

### Cited Findings
- **The query:**
  - Each suspension casts "downwards along the direction of suspension travel" from max compression to max droop. Raycast from the top of the wheel, or a sweep of a cylinder scaled by the wheel's radius and half-width.
  - The sweep "generally produces better results but at increased computational cost".
  - The vehicle needs no actor for the query to work.
  - Any component may write `PxVehicleRoadGeometryState` (plane and friction) instead, so a custom road model is possible.
  - Source: [PhysX 5.4.1 Vehicles](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- PhysX 5 road-geometry query params take a default filter data, optional per-wheel `filterDataEntries` (which override the default) and a filter callback — [PxVehiclePhysXRoadGeometryQueryParams (5.6.0)](https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/_api_build/structPxVehiclePhysXRoadGeometryQueryParams.html)
- **Drivable vs non-drivable (PhysX vehicle manual):**
  - "suspension raycasts must not hit the shapes of the vehicle issuing the raycasts".
  - Ground is tagged drivable through query filter data.
  - "non-drivable surfaces interact with wheels only through rigid body contact".
  - "The aggregated force of all tire and suspension forces is then applied to the rigid body actor" (the vehicle's).
  - Raycasts ignore the wheel's lateral and longitudinal extent, and sweeps fix that: "all possible contact planes are considered". A blocking sweep returns only the first contact, while non-blocking sweeps return all contacts.
  - Wheel contact modification (SnippetVehicleContactMod) splits contacts into drivable planes (near the suspension direction) and rigid-body contacts (far from it).
  - When a cached plane is used, the reported hit actor and shape are NULL.
  - Source: [PhysX Vehicles manual (archive)](https://archive.docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Vehicles.html)
- If a wheel cannot reach the ground within max compression, it is clamped and "a custom constraint on the vehicle's actor pushes the vehicle upward" over the next step. With `limitSuspensionExpansionVelocity`, wheels may stay airborne — [PhysX 5.4.1 Vehicles](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/docs/Vehicles.html)
- Omniverse vehicle setup uses collision groups GroundQuery, GroundSurface, Chassis and Wheel. The GroundQuery group is set on the wheel attachment's `collisionGroup`, because "the PhysX Vehicle SDK does not simulate the wheel as a separate rigid body". Wheel-chassis pairs under one vehicle root are ignored automatically — [omni.physx.vehicle Collision Filtering & Materials (104.1)](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/104.1/source/extensions/omni.physx.vehicle/docs/collision_filtering_and_materials.html) (from the search summary; the page returned 403 when fetched)
- Forum: by tweaking the collision and raycast filters "you can decide which shapes in your scene will be drivable". If the query filter is wrong, the wheels sink until the chassis hits the ground — [NVIDIA forum: Car with PxVehicleDrive4W](https://forums.developer.nvidia.com/t/car-with-pxvehicledrive4w-no-suspension-and-no-grip/28494)
- Search summary of the PhysX docs: suspension forces go to the chassis, and applying an opposite force to a hit dynamic actor is "not covered". One developer applied the suspension force along the ray-hit normal to the hit body himself ("works well enough") — [gamedev.net raycast vehicle thread](https://gamedev.net/forums/topic/685988-raycast-vehicle-physics-problem)
- In the POLAR-Sim VIPER sim, wheels in mesh contact with rocks too large to climb slip around them — [arXiv 2309.12397](https://arxiv.org/pdf/2309.12397)

### Inferences
- **Design pattern for this project:**
  1. Big static boulders are drivable (in the GroundQuery-hit set) so a wheel can climb onto them. They also collide with chassis shapes, so underbody strikes and high-centering come from ordinary rigid contact. Add a coarse underbody collider (a box or convex) at real clearance height.
  2. Dynamic 1-50 kg rocks: take them out of the query filter for small rocks. Otherwise the wheel "stands" on a stone that feels nothing, and the stone acts as an immovable step that never rolls.
     - Give the wheels thin collision shapes (cylinders or capsules in the Wheel group) that collide with the Rock group but not the ground. Contact then shoves and rolls stones, and the stone pushes back on the chassis through the wheel shape.
     - Or keep them drivable and, every step, apply −(suspension force) at the hit point to the hit actor. The query state reports the hit actor, unless a cached plane is used.
  3. Medium rocks (≥20-50 kg, h ≳ 0.3 r) can be drivable and dynamic, with the reaction force added. They then tip, roll and settle under the wheel.
- Use sweeps, not raycasts, for ego wheels in rock fields. A raycast misses a rock struck by the tire's front face, which then hits the wheel shape or chassis abruptly. The sweep's cost matters only per wheel (4 queries per car), which is negligible next to body counts.

### Gaps
- I could not confirm from PhysX 5 docs whether `PxVehiclePhysXRoadGeometryQueryState` exposes the hit actor/shape in 5.x, or whether any 5.x component applies reaction forces to dynamic hit actors. Read the PhysX 5 vehicle2 headers to check.
- I found no Omniverse/Isaac sample showing a vehicle kicking loose rigid stones.

## 4. Procedural placement: rock size distributions and road placement

### Takeaway
The standard planetary model (Golombek & Rapp 1997) gives the cumulative fractional area covered by rocks of diameter ≥ D as F_k(D) = k·exp(−q(k)·D). Here k is the total rock-covered area fraction, and q(k) = 1.79 + 0.152/k is the form used in later Mars work. Robotics simulators mostly scatter rocks with Poisson-disk sampling on noise terrain. I found no sourced road-specific (edge or rut) placement rules.

### Cited Findings
- Golombek & Rapp (1997): rock size-frequency at the Viking sites and Earth analog sites "all have the general shape of simple exponential curves". The pre-exponential is the total area covered by rocks — [EarthRef ERR 60872](https://earthref.org/ERR/60872/)
- Model as restated by Golombek et al. 2021 (InSight): F_k(D) = k·exp[−q(k)·D], where F_k is the cumulative fractional area covered by rocks of diameter ≥ D, k the total fraction covered, and q(k) how fast the coverage drops with D — [Golombek et al. 2021, Earth and Space Science (Caltech PDF)](https://authors.library.caltech.edu/records/d1t32-akr32/files/Earth_and_Space_Science_-_2021_-_Golombek_-_Rock_Size_u2010Frequency_Distributions_at_the_InSight_Landing_Site_Mars.pdf?download=1); [DOAJ](https://doaj.org/article/98ae1be227d846ee84e6d8001674918b)
- q(k) = 1.79 + 0.152/k (D in metres) comes from a 2003 Golombek et al. rock-distribution paper for MER landing safety, not verified against the 1997 constants — [NTRS 20030111177](https://ntrs.nasa.gov/citations/20030111177); [Rock size-frequency distributions on Mars ... MER (PSU)](https://pure.psu.edu/en/publications/rock-size-frequency-distributions-on-mars-and-implications-for-ma/)
- Sim2Dust / Space Robotics Bench: Perlin-noise base mesh plus Poisson-disk scattering of procedural boulders and rocks, with domain randomization of physical parameters — [arXiv 2508.11503](https://arxiv.org/pdf/2508.11503)
- POLAR-Sim builds a Chrono twin from NASA POLAR terrain and rock meshes — [arXiv 2309.12397](https://arxiv.org/pdf/2309.12397)
- Rocky (GitHub) is a procedural rock and boulder generator whose placement metadata includes alignment to the terrain normal and tilt limits — [Gabryss/Rocky](https://github.com/Gabryss/Rocky)
- OmniLRS is an Isaac-based lunar simulator with procedural environment generation — [arXiv 2309.08997](https://arxiv.org/pdf/2309.08997.pdf)

### Inferences
- **Converting to counts:**
  - The number of rocks of diameter in [D, D+dD] per m² is −dF/dD / (π D²/4) = k·q·exp(−qD)/(π D²/4).
  - With k = 0.05 (5 % rock-covered, a moderate Mars site), q ≈ 1.79 + 3.04 = 4.83 /m. The fraction of area covered by rocks ≥ 0.3 m is then 0.05·e^(−1.45) ≈ 1.2 %.
  - Integrate per size bin to get counts per km of a 6 m wide road plus shoulders.
  - Small sizes dominate counts, so truncate below the gravel cutoff (section 1) and represent those as the surface.
- **Roads:** the rock-covered fraction k should be low in the travelled lane, higher on the shoulders and verge, and highest on rally stage edges and in water-cut ruts. Apply a lateral k(y) profile and Poisson-disk sampling with a minimum spacing scaled to D, then align rocks to the terrain normal and embed static boulders 20-40 % of their height.
- Deterministic seeding per road tile lets the activation scheme (section 6) recreate the same rocks when a tile streams in again.

### Gaps
- I did not obtain the full text of Golombek & Rapp 1997, so the original constants and the Earth analog k values are unverified.
- I found no source on Fuller-style gradations for road gravel in simulation, or on measured rock statistics on rally or gravel roads.
- I did not research JPL ROAMS, ARL/MIT off-road sims, Clearpath, RELLIS-3D/TartanDrive, Duality Falcon or CARLA off-road forks for rock handling. No sourced claims are available for them here.

## 5. Game practice (BeamNG, SnowRunner, Dirt Rally/WRC, Forza, GT, Spintires, Arma)

### Takeaway
The little sourced evidence points the same way: rocks are mostly static. Dynamic props are deliberately few for performance (BeamNG). Thrown stones are audio and particle effects, not physics (Dirt Rally). I found no sourced detail on rock-induced punctures or tire damage.

### Cited Findings
- BeamNG intentionally reduced dynamic objects in official stages for performance (team programmer, forum). Light props often lack collision. Soft-body rocks feel "unrealistically bouncy" (community) — [BeamNG threads](https://beamng.com/threads/dynamic-hard-body-physics-for-props.91433); [BeamNG posts](https://beamng.com/posts/1866850)
- Dirt Rally (Codemasters, Ego Engine 4.0): the physics were rebuilt with a new tire and surface model, and the sound team recorded gravel kick-up — [Wikipedia: Dirt Rally](https://en.wikipedia.org/wiki/Dirt_Rally)
- Reviewers report stones audibly pinging off the body — [PC Gamer review](https://www.pcgamer.com/au/dirt-rally-review-early-access); [Richer Sounds review](https://blog.richersounds.com/game-review-dirt-rally/)
- A Particles+ mod increases particles across the game's "3,000+ particle emitters" for skids and collisions, which shows that debris is particle-based — [Overtake: Particles+](https://www.overtake.gg/downloads/particles.8651/)

### Inferences
- Copy the game split: stone throw-up is a GPU particle effect driven by the wheel's slip speed and surface type (gravel), plus audio. Physics rocks are reserved for the few that should move the car or be moved.

### Gaps
- I found no sourced material on SnowRunner/Spintires, Forza Horizon, Gran Turismo rally, Arma or WRC (KT Racing / Codemasters EA WRC) rock and debris handling or puncture models. Searches returned only reviews.

## 6. Activation regions, physics LOD, and granular gravel (DEM/particles) vs surface models

### Takeaway
Wake or spawn dynamic rocks only near vehicles, and keep distant ones asleep or static. Persist their sleep state across streaming so loading does not wake everything. Full DEM gravel is orders of magnitude too costly for real-time km-scale driving: about 10^7 grains for 3 s of dynamics took up to 7 hours on two A100s in one engine. PhysX particles and deformables are "considerably more expensive than rigid bodies". So gravel should be a surface model plus cosmetic particles.

### Cited Findings
- **Streaming:** saving each object's sleep flag with its chunk and forcing it back to sleep on load stopped streamed-in objects from all waking — [Township Tale: Streaming a Physical World](https://townshiptale.com/news/streaming-a-physical-world)
- **Grid-cell deactivation:** disabling bodies in idle grid cells (with a margin) is used in Unity DOTS physics, but waking them correctly is "tricky" — [Unity discussions](https://discussions.unity.com/t/unity-physics-broadphase-narrowphase-jacobians-and-solving-systems-query-all-entities-with-colliders-no-way-around-it/1540540)
- Jolt defaults to manual wake control, so streaming does not disturb active bodies. The source is a low-reliability blog — [brightcoding blog](https://converter.brightcoding.dev/blog/joltphysics-why-aaa-studios-are-ditching-legacy-engines)
- Sleeping bodies leave the solver — [Box2D islands](https://box2d.org/posts/2023/10/simulation-islands/)
- **Chrono::GPU (DEM):** linear scaling up to about 130 million frictional particles (GPU-memory bound), and triangle-mesh co-simulation with wheeled and tracked vehicles — [SBEL: Chrono::GPU](https://sbel.wisc.edu/2020/09/23/chronogranular-modeling-and-simulation-of-granular-dynamics-using-gpu-computing/); [NSF PAR](https://par.nsf.gov/biblio/10349475-chrono-gpu-open-source-simulation-package-granular-dynamics-using-discrete-element-method)
- **DEM cost:** "simulating three seconds of dynamics for a problem involving 10 million grains can take up to seven hours on two NVIDIA A100 GPUs" (DEME engine). Chrono::CRM (SPH) reaches "near interactive rates" on terrain up to 10 km long with 100 million SPH particles. SCM is about an order of magnitude faster than CRM and is "widely used ... when simulation speed is important" — [Chrono::CRM, arXiv 2507.05643](https://arxiv.org/html/2507.05643v2)
- "Deformables and particles are considerably more expensive than rigid bodies" in omni.physx — [Omni Physics performance guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/guides/physics-performance.html)
- Sim2Dust trains rover control on granular media in Isaac Lab with procedural rocks. It is evidence that granular plus rocks is used for small robots, not km-scale cars — [arXiv 2508.11503](https://arxiv.org/pdf/2508.11503)

### Inferences
- **Activation scheme for this project:**
  - Rocks live as deterministic per-tile records (seed, prototype, pose, mass).
  - Within a radius of each vehicle, sized as speed × lookahead (e.g. 30 m/s × 2 s + 20 m ≈ 80 m), the dynamic class is instantiated as pooled convex bodies, created asleep.
  - Outside the radius, a rock that has come to rest is despawned and its moved pose written back to the tile record, so pushed rocks persist.
  - Static boulders stay permanently in the scene as static shapes. Static shapes cost only broadphase and memory, so a km-scale count of a few 10^4 is plausible within GPU buffers. That is unmeasured, so verify with `PxSimulationStatistics`.
  - NPC cars that SSv2 puppeteers (via `set_transform`, so kinematic-like) would crush dynamic stones. Filter them out of the Rock group, or accept that.
- **Gravel:** a friction and rolling-resistance coefficient per surface material, where the PhysX vehicle takes friction per wheel from the hit material. Add optional extra slip or noise and particle stone throw. DEM or PBD gravel is worth it only for a short local study (a single-wheel test bed, under 10^5 grains), never along the route.

### Gaps
- I found no published "physics LOD" numbers from shipping racing or open-world games (activation radii, awake-body budgets).
- I found no measured cost of PhysX 5 PBD particles used as gravel under a vehicle in Omniverse.
- I found no real-time factor for Chrono SCM with a full car and rigid rocks.
