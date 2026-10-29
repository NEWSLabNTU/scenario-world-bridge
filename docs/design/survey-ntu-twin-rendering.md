# NTU campus MLS scan to an Isaac Sim world that cameras and lidar can drive in

Research note, 2026-10-29. This is web research only. Nothing here was measured on our host. Each claim is marked:

- **[V]**: verified in a primary source (NVIDIA docs, release notes, paper abstract or README) read for this note.
- **[C]**: claimed by a secondary source (a blog, forum user or third-party article), or by a paper about its own method.
- **[E]**: our own estimate or inference.

The question: turn a 5.5 G-point coloured MLS scan of NTU (1113 x 649 m, LiDAR360MLS, with trajectory and IMU, and with the 2231 posed images not yet in hand) into an Isaac Sim 6.x world. The ego's cameras must be realistic enough to evaluate camera-only end-to-end stacks and VLM agents in closed loop. The ego's RTX lidar must be good enough for Autoware's NDT on the PCD.

---

## 0. Bottom line

1. **Gaussians in Isaac Sim are now a first-class camera primitive, and still not a dependable lidar primitive.**
   - Isaac Sim 6.0 renders OpenUSD `ParticleField3DGaussianSplat` natively in RTX. Splats receive shadows from meshes and can be set to cast them. NVIDIA steers new assets to ParticleField and away from the older NuRec volume format. [V]/[C]
   - No NVIDIA document says that RTX lidar intersects ParticleField. A GitHub discussion asking exactly that (Aug 2026) was closed unanswered. Third-party reports contradict each other: "not detected" (Oct 2025) against "returns, plus phantom hits from floaters, uniform reflectivity" (Sep 2026, a LinkedIn post relayed second-hand).
   - Plan on lidar seeing **meshes**. Treat gaussians as camera-only until we test it ourselves.
2. **NuRec, NVIDIA's reconstruction service, has matured since the 2026-10-10 survey.** Re-check the "too heavy" verdict.
   - The 26.04 and 26.06 releases render **lidar** from the gaussians through their gRPC renderer (GSplat). 26.06 adds joint camera and lidar training. [V]
   - Its stated floor is a GPU with more than 24 GB, recommending more than 48 GB. Blackwell needs driver R580 or later. The RTX 5090 is not on the supported list (the closest entry is the RTX Pro 6000D). [V]
   - Input must be NCore v3/v4, built for **vehicle rigs**: pinhole, fisheye or f-theta cameras plus structured spinning lidar. A backpack's equirectangular 360 images and a non-structured MLS cloud do not fit without conversion work. [V]/[E]
   - Verdict: a single 5090 is **at the floor** for training and is not a tested configuration. It is not ruled out, but it is a risk, not a plan.
3. **Without images, the mesh route still wins.** A textured mesh (a ground heightfield with a 1-2 cm orthophoto, per-class building meshes, and tree proxies) is the one representation both RTX camera and RTX lidar see reliably, with real materials for lidar intensity.
   - Camera realism is "good game engine": fine for lane and object perception. Point-coloured surfaces are visibly blurry and the vegetation is blobby, so a VLM or end-to-end net will notice the domain gap.
4. **With the vendor images, add gaussians for the camera only.** Use a hybrid: gaussians for camera appearance, mesh for lidar, collision and shadows.
   - Every reconstructed-scene closed-loop system of 2025-2026 does this or renders lidar from gaussians outside the engine: HUGSIM, NAVSIM v2, RAD, AlpaSim/NuRec.
   - Off-trajectory quality, at a 3 m lane shift and beyond, is the known weak point. It needs a diffusion "fixer" (Difix3D+/NuRec Harmonizer, GenRe, StreetCrafter).

---

## 1. Isaac Sim 6.x and NuRec today

### 1.1 Rendering gaussians for cameras

**Two formats in 6.0.** These come from a forum thread (June 2026) that quotes the 3DGRUT export README. [C, README quote relayed by a forum user]

| Format | 3DGRUT export flag | USD type | Reported trade-off |
|---|---|---|---|
| NuRec | `--format nurec` | `OmniNuRecFieldAsset` volume | Higher fidelity, weaker integration with RTX |
| ParticleField | `--format lightfield` | `UsdVol.ParticleField3DGaussianSplat` | Better shadows and interaction; more flicker and noise |

- The README says: "NuRec is going to be deprecated and replaced by `ParticleField`. Prefer `ParticleField` for new assets."
- Isaac Sim's own NuRec page publishes environments as ParticleField USD (`stage_particle.usdz`), trained with the open-source **3DGRUT**. [V, docs 6.0.0]

**Omniverse RTX docs (Particle Fields page).** [V]

- Fields render natively in path tracing, with every camera model, defocus and motion blur, and they appear in reflections and refraction.
- They **receive** shadows from meshes, which look "blobby". They **do not cast** shadows unless "Casts Shadows" is checked.
- **No relighting**: light intensity does not brighten splats. Brightness is a material knob, `emission_intensity`.
- Radiance is treated as display-referred sRGB, so **tonemapping is skipped**.
- Default accumulation budget: 48 gaussian hits per ray.
- DLSS Frame Generation causes artefacts.
- A forum thread reports that camera ISO and exposure change the meshes but not the splats. [C]
- Consequence for us: an inserted ego or NPC mesh is lit by RTX lights, while the background has its lighting baked in. Exposure and colour must be matched by hand (a dome light and sun matched to capture time, and a tuned `emission_intensity`). [E]

**Version changes.**

- **6.0.0** treats a stage with ParticleField prims as NuRec and forces replay to RGB only. [V, release notes]
- **6.0.1** adds `isaacsim.replicator.nurec_utils`. [V]
- **6.2.0** (Oct 2026) enables accumulated depth and albedo for NuRec scenes, capped at 360 gaussians. Replay honours RGB and depth. [C, radiancefields.com summary of the release notes]
- **Known issues** (HF NuRec dataset card, 6.1): dark patches and speckles from an RTX sorting issue. The workaround is sorting mode `rayHitDistance`. [C]
- **Inserting assets** into a `.usdz` root stage leaves empty red prims. Use a `.usd`/`.usda` root that references the usdz. [V]

**Performance.** Nobody publishes FPS or VRAM figures for large ParticleField scenes in Isaac Sim. [V, absence]

- 3DGUT's paper reports about 265 FPS for 3DGUT against about 52 FPS for 3DGRT on Mip-NeRF360-scale scenes. [C]
- For a campus: 3DGS with SH degree 3 costs about 59 floats (≈236 B) per gaussian. A street-scene reconstruction typically holds 1-5 M gaussians per 100 m block, so a 0.72 km² campus is roughly 30-100 M gaussians, or 7-24 GB. That is too much for one stage next to the ego stack on a shared 32 GB card. [E]
- So: tile the field (for example 100 m tiles) and load only tiles within about 150-250 m of the ego. Reduce SH degree, or use NuRec's "SH-optimized" export (26.06). [E]

### 1.2 Does RTX lidar see gaussians?

**Not documented.** [V, absence]

- The 6.0.1 RTX Lidar page describes `OmniLidar` prims and material-driven returns on USD prims. It never mentions ParticleField.
- GitHub isaac-sim/IsaacSim discussion #798 (26 Aug 2026) asks exactly whether RTX lidar and radar intersect ParticleField. An NVIDIA collaborator redirected it to the NuRec forum, and it was closed unanswered. [V]

**Contradictory field reports.**

| Date | Source | Claim |
|---|---|---|
| Oct 2025 | Evercoast | "Splats not detected by LIDAR"; no shadows, no reflections (older version, unnamed) [C] |
| Sep 2026 | LinkedIn post, relayed in a GitHub issue | RTX lidar returns on splats; low-opacity floaters produce phantom hits; every splat has the same reflectivity, because splats carry no non-visual material [C] |

**Consequences.**

- Even where lidar does hit splats, intensity is meaningless and floaters inject noise. For NDT against the vendor PCD, a mesh (or the original points) is the right lidar target. [E]
- Isaac Sim 6.0 has no proxy-mesh mechanism specific to gaussians. Its sample scripts only add an invisible collision ground plane. We provide the collision and lidar mesh ourselves, as an invisible-to-camera mesh that the lidar does see. Check that RTX lidar honours camera-visibility flags, or use render-layer purposes. [V for the sample scripts; E for the rest]

### 1.3 NuRec reconstruction: what it takes

**Containers.** [V]

- `nre-tools` handles preprocessing, depth and segmentation auxiliaries.
- `nre-ga` does the training. Version 26.04.01 is on NGC; the docs reach 26.06 (Jul 2026).

**Requirements.** [V]

- Linux x86_64, CUDA ≥ 12.8, Docker ≥ 23.0.1, Container Toolkit ≥ 1.13.5.
- More than 24 GB GPU memory, recommended more than 48 GB. Ampere, Ada, Hopper or Blackwell; Blackwell needs R580 or later.
- The supported list names A100, A40, A6000, L40(S), H100 and RTX Pro 6000D. No GeForce card is listed.
- Separately, the Harmonizer NIM's smallest qualified GPU is the 24 GB RTX 4090; it needs driver ≥ 595.58 and 32 GiB of host RAM. [V]
- NVIDIA's own nurec-skills repo calls 24-48 GB "recommended" for training. [C]

**Input.** [V]

- **NCore v3/v4** (json manifest plus `.zarr.itar`).
- Cameras: pinhole, fisheye or f-theta.
- Lidar: "structured spinning lidar (e.g. Hesai Pandar, AT128)", selected by `camera_ids` and `lidar_ids`. Training uses one GPU unless `world_size` is set; the default is 30 epochs.
- NVIDIA ships a "Physical AI Data Converter" for NCore.

**Our gaps for NCore.** [E]

- Equirectangular 5.7K panoramas need resampling into a virtual pinhole or f-theta rig.
- Per-sweep structured lidar packets with timestamps: we have the georeferenced cloud plus GPS time, so per-scanner sweeps would have to be re-sliced from GPS time and the trajectory.
- Camera-lidar extrinsics and timestamps.
- 26.06 also improves **lidar-free** training (ground-mesh initialization), so a camera-only NuRec run, with the MLS cloud as a prior, is an option. [V/E]

**Output.** [V]

- `last.usdz` holds the gaussians, an XODR drivable map, tracks and the rig trajectory.
- 26.06 adds USD export of novel `rig.json` cameras and "SPG-runtime" and "SH-optimized" PPISP integration modes. No mesh export is documented.

**Rendering lidar.** [V]

- The NuRec gRPC renderer gained lidar rendering in 26.04, one lidar per artifact.
- 26.06 fixed a zero-ray bug on **novel poses**.
- Validation metrics include distance MAE and intensity MAE (25.09).
- This runs **outside** Isaac's RTX lidar. Using it means a sidecar renderer driven by our ego pose, not an `OmniLidar`.

**Off-trajectory repair.** "Harmonizer" (formerly Difix) fixes artefacts at novel views. Its multi-GPU training path has a device-mismatch bug; single-GPU use is the workaround. [V]

**Verdict for 2 x RTX 5090 (32 GB).**

- **Training**: above the 24 GB floor, unsupported, and with a 0.72 km² scene it must be split into segments (NuRec is designed for roughly 20 s AV clips). [E]
- **Rendering in Isaac**: feasible if tiled. [E]
- **Real-time gRPC rendering beside Isaac on the same card**: tight. [E]
- The earlier "too heavy" verdict is **softened, not reversed**. NuRec is now plausible for a few corridors (r01 and r02), not yet for the whole campus.

---

## 2. Without images: from the coloured point cloud only

**Inputs.** 5.5 G points with RGB, class and GPS time, plus the trajectory. Colour resolution is bounded by point spacing and the vendor's colourization from the panoramas: no better than about 1-3 cm per point on the ground near the path, and much worse on facades away from it. [E]

**Point-to-mesh, per class. This is the recommended base.**

| Class | Share | Method |
|---|---|---|
| Ground | 5 % (plus misclassified unclassified points) | Re-classify with PDAL SMRF/CSF. Build a **heightfield** (DTM at 5-10 cm) and an **orthophoto** from point RGB at 1-2 cm, in UDIM or virtual-texture tiles: about 1.8 G texels at 2 cm, in tiles [E]. This is the surface cameras look at most, and an orthophoto from dense MLS points is the most realistic thing available. Lane paint comes out if the RGB, or better the intensity, is sharp enough. |
| Buildings | 4 % | Screened Poisson (Kazhdan) or TSDF per tile, decimated, vertex-coloured or baked to texture. Facades are smeared where the scan is sparse. |
| High vegetation | 46 % | The hard case. Meshing gives blobs. Options: (a) replace trees by instance with library assets (SimReady or SpeedTree) placed at detected trunks; (b) point sprites or point-initialized gaussians for the camera, plus a coarse alpha-wrapped proxy mesh for lidar and collision. |
| Unclassified | 40 % | Must be classified first (poles, signs, parked cars, bikes, fences, people). Tools like PTv3 or KPConv-family segmenters, or vendor reclassification. Parked vehicles and people in the scan must be **removed**: they are frozen ghosts to both sensors. |

- Precedent: a TU Wien thesis (2024) builds CARLA digital twins from point clouds and images "in hours with <20 min user intervention", with road layout from OSM. A 2025 TKTP paper compares CloudCompare, MeshLab and Metashape for MLS-to-mesh. [C]
- **Ground truth is ours**: Lanelet2 and xodr already exist for r01 and r02 and must be registered to EPSG:3826 together with the mesh (NDT uses the same PCD).

**Point-initialized gaussians without training.** `ply-to-splat` turns coloured points straight into a splat ply: isotropic, sized by local spacing, opacity 1, colour as SH0. It is for visualization only. [C]

- In Isaac this gives a dense "point cloud look". It beats a Poisson mesh on vegetation and loses to it on flat ground (no view-dependence, holes at low density). [E]
- Training gaussians against **lidar alone** (GS-LiDAR, LiDAR-GS, SplatAD's lidar branch) improves geometry and intensity, not camera appearance. [C]

**Neural point rendering** (ADOP/TRIPS-style). Good quality, but it needs images for training and has no Isaac/RTX integration. Not recommended. [E]

**Camera realism on the mesh route.**

- Expect CARLA-like or better geometry with realistic textures on the ground, and visibly low-frequency facades and trees.
- Lighting is baked into the vertex colours (shadows from capture time), then RTX shades it again. Mitigate with de-lighting: an albedo estimate, or simply lower contrast and an overcast sky. [E]
- Good enough for lane keeping, ACC and AEB testing with a monocular stack.
- A VLM will likely describe the scene as "simulated". Closing that gap is what the images (section 3) or a generative pass (section 4, Cosmos) buy.

**Lidar realism on the mesh route.**

- Geometry matches the real campus by construction. NDT should localize against the vendor PCD as well as on the real car, apart from vegetation sway and the dynamic objects we remove. [E]
- Intensity comes from SimReady non-visual materials per prim (section 5): asphalt, paint, concrete, metal, foliage. Assign by class.
- Real lane-paint reflectivity needs paint as **separate geometry or prims**, segmented from intensity or RGB. Per-texel reflectivity is undocumented. [E]
- Ray drop and noise: not documented on the RTX Lidar page. Add them in post-processing if needed. [V absence/E]

---

## 3. With the vendor images

**What the images probably are.** GreenValley's LiBackpack DGC50H lists a 5.7K panoramic camera, 5760x2880, captured as time-lapse at 2 s per frame. [V, datasheet] So the 2231 images with 6-DoF poses are likely equirectangular panoramas every 2 s, about 74 min of capture. [E]

- On foot that is one panorama every 2-3 m: good coverage.
- On a vehicle at 10 m/s it is every 20 m: sparse for gaussians, and needing lidar depth supervision.
- Ask the vendor for:
  - the raw (unstitched, if possible) images;
  - per-image timestamps, exposure and white balance;
  - camera-lidar calibration;
  - per-scanner raw sweeps, needed for NCore-style lidar.

**Methods.**

| Method | Status | Camera | Lidar render | Dynamic actors | Engine export | Notes |
|---|---|---|---|---|---|---|
| **NuRec** (NVIDIA, 26.06) | product, GA containers | yes, plus Harmonizer fixer | yes (gRPC, 1 lidar) | dynamic rigids layer | USDZ / ParticleField for Isaac | ≥ 24 GB, NCore input [V] |
| **3DGRUT / 3DGUT** (NVIDIA, CVPR'25) | open source | yes, any camera model, rolling shutter | via SimULi / XSIM extensions (2025-26) [C] | no | ParticleField / NuRec USD export | The route Isaac documents [V] |
| **gsplat** (nerfstudio) | open source | yes | spinning-lidar rasterization added Mar 2026 (mirror changelog) [C] | no | ply, then 3DGRUT converter | The library NuRec now uses [V] |
| **SplatAD** (CVPR'25) | open source | yes | yes: intensity, ray-drop, rolling shutter [C] | yes | ply | Best published camera plus lidar realism |
| **Street Gaussians**, **OmniRe** (ICLR'25, drivestudio), **PVG** | open source | yes | depth only | yes | ply | About 30 min per scene, 135 FPS (Street Gaussians) [C] |
| **HUGSIM** (Dec 2024) | open source | RGB, depth, semantics | no | inserted 3DGS vehicles | own renderer | Closed-loop benchmark, 70+ sequences, 400+ scenarios [V abstract] |

**Off-trajectory quality.** All of these degrade at a lane shift, where unseen surfaces and floaters appear. Fixes: [C]

- diffusion fixers: Difix3D+/Harmonizer, GenRe (Waabi, 3 m shifts with an SD-Turbo fixer), LidarPainter;
- conditioning on lidar renders: StreetCrafter;
- progressive novel-view training: ReconDreamer.
- NAVSIM v2 / SimScale discard reconstructions with novel-view PSNR < 27. [C]

**For a campus.** Our capture covers each road once or twice. Expect good views within ±1-2 m of the scan path and visible artefacts at ±3 m and beyond. Panoramas help, because every direction is seen. A dense MLS cloud as initialization plus a depth loss helps more than it does for car datasets. [E]

---

## 4. Closed-loop evaluation systems in reconstructed scenes (2025-2026)

**HUGSIM** (arXiv 2412.01718)

- 3DGS lifted from RGB, real time, Gymnasium API.
- Actors are inserted as 3DGS vehicle assets with normal or aggressive behaviour models.
- Metric: **HD-Score**. Per step, NC × DAC times a weighted average of TTC and comfort; averaged, then multiplied by route completion. The benchmark is now 436 scenarios. [V/C]

**NAVSIM v2 pseudo-simulation** (arXiv 2506.04218, CoRL 2025)

- Two stages: real log frames, then views pre-rendered by 3DGS from perturbed start states.
- Metric: EPDMS. Pseudo-sim correlates R² = 0.8 with full closed loop over 83 planners (against 0.7 for the best open-loop metric).
- The leaderboard split is `navhard`.
- It is **not** reactive closed loop; it is cheap. [V]

**RAD** (NeurIPS 2025, HUST and Horizon)

- 3DGS environments used for **RL training** of an end-to-end policy, plus a 3DGS closed-loop benchmark.
- Reports 3x lower collision rate than imitation learning. [V abstract]

**Bench2Drive-R** (arXiv 2412.09647)

- A *generative* renderer: a diffusion model with retrieval of the nearest real images, plus a nuPlan reactive behaviour controller.
- Not real time. [C]

**AlpaSim** (NVlabs, Apache 2.0)

- gRPC microservices with a pluggable renderer: NuRec by default, or OmniDreams video-model rendering via FlashDreams.
- Traffic sim, controller and evaluation services.
- The README describes **cameras only**. [V]

**OmniDreams** (arXiv 2606.03159, v3 Sep 2026)

- A Cosmos-based action-conditioned video world model.
- "Real time", closed loop with Alpamayo-1 in AlpaSim, with no reconstruction needed. Hardware is not stated. [V abstract]

**Cosmos-Transfer 2.5**

- Sim-to-real video restyling.
- The 2B model needs 65.4 GB for single-GPU inference, about 92 s per 93 frames on a B200. [V]
- **Offline only for us**: useful for producing realistic datasets from Isaac runs, not for closed loop.

**CARLA 0.9.16** ships a beta NuRec integration: a gRPC container renders camera frames for CARLA-driven actors. [V]

**Shared pattern.**

- The background is a reconstruction. Actors are inserted as separate assets: 3DGS cars, or meshes rendered by the engine.
- Behaviour comes from a separate controller (nuPlan- or Trafficsim-style).
- Metrics are PDMS-family products of safety multipliers and progress: NC, DAC, TTC, comfort, route completion.
- Lidar is rarely in the loop. This matches our split: SSv2 drives the actors, Autoware's lidar sees meshes.

**"VisionPilot"** is the Autoware Foundation's **Vision Pilot** (`autowarefoundation/autoware.privately-owned-vehicles`, also `hakuturu583/vision_pilot`). [V README]

- An Apache-2.0 L2 stack on a **single front monocular camera**: 52-55° HFOV, 1-2 MP.
- Mapless, with no HD map.
- Models: AutoSpeed (closest in-path object), AutoSteer (path waypoints) and AutoDrive (end-to-end distance and curvature).
- Closed-loop testing on **CARLA 0.9.16**; ROS 2 input optional.
- Implication: one well-rendered front camera matters more than a surround rig. Lane markings and the in-path vehicle must be crisp, which favours the orthophoto ground and mesh NPCs.

---

## 5. Lidar realism in Isaac Sim 6.x

**Models.** [V, 6.0.1 docs]

- `OmniLidar` prims with `OmniSensorGenericLidarCoreAPI`, created from named configs and variants such as Ouster OS1 REV6, HESAI XT32 and SICK picoScan.
- A custom sensor is authored as prim attributes: emitter pattern, scan rate and the like. A Velodyne VLP-16/32 or Hesai profile matching the real car can be authored or picked from the asset library.
- `tick_rate` must equal `scanRateBaseHz`, or scans come out partial.
- **Multi-GPU can crash with CUDA error 700. The workaround is disabling multi-GPU rendering.** This matters on our 2 x 5090 host: pin Isaac to one GPU.

**Intensity.** [V, 6.1/6.2 docs]

- Non-visual materials are USD attributes following the SimReady spec: `omni:simready:nonvisual:base`, `:coating` and `:attributes`.
- They are exposed as a material ID and intensity in `GenericModelOutput`. The example `apply_nonvisual_materials.py` shows intensity changing with material.
- The old `RtxSensorMaterialMap.csv` is **ignored since 5.1**.
- A forum report (6.0) says the materials did nothing until they were actually bound. Use the "Non-Visual Material ID" debug view. [C]
- The docs do not state the intensity model (BRDF or angle dependence), per-texel variation, ray drop or noise. [V absence] Calibrate empirically: drive the virtual car along the scan path and compare intensity histograms per class with the vendor LAS intensity.

**Matching NDT.** Autoware's NDT tolerates intensity errors (it uses geometry) but not missing geometry. The mesh must keep poles, kerbs and building edges, and trees should be close in volume. The PCD for Autoware should be the **same** cleaned cloud the mesh came from, voxelized as Autoware expects. [E]

---

## 6. Recommended pipeline

### Stage A: geometry twin, about 2-4 weeks of work, no images needed (fast first version)

1. **Tiling and cleaning.** PDAL or LAStools: tile into 100 m tiles in EPSG:3826, a local ENU origin, and a shared transform with the Lanelet2 map and PCD. Reclassify ground (SMRF/CSF), segment the unclassified points (PTv3 or KPConv, or a vendor rerun), and remove dynamic objects with GPS-time consistency.
   - Compute: CPU-bound. About 1-2 days of wall time on the 5.5 G points. [E]
2. **Ground.** A DTM heightfield mesh plus a 2 cm RGB orthophoto, and an intensity orthophoto used to extract lane paint as separate prims.
3. **Buildings and structures.** Poisson per tile, decimated to about 0.5-2 M triangles per tile, vertex colour baked to textures.
4. **Vegetation.** Trees replaced by instance with SimReady or other assets matched by height and crown, plus alpha-wrapped proxies for lidar. Alternative: point-initialized splats for the camera over the proxy mesh.
5. **USD assembly.** Tiled payloads, SimReady non-visual materials by class, collision on ground and buildings. Load on demand around the ego.
6. **Validation.**
   - NDT on the vendor PCD with the RTX lidar (scan-matching score, iteration count, against the real-car baseline from issue 017's metrics).
   - Camera: run Vision Pilot (CARLA-tested) and our VLM agent on r01 and r02.
   - Optionally compare against real dashcam footage with FID or LPIPS if we record some.

**Risks.**

- Vegetation looks fake; this hurts VLMs.
- Baked shadows double up with RTX shadows.
- The texture budget (about 1.8 G texels) needs streaming.
- Multi-GPU RTX lidar crash.

### Stage B: camera fidelity from the vendor images (needs the images; about 3-6 weeks)

1. Get the raw panoramas, timestamps, exposure and calibration. Convert them to 6 to 8 pinhole or f-theta virtual cameras per panorama. Mask the operator, vehicle and sky.
2. Train **3DGUT or gsplat** per 100-200 m segment along r01 and r02 first. Initialize from MLS points and supervise depth from the cloud.
   - Estimate: about 1-3 h per segment on one 5090 (32 GB is enough for a segment). The whole campus is about 50 segments, roughly 3-6 GPU-days. [E]
3. Export ParticleField USD (prefer it over NuRec format). Put it in the same tiled stage as the Stage A mesh. Make the mesh **camera-invisible but lidar-, collision- and shadow-visible**. Splats receive the ego and NPC shadows; let them cast shadows only if it looks right.
4. Match exposure: splats skip tonemapping, so set `emission_intensity` and the inserted actors' lighting to match. Check sorting mode and DLSS settings.
5. Measure off-trajectory quality: render ±1.5 m and ±3 m lateral shifts. Reject or fix tiles with PSNR or LPIPS below a threshold, following SimScale's PSNR < 27 rule.

**Risks.**

- Panoramas every 2 s may be sparse on vehicle legs.
- Stitching seams.
- Floaters, which show up as phantom lidar if lidar does see splats: keep the splats out of the lidar's view if possible, and test.
- ParticleField flicker.
- Isaac FPS with tens of millions of splats.

### Stage C: optional, higher fidelity and lidar-from-reconstruction

- **NuRec trial on one corridor.** Convert to NCore (virtual rig from the panoramas, lidar re-sliced into structured sweeps if the vendor provides raw scanner data), train on one 5090, and compare with Stage B.
  - If it trains, use the **Harmonizer** for lane-shift views. Optionally use the NuRec gRPC lidar render as an alternative lidar sidecar to compare against RTX-on-mesh.
  - Expect it to fail on VRAM or driver support. Budget 1 week to find out.
- **Cosmos-Transfer 2.5 offline**: restyle recorded Isaac camera streams for dataset and VLM evaluation in open loop. It needs about 65 GB, so it does not fit a 5090 unless distilled or multi-GPU.
- **Evaluation harness**: SSv2 scenarios on the campus (actors as mesh vehicles from our catalog, inserted by the bridge). Report HD-Score-style and EPDMS-style metrics (NC, DAC, TTC, comfort, route completion) next to SSv2's pass/fail, so results line up with HUGSIM and NAVSIM numbers.

### GPU budget on the shared host [E]

- Isaac world with Stage A mesh tiles: about 10-16 GB on one 5090. Adding Stage B splat tiles within a 250 m radius: +4-10 GB.
- Autoware (CenterPoint, NDT): about 4-6 GB.
- So run Isaac on GPU 0, and put Autoware and the VLM or E2E agent on GPU 1.
- Training (Stage B or C) needs a dedicated card, so it cannot run beside the live simulation.

---

## Sources

- Isaac Sim 6.0.1 release notes: https://docs.isaacsim.omniverse.nvidia.com/6.0.1/overview/release_notes.html
- Isaac Sim 6.0.0, Neural Volume Rendering (NuRec assets): https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_nurec.html
- Isaac Sim 6.0.1, NuRec utils: https://docs.isaacsim.omniverse.nvidia.com/6.0.1/assets/nurec_utils.html
- Isaac Sim 6.0.1, RTX Lidar: https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx_lidar.html
- Isaac Sim 6.2.0 / 6.1.0, RTX non-visual materials: https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_rtx_materials.html , https://docs.isaacsim.omniverse.nvidia.com/6.1.0/sensors/isaacsim_sensors_rtx_materials.html
- Forum, non-visual material in 6.0: https://forums.developer.nvidia.com/t/isaac-sim-6-0-what-are-the-functions-of-the-rtx-sensor-non-visual-material/374657
- Omniverse RTX, Gaussian Splats (Particle Fields): https://docs.omniverse.nvidia.com/materials-and-rendering/latest/particle-fields.html
- GitHub IsaacSim discussion #798 (lidar vs ParticleField, Aug 2026): https://github.com/isaac-sim/IsaacSim/discussions/798
- Forum, NuRec vs ParticleField (Jun 2026): https://forums.developer.nvidia.com/t/question-on-two-3dgs-rendering-pipelines-nurec-vs-particlefield-and-expected-quality-tradeoff-in-isaac-sim-6-0/373260
- Forum, ISO/exposure and splats: https://forums.developer.nvidia.com/t/camera-iso-exposure-affects-mesh-objects-but-not-3d-gaussian-splat-usd-in-isaac-sim/371844
- radiancefields.com, Isaac Sim 6.2.0 NuRec depth and albedo (Oct 2026): https://radiancefields.com/nvidia-isaac-sim-6-2-0-turns-on-gaussian-depth-and-albedo-for-nurec-scenes
- Evercoast, 4D splats in Omniverse and Isaac (Oct 2025): https://insights.evercoast.ai/p/integrating-4d-gaussian-splats-into
- GitHub issue relaying the RTX-lidar-on-splats claim (Sep 2026): https://github.com/freshNfunky/IE2025-Research-Paper/issues/23
- HF NuRec robotics dataset: https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-NuRec
- NuRec hardware (26.04): https://docs.nvidia.com/nurec/archives/26.04/basics/hardware.html
- NuRec, reconstruct an AV scene (26.04): https://docs.nvidia.com/nurec/archives/26.04/nurec/reconstruct-av-scene.html
- NuRec release notes (index, 26.04, 26.06): https://docs.nvidia.com/nurec/release-notes/index.html , https://docs.nvidia.com/nurec/release-notes/_includes/26-04.html , https://docs.nvidia.com/nurec/release-notes/_includes/26-06.html
- NGC nre-ga container: https://catalog.ngc.nvidia.com/orgs/nvidia/nre/containers/nre-ga/-?_lr=1 ; nre-tools: https://catalog.ngc.nvidia.com/orgs/nvidia/nre/containers/nre-tools-ga/-?_lr=1
- NVIDIA nurec-skills: https://github.com/NVIDIA/nurec-skills
- Harmonizer NIM prerequisites: https://docs.nvidia.com/nim/harmonizer/prerequisites
- CARLA 0.9.16 NuRec: https://carla.readthedocs.io/en/0.9.16/nvidia_nurec/ ; release: https://carla.org/2025/09/16/release-0.9.16
- AlpaSim: https://github.com/NVlabs/alpasim
- OmniDreams: https://arxiv.org/abs/2606.03159
- Cosmos-Transfer 2.5 quickstart: https://docs.nvidia.com/cosmos/latest/transfer2.5/quickstart_guide.html
- 3DGUT: https://research.nvidia.com/labs/toronto-ai/3DGUT/ ; 3DGRT: https://arxiv.org/abs/2407.07090
- SimULi: https://arxiv.org/html/2510.12901v1 ; XSIM: https://arxiv.org/abs/2602.05617
- SplatAD: https://arxiv.org/abs/2411.16816
- gsplat lidar changelog (mirror): https://gitee.com/kk357924266/gsplat
- Street Gaussians: https://arxiv.org/abs/2401.01339 ; OmniRe: https://ziyc.github.io/omnire
- HUGSIM: https://arxiv.org/abs/2412.01718
- NAVSIM v2 pseudo-simulation: https://arxiv.org/abs/2506.04218 ; SimScale: https://arxiv.org/abs/2511.23369
- RAD: https://arxiv.org/abs/2502.13144
- Bench2Drive-R: https://arxiv.org/abs/2412.09647
- GenRe (Waabi): https://waabi.ai/genre/ ; FaithFusion: https://arxiv.org/abs/2511.21113 ; LidarPainter: https://arxiv.org/abs/2507.12114
- ply-to-splat: https://github.com/PrintedForFun/ply-to-splat
- TU Wien thesis, CARLA content from point clouds: https://repositum.tuwien.at/handle/20.500.12708/205279 ; TKTP 2025 MLS-to-mesh: https://ceur-ws.org/Vol-4181/paper07.pdf
- GreenValley LiBackpack DGC50H datasheet: https://globalgpssystems.com/wp-content/uploads/2022/06/EN-Libackpack-DGC50H-20240729EN_compressed.pdf
- Autoware Vision Pilot: https://github.com/autowarefoundation/autoware.privately-owned-vehicles ; https://github.com/hakuturu583/vision_pilot
