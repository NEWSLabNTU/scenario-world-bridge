# Validating a simulated car on rough roads, and what roughness does to its sensors and Autoware

Scope: (a) published tests, standards and datasets a simulated car's response to rough roads, obstacles and jumps can be checked against; (b) what small-scale road features do to the lidar, cameras, IMU, NDT localization and ground segmentation, and how simulators (Isaac Sim 6.1 RTX in particular) model it. Contact models, roughness generation and rock physics are out of scope (other researchers cover them).

Research budget note: about 30 web searches/fetches. Many primary sources (SAE J27xx, ISO 2631-1, ISO 8608 text, the Schmeitz and Zegelaar theses, the force tables in SAE pothole papers) are paywalled or were not indexed, so several requested *numbers* are missing. They are listed under Gaps instead of being filled in from memory.

## Validation: standard tests and published measurements (cleat, drum, pothole, bump, washboard, jump, ISO 2631 / ISO 8608)

### Takeaway
The standard test methods exist and are well defined: SAE J2731 low-speed cleat enveloping, SAE J2730 dynamic cleat, ISO 2631-1 weighted RMS with its comfort bands, and IRI from the golden quarter-car. But the open web gives few reference numbers you could reuse directly. The usable figures are the ISO 2631-1 comfort bands, the IRI golden-car parameters, one SUV pothole static load (6.7 kN at 40 km/h, event under 0.5 s), washboard geometry (0.5-1 m pitch, up to 50 mm amplitude; one measured road at 20.6 mm peak to peak), and speed-bump test speeds (10-50 km/h). The force traces themselves (Zegelaar/Schmeitz cleats, pothole peaks) sit in paywalled sources.

### Cited Findings
**Tyre cleat / drum tests**
- SAE J2731, "Low Speed Enveloping Test with Perpendicular and Inclined Cleats": measures spindle forces and moments as a tyre rolls over a rectangular cleat at very low speed. The cleat crest is either perpendicular (90°) or inclined to the tyre path, and the output is envelopment force and moment against distance travelled. The abstract does not state the cleat size. — [SAE J2731](https://saemobilus.sae.org/content/j2731_201805); [DIN Media listing](https://www.dinmedia.de/en/standard/sae-j-2731/299592027)
- SAE J2730, "Dynamic Cleat Test with Perpendicular and Inclined Cleats", is the dynamic (higher-speed) counterpart. — [SAE J2730](https://saemobilus.sae.org/content/J2730_200608/)
- Zegelaar measured a 205/60R15 tyre over three short road unevennesses ("cleats") at three constant axle heights on a drum turning at 0.2 km/h, slow enough to remove dynamic effects. He recorded vertical force F_V, longitudinal force F_H and effective rolling radius r_e. This dataset is what the "basic function" enveloping approach is fitted to. — [Pacejka, *Tire and Vehicle Dynamics*, ch. 10 (Scribd copy)](https://www.scribd.com/document/535583154/70165-10)
- Schmeitz, A.J.C. (2004), *A Semi-Empirical Three-Dimensional Model of the Pneumatic Tyre Rolling over Arbitrarily Uneven Road Surfaces*, PhD thesis, TU Delft. It extends Zegelaar's 2D approach to 3D effective road surfaces combined with the rigid-ring dynamic tyre (the basis of MF-Swift's obstacle response). — [Altair reference list](https://help.altair.com/hwdesktop/hwx/topics/motionview/fiala_tire_references_r.htm); [TU/e record of the conference paper](https://research.tue.nl/en/publications/a-semi-empirical-three-dimensional-tyre-model-for-rolling-over-ar/)
- Schmeitz, Besselink, Jansen, "Application of a semi-empirical dynamic tyre model for rolling over arbitrary road profiles", *Int. J. Vehicle Design* 36 (2004) 194-215. This is the journal version with measured-versus-simulated cleat responses. — [TU Delft repository](https://repository.tudelft.nl/record/uuid:66810311-ed19-4c8e-93b8-816b77219c9e); [Inderscience](https://www.inderscience.com/info/inarticle.php?artid=5356)
- FTire cleat-impact correlation to experiment (SAE 2004-01-1575) is a further measured-cleat reference, behind a paywall. — [SAE 2004-01-1575](https://saemobilus.sae.org/content/2004-01-1575)

**Pothole impact**
- Michigan Scientific wheel-force-transducer example: an SUV's right front wheel hits a pothole at 40 km/h (25 mph). Static radial load is just under 6.7 kN (~1,500 lb). The vertical force drops as the wheel falls into the hole, and the largest peak comes when it strikes the far edge. The whole event lasts under 0.5 s. The longitudinal force spikes at the edge strike. The page gives no numeric peak. — [Michigan Scientific, durability campaigns](https://www.michsci-europe.com/durability-campaigns)
- Simulated pothole tests at 30/40/50 km/h: across that speed range, wheel-centre longitudinal force rose 5.4x while vertical force rose less than 2x. This is a simulation, not a measurement. — [Simulation of Vehicle Pothole Test and Techniques Used (ResearchGate)](https://www.researchgate.net/publication/280625539_Simulation_of_Vehicle_Pothole_Test_and_Techniques_Used)
- SAE 2007-01-1647 studies how braking, speed, front corner weight and suspension stiffness change pothole-edge suspension loads. Full text is paywalled. — [SAE 2007-01-1647](https://saemobilus.sae.org/papers/effects-braking-suspension-loads-potholes-2007-01-1647)
- Order of magnitude for obstacle strikes: a patent example gives 37.7 kN measured when a VW Golf 1.4T's left front wheel hit a standard convex obstacle block at 64 km/h. That is a raised obstacle, not a pothole. — [US patent 10386270 (PDF)](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10386270)
- Wheel force transducer ranges, for scale: MTS SWIFT Evo 40 is calibrated to 40 kN Fz/Fx, and the PM-Instrumentation LW12.8-50 (passenger car/SUV) to 50 kN. Rough-road and pothole events are instrumented to tens of kN. — [MTS SWIFT Evo](https://www.mts.com/en/products/test-system-components/swift-evo); [LW12.8-50](https://en.pm-instrumentation.com/lw128-50-6-axis-wheel-force-transducer-50-kn-passenger-car-and-suv)

**Speed bumps / single bumps**
- Skoda passenger car over a speed bump at 10, 20 and 30 km/h, with a half-car model validated against the test. Bump height is the key parameter. — [Haniszewski & Michta, Transport Problems 2019](https://bibliotekanauki.pl/articles/374733.pdf)
- VW Passat B5, body vertical acceleration at 20/30/40 km/h with two shock-absorber sets. — [Janczur, Cracow Univ. of Technology (PDF)](https://yadda.icm.edu.pl/baztech/element/bwmeta1.element.baztech-cc0e024a-c36d-4971-81f5-3ef555e124c9/c/vol67-janczur-en-47-60.pdf)
- Real speed bumps at 30/40/50 km/h, evaluated under ISO 2631-1: no occupant discomfort up to 67 km/h for that bump. — [ITU study on parabolic speed hump profiles](https://research.itu.edu.tr/en/publications/an-investigation-into-the-effect-of-parabolic-speed-hump-profiles/)
- Rectangular-bump ride test at 10 km/h compared with a model, reporting sprung-mass vertical accelerations. — [Cavacece et al., Ride test (PDF)](https://iris.unicas.it/retrieve/b5a32057-8d88-4f82-8e47-b7bee19df2d9/CavaceceRideTest.pdf)
- Multibody model against measured CG vertical acceleration over bumps (MBSVT). — [IMSD 2018 paper 101](https://imsd2018.ist.utl.pt/Web_Abstracts_IMSD2018/pdf/WEB_ABSTRACTS/IMSD2018_paper_101.pdf)
- Semi-trailer truck, for contrast (not a car): peak vertical acceleration 0.76 g at 5 km/h, 2.4 g at 10 km/h and 1.9 g at 15 km/h over transient events. — [Analysis of Transient-Like Vertical Acceleration Events on Semi-Trailer Truck (IOS Press)](https://ebooks.iospress.nl/doi/10.3233/ATDE240548)

**Washboard (corrugated gravel)**
- Dirt-road corrugations have a pitch of 0.5-1 m and amplitudes up to 50 mm. — [Mays/Taberlet washboard page](https://perso.ens-lyon.fr/nicolas.taberlet/washboard/); [Corrugation of Roads, arXiv cond-mat/0011510](https://ar5iv.labs.arxiv.org/html/cond-mat/0011510)
- One measured gravel road (Universidad de los Andes): mean peak-to-peak wave amplitude 20.6 mm over a 1.6 m profile, with vehicle responses compared across speeds. — [ISSMGE paper 1394 (PDF)](https://www.issmge.org/uploads/publications/1/141/1394.pdf)
- Washboard wavelength is not simply wheel speed times bounce frequency. — [Taberlet washboard page](https://perso.ens-lyon.fr/nicolas.taberlet/washboard/); also [Modeling Washboard Road, arXiv 1209.5560](https://ar5iv.arxiv.org/html/1209.5560)
- Vehicle vertical oscillation, excited by the initial surface irregularities, drives corrugation growth. — [arXiv 2007.05109](https://ar5iv.labs.arxiv.org/html/2007.05109)

**ISO 2631-1 whole-body vibration and IRI**
- ISO 2631-1 comfort bands (frequency-weighted RMS acceleration):
  - < 0.315 m/s²: not uncomfortable
  - 0.315-0.63: a little uncomfortable
  - 0.5-1.0: fairly uncomfortable
  - 0.8-1.6: uncomfortable
  - 1.25-2.5: very uncomfortable
  - > 2 (some sources > 2.5): extremely uncomfortable

  The bands overlap by design, and sources disagree on the top cutoff. — [Engineering Toolbox](https://www.engineeringtoolbox.com/people-vibration-d_1292.html); [PMC table](https://pmc.ncbi.nlm.nih.gov/articles/PMC8125624/table/sensors-21-03127-t003); [PMC table (variant)](https://pmc.ncbi.nlm.nih.gov/articles/PMC7904064/table/T3)
- ISO 2631 uses the RMS method when the crest factor is below 9. Above that, use the vibration dose value (VDV), which applies to potholes, rocks and landings. — [Scribd ride-comfort summary](https://www.scribd.com/document/385036592/ride-comfort-pdf)
- Granlund (IRI vs ISO 2631):
  - IRI is the golden quarter-car's accumulated suspension stroke over distance at 80 km/h, in mm/m.
  - Golden-car parameters, normalised to m_s = 1: c_s = 6 s⁻¹, k_t = 653 s⁻², k_s = 63.3 s⁻², m_u/m_s = 0.15.
  - For a 5 m sinusoid at 80 km/h, ISO-weighted floor vibration / IRI ≈ 0.16 (m/s² per mm/m).
  - Road PSD exponent n ≈ 2. 0.8 m/s² is taken as the "uncomfortable" limit.
  - Motion-sickness band 0.1-0.63 Hz is not covered.

  — [Granlund, IRI vs ISO2631 (SlideShare)](https://www.slideshare.net/slideshow/iri-vs-iso2631/34121158)
- Nine cars on ~1,860 km of public road: ISO-weighted seat and floor RMS in three axes. Whole-body vibration was ~20 % higher on motorways than on 1st/2nd-class roads of the same IRI class (paved roads only). — [Communications 2019 (DOAJ)](https://doaj.org/article/d8859c9d7f9743bbada2a0a5deb85d4b)
- VDV rises with speed and roughness for mid-size saloons on asphalt. — [Sinop Univ. repository](https://acikerisim.sinop.edu.tr/items/c79a7356-b5a7-4518-91e2-b9874bc1ee2f/full)

### Inferences
- A practical ride-validation chain for the Isaac Model 3 needs no paywalled data:
  1. Drive the synthetic gravel road at fixed speeds.
  2. Log floor (chassis IMU) vertical acceleration.
  3. Compute ISO 2631-1 W_k-weighted RMS, plus VDV when the crest factor is above 9.
  4. Compute IRI from the same road profile with the golden-car parameters above.

  Then check that the IRI-to-weighted-RMS ratio is near 0.16 per mm/m at 80 km/h, and that the weighted RMS falls in a plausible band for the stated road class. A bare gravel or forest road at 30-60 km/h landing far below 0.315 m/s² or far above 2.5 m/s² should be a red flag. This is my sanity band, not a published gravel figure.
- Pothole sanity check: in a 40 km/h edge strike, the vertical wheel load should first drop toward zero and then peak on the far edge within under 0.5 s, from a static load of the car's corner weight. For a Model 3 that is roughly 4.5-5 kN (my estimate from curb weight, not sourced). Edge-strike peaks in the tens of kN are the right order (cf. 37.7 kN at 64 km/h on a convex block). Peaks of a few kN, or a smooth trace without the drop, would indicate the contact or tyre model is filtering the edge.
- Washboard: at 0.5-1 m pitch, 30-60 km/h gives 8-33 Hz excitation. That is near or above the wheel-hop mode (~10-15 Hz). So washboard tests the unsprung dynamics and the physics step rate: at a 60 Hz physics step, 33 Hz input is barely sampled.

### Gaps
- No open numeric force traces from the Zegelaar or Schmeitz cleat measurements, and no confirmation of the "10x10 mm / 20x20 mm" cleat sizes. They are in Pacejka ch. 10 figures and the IJVD paper (paywalled). SAE J2731 does not publish its cleat sizes in the abstract.
- SAE J2733 could not be confirmed as a pothole standard. I found no source for it.
- No published ISO 2631 weighted-RMS value for a passenger car on gravel or unpaved roads was found. ISO 8608 class boundaries (the Gd(n0) values for classes A-H) were not retrieved in this pass.
- No published vehicle jump or crest landing measurements (vertical g at landing against airtime) for passenger cars were found.
- Peak pothole forces against speed and depth for passenger cars remain unquantified in open sources.

## Validation: open datasets of vehicles on rough/off-road terrain, and how to compare a sim run to them

### Takeaway
Only TartanDrive (1.0 and 2.0) records suspension shock travel and wheel speed, on a Yamaha Viking ATV. RELLIS-3D, GO, ORFD, CaT, FinnForest and Boreas give IMU/GNSS plus lidar/camera, but no suspension data. Boreas (200 Hz Applanix) and RELLIS-3D (100 Hz IMU) are the most usable for vertical-acceleration spectra. None is a passenger car on forest gravel with suspension channels, so comparisons will be spectral and statistical, not trace to trace.

### Cited Findings
- **TartanDrive 1.0**:
  - Platform and size: modified Yamaha Viking ATV, ~5 h, 630 trajectories, ~200k interactions (184k in Table I). Terrain: rocks, mud, tall grass, puddles, steep slopes, loose gravel, open road (Pittsburgh).
  - Sensors: Multisense S21 stereo + IMU (images 10 Hz, IMU upsampled to 200 Hz); Racepak G2X Pro logging all four wheels' RPM (50 Hz) and shock travel (rate not stated) plus brake position; NovAtel PROPAK-V3-RT2i GNSS. No lidar.
  - Benchmark: RMSE after 20 forward steps. Kinematic bicycle 1.1638, best learned model 0.1674. Difficulty metric: average height change per second.

  — [TartanDrive, arXiv 2205.01791](https://ar5iv.labs.arxiv.org/html/2205.01791); [GitHub castacks/tartan_drive](https://github.com/castacks/tartan_drive)
- **TartanDrive 2.0**: ~7 h, at higher speeds. Two Velodyne VLP-32 and a Livox Mid-70 at 10 Hz, Multisense S21 at 10 Hz. Racepak per-wheel RPM, but shock travel for the **rear two wheels only**: the front shock sensors were often damaged in aggressive driving. A secondary summary reports NovAtel IMU at 100 Hz and Multisense IMU at 400 Hz (not verified in the paper). — [TartanDrive 2.0, arXiv 2402.01913](https://arxiv.org/pdf/2402.01913); [Moonlight summary (secondary)](https://www.themoonlight.io/de/review/tartandrive-20-more-modalities-and-better-infrastructure-to-further-self-supervised-learning-research-in-off-road-driving-tasks)
- **RELLIS-3D** (Texas A&M / ARL, Clearpath Warthog): Ouster OS1-64 (2048 horizontal, 10 Hz) and a VectorNav VN-300 dual-antenna GNSS/INS (100 Hz IMU). The GPS/IMU is not PTP-synchronised. — [RELLIS-3D, arXiv 2011.12954](https://arxiv.org/pdf/2011.12954); [GitHub](https://github.com/unmannedlab/RELLIS-3D)
- **GO (Great Outdoors)**, also a Warthog: Ouster OS1, MicroStrain 3DM-GX5-AHRS at 200 Hz, Sparkfun RTK Facet (1.4 cm, 4 Hz). — [GO, arXiv 2501.19274](https://ar5iv.labs.arxiv.org/html/2501.19274)
- **ORFD**: 12,198 lidar + RGB pairs from a 40-line lidar and a 1280x720 camera, labelled traversable / non-traversable / unreachable, over woodland, farmland, grassland and countryside. No proprioception reported. — [ORFD, arXiv 2206.09907](https://arxiv.org/pdf/2206.09907)
- **CaT (CAVS Traversability)**, MSU CAVS: about 12,300 camera images (1,812 labelled in one count) from three trails at the CAVS proving ground. Traversability is labelled per vehicle class (sedan, pickup, off-road). Lidar and GNSS/INS were recorded in rosbags but the labels are on images. A separate CAVS set is synthetic, generated with MAVS (1,947 image pairs). — [CAVS open datasets](https://cavs.msstate.edu/resources/autonomous_dataset.php); [CaT, IEEE Access 2022 (DOAJ)](https://doaj.org/article/fac88761ed80462db369c84257c71d77); [datasetninja](https://datasetninja.com/cat)
- **FinnForest**: four RGB cameras, IMU and GNSS on a car in Finnish forest roads, for visual SLAM. IMU rate not found. Do not confuse with FinnWoodlands (2023, different authors). — [Tampere Univ. news](https://research.tuni.fi/3dmedia/news/finnforest-dataset-a-forest-landscape-for-visual-slam-published-in-robotics-and-autonomous-systems/); [FinnWoodlands](https://ar5iv.labs.arxiv.org/html/2304.00793)
- **Boreas** (U. Toronto, multi-season, paved): Applanix post-processed IMU and ground truth at 200 Hz (`applanix/imu.csv`, `gps_post_process.csv`). Boreas-RT lists a Silicon Sensing DMU41 at 200 Hz. — [Boreas, arXiv 2203.10168](https://arxiv.org/pdf/2203.10168)
- **Verti-Bench** is a Chrono-based off-road mobility benchmark on vertically challenging terrain, "further validated on a physical testbed" (small-scale robot, not a car). — [Verti-Bench, arXiv 2502.11426](https://arxiv.org/pdf/2502.11426)

### Inferences
- Comparison method, given these datasets:
  1. Compute the PSD of chassis vertical acceleration in 0.5-30 Hz from the sim IMU and from the dataset IMU, at matched speed bins.
  2. Compare the body-bounce (1-2 Hz) and wheel-hop (10-15 Hz) peak frequencies and the PSD slope.
  3. Compare ISO-weighted RMS per speed bin.
  4. With TartanDrive's rear shock travel, compare shock-travel PSD and the fraction of time at bump stop.

  Note what each dataset supports. 100 Hz IMUs (RELLIS-3D, VN-300) cap analysis at 50 Hz Nyquist, enough for wheel hop. TartanDrive 1.0's IMU is *upsampled* to 200 Hz from a lower native rate, so above ~50 Hz is not trustworthy.
- An "effective IRI" can be backed out of a dataset run by driving the golden quarter-car with the vertical displacement inferred from GNSS/IMU. This is crude, since the sprung mass filters the input. Comparing IRI of the synthetic road surface directly is cleaner.
- Platform mismatch matters. TartanDrive/RELLIS/GO are ATV or Warthog (short wheelbase, long travel), so their spectra will not equal a Model 3's. Use them for terrain-response trends (how RMS scales with speed and roughness), not absolute levels.

### Gaps
- No open dataset of a road car on gravel or forest roads with suspension travel was found. ROOAD, "CARLA-offroad", M2P2 and public rally telemetry were not verified in this pass.
- Native IMU rates for TartanDrive 1.0 and FinnForest, and the shock-travel rate, are unconfirmed.
- MAVS physics-validation papers (vehicle dynamics, not synthetic images) were not retrieved.

## Validation: simulation-vs-real studies of off-road vehicle dynamics

### Takeaway
The NATO Next-Generation NRMM (NG-NRMM) benchmark is the main organised effort, with Chrono among the entries and a "blind correlation to real vehicle test data" tier. I found no open source reporting Chrono HMMWV vertical-acceleration agreement numbers on a test course. Chrono's own tyre docs claim validation for handling (steady-state cornering), not ride.

### Cited Findings
- The NG-NRMM Tracked and Wheeled Vehicle Benchmark includes multiple vendors' simulators (Chrono among them), with a validation tier of "blind correlation to a real vehicle test data set". — [ESD / NATO benchmark (PDF)](https://events.esd.org/wp-content/uploads/2018/07/Tracked-and-Wheeled-Vehicle-Benchmark-A-Demonstration-of-Simulation-Maturity-for-Next-Generation-NATO-Reference-Mobility-Model.pdf)
- Chrono's handling tyre models were validated against "measured real vehicle test data" from a technology demonstrator (handling, not vertical ride). — [Project Chrono tire models](https://api.projectchrono.org/wheeled_tire.html)
- Chrono's HMMWV_Full is used as a high-fidelity reference, at 100 Hz, on flat, deformable (CRM) and bumpy rigid terrain. That is sim-to-sim. — [arXiv 2608.19375](https://arxiv.org/pdf/2608.19375)
- A Mini Baja multibody model's vertical acceleration matched measured signals for most obstacles. — [Scientia cum Industria](https://sou.ucs.br/revistas/index.php/scientiacumindustria/article/view/1308)

### Inferences
- For our purpose the cheaper route is the bump, pothole and ISO 2631 sanity checks above. Large-programme validations (NG-NRMM) are about soft-soil mobility and go/no-go, which the gravel/rock case does not need.

### Gaps
- No ERDC/ARL NRMM or Chrono HMMWV ride-validation numbers (RMS error, peak-acceleration agreement) were retrieved.
- No MSU CAVS MAVS vehicle-dynamics validation against measured data was found in this pass.

## Sensors: what lidar sees of rocks, roughness, negative obstacles and dust, and Isaac RTX fidelity

### Takeaway
At range, small rocks get only a few lidar returns, and negative obstacles (potholes, ditches) lose information much faster with range than positive ones. Dust produces low-intensity false returns, best handled with last-return mode plus intensity filtering. Isaac 6.1 RTX lidar models materials and reflective/transparent surfaces. But motion compensation and motion-distortion fidelity need Motion BVH, which is **off by default**. Multi-echo `echoId` is broken (always 0), so dust and last-return behaviour cannot be reproduced faithfully today.

### Cited Findings
- Information about negative obstacles falls off "much more rapidly" with range than for positive obstacles. — [Lebakula, MSU thesis 2021](https://scholarsjunction.msstate.edu/td/5329)
- Tilting a lidar 40° cut the near-field blind spot for negative obstacles from 3 m to 0.21 m (orchard UGV). — [LiDAR-Based Negative Obstacle Detection in Orchards (PMC)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11679008/)
- Shang et al. (JFR 2016) report a multi-lidar setup with a 20 % longer negative-obstacle detection range than prior methods. Ditch detection uses range jumps along scan rings and dense points at the ditch's upper edge. — [JFR 10.1002/rob.21609](https://zendy.io/title/10.1002/rob.21609); [UTS paper](https://opus.lib.uts.edu.au/handle/10453/127018)
- Dust:
  - With the STRONGEST return mode, dense dust dominates the cloud. LAST return preserves the ground and the objects behind it.
  - Dust returns had intensity ≤ 1 on Hesai AT128, while cones and a mannequin stayed high.
  - Their filter: flag low-intensity points per lidar, keep them through ground segmentation, drop them afterwards. It breaks with multiple lidars whose intensity scales differ.

  — [Autoware in Construction, arXiv 2610.04187](https://arxiv.org/html/2610.04187)
- Dust returns have lower intensity than real objects. A de-dust filter (intensity threshold, then statistical outlier rescue) removes dust without losing surrounding data. — [Ontario Tech / KIMM (MDPI Eng. Proc.)](https://mdpi.com/2673-4591/10/1/70); [Ontario Tech thesis](https://ir.library.ontariotechu.ca/handle/10155/1505)
- Lidar false returns from dust, snow and water show up as noise in aggregated maps. TerrainNet removes them with a voxel-count threshold plus a semantic outlier class. — [TerrainNet, arXiv 2303.15771](https://arxiv.org/pdf/2303.15771)
- Isaac Sim 6.1 RTX lidar:
  - "Motion BVH must be enabled for RTX Lidar motion compensation to work correctly". It is disabled by default for performance (`--/renderer/raytracingMotion/enabled=false`). Enable it with `enable_motion_bvh: True` in `SimulationApp`, or with the `raytracingMotion` flags plus `enableHydraEngineMasking` and `enabledForHydraEngines='0,1,2,3,4'`. It "can significantly increase rendering time" and VRAM.
  - `omni:sensor:tickRate` must equal `omni:sensor:Core:scanRateBaseHz`, and `accumulateOutputs=true` is needed for full scans.
  - When rotation is slower than the frame rate, accumulated scans mix returns from multiple frames.
  - Known issue: `echoId` is always 0, even for multi-echo profiles, with no workaround.
  - Non-visual materials (`omni:simready:nonvisual`) come off by default. They need `--/rtx/materialDb/nonVisualMaterialCSV/enabled`.

  — [Isaac Sim 6.1 RTX Sensors](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/sensors/isaacsim_sensors_rtx.html); [6.0.1](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/sensors/isaacsim_sensors_rtx.html)
- Standalone Python needs `--/app/player/useFixedTimeStepping=true`, or the lidar model discards points when a frame runs long. That setting is on in the full app and off in standalone. — [Isaac Sim 6.0.0 RTX Sensors](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/sensors/isaacsim_sensors_rtx.html)
- A forum post (not official docs) says the RTX 2D rotary lidar supports motion-distortion correction since 5.0, through an output mode that is "raw/distorted" by default. — [NVIDIA forum](https://forums.developer.nvidia.com/t/about-isaac-sim-rtx-lidar-2d-rotary/368538)

### Inferences
- Rough angular arithmetic: a 0.3 m rock at 50 m subtends ~6 mrad. A lidar with 0.2° (3.5 mrad) vertical ring spacing hits it with about 2 rings and a handful of points per ring. This is my arithmetic, not a measured figure. Real counts depend on the profile, so compute them from the Isaac lidar profile's elevation table. Below ~10 cm, a rock at 30+ m is effectively invisible to ground segmentation either way.
- Because Motion BVH is off by default, an Isaac RTX lidar on a pitching, bouncing chassis probably scans a static snapshot per render frame. Within-scan motion distortion from vibration is then under-represented, unless Motion BVH is on and the rendered frame rate is well above the lidar rate. Check this before relying on the sim for distortion-correction testing.
- With `echoId` broken, sim lidar cannot reproduce the strongest/last-return difference that dominates real dust handling. Dust is better left out of the sim than modelled half-way.

### Gaps
- No published measurement of Isaac RTX lidar accuracy on small geometry (rocks of 5-30 cm). No noise or beam-divergence parameters were documented on the RTX page.
- No quantitative dust false-return rate against dust density was found.

## Sensors/autonomy: vibration and pitch effects on lidar, camera, IMU, NDT and Autoware ground segmentation; how simulators model sensor shake

### Takeaway
Pitch vibration is the dominant lidar error. A few tenths of a degree moves the ground hit point by metres at range, which shows up as false obstacles. Autoware's `scan_ground_filter` has a documented trade-off: thresholds tight enough to keep uneven ground as ground pass small rocks as ground, and the reverse flags uneven ground as obstacles. Unfiltered ground points are a known cause of Autoware's obstacle stops. Lidar-only localization degenerates in feature-poor or self-similar scenes, which is what forests and open fields are. In the sources I found, no simulator applies an added vibration model to sensors: sensors are rigidly attached and inherit chassis motion only.

### Cited Findings
- A DARPA Grand Challenge team (Overbot), 75 Hz scanner on a shock mount, with vibration above a supposed 20 Hz filtered: "small amounts of vibration ... especially in the pitch axis, result in large range errors, as the scan hits the ground in the wrong place", causing false obstacles. A "few degrees" of error came from mount and frame flex, and usable sight fell to 10-12 m out of 20-25 m. Their fix estimated actual pointing from the scan's own ground returns (smoothed per-line ground distance, with the residual converted to a tilt correction). — [Overbot Note #51](http://www.overbot.com/grandchallenge/note51.html)
- Autoware `scan_ground_filter`:
  - A large slope threshold "may reduce false positive of high slope road classification but ... increase false negative of non-ground classification, particularly for small objects".
  - It "might cause false positive for small step-like road surface or misaligned multiple lidar configuration".

  — [Autoware Scan Ground Filter docs](https://autowarefoundation.github.io/autoware.universe/pr-8870/perception/autoware_ground_segmentation/docs/scan-ground-filter)
- Autoware CUDA ground segmentation: `global_slope_max_angle_deg` defaults to 10°. Larger values cut false negatives on steep roads but raise false positives (objects called ground). `use_recheck_ground_cluster` is on by default and re-checks ground clusters to reduce false ground on elevated points. — [autoware_ground_segmentation_cuda](https://autowarefoundation.github.io/autoware_universe/latest/perception/autoware_ground_segmentation_cuda)
- Off-road Autoware (construction dump truck, Hesai AT128):
  - One `scan_ground_filter` threshold set "misclassifies uneven ground as obstacles", the other "incorrectly classifies small obstacles, such as rocks, as ground". GSeg3D did better qualitatively.
  - Traversability matters: a 30 cm rut is passable, a 30 cm rock is not.
  - "Strong vibrations degrade IMU measurements" and "severe vertical vibration requires 3D LiDAR distortion correction". Full 6-DoF pose is needed on slopes and rough terrain.
  - Map-based localization is challenged by feature-poor, changing sites.
  - No quantitative results.

  — [Autoware in Construction, arXiv 2610.04187](https://arxiv.org/html/2610.04187)
- Autoware discussion #7038: Obstacle Stop Planner stopped on a clear road. A maintainer confirmed obstacle points right in front of the ego that were "probably ground points that were not properly filtered". — [autowarefoundation discussion #7038](https://github.com/orgs/autowarefoundation/discussions/7038)
- A TIER IV-linked discussion notes that ground segmentation can fail on "speed bumps or rough road surfaces", leaving ghost points in front of the car (motivating a camera-segmentation pointcloud filter). — [autowarefoundation discussion #3974](https://github.com/orgs/autowarefoundation/discussions/3974)
- Patchwork (ground likelihood estimation) is aimed at uneven terrain and "dramatically" cuts false positives on SemanticKITTI and rough-terrain data. It is an alternative to `scan_ground_filter`. — [KAIST record](https://koasas.kaist.ac.kr/handle/10203/289118); code github.com/LimHyungTae/patchwork
- Lidar degeneracy is ill-conditioning of the registration optimization from missing information along some directions (tunnels, self-similar scenes). An ETH field study (ANYmal, forest and open field) found degeneracy-aware constraints applied *before or during* optimization beat post-hoc ones in the wild. — [Tuna et al., arXiv 2408.11809](https://arxiv.org/html/2408.11809v1)
- Intensity features "drastically improved robustness against geometric degeneracy" where geometric baselines failed. — [Real-Time SLAM with LiDAR intensity](https://cs.paperswithcode.com/paper/real-time-simultaneous-localization-and-mapping-with-lidar-intensity)
- Degeneracy-resilient teach-and-repeat with FMCW (Doppler) lidar addresses geometrically challenging environments. — [arXiv 2603.10248](https://arxiv.org/pdf/2603.10248)

### Inferences
- Pitch-error geometry (my arithmetic): for a lidar ~1.9 m high on a Model 3 roof rack, the ground hit at range R sits at a depression angle of about h/R. A pitch error Δθ moves the hit point by roughly R²Δθ/h. At R = 20 m, 0.5° (8.7 mrad) shifts it by ~1.8 m along track. Equivalently, the ground appears ~R·Δθ ≈ 17 cm too high or low at that range. That is above typical `scan_ground_filter` non-ground height thresholds, so pitch bounce on gravel can create transient "obstacles". This is the Overbot failure mode, and plausibly the emergency-stop trigger on rough roads.
- Body pitch at 1-2 Hz is slow relative to a 10 Hz scan, so it mostly tilts whole scans; Autoware's distortion corrector, which uses the IMU and twist, handles it if the IMU is good. Wheel-hop and washboard vibration at 10-30 Hz is within-scan, so it smears each scan unless corrected per point. In Isaac it only shows if Motion BVH is on (previous section).
- NDT on forest roads gets both problems at once: (1) a geometrically self-similar corridor of trunks, degenerate along the road axis; (2) roughness, rocks and berms that are not in the point-cloud map, or are sampled differently, adding residual. Pitch and roll excitation also stress the 6-DoF initial guess from the EKF. Expect along-track drift and higher iteration counts. That ties to the project's own `ndt_scan_matcher` iteration-limit fix (docs/issues/002) and to issue 017 (NDT lost at 30 m/s).
- Simulators: the Isaac RTX docs describe sensor timing and motion compensation but no mount-vibration model. A rigidly attached sensor inherits chassis motion exactly. Real mounts add compliance (the Overbot shock mount gave "a few degrees" of error), so the sim is optimistic about pitch jitter unless a mount-compliance or added-jitter model is injected. The cameras' rolling shutter was not found in the Isaac docs retrieved.

### Gaps
- No Autoware GitHub issue explicitly tying rough-road pitch to `scan_ground_filter` false obstacles and emergency stops was found. The evidence is the threshold-trade-off docs, the off-road paper and discussion #7038.
- No quantitative study of NDT (as opposed to ICP) degeneracy in forests, or of NDT error against vibration amplitude, was found.
- Rolling-shutter modelling in Isaac RTX cameras, IMU vibration-rectification/aliasing figures, and CARLA/AWSIM/MAVS sensor-vibration models were not covered in the sources retrieved.
