# 086: Track C: Road and Terrain Physics

One tyre-and-ground model: surfaces and soil (wet, gravel, mud, sand, snow), calibration against measurements, the scenario choosing its ground, small obstacles and roughness, and the road section itself (bank, crown, edges).

**Status**: consolidated 2026-11-18 from 047, 052, 053, 056, 066, 069. Owner swb 118 from
2026-11-20 (user, through 133). 2026-11-20: re-planned as one model ("The plan" below),
approved by 133 the same day with its three decisions; C1 next.

**Research rule** (user, 2026-11-18): a research track does not wait for scensim releases.
Its work runs on current scensim main, and it moves to each new scensim release when that
release is published (rebuild, rerun the track's reference runs), as a deliberate step.

**Consolidated** 2026-11-18 (user: "Have A-D"): the phases below keep their own documents as
history and detail; this document is the track's plan and status from now on, and new work
is planned here.

## Parts

| phase | what | status | left |
|---|---|---|---|
| [047](047-off-road-surfaces.md) | off-road surfaces | done 2026-10-23 (every step and acceptance item) | nothing; its open ends moved to 052 and 053 |
| [052](052-isaac-physics-completeness.md) | Isaac physics, feature-complete | in progress: steps 1-5 done (physics egos) | the lead NPC's sinking seen by camera and lidar; Autoware egos with 039 |
| [053](053-terramechanics-calibration.md) | soil calibration against published measurements | in progress: sand, mud and snow fitted | dirt (no data); dropping the flags; the field (040) |
| [056](056-scenario-surface-maps.md) | the scenario chooses its ground | planned | named surface maps, label classes |
| [066](066-small-scale-road-physics.md) | small-scale road physics: rocks, roughness | in progress: R0, R2, R3, R5 done; R4 three slices | R1 checks, R4, R6 wheel hop, R7 sensors, R8 the section |
| [069](069-road-tilt-load-and-edges.md) | road tilt, tyre load and the road's edge | planned, design only | the road-section record (E0, this track's C3); E1, E2, E4: Track B, unassigned |

## Folded in

- Isaac only today; CARLA 1.0's Chaos vehicle ([083](083-carla-1-0.md) step 5) is a second target for the surface classes.

## The plan (re-planned 2026-11-20)

### What there is today

Three files describe the ground to the simulator, written by different hands and read
separately:

| file | phase | holds | written by | read by |
|---|---|---|---|---|
| `<map>/surfaces.yaml` | 047 | a class per ground label (`defaults`), zones (polygons, class, wetness, water depth, soil) | hand; the course builder (labels only) | the Isaac world server's surface layer; nothing on CARLA |
| `scensim_isaac_world/config/surface_classes.yaml` | 047, 053, 054 | per class: friction, wet factors, the tyre curve's shape, hydroplaning, soil (Bekker-Wong-Janosi), its calibration source | hand; 053's fits | the Isaac surface layer and soil |
| `<map>/road_detail.yaml` | 066 R3 | corridors: a centreline, a seed, roughness class, one constant `section` (half width, crown, shoulder), ruts, potholes, washboard, water bars, rocks | hand | the Isaac road layer (contact, render tiles, rocks) |
| `course/section.npz` | 069 E0 | per 1 m station: bank, crown, vertical curvature, per side shoulder, lip, foreslope, ditch, run-off, barrier, hazard class | nobody yet (planned) | planned: Isaac's ribbon, `road_detail`, the rails, autoware-drift's `Track` and plant |

Two of them already overlap: `road_detail.yaml`'s constant `section` and 069's per-station
section are the same thing at two resolutions, and 069's `shoulder_surface` and
`runoff_surface` are surface classes that `surfaces.yaml` would otherwise hold as zones. And
the course builder computes the ground's drop for the guardrails (`course/props.py`) and
throws it away.

### What 070 and 072 taught

- **070 (the Isaac ego did not start downhill).** PhysX's sticky-tyre hold leaves only on drive
  torque, so a brake request below the floor held the car on a 3.1 deg slope. The ground's
  grade decides whether a car moves at all at low speed, and the fault sat between the
  car's computers and PhysX, not in the ground. So: a step that touches the tyre contact
  (066 R5, R6, R8) reruns aib's slope matrix (`aib.steptest`: start and hold on a downhill,
  flat and uphill) as part of its exit check, and the car's side (aib, sim_vehicle_ecu) stays
  a consumer of the ground record, never a writer.
- **072 (Haruna's point cloud was too large for the compare-map filter).** A map's files are
  read by consumers we do not own, and their limits (an int32 voxel count) set the format. The
  error showed up 230 m down the course as an obstacle stop, far from its cause. So: every
  record is checked when it is loaded, by the code that reads it, naming the fix; a road
  change that moves the ground (bank, crown, lip) regenerates Autoware's maps (point cloud,
  lanelet heights) from the same meshes, in the layout the profile expects (divided above a
  size); and a record change in a published map is a dataset revision with a new tag (scensim
  resolves the newest tag), which the user publishes.

### Shared data and consumers

**Shared data: two records, both in the map directory, both simulator-neutral.**

1. **The surface record**: `surfaces.yaml` as it is (label defaults, zones), plus the class
   table it names. The class table moves out of the Isaac world server into
   `scensim_world_protocol` (installed share, versioned with the protocol), so that the CARLA
   world server and the road record read the same class names and values without depending on
   Isaac's package. Classes keep every field they have; a simulator uses what it can (CARLA:
   `mu`, `wet`; Isaac: all of it).
2. **The road record**: `<map>/road.yaml`, a list of corridors, each with a centreline, a seed
   and its section **per station**, which replaces both `road_detail.yaml`'s constant `section`
   and 069's planned `course/section.npz`:
   - per corridor in YAML: `centreline` (or `stations: <corridor>.npz`), `seed`, `roughness`,
     `ruts`, `potholes`, `washboard`, `water_bars`, `rocks` (066 R3/R4's keys, unchanged);
   - the section: either constants in YAML (today's `section:`, expanded to every station) or
     arrays in `<corridor>.npz` over the corridor's 1 m stations: 069's fields (`bank`,
     `crown_left`, `crown_right`, `crown_offset`, `kappa_v`, and per side `shoulder_w`,
     `shoulder_surface`, `lip_h`, `lip_angle`, `foreslope`, `ditch_depth`, `runoff_len`,
     `runoff_surface`, `barrier_offset`, `hazard`);
   - surfaces in the section are class names of the surface record's table; the shoulder and
     run-off strips become zones of the surface layer, generated from the record at load, not
     written twice;
   - `schema: 1` at the top; a reader refuses a newer schema by name.

**Who writes them.** Never the simulator at run time.

| writer | writes | when |
|---|---|---|
| the course builder (`scensim_isaac_world.course`) | `road.yaml` and the `.npz` for a generated course (Haruna, Tsukuba): sections from its own geometry, `kappa_v` from the smoothed profile, the hazard classes from one classifier (today's rail rule, kept) | at build; shipped in the course's dataset |
| a map author | `road.yaml` with constant sections for a town corridor (today's `road_detail.yaml` files), `surfaces.yaml` zones | by hand, checked by `road check` and `surfaces-check` |
| 053's fits | the class table's soil rows and their `calibration:` | per fit |
| later, a twin fitter (087) | sections fitted to a scan with a curvature penalty (069's note on noisy sections) | per twin |

**Who reads them (the consumers).**

| consumer | reads | phase |
|---|---|---|
| Isaac surface layer | surface record; the record's shoulder and run-off strips as zones | 047 (done), extended |
| Isaac road layer: contact, render tiles, rocks | road record (per-station section, detail keys) | 066 R3-R7 |
| Isaac course geometry: ribbon, rails, Autoware maps | road record (bank, crown, lip, barrier) | 066 R8 |
| the scenario | names a surface map, sets label classes (`swb.surface_map`, `swb.default.<Label>`) over the surface record | 056 |
| CARLA world server (0.9.16 and 1.0 Chaos) | surface record: zones as friction trigger boxes, `mu` and `wet` | new, step C7 |
| autoware-drift's planner and plant (Track B, 085) | road record (`Track.from_swb_course`) | 069 E1, E2, E4 (Track B, unassigned) |
| aib, sim_vehicle_ecu | nothing directly; they feel the ground through PhysX and the contact forces | 070's slope matrix as a check |

So of 047/056/069: **047's `surfaces.yaml` and class table, and 069's section, are the shared
data**; 056, 066's layers, CARLA and the planner are consumers. 069 splits: E0's record is
this track's, E1, E2 and E4 (the planner's equations and edge pricing) are Track B's work in
autoware-drift, reading the record: Track B, unassigned (085 has no active owner).

### Steps

Each step on current scensim main; the exit check is what lets the next step start.

| step | what | effort | exit check |
|---|---|---|---|
| **C1. The road record and its reader** | `road.yaml` schema 1; a pure reader and checker in `scensim_isaac_world` (`road_record`, every problem named, `road check <map>`); constant sections expand to stations; `road_detail.yaml` read as a road record of constant sections (refused by name once converted, no alias kept); the example maps converted | 1 week | every converted example builds the same detail tiles, byte for byte (tile hashes before and after); `test_road_detail`, `test_road_layer` pass; a malformed record lists every problem |
| **C2. The class table moves to the protocol package** | `surface_classes.yaml` into `scensim_world_protocol`'s share; both world servers find it there; the road record's surface names checked against it | 3 days | Isaac: the Town01 soil, mud and snow scenarios pass as on 053 step 5; an unknown class in a section or zone is refused at load |
| **C3. The course builder writes the record** (069 E0, swb half) | `road.yaml` + `<corridor>.npz` from the builder: bank 0, today's section, `kappa_v` from z, the hazard classifier from today's rail rule; `props.py` places rails from `barrier_offset` | 1 week | Haruna and Tsukuba rebuilt: the same rails at the same positions, the world USD unchanged except the new files, `haruna_down` still passes; the course's dataset revision prepared, published only on the user's go |
| **C4. Named surface maps** (056) | as 056's steps 1-5 | 1-1.5 weeks | 056's acceptance: Town01's soil scenarios one after another on one world server, no file copied |
| **C5. Isaac's geometry from the record** (066 R8) | the ribbon tilted by bank and crown; `road_detail` per station; the lip; the shoulder and run-off strips as surface zones; Autoware's point cloud and lanelet heights regenerated from the same meshes, divided when large | 1.5-2 weeks | Isaac's road height and cross slope within 5 mm and 0.1 deg of the record at every station; lip re-entry fixtures (5, 10, 15 cm at 90, 45, 30 deg); aib's slope matrix unchanged; `haruna_down` passes on a course with a nonzero crown |
| **C6. The rest of 066** | R1's checks, R4 rocks finished, R6 wheel hop, R7 sensors and Autoware, all reading the road record | 5-7 weeks, in 066's order | 066's acceptance; aib's slope matrix and 055's manoeuvre suite unchanged on smooth road |
| **C7. CARLA as a second target for the surface record** | the CARLA world server reads `surfaces.yaml`'s zones as friction trigger boxes (class `mu` x `wet`) and takes `swb.zone.<id>.*`; on 0.9.16 first, then on CARLA 1.0 once 083 step 5 maps the spec to Chaos; label defaults stay asphalt (CARLA's towns have no ground labels per wheel) | 1 week (0.9.16), 3-5 days more (1.0) | a wet zone and a split-mu zone change the CARLA ego's stopping distance and yaw the same way as on Isaac, within 25 % of Isaac's for the same class, on both CARLA versions; the T3 suites unchanged without zones |
| **C8. Calibration and the open 052 item** (independent) | 053: dirt when data comes, step 4 (dropping the flags); 052: the lead NPC's sinking seen by camera and lidar | as data allows; 052's item about 1 week | 053's and 052's own acceptance |

Order: C1, C2 and C3 make the shared data; C4 and C5 are its first consumers, C5 after C3.
C6 follows C5 (rocks and wheel hop on the record's road). C7 can start after C2 on 0.9.16,
and on 1.0 after 083 step 5. C8 runs beside the others. 069 E1 and E2 (Track B, unassigned) can start once
C3 has written Haruna's record. About 4-5 weeks of work to C5, which gives one road from the
record to Isaac, Autoware's maps and the planner.

### Decisions (accepted by 133, 2026-11-20)

- **The class table in `scensim_world_protocol`** (C2): the alternative is to keep it in the
  Isaac world server and have the CARLA one read a copy. Decided: move it, one table.
- **069 E1, E2, E4 are Track B's** (the planner's side), as the record is the only thing they
  share: Track B, unassigned (085 has no active owner).
- **Dataset revisions** for Haruna and Tsukuba after C3 and C5: the user's to publish.
