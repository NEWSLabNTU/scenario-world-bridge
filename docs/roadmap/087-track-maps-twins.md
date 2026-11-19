# 087: Track D: Maps and Digital Twins

The places scenarios run in: the NTU campus (scan, Isaac, CARLA), its roads from public data and its off-road field, SVL's Shalun map, maps for any scenario, and the CARLA 1.0 towns' Autoware maps.

**Status**: consolidated 2026-11-18 from 019, 059, 072, 062, 040, 064, 031, 083.

**Research rule** (user, 2026-11-18): a research track does not wait for scensim releases.
Its work runs on current scensim main, and it moves to each new scensim release when that
release is published (rebuild, rerun the track's reference runs), as a deliberate step.

**Consolidated** 2026-11-18 (user: "Have A-D"): the phases below keep their own documents as
history and detail; this document is the track's plan and status from now on, and new work
is planned here.

## Parts

| phase | what | status | left |
|---|---|---|---|
| [019](019-ntu-campus-twin.md) | NTU campus twin from the scan | see doc | (feeds 059 and 072) |
| [059](059-ntu-twin-on-isaac.md) | the NTU twin on Isaac Sim | in progress, on 133 (/mnt/p5plus/aeon/ntu-twin) | stalled since 2026-10-30 |
| [072](072-ntu-campus-in-carla.md) | the NTU campus as a CARLA map | in progress: r01's corridor in a stock 0.9.16 | retarget to CARLA 1.0 content packs |
| [062](062-ntu-roads-from-open-data.md) | roads around NTU from public data, audited | in progress: O1-O3, O5 done; O6 drives pass on Isaac | O4's paint audit |
| [040](040-ntu-offroad-field.md) | NTU off-road field | planned |  |
| [064](064-svl-shalun-map-import.md) | SVL's Shalun map in Autoware, CARLA and Isaac | in progress: CARLA imported, packaged, driven |  |
| [031](031-maps-for-scenarios.md) | maps for any scenario | deferred 2026-10-13 |  |
| [083](083-carla-1-0.md) | the CARLA 1.0 towns' Autoware maps | new: Town01_Opt's road file equals 0.9.16's; CARLA 1.0 ships an Autoware map generator | regenerate or check every _Opt town |

## Folded in

- CARLA 1.0's content packs install maps without rebuilding the package, and its map generator writes lanelet2 with stop signs and light links: 072 and 064's CARLA side should target 1.0, not 0.9.16.
- The published map dataset's token fix and cards wait for the refactor (user, 2026-11-17).

## Maps v2026.11b: the T3 re-reference

Revision v2026.11b of `jerry73204/carla-town-autoware-maps` (dataset commit 6b1bd8b9) is
1bbdd63's repair: each town's TransverseMercator projector at 0,0 declares CARLA's scale factor,
1.0, where Autoware's default, 0.9996, applied before; the local coordinates were regenerated with
acb 6c0325f (issue 067). Registration p95: Town01 0.159 to 0.007 m, Town02 0.012, Town05 0.005,
Town10 0.007, the ring 0.005; Town03 stays at 1.365 m (issue 068). Every project lock moved to it
in 2ce93408. The T3 suites ran again on the new maps (swb 243, host 243, 2026-11-19 and 20) to
see which references move: CARLA 0.9.16 first, then CARLA 1.0 against the same references.

### CARLA 0.9.16

Build 410915c8, checkout 78d15cf7; each stack started its own CARLA 0.9.16 at Epic. Every
suite started at a 1-minute load of 48 or less (1.5 times the 32 cores), with at least 10 GB
of VRAM and 12 GB of memory free.

| suite | runs | result |
|---|---|---|
| bench/bench | 3 | as the reference |
| basic/town01, basic/towns | 6, 4 | as the reference |
| safety_suite/nightly | 2 (UC-ACC's 27 variants, UC-AEB's 9) | as the reference |
| set050/default, bench, two_av | 8, 1, 1 | as the reference |
| c2b/c2b | 24 | as the reference |
| demo/incidents | 5 | as the reference |
| two_av/two_av | 1 | as the reference |

No suite moved, so the references stay as they are. The one note,
`town01_truck_pass` at 53 s against the reference's 32 s, is wall time. It ended on the same
condition, the truck's collision with the ego (an expected failure).

**Load.** This half sampled no load during runs. As a proxy, the runs' own tick overruns
(ticks over the frame budget) were checked. Four runs had more than 5 % of their ticks over;
all others had 4.2 % or less. Those four ran again one at a time. Each started after four
samples 15 s apart at load 48 or less, and a sampler read the 1-minute load every 5 s:

| run | verdict, both runs | load at start | peak load in the run | ticks over, rerun | ticks over, first run |
|---|---|---|---|---|---|
| two_av | passed | 5 | 616 | 0.4 % (4 of 1000) | 11.5 % |
| c2b d2_v2, pair eq, r0 | passed | 17 | 9.5 | 0.0 % | 10.7 % |
| demo i2_ped_dart | passed | 3 | 90 | 0.0 % | 8.8 % |
| set050 truck_pass | failed (expected) | 17 | 100 | 0.0 % | 7.8 % |

The load average is a poor gate during a run. In two_av it went from 78 to 616 in the 50 s after
the run started and then decayed: its two Autoware stacks starting. Yet that run had 0.4 % of
its ticks over budget. CPU pressure (`/proc/pressure/cpu`, full) stayed 0 and I/O pressure about
1 %. The load average counts our own pilots' start-up, not contention the world server feels.
What tells a disturbed run is its ticks over budget (and pressure). The first runs' overruns
(7.8 to 11.5 %) most likely came from other work on the shared host at the time; no process
list was recorded to show it. The reruns' verdicts are
the first runs': the 0.9.16 half stands.

### CARLA 1.0

The same 10 suites on the CARLA 1.0 package (Epic, port 2400, the `carla-10` builds of acb_bridge
and scensim_carla_world, checkout bfa4e806), compared with the same 0.9.16 references.

All 10 suites compare as the reference: bench 3, basic/town01 6, basic/towns 4,
safety_suite/nightly 2, set050 8, 1 and 1, c2b 24, demo/incidents 5, two_av 1.

**Contention, and a gate per run.** The first 1.0 script gated each project only. bench,
basic and safety ran that way, and basic's and safety's results stand. Three runs went over 5 % of
their ticks. In them, CPU pressure (some, avg10) reached 68 to 82 % and the load 68 to 199:
- bench town01_npc_10: 6.8 %, and its frame_budget failed (processing p95 12.5 ms against
  10 ms; the reference passed);
- basic town02_ego_drive: 7.7 %;
- basic town01_ego_drive: 5.0 %.
A top sampler (top's own %CPU every 30 s) named the source: Rust builds of another project on the
shared host, about seven cores of rustc at 03:31 to 03:34. In set050's first run, Autoware stayed
INITIALIZING for 120 s, every tick took 100 to 230 ms, and SSv2 timed out.

The rest ran with a gate before every run (`--shard i/N`, one run per command, its link moved
into the suite's directory and the summary rebuilt). Each run waited for load 48 or less and
CPU pressure 25 % or less, four samples 15 s apart, with memory and VRAM as before. 25 % is the
top of the uncontended 1.0 runs (5 to 23 %); under the builds it was 45 to 80 %.

That pass ran set050, c2b, demo and two_av, and the three runs above again. The gate never held
a run for long: the longest wait between runs was 347 s (with a stack stop between projects),
the mean 103 s. Each run started at load 7 to 30 and CPU pressure 0 to 18 %. No run went over
5 % (most 4.0 %, two_av), so none ran a third time. Quiet, bench town01_npc_10 passed its frame
budget: 0 of 1200 ticks over, processing p95 3.4 ms. Its failure was the contention, not 1.0.

**Result.** On the repaired maps, no suite moved on either version, and CARLA 1.0 gives the 0.9.16
references' verdicts in all 10 suites (55 runs). The references stay as they are.

## Next: the CARLA 1.0 map generator study

What it takes to produce a CARLA 1.0 map and its Autoware map (lanelet2, point cloud,
`map_projector_info`) in one frame, from our pipeline. Planned 2026-11-19 (241; manager's calls the same day).

**What exists.**
- **OpenDRIVE to lanelet2:** acb's `carla_map_gen` (crdesigner 0.8.5 plus its postprocess) and the
  `scripts/repair_lanelet_*` fixes (light links, right of way, speed limits, turn directions, and more),
  with `lanelet_opendrive_registration.py` to measure the fit.
- **The point cloud:** acb's `carla_pcd_gen` (lidars at the spawn points), `densify_pcd_along_lanes.py`
  (lidars along every lane), and sampling from meshes (`open_maps/world.py`).
- **CARLA 1.0's own Autoware map tools** (`PythonAPI/examples/av_stacks/autoware/map_tools`) do the
  same: crdesigner 0.8.5, stop signs and lights from the live actors, and a noise-free 64-beam lidar
  along the waypoints. They overlap ours; the stop signs are their one addition.

**What CARLA 1.0 needs.**
- **A road-only map:** OpenDRIVE standalone mode (`generate_opendrive_world`) builds one from an
  `.xodr` on the packaged server, with no editor.
- **A map with buildings:** `Util/Tools/Import.py`, `carla-pack` and the Digital Twin Tool all need a
  CARLA UE 5.8 source build with the editor, which no host has (about 200 GB). The content pack it
  makes installs into the packaged server of the same release.

**Gaps.**
- Our CARLA-client tools are on 0.9.16's API.
- Only 072's `lanelet_to_xodr` writes an `.xodr` from our own data, and it carries no signals; open_maps
  writes none.
- The towns' frame (Local, or TM at 0,0) differs from the real places' (MGRS).
- `autoware_lanelet2_map_validator` has not been run on our maps, nor point-cloud coverage against the
  meshes.

**Steps.**
1. **S1, Town01_Opt, no editor** (host 241, or 139 after 084e).
   - Port the CARLA-client point-cloud tools in acb under its CARLA version feature, so 0.9.16 keeps
     working.
   - Generate Town01_Opt's Autoware map with CARLA's map_tools and with ours.
   - Compare both with `carla-town-autoware-maps`' Town01: registration offsets, the validator, and the
     point cloud against the meshes.
   - Drive Autoware on each. This decides 083's decision 2 (replace or feed), and whether the `_Opt`
     towns need new maps.
2. **S2, an OpenDRIVE of our own, no editor.**
   - A private map's `.xodr` through `generate_opendrive_world` on packaged 1.0, then the same
     generation, checks and drive. This proves the road-only path end to end.
   - Private data: its results go to the NAS (and `2026-taiwan-testbenches` if needed), never into
     this repository.
3. **S3, the editor build** (UE 5.8 plus CARLA `ue58-dev`), only if S1 and S2 show the need, and the
   user's call with their numbers.
   - Import with `Import.py`, package with `carla-pack`, install into the packaged server, drive.
   - Then 072's campus as a content pack.
4. **S4, one tool:** 031's `map_build`, from an `.xodr` (and meshes) to a CARLA pack and the Autoware
   map directory, with the gates (validator, registration, point-cloud coverage, a drive), per CARLA
   version.

Ahead of 059's Isaac twin (manager, 2026-11-19).

**S1 and S2 results** (241, 2026-11-19; NAS `logs/2026 SSv2 on CARLA/2026-11-19 087 S1 CARLA 1.0 map generator
Town01_Opt (241)/`; S2's report is private).

- **S3, the editor build, is not needed so far.** OpenDRIVE standalone mode on the packaged server builds a
  road-only map from an `.xodr` in about 2 s, with its traffic lights and stop signs as actors.
- **Town01_Opt:**
  - its `.xodr` is byte-identical to 0.9.16's `Town01`;
  - acb's `carla_map_gen` puts lanelets on the OpenDRIVE lanes within 0.7 cm (p95), against 16 cm for our current
    Town01 map;
  - acb's `carla_pcd_gen` runs on 1.0 as built with `--features carla-10`.
- **The validator** (`autoware_lanelet2_map_validator`, from autoware_tools 0.2.0; 0.3.0 no longer has it): no
  generator passes it as generated.
  - Our current map has 36 errors (a light's ref_line is not a stop_line).
  - `carla_map_gen`, raw, has 126; CARLA's map_tools have 108 (local coordinates, light facing).
- **CARLA's map_tools have two defects.**
  - On an asynchronous server the point-cloud tool's `--tick` does nothing, so each lidar measurement is one server
    frame: 170k points for Town01_Opt instead of 3.9 M with the world set synchronous at 0.1 s.
  - The tools write `projector_type: Local`, which SSv2 rejects.
- **`carla_map_gen` is town-only.** It writes TransverseMercator at 0,0 and ignores the `.xodr`'s georeference, and
  a real place's georeference can carry a false origin that Autoware's TransverseMercator cannot hold.
- **Next (manager, 2026-11-19).**
  - In acb: the lanelet repairs move into its map tools, and `carla_map_gen` carries the georeference (MGRS, or an
    origin shift) and runs the repairs by default. The target is 0 validator errors on Town01_Opt.
  - Then an Autoware drive on Town01_Opt with each map, on a host with a scensim stack on 1.0.

**Town projectors at CARLA's scale** (241, 2026-11-19, as its own commit, landed after 243's T3 CARLA 1.0
comparison). CARLA georeferences its towns as `+proj=tmerc +lat_0=0 +lon_0=0 +k=1`. Our towns'
`map_projector_info.yaml` named no `scale_factor`, so Autoware and SSv2 took 0.9996, and the map was 0.04 % small
against CARLA: 0.13 m at 330 m, Town01's lanelets 0.159 m (p95) off their OpenDRIVE lanes. `download_maps.sh` now
declares `scale_factor: 1.0` on every TransverseMercator town at 0,0 that names none (p95 0.007 m on Town01, 0.005 m on
Town05), and `gen_npc_benchmark.py` reads the map's scale. **Every T3 reference recorded before this commit was on the
0.9996 maps**: positions near a town's far edge move by up to 0.13 m.

