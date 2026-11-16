# 077 inventory: everything a user or a study runs, and what the project workflow lacks for it

Written 2026-11-15 for [roadmap 077](../077-migration-and-test-matrix.md), read-only, from
the roadmaps 017 to 076, the justfile, `scripts/`, `scenarios/`, `src/swb_examples/`,
`src/swb_launch/scripts/`, `src/swb_agent/`, `src/swb/` (the 076 workflow as built), acb's and
aib's justfiles and `src/swb_isaac_world/`. No simulator was started.

The 076 workflow as built (2026-11-15), the yardstick for "exists now":

- `swb.yaml` keys: `simulator` (carla, isaac), `assets` (`dataset` on Hugging Face, or `path`),
  `maps`, `pilots`, `vehicles` (checked, not loaded), `ground` (Isaac, per map: `surfaces`,
  `road_detail`), `logging` (`capture`, `keep`, `budget`, `topics`), `judges`.
- Pilot kinds: `autoware` (`autoware_launch` swaps the profile's launch), `carla_basic`
  (CARLA only), `launch` (`launch`, `agent: own`), `command` (`run`), `none`. `simple` is
  accepted by `project check` but has no launch in `swb.stack.PILOT_FILES` (simple_pilot is
  070's, not built); `container` later. Common keys: `vehicle`, `args`, `lifetime`, `ready`,
  `inputs`, `rviz`.
- Suites: `scenarios`, `parameters` (SSv2 `override_parameters`), `distribution` (passed as is;
  this SSv2 does not expand ParameterValueDistribution), `repetitions`, `seed`, `expect`,
  `retry`, `pilot_args`, `logging`; `--shard i/n`, `--resume`, `--start-stack`,
  `--stop-stack`; `swb.api.run_one`.
- The stack: `swb.launch.xml` args `project`, `seat`, `simulator`, `simulator_start`, `map`,
  `logs`, `gpu`, `stack_args` (raw `stack.launch.py` arguments; `ego_launch`,
  `ego_launch_args`, `agent`, `map_path` refused by name). The runner passes SSv2 only
  `scenario`, `port`, `output_directory`, `global_timeout`, `vehicle_model`,
  `override_parameters`.
- Not sourced by the stack or its pilots: the Autoware overlay (042), the TensorRT 10.8
  runtime (`AUTOWARE_TENSORRT_LIB`), `third_party/cuda_ndt_matcher`. The justfile's ego
  recipes source all three.

Legend for "exists": **yes** (runs as a project today), **partly** (runs, but a part of the
old item is lost or needs `stack_args`/hand work), **no** (blocked; the gap id from section 3).

## 1. Runnable items

Hosts: "GPU" = an RTX-class GPU with room for CARLA plus Autoware (CenterPoint needs CUDA);
"Isaac" = a host with Isaac Sim 6.1, about 16 GB VRAM and 17.5 GB RAM free for a Town01 world
server. Host names are the lab's (139, 241, 118, 133, 243).

### 1a. Scenario sets on CARLA 0.9.16

| # | Item | Roadmap | Sim | Autopilot | Runs today | New workflow needs | Exists | Pass criterion; reference | Size, time | Host |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | `basic/` starters, Town01 (ego_drive, engage_state, pedestrian, rear_contact, simulator_autopilot, traffic_light) | 020, 023, 024, 032 | CARLA 0.9.16 | Autoware (clustering) | `just scenario <xosc>`; `run_suite basic/ --physical-verdict` | project with Town01, `autoware` pilot, suite over the dir, `physical_verdict` judge | yes (first_drive covers 3 of 6) | SSv2 PASS, `carla_contact` pass (rear_contact declares its contact); 020: 7/7 twice 2026-10-11; 034: 6/6 2026-10-16 | 6 runs, 40-220 s each | GPU |
| 2 | `basic/` other towns: town02_ego_drive, town03, town05, town10 | 020 | CARLA 0.9.16 | Autoware | `just scenario` (stack replaced per map) | `maps:` with 5 towns, map switch | partly (G9: each switch one extra episode) | SSv2 PASS; 020 7/7 2026-10-11 | 4 runs, plus a town load each | GPU, Epic only |
| 3 | `basic/town02_episode_change` | 015, 024 | CARLA 0.9.16 | none (no ego) | `just scenario` | `pilots: {}` or `none`, town load inside the run | partly (G9; Town02 load timeout history) | SSv2 PASS; 024: fail on load timeout, rerun 2/2 2026-10-13 | 1 run | Epic only (Low crashes) |
| 4 | `awf/` UC-ACC-001-0001 (27 variants), UC-AEB-001-0001 (9) | 020, 076 | CARLA 0.9.16 | Autoware clustering, `lifetime: run` | `run_suite awf/`; now `safety_suite` `nightly.yaml` | ScenarioModifiers, timeout scaled by variants, fresh Autoware | yes | 27/27 and 9/9, `physical_verdict` pass; 076 2026-11-15 (`--stop-stack`, `carla.start: auto`) | UC-ACC about 2258 s, UC-AEB about 15 min | GPU |
| 5 | `awf/` with CenterPoint (`safety_suite/suites/centerpoint.yaml`) | 020 | CARLA 0.9.16 | Autoware centerpoint | `just ego-av lidar_detection_model:=centerpoint` then `run_suite` | `args: {lidar_detection_model: centerpoint}`, TensorRT 10.8 | partly (G1: no 10.8 runtime, engines rebuilt each start, 3-11 min) | expected 27/27 ACC, 2/9 AEB (020, 2026-10-11): a known difference, `expect:` it | as 4 | GPU with CUDA |
| 6 | `tutorial/town01_follow_to_goal` | 027 | CARLA 0.9.16 | Autoware | tutorial, `just scenario` | as 1 | yes | SSv2 PASS; tutorial check 2026-10-12 | 1 run | GPU |
| 7 | `multi_av/town01_two_av` | 010, 017, 050 | CARLA 0.9.16 | 2 Autoware (ego, `bg_av_1`) | `just two-av [record:=true]` | `bg_av_*: {kind: autoware}`; screencast | partly (G4: no screencast) | SSv2 PASS both; multi_pilot `two_av.xosc`, `four_pilots` passed 99 s 2026-11-15 | 1 run, 2 stacks | GPU, 2 Autoware |
| 8 | Demo incidents i1 to i5 | 033, 034 | CARLA 0.9.16 | Autoware clustering | `just scenario incidents/<i>.xosc`, then `incident_judge` on the trace | gap trace per run, `incident_judge` as a judge | no (G2) | SSv2 PASS and judge PASS; 034: 5/5 2026-10-16; 25/25 in five rounds (033) | 5 runs, 80-125 s | GPU |
| 9 | Demo incident i6 (cyclist inside), also `scenarios/demo/town01_i6_cyclist_inside` | 033, 042 | CARLA 0.9.16 | Autoware centerpoint + overlay | `just ego-av lidar_detection_model:=centerpoint` (overlay on) | overlay sourced, TensorRT | no (G1, G2) | judge yields to cyclist; 3/3 2026-10-18 | 1 run | GPU with CUDA |
| 10 | Demo incidents i7 cut-in, i8 rear approach | 025 | CARLA 0.9.16 | Autoware | rendered by `gap_genome.py`, never run live | as 8 | no (G2) | none yet: first live run is the reference | 2 runs | GPU |
| 11 | Demo tour `demo/town01_tour` | 033, 042 | CARLA 0.9.16 | Autoware centerpoint, `max_velocity:=11.11`, overlay, `use_multi_channel_tracker_merger:=false`, cast RViz | `just demo-tour [record:=true]` (ClearNoon, chase view, captions, gap trace, screencast, `global_timeout:=1200`) | weather, tools, video, trace, judge, overlay, RViz config | no (G1, G2, G3, G4) | PASS, `incident_judge` 6/6, no contact, G1 0 G2 0; 3/3 rehearsal and take_1 2026-10-21 (4.6 min sim, 5.7 min wall) | 1 run, about 7 min | GPU, a display for RViz/record |
| 12 | `scenarios/gap/` entity_bodies, motorbike_body, npc_shoulder, truck_pass (positive control) | 022, 023, 024, 032 | CARLA 0.9.16 | Autoware | `just scenario`, bridge `gap_trace_dir`, `gap_oracle`, `physical_verdict` | `$(env SWB_MAPS)` rewrite (now `CARLA_MAPS`), trace, oracle judge, `npc_motion` | no (G2, G10) | entity_bodies 2/2 (022, 2026-10-12); motorbike 5/5 G1 0 G2 0 (032, 2026-10-15); truck_pass must DISAGREE (024) | 4 runs | GPU |
| 13 | `scenarios/bench/` town01_npc_10/20/50 | 016, 032 | CARLA 0.9.16 | none | `just scenario` with the bridge's frame timing in its log | `none`/no pilot, a frame-budget metric from the bridge log | partly (no metric in result.json; CARLA_MAPS) | 20 NPCs frame p95 1.79 ms (016); npc_05 snap fixed (032) | 3 runs | GPU |
| 14 | 050 measurement set (`scripts/run_050_set.sh <label> ego tour bench two_av`, `report_050.py`) | 050 | CARLA 0.9.16 | Autoware, 2 Autoware | the script starts sides with `gap_trace_dir`, groups of stacks | suite with trace, per-group pilot args | no (G2) | before/after tables 2026-10-23 | about 15 runs | GPU |
| 15 | Free-standing scenario on upstream SSv2 (068 step 9) | 068 | CARLA 0.9.16 | Autoware | not built | upstream SSv2 | no (planned) | none | | GPU |

### 1b. Scenario sets on Isaac Sim

| # | Item | Roadmap | Sim | Autopilot | Runs today | New workflow needs | Exists | Pass criterion; reference | Size, time | Host |
|---|---|---|---|---|---|---|---|---|---|---|
| 16 | `basic/town01_ego_drive`, town02_ego_drive on Isaac | 038, 043, 046, 051 | Isaac (CARLA towns as USD) | Autoware (aib rig) | `just scenario <x> sim:=isaac`, `just acceptance <x> 3 sim:=isaac`, `just e2e sim:=isaac` | `isaac_first_drive` project, one world per stack | partly (never run live as a project; 241 owes it) | 038: town01 3/3 2026-10-18, tracking inside acceptance limits; 043 town02 screencast PASS 2026-10-19; 051 e2e PASS 2026-10-24 | 1-3 runs | Isaac |
| 17 | Incidents 1-6 and UC-ACC on Isaac | 044 | Isaac | Autoware (i6 centerpoint + overlay) | `just scenario sim:=isaac` | as 8/9 on Isaac | no (G1, G2) | 2026-10-22 i1, i4, i5, UC-ACC PASS; i3 2/2; i6 PASS 2026-10-23; i2 rerun PASS 2026-10-24 | 7 runs | Isaac |
| 18 | `isaac/town01_rear_contact` (standing unmanaged ego) | 037, 044 | Isaac | none (SSv2 not driving the ego) | `just scenario` with `managed_ego:=false` | SSv2 `managed_ego:=false` per scenario | no (G3) | SSv2 PASS with contact | 1 run | Isaac |
| 19 | `scenarios/surfaces/` snow, soil, mud, wet, weather, zones, mixed_ground, mud_right, mud_curve, mud_stop (each with its surfaces yaml) | 047, 052, 054, 056 | Isaac | Autoware | `just acceptance <x> 4 sim:=isaac` (surface map copied in per scenario) | `ground:` per map, but these are per scenario; tracking judge; weather from scenario | partly (G7 per-scenario ground, G11 no acceptance judge) | 052: snow, soil, mud 4/4 2026-10-24 (tracking 0.08-0.30 m/s^2); 054: mud_right, mud_curve PASS 2026-10-26; mud_stop does not drive off (explained) | 10 scenarios x 4 | Isaac |
| 20 | `road_physics` example (after_rain, mud_and_gravel) | 076 | Isaac | Autoware | project only | as built | partly (never live) | none yet | 2 runs | Isaac |
| 21 | Rough road on Town01 (`road_detail.yaml`) | 066 | Isaac | Autoware (not validated) | spikes `rough_road.py`, `rock_world.py` | `ground.road_detail` | partly (key exists; not validated under Autoware; 066 in progress) | 066 R5 fixtures within 4 % of 4 kHz reference | | Isaac |
| 22 | NTU twin on Isaac: `ntu_twin_r01`, `_r02`, `_r01_lead`, `_r01_pedestrian`, 114 `ntu_twin_tour_NN` | 059 | Isaac (NTU twin world) | Autoware (stock, no overlay) | `just isaac-world world:=<twin>`, `just scenario`, per-leg chase/front recording (P6), `tour_monitor.py` | private asset (`ntu-campus-maps`, `SWB_NTU_TWIN` -> `path:` or token), recording, tools | partly (G4, G8) | 116 of 116 legs, 10.0 km, no MRM 2026-11-05 (118; leg 113 GPU OOM rerun) | 118 runs, hours | Isaac, 32 GB VRAM |
| 23 | NTU open-data roads `scenarios/ntu/` (campus_loop, loop_signals, gate_xinsheng, left_turns) | 062 | Isaac (generated world) | Autoware | `SWB_OPEN_MAPS`, `just scenario sim:=isaac` | generated asset (`open_maps/generate.py`), signals cycling on city plans | partly (G8: `path:` works, no `generate:`) | O6 first drive 2026-10-30, loop 2026-10-31, 15 km/h both ways (118) | 4 runs, 2.2 km loop | Isaac |
| 24 | Haruna `haruna_down.xosc` through swb, Tsukuba | 058, 060 | Isaac | Autoware low speed, then race stack | world server + scenario; the drift itself in `course_probe` | dataset `haruna-tsukuba-courses` (yes), `drift_mustang` ego, `launch` pilot, `frame_rate`, `physics_hz` | partly (G3, G6) | 058 C9: NDT NVTL >= 2.5, low-speed run | 1 run | Isaac |
| 25 | Town05 ring drift lap (autoware-drift `town05_ring_lap.xosc`, drift controller as `extra_launch`) | 057 | Isaac | Autoware + drift_controller, `drift_mustang` | `just scenario ... sim:=isaac` with `frame_rate` 10 ms, `physics_hz` 240 | `autoware_launch`/`launch` pilot, SSv2 frame rate, Isaac physics rate | no (G3, G6) | lap 32: full lap, one drift, no contact 2026-10-28 (241) | 1 lap, about 1.5 km | Isaac |
| 26 | Vision-only speed ladder (rig_localization, `ndt:=none`, ground-truth pose seed, rungs 10-30 m/s) | 063 | Isaac (Town05 ring) | Autoware + rig_localization | `just scenario town05_ring_lap sim:=isaac`, `ego_rec.yaml`, rosbag per rung, `ladder_score.py` | per-run Autoware args (`max_velocity` per rung = `lifetime: run` + parameters), lockstep, ego config, rosbag topics | partly (G6 ego config and lockstep, G12 profile args checked) | planned; step 4 recordings 2026-11-04 | 5 rungs | Isaac |
| 27 | Shalun map (CAR Lab) | 064 | CARLA 0.9.16 (imported package), Isaac (not rendered) | none yet (driven in CARLA by hand) | CARLA package installed; `just isaac-world world:=` | custom CARLA map in the binary (G13), private dataset `shalun-map` | no (no scenario, no Autoware run) | CARLA: imported, 79 spawn points, 28 lights 2026-10-30 (241) | | GPU; Isaac with free VRAM |
| 28 | Golf cart Isaac twin (`golfcart_twin twin.launch.py route:=N`, includes `isaac.launch.xml`) | 073 | Isaac (NTU twin) | golfcart Autoware (`golfcart_launch`, clustering + CenterPoint, speed cap) | in NEWSLabNTU/2026-golf-cart | package project, own car (`vehicles:`), own Autoware launch, own DDS profile, routes/free drive, `gpu` | no (G5, G14, G15) | 073 S1-S9 done 2026-11-08 | 4 routes | Isaac |
| 29 | Several Autoware egos on Isaac, V2X | 039 | Isaac | 2+ Autoware | not built | per-entity rigs, a ROS context per ego | no (076 ruling: one ego on Isaac) | none | | Isaac |
| 30 | NTU off-road field, competition director | 040, 041 | Isaac | teams' Autoware | not started (vendor mesh) | heats, scoring from trace | no | none | | Isaac |

### 1c. Studies (driver scripts)

| # | Item | Roadmap | Sim | Autopilot | Runs today | New workflow needs | Exists | Pass criterion; reference | Size, time | Host |
|---|---|---|---|---|---|---|---|---|---|---|
| 31 | Cut-in batch C3a (ground-truth objects) and C3b (CenterPoint) | 074 | CARLA 0.9.16 Town03 | Autoware (`max_velocity` 6, `publish_ground_truth_objects`, laserscan grid / centerpoint) + `cutin_agent.py` (SDK) | `OUT=.. scripts/coop/c3_run.sh` (own CARLA on 2400, GPU pick, speed gate, `c3.py`, retries x3, `.failed1`, SEED0 split 118/241, `--rescore`) | `command` pilot with `{param:}`/`{seed}`, `lifetime: run` Autoware, cells as suite parameters (needs a parameterized xosc, c3 generates per cell), rosbag of 6 topics in the ego domain, gate run, shards on two hosts | partly (G16 gate, G17 two-host merge, G2 trace) | C3a 4/6 cells agree with the arena 2026-11-14; C3b same classes, +0.1-0.45 s reaction; C3a rerun 2026-11-14 | 24 runs per batch | GPU (CUDA for C3b), 2 hosts |
| 32 | C2b swb check of the arena (cut-in light policies through swb) | 074 | CARLA 0.9.16 Town03 | 2 SDK agents (`cutin_agent`), no Autoware | `c2b.py` over swb runs | 2 `command` pilots, per-run parameters | partly (as 31) | outcomes 24/24, world-frame payoffs 42/48 (88 %) 2026-11-15 (241) | 24 runs | GPU |
| 33 | Cut-out S1 on the real stack | 074 | CARLA 0.9.16 | Autoware centerpoint + `cutout_agent.py` | `scripts/coop/s1_run.sh` (`s1_aw.py`) | as 31 | partly (G1, G16) | in progress | | GPU with CUDA |
| 34 | E2 lane change gate (`scenarios/coop/town03_e2_lane_change_west`, `_lead_brake`) | 074 | CARLA 0.9.16 Town03 | Autoware | `gen_e2_scenarios.py`, `e2_bag.py`, `just scenario` | rosbag `/localization/kinematic_state`, a speed judge | partly (`capture: full` + `topics`, judge to write; CARLA_MAPS) | lane change 4 s + 4.8 s 2026-11-13 | 2 runs | GPU |
| 35 | E3 Autoware knobs (13 settings, obstacle_cruise param file) | 074 | CARLA 0.9.16 | Autoware with param-file overrides | `scripts/coop/e3_knobs.sh`, fresh stack per setting | per-pilot ROS parameter overrides (load-time, not launch args) | no (G18) | stock vs yield moves uB by -0.24 at (10, 0) 2026-11-15 (118) | 13 stacks x cells | GPU |
| 36 | E5/S5 left turn at junction 861 on the real stack | 074 | CARLA 0.9.16 Town03 | Autoware + SDK agent | 118's scripts, in progress | as 31 | partly | in progress (Town03 lanelets link no lights) | | GPU |
| 37 | Gap campaign live loop (generate, run, trace, oracle, confirm, bisect, shrink) | 025 | CARLA 0.9.16, then Isaac | Autoware | `gap_campaign.py`/`gap_search.py`/`gap_exploit.py` offline; live loop `scripts/gap_live.py` since 2026-11-16 | `run_one` (yes), trace per run (G2), infra vs failed (yes) | partly (G2) | offline tests 2026-10-25; rediscovery of truck_pass bucket is the first live reference | a night, hundreds of runs | GPU (Isaac 16 GB later) |
| 38 | Live race planner, grip-or-drift, styles, route generalisation (`course_probe` in Isaac, L1 IPOPT process, model-in-loop studies) | 060, 061, 065, 071 | Isaac (no SSv2, no ROS) | race_planner L1/L2 (autoware-drift) | `play_log/race_planner/live.sh <tag>`, `course_probe LIVE=1`, sweeps `sweep*.sh` | harness mode: project's assets and car resolved without the stack | no (G19) | 060: Haruna 77.38 s vs L0 74.74 (+3.5 %), Tsukuba 72.64 vs 71.05, no contact 2026-10-30 | minutes per lap | Isaac |
| 39 | Hairpin drift H1 (`course_probe`, drift law) | 058 C9+ | Isaac | drift_controller law in-process | `sweep2.sh` | as 38 | no (G19) | first H1 drift without Autoware 2026-10-29 | | Isaac |
| 40 | Arenas (cut-in C2a, crossing, cut-out, merge, narrow, overtake, roundabout, left turn, zipper, tiebreak, c6) | 074 | CARLA 0.9.16 without SSv2 | light policies | `scripts/coop/*_arena.py` (K pairs per CARLA, ports 2600-2620) | none (stays a harness); project env for maps | no (G19), by design | C2a grid and double oracle done 2026-11-13 to 2026-11-15 | about 1000 runs/h target | GPU |
| 41 | Kinematic games and fits (`*_game.py`, `double_oracle.py`, `n1`-`n4`, `c3_fit.py`, `s0_rescore.py`) | 074 | none | models | `python3 scripts/coop/<x>.py` | none (T0 tests) | n/a | C1 2026-11-13, S0-S6, U1-U2 2026-11-14/15 | seconds to minutes | any |
| 42 | Coop atlas (`just coop-atlas`, `atlas.py`) | 074 | none | none | `just coop-atlas out:=` | none; T0 `--selftest` | n/a | atlas.html builds | seconds | any |
| 43 | Soil calibration, manoeuvre suite, step responses, multi-ego soil, tire curves (`spikes/*.py`) | 052, 053, 054, 055 | Isaac without SSv2 | physics egos, ECU | `src/swb_isaac_world/spikes/run_spike.sh <spike>`, `aib just isaac-steptest` | harness mode | no (G19) | 054: FMVSS 126 sine with dwell passes with ESC; 052: 4 egos on soil 77 ms/frame | minutes each | Isaac |
| 44 | 13-surface proving ground (`isaac-road-physics-stages`, `proving_ground.yaml`) | 047, 075 | Isaac | scripted | demo script only | a lanelet map, then a project | no (no lanelet map) | materials video (243) | | Isaac |
| 45 | CARLA realism (CARLA2Real, CMMD) | (2026-FastSlowRR) | CARLA 0.9.16 | none | uncommitted scripts | none | n/a | deferred 2026-11-07 | | GPU |

### 1d. Bring-up, vehicle checks and developer recipes

| # | Item | Roadmap | Sim | Autopilot | Runs today | New workflow needs | Exists | Pass criterion; reference | Size, time | Host |
|---|---|---|---|---|---|---|---|---|---|---|
| 46 | Autoware on CARLA without SSv2 (acb `just sim`, `just autoware map:=`) | 051 | CARLA 0.9.16 | Autoware | acb's justfile | free drive with a spawn and a goal | no (G14) | 051: drives stand-alone 2026-10-24 | minutes | GPU |
| 47 | acb demo pilot (`just pilot <poses>`, `scenarios/ego_poses.yaml`, `bg_av_1_poses.yaml`) | 017 | CARLA 0.9.16 | Autoware via acb_pilot goals | acb `just pilot` | routes/goals (G14) | no | goal reached | minutes | GPU |
| 48 | Autoware on Isaac without SSv2 (aib `just isaac-ego`, `just autoware map_path:=`) | 051, 059 | Isaac | Autoware | aib's justfile | as 46 | no (G14) | 059 N8 drives 2026-10-30 | minutes | Isaac |
| 49 | `vehicle-check` (acb, aib): the car's contract and dynamics | 051, 055 | CARLA 0.9.16; Isaac | none (scripted inputs) | acb/aib `just vehicle-check [vehicle_model:=]` | harness (T1/T2 cell per car) | no (G19), stays a harness | spec contract pass; acb longitudinal 0.051 m/s^2 vs 0.35 limit after rebuild | minutes | GPU; Isaac |
| 50 | `just acceptance <xosc> N [sim:=]` (acb `acceptance.py`: tracking limits from a bag) | 038, 043, 051 | CARLA, Isaac | Autoware | justfile | `repetitions`, `capture: full` (bag), acceptance as a judge | partly (G11) | tracking under limits (0.35 m/s^2 longitudinal); 038 Isaac beside CARLA | N runs | GPU; Isaac |
| 51 | `just e2e [xosc] sim:=carla|isaac` | 043, 046, 051 | CARLA, Isaac | Autoware | justfile (starts and stops what it started) | `swb run --stop-stack` | yes on CARLA (076 nightly); partly on Isaac (not live) | PASS both sims 2026-10-24 | 1 run plus bring-up | GPU; Isaac |
| 52 | `just run` long-lived dev stack (`carla.launch.xml`, `isaac.launch.xml`, `ego_launch`) | 073 | CARLA, Isaac | Autoware default | justfile | `swb.launch.xml` | yes (superseded on the user path; kept for developers) | 073 done 2026-11-08 | | GPU |
| 53 | `just ego-av`, `ego-isaac`, `bg-av` (separate ego and background AV stacks) | 017, 042 | CARLA, Isaac | Autoware (+ overlay, TensorRT 10.8, cuda NDT) | justfile | `autoware` pilot; overlay/TRT/cuda NDT | partly (G1) | ego ready (`ego_ready`) | | GPU |
| 54 | `just carla-start map:= quality:= render:=` | 018, 046 | CARLA 0.9.16 | | justfile | `simulator_start: auto|start` (Epic only) | partly (boot map, render mode, extra args not settable) | RPC up in about 50 s | | GPU |
| 55 | `just isaac-world ego:= world:= follow:= render:= record:= physics_hz:=` | 037, 043, 047 | Isaac | | justfile | Isaac options in the project | partly (G6: `stack_args` only) | world server ready | about 7 min preload for Town01 | Isaac |
| 56 | `just world-health`, `world-check` (`swb_world_check conformance`) | 046 | CARLA, Isaac | | justfile, `ros2 run swb_world_check` | T1 cell per world server | yes (tool) | conformance pass | seconds | sim host |
| 57 | Two checkouts on one host (moved `.env` sets) | 010, 017 | CARLA | Autoware | `.env` ports and domains | seats (`seats.carla: {base, stride}` since dbe69d9) | yes on one host; two hosts partly (G17) | 076 acceptance 3: 5/5 sharded on one host 2026-11-15 | | GPU |
| 58 | CARLA 0.10.0 `scenarios/carla0100/town10_ego_drive` (lean profile, port 2100, mkz configs) | 019, 068 | CARLA 0.10.0 | Autoware | worktree `~/repos/csb-carla0100`, branch `carla-0100` | 0.10 on main, a simulator version axis | no (G20) | 019 runs on Town10 (0.10 profile README on /mnt/disk1) | 1 run | GPU |
| 59 | NTU twin in CARLA: `ntu_carla_r01`, `_r02`, `_r01_lead`, `_r01_pedestrian`, 114 `ntu_carla_tour_NN` | 072 | CARLA 0.9.16 + NTU package installed | Autoware | `CARLA_PORT=2400 just scenario scenarios/ntu_twin/ntu_carla_r01.xosc`, `tour_monitor.py`, videos | custom map in the CARLA binary (G13), private map asset, recording | no (G13, G8, G4) | r01 3 passes; 116 of 116 legs 2026-11-05 (133) | 118 runs, hours | GPU, 133 setup |

### 1e. Tools a run uses

| # | Item | Roadmap | Runs today | New workflow | Exists | Check |
|---|---|---|---|---|---|---|
| 60 | `run_suite` (restart ladder, relay pre-flight, `--physical-verdict`) | 020, 021, 030 | `ros2 run swb_launch run_suite <dir>` | `ros2 run swb run suite.yaml` (`retry`, `lifetime: run`) | yes (replaced) | T0 tests |
| 61 | `physical_verdict` | 021 | `run_suite --physical-verdict` | judge (`safety_suite`) | yes (time window on the default contacts dir, not a per-run dir) | 2026-11-15 live |
| 62 | gap trace + `gap_oracle` | 024 | bridge `gap_trace_dir`, `ros2 run swb_launch gap_oracle` | trace into the run dir, oracle judge | no (G2) | 024 baseline 2026-10-13 |
| 63 | `incident_judge` | 033 | on the tour's trace | judge | no (G2) | 6/6 2026-10-21 |
| 64 | `chase_view`, `demo_captions`, `make_showreel.sh`, `vnc_layout.sh`, Isaac `record_video.py` | 027, 033, 043 | recipes start them | per-run `tools:` and video | no (G4) | the take of 2026-10-21 |
| 65 | `carla_poses` (authoring) | 027 | `ros2 run swb_launch carla_poses` | stand-alone against CARLA | yes | |
| 66 | `ego_ready` | 070 | `ros2 run swb_launch ego_ready` | `ready:` default for autoware | yes | |
| 67 | `swb-agent check/new/supervise`, `ros_adapter`, `skeleton` | 030 | `ros2 run swb_agent swb-agent` | `command`/`launch` pilots | yes (check), partly (`ros_adapter` never a pilot) | 16/16 on fake Autoware (030) |
| 68 | Probes (perception, tracking, traffic light, lead vehicle, stall, clock, steering, pose offset, entity perception, classifier, roi_capture) | 009-034 | `python3 scripts/<probe>.py` in a domain | `ros2 run swb seat env <group>` then the probe | yes (by hand) | |
| 69 | Weather services (`set_weather`, `get_weather`) | 018 | `ros2 service call` in domain 9; `just run weather:=` | `seat env scenario` + call; nothing in swb.yaml | partly (G10) | |
| 70 | Map and asset builders: `download_maps.sh`, `repair_lanelet_*`, `generate_signal_groups.py`, `densify_pcd_along_lanes.py`, `pcd-gen`, acb `generate-map`, `open_maps/`, `ntu_twin/`, `ntu_carla/`, `svl_import/`, `isaac-town`, `isaac-vehicles`, `isaac-world-gen`, `isaac-assets`, `swb_isaac_world.course`, `make_world` | 019, 031, 043, 058, 059, 062, 064, 072 | repository scripts | `assets` source `generate:` | no (G8); outputs published as datasets instead (075) | `assets check` against MANIFEST |
| 71 | `carla_bench.py`, `stall_probe.py` (launch config cost, stalls) | 016, 014 | script | stand-alone | yes | |
| 72 | `prune-logs` | 028 | `just prune-logs` | `ros2 run swb runs prune` | yes | |
| 73 | `check_scenario_vehicles.py` (050 lint) | 050 | `just test` | `project check` catalog geometry | yes | |

Row count: 73.

## 2. Autopilots and simulators

### 2a. Autopilots

| Autopilot | Lives in | Used by | Pilot kind today | Carried today? |
|---|---|---|---|---|
| Autoware 1.5.0, default profile (clustering, CPU NDT) | `/opt/autoware/1.5.0`, `autoware_sim_vehicle` (`autoware_sim_profile`), the vehicle agent in `swb_agent` | 1-8, 12-14, 16, 19, 31, 34-36 | `autoware` | yes |
| + CenterPoint (`lidar_detection_model:=centerpoint`) | same, TensorRT 10.8 runtime (`scripts/autoware_tensorrt.bash`) | 5, 9, 11, 17, 31 C3b, 33 | `autoware` + `args` | partly: engines rebuilt each start without 10.8 first on `LD_LIBRARY_PATH` (G1) |
| + Autoware overlay (blind_spot, pose_instability, ndt step fix) | `third_party/autoware_overlay` (042) | 9, 11, 17 (i6), every Autoware run (NDT fix, pose_instability crash) | none | no (G1) |
| + CUDA NDT (`ndt:=cuda`) | `third_party/cuda_ndt_matcher` (NEWSLabNTU) | ego recipes on request, 058 | `args` | no: not sourced (G1) |
| + ground-truth objects (`publish_ground_truth_objects:=true`, laserscan grid) | acb | 31 C3a, 35 | `args` | yes |
| + param-file knobs (obstacle_cruise) | Autoware's param YAML | 35 | none | no (G18) |
| + `ndt:=none` with ground-truth pose, rig_localization (vision-only) | aib `ground_truth.pose_estimate`, rig_localization | 26 | `args` + ego config | partly (G6) |
| + drift controller (`extra_launch`, `drift_mustang`) | jerry73204/autoware-drift | 25 | `autoware_launch` or `launch` | partly (G3, G6) |
| golf cart Autoware (`golfcart_launch`) | NEWSLabNTU/2026-golf-cart | 28 | `autoware` + `autoware_launch` | no (G5, G15) |
| race_planner L1/L2, drift law | autoware-drift, in `course_probe` | 38, 39 | none (no ROS, in-process) | no (G19, by design) |
| SDK agents: `cutin_agent.py`, `cutout_agent.py`, multi_pilot `cut_in.py`, `skeleton` | `scripts/coop/`, `src/swb_examples/projects/multi_pilot/pilots/`, `swb_agent` | 31-33, 36, multi_pilot | `command` | yes (`--seed`, `--run-dir`, `{param:}`) |
| `ros_adapter` (YAML-configured ROS autopilot) | `swb_agent` | 030 docs | `launch`/`command` | partly: never run as a pilot |
| CARLA BasicAgent | CARLA's `agents`, `swb_agent.adapters.carla_basic` | multi_pilot `lead` | `carla_basic` (CARLA only) | yes (2026-11-15) |
| acb_pilot goal sender | acb | 47 | none | no (G14) |
| simple_pilot (pure pursuit, speed PID) | 070, not built | Isaac baseline in 076 study | `simple` accepted, no launch | no |
| A non-Autoware autopilot on the vehicle interface | none (077 step 5) | matrix | `launch` with `agent: own` | no (documented, never run) |
| CARLA Traffic Manager autopilot for an NPC | CARLA, via the bridge | 1 (`town01_simulator_autopilot`) | scenario's controller | yes |
| none (SSv2 drives or no ego) | SSv2 | 3, 13, 18 | `none` / no pilot | partly (18 needs `managed_ego:=false`, G3) |
| Arena light policies | `scripts/coop/arena*.py` | 40 | none | no (harness) |
| Vision Pilot, VLM agents, competition teams | not built | 059 aim, 041 | `launch` | no |

### 2b. Simulators

| Simulator | Lives in | Worlds | Carried today? |
|---|---|---|---|
| CARLA 0.9.16 (release binary, Epic) | `swb_carla_world` (carla 0.15.1, feature `carla-0916`), acb | Town01, 02, 03, 05, 10HD with Autoware maps (`carla-town-autoware-maps`) | yes: `simulator: carla`, `simulator_start: auto|start|external` |
| CARLA 0.9.16 with an installed map package (NTU 072, Shalun 064) | the binary on 133 / 241 (ImportAssets) | NTU campus, Shalun | no (G13) |
| CARLA 0.10.0 (lean profile, port 2100) | worktree `~/repos/csb-carla0100`, branch `carla-0100`, acb origin `carla-0100` | Town10 | no (G20) |
| CARLA 0.9.14, 0.9.15 | build-verified only (027) | | no (untested tier) |
| Isaac Sim 6.1 | `swb_isaac_world`, aib, `sim_vehicle_ecu` | CARLA towns as USD (`carla-0.9.16-isaac-usd`), surfaces and road detail (`isaac-road-physics-stages`), Haruna, Tsukuba (`haruna-tsukuba-courses`), NTU twin (`ntu-campus-maps`, private), NTU open-data roads (generated), Shalun (`shalun-map`, private, not rendered), Town05 ring | partly: one ego, one world per stack, Isaac options only through `stack_args` (G6, G7) |
| A user's simulator package (`launch/world.launch.xml`) | 073 | | partly: `stack.launch.py` takes it, `project check` allows a name; no test implementation |
| Chrono (rally raid) | paused, `~/chrono-probe/` | | no (out of scope) |

## 3. Gaps in the project workflow

| Gap | What the old path has | Blocks |
|---|---|---|
| G1 Autoware environment | ego recipes source the overlay (042), put TensorRT 10.8 first on `LD_LIBRARY_PATH`, source cuda NDT; the stack's pilots do none | 5, 9, 11, 17, 31 C3b, 33, 53; every Autoware run loses the NDT step and pose_instability fixes |
| G2 Gap trace per run | bridge `gap_trace_dir`; `gap_oracle`, `incident_judge` read it | 8-12, 14, 17, 31, 37, 62, 63 |
| G3 SSv2 run settings | `managed_ego:=false`, `frame_rate` (100 for a 10 ms step), `global_timeout` per scenario | 18, 24, 25 |
| G4 Recording and tools | screencast (`record:=true`), chase view, captions, RViz config, per-leg chase/front video (059 P6), Isaac `record:=` | 7, 11, 22, 59, 64 |
| G5 Own cars | `vehicle_catalogs`, `spec_hash_file`, `ego:=` rig, `vehicle_model` launch args; `vehicles:` is checked, not loaded | 28; any car outside `sim_vehicle_spec` |
| G6 Isaac options | `ego:=` config (lockstep, ground-truth pose, camera), `render`, `follow`, `physics_hz`, `worlds` preload | 24-26, 55 |
| G7 Ground per scenario | each surfaces scenario has its own surfaces yaml (056 planned); `ground:` is per map | 19 |
| G8 Asset sources | generated worlds (course builder, open maps, NTU), private datasets (`ntu-campus-maps`, `shalun-map`) need a token; `path:` exists | 22, 23, 27, 59, 70 |
| G9 Town preload | the stack switches maps by restarting pilots; no `~/load_map`, so one extra episode per switch | 2, 3, mixed-town suites |
| G10 World config | `weather:=`, `npc_motion:=`, `contact_report_dir:=`, `config_file:=` as side args; none in `swb.yaml`; `stack_args` side_args would replace the stack's `state_dir` | 11, 12, 69, ClearNoon demo |
| G11 Acceptance judge | acb `acceptance.py` (tracking limits from a bag) | 19, 50 |
| G12 Profile arguments | `check_launch_args` refusals (e.g. `drift.yaml` under `ndt:=none`); a project's `args` reach them only at pilot start | 26 |
| G13 CARLA map packages | a map installed into the CARLA binary is a host fact, not an asset | 27, 59 |
| G14 Free drive, spawn, routes, goals | `spawn:=` (073), acb `just sim`/`pilot`, `twin_go.py N` | 28, 46-48 |
| G15 Own DDS profile, package project | golf cart's `cyclonedds-unicast-lo.xml`; `pkg://` reads a project, its state goes where? | 28 |
| G16 Gate before a batch | c3's speed check on the lane change, no batch unless the ego holds v_B | 31, 33 |
| G17 Two hosts | shard links name local paths; merge needs copying first | 31, 57 |
| G18 Per-pilot ROS parameter overrides | knobs in obstacle_cruise's param file (load time) | 35 |
| G19 Harness mode | `course_probe`, arenas, spikes, `vehicle-check` need the project's maps, car, rig without the stack (076 C6 `project env`, not built) | 38-40, 43, 49 |
| G20 CARLA 0.10 | branch only, not on main (068 step 8) | 58 |
| G21 Two SSv2 clients | a stray SSv2 joins a run (no Initialize client id) | every live run (robustness) |
| G22 `simple` kind | accepted by `project check`, no launch | Isaac baseline, 070 starter |
| G23 Bench metrics | frame time p95 only in the bridge log, not in `result.json` | 13 |
| G24 Developer scenarios on `CARLA_MAPS` | `scenarios/` bench, gap, surfaces, coop, carla0100 name `$(env CARLA_MAPS)`; `project check` refuses them; ntu/ntu_twin name `SWB_OPEN_MAPS`, `SWB_NTU_TWIN` | 12-14, 19, 22, 23, 34, 58, 59 |

## 4. Status hints

Tested as they stand, not reworked:

- **On hold since 2026-11-02 (user)**: 061 grip-or-drift, 065 drift style, 071 route
  generalisation (row 38). State at hold: 061 chain 10/30, 065 S2 adopted, 071 planned only.
- **Another session's**: the hairpin drift (058 C9+, rows 24, 39) is swb 243's since
  2026-10-29; the rig and golf cart (rig_localization, 063's rig parts, row 28) since
  2026-10-29; 057's ring lap (row 25) was 241's. The Isaac live acceptance of 076
  (`isaac_first_drive`, row 16) and the second host of the shard check (row 57) are swb 241's.
  074 E5/S5 (row 36) and E3 (row 35) are swb 118's; C2b (row 32) swb 241's.
- **Deferred**: 031 maps for scenarios (2026-10-13); the realism experiment (row 45,
  2026-11-07); 029 folded into 070 (planned, no code).
- **Planned, nothing to migrate yet**: 039, 040, 041 (rows 29, 30), 056 (row 19's per-scenario
  ground), 063 (row 26, recordings only), 068 (row 15 and spectator, code only), 069, 070
  (simple_pilot).
- **In progress elsewhere, test the last good state**: 052 (NPC sinking in the ego's sensors
  open), 053 (dirt waits for data), 059 and 072 (row 22, 59: 116/116 on 2026-11-05), 062 (row
  23), 064 (row 27), 066 (row 21), 074.

## 4. Step 4 hand-out (2026-11-16)

For handing step 4 out by host. "Host" as section 1 says it, reduced to what decides who
can run an item: CARLA (one GPU host), CARLA + CUDA, two Autoware stacks, Isaac, two
hosts, or private assets (NTU, Shalun, the golf cart: the user checks those with their
own configuration, so they are not handed out). "Open gaps" are the item's gaps not
built by this date; built: G1, G2, G3, G5 (CARLA), G6, G9, G10, G12, G14, G15, G21 to
G32. An item with no open gap can be migrated now.

| # | Item | Host | Open gaps | State |
|---|---|---|---|---|
| 1 | `basic/` starters, Town01 | CARLA | - | migrated 2026-11-16 (`scenarios/projects/basic`, suites/town01, 6 of 6) |
| 2 | `basic/` other towns: town02_ego_drive, town03, town05, town10 | CARLA | - | migrated 2026-11-16 (suites/towns, 4 of 4) |
| 3 | `basic/town02_episode_change` | Epic only (Low crashes) | - | migrated 2026-11-16 (suites/episode_change, passed) |
| 4 | `awf/` UC-ACC-001-0001 | CARLA | - | migrated 2026-11-15 (`safety_suite` nightly, 27/27 and 9/9) |
| 5 | `awf/` with CenterPoint | CARLA + CUDA | - | migrated 2026-11-16 (safety_suite centerpoint, 27/27 and 9/9) |
| 6 | `tutorial/town01_follow_to_goal` | CARLA | - | migrated 2026-11-16 (`scenarios/projects/basic` suites/tutorial) |
| 7 | `multi_av/town01_two_av` | CARLA, two Autoware | G4 (recording only) | migrated 2026-11-16 without recording (swb 118, `scenarios/projects/two_av`, 7c277ee; `just two-av`) |
| 8 | Demo incidents i1 to i5 | CARLA + CUDA | - | migrated 2026-11-16 (`scenarios/projects/demo` suites/incidents, 5 of 5 on SSv2, incident_judge and physical_verdict; the tour's ego, as the incidents are timed for since 2026-10-20) |
| 9 | Demo incident i6 | CARLA + CUDA | - | migrated 2026-11-16 (suites/i6_centerpoint, 3 of 3, the ego yielding) |
| 10 | Demo incidents i7 cut-in, i8 rear approach | CARLA + CUDA | - | migrated 2026-11-16 (suites/cut_in_rear; first live runs: i8 passed, i7 a contact, see 077) |
| 11 | Demo tour `demo/town01_tour` | CARLA | G4 (recording only) | migrated 2026-11-16 without recording (`scenarios/projects/demo`, suites/tour, d5c9843; `just demo-tour`) |
| 12 | `scenarios/gap/` entity_bodies, motorbike_body, npc_shoulder, ... | CARLA | - | migrated 2026-11-16 (scenarios/projects/gap, suite gap, 8 of 8 as expected) |
| 13 | `scenarios/bench/` town01_npc_10/20/50 | CARLA | - | migrated 2026-11-16 (swb 118, scenarios/projects/bench, 3 of 3 with a frame_budget judge) |
| 14 | 050 measurement set | CARLA | - | migrated 2026-11-16 (swb 118, scenarios/projects/set050 and set050_tour; the tour's static-mesh contact open, see 077) |
| 15 | Free-standing scenario on upstream SSv2 | CARLA | not built (068 step 9: upstream SSv2) | planned |
| 16 | `basic/town01_ego_drive`, town02_ego_drive on Isaac | Isaac | - | migrated 2026-11-16 (`scenarios/projects/isaac_basic` suites/towns, 2 of 2, 542a50d) |
| 17 | Incidents 1-6 and UC-ACC on Isaac | Isaac | G74 | migrated 2026-11-17 in part (swb 241, `scenarios/projects/isaac_demo`, 12d284d: i1 to i5 5 of 5; i6 an Isaac-only known limit, out of T3; UC-ACC open) |
| 18 | `isaac/town01_rear_contact` | Isaac | - | migrated 2026-11-17 (swb 241, isaac_basic suites/rear_contact, 70df975; T3 part 2) |
| 19 | `scenarios/surfaces/` snow, soil, mud, wet, weather, zones, ... | Isaac | G7, G11 | blocked |
| 20 | `road_physics` example | Isaac | - | ready to migrate |
| 21 | Rough road on Town01 | Isaac | - | migrated 2026-11-17 (swb 241, `scenarios/projects/isaac_rough_road` suites/rough, 2 of 2 after issue 042, e3c29e9; T3 part 2) |
| 22 | NTU twin on Isaac: `ntu_twin_r01`, `_r02`, `_r01_lead`, ... | private assets (the user checks these, 077 ruling); Isaac | G4, G8 | blocked |
| 23 | NTU open-data roads `scenarios/ntu/` | private assets (the user checks these, 077 ruling); Isaac | G8 | blocked |
| 24 | Haruna `haruna_down.xosc` through swb, Tsukuba | Isaac | - | ready to migrate |
| 25 | Town05 ring drift lap | Isaac | - | ready to migrate |
| 26 | Vision-only speed ladder | Isaac | - | ready to migrate |
| 27 | Shalun map | private assets (the user checks these, 077 ruling); Isaac | no scenario, no Autoware run yet | blocked (no scenario) |
| 28 | Golf cart Isaac twin | private assets (the user checks these, 077 ruling); Isaac | G5 (Isaac rig) | blocked |
| 29 | Several Autoware egos on Isaac, V2X | Isaac | one ego on Isaac (076 ruling) | by design |
| 30 | NTU off-road field, competition director | private assets (the user checks these, 077 ruling); Isaac | not built | blocked |
| 31 | Cut-in batch C3a | two hosts; CARLA + CUDA | G16, G17 | blocked |
| 32 | C2b swb check of the arena | CARLA | - | migrated 2026-11-16 (swb 118, `scenarios/projects/c2b`, a9be59a; a developer project) |
| 33 | Cut-out S1 on the real stack | CARLA + CUDA | G16 | blocked |
| 34 | E2 lane change gate | CARLA | - | migrated 2026-11-16 (swb 118, `scenarios/projects/e2`, 1adab31) |
| 35 | E3 Autoware knobs | CARLA | G18 | blocked |
| 36 | E5/S5 left turn at junction 861 on the real stack | CARLA | - | ready to migrate |
| 37 | Gap campaign live loop | CARLA (Isaac later) | a blind-follow gene | live loop migrated 2026-11-16 (swb 118, `scenarios/projects/gap_campaign`, f303654); rediscovery and exploit cannot reach C1 in the incident templates (025 steps 6, 7) |
| 38 | Live race planner, grip-or-drift, styles, route generalisation | Isaac | G19 | blocked |
| 39 | Hairpin drift H1 | Isaac | G19 | blocked |
| 40 | Arenas | CARLA | G19 | blocked |
| 41 | Kinematic games and fits | any | - | no simulator; nothing to migrate |
| 42 | Coop atlas | any | - | no simulator; nothing to migrate |
| 43 | Soil calibration, manoeuvre suite, step responses, multi-ego soil, ... | Isaac | G19 | blocked |
| 44 | 13-surface proving ground | Isaac | no lanelet map | blocked |
| 45 | CARLA realism | CARLA | - | a study of CARLA images; nothing to migrate |
| 46 | Autoware on CARLA without SSv2 | CARLA | - | migrated 2026-11-16 (`swb drive`, CARLA, simple and Autoware) |
| 47 | acb demo pilot | CARLA | - | migrated 2026-11-16 (`swb drive --poses`, CARLA, Autoware) |
| 48 | Autoware on Isaac without SSv2 | Isaac | - | migrated 2026-11-16 (`swb drive`, Isaac, simple and Autoware) |
| 49 | `vehicle-check` | CARLA; Isaac | G19 | blocked |
| 50 | `just acceptance <xosc> N [sim:=]` | Isaac | G11 | blocked |
| 51 | `just e2e [xosc]` on Isaac | Isaac | - | checked 2026-11-17 (swb 241: town01_ego_drive with `project:=scenarios/projects/isaac_basic` passed in 6 min 9 s, the stack it started stopped, nothing left) |
| 52 | `just run` long-lived dev stack | CARLA | - | checked 2026-11-16 on CARLA (swb 118, 2788710): kept for developers, works with G55, G58, G59; Isaac half 241's |
| 53 | `just ego-av`, `ego-isaac`, `bg-av` | CARLA | - | ready to migrate |
| 54 | `just carla-start map:= quality:= render:=` | CARLA | - | ready to migrate |
| 55 | `just isaac-world ego:= world:= follow:= render:= record:= ... | Isaac | - | ready to migrate |
| 56 | `just world-health`, `world-check` | sim host | - | migrated 2026-11-17 (swb 118 and 241, `scripts/test_world_conformance.py`: CARLA 28/0, Isaac 25/0) |
| 57 | Two checkouts on one host | two hosts; CARLA | G17 | blocked |
| 58 | CARLA 0.10.0 `scenarios/carla0100/town10_ego_drive` | CARLA | G20 | blocked |
| 59 | NTU twin in CARLA: `ntu_carla_r01`, `_r02`, `_r01_lead`, ... | private assets (the user checks these, 077 ruling); CARLA | G4, G8, G13 | blocked |
| 60 | `run_suite` | T0 tests | - | tool or harness |
| 61 | `physical_verdict` | 2026-11-15 live | - | tool or harness |
| 62 | gap trace + `gap_oracle` | 024 baseline 2026-10-13 | - | tool or harness |
| 63 | `incident_judge` | 6/6 2026-10-21 | - | tool or harness |
| 64 | `chase_view`, `demo_captions`, `make_showreel.sh`, ... | the take of 2026-10-21 | - | tool or harness |
| 65 | `carla_poses` | any | - | tool or harness |
| 66 | `ego_ready` | any | - | tool or harness |
| 67 | `swb-agent check/new/supervise`, `ros_adapter`, `skeleton` | 16/16 on fake Autoware (030) | - | tool or harness |
| 68 | Probes | any | - | tool or harness |
| 69 | Weather services | any | - | tool or harness |
| 70 | Map and asset builders: `download_maps.sh`, `repair_lanelet_*`, ... | private assets (the user checks these, 077 ruling) | - | tool or harness |
| 71 | `carla_bench.py`, `stall_probe.py` | any | - | tool or harness |
| 72 | `prune-logs` | any | - | tool or harness |
| 73 | `check_scenario_vehicles.py` | any | - | tool or harness |
