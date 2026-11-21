# Phase 092: Scenario to world, and sampling

Turn a declared scenario into a world on a chosen target (an Isaac Lab batch for training,
Isaac Sim or CARLA on seats for evaluation), and sample scenarios with an identity that
replays across targets. Published as adaptable components and specifications, not a
framework; Isaac Lab keeps training throughput.

**Status**: design accepted for now (user, 2026-11-20: "The shape is better ... Accept the
decision for now"); the scenario-to-world compiler is being studied further before any work
item starts.

**Source**: feedback from a RoboRacer team after the scensim presentation, 2026-11-20 (Jazzy;
hand-set-up runs; slow data between autopilot, simulator and ML infrastructure), and four
rounds of review with the user the same day.

**Design**: [ml-training-interface.md](../design/ml-training-interface.md) (revision 4):
the contributions C1 to C5, decisions D1 to D12, the layers, the architecture and workflow,
throughput against Isaac Lab, usage by user.

**Studies**: [survey-risk-categories.md](../design/survey-risk-categories.md): known risk
categorisations (NHTSA pre-crash typology, PBCAT, Euro NCAP, UN R157, JAMA and ISO 34502,
SOTIF, Waymo's collision avoidance testing, accelerated evaluation) and a coverage model for
the two purposes the user chose (2026-11-20): a realistic risk rate, and failure-mode search.
Scenario and world stay separate concepts, joined by the compiler (user: option A).
[survey-vehicle-risk-taxonomy.md](../design/survey-vehicle-risk-taxonomy.md): the whole space
of vehicle risk in eight families (traffic conflicts, VRUs, infrastructure and maps,
perception, vehicle motion, the ego's own faults, interaction, ODD boundaries), what a
scenario must express for each, three layers of coverage and realism per family.

## Work items

From the design's section 10.

| Item | What | Kind | Depends on | Owner | Status |
|---|---|---|---|---|---|
| S1 | The sample record: schema, hashing, checker ([sample-record.md](../design/sample-record.md)) | spec | none | swb 139 | done (scensim_sampler) |
| S2 | The compilable subset of OpenSCENARIO ([compilable-subset.md](../design/compilable-subset.md), `scensim-sample compilable`) | spec | the compiler study | swb 139 | done; SSv2's examples: 58 of 83 compile with the ego as the batch's policy (18 unconditionally), 41 with `--ego seats` |
| S3 | The batched control component (`step_batch`; roboracer-indoor-twin.md 2.6, sim_vehicle_ecu.batch) | spec | 091 | swb 139 | done (sim_vehicle_ecu d575758); follow-ups: `ideal`/`ev` vectorised (they run as LoopBatch, flagged), and the roboracer stand-in VESC (091 W23) converted to the array form once W23 lands, with 118 |
| S4 | The stage record (091 W15) and the truth stream's content ([truth-stream.md](../design/truth-stream.md)) | spec | 091 W15 | swb 243 | done 2026-11-21: the stage record is 091 W15's `provenance.stage`; the truth stream is the gap trace at schema 2 (file, by default; the simulator's values under neutral names, read by schema) and `/scensim/truth` (on request), from the bridge, no world-protocol change; route progress and occlusion named as later capabilities |
| K1 | The sampler (pure Python): `src/scensim_sampler`, `scensim-sample` | component | S1 | swb 139 | done (`scensim-sample`) |
| K2 | The Isaac Lab adapter, with `scensim bench` | component | S1, S2, S3, 091 W5 | | open; acceptance includes running each S2 "batch" row against seats on the same sample |
| K3 | Replay of a sample record on seats, fidelity measured: `sample:`, `run --replay`, `runs fidelity` | component | S1, K1 | swb 139 | done; measured 2026-11-21 on CARLA 0.9.16 (Town01, simple pilot, 5 records + 5 replays): 5 of 5 same outcome, but all 10 passed, so the measure did not discriminate, and the ego was a stack pilot (its seed not applied); then a discriminating set (time_limit U(8,14) s, goals 46-79 m, n=10, the ego a lifetime: run pilot so the record's seeds applied): 8 passed, 2 failed on their time limit, and 10 of 10 replays reached the same outcome, both failures on the same simulation tick (9.25 s, 10.20 s); no contact case yet |
| K7 | The coverage report: `scensim-sample coverage`, `scensim runs coverage` (sample-record.md section 8): category, condition (t-wise) and criticality coverage, an exposure-weighted overall rate only with the study's own weights | component | S1, K1 | swb 139 | done; **open**: under heavy weights (n_eff/n < 0.1) the weighted interval under-covers, down to 0.61 at nominal 95 % (measured; the spec's table), which matters for purpose A (realistic risk rates); to evaluate later: a bootstrap over records, or an exact-style interval on the effective counts |
| K4 | The truth stream, judge and attachment hooks, the paced clock | components | S4 | swb 243 (the truth stream) | in progress: the truth file done 2026-11-21 (the gap trace at schema 2 from the bridge, recorded by default, read by schema by `gap_trace.py`, the gap oracle and `incident_judge`); next `/scensim/truth`. scensim_carla_world adapts to the renamed recorder API when its `deps/scensim_world_protocol` next moves (its trace is off) |
| K5 | GPU transport for evaluation (NITROS first) | component | none | | open |
| K6 | Jazzy for the whole set | all | 082 | | open |
| E1 | Examples | examples | the above | | open |
