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
| S1 | The sample record: schema, hashing, checker ([sample-record.md](../design/sample-record.md)) | spec | none | swb 139 | in review |
| S2 | The compilable subset of OpenSCENARIO | spec | the compiler study | | open |
| S3 | The batched control component (`step_batch`) | spec | 091 | | open |
| S4 | The stage record (091 W15) and the truth stream's content | spec | 091 W15 | | open |
| K1 | The sampler (pure Python): `src/scensim_sampler`, `scensim-sample` | component | S1 | swb 139 | in review |
| K2 | The Isaac Lab adapter, with `scensim bench` | component | S1, S2, S3, 091 W5 | | open |
| K3 | Replay of a sample record on seats, fidelity measured | component | S1, K1 | | open |
| K4 | The truth stream, judge and attachment hooks, the paced clock | components | S4 | | open |
| K5 | GPU transport for evaluation (NITROS first) | component | none | | open |
| K6 | Jazzy for the whole set | all | 082 | | open |
| E1 | Examples | examples | the above | | open |
