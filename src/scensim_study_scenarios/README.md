# scensim_study_scenarios

The scenarios several of this repository's study testbenches share (roadmap 081 step 4). They
were `scenarios/bench/` and `scenarios/gap/` before the studies became testbench packages.

- `bench/`: the bridge's frame-budget benchmark, Town01 with 10, 20 and 50 NPCs, written by
  `scripts/gen_npc_benchmark.py`. Used by the `bench` and `set050` testbenches.
- `gap/`: reproducers of fixed SSv2 and bridge gaps (entity bodies, a motorbike's body, an NPC
  on the shoulder, a truck passing). Used by `gap`, `set050` and `gap_campaign`.

A testbench names them as `pkg://scensim_study_scenarios/bench/town01_npc_20.xosc` and declares
`<exec_depend>scensim_study_scenarios</exec_depend>`. Each names its map as
`$(env SCENSIM_MAPS)/Town01` and the vehicle catalog as `$(find-pkg-share sim_vehicle_spec)`.
