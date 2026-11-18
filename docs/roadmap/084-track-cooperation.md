# 084: Track A: Cooperation Between Automated Vehicles

How automated vehicles share conflict zones: the cases as games whose payoffs scensim measures (074), the strategy turned into a node beside Autoware and tested (078), several Autoware egos on one world for the multi-car runs (039).

**Status**: consolidated 2026-11-18 from 074, 078, 039.

**Research rule** (user, 2026-11-18): a research track does not wait for scensim releases.
Its work runs on current scensim main, and it moves to each new scensim release when that
release is published (rebuild, rerun the track's reference runs), as a deliberate step.

**Consolidated** 2026-11-18 (user: "Have A-D"): the phases below keep their own documents as
history and detail; this document is the track's plan and status from now on, and new work
is planned here.

## Parts

| phase | what | status | left |
|---|---|---|---|
| [074](074-cooperation-study.md) | the cooperation study: games, arena, strategy atlas | started 2026-11-13; most site types done in the kinematic model and the arena (S1-S6, U1-U3, Y merge); stock-Autoware runs for the Y merge and the left turn; atlas v11 | Autoware on the four-way stop (U3); the reflectance sweep (user's call) |
| [078](078-conflict-zone-protocol.md) | the conflict-zone protocol: library, node, evaluation | phases 0-1 done; phase 2 on the crossing: G0 passes (P2e, 2026-11-18: the mutual-rest guard plus `rest_first`) | the scensim grid; phases 3-4 |
| [039](039-isaac-multi-autoware.md) | several Autoware instances on Isaac Sim, V2X | planned | one ROS domain per ego on the world server; V2X |

## Folded in

- The Isaac lidar intensity fix (the user's option 1: an intensity table in the car spec, times the cosine, by object id and normal): **landed 2026-11-19** (aib a37d23e, scensim_isaac_world c3d0e2f; report on the NAS, "2026-11-18 084 Isaac lidar intensity fix (139)"). Points of bodies spawned after the map is made take the class of the entity box they fall in. Car intensity now matches CARLA's (medians 69/29/28/41 against 77/27.5/17/38; before 15/5/3/9); TransFusion false positives 1.64 to 1.00 per output (CARLA 0.40). TransFusion's broadside-car recall falls to 0.74, as CARLA's does with the same table (0.36-0.40). Its 35 m recall (0.175 against CARLA's 0.47-0.51) is a point-density gap, not intensity: Isaac's cloud has ~30 % fewer points (88.7k against 112k). Next: the density against CARLA's semantic lidar, then the reflectance sweep.
- CARLA 1.0's deterministic collisions ([083](083-carla-1-0.md)): contact verdicts that repeat between runs.
- Deferred: Taiwan's scooters and the night market (reactive NPC populations); the heavy-vehicle spec (built when an Autoware test needs a truck).

## Next

1. 078 Phase 2: the scensim grid on the crossing.
2. 074 U3 (four-way stop) on Autoware.
3. The Isaac lidar point density against CARLA's (139).
