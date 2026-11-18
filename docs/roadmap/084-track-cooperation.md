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
| [078](078-conflict-zone-protocol.md) | the conflict-zone protocol: library, node, evaluation | phases 0-1 done; phase 2 on the crossing: 2d failed G0 on fresh seeds (a go against a car just at rest) | the 2d fix (running 2026-11-18, P2e), G0, then the scensim grid; phases 3-4 |
| [039](039-isaac-multi-autoware.md) | several Autoware instances on Isaac Sim, V2X | planned | one ROS domain per ego on the world server; V2X |

## Folded in

- The Isaac lidar intensity fix (the user's option 1: an intensity table in the car spec, times the cosine, by object id and normal) and the reflectance sweep: they decide which detector sees what, which 074's detector results rest on.
- CARLA 1.0's deterministic collisions ([083](083-carla-1-0.md)): contact verdicts that repeat between runs.
- Deferred: Taiwan's scooters and the night market (reactive NPC populations); the heavy-vehicle spec (built when an Autoware test needs a truck).

## Next

1. 078 Phase 2d fix, then G0 (subagent on 133, 2026-11-18).
2. 074 U3 (four-way stop) on Autoware.
3. The Isaac lidar intensity fix.
