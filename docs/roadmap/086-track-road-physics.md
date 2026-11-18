# 086: Track C: Road and Terrain Physics

One tyre-and-ground model: surfaces and soil (wet, gravel, mud, sand, snow), calibration against measurements, the scenario choosing its ground, small obstacles and roughness, and the road section itself (bank, crown, edges).

**Status**: consolidated 2026-11-18 from 047, 052, 053, 056, 066, 069.

**Research rule** (user, 2026-11-18): a research track does not wait for scensim releases.
Its work runs on current scensim main, and it moves to each new scensim release when that
release is published (rebuild, rerun the track's reference runs), as a deliberate step.

**Consolidated** 2026-11-18 (user: "Have A-D"): the phases below keep their own documents as
history and detail; this document is the track's plan and status from now on, and new work
is planned here.

## Parts

| phase | what | status | left |
|---|---|---|---|
| [047](047-off-road-surfaces.md) | off-road surfaces | planned | per-region surface classes everywhere, soil in authored zones |
| [052](052-isaac-physics-completeness.md) | Isaac physics, feature-complete | in progress: steps 1-5 done (physics egos) | the lead NPC's sinking seen by camera and lidar; Autoware egos with 039 |
| [053](053-terramechanics-calibration.md) | soil calibration against published measurements | in progress: sand, mud and snow fitted | dirt; the remaining steps |
| [056](056-scenario-surface-maps.md) | the scenario chooses its ground | planned | RoadCondition properties to a surface map |
| [066](066-small-scale-road-physics.md) | small-scale road physics: rocks, roughness | in progress: R0 done (PhysX misses short steps, issue 029); R5's first slice | the vertical tyre contact on our side |
| [069](069-road-tilt-load-and-edges.md) | road tilt, tyre load and the road's edge | planned, design only | the road-section record |

## Folded in

- Isaac only today; CARLA 1.0's Chaos vehicle ([083](083-carla-1-0.md) step 5) is a second target for the surface classes.

## Next

1. Re-plan as one model: which of 047/056/069 is the shared data (a road-section and surface record) and which are consumers.
