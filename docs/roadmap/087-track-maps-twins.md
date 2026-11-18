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

## Next

1. Decide 072's target (CARLA 1.0 content pack).
2. Try CARLA 1.0's map generator on Town01_Opt against our map (083 decision 2).
