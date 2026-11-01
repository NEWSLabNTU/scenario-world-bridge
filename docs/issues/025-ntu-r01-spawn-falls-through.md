# 025: the Isaac ego spawned at r01's start facing north falls through the NTU twin

**Severity**: low (one spawn pose on one world; any other pose tried holds, and r01 now
starts elsewhere). **Status**: fixed (`build_meshes.py --lanes`: the ground is meshed under
every lanelet; world `n5v3`, 2026-11-02).
**Found**: 2026-10-30, roadmap 059 N8 on newslab-server133 (aib's runner, `just isaac-ego`).

## What is wrong

`pose:=314.25,105.19,12.83,1.5773` (world frame; map (52814.25, 67805.19), r01's first
lanelet 1004948, 10 m in, facing along it) drops the car through the ground within a second:
GNSS altitude -12 km, vertical speed -298 m/s a few seconds later.

## What holds

At the same x, y: yaw 0 and yaw pi. At yaw 1.5773: 3 m south and 10 m north. r02's spawn at
yaw pi/2. A box (`DynamicCuboid`) dropped on the same spot rests on the ground. Only ground
triangles lie within the car's footprint there (no "other" collider).

## What it was not

An earlier fall at r01 was a hole in the ground mesh where something stood during the scan;
`build_meshes.py`'s `ground_keep` now fills every enclosed hole. This fall came after that fix.

## Open

Next: retry on the rebuilt world (DTM from the lowest return on the lanes, meshes rebuilt
2026-10-30). If it still falls: log the four wheels' raycast hits and the chassis' contact
report at the first frames; compare the chassis' convex hull with the ground triangles under
it (a wheel spawned inside a triangle's back face would explain a heading dependence).

## Cause (2026-11-02)

A hole in the ground mesh under the car. Facing north at that spot, 711 of 1056 points under
the chassis box (-0.98 to 3.81 m along, +-1.08 m across base_link) lay on no ground triangle:
a diagonal band of DTM cells with no ground return within GROUND_REACH, where vehicles stood
throughout the scan. `ground_keep` fills only enclosed holes, and this band runs on into
others, so it stayed open. The same spot facing east (yaw 0) now fell too, for another reason:
that box lies over a 4 m spike of the DTM beside the lane (16.4 m against 12.3 m, a tree's
foot taken for ground) and overlaps a trunk collider (N5), so PhysX threw the car out.

## Fix

`build_meshes.py --lanes LANELET2_OSM` meshes every DTM cell under a lanelet grown by
LANE_KEEP (1 m), whatever the ground classes say (`--ground-only` rewrites the ground meshes
without reading the scan; 167 tiles in about a minute). The DTM has a height there already.
After it, every point under the north-facing car is on a triangle, and on world `n5v3`
(meshes_n3, the lanelets as widened on 2026-11-02):

| spawn (world frame) | rest offset along / across | altitude |
|---|---|---|
| (314.25, 105.19), yaw 1.5773, the pose that fell | +0.028 / +0.005 m | holds |
| (314.25, 102.19), yaw 1.5773, 3 m south | -0.095 / +0.243 m | holds |

Facing east the car rested 2.5 m off against the trunk: that pose points the car across the
lane into a tree, so it is not one a scenario uses.
