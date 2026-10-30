# 025: the Isaac ego spawned at r01's start facing north falls through the NTU twin

**Severity**: low (one spawn pose on one world; any other pose tried holds, and r01 now
starts elsewhere). **Status**: open, not explained; not retried on the rebuilt world.
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
