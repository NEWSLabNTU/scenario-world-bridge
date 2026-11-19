# 072: Haruna's point cloud map is too large for Autoware's compare-map filter: static points stop the ego

**Severity**: high (the how-to's Haruna course cannot be completed). **Status**: open.
**Found**: 2026-11-20, swb 118, haruna_down on `isaac_first_drive` with scensim's profile, once
issue 070 was fixed.

## What happens

The ego starts, drives about 230 m down the course, and stops for good at (82.0, 428.5), a
3.5 deg downhill, until the scenario's 600 s limit. Read from the ego's domain while it stood:

- `/api/planning/velocity_factors`: `route-obstacle`, status 2, 4.0 m;
- `/perception/object_recognition/objects`: 38 objects, on a course with no other entity;
- `/perception/obstacle_segmentation/pointcloud`: 14916 points.

Autoware's `voxel_based_compare_map_filter` warns from the start, and keeps warning:

```
Map loader status: Given map voxel grid is not feasible. (Number of voxel overflows int32_t
limit) ... (1) If use_dynamic_map_loading is false, consider to enable use_dynamic_map_loading
(2) If use_dynamic_map_loading is true, consider to adjust map pointcloud split size smaller
```

## Cause

The filter removes the map's static points (terrain, guardrails, trees) from the obstacle
cloud by a voxel grid over the whole map. Haruna's `pointcloud_map.pcd` (dataset
`haruna-tsukuba-courses`, 15.8 M points, 189 MB) spans 907 x 1215 x 370 m: a mountain, so its
voxel count overflows int32 where a CARLA town's (about 600 x 600 m and flat) does not. The
filter then removes nothing, the static points reach detection as obstacles, and the obstacle
stop module stops the car at the first that falls in its path. The course's map has no
`pointcloud_map_metadata.yaml`; the towns' maps have one (a single cell).

## What it needs

A divided map: the course's point cloud split into tiles (Autoware's `pointcloud_divider`, for
example 100 m cells) with their `pointcloud_map_metadata.yaml`, so the filter loads the cells
near the car (dynamic map loading). That is a new revision of `haruna-tsukuba-courses` (its
maintainer's, the user's to publish). Whether the stack's map loader and profile take a divided
map as they are is to be checked with it.

## Evidence

NAS `logs/2026 SSv2 on CARLA/2026-11-20 0.2.1 Isaac examples live (118)/` (run 5, with the
ego's pose every 20 s).
