# 072: Haruna's point cloud map is too large for Autoware's compare-map filter: static points stop the ego

**Severity**: high (the how-to's Haruna course cannot be completed). **Status**: open: the profile part is landed (autoware_sim_vehicle 1e0bc40, scensim pins c0254a9e); the divided map waits for a dataset revision.
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

## What it needs (a candidate tried, 2026-11-20)

Two things together, tried on 118 with a local copy of the dataset (the course's point cloud
divided into 140 tiles of 100 m with a `pointcloud_map_metadata.yaml`, every point kept):

| Map | Compare-map filter | Objects on the empty course | Result |
|---|---|---|---|
| one PCD (the dataset) | static (the profile) | 38 | stops at (82.0, 428.5); failed at 600 s |
| divided | static (the profile) | 33-35 | the same stop |
| divided | dynamic (`use_dynamic_map_loading: true`) | 2-8 | **passed**: the whole course in 384.5 s, 0 contacts |

With static loading the filter merges the tiles into one grid again and it still overflows;
with dynamic loading it loads the tiles within 150 m of the car. So the fix is a divided map
(a new revision of `haruna-tsukuba-courses`, the user's to publish) and dynamic loading for
it. scensim's profile turned dynamic loading off for acb issue 029 (a street lamp held the
ego at Town01's spawn point 0), so it is not turned on again everywhere.

## The profile's part (landed 2026-11-20)

autoware_sim_vehicle 1e0bc40: the profile picks the filter's loading by the map, derived and
not configured. A divided map (`pointcloud_map.pcd` a directory of tiles, with
`pointcloud_map_metadata.yaml`) gets dynamic loading, upstream's own parameter file; anything
else (one PCD file, as every CARLA town, whose metadata names that one file) keeps the
profile's static file. The launch logs the choice:

```
compare-map filter: dynamic map loading, a divided map, 140 tiles in .../haruna/pointcloud_map.pcd
compare-map filter: static map loading, one point cloud file (.../Town01/pointcloud_map.pcd)
```

Checked on 118 with it as landed: haruna_down on the divided candidate map passed (382.4 s
simulated, 0 contacts, no overflow warning); Town01 on CARLA with the published map
(first_drive's pedestrian_crossing) loaded statically and passed. What remains is the dataset
revision with the divided map (the user's to publish); until then the published Haruna map is
one PCD and loads statically, as before.

## Evidence

NAS `logs/2026 SSv2 on CARLA/2026-11-20 0.2.1 Isaac examples live (118)/` (run 5, with the
ego's pose every 20 s). The candidate: NAS `home/2026-11-20 issue 072 haruna divided map
candidate (118)/` (the divided map, its PROVENANCE, both runs' samples, the passing run).
