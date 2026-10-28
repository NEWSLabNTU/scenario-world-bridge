# Phase 058: A Mountain Pass from Open Data (Haruna's Hairpins)

A small, realistic Isaac world built from public data rather than a CARLA town: a 2 km section
of a real Japanese mountain pass (touge) with its hairpins, its terrain and its aerial photo.
It gives the drift campaign (057) the turns real drift events use, hairpins taken at 10-15 m/s,
in a world that costs a fraction of a CARLA town's GPU memory and RAM.

**Status**: in progress, 2026-10-29: C1-C8 done offline for the Haruna section
(swb_isaac_world `swb_isaac_world/course/`, `python3 -m swb_isaac_world.course
swb_isaac_world/courses/haruna.yaml data/courses/haruna`, the USD step in Isaac's venv); C9
waits for a GPU.

Progress:

- C1-C2: a 1.53 km downhill section of road 33 (about 900 m to 810 m), eight hairpins of
  136-186 deg at radii 9-20 m (H1-H8, `course/centreline.json`).
- C3-C7: terrain from dem5a at 2 m, the aerial photo at z18 as its texture, the road ribbon
  and its lines, guardrails and trees, `isaac/world.usdc` (9 MB); a Lanelet2 map of 16
  lanelets each way and a point cloud of 15.8 M points.
- C8 (swb_isaac_world 92430eb): `scenarios/haruna_down.xosc` (s 20 m to 30 m before the end, in
  the left lane), `surfaces.yaml` (asphalt road, grass terrain) and `course/drift_points.yaml`
  (eight points, on the lane at each arc's start, signed radii 8.1-18.6 m). Checked offline:
  the start, the goal and every drift point lie in lanelets along the section, and Lanelet2
  routes start to goal over 15. The run up the pass is not written yet.
- C9a (swb_isaac_world 900457c, 243, 2026-10-29): the world loads and renders in about 10 s;
  Isaac takes ~4 GB of GPU memory. `spikes/course_probe.py` drove drift_mustang without
  Autoware, by pure pursuit on the scenario's lane at 8 m/s (5 m/s within 30 m of an apex),
  580 m from the start through H1-H4 in 85 s: at most 0.55 m off the lane, on the road ribbon
  (0.07 m under its centreline height), no contact. The props were missing at first: the
  world named 241's $CARLA_USD (issue 020, fixed by a `carla-usd` link beside the world).
  The course camera shows the forest, the guardrails and the hairpins.

**Source**: user direction 2026-10-29: "work on a smaller map on our host ... make a drift
along the hairpin turn, like the real drift race ... build a smaller map in Isaac Sim. We dont
need to load the whole town from Carla. However, the map should look realistic ... find a real
world sample". Decisions (user, same day): a Japanese touge (Mt. Haruna); a rough first
version (terrain, road, aerial-photo texture, basic props), refined later.

**Depends on**: [038](038-isaac-vehicle-bridge.md) (the Isaac world file, `isaac/world.yaml`, and the
point-cloud world), [056](056-scenario-surface-maps.md) (surface maps),
[057](057-drift-campaign.md) (the car, the drift controller, the lap tooling).

## The course

Gunma prefectural road 33 (渋川松井田線), from Lake Haruna down towards Ikaho: two lanes,
asphalt, 30 km/h posted (OpenStreetMap). Between 36.486 and 36.502 N, 138.896 and 138.909 E it has eight hairpins with more
than 120 deg of turn within 60 m, six of them in a chain at 36.497-36.500 N,
138.904-138.906 E (the famous consecutive hairpins), plus two lower down, in about 2 km of
road. Elevation about 1100 m at the lake.

## The data

| What | Source | Resolution | Terms |
|---|---|---|---|
| Road centreline, lanes, surface | OpenStreetMap (Overpass API) | the mapped geometry, about 5-20 m between nodes | ODbL, attribute OpenStreetMap contributors |
| Terrain | GSI 基盤地図情報 数値標高モデル, tile `dem5a` (z15, 256 x 256 text grids) | 5 m (laser survey) | GSI terms; attribute 出典：国土地理院 |
| Ground texture | GSI シームレス空中写真, tile `seamlessphoto` (z18) | about 0.5 m per pixel | as above |
| Props | the CARLA 0.9.16 Isaac set (`$CARLA_USD/Town05/Props`): pines, barriers | | CARLA's licence |

Raw downloads stay out of git, under `data/courses/<name>/raw/` (`data/` is ignored); the
generator fetches what is missing.

## Frames

The map frame is a local transverse Mercator at the course's origin (its centre, rounded),
metres east and north, with z the elevation minus the origin's. `map_projector_info.yaml`
is `TransverseMercator` with that origin's latitude, longitude and altitude. Every Lanelet2
point carries `local_x`/`local_y` (as the repaired CARLA maps do), so no consumer depends on
the projector's scale. The Isaac world is in the same frame (`map_origin` 0, like the SimReady towns).

## Work items

**C1. Fetch.** OSM ways of the road (and every highway in the box, for context), the DEM
tiles and the photo tiles covering the section plus a margin of 300 m; cached, so a rerun
is offline. Done for Haruna by hand (2026-10-29); folded into the generator.

**C2. The centreline.** Chain the road's ways, cut the section between two points, resample
at 0.5 m and smooth (OSM's nodes are coarse in the hairpins). Heights along the road come from the DEM,
smoothed along the road so the profile has no steps (the DEM sees trees and banks next to the
road); grade and curvature are reported. The lane width is 3.25 m per lane (two lanes, no
centre median), the shoulder 0.75 m.

**C3. The terrain.** A grid mesh from the DEM over the section's box. Under the road and its
shoulders the terrain is cut or filled to the road's profile, blending back to the DEM over
8 m. UVs map the aerial-photo mosaic onto it, so the forest, the road's own scar and the
buildings by the lake show as in the photo.

**C4. The road.** A ribbon mesh along the centreline at the lane widths, asphalt-coloured,
with its centre line (Japan: a yellow line, no passing) and edge lines as thin meshes on top.
It is labelled as SimReady labels CARLA's road and markings (`semantics:labels:wikidata_qcode`),
so the world server's ground paths and surface classes pick it up without a special case.

**C5. Props, rough.** A guardrail (the barrier asset, repeated) along the outside of each
hairpin and wherever the terrain drops away from the road edge; pines scattered on the
terrain where the photo is forest, kept 3 m clear of the road. Enough for the lidar and the
cameras to see structure; refined later.

**C6. The world.** One USD stage: terrain, road, markings and props, colliders on the
terrain, road and guardrails, a sky dome and a sun. `isaac/world.yaml` names it, with
`map_origin` 0 and the ground paths, as `usd_world` writes for the towns.

**C7. Autoware's maps.** `lanelet2_map.osm`: one lanelet per lane per stretch, both
directions, `speed_limit` set to the drift campaign's need (not the posted 30), `local_x`/`local_y`
on every point. `pointcloud_map.pcd`: sampled from the same meshes the lidar sees (road,
terrain, guardrails, trees), voxelised at 0.2 m, so NDT's map and the lidar's world are one.
`map_projector_info.yaml` and `map_config.yaml` as above.

**C8. Scenario and drift points.** An OpenSCENARIO run down the pass (and one up), the
surface map (asphalt everywhere for now), and autoware-drift drift points at each hairpin with
its radius and turn angle, from C2's curvature.

**C9. Validation** (needs a GPU with about 8 GB free): the world loads and renders; a lidar
scan matches the point-cloud map (NVTL at least 2.5 along the road); Autoware routes from the
start to the goal; a lap at low speed passes; then the first hairpin drift.

## Steps

1. C1-C2 on Haruna: the centreline, its profile and its hairpins (a plot to check against the
   photo).
2. C3-C4: terrain and road meshes, viewed offline (a render or a top-down check), no GPU.
3. C5-C7: props, the USD world, the Autoware maps.
4. C8, then C9 when a GPU is free (241's are held by another user's vLLM as of 2026-10-29;
   243 is the alternative).

## Risks

- **Coarse OSM in the hairpins.** If the smoothed centreline misses the real road by more than
  a lane, it is corrected against the photo (the road shows through the forest).
- **The DEM sees trees.** The 5 m laser DEM is ground, not canopy, but banks and walls next
  to the road give steps; the road profile is smoothed, and the cut and fill hides the rest.
- **Hairpin speed against the controller.** The ring's drift law enters at 27 m/s on a 106 m
  arc; hairpins of 15-25 m radius drift at 8-15 m/s. Its entry speed and targets need
  retuning for this course (057's published regime: 8-15 m/s on 13-22 m radii).
- **Autoware in a hairpin.** Its smoother and lane following were tuned for the ring's large
  radii; a 15 m hairpin at speed may trip its validators as the ring's lanelet joints did
  (057, lap 25).
