# NTU twin map revisions: one source, two exports

The NTU campus twin exists twice: as a CARLA 0.9.16 map
([072](../roadmap/072-ntu-campus-in-carla.md), swb 133) and as an Isaac Sim world
([059](../roadmap/059-ntu-twin-on-isaac.md), swb 118). Both must show the same place, so that a
leg that passes on one and fails on the other says something about the simulator, not about two
different maps. This document is how a change to the place reaches both.

**The rule:** every revision starts from the original NTU dataset (the vendor's scan and the
Autoware maps). The shared stages S0-S4 turn it into a set of artefacts, and each simulator's
world is an export of that set. Neither world is ever converted into the other: no CARLA mesh
goes into the USD, no USD into the FBX.

```
NTU dataset ──S0──S1──S2──S3──S4──┬── E-CARLA ── NTU package (CARLA 0.9.16)
 (scan, Autoware maps)            └── E-Isaac ── world.usdc + world.yaml (Isaac Sim)
```

## A revision

A revision, `n<k>`, is one set of S-stage artefacts, each named by its md5, built by the scripts
in `scripts/ntu_twin/` at one commit. Its manifest lives on the NAS:

```
dataset/2026-10-30 NTU twin map/revisions/n<k>/MANIFEST
```

one line per artefact: stage, path (relative to the twin's root), md5, the script and its
commit. Meshes and USD files are not byte-stable across hosts, so S3 and the Isaac export
record a checksum of their content (the points and faces, per tile) rather than of the file.

Each world names the revision it was exported from: the CARLA package's README, and
`world.yaml`'s `revision:` and `manifest_md5:` on Isaac.

## The shared stages

| Stage | What | Scripts (`scripts/ntu_twin/`) | Output | n15 |
|---|---|---|---|---|
| S0 Source | the vendor scan tiled, moving objects out, trees, the PCD map | `retile_laz.py` (167 tiles of 50 m, MGRS 51RUH local), `transients.py`, `trees.py`, `export_map.py` | `tiles_50m_mgrs/`, `trees_v2/`, `map/pointcloud_map/`, projector | unchanged since n2 |
| S1 Lanes | r01 and r02 merged, registered to the scan, fixed | `register_lanelets.py`, `fix_lanelets.py` steps A-E | `map/lanelet2_map.osm` | `80f24846` |
| S2 Ground | DTM and orthophoto, refilled, lanes smoothed, humps opened | `ground_ortho.py`, `ortho_refill.py`, `smooth_dtm.py --lanes <S1>` | `ground_n<k>/dtm.npy`; the refilled orthophoto, its own manifest entry (both exports texture the ground from it) | `8049e322` |
| S3 Meshes | buildings and objects per tile; the ground mesh on S2 | `build_meshes.py` (`--ground-only` when only S2 changed) | `meshes_n<k>/` (building and other PLY, ground npz) | per-tile ground md5s |
| S4 Clean-up | parked vehicles; low vegetation as a height field | `parked_vehicles.py`, `hedges.py --parked` | `parked.npz`, `hedges.npz` (S2's grid) | `2f160ede`, `af499af6` |

S1 and S2 are already reproducible across hosts: 133 and 118 both build n15's `dtm.npy` from
the same inputs and get the same md5, and `fix_lanelets.py` writes lat/lon at 9 decimals so the
map's md5 is the same everywhere (890c7b9).

`hedges.npz` holds `mask` (bool), `top` (float16, metres over S2's DTM, 0 outside) and
`colour` (uint8 sRGB), on S2's grid with rows by y as `dtm.npy`. `parked.npz` holds `mask` and the
blob table.

## The exports

Each export reads S0-S4 (S0 for the trees) and nothing else.

**E-CARLA** (`scripts/ntu_carla/`, on 133):

1. `lanelet_to_xodr.py` S1 -> `NTU.xodr`
2. `carla_meshes.py` S3 S1 S2 S0's trees `--parked` S4 `--hedges` S4 -> per-class FBX meshes
3. the UE import, materials and package (`site_rebuild.sh`), then `ImportAssets.sh` into the
   0.9.16 binary.

n15's run of these is `n15_build.sh` (072).

**E-Isaac** (`scripts/ntu_twin/world_usd.py`, on 118). The target is a fresh build:

1. `world_usd.py` S3 (refilled orthophoto) S1 `--clear-road` S2 `--trees` S0 `--parked` S4
2. `world_usd.py --retouch` (crown cut-outs, de-lighting, shards: 059 N10)

The light (sun and sky dome, `sky.hdr`) is written by `world_usd.py` too (`--lights-only` on an
existing world): it is export, not place, as CARLA's weather is.

`world_usd.py` gains `--parked` (the faces `carla_meshes.py` drops, by the same rule: under the
mask, below 2.6 m over S2's DTM) and writes `revision:` into `world.yaml`.

Until that build exists, Isaac's world is a chain of edits on the n5v3 world: `--retouch` made
n10, and `swap_ground.py` rewrote only the ground points to S2's DTM at n11-n15 (f172b67),
dropping Other/Tree faces the new DTM puts on the road. `swap_ground.py` stays as a fast path
for an S2-only revision, provided its ground points match the fresh build's (the manifest's
per-tile checksum).

## What belongs where

- **A fix to the place** goes into S0-S4 and reaches both worlds: the ground, the lanes, parked
  vehicles, hedges, shards on the road. Never into one export alone.
- **A fix to how a simulator shows the place** stays in its export: materials, exposure,
  colliders, CARLA's semantic mesh names, Isaac's payload layout.
- `scripts/ntu_twin/` is shared. A change there is told to the other session before it is
  pushed. `scripts/ntu_carla/` is CARLA's export only.

## Making a revision

1. Whoever needs the change starts `n<k+1>`: changes the S-stage scripts, rebuilds the stages
   from the first one the change reaches, writes the manifest to the NAS and tells the other
   session.
2. The other side rebuilds its S stages (or copies them, checking the manifest's md5s), then
   its export, and confirms the md5s match.
3. Proof runs and comparisons name the revision. A side-by-side pairs only worlds of the same
   revision; when the two differ in a stage, the video says which.

## Known differences

- **Hedges, n15 and before.** CARLA replaces the scan's low vegetation with S4's height field;
  Isaac keeps the scan's Tree/Other faces. To close: an Isaac hedge step that builds from
  `hedges.npz` (059).
- **Parked vehicles, n15.** CARLA's n15 world leaves them out; Isaac's n15 world keeps them. n16
  closes it: `swap_ground.py --parked` on n15's world dropped 275 383 Other/Tree faces in 56 tiles
  (271 877 Other, 3 506 Tree), legs 64, 24, 104, 109, 03, 83, r01 and r02 under test; then
  `world_usd.py --parked` in the fresh build.
- **The 2026-11-05 proof runs** (`proof_2026-11-05/`, and the side-by-side beside them) share
  S1 and S2 (n15), not S4: Isaac's world had the parked vehicles and the scan's hedges.
- **The lidar.** Each simulator's own (RTX, CARLA raycast) on its own meshes. NDT's map (S0's
  PCD) is shared and still holds the parked vehicles on both.
