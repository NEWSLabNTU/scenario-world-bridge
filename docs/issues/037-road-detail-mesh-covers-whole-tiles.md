# 037: The road detail's render mesh covers whole tiles, so the road's edges look patchy

**Severity**: low (pictures only: the wheels, the rocks and the lidar's collision use the
detail's heights, which are right). **Status**: fixed in swb_isaac_world `f364bc8` (see below).
**Found**: 2026-11-05 in the 066 road-conditions tour video (`spikes/rock_world.py`
recording), Isaac Sim 6.1.

## What happens

In the recorded videos the gravel road's edges are ragged: green patches eat into the
carriageway's sides, the ditches do not show, and wide beige bands run beside the road far
beyond it.

## Why

`road_layer.MeshStream` made one mesh per 16 m detail tile over the whole tile. Beyond a
corridor's reach the detail is 0, so the mesh there is a level sheet at the base's height,
laid over the world's own ground: in the spike it took the road's gravel look (the beige
bands); in the world server it covers whatever the town's ground is with the detail's
material. The spike's grass was a plane 4 cm under the base, which hid every part of the mesh
lower than that: the crowned carriageway's sides (4 % over 3 m is 12 cm) and the ditches.

## Fix

- `Detail.zones_tile` says what each sample is: road (carriageway and shoulder), verge
  (foreslope, ditch, backslope) or outside every corridor's reach.
- `MeshStream` keeps only the quads within a corridor's reach and puts them in two face
  subsets, `road` and `verge`, each bound to a material when `looks` names one.
- The spike dresses the road gravel and the verge grass, and lays its grass as two strips
  beside the corridors from half a metre inside the mesh's outer edge, 1 cm under the base,
  so nothing covers the ditches.
