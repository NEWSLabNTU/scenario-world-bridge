# 011: a soil zone of 221 tiles runs RTX out of resource descriptors, and the world server hangs

**Severity**: medium. **Status**: open (worked around: smaller zones). **Found**: 2026-10-26,
roadmap 054 step 8 (`town01_mud_curve`'s first surface map).

## What is wrong

A soil zone with `ground_z` is covered by 1.6 m tiles over its bounding box (`soil_tiles`), each
with a render mesh of 33 x 33 vertices and the class's material. `town01_mud_curve.yaml` first
put the mud over Town01's south-west bend and 10 m of road either side, a bounding box of
20.8 x 27.2 m: 221 tiles. The world server cut the town's ground under it (16 meshes, 1.35 s),
built the tiles and settled, then 2.5 minutes into its first renders logged

```
[Fatal] [omni.rtx] Out of resource descriptors!
```

and stopped answering without exiting: no Hello, so `scripts/isaac_up.sh` waited, and the
launch's respawn never fired (`play_log/isaac/2026-10-26_12-26-06`).

The same Town01 world with 32 tiles (`town01_mud`), 60 (`town01_mud_right`), 90
(`town01_mud_stop`) and 99 (the bend alone, `town01_mud_curve` as committed) loads and runs.
Where between 99 and 221 the limit lies, and what holds the descriptors (the tiles' meshes,
their material bindings, or the cut town meshes), is not measured.

## Why it matters

A zone's bounding box, not its area, sets the tile count, so any zone along a bend or a
diagonal road grows quickly: a 90 degree bend of a 4 m lane with 10 m either side already
needs about 200. And a world server that hangs instead of exiting is not restarted by its
supervisor.

## Next

- Count what each tile adds to RTX (a sweep of zone sizes on Town01, the descriptor count if
  RTX exposes one) and find the limit.
- Tiles only where the polygon is, not over its whole bounding box.
- The world server should exit on an RTX fatal (or its health check should notice), so the
  launch respawns it.
