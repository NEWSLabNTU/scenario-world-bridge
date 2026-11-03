# 032: omni.physx logs "Collisions are supported currently only in one collision group" for every streamed rock

**Severity**: low. Every collider is in exactly one group, and the filtering works. The warning
was logged about 1500 times per 800 m of rock road, one per rock prim made or reused, and it
buried the log's other warnings. **Status**: fixed in swb_isaac_world 12e3ffc.
**Found**: 2026-11-02 on newslab-server-241, roadmap 066 R4 (`spikes/rock_world.py`), Isaac
Sim 6.1 (omni.physx 110.3.2).

## What happens

The world server's rock stream (`rock_layer.RockStream`) makes collision tiles and rock prims
while the simulation runs. Each one is put in its collision group by its own path (issue 030):
tiles in `roadDetail`, rocks in `rocks`, both filtering the wheels' `groundQuery`. With the
stream running, the Kit log filled with:

```
[Warning] [omni.physx.plugin] Collisions are supported currently only in one collision group.
```

Over 800 m at 80 km/h there were 1472 such lines; at 40 km/h over 100 m, 239.

## What it was not

- **Not a collider in two groups.** A scan of every collider on the stage after a run (USD's
  membership query over every `CollisionGroup`, `ROCK_GROUPS=1` in the spike) found each of the
  205 in exactly one group.
- **Not the filtering.** Rocks fall through the base road their group filters and rest on the
  collision tiles (rest gap 0 mm); no wheel was thrown by a rock (030's sign).
- **Not reproduced in isolation** (`spikes/group_late.py`): colliders made after play, several
  in one step, removed, rigid bodies switched on and off, collision switched on and off, aib's
  groups and vehicle, a contact report, forces and reads: no warning, until shutdown.

## Why

The message comes from omni.physx's `CollisionGroup.cpp` (`getCollisionGroup`), on its Fabric
path (`updateFabricCollisionGroups`): a collider whose group it resolves to more than one. In
the spike the stage's groups are updated through Fabric at run time (`omni.fabric.plugin`
logs "attribute collection:colliders:includes not found for path .../roadDetail" with the
first tiles), and in that state two things made it warn:

1. **Several colliders added to one group in one step.** For k colliders added in a frame, k-1
   warnings followed the step: the first frame's five tiles gave five, each later frame's four
   rocks three. Our own group edits, one `AddTarget` per collider.
2. **A pooled rock prim rescaled.** A rock leaving the window parks its prim, and the next rock
   of the same shape takes it, moved, turned and rescaled (swb_isaac_world a28cf19). With the
   scale left as it was the warnings stopped (0 in the same run); with it set, out of the group
   and with collision off while rescaled, or added back a frame later, they did not. PhysX
   rebuilds a rescaled shape and resolves its group again, and its Fabric copy of the group
   seems to keep the earlier entry: an omni.physx matter, not ours to change.

## Fix

- **One edit of a group a frame** (`RockStream._group`, `_flush_groups`): the frame's adds and
  removals of a group's colliders gather and go in with one `SetTargets` before the step. This
  took the 100 m run from 239 warnings to 86.
- **A pooled prim is never rescaled.** Rock sizes come on a ladder of 10 % steps
  (`rocks.SIZE_STEP`, `size_rung`), and a parked prim is reused only by a rock of the same
  shape and size, as it is. 800 m at 80 km/h: no warning. A 10 cm rock's height is at most
  5 mm off the size Golombek and Rapp's law drew for it.
- `RockStream.remove` empties both groups' includes.

The cost: the pool finds a parked prim of the same shape and size less often. Over 800 m at 80
km/h, 624 prims made and 880 reused, against 276 and 1237 before. 20 % steps gave 529 made;
not worth the coarser sizes.
