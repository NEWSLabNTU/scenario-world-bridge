# 020: a course world's props name the build host's CARLA_USD, so on another host it has none

**Severity**: medium (the world loads and drives; it has no guardrails or trees, and the point
cloud has them, so the world and the map disagree). **Status**: fixed (swb_isaac_world 900457c);
Haruna's world rebuilt on 243. **Found**: 2026-10-29, roadmap 058 C9a, Haruna's first GPU
run on 243 (`spikes/course_probe.py`).

## What is wrong

`swb_isaac_world.course.usd` references the guardrail and tree assets of the CARLA Isaac set
by absolute path, `$CARLA_USD/<asset>` as the build host has it. Haruna's `world.usdc` was
built on 241, where the set is `/mnt/toshiba/aeon/carla-usd`. On 243 it is
`/mnt/disk1/carla-usd`, so every reference failed to resolve. The world loaded with an empty
guardrail xform per segment and a tree PointInstancer with no prototypes; the log has thousands
of `PopulatePointInstancerBucket invalid protoIndex=... numPrototypes=0`, and the course camera
shows bare terrain beside the road. The guardrails' box colliders are the generator's own
cubes, so they were still there: invisible walls.

The map data is meant to move between hosts (it went 241 -> NAS -> 243), so any absolute path
inside it breaks the move.

## Fix

The generator references the assets through a link beside the world, relative to the layer:
`<map>/isaac/carla-usd` is a symlink to `$CARLA_USD` (the default `/mnt/disk1/carla-usd`), and
the references are `./carla-usd/<asset>`. The USD step creates or repoints the link. On a host
whose set is elsewhere, repoint it without rebuilding:

    ln -sfn "$CARLA_USD" <map>/isaac/carla-usd

The props' surface sampling for the point cloud reads the same assets through the same link.
