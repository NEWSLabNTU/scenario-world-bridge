# 043: A project's ground file in a fetched dataset is refused at the stack's second start

**Severity**: medium (a project whose `ground:` names a file under `assets/` starts once;
every later start is refused until `state/<map>/` is cleaned by hand; tutorial 7's exercise
2 does exactly this and then restarts the stack). **Status**: fixed in swb (this commit). **Found**: 2026-11-17,
swb 241, roadmap 077 item 21 (`scenarios/projects/isaac_rough_road`, swb 2bf9f30).

## What happens

The first `swb run --start-stack` links the file. The next start fails before the stack
parses its launch file:

```
StackRefused: .../isaac_rough_road/state/Town01/road_detail.yaml is not a link swb made:
move it into the project and name it under ground.Town01.road_detail
```

The link it refuses is the one it made itself:

```
state/Town01/road_detail.yaml -> /mnt/evo/aeon/swb-cache/datasets/jerry73204/isaac-road-physics-stages/<sha>/procedural/spikes/road_town/road_detail.yaml
```

## Why

`swb.ground.apply` links `state/<map>/<kind>.yaml` to `(project / f).resolve()`. For
`f = assets/isaac-road-physics-stages/...`, `assets/<dataset>` is itself a symlink into the
shared cache, so the resolved target lies outside the project. On the next start, `_ours`
recognises a link as swb's only when its target starts with the project's path, so the link
fails that test, and the start is refused as if a user had placed the file. A ground file in
the project proper (`ground/town01.yaml`, tutorial 7's own `surfaces`) resolves inside the
project and is not affected.

## Fix

`ground.apply` links to the file's path in the project, `project.resolve() / f`, without
resolving the `assets/` link: the target names the project, `_ours` recognises it, and the
world server reads the file through the dataset link as before. A link an older swb made to
the resolved file (into the cache) is also taken as swb's when it resolves to the same file,
so a project linked before the fix needs no cleaning. `test_ground.py`'s
`test_apply_again_with_a_file_in_a_linked_dataset` covers both; it fails without the fix.
