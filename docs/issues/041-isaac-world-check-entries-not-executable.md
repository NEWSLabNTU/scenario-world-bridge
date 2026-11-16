# 041: `ros2 run swb_isaac_world road_detail` and `surfaces` say "No executable found"

**Severity**: medium (tutorial 7's first check and its exercise 2 fail on a `just build`
workspace; a plain `colcon build` is not affected). **Status**: fixed in swb_isaac_world `5e28b17` (the template committed as 100755). **Found**: 2026-11-17,
swb 241, walking roadmap 077 item 21 (tutorial 7, exercise 2): swb 2bf9f30, swb_isaac_world
1c04b37.

## What happens

```
$ ros2 run swb_isaac_world road_detail check state/Town01/road_detail.yaml
No executable found
```

`ros2 pkg prefix swb_isaac_world` finds the package, and
`install/swb_isaac_world/lib/swb_isaac_world/` lists `make_world`, `road_detail`, `surfaces`
and `swb_isaac_world_server`. Only the last one runs.

## Why

The three entries are made from one template, `scripts/swb_isaac_world_entry.in`, by
`configure_file`, which gives the generated file the template's mode. The template is
committed as `100644`, so `build/swb_isaac_world/road_detail` is `-rw-r--r--`.
`install(PROGRAMS ...)` would set the execute bits on a copy, but `just build` installs with
`--symlink-install`, and the installed entry is a symlink to the non-executable build file.
`ros2 run` lists only executable files, so it finds none. `swb_isaac_world_server.in` is
committed as `100755` and its entry works, which is why the world server never showed it.

## Fix

Commit `scripts/swb_isaac_world_entry.in` as `100755`, so every generated entry is
executable however it is installed.

Verified with a fresh `colcon build --symlink-install` of swb_isaac_world at `5e28b17`: the
four entries are `-rwxr-xr-x`, and
`ros2 run swb_isaac_world road_detail check` on the stages dataset's
`procedural/spikes/road_town/road_detail.yaml` prints `1 corridor(s)` and
`corridor 0: 250 m, ...`, as tutorial 7's exercise 2 says. A workspace built before the fix
needs `just build` again with the template's mode change; colcon may not re-run
`configure_file` on a mode change alone, so touch the template or remove
`build/swb_isaac_world` if the entries stay `-rw-r--r--`.
