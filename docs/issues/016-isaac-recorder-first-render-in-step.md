# 016: the Isaac recorder's cameras render for the first time in a Step, which can take 20-34 s

**Severity**: medium (a recorded run fails before it starts, at random; unrecorded runs are not
affected). **Status**: fixed (swb_isaac_world: the recorder warms its cameras up in the ego's
Spawn). **Found**: 2026-10-27, roadmap 057, Town05 laps 18 and 21.

## What is wrong

`just isaac-world record:=<dir>` gives the first ego three cameras of its own (dash, wheel,
chase; `swb_isaac_world.recorder`), each a 960 x 540 render product with an `rgb` annotator.
The world server creates them in the ego's Spawn, but after the ego's warm-up
(`aib.isaac.ego.Ego.warm_up`, which renders until the ego's own sensors deliver). So their first
render, which sets up the render products and their RTX pipeline, falls in the first Step. The
bridge allows a Step `request_timeout_seconds` (10 s) and a Spawn `load_timeout_seconds` (120 s),
because a first render belongs in a Spawn (`bridge_config.yaml`: "an Isaac ego waits for its
sensors, 038; a body's first render compiles its materials, 044").

Usually that first Step took 0.5-1.9 s (27 recorded runs on 2026-10-27). On a loaded host it
took far longer, and the bridge ended the run:

```
[swb_isaac_world] slow step at sim time 0.10: 20013 ms (commands 13, physics and render 19639, publishing 361)
scenario_world_bridge: the isaac world server did not answer Step within 10 s; the bridge ends so its supervisor starts a fresh one
```

That was lap 21. Lap 18 failed the same way, with a 33.9 s first step. SSv2 reported "The
simulator reported a failed frame (UpdateEntityStatus, UpdateTrafficLights or UpdateFrame)",
and the recorder wrote 0 frames.

## Fix

The recorder warms itself up in the Spawn that creates it: after the ego's warm-up, the world
server keeps stepping (physics and a render, the ego on its hold brake, as the ego's warm-up
does) until every recorder camera's annotator returns a picture, at most the ego's warm-up
frame count. The Step then carries only a steady-state render. The frames this takes are in
the spawn log line's render count.
