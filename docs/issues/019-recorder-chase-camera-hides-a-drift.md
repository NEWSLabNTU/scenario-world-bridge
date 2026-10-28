# 019: the recorder's chase camera turns with the car, so a drift does not show in it

**Severity**: low (recording only; the runs are unaffected). **Status**: fixed (swb_isaac_world 24bb78e:
a `course` camera that follows the car's velocity); first run on a GPU 2026-10-29 on 243 (the
materials demo, Haruna C9a), where it works as designed. **Found**: 2026-10-29, roadmap 057, Town05
lap 32's proof video.

## What is wrong

`just isaac-world record:=<dir>` films the ego with a chase camera under the ego's body prim
(`recorder.py`: 7.5 m behind, 3 m to the left, 3 m up, in base_link). PhysX carries it with the
body, so it yaws with the car. A drift is the car's heading turning away from its direction of
travel, and the camera turns with the heading, so the car sits at the same angle in every
picture. Lap 32's north-east drift (sideslip up to 16 deg at 26-28 m/s) looks in the chase frames like a
car driving round the bend; only the road's curve gives it away.

## Fix

A fourth ego camera, `course`, is not under the body. It is a world-level prim that the world
server places after every rendered frame. It sits 9 m behind and 3.2 m above the car, along the car's
velocity direction (smoothed over about 0.3 s; below 2 m/s, its heading), and looks 4 m ahead
of the car. The position is extrapolated one frame, so the car does not slip forward in the
picture. In it, a drifting car shows its side to the camera by its sideslip angle. Its pictures
go to `<dir>/<time>-<ego>/course/`, beside the others; autoware-drift's `lap_video.py
--camera course` shows them.
