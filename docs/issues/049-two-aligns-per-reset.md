# 049: Every reset after the first runs two NDT aligns back to back

**Severity**: low (about 7 s more per reset; every run passes). **Status**: fixed on branch
`049-one-align-per-reset` (the cause proven by swb 118; one align per reset measured).
**Found**: 2026-11-18, swb 241, the post-landing check of Tutorials 1 and 5 on main 546b258.

## What happens

In Tutorial 5 (six runs of `follow_to_goal.xosc` on one stack), each reset of the ego runs two
NDT aligns in Autoware's pose_initializer, one right after the other: the second `Call align
server.` comes within ~10 ms of the first `align server succeeded.`. They never overlap, so
this is not issue 046's queue of timed-out calls, and Autoware's GNSS initializer did not run
(`gnss_enabled:=false` for a managed ego, 046). The agent reports the reset ready only after
the second (`localized at the reset pose after 12.4-25.3 s`).

The first reset of a fresh stack (Tutorial 1) runs one align.

| Run | Aligns | Durations |
|---|---|---|
| post-landing, main 546b258 (load gate < 40) | T1: 1; T5: 2 per reset, 12 | 13.6 s; 5.4-14.6 s |
| 080 exit, scensim-rename ac5b709 (before 046) | T5: 2 per reset, 14 | 4.1-10.6 s |

So it predates 046 and is not a regression of the landing.

## Leading candidate: acb_bridge's re-seed on a teleport

Each T5 reset follows a teleport at the episode's start, and acb_bridge logs, about 50 ms
before the agent receives its `reset`:

```
acb_bridge::autoware: Localization re-seed requested; publishing /initialpose until the estimate agrees
acb_bridge::autoware: Sent /initialpose #1 at (299.40, -129.50); the estimate was 174.8 m away
[scensim_agent]: command reset
[scensim_agent.AutowareAgent]: re-localizing at (299.40, -129.50, yaw 3.14)
pose_initializer: Call align server.        (406.77)
pose_initializer: align server succeeded.   (412.15)
pose_initializer: Call align server.        (412.15)
pose_initializer: align server succeeded.   (418.88)
acb_bridge::autoware: Localization seeded: the estimate is 0.13 m from the vehicle after 1 /initialpose
[scensim_agent.AutowareAgent]: localized at the reset pose after 12.4 s
```

Two requests reach pose_initializer for one reset: acb_bridge's `/initialpose` and the agent's
`/api/localization/initialize`, both at the scenario's pose. pose_initializer serves them one
at a time, so they align in turn. At Tutorial 1's first reset acb_bridge logs the re-seed
request but sends no `/initialpose` (no estimate yet to compare), and there is one align.
Every T5 reset shows the pair of lines and the pair of aligns. Not proven: which request
starts which align is not logged.

## Cause (proven, swb 118)

acb_bridge's `seed_localization_on_attach`, true by default for stand-alone use, publishes
`/initialpose` when acb attaches to the car (each teleport). Autoware's initial_pose_adaptor
turns it into an `/api/localization/initialize` call (its `fit_target` log line follows each
`Sent /initialpose` within 1 ms), beside the agent's own initialize at the scenario's pose. In
the post-landing T5 the counts close exactly: 7 agent resets + 6 `/initialpose` = 13
initialize requests = 13 aligns (118's write-up and trace: `~/nas/home/2026-11-18 049 align
source (118)/`). The just recipes already passed `seed_localization_on_attach:=false` for a
managed ego (justfile, `ego-av` and the stack's ego); the stack's pilot launch,
`scensim_launch`'s `vehicle_side_carla.launch.xml`, defaulted it to true.

## Fix

`vehicle_side_carla.launch.xml` defaults `seed_localization_on_attach` to false with a relay (a
managed ego, which its agent initializes) and true without one (acb as before). Test:
`test_pilots.py` evaluates the default both ways.

Measured (2026-11-18, newslab-server-241, branch ab1441a, Tutorial 5's suite on a fresh stack,
load 15-30): 6/6 passed; 6 resets, 6 aligns (4.2-7.3 s), no `/initialpose` and no `fit_target`;
the agent localized 4.6-7.7 s after each reset (12.4-25.3 s before). `~/nas/autoveh/logs/2026
SSv2 on CARLA/2026-11-18 049 one align per reset T5 (241)/`.

## Evidence

- `~/nas/autoveh/logs/2026 SSv2 on CARLA/2026-11-18 post-landing T1 T5 (241)/`:
  `script-out/aligns-t1-t5.txt`, `stack/proc/pilot-ego/console.log` (the lines above).
- `~/nas/autoveh/logs/2026 SSv2 on CARLA/2026-11-18 080 exit T1 T5 (241)/aligns-t5.txt`.
