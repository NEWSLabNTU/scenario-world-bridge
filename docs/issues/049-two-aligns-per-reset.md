# 049: Every reset after the first runs two NDT aligns back to back

**Severity**: low (about 7 s more per reset; every run passes). **Status**: open, the fix not
designed; the source of the second align has a leading candidate below, not yet proven.
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

## Open

- Prove the source (log the caller, or run T5 with acb_bridge's re-seed off).
- Then one initializer per reset for a managed ego: the agent initializes at the scenario's
  pose already, so acb_bridge's re-seed may be redundant when an agent is registered (it is
  needed stand-alone, without one). The fix lands where the double request comes from.

## Evidence

- `~/nas/autoveh/logs/2026 SSv2 on CARLA/2026-11-18 post-landing T1 T5 (241)/`:
  `script-out/aligns-t1-t5.txt`, `stack/proc/pilot-ego/console.log` (the lines above).
- `~/nas/autoveh/logs/2026 SSv2 on CARLA/2026-11-18 080 exit T1 T5 (241)/aligns-t5.txt`.
