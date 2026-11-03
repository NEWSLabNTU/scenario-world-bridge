# 017: NDT loses the Isaac ego at 30 m/s on Town05's south straight and does not recover

**Severity**: medium (ends a run above ~29 m/s; nothing in town traffic goes that fast).
**Status**: cause found (issue 002's cycle, in cuda_ndt_matcher); fix staged; laps held to 28 m/s until it is verified. **Found**: 2026-10-27,
roadmap 057, Town05 lap 24 (cuda_ndt_matcher, lockstep, issue 015's stamp fix in).

## What happened

drift_mustang reached 30.3 m/s on the ring's south straight (Autoware's `max_velocity` was
29.06; the speed overshot it by 1.2 m/s). At x -42 NDT's nearest-voxel likelihood fell from 3.02
to 2.14 between two scans, then below `ndt_scan_matcher`'s 2.0 threshold:

```
[ndt_scan_matcher]: Max iterations reached: iter=30, skip_count=193
[ndt_scan_matcher]: Score below threshold: NVTL=1.956 < 2.000, skip_count=193
```

From then on every scan was skipped (the last pose at simulated 46.7 s). The EKF dead-reckoned
(1.3-2.0 m from the recorder's truth within 5 s), its accuracy diagnostic went to WARN, the
diagnostic graph took autonomous mode away, and the MRM's comfortable stop (-2.5 m/s²)
stopped the car at (167, -192). SSv2's global timeout ended the run with no verdict.

Until that scan, NDT was 0.06-0.37 m from truth along the whole lap, the south straight included.

## What is known

- It is speed-dependent, not a map hole. Lap 23 drove the same stretch at 21-28 m/s with NVTL
  steady at 2.85-2.91, and passed. Lap 24 held 3.0-3.15 up to 30 m/s on the west part of the
  straight, then collapsed once at 30.2 m/s.
- It does not recover. A skipped scan leaves the EKF's prediction as the next initial guess,
  and at 30 m/s that guess drifts by a metre within seconds. The next scans converge worse
  (iteration limit), so they are skipped too.
- It is not the scan stamp: issue 015's fix was in, and NDT fitted truth at its stamp.

## Cause

It is issue 002's two-point cycle, in cuda_ndt_matcher this time. NDT's iterations on the scans
before the collapse, with the score still good:

| simulated s | 45.45 | 45.65 | 46.5 | 46.5 | 46.55 | 46.55 | 46.6 |
|---|---|---|---|---|---|---|---|
| iterations | 16 | 14 | 24 | **30** | 28 | **30** | **30** |
| NVTL | 3.11 | 3.07 | 3.02 | 3.01 | 3.01 | 2.74 | 2.14 |

At the iteration limit ndt_scan_matcher skips the scan whatever its score. So the first skips
came from scans that matched well (NVTL 3.0) but whose solver cycled until max_iterations (30),
as on town01_soil. The faster the car, the farther the EKF's guess is from the scan's pose and
the more iterations a scan takes, so 30 m/s is where it starts to hit the limit.

The CPU ndt_scan_matcher has issue 002's fix in the Autoware overlay: halve the step bound each
time the step reverses. cuda_ndt_matcher, which `ndt:=cuda` runs, did not: its graph kernels
clamped every step to the fixed `ndt.step_size` (0.1), with line search on or off. They counted
the reversals (`OSC_COUNT`, cosine below -0.9) and did nothing with them.

## Fix

cuda_ndt_matcher's graph kernels keep a step bound per alignment (`StateOffset::STEP_BOUND`). It
starts at `step_size` and is halved at each reversal, at the same -0.9 cosine. It bounds both the
line search's candidates and the plain clamped step. A search that never reverses takes the steps
it took before.

## Open questions (before the cause was found)

Why one scan at 30 m/s converged badly: the scan itself (RTX lidar at that speed), the
lockstep's timing of that scan, or the initial guess. And whether NDT should re-align from a
wider search (Autoware's NDT has no such recovery in autonomous mode). Worked around by
`max_velocity` 28 m/s on the ring until then.
