# 017: NDT loses the Isaac ego at 30 m/s on Town05's south straight and does not recover

**Severity**: medium (ends a run above ~29 m/s; nothing in town traffic goes that fast).
**Status**: open; worked around by holding the ring's laps to 28 m/s. **Found**: 2026-10-27,
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

## Open questions

Why one scan at 30 m/s converged badly: the scan itself (RTX lidar at that speed), the
lockstep's timing of that scan, or the initial guess. And whether NDT should re-align from a
wider search (Autoware's NDT has no such recovery in autonomous mode). Worked around by
`max_velocity` 28 m/s on the ring until then.
