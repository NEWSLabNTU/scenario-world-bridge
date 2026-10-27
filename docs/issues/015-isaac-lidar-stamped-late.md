# 015: the Isaac ego's lidar scans are stamped 0.13 s after they were taken

**Severity**: high at speed (every localization estimate lags the car by its speed times
0.13 s: 2.6 m at 20 m/s, 3.6 m at 100 km/h). **Status**: fixed (aib: the scan takes the stamp
of the frame its data is from). **Found**: 2026-10-27, roadmap 057, Town05 lap 19's proof
video (the recorder's truth against NDT's poses).

## What is wrong

aib publishes an RTX lidar's scan one frame late with the stamp of the frame it read it in
(`aib.isaac.sensors.Lidar.publish`, `docs/design.md` "the lidar one frame late"). But the
annotator (`IsaacExtractRTXSensorPointCloud`) does not hold the frame just rendered: its data
is two renders old, which aib already knows at a spawn (`fresh = 2`, "the annotator holds the
frame the last vehicle's lidar saw: two renders of its own"). So each scan carries a stamp two
frames (0.10 s) younger than the world it shows, and NDT, which stamps its pose with the scan's
stamp, places the car where it was 0.10 s before that stamp (0.125 s counting the sweep's middle,
half of a 50 ms rotation earlier).

## How it was confirmed

Lap 19 (`just isaac-world record:=`, Town05, drift_mustang at 6-22 m/s): NDT's poses against the
recorder's truth at the pose's stamp plus an offset, 60 poses between 15 and 25.5 s:

| offset | -0.22 | -0.17 | -0.15 | -0.13 | -0.11 | -0.09 | 0 |
|---|---|---|---|---|---|---|---|
| mean error, m | 1.23 | 0.47 | 0.35 | **0.34** | 0.49 | 0.79 | 2.46 |

The error at the stamp grows with the speed (the proof video's localization panel: 4 m at
24 m/s), the EKF's with it (it fuses NDT), and neither is the matching's fault: NVTL stays at
3.2. Two frames of annotator latency (0.10 s) and half a sweep (0.025 s) make 0.125 s.

At town speeds this was hidden in the noise (0.1-0.3 m at 2-3 m/s). On the ring it put the
drift controller's position and its NDT-based sideslip 0.13 s in the past while it held a
drift at 20+ m/s; the hold's steering oscillated between full locks and the car hit the
inner barrier (lap 19).

## Fix

The scan takes the stamp of the frame its data was rendered in: aib keeps the last frames'
stamps and stamps a scan `latency` frames back (the rig's `isaac.latency_frames`, default 2,
the annotator's). It still goes out one frame after it is read, so the EKF pose of its stamp
is in ndt_scan_matcher's buffer before it (roadmap 038's race). The half sweep is left: the
cloud carries no per-point time, as CARLA's does not.

## Verified

Lap 20 (the same ring, drift_mustang at up to 22.5 m/s, with the fix): NDT's poses against the
recorder's truth, 140 poses above 8 m/s:

| offset | -0.13 | -0.10 | -0.05 | -0.025 | 0 | +0.025 |
|---|---|---|---|---|---|---|
| mean error, m | 2.24 | 1.70 | 0.82 | 0.41 | **0.12** | 0.51 |

The best fit moved from 0.13 s before the stamp to the stamp itself, and the error there from
2.46 m to 0.12 m. The half sweep did not show: at 0.12 m it is below what NDT resolves.
