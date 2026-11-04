# 035: NDT's pose buffer runs dry when the EKF publishes at the scan's rate, and a 0.2 s MRM follows

**Severity**: medium. A 0.2 s PULL_OVER MRM in the middle of a drive. The ego keeps its speed,
because the MRM clears before it brakes, but a scenario that counts MRMs fails, and a longer gap
would stop the car. **Status**: fixed in the Autoware overlay, autoware_core `879b7ce5` on
NEWSLabNTU's `csb/ndt-step-reversal-1.5.0` (autoware_localization_util). It is pinned in
`scripts/autoware_overlay.bash` and takes effect once the overlay is rebuilt. **Found**:
2026-11-04 on 133, roadmap 072's tour of the NTU map on CARLA 0.9.16: r01, 50 s in, at map minus
origin (229, 186).

## What happens

```
[INFO] [1793765656.254256702] [localization.pose_estimator.ndt_scan_matcher]: pose_buffer_.size() < 2
[INFO] [1793765656.272709192] [system.mrm_handler]: MRM State changed: NORMAL -> MRM_OPERATING
[WARN] [1793765656.274028553] [system.mrm_handler]: EMERGENCY_STOP is operated.
[INFO] [1793765656.471418351] [system.mrm_handler]: MRM State changed: MRM_OPERATING -> NORMAL
```

In the same 0.3 s, play_launch's `diagnostics.csv` has one new non-OK status:
`ndt_scan_matcher: scan_matching_status`, WARN, "Couldn't interpolate pose. Please verify that
(1) the initial pose topic (primarily come from the EKF) is being published, and (2) the
timestamps of the sensor PCD messages and pose messages are synchronized correctly",
`is_succeed_interpolate_initial_pose False`. The diagnostic graph takes
`/autoware/localization/scan_matching_status` into the autonomous mode's `and`, so one WARN scan
makes the mode unavailable and mrm_handler raises the MRM. The next scan clears it.

On a stack's start the same line comes twice per scenario, right after the initial pose. That
is expected, because the buffer is cleared there. In the middle of a drive it is this issue.

## Why

Measured with `scripts/ntu_carla/tour_clock.py`, which logs the arrival time of /clock, the EKF's
biased pose (the pose ndt_scan_matcher buffers) and the NDT input scan, in the ego domain:

- /clock is steady: 20 Hz, wall step p1-p99 47-53 ms, real-time factor 1.000. **No burst.**
- The EKF publishes once per clock tick. Its timer asks for 50 Hz, but a sim-time timer fires at
  most once per /clock message, so it runs at 20 Hz: 787 poses for 787 ticks.
- NDT's input scan also comes once per tick (CARLA's lidar turns once per 0.05 s frame), about
  31 ms after its tick. The EKF pose comes within 1 ms of its tick, but its p100 is 45 ms.
- `SmartPoseBuffer::pop_old(scan_stamp)` drops every pose older than the scan it has just used.
  Replaying the arrival order through the buffer's rules gives 2 poses at 734 of 762 scans, 3 at
  26, and fewer than 2 at 2: at each scan the buffer holds exactly the minimum
  `interpolate()` needs. If one EKF pose arrives after its scan, the scan fails.

On a car, the EKF runs at 50 Hz against a 10 Hz lidar, so the buffer holds about 5 poses, and a
late pose does not matter. Here the margin is zero because one clock drives both at the same
rate. CARLA's side is not bursting.

## Fix

`pop_old` keeps the newest pose older than the target as well
(`localization/autoware_localization_util/src/smart_pose_buffer.cpp`). The next scan then has
the pose before the last scan and the last scan's own, so it interpolates: with `time_last <
target`, `interpolate()` takes the last pose, as upstream already allows. The new unit test
`pose_for_the_scan_arrives_late` covers it, and `pop_old`'s test now expects one pose kept. The
overlay carries the package, which other packages also link against, with the same ABI.

The diagnostic-graph alternative was a `hysteresis: 0.5` on the `scan_matching_status` diag unit
in the profile's `autoware-sim.yaml` (0.48's aggregator supports it). It masks the WARN and not
the cause, and it changes every scenario's profile. It is on hold pending the user's decision,
and it would need the Town01 regression set run before and after.

## Check

After `just build-autoware-overlay` and a fresh ego stack:

- `grep localization_util /proc/<ndt_scan_matcher container pid>/maps` names the overlay's
  library;
- no `pose_buffer_.size() < 2` after the two lines at a scenario's start;
- no MRM in tour_monitor.py's log while driving.
