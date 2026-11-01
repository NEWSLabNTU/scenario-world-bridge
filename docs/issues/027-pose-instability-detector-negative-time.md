# 027: Autoware's pose_instability_detector died at start with a negative rclcpp::Time

**Severity**: low (once in about 15 stack starts; localization runs without it, but its
diagnostics are gone for the drive). **Status**: fixed (the Autoware overlay:
autoware_universe f202609, branch `csb/pose-instability-empty-clip-0.48.0`; upstream main
still has it).
**Found**: 2026-10-30 21:21 (+08), roadmap 059 N8 on newslab-server133, aib's runner on the
NTU twin, `play_log/ego/2026-10-30_21-20-53` (aib's checkout on /mnt/p5plus).

## What happened

31 s after the launch, `/localization/pose_twist_fusion_filter/pose_instability_detector`
exited without a code; its err ends with:

```
terminate called after throwing an instance of 'std::runtime_error'
  what():  cannot store a negative time point in rclcpp::Time
```

## Suspect (before the source was read)

Not the clock's origin: aib's runner stamps `/clock` as swb does, sim time plus `BASE_NS`
(1000 s, `aib.ros.stamp`). More likely a start-up race: under `use_sim_time` a node's
`now()` is 0 until its first `/clock`, and the detector's timer, or a message stamped 0
(an unset header), less its time window goes below zero. A loaded host (the 15 starts ran
beside other users' GPU jobs) widens the window before the first `/clock` arrives.

## Open

Read the detector's 1.5.0 source for the subtraction; check the first stamps of
`/localization/kinematic_state` and the twist it reads at start. If it is the race, it is
Autoware's to guard (a fork fix in the overlay, as docs/issues/002), not the simulator's.

## Cause

Not the start-up race. `clip_out_necessary_twist` (0.48.0, and upstream main on 2026-11-02)
takes the twist right before the start time (`start_it`, the last stamped at or before it)
and the one right after the end time (`end_it`, searched back from the newest), and builds
`result_deque(start_it, end_it)`. When the newest twist is stamped exactly at the previous
odometry's time, both iterators point at it: the deque is empty, and the next line reads its
`front()` anyway. That is undefined; a garbage stamp with a negative `sec` makes
`rclcpp::Time`'s constructor throw this message. A real car's twist and the EKF's odometry
rarely share a stamp to the nanosecond; a simulator stamps both on the same clock tick, so
they meet now and then, which fits once in about 15 starts.

## Fix

Treat the empty range as the single-twist case: hold that twist from the start till the end.
On the fork, from Autoware 1.5.0's 0.48.0 with the blind_spot fix (one source in
`scripts/autoware_overlay.bash`), with a test: `dead_reckon` over a deque whose last twist is
stamped at the start pose's time and none later. Before the fix the pose was not dead-reckoned
at all (10.0 m where 10.1 m was due; it did not throw that time, the read being undefined);
after it, the new test and the package's three others pass.

The overlay reaches a stack that sources it: `just ego-isaac`, `ego-av`, `bg-av`. aib's
stand-alone `just autoware` (the NTU drives) runs stock Autoware unless `AUTOWARE_SETUP` is
the overlay's `install/setup.bash`; with it, the NTU r01 and r02 drives of 2026-11-02 ran on
the overlay (its libraries in the containers' maps) and arrived.
