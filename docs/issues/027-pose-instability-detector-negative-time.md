# 027: Autoware's pose_instability_detector died at start with a negative rclcpp::Time

**Severity**: low (once in about 15 stack starts; localization runs without it, but its
diagnostics are gone for the drive). **Status**: open, seen once.
**Found**: 2026-10-30 21:21 (+08), roadmap 059 N8 on newslab-server133, aib's runner on the
NTU twin, `play_log/ego/2026-10-30_21-20-53` (aib's checkout on /mnt/p5plus).

## What happened

31 s after the launch, `/localization/pose_twist_fusion_filter/pose_instability_detector`
exited without a code; its err ends with:

```
terminate called after throwing an instance of 'std::runtime_error'
  what():  cannot store a negative time point in rclcpp::Time
```

## Suspect

Not the clock's origin: aib's runner stamps `/clock` as swb does, sim time plus `BASE_NS`
(1000 s, `aib.ros.stamp`). More likely a start-up race: under `use_sim_time` a node's
`now()` is 0 until its first `/clock`, and the detector's timer, or a message stamped 0
(an unset header), less its time window goes below zero. A loaded host (the 15 starts ran
beside other users' GPU jobs) widens the window before the first `/clock` arrives.

## Open

Read the detector's 1.5.0 source for the subtraction; check the first stamps of
`/localization/kinematic_state` and the twist it reads at start. If it is the race, it is
Autoware's to guard (a fork fix in the overlay, as docs/issues/002), not the simulator's.
