# 008: aib's vehicle-check waits forever once the host's loopback carries multicast

**Severity**: low. **Status**: fixed (aib `justfile`). **Found**: 2026-10-25, roadmap 055
step 6.

## What is wrong

`just vehicle-check` in aib starts the Isaac ego with the caller's `CYCLONEDDS_URI` (this
workspace's `config/cyclonedds-localhost.xml`: loopback unicast, `DomainGain` 1000), then
sources Autoware's `setup.bash` and waits for `/vehicle/status/velocity_status` with
`ros2 topic echo`. Autoware 1.5.0's `setup.bash` replaces `CYCLONEDDS_URI` with its own
profile (`/opt/autoware/1.5.0/config/cyclonedds.xml`) whenever loopback multicast is on and
the sysctls are tuned (`_autoware_check_network`). With two profiles the two sides use
different port maps, never discover each other, and the wait loop has no limit: the check
never starts. When multicast on `lo` is off, Autoware leaves the variable alone, which is why
the recipe worked when it was written (051 K5).

aib's `just autoware` already keeps the caller's profile across that `source`; `vehicle-check`
did not.

## How it was confirmed

On 2026-10-25 `lo` showed `MULTICAST` (`ip link show lo`). The Isaac ego published on domain 1
(`ros2 topic echo --once /vehicle/status/velocity_status` under the localhost profile returned
a message), while the recipe's waiter had
`CYCLONEDDS_URI=file:///opt/autoware/1.5.0/config/cyclonedds.xml` in `/proc/<pid>/environ`
and waited 30 minutes. An earlier attempt on domain 41 waited two hours, then the check's own
nodes saw nothing and reported "no vehicle side".

A second cause hid behind the first: the waiter's `ros2 topic echo` goes through the ROS 2
daemon, and the daemon keeps the environment of whichever command started it. The first failed
run had started one per domain under Autoware's profile (domains 41 and 1, still alive hours
later), so restoring the variable alone still waited 40 minutes.

## Fix

`vehicle-check` saves `CYCLONEDDS_URI` before sourcing Autoware and restores it, as
`just autoware` does, and its waiter runs `ros2 topic echo --no-daemon`, so no daemon's
profile can hide the ego. The stale daemons were killed. The superproject's recipes are not
affected: each exports the localhost profile after every `source`.
