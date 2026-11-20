# 074: czp publishes its velocity limits in a type Autoware 1.5.0 does not subscribe to

**Severity**: high. czp's velocity limits never reached Autoware. That covers the probe's creep speed (1.0 m/s) and the stop
before a zone that no RTC module covers. So every creep czp asked for ran as a plain ACTIVATE, at Autoware's own
speed. The watchdog's clear never arrived either. **Status**: fixed on p2g-launch. **Found**: 2026-11-20, by 241,
in swb 133's issue 069 grid (NAS `logs/2026 SSv2 on CARLA/2026-11-20 078 issue 069 grid left t1.5p5 (133)`).

## What happens

czp's node (`czp/node.py`) publishes `tier4_planning_msgs/msg/VelocityLimit` on
`/planning/scenario_planning/max_velocity_candidates` and `VelocityLimitClearCommand` on
`/planning/scenario_planning/clear_velocity_limit`. Its watchdog (`czp/watchdog.py`) publishes the same clear.

Autoware 1.5.0's `external_velocity_limit_selector` subscribes to both topics with the
`autoware_internal_planning_msgs` types (`external_velocity_limit_selector_node.hpp`, lines 33-34). The fields are the
same, but the type names differ, so DDS treats them as two topics and the selector never receives czp's messages.
Nothing fails loudly. The one sign is in the bag recorder's log of every run since P2:

```
[ERROR] [ROSBAG2_TRANSPORT]: Topic '/planning/scenario_planning/max_velocity_candidates'
    has more than one type associated. Only topics with one type are supported
```

Most runs' bag facts list no czp limit at all, because the recorder took Autoware's type. One run of P2g's grid
(`on_dt0_t1.5p5_s7`) recorded czp's, which only shows that it was published.

In the 069 grid's contact (`on_dt1_t1.5p5_s13`), B rested 0.709 m from Z. After the creep's ACTIVATE it accelerated
freely, at up to 0.9 m/s^2, and was at 1.1 m/s when it entered Z. Under a 1.0 m/s cap it would have crept.

## Cause

The VelocityLimit messages moved from `tier4_planning_msgs` to `autoware_internal_planning_msgs` before Autoware 1.5.0,
and czp was written against the old package. Autoware 1.5.0 still ships `tier4_planning_msgs`, so the import works
and the node starts.

## Fix

czp's node and watchdog import both messages from `autoware_internal_planning_msgs`, and `package.xml` depends on it.
`test_strategy.py::test_velocity_limit_types_are_autoware_s` pins the type: it reads the imports of `node.py` and
`watchdog.py`. Where Autoware 1.5.0 is installed, it also checks the selector's header for the same package.

The stop limit for a zone no module covers (`sqrt(2 b (s_entry - 2))`) was never live before this fix either. It is
live now.

## What it changes in earlier numbers

Every czp run on Autoware so far ran without its velocity limits: 078 P2 to P2g, the 066 rerun, the 069 grid, and
139's 074 U3 stage 1. Their creeps were uncapped ACTIVATEs. The probe's stop margin (issue 069 part 1, 0.1 m +
1.0 s x v) was fitted on such creeps, so it is refitted with the cap in force. The 078 doc carries this as a caveat.
