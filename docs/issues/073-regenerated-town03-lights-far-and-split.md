# 073: Regenerated Town03: light elements refer to far lights, and split heads stay unmapped

**Severity**: high for the next Town03 revision (068's fix): Autoware stops at signals it never
receives. **Status**: open; the split heads are fixed in scensim_carla_world (pending landing),
the far elements need a carla_map_gen change before the revision. **Found**: 2026-11-20, swb 243,
in issue 068's drive check (s5 on the regenerated Town03).

## What happens

On 068's candidate Town03 (`carla_map_gen` from 0.9.16's OpenDRIVE, its 14 repairs), s5's ego
waited at every gap, g8 and g9 included, where the reference goes. It stopped 2.5 m before the stop
line of traffic-light element 138167, which its own lanelet (9138) references, and Autoware logged
"the traffic signal data associated with regulatory element id 138167 is not received": an unknown
signal is a stop. The published Town03 references none of its lights (the validator's 38
`traffic_light.missing_referrers`), so Autoware never stopped for a Town03 light before.

Two defects meet there.

### 1. Split heads that no CARLA light claims (scensim_carla_world)

`light_facing` splits a light way that two elements share into one bar per element ("30 shared
stub(s) split into one bar per regulatory element" on Town03). For 138167 the bar it referred to,
138067, shared with 138147, became a new bar 138448. crdesigner also draws one light way per
signal head: the candidate has 156 light ways for CARLA's 38 lights. scensim_carla_world's
position matcher binds each CARLA light to one Lanelet2 element only, so 118 of the 156 stayed
unmatched, all within 0.4 m of a light another element had claimed (p50 0.3 m). An unmatched way is
never set GREEN at Initialize and never published by acb: 60 of the 94 elements had no mapped light
at all. 087's v2 Town01, made the same way, had it too (47 elements, 36 mapped, 11 unmatched).

The repair is not wrong: each element gets its own bar, on the light's own position. The matcher now
lets a way share a light another element claimed when that light is within 1.0 m
(`SIGNAL_SHARE_M`, scensim_carla_world bbcb652); distinct lights metres apart still bind one-to-one.

### 2. Elements that refer to far lights (crdesigner; a repair exists and is not run)

crdesigner groups lights from different junctions into one element. 138167 refers to its own light
(16 m from its stop line) and two lights 328 and 329 m away (OpenDRIVE 2023 and 2024), already in
crdesigner's output before any repair. On the candidate, measured from each element's stop line
(`ref_line`) to its lights:

| element's lights | elements |
|---|---|
| all within 60 m | 43 |
| all beyond 60 m | 37: the nearest 60 to 100 m in 6, beyond 100 m in 31 (p50 141 m) |
| some near, some beyond 60 m | 14, with 6 far lights at 60 to 100 m and 16 beyond 100 m |

acb publishes the lights within `ground_truth_range_m` (100 m) of the ego. An element whose lights
are all beyond 100 m from its stop line is never published while the ego waits there: the ego stops
for good, as on Town04 (roadmap 031: 69 of 128 elements). `carla_map_gen`'s `far_traffic_lights`
repair unlinks those elements from their lanelets (every light beyond 60 m), but
`carla_map_gen.repair` does not run it: it is not in `STEPS`.

Does Autoware read a far light's state at a near stop line? For the mixed elements, through acb, only
when the far light is within 100 m of the ego as well. At 138167's stop line its far lights are 328
m away, so no: the element's colour is its own light's. The 6 far members at 60 to 100 m can be in
range together with the near light, and then acb gives the element the most restrictive of the two:
a far red stops the ego at a near green.

## Fix

Before the Town03 revision:
1. scensim_carla_world bbcb652 (the share pass), with its signal table measured on the candidate,
   the published Town03, Town01 and Town05.
2. `carla_map_gen.repair` runs `far_traffic_lights`, and it drops the far `refers` of a mixed
   element too (keeping the element and its near lights), so no element refers to a light beyond
   60 m of its stop line. acb (autoware_carla_bridge) code; then the candidate regenerated, the
   validator, and s5 and c2b again.
