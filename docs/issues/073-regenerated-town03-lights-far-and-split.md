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

## Status (2026-11-20)

Both fixes are in, and the candidate regenerated with them is ready for the Town03 dataset revision
(which is the user's call).

1. The share pass: scensim_carla_world bbcb652, pinned in 82861cdf. On the candidate with it, s5
   mapped 156 of 156 light ways, and B's decision matched the reference at every gap (without it
   every gap yielded: the junction's lights were unknown, and Autoware stops for an unknown light).
2. `far_traffic_lights` in `carla_map_gen.repair` (acb 9cb0241): an element whose every light is
   beyond 60 m of its stop line is removed, with its stop line and the light ways nothing else uses;
   a mixed element keeps its near lights. A first version (a13efb4) only took such elements off
   their lanelets and kept them, and Autoware's validator counted each as
   `traffic_light.missing_referrers` (swb 241, autoware_tools 0.3.0's validator, `-p
   transverse_mercator --lat 0 --lon 0`):

   | check | 068 candidate (acb 6c0325f) | a13efb4 | 9cb0241 |
   |---|---|---|---|
   | crosswalk.missing_regulatory_elements | 73 | 73 | 73 |
   | stop_line.missing_regulatory_elements | 0 | 0 | 0 |
   | traffic_light.missing_referrers | 0 | 37 | 0 |

   The 73 crosswalk errors are issue 071. On Town03 the repair removes 37 elements (with their 37
   stop lines and 52 light ways), trims 14 and removes the 21 light ways only the trimmed groupings
   referred to: 57 elements remain, every one referred to by a lanelet.

### The published towns

`download_maps.sh` runs the repairs on the published towns too. On v2026.11b `far_traffic_lights`
removes 7 elements in Town01, 2 in Town02 and 2 in Town05, and none in Town03 or Town10. T3 on those
towns as a13efb4 left them (CARLA 0.9.16, quiet-gated) compared as the reference in all 11 suites, 34
runs: basic town01 (6), towns (4), episode_change, tutorial; set050 default (8), bench, two_av; demo
incidents (5), i6_centerpoint (3), cut_in_rear (2), tour. 9cb0241 changes no mapping that remains:
the signal tables of Town01, Town02 and Town05 under both commits (`--generate-signal-table`) map
every remaining way and element to the same CARLA light; only the removed elements' 11 light ways
are gone, and no scenario or testbench names one. So the T3 above stands for 9cb0241.

### s5 becomes a signalized left turn

The candidate signalizes junction 861: B's approach (lanelet 9138) refers to element 138167 and A's
(5323) to 138155; the published Town03 has no light there. With the share pass the lights are mapped
and GREEN, so B meets a permissive green where the references met an unsignalized junction. Decided
(133, 2026-11-20): keep the lights as the map has them. s5 commands every light way of junction 861
green at Init (the converter groups one pole's heads across approaches, so a red cross street would
set one CARLA light both ways; s5 has no cross traffic), and s5 is re-referenced with the Town03
revision.

One s5 run on the candidate with the share pass (g6.5 r1) had a contact while B yielded and never
entered A's lane. It did not come back: at g6.5, 1 contact in 12 runs on the candidate (10 with the
share pass, 2 without) and 0 in 12 on the published map. Measured from A's truth log in 8 of them
(4 per map), B waits for A with its front up to 0.5 m (candidate) and 1.1 m (published) inside A's
lanes, at x -78.5 to -79.1, y -9.1 to -7.2, heading 98 to 104 deg, and A passes it with 3.1 to 3.9 m
between corners. That is Autoware's yield position on both maps, not the lights or the lane shift
(068): B's lanes are the same on both maps, and A's spawn is off its lane toward the side away from
B. The contact needs B about 3 m further in than any of the 8, so it reads as one run where Autoware
crept or stopped late; its track was pruned (issue 080). s5 is treated as signalized and noisy.

On the candidate regenerated with 9cb0241 (`/var/tmp/aeon/i073b-town03` on newslab-server243, NAS
`home/i073b-town03`), with junction 861 commanded green, s5 decided as the reference at every gap
(12 runs) with no contact, and c2b compared as the reference (24 runs). The T3 compare reads s5
as moved, for one run: at 7 s, where the reference splits (one go with a contact, one yield), both
runs yielded, so physical_verdict passed where the reference's go failed. The signal table maps 83
of 83 light ways in 57 elements (one CARLA light unmapped).
