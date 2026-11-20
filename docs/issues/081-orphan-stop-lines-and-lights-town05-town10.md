# 081: Town05's and Town10's maps hold stop lines and light elements nothing refers to

**Severity**: low. `autoware_lanelet2_map_validator` reports them; no lanelet uses them, so driving does
not see them. **Status**: open, unassigned (swb 133, 2026-11-20), for the next revision of
`jerry73204/carla-town-autoware-maps`. **Found**: 2026-11-20, swb 241, validating v2026.11c (issue 073).

## What happens

`autoware_lanelet2_map_validator` (autoware_tools 0.3.0, `-p transverse_mercator --lat 0 --lon 0`) on
v2026.11c, the same counts as on v2026.11b:

| map | stop_line.missing_regulatory_elements | traffic_light.missing_referrers |
|---|---|---|
| Town05 | 10 | 0 |
| town05_ring_110kmh | 10 (the same ids) | 0 |
| Town10 | 21 | 2 |

Town01, Town02 and Town03 have none of either. The crosswalk errors on the same maps are issue 071.

**Stop lines no element refers to.** Each is a way tagged only `type=stop_line`, which no regulatory
element names ("No regulatory element refers to this stop line"):

- Town05 and the ring: 131344, 131347, 131350, 131353, 131356, 131359, 131362, 131365, 131368, 131371.
- Town10: 37081 to 37141 in steps of 3 (21 ways).

They spread over each town (Town05: (-273.9, 13.3), (-267.3, -16.7), (43.2, -190.1), (-109.3, 143.0)
among them, in `local_x`/`local_y`). Consecutive ids at a stride of 3 suggest one pass of the
converter. A guess (241): stop lines written for stop or yield signs that never got their element.
CARLA 0.9.16's OpenDRIVE has 5 stop signs (signal type 206) in Town05 and 3 stop signs and 1 yield
sign (205) in Town10, so the counts do not match one to one; not checked further.

**Light elements no lanelet refers to** ("Regulatory element of traffic light must be referred by at
least one lanelet"), Town10 only:

| element | light way (`refers`) | stop line (`ref_line`) |
|---|---|---|
| 37148 | 37055 | 37147 |
| 37150 | 37058 | 37149 |

Both lights sit at (-57.2, -134.2), one pole's two heads; their stop lines are 15 to 16 m west. No
other element refers to these lights.

## Fix

Open. Either give each its element (the stop lines: a `right_of_way` or stop-sign element on the
lanelets they cross, if CARLA's signs are there; the two light elements: the lanelets they control),
or remove them, as `carla_map_gen`'s `far_traffic_lights` removes an element nothing uses (acb
9cb0241). Which one needs a look at the junctions in CARLA.
