# 024: a course's road is folded every half metre in its hairpins, and the wheels' loads chatter there

**Severity**: medium (every course world, today Haruna: in the hairpins, where the road climbs
round a tight bend, the car's wheel loads jump by 2 kN from one 20 ms frame to the next and a
wheel can lose the ground, so the car brakes, turns and drifts on a road that is not the one
modelled; a race planner's plan and its tracking part there). **Status**: fixed (swb_isaac_world, `course/geometry.py` ROAD_STRIP).
**Found**: 2026-10-29, tracking autoware-drift's race_planner plan on Haruna s 60-620
(`play_log/race_planner/track12`, `track14` on 243; course_probe's `wheels` log).

## What is wrong

In the hairpins the four wheels' loads chatter, on the straights they do not. The suspension
force PhysX reports, per 40 m of track:

| s (m) | bend | p95 load change per 20 ms frame | power above 8 Hz | ground normal's change per frame (p95) |
|---|---|---|---|---|
| 104-224 | straights, R 68 m | 100 N | 2 % | 0.0001 |
| 264-304 | R 9 m | 2087 N | 57 % | 0.0024 |
| 384-424 | R 10 m | 2120 N | 62 % | 0.0033 |
| 504-544 | R 10 m | 2177 N | 76 % | 0.0034 |

The four loads' total swings between 0.7 and 1.4 times the car's weight frame to frame, which a
1450 kg body cannot do. The suspension never reaches its travel (jounce at most 0.12 of 0.15 m).

## Why

`course/geometry.py`'s `ribbon` makes the road one quad across its whole width (8 m) per
0.5 m centreline section, the section level across (no crossfall). On a bend that also climbs,
the two sections' level lines are neither parallel nor at one height, so each quad is twisted,
and PhysX splits it into two triangles that meet at an angle: 2-3.4 deg in Haruna's hairpins,
under 0.2 on its straights; the angle follows grade x curvature (correlation 1.00). A wheel's
raycast meets a new triangle every few centimetres of the bend and with it a ground plane
tilted by degrees; the suspension and the tyre forces jump with it.

Confirmed by elimination, each on the same plan:

- the tyre force's application point (issue 023): with it back at the wheel centre
  (`track13_centre`) the chatter is the same (45-78 % above 8 Hz);
- the car: on the manoeuvre suite's flat pad (one plane), the 40 m circle to 1.03 g at 19.5
  m/s shows none (p95 2-26 N per 50 ms frame, `play_log/model_check/circ4`);
- the road's profile (smooth, at most 1.1 m/s^2 vertical at 20 m/s), the lines (not colliders;
  the wheels saw only asphalt and, off the edge, grass), the tracker's steering steps (load
  jumps follow them no more than by chance).

## Fix

The road as strips across its width, each about 0.5 m (16 on Haruna's 8 m), so each quad's
twist is small: the fold within a quad falls from 3.41 to 0.58 deg at most, 2.08 to 0.28 on
average in the hairpin at s 384-424 (offline, from `course/meshes.npz`).

Haruna rebuilt with it (the `centreline geometry` steps, then `course.usd`; the centreline,
terrain, lines and props byte for byte as before; the road 48864 quads, 16 across) and the same
plan tracked again (`track15`):

| s (m) | p95 load change per frame, before (track14) | after (track15) | power above 8 Hz, before | after |
|---|---|---|---|---|
| 104-224 | 103 N | 94 N | 2 % | 2 % |
| 264-304 | 2087 N | 264 N | 60 % | 10 % |
| 384-424 | 2120 N | 312 N | 63 % | 12 % |
| 504-544 | 2177 N | 219 N | 75 % | 8 % |

The ground normal's change per frame in the hairpin at s 384-424: 0.0033 before, 0.0003 after.
The time the tracked car loses in the hairpins is not from this: there its inner front wheel
leaves the road at the apex (the car 1.5 m inside the plan, which runs 0.3 m from the edge).
