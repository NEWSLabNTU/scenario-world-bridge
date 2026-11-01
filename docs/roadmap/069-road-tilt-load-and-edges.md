# Phase 069: Road Tilt, Tyre Load and the Road's Edge

The race planner (autoware-drift `race_planner`) sees the road as a level ribbon with a grade:
gravity along the line is in L1, L2 and the test plant, nothing else is. It does not know the
road's bank or crown, that a crest unloads the car and a sag loads it, or what lies past either
edge. Every edge is alike (one margin, `window.EDGE_PENALTY` 10 s per metre past it), and the
plant ends a run `off_road` on either side, so a crash and a recoverable excursion score the
same ([autoware-drift issue 014](https://github.com/jerry73204/autoware-drift/blob/main/docs/issues/014-road-edge-consequence-unknown.md)).
swb's courses agree with it only because they lack the same things: the ribbon is level across
(`course/geometry.py`), and the guardrail rule in `course/props.py` computes where the ground
falls away and then throws the answer away.

This phase gives the stack one description of the road's section and edges, read by the course
builder, Isaac, the planner and the plant; puts tilt and load in the planner's equations; and
makes the planner price an edge by what is past it. The Isaac geometry (banked ribbon, the
section per station, the edge lip) is 066's stage R8, built on this phase's record.

**Status**: planned, 2026-11-02. Design only; nothing built.

**Source**: user, 2026-11-01: "Does the road physics model take the slope into account? That
is, running on a flat road and on a cliff would lead to different consequences"; 2026-11-02:
"Let's see how to model side ways road tilt and tyre load", "I have an off-road project running
in parallel, which should model road entering and exiting effects. We can study this and
propose a design, and leave phase docs for implementation later"; "Take your advice on where
the work items live".

**Survey**: `reports/Road tilt load and edge transitions.md` (notes in
`research_notes/Road tilt load and edge transitions/`). Its recommendation section is this
phase's design; read it for the sources.

**Depends on**: 061 and 065 (the planner, the styles' gates), 066 R3 (`road_detail`'s section).
**Feeds**: 066 R8 (Isaac's geometry), the rally stage (067).

## What the survey settled

- **Crests cost more than bank.** Research planners scale every tyre limit by the apparent
  vertical acceleration g~ = g cos(theta) cos(phi) + V s_dot (kappa_v cos(phi) + kappa
  sin(phi)) (theta grade angle, phi bank, kappa_v the vertical curvature, positive in a sag).
  Rowold et al. (IV 2023): leaving the vertical-curvature term out cost 3.3 s of a 120 s
  Bathurst lap and let the plan exceed the true grip limit by 12.6 m/s^2. At 30 m/s over a
  crest of 200 m radius, g~ is about 0.54 g.
- **Bank enters twice**: a sideways share of gravity, and through g~. A crowned road is a bank
  that changes across the road; on a 20 m hairpin a 4 % crown moves corner speed by about
  4 %.
- **Games store camber and grade but drive on recorded speed.** No game stores what lies past
  the edge; rFactor 2's four widths per waypoint (legal, driveable, collision, cut) are the
  nearest. Penalties that work scale with speed squared (Fuchs et al., GT Sophy); fixed ones
  were ignored or froze the car.
- **The lip on re-entry is the hazard, not the excursion.** A vertical 4.5 in (11 cm) edge was
  unsafe at 35 mph (Zimmer and Ivey, TTI 1982); a 30 degree taper is far milder. Gravel slows
  a car at 0.3-0.9 g. A foreslope of 1V:4H or flatter is recoverable, 1V:3H to 1V:4H
  traversable but not recoverable (AASHTO), which is 066 R3's default and matches R3's own
  80 km/h run stuck in the ditch at 794 m.
- No source prices leaving the road by its roadside, and none models a tyre climbing a lip at
  a planner's level: the classes below are this phase's own, and the lip stays the plant's.

## The design

### The record

swb's course builder writes `course/section.npz` beside `centreline.npz`, over the same 1 m
stations. Everything else reads it: Isaac's ribbon and `road_detail` (066 R8), the guardrails,
`Track.from_swb_course`, the plant.

| field | meaning |
|---|---|
| `bank` | road roll, rad, positive leaning left |
| `crown_left`, `crown_right`, `crown_offset` | each half's cross slope, down and away from the crown line, and the line's lateral offset |
| `kappa_v` | vertical curvature, 1/m, positive in a sag, from the same smoothed z |
| per side: `shoulder_w`, `shoulder_surface` | shoulder width and surface class |
| per side: `lip_h`, `lip_angle` | edge drop-off height (m) and face angle (90 vertical, 30 tapered) |
| per side: `foreslope`, `ditch_depth`, `runoff_len`, `runoff_surface` | 066 R3's section terms, per station |
| per side: `barrier_offset` | distance to a guardrail or wall, or none |
| per side: `hazard` | the class, from one classifier over the fields above and the DEM |

The classifier replaces `props.py`'s inline rule and keeps its intent (ground more than `DROP`
down within `DROP_AT`, or the outside of a bend tighter than `RAIL_RADIUS`, gets a rail), and
writes the result down. Fields are exact for generated roads, derived from the smoothed
profile for DEM courses, and fitted by a curvature-penalised smoother for scanned twins (a
noisy section breaks a minimum-time solve: Limebeer and Warren 2022).

### Hazard classes

| class | defined by | planner (L1, L2) | styles | plant outcome |
|---|---|---|---|---|
| `shoulder` | flush paved or gravel shoulder | in bounds at the shoulder's mu; a wheel on it puts its axle on the lower mu | all may use it | continues, friction per wheel |
| `runoff` | 1V:4H or flatter, length and surface known | soft exact penalty plus a speed-squared term | margin m0 | `runoff`: recoverable, time lost scored |
| `lip` | `lip_h` over about 5 cm, face steeper than 45 deg | wheels kept on the pavement that side; re-entry only below a speed cap and above a heading angle | m1; S3 plans no slide toward it | `lip`: shallow fast re-entry scored `upset` |
| `ditch` | foreslope steeper than 1V:4H into a ditch | exact penalty, large weight | m1; S2, S3 no planned slide that side | `ditched`: stop |
| `barrier` | barrier within a few metres | exact penalty at the barrier less half the car, speed-squared contact cost | m1; S2 no slide that side; S3 lower grip share | `barrier`: crash, impact speed logged |
| `drop` | ground over 1 m down within 3 m, steeper than 1V:3H, no barrier | hard row, largest margin | m2; no planned slide; S3 grip share capped | `crash`: run ends |

Margins m0 < m1 < m2 are half the car plus a quantile of L2's lateral tracking error for the
style, from logs (for example p95 for `runoff`, p99.9 for `drop`). Today's 0.3 m (L1) and
0.1 m (L2) become the `runoff` case. Margins are invariants no style loosens (`style.py`
already refuses that); the style column is a gate on what a style may plan.

### The planner's equations

L1 and L2 gain per stage `bank`, `kappa_v` and the crown terms, and per-side margins and
weights in place of the global `EDGE_PENALTY` and `EDGE_MARGIN`.

- Bank's gravity in body frame, beside today's grade term:
  dvx += g cos(theta) sin(phi) sin(xi), dvy += g cos(theta) sin(phi) cos(xi).
- Static loads times g~/g; the load transfers stay on the tyre-made accelerations (they are
  already specific forces in `axle.py`).
- Crown as a bank smooth in n: phi(s, n) = bank(s) + crown_left(s) sigma(n - c) -
  crown_right(s) (1 - sigma(n - c)), sigma a logistic about 0.5 m wide, so IPOPT's
  derivatives stay continuous.
- A row g~ >= g_min (about 0.3 g): no planned airtime. A crest marked as a jump lifts it and
  adds a landing speed row (not in this phase).
- Rowold's h d(omega)/dt and h omega omega terms are left out at first; the rate of bank
  change matters where the line crosses a crown, and is measured (E1), not assumed.

The plant takes the same terms with the true crown at each wheel's lateral position and each
wheel's own surface, so it stays the stricter of the two.

## Stages

- [ ] **E0. The record and the scoring** (swb_isaac_world course builder, autoware-drift
  `track.py`, `sim.py`, `live.py`; about 1 week). `section.npz` written with bank 0, today's
  constant section, `kappa_v` from z, and the classifier from today's rail rule; `props.py`
  reads it (the same rails as today, checked); `Track` reads it with zero defaults (every
  digest unchanged: Haruna 76.28 s, Tsukuba 74.33 s); the plant's `off_road` splits into
  `runoff`, `lip`/`upset`, `ditched`, `barrier`, `crash`. Buys issue 014's scoring item:
  experiments tell a crash from a recoverable stop. Nothing in planning changes.
- [ ] **E1. Load and tilt in the planner** (autoware-drift; 1.5-2 weeks). g~ and bank in L1,
  L2 and the plant; crown as bank in n; the g_min row. Experiments:
  - crest and sag fixture (straight road, vertical radius 50, 100, 200 m): each wheel's load
    in the plant against g~, take-off speed against sqrt(g R_v);
  - banked skidpad (cross slope -6 % to +6 %): steady corner speed against the bank-aware
    limit;
  - ablation on Haruna (L1 with and without g~, bank, crown): lap time and the worst tyre
    over-use the plant measures;
  - crown crossing: L2's lateral error through hairpins whose line crosses the crown;
  - L1 p95 within 0.9 s (060's budget) with the new parameters.
- [ ] **E2. The edge's consequence in the planner** (autoware-drift; about 1.5 weeks).
  Per-side margins and weights by class, the style gates, the shoulder's friction per axle;
  margin calibration from E1's logs (L2's lateral error by style). Experiments: forced
  excursions per class in the plant (run-off deceleration within 0.3-0.9 g, ditch entries
  ending stuck); every 061 and 065 result on courses without hazards unchanged.
- [ ] **E4. Deliberate use and recovery** (after E2 and 066 R8). Run-off as a planned
  resource in S3; a recovery mode triggered by an off-road score summed over a few seconds
  (GameAIPro ch. 38), driving back at a speed and heading the lip allows; pace-note export
  (crest, over crest, adverse camber, don't cut, caution) for Autoware and for human checks.

(E3, the Isaac geometry, is 066 R8.)

## Acceptance

- [ ] One record: Isaac's road height and cross slope agree with `section.npz` within 5 mm and
  0.1 deg at every station (with 066 R8).
- [ ] Crest fixture: plant wheel loads within 5 % of g~ m g / 4 per wheel; the planner
  plans no airtime and no tyre past its limit over a crest.
- [ ] Haruna ablation reported (lap time and tyre over-use with and without each term), and
  the full model's plan drives clean in the plant.
- [ ] Every excursion is scored by class; no planned slide beside a `drop` or `ditch` in any
  style; margins come from measured tracking error.
- [ ] Courses with bank 0, no crown and no hazards: plans and digests unchanged.
- [ ] L1 p95 at most 0.9 s, L2 within its 20 ms tick, in every style.

## Not in this phase

Isaac's geometry and any edge force on the tyre (066 R8); jumps and landings; drifting on
banked roads (no source found; measure after E1); tyre scrub on a lip in the planner (the
plant's); CARLA (no channel for section data).
