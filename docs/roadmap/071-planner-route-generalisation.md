# Phase 071: Does the Race Planner Generalise to Any Route?

The race planner (autoware-drift `race_planner`: L0 the whole-track plan, L1 the IPOPT window of
200-260 m with its starts G, H, D, F and B, L2 the 50 Hz RTI tracker, the styles S1 and S2) takes
any road written as one ribbon: a centreline with heading, curvature and grade, the free width
either side, and friction and tyre per metre. Structurally it runs on any such route. What it
does well has been shown on few: Haruna and Tsukuba, a corner suite (radius 9-25 m, 90-180 deg,
six surfaces), and one 8 m zone chain. Several of its parts were fitted on exactly those, and
nothing has been measured on a route it was not tuned on.

This phase measures the planner on routes it has never seen, finds where it stops working and
why, fixes what is general rather than what is course specific, and keeps a held-out set that is
never tuned on.

**Status**: planned, 2026-11-02. Design only; nothing built.

**Source**: user, 2026-11-02: "Does the current planners generalize to any route?"; on the answer
(partly: general structure, fitted parts, a held-out test proposed): "Create a phase doc first".

**Depends on**: 061 (the mode gate, the chain), 065 (the styles; S2 adopted 2026-11-02 with
R6 met at 0.85 s budgets). **Feeds**: 061's gate (robustness off its own chain), 065 M5 onward,
069 (bank, crest and edges are left to it), 067 (the rally stage).

## What is general and what is fitted (2026-11-02)

General by construction: the track input (`Track`, open or closed, a closed one unrolled into
laps); G and H, solved afresh each window from the road ahead; L0; L2; the plant and the car's
ECU (sim_vehicle_ecu).

Fitted to the courses tuned on:

| part | where | fitted to | on a route outside it |
|---|---|---|---|
| corner library (the D and F starts) | `modes.Library`, `corner_library.npz` | 60 archetypes: radius 9, 15, 25, 40, 60 m; turn 90 or 180 deg; friction 0.3, 0.45, 1.12; sliding 0.6, 0.8, 1.0 | a corner further than `tol` 1.2 from every archetype gets no drift start (radius under 9 m, 45 or 270 deg turns, friction between the samples) |
| corner detection | `modes.corners` | curvature thresholds on the tuned roads | esses, chicanes, decreasing-radius corners and long sweepers untested |
| S2's prices | `budget.CoursePrices`, offline per course | Haruna, the corner courses | a new route has the generic loss map only (`budget.LossMap`) |
| tuned constants | `tracker.LINE_SLIDING` (0.8, 1.2), `window.RECOVER` 1.1 and `PROBE` 50 m, `multistart.THRESHOLD` 0.003, `AHEAD` 25 m, `SEED_LEAD`, `CORNER_FROM`, issue 017's packed-snow front row | the corner suite and the chain (issues 012, 013, 016, 017) | unknown |
| width and zones from swb | `Track.from_swb_course` | one constant half width; the road's one class ("zones are not read yet") | varying width and surface zones of a real course not seen |

Known not to hold: mixed surfaces (the chain finishes 10 of 30, 061); bank, crown, crest
unloading and the edge's consequence (069); junctions, traffic, obstacles, reversing (out of
scope).

## The design

### Three route families

1. **Synthetic** (autoware-drift `studies/route_gen.py`, new): a route from a curvature profile,
   a sequence of elements drawn from a seed: straights (20-300 m), single corners (radius
   log-uniform 6-120 m, turn 20-270 deg, either hand, clothoid entry and exit of 0-40 m),
   linked corners (esses and chicanes, 0-30 m between), decreasing- and increasing-radius
   corners, crests and dips as grade only (within +-10 %, until 069). Width per route 5-10 m,
   constant or varying along it. Length 1.5-4 km. One surface for the whole route.
2. **Mixed surface**: family 1 with a surface zone per corner, drawn from the six classes of
   `surface_classes.yaml` (asphalt, gravel, wet mud, loose snow, packed snow, ice), the zone
   starting on the approach as the chain's do; a share declared uncertain or wrong (061's 10 %
   wrong maps).
3. **Real roads** (swb course builder, `swb_isaac_world.course` from OSM and the DEM): 6-10
   mountain-road sections not used so far, chosen for hairpins, esses and long sweepers, built
   as Haruna and Tsukuba were, their width and surface zones read per node (`Track` gains
   both; the section record of 069 E0 when it exists).

Each family has a **tuning set** and a **held-out set**, drawn from disjoint seeds (real roads:
split by road). The held-out set is frozen when drawn (route files and seeds committed), run
once at G1 for the baseline and once at G4 for the result, and never used to choose a constant,
an archetype or a fix.

### Reference and measures

- **Reference time**: L0 on the whole route (the planner's own whole-track plan, no window, no
  latency), the same reference on every route. Lap time is reported as the gap to it.
- **Per route and style** (S1, S2 without course prices, S2 with them where priced), repeatable
  runs (`RP_DET`, `RP_ITERCAP` draws, `--jit`; Welch se): finished or not; time and gap to L0;
  where and why it stopped.
- **Stop classes** (one classifier over the run's records, so every stop is counted once):
  no first plan (issue 001's kind); a streak of failed re-plans; L2 at the steering lock; off the
  road inside or outside; a slide not recovered; stuck or stopped. Each stop also records the
  route element and surface it happened on.
- **Coverage**: per corner, whether the library had an archetype within `tol`; per S2 corner,
  priced or generic.
- **Sample size**: the chain showed that per-zone counts on 20 runs move by 2-3 between codes that
  change nothing there (061, 2026-11-02). So a comparison is between finish rates over many
  routes (at least 30 routes a family, 3 draws each), not per-corner counts, and is reported with
  its standard error.
- **Timed subset**: a few routes timed as 065's E3 (load under 6), for R6 on new geometry.

### Fixes, if the baseline asks for them

Fixes target what is general, judged on the tuning set only:
- the corner library extended over the gaps the coverage measure finds (radius 6-120 m, turns
  45-270 deg, friction between the samples), or interpolated between archetypes rather than
  snapped to the nearest;
- each tuned constant swept on the tuning set; a constant whose best value differs by family is
  written as a function of the road (radius, width, friction) or kept and its limit documented;
- S2 on an unpriced route: the generic map's cost against course prices measured; online
  pricing only if the gap is large.

Each bug found is written as an autoware-drift `docs/issues/NNN` before it is fixed, as usual.

## Stages

- [ ] **G0. Generator, reference and harness** (autoware-drift; about 1 week).
  `studies/route_gen.py` (families 1 and 2 from a seed, deterministic, a digest per route);
  `Track` reads width and surface zones per node from a swb course; 6-10 real roads built by the
  course builder (swb_isaac_world, `data/courses/`); L0 on every route; the stop classifier; a
  batch runner with per-route results. Tuning and held-out sets drawn and committed. Check: on
  Haruna, Tsukuba and the chain the harness reproduces today's digests and stops.
- [ ] **G1. Baseline** (about 3 days of runs). Every family's tuning and held-out sets, S1 and
  S2, repeatable runs. Report: finish rate per family and style, gap to L0, stops by class and
  by route element, library coverage. This is the answer to "does it generalise" as of today.
- [ ] **G2. General fixes** (1-2 weeks, sized by G1). The largest stop classes on the tuning set,
  root caused and fixed (library coverage, constants as functions of the road, corner detection);
  each fix checked against Haruna / Tsukuba digests, the corner suite and the chain, and its
  effect on the tuning set's finish rate.
- [ ] **G3. S2 off its prices** (about 3 days). S2 on unpriced routes against priced: time and
  drift metres; online pricing only if the gap asks for it.
- [ ] **G4. Held-out result** (about 2 days). The held-out set once more on the final code: finish
  rates and gaps against G1's, with their se. Timed subset for R6.

## Acceptance

- [ ] Held-out set run twice only (G1, G4); its route files and seeds unchanged since drawn.
- [ ] Single-surface held-out routes (families 1 and 3): S1 finishes at least 95 %; S2 at least
  S1's rate less 2 se; the median gap to L0 reported, and within the tuning set's median gap
  plus 1 %.
- [ ] Mixed-surface held-out routes: finish rate reported against the tuning set's (within 2 se),
  and against 061's chain on the same code.
- [ ] Every stop classified; every class with more than 5 % of the stops has an issue doc.
- [ ] Library coverage on the held-out corners reported; corners without an archetype finish at a
  rate within 2 se of those with one, or the gap is filed.
- [ ] Haruna and Tsukuba digests, the corner suite and the chain unchanged, or each change
  explained and accepted.
- [ ] Timed subset: L1 p95 at most 0.9 s, as 065's R6.

## Not in this phase

Bank, crown, crest load and the edge's consequence (069); junctions, branches, traffic and
obstacles; reversing; the real car, Isaac and CARLA (the plant only); a learned planner.
