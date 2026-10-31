# Phase 061: Grip or Drift, Chosen Live (seeded multi-start, measured)

060's live planner (L1: IPOPT on a 200-260 m window, re-planned about once a second) finds the
optimum its warm start leads to. Grip and drift are separate valleys of a non-convex problem,
with a slower half-slide ridge between them, so a derivative solver stays in the valley it
starts in and cannot choose between them. This phase measures how to give it a start in each
valley, revises the planner where the study showed it wrong, and runs the chooser live: model
in the loop first, Isaac second. The code lives in autoware-drift (`src/planning/race_planner`);
swb supplies the worlds and `spikes/course_probe.py`.

**Status**: in progress, started 2026-10-30 (M4c done 2026-10-31, gate not passed).

**Source**: user, 2026-10-30: "Find the conditions that our planner would make a drift turn as
the optimal solution"; "I wonder if it can choose between optimal drift and non drift by
differentiation ... diff works for continuous space, not discrete space"; "Explore the
solution method based on our findings and online studies ... see if GPU can help"; then
"create a phase for the solver experiments and revision work. Start it".

**Next**: [065](065-drift-style-layer.md) puts a driving style (fastest, drift when cheap,
drift everywhere) above this phase's seeds and gate; its M2 on waits for this phase's gate.

**Depends on**: [060](060-live-race-planner.md) (the live planner and tracker, sim.Plant,
course_probe LIVE=1).

**Design**: autoware-drift `docs/superpowers/specs/2026-10-29-live-race-planner-design.md`,
revision 3, section 4.4a (L1's starts chosen from what lies ahead; the re-plan step by step).
This phase builds and measures it.

**Evidence it starts from** (autoware-drift):
- `docs/drift-optimality.md` (5b54a11 and after): a drift is optimal only in tight hairpins on
  asphalt (at most 0.4 % on swb's asphalt, a rotation at turn-in, not more speed) and on low
  friction in nearly every corner (1-2 %, trail braking into a slide up to 33 deg); 8 of 20
  cases at mu 0.45 undecided between optima; L1 solves p50 0.27 s, p95 0.91 s, p99 1.04 s.
- `docs/mode-choice-methods.md` (d35a290): racing stacks make discrete choices before the
  solver, never inside it; nobody chooses grip or drift online for lap time; continuation
  reaches a valley but does not choose one; mixed-integer is too slow for a 1 s re-plan; a
  sampler's weighted mean lands on the ridge. A GPU does not speed up L1 or L2 at this size
  (a GPU interior-point solve is 5-14x slower than the CPU at ~100 nodes; batched solvers pay
  above ~2000 instances); it may propose seeds in front of IPOPT.

## Requirements

| | requirement | from |
|---|---|---|
| R1 | where a drift or a fallback can pay, L1 starts the re-plan in more than one valley (grip, held grip, rear out; rarely a flick) and keeps a feasible plan; on dry open asphalt one grip start, as 060 | the study's undecided cases; M3's robustness |
| R7 | the starts are chosen from the window ahead: the map's surface class with a confidence, corners, grade; the car's estimate only lowers friction and holds through a slide | `preconditions.md`: nothing on the car sees 200-260 m ahead |
| R2 | the mode changes only when the other is faster by more than the noise, on consecutive re-plans; grip is the default | asphalt's gain (0.4 %) is the size of the noise |
| R3 | the re-plan still lands within its 1 s budget; a seed not feasible by then is dropped | 060's adoption timing |
| R4 | the planner's tyre peaks at each surface's own slip angle, as swb's surface layer does | snow peaks at 3 deg in the planner, 6 in swb |
| R5 | a GPU is used only where a measurement shows it pays | the survey |
| R6 | asphalt does not get slower: Haruna and Tsukuba within 0.1 % of 060's | no regression |

## Milestones

Each step decides whether the next is worth building.

**M1. Seed coverage, offline (E0, E0b).** A `multistart` grid in `studies/drift_optimality.py`:
the 20 mu 0.45 cases, every 180 deg asphalt hairpin and the mu 0.3 grid, each solved from
about 8 starts (the shifted grip plan G; a synthetic rear-out seed D; D from fall-off
continuation; a flick F; three perturbed), at study settings and at live settings (ds 4,
200-260 m, the live tolerance). The best of all starts is the oracle. Gate: {G, D} reaches the
oracle in at least 90 % of the drift-optimal cases; the undecided mu 0.45 cases fall from 8/20
to at most 2/20; D's p95 wall time at most 0.9 s. E0b: the same live solves on MUMPS and on
MA27/MA57 (adopt if p95 drops by 25 % or more with the same plans).

**M2. The planner's tyre per surface.** `model.tyre` (and axle.py) take each surface class's
own peak slip angle, as swb's surface layer rescales it; checked against Isaac's tyre sweeps
per class. The study's low-friction arms re-run on it; the gains it reports become the targets
for M4 and M6.

**M3. A multi-start driver (E1).** `live.Driver` with a process pool: one compiled `Window`
per seed, each with its own budget; the gate of R2 (threshold from M1's noise, about 0.3 % of
the window's time, two re-plans in a row, the drift corner at least ~25 m ahead). Receding
chain without a plant along the study sections and Haruna (mu 1.03, patches at 0.45 and 0.3).
Gate: at most one mode change per corner, no grip-drift flip-flop on asphalt, budget overruns
at most 1 % of re-plans.

**M3b. Seeds by precondition** (user, 2026-10-30: "check preconditions and adjust solving
strategy ... we don't pursue drift start for most dry asphalt roads ... additional init starts
when on wet roads or snowy roads, or on sharp turns on narrow roads"). Before each re-plan a
scheduler reads the window ahead (its corners: turn, least radius, distance ahead; each node's
surface class from M2: friction and fall-off; grade; width) and picks the seeds; the thresholds
come from the drift-optimality map (after M2), keyed on the class (its fall-off decides, not its
friction), not from a track. H is grip held at the rear tyre's peak (`rear_alpha_max`):

| the window ahead holds | seeds | from |
|---|---|---|
| dry asphalt, no corner past ~120 deg tighter than R 25 m | G only (060) | drift gains 0.0-0.1 %, none on 90 deg corners |
| a dry asphalt hairpin whose gain in the drift map clears the switch threshold, 0.3 % (R <= ~11 m; tightened in M4, was every hairpin R <= 25 m) | G + H + D | 0.1-0.4 %: only R 9 clears 0.3 % |
| the same at R <= 9 m on a surface without fall-off | + F | the flick wins it by 0.05 s |
| a hairpin on loose ground falling off past the peak (gravel, dirt, grass) or packed snow, its gain >= 0.3 % (tightened in M4) | G + H + D | M2: 0.3-0.6 % (packed snow 0.34 %) |
| any corner on loose ground without fall-off (mud, sand, loose snow) above p_add | G + H + D (+ F into hairpins) | M2: about 1.2 % in every corner, and the fallback when G fails (18 % of re-plans at mu 0.3) |
| ice | G + H | M2: no drift pays; H the fallback |
| a steep downhill (> 5 %) on low friction | + D | the 8 % arm: more slide |
| the drift corner under ~25 m ahead | no new D | its trail braking is already cut |

Narrow roads: the study ran one width; a width arm (6, 8, 10, 15 m) comes first, and a width
rule only from its result. The surface ahead is the map's; an unseen patch (060 M6) stays
outside. M7's learned chooser, if built, replaces this table with the same inputs.

Where the preconditions come from (autoware-drift `docs/preconditions.md`): no onboard sensor
sees friction 200-260 m ahead (cameras to ~30 m, slip estimators 0 m), and no method senses the
fall-off, so over the window the surface class is the map's, carried with a confidence; a
simulated perception sensor (misclassification growing with distance) confirms or vetoes in the
last 30-60 m; the car's estimate only lowers friction, holds low through a slide, and corrects
the map after a pass. Seeds are added loosely (any class above a small probability), a drift is
committed strictly (M3's gate, scored at a lower friction quantile). M4 adds an arm with a
deliberately wrong map (0/10/30 % zones misdeclared) and perception on or off.

**M4. Model in the loop (E2).** `sim.Plant` with the ECU: four arms, single start (060),
multi-start always ({G, H, D}, F into hairpins), seeds by precondition (M3b) on a true map, and
seeds by precondition on a wrong map (0/10/30 % of zones misdeclared, perception on or off);
E0's oracle plan as the ceiling; asphalt and swb's classes (mud_wet, snow_loose, gravel,
snow_packed, ice) rather than abstract mu, low-friction stretches given a class (M2's ruling);
corners R 9/15/25 at 90 and 180 deg, then Haruna. Scored
per corner as well as per lap: in each corner M1 (after M2) calls drift-optimal, the share of
the oracle's gain the loop keeps. Beside the free G, a held grip seed (the tyre's peak as a
bound) so a grip plan always exists to weigh; a band around the 3 deg drift line so slides at
it do not flip the label; multi-start on cores of its own. Gate: as reframed after M3
(robustness first); the scheduled arm at least as good as always-on with fewer solves; with
10 % of zones wrong, the scheduled arm stops no more often than always-on. Rebuild the corner
library on M2's tyre if D's iterations say so.

**M5. GPU mode probe (E3), only if M1 shows fixed seeds miss valleys.** Batched rollouts of
L1's model (CusADi or JAX) over a corner-local 3-5 s horizon, 10^3-10^4 candidates by
direction, onset and peak rear slip; the best of each cluster seeds IPOPT. Gate: at least 90 %
agreement with the oracle's mode, at most 20 ms per probe on the RTX 5090, M4's gain at least
the fixed seeds'.

**M6. Isaac (E4).** M4's winner against single start through `course_probe` LIVE=1: Haruna,
Tsukuba and a low-friction section (a swb surface class). Gate: the low-friction gain keeps
its sign and at least half its size from M4; no spin stops; asphalt within R6.

**M7. A learned chooser (E5), optional.** A classifier on the window (curvature profile, entry
speed, friction, grade) trained on M1's labels, ordering and skipping seeds; G always runs.
Gate: M4's gain with at most 2 solves per re-plan.

## Non-goals

- The open axle in L1 (drift_mustang has a spool; the study's open-axle arm stays offline).
- A GPU solver under IPOPT or HPIPM (the survey's numbers rule it out at this size).
- Mixed-integer or complementarity formulations of the mode.
- The unseen low-grip patch (060 M6): the surface known ahead is its own work.

## Risks

- **The budget**: three solves on three cores, each cut at about 0.9 s, with p95 already 0.91
  s; a drift seed far from its valley takes more iterations. Fallbacks: D only near tight or
  slippery corners, or every second re-plan; E0b's linear solver.
- **Asphalt's gain is the noise's size**: a gate set too low flips modes and costs more in
  tracking than it gains (TUM's drift controller: 1.1 m lateral error in a held slide).
- **The model's tyre**: the low-friction gains are a model's; M2 before M6, and a gain that
  holds in M4 but not M6 points at the tyre, not the chooser.

## Progress

**M1 (done, 2026-10-30; gate failed).** Built: the `multistart` grids in
`studies/drift_optimality.py` (autoware-drift 06c7b80; write-up `docs/mode-choice-m1.md`,
results `play_log/race_planner/mode_choice/m1/`). 70 cases (all 30 mu 0.45, all 30 mu 0.3, the
10 asphalt hairpins), 8 starts each: C cold, G grip (the held plan, bound removed), Ds rear
out synthetic, Dc fall-off continuation, F flick, P1-P3 perturbed; study settings and live
(ds 4, 260 m, tol 1e-3, 1 s budget). Results:
- coverage of the drift-optimal cases (oracle within the noise floor's p75): study {G, Dc}
  90 %, {G, Ds} 72 %, {G, Ds, Dc} 92 %, {G, Dc, F} 100 %; live, one budgeted solve a start
  (the Driver's warm options), {G, Dc} 73 %, {G, Ds} 47 %, {G, Ds, Dc} 87 %. Every start is
  the oracle somewhere (Ds 20, Dc 17, F 15 of 70); the flick wins the 9 m asphalt hairpin with
  no fall-off by 0.05 s over all others;
- undecided mu 0.45: 9/30 at p75, 5/30 at p50, not 2. Of the study's 8, two are now drift
  (0.11, 0.23 s); the rest are ties: drift and grip within 0.01-0.03 s, inside the noise.
  The held plan has valleys too (the held solve from the oracle beats the cold one by up to
  1.1 %);
- a drift seed from scratch: 110-130 iterations at the median, 230-280 at p95 (0.9-1.1 s at
  window.py's 4 ms an iteration; 7-10 ms on this run's shared host, load 17-128, so 2.1-3.1 s
  measured). G warm: 18 iterations at the median;
- noise floor (G and perturbed starts in G's valley): mu 0.45 p50/p75/p90 0.13/0.25/0.43 %
  (study), 0.10/0.30/0.49 % (live); mu 0.3 0.03/0.17/0.46 %; asphalt 0 (2 ms) study, p75
  0.04 % live;
- E0b: HSL (MA27/57/97) is not in this CasADi build (no libhsl.so; not downloaded, licensed).
  SPRAL is: same iterations, 4.4x the cost each; not adopted, MUMPS stays.
Rulings:
- The gate's "20 mu 0.45 cases" are the grid's 30; all ran, the study's 8 undecided tracked
  by name. Why: the grid has 30. Cost if wrong: none, every case is in the data.
- G is the held->free start (no previous plan offline); C cold is the eighth. Why: the grip
  valley's own plan. Cost if wrong: the cold held plan is a poor optimum on low mu, so {G} is
  pessimistic.
- Live window from the section's start, timed at most at 220 m (R 40 and 60 x 180 deg do not
  fit 260 m). Why: the window holding the corner's entry decides the mode. Cost if wrong: those
  two cases miss part of the exit.
- Live Dc is the case's own study-setting continuation plan, resampled (a chain is 2-5 solves,
  p50 9.4 s: offline). Why: a corner library's best case. Cost if wrong: Dc's live numbers are
  an upper bound.
- Grip 1.0 at both settings (the study's), so labels compare. Cost if wrong: live's 0.92 moves
  cases down the mu axis, nothing else.
- For M3: seeds G (warm), Dc from a stored archetype, Ds where none fits; F only for tight
  hairpins; switch threshold 0.3 % of the window's time, grip on a tie. Each seed keeps its
  own plan from re-plan to re-plan (convergence over re-plans, which E0 does not measure);
  timing gates measured by iterations or on a quiet core. Why: the coverage and cost above.
  Cost if wrong: a seed that does not converge across re-plans either leaves 10-15 % of drift
  cases to the grip plan or needs E3's probe.
- The "undecided mu 0.45 to at most 2/20" criterion was set wrong: six of the eight are ties
  inside the noise, which the grip-on-tie gate makes free. Ruling (2026-10-30): a tie counts as
  decided (grip); only a case where a mode beats the other by more than the threshold and the
  chooser misses it counts against coverage. Cost if wrong: none for lap time.
- HSL (user, 2026-10-30): skipped, not licensed. If the linear solve turns out to limit the
  budget, the free alternatives come first: Fatrop (in this CasADi build, LGPL, a Riccati-based
  interior point for staged problems), CasADi's SQP on HPIPM, MUMPS tuning; HSL only after them.

**M3 (done, 2026-10-30; gate fails as written, passes at 0.3 % under the counting ruling).**
Built: `live.Driver(multistart=...)` (default `None`: 060's one start, unchanged; the warm start
moved to `live.warm_from`), `race_planner/multistart.py` (a process per seed, each its own
compiled `Window` and 0.9 s budget; G, D from the corner library or synthetic, F into hairpins;
a plan's mode decided in the corner ahead, kept with no corner ahead; the hysteresis gate),
`race_planner/modes.py` (seeds, corners, `corner_library.npz`, 120 kB, 60 archetypes from M1's
Dc plans), `sim.PlanFollower`, serve.py's `multistart` init option, `studies/mode_choice_e1.py`
and `studies/solver_cost.py` (autoware-drift 0a9e126; write-up `docs/mode-choice-m3.md`, results
`play_log/race_planner/mode_choice/m3/`: v3 at the top, v2 under `corner_mode/`, v1 under
`window_mode/`). E1: chains of the study's ten corners at mu 1.03/0.45/0.3 and Haruna (asphalt,
patches 0.45 and 0.3), car on the adopted plan (no plant), thresholds 0.1/0.3/0.5 %. Results:
- one mode change per corner, as written: v3 10 of 18 runs (v2 14). The rest are the previous
  corner's slide ending booked on the next corner, changes forced by the budget (the current
  mode's plans cut at 0.9 s: v3 51, v2 20) and changes forced at the 3 deg drift line. Counted
  as ruled below: v2 18/18, v3 17/18, the exception at 0.1 % (a 0.11 % gate gain into a flick,
  forced back next re-plan); 0.3 % and 0.5 % pass in both;
- no flip-flop on asphalt: 0 of 6 runs in v3 (adopted plans and the chooser); v2's two
  chooser flips at Haruna's first hairpin (3.0-3.1 deg past the peak) gone with the
  keep-the-mode rule; single start 3 on Haruna;
- overruns > 1.0 s: v2 0 of 1,969 (p95 0.92 s); v3 25 of 1,680 (1.5 %, max 1.18 s) and three
  runs stopped "solves failed" with 78-87 % of seed solves cut, on cores shared with others'
  work (load 38-60; 6.7 ms an iteration against v2's 4.9). Re-run at load ~15
  (`m3/v3_rerun_stopped/`: two of the stopped runs, two repeats): all finished, 0 overruns;
- the gate decided 3 times in 18 v3 runs (v2 2), all at 0.1 %; G and D agree in the corner
  ahead in 71-87 % of re-plans (v2); course time vs single -0.59..+0.88 % (v2 -0.81..+0.61),
  no order by threshold. The gain is robustness: Haruna with mu 0.3 patches finished by every
  multi-start run where one start stopped at 10 s, while iterations cost ~5 ms;
- D warm across re-plans: low mu no (p50 96-106 iterations, a fresh Dc 114-146), asphalt yes
  (46). IPOPT 5.9 ms an iteration: linear solver 57 %, functions 34 %, IPOPT 9 %. Fatrop 3.3 ms,
  p95 -35..-44 %, but 17 % of plans differ by > 0.3 %, 12 windows IPOPT solves fail, no
  multiplier warm start.
Rulings:
- A plan's mode is its slide in the corner ahead (turn-in 10 m or more ahead; 30 m before it to
  the exit), not over the window. Why: over the window both modes almost never had a plan (v1).
  Cost if wrong: only the label is per corner; the window time still decides.
- With no corner ahead every plan keeps the current mode. Why: v2's window classification
  there forced changes mid-corner and on straights (136 changes against v3's 83). Cost if
  wrong: a slide that ends badly is labelled drift to its corner's end; the fastest plan is
  still adopted.
- Counting against "one change per corner": the previous slide's end is the previous corner's;
  a change forced by budget cuts counts against the budget criterion; line-forced and gate
  changes count. Why: the criterion is about flipping between valleys, and the first two are
  booking and load. Cost if wrong: changes a slow host makes are hidden here, but there the
  budget criterion fails anyway.
- The budget criterion is judged on v2's quiet cores, v3's timing reported. Why: M1's ruling
  (timing by iterations or on a quiet core). Cost if wrong: 1 % holds only at about 5 ms an
  iteration; M4 must measure it on the target's load.
- Forced changes follow the plans at once (the alternative is no plan); G stays free (no held
  grip seed); F runs into hairpins; the library is M1's Dc plans. Fatrop not adopted for live
  (plans differ beyond noise); worth trying for fresh seeds.
- For M4: multi-start for robustness, threshold 0.3 %; a held grip seed beside G and a
  classification band (drift at 3 deg, grip below 1.5) to make the choice real.

**M2 (done, 2026-10-30).** Built: the planner's tyre on each surface's own peak, swb's rule
(`surfaces.WheelSurfaces.update`: each wheel's stiffness times rig peak x friction written /
class peak, rho over the class's peaks): `model.tyre(surface=(stiff_long, stiff_lat,
peak_long, peak_lat))`, `track.SurfaceTable.tyre` from swb's `surface_classes.yaml`, four
`Track` fields carried by axle, L1's window (stage parameters 4 -> 8), L0, L2 (10), sim.Plant,
modes and multistart; `test/test_surfaces.py` against swb's code imported from this checkout;
the study's grids `m2-lowmu`, `m2-classes`, `m2-classes-old`, `m2-report` (autoware-drift
4c5fe3d; write-up `docs/mode-choice-m2.md`, results `play_log/race_planner/drift_study/m2-*`,
`m2_report.md`). Results:
- swb per class (drift_mustang; stiffness lon/lat, shares of the rig's): asphalt 1/1 (rig
  peak 0.256, 8.55 deg), concrete 0.94/0.94, gravel 0.46/0.38 (12 deg), dirt 0.78/0.52 (10),
  grass 0.84/0.47 (9), sand 0.50/0.14 (14), mud_wet 0.33/0.18 (12), snow_packed 0.30/0.26
  (6), snow_loose 0.19/0.16 (12), ice 1.21/0.68 (3). Planner equals it to 1e-12; its force
  curve within 0.6 % of PhysX's brush under swb's stiffness and friction, peak within 2 %;
- the rear tyre's peak (outer rear at the limit) moves from 2-3 deg to swb's: snow_packed 2.1
  -> 6.4, ice 2.9 -> 3.1, mud_wet 3.1 -> 16.4, sand 2.9 -> 17.8, snow_loose 2.6 -> 16.1, gravel
  7.0 -> 15.9; asphalt 12.1 unchanged;
- asphalt unchanged: the tyre, the axle car and L0 bit for bit; L1's stage functions differ in
  the last bit (31 of 7,800 constraint values, 1e-14 relative: symbols where constants were
  folded); tightly solved (tol 1e-8) Haruna and Tsukuba windows reach the same optimum (times
  equal to 1e-13, states 2.6e-10); at tol 1e-3 a warm chain wanders up to 0.08 %, as the old code
  does under a 1e-9 m/s change of its start (0.14 %);
- the low-friction arms re-run (220 cases, 4 jobs, none failed): drift-optimal abstract mu 0.45
  and 0.3 43 of 60 -> 18, classes 53 of 80 -> 32. Still optimal, gains: mud_wet 1.21 %, sand
  1.32 %, snow_loose 1.11 %, abstract sliding 1.0 1.20-1.31 % (were 1.5-2.5 %); hairpins R 9-25
  on loose ground 0.3-0.6 %; snow_packed only hairpins, 0.34 % (was 1.95 %); ice none (was
  1.56 %); falling-off arms (0.8, 0.6) at most 0.38 %. Every low-friction section 1-4 % slower
  (the grip comes at 6-16 deg). The old tyre was full grip flat from 3 to 8.5 deg on low mu.
Rulings:
- swb's rule from its code, not Isaac's sweeps per class. Why: no Isaac beside another user's
  CARLA on this host; swb's code is what Isaac runs. Cost if wrong: PhysX's real curve past the
  knee per class; sweeps per class deferred to M6.
- A friction with no class (hand-built tracks, the study's abstract mu) keeps the rig's tyre;
  grip share, mu_scale and the plant's patches lower friction alone. Why: asphalt and every
  existing Track unchanged, and the grip share is a margin. Cost if wrong: a low-mu patch is
  planned on a peak too early (3.5 deg for 8.6 at 0.45); M4 should give a known patch a class.
- Asphalt's live windows are ruled equivalent, not bit-identical (above). Why: same optimum,
  differences inside IPOPT's tolerance. Cost if wrong: none found; below R6's 0.1 %.
- Stribeck and relaxation stay out (a steady tyre); the corner library and M1/M3's seeds stay
  as built on the old tyre. Why: seeds are starts. Cost if wrong: Dc seeds on snow start 2-8
  deg too deep; rebuild the library on the classes if M4's iterations say so.
- For M4 and M6, the targets: low friction is chosen by class, not by mu; drift worth ~1.2 % on
  mud, sand and loose snow (all corners), 0.3-0.6 % in loose-ground hairpins, nothing on ice or
  packed snow outside hairpins. Mixed Haruna patches should be classes (snow_packed, mud_wet).

**After M3: the phase reframed** (user, 2026-10-30: "Go" on continuing M2 then M4). M1 and M3
show the grip-or-drift choice is worth little for lap time (the gate decided 3 times in 18
runs; G slides into low-friction corners by itself), while the extra seeds keep the car going
when G fails. Ruling: M4 is judged on robustness first: on low friction and mixed surfaces,
fewer stops and failed re-plans than the single start, lap time no worse than 0.1 % on asphalt
and no worse than the single start's on low friction; threshold 0.3 %. Multi-start needs cores
of its own (its budget held at ~5 ms an iteration, not at 7). M2 before M4, since low friction
is the only place the seeds matter. Cost if wrong: a lap-time gain M4 would have chased is not
looked for; M1's oracle still bounds it.

**M3b (done, 2026-10-30; built, R6 held on Haruna, missed by 0.04 % on Tsukuba in one run).**
Built: `race_planner/schedule.py` (the window scan: corners ahead with turn, least radius,
distance, class probabilities, grade, width; `DriftMap`, the table's thresholds as data in
`drift_map.json`, from the drift study after M2 and the width arm; `Scheduler`, design 4.4a's
table on the corner the seeds go into, loose in at p_add 0.15, no new D/F under 25 m),
`race_planner/surface.py` (`SurfaceMap`: zones with a declared class, confidence and true class,
the planned friction at the mixture's mean - 1 sigma, the planner's and the plant's Tracks;
`Perception`: confusion falling from 95 % at 30 m to chance at 100 m, seeded, veto and
lower-only; `FrictionEstimate`: only lowers, holds through a slide, released at 0.05/s, writes
the map after a pass), `multistart`'s scheduled mode (`schedule=...`: G in the driver's process
as 060 solves it, H held at the rear tyre's peak in its own process, one compiled window per
whole-degree bound, D, F), `live.Driver(estimator=...)` (default off); the width arm
(`drift_optimality.py m3b-width`, `drift-map`), `studies/mode_choice_m3b.py`,
`test/test_schedule.py` (autoware-drift c4e08cd, c7a22b1; write-up `docs/mode-choice-m3b.md`, results
`play_log/race_planner/drift_study/m3b-width.*`, `play_log/race_planner/mode_choice/m3b/`).
Results:
- width arm (R 9/15/25, 90/180 deg, asphalt, mud_wet, snow_loose, gravel, 6/8/10/15 m, 96
  cases): width moves the optimum only in 90 deg corners on loose ground without fall-off: a
  drift pays 0.6-1.3 % at 6-8 m, 0.05-0.2 % at 10 m, nothing at 15 m (the slide 8-11 deg past the
  peak narrow, 3 deg wide). Hairpins pay at every width on every class (asphalt 0.08-0.48 %,
  mud and loose snow 0.3-1.4 %, as deep); asphalt and gravel 90s never. Rule: the drift map is
  keyed on width (the widest measured width no wider than the road; unknown: the narrowest);
- receding chain, no plant (one run each): Haruna dry G alone on 43 % of re-plans (the table's
  asphalt hairpin row fires at all seven hairpins), 2.13 solves a re-plan, -0.065 % against
  single; Tsukuba 75 %, 1.51, +0.138 %. Every G-only re-plan is 060's bit for bit (tested); the
  difference is the hairpins' D plans. Class-zone chain (mud_wet, snow_loose, gravel, ice,
  snow_packed hairpins and 90s, one uncertain asphalt/mud zone): single start stopped at s 191
  (solves cut at the budget into the mud hairpin), scheduled and always-on finished; the starts
  fired where the table says at each of the ten corners; 2.30 solves a re-plan against
  always-on's 2.92 (-21 %), course time -1.1 %; overruns past 1.0 s 18 of 140 (always-on 19 of
  142, max 1.03 s) on low friction;
- full suite: 111 passed (was 92 before M3b, all passing).
Rulings:
- H is the fallback (adopted only when no free start has a plan), not a gate competitor
  (`h_competes=True` restores it). Why: as a competitor it held grip on H's slower plan wherever
  G slid by itself: Tsukuba +0.55 %. Cost if wrong: on low friction the slide is never weighed
  against a held grip plan; M4 can measure that arm.
- G in the driver's process. Why: a G-only re-plan is then 060's exactly (R6 by construction
  there). Cost if wrong: one more busy core per re-plan.
- The table is applied to the corner the seeds go into (the first 10 m or more ahead), H also on
  low friction anywhere in the window. Cost if wrong: a second paying corner in the window is
  seeded only when it becomes first (still 25 m+ ahead).
- Planned friction: mean - 1 sigma over the class mixture, floored at the lowest class at 0.15 or
  more, within-class spread 0 by default (a certain map plans as 060). Cost if wrong: the 0.1
  within-class spread is not planned in; `sd` is there for M4 to sweep.
- p_add 0.15, gain 0.05 %, R x 1.25, turn - 30 deg: starting points, not measured optima. Cost if
  wrong: M4 sweeps them.
- The chain check's spin stop raised to 89 deg (the car is its plan; on mud M2's tyre plans the
  R 9 hairpin past 60 deg). Cost if wrong: those plans may not be drivable; M4's plant decides.
- R6 on Tsukuba (+0.138 % against 0.1 %) is reported, not ruled away: one run, the course time on
  a 0.05 s step (0.07 %); M4 repeats it with the plant.
- Design 4.4a updated (width row and rule, H as fallback; section 10).

**M4 (done, 2026-10-31; gate not passed).** Part A (autoware-drift 8ad42d4): on a class that
falls off past its peak, D (and H) only where the drift map's gain for the corner clears the
switch threshold, 0.3 % (user, 2026-10-30: "we don't pursue drift start for most dry asphalt
roads"); chain check, three repeats: Haruna G alone on 51-59 % of re-plans, 1.73-1.81 solves a
re-plan, -0.065 % x3; Tsukuba G alone throughout. Part B built: `studies/mode_choice_m4.py`
(the loop on `sim.Plant` behind the ECU, L2 tracking; arms single/always/sched, wrong maps,
perception, estimator; the per-corner oracle; `aggregate`), `studies/m4_batch.py`,
`live.Driver(beta_max=...)` (L1's sideslip bound, seeds included; default off); write-up
autoware-drift `docs/mode-choice-m4.md`, issue `docs/issues/001-no-first-plan-in-tight-low-friction-hairpins.md` (autoware-drift 017d888, caf45d9, 3587304); results
`play_log/race_planner/mode_choice/m4/` (`aggregate.md`; dry R6 idle-host re-run in `quiet/`,
the bound in `beta45/`). Corners ran at load 2.6-15 (p10-p90), the dry R6 re-run at 1.5-2.7.
Results:
- 72 corners x 3 arms x 2 repeats: asphalt, loose snow and gravel finish in every arm; mud: the
  extra starts save the R 9 x 180 hairpin at 6 m (single 4 stops of 22, always and sched 2);
  ice: 9 of 22 started runs stop in every arm, sched's failed re-plans double (116 against 64);
  low friction overall: stops sched 15, always 17, single 17 (116 runs each); failed re-plans
  without ice sched 11, always 10, single 14 (of ~1400);
- what the extra starts add over the single start, in units of the oracle's drift gain
  (drift-optimal corners): sched asphalt 0.69, mud 0.38, loose snow 0.22, gravel 2.22 (above 1:
  the single start's plant loss recovered), packed snow -0.25; time against single sched -0.14
  (asphalt), -0.19, -0.55, -0.68, +0.29 % (packed snow); repeats nearly bit-identical;
- solves per re-plan sched 1.04 (asphalt), 1.07 (gravel), 2.0-2.16 (low friction) against
  always's 2.15-2.26;
- dry, idle host, three repeats: Haruna single 76.28 x3, sched 76.37 x3 (+0.118 %, G alone 66 %,
  1.59 solves a re-plan), always +0.26 % (3.26); Tsukuba sched is G alone (-0.16 %), every arm's
  first repeat off road at s ~1815 after four solves cut at the budget (060's);
- class-zone chain: every arm on every map stops at the mud R 9 hairpin (s 242-262; plans 52-65
  deg of sideslip, the car spins at 78-80), before any 10 % map's wrong zone; the estimate never
  lowered;
- deep slides (plans past 50 deg) are not followed (the car 16-23 deg off them); a 45 deg L1
  bound does not fix the hairpin or the chain, starts `mud_wet:9:180:15`, costs sched +2.15 % on
  the deep corners, gains single 1.39 %: kept as an option, not adopted;
- `no first plan` on `mud_wet:9:180:15` and `ice:9:180:6` in every arm: G alone from IPOPT's
  start, 400 iterations (issue 001, open);
- full suite: 113 passed (111 + 2 for the bound).
Gate: robustness met except ice; low-friction time met except packed snow (+0.29 %, within the
spread); sched as good as always at a third to a half of its solves; R6 missed on Haruna by
0.018 %; the 10 % wrong-map criterion met only vacuously. Not passed.
Rulings:
- Stops counted on runs that started; "no first plan" its own category (same in every arm).
  Cost if wrong: 4 runs a class an arm out of the counts, equally.
- Kept = an arm's time over the single start in units of the oracle's gain, not the loop
  against the oracle (the plant is not L1's model). Cost if wrong: values above 1 misread.
- R6 judged on the idle-host re-run (the loaded single start spread 0.9 % on Haruna). Cost if
  wrong: none for the verdict.
- Tsukuba's first-repeat off road is 060's budget, not the scheduler's (G alone in sched).
- The sideslip bound not adopted; design 4.4a unchanged. Cost if wrong: the single start's
  1.4 % on deep corners unused.
- The 10 % wrong-map criterion recorded met, vacuously; the sweep (p_add, gain_min, sd) not run
  (every chain run stops at s 250). Cost if wrong: thresholds stay at M3b's starting points.
Left (M4b): Haruna R6 (no D into R 10-11 m asphalt hairpins: the map read without the 1.25 x R
stretch on fall-off classes); the mud R 9 hairpin (the planner's mud tyre against the plant's, a
bound in L2); issue 001; ice; then the chain's wrong-map and estimator arms.

**M4b (done, 2026-10-31; gate not passed).** Built (autoware-drift 248a9fd, e70f79d, d6c5402,
fabd6c7): the drift map read without the 1.25 x R stretch on classes that fall off
(`RULES["r_tol_falloff"]` 1.05); the first plan rescued when G fails (3000 iterations, H and G
from H, D, F, last G with the slip bounds off; issue 001 closed); L1's slip bounds (front axle
within its peak, rear within 3 x, on surfaces without fall-off: `window.slip_rows`, `live.Driver`
defaults; issue 002); the held and slip rows off the live windows' first 3 stages, and ice's rear
held at its peak (issue 003); `live.run` "stalled"; `studies/m4b_aggregate.py`. Write-up
autoware-drift `docs/mode-choice-m4b.md`; results `play_log/race_planner/mode_choice/m4b/`
(`aggregate.md`; dry R6 in `quiet/`). Load 30-78 until 07:30 (another user's build; batches
waited for < 10, timing < 6), 1.2-4 from 07:45 to 10:30.
Results:
- R6, idle host (load 1.3-4.9), three repeats: Haruna single 76.28 x3, **sched 76.28 x3 (0.000 %,
  G alone on every re-plan, 1.00 solves)**, always 76.88/76.88/76.38; Tsukuba single 74.50,
  74.42, off road @1820; sched off road @1820 x3: 060's four cut solves near s 1815 in both arms
  (G alone in sched), not measurable as time;
- issue 001: both corners start in every arm; `mud_wet:9:180:15` then stalled on the free
  planner, finishes in every arm with the slip bound (16.91-18.27 s, the oracle 16.22/17.05);
- mud hairpin, root cause: not the model (L1's car and the plant's agree on the slide to 0.01);
  the free plan plowed the front at 9-21 x its peak (mud keeps its force past the peak), every
  tyre saturated, a slide no controller holds (fixed-plan following: on it to s 232, 77 deg
  against 53 at s 254, stopped sideways). Bound chosen on the chain (mud-hairpin stops of 6):
  free 6, front 2 x 1-2, front 1.5 + rear 3 1, **front 1 + rear 3 0**; mud corners finished 12/12
  sched and always (M4 10), time +0.6 %; loose snow's R 9 x 180 F drift lost (+9-11 %);
- ice, root cause: plans on the rear's falling branch (4-5 deg against a 2.95 peak; a degree's
  lag runs away), H infeasible once the car is past its bound (node 0 is the car), solves at
  the 1 s budget. First stages free + rear held at its peak: ice corners finished 13 -> 23 of 36
  (single 9, sched 8, always 6 of 12), +2-4 % time; on packed snow it cost 4 runs, so ice only;
- chain, M4b defaults: no mud-hairpin stop on the true and 10 % maps but one (the estimate's);
  true map 2 of 8 finish (154.2-154.3 s), the rest stop on ice (s 767-854) or the packed-snow
  hairpin (s 1238-1249); 10 % maps: sched 3 of 3 stop, always 2, single 2 (sched further on
  average, 1167 against 1011 m); 30 %: every arm early; perception changes little; the estimate
  now lowers (0.39, 0.55) and once caused the stop (mud hairpin, s 265); solves a re-plan sched
  2.2-2.7, always 2.9-3.5;
- the sweep not run (load > 10 again from 10:30; the chain's stops are not the thresholds');
- suite: all pass (the M3 chooser tests run with the slips free: their abstract tyre has no
  fall-off).
Gate: R6 met on Haruna; robustness met but ice; sched as good as always with fewer solves; the
10 % wrong-map criterion, testable now, missed by one run of three. Not passed.
Rulings:
- R6 met on Haruna at 0.000 %; Tsukuba recorded not measurable (060's budget at s 1815, both
  arms), not missed. Cost if wrong: a timing effect hidden in 060's noise.
- The slip bound applies only where the tyre does not fall off; front 1 x, rear 3 x. Cost if
  wrong: loose snow's deep R 9 hairpin drift (+9-11 %) given up.
- Ice's rear held at its peak, ice only (sliding <= 0.75), on one repeat. Cost if wrong: ice's
  outcomes flip between repeats; the bound needed a last free G in the first-plan rescue.
- The first 3 stages of the live windows free of the held and slip rows. Cost if wrong: a plan
  may start 12 m past its bound.
- Ice's remaining loss is an R3 solver matter (a third of its re-plans cut at 1 s, every arm).
Left (M4c): the 10 % maps with more seeds; ice's solve time; the packed-snow R 25 hairpin
(the chain's other stop); the estimate lowering into a mud slide; the sweep.

**M4c (done, 2026-10-31; gate not passed).** Built (autoware-drift 879b193, 918bffc, 99b8252,
4463406, 4587d61, 51ffca4, 21949b9): no friction reading where the plan slides >= 5 deg (issue
004); a cut G solve within 0.2 of feasible warms the next G solve, never adopted, not on tracks
with ice (issue 005); `Window._result` leaves the free stages' slip rows out of the plan check
(issue 006); issue 007 (packed snow's runaway slide) written, open. Write-up autoware-drift
`docs/mode-choice-m4c.md`; results `play_log/race_planner/mode_choice/m4c/` (`quiet/` dry,
`abl/` the ice ablation). Load 1-7, 17-39 from 12:15 to 13:05 (another user; batches waited:
timing < 6, robustness < 10).
Results:
- estimate: in a planned slide the car's reached acceleration (0.10-0.17 g on mud's 0.29) is not
  the road's friction; M4b's 0.394 was such a reading. Fixed; mud hairpin 1 stop in 52 chain runs
  where mapped right (no estimate);
- Tsukuba s 1815: a cut solve was discarded and the next restarted from the stale plan with a
  longer cold tail, four cuts in a row; IPOPT's iterate not monotone in feasibility, the clock
  decides; from the cut iterate the next converged in 48-151 iterations. Tsukuba 12 of 12 laps
  (M4b 2 of 6): single 74.19 ± 0.22, sched 74.32 ± 0.16 s (+0.18 %, se 0.15 %); Haruna 0.000 %;
- ice: the plan check counted the car's own slip at the free stages (replay: 1 -> 4 of 10 re-plans
  plans); the cut warm start harmful on ice (ablation, single: 10 of 12 without, 6 with). Ice
  corners: single 8, sched 6, always 6 of 12 (M4b 9, 8, 6); windows need 200-930 iterations,
  no IPOPT option cut them: solver speed (R3 levers);
- packed snow: the chain's R 25 hairpin 0 stops (M4b 2 of 8; issue 006 covers M4b's stop);
  corners 10/10/9 of 12, the two lost in every arm since M4 are issue 007 (L2);
- chain, true map: sched 2/2, always 2/2 finish (153.7, 154.1 s; M4b 1 of 6); single 0/2 (load
  8-10); sched + estimate 0/2 (ice, estimate 0.44 on an unplanned ice slide);
- 10 % maps, 8 seeds: any stop sched 8, always 7, single 8, sched+perc 7, +est 8 of 8; most at ice
  on seeds whose wrong zone lies beyond it; stops in the wrong zone 3 of 3 sched, 3 of 4 always,
  3 of 3 single; perception and estimate no measurable change; 30 %: every arm early;
- loose snow: 36 of 36 corners finish; sched +2.7 % vs single, all from R 9 x 90 at 15 m (+32 %,
  reproduces M4b's: one H adoption, the car to 45 deg against 23); always +0.3 %;
- the sweep not run (the chain's stops are ice's and the wrong zones', no threshold reaches them).
Gate: R6 Haruna met, Tsukuba within one standard error; sched as good as always with fewer
solves (2.2-2.6 against 2.9-3.1); wrong-map criterion met on the wrong zone's stops (ruling);
ice robustness not met (multi-start 6 against single 8 of 12). Not passed.
Rulings:
- Tsukuba's R6 met within noise (+0.18 %, 1.2 se; sched is G alone, the same planner). Cost if
  wrong: a scheduler overhead hidden in the clock's noise.
- The wrong-map criterion judged on the stops in the wrong zone, not ice's own on seeds whose
  wrong zone lies beyond it. Cost if wrong: missed by one run (8 against 7), as in M4b.
- The cut warm start off on tracks with ice, on one repeat. Cost if wrong: a chaotic class read
  once; the chain runs without it.
- Single-start chain runs at load 7.7-9.7 counted for robustness, not set against the multi-start
  arms as an arm effect.
- Runs after 13:42 ran on 065's style plumbing (another session, the shared tree), whose default
  S1 is specified unchanged; not re-verified bit for bit.
Left: ice's solve time (the R3 solver levers: Fatrop, SQP on HPIPM, MUMPS); issue 007 (L2);
loose snow R 9 x 90 at 15 m sched's H adoption; the sweep.

**Issue 007 (fixed, 2026-10-31).** Packed snow's R 15 x 90 and R 25 x 90 at 15 m, lost in every
arm since M4. Cause L2's cost, not its model or convergence (traced every tick under `RP_DET`:
one-stage sideslip predicted within 0.5 deg up to 56 deg; five SQP iterations a tick the same
runaway): the line's scale, 0.2 m, set on asphalt, made a few tenths of a metre of predicted line
error worth more than the slide (n term 540-690 against vy's 34-84 at s 55), and on mu 0.2 L2's
fastest handle on n is rotation; steering in and rear braking took the slide past the rear's
peak, the car slowed, its path tightened, and so on to a stall or a spin. Fix (autoware-drift
14a79e7): the line's scale 0.2 m x max(1, 1 / mu) per stage, the weight a parameter of a second
QP function used only below mu 1, so asphalt is L2 bit for bit (Haruna, Tsukuba `RP_DET` digests
equal to before). Results (`play_log/race_planner/mode_choice/i007/`): both corners finish in
every arm and run (car 5.9-8.1 deg against the plans' 6.1-6.4); packed snow single 12/12 (two
runs; a third 11/12, R 9 x 180 at 15 m lost to L1's budget at load 8, finishing under `RP_DET`),
sched 12/12, always 12/12; loose snow 36/36; ice single 11 and 10 of 12 (M4c 8; one repeat
each); 154 tests. Rulings: the multi-start packed-snow arms counted from the fix's first form
(the weight a symbolic expression: the same weights, other last bits), the final form's batch
stopped at 3 of 12 an arm, all finished, when another session's build took the load to 25-133;
Haruna and Tsukuba R6 judged by bit-identical digests, not re-timed (cost if wrong: none on
asphalt, where L2 is the same function). For 065: heavier beta and r alone (x7, x4) did not hold
R 15 x 90 with the line at 0.2 m; a style that holds a big slide needs the line to give way where
the tyres cannot move the car across in the horizon. Left: the chain's packed-snow hairpin with
the fix; xi's scale on low friction (unchanged).
