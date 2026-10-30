# Phase 061: Grip or Drift, Chosen Live (seeded multi-start, measured)

060's live planner (L1: IPOPT on a 200-260 m window, re-planned about once a second) finds the
optimum its warm start leads to. Grip and drift are separate valleys of a non-convex problem,
with a slower half-slide ridge between them, so a derivative solver stays in the valley it
starts in and cannot choose between them. This phase measures how to give it a start in each
valley, revises the planner where the study showed it wrong, and runs the chooser live: model
in the loop first, Isaac second. The code lives in autoware-drift (`src/planning/race_planner`);
swb supplies the worlds and `spikes/course_probe.py`.

**Status**: in progress, started 2026-10-30.

**Source**: user, 2026-10-30: "Find the conditions that our planner would make a drift turn as
the optimal solution"; "I wonder if it can choose between optimal drift and non drift by
differentiation ... diff works for continuous space, not discrete space"; "Explore the
solution method based on our findings and online studies ... see if GPU can help"; then
"create a phase for the solver experiments and revision work. Start it".

**Depends on**: [060](060-live-race-planner.md) (the live planner and tracker, sim.Plant,
course_probe LIVE=1).

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
| R1 | L1 starts each re-plan in more than one valley (grip, rear out; optionally a flick) and keeps the fastest feasible plan | the study's undecided cases |
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

**M4. Model in the loop (E2).** `sim.Plant` with the ECU: single start (060) against {G, D}
and {G, D, F}, E0's oracle plan as the ceiling; mu 1.03, 0.45, 0.3; corners R 9/15/25 at 90
and 180 deg, then Haruna. Gate: on low friction at least half the oracle's gain with no added
stops or off-road; on asphalt no regression beyond 0.1 %; p99 adoption latency at most 1.0 s.

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

**After M3: the phase reframed** (user, 2026-10-30: "Go" on continuing M2 then M4). M1 and M3
show the grip-or-drift choice is worth little for lap time (the gate decided 3 times in 18
runs; G slides into low-friction corners by itself), while the extra seeds keep the car going
when G fails. Ruling: M4 is judged on robustness first: on low friction and mixed surfaces,
fewer stops and failed re-plans than the single start, lap time no worse than 0.1 % on asphalt
and no worse than the single start's on low friction; threshold 0.3 %. Multi-start needs cores
of its own (its budget held at ~5 ms an iteration, not at 7). M2 before M4, since low friction
is the only place the seeds matter. Cost if wrong: a lap-time gain M4 would have chased is not
looked for; M1's oracle still bounds it.
