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

(none yet)
