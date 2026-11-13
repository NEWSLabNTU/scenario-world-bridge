# Phase 074: Cooperation Study, the Cases swb Can Run Now

Measure how autonomous vehicles cooperate, as games whose payoffs swb measures on real stacks,
starting with the cases swb can run today. The method, the theory and the literature are in
[design/survey-multi-vehicle-cooperation.md](../design/survey-multi-vehicle-cooperation.md);
this phase is its first experiment and the urban cases next to it.

**Status**: started 2026-11-13; C1 done; C2 redesigned for mass runs of light policies (C2a arena, C2b swb check). User direction: "Start from those already doable in
swb. We'll study npc things once we reach the scooter and night market cases." So Taiwan's
scooters and the night market (reactive NPC populations) are out of this phase.

## What "doable now" means on this host

The CARLA 0.9.16 binary has Town01-05 and Town10HD. Autoware maps (`data/carla-autoware-bridge/`)
exist for Town01, 02, 03, 05 and 10 only. SSv2 needs the lanelet map for every scenario, so a
town without one is out, whether the cars are light agents or Autoware stacks.

| case | where | ready |
|---|---|---|
| two-car cut-in (the first experiment) | Town05: several lanes each way | yes |
| unprotected left against an oncoming queue | Town05 or Town10 junctions, the light green for both | yes; permissive left on SSv2's signal control to check |
| roundabout entry | Town03's roundabout | yes |
| 4-way stop | needs an unsignalled 4-way junction | to find: lights switched off on a Town05 junction, or an unsignalled one |
| on-ramp, weaving | Town04 or Town06 | no: no Autoware map here (Town06 is not in the binary either); a map is 031's work |
| lane drop | a lane that ends | to find on Town03/05; else as the on-ramp |

## Steps

### C1. The cut-in in a kinematic model (done 2026-11-13)

`scripts/coop/cutin_game.py`: the user's 2 x 2 (A goes or waits, B yields or holds) on a straight
two-lane road, each profile simulated with the cars' limits and reaction time; payoffs from lost
progress, the smallest TTC and time headway, contact and peak braking; pure and mixed equilibria
per context (d0 = A's rear ahead of B's front, dv = A's speed minus B's). `--selftest` checks the
solver against the survey's chicken threshold kappa and the obvious cases.

Result for two Model 3-like cars at 12 m/s, reaction 1 s (`--out <dir>`, figure in its PNG): four
regions, in the order the theory predicts as d0 grows:

- **wait / hold** while A is beside or just ahead of B;
- **chicken**, a band 4-8 m wide: both (go, yield) and (wait, hold) are equilibria, and which one is
  played is a convention or a belief (the survey's kappa);
- **go / yield**: A goes and B's best reply is to open the gap;
- **go / hold**: A is far enough ahead that B need do nothing.

The bands lean with dv: a faster A reaches each region at a smaller d0. With a golf cart as A
(1.5 m/s^2, 25 km/h) the bands stop moving with dv above about +1 m/s, where the cart's speed
limit binds: the car model restricts the outcomes, as the user framed it.

The capabilities are first numbers (`TYPES`): the Model 3's length and limits from
`sim_vehicle_spec`, Autoware's own 1 s rear reaction, an assumed golf cart. C5 replaces them.

### C2. Light policies in mass: the payoff map, filled (next)

User direction 2026-11-13: "We can employ light policies to exploit the cost/payoff map in mass."

A light policy is a rule $\pi(a \mid b, s; \psi)$ in a few lines of Python on ground truth: no
perception, no localization, no planner. That makes a run cost CARLA's physics and nothing else, so
the map can be filled by the thousands, not the tens. Two parts:

**C2a. The arena (mass).** `scripts/coop/arena.py`, a study tool, not part of the swb stack:

- Its own CARLA 0.9.16 on its own port (the stack's CARLA and `swb_carla_world` untouched; the
  arena exits with its server, so no client outlives one), synchronous, fixed `dt` 0.05 s,
  `no_rendering_mode`, no sensors. Town05, the same lanelet geometry SSv2 uses.
- K pairs at once in one world, each on its own straight two-lane stretch of Town05, far enough
  apart not to interact; each pair runs one profile at one context.
- Each car's body and physics from `sim_vehicle_spec` (`carla_physics.yaml`), so the capability
  is the simulator's, not C1's numbers; actuation is CARLA's `VehicleControl` from a small
  longitudinal and lateral controller under the policy, so the limits bind as they do for acb.
- Payoffs from the arena's own per-tick ground truth with C1's formulas; contacts from collision
  sensors. One CSV row per (context, profile, seed), in C1's schema plus run metadata.
- Several arenas in parallel (one CARLA each); throughput to be measured, the target is
  thousands of runs per hour against tens for an Autoware run.

**The policy space is continuous, not 2 x 2.** A's rule: the gap threshold to go, the
acceleration while going, a nudge (lateral probe) and its length; B's: the yield threshold, the
yield deceleration, the headway it holds. Plus the Autoware-like rules, light imitations of
Autoware 1.5's fixed assumptions (the lane change's rear car braking at -1 to -2 m/s^2 after
1-2 s, obstacle cruise's -1 m/s^2), so the arena can ask which rule exploits Autoware's.

What the mass runs give:

1. **The map**: every cell of C1's grid with enough seeds for a confidence interval; C1 against
   the arena, cell by cell, where the model is right and where CARLA's physics moves a boundary.
2. **Equilibria over continuous rules**: the empirical game over sampled $\psi$, a GP surrogate
   per payoff, and double oracle / PSRO rounds, each best response found by search in the arena.
3. **Exploitability**: for any rule, including the Autoware-like one, the best counter-rule and
   what it gains, $\varepsilon(\pi)$; the rules that exploit Autoware's assumptions are the
   candidates C3 checks on the real stack.
4. **Welfare**: $W$, the social optimum, PoA and PoS per cell (C1+).

**C2b. The swb check (sample).** A few cells per region, chosen where the arena's map changes,
rerun through swb itself: `multi_av/town05_cut_in.xosc` with A and B as agent entities, each
driven by the same light policy behind acb (control commands, no Autoware), the trace (024) and
contacts (021) as payoffs. It shows the arena measures what swb measures; any difference names a
gap in the arena (spawn, physics settings, timing) before its numbers are trusted.

### C3. B as Autoware

B a full Autoware stack (the ego, or `just bg-av`), A a light agent. B's yield and hold have to be
found among Autoware's own knobs first (obstacle cruise's following distance, the lane change
module's safety margins; the survey's "Strategy knobs" gap); until then B is Autoware as shipped,
one strategy, and the map shows which of A's choices it rewards. Then A by RTC (go now / wait,
`enable_rtc` on), then both Autoware.

### C4. Fit and explain

SVO angles and potential-game weights fitted to C2 and C3; residuals by region.

### C5. Car models

C1-C3 for pairs from the catalog (Model 3, golf cart, a heavier car when one is specified), the
capabilities from `just vehicle-check` instead of `TYPES`.

### C6. Hidden capability

Each light policy believes the other's class and limits from a prior and the observed motion, not
the truth; the type-confusion matrix and misbelief regret per context.

### U1-U3. The urban cases

In this order, each first in a kinematic model as C1, then with light agents as C2:

1. **U1 unprotected left against an oncoming queue** (Town05/10): A against a chain of 1-4 oncoming
   cars; gap acceptance by queue length. No game model of it was found in the literature.
2. **U2 roundabout entry** (Town03): entry assertiveness against the circulating flow.
3. **U3 4-way stop**, once a junction is found: symmetric arrivals, the wait-for graph, tie-breaks.

On-ramp and lane drop wait for a map (031); scooters and the night market for reactive NPCs.

## Out of scope

Reactive NPC populations; a Town04 Autoware map; Isaac (039's several stacks) until CARLA's maps
are measured.
