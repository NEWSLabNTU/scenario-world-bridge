# Cooperative driving: theory and learning, measured

How a car decides when to go, yield or wait among other road users, in a way that can be
explained and deployed, and how swb measures whether it works. This folds roadmap 074's findings
(scoring, the two-car games, the N-car formulation, the strategy layer on Autoware) into one
design. The literature and the long arguments are in `survey-multi-vehicle-cooperation.md`;
the steps and results in `docs/roadmap/074-cooperation-study.md`.

## The design in one picture

```
                OFFLINE: build and check the model                          ON THE CAR: use it
 ┌──────────────────────────────────────────────────────────────┐   ┌──────────────────────────────┐
 │ contexts (gap, speeds, car types)                            │   │ perception 10 Hz             │
 │      │                                                       │   │      │                       │
 │      v                                                       │   │      v                       │
 │ runs: kinematic ─> arena (CARLA physics) ─> swb (Autoware)   │   │ interaction_belief 10 Hz     │
 │      │            trusted by D2           ground truth       │   │  capability + intent / object│
 │      v                                                       │   │      │                       │
 │ ① SCORING  raw measures ─> utility U ─> objective J, welfare │   │      v                       │
 │      │                                                       │   │ interaction_strategy 2-5 Hz  │
 │      v                                                       │   │  rule ψ: go iff P(yield) > κ │
 │ ② TWO-CAR GAMES  equilibrium map E(context, types)           │──>│  κ from the payoff table     │
 │      │           double oracle, exploitability check         │   │      │  + decision record    │
 │      v                                                       │   │      v                       │
 │ ③ N CARS  pairs composed per measure + measured residual     │   │ Autoware: RTC go/wait,       │
 │           + mean field beyond the conflict set               │   │  speed limit, safety bounds  │
 │      │                                                       │   │      │                       │
 │      v                                                       │   │      v                       │
 │ ④ LEARNING  fills residuals, proposes rules — never decides  │   │ SHIELD: AEB, obstacle stop,  │
 └──────────────────────────────────────────────────────────────┘   │  RSS checks (never bypassed) │
                                                                    └──────────────────────────────┘
```

## Principles

1. **Theory gives the structure, learning fills what theory leaves out.** A kinematic game model
   explains *why* (who yields, at what threshold); a learned term corrects *how much*, and is
   reported separately so the explanation stays honest.
2. **Learning proposes, measurement decides.** A learned proposer adds candidate rules; the game
   over measured payoffs decides what is played; an independent search for profitable deviations
   checks it. A bad proposer costs time, never correctness.
3. **Safety is a constraint, not a price.** No efficiency gain buys contact risk. The shield (AEB,
   obstacle stop, RSS) stays underneath everything and is never bypassed.
4. **Every number is measured on a real stack, or on a model checked against one.** swb runs full
   Autoware stacks; the arena is fast and trusted only where it agrees with swb (D2).
5. **A decision is explained by its inputs:** the belief, the candidates' payoffs, the threshold.

## ① Scoring: from a run to a payoff

Four layers, each one reported:

| layer | what | where |
|---|---|---|
| raw measures | physical quantities from truth, per car per run | `scripts/coop/measures.py` |
| utility $U_i$ | a car's own cost of the outcome, normalized | `cutin_game.utility` |
| objective $J_i$ | what the car optimizes: $J_i = \cos\varphi\,U_i + \sin\varphi\,U_{-i}$ (social value orientation) | the game |
| welfare $C$ | $C = -\sum_i U_i$ over everyone (own utilities, never $J$, which would count others twice) | price of anarchy, of stability |

**Raw measures**

| kind | measures |
|---|---|
| safety | contact, closing speed at impact; minimum time-to-collision (TTC) and time headway; the deceleration needed to avoid a crash (DRAC) and the **capability margin** $1 - \text{DRAC}/b_{max}$; time below 3 s TTC and its integral (TET, TIT); post-encroachment time for crossings |
| efficiency | lost progress against free flow over a fixed horizon; stops and seconds stopped; manoeuvre time; deadlock |
| comfort | peak braking and peak jerk over 0.5 s windows |
| latency | event times (A's lateral move, each car's first braking, A's decision) and the reaction times between them |

**Utility** (today's weights): $U_i = -\text{lost}_i - \text{risk}_i - 0.3\,(\text{peak}_i/b_{max})^2 - 10\cdot\text{contact}$,
with $\text{risk} = \max$ of the TTC and headway shortfalls against 3 s and 1 s.

**Rules that make scores comparable** (learned the hard way, D2: 71 % → 92 %):
- defined over *time*, never over samples (0.05 s arena ticks and 0.1 s swb logs must agree);
- never scaled by the run's length (a fixed horizon; a longer run adds loss, it does not dilute it);
- a manoeuvre's end by its plan, not by a threshold a curved road may never reach.

**Statistics**: a mean and a bootstrap interval per profile, context and type pair; an
equilibrium is reported as an ε-equilibrium with its confidence. Contact is rare, so continuous
surrogates (TTC, DRAC, margin, TIT) rank risk, and contact rates are estimated with importance
sampling.

**Latency is a type, not a payoff.** Perception delay (0.19 s measured), decision delay (1.25 s
from a lead's braking to Autoware's command), Autoware's 3 s lane-change prepare phase, actuator
lag: they enter the game as the car's type, and their cost appears in the outcome.

## ② The two-car game: the unit everything is built from

- **Type** $t = (\theta, \varphi)$: capability $\theta$ (braking, acceleration, lag, size, reaction,
  prepare time) and preference $\varphi$. Own type known; the other's private.
- **Strategy** = a short **reactive rule** with a few parameters, not a trajectory: A's gap check
  $\text{gap} \ge \text{buffer} + v_B\tau + \max(0, v_B^2 - v_A^2)/2a_{rear}$, patience, whether to
  use a yield, prepare time; B's courtesy band, reaction gap, yield deceleration. A car model
  restricts which rules it can execute: $\Psi(\theta) \subseteq \Psi$.
- **Belief** over the other's type from what it does: capability evidence (a 6 m/s² stop proves
  braking ≥ 6, softly, as tracks are noisy) and intent evidence (a deceleration that starts when A
  signals is a yield).
- **Decision**: A goes iff $P(\text{B yields}) > \max(\kappa, 1-\delta)$, with
  $\kappa = (U_{wait} - U_{go,hold})/(U_{go,yield} - U_{go,hold})$ from the payoff table and
  $\delta$ the contact-risk bound. One number, one comparison: explainable.

What it showed (arena, thousands of runs per hour, trusted by D2):
- the cut-in is sequential, so the "chicken" band of a simultaneous model mostly disappears; it
  returns where latency or a prepare phase forces a commitment before the other's move is seen;
- contacts come from perception delay, not from bad rules;
- **deadlock is an equilibrium** ("after you", 910 runs): its cure is a tie-break, not a better car;
- the double oracle's equilibrium (A: τ 0.68 s, a_rear 2.1, buffer 4.2 m, no prepare; B reacts)
  has a lower welfare than the Autoware-like pair (−0.167 vs −0.124): selfish rules cost; A with
  Autoware's conservative setting never cuts in;
- left turn against a queue: short queues are a coordination game, long sparse ones leave only the
  yield equilibrium; a courtesy costs every follower (chain externality).

## ③ N cars: pairs, composition, residual, mean field

A car interacts with few others: interaction events in naturalistic data involve 3.2 vehicles on
average, 7 at most. So:

1. **Conflict set and slots.** Each car's payoff depends on the cars whose reachable sets meet its
   own (lead, rear, four adjacent-lane neighbours, crossing, oncoming): a graphical game.
2. **Pair terms** are the measured two-car games, per slot, context and types.
3. **Composition, per measure**:

   | measure | rule | reason |
   |---|---|---|
   | risk | worst pair ($\max_j$); sum as an upper bound | one car hits you at a time |
   | delay | max-plus down the queue: $d_j = \max(d_j^{own}, d_{lead} - \text{slack}_j)$ | a yield propagates to followers |
   | braking | worst pair | a peak, not a total |

4. **Residual.** Run the N-car scene; $r = J - \hat J^{composed}$ with its interval. Within noise:
   pairs suffice. Systematic: learn it (④). Large and irregular: that scene is its own game.
5. **Grow N by error**: three cars only where pairs fail the tolerance, four only where three do.
6. **Equilibria**: symmetric pairs make a potential game (an equilibrium exists, best response
   converges); asymmetric ones (right of way, capability) a near-potential game whose fit error
   bounds the distance to equilibrium.
7. **Mean field** beyond the conflict set: $J_i = J^{near}_i(\text{conflict set}) + J^{far}(\rho, \bar v, \text{rule mix})$,
   near field exact, far field as density, mean speed and the population's rule mix. Valid at
   large N (the only measured threshold is ~30 cars, for routing); swb measures the split's error
   by running the same scene all-explicit and as a density.

## ④ Theory and learning: who does what

| component | theory gives | learning adds | checked by |
|---|---|---|---|
| belief | Bayes over types, class prior, capability bounds | the likelihood of observations under real perception noise | calibration: predicted vs observed yield rates |
| payoff | kinematic game, pair composition | residual $g_\theta$ over the conflict set (permutation-invariant per slot) | held-out swb runs; the residual's share is reported |
| rules | threshold form, double oracle | a proposer of candidate rules (PSRO-style) | exploitability: an untrained search for profitable deviations |
| size | slots bound the local structure | a set model trained on 2-3 cars | N+1 residual; exact games at small N |
| far field | mean-field game | the far field's response to density | explicit runs at increasing N |

**Explainability.** Every decision writes a record: the belief per object, the candidates'
payoffs split into the theory part and the learned residual, the threshold, the choice. Example:
*"A went: P(B yields) = 0.72 > κ = 0.55; gap 10.2 m, B a car (braking ≥ 6.1 m/s² seen); margin
0.6; residual −0.01 of a 0.17 margin."* When the learned part carries a decision (its share of the
margin above a bound), the record says so: that decision is learned, not explained.

## ⑤ On the car: two nodes beside Autoware

```
perception 10 Hz ─> prediction ─┬──────────────────────────────> planning 10 Hz ─> control 33 Hz
                (V2X, untrusted)┤                                   ^   ^
                                v                                   │   │
                    interaction_belief 10 Hz                        │   │
                    per object in the conflict set                  │   │
                                v                                   │   │
                    interaction_strategy 2-5 Hz, dwell ≥ 1 s ───────┘   │
                    table lookup (μs) or a small game solve (< 50 ms)   │
                    out: go/wait (RTC), speed limit, per-object bounds ─┘
```

- **Into Autoware**, least invasive first: go/wait through RTC or the external lane-change request,
  yield/hold through the API's velocity limit; then belief-shaped predictions; then per-object
  safety bounds replacing Autoware's fixed constants (rear car brakes at 1-2 m/s², reacts in
  1-2 s, for a truck and a golf cart alike), carried in the Autoware overlay.
- **Rates**: telling a yield from a hold takes 0.5-1.5 s of filtered acceleration, the lane change
  decides within 2-4 s, so the belief has one to three updates of evidence; a nudge (probe) makes
  the other car show itself sooner.
- **Deployment rules**: the shield stays; with the nodes dead or unsure, stock Autoware runs; V2X
  is a prior checked against motion; own capability from the vehicle spec; fits an ECU.

## ⑥ Validation: a ladder of gates

| gate | question | bar | status |
|---|---|---|---|
| E1/D1 | is a fast arena faithful to the car? | vehicle check within 0.01 m/s² | met: 10-35 k runs/h |
| D2 | does the arena agree with swb on Autoware's world? | ≥ 90 % of payoffs inside the arena's intervals | met: 92 %, outcomes 24/24 |
| D9 | do pairs predict three cars? | residual within tolerance in ≥ 90 % of contexts | next (N1-N2) |
| D6 | is acting on beliefs worth a node? | regret or value of information ≥ 10 % of stock's welfare loss | after C3 |
| D7 | deployable? | never worse than stock on contact and TTC, a significant welfare gain, within the rate budget | after P1 |

Ablation on the same scenarios: stock constants, belief, belief + V2X. The differences are the
value of information, measured on deployment code at deployment rates.

## Status and open problems (2026-11-13)

- Done: scoring and raw measures; two-car games (C1, C2a, double oracle, U1); arena trusted (D2);
  Autoware's timing (E2).
- Autoware's lane change measured (E2): about 4 s before the lateral move from a start, 4.8 s
  across. An earlier stall was AEB stopping a change into Town03's tunnel divider, which the
  lanelet map marks as a driving lane (a map-tool issue).
- Next: C3 (Autoware as a player, on the west straight), N1-N2 (conflict-set sizes, three-car
  composition), E3 (Autoware's knobs as rules).
