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

## ⓪ The scenario space

Before the formulas: which encounters must the design cover? The standard pre-crash typologies
list them: NHTSA's 37 pre-crash scenarios (Najm et al. 2007, DOT HS 810 767) [M], Euro NCAP's
car-to-car AEB and lane-support tests (car-to-car rear stationary, moving, braking; turn across
path; crossing straight; head-on; cut-in, cut-out) [M], ISO 34502:2022's scenario-based safety
evaluation [M]. Read for the game, they reduce to **how the two cars' paths relate**. Each car
has a route, a path with arc length $s$; a pair's **relation** fixes where they can meet (the
conflict zone $Z$), what the outcome is (who passes $Z$ first), and which risk measure applies.

| relation | paths | conflict zone $Z$ | outcome | scenarios |
|---|---|---|---|---|
| **follow** | the same path, one behind the other | the whole path | the gap kept | car following ("chasing"), lead brakes (stationary, moving, braking lead), stop-and-go |
| **merge** | two paths join at a point | the joined path from the merge point on | who is ahead after it | cut-in, on-ramp, lane drop and zipper, merging out of a T-junction, entering a roundabout |
| **cross** | two paths intersect | the crossing box | who passes the box first | straight crossing paths, left turn across path (opposite or lateral direction), right turn into path, 4-way stop, a pedestrian or scooter crossing |
| **oppose** | the same stretch, opposite directions | the shared stretch | who enters it first (the other waits) | a narrow road or a single-lane passage, overtaking through the oncoming lane, an obstacle in one lane |
| **diverge** | two paths split | none between the pair | none; it reveals a third car | cut-out (the lead leaves its lane and reveals a stopped car), exits |

Some encounters are sequences of these: the left turn is a cross (with the oncoming car) followed
by a merge (into the target road); overtaking is a merge out, an oppose, then a merge back; a
cut-in is a merge that becomes a follow. Head-on drift and running a red light are faults, not
games: they belong to the safety layer, and the design treats them only as type uncertainty
(a car that may not stop).

**What each scenario needs, and what swb has:**

| scenario | relation | game when rules are symmetric | arena | swb site | status |
|---|---|---|---|---|---|
| following, lead brakes | follow | none: one-sided reaction; a chain externality down a queue | parallel road (exists) | Town03 east straight (E2) | timing measured (E2) |
| cut-in | merge | sequential leader-follower; chicken only under a commitment | parallel road (exists) | Town03 west straight | C1-C3 done |
| cut-out | diverge + follow | one-sided reaction to a revealed obstacle | parallel road + a stopped car | west straight | open |
| on-ramp, lane drop, zipper | merge with a deadline | war of attrition: the lane's end forces a decision | a lane that ends (to add) | none on Autoware's maps yet (E7) | open (D9) |
| T-junction exit, roundabout entry | merge into priority traffic | gap acceptance against a stream; chain | a junction, a ring (to add) | Town01/02 T-junctions; Town03 roundabout (U2) | open |
| straight crossing, unsignalled | cross | chicken without a priority rule; a priority rule selects one equilibrium | a crossing (to add) | Town01/02 junctions with the lights off (E6) | open (U3) |
| left turn across oncoming | cross, then merge | gap acceptance against a queue; coordination for short queues | a junction (to add) | Town05/10 (E5) | kinematic done (U1) |
| narrow road, single-lane passage | oppose | war of attrition; a convention (who is nearer the passing place) selects | a two-way road with a narrow stretch (to add) | none yet | open |
| overtaking via the oncoming lane | merge, oppose, merge | a timing game against the oncoming gap | a two-way road (to add) | Town01/02 two-lane roads | open |
| pedestrian or scooter crossing | cross with a vulnerable road user | courtesy and attrition; capability types differ most | a crossing with a walker | night market, scooters: later phase | open |

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

**Utility**, the same function for every scenario:

$$U_i = -\ell_i - \rho_i - c_i - 10\cdot\mathbf 1[\text{contact}]$$

- **Progress** along the car's own route, so a turn or a ring counts as a straight does:
  $\ell_i = \dfrac{v_{i,0}\,T - \Delta s_i}{v_{i,0}\,H}$, $\Delta s_i$ the arc length covered, $H = 20$ s.
- **Risk**, the worst shortfall over the cars in its conflict set and over the measures that fit
  their relation:
  $$\rho_i = \max_{j \in K_i}\max\left(\frac{(\tau^* - \min_t \mathrm{TTC}^{2D}_{ij})^+}{\tau^*},\ \frac{(h^* - \min_t h_{ij})^+}{h^*},\ \frac{(\pi^* - \mathrm{PET}_{ij})^+}{\pi^*},\ \frac{(d^* - \min_t d_{ij})^+}{d^*}\right)$$
  with $\mathrm{TTC}^{2D}_{ij}$ the first time the two bodies would touch at constant velocity
  (it is $g/\Delta v$ for a follow, uses the closing speed $v_i + v_j$ for an oppose, and turns
  positive during a merge before the lanes are shared), $h_{ij}$ the time headway while one follows
  the other, $\mathrm{PET}_{ij}$ the post-encroachment time at a cross's box, and $d_{ij}$ the body
  clearance (the smooth term near contact, which C3 showed is needed). References $\tau^* = 3$ s,
  $h^* = 1$ s, $\pi^* = 1.5$ s, $d^* = 1$ m.
- **Comfort** from both directions:
  $c_i = 0.3\left(\dfrac{a^{\text{lon,peak}}_i}{b^{\max}_i}\right)^2 + 0.3\left(\dfrac{a^{\text{lat,peak}}_i}{a^{\text{lat,max}}_i}\right)^2$.
- **Contact** as before. Today's code implements the follow case of $\rho$ (TTC and headway in a
  shared lane) and longitudinal comfort; the rest is the revision (step S0 below).

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

**One game for every relation: the passage game.** A pair meets at a conflict zone $Z$; the
outcome is the order of passage; the context is where each car stands relative to $Z$:

$$c = \bigl(\text{relation},\ \Delta t = t^Z_A - t^Z_B,\ v_A,\ v_B,\ \text{right of way } r,\ \text{geometry}\bigr),\qquad t^Z_i = \frac{s^Z_i - s_i}{v_i}$$

The car that must claim $Z$ (the one cutting in, turning, entering, overtaking) plays a **gap
acceptance** rule that generalizes the cut-in's:

$$\text{go} \iff t^{Z,\text{in}}_B - t^{Z,\text{out}}_A \;\ge\; \tau_{\text{acc}} + \frac{(v_B^2 - v_A^2)^+}{2\,a_{\text{rear}}\,v_B}\quad(\text{merge}),\qquad
t^{Z,\text{in}}_B - t^{Z,\text{out}}_A \;\ge\; \tau_{\text{acc}}\quad(\text{cross, oppose})$$

with a patience, an option to go on a seen yield, and a prepare time; the car that holds $Z$ plays
a **yield** rule: yield within a band before any move is seen, or on the claimant's onset within a
reaction gap, at a chosen deceleration (C3 measured Autoware's: onset + 1.0 s). The relation
fixes the game's shape: a merge is sequential (the holder sees the move), a cross or an oppose is
close to simultaneous when $|\Delta t|$ is small and no right of way applies (chicken, or a war of
attrition when both can wait), and a right-of-way rule or a convention selects one equilibrium.
The cut-in below is the merge instance; the left turn (U1) the cross instance.

**The cut-in, in this form:**


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
- C3 done: Autoware as B on Town03's west straight, with ground truth (C3a) and CenterPoint (C3b).
  Four to five of six cells agree with the arena once its Autoware imitation reacts to the lane
  change's onset (1.0 s, perception adds 0.1-0.45 s); a slow cut-in 14 m ahead is a near miss
  (0.2-0.6 m clearance) that the arena grazes. Bugs found and fixed on the way: the ego's pedal maps
  under the spec's physics, acb's ground-truth headings, the agent at junctions.
- Next: the scenario plan below; N1-N2 (conflict-set sizes, three-car composition) and E3
  (Autoware's knobs as rules) beside it.

**The scenario plan** (roadmap 074, S0-S6), one relation at a time, each through the same ladder:
kinematic game, arena (with the geometry it needs), swb with a light agent, swb with Autoware.

| step | what | needs |
|---|---|---|
| S0 | the general utility: route progress, $\mathrm{TTC}^{2D}$, PET, clearance, lateral comfort, in `measures.py` and the utility; re-score C2b and C3 with it (the cut-in must keep its classes, and the near-miss cell becomes smooth) | offline, on recorded poses |
| S1 | cut-out: the lead leaves its lane and reveals a stopped car | the parallel road plus a parked car; Autoware's reaction (E2's numbers) |
| S2 | cross: straight crossing at an unsignalled junction, with and without a priority rule | an arena crossing; Town01/02 junctions with the lights off (E6) |
| S3 | merge with a deadline: a lane drop or an on-ramp (war of attrition) | an arena lane end; a site on Autoware's maps (E7) |
| S4 | oppose: a narrow passage, who waits | an arena two-way road with a narrow stretch |
| S5 | the left turn in the arena and swb (U1's kinematic results first) | an arena junction; Town05/10 (E5) |
| S6 | overtaking via the oncoming lane: merge, oppose, merge | a two-way arena road; Town01/02 |

Each step reports the equilibrium map over $(\Delta t, v_A, v_B, r)$, the welfare loss against the
optimum, and where Autoware departs from its imitation.
