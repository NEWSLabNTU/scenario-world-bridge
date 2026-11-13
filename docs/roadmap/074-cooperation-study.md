# Phase 074: Cooperation Study, the Cases swb Can Run Now

Measure how autonomous vehicles cooperate, as games whose payoffs swb measures on real stacks,
starting with the cases swb can run today. The method, the theory and the literature are in
[design/survey-multi-vehicle-cooperation.md](../design/survey-multi-vehicle-cooperation.md);
this phase is its first experiment and the urban cases next to it.

**Status**: started 2026-11-13; C1 done; C2 redesigned for mass runs of light policies (C2a arena, C2b swb check). Design review and the staged plan with exploration and decision steps
(E1-E7, D1-D9) below; next: E1, the arena spike. User direction: "Start from those already doable in
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

`scripts/coop/cutin_game.py`: the two-car cut-in on a straight two-lane road, each strategy pair
simulated with the cars' limits, reaction times and lane-change timing; payoffs from lost progress,
the smallest TTC and time headway, contact and peak braking, weighted by each car's SVO angle;
pure and mixed equilibria per context (d0 = A's rear ahead of B's front, dv = A's speed minus B's);
welfare as a cost, the optimum, the worst and best equilibrium. `--selftest` checks the solver on
textbook chicken (kappa = 10/11) and the obvious cases.

**Strategies are contingent rules** (since the review below): A goes, waits (drops behind), or
goes if it sees B yield within a watch window; B pre-yields (courtesy before any signal) or reacts,
continuing with whichever of yield / hold is its best reply once it has seen A's lane change.
B's safety layer brakes under every strategy.

Results, two Model 3-like cars at 12 m/s, reaction 1 s, as of the review:

- **Selfish cars (SVO 0) have no chicken band.** Once B's reply is sequentially rational, a threat
  to hold that B would not carry out is gone, and the 4-8 m chicken band of the first version
  disappears. What is left, as d0 grows: A drops behind; a band 3-4 m wide with only mixed
  equilibria; go and B yields; go and B need not.
- **Selfish equilibria waste welfare where A is close and faster.** For d0 about 0-12 m and dv > 0
  the equilibrium is "A drops behind", while the optimum is "B pre-yields, A goes when it sees
  it": courtesy pays socially but not individually. The worst equilibrium's loss reaches the
  map's top class (>= 0.5) there. That gap is what a convention or a prosocial rule can win.
- **A prosocial B (SVO 30 or 60 degrees)** turns that region into a negotiated cut-in (A waits
  for B's yield, then goes), with coordination regions (several equilibria) beside it.
- **"After you" deadlock** appears by itself: when B pre-yields and A drops behind, both slow and A
  never gets in (the war of attrition of the survey).
- Golf cart as A (first version): the bands stop moving with dv above about +1 m/s, where its speed
  limit binds.

The capabilities are first numbers (`TYPES`), including an `autoware` type with a 3 s prepare
phase before the lateral move. C5 replaces them with measured ones.

### C2. Light policies in mass: the payoff map, filled (next)

User direction 2026-11-13: "We can employ light policies to exploit the cost/payoff map in mass."

A light policy is a rule $\pi(a \mid b, s; \psi)$ in a few lines of Python on ground truth: no
perception, no localization, no planner. That makes a run cost CARLA's physics and nothing else, so
the map can be filled by the thousands, not the tens. Two parts:

**C2a. The arena (mass).** `scripts/coop/arena.py`, a study tool, not part of the swb stack:

- Its own CARLA 0.9.16 on its own port (the stack's CARLA and `swb_carla_world` untouched; the
  arena exits with its server, so no client outlives one), synchronous, fixed `dt` 0.05 s,
  `no_rendering_mode`.
- **A generated road, not Town05.** A cut-in at 12 m/s needs about 250 m of straight two-lane road
  with no junction; Town05 has few such stretches and every pair would sit on a different one. The
  arena loads one long straight road built for it (`generate_opendrive_world` from an OpenDRIVE
  string: two lanes one way, several km), with K pairs spaced along it 300 m apart, or K parallel
  roads. Same physics, one geometry; C2b checks it against Town05.
- **Warm-up.** CARLA spawns a car standing still; each pair spawns behind its start, is driven up
  to its context speed, and the run starts (t = 0) when both cars hold their speed and the
  context's d0 within a tolerance. The context reached is recorded, not the one asked for.
- Each car's body and physics from `sim_vehicle_spec` (`carla_physics.yaml`), so the capability
  is the simulator's, not C1's numbers; actuation is CARLA's `VehicleControl` from a small
  longitudinal and lateral controller under the policy, so the limits bind as they do for acb.
- Payoffs from the arena's own per-tick ground truth with C1's formulas. Contact from the cars'
  oriented boxes each tick (as C1), collision sensors as a cross-check; every sensor destroyed
  before its car (CLAUDE.md). One CSV row per (context, profile, seed), in C1's schema plus the
  context reached and run metadata.
- **Randomness is explicit.** CARLA in synchronous mode at a fixed step is close to
  deterministic, so repeating a run gives no confidence interval. The seed draws what varies in
  traffic: the start's jitter, each car's reaction time from a distribution, observation noise
  and delay on what each policy sees. A confidence interval is over those distributions, stated
  with them.
- **Light policies are not omniscient.** They see the other car through a model of a stack's
  perception: sampled at 10 Hz, delayed by the perception latency, noisy, with the reaction time
  of the type. An Autoware-like rule also has Autoware's lane-change prepare phase (2-4 s before
  the lateral move). Without these, every map is optimistic against C3.
- **The car is the spec's.** The arena applies `sim_vehicle_spec`'s generated
  `carla_physics.yaml` itself, which is a second implementation of what `swb_carla_world` does;
  before any map is trusted, one step response (full throttle, full brake, a lane change) is
  compared with the same car in swb (`just vehicle-check`'s manoeuvres).
- Several arenas in parallel (one CARLA each); throughput to be measured, the target is
  thousands of runs per hour against tens for an Autoware run.

**The policy space is continuous, not 2 x 2.** A's rule: the gap threshold to go, the
acceleration while going, a nudge (lateral probe) and its length; B's: the yield threshold, the
yield deceleration, the headway it holds. Plus the Autoware-like rules, light imitations of
Autoware 1.5's fixed assumptions (the lane change's rear car braking at -1 to -2 m/s^2 after
1-2 s, obstacle cruise's -1 m/s^2), so the arena can ask which rule exploits Autoware's.

What the mass runs give:

1. **The map**: every cell of C1's grid with enough seeds for a confidence interval. C1 and the
   arena are compared on payoffs with their intervals, not on class letters: a class flips on a
   0.001 payoff difference near a boundary.
2. **Equilibria over continuous rules**: the empirical game over sampled $\psi$ and double oracle
   / PSRO rounds, each best response found by search in the arena. Payoffs jump at contact and at
   a lane change that does or does not happen, so a single smooth surrogate (a GP) is wrong
   there: the surrogate is split, a classifier for the discrete outcome (contact, A got in) and a
   regression per outcome class.
3. **Exploitability**: for any rule, including the Autoware-like one, the best counter-rule found
   and what it gains. Search finds a lower bound on $\varepsilon(\pi)$, never the value: an
   equilibrium is reported as "no deviation gaining more than x found by a search of budget n".
   The rules that exploit Autoware's assumptions are the candidates C3 checks on the real stack.
4. **Welfare**: $W$, the social optimum, PoA and PoS per cell (C1+).

**C2b. The swb check (sample).** A few cells per region, chosen where the arena's map changes,
rerun through swb itself: `multi_av/town05_cut_in.xosc` with A and B as agent entities, each
driven by the same light policy behind acb, the trace (024) and contacts (021) as payoffs. The
policy is a small ROS node per car: it reads its own odometry and the other car from acb's ground
truth (`publish_ground_truth_objects:=true`, `/perception/object_recognition/objects`, Autoware's
perception off) and publishes Autoware's control command to acb. No Autoware is needed. It shows the arena measures what swb measures; any difference names a
gap in the arena (spawn, physics settings, timing) before its numbers are trusted.

### C3. B as Autoware

First with perception off and acb's ground-truth objects (planning and control are Autoware's,
the objects exact): this isolates the planner's strategy from perception. Then the full stack.
B a full Autoware stack (the ego, or `just bg-av`), A a light agent. B's yield and hold have to be
found among Autoware's own knobs first (obstacle cruise's following distance, the lane change
module's safety margins; the survey's "Strategy knobs" gap); until then B is Autoware as shipped,
one strategy, and the map shows which of A's choices it rewards. Then A by RTC (go now / wait,
`enable_rtc` on), then both Autoware.

### C4. Fit and explain

Note on identifiability: from 2-3 discrete choices per run, an SVO angle and a capability are
confounded (a gentle brake is "cannot" or "will not"), and a few hundred L3 runs do not separate
them. C4 fits on C2's continuous rules first, where the arena controls both, and reports which
parameters the data identify.

SVO angles and potential-game weights fitted to C2 and C3; residuals by region.

### C5. Car models

C1-C3 for pairs from the catalog (Model 3, golf cart, a heavier car when one is specified), the
capabilities from `just vehicle-check` instead of `TYPES`.

### C6. Hidden capability

Each light policy believes the other's class and limits from a prior and the observed motion, not
the truth; the type-confusion matrix and misbelief regret per context.

### C7. Conventions

A convention picks one equilibrium for every car: a priority rule, a courtesy band in which B
pre-yields, a V2X negotiation. Each is played in the arena as a correlation device over C2's
rules, and measured by the part of the price of stability it recovers. This says whether
cooperation needs a convention or a new layer on the car, and feeds D6.

### U1-U3. The urban cases

In this order, each first in a kinematic model as C1, then with light agents as C2:

1. **U1 unprotected left against an oncoming queue** (Town05/10): A against a chain of 1-4 oncoming
   cars; gap acceptance by queue length. No game model of it was found in the literature.
2. **U2 roundabout entry** (Town03): entry assertiveness against the circulating flow.
3. **U3 4-way stop**, once a junction is found: symmetric arrivals, the wait-for graph, tie-breaks.

On-ramp and lane drop wait for a map (031); scooters and the night market for reactive NPCs.

## Plan: exploration and decision steps

User direction 2026-11-13: "Show me the road map. You can have design exploration and decision
steps." Each stage builds (B), explores to learn what the build needs (E), and ends in a decision
(D) with its criterion and options, so a stage that does not pay off changes the plan instead of
being carried.

| stage | step | kind | what | out / criterion |
|---|---|---|---|---|
| 0 theory | C1, review | B | kinematic cut-in, sequential; design review | done (a8505ad) |
| 1 mass | E1 arena spike | E | own CARLA, generated road, one pair then K = 4, 16; runs per hour, tick time, CPU and GPU; step responses against the same car in swb | throughput and physics parity numbers |
| | D1 arena or calibrated model | D | does the CARLA arena pay for itself? | arena if >= 1000 runs/h per host at K >= 4 and parity holds (speed RMSE <= 0.2 m/s, stopping distance within 5 %, lane change within 0.2 m); low throughput: a kinematic model calibrated on CARLA's step responses, the arena for spot checks; parity fails: fix the physics or drive the arena through `swb_carla_world` |
| | E2 stack timing | E | Autoware ego on Town05: perception latency, tracked velocity and acceleration noise, lane-change prepare and total time, cruise reaction | the light policies' perception model; the `autoware` type's numbers |
| | C2a arena | B | noise sources, warm-up, maps with intervals, double oracle, exploitability of the Autoware-like rule | maps, equilibria, exploits |
| | C2b swb check | B | Town05 scenario, light policies on acb's ground truth | sampled cells |
| | D2 trust the arena | D | do arena and swb agree? | >= 3 cells per region, >= 90 % of swb payoffs inside the arena's 95 % intervals, boundaries within one grid step; else locate and fix (spawn, timing, physics) and repeat; persistent: mass runs through swb itself on parallel pipelines (026), at lower volume |
| 2 real stack | E3 Autoware knobs | E | which Autoware 1.5 parameters move B between yield and hold (obstacle cruise's idling time and RSS accelerations, lane change's rear-car assumptions, RTC), and whether they change at run time | knob-to-behaviour table |
| | D3 B's strategies on Autoware | D | what is a strategy of a real stack? | stock Autoware as one fixed strategy first (A's best reply to it); knob settings as strategies where >= 2 settings move outcomes measurably; an overlay switch (042) only if neither works |
| | C3a, C3b | B | B Autoware with ground-truth objects, then with perception | real-stack maps |
| | D4 where the stack leaves the model | D | residual map against the Autoware-like rule | refit the imitation and rerun C2a; or perception-caused (C3a against C3b): update the perception model; or record as a finding |
| | C3c | B | A by RTC, then both Autoware | cross-play of two configurations |
| 3 explain | C4 fit | B | kappa rules, SVO, potential, level-k; identifiability | fitted models, residuals |
| | D5 which explanation | D | which model the strategy layer carries | held-out payoff error and readability; default the threshold (kappa) rules |
| | E4 heavier car | E | is a truck or bus spec available or buildable in `sim_vehicle_spec` | spec or none |
| | C5 car models | B | pairs from the catalog, measured capabilities | maps per pair; the commitment hypothesis |
| | C6 hidden capability | B | beliefs, type confusion, misbelief regret, value of information | regret and VoI maps |
| | C7 conventions | B | a priority rule, a courtesy band for B, a V2X negotiation; each as a correlation device | the part of PoS each recovers |
| | D6 build the strategy layer? | D | is acting on beliefs worth a node on the car? | build if regret or VoI reaches >= 10 % of stock's welfare loss in a non-trivial share of contexts; else stop at the findings, or recommend a capability broadcast only |
| 4 vehicle | P1 strategy layer | B | `interaction_belief`, `interaction_strategy` on Autoware; ablation stock / belief / belief + V2X on the same maps | ablation maps |
| | D7 deployable? | D | | never worse than stock on contact and the TTC distribution, a significant welfare gain, within the message-rate budget: propose it; else keep it as research |
| 5 cases | E5 left-turn signals | E | can SSv2 give the left turn and the oncoming queue green together on Town05/10 | yes or a workaround |
| | U1, U2 | B | left turn against a queue; roundabout entry (Town03); each kinematic, arena, swb | maps |
| | E6 4-way stop site | E | lights switched off on a Town05 junction, or an unsignalled junction | site |
| | D8 4-way stop | D | | site found: U3; else skip |
| | E7 merge site | E | a lane drop on Town03/05 | site or none |
| | D9 N-car merge | D | where to study N = 3-4 | the arena on a generated on-ramp or lane drop (no Autoware map needed) for the game; the swb check on a found site, or a Town04 map (031) only if the arena's results call for one |

Order: stage 1 first. Stage 5 starts after D2 (it needs a trusted arena) and runs beside stages
2-4. Scooters and the night market are a later phase (reactive NPCs).

## Design review, 2026-11-13

User direction: "Review the design. Explore design flaws in theory, impractical assumption and
workflow. Fix them if there is any." Found and fixed:

| # | flaw | fix |
|---|---|---|
| 1 | Theory: the cut-in was solved as a one-shot simultaneous 2 x 2, but B sees A's lane change and replies; B's "hold" against a car that went was a threat B would not carry out, and the chicken band came from it | contingent strategies, B's reply sequentially rational (C1 rewritten); chicken survives only where something forces simultaneity: B's courtesy before any signal, latency, or prosocial preferences |
| 2 | Theory: PoA and PoS as ratios of welfare, but every utility is <= 0 | welfare as a cost C = -(U_A + U_B) >= 0; ratios when C_opt is not near 0, and the loss C_eq - C_opt always |
| 3 | Theory: summing utilities normalised per car ("fraction of free flow") compares a golf cart's 10 % with a car's 10 % | the global target in common units (vehicle-seconds of delay, safety as a constraint) for welfare; per-car normalisation stays inside each car's own payoff (survey) |
| 4 | Theory: safety stated as a constraint, but kappa and C1 trade contact for progress | A may go only if its belief puts contact under delta: go iff $p > \max(\kappa, 1-\delta)$ where going against a hold is contact (survey); C1 keeps the penalty and flags contact equilibria |
| 5 | Theory: capability evidence as hard set pruning, from accelerations twice differentiated out of 10 Hz tracks, and cars seldom show their limits | soft bounds with a noise model; the class prior carries most of the capability belief (survey) |
| 6 | Theory: exploitability "measured" by search | it is a lower bound; equilibria reported with the search's budget (C2a) |
| 7 | Method: one smooth GP surrogate over payoffs that jump at contact and at a lane change | split surrogate: a classifier for the discrete outcome and a regression per class (C2a) |
| 8 | Practice: "seeds" on a near-deterministic synchronous CARLA give no interval | the noise is explicit: start jitter, reaction time, observation noise and delay (C2a) |
| 9 | Practice: light policies with perfect, instant perception, and a lane change with no prepare phase | perception model per policy (10 Hz, latency, noise); Autoware-like rules with the 2-4 s prepare phase; C1's `autoware` type (C1, C2a) |
| 10 | Practice: K pairs on Town05's few long straight stretches | a generated straight road for the arena; Town05 kept for the swb check (C2a) |
| 11 | Practice: CARLA spawns cars standing still; a context needs them at speed | warm-up to the context, the context reached recorded (C2a) |
| 12 | Practice: the arena applies the spec's physics a second time | a step-response comparison with the same car in swb first (C2a) |
| 13 | Workflow: the swb check's light policy had no way to see the other car | acb's ground-truth objects, perception off (C2b) |
| 14 | Workflow: from light agents straight to the full stack | Autoware with ground-truth objects first, then with perception (C3) |
| 15 | Workflow: maps compared by class letters | compared on payoffs with intervals (C2a) |
| 16 | Workflow: SVO and capability fitted from a few hundred discrete choices | fit on the arena's continuous rules first; report what is identifiable (C4) |

## Out of scope

Reactive NPC populations; a Town04 Autoware map; Isaac (039's several stacks) until CARLA's maps
are measured.
