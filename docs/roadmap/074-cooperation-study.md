# Phase 074: Cooperation Study, the Cases swb Can Run Now

Measure how autonomous vehicles cooperate, as games whose payoffs swb measures on real stacks,
starting with the cases swb can run today. The method, the theory and the literature are in
[design/survey-multi-vehicle-cooperation.md](../design/survey-multi-vehicle-cooperation.md);
this phase is its first experiment and the urban cases next to it.

**Status**: started 2026-11-13; C1 done; C2 redesigned for mass runs of light policies (C2a arena, C2b swb check). Design review and the staged plan with exploration and decision steps
(E1-E7, D1-D9) below. E1 done and D1 decided (build the arena); E2 measured (lane change open); C2a in progress. User direction: "Start from those already doable in
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

### E1. The arena spike (done 2026-11-13) and D1

`scripts/coop/arena.py` on a CARLA 0.9.16 of its own (port 2600, Epic, `no_rendering_mode`;
logs and runs in `/mnt/p5plus/aeon/coop/e1/`), `scripts/coop/arena_game.py` for the game.

- **The world**: K parallel straight roads from `generate_opendrive_world`, two lanes one way, 900 m,
  60 m apart; lane centres asked of the map. Loading it takes about 1 s.
- **The car**: `sim_vehicle_spec`'s `carla_physics.yaml` applied with its own Python module
  (`carla_physics.apply`, read back with `differences`); swb_carla_world's timing (0.05 s ticks,
  16 substeps); acb's control path ported: the pedal maps inverted as `command_for` does, the
  pedal lags of `pedal_lag.rs`, the steer command of `steer_command_for`.
- **Parity** (`--parity`): vehicle-check's 12 steps (`autoware_sim_check.dynamics`, its own plan
  and judge) all pass. Acceleration and brake steps are within 0.01 m/s^2 of the request (the
  limit is 0.35); steering is within 0.002 rad and the yaw rate within 6 % of the bicycle model
  (the limit is 15 %). Measured capability: 0-12 m/s in 2.55 s; full braking from 12 m/s stops
  in 9.72 m, 7.4 m/s^2 mean with the lags and 9.6 m/s^2 peak (C1 assumed 3 and 7;
  `carla_model3` in `TYPES` now). This is parity with vehicle-check's acceptance on the same
  physics and maps; a side-by-side with swb's own run of the car is still open (the stack's
  CARLA is owned by the running stack).
- **Throughput** (288 runs = 8 contexts x 9 profiles x 4 seeds, 15 s warm-up + 12 s run):

  | K pairs per CARLA | runs per hour | faster than real time | tick, p50 / p95 |
  |---|---|---|---|
  | 1 | 10 000 | 75x | 0.54 / 0.94 ms |
  | 4 | 18 400 | 34x | 1.16 / 1.82 ms |
  | 16 | 19 800 | 9x | 4.5 / 6.0 ms |
  | 2 CARLAs x 4, in parallel | 35 400 | | 1.2 ms each |

  CARLA's own physics is the cost (about 0.14 ms per car per tick at 16 substeps), not the
  Python client (one snapshot and one batch of controls per tick changed nothing). Beyond K = 4
  a CARLA does not get faster; more CARLAs do. One arena holds 1.35 cores (CARLA) + 0.9 (the
  client), 2.7 GB RAM and 3.9 GB VRAM: by memory, about 6 fit beside today's load on 133
  (estimate). With a 20 s horizon, K = 16 gives 13 700 runs per hour.
- **Context accuracy**: the warm-up hits d0 within 0.22 m and dv within 0.08 m/s; B settles at
  11.94 m/s for 12 (a P controller's offset), recorded as `vb_real`.
- **Findings that changed the design**:
  1. *Perception noise reads as intent.* 0.1 m/s of speed noise at 10 Hz, differenced, gives
     about 1.4 m/s^2 of acceleration noise, and A took it for B yielding. The observer now
     takes the slope over 1 s, and a false yield still appears in about 1 run in 16. Intent
     evidence is weak even in the arena, as the review expected.
  2. *A horizon cliff.* A wait at d0 = 20 m could not finish its merge in 12 s and scored as a
     total loss, while the same merge at 12.1 s would have cost 0.4. The horizon is 20 s in C1
     and the arena. C1's conclusions hold at 20 s (no selfish chicken band, the welfare loss
     where A is close and faster, a negotiated cut-in with a prosocial B). An unfinished merge
     still scores as a total loss; C2a replaces that with the time it still needs.
  3. *Arena against C1*: the equilibrium class agrees in 4 of 8 contexts (d0 4-20 m, dv 0 and
     +2), all four at d0 >= 12 m. At d0 4-8 m the arena finds coordination (A goes against B's
     pre-yield, or waits) where C1 has A dropping behind. Seed spreads reach 0.5 there, so 4
     seeds cannot separate the two; that is C2a's work (intervals, more seeds, the boundary).
     C1 with the measured capability gives the same classes.

**D1, decided: build the arena.** Throughput is 10-35 times the criterion (1000 runs per hour)
and parity holds against vehicle-check's acceptance. The calibrated kinematic model is not
needed for mass runs; C1 stays the prediction.

### E2. Autoware's timing on CARLA (2026-11-13; partly open)

What the light policies' perception model and the Autoware-like rules need, measured on the
stack (`just run`, CARLA 0.9.16 on port 2400, a fresh build of both workspaces at the pins).
Scenarios `scenarios/coop/town03_e2_{lead_brake,lane_change}.xosc` (written by
`scripts/coop/gen_e2_scenarios.py`); a ROS bag in the ego domain around each run
(`/mnt/p5plus/aeon/coop/e2/run_one.sh`), the bridge's gap trace as the truth, and
`scripts/coop/e2_bag.py` for the numbers. `max_velocity:=10` (the profile's default is 4.17 m/s,
which kept the ego from ever closing on the lead).

| | clustering (the default) | CenterPoint (`lidar_detection_model:=centerpoint`) |
|---|---|---|
| objects' latency, sim time at receipt minus the scan's stamp | 0.175 s p50, 0.184 p95 | 0.19 s p50, 0.21 p95 |
| tracked speed of the moving lead | **always 0** | error p50 0.13 m/s, p95 1.17 (in the brake), mean -0.09 |
| tracked position (box centre against the trace's origin) | 2.1 m p50 | 0.28 m p50, 0.61 p95 |
| tracked acceleration | never reported | never reported |
| lead brakes at 4 m/s^2, ego 24-28 m behind at 5.7-6.7 m/s: Autoware's braking command | 2.3 s after (speed falls 2.7 s after) | 1.25 s after (1.35 s) |

- **With the default perception Autoware sees other cars as standing still**: clustering gives
  UNKNOWN objects whose tracked velocity is 0, so a cutting-in car has no speed for the lane
  change's or the cruise's checks. C3 runs Autoware with CenterPoint; its numbers are the ones
  the light policies take (`arena.py`'s defaults: delay 0.19 s, speed noise 0.2 m/s, position
  0.3 m).
- **No acceleration from the tracker**: a light policy that reads intent from the other car's
  braking must infer it from speeds, as `arena.py`'s observer does (1 s slope).
- **Lane change, open.** On Town05 a route-forced lane change ran, its lateral move took about 7 s
  at 3.5 m/s (on a 10 % ramp, below). On Town03 the lane-change module reports the change safe,
  automatic and waiting for execution 9 m ahead, while the ego stands after 5 m, three runs out
  of three; the prepare phase is still to be measured on a start where the ego is already
  moving.
- **Sites.** Town05's only long multi-lane straights are the ring: its north side is a 10 %
  ramp to 10 m (the lanelet map is flat at z = 0), and each lane is one 831 m lanelet, on which
  a single NPC made SSv2's own frame take 2.2 s. Town03's east straight (lanelets 69738, 69252,
  68766; 120 m, flat) is the site; one NPC still costs SSv2 about 0.5-1 s a frame there (522 of
  ~740 frames warned), which makes these runs slow, not wrong.

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
| 1 mass | E1 arena spike (done) | E | own CARLA, generated road, one pair then K = 4, 16; runs per hour, tick time, CPU and GPU; step responses against the same car in swb | throughput and physics parity numbers |
| | D1 arena or calibrated model (decided: arena) | D | does the CARLA arena pay for itself? | arena if >= 1000 runs/h per host at K >= 4 and parity holds (speed RMSE <= 0.2 m/s, stopping distance within 5 %, lane change within 0.2 m); low throughput: a kinematic model calibrated on CARLA's step responses, the arena for spot checks; parity fails: fix the physics or drive the arena through `swb_carla_world` |
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
