# Phase 074: Cooperation Study, the Cases swb Can Run Now

Measure how autonomous vehicles cooperate, as games whose payoffs swb measures on real stacks,
starting with the cases swb can run today. The method, the theory and the literature are in
[design/survey-multi-vehicle-cooperation.md](../design/survey-multi-vehicle-cooperation.md);
this phase is its first experiment and the urban cases next to it.

**Status**: started 2026-11-13; C1 done; C2 redesigned for mass runs of light policies (C2a arena, C2b swb check). Design review and the staged plan with exploration and decision steps
(E1-E7, D1-D9) below. E1 done and D1 decided (build the arena); E2 measured (lane change open); C2a's grid and double oracle done; U1 kinematic done; C2b: outcomes agree 24/24, payoffs 92 % after a scoring fix (D2 met); N-car path planned (N1-N5); E2 done (lane change 4 s + 4.8 s; the east straight's target lane is a divider); C3a done (4/6 cells agree with the arena; the imitation reacts too late to a slow cut-in); C3b done (same classes; perception adds 0.1-0.45 s to Autoware's reaction; 14/-3 is a near miss, 0.2-0.6 m); the arena's B refitted to a reacting rule; double oracle rerun with it (do3: Autoware's pair unchanged); utility generalized to every path relation (design ①, 2026-11-14); S0 done (the general utility; cut-in results re-scored: classes and equilibria unchanged, D2 92 %, 14/-3 a shallow footprint touch); next: E3, N1-N2, S1-S6. User direction: "Start from those already doable in
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

**Utility, revised 2026-11-14.** The payoffs above, and every result in this phase up to S0, use
the cut-in's follow-case utility: progress along one lane, risk from the longitudinal TTC and time
headway to the car ahead. `docs/design/cooperative-driving.md` (①) generalizes it to any path
relation: $U_i = -\ell_i - \rho_i - c_i - 10\cdot\mathbf 1[\text{contact}]$, with $\ell_i$ the lost
route arc-length progress over the fixed horizon H = 20 s, $\rho_i$ the largest shortfall over the
cars in $i$'s conflict set $K_i$ of 2D TTC ($\tau^* = 3$ s), time headway ($h^* = 1$ s), PET
($\pi^* = 1.5$ s) and clearance ($d^* = 1$ m), and $c_i$ longitudinal and lateral comfort. On a
cut-in it reduces to the old terms except where the cars are side by side (clearance, 2D TTC) or
cross paths (PET). S0 implemented it and re-scored C1, C2b, C3 and the double oracle (see S0
below): 3.7 % of C1's cells move class, all on band edges, and the equilibria stay; the numbers
below are the follow-case ones unless marked.

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

### E2. Autoware's timing on CARLA (2026-11-13; done)

What the light policies' perception model and the Autoware-like rules need, measured on the
stack (`just run`, CARLA 0.9.16 on port 2400, a fresh build of both workspaces at the pins).
Scenarios `scenarios/coop/town03_e2_{lead_brake,lane_change_west}.xosc` (written by
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
- **Lane change.** On Town05 a route-forced lane change ran, its lateral move took about 7 s
  at 3.5 m/s (on a 10 % ramp, below). On Town03's east straight it never completed, from a
  standstill or on the move (`town03_e2_lane_change_long.xosc`, twice): the ego got 1.7 m across
  at 5.5 m/s, then AEB fired 627 times and the MRM's emergency stop cycled every 2.5 s
  (on 1.9 s, off 0.6 s), so the ego crept to the timeout. **AEB was right**: the target, the
  "innermost" lane (lanelet 68766), is CARLA's Bidirectional lane 1 of road 65, the tunnel's
  divider, with walls (`BP_RepSpline65`) at x = 240.2 from y = 62. **The Town03 lanelet map turns
  that lane into a driving lanelet with a legal change into it**; the inner | outer line is
  solid, so that road has no valid lane change. Any scenario that drives lanelet 68766 meets the
  walls. **Fixed (2026-11-13)**: `scripts/repair_lanelet_non_driving_lanes.py`, in
  `download_maps.sh`'s chain, closes to vehicles (`participant:vehicle=no`) every road lanelet on an
  OpenDRIVE lane that is not a driving lane: Town03's twelve `bidirectional` lanes, 68766 among them,
  none in the other towns. Routing and lane changes now leave them out; the two east-straight
  lane-change scenarios are retired. Found from `/diagnostics` in the bag
  (`autonomous_emergency_braking: aeb_emergency_stop`, `/system/fail_safe/mrm_state`); no object
  was tracked, AEB acted on the points.
- **Lane change, measured** on Town03's west straight (two driving lanes, dashed line, 244 m, no
  signals; lanelets 5750 | 5518, `town03_e2_lane_change_west.xosc`, passes): the module asks to
  start 10 m ahead and finish 60 m on; the lateral move starts **4.05-4.10 s** after the ego
  starts moving (at 4.2 m/s) and takes **4.80-4.85 s** at 6.6 m/s mean, two runs alike. So the
  `autoware` type's t_prep 3 s and t_lc 3 s are short: on this stack from a start, about 4 s and
  4.8 s.
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

**C2a, first slice (2026-11-13).** Two arenas (ports 2600 and 2610, K = 4 each) ran C1's grid,
coarser: d0 -6..30 m step 2, dv -4..4 m/s step 1, 171 contexts x 9 profiles x 4 seeds = 6156
runs in 17 minutes, 13 800 runs per hour per arena (runs are now up to 40 s). Perception from E2
(delay 0.19 s, speed noise 0.2 m/s, position 0.3 m), reaction 1 s +- 0.2.
`scripts/coop/arena_game.py --grid` solves each cell and bootstraps it over the seeds (200
resamples); data in `/mnt/p5plus/aeon/coop/c2a/grid2/`.

```
arena (lower case: bootstrap < 90 %; '.': < 60 %)   C1 (carla_model3)
       d0 -6 .. 30 m, step 2
 +4.0  WccCggggggggggggggg   |  WWWWMMggggggggggggg
 +2.0  WwWcCCcgggggggggggg   |  WWWWWWMGggggggggggg
 +0.0  WWWWw.cCGGGGggggggg   |  WWWWWWWWMGggggggggg
 -2.0  WWWWWWW.CCCGGGGGGGG   |  WWWWWWWWWWGGGGGGGGG
 -4.0  WWWWWWWWWWCCCCgGGGG   |  WWWWWWWWWWWWGGGGGGG
```

- The bootstrap is confident (>= 90 %) in 146 of 171 cells; in 118 of those the arena's class
  is C1's. The four regions are the same (drop behind, a boundary band, go and B yields, go and
  B need not); they differ in the band.
- Where C1's band has only mixed equilibria (M), the arena's has coordination (C): both "drop
  behind" and "go" are equilibria. Uncertainty from noise and reaction-time spread turns C1's
  knife-edge into two stable outcomes, which is where a convention would decide.
- At dv > 0 the arena reaches "go" at smaller d0 than C1 (CARLA's car accelerates harder).
- **Contacts (479 runs) are all A going against a closing B**: d0 8-18 m with A 3-4 m/s slower.
  B sees the lane change only after the perception delay and a lateral threshold, then reacts a
  second later; C1, which sees the move at once, calls the same cells safe. So the perception
  delay, not the physics, moves the safety boundary.
- **"After you" deadlock (910 runs)**: B yields first (pre_yield) while A drops behind or waits
  for a yield it misreads, both slow, and A is not in after 40 s.
- Two bugs found and fixed on the way: the analysis took "A merged" for "A merged ahead"
  (`ahead` is recorded now; with it agreement rose from 82 to 118 cells); and the grid script's
  `wait` also waited for the CARLAs it had started.

Still to do in C2a: rules over a continuous psi, double oracle with exploitability (and the
Autoware-like rule's), the split surrogate, more seeds where the bootstrap is unsure, and $W$,
PoA and PoS per cell from the arena's payoffs.

**C2a, double oracle (2026-11-13).** `scripts/coop/double_oracle.py` plays the rules of
`arena.py`'s parameterized families on three arenas (ports 2600-2620, K = 4), over six contexts
(the band and the contact cells: d0, dv = (2, 2), (6, 0), (6, 3), (10, 0), (10, -2), (14, -3)) and
two seeds each:

- A: go when the gap ahead of B (as seen) >= buffer + v_B tau + max(0, v_B^2 - v_A^2)/(2 a_rear), the
  form of Autoware's lane-change check; checking for `patience` s, optionally also on a seen
  yield; then drop behind; `t_prep` before the lateral move.
- B: yield at b_y when A is within `pre_band` before any lane change is seen, or within
  `react_gap` once it is; otherwise hold. The safety layer always brakes.
- Starting rules: Autoware-like A, conservative (tau 2 s, a_rear 1 m/s^2, 3 s prepare) and
  permissive (1 s, 2 m/s^2, 3 s prepare); a bold A; Autoware-like B (holds: no yield to a cut-in,
  only the safety layer), a courteous B, a reactive B.
- Meta-game: every Nash equilibrium by support enumeration, the highest-welfare one kept; best
  responses by 40 random rules plus two rounds of local refinement around the best three, earlier
  best responses retried. 7812 runs in 13 minutes. (A first run with 16 + 8 candidates stopped at
  round 0 on a best response that a later search beat: the search budget matters; data in
  `c2a/do1`.)

Result (`/mnt/p5plus/aeon/coop/c2a/do2/`):

| round | equilibrium | vA | vB | welfare | best-response gain A / B |
|---|---|---|---|---|---|
| 0 | A_aw_perm / B_aw (holds) | -0.117 | -0.007 | -0.124 | -0.000 / 0.000 |
| 1 | A_aw_perm / B_aw | -0.117 | -0.007 | -0.124 | **+0.008** / 0.000 (A: tau 0.68, a_rear 2.07, buffer 4.2 m, no prepare) |
| 2 | that A / B_react (yields when the lane change is seen within 10 m) | -0.108 | -0.059 | **-0.167** | 0.001 / 0.003: stop |

- **The equilibrium**: A checks the gap with tau = 0.7 s and a_rear = 2 m/s^2 (Autoware's
  permissive assumptions, roughly, but without the 3 s prepare phase); B yields once it sees the
  change within ~10 m. No contacts on these contexts. Exploitability found: 0.003 (a lower
  bound).
- **Smarter selfish rules lower welfare**: A's better rule gains A 0.009 and costs B 0.052, which
  then yields; the equilibrium's welfare (-0.167) is below the Autoware-like pair's (-0.124). A
  first measured instance of a price of anarchy between rules, not just within one context.
- **Autoware's conservative lane-change assumption never cuts in** on these contexts (A ahead in
  0 % of runs against every B, utility -0.24 against the equilibrium A's -0.11): waiting behind
  is its only outcome. The permissive one cuts in 1 time in 3 against a holding B.
- **Autoware-like B (holding) is almost a best response for a selfish B** (gain 0.0001 at round 0);
  only against the sharper A does reacting pay B (0.003).
- Noise: two seeds per context, so differences under ~0.01 are not resolved, and a counter-rule
  search against B_aw found a weaker rule (-0.159) than one the oracle already had (-0.108); the
  exploit numbers are lower bounds of lower bounds. More seeds and a shared candidate archive
  next.

### U1. The left turn against an oncoming queue, kinematic (2026-11-13)

`scripts/coop/left_turn_game.py`: A waits to turn left across one oncoming lane; a chain of n
cars at speed v and headway h; A's rule is a gap-acceptance threshold G (1-8 s, seen with the
0.19 s delay); the chain's first car C1 holds or yields (2 m/s^2 when A is waiting within 40 m);
its followers follow (IDM). Welfare counts every car in the chain. At v = 10 m/s:

- For short queues the game is a coordination game with two equilibria: A waits for a 3.5-4 s
  gap while C1 holds, or C1 yields and A goes at once. Holding is the welfare optimum: a yield
  delays every follower, and the yield equilibrium costs 2-2.5 times as much for n = 1-3.
- For long, sparse queues (n = 5, h 2.5-4 s) only the yield equilibrium is left, and at h = 4 s it
  is also the optimum: holding makes A wait 12 s for the whole queue.
- So courtesy at a left turn pays only when the queue behind the courteous car is long enough to
  trap A; a convention "yield to a waiting left turner" is right in sparse traffic and wrong in
  dense. The arena and swb versions are next.

**C2b. The swb check (sample).** *Design settled (2026-11-13).* The user chose a ground-truth pose for the light policies over
Autoware's localization. Two ways were found that need no change to acb: acb already publishes
the ego's true pose on `/carla/ground_truth/odom`, always; and swb already has an agent that
drives a scenario entity with no ROS and no Autoware, `swb_agent.adapters.carla_basic`
(CARLA's BasicAgent; it reads the world, applies control to its own vehicle only, on world
ticks, and exits when the server goes). The world server applies the spec's physics to every
vehicle a stack drives, `agent` entities included. So C2b is a cut-in agent on carla_basic's
pattern (`scripts/coop/cutin_agent.py`): it finds its car and the other by role name, reads both
from the world snapshot, and runs `arena.py`'s rules, perception model and controls (pedal maps,
lags, steering) on world ticks; the scenario has A and B as `agent` entities and no Autoware;
the context (d0, dv, v_B) and the rules are the scenario's parameters, and the agents do the
warm-up themselves, as the arena does. Site: Town03's east straight and the curve after it
(lateral targets follow the lane's waypoints, not a straight line). A few cells per region, chosen where the arena's map changes,
rerun through swb itself: `multi_av/town05_cut_in.xosc` with A and B as agent entities, each
driven by the same light policy behind acb, the trace (024) and contacts (021) as payoffs. The
policy is a small ROS node per car: it reads its own odometry and the other car from acb's ground
truth (`publish_ground_truth_objects:=true`, `/perception/object_recognition/objects`, Autoware's
perception off) and publishes Autoware's control command to acb. No Autoware is needed. It shows the arena measures what swb measures; any difference names a
gap in the arena (spawn, physics settings, timing) before its numbers are trusted.

**C2b and D2, first sample (2026-11-13).** `scripts/coop/cutin_agent.py` (the light policies as
swb agents), `scripts/coop/c2b.py` (a two-agent Town03 scenario per run through `just scenario`,
the agents' truth logs scored as the arena scores, and the same cells rerun in the arena with
`--arena-port`), `scripts/coop/d2_compare.py`. Stack: `just run map_path:=<Town03>
ego_launch:=none vehicle_physics:=<the spec's carla_physics.yaml>` (without `vehicle_physics` the
agent cars run the blueprint's physics, and both levelled off while asking for acceleration).
Cars in CARLA's driving lanes 2 and 3 of road 65 (its lane at x = 239 is Bidirectional, though
Autoware's lanelet 68766 there is a driving lane). v_B 8 m/s, 8 s warm-up; six cells x two rule
pairs (the double oracle's equilibrium; the Autoware-like pair) x two seeds = 24 swb runs, against
4 arena seeds each. Data in `/mnt/p5plus/aeon/coop/c2b/run1/`.

- **Outcomes agree in 24 of 24 runs**: who merges ahead and whether anyone touches (nobody does)
  are the same in swb and the arena, cell by cell, for both rule pairs. Contexts reached in swb:
  d0 within 1.2 m, dv within 0.5 m/s of the target (the arena's warm-up is tighter).
- **Payoffs: 34 of 48 (71 %) inside the arena's 95 % intervals; D2's bar (90 %) is not met.**
  The misses are one-sided: where A drops behind (6/0, 10/-2, 14/-3) or B yields (10/0), the
  arena's car loses 0.02-0.04 more utility than swb's (10-25 % of the value). Where A goes
  ahead cleanly the two agree.
- A first scoring of the swb runs left A's braking term out and differenced B's speed raw;
  rescored as the arena measures (smoothed), the miss narrowed from 69 % to 71 % inside and the
  offsets from ~0.05 to 0.02-0.04 (the first scoring is kept as `c2b.first_scoring.jsonl`).
- **D2: mismatch; locate before trusting the arena's payoff levels** (its outcome classes are
  supported). Candidates, in order: lost progress is measured from straight-line positions in the
  arena but from integrated speed along a curving path in swb; the run's end and the warm-up's
  end state; one tick of control latency in the agent (it acts on the world tick's callback).
  The equilibrium classes rest on payoff *differences* within a cell, which a common offset does
  not move; the welfare levels and the double oracle's small gains (~0.003-0.01) do depend on it.

**D2, second sample (2026-11-13): met.** The offset was the scoring, not the physics. Both sides
scored lost progress over the run's own length, and swb's runs lasted 23-30 s (its lateral
measure on the curve plateaued at ~0.96, so "merged" came late), which diluted or inflated the
loss; and peak braking was the largest sample, so it depended on the logging rate. Fixed in both
(`cutin_game`, `arena`, `c2b`): a merge is complete at decide + t_prep + t_lc (the plan, as the
agent runs it) with the lateral at >= 0.5 of a lane; lost progress is a share of free flow over the
fixed HORIZON (a longer run adds its loss without diluting it); peak braking is the speed drop over
a 0.5 s window. In the agent: lanes are tracked by continuity (the lane lookup failed on the
curve at 20/1), and B sees A's lateral through the observer, as in the arena, not from truth.
Rerun of all 24 runs (`run2/`, against `run1`'s arena runs rescored the same way):
- **Payoffs: 44 of 48 (92 %) inside the arena's intervals; outcomes agree in 24 of 24. D2 met.**
- The four misses are all one cell, 10/0, where B yields: the arena's B loses 0.123, swb's
  0.086-0.100. B's yield is softer in swb (the observer sees A's move later on the curve than on
  the arena's straight road). The cell's class (A ahead, no contact) agrees.
- The arena is trusted for outcome classes and payoff levels within 0.02, except a yielding B's
  cost, which it overstates by up to 0.04 at 10/0; stage 5 may start.

**Raw measures (2026-11-13).** `scripts/coop/measures.py`, recorded by the arena and by `c2b.py`'s
scoring beside the utility (which is unchanged: run2's rescored payoffs are identical): DRAC and
the capability margin, TET/TIT, impact speed, stops and stopped time, peak jerk, and event times
for the reaction latencies. In run2 every run has margin 1 and TET 0 (no closing on a shared
lane); B's reaction to A's move where B yields (10/0) is 1.5 s (equilibrium pair) and 2.6 s
(Autoware-like pair), A's braking jerk when it drops behind 3.6-5.5 m/s^3.
Against the arena's runs of the same cells (arena / swb, means): B's reaction to A's move
1.64 / 1.50 s and 2.69 / 2.60 s; A's lateral start within 0.05-0.3 s in 10 of 12 cells (the 2/2
cell 1.0-1.2 s later in swb); A's jerk when it drops behind 3.7-5.7 / 3.6-5.5 m/s^3. The one
clear difference is B's jerk when it yields (10/0): 5.3-5.6 in the arena, 3.7-3.8 in swb, the
same softer yield that D2's remaining misses show.

### C3. B as Autoware

First with perception off and acb's ground-truth objects (planning and control are Autoware's,
the objects exact): this isolates the planner's strategy from perception. Then the full stack.
B a full Autoware stack (the ego, or `just bg-av`), A a light agent. B's yield and hold have to be
found among Autoware's own knobs first (obstacle cruise's following distance, the lane change
module's safety margins; the survey's "Strategy knobs" gap); until then B is Autoware as shipped,
one strategy, and the map shows which of A's choices it rewards. Then A by RTC (go now / wait,
`enable_rtc` on), then both Autoware.

**C3a, first map (2026-11-14).** B the Autoware ego with ground-truth objects
(`publish_ground_truth_objects:=true`, occupancy grid `laserscan_based`: another user's vLLM held
27 of 32 GB of 133's GPU), A a light agent (`scripts/coop/c3.py`, `c3_run.sh`), on Town03's west
straight (two driving lanes, dashed line; the east straight's third lane is the tunnel's divider).
v_B 6 m/s: at 8 the 20 s scoring horizon ran A off the straight's end. The six C2b cells x the two A
rules x 2 seeds, against the arena's same cells at 6 m/s with B the Autoware-like rule (B_AW, 4 seeds).
Data `/mnt/p5plus/aeon/coop/c3/c3a{,_6_0}/`.

| cell (d0, dv) | arena, B = B_AW | Autoware |
|---|---|---|
| 2, +2 | A ahead, both ~0 | the same; Autoware does not brake |
| 6, 0 | A_aw_perm ahead 4/4 (uB -0.25), A_eq 1/4 | A behind 4/4 (uA -0.28) |
| 10, 0 | A ahead, uB -0.09 | A ahead, uB -0.12 |
| 10, -2 | A behind | the same |
| 14, -3 | contact: A_eq 4/4 at 3 m/s, A_aw_perm 2/4 | no contact 4/4; Autoware brakes (uB ~-0.4), A ahead 1/4 |
| 20, +1 | A ahead, ~0 | the same |

- **Four of six cells agree** in outcome and within ~0.03 in payoff.
- **D4's first residual (14/-3): the imitation reacts too late.** B_AW reacts only once A is in its
  lane; Autoware brakes on the lane change's onset (its prediction sees A's lateral motion), so a
  slow car cutting in 14 m ahead is a contact in the arena and a firm brake on the real stack. The
  arena overstates contact where A is slower than B. Next: give B_AW a reaction to A's lateral
  motion (the observer's lateral), refit, rerun C2a's band.
- **Boundary sensitivity (6/0).** Autoware cruises ~0.2 m/s above its limit's set point relative to
  A (5.8 against 5.6), so the 6.6 m gap closes and A_aw_perm (needs 8 m) waits out its patience
  and drops behind; B_AW holds its speed exactly and the gap lets A in. The cell sits on the
  ahead/behind boundary; its class flips on a 0.2 m/s cruise difference.
- **Bugs found on the way, fixed**: the stack never gave the ego its spec physics' pedal maps
  (`stack.launch.py`: the ego ran to 30 m/s against a limit of 8), acb's ground-truth objects all
  faced east (headings converted to radians twice, so a car in the next lane had a path across the
  ego's lane and obstacle_stop halted it), the agent took CARLA's turn branch at a junction (both
  branches start tangent; now judged 6 m along).
- **Refitting the imitation (`scripts/coop/c3_fit.py`)**: B rules reacting to A's lateral motion
  (react_gap 12-30 m), B's reaction 0.5-1.0 s, braking 2.5-7 m/s^2, B seeing ground truth and A
  the agent's observer (the arena now takes per-side observers and a per-run reaction). Autoware's
  measured reaction is 1.0 s from A's lateral onset, with a hard brake (B jerk 8.8-11 against the
  arena's 6.5). Five cells fit with any reacting rule; 14/-3 with A_aw_perm (3 s prepare) does not:
  the arena side-swipes (impact ~0.1 m/s) in 3/4 runs at 2.5 m/s^2 and 2/4 at 7, while on the real
  stack A ends at a standstill and nobody touches. Best so far B_r30_b7_t1 (error 5.0, against
  10.4 for B_AW). Open: whether the gap is B's braking or A's port (the agent stops; the arena's A
  presses on into B's flank), to locate with both cars' traces side by side. **Located**: the
  real runs are near misses, not clear passes. A (3 s prepare, 2.3 m/s) is part-way over (lat
  0.27) when B draws alongside; A brakes to a stop following B, Autoware stops too, and the
  bodies clear by 0.42 and 0.61 m (seeds 0, 1; 1.35 m in C3b's seed 2). The arena's same
  sequence ends a few tenths of a metre further over and grazes at ~0.1 m/s. The cell is on the
  contact edge, where a binary contact flips on ~0.5 m; compare it by the minimum clearance (and
  TET/margin), not by contact. Under C3b's perception (B with CenterPoint's observer in the
  arena) the fit is the same: five cells with any reacting rule (best B_r20_b2.5_t1), this one
  on the edge.

**Double oracle with the measured Autoware (do3, 2026-11-14).** B's starting pool gained
B_aw_react (react to the lane change's onset within 20 m, 2.5 m/s^2), 10 512 arena runs at the
double oracle's contexts (v_B 12 m/s). The Autoware pair is unchanged: A_aw_perm against B_aw_react
gives uA -0.117, uB -0.008 (against B_aw -0.116, -0.007). The equilibrium is again an aggressive A
rule (no prepare) against a reacting B, welfare -0.165, no contact, exploitability lower bound
0.0005. Exploiting B_aw_react: A without prepare (tau 0.63 s, buffer 5.2 m) gains ~0.007 and moves
B from -0.008 to -0.065, no contact; against B_aw the same counter gives B -0.082. At these
contexts the onset reaction matters little; C3's differences were at 6 m/s with slow cut-ins.

**C3b, Autoware on its own perception (2026-11-14).** The same cells with CenterPoint
(`lidar_detection_model:=centerpoint`), run on 118 (GPU 0, seeds 0-3, 48 runs, all valid) and 241
(GPU 1, the arena's cells; 14 of its 24 runs lost to Autoware staying INITIALIZING after the ego's
restart on a loaded host, so c3.py now retries such a run with its cause). Data
`~/nas/autoveh/dataset/2026-11-13_coop-c3/` (`c3b_118`, `c3b_118_s01`, `c3b_241`).
- **Every outcome class is the same as with ground truth**, and no run touches. Perception adds
  0.1-0.45 s to Autoware's reaction to the lane change's onset (1.1-1.45 s against 1.0), as E2's
  0.19 s object latency plus tracking predicts. Payoffs move only at 14/-3.
- **Minimum clearance** (body to body, from the agents' truth logs) per cell, C3a | C3b: 2/+2
  2.95-2.97 | 2.91-3.21 m; 6/0, 10/-2 1.36 | 1.35-1.36 (side by side in the next lane); 10/0
  9.4-10.0 | 9.1-9.9; 20/+1 20.4 | 20.3-20.5; 14/-3 A_eq 1.35-3.79 | 1.35-2.88; **14/-3 A_aw_perm
  0.42-0.61 | 0.21-1.35**. The 3 s prepare at 14/-3 is the one near miss, on both perceptions.
- **The imitation refitted under C3b's perception** (B seeing through the CenterPoint observer):
  the same five cells fit with any reacting rule, best B_r20_b2.5_t1 (error 10.7 against 27.2 for
  B_AW's no reaction); the rest is the 14/-3 contact edge.
- **D4 for C3**: the arena's B becomes a reacting rule (react to A's lateral motion within
  20-30 m, 1.0 s, 2.5 m/s^2; under ground truth B's observer is exact); the arena is trusted for
  outcome classes in these cells and flags the near-miss cell by clearance, not by contact. Next:
  rerun C2a's band and the double oracle with it.

### S0. The general utility (2026-11-14)

**Implemented** (22741ab): `scripts/coop/measures.py` scores a run from both cars' poses as
oriented boxes (the spec's footprint, 4.79 x 2.163 m): 2D TTC, time headway, PET, body
clearance, route arc length, peak lateral acceleration, `rho` and `utility` as design ① defines
them. Contact is now footprint overlap everywhere (C1, the arena, swb's scoring), with the
overlap depth `pen` and the relative speed at first touch `v_touch` recorded beside it.
`cutin_game.py`, `arena.py` and `c2b.score` use it; `c2b.score` rebuilds poses from the agents'
logs and holds the lane tracker's lateral jumps at the east straight's junction end;
`s0_rescore.py` re-scores recorded swb runs offline. `cutin_agent` logs both yaws, so future
logs carry the heading; older ones are scored in the lane frame. Data
`/mnt/p5plus/aeon/coop/s0/`.

- **C1, kinematic map**: 23 of 629 cells change class (3.7 %), all at band edges: 10 M->W, 9
  G->M, 2 g->G, 2 g->M; no contact cell changes. The cause is the new terms, not a bug: lateral
  comfort charges every lane change about 0.063 (a 3 s cosine change peaks at 1.83 m/s^2), and the
  footprint enters B's corridor before lat 0.5, so the go/react cells near the edge see headways
  of 0.92-0.96 s (rho 0.04-0.08). C1 now takes ~38 s instead of 3 s.
- **swb, 130 runs re-scored** (C2b, C3a, C3b): the merge order is unchanged in every run. Two
  contact changes, both explained:
  - C3a 6/0, 4 runs: the old contacts were false. A had turned off at a junction, 26 m from B,
    and the old rule projected `along` on A's heading. Removed. The same junction turns (the runs
    predate the cutin_agent branch fix) leave C3a 2/+2 A_aw_perm at uA -0.31 from a real 4.3 m/s^2
    lateral peak (A turned at 7.5 m/s); C3b's same cell peaks at ~1 m/s^2. These C3a cells are
    artefacts of the agent, not of Autoware.
  - 14/-3 A_aw_perm: the footprints overlap in 6 of 8 C3a/C3b runs (C3a 2/2, c3b_118 1/2,
    c3b_118_s01 2/2, c3b_241 1/2), 0.1-7.9 cm deep, at 3.3-4.4 m/s relative speed. A has stopped
    at ~16 degrees of yaw with its front corner in B's footprint while Autoware's B creeps past;
    B's motion stays smooth, so this is a graze of square-cornered boxes at most, not a visible
    bump. The 0.2-0.6 m "near miss" of C3 was measured between straight boxes. The two runs
    without overlap clear by 0.18 m (uA -1.24) and 1.12 m (-0.07).
- **D2** (C2b's run2 re-scored against the arena under the same utility, `d2/d2_compare.txt`):
  44 of 48 payoffs inside the arena's 95 % intervals (92 %), outcomes 24/24, as before S0. The
  four misses are uB at 10/0 (swb -0.09 to -0.105, arena -0.125).
- **C3's refit** (`c3_fit.py`, `fit_gt/`, `fit_cp/`): ground truth best B_r30_b4_t0.5, error
  1.58, against 1.61 for B_aw_react (r20, b2.5, t1), the error now dominated by the C3a
  artefacts above; CenterPoint best B_r20_b2.5_t0.5, 0.164, against 0.191. Before S0 the errors
  were 5-27, carried by the contact mismatch at 14/-3, which is gone: under the footprint both
  the arena (contact 0.75) and the real stack touch there. B_AW_REACT stays as it is (within
  0.03 of the best).
- **Double oracle, do4** (the same contexts and pools as do3, v_B 12 m/s, `do4/`), 6984 runs,
  converged in round 1:

| | do3 (follow-case utility) | do4 (general utility) |
|---|---|---|
| round 0 equilibrium | A_aw_perm / B_aw | A_aw_perm / B_react |
| final equilibrium | A2 (tau 0.49 s, buffer 5.65 m, no prepare) / B3 (react_gap 7.5 m, 1.9 m/s^2) | A1 (tau 0.71 s, buffer 4.3 m, no prepare) / B_react (react_gap 10 m, 2.5 m/s^2) |
| vA, vB, welfare | -0.108, -0.058, -0.165 | -0.148, -0.062, -0.210 |
| contact, A ahead | 0, 0.5 | 0, 0.5 |
| rounds, exploitability lower bound | 3, 0.001 | 1, 0.004 |
| A_aw_perm / B_aw_react | -0.117, -0.008 | -0.156, -0.010 |
| A_aw_perm / B_aw | -0.116, -0.007 (A ahead 4/12) | -0.153, -0.042 (A ahead 5/12) |
| exploiting B_aw_react: A's gain, B's value | 0.007, -0.065 | 0.004, -0.068 |
| exploiting B_aw: B's value | -0.082 | -0.083 |
| A_aw_cons against its counter | -0.478 | -0.522 |

  The equilibrium class stays: an aggressive A without prepare that gets ahead in half the
  contexts, against a reacting B, no contact. A's values fall by about 0.04 everywhere it changes
  lanes (lateral comfort and route arc length), which moves welfare by the same amount and leaves
  the ranking alone. B_aw against A_aw_perm drops to -0.042 because one of the twelve contexts
  flips to A ahead, a boundary context; that is also why round 0's B is B_react, which was within
  0.001 of B_aw in do3. Exploiting the Autoware-like rules gives the same picture as do3.

Verdict against the criterion:
- **Classes unchanged**: met. C1 moves 3.7 % of cells, all on band edges, for reasons the new
  terms name; swb's merge order is unchanged in all 130 runs; every contact change is explained
  (false junction contacts removed, 14/-3 now a touch).
- **Equilibria unchanged**: met (do4 above).
- **The 14/-3 near miss scored smoothly by clearance**: not met as stated, and recorded as a
  finding. Under the footprint it is not a near miss but a shallow touch, 0.1-7.9 cm, in 6 of 8
  runs, which the arena shows too, so the clearance term has nothing left to smooth: the score
  still jumps by 10 at contact. Clearance does smooth the runs that stay apart (0.18 m: -1.24;
  1.1 m: -0.07). `pen` and `v_touch` stay beside the binary contact, so a graze of square boxes
  can be told from a crash when the cell is analysed.
- **D2 >= 90 %**: met, 92 %.

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
| | D7 deployable? | D | | never worse than stock on contact and on the distributions of 2D TTC, PET and clearance (①), a significant welfare gain, within the message-rate budget: propose it; else keep it as research |
| 5 cases | E5 left-turn signals | E | can SSv2 give the left turn and the oncoming queue green together on Town05/10 | yes or a workaround |
| | U1, U2 | B | left turn against a queue; roundabout entry (Town03); each kinematic, arena, swb | maps |
| | E6 4-way stop site | E | lights switched off on a Town05 junction, or an unsignalled junction | site |
| | D8 4-way stop | D | | site found: U3; else skip |
| | E7 merge site | E | a lane drop on Town03/05 | site or none |
| | N1 conflict-set sizes | E | how many cars are in each car's conflict set, per slot, in the arena's and SSv2's multi-car scenes (and highD if it is at hand) | the size distribution; the literature's 3.2 mean / 7 max (survey, N cars) checked on ours |
| | N2 composition | B | three-car arena games (a cut-in between B and its follower; a cut-in with a lead ahead of A) against the pair terms composed per measure (risk max, delay max-plus, braking max) | the residual $r$ per context with its bootstrap interval |
| | D9 pairs enough? | D | does the composed model predict three cars | $r$ within the D2 tolerance in >= 90 % of contexts: pairs suffice, N grows only by composition; systematic: N3; large and irregular: those contexts stay explicit games, try N = 4 there only |
| | N3 residual model | B | a permutation-invariant residual over the slots, fitted on three-car runs, tested on four | held-out error against the composed model's |
| | N4 far field | B | a long merge or ring with 10-50 cars: near field exact, the rest as density and mean speed, against all cars explicit, at increasing N | the mean-field error against N, and where it falls below the tolerance |
| | N5 swb check | B | the merge on a found site (E7), or a Town04 map (031) only if N2-N4 call for one | D2's agreement for three cars |
| 6 scenarios | S0 general utility | B | route progress, 2D TTC, PET, clearance, lateral comfort (`docs/design/cooperative-driving.md` ⓪, ①) in `measures.py` and `cutin_game.utility`; C2b, C3a, C3b and the double oracle re-scored offline from the recorded runs; prerequisite for S1-S6 and for N2's pair risk $\rho_{ij}$ | done (22741ab; S0 above): classes and equilibria unchanged; D2 92 %; 14/-3 is a 0.1-8 cm footprint touch, not a near miss, so it still scores as contact (`pen` kept beside it) |
| | S1-S6 | B | cut-out; unsignalled crossing; lane drop or on-ramp; narrow passage; left turn in the arena and swb; overtaking via the oncoming lane: each as a passage game (relation, Δt, speeds, right of way), kinematic, arena, swb light, swb Autoware | equilibrium maps per relation; Autoware's departures |

Order: stage 1 first. S0 comes before any new relation (S1-S6) and before N2, whose pair terms
compose $\rho$ per pair; the cut-in's results are re-scored, not re-run. Stage 5 starts after D2 (it needs a trusted arena) and runs beside stages
2-4. Its N-car path (N1-N5) follows the survey's formulation: pair terms, composition per
measure, a measured residual, N grown by error, mean field past the conflict set. Scooters and the night market are a later phase (reactive NPCs).

## Design review, 2026-11-13

User direction: "Review the design. Explore design flaws in theory, impractical assumption and
workflow. Fix them if there is any." Found and fixed:

| # | flaw | fix |
|---|---|---|
| 1 | Theory: the cut-in was solved as a one-shot simultaneous 2 x 2, but B sees A's lane change and replies; B's "hold" against a car that went was a threat B would not carry out, and the chicken band came from it | contingent strategies, B's reply sequentially rational (C1 rewritten); chicken survives only where something forces simultaneity: B's courtesy before any signal, latency, or prosocial preferences |
| 2 | Theory: PoA and PoS as ratios of welfare, but every utility is <= 0 | welfare as a cost C = -(U_A + U_B) >= 0; ratios when C_opt is not near 0, and the loss C_eq - C_opt always (unchanged under the general utility: C = sum of $\ell + \rho + c$ + contact) |
| 3 | Theory: summing utilities normalised per car ("fraction of free flow") compares a golf cart's 10 % with a car's 10 % | the global target in common units (vehicle-seconds of delay, safety as a constraint) for welfare; per-car normalisation stays inside each car's own payoff (survey); since ①, $\ell_i$ is the route arc length lost over H as a fraction of $v_{i,0} H$, inside the car's own payoff |
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
