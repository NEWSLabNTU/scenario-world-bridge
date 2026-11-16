# Phase 074: Cooperation Study, the Cases swb Can Run Now

Measure how autonomous vehicles cooperate, as games whose payoffs swb measures on real stacks,
starting with the cases swb can run today. The method, the theory and the literature are in
[design/survey-multi-vehicle-cooperation.md](../design/survey-multi-vehicle-cooperation.md);
this phase is its first experiment and the urban cases next to it.

**Status**: started 2026-11-13; C1 done; C2 redesigned for mass runs of light policies (C2a arena, C2b swb check). Design review and the staged plan with exploration and decision steps
(E1-E7, D1-D9) below. E1 done and D1 decided (build the arena); E2 measured (lane change open); C2a's grid and double oracle done; U1 kinematic done; C2b: outcomes agree 24/24, payoffs 92 % after a scoring fix (D2 met); N-car path planned (N1-N5); E2 done (lane change 4 s + 4.8 s; the east straight's target lane is a divider); C3a done (4/6 cells agree with the arena; the imitation reacts too late to a slow cut-in); C3b done (same classes; perception adds 0.1-0.45 s to Autoware's reaction; 14/-3 is a near miss, 0.2-0.6 m); the arena's B refitted to a reacting rule; double oracle rerun with it (do3: Autoware's pair unchanged); utility generalized to every path relation (design ①, 2026-11-14); S0 done (the general utility; cut-in results re-scored: classes and equilibria unchanged, D2 92 %, 14/-3 a shallow footprint touch); N3 done, D9 met (composition holds for queues up to seven cars); E4/C5 done (commitment not supported; a heavy-vehicle spec deferred until an Autoware test needs one); C4 done, D5 decided (κ rules on the car, the potential model to predict others); C6 done, D6 decided and confirmed by the user (no belief node: B's intent, read by a probe rule, carries the value; capability broadcasts are worth 0); C7 done (binding priority at a tie ends every restart crash; the zipper shortens the queue, not throughput); S6 kinematic and arena done (overtaking: chicken only in a narrow band where B can give way; the abort fails, harder in the arena; only the margin keeps it safe; D2 accepted for outcomes, 96 %); S1 kinematic and arena done (the cut-out's danger is look-through, not the gap; L does not dump risk on B; D2 accepted for outcomes and the risk band); S3 kinematic and arena done (a lane drop's deadline makes the merge a coordination game near the end; D2 accepted for classes, 14 of 16, and outcomes, 93 %; the zipper needs a queue); S2 kinematic and arena done (a near-tie, |Δt| ≤ 0.2 s, is a war of attrition and costly either way; the crossing's D2 accepted for outcomes, 95-98 %, and classes, payoff levels from the arena); N1-N2 done (conflict sets sparser than the literature's; composed pairs predict 81 % of car-cells, D9: not enough, the residual systematic, so N3); N3 first round (window fixed, follower bug fixed; held out 79 % of contexts, 92 % of cells; one more round decided); C3a rerun done (14/-3 a 1 cm near miss on ground truth; the arena's B lacks Autoware's last-moment stop); N3 second round done (held out 82 % of contexts, 92 % of cells; D9 decided: composition for queues up to 4 cars, a closing lead and longer queues stay explicit); N3 window cap and N4 done (composition holds for any queue with slack; tight queues need a depth term; in a merge N does not matter past about 10 cars, mean-field error 0.05); E3 done, D3 decided (stock Autoware is one strategy; the cruise-gap knobs, load-time only, are strategies for how far B yields back); friction canaries done (no verdict changes); C7's coin at the stop done (it recovers the gap at a crossing with no restart crash; gate it to the conflict window, and a car that cannot stop keeps the turn); C3c rerun on the friction fix done (swb 241; no verdict changed, C3c final); U3 kinematic and arena done (stop-then-yield breaks ties by noise; FCFS alone crashes at ties; FCFS + right-before-left removes them; mixed conventions deadlock), its stop-sign map variant built (5d94dee); the Autoware 2.0.0 profile done (swb 139, c2a2879; autoware_release:=2.0.0, 1.5.0 the default);  the swb friction fix, full CenterPoint as `centerpoint` and the spec's lidar intensity model done (fe63f5d; TransFusion sees cars offline); closed-loop S5 on the new lidar done (full CenterPoint unchanged, 1 contact in 8; TransFusion 6 in 8, its detector reaches 20-25 m and loses A in the junction: the reflectance table needs a replay sweep); open: a heavy-vehicle spec deferred by the user until an Autoware test needs a truck; arena tracking checked (the D2 gaps are a launch delay and S4's held-car crawl in the kinematic model; switches adopted, crossing family re-run: D2 payoffs up, e.g. S2 73/75 %, S4 outcomes 89/90 %, no contact in any equilibrium; HOLD_STOP kept out of the arena, which coasts to rest; held arena cars brake to rest, done (4fcaa8d; D2 moves a few points either way; the near-tie gap is perception noise, not a brake-release lag: the band is scored by contact share (outcomes up 1-6 points, S2 v14 96 → 95 %); at 20 arena seeds D2 stands, share gap median 0.05)); N5 (a) done (the Y merge is S2's crossing game with a cheap fall-in-behind; N stops mattering by about 10); N5's swb check done (three cars, outcomes 15 of 15, payoffs 68 %; D2 accepted); with stock Autoware as B, paced rerun: outcomes 11 of 15 against HOLD, the misses Autoware's STOP-then-go policy against a cautious A, mutual-yield waits in 10 of 30 runs at A-first Δt, late tracking is static occlusion at the corner; next: U3 on Autoware (swb 139); C8 on U3 done (fcfsr unsafe for any e > 0; the fail-safe rule with the map order, (e), safe and live for 2-4 cars in the model, 0 contacts in arena self-play; never worse than stock not met against stock's static-at-rest; noise ×2 needs guard hysteresis, heavy cars their own zone interval; G0 in 078); 078 P1 built (czp; G0: only the unsignalled crossing enters the table, the all-way stop, narrow passage and lane drop stay stock: a car waiting by its role is read as static by a gap-test driver; G1 passed for the crossing), then P1 with no V2X (stock, + convention and probe, + learned role estimate and response model; intent messages last, optional, by adoption rate). User direction: "Start from those already doable in
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

### C3a rerun with the fixed agent (2026-11-14)

Run by swb 241 at eacaae0, which includes the branch fix and S0. Autoware ran on ground-truth
objects at 6 m/s: 48 runs (6 cells, 2 A rules, 4 seeds), all clean. A stays on the west straight
through the scored window in every run. Data: NAS `logs/2026 SSv2 on CARLA/2026-11-14 coop C3a
rerun/`.

- **The old junction-turn artefacts are gone.** 2/+2 A_aw_perm now scores uA -0.037; the old runs
  scored -0.31.
- **14/-3 A_aw_perm is a near miss, not a touch, on ground truth.** There is no contact and no
  footprint overlap in 4 of 4 runs. The smallest clearance is 0.01 m, rho is 0.49, and uA and uB are
  -0.77 and -0.72. On CenterPoint (C3b) the footprints touched in 4 of 6 runs, by 0.1-8 cm.
  Autoware stops within centimetres when it sees exact objects.
- **The arena makes contact there under every B rule** (3 of 4 seeds, uA -9.0), so the
  ground-truth refit is flat: error 11.54-11.57 for all 18 rules. Over the other 11 cells the best
  rules are b_y 2.5 m/s^2 at 0.5 s (error 0.09), and B_aw_react scores 0.13.
- **Finding:** the arena's B lacks Autoware's last-moment stop: obstacle stop and emergency
  braking on a car inside its path. No gap-or-reaction parameter of the yield rule reproduces it.
  Next: identify which module braked, and how hard, from the bags, then give the arena's B that
  safety layer (swb 241).
- **Autoware's last-moment brake is obstacle_stop** (7b34f1c, swb 241; NAS `logs/2026 SSv2 on
  CARLA/2026-11-14 coop B stop layer/`). How it acts:
  - It raises STOP at the onset of A's lane change: A 0.1-0.25 m out of its lane, moving sideways at
    0.23-0.37 m/s.
  - It puts a stop point 5 m before A, adjusted by RSS. At 14/-3 that point is already behind B, so
    the PID's strong stop takes over: a ramp at about -4.8 m/s^3 to -3.4 m/s^2, held to standstill in
    1.9-2.15 s over about 8 m.
  - The command moves 0.2-0.3 s after the factor, and B's speed about 0.8 s after it.
  - There was no AEB and no abort, and MRM stayed normal.

  It is now an arena B safety layer (`arena.STOP_DEFAULTS`, B rule key `stop`, off by default;
  `c3_fit.py FIT_STOP=1`). It does not end the arena's contact at 14/-3. Refit error with the layer
  on, against without:

  | perception | best rule | with the layer | without |
  |---|---|---|---|
  | ground truth | B_r12_b2.5_t1_s | 11.66 | 11.55 |
  | CenterPoint (C3b, S0-rescored) | B_r30_b2.5_t0.5_s | 0.122 | 0.055 |
- **The contact at 14/-3 comes from the arena's A, not its B.**
  - In swb, B passes A's centre while it brakes. Then the agent's A brakes at -7 m/s^2 and stops
    0.23 of a lane into B's lane: a 1 cm miss on ground truth, and a footprint touch in 3 of 4
    CenterPoint runs.
  - In the arena, B brakes earlier and never passes A, so the arena's A keeps cutting in to 0.23 and
    touches.

  So the arena's A and `cutin_agent`'s A differ in how they abort or hold a lane change that B
  answers by braking. That parity gap, not B's model, is what D4 has left at this cell. Next: align
  the arena's A with the agent's abort and hold logic, then refit (swb 241, after the C2b rerun).
- **A's abort parity: the gap was A's observation, not its rule** (b08fb77, swb 241; NAS
  `.../2026-11-14 coop A abort parity/`).
  - The two A rules already matched: the same trigger (lat > 0.05 with B's centre seen ahead), the
    same braking (follow_cruise down to -7 m/s^2), and the "hold" is simply the car stopping.
  - `cutin_agent` delays the other car's position relative to itself. The arena delayed B's absolute
    position and compared it with A's current x, so B appeared a further v_A x 0.19 s back: at 14/-3,
    A saw B draw level about 0.13 s late.
  - `arena.py` now uses the relative position for both cars (`LEGACY_OBS=True` restores the old one).

  Contact at 14/-3 with A_aw_perm:

  | | contact |
  |---|---|
  | arena, before the fix | 10 of 16 (15 with B's stop layer) |
  | arena, after the fix | 6 of 16 (7 with B's stop layer) |
  | Autoware, C3a (ground truth) | 0 of 4 |
  | Autoware, C3b (CenterPoint, scored under S0) | 3 of 4 |
  | Autoware, both together | 3 of 8 |

  Both are an edge cell with a contact share of about 0.4. Refit over the 12 cells:

  | perception | best rule | error | before the fix | without 14/-3 | before, without 14/-3 |
  |---|---|---|---|---|---|
  | ground truth | B_r30_b2.5_t0.5 | 4.70 | 11.55 | 0.032 | 0.091 |
  | CenterPoint | B_r20_b2.5_t0.5 | 1.53 | 0.055 | 0.012 | 0.060 |

  The stop layer still worsens every fit.
- **Decided (2026-11-15).**
  - At an edge cell, contact is fitted as a share against its binomial interval over seeds: inside
    the interval costs nothing. With four seeds, one contact must not dominate the fit.
  - B_aw_react's reaction time moves from 1.0 s to 0.5 s (react within 20-30 m, 2.5 m/s^2), which both
    perceptions now prefer.
  - The double oracle is rerun with it (do5). This is swb 241's work, after the route-curvature
    rescore.
- **The contact-share fit and do5** (c7c12c7, 7f36087, swb 241; NAS `.../2026-11-15 coop do5
  refit/`).
  - **Straight check.** c2b moves its starts 120-161 m back, so the scored route stays on the
    straight. c3 only warns, because it has no room behind its start. At 2/+2 the route still ends
    2.4-3.4 m short of the straight.
  - **Refit,** with outcome shares scored against Autoware's Wilson 95 % interval: ground truth best
    B_r30_b2.5_t0.5 (error 4.65), CenterPoint best B_r20_b2.5_t0.5 (1.51). 14/-3's contact share now
    sits inside both of Autoware's intervals; the remaining error there is the payoff means.
  - **B_AW_REACT now reacts in 0.5 s** (react gap 20 m, b_y 2.5). The arena and the agent take B's
    reaction from the job's react_b, else the rule's react, else 1.0 s. Before this, every double
    oracle B reacted at 1.0 s.
  - **do5** (12 384 runs, 53 min):

    | | do4 | do5 |
    |---|---|---|
    | equilibrium | A1 / B_react | A1 (τ 0.56, buffer 4.0, no prepare) / B_aw |
    | vA, vB, welfare | -0.148, -0.062, -0.210 | -0.141, -0.023, -0.164 |
    | contact, A ahead | 0, 0.5 | 0, 0.42 |
    | A_aw_perm ahead (of 12 contexts) | 5 | 2 |
    | B's loss when B_aw_react is exploited | 0.068 | 0.047 |

    With a B that reacts in 0.5 s, holding (B_aw) becomes B's equilibrium rule. A's aggressive rule
    no longer needs B to yield, and Autoware's A with its 3 s prepare rarely gets ahead
    (-0.19 against B_aw_react).
  - **do5 did not converge.** Its exploitability lower bound is 0.022. B's search reported a gain of
    0.018 every round, but none of its best responses beat B_aw in the table: a winner's curse.
    `Pool.worker` seeded its runs from `hash(tag)`, which Python randomises per process.
- **Decided (2026-11-15):** seeds become stable (crc32 of the tag). Every best response found by
  search is re-evaluated on fresh seeds before it enters the pool. Then rerun as do6 (swb 241).
- **do6 converged** (f4e7672, swb 241; NAS `.../2026-11-15 coop do6/`).
  - Each run is seeded with crc32(context:seed:salt) and gets its own generator, so every rule pair
    sees the same noise.
  - A searched best response enters the pool only if, on 4 fresh shared seeds per context, its gain
    over the incumbent mixture exceeds 2 standard errors. The search had overstated gains by
    0.03-0.06.
  - Converged in round 3, with an exploitability lower bound of 0.
  - Equilibrium: A2 (τ 0.24, a_rear 4.0, buffer 5.1, no prepare) against B3 (react gap 12.6 m, b_y 1.0):
    vA -0.137, vB -0.038, no contact, A ahead in half the contexts. **It is the same class as do4 and
    do5:** an aggressive A without a prepare phase, against a B that lets it in late and gently.
  - CARLA still flips about 1 context in 12 between repeat evaluations.

### N1, N2. Conflict sets and three-car composition (2026-11-14)

Commit 20cef0f. Scripts: `n1_conflict_sets.py`, `arena_n.py` (a third car on the arena's
road, `arena.py` unchanged), `n2_composition.py`. Data: NAS `logs/2026 SSv2 on CARLA/2026-11-14 coop
N1 N2/`.

**N1, the size of a car's conflict set.** Measured per 1 s slot. A car is in the set if its 2D TTC
is under 3 s, its headway under 1 s, or its PET under 1.5 s (definitions from `measures`).

| source | car-slots with a conflict | mean \|K_i\| when > 0 | max | event size mean / max |
|---|---|---|---|---|
| arena three-car runs (576 runs) | 0.3-0.4 % | 1.00 | 1 | 2.0 / 2 |
| SSv2 Town01 NPC benches (npc_10/20/50, npc_shoulder; replayed along the lanelets) | 5.3 % | 1.19 | 4 | 2.40 / 5 |
| literature (survey) | | | | 3.2 / 7 |

- Our scenes are sparser than the literature's: none is a dense merge or junction.
- The ego scenes were not replayed: that needs a recorder for every entity's pose. highD is not on
  the NAS.
- **The coupling in the arena's three-car runs does not come through risk.** No car ever has two
  cars in its conflict set at once, yet the runs differ from the pairs by up to 0.47. A conflict set
  defined only by risk thresholds is too narrow for composition.

**N2, the residual of composed pair terms.** Setup:
- Scene a: C follows B, at a headway of 1 or 2 s.
- Scene b: a lead L ahead of A, at a gap of 15 or 30 m.
- Grid: d0 {2, 6, 10, 14} m × dv {-2, 0, 2} m/s, v_B 12 m/s.
- Profiles: A_aw_perm, and do4's equilibrium A, each against B_aw_react.
- 8 seeds with common random numbers; 3072 runs in 13 min on three CARLAs; no contact.

Results:
- **Composed per measure** (risk max, delay max-plus, braking max), 234 of 288 car-cells (81 %)
  fall within the D2 tolerance, and only 61 of 96 contexts (64 %) for every car. The residuals range
  from -0.32 to +0.47, with bootstrap intervals about 0.0003 wide: these are effects, not noise.
- **Scene a:**
  - When the B-C slot is shorter than A's gap check needs, A drops behind C as well. A loses 0.075 to
    0.12 more than the pairs predict: the slot's length divided by v·H.
  - The A-C pair is a cut-in ahead of C that never happens with three cars, so C's composed payoff is
    0.12 to 0.47 too pessimistic.
- **Scene b:** L slows A before it decides. At d0 2 and 6 with a 15 m gap, A's outcome flips from
  ahead to behind (r_A -0.28 to -0.32). The three-car outcome is the A-B pair at the speed difference
  L imposes, not at the nominal one.
- **A relation-aware composition**, built after seeing the residuals, reaches 265 of 288 (92 %):
  - it drops pair terms whose relation cannot occur given the A-B outcome;
  - it charges A the queue's length when the slot is too short.

  It needs a hold-out before it counts.

**D9 decided: pairs are not enough, and the residual is systematic, so N3 follows.** Two things
shape N3:
- The residual comes from a discrete change in the A-B outcome, so the model takes that outcome (or
  the context L shifts it to) as an input or a class, not only as a smooth correction.
- Its predictors are the target lane's queue (the slot against A's gap need) and A's own lead (gap
  and closing). Neither is in a risk-defined conflict set.

Nothing calls for N = 4 explicit games.

**Open:**
- **Hold-out for the relation-aware composition:** other headways, other v_B, a four-car queue.
- **Scene b's context shift has no composition rule yet.** The A-B pair at dv 0 predicts the outcome
  but not the payoff: -0.27 against -0.48 measured.
- **Payoff levels depend on the scoring window.** The arena's followers never regain their start
  speed, so every loss depends on the window, in `arena.Pair` too. Payoffs scored over different
  windows (20 s against 30 s) do not compare.
- **N1 on SSv2 is a replay of NPC-only scenes**, not a run.

### N3. The residual model, first round (2026-11-14)

Commits 9ed8d89 and 43b45a3: `n3_model.py`, the scoring window and a follower fix. Data: NAS
`logs/2026 SSv2 on CARLA/2026-11-14 coop N3/`.

- **One scoring window.** Route progress is scored over a fixed [0, H], H = 20 s from the run's
  start (`measures.lost_progress`). Risk, comfort and contact still count over the whole run. It is
  used by `arena.Pair`, `arena_n` and `c2b.score`. Design ① is updated.
- **Follower bug, fixed.** Followers never regained their speed: IDM's desired speed was the car's
  own start speed, so behind a 12 m/s leader a follower ran at 11.4 m/s with a growing gap.
  `cutin_game.follow_cruise` (IDM's following term capped by cruise) settles at the leader's speed.
  It is used by `arena.Pair`, `arena_n` and `cutin_agent`; `arena.LEGACY_FOLLOW` keeps the old one.
  D2 still holds: 44 of 48 (92 %), outcomes 24 of 24. The four misses are uB at 10/0, because C2b's
  swb runs were recorded with the old agent; C2b with the fixed agent is not re-run yet. C1 still
  uses the old follower.
- **The model, frozen before any hold-out was scored:**
  - v2 is extended to a queue of any length: A, dropping back, pays for each slot too short for its
    gap check. Only adjacent followers enter as pair terms, and delay propagates down the queue.
  - Scene b uses the context L imposes: the A-B pair term is read from a pair table at the gap and
    speed L leaves A.
  - A 3-parameter linear residual per outcome class. C and D share one class, so the model is
    permutation-invariant over the followers.
- **Held-out results:**

  | set | contexts, v2 | contexts, N3 | car-cells, N3 |
  |---|---|---|---|
  | fit (in-sample) | 77/96 | 93/96 | 283/288 |
  | C's headway 1.5 and 3 s | 27/48 | 35/48 | 127/144 |
  | v_B 8 and 16 m/s | 150/192 | 156/192 | 529/576 |
  | L's gap 22 m | 18/24 | 20/24 | 65/72 |
  | four-car queue | 22/48 | 36/48 | 180/192 |
  | **held-out, pooled** | **217/312 (70 %)** | **247/312 (79 %)** | **901/984 (92 %)** |

  v2 predicts B, C, D and L at 90-100 %. It fails for A whenever A drops back.
- **Causes of the misses**, found on the hold-out:
  - **C's headway 3 s:** A drops back at 5.3-5.6 m/s, and at that speed its own gap check against C
    needs about 48 m. The model tested the slot at A's start speed. Fix (physical): test it at A's
    speed when it drops.
  - **The four-car queue:** the residual was fitted with one skipped slot. Fix (physical): a residual
    per skipped slot.
  - **v_B 16 m/s with a 15 m lead gap:** inside L's following gap, a regime the fit never had. At
    dv −2 the shift should not apply, since L pulls away; at dv +2, A's loss reaches about 0.7
    beyond the fit's range.

  With all three fixes, refitted on the fit set only: 95 % of cells and 87 % of contexts. That is
  post hoc, not a held-out score.
- **D9 after the first round:** not met on held-out contexts (79 %), but the misses are systematic
  and named. **Decided: one more short N3 round, then N4.**
  - Adopt the three fixes.
  - Fit at headways 1-3 s, L's gap 1-2.5 s (in time) and v_B 8/12/16 m/s.
  - Hold out v_B 10 and 14 m/s, headway 2.5 s, L's gap 1.75 s and a five-car queue.

  If scene b still misses, its contexts inside L's following gap stay explicit three-car games,
  and N4 starts anyway.

### N3. Second round, and D9 (2026-11-14)

The model was frozen in d17fdc3 before any run of this round. Changes from round 1:
- A's slot is tested at A's speed when it drops.
- The queue residual is applied per skipped slot.
- L shifts the A-B context only when A closes on it.
- The lead feature is A's initial braking toward L. L's gap is given in time.

The fit set has 16 128 runs: scene a at headways of 1, 2 and 3 s; scene b at L's gap of 1-2.5 s;
v_B 8, 12 and 16 m/s. Held out: v_B 10 and 14 m/s, a headway of 2.5 s, L's gap of 1.75 s, and a
five-car queue. Data is in `/mnt/p5plus/aeon/coop/n3b/` (summary `runs/n3_summary_r2.txt`).

| held-out set | contexts within tolerance | car-cells within tolerance |
|---|---|---|
| v_B 10 and 14 m/s | 177/192 (92 %) | 551/576 (96 %) |
| headway 2.5 s | 24/24 (100 %) | 72/72 (100 %) |
| L's gap 1.75 s | 16/24 (67 %) | 60/72 (83 %) |
| five-car queue | 18/48 (38 %) | 204/240 (85 %) |
| **pooled** | **235/288 (82 %)** | **887/960 (92 %)** |

For comparison, v2 without the model scores 64 % of contexts and 86 % of cells.

- **The five-car queue: the composition overcharges A,** by up to +0.2 when A skips three or four
  slots. Delay composed max-plus down the queue keeps growing, while the loss scored in the fixed
  20 s window saturates. The last follower, E, is off by only -0.03. Fix: cap A's composed delay at
  what the window can hold.
- **L's gap 1.75 s, dv +2: the shifted lead regime again.** Residuals reach +0.13 to -0.23 for A
  and B.

**D9 decided (2026-11-14).** Pair terms composed by the N3 model hold to D9's 90 % of contexts for
queues of up to four cars (headway 1-3 s) across v_B 8-16 m/s. Two regimes stay explicit games:
- a lead L that A closes on (dv > 0) within about 2 s of gap;
- queues of five or more cars until the window cap is in. The window cap is a one-line change,
  checked against this round's data before N4 relies on it.

N grows by composition elsewhere. **Next: N4**, the far field: a long merge or ring of 10-50 cars,
the near field exact, the rest as density and mean speed.

### N3 third round (the window cap) and N4, the far field (2026-11-15)

Commits 5c73c33 (cap, six-car scene) and c203016 (N4). Data: NAS `logs/2026 SSv2 on CARLA/
2026-11-14 coop N4 far field/` (with `n3_round3_window_cap/`) and `.../coop N3 round 2/`.

**The window cap.** A's predicted loss is capped at what the 20 s window can hold: A waits at its
drop speed until H. On the five-car queue it lands within 0.005-0.02 of the measured loss.
Re-scored post hoc on round 2:

| | five-car contexts | A's cells | pooled held-out contexts |
|---|---|---|---|
| before the cap | 38 % | 29/48 | 82 % |
| after the cap | 62 % | 46/48 | 86 % |

**A six-car hold-out with the cap frozen first** (6144 runs): 50 % of contexts, 86 % of cells.
- A is now right in 48 of 48 cells (23 of 48 without the cap). At a headway of 2 s every context
  passes; at 1 s none does.
- All the misses are followers. E and F are off by -0.028 and -0.040 in every context at headway
  1 s: the queue sits at IDM's desired gap with no slack, and B's braking amplifies down it.
  Max-plus composition passes delay on but never amplifies it.

**D9, refined (2026-11-15).** Composition with the cap holds for queues of any length that have
slack (headway ≥ 2 s). Tight queues (headway ≈ 1 s) of five or more cars need a follower term for
depth and amplification. Until then they stay explicit games.

**N4, the far field.** A merges into a stream of N-1 cars on a straight road. Headways are drawn
from a shifted exponential with mean 3 or 4 s; 16 draws, shared across N; 1280 group runs.
- Model: the near field (A, B, three followers) uses the N3 composition with the cap. Beyond that
  the stream is a density and a speed: each slot fits with p = P(h ≥ h_need), including the chance
  that A never gets in.
- A merge was chosen over a ring, because a ring's shock wave would come back to A's own leader.

| N | 10 | 20 | 30 | 50 |
|---|---|---|---|---|
| mean-field mean abs. error | 0.053 | 0.050 | 0.048 | 0.055 |
| composition on the realized gaps | 0.040 | 0.037 | 0.036 | 0.048 |

The mean field is within D2's tolerance in 15 of 16 cells at every N, but within the 0.02 floor in
only 4-6.

- **N does not matter past about 10 cars in a merge.** Explicit U_A does not depend on N. A never got
  past the sixth follower, and in 66 of 256 runs it never got in.
- **The density term costs only about 0.014** against the realized gaps. The remaining error is the
  near field's bias: about 0.03 optimistic for long waits.
- **Not tested:** N < 8, rings, and congestion. A's externality on the stream (about 0.001 per
  follower) is at the noise floor.

**Next on the N path:**
- the follower depth term;
- the near-field bias for long waits;
- a congested stream, which is also the zipper's open case (C7);
- N5, the swb check on a found merge site, waits for E7.

### N3 round 4: the follower depth term (2026-11-15)

Commit ea82020, frozen before the hold-out. Data: NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop N3
depth term/`.

- **Why tight queues missed: a lag of each follower's own, not amplification.** At headway 1 s,
  every follower ends about 0.26 s behind its leader, whatever B does. Its pair run alone shows the
  same increment; at 1.5 s and above both are 0. At zero slack the follower brakes on observer
  noise and, capped by cruise, never closes the gap again. IDM's string-stability gain with the
  arena's lags is 1.000 at every headway, so the queue is string-stable.
- **The term:** each follower adds its own measured lag on top of its leader's delay (max-plus),
  with one scale c = 1.10 fitted on fit2.

**Seven-car hold-out:**

| | contexts | cells |
|---|---|---|
| N3 model | 44/48 (92 %) | 99 % |
| v2 + depth term | 47/48 | |
| v1 | 10/48 | |
| round 3's six-car hold-out | 50 % | |

Pooled over the hold-outs of rounds 2-4: 92 % of contexts, 97 % of cells.

**D9, met (2026-11-15).** Composition (pair terms, window cap, depth term) meets the 90 % bar for
queues of up to seven cars at headways of 1-1.5 s, and for any queue with slack. Explicit games
remain only for a lead that A closes on within about 2 s.

### S1. The cut-out, kinematic (2026-11-14)

`scripts/coop/cutout_game.py` (01bc891, swb 139).
- **Scene:** three cars. B follows L in one lane. L changes lanes away (cosine, 3 or 5 s) and reveals
  an obstacle O ahead, stopped or at half speed.
- **L's rule:** cut out when its time to O reaches T_L ∈ {6, 4.5, 3.5, 2.5} s.
- **B's rule:** IDM with a time gap of 0.8, 1.5 or 2.5 s, with or without look-through. Without it,
  B sees O only once L's body has left the lane, plus 0.19 s of perception and 1.0 s of reaction.
- **Context grid:** h0, B's headway to L at the start (0.6-3 s); v = 8-20 m/s; O stopped or slow;
  the left lane at the same speed or slower. 224 contexts per lane-change time.
- **Payoffs:** S0's utility. B's risk counts O and L; L's counts O and B.

Data: NAS `logs/2026 SSv2 on CARLA/2026-11-14 coop S1 cutout kinematic/`.

- **The danger is perception, not the gap.** Without look-through and with a stopped O, no gap up to
  2.5 s keeps B's risk under 0.2 against a late cut-out at v ≥ 12 m/s:
  - 12 m/s, h0 1.5 s: risk 0.60 / 0.51 / 0.34 at gaps of 0.8 / 1.5 / 2.5 s;
  - 20 m/s with the 5 s lane change: contact at a 0.8 s gap.

  With look-through, a 2.5 s gap is safe everywhere (h0 ≥ 0.9 s) whatever L does. Against a slow O,
  0.8-1.5 s is safe either way.
- **No equilibrium has contact** (0 of 448 contexts). There are 18 off-equilibrium contacts, all a late
  cut-out against a 0.8 s gap at high speed without look-through.
- **L does not shift risk to B.** L's equilibrium timing costs B at most 0.018. L cuts out early (6 s)
  when the left lane is as fast. It delays to 4.5 or 3.5 s when the left lane is slower, and never
  takes the latest timing: its own risk from O there (0.24-0.33) outweighs the progress it saves. A
  risk-dumping L needs a type that bears no risk from O, or ignores it: a preference type, as in C5
  and C6.
- **For Autoware:** the question is whether its perception looks through a lead car (it does not see
  occluded objects). E3 and C3 should add a cut-out cell on Town03's west straight.
- **Caveats:** coarse grids (gaps beyond 2.5 s untested); O is a single car; IDM's comfortable braking
  is capped at 2 m/s² unless the closing needs more.

**Revised and checked in the arena (aa06d45, swb 139).** Data: NAS `.../2026-11-14 coop S1 cutout
arena/`; the kinematic re-run is in its `kinematic/`; the note is the kinematic folder's
REVISION.md.

cutout_game's B now follows with the fixed follower (as `follow_cruise`), and progress is scored
over exactly H. The conclusions hold:
- without look-through and with a stopped O, no gap up to 2.5 s is safe at 12 m/s and above;
- no equilibrium has contact;
- L does not shift risk to B (at most 0.027 with the 3 s lane change, 0.047 with the 5 s one).

Off-equilibrium contacts rose from 0 and 18 to 12 and 60 rule pairs.

In the arena: look-through off, 12 and 20 m/s, O stopped and slow, 3 values of h0, 5 seeds.

| check | 12 m/s | 20 m/s |
|---|---|---|
| payoffs inside the arena's interval | 78 % (median gap 0.009) | 60 % |
| outcomes agree | 90 % | 90 % |
| B's risk band in the worst equilibrium agrees | 12 of 12 cells (both speeds) | |
| contact in equilibrium | none on either side | |

- **L's equilibrium timing** differs only in cells where the model has L tied between 6 and 4.5 s.
- **The arena makes the latest cut-out (2.5 s) worse:** contact at 12 m/s, h0 1.5 s, gap 0.8 s,
  where the model has risk 0.70. That rule pair is off equilibrium.
- **D2 for the cut-out, decided (2026-11-14):** as for the crossing, accepted for outcomes and the
  equilibrium risk band. Payoff levels come from the arena (78 % at 12 m/s, 60 % at 20 m/s). The
  kinematic model understates the latest cut-out's harm, so its off-equilibrium risk is a lower
  bound.

**On the real stack, with CenterPoint** (swb 118; NAS `.../2026-11-15 coop S1 cutout
autoware/`). Town03's west straight; L a light agent, O a stopped car, B the Autoware ego; 20 runs.
- **No contact; B always stops 4.9 m short of O.**
- **Autoware keeps its own gap of 3.3-4.6 s,** so B's headway h0 cannot be set from outside.
  Autoware sits at a gap the kinematic model calls safe even without look-through.
- **CenterPoint is limited to about 45-50 m.** With T_L ≤ 3.5 s it sees O 1.5 s after the cut-out
  starts, about 1 s before L clears the lane. The lidar sees part of O around L, so B's perception is
  between "look-through off" and "on".
- **At 6 m/s, uB ≈ -0.44,** against the kinematic model's -0.39 at a 2.5 s gap (look-through on and
  off are equal at that h0). 8 m/s is clean. 12 m/s reaches speed, but the goal stop confounds it:
  the straight is too short.
- **Reading:** Autoware's long following gap is its defence in the cut-out. The risky regime (short
  gap, high speed) needs a longer straight than Town03 has.

### S2. The unsignalled crossing: kinematic and arena (2026-11-14)

The crossing as design ②'s passage game, by swb 139. The kinematic model is
`scripts/coop/crossing_game.py` (c48ce7c, corrected in 3163cce); the arena is
`crossing_arena.py` (3163cce).
- **Geometry:** two straight roads at right angles, a 3.5 x 3.5 m conflict box Z, Model 3 footprints.
- **Context:** Δt = t_A^Z − t_B^Z in −4..+4 s, the speed v = 4-14 m/s, the right of way r = none, A
  or B.
- **A, the claimant:** gap acceptance with τ ∈ {−∞, 0.5, 1.5, 3, +∞}, patience 2 or 5 s, with or
  without going on a seen yield (17 rules).
- **B, the holder:** hold; react (sequentially rational); yield with a band of 1, 2 or 4 s and
  patience 2 or 5 s (8 rules).
- **Timing:** 0.19 s of perception, then 0.5 s to decide; the safety layer reacts after 1.0 s.
- **Payoffs:** S0's general utility, plus a fault cost for the car without right of way when it
  forces the other.

Data on the NAS: `logs/2026 SSv2 on CARLA/2026-11-14 coop S2 crossing kinematic/` (the first
results are kept in `v1-superseded/`) and `.../2026-11-14 coop S2 crossing arena/`.

**The arena corrected the kinematic model.** Its first runs exposed two faults in the rules:
- a rule compared the other car's 0.69 s old position with its own current one;
- A's "B clears before I arrive" test used A's current speed.

Both are fixed in 3163cce and the kinematic map was re-run. The first results (chicken for
|Δt| ≤ 0.5 s, a loss up to 0.79, PET ≥ 1.9 s, a right of way that overrode first-come by up to
3 s) are superseded.

Kinematic results, corrected:
- **Chicken only for |Δt| ≤ 0.2 s, at every speed.** Outside that band the nearer car passes first
  in every equilibrium.
- **The tie itself is expensive, whoever goes.** In the band the worst equilibrium costs at most
  0.10 more than the optimum. But near a tie even the optimum costs about 2: one car brakes hard and
  waits, in every pairing of rules. Away from the band the cost is 0.03-0.2. So a tie calls for
  avoiding arriving tied (adapting speed on the approach), not for a better choice at the line.
- **No equilibrium has contact or a war of attrition.** The smallest equilibrium PET is 0.9 s,
  below the 1.5 s reference, now that A judges gaps on true positions.
- **Right of way at a tie:** a fault cost of 0.5-1 selects one equilibrium in about half the cells
  of the band; no swept value up to 8 selects it in the rest, where patience decides.
- **Enforcement outside a tie is a deadweight cost.** At fault 2 with r = A, B still passes first at
  Δt = +1..+3 s and pays the fault: C rises by about 2 and the order does not change. A priority
  rule belongs in a conflict window (|Δt| ≲ 1 s), for this reason.

**In the arena** (K = 8 crossings at grade on a generated world; the spec's Model 3 with acb's
controls and lags; `crossing_game.Policy` on `arena.Observer`'s perception; 6 A rules × 5 B rules;
2 × 1350 runs at 10 and 14 m/s, 5 seeds):
- **D2 is not met.**

  | | v = 10 m/s | v = 14 m/s |
  |---|---|---|
  | payoffs inside the arena's intervals | 47 % | 53 % |
  | outcomes agree | 78 % | 72 % |
  | classes agree | 5 of 9 cells | 6 of 9 cells |

- **Away from a tie (|Δt| ≥ 0.5 s) the models agree on who goes first:** 11 of 12 cells. The arena's
  payoffs are 0.02-0.09 lower, because the brake lag makes the yield later and harder and the restart
  from rest is slower.
- **At |Δt| ≤ 0.2 s the arena gives a war of attrition** (both cars stop) where the kinematic model
  gives chicken. Brake lag, a slow start from rest and noisy 10 Hz perception make a mutual stop the
  stable outcome. The arena's answer is the more physical one, so the near-tie class stays uncertain
  until the models agree.
- **Arena-only contact at near-ties**, in 4-5 rule pairs, all a go-on-seen-yield A against a holding
  B. Noisy speed estimates pass the gap test while both cars brake and restart. Going on a seen
  yield is unsafe under perception noise at a tie.
- **Next, to meet D2:**
  - give the kinematic model the car's brake dead time, lag and acceleration from rest;
  - define "both stopped" over a band of speed;
  - compare away from the rules' thresholds (Δt = +1 s is exactly one rule's band).

**The crossing's D2, closed (319e667, swb 139).** crossing_game was given:
- the car's brake dead time and lag, and its acceleration from rest;
- "both stopped" defined over a band of speed;
- comparison cells away from the rules' thresholds.

Data: NAS `logs/2026 SSv2 on CARLA/2026-11-14 coop S2 crossing D2/`.

| criterion | before | after |
|---|---|---|
| outcomes agree (rule pairs) | 72-78 % | 95-98 % |
| classes agree (of 9 cells) | 5-6 | 7-8 |
| payoffs inside the arena's interval | 47-53 % | 41-51 % |
| payoffs within 0.1 | | 94-95 % (median gap 0.03) |

- **The near-tie is a war of attrition in the kinematic model too.** It is W at |Δt| ≤ 0.2 s at
  every speed, with or without the car's lags. The "both stopped" band changed the label, not the
  car model. **The kinematic map's chicken band above therefore reads W:** at a near-tie both cars
  stop before one goes.
- **Why the payoff share stays low:** the arena is near-deterministic (sd ≈ 0.002), so its interval
  is the ±0.02 floor. The misses are small systematic offsets: lost progress +0.02, because CARLA's
  car regains speed more slowly; comfort +0.015, from peak braking of 3.2-3.5 against 2.8 m/s^2.
- **D2 for the crossing, decided (2026-11-14): accepted for outcomes and classes.** The kinematic
  model is trusted for who goes first and for the class map. Payoff levels come from the arena,
  with the kinematic model's known offsets (+0.02 progress, +0.015 comfort) where needed. The
  payoff criterion is not relaxed in general: here it fails on an interval floor against a
  deterministic arena, not on behaviour.

### S3. The merge at a lane drop, kinematic (2026-11-15)

`scripts/coop/merge_game.py` (dc0be47, swb 139). Data: NAS `logs/2026 SSv2 on CARLA/2026-11-15
coop S3 lane drop kinematic/`.

**Scene.** A's lane ends 40, 80 or 160 m ahead, and A must merge into B's lane. The context is
cutin_game's d0 (-10..30 m) and dv (-3..3 m/s), plus that distance.

**Rules.**
- A has 16 rules. Each is gap acceptance with τ 0.6 or 1.2 s, merging early or zipper-style
  (within 6 s of the end), with or without going on a seen yield. At the last point where a 3 s lane
  change still fits, A either:
  - drops: falls behind B, with a forced stop at the end if needed; or
  - forces: changes lanes anyway.
- B has 4 rules: hold, react, or courtesy (within 20 m and 10 s of the end at 2 m/s^2, or within
  40 m at 3 m/s^2).
- Perception is 0.69 s old and predicted to now. Followers use `follow_cruise`.
- Payoffs are S0's utility over H. A's front passing the end while A is less than halfway across
  counts as contact.

**Findings.**
- **The deadline turns the merge into a coordination game near the end.** The band of contexts
  where both orders are equilibria widens as the end nears, measured in seconds of travel. The worst
  equilibrium's welfare loss is about 1.0 at 40 m from the end at 12 m/s, and at 80 m at 20 m/s.
- **It is not strict chicken.** A forcing against a holding B mostly costs B a hard brake; contact
  follows only in some cells. Mutual courtesy (A drops while B is courteous) is A's worst outcome: a
  forced stop at the end.
- **Equilibria:** forced stops in 1-2 cells, and no contact. Off equilibrium, contact comes from
  forcing against a hold at short ends.
- **The zipper recovers nothing with two cars** (mean +0.001). It needs a queue: a follower C
  behind B, which is N2's scene a. What removes the bad equilibrium is a convention on who yields at
  the end (C7).

**In the arena** (d7cc573, swb 139; NAS `.../2026-11-15 coop S3 lane drop arena/`). 1440 runs at
12 m/s, lane ends 40 and 80 m away, 16 contexts. `arena.xodr_lane_drop` keeps the lane at full width
to the model's end, then tapers over 30 m. Both models share `merge_game.Rules`. Cars that leave the
mesh or touch are replaced, because CARLA leaves them stuck.

| check | result |
|---|---|
| classes agree | 14 of 16 contexts |
| outcomes agree | 93 % |
| payoffs inside the arena's interval | 72 % (median gap 0.007), the best of the three arenas |

- **The coordination band at 40 m holds in the arena.** The two class misses sit at the band's edge
  at 80 m.
- **The outcome misses are all at the deadline.** In the arena, courtesy against a dropping A ends
  in more forced stops, and forcing from slightly ahead and slower becomes contact (steering lag).
- **D2 for the lane drop, decided (2026-11-15):** accepted for classes and outcomes, as for the
  crossing and the cut-out, with payoff levels from the arena. Near the deadline the arena is
  harsher, so the kinematic model's forced stops and contacts there are lower bounds.

**The zipper with a queue, kinematic** (1a54980, swb 139; NAS `.../2026-11-15 coop S3 zipper
queue/`). Conventions compared by the share of the welfare gap (worst against best equilibrium with
no convention) each recovers:

| convention | gap recovered | forced stops | last car |
|---|---|---|---|
| early merge | most: +0.78 (3 cars), +0.89 (2 cars), +0.49 (v_B 8) | most (16 of 40) | slowest |
| yield at the end | about 0.45 | almost none | about 1 s sooner |
| zipper, with A's best reply | about 0.45 | almost none | about 1 s sooner |
| zipper with A fixed late | the worst; it is A's best reply in only 33 % of contexts | | |

- At v_B 8 m/s neither yield-at-end nor the zipper recovers anything.
- No convention has contact at 12 m/s. At 8 m/s the fixed-late zipper has one.
- Caveat: the queue is short and free-flowing, so the congestion benefit claimed for the zipper (a
  shorter queue upstream) is not measured.

Reading: the conventions trade welfare against forced stops. Early merge settles who goes first at
the price of stopping cars. Yield-at-end and the zipper recover half the gap and keep traffic moving,
but only if A plays its best reply. A dense, congested queue is the case still to test.

**In the arena** (5124573, swb 139; 1440 runs, three cars at v_B 12 m/s). The arena matches the
kinematic model:
- outcomes agree in 93 %, payoffs in 76 %, and the cheapest convention agrees in 13 of 16 contexts;
- early merge has the lowest mean cost (0.58) but the most forced stops (10 of 16 contexts);
- yield-at-end and the zipper with A's best reply recover about 0.73 of the gap and get the last car
  through about 1.1 s sooner;
- the fixed zipper is worst;
- no convention has contact.

**For C7:** a convention that leaves A its best reply (yield-at-end, or a zipper that A times
itself) recovers most of the gap without forced stops. Early merge buys the lowest cost with stops,
and a rigid zipper buys nothing. Open: a congested queue.

### E3 and D3. Autoware's knobs (2026-11-15)

Run by swb 118 on C3a (Autoware on ground-truth objects) at 9ed8d89, on the Town03 west straight
at 6 m/s. Cells (6, 0), (10, 0) and (14, -3), both A rules, 13 knob settings, one fresh stack per
setting; stock and yield were also run at three seeds. Data: NAS `logs/2026 SSv2 on CARLA/
2026-11-14 coop E3 autoware knobs/`.

- **Stock Autoware is one fixed strategy for A's order and for when B starts braking.** No knob
  changed A's order in a robust cell, and none moved B's braking onset: 2.1-2.7 s under A_eq,
  5.1-5.9 s under A_aw_perm.
- **The cruise-gap knobs move B's payoff after a cut-in, well beyond noise.** At (10, 0), with B
  behind A:

  | setting | uB A_aw_perm | uB A_eq | Δ against stock |
  |---|---|---|---|
  | stock (3 seeds) | -0.125..-0.134 | -0.123..-0.139 | 0 (spread 0.016) |
  | yield (idling 4 s, RSS -0.5, margin 10 m, lateral margin 2 m; 3 seeds) | -0.348..-0.367 | -0.372..-0.387 | -0.24 |
  | `idling_time` 4 s | -0.255 | -0.268 | -0.13 |
  | `min_ego_accel_for_rss` -0.5 | -0.261 | -0.265 | -0.13 |
  | `safe_distance_margin` 10 m | -0.171 | -0.183 | -0.04 |
  | hold (idling 1, RSS -2.5, margin 2, ...) | -0.112 | -0.095 | +0.02..+0.04 |

  Yielding is cheap to switch on with any one of the three knobs. Holding is not: stock is already
  near the tight end.
- **Where the knobs live:** all are in obstacle_cruise's parameters, under
  `.../lane_driving/motion_planning/motion_velocity_planner/obstacle_cruise.param.yaml`. They are
  **load-time only**; just the PID planner's gains and `min_accel_during_cruise` update at run time.
  So a knob strategy is chosen per stack start.
- **What does not act here:**
  - The lane change's rear-car assumptions and RTC never act, because B does not change lanes and
    RTC is off in every module.
  - (6, 0) moves under no knob.
  - (14, -3) is decided by the seed: stock alone flips A's order and touch against no touch, which
    is the 1 cm edge of the C3a rerun.
- **Cut-out:** acb's ground-truth objects see through a lead car (within 100 m); CenterPoint sees
  only what the lidar hits. A real-stack cut-out must run on CenterPoint.
- **Found on the way:** `c3_run.sh` with several cells in CELLS ran only the first. The fix is in
  swb 118's report, not pushed yet.

**D3 decided (2026-11-15).**
- Stock Autoware is one fixed strategy: A's best reply is to stock.
- Knob settings are strategies too. At least two (stock and yield) move B's outcome measurably, so
  D3's condition holds and the overlay switch (042) is not needed.
- Caveats: the knobs move how far B yields back, not who goes first; A's best reply against yield
  is not computed yet. Next: add B_aw_yield, fitted on the yield setting, to the double oracle's B
  pool.

### C2b rerun with the fixed agent (2026-11-15)

swb 241 ran it at 7b34f1c: 24 swb runs on Town03 plus the arena's cells. Data: NAS `logs/2026 SSv2
on CARLA/2026-11-14 coop C2b rerun/`.
- **Outcomes agree 24 of 24.** D2 as c2b.py scores today is 14 of 48 payoffs (29 %).
- **The cause is the road.** The new logs carry world x, y and yaw, so c2b.poses scores on Town03's
  real road. Within the 20 s window both cars reach the curve at the east straight's end: B's
  heading turns from -89 to -128 degrees by t = 14 s. The lateral comfort term then charges the
  curve (latB 2.0, latA about 3 m/s^2), so swb's uB is -0.043 in every run against the arena's
  -0.005.
- **Scored in the lane frame** (the same logs without x/y/yaw, as runs 1-2 were logged), D2 is 46 of
  48 (96 %). The two misses are uB at 10/0 with A_eq/B_eq.
- **Decided (2026-11-15):** lateral comfort counts only the acceleration a car adds to its route,
  a_lat - v^2 κ(s), with κ the route's curvature at the car (design ①). Agents log κ at their
  position; the arena's and the kinematic roads are straight (κ = 0). Until that is in, C2b's D2
  stands at 96 % in the lane frame. Implementation: swb 241, after the A abort parity.
- **Route curvature implemented** (352e1ff, swb 241; NAS `.../2026-11-15 coop route curvature/`).
  - `route.py` computes a lane's curvature from CARLA's waypoints, smoothed over 5 m. `cutin_agent`
    logs `kappa` and `okappa`.
  - `measures.push_poses(kappa=)` subtracts v^2 κ, averaged over the same 0.5 s window as the lateral
    slope (subtracting it per instant overshot at sharp lane shifts and gave 79 %).
  - Old logs get κ from the map (`c2b.py --rescore --kappa-port`).
  - **World-frame D2 is 42 of 48 (88 %)**, against 29 % before and 96 % in the lane frame. Outcomes
    agree in 24 of 24.
  - The remaining misses are real geometry. At 2/+2, A at 10.4 m/s reaches Town03's lane shift
    (x 145-160) inside the window, and its pure pursuit cuts the shift (lateral 2.5 m/s^2), effort
    the straight arena road does not have. The two 10/0 uB misses are the pair the lane frame also
    misses.
- **Decided (2026-11-15): C2b's D2 is met with the misses explained.** From now on, swb runs used
  for D2 keep the scored window on road geometry the arena also has (a straight, or the arena gets
  that geometry). Scenario generators (c2b.scenario, c3) check that the route over v·H stays on the
  straight.

### S4. The narrow passage, kinematic (2026-11-15)

The first oppose relation (92bc8fb, swb 139; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop S4
narrow passage kinematic/`). A two-way road narrows to one lane over a length W_n, and A and B
approach from opposite ends.

**Findings.**
- **A war of attrition within |Δt| ≤ 0.2 s,** at every W_n and speed: both cars stop, then one goes,
  in either order. This is the crossing's near-tie again.
- **Chicken only where the passage is long relative to speed:** 30 m at 6 m/s, Δt -2..+0.5 s.
- **"The obstacle side yields" selects B first at a fault cost of 0.5-1 when A is slightly ahead,
  but never inside the war band.** S2's fault model excuses a car that came to rest at its line, so
  once both have stopped the rule no longer binds. A right of way meant to break a tie must also
  bind after a stop. That is a design point for C7's conventions.
- **No deadlock** (patience is finite), and no contact or restart crash in any equilibrium.
- **But over all rule pairs, 38 % of the runs where both waited end in a restart crash,** 77 % when
  both have the same patience. Mismatched or identical waiting rules are the hazard: a tie-break that
  survives a mutual stop is a safety need, not only an efficiency one.

**In the arena** (4122b1a, swb 139; NAS `.../2026-11-15 coop S4 narrow passage arena/`; 3024 runs,
W_n 30 and 6 m).

The qualitative map holds:
- a war of attrition at the tie on both sides, wider in the arena (±0.3 s against ±0.2 s);
- "the obstacle side yields" gives B first at Δt -0.7 s in all four cells (the model in three, with
  chicken in the fourth);
- the share of both-waited runs ending in a restart crash is 38-50 % (the model's 38 %).

The numbers do not:
- outcomes agree in 74-75 %, classes in 6-10 of 14 cells, payoffs in only 21-28 %. Pure pursuit
  tracks with errors of up to 0.5 m on close passes;
- the arena has 20 deadlocks (30 m, 6 m/s, 2 s patience) the model lacks. The likely cause, not
  verified, is creep keeping a car from counting as stopped, so its patience never starts.

**D2 for S4, decided (2026-11-15): not met; the classes are trusted qualitatively, the payoffs are
not.** Before S4's numbers are used:
- patience starts from a speed band, as "both stopped" does since the crossing's D2;
- the arena's lateral tracking on close passes is fixed or measured;
- the comparison is rerun (swb 139).

**S4's D2, second round** (c381962, swb 139; NAS `.../2026-11-15 coop S4 narrow passage D2/`).
**Not met.**
- **The 20 "deadlocks" were collisions, not creep:** 0-3 mm of clearance, an impulse of about
  12 m/s^2, then both cars stuck. Contact is now detected to within 1 cm; no deadlocks remain, and
  patience is unchanged.
- **The swerve around the narrowing was undrivable:** a kinked 10 m taper, 17 m/s^2 lateral at
  10 m/s. It is now a cosine over 25 m in both the model and the xodr. **This revises the S4
  kinematic map:** more chicken on A's side, and "the obstacle side yields" now needs a fault cost
  of 1.
- A look-ahead of max(3 m, 0.4 v) halves the tracking error, to 0.28-0.31 m from 0.60-0.71 m.

| check | first round | second round |
|---|---|---|
| payoffs inside the interval | 21-28 % | 35-38 % |
| outcomes agree | 74-75 % | 73-78 % |
| classes agree (of 14) | 6-10 | 8-10 |

Not yet diagnosed: the arena's war band is wider (±0.3 s), and the arena has fewer contacts than
the model (146 against 249).

**Decision (2026-11-15):** S4 stays qualitative: the war at the tie, the obstacle rule's need for a
fault, and the restart-crash hazard. Its numbers are not used. The contact deficit is backlogged
(likely the brake lag stopping the arena cars earlier); it is not worth a third round before
C7's tie-breaks, which address the same tie.

### S6. Overtaking via the oncoming lane, kinematic (2026-11-15)

An oppose plus a merge (aa3af10, swb 139; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop S6
overtaking kinematic/`). A follows a slow lead L on a two-lane two-way road and B comes the other
way. A can go, wait, or abort and drop back; B can hold, brake, or give way.

- **Chicken with B appears in a 1.5-2.5 s band of time-to-meet, just short of the free-pass
  region:** 6-7 s at a 10 m gap to L, 8-10 s at 25 m. It exists only because B can give way;
  without B's courtesy the band is a plain wait.
- **The abort rule makes nothing safer.**
  - Gap tests that judge B's speed correctly are contact-free anyway.
  - With A misjudging B's speed by 20-30 %, the abort saves 1 of 6-11 crashes. The error shows only
    after A is past L's front, where it can no longer drop back.
  - Safety comes from the margin in A's gap test: 0-2 crashes at a 1.5 s margin, none at 3 s.
- **B's courtesy is double-edged.** It improves the best equilibrium (cost 0.465 against 0.471) but
  worsens the worst (0.538 against 0.485). It cuts blind-go contact from 43 to 28-38 of 108 pairs.
- **Reading:** in an overtake the only protection is margin and perception. An abort that triggers
  after the commit point is too late. A strategy layer should judge the oncoming speed
  conservatively before committing, not count on aborting.

**In the arena** (190cfd5, swb 139; NAS `.../2026-11-15 coop S6 overtaking arena/`; 3456 runs).
It matches the model on D2's yardstick at S3's level:

| check | result |
|---|---|
| payoffs inside the interval | 74 % |
| outcomes agree | 96 % |
| classes agree | 43 of 48 |
| chicken band reproduced | 13 of the model's 16 C cells |

The misses are only at the band's edges.

**The abort fails harder in the arena.**
- At a 30 % misjudgement of B's speed, A first sees the danger a median 1.6 m past L's front, after
  the commit point in 67 % of cases. The model has -4.4 m and 44 %.
- All 10 arena aborts still crashed.
- Adding the abort to A's 0.5 s-margin rule changes nothing: contacts go from 33 to 33 (model: 45
  to 42).
- The car's lags move the commit point later. Safety still comes only from the margin: a 1.5 s
  margin leaves 6 contacts, all against a B that speeds up after A has started back.

**D2 for S6, decided (2026-11-15):** accepted for outcomes (96 %) and classes. Payoff levels come
from the arena.

### S5. The unprotected left turn in the arena (2026-11-15)

swb 139 (7fa16fb, 65cfc12; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop S5 left turn arena/`;
378 runs).

**U1's model was rebuilt** as `left_turn_queue.py`, with S2's fixes: true arc geometry, predicted
perception, the car's lags, and the general utility. **The rebuilt model's classes differ from U1's**
in 5 of 15 cells:
- With true geometry A needs about 3.9 s to clear the conflict point.
- So the yield equilibrium (the queue lets A go) appears only from three oncoming cars.
- Holding stays the welfare optimum everywhere; yielding costs 1.2-2.8 times as much.

U1's kinematic map above is superseded for numbers.

**The arena matches the revised model:**

| check | result |
|---|---|
| classes | 9 of 9 |
| outcomes | 108 of 108, excluding gaps of exactly 3.0 s |
| payoffs inside the interval | 55 % raw, 78 % after constant offsets (A -0.028, C1 -0.007) |

- The 18 outcome misses all sit at the first oncoming car's 3 s arrival, a threshold cell.
- PET in yield equilibria is 1.65-1.7 s (model 1.55 s), just above the 1.5 s reference.
- For the real stack: hold equilibria at gaps of 3.5-6 s. A gap of 3 s or less is unsafe against a
  holding first car.
- Arena fix: a reused A crept at its line after being teleported. It is now re-parked.

**D2 for S5, decided (2026-11-15):** accepted for classes and outcomes. Payoff levels come from the
arena, with the constant offsets.

### U2. The roundabout entry, kinematic (2026-11-15)

swb 139 (949693c; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop U2 roundabout kinematic/`). A ring
of radius 18 m at 7 m/s, true geometry, with route curvature taken out of comfort. A enters; B
circulates, with right of way.

- **Mostly not chicken.** A waits when B is under 2 s from the conflict stretch, and joins freely
  from 5 s.
- **In between (B 2-4.5 s away), A forces B,** because B's safety layer can always give way.
- **B's right of way stops the forcing only at a fault cost of 1-2.** At 0.5 it selects only at
  2 s. Chicken appears once, at fault 2 and 3 s, and only when B can slow.
- **Courtesy buys little:** the mean equilibrium cost is 0.743 against 0.752 without a fault, and
  worse at fault 1.
- **2-3 circulating cars change nothing.** The pair game predicts the queue's class in 64 of 64
  contexts, so composition holds here.
- **No contact in any equilibrium.**
- **Reading:** a circulating car's ability to give way invites forcing. Right of way restrains it
  only if failing to yield costs about a whole horizon of progress. This is the same pattern as the
  crossing: liability, not courtesy, is the lever.

**In the arena** (71ba6cf, d439727, swb 139; NAS `.../2026-11-15 coop U2 roundabout arena/`; 882
runs on a generated ring with one entry). It matches the kinematic map:

| check | result |
|---|---|
| outcomes agree | 94 % |
| classes agree (no right of way / fault 1 / fault 2) | 11 / 10 / 12 of 14 |
| payoffs inside the interval | 67 % raw, 89 % after constant offsets (A -0.058, B -0.004) |

- The forcing band (B 2-4 s away) and the single chicken cell both reproduce.
- The misses sit at the band's edges: mostly the 0.5 m/s^2 threshold that defines "forced", plus 4
  threshold cells at 5 s.
- Two warm-up faults were found and fixed first: cars teleported onto parked cars got stuck, and
  some starts missed their speed. Runs are now validated, and redone if B misses its speed by more
  than 10 % or its arrival by more than 0.2 s.

**D2 for U2, decided (2026-11-15):** accepted for outcomes and classes, with payoff levels from the
arena and its offsets.

**Through swb on Town03's roundabout** (b1c0684, swb 139; NAS `.../2026-11-15 coop U2 roundabout
swb/`; 30 of 30 runs valid). The first scenario besides the cut-in checked on the swb stack.

| check | result |
|---|---|
| outcomes agree with the arena | 14 of 15 |
| outcomes agree with the model | 15 of 15 |
| payoffs inside the interval | 70 % raw, 88 % after offsets (A -0.014, B -0.012) |

**Found on the way: CARLA 0.9.16 resets tyre friction to 3.5 on every car of the same blueprint
whenever another one spawns.**
- `arena.put_physics` is fixed in b1c0684: physics are re-applied after every spawn. With the fix,
  Town03's arena-vs-model classes are 6/6/6.
- **Every 074 arena run before b1c0684 had stock tyres on the cars spawned earlier.** The cars' grip
  (and so their braking and steering) differed from the spec's. The arena's steady payoff offsets
  against the kinematic models may partly be this.
- **`swb_carla_world` has the same bug**, not yet fixed. It affects the first agent car, or an ego,
  whenever a later Model 3 spawns. That includes C2b, C3 with agent cars, two-AV runs, and C3c.
- Agent cars need `vehicle_physics:=` under `ego_launch:=none`.
- **Decided (2026-11-15):**
  - fix `swb_carla_world` (re-apply the spec's physics after every spawn of the same blueprint,
    and verify it, as acb verifies the ego's);
  - re-run one canary set per tier with the fix (C2b's D2 cells and do7's final table) and re-open
    a result only if a class or a D2 verdict changes.

  The tie to the spec makes this a swb bug, the user's to schedule.
- **Confirmed in C3c** (swb 241): in a two-Autoware run, the first-spawned ego (hero, B) runs tyre
  friction 3.5 on all four wheels for the whole run (146 samples over 8 runs), while the second
  (A) runs the spec's 1.2. Mass stays right.
  - **acb's verify does not catch it:** each acb_bridge verifies once at its own attach, before the
    other car spawns.
  - C3c so far (stock 8 runs, yield 6) is therefore **provisional**. It will be rerun once
    `swb_carla_world` re-applies physics after every spawn and acb re-verifies after every spawn,
    not once.
  - Until then, no new multi-Model-3 swb batches.
- **C3c, two real stacks, provisional** (1fb304e, swb 241; NAS `.../2026-11-15 coop C3c
  cross-play/`).
  - A is an Autoware ego whose route goal lies in B's lane; B is the other Autoware ego. RTC is off,
    so A's own lane-change check decides when it moves.
  - In every swb run, B ran on friction 3.5.
  - **Stock against stock:** A ends behind B at 6/0, 10/0 and 14/-3, and ahead at 2/+2. No contact.
    Real Autoware cuts in only from well ahead.
  - **B on E3's yield setting:** B's cost rises at 6/0 and 10/0 (uB -0.52 / -0.58 against
    -0.44 / -0.49) and is unchanged at 14/-3.
  - **The arena's A_aw_perm against B_aw_react,** rerun at 1fb304e: 14/-3 now has contact in 1 of 4
    runs (3.03 m/s), and 6/0's uA is -0.27 against -0.46 before. The rest is unchanged.
  - To be rerun once the friction fix lands.
- **C3c rerun on the friction fix: no verdict changed** (swb 241, swb 8ed13d7, clean build; NAS
  `.../2026-11-15 coop C3c rerun spec tires/`).
  - `check_spec_tires.py` passed, and the probe read both egos (hero and bg_av_1) on 1.2 in every
    sample (stock 154/154, yield 180/180).
  - Stock against stock: A still ends behind B at 6/0, 10/0 and 14/-3 and ahead at 2/+2, with no
    contact.
  - B on E3's yield setting still costs B: uB -0.542 against -0.465 at 6/0, -0.556 against -0.509 at
    10/0. The earlier stock 10/0 uA of -0.391 was an outlier (now -0.676).
  - Usable runs: stock 6 of 8, yield 7 of 8. Two lost to A's Autoware stuck in INITIALIZING, one
    still misaligned after its retry.
  - C3c is no longer provisional.

### E5 and S5 on the real stack: first findings (2026-11-15)

swb 118, in progress.
- **E5: Town03's Lanelet2 map links none of its 38 traffic lights,** so for Autoware every Town03
  junction is unsignalised and the lights question does not arise there. (Autoware on Town03 never
  obeys a signal; any study of signalised junctions needs the lights linked in the map.)
- **The site is junction 861:** B, the Autoware ego, turns left from road 4 onto road 62; A, a light
  agent, runs north in the inner lane.
- **Pilots:**
  - B decides at its stop line, where its tracker sees only out to about 45 m.
  - Its turn takes about 9 s, so an A unseen at the stop line can arrive mid-turn.
  - In pilot 4, B did not yield and PET was 3.1 s at a lag of 9 s.
  - **Autoware crawls the 27 m turn at about 2 m/s,** taking 7-9 s to reach A's lane: about twice the
    3.9 s clear time of the arena's and the model's turner. (Lag here means A's time to the conflict
    point when B's front crosses its stop line.)
  - **Pilot at lag 7 s: a collision.** A was beyond the tracker's 45 m view at the stop line and was
    tracked only once B was mid-turn. At lag 3 s B stopped inside the junction, A passed first, and
    the clearance was 1.6 m.
  - The first real-stack contact in this study. It comes from the short view and the slow turn
    together, not from the gap rule.

  The grid follows: v 10 m/s at lags 2-9 s (plus 3.5, 4.5, 5.5), and v 8 m/s at 3, 5, 7 s.

**Results** (91a2a91, swb 118; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop S5 left turn
autoware/`): 28 runs with CenterPoint. Lag here is A's time to the conflict point when B's front
crosses its stop line.
- **Autoware yields only when A is within about 50 m at B's stop line.** At 10 m/s it yielded at
  every lag up to 5.06 s and went at every lag from 5.49 s, a critical lag of 5.3 s. At 8 m/s it
  yielded at 5.05 s and went at 7.0 s.
- **A danger band at lags of 5.5-7 s.** B's turn takes about 9 s, so it goes while A is still beyond
  its view, and is hit:

  | v | contacts in the band |
  |---|---|
  | 10 m/s | 6 of 6 |
  | 8 m/s | 2 of 2 |

  Beyond the band, a lag of 8 s gave PET 0.7 s and 9 s gave 1.4 s.
- **Yields are safe:** PET 4.4-7.4 s, clearance 1.0-1.6 m. uB is about -0.6, against -0.38 for a
  free go. The lag-3 cells did not flip (4 of 4 yielded).
- **Against the models:**
  - U1 (G* 3.5 s) agrees in 14 of 22 runs at 10 m/s. Autoware yields at 3.5-5 s where U1 goes, and
    U1 misses the band at 5.5-9 s because its turner clears in 3.2 s, not 9.
  - Against the arena's left_turn_queue: Autoware's unsafe gaps are medium (5.5-9 s), not short
    (≤ 3 s).
- **The cause is the stack's reach against its own turn time,** not its gap rule. A view of about
  50 m at the stop line covers 5 s at 10 m/s, while the turn needs about 9 s. Whether the 50 m is
  CenterPoint's configured range, our simulated lidar's range, or the tracker is the next question
  (swb 118). If it is our sensor configuration, it is a bug in swb's car, not a finding about
  Autoware.
- **Reach and turn, diagnosed** from the S5 bags and configs (swb 118; NAS `.../2026-11-15 coop S5
  reach and turn/`). **Neither is our car.**
  - **The reach is Autoware's default `centerpoint_tiny`.** The raw cloud (clustering) holds A in
    90 % of frames at 40-50 m and 57 % at 50-60 m. CenterPoint holds it in 64 % at 30-40 m, 18 % at
    40-50 m and 1 % at 50-60 m, its scores falling to the 0.35 threshold. The tracker starts only from
    CenterPoint, so at B's decision deadline its effective reach is about 39 m. The intersection
    module's attention area is 200 m, so it is not the limit.
  - **The turn speed is velocity_smoother's lateral limit of 1.0 m/s^2:** sqrt(1/0.079) = 3.56 m/s on
    the arc, measured at 3.56-3.69 m/s. There are no velocity factors, no creep and no occlusion
    stop. An unimpeded crossing takes 7.3 s, not 9.
  - **Candidate fixes** (launch arguments, untested):

    | change | effect on the danger band at 10 m/s |
    |---|---|
    | TransFusion (±92 m), or any reach ≥ 85 m | closes it (5.3-9.5 s) |
    | full CenterPoint, capped at its 76.8 m grid | closes it to about 8.8 s |
    | a lateral limit of 2.0 m/s^2 (the profile's velocity_smoother_param_path) | only lowers its top to about 8 s |

    No sensor spec change is needed.
  - **Next:** S5 at lags 5.5-9 s with each detector (swb 118). Which detector swb's Autoware profile
    should default to is a decision for the user, once those runs are in.
- **Detectors compared** (swb 118; NAS `.../2026-11-15 coop S5 detectors/`; 48 runs, 0 errors; swb
  133858c records the detector's debug timing). Contacts at lags 5.5-7 s:

  | detector | contacts | notes |
  |---|---|---|
  | centerpoint_tiny (today's default) | 5 of 8 | |
  | **full CenterPoint** | **1 of 8** | critical lag 6.92 s; reach about 56 m against 44 m; a one-time 33 s engine build; +600 MiB VRAM at peak; 20 Hz |
  | TransFusion | 8 of 8 | blind on our lidar: 66 objects in 1363 frames, most likely CARLA's lidar intensity |
  | tiny with a lateral limit of 2.0 m/s^2 | 3 of 8 | PET at lags 8/9 s rises to 2.2/3.5 s, from 0.8/1.5 |

  - **Recommendation, for the user to decide:** full CenterPoint as swb's default when a GPU detector
    is used; the 2.0 m/s^2 lateral limit as a separate option. Full CenterPoint with the higher limit
    was not run.
  - **TransFusion's blindness is a swb-side question:** what intensity CARLA's lidar gives against
    what TransFusion was trained on. It is backlogged.
- **Full CenterPoint with the 2.0 m/s^2 lateral limit, and TransFusion** (swb 118; NAS
  `.../2026-11-15 coop S5 full lat2 and transfusion/`; 12 runs, 0 errors).
  - **The higher limit does not close the band; it moves it down.** B went at every lag (12 of 12).
    It made contact at 5.5 s (2 of 2), near misses at 6 s (PET 0.1 s, 0.02-0.17 m clear), and was
    safe from 6.5 s with the best PET of any configuration (0.7/1.3/2.2/3.4 s at 6.5/7/8/9 s).
    Full CenterPoint alone yielded safely up to 6.9 s.
  - **The likely cause** (not checked in the planner's debug output): the faster planned turn makes
    the intersection module's collision check accept the gap. Perception was no later.
  - **The cost for B:** its turn takes 7.85 s instead of 10.35 s, and uB at lags 8-9 s is -0.39
    against -0.68. Over lags 5.5-7 s, uB is -3.8 against -2.2.
  - **So full CenterPoint at the stock 1.0 m/s^2 stays the recommendation.** The higher lateral
    limit trades safety in the band for speed outside it.
  - **TransFusion is blind because of intensity, not point layout.**
    - The spec's `atmosphere_attenuation_rate` 0 makes CARLA's intensity 1.0, so acb sends 255 on
      every point (115k of 115k, seen live).
    - TransFusion takes raw uint8 intensity as a feature; CenterPoint does not.
    - An offline replay of a recorded cloud: 255 or a constant 20 gives 0 hits on A. Road 5 / rest
      60 gives 0.97 at 10-20 m and 0.50 at 30-40 m.
    - Proposed, not made: acb on CARLA's semantic lidar with intensity = R[tag] × cos(incidence),
      the per-tag table in sim_vehicle_spec. It is non-coop vehicle-side work, for the user to
      decide. Side finding: acb's ring channel is 0 on every point.

### Friction fix, full CenterPoint, lidar intensity (2026-11-15)

Approved by the user; built by 133's subagent. NAS `logs/2026 SSv2 on CARLA/2026-11-15 friction fix,
CenterPoint default, lidar intensity/`. Superproject commits: bb284b1, 3e01b32, 3ab735e, fe63f5d.

**The friction reset is fixed in swb** (swb_carla_world d56e99e, acb 23ab103).
- **What CARLA does:** only a spawn of a car of the same blueprint resets friction. It goes to 3.5,
  at once, on the car whose physics was applied last.
- **The fix:**
  - the world server reads every spec'd car back after each of its own spawns, and after each tick
    that shows a new actor, and re-applies the spec if it is off;
  - acb re-checks the hero for up to 20 frames after a new actor.
- **Checked live** (`scripts/check_spec_tires.py`): every spec'd car stays at 1.2 after every spawn.
  The old server gave 24 wrong reads.
- **This unblocks multi-Model-3 swb batches:** C3c's rerun, N5's swb check, and U3 on Autoware.

**`lidar_detection_model:=centerpoint` is now full CenterPoint** (autoware_sim_vehicle d249ebc), on
both releases.
- `clustering` stays the default; `centerpoint/centerpoint_tiny` keeps tiny.
- The lateral limit stays 1.0.
- It loads in an ego-stack start. On a host without the TensorRT 10.8 runtime, the engines are
  rebuilt at every start.

**Lidar intensity comes from the spec** (sim_vehicle_spec 9b7e48d, acb 8ee6ca9).
- The Tesla's top lidar is CARLA's semantic lidar: intensity = R[tag] × |cos(incidence)|, with a
  per-tag reflectance table in the spec (road 5, cars 90, ...). The ring channel is now set.
- spec_hash changed to 01b4d3f5; physics_hash did not, so the pedal maps stand.
- **Offline replay:** TransFusion goes from blind to 0.99 / 0.73 / 0.47 / 0.36 of frames on cars at
  10 / 20 / 35 m and in the side lane. CenterPoint is unchanged.
- **Closed-loop S5 on the new lidar** (swb 118, swb fdd8d53; NAS `.../job4_s5_closed_loop/`; lags
  5.5-9 s, seeds 0-1, 12 runs per detector, 0 errors; the friction check passed on 118 first):

  | | contacts, lags 5.5-7 | critical lag | tracker reach (share 0.5) | 0-10 m det / tracker |
  |---|---|---|---|---|
  | full CenterPoint, old lidar | 1 / 8 | 6.92 s | ~56 m | 0.30 / 0.31 |
  | **full CenterPoint, new lidar** | **1 / 8** | **6.94 s** | ~56 m | **0.95 / 1.00** |
  | TransFusion, old lidar | 8 / 8 | 7.40 s | none | 0 / 0 |
  | **TransFusion, new lidar** | **6 / 8** | overlap (yields and goes at 5.5-7) | ~40 m | 0.13 / 0.18 |

  - **TransFusion does not close the band.** It sees cars now, but its detector reaches only
    20-25 m (score falls towards the 0.1 threshold; nothing past 60 m, so the ±92 m grid is not the
    limit). It loses A at 0-10 m, inside the junction where the contacts happen, and its tracker
    first holds A 0.5-2 s later than CenterPoint's, after B has committed.
  - **Full CenterPoint is unchanged** apart from close range (0-10 m 0.95 against 0.30; it takes no
    intensity, so the semantic lidar's other changes are the likely cause, not checked). No outcome
    depends on it. Full CenterPoint stays the profile's choice.
  - **The reflectance table looks wrong in five places** (swb 118; not tuned, the table is the
    spec's): cars 90 are too bright and uniform (real paint about 5-60; an earlier replay with rest
    60 held A better at 10-30 m); glass returns like the body; no retroreflectors (plates and
    reflectors 200-255 on a real sensor); road 5 × |cos| leaves the ground near 0 (a Hesai or Ouster
    reports compensated intensity, about 5-20 on asphalt); lines 50 and signs 150 look right.
- **Open:**
  - a replay sweep of the reflectance table (record the semantic cloud once, rewrite intensity by
    tag, as `tfrec/rewrite.py` does), then one closed-loop check; it matters only if TransFusion is
    wanted, since full CenterPoint already meets S5;
  - the same check on Isaac (RTX lidar intensity depends on materials; swb 139, option 1).

### E6, E7: site search, and D8 (2026-11-15)

`scripts/coop/sites.py` (646ebf3), offline, from CARLA 0.9.16's OpenDRIVE and the Autoware
Lanelet2 maps. Data and plots: NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop E6 E7 sites/`.

**E6: a 4-way site exists.**
- No 4-arm junction on Town03, Town05 or Town10HD has stop signs.
- The Lanelet2 maps link no light at any Town03 or Town05 4-way.
- The maps hold no `right_of_way` and no `traffic_sign` elements, only `traffic_light`.
- **How Autoware treats such a junction:** its intersection module yields to anyone predicted to
  conflict and never stops when the junction is clear. That is a mutual yield, not an all-way stop.
  Its stop line module acts only on `traffic_sign` elements of type `stop_sign`, which these maps
  lack. Autoware has no first-come-first-served logic.
- **Best site: Town05 j979.** It has 2-lane arms and about 120 m of straight approach on every arm;
  its lights are not linked in the map, and SSv2 can switch them off. j1427 is the alternative.

**D8 decided: U3 goes ahead on Town05 j979.**
- Kinematic and arena first, the arena mirroring the straight 4-way.
- On the real stack, two variants:
  - stock: a mutual yield, with no stop;
  - a stop-sign map variant: four `traffic_sign` stop_sign elements at the approach lanelets'
    stop lines, giving "stop 1 s, then yield". It is a local copy of the Town05 map, built for the
    study, not swb's default map.
- The swb runs wait on the friction fix.

**The stop-sign map variant is built** (5d94dee, `scripts/coop/stop_sign_map.py`). Data: NAS
`dataset/2026-11-15 Town05 j979 all-way stop map/`; use `map_path:=` its `Town05/`.
- It adds a stop_sign `traffic_sign` element on all 8 approach lanelets of j979, two per arm.
- It parses with Autoware 1.5.0's loader libraries offline, and the stop_line module's lookup
  finds all 8 stop lines.
- It has not been run on a stack; that waits on the friction fix.

**U3, the four-way stop: kinematic and arena** (swb 139, 0daa4a3 `four_way_game.py`,
`four_way_arena.py`, on 4fcaa8d's brake-to-rest arena; NAS
`logs/2026 SSv2 on CARLA/2026-11-15 coop U3 four-way stop/`). Two cars on perpendicular approaches,
dt the arrival offset, 6 and 10 m/s, 6 rules × 6, 1620 arena runs per speed.
- **Stop-then-yield never deadlocks, but it breaks ties by noise.**
  - The order is arbitrary even at ±6 s: A goes first in 1-4 of 5 seeds.
  - Cars flip between go and hold 5-41 times per arena run.
  - Contacts: 3 of 45 self-play runs at 6 m/s, 1 of 45 at 10 m/s.
- **Stock mutual yield inverts the arrival order** for |dt| ≲ 4-5 s: the early car's 4 s margin
  stops it, and once stopped it is predicted static. At a tie it crashes after restarts
  (kinematic).
- **FCFS alone crashes at ties.** At |dt| ≤ 0.25 s both cars "stopped first": 14-15 of 15 arena
  runs make contact.
- **FCFS + right-before-left at a tie removes the ties** (C7's binding priority). One class at
  every dt in the kinematic grid, with no flips.
  - Arena residue: 3 of 30 runs at dt -0.25 s. Perception puts the two stops on opposite sides
    of the 0.5 s tie threshold, so each car takes a different branch.
  - A binding rule needs both cars to see the same fact.
- **Right-before-left alone removes the ties but overrides arrival:** B goes first even when A is
  about 3 s earlier.
- **Mixed conventions deadlock in the arena** (42 runs): A on right-before-left waits for B on the
  right, while B on FCFS waits for A, who stopped first.
- **D2:** outcomes 86 % (94-95 % off the tie), classes 7 of 9 per speed.
- **In the protocol frame** (design, 9855368):
  - FCFS lacks symmetry breaking at a tie;
  - stop-then-yield breaks symmetry by noise, randomized and unbounded;
  - FCFS + right-before-left breaks it deterministically by role;
  - mixed conventions are the BAR case.
  The residue is a shared-observation problem: the role must come from a fact both cars see
  alike, or from a message.

**E7: no lane drop or on-ramp on any town with an Autoware map** (Town03, Town05, Town10HD).
- The only merge is a 45° Y merge, Town03 j1901, handled by Autoware's intersection module; the
  arena has no geometry for it.
- The arena-exact lane drop is on Town06, road 48 (140 m at full width, a 44 m cubic taper).
  Town06 is not installed here (it is in the NAS's AdditionalMaps archive, 14.8 GB) and has no
  Lanelet2 map. Town04 is installed but also has no Lanelet2 map.

**N5 (a), the 45° Y merge, kinematic and arena** (swb 118, 9b2f95c: `arena.xodr_ymerge`, which
mirrors Town03 j1901 with lanes within 0.02 m of the map, `ymerge_game.py`, `ymerge_arena.py`;
NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop N5 Y merge/`).
- **Two cars: the nearer car goes first** everywhere except a both-orders band at |Δt| ≤ 0.1-0.2 s.
  There the worst equilibrium costs up to 1.35. No equilibrium has contact. A right of way barely
  selects: it shifts the band and adds pockets of waiting.
- **D2:**
  - at v 10: payoffs 72 % inside, outcomes 92 %, classes 7 of 7 (the crossing S2 had 47-53 % and
    5-6 of 9). Accepted for outcomes and classes.
  - at v 14: 39 % and 5 of 7, likely the arena's tracking error on the 22 m arc. Not accepted.
- **Stream** (kinematic only):
  - N stops mattering early: uA is fixed by N = 3, A's externality is within 0.05 by N = 10 and
    exact from 20.
  - N3's walk holds for τ 0.5 and 1.5 (error ≤ 0.003). It fails for τ 3 (mean 0.1, worst 0.37):
    courteous followers slow for a waiting A, S3's "after you" in a stream.
- **Verdict: the Y merge is S2's crossing game with a cheap fall-in-behind, not S3's deadline
  game.**
- **The three-car swb check** (swb 118, swb 166f3c3, b0dceba; NAS `.../2026-11-15 coop N5 swb
  three-car Y merge/`): Town03 j1901, three light agents through the relay (A on the diagonal with
  the merge rule; B first on the main road, C behind at h 1.5 s), v 10, dt −1..1, A's rules t0.5yp2,
  t1.5yp2, t3yp2, 2 seeds: 30 of 30 runs valid, spec tyres (1.2) on all three in every run, no
  contact. Against the brake arena (45 runs, j1901's geometry): **outcomes 15 of 15**, payoffs
  inside the arena's interval **61 of 90 (68 %)**, the near-tie share 0 on both sides.
  - The payoff misses are small and systematic: A starts at 10.23-10.26 m/s after its warm-up's slow
    zone (0.02-0.03 more lost progress), and Town03's main road curves about 47 m past the merge
    (radius about 29 m), which the arena's straight road lacks. Neither changes a slot or a contact.
  - The site needed three run-setup fixes: `vehicle_physics:=` (the spec's
    `carla_physics.yaml`) on `just run`, whose default `none` leaves agent cars on the blueprint's
    friction 3.5; a 4 m/s cap through connector 943's ~70° kink on A's warm-up; lane paths 260 m past
    the zone (at 160 m, B and A braked to the path's end inside the window: payoffs 47 %).
  - **D2 for three cars, decided (manager, 2026-11-15): accepted for outcomes and classes**, payoff
    levels from the arena, as for S2. N5 (a) and the swb check close N5 for the Y merge.
  - Not done: tuning out A's start overshoot.
- **Stock Autoware as B** (swb 118, swb 377bece; NAS `.../2026-11-15 coop N5 Y merge Autoware B/`;
  B the Autoware ego with spec physics and full CenterPoint, A and C light agents, 30 of 30 valid,
  no contact):
  - **Outcomes 11 of 15 against the arena at the nominal Δt, 14 of 15 at the Δt Autoware actually
    gives.** Autoware is still accelerating when the run starts (8.9-9.8 m/s), so it gains about
    0.3 s on A's warm-up: nominal −0.3 is a level start. The closest arena B is HOLD (REACT gives the
    same outcomes); every yb* rule matches less.
  - **Autoware uses the intersection module only when A is first** (Δt −1): no light, so the
    not_prioritized margins (4.0 s / 3.0 s, hold 0.5 s). Its STOP comes 0.3-0.4 s after A is first
    tracked; B brakes at 3.4-4.0 m/s² peak and stops at the zone's edge. Elsewhere A has yielded
    before Autoware tracks it, and B only slows at 0.8 m/s² for the walkway and curve ahead.
  - **Mutual yield seen:** with a cautious A (t1.5yp2, t3yp2) at Δt −1, both cars wait until
    11.7-13.1 s, when the STOP ends; once (t3yp2 s0) B goes first instead. This is the coordination
    failure the conventions target, on the real stack.
  - **Autoware tracks A late:** 2.6-3.9 s into the run, with A 20-32 m before M, though A is in the
    lidar from the start. Not yet explained (occlusion at the corner or the object filter).
  - Payoffs are not comparable (2 of 90): Autoware's walkway module stops B for about 2.3 s, 40 m past
    M, inside the 20 s window.
  - **Rerun** (swb 118, swb 0c01de5; NAS `.../2026-11-15 coop N5 Y merge Autoware B rerun/`; A paced
    on B's acceleration, nominal Δt within 0.02 s; 30 runs plus 10 Autoware-alone baseline runs,
    all valid, no contact):
    - **Late tracking is static occlusion at the corner, not the stack.** 0 of 15 ray casts reach A
      until about 2.8 s. The detectors pick A up at 2.5-3.2 s (28-31 m), and the tracker follows
      0.05-0.15 s later. Stock occlusion handling in the intersection module is off.
    - **Outcomes match HOLD in 11 of 15 cells, and the misses are Autoware's policy.** All 4 are
      A-first cells with a cautious A (Δt −0.3 for all three rules, Δt −1 for t1.5yp2): Autoware
      issues an intersection STOP, both cars wait, then B goes first. Classes agree at 4 of 5 Δt
      (not −0.3).
    - Against the baseline, 11 of 90 payoffs fall inside the arena's interval; uB matches the arena
      at Δt ≥ 0.
    - **Mutual-yield waits:** 2 of 30 strict and 8 of 30 crawling, only at Δt −1 / −0.3 (2.1-4.3 s,
      starting as the STOP lifts). None in the baseline or at Δt ≥ 0.
    - What it means for 078: stock Autoware at a Y merge yields to a car that is first, then loses
      the tie to a cautious one: the coordination failure the merge row targets. The row stays
      stock (G0), so this is the baseline T2 measures against.
  - **Autoware's occlusion handling on** (swb 118; autoware_sim_vehicle 8be9363 added
    `intersection_param_path`, default upstream, and `intersection_occlusion.param.yaml`; NAS
    `.../2026-11-15 coop N5 Y merge occlusion on/`): **keep it off by default.** The corner at j1901 is
    occluded by static buildings, so occlusion handling is active in every run, even with A parked.
    B creeps (3.7 m/s at u −60, 2.6-2.8 at the stop line), so 0 of 40 runs are valid for the paced
    protocol. On an empty junction it costs +11.6 s from u −100 to M (32.0 against 20.4 s). B stops at
    the line in 30 of 30 runs (was 7), crawl waits in 30 of 30 (was 8), strict waits in 11 (was 2).
    Outcomes match HOLD in 4 of 15 (was 11): A is never first. B's peak braking before M falls to
    1.3 m/s² (was 4.1), but Autoware slows the same way with no A at all, for the occlusion and not
    for A. 1 contact, A with C (light agents only). Not tuned. The argument stays for occlusion
    scenarios.
- **Still open:** N5 (b), Town06's lane drop, still needs a Lanelet2 map.

### Audit: did any Autoware run drive blind? (2026-11-17)

077 G54 (swb 243) found that an Autoware pilot can start a run while lidar_centerpoint is still
building its TensorRT engine, so the ego drives with no lidar objects (a demo incident collided at
11 m/s). Every Autoware run in 074 and 078 was audited, by grepping each start's lidar_centerpoint
log for an engine build and checking that the other car was tracked while in range. **None was
affected:**
- **078's crossing rounds (host 133):** 112 CenterPoint logs, none with a build (the 10.8 runtime
  is set in .env); A tracked in 40 of 40 runs.
- **swb 241 (C3a's half, every C3c batch, the Isaac and multi-pilot cells):** no CenterPoint at all,
  only ground-truth objects or clustering. A was never missing within acb's 100 m ground-truth
  range.
- **swb 118, N5 with Autoware as B** (first batch, rerun, occlusion on; 172 starts): every start
  loaded a cached engine; A was tracked in every run where it drove. **S5 detectors:** two starts
  built an engine (full CenterPoint 31 s, TransFusion 58 s, both at stack start), and every run had
  objects. TransFusion's blindness on the old lidar persists after cached restarts, so it is the
  intensity input, not a build. **S5 closed loop and S5e:** the stack logs are gone, but every
  run's rows show tracked objects. **E3** ran on ground-truth objects.

### C8: the four-way stop as a verified protocol, no V2X (2026-11-15)

133's subagent (8481ce6, 6fecc16, 9475e10: `c8_fourway_check.py`, an explicit-state checker over
discrete time Q = 0.05 s; rules `fcfsm`/`fcfse` in `four_way_game.py`; `xodr_crossings(angle=)`).
NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop C8 four-way protocol/`. U3's outputs reproduce
unchanged.

**The formal model** (each car reads the other's rest stamp within ±e, 0.70 s late; every other
phase change 0.70-0.80 s late; e 0.25, m 0.3, H2 2 s; 161 contexts per speed for 2 cars):
- **U3's FCFS + right-before-left (fcfsr) is unsafe for any e > 0**: a band of width 2e near
  Δt ≈ −T where one car reads "first" and the other "tie, I am on the right" (10 unsafe and 10
  deadlocked contexts; e 0.45: 18/18). U3's 1 s grid missed it.
- **The fail-safe rule (c)** (margin m, both wait when the reading is ambiguous, a right-before-left
  second round once the junction is quiet) is safe and live for 2 and 3 cars. The smallest safe
  margin is **m = e**, with **H2 ≥ e + 1.2 s**; the worst wait is about 7.4-8 s.
- **Four cars:** (c) deadlocks in every tied context (right-before-left is a cycle). **(e)**, (c) plus
  the map's fixed approach order as a last resort, is safe and live for 2, 3 and 4 cars (worst wait
  16-19 s, wait-for cycles ≤ 2.95 s). Principle 3c holds.
- Exit blocked: (c)/(e) 0 of 81 unsafe, if the blocked exit's release counts as a junction event.
- **BAR, with the safety layer:** (e)'s unsafe set lies inside stock's against every rule except
  stock yield, which at rest goes on a "static" (e) car (|Δt| 1.15-1.75 s).

**e, measured:** stamp − rest −0.10..+0.40 s (1-99 %) in the arena; centred (bias 0.2 s), e99 ≈ 0.25,
e99.9 ≈ 0.4; at noise ×2, e99 ≈ 0.43. The tail is unbounded: a quantile and a contract.

**Arena** (8 rules × 8, U3's Δt, 5 seeds, 2880 runs per speed):
- (c)/(e) self-play: **0 contacts, 0 deadlocks** in 90 runs; fcfsr 2/90, fcfs 28/90.
- **"Never worse than stock" not met:** (e) is worse in 4 of 12 BAR cells at each speed (totals v6
  26 vs 42, v10 23 vs 17). The cause is stock Autoware's "a car at rest stays": the both-wait pause
  invites a stock car in, and (e)'s second-round go races its restart. The rbl cell is a car that
  ignores arrival order.
- Cost: −0.18..−0.33 per car away from ties against fcfsr (the zone guard waits for a cruising car to
  brake visibly), +0.67 at ties; better than stop1y everywhere. D2 74/64 % payoffs, 91/89 % outcomes.

**Deployment slice** (v6, 27 runs per pair): skew 60° and 120° change timing, not the protocol.
Noise ×2 with m 0.45: **3 of 27 self-play contacts, 1 deadlock**: the zone guard and the "moved off
its line" test flip on noise (up to 141 flips per run); the guards need hysteresis, the margin is not
what fails. **A heavy vehicle** (HGV, 1.4 m/s² launch) under-estimates its own zone interval about
2× with the Model 3's launch: zone intervals must come from the car's own capability.

**What C8 leaves:** guard hysteresis (latch a go for the launch dead time, filter 2-3 samples);
per-vehicle zone intervals; a legible probe (creep 0.5 m before a round-2 go) against stock's
static-at-rest; (e) on swb against stock Autoware; TLA+ cross-check; 4 cars in the arena; V4 (the
other rows). G0 in 078.

### C7: the coin at the stop (2026-11-15)

swb 241; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop C7 coin at stop/`. The coin is sent at the
first stop instead of at the restart.

| | restarts with contact (arena) | gap recovered: arena (kinematic) |
|---|---|---|
| crossing, coin at stop | 0 % | +1.00/+0.84 (+0.19) |
| crossing, coin (C7) | | +0.99/+0.71 (-0.37) |
| crossing, binding priority | | 0/+0.80 |
| narrow passage, coin at stop | 5 % | +0.74/+0.81 |
| narrow passage, coin (C7) | 6 % | +0.50/+0.55 |
| narrow passage, binding priority | 0 % | +0.76/+0.81 |

- **Every remaining crash is the coin's winner against a B whose rule never stops** (`go`): a BAR
  case, where a car that cannot or will not stop ignores the coin.
- **Away from ties it costs.** At the narrow passage the kinematic loss is -0.46, and it is
  self-enforcing in only 56 % of contexts.
- **So the coin must be gated to the conflict window, and a car that cannot stop keeps the turn.**
  That is C8's fail-safe rule in another form.
- C7's own none/bind/coin numbers do not change on the fixed arena.

### Friction canaries (2026-11-15)

swb 241, at 1fb304e; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop friction canaries/`.
Re-run on the arena's friction fix (b1c0684); **no verdict changes**.
- **C2b's D2:** 44/48 (92 %), outcomes 24/24, the same four uA misses.
- **do7's table** (240 runs): the same pair, A1_s1l49_1 / B2_s1l42_0, vA -0.137, vB -0.038.
  Every cell is within 0.011 of do7.
- **Only C3c's 14/-3 arena cell moved:** 1 of 4 runs touch, the known knife edge.
- Nothing from 11-14 or 11-15 re-opens on these two.

### Arena tracking and the D2 gaps (2026-11-15)

swb 118, 407437a; NAS `logs/2026 SSv2 on CARLA/2026-11-15 coop arena arc tracking/`.
- **It is not the arc or the controller.** Pure pursuit tracks the Y merge's 22 m arc within
  0.37 m at 6-14 m/s, and S4's swerve within 0.09 m. A feed-forward + Stanley tracker was worse and
  was reverted.
- **The gaps are on the kinematic side:**
  - **Launch delay.** From rest the arena (and swb) car moves about 0.35 s later.
    `crossing_game.LAUNCH_DEAD` 0.5 (`--launch-dead`) matches it to 0.06 m. With it, S2's payoffs
    inside go from 41/51 % to 74/83 %.
  - **S4's contact deficit is a model artifact.** A held car crawls at 0.09 m/s past the zone edge,
    where its safety layer stops acting. `HOLD_STOP` (`--hold-stop`) closes it:
    - contacts, arena/model: 146/249 → 146/149 and 148/219 → 148/148;
    - outcomes: 73-78 % → 86-87.5 %;
    - payoffs: 35-38 % → 55-60 %.
  - **The Y merge at v 14 stays at 44 %.** The causes are PET's 0.05 s sampling (one sample is
    about 0.025 of risk, over D2's 0.02 floor) and the 30° heading-filter knife edge in
    `measures._pet`. A fix is proposed, not made.
- **Old results do not move with the switches off:**
  - kinematic maps byte-identical (crossing, S3, S4, Y merge);
  - S2's published D2 reproduces;
  - do7 within 0.0007.
- **Decision (manager):** adopt both switches as the kinematic model's defaults, as with the earlier
  parity fixes (A's observation delay, route curvature). Re-run the crossing-family maps and their
  D2 on the new defaults, plus a continuous-time PET in `measures`. Assigned to swb 118. S4's D2
  is re-judged on the result.
- **Re-run on the new defaults** (swb 118, e995a7b + 6e0a7d1; NAS `.../2026-11-15 coop kinematic
  defaults rerun/`; U3 left to swb 139).
  - **The defaults:** LAUNCH_DEAD 0.5, HOLD_STOP on, and a continuous heading-weighted PET (within
    0.01 s of exact). The legacy flags reproduce every old map byte for byte.
  - **The cut-in does not move:** C2b's D2 stays 44/48; do7's cells are within 0.0002.
  - **D2, payoffs inside the arena's interval, published → new:**

    | game | published | new | other |
    |---|---|---|---|
    | S2 crossing | 41/51 % | 73/75 % | |
    | S4 | 35/38 % | 52/59 % | outcomes 73/78 → 89/90 % |
    | S5 left turn (arena) | 49 % | 68 % | |
    | Y merge, v 14 | 39 % | 48 % | |
    | Y merge, v 10 | 72 % | 68 % | |
    | U2 | 66 % | 66 % | outcomes 94 → 89 %; right-of-way classes 10/12 → 8/8 |

  - **Class changes, each traced to one switch:**
    - S4, W_n 30 v 6: the chicken on B's side becomes B-first, all from HOLD_STOP (the crawl artifact).
    - U2: the forcing band at tb 2-2.5 s moves to B/C, all from the launch delay.
    - S2 changes only at v 4; the Y merge only in its tie band; S5 in one cell (C → H).
  - **No equilibrium has contact anywhere.**
  - **HOLD_STOP and the arena** (swb 118, e1695f5; NAS `.../2026-11-15 coop hold-stop arena check/`).
    - **The arena's policies are restored:** HOLD_STOP no longer acts there by default
      (`--arena-hold-stop` turns it on).
    - **When held, the arena car coasts to rest.** Creep mean 0.02 m, max 0.65 m; 5 % of holds are
      still moving after 0.5 s. The old kinematic crawl reached 1.01 m, with 23 % still moving.
      So HOLD_STOP is the better model of the arena.
    - **S4's contacts:** 146/148 published → 127/126 restored, mostly from b1c0684's tyre fix
      (136/138 on the old tyres). The kinematic model gives 149/148 with HOLD_STOP, against
      247/217 with the old crawl.
    - **D2 with the arena restored, payoffs inside:** S2 70/71 %, S4 56/42 % (outcomes 90/91 %;
      S4's payoff share swings ±15 points between arena reruns), Y merge 70/48 %.
    - **Decided by the user (2026-11-15): held arena cars brake to rest,** like Autoware's STOPPED
      state (stopped_acc -3.4). This changes the reference: swb 118 makes it the arena's default,
      with the coast behaviour as a legacy switch, and reruns the arena cells it touches.
    - **Done** (swb 118, 4fcaa8d; NAS `.../2026-11-15 coop arena brake to rest/`).
      - **The change:** below 0.5 m/s, arena.Longitudinal commands at most -3.4 m/s^2 for a held car,
        in every arena and in the light swb agents. `--legacy-arena-coast` restores coasting.
      - **Held cars stop creeping:** 1-4 still move after 0.5 s per study, against 54-112 before;
        the max is 0.3 m.
      - **D2, coast → brake:** S2 70/71 → 62/77 %; S4 56/42 → 51/54 % (outcomes 90 %; classes
        11/10 and 11/11 of 14); Y merge 70/48 → 72/47 %; U2 66 → 63 %; S5 68 → 68 %.
      - **Unchanged:** C2b's 44/48 and do7. C7's outcomes are 93-95 % (more classes agree at the
        narrow passage), and the zipper's ranking holds.
      - **Kinematic follow-up, open:** in S2's near-tie band the braked arena now avoids restart
        crashes that the kinematic model still has (S2 v 10 outcomes 97 → 93 %). The likely cause was
        thought to be a missing brake-release lag after a braked hold.
      - **Checked, not the lag** (swb 118, swb eed85be; NAS `.../2026-11-15 coop kinematic brake
        release/`). After a braked hold the brake arena's car first moves 0.60 s after the go, the
        same as the kinematic model (LAUNCH_DEAD was fitted on a braked hold), and the two are
        0.04 m apart after 5 s; the coast arena moved at 0.25 s. **The cause is perception noise**,
        which the kinematic model lacks: with the arena's noise added, the kinematic go/hold contact
        share at |Δt| ≤ 0.1 s falls from 1.00 to 0.45-0.55 (brake arena 1-3 of 5); with a fast launch
        it stays 0.93-0.97 (coast arena 4-5 of 5). At a near tie the race is a coin flip.
        (`LAUNCH_DEAD = 0.5` is the dead time from the first drive command, brake release included;
        the first visible motion comes about 0.1 s later.)
      - swb 241's partial check agrees (not on the NAS; 241's `coop-c3/brlag`): braked rest to
        0.1 m/s in 0.45-0.55 s, to 1 m/s in 0.75-1.43 s for a_go 3 down to 1, whatever the hold
        time. The one real lag gap is a release just before rest (the kinematic car goes on from
        about 0.24 m/s, the arena car stops first); a switch for it moves D2 payoffs 0 to +3 points
        and no outcome or class, so it is not adopted.
      - **Re-scored** (swb 241, commit 58498f2, not yet pushed; NAS `.../2026-11-15 coop D2 near-tie
        share/`): `--tie-share N` on the crossing, narrow, Y-merge, four-way and tiebreak arenas,
        default off (off reproduces every published `compare.txt` byte for byte). Kinematic side
        rerun with N = 20 seeds per pair, against 118's brake-to-rest arena runs and 139's U3 runs.
        Outcomes old → new (v low/high): S2 93/96 → 96/95 %, S4 90/90 → 92/91 %, Y merge 92/92 →
        98/96 %, U3 86/86 → 89/90 %, C7 crossing 95/93 → 97/96 %, C7 narrow 95/94 → 97/96 %. Payoffs
        and out-of-band classes unchanged. 3126 of 3708 band pairs are 0/0 or 1/1 on both sides; in
        the 582 others the mean |kinematic − arena| share gap is 0.21. The arena has only 3-5 seeds
        per pair, so the share test (2 pooled SE, at least 0.1) is lenient; more arena seeds in the
        band would tighten it.
      - **Arena at 20 seeds per band pair** (swb 241; NAS `.../2026-11-16 coop D2 near-tie arena
        seeds/`; `tiebreak_arena --variants`, default unchanged, local db1a7ee). Outcomes old → new:
        S2 96/95 → 95/97 %, S4 92/91 → 91/92 %, Y merge 98/96 → 97/96 %, U3 89/90 → 89/89 %, C7
        crossing 97/96 → 97/96 %, C7 narrow 97/96 → 96/96 %. **D2 verdicts stand.** Band pairs
        agreeing by share 3639 → 3612 of 3708. The share gap is tighter: over mixed pairs (582 → 720) the
        median falls 0.15 → 0.05 and the 90th percentile 0.45 → 0.30. More pairs differ (66 → 96)
        because the test is sharper. 66 of the 96 are C7 narrow, in three systematic patterns: `none`
        t1.5yp5 against tinfp2 / flash / t0.5p2 (kinematic 0.85-0.95, arena 0.3-0.6); `coin/A` X/go at
        Δt −0.1, v 6 (arena 1.00 in 20 of 20, kinematic 0.45-0.70); `stagger` t1.5yp5/go and go/t0.5p2
        (kinematic below the arena). The narrow passage stays stock in 078 (G0), so these do not block.
      - **Decided (manager, 2026-11-15): score the near-tie band by contact share, keep the
        kinematic model noise-free.** In |Δt| ≤ 0.1 s an outcome is a contact probability, not a
        class; D2 there compares the kinematic share (with the arena's noise, sampled) against the
        arena's share. Seeded noise in the kinematic model itself would move every study's numbers
        and buys nothing outside the band. The band lies inside the protocol's tie margin
        (2e ≈ 0.3 s), where the convention, not the race, decides, so this scoring is what 078's
        EV1 uses there.

### C4. Fit and explain

Note on identifiability: from 2-3 discrete choices per run, an SVO angle and a capability are
confounded (a gentle brake is "cannot" or "will not"), and a few hundred L3 runs do not separate
them. C4 fits on C2's continuous rules first, where the arena controls both, and reports which
parameters the data identify.

SVO angles and potential-game weights fitted to C2 and C3; residuals by region.

**Results (2026-11-15).** Commits 8e351bc, d649875, 6b82f6c. Data: NAS `logs/2026 SSv2 on
CARLA/2026-11-15 coop C4 fit/`.

**New arena data,** with seeded noise shared across profiles:
- payoff tables for C1's 9 profiles over C2a's grid (171 cells, 6156 runs);
- what do6's equilibrium and the Autoware-like pair actually did on that grid;
- tables at C3's 6 cells.

Play is scored on what both cars can see.

Held-out payoff error (5-fold over cells):

| model | equilibrium play | Autoware-like play |
|---|---|---|
| table-free baseline | 0.140 | 0.060 |
| **potential (α)** | **0.111** | **0.046** |
| κ rules | 0.126 | 0.079 |
| SVO QRE | 0.20 | 0.080 |
| selfish QRE | 0.20 | 0.186 |
| level-k | 0.17 | 0.152 |

- **The rules read as preferences.** Selfish rules read as φ_A ≈ 10°, φ_B ≈ 0°, α ≈ 8°. The
  Autoware-like rules read as very prosocial (φ_A 66°, φ_B 89°, α 66°): their conservative gap check
  and prepare phase look like courtesy. In κ terms, the equilibrium A acts as if B always yields
  (p0 ≈ 1), Autoware's A as if B never does (p0 ≈ 0).
- **The largest misfit** is where A is close and faster: A gets ahead more often than any model
  predicts.
- **Identifiability.**
  - With SVO types planted on the arena's tables, φ_A is recovered within a few degrees, but φ_B's
    95 % intervals are 30-60° wide.
  - B's reaction time is identified. A wrong assumed reaction widens φ_B but does not invent
    courtesy.
  - On the real stack, neither φ_B nor a courtesy term is identified. At 10/0 Autoware brakes
    (uB -0.12) where the arena's yield and hold both cost -0.005: no preference explains it, so it is
    capability or safety behaviour.

**D5 decided (2026-11-15).**
- **On the car,** the κ rules stay the decision rule: readable, second on held-out error, parameters
  identified.
- **To predict other cars,** carry the potential model: one α (and λ) per class of car. It is best
  held out on both play sets, with α identified to about ±5°. Read α as a description of a rule
  family, not a motive.
- **Not SVO or level-k for prediction.** SVO stays in the arena as a knob for planting
  preferences.
- **Departures such as Autoware's braking at 10/0** belong in the car's capability model.

This fits D6: no belief node; the model explains and predicts other cars.

**Also, the arena's merge-end bug, fixed** (f8bb156, a247a77; NAS `.../2026-11-15 coop arena merge
end/`). A run now continues 1.5 s past the planned end of A's lane change before got_in is scored.
- Records keep the old scoring beside the new (`*_legacy`), so every impact is a paired difference.
- Only one old cell was mis-scored: 6/0 with A_aw_perm in C3's refits.
- do6's table re-scored keeps its equilibrium (welfare -0.174). do7, the rerun with the fix,
  converged in round 2 to the same class: an aggressive A without a prepare phase against a B that
  lets it in late and gently (vA -0.136, vB -0.038).
- The Autoware-like A rules gain: A_aw_perm -0.25 → -0.19, A_aw_cons -0.48 → -0.25.
- C2b's D2 is unchanged (44 of 48).
- The refit's best error over the 11 cells without 14/-3 drops to 0.018 (ground truth) and 0.004
  (CenterPoint). Most of the stop layer's penalty was this bug: 0.097 → 0.047 and 0.067 → 0.016,
  though it still worsens every fit.

### C5. Car models

C1-C3 for pairs from the catalog (Model 3, golf cart, a heavier car when one is specified), the
capabilities from `just vehicle-check` instead of `TYPES`.

**E4 and C5 results (2026-11-15).** Commits 5f6da18, eb80eef, bc50b2d. Data: NAS `logs/2026 SSv2
on CARLA/2026-11-15 coop C5 car models/`.

**E4: heavy vehicles in CARLA 0.9.16**, stock physics:

| blueprint | mass | 0-12 m/s | full brake from 12 m/s |
|---|---|---|---|
| european_hgv (6 wheels) | 7000 kg | 8.7 s | 22.4 m, 4.56 m/s^2 peak |
| firetruck | 10 000 kg | 10.6 s | 26.0 m, 3.67 peak |
| carlacola | 4600 kg | 5.95 s | 13.5 m, 6.88 peak |

- The bus and the cybertruck are car-like.
- **A heavy-vehicle spec would need:** a 4-wheel body (the format refuses the 6-wheel HGV); chosen
  mass, brake torque with a slower air-brake lag, resistance and steering; an `ev` or `ideal`
  powertrain; and downstream, pedal maps, vehicle_info, a sensor mount, an SSv2 catalog entry and an
  Isaac USD.
- Whether to add one to sim_vehicle_spec is the user's decision; nothing was added.

**C5: the commitment hypothesis is not supported.** A car that cannot brake or accelerate hard is
not yielded to more often.

| pair, tier | weak car yielded to | Model 3 yielded to |
|---|---|---|
| HGV as B, kinematic | 265 of 629 | 266 |
| HGV as A, kinematic | 138 | 132 |
| HGV as B, arena | 8 of 50 | 13 |
| HGV as A, arena | 15 of 40 | 15 |

- Where the heavy A earns more, it is its assumed gentler lane change, not its weakness: with a
  3 s lane change it earns more nowhere.
- The cut-in's yield needs about 2 m/s^2, which every car here has.
- The golf cart (kinematic, v_B 6) is yielded to in 126 contexts against the Model 3's 120.

### C6. Hidden capability

Each light policy believes the other's class and limits from a prior and the observed motion, not
the truth; the type-confusion matrix and misbelief regret per context.

**Results (2026-11-15).** Commits a5c47e8, 363514f, bda6052. Data: NAS `logs/2026 SSv2 on
CARLA/2026-11-15 coop C6 hidden capability/`.

- **Setup.** Kinematic maps at v_B 12 m/s (1368 cells, 263k runs) and at v_B 6 m/s with the golf
  cart (1482 cells). Types: the Model 3, an assumed truck-like car, the golf cart; B yielding or
  holding.
- **Bar.** A measure counts in a cell if it is ≥ 10 % of the Autoware pair's welfare loss against
  the optimum, ≥ 0.01, and > 2 se over 8 paired seeds.

Share of cells over the bar, at v_B 12 m/s:

| measure | selfish A | prior-weighted | prosocial A | prior-weighted |
|---|---|---|---|---|
| misbelief regret (truth vs class) | 2.7 % | 5.6 % | 2.1 % | 3.2 % |
| VoI of a capability broadcast | 0 | 0 | 0 | 0 |
| VoI of a belief over a probing rule | 0 | 0.6 % | 0 | 0.6 % |
| VoI of probing as a fixed rule | 7.9 % | 14.6 % | 4.2 % | 7.9 % |
| VoI, truth vs prior only | 9.2 % | 19.6 % | 5.5 % | 10.8 % |

- **What carries the value is B's intent, not capability.** A probe (a 0.12-lane nudge, then the gap
  check) recovers almost all of it, and a fixed rule with no belief can do that. Mean regret is
  0.0018; the capability broadcast's value is 0.
- **No contact in any mode,** even when A acts on a wrong fixed belief.
- **Type confusion.**
  - Motion alone never identifies a truck: cars do not show their limits before A commits.
  - The class cue (box size) is right 96-98 % of the time.
  - Intent is read right 65 % of the time for yielders and 91-93 % for holders, erring toward
    "holds", the safe side.
  - Believing a holder yields costs more (worst cell -0.41) than the reverse; capability misbeliefs
    cost nothing.
- **B's side gains nothing from a belief over A** (regret ≤ 0.008).
- **The arena** (36 cells, 6912 runs): outcomes agree in 99 %; payoffs 84 %, with a steady offset
  in A's cost of dropping behind. The D6 measures are paired differences, so the offset cancels. The
  number of cells over the bar agrees: regret 3 against 4 (33 of 36 cells agree), broadcast 0 and
  0, belief 0 and 0.
- **Caveats:** cut-in only (the crossing was not run); the truck is assumed (no truck in the spec);
  the spread of capability within a class is not modelled.
- **Bug found in arena.py, not yet fixed.** It ends a run at the planned end of the lane change once
  past the 20 s horizon, then scores got_in = lat ≥ 0.9. With CARLA's steering lag, a merge planned
  to end after 20 s scores as never in. do6's contexts merge earlier; c6 uses a 1.5 s settle.

**D6, decided by its criterion and confirmed by the user (2026-11-15): stop at the findings.**
- No belief node, and no capability broadcast for the cut-in.
- Over a probing rule, a belief earns 0-0.6 % of cells, far below the 10 % bar.
- What would pay is a probe in A's rule set (a fixed rule) and an intent message (B announces yield
  or hold, a C7-style convention), which takes most of the remaining 2-3 %.
- P1's ablation is reshaped accordingly: stock, then stock plus probe, then stock plus probe plus an
  intent message.

### C7. Conventions

A convention picks one equilibrium for every car: a priority rule, a courtesy band in which B
pre-yields, a V2X negotiation. Each is played in the arena as a correlation device over C2's
rules, and measured by the part of the price of stability it recovers. This says whether
cooperation needs a convention or a new layer on the car, and feeds D6.

**Results (2026-11-15).** Commits ef0ff33, e416e19, efe5407. Data: NAS `logs/2026 SSv2 on
CARLA/2026-11-15 coop C7 conventions/`.

**Tie-breaks at the crossing and the narrow passage.**
- The priority car is B: the right-hand rule at the crossing, the free side at the narrow passage.
- Each convention runs on top of every rule; the near-tie band is |Δt| ≤ 0.3 s.
- The restart-crash share is counted over all rule pairs, among runs where both cars stopped.
- Kinematic numbers, with the arena in brackets:

| convention | restart crash, crossing | restart crash, narrow | gap recovered | self-enforcing |
|---|---|---|---|---|
| none | 22 % (25-27 %) | 54 % (38-39 %) | 0 | - |
| a fault for the priority-breaker, binding even after rest | 22 % | 54 % | crossing +0.50 of a tiny gap, narrow -0.16 | 100 % |
| **binding priority in behaviour** | **0 % (0 %)** | **0 % (0 %)** | narrow +0.99 (0.69-0.75), crossing +0.38 | 100 % |
| a signalled fair coin | 0 % (0-0.1 %) | 1 % (6 %) | narrow +0.98 (0.54-0.68), crossing -0.31 | 86-96 % |
| staggered patience (the right car waits shorter) | 50 % (37 %) | 36 % (26-27 %) | narrow +0.99 in equilibrium, but mean cost over all pairs 10-12 | 95-100 % |

- **Binding priority ends every restart crash**, in both models. It costs about +0.3 s of wait per
  car at the crossing and +1.0 s at the narrow passage. A fault alone only re-balances the war of
  attrition's mixed equilibria; behaviour is what selects.
- **The coin's residual crashes** come from a committed car restarting in the 0.5 s before the flash
  is seen.
- **Staggered patience fails:** a rule that does not wait collides with the right car's short
  patience.
- **D2 on outcomes:** 95 / 91 % at the crossing (10 / 14 m/s), 91 / 90 % at the narrow passage
  (W_n 6 / 30 m).

**The zipper under congestion.** A lane drop fed on both lanes: 20 + 20 cars at 12 m/s, mean headways
1.2 and 1.5 s (about twice one lane's capacity). Arena first, kinematic second:

| convention | throughput (veh/min) | maximum queue (m) | notes |
|---|---|---|---|
| zipper, rigid one-for-one | 26-27 / 31 | **125-135** / 121-140 | 3-6 forced stops per run |
| early merge | 28-32 / 31-34 | 242-315 / 111-135 | lane 2 waits 28-38 s, lane 1 none |
| zipper with lane 2's best reply | 26-35 / 31-35 | 128-250 / 120-178 | |
| yield at the end | 22 / 26-27 | 129-144 / 138-144 | slowest; splits delay equally (23-34 s per lane) |
| none | about 30-32 | 230-346 / 178-234 | 2-3.6 merge contacts per run |

- **Throughput stays at the bottleneck's ~30 veh/min under every convention:** the claimed
  throughput gain does not appear.
- **The zipper's shorter queue appears only against early merge, and only in the arena.** In the
  kinematic model, early merge cannot execute in dense traffic.
- **Every convention brings merge contacts down** to 0-0.75 per run, from 2-3.6 without one.
- Caveats:
  - S3's pair rules needed three changes for dense streams: a cut-in brake for lane-1 cars,
    go-on-yield only without overlap, and a 2.5 m gap ahead of a lag car at rest.
  - The arena's lane-2 tail rear-ends itself when it merges all at once at low speed (3.5-4.75 per
    run), counted apart from merge contacts. It flatters some conventions' delay.

**Conventions to carry, decided (2026-11-15):**
- At a tie: **binding priority as behaviour**, applied only within a conflict window (|Δt| ≲ 1 s).
  It needs no message.
- Where no priority exists: a **signalled coin as a V2X message**, sent when the first car stops so
  it lands before either may restart. That variant is still to be tested.
- **Never staggered patience** on its own.
- At a congested lane drop: **the one-for-one zipper**, for the shortest queue and split delay, not
  for throughput.

**For D6:** the large losses at ties and merges are coordination failures that a convention fixes
without beliefs (0.97-0.99 of the narrow passage's gap). C7 gives no reason to build a belief layer
for these relations. D6 rests on C6's hidden-capability results.

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
| 2 real stack | E3 Autoware knobs | E | which Autoware 1.5 parameters move B between yield and hold (obstacle cruise's idling time and RSS accelerations, lane change's rear-car assumptions, RTC), and whether they change at run time | knob-to-behaviour table. Done (swb 118; E3 above) |
| | D3 B's strategies on Autoware | D | what is a strategy of a real stack? | stock Autoware as one fixed strategy first (A's best reply to it); knob settings as strategies where >= 2 settings move outcomes measurably; an overlay switch (042) only if neither works. Decided 2026-11-15: stock as one strategy, plus knob settings (yield) as strategies; no overlay |
| | C3a, C3b | B | B Autoware with ground-truth objects, then with perception | real-stack maps |
| | D4 where the stack leaves the model | D | residual map against the Autoware-like rule | refit the imitation and rerun C2a; or perception-caused (C3a against C3b): update the perception model; or record as a finding |
| | C3c | B | A by RTC, then both Autoware | cross-play of two configurations |
| 3 explain | C4 fit | B | kappa rules, SVO, potential, level-k; identifiability | fitted models, residuals |
| | D5 which explanation | D | which model the strategy layer carries | held-out payoff error and readability; default the threshold (kappa) rules. Decided 2026-11-15: κ rules on the car; the potential model (α per class) to predict others |
| | E4 heavier car | E | is a truck or bus spec available or buildable in `sim_vehicle_spec` | spec or none |
| | C5 car models | B | pairs from the catalog, measured capabilities | maps per pair; the commitment hypothesis |
| | C6 hidden capability | B | beliefs, type confusion, misbelief regret, value of information | regret and VoI maps |
| | C7 conventions | B | a priority rule, a courtesy band for B, a V2X negotiation; each as a correlation device | the part of PoS each recovers |
| | C8 protocols per band | B | each band's convention as a decentralised mutual-exclusion protocol (design, "The protocol frame"), with no V2X: a rulebook/STL spec; timed-automata check of safety and liveness, including two cars reading the role differently (fail safe, 3b); incentives (obedience, evolutionary stability); BAR mixed traffic (stock, convention, deviating); then the arena and swb over the deployment test matrix (vehicles, geometry, perception, V2X adoption, others). U3's four-way stop first | per band: a verified convention, never worse than stock against non-compliers |
| | D6 build the strategy layer? | D | is acting on beliefs worth a node on the car? | build if regret or VoI reaches >= 10 % of stock's welfare loss in a non-trivial share of contexts; else stop at the findings, or recommend a capability broadcast only. Decided 2026-11-15 by its criterion, confirmed by the user: no belief node; a probe rule and an intent message instead |
| 4 vehicle | P1 implementation (carried out by [078](078-conflict-zone-protocol.md)) | B | one library for the workflow (design, "The per-vehicle workflow": zones, relation, intervals, band, role, convention, probe, contracts, record), used by both the arena and an `interaction_strategy` node beside Autoware (RTC, velocity limit); the convention table as per-jurisdiction config; no belief node | the node runs on swb with the shield; unit and property tests (a total order, no wait-for cycle) |
| | P1 evaluation | B | replace "Assumed parameters" by measurements: the role-time error from Autoware tracks against ground truth, the yield classifier, α per class, the critical gap, the reach contract, capability bounds; tuned on one seed set, read on a pre-registered other | each parameter measured, with its held-out error |
| | P1 test runs | B | the scenario walkthroughs (12) × the deployment test matrix, in the arena then on swb; ablation with no V2X: stock / + convention and probe / + learned parameters; intent messages last at 0/30/100 % | contact, TTC/PET/clearance distributions, deadlock and time to clear, welfare against stock, flips, the learned share of decisions |
| | D7 deployable? | D | | never worse than stock on contact and on the distributions of 2D TTC, PET and clearance (①), a significant welfare gain, within the message-rate budget: propose it; else keep it as research |
| 5 cases | E5 left-turn signals | E | can SSv2 give the left turn and the oncoming queue green together on Town05/10 | yes or a workaround |
| | U1, U2 | B | left turn against a queue; roundabout entry (Town03); each kinematic, arena, swb | maps |
| | E6 4-way stop site | E | lights switched off on a Town05 junction, or an unsignalled junction | site. Done: Town05 j979 (j1427) |
| | D8 4-way stop | D | | site found: U3; else skip. Decided 2026-11-15: U3 on Town05 j979 |
| | E7 merge site | E | a lane drop on Town03/05 | site or none. Done: none on Autoware-mapped towns; Town03 j1901 (45° Y merge), Town06 road 48 (no Lanelet2) |
| | N1 conflict-set sizes | E | how many cars are in each car's conflict set, per slot, in the arena's and SSv2's multi-car scenes (and highD if it is at hand) | the size distribution; the literature's 3.2 mean / 7 max (survey, N cars) checked on ours. Done (20cef0f; N1, N2 above): arena 1.0 / 1, SSv2 benches 1.19 / 4 (mean / max when > 0) |
| | N2 composition | B | three-car arena games (a cut-in between B and its follower; a cut-in with a lead ahead of A) against the pair terms composed per measure (risk max, delay max-plus, braking max) | the residual $r$ per context with its bootstrap interval. Done: 81 % of car-cells, 64 % of contexts within tolerance; residual systematic |
| | D9 pairs enough? | D | does the composed model predict three cars | $r$ within the D2 tolerance in >= 90 % of contexts: pairs suffice, N grows only by composition; systematic: N3; large and irregular: those contexts stay explicit games, try N = 4 there only. Decided 2026-11-14: not enough, systematic: N3, with the A-B outcome as a class and the queue and lead as predictors Final after N3: composition holds for queues ≤ 4 cars; a closing lead within ~2 s and 5+ queues stay explicit games Met after N3 round 4 (2026-11-15): 92 % of held-out contexts |
| | N3 residual model | B | a permutation-invariant residual over the slots, fitted on three-car runs, tested on four | held-out error against the composed model's. Done in two rounds (9ed8d89, 43b45a3, d17fdc3): held out 82 % of contexts; composition for queues ≤ 4 cars, the closing lead and 5+ queues explicit |
| | N4 far field | B | a long merge or ring with 10-50 cars: near field exact, the rest as density and mean speed, against all cars explicit, at increasing N | the mean-field error against N, and where it falls below the tolerance |
| | N5 swb check | B | the merge on a found site (E7), or a Town04 map (031) only if N2-N4 call for one | D2's agreement for three cars. (a) kinematic and arena done 2026-11-15 (Y merge = S2's game; D2 at v 10 accepted for outcomes and classes); swb check done 2026-11-15 (three cars on Town03 j1901: outcomes 15 of 15, payoffs 68 %; D2 accepted for outcomes and classes) |
| 6 scenarios | S0 general utility | B | route progress, 2D TTC, PET, clearance, lateral comfort (`docs/design/cooperative-driving.md` ⓪, ①) in `measures.py` and `cutin_game.utility`; C2b, C3a, C3b and the double oracle re-scored offline from the recorded runs; prerequisite for S1-S6 and for N2's pair risk $\rho_{ij}$ | done (22741ab; S0 above): classes and equilibria unchanged; D2 92 %; 14/-3 is a 0.1-8 cm footprint touch, not a near miss, so it still scores as contact (`pen` kept beside it) |
| | S1-S6 | B | cut-out; unsignalled crossing; lane drop or on-ramp; narrow passage; left turn in the arena and swb; overtaking via the oncoming lane: each as a passage game (relation, Δt, speeds, right of way), kinematic, arena, swb light, swb Autoware | equilibrium maps per relation; Autoware's departures. S1 kinematic done (01bc891), S2 kinematic and arena done (3163cce; D2 not met at the tie); sections above |

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
