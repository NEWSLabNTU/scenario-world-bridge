# 078: The conflict-zone protocol, built, evaluated and tested

## Goal

User, 2026-11-15: "Document the design and phase docs for build our conflict zone protocol
design. You can add evaluation and decision points."

Build the strategy that roadmap 074 found works, put it on Autoware, and decide from measurements
whether it ships. The build spec is [design/conflict-zone-protocol.md](../design/conflict-zone-protocol.md);
the theory and evidence [design/cooperative-driving.md](../design/cooperative-driving.md); the
study [074](074-cooperation-study.md), whose P1 rows this phase carries out.

**Status**: Phases 0-1 done 2026-11-15. G0: **only the unsignalled crossing enters the table**;
the all-way stop, the narrow passage and the lane drop stay stock. G1 passed for the crossing
(the node drives that relation only). Next: Phase 2 on the crossing, starting with RTC on a real
conflict on swb and EV4.

## Inputs, already in place

- 074's findings: the bands, the conventions (C7, U3), D6 (no belief node), D9 (pairs compose),
  the arena trusted per relation (D2), the arena with brake to rest (4fcaa8d), the kinematic
  defaults (e995a7b).
- The stack fixes 074 needed: CARLA's friction reset (swb_carla_world d56e99e, acb 23ab103, acb's
  drift exit 41705ca), full CenterPoint as `centerpoint` (autoware_sim_vehicle d249ebc), lidar
  intensity from the spec on CARLA (fe63f5d), TensorRT 10.8 on host 133.
- The Autoware 2.0.0 profile (c2a2879), for the second autopilot release in the test runs.
- The lab's neighbour projects: UWB and V2X leases (coop 118), signal control (traffic 241).

## Phases

Each phase ends at a gate. A gate is a decision with a criterion stated before the runs; the
evaluation points (EV) feed it. The user decides the deployment gate (G4).

### Phase 0: verify the conventions (074's C8)

| step | what | output |
|---|---|---|
| V1 | The four-way stop as a formal model, 2-4 cars, each reading the other's stop time within ±e; rules FCFS, FCFS + right-before-left, the fail-safe rule (margin, both-wait, second round), the map's total order, mixed pairs | safety and liveness per rule; counterexamples; margin(e); the worst wait |
| V2 | The fail-safe rule in the kinematic model and the arena, self-play and against every other rule, U3's contexts | contacts, deadlocks, cost away from ties |
| V3 | A first slice of the deployment matrix: 60° and 120° junctions, noise ×2, a bus | how margin(e) and the results move |
| V4 | The same for the other rows of the convention table (crossing, narrow passage, lane drop), formal model first | one verified row per relation |

**G0, which conventions enter the table.** A row enters when the formal model shows safety and
liveness for 2-4 cars within ±e, and the arena shows no contact in self-play, no deadlock, and
never worse than stock against non-compliers. A row that fails stays stock for its relation.

**Phase 0 results, the all-way stop (C8, 2026-11-15; 074 C8).**
- V1: the hard tie threshold (U3's fcfsr) is unsafe for any e > 0. (e), the fail-safe rule with the
  map's order as last resort, is safe and live for 2-4 cars; m = e, H2 ≥ e + 1.2 s. e99 ≈ 0.25 s
  centred in the arena.
- V2: (e) has 0 contacts and 0 deadlocks in self-play, but is worse than stock in 4 of 12 BAR cells:
  stock Autoware's "a car at rest stays" races (e)'s second-round go.
- V3: skew is harmless; noise ×2 flips the guards (3/27 contacts, 1 deadlock); a heavy car needs its
  own zone interval.

**G0 ruling, all-way stop (manager, 2026-11-15): not passed yet; deferred to V2b.** The formal and
self-play criteria pass, "never worse than stock" does not, and the criterion stays as stated.
The fixes are design changes, so they go into the build, not into another study round:
- **guard hysteresis** (latch a go for the launch dead time; the zone guard and the "moved" test
  filtered over 2-3 samples), the design's latch extended to the guards;
- **zone intervals from the car's own capability** (launch, length, braking from the spec or class);
- **the legible probe before a round-2 go** (creep about 0.5 m, then watch), against stock's
  static-at-rest.

**V2b**, after I3: the arena adapter with these three reruns C8's BAR table and noise ×2. G0 for
the all-way stop is decided there, by the same criterion. If it still fails against stock, the
all-way stop stays stock where the other car may be stock (T4 measures how often that is), and (e)
applies between compliant cars.

**V4 results, the other rows (133's subagent, 2026-11-15; 9f214f6, a6ebb52, d81460d; NAS `logs/2026
SSv2 on CARLA/2026-11-15 078 V4 rows/`).**
- **Formal** (`czp_rows_check.py`: the built library through the adapter, the cars' lags, no
  post-start shield; exhaustive over a family of perception adversaries per context: delays, a
  constant ±e or 0 error, square waves): at e 0.25 every row is safe and live (crossing 2 and 3
  cars, worst wait 7.0 / 10.4 s; narrow passage 2 cars and a 3-car queue, 16.1 s; lane drop 2 cars).
  At e 0.5, past the margin, the crossing and the lane drop fail, the check's sanity case.
- The checker and the arena forced library changes: the entry band strict at m − e (176 unsafe
  narrow runs before); an ambiguous-band rule for role-first goes; `rest_tie` (both at rest binds
  the role, the winner goes after the 3 s fresh-rest guard); `claim` at the narrow passage, with
  round two by the obstruction side; shadowed queues; bystanders counted for the quiet epoch (3-car
  crossing: 45 unsafe runs before); a `gap` band mode for the lane drop.
- **Arena BAR** (5 seeds; stock = the study rule nearest stock Autoware: crossing A t1.5p5 / B
  yb2p5, narrow t1.5p5, lane drop A e1.2d / B hold):

  | row | self-play | worse than stock | G0 |
  |---|---|---|---|
  | **crossing** | 0 contacts, 0 deadlocks of 90 | **0 of 12 cells** | **pass** |
  | narrow passage, W_n 30 / 6 | 0 / 0 of 70 | 4 of 14 at both: czp as B (free side) goes by its role after the guard, and gap-test rules read it as static and restart into it (the legible probe cut the worst cell 20 → 6, not to 0) | fail |
  | lane drop | 0 / 0 of 80 | 3 of 11 (czp as B against drop rules' own run-outs); czp as A: 7 not merged against c20 (stock 38) | fail |

**G0 rulings (manager, 2026-11-15).** The criterion as stated:
- **The unsignalled crossing enters the table.**
- **The narrow passage and the lane drop stay stock.** Their failures share the all-way stop's
  cause: a car that waits safely and then goes by its role is read as static by a gap-test rule,
  which restarts into it. The lane drop also deadlocks, 7 times, where stock deadlocks 38.
- **Caveat:** "stock" in the arena is a study rule chosen to be near Autoware. G3 decides on swb
  against Autoware itself.
- **What 074 learns:** with no V2X, a convention pays only where the waiting car is never asked to
  go first from rest against a car that cannot read it. That is true of the crossing (nearer-first,
  both moving) and of none of the other three rows. This is principle 3's limit, and it should be
  stated in the design.

### Phase 1: implementation

| step | what | output |
|---|---|---|
| I1 | `czp-zones`: zones from Lanelet2 (overlaps, approaches, `order`, `priority`, widths, exit box) for Town01, Town03, Town05, Town10HD and the NTU twin map | `zones.yaml` per map, with a plot per junction |
| I2 | `czp` library: steps 2-10 as pure functions over the build design's data model | the library, unit and property tests |
| I3 | Arena adapter: the arena's car decided by `czp` in place of each study's rules | parity runs |
| I4 | ROS adapter: the `interaction_strategy` node in the ego domain, through RTC and the velocity limit; a profile switch `interaction_strategy:=true`, off by default; RTC in manual-cooperate mode for the modules it drives | the node on swb |
| I5 | Contracts and fallback, the decision record, the config files (`conventions/generic.yaml`, `params.yaml`, `contracts.yaml`) | |

**EV1, parity.** The arena adapter reproduces U3, C7, S2, S3 and S4 (outcomes and classes) within
D2's tolerance; in the near-tie band (|Δt| ≤ 0.1 s) parity is on the contact share, not the class
(074, S2's kinematic follow-up). **EV2, stock parity.** With the node on but every zone `clear`, and with the node
off, an ego run matches stock Autoware (same route time within noise, no RTC command issued).
**EV3, cost.** Cycle time p99 ≤ 20 ms on host 133 with 20 objects.

**Phase 1 results (133's subagent, 2026-11-15; 9bca773..170b03f; NAS `logs/2026 SSv2 on
CARLA/2026-11-15 078 P1 czp/`).**
- **I1-I5 built.** `src/czp` (ament_python): the library with no ROS import, `czp-zones`, the
  ROS-free `strategy` and the rclpy `node`, config in the package. 35 unit and property tests
  under `just test`; the property tests found two bugs (rounds 2-3 from one common quiet epoch among
  every car at rest; right-before-left between opposite arms falls back to the map order).
  Zones: Town01 93, Town03 354, Town05 640, Town10HD 123 (114 signalled), NTU twin 281; no map has
  right_of_way or stop_sign elements, so priority is equal everywhere. The switch is
  `interaction_strategy:=true` in `swb_launch/vehicle_agent.launch.xml`; RTC goes to manual only
  while a non-stock decision is held; the velocity limit goes through the selector's
  `max_velocity_candidates`.
- **EV1:** U3 kinematic: czp0 (no V2b guards) 571/572 outcomes against C8's (e), czp 518/572 with
  classes 44/44; U3 arena against kinematic 94.9 / 91.9 %. S2 against C7's bind 46/48 outcomes,
  41/48 classes (W against B at |Δt| 0.2-0.3); S4 63/64. **S3: 435/450 outcomes but 10 contacts
  and 17 forced stops** that S3's equilibria lack (dv +3, the drop at the deadline). Arena side for
  S2-S4 not done.
- **EV2 passes:** town01_ego_drive, 4 runs on and 4 off, all pass; the node issued 0 RTC commands,
  0 mode switches, 0 limits. **EV3 passes:** bench p99 2.1-8.7 ms with 20 objects; live 18-21 ms
  after a first 52 ms on a loaded host (the self contract falls back to stock).
- **V2b:** C8's three fixes alone made it worse (8 of 12 cells at v6, mostly deadlocks); five more
  were needed (an averaged rest position with filtered in-zone and at-rest tests; a watch that keeps
  rolling at 0.4 m/s; the last resort before "another car wins round 2"; a fresh-rest guard; a
  probe before a first go while another car still approaches). Worse-than-stock cells, czp / C8's
  (e): v6 3 / 5, v10 4 / 5, noise ×2 m 0.3 3 / 7, m 0.45 1 / 3. Self-play: 0 contacts everywhere;
  0 deadlocks at noise ×1, **1 and 2 of 27 at noise ×2**. The remaining worse cells: against
  stop1y and aggr near ties (1-2 contacts against stock's 0), czp as B against fcfs at v10 (5
  against 1), and at noise ×2 deadlocks against yield and stop1y. Self-play welfare (near/away), v6:
  czp −2.00/−1.61, (e) −1.92/−1.26, stock yield −3.66/−0.43.

**G0 ruling, all-way stop (manager, 2026-11-15): not passed. The rule stated above applies.**
- With no V2X a car cannot know that the other car complies, so **the all-way stop stays stock**
  in the no-V2X deployment. The czp row is kept for pairs known to comply, which T4 (V2X adoption)
  tests.
- Even between compliant cars the row needs a perception contract: e at the arena's level
  (e99 ≈ 0.25 s). At noise ×2 it deadlocks 1-2 times in 27.
- What 074 learns from it: at the all-way stop a protocol that waits safely loses to stock
  Autoware's own restarts. Near ties, the cost lies in mixing with a car that treats a waiting car
  as static. The convention alone does not fix this.

**Phase 1, second round (V4's subagent).** S3 fixed in the kinematic model (0 contacts, was 10;
12 forced stops, was 17; 435/450 outcomes; stock's zipper alone has 19 contacts). Arena EV1:
S2 167/168, S4 149/150 and 145/150, S3 202/208; near-tie shares within tolerance. Kinematic S4
against C7's bind falls to 52/64 outcomes under the claim rule; that relation stays stock anyway. A
**watchdog** (`czp.watchdog`, started with the node): the node publishes a heartbeat every
cycle; after 0.5 s without one, the watchdog calls Autoware's own `enable_auto_mode` for every held
module and clears the limit. Back in auto 0.52 s after a SIGKILL or a hang.

**G1 ruling (manager, 2026-11-15): passed for the crossing.** EV1 (S2 kinematic 46/48, arena
167/168), EV2, EV3, property tests and the watchdog all pass. Per G1's rule the node drives only
the crossing relation; every other zone kind is `stock`. Not yet tested: RTC acting on a real
conflict. It is Phase 2's first step, before EV4.

**G1, ready to measure.** EV1, EV2 and EV3 pass and the property tests are green. Else: fix, or
narrow the relations the node drives to those that pass.

### Phase 2: evaluation of the assumed parameters

Each parameter starts at the build design's assumed value. Each is measured on one seed set and
read on a second, pre-registered held-out set; the held-out error is reported with it.

| step | parameter | how | decision it feeds |
|---|---|---|---|
| EV4 | role-time error e (stop and arrival times) | Autoware's tracks against swb's ground truth, by distance and class | the tie margin 2e; whether `stopped_first` is usable at all at the measured e |
| EV5 | the yield test | probe runs: Autoware, the light agents and the arena's B as the other car | the probe's thresholds and time |
| EV6 | α per class | held-out yield rates per class (car, truck, two-wheeler) | the others' response model |
| EV7 | the critical gap and the reach contract | left turns and roundabout entries on swb, full CenterPoint | `gap_ge_critical`; when the turn is refused |
| EV8 | braking bounds | E4-style runs per blueprint, dry and wet | the intervals' latest arrival |
| EV9 | radar reach (option) | CARLA's radar into Autoware's radar fusion, the left turn's oncoming queue | **D-radar**: add radar to the ego when it moves the reach contract from failing to passing |
| EV10 | UWB (option) | coop 118's chip model (seps_uwb, simple mode first) in the arena; the presence bound and the attestation gate | **D-uwb**: adopt UWB when it raises a measure (fewer refused gaps, shorter waits) and never lowers safety |
| EV11 | stop instant from UWB range rate (idea) | both equipped cars at a stop, in the arena | **D-shared-stop**: make `stopped_first` a shared fact for equipped pairs when both cars agree within the margin in ≥ 99 % |

**G2, parameters fixed.** Every parameter used by a verified row has a measured value with a
held-out error within its tolerance (stated per parameter in its run's report). A parameter
that cannot be measured to tolerance takes its conservative bound, and the report says so.

### Phase 3: test runs

| step | what | measures |
|---|---|---|
| T1 | the twelve scenario walkthroughs in the arena, over the deployment matrix: vehicles (car, van, bus, truck, scooter), geometry (skewed, curved, narrowing, sloped, occluded), perception (ground truth, the detectors, latency added), others (stock, aggressive, logit and level-k) | contact; TTC, PET and clearance distributions; deadlock and time to clear; welfare against stock; flips; the learned share |
| T2 | swb with Autoware, no V2X, ablation: stock; + convention and probe; + learned parameters. On Town05 j979, the S2/S5 junctions, Town03's Y merge, a lane drop | the same, per arm |
| T3 | mixed traffic as BAR: our car with stock Autoware cars, with a deviating (RL-audited) car, with a car that never stops | R4: never worse than stock |
| T4 | V2X intent last: 0, 30 and 100 % adoption, loss and delay, false intent | the value of the message over motion |
| T5 | the NTU twin map: its junctions through `czp-zones`, the same suite | the same, on real geometry |
| T6 | Autoware 2.0.0 and Isaac (after the Isaac lidar fix): the T2 suite | the same, per autopilot release and simulator |
| T7 | the RL audit: a search for profitable deviations against the final table | exploitability per row |

**EV12, the deployment criterion** (074's D7, stated before T2): on every T2 and T3 cell, contact
no higher than stock (a one-sided test at 95 %); the TTC, PET and clearance distributions no worse
than stock (a paired test with TOST at ±2 %); a significant welfare gain on the band cells; no
deadlock; the cycle within budget.

**G3, per relation.** For each relation: **adopt** (EV12 passes), **keep stock** (no significant
gain, or a failure), or **research** (a gain with an open safety question). A relation can be
adopted while another stays stock.

### Phase 4: the decision

**G4, deployable? (the user).** On the evidence of G0-G3: propose the node with the adopted
relations as an option of the Autoware profile (`interaction_strategy:=true`), keep it as
research, or stop. If proposed: the docs (076's reader-oriented docs), the Taiwan convention
file, and a hardware trial plan are the next phase.

## Decision points, in one list

| id | when | question | criterion | who |
|---|---|---|---|---|
| G0 | after V1-V4 | which convention rows enter the table | formal safety and liveness within ±e; arena: no contact in self-play, no deadlock, never worse than stock | manager |
| G1 | after I1-I5 | ready to measure | EV1 parity, EV2 stock parity, EV3 cost, property tests | manager |
| D-radar | EV9 | add radar to the ego | the reach contract passes with it and fails without | user |
| D-uwb | EV10 | adopt UWB as an option | a measure rises, safety never falls | user |
| D-shared-stop | EV11 | UWB stop instant as a shared fact | agreement within the margin ≥ 99 % | manager |
| G2 | after EV4-EV11 | parameters fixed | held-out errors within tolerance, else conservative bounds | manager |
| G3 | after T1-T7 | adopt, keep stock or research, per relation | EV12 | manager, reported to the user |
| G4 | after G3 | deployable | the evidence of G0-G3 | user |

## Order and parallel work

Phase 0 is running. I1-I2 can start on the conventions C7 and U3 already found (FCFS +
right-before-left with the map order); G0 only adds rows. EV4-EV8 need I4 (the node logs the
inputs they measure) but not G1. T1 can start once G1 passes on the arena side; T2-T6 need G1
and G2.

## Risks

- **RTC's coverage.** The intersection module's RTC may not expose every decision the table makes
  (a second round, a creep). Mitigation: the velocity limit, or a small Autoware overlay package.
- **e may be too large.** If EV4's role-time error at 30 m is near the tie band itself,
  `stopped_first` is unusable and the all-way stop falls back to `map_order` alone. That is safe
  and live, but ignores arrival order. The report must say what that costs.
- **Arena vs swb.** D2 holds per relation, not everywhere (S4's payoffs swing ±15 points between
  reruns). G3 uses swb (T2), not the arena alone.
- **The stack's own failures** (principle 1) can mask the node's effect: T2 runs on the fixed stack
  only, and its stock arm is the baseline.

## Progress and rulings

- 2026-11-15: phase doc written. Phase 0 (C8) running on host 133. P1 implementation (I1-I5)
  assigned to 133's subagent, to start when C8 reports.
- 2026-11-15: C8 reported (V1-V3 for the all-way stop). G0 deferred to V2b (above). P1 started
  on 133's subagent with the three C8 fixes in I2. Near-tie parity on contact share (EV1).
- 2026-11-15: P1 built (above). G0 for the all-way stop not passed: stock with no V2X. G1 not
  passed: S3's contacts and arena EV1 open; per G1's rule the node drives no relation until a row
  passes G0 and its EV1. Next on 133's subagent: V4 (crossing, narrow passage, lane drop), S3's
  fix, arena EV1 for S2-S4, the node's watchdog.
- 2026-11-15: V4 and the second round reported. G0: crossing in; narrow passage and lane drop stock.
  G1 passed for the crossing. Phase 2 on the crossing next: a crossing light agent, RTC on a real
  conflict on swb, then EV4.
