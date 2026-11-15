# 078: The conflict-zone protocol, built, evaluated and tested

## Goal

User, 2026-11-15: "Document the design and phase docs for build our conflict zone protocol
design. You can add evaluation and decision points."

Build the strategy that roadmap 074 found works, put it on Autoware, and decide from measurements
whether it ships. The build spec is [design/conflict-zone-protocol.md](../design/conflict-zone-protocol.md);
the theory and evidence [design/cooperative-driving.md](../design/cooperative-driving.md); the
study [074](074-cooperation-study.md), whose P1 rows this phase carries out.

**Status**: Phase 0's four-way stop (V1-V3) done 2026-11-15; G0 for the all-way stop deferred to a
re-check on the built library (V2b); V4 open. Phase 1 starting.

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
