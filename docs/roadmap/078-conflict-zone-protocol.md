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
(the node drives that relation only). Phase 2 on the crossing (2026-11-16): with a committed go that gives way to
a seen start, G0 for the crossing passes again; e 0.55 s confirmed on held-out runs; a base_link pose
bug fixed. Phase 2c: a second own-rest bug found; G0 at a fresh seed failed by one cell; the guards
must not take the rest-stamp correction. Next: G0 on three fresh seed sets, then the swb rerun.

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

**EV8 done (swb 118, 9e9d4f0; NAS `logs/2026 SSv2 on CARLA/2026-11-15 078 EV8 braking/`; 480 runs,
39 CARLA blueprints + the spec Model 3).** The class bound is q0.1 of effective deceleration, dead
time included:

| class | dry | wet | assumed (dry / wet) | params.yaml `b_bound` |
|---|---|---|---|---|
| car | 4.28 (2 of 23 below 4.0: a2 3.72, microlino 3.83) | 3.51 | 4 / 2.4 | 4.0, kept |
| truck and van | 2.56 (firetruck 2.35) | 2.43 | 3 / 1.8 | **2.3** |
| bus | 5.69 (n = 1) | 5.09 | 3 / 1.8 | 3.0, kept |
| two-wheeler | not physical (CARLA's bikes are 4-wheel bodies) | | 4 / 2.4 | 4.0, assumed |

- CARLA 0.9.16's tyre friction ignores wetness: every wet preset brakes the same. Wet is therefore
  friction × 0.6. The lowest measured wet/dry ratio is 0.60, so `wet_factor` 0.6 stands.
- Stock blueprints ship friction 3.5 and are limited by brake torque.
- The spec Model 3: 8.37 m/s² dry, 6.57 with its command lags, 4.64 wet with lags.
- Dead time is about 0.05 s (one tick), up to 0.23 s when the gearbox hunts.
- `b_max` (a ceiling on others' braking) is exceeded by stock CARLA actors: peaks car 13.1, truck
  11.7. Raise it only if czp has to watch CARLA-physics actors.
- The manager set `b_bound` and the source tags (truck 2.3, `b_comfort` 2.0 to stay below it); the
  czp tests pass.

**Phase 2 on the crossing: RTC on a real conflict, EV4, EV5 (133's subagent, 2026-11-15/16;
039cb1a, d043485, 291816f; NAS `logs/2026 SSv2 on CARLA/2026-11-15 078 P2 crossing RTC and e/`).**
Town05 j979 (`cross_unsignalled`). The ego is Autoware 1.5.0 with full CenterPoint and spec
physics. A is a light agent (`crossing_agent.py`) from the ego's left, paced on the ego's measured
acceleration: Δt at the lock is within 0.03 s of the cell in all 58 valid runs. Rules t1.5p5, go and
tinfp5; node on and off. 0 contacts.
- **A show-stopper found and fixed:** the node and the watchdog called `enable_auto_mode` with the
  wrong service type (`AutoModeWithModule`; rtc_interface serves `tier4_rtc_msgs/srv/AutoMode`), so
  no module ever went back to auto. A cooperate command clears auto mode by itself, so the ego
  stood at its line for 45 s on the last deny. P1's EV2 and watchdog checks passed only because
  they saw no conflict, or a fake of the same wrong type. Rechecked with the real type: kill
  0.52 s, hang 0.49 s. Also fixed: the self contract flapped at a live cycle p50 of 18.6 ms (now
  the median of 10 cycles); cycle p50 is about 6 ms and p99 about 20 ms live.
- **RTC carries the node's decisions:** 114 of 114 commands shown by the module within 1 s; 42 of
  44 approves removed the stop factor; 50 of 70 denies restored one (the rest came too late:
  ego stopped, inside, or past the line); back in auto at the end in 26 of 26 runs. **Its gaps:**
  - no latch or commit;
  - an approve overrides the module's own collision check (7 approves while `safe` was false,
    after which safety rests on obstacle stop alone);
  - no creep (the velocity limit is the only way);
  - the status uuid persists past the line;
  - clear or contested readings at band edges toggle the mode: 216 switches in 26 runs.
- **The node is worse than stock in the band** (t1.5p5, seeds 0-3): minimum clearance 0.50 m
  against stock's 1.00, minimum TTC 0.74 s against 1.30; ego first 10 of 12 band runs against
  12; the ego clears the zone at 15.5 s against 11.5. Cause: a go from rest is not committed. Once
  the ego moves its band reads ambiguous, the node denies, and go and wait alternate every
  1-2 s until A's patience wins. With `commit: on_move` (6 runs) the chatter is gone (ego first
  6 of 6, clearance 1.01 m, TTC 1.32 s, PET 1.54 s), but the ego still clears about 1 s later
  than stock (13.3 s against 12.4).
- **EV4, pre-registered split** (seeds 0-1 fit, 2-3 held out):

  | bin | held-out e99 (centred) | bias |
  |---|---|---|
  | 0-10 m | 0.57 s | −0.24 s |
  | 10-20 m, tracks ≥ 0.5 s old | 0.23 s | −0.24 s |
  | pooled 0-30 m, tracks ≥ 0.5 s old | **0.53 s** (cover 98 %) | −0.24 s |

  The bias is the tracker's speed lag while A brakes (+0.78 m/s). **A is never tracked beyond
  25 m:** first tracked at a median 21.6 m from the zone in 40 of 40 runs (corner occlusion, as at
  the Y merge), so the node reads the band only about 2.5 s before the zone. **The stop instant from
  tracks comes 0.7 s late** (median; 0.45-3.55 s).
- **EV5:** the yield test agrees with A's true acceleration in 52 of 52 runs (lag 0.65-0.8 s), and
  with what A did in 43 of 52. The 9 misses read a brief safety brake or a patience start as a
  yield. A non-yielding A was nearly absent, so this is weak evidence.

**Rulings (manager, 2026-11-16).**
- **The crossing row needs a committed go** (`commit: on_move`, or a latch until the ego is
  inside). V4 verified the row without it, so **G0 for the crossing is reopened**: formal and arena
  recheck with the commit, at the measured e.
- **e (G2, provisional): e = 0.55 s with bias −0.24 s** (EV4's pooled held-out e99, rounded up),
  replacing the assumed 0.25 s, until the held-out set is larger. `stopped_first` and `rest_tie`
  timing on tracks is unusable at this e (stop instant +0.7 s late). The crossing's arrival-order
  role must therefore hold at m = 0.55 s: the G0 recheck uses it.
- **A mode hysteresis** goes into the strategy, so that readings at band edges stop toggling RTC.
- **Occlusion is a contract, not a parameter:** with the other car first seen 22 m out, the node is
  stock until 2.5 s before the zone. The phantom at the occlusion edge (design step 3) and Autoware's
  own occlusion handling (swb 118) were the two candidate fixes; radar is EV9.
  **2026-11-16:** Autoware's occlusion handling is rejected as a default. At a statically occluded
  corner it creeps on every approach, even with no car there: +11.6 s on an empty junction, and A
  never goes first (074 N5). The phantom at the occlusion edge remains the candidate, and it must
  be checked against the same empty-junction cost.
- **G3 risk recorded:** until the go is committed, the node is worse than stock on clearance and TTC
  in the band.

**Phase 2b on the crossing: committed go, mode hysteresis, e 0.55 (133's subagent, 2026-11-16;
c05c85b, bcafc5a, f7bbeeb; NAS `logs/2026 SSv2 on CARLA/2026-11-16 078 P2 crossing commit/`).**
- **G0 recheck** (V4's criterion, 5 arena seeds):

  | variant | formal 2 / 3 cars at e 0.55 | arena self-play | cells worse than stock |
  |---|---|---|---|
  | plain `on_move` commit | safe, live (7.3 / 10.9 s worst wait) | 0 / 90 | **2 / 12** (a committed go cannot retract against a patience or gap-test restart; 1 / 12 at e 0.25 too) |
  | **commit + `commit_release: turn`** | safe, live (7.3 / 10.9 s) | 0 / 90 | **0 / 12**, 0 contacts in every czp cell; again 0 / 12 on fresh noise (`--seed 2`) |

  The release gives the committed go up when the other car is seen starting (off its line or in
  the zone) while the ego can still stop comfortably. Sanity case: e 0.8 fails formally, e 0.65 and
  0.7 hold. Cost of e 0.55 against V4's 0.25: the czp car's lost time is about +10 % (0.349 against
  0.318 s over the BAR cells).
- **Mode hysteresis:** `mode_hold` 1.5 s. While it holds, the node sends what the module's own `safe`
  says. Switches fell to 2.7 per run (54 in 20), from 8.3.
- **params:** `role_time_error_e` 0.55, `role_time_bias` −0.24 (reading − truth; the entry band
  reads A's arrival 0.24 s later).
- **swb, t1.5p5, seeds 0-3, node on and off, 40 valid runs, before the base_link fix:** 0 contacts.
  The node's minimum clearance and TTC are now stock's (1.00 m / 1.18 s against 1.01 / 1.19; last
  round 0.50 / 0.74). In the band, the node puts the ego first in 10 of 12 runs (stock 12) and
  clears at 14.8 s against 13.3. RTC: 70 of 70 commands shown, 16 of 16 approves effective, 44 of
  54 denies.
- **The two lost band runs were a node bug:** it read Autoware's `kinematic_state` pose (base_link,
  the rear axle) as the car's centre, so the ego's front read 1.38 m short. Its rest at the line
  never registered and `rest_tie` never bound. Fixed in f7bbeeb (`own_base_to_centre` 1.416 m, from
  the spec); an offline replay of that bag binds `rest_tie`. The bug likely affected the first
  Phase 2 round too. A second effect: Autoware takes about 1.4 s from ACTIVATE at rest to motion,
  longer than the 0.5 s go latch.
- **EV4 on the new held-out runs** (pre-registered: fit = 2026-11-15 seeds 0-1, held out = all 40
  runs of this round; tracks ≥ 0.5 s old): pooled 0-30 m held-out e99 **0.56 s**, bias −0.25 s; e 0.55
  covers 98.8 %. **e 0.55 s is confirmed** (G2 for e, manager). Without the age filter, the 20-30 m
  bin is still track start-up (cover 66 %); the node must not read a track younger than 0.5 s.
- **Not run:** the node arm after the base_link fix, EV5 with the held-speed A, the right-hand
  deny path (1 of 44 runs done).

**Rulings (manager, 2026-11-16).**
- **G0 for the crossing: passed again**, with `commit_release: turn`. The variant was chosen after
  the plain commit failed on the same seeds. It was confirmed on fresh noise, and it gets one
  more fresh-seed arena check in the next round before G3.
- **e = 0.55 s, bias −0.24 s, a track-age floor of 0.5 s:** fixed for the crossing (G2, this
  parameter).
- **Next:** the post-fix swb rerun (the node arm of the grid; EV5 held-speed; right-hand deny path);
  a go from rest latched until motion, or `launch_dead` from Autoware's 1.4 s; `role_time_bias`
  split into arrival and rest-stamp biases.

**Phase 2c on the crossing (133's subagent, 2026-11-16; b7adf9d, 75e00ab on main; 938d5dd on branch
`p2c-own-rest-window`; NAS `logs/2026 SSv2 on CARLA/2026-11-16 078 P2 crossing final/`). Stopped
at the G0 fresh-seed check, as the brief said.**
- **Done:** a go from rest is latched until the car first moves (`go_latch_motion_timeout` 3 s),
  for Autoware's 1.4 s launch. `role_time_bias` is split into `arrival_bias` −0.24 s (entry band)
  and `rest_stamp_bias` +0.7 s (`stopped_first`, rest_tie and fresh-rest guards, claim, second-round
  epoch). Replaying P2b's bag, rest_tie still binds, 0.7 s earlier.
- **G0 at fresh noise (`--seed 3`, 5 seeds): fail.** With the arena's biases 0, czp is worse in
  1 of 12 cells (1 contact as B vs t0.5yp2; stock 0). On the same seed the code before this round
  passes 0 of 12, so the latch is the only difference. But stock cells move by up to 3 contacts
  between runs, so the cell is not clearly the latch. With `rest_stamp_bias` 0.7: 3 of 12, up to
  58 of 90 contacts. **The correction shortens the guards** (3.0 → 2.3 s), and a 2 s-patience
  car restarts into the go. On Autoware the stamp lag is 0.45-3.55 s, so the median correction can
  leave a guard short. Formal checks pass in both variants.
- **A second own-rest bug:** Autoware stops 0.6 m before its intersection stop line, which at j979
  lies 2.7-3.2 m before czp's zone edge. So the ego rests at s_entry 3.3-3.8 m, outside
  `rest_window` 2 m, and rest_tie never binds. That, as much as the base_link offset, is why the
  node lost its band runs in P2 and P2b. The branch fix `own_rest_window` 4.5 m binds rest_tie
  in a replay; it is not yet G0-checked.
- **A flicker:** the other car dropped from the conflict set for a moment. Under the mode
  hysteresis the node then sent the module's `safe` = true, and the ego crept 1 m while A was at
  rest.
- **swb:** 2 node runs before the stop, 0 contacts. In the band run, A went first and the ego
  cleared at 20.9 s against stock's 12.6 s (the own-rest bug).

**Rulings (manager, 2026-11-16).**
- **Guards take no rest-stamp correction.** `rest_stamp_bias` orders `stopped_first` only.
  rest_tie's guard, the fresh-rest guard, the claim and the second-round epoch use the raw stamp.
  A correction may only lengthen a guard, never shorten it.
- **A dropped conflict partner is not clear.** A car that leaves the conflict set without being
  seen to exit the zone, or to turn away, keeps the last decision for `mode_hold`. It does not
  hand over to the module's `safe`.
- **`own_rest_window` merges after G0**, together with the two rulings above.
- **G0 for the crossing is re-run on three fresh seed sets** (5 seeds each, not used before), so
  that one contact is told from noise. The criterion is unchanged: 0 cells worse in each set. If one
  set shows a single-contact cell, the cell is rerun at 20 seeds, and it fails only if it stays
  worse.
- The latch stays unless that rerun pins the contact on it.
- Seeds used for G0 so far: V4 base, `--seed 2` (P2b), `--seed 3` (P2c).

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
  (a second round, a creep). Mitigation: the velocity limit, or a small Autoware overlay package. **Confirmed 2026-11-16:** no latch, an approve overrides the module's `safe`, no creep, mode toggling at band edges (Phase 2 above).
- **e may be too large.** If EV4's role-time error at 30 m is near the tie band itself,
  `stopped_first` is unusable and the all-way stop falls back to `map_order` alone. That is safe
  and live, but ignores arrival order. The report must say what that costs. **Confirmed 2026-11-16 for `stopped_first`** (stop instant +0.7 s late on tracks; pooled e99 0.53 s).
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
- 2026-11-16: Phase 2 on the crossing reported (RTC auto-mode fault fixed; node worse than stock in
  the band until the go is committed; EV4 e 0.53 s held out, tracks only within 25 m). G0 for the
  crossing reopened with `commit: on_move`; e set to 0.55 s provisionally.
- 2026-11-16: Phase 2b: G0 for the crossing passed with `commit_release: turn`; e 0.55 s confirmed
  (held out 0.56 s); the node's base_link bug fixed; the post-fix swb rerun is next.
- 2026-11-16: Phase 2c stopped at G0 (`--seed 3`: 1 of 12, and 3 of 12 with the rest-stamp
  correction on the guards). Ruled: guards uncorrected, a dropped partner keeps the decision,
  `own_rest_window` merged after G0 on three fresh seed sets.
