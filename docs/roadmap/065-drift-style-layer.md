# Phase 065: Drift Style, Chosen by a Layer over the Solvers

061 gives the live planner a start in each valley, grip and drift, and a gate that picks the
faster. This phase puts a **style layer** above it, so the same planner can drive three ways:
the fastest lap (S1, today), a fast lap that takes a drift where it costs little (S2), and a
drift wherever the car can, corners and straights, keeping speed (S3, D1-style). The layer
does not add a solver or change L1's minimum-time objective. It picks which formulation each
existing piece runs: the offline data, the seed schedule, L1's rows, the gate and L2's weights.
The code lives in autoware-drift (`src/planning/race_planner`).

**Status**: in progress, started 2026-10-31 (user: "starter phases"). M1 runs beside 061 M4c,
as the roadmap allows (S1 must change nothing); M2 on waits for DP0.

**Source**: user, 2026-10-31: "We pursue for laps with shortest time. Later, we may request for
strong performance like D1 GP ... The D1 would require drift on every turn and the car has to
maintain speed. The other is to chase for fast lap but we may change to drift turns in the case
that the drift trades off no or little time"; "the behavior should be adjustable ... fastest
lap, drift for little trade off or always making drift if possible even on straight roads";
"The style or behavior is like a layer we can decide the combinations"; "Show me the proposed
design and the roadmap plan. add experiments and decision points."

**Depends on**: [061](061-grip-or-drift-mode-choice.md) (seeds G/H/D/F, the gate, the schedule
by surface class, the slip caps, M4c).

**Design**: autoware-drift `docs/superpowers/specs/2026-10-31-drift-style-layer-design.md`
(611c775), and the live planner design's revision 4 (section 11).
Evidence: autoware-drift `docs/drift-style.md` and `docs/drift-style-notes/`.

## Requirements

| | requirement | from |
|---|---|---|
| R1 | One style record selects S1, S2 or S3; a change applies at the next re-plan without a stop, never within 25 m of a committed drift zone | user: "adjustable" |
| R2 | S1 is bit-identical to 061's scheduled arm | anti-goal: style must not cost the fast lap |
| R3 | S2 loses at most the user's budget (+ 0.3 % noise) per lap and drifts more as the budget grows | user: "no or little time" |
| R4 | S3 drifts every zone the plan holds, including straights, with zone speed near S1's | user: "drift on every turn ... maintain speed ... even on straight roads" |
| R5 | No style loosens a safety invariant (design section 5); ice is never a planned slide | 061 M4b |
| R6 | L1 p95 ≤ 0.9 s in every style | 060 budget |

## Experiments

| | question | setup | measures |
|---|---|---|---|
| E1 | Does the record's plumbing change anything in S1? | Haruna, Tsukuba dry, ×3, idle host; S1 record vs 061 scheduled arm | plan diff (bit-identical), lap time, solves per re-plan |
| E2 | What does drift cost, per corner and class? (the first lap-time vs drift-share curve) | offline forced-drift solves on the drift-map grid (turn, R, width, class), plus Haruna and the class-zone chain | signed loss per cell; Pareto curve lap time vs drift share for ε ∈ {0.3, 1, 2, 3} %, eps_lap_s ∈ {0.1, 0.2, 0.5} s |
| E3 | Does S2 hold its budget in closed loop? | sim.Plant, Haruna + chain, budget sweep, ×3 | loss vs budget, drift share, stops/spins/off-road vs S1, mode changes per corner, adoption p99 |
| E4 | Do the ECU's traction control and ESC fight a planned slide? | sim.Plant, mud and gravel hairpins, D forced, ECU profiles on/off | torque cuts and ESC brake events in a planned slide, tracking error, stops |
| E5 | Can S3 hold a slip floor and a speed floor together in corners? | sim.Plant, mud and sand corners (R 9/15/25, 90°/180°, 6/15 m), slip floor and speed floor swept | share of zone past the floor, zone speed vs S1, spins, LQR rate, L1 p95 |
| E6 | Can L1 plan a manji on a straight within budget, and can the car drive it? | M library built offline; mud straights 100–300 m, speed bands; then packed snow | L1 iterations/p95 with M, share past floor, switch-point error, swing rate, speed loss |
| E7 | Can S3 hold a drift on asphalt (falling branch)? | asphalt corners, then straights; slip floor swept down from 3° past peak | as E5, plus runaway events (slip growing while plan holds) |
| E8 | Does it carry to Isaac? | best plant arm through `course_probe` LIVE=1 (only when no other user's CARLA is on the host) | as E3/E5 plus body β as a diagnostic against D1's 30–45° |

## Decision points

| | when | question | options |
|---|---|---|---|
| DP0 | before M1 | Is 061's gate passed (M4c), or does the user rule it good enough to build on? | start / wait for M4c / start M1 only (plumbing is safe) |
| DP1 | after E2 | Where is drift cheap? If asphalt drifts all cost more than any useful budget, S2 on asphalt is budget-only, and S2's value is on low friction | S2 everywhere / S2 low friction only / S2 offline-only on asphalt |
| DP2 | after E3 | Adopt S2? | adopt / tune (ε, h) / drop the live gate, keep offline budget only |
| DP3 | after E4 | Does the ECU fight planned slides? | no change / a race ECU profile in `sim_vehicle_ecu` (authorized repo) / ESC off in S3 zones only |
| DP4 | after E5 | Can slip floor and speed floor hold together in corners? | keep both / relax speed floor ("deceleration by angle", DM's red zones) / slip floor only |
| DP5 | after E5/E6 | **Actuation**: if the speed floor fails where rear torque runs out (straights, high speed), does the car get an independent brake input (handbrake or per-wheel brake), or does S3 live with the single acceleration request? A new actuator touches `sim_vehicle_spec` (not authorized for this session) and the ECU | **user decides**: keep today's actuation / add handbrake / add per-wheel brake or torque vectoring |
| DP6 | after E6 | Manji on straights: within budget and drivable? | adopt / straights only on low friction / corners and S-bends only (`straights: false`) |
| DP7 | after E7 | S3 on asphalt? | adopt / corners only / no-fall-off classes only |
| DP8 | after E8 | Phase done, or a follow-up (tandem, tyre wear, judged scoring)? | close / new phase |

## Milestones

| | milestone | experiments | gate | then |
|---|---|---|---|---|
| M0 | 061 M4c done | — | DP0 | M1 |
| M1 | **Style record and plumbing**: record, load-time validation (refuse by name), invariants not fields, style switching at re-plan, S1 default | E1 | R1, R2: S1 bit-identical, Haruna 0.000 % | M2 |
| M2 | **Signed drift map**: forced-drift solves per cell, signed losses, knapsack over a lap's corners | E2 | map complete for swb's classes; Pareto curve drawn | DP1 → M3 |
| M3 | **S2 live**: lex-ε gate with hysteresis, lap account, schedule adds D where loss fits the budget | E3 | R3 on the plant; no added stops vs S1; R6 | DP2 → M4 |
| M4 | **ECU check** for planned slides | E4 | profile decided | DP3 → M5 |
| M5 | **S3 corners**: zone plan (auto), elastic slip/speed/swing rows, no-return penalty, L2 weights, grip-then-stop fallback; mud and sand | E5 | R4 in corners on no-fall-off classes; zero spins; R6 | DP4, DP5 → M6 |
| M6 | **S3 straights**: M seed library, manji zones; mud then packed snow | E6 | R4 on straights; R6 with M | DP5, DP6 → M7 |
| M7 | **S3 asphalt**: corners then straights | E7 | R4 or a measured limit; no runaways | DP7 → M8 |
| M8 | **Isaac** | E8 | plant results hold within 060's sim-to-plant spread | DP8 |

Order inside S3: mud and sand → mud straights → packed snow → asphalt corners → asphalt
straights; never ice. M1 is safe to build before DP0 (S1 must not change anything).

## Non-goals

Tandem; DOSS-style judged scoring; a required "D1 look" (body angle is reported only); a GPU
or learned policy; drifting on ice.

## Risks

- **Single acceleration channel** (DP5): S3's speed keeping may be limited by the car, not the
  planner.
- **Falling-branch asphalt** (E7): the regime that lost ice in 061 M4b.
- **L1 budget** with M seeds (E6) on a solver already at p95 0.91 s.
- **061's gate unpassed**: S2/S3 inherit its failures (ice, wrong-map).
- **Unmeasured thresholds**: slip floor, speed floor, swing rate, width use, guardrail minimum.
- **Tyres**: heat and wear unmodelled; S3 is not free on tyres.

## Progress

- 2026-10-31: design, research report and live planner revision 4 committed (autoware-drift
  611c775). M1 (style record and plumbing, E1) started.
