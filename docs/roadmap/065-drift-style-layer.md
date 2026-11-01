# Phase 065: Drift Style, Chosen by a Layer over the Solvers

061 gives the live planner a start in each valley, grip and drift, and a gate that picks the
faster. This phase puts a **style layer** above it, so the same planner can drive three ways:
the fastest lap (S1, today), a fast lap that takes a drift where it costs little (S2), and a
drift wherever the car can, corners and straights, keeping speed (S3, D1-style). The layer
does not add a solver or change L1's minimum-time objective. It picks which formulation each
existing piece runs: the offline data, the seed schedule, L1's rows, the gate and L2's weights.
The code lives in autoware-drift (`src/planning/race_planner`).

**Status**: in progress, started 2026-10-31 (user: "starter phases"). M1 done; DP0 passed for
S2's classes (user, 2026-11-01); M2 done (S2 built through M3's gate; E2, a first E3); M3 run
(R3 met on the plant, R6 unmeasured; DP2: tune, then adopt).

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
- 2026-10-31: **M1 done** (autoware-drift 3667c87, 26cb0be, b96a77a on main: the record, the
  plumbing, the E1 harness options). R1's plumbing and R2 met on the plant.
  - `race_planner/style.py`: `Style`, design 4.5's fields (no `ecu_mode`), `fastest` by default,
    from a dict (`{"style": {...}}` or the inner one) or YAML. Refused by name (a `ValueError`
    whose message starts with the field): unknown fields; invariants given as fields (a named
    list: `grip`, the slip caps, `free_stages`, margins, quantile, estimator, stop, ...);
    `eps_window` nonzero outside [0.003, 0.03]; ice in `classes`; a field the style does not
    read set off its default; S3's slip floor past the rear cap where the peaks are known.
  - `live.Driver(style=...)`, `MultiStart(style=...)` (the driver's by default),
    `serve.py`'s init `style` (refused before the track loads, so before `ready`) and a runtime
    line `{"style": {...}}`, answered `{"style", "pending"}` or `{"style_error"}` (the style
    kept, the car not stopped). `Driver.set_style` validates at once; `style_due`, at the top
    of every re-plan, applies it unless the car is within `AHEAD` (25 m) before a committed
    drift or in it (deferred, retried each re-plan; `style_log` records requested / deferred /
    applied). No Window variant compiled (call 1's S3 window comes at M5).
  - Tests: `test/test_style.py`, 35 (23 refusal cases, each matching `^'<field>'`; default =
    fastest; S2/S3 refused by Driver and MultiStart; slip-floor rule; serve refuses a bad init
    record before `ready`; the switch deferred near a committed drift, applied past it, applied
    at once on a grip plan; a refused runtime style keeps the current one; S1 with
    `Style()` plans bit for bit as without, on a short road through four or more re-plans).
    Full suite: 153 passed (14 min, load 6-8).
  - **E1** (`studies/mode_choice_m4.py run <course> --arm sched --digest [--style ...]`; sha256
    of every plan's X, U, s, t and of every plant state). The closed loop is not repeatable
    bit for bit with the clock in it (IPOPT's `max_wall_time`, adoption at max(latency, solve
    time), L2's fallback on its budget): the first pair of timed runs on Tsukuba parted at
    plan 57 of 73, one solve cut differently (failed 2 against 1). So the bit-for-bit arms ran
    with `RP_DET=1` (no clock: solves uncut, timed 0 s, adopted at the fixed 1 s latency),
    against the commit before M1 (611c775, `RP_ROOT`), on the same host:

    | course | 611c775, no record | M1, no record | M1, `fastest` record | lap (det) | timed lap, none / S1 |
    |---|---|---|---|---|---|
    | Haruna | `9524862c` / `9160b94d` | same | same | 76.28 s | 76.28 / 76.28 s (bit-identical too) |
    | Tsukuba | `d7ca9f95` / `82bd3c28` | same | same | 74.33 s | 74.37 / 74.30 s (parted at plan 57) |
    | zone chain (seeds on) | `ac0fd2f1` / `8207698c` | — | same | off road at s 1236, both | — |

    (plans / trajectory digest, first 8 hex.) Every det arm: 76 and 73 plans, 1.00 solves a
    re-plan (G only on dry asphalt), no failed solve. Haruna's lap loss S1 vs no record:
    0.000 %. The zone chain was added because the dry courses never send a seed: with G, H, D, F
    live (2.48 solves a re-plan, 10 mode changes) the record changes nothing either. (Its
    off-road at s 1236 is 061's, in both arms, unclocked.) Timed runs at load 5.7-6.9 (32
    cores), one repeat: lap times are indicative only; the three-repeat idle-host timing of E1's
    table is not done, and bit identity does not need it.
  - **Rulings** (builder's, M1):
    1. S2 and S3 validate, and the planner refuses to run them: `NotImplementedError` where a
       style is accepted (Driver, MultiStart, `set_style`, serve's init), never mid-re-plan;
       a runtime request is answered `style_error` and the current style kept. Not "run as S1
       with a warning": that is a silent clamp.
    2. Call 9 extended: a field set off its default under any style that does not read it is
       refused (S3's under S2, S2's under S3, both under S1). `drift_past_deg` is read by S2
       and S3. `eps_lap_s` and `eps_window` may both be set under S2 (call 3 uses both).
    3. S3's default classes in swb's names: mud = `mud_wet`, `mud_soft`; sand = `sand`,
       `sand_loose`; plus `snow_loose`, `gravel`, `dirt`, `grass`, `snow_packed`. Checked
       against a `SurfaceTable` when one is given.
    4. The slip-floor rule: peak + `slip_floor_past_deg` <= `rear_slip_max` x peak (3) on every
       surface without fall-off; the Driver computes the peaks from its track (S3 only).
    5. "Committed drift" for the 25 m rule: the adopted plan slides (`slides_in`, now a module
       function of multistart) in a corner whose zone (`CORNER_FROM` before turn-in to exit)
       the car is within `AHEAD` before, or inside. A one-start plan, never classified, is read
       at the map's friction.
    6. E1's bit identity is measured unclocked (`RP_DET=1`); the timed arms give lap times.
  - Left for M2 on: everything S2/S3 does (signed map, knapsack, lex-ε gate, zone plan, elastic
    rows, M seeds, L2 weights); E1's three-repeat timing on an idle host; DP0 (M4c).
- 2026-11-01: **DP0 passed for S2's surface classes** (user: M2 starts after 061 M4e; ice still
  open on 6 m roads, and S2 never prices or buys ice; asphalt held per the design).
- 2026-11-01: **M2 done**, and S2's live half (M3's gate, account, schedule) built with it
  (autoware-drift 0a624f2 on main; write-up `docs/drift-style-m2.md`; results under
  `play_log/race_planner/style/m2/`).
  - Built (`race_planner`): `budget.py` (the signed map `drift_losses.json` with a conservative
    bracketed price per corner, an exact knapsack over `eps_lap_s`, the account debited once a
    corner by the price and never credited, per run or per lap); `forced.py` (a drift held 3 deg
    past the rear's peak from turn-in to the corner's middle, by the window's held-slip rows with
    a negative bound: no new rows, nothing loosened); `MultiStart._choose_s2` (a bought corner's
    drift within price + 0.3 %, left past + `exit_hyst`; the lex-eps gate on classes without
    fall-off; else S1's gate, moved unchanged) and slot B (the forced drift into a bought corner).
    S2 needs the scheduled starts (refused by name); S3 still refused. Tests: 26 new; suite 181
    passed.
  - **S1 bit for bit** (`RP_DET`, sched, no record and `fastest`, before and after the rebase):
    Haruna `9524862c`/`9160b94d` 76.28 s, Tsukuba `d7ca9f95`/`82bd3c28` 74.33 s.
  - **E2**: 152 cells (8 classes, R 9-60, 90/180 deg, 6-15 m). Every drift the free window found
    was a gain (-0.08 to -0.97 %); where a drift does not pay it has **no valley**: every drift
    seed slides back to grip. Forced (49 cells), a drift costs asphalt +0.02-0.05 s a corner
    (+0.27-0.69 %), gravel/grass/dirt up to 0.11 s, packed snow up to 0.23 s. Predicted curve:
    Haruna 0 / 0.1 / 0.2 / 0.5 s buys 1.6 / 6.1 / 8.6 / 9.8 % of the lap in drift for 0 / 0.08
    / 0.17 / 0.23 s; every course's whole lap of drifts costs <= 0.37 s, so eps >= 0.3 % buys
    everything (`pareto.md`).
  - **E3** (first, one run each, `RP_DET`): every S2 run that finished stayed within budget +
    0.3 % of S1 (Haruna +0.05 to +0.39 s at 0.05-0.5 s; Tsukuba -0.05 to +0.19 s); no added stop,
    spin or off-road. But the plant's drift does **not** grow with the budget (Haruna 17 / 0 / 0
    / 28 m): the live window prices forced asphalt drifts 2-4x the map (0.05-0.10 s against
    0.026-0.029), so most bought corners are never entered, and the car slides little on the ones
    that are (rear <= 5.2 deg past its peak). On gravel, grass, dirt and packed-snow corners S2 is
    0.04-0.08 s **faster** than S1: it takes the near-free drifts S1's 0.3 % switch refuses. Mud and
    loose snow: S2 = S1 (S1 drifts already). Haruna-geometry mud and loose snow leave the road at
    s 269 in every arm (061's limit).
  - **DP1** (builder's ruling, for the user): S2 everywhere, offline budget, with the forced drift
    (B) live; drift is cheap on every class the floor holds, but costly drifts exist only forced.
    **Confirmed by the user 2026-11-01** ("Confirm DP1 and start M3").
  - **DP2** (M3's, preliminary): tune: price the map at the live settings (grip 0.92, ds 4, the
    live window) and look at L2 on a planned asphalt slide, before adopting.
  - **Rulings** (builder's): forced drift by hard floor rows (a floor that cannot hold leaves
    grip); price = worst bracketing cell, none outside 1.25x the measured radii; allowance
    max(price, 0) + 0.3 %; debit at entry; budget per run (per lap when told the lap); a runtime
    switch to S2 gets a fresh account and no B; gain corners bought free and entered at a tie.
  - Video-ready (S1 vs S2, each with `--traj`'s `.traj.json` for `tools/arm_video.py`; RP_DET,
    repeat bit for bit): `play_log/race_planner/style/m2/video/` Haruna s1 / s2b0.5, gravel
    R 15x90 s1 / s2b0.05, packed snow R 15x90 s1 / s2b0.05, Tsukuba s2b0.1.
  - Left for M3: the map at live settings; L2 on planned asphalt slides; E3 x3 and timed on an
    idle host (p99 adoption unmeasured: det runs have no clock); the lex gate in closed loop (no
    flat-class corner is a loss in today's map); the zone chain (061's off-road at s 1236).
- 2026-11-01: **DP1 confirmed** (S2 everywhere, offline budget, forced drift live); **M3 started**.
- 2026-11-01: **M3 run** (autoware-drift 8bef811, da6e230 on main; write-up
  `docs/drift-style-m3.md`; results under `play_log/race_planner/style/m3/`; ECU sim_vehicle_ecu
  d68637a). M3's gate is met except R6, which is not measured.
  - **Price as the gate sees it** (item 1): `s2-live` prices a cell at the live settings
    (live.Driver's window 200-312 m, its caps, `v_end`; from 45/75/105 m before the turn-in, the
    largest loss). Flat asphalt R 25 x 90: 0.024 s (M2) -> 0.076 s. Haruna's own corners
    (`course-prices`): its downhill hairpins **gain** (-0.003 to -0.058 s, M2 priced +0.029), s 1195
    0.06-0.09 s, s 1411 0.11-0.15 s, the s 1071 kink 0.15-0.93 s. A cell map cannot carry grade
    and corner runs, so **course prices** (`budget.CoursePrices`, `--prices`) come before the map;
    the map (`drift_losses.json`, 208 cells incl. 45 deg) is rebuilt at the live settings and now
    has losses on mud, loose snow and sand (+0.02 to +0.11 s).
  - **Floor past the car's peak**: the forced floor was 3 deg past the planner's peak (grip 0.92),
    1.9 deg past the car's real one on asphalt; now at the map's friction (16 deg on asphalt).
  - **L2** (item 2): holds a sustained planned slide within ~1 deg (s 1195: -0.5 deg mean); lags
    short spikes 1-5 deg (yaw transient); weight changes (slide x4, line 0.5 m) moved corners both
    ways. M2's "5.2 deg at most" was the plans and the measure, not L2. **No L2 change.**
  - **Issue 010** (filed, fixed): S2 carried a drift into the next bought corner without its
    allowance check or debit (Haruna 0.5 s: 0.058 s booked against 0.146 driven).
  - **E3 x3** (RP_DET, `--jit` 0/1e-9/2e-9; corner courses with a 160 m entry):

    | course | budget s | vs S1 s, mean (spread) | within budget + 0.3 % | plant drift m, mean (range) |
    |---|---|---|---|---|
    | Haruna (S1 76.49, draws 76.28-76.67) | 0 / 0.1 / 0.2 / 0.5 / 1.0 | +0.13 (0.20) / +0.21 (0.24) / +0.21 / +0.45 (0.38) / +0.45 | means all; draws 2 of 3 at 0 and 0.1 | 5 (0-9) / 36 (18-54) / 36 / 40 (30-54) / 40; S1 0 |
    | asphalt R 25 x 90 | 0-0.1 / 0.2 | 0 / -0.00 (0.20) | yes | 0 / 6 (0-16) |
    | dirt R 25 x 90 | 0-0.05 / 0.1-0.2 | 0 / -0.07 (0.33) | yes | 0 / 28 (11-53) |
    | grass R 15 x 90 | 0-0.1 / 0.2 | 0 / +0.16 (0.04) | yes | 0 / 15 |
    | gravel R 15 x 90 | 0 / 0.05-0.2 | 0 / -0.18 (0.04) | yes | 0 / 0 (rear 1.5 deg past) |
    | packed snow R 15 x 90 | 0-0.2 | 0 (corner unpriced) | yes | 20 = S1 |
    | mud R 15/25 x 90 | lex eps 0.01, 0.1 s | off road in every arm, S1 too | | |

    R3 met on means; drift grows with the budget. No added stop, spin or off-road. Mode changes <= 1
    a corner. **Mud**: S1 left the road 3 of 3 only after the rebase onto 474a9bf (0c61d28; before:
    3 of 3 finished on R 25 x 90 at 100 m entry); the other S1 runs are bit for bit the same, so the
    lex gate entered and debited its corner but its finished-corner effect is unmeasured (item 4).
  - **R6 not measured**: load 15-180 all session, no timed run. Unclocked proxy: re-plan IPOPT
    iterations p95, Haruna S1 150-158, S2 158-229 (B the slowest, 400 cap hit once): ~0.9 vs
    1.0-1.3 s at 5.8 ms an iteration.
  - **S1 bit for bit** after the rebase: Haruna `9524862c`/`9160b94d` 76.28 s, Tsukuba
    `d7ca9f95`/`82bd3c28` 74.33 s.
  - **DP2** (builder's ruling, for the user): **tune, then adopt**: keep the live gate with course
    prices; adopt after a timed E3 on an idle host shows R6 (else give B its own iteration budget).
    Not "drop the live gate": the gate is what refuses corners whose live cost exceeds the price.
  - Video-ready (`--traj`, RP_DET, reproduce E3 bit for bit): `play_log/race_planner/style/m3/video/`
    Haruna s1_j0 76.28 / s2b0.5_j0 76.62 (36 m at s 1195); dirt R 25 x 90 s1_j0 / s2b0.1_j0 (53 m);
    asphalt R 25 x 90 s1_j2 / s2b0.2_j2 (16 m).
  - Left: timed E3 (R6, adoption p99); B's iteration budget; lex gate on a finishing flat corner
    (mud vs 0c61d28, for issue 009's owner); packed snow unpriced; course prices for Tsukuba and the
    zone chain.
