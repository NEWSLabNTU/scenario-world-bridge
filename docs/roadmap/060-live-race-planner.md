# Phase 060: A Live Race Planner and Tracker (fastest lap, drift where it pays)

A track-agnostic racing stack for the Isaac car: from any track's map and the car's spec it
finds the fastest way round, drifting only where the physics makes that faster, and drives it
live, re-planning from wherever the car is. It replaces 057/058's hand-tuned drift points and
the one-shot whole-track plan. The code lives in autoware-drift (`src/planning/race_planner`);
swb supplies the worlds, the car's physics and the test harness (`spikes/course_probe.py`).

**Status**: done, 2026-10-30. Haruna (M5) and Tsukuba (M7, unchanged) driven live in Isaac
within 3.5 % of the whole-track plan, no contact, every wheel on the road; a yaw kick (M6)
absorbed; an unseen low-grip patch at the limit (M6) not, recorded as the next work. Results
and rulings under Progress. Design: autoware-drift
`docs/superpowers/specs/2026-10-29-live-race-planner-design.md` (revision 2 + system
requirements), the offline planner's spec beside it (`2026-10-29-min-time-planner-design.md`),
and the page [Live Race Planner](https://claude.ai/artifact/1Q5G1CBMUAQmd1i8uSXrTA).

**Source**: user, 2026-10-29: "a track-agnostic method ... no per-route tuning ... drift only
where physics makes it faster"; then "a live planning/tracking system that runs in real time:
seconds to compute a medium-range plan, and another part to update routes/control when the
vehicle drifts"; system requirements: at least 110 km/h with no cap ("it can go as fast as
possible"), the fastest lap, and the vehicle's own capability (max speed, acceleration,
deceleration) respected.

**Depends on**: [055](055-onboard-computers.md) (sim_vehicle_ecu, the car's drive and brake
split), [057](057-drift-campaign.md) (drift_mustang, the drift tooling),
[058](058-touge-course-from-open-data.md) (Haruna's world and its course generator).

## Requirements

| | requirement | from |
|---|---|---|
| R1 | plans arrive within a fixed budget, from the current state | the whole-track solve takes ~30 min |
| R2 | after a deviation, re-plan from where the car is; never pull it back onto a stale line | the s 935 crash |
| R3 | the tracker holds the plan's sideslip and yaw rate, not just its line | 1.1-1.4 s lost per hairpin |
| R4 | plan in the car's real inputs: steering, one acceleration request | the plan drove the motor against the brakes |
| SR1-2 | at least 110 km/h, no cap; the fastest lap | user |
| SR3 | the car's top speed, acceleration, deceleration and steering limits from its spec, in every layer | user |

## Architecture

| layer | rate, budget | job | why it exists |
|---|---|---|---|
| L0 global plan | offline, minutes; optional | whole-track minimum-time OCP | gives each L1 window a speed ceiling `v_max(s)` and the track's `v_top`; a window cannot see past its end |
| L1 medium-range planner | ~1 Hz, <= 1 s (<= 3 s with a longer window) | minimum time over the next 150-300 m from the predicted state; grip share g1 < 1; 2-3 warm starts (last plan, grip line, drift seed) | a corner's decisions need 150-300 m of view and a converged solve |
| L2 tracker | 50 Hz, <= 10 ms | NMPC (acados RTI), ~1 s horizon, contouring to L1's plan with its sideslip and yaw rate as references; LQR fallback | a slide diverges in tenths of a second; one iteration per tick corrects around a plan |
| ECU | physics rate | sim_vehicle_ecu: drive or brake split, locked axle, steering | the car's own; its limits are the last guard |

One problem cannot be both long and fast: at 37 m/s, 300 m ahead is 400 steps of 20 ms.
Hence a slow layer that converges over the long view and a fast one that corrects once per
tick. Failure handling: a late L1 leaves L2 on the last plan; leaving the plan's envelope
re-plans at once; a missed L2 tick falls back to the LQR; a spin brakes to a stop (the
spatial planner cannot plan through it).

## Milestones: build, eval gate, decision

**M0. Offline planner and a trustworthy simulator** (done, 2026-10-29). Built: race_planner's
whole-track OCP (CasADi/IPOPT, two-track model with PhysX's tyre, load transfer, the brake
system), its model checked against Isaac's manoeuvre suite; Isaac fixed where it was not the
car: issue 022 (locked axle, sim_vehicle_ecu d68637a), 023 (tyre forces at the ground, aib
abaf103), 024 (folded road mesh, swb_isaac_world a038333); the plan never drives against the
brakes (autoware-drift 959600b). Result: Haruna s 60-620 tracked at 5.2 % over the plan with
no contact; the full section crashed at s 935. Decision: move to the live layered system.

**M1. L1 speed spike** (the main risk). Build, throwaway: a warm-started 200 m window of
today's OCP stepped along Haruna, with 1 and 3 warm starts. Eval: solve time p50 / p99,
convergence from a shifted warm start, how often the drift seed wins. Decide: window length
and grid; L1's model (two-track, or single-track with load transfer); its solver (IPOPT, or
acados SQP in s); if over 3 s, a shorter window, a simpler model or a time-domain MPCC.

**M2. The car's limits in the model (SR3).** Build: the motor's rpm limit, regen within its
torque and power, the steering rate from the spec (today a planner constant); L0 re-solved.
Eval: one test per limit (a long straight, a full stop, a quick direction change). Decide:
which limits specs should declare explicitly (speed limiter, the gate's request range).

**M3. Model-in-the-loop, L1 re-planning.** Build: `sim.py`, race_planner's model as a plant
faster than real time, plus a 10 % wrong variant; L1 re-planning against it with today's
tracker, latency emulated. Eval: full Haruna completes in sim; re-plan latency; whether L1
alone removes the s 935 failure; the wrong plant. Decide: the off-plan thresholds that trigger
a re-plan, g1, the window-end bound.

**M4. L2 tracker in the model loop.** Build: acados installed, the NMPC generated from the same
CasADi model, the LQR fallback. Eval: p99 <= 10 ms; hairpin tracking error; sideslip held vs
the plan; NMPC vs LQR head to head; the wrong plant. Decide: L2's model and weights; whether
the LQR can lead on grip sections.

**M5. Isaac: the full Haruna run.** Build: L1 and L2 in course_probe's loop, L1 in its own
process with real latency. Eval: s 60-1520 completes, no guardrail contact, no wheel off the
asphalt over 0.2 s, within 5 % of L0's 74.8 s, budgets met to 134 km/h. Decide: the track
margin per edge type (guardrail, grass drop, run-off), the final g1; a gap in the model, not
the control, is refitted before going on.

**M6. Disturbances.** Build: an injection hook in course_probe (a 30 deg/s yaw kick before a
hairpin; a 20 m patch at mu 0.6 at a hairpin's entry). Eval: each run completes, no contact;
time lost. Decide: whether an online friction estimate is needed now or with the race
estimator.

**M7. Tsukuba, unchanged.** Build: Tsukuba from open data with 058's course generator; no code
or setting changes. Eval: M5's criteria, and the drift pattern on Haruna (locked vs open
axle, asphalt vs snow). Decide: generality accepted, or the settings that moved become
car- or surface-derived; then the race stack's port (autoware-drift `race-stack-design.md`:
ROS 2 nodes, L2 in C++ on acados' generated C, Autoware's monitors set for racing).

## Progress

**M1 (done, 2026-10-30).** Built: `window.py`, L1 on the car's real inputs (steering, one
acceleration request; the ECU's split) instead of the two-track OCP's wheel torques. The two-track
window took 6-21 s a solve and diverged from shifted warm starts. Results on Haruna s 76-1319,
re-solving every 1 s along the last plan: 56/56 feasible, p50 0.22 s, p95 0.54 s, p99 0.77 s
(autoware-drift 844e8e0). What it took: a NaN-free friction ellipse, the problem as one mapped
stage function compiled to C, the Hessian by stage, the warm start's tail from where the plan is
left, a soft road edge (10 s/m) before the window's last node, multipliers passed as a pair.
The whole NLP as one JIT unit (130 MB of C) never compiled; a hand-written SQP on HPIPM failed
from shifted warm starts; fatrop was no faster than IPOPT.
Rulings:
- L1's model is the axle car with the ECU's split, not the two-track. Why: speed, and it plans
  in the car's inputs (R4). Cost if wrong: its tyres must be checked against the two-track's, and
  were (see M3).
- Window 200 m at 4 m, one warm start (the shifted plan), no multi-start. Why: p95 already uses
  half the budget. Cost if wrong: a grip/drift mode change seen late, one window later.
- L1's budget is 1.0 s, and a plan is adopted a fixed 1.0 s after its solve starts, from the
  state predicted for then. Why: a plan made from the car's state now but adopted later started
  42 m on at the old speed; L2 then braked at -15 m/s^2 and went to full lock at 100 km/h (first
  Isaac LIVE run). Cost if wrong: 1 s of latency on every correction; the envelope's re-plans
  wait on it.

**M2 (done).** The motor's rpm, regen, the steering rate from the spec (db7e2eb).

**M3/M4 (done).** `sim.py` (two-track + wheel spin behind sim_vehicle_ecu), `tracker.py`
(L2: NMPC by RTI on HPIPM, p50 1.4 ms, p99 1.8-3.2 ms; LQR fallback), `live.py` (the loop),
`serve.py` (the loop as a process for Isaac). Safety layer: three failed solves in a row, a
sideslip past 60 deg or the plan's end within 10 m stop the car at -6 m/s^2. While stopping,
the car steers with a geometric law, not L2: L2 is linearised about the plan's speed and, braking
far below it, countersteered the car into a spin.
Model-in-the-loop on Haruna, finding by finding:
- off road at s 180, every solve ok: L1 planned a 7 deg slide under full drive on a gentle bend
  (L0: 1 deg). The axle ellipse let the rear carry 0.98 muFz drive at 7 deg slip angle; PhysX's
  tyre gives 0.84 there. Fixed: the rear tyres are PhysX's own, the rear axle's speed a control,
  the rear force held to the ECU's (autoware-drift 5964e8c). Plan sideslip there now within
  0.3 deg of L0, solves p95 0.72 s;
- off road at s 300, the hairpin entry: the car cannot brake as the plan does (17.5 m/s planned,
  20.8 driven at s 281). The front ellipse is optimistic past the peak slip angle, and with no
  ABS a brake force past the tyre's real peak locks the wheel. Fixed: the front tyres are PhysX's
  too, each wheel's speed a control, its force held to its brake (bfb6094); the inner front no
  longer locks (kappa 0.3 against -0.6 to -0.8 before);
- the brakes: the ECU brakes on regen first (rear only, up to min(2.8 kN, 60 kW / v)), so below
  about 2-4 m/s^2 all braking is on the rear; L1 assumed 50:50. L1 now does exactly what the ECU
  does (fb5f3d6; checked against the ECU over 25 (a, speed) points, within 60 N + 2 %);
- L2 predicted with the ellipse tyre while L1 planned with PhysX's: it asked a third of the
  plan's braking into the hairpin. L2 now predicts with L1's model (8c9b527; p99 2.7-3.5 ms),
  the LQR fallback keeps the ellipse. Result: through the first hairpin, off road at s 435;
- open: L1 plans rear slip 0.4-1 on straights (the force balance has a second solution past the
  tyre's peak, unrealisable: there the wheel spins up), and L2 steers 20+ deg from the plan's
  steering at a bend's exit, leaving the next plan a steering angle it cannot unwind. Fixed:
  every wheel on its tyre's rising branch, dFx/dkappa >= 1 % of the stiffness (a68f9fc; the
  plant confirms a wheel past the peak spins up: slip 0.27 -> 0.54 in 1.5 s); a cost on L2's
  steering against the plan's (771af5c). Off road at s 1150 / 703 (two runs);
- the latency's prediction: plans flipped sideslip from window to window (-29, +6, -16, +21,
  +67 deg) because each started from the old plan's state plus the car's whole deviation. The
  car integrated open loop instead diverged in a drift (unstable without the tracker: vy 8.6 m/s
  off in 1 s). Fixed: the deviation decayed as L2 takes it out (0.5 s); and an open course gets
  a straight run-off past its finish for the window's last 200 m (autoware-drift 1b4602a,
  59b7ffb).

**M3/M4 result (2026-10-30): Haruna s 60-1500 completes in the model loop**, from 5 m/s:

| run | result | time | L0, same section | solves ok | L1 p95 |
|---|---|---|---|---|---|
| 1 | done | 75.40 s | 74.04 s (+1.8 %) | 74/75 | 0.67 s |
| 2 | done | 75.73 s | (+2.3 %) | 72/75 | 0.90 s |
| no L0 (quasi-steady ceiling) | done | 75.43 s | | 75/75 | 0.45 s |
| plant grip x0.9, transfer x1.1 | off road, s 1443 | 76.03 s | 70.62 s | 75/75 | 0.62 s |

L2: p50 1.7 ms, p99 2.7-4.3 ms. The off-plan re-plan never fired: L1 is busy on its 1 s timer
throughout, so the envelope adds nothing at this budget.
Rulings:
- L1's grip share g1 stays 0.92. The wrong plant shows it is not a margin for a 10 % error in
  the friction itself: with the plant at 0.9 the plan asks 102 % of what there is and the car
  leaves the road late in the run. That is M6's question (an online friction estimate), not a
  smaller g1 for every track. Cost if wrong: a car that meets less grip than its model crashes
  until the estimate exists.
- L0 is optional: the quasi-steady ceiling gave the same time.
Rulings:
- L1 and L2 share one car model (axle.py with PhysX's tyre at every wheel, the ECU's brake
  blending); only the LQR fallback keeps a simpler one. Why: every mismatch between them showed
  up as the car failing to do what the plan assumed. Cost if wrong: L2's QP is larger (7
  controls), still under 4 ms at p99.
- Tsukuba (M7) built from open data (swb_isaac_world 163f6e5: closed courses in the generator;
  OSM's line is 4.7 % long, 2142 m); its L0 lap is 71.05 s, 196 km/h at the top, 10 deg slides
  in the hairpins. The live loop drives a circuit as laps of open road (autoware-drift bf8e094).

Later fixes from Isaac and Tsukuba, all in the live loop:
- a window solved for where the car will be assumed the car keeps the plan's schedule; behind
  it, the next window started ahead of the car, the tracker held the plan's first node, and the
  gap compounded (Isaac, Tsukuba lap 2: spun). The distance now carries the car's speed
  deviation, and a plan is adopted only once the car reaches its first node (7211f30). Haruna
  costs 0.76 % for it (accepted: it is what correct hand-overs cost);
- at 55 m/s a 200 m window ended inside the next corner's braking zone and the next windows
  started too fast for it (Tsukuba's back straight, solves at the wall). The window is now as
  long as a stop from the course's top speed plus 100 m: 260 m on Tsukuba, 200 m on Haruna; three
  failed solves keep the plan while it still covers the stop (the old rule braked at -6 m/s^2
  from 196 km/h into a corner) (aa77b7c);
- a circuit's first position search took the third lap of the unrolled road (81805f4);
  course_probe's "off the lane" was Haruna's 6 m on Tsukuba's 15 m road (swb_isaac_world
  f21cbbc).

**M5 result (2026-10-30): Haruna in Isaac, live.** s 60-1518 from a pure-pursuit run-up:

| run | time | L0, same section | contacts | wheels off asphalt | solves ok | peak |
|---|---|---|---|---|---|---|
| first (finish crashed the planner: run-off too short) | 77.02 s | 74.56 s (+3.3 %) | 0 | none | all | 131 km/h |
| after the run-off fix | 77.24 s | 74.74 s (+3.35 %) | 0 | none | 76/78 | 131 km/h |
| final code | 77.38 s | 74.74 s (+3.53 %) | 0 | none | 76/78 | 130 km/h, beta 16 deg |

Worst lateral error from the plan 0.65-1.1 m. Budgets: L1 p95 under 1.0 s, L2 under 5 ms.

**M6 result.** Yaw kicks of +-30 deg/s at s 270 (30 m before H1, braking): both runs reached
the end, no contact, 77.28 / 77.18 s against 77.24 s undisturbed; L2 took the kick out within
1 s (peak sideslip 6.7 deg). A 20 m patch at mu x0.6 at s 490-510 (H3's entry): finished, 2.3 s
lost, but 5.4 m off the plan at 27 deg with wheels on the terrain and guardrail contacts: fail.
In the model loop (same patch): x0.9 and x0.8 pass, x0.7 survives 5.3 m off, x0.6 leaves.
Decision: an online friction estimate, the car's own (sim_vehicle_ecu's, at the limit), is
wired in (26e0257) but **off by default** (04f6cdf): it saved 0.11 s and 1 m at x0.8 and turned
x0.7 into a crash (its scale back to the map's while the car still slid). An unseen low-grip
patch met at the limit is not caught by an estimate made after the slide began. Next: the
surface known ahead (the map's surface classes with their uncertainty planned in, the race
estimator's friction), and the estimate's exit fixed before it is used.

**M7 result: Tsukuba, unchanged.** Built from open data (swb_isaac_world 163f6e5: closed
courses in 058's generator; OSM's line is 2142 m, the official 2045 m). L0's flying lap 71.05 s
(196 km/h; 10 deg slides in the two hairpins, grip elsewhere). Model loop, 2 laps: flying lap
72.72 / 74.36 s. Isaac, 2 laps live: flying lap **72.64 s (+2.2 %)**, no contact, every wheel
on the road, 144/144 solves, 193 km/h. No setting moved between the tracks; the window's length
follows the course's top speed (a rule, not a setting).
Rulings:
- generality accepted: the same code and settings drove a mountain pass and a circuit.
- The drift pattern (locked vs open axle, asphalt vs snow) was not swept here: L0 on snow and
  the open axle did not converge in M0, and the live loop's windows were not tried on them.
  Left for the race stack's phase.

**Open, for the next phase** (the race stack's port, autoware-drift `race-stack-design.md`):
the surface known ahead (M6); low grip (snow) and the open axle; the off-plan re-plan never
fires while L1 is busy on its 1 s timer (a pre-emptive re-plan would need a second solver or
an abortable one); Isaac runs wait for each L1 solve (a solver on its own core in real time is
the ROS port's).

## Risks

- **L1's solve time** (M1): a cold 400 m solve takes 200 s today; a warm 200 m window must
  converge in about a second.
- **Grip vs drift as local optima**: a warm start stays in its mode; the multi-start may cost
  more cores than the budget allows.
- **Localisation at speed** (outside this phase): NDT lost the car at 30 m/s on Town05 (issue
  017); the race estimator must hold to `v_top`.
- **Full-track solves that do not converge** (open diff, packed snow): L0 is optional, but
  M7's snow pattern needs either L0 or L1's windows to solve on low grip.
