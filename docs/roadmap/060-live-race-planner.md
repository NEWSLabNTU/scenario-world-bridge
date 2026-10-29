# Phase 060: A Live Race Planner and Tracker (fastest lap, drift where it pays)

A track-agnostic racing stack for the Isaac car: from any track's map and the car's spec it
finds the fastest way round, drifting only where the physics makes that faster, and drives it
live, re-planning from wherever the car is. It replaces 057/058's hand-tuned drift points and
the one-shot whole-track plan. The code lives in autoware-drift (`src/planning/race_planner`);
swb supplies the worlds, the car's physics and the test harness (`spikes/course_probe.py`).

**Status**: in progress, 2026-10-29. M0 done; M1 next. Design approved in outline, under
review in detail: autoware-drift
`docs/superpowers/specs/2026-10-29-live-race-planner-design.md` (revision 2 + system
requirements), the offline planner's spec beside it
(`2026-10-29-min-time-planner-design.md`), and the page
[Live Race Planner](https://claude.ai/artifact/1Q5G1CBMUAQmd1i8uSXrTA) (architecture and this
roadmap).

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

**M1. L1 speed spike** (next; the main risk). Build, throwaway: a warm-started 200 m window of
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

## Risks

- **L1's solve time** (M1): a cold 400 m solve takes 200 s today; a warm 200 m window must
  converge in about a second.
- **Grip vs drift as local optima**: a warm start stays in its mode; the multi-start may cost
  more cores than the budget allows.
- **Localisation at speed** (outside this phase): NDT lost the car at 30 m/s on Town05 (issue
  017); the race estimator must hold to `v_top`.
- **Full-track solves that do not converge** (open diff, packed snow): L0 is optional, but
  M7's snow pattern needs either L0 or L1's windows to solve on low grip.
