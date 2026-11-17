# The conflict-zone protocol: build design

What we build so that one automated car, running Autoware, settles who goes first at a conflict
zone the way the cooperation study found works: an explainable convention per tie band, with
learned parameters, safe and live with no V2X. This document is the build spec: requirements,
architecture, data, algorithms, interfaces, configuration, contracts, tests. The theory and the
evidence are in [cooperative-driving.md](cooperative-driving.md) (principles, the solution
concept, the protocol frame, the per-vehicle workflow, the scenario walkthroughs, the sensors);
the related work is in [survey-multi-vehicle-cooperation.md](survey-multi-vehicle-cooperation.md);
the phases, evaluation and decision points are [roadmap 078](../roadmap/078-conflict-zone-protocol.md).

## Scope

- **In:** one car's decision at a conflict zone: unsignalled crossings, all-way stops, merges
  (lane drop, Y merge, cut-in, zipper), narrow passages, left turns against oncoming traffic,
  roundabout entries. Its inputs (perception, map, own state; UWB, radar and V2X when present) and
  its outputs through Autoware's own interfaces.
- **Out:** signalised junctions (the signal decides; see the lab's signal-control project), a
  central manager or roadside unit, trajectory planning (Autoware plans), the safety shield (AEB,
  obstacle stop, RSS: kept, never replaced).

## Requirements

| id | requirement | from |
|---|---|---|
| R1 | Never two cars in a zone at once, given the contracts (safety) | principle 4 |
| R2 | No deadlock, livelock or starvation: every car crosses within a bound (liveness) | C7, U3 |
| R3 | Safe and live with no V2X and no UWB on any car | principle 3a |
| R4 | Never worse than stock Autoware on contact and on the TTC, PET and clearance distributions, also against a car that ignores the convention | D7 |
| R5 | When the two cars read the role differently, both wait; never both go | principle 3b, U3 |
| R6 | One total order over the conflict set, from the map, so no wait-for cycle survives | principle 3c |
| R7 | Every decision recorded and explainable by band, role, convention and signals; the learned share reported | principles 5, 7 |
| R8 | Outside a tie band, behaviour equals stock Autoware | principle 2 |
| R9 | With the node dead, late or unsure, stock Autoware runs | ⑤ |
| R10 | Decisions at 2-5 Hz, latched ≥ 1 s; the cycle within 20 ms on the car's computer | ⑤, U3 flips |
| R11 | The same library runs in the arena and on the car | principle 6 |
| R12 | Conventions are configuration per jurisdiction, not code | traffic law |

## Architecture

```
                 offline, per map                         every cycle (10 Hz in, 2-5 Hz out)
 lanelet2 map ──> czp-zones ──> zones.yaml ─┐
                                            v
 perception objects ─┐                 ┌──────────── czp (library) ─────────────┐
 own pose, plan ─────┼──> adapter ───> │ relation → intervals → band → role →  │ ──> decision
 UWB, radar, V2X ────┘    (arena | ROS) │ convention → probe/commit → contracts  │     + record
                                        └────────────────────────────────────────┘
                                                         │
                     arena adapter: commands the arena car│ ROS adapter: Autoware's RTC
                                                         v     (intersection, lane change,
                                                                stop line), velocity limit
```

- **`czp`** (library, Python first; a Rust port only if R10 needs it): pure functions over plain
  data, no ROS and no CARLA. Steps 2-10 of the workflow.
- **`czp-zones`** (offline tool): step 1, from a Lanelet2 map to `zones.yaml`.
- **Arena adapter**: feeds the arena's observer output to `czp` and executes its decision, in place
  of each study's hand-written rules. It is how the library is checked against the studies.
- **ROS adapter**: the `interaction_strategy` node beside Autoware, in the ego domain. Subscribes
  to Autoware's objects, localization, the planned trajectory and the RTC status; publishes RTC
  commands, a velocity limit, and the decision record.
- **Config**: `conventions/<jurisdiction>.yaml`, `params.yaml` (the assumed parameters until
  measured), `contracts.yaml`.

## Data model

| type | fields |
|---|---|
| `Zone` | id; polygon; the lanelets that cross it; per approach: lanelet id, stop line, entry and exit arc length, `order` (the map's fixed approach order, an integer), `priority` (law: major, minor, all-way stop, yield-to-circulating); `width` per lanelet; `exit_box` (the space after the zone that must be clear) |
| `Agent` | id (track id, or UWB id when bound); class; footprint polygon; pose, velocity, covariance; source flags (lidar, radar, UWB, V2X); capability bounds from the class (`params.yaml`) |
| `Relation` | agent; zone; kind: `lead`, `cut_in`, `cross`, `merge`, `oppose`, `oncoming_left`, `circulating` |
| `Interval` | agent; zone; `t_in`, `t_out` as [earliest, latest]; reason (track, phantom, UWB bound) |
| `Band` | `clear` (with who goes), `band`, `contested` |
| `Role` | the role fact used (`map_order`, `map_priority`, `stopped_first`, `ahead_at_end`, `obstruction_side`), its value per agent, its confidence, `ambiguous` |
| `Decision` | `go`, `yield`, `wait`, `wait_second_round`, `creep`, `stock`; latched until; commit state |
| `Record` | time; zone; conflict set; per agent the interval and relation; band; role and its fact; the convention row; probe evidence; contract states; the decision; the learned parameters' share of the margin |

## The cycle, as functions

Each step names its input, output and the rule it implements (the reasons are in the design's
workflow and scenario walkthroughs).

1. **Zones** (`czp-zones`, offline). Overlap of the lanelets' swept areas for the widest vehicle
   class, plus, per vehicle pair, whether the zone is shared at all (two widths and margins
   against the free width). `order` from a fixed rule (approach heading, clockwise from north) so
   every car computes the same order. Output `zones.yaml`, versioned with the map.
2. **Relation** (`relate(agents, route, zones)`). Match each agent's predicted path to lanelets;
   off-lane agents by path intersection with the route. Output: the conflict set per zone.
3. **Intervals** (`intervals(agent, zone, params)`). Own: from the planned trajectory and the
   spec's acceleration and braking. Other: earliest arrival at the class's maximum acceleration up
   to the speed limit, latest at the class's braking bound; footprint swept along the path. Phantom
   at an occlusion edge at the speed limit. UWB: a presence bound (an equipped car within r), and a
   tighter range only when it falls on a track within 2.5 m + 6 % of the range.
4. **Band** (`band(own, other, params)`). `clear` when one interval's latest exit precedes the
   other's earliest entry by the margin; `contested` when the other's interval shows no braking
   that can stop it before the zone; else `band`. Own interval from the car's own capability
   (launch, length, braking by spec or class), never a default car's: a heavy car using the Model
   3's launch under-estimates its own interval about 2× (074 C8).
5. **Role** (`role(zone, relation, agents)`). The map's fact first (`map_priority`, then
   `map_order`); a perception fact (`stopped_first`, `ahead_at_end`) with the tie margin; inside
   the margin, `ambiguous`. The perception fact is bias-corrected first. A hard threshold with no
   margin is unsafe for any perception error (074 C8: a band of width 2e); the margin must be at
   least e (the error's 99 % quantile, centred), so e on each side.
6. **Convention** (`decide(conventions, relation, role)`). The jurisdiction's table, row by
   relation, then role. `ambiguous` → `wait`, then `wait_second_round` resolved by `map_order`,
   H2 ≥ e + 1.2 s after the junction is quiet (no car seen starting or leaving the zone; a blocked
   exit's release counts as an event). Before a round-2 `go`, the legible probe (creep about 0.5 m,
   then watch): stock Autoware reads a car at rest as static and restarts into the round-2 go. A
   wait-for graph over the conflict set; any cycle is broken by `map_order`. `go` requires the
   zone's `exit_box` clear.
7. **Probe and commit** (`probe(decision, own, other, params)`). When `go` needs the other to
   yield: a creep (1 m/s) or a lateral drift with the indicator; classify the other's filtered
   acceleration (yield: ≤ -0.5 m/s² held 0.5 s) within 0.5-1.5 s; `hold` → `yield` only before
   the commit point (the last point from which the car can still stop before the zone); after it,
   continue. A creep that gains nothing within 3 s steps down to `yield`.
8. **Act** (adapter). `go` → RTC approve for the module that owns the zone (intersection, lane
   change, stop line); `yield`/`wait` → RTC deny and, if needed, a velocity limit; `stock` → leave
   Autoware alone. Latch ≥ 1 s unless the band or the role changes beyond the hysteresis. The
   guards get hysteresis too: a `go` is latched for the launch dead time, and the zone guard and
   the "moved off its line" test are filtered over 2-3 samples (074 C8: unfiltered, they flipped
   up to 141 times a run at noise ×2 and caused contacts).
9. **Contracts** (`check(contracts, inputs)`). Each a predicate with a fallback: perception latency
   ≤ 0.45 s; reach ≥ v_max × (turn time + 1 s) for the left turn; track stable ≥ 0.5 s; map and
   perception agree on the zone's lanes; others within their class bounds; the node's own cycle
   time. A failure → `stock`, or a minimal-risk stop if stock is unsafe there.
10. **Record** (`record(...)`). One line per decision change, on the decision topic and a log.

## The convention table

A YAML per jurisdiction. Rows by relation; each row maps roles to decisions and names the
second-round rule. A first version, from the studies:

```yaml
jurisdiction: generic            # a Taiwan file overrides the rows its traffic law sets
all_way_stop:                     # U3
  first: stopped_first            # perception fact, with the tie margin
  tie: right_before_left          # within the margin
  last: map_order                 # four at once: right-before-left is cyclic
cross_unsignalled:                # S2, C7
  first: map_priority             # major road goes
  tie: right_before_left
  last: map_order
merge_lane_drop:                  # S3, C7
  first: ahead_at_end
  congested: zipper
  last: map_order
narrow_passage:                   # S4
  first: obstruction_side         # the obstructed side yields
  tie: map_order
left_turn:                        # S5
  first: map_priority             # the turner yields
  go_if: gap_ge_critical          # and the reach contract holds
roundabout_entry:
  first: map_priority             # yield to circulating
  go_if: gap_ge_critical
cut_in:                           # C2, C3
  first: committed_claimant       # yield to a car whose lane change has begun, if the gap allows
contested: yield                  # a car that cannot or will not stop keeps the turn (C7)
```

## Interfaces with Autoware

- **Inputs:** `/perception/object_recognition/objects`, `/localization/kinematic_state`,
  `/planning/scenario_planning/trajectory`, the RTC status topics of the intersection, lane-change
  and stop-line modules, the Lanelet2 map.
- **Outputs:** the RTC cooperate commands (approve/deny per module and uuid), a velocity limit on
  the velocity selector's `max_velocity_candidates` (so it composes with SSv2's limit, rather than
  the AD API's single external limit), `/interaction_strategy/decision` (one JSON record per
  decision change).
- **Autoware does not read reservations** (the lab's V2X lease project): every effect goes through
  RTC and the velocity limit, so Autoware's own planner and shield stay in charge of the motion.
- RTC is switched to manual-cooperate for a driven module only while the node holds a decision
  that is not stock's, then back to auto. The switch is `interaction_strategy:=true` in
  `scensim_launch/vehicle_agent.launch.xml`, off by default (R9). Open: a watchdog, since a node that
  dies while a module is manual leaves that module stopped at its line instead of running stock.

## Contracts and fallback

| contract | predicate | fallback |
|---|---|---|
| latency | objects' age ≤ 0.45 s | stock |
| reach | detection reach ≥ v_max × (turn time + 1 s) on the conflicting approach | refuse the gap, creep (left turn, roundabout) |
| track | the conflicting object tracked ≥ 0.5 s | treat as phantom at its first position |
| map | the zone's lanes match perception's lane evidence | stock |
| others | observed accelerations within the class bounds | `contested` for that agent |
| self | cycle time ≤ 20 ms; node alive | stock (Autoware ignores a stale RTC command) |

## Parameters

`params.yaml` starts from the design's "Assumed parameters, to be measured" table (role-time error
σ 0.15 s within 30 m; tie margin m = e with e the centred 99 % quantile (C8: arena e99 ≈ 0.25 s,
bias 0.2 s; m 0.3), H2 ≥ e + 1.2 s (2.0 s), zone-guard margin 0.5 s; yield test; α per class; critical gap 6.5 s; reach contract;
braking bounds car 4, truck 3, two-wheeler 4 m/s², wet × 0.6; creep 1 m/s and 3 s; radar and UWB
figures). Each entry carries its source (`assumed`, or the run that measured it) and its held-out
error once measured. Roadmap 078's evaluation phase replaces them.

## Tests

- **Unit:** each function against hand cases from the scenario walkthroughs.
- **Property:** for random conflict sets and map orders, the order is total; no wait-for cycle
  survives `decide`; `ambiguous` never yields `go`; `go` never without a clear exit box; outside a
  band the decision is `stock`.
- **Model check:** C8's formal model of each convention row (safety and liveness, the two cars'
  readings disagreeing within ±e, two to four cars).
- **Parity:** the arena adapter reproduces the per-study results (U3, C7, S2, S3, S4) within D2's
  tolerance before it replaces their rules.
- **Live:** roadmap 078's test runs.

## What the build changed (078 P1, 2026-11-15)

- **Band mode per convention row.** `interval` (one car's latest exit before the other's earliest
  entry, by the margin) for the all-way stop; `entry` (arrival order, margin 0.2 s) for crossing,
  narrow passage and merge. The interval test alone makes the band about 2 s wide at a crossing and
  overrides S2's nearer-car-first.
- **Rounds 2 and 3** start from one common quiet epoch (latest rest + dwell + launch + see_delay)
  and are decided among every car at rest; the last resort comes before "another car wins round 2".
  Right-before-left between opposite approaches falls back to `map_order`.
- **Noise-robust guards:** the rest position is averaged over its samples; the in-zone and at-rest
  tests use the filtered "moved" test. The probe's watch keeps rolling at 0.4 m/s instead of
  standing. A fresh-rest guard holds a first go until a car that has only just stopped would be seen
  starting; a first go from rest is probed while another car still approaches.
- New parameters: `fresh_rest_guard`, `probe_first_moving`, `see_delay`, `probe_watch_speed`,
  `band_margin_entry`. A contested yield counts as stock-equivalent. The latency contract measures
  perception latency, not decision delay.
- **The all-way stop row stays stock with no V2X** (078 G0): with these changes it is still worse
  than stock Autoware in 1-4 of 12 mixed cells, because stock reads a waiting car as static.
- **Second round (078 V4):** band mode `gap` for `merge_lane_drop` (who leads by a headway now,
  `merge_horizon` 1.5 s, `merge_headway` 0.6 s); the entry band strict at m − e; a role-first go in
  an ambiguous band waits while the other's lead as read is ≥ m − e; `rest_tie` (both at rest: the
  role binds, the winner goes after the 3 s fresh-rest guard); `claim` at the narrow passage (a go
  that does not clear before the other could enter needs the other at rest), round two by
  `obstruction_side`; a queued car is covered by its leader's verdict; bystanders' starts and exits
  count for the quiet epoch (`Protocol.step(bystanders=)`); the merge act plans its end stop 1 m
  before the end after the brake's lag, merges ahead only on the full gap test before its commit
  point, and as B makes room once A's lane change is seen.
- **Watchdog** (contracts and fallback): the node publishes `/interaction_strategy/heartbeat` every
  cycle with the modules it holds manual and its limit. `interaction_strategy_watchdog`, a separate
  process, calls `enable_auto_mode` for them and clears the limit after 0.5 s without one.
- **Which rows are on (078 G0):** only `cross_unsignalled`. The all-way stop, narrow passage and lane
  drop are `stock`. **The limit of conventions without V2X:** a convention pays only where no
  waiting car must go first from rest against a car that cannot read it. A gap-test driver reads a
  car waiting by its role as static and restarts into it.

- **Phase 2, the crossing on scensim (078, 2026-11-16):** `enable_auto_mode/<module>` is
  `tier4_rtc_msgs/srv/AutoMode` (bool enable), and a cooperate command clears auto mode by itself.
  The self contract reads the median of the last 10 cycle times. The crossing row needs a commit once
  moving (`commit: on_move`): an uncommitted go from rest chatters go/wait. The strategy needs a
  mode hysteresis, because band-edge readings toggle RTC's mode. `role_time_error_e` 0.55 s and
  `role_time_bias` −0.24 s are measured on Autoware's tracks (EV4, held out). An occlusion contract:
  the other car is first tracked about 22 m out, so the node is stock until then unless a phantom
  stands at the occlusion edge. An RTC approve overrides the module's own `safe`, so a go
  approved while `safe` is false rests on obstacle stop alone.

- **Phase 2b (078, 2026-11-16):** `cross_unsignalled` commits a go begun from rest (`commit:
  on_move`). The commit gives way when the other car is seen starting while the ego can still stop
  comfortably (`commit_release: turn`); a plain commit failed G0. `mode_hold` 1.5 s: a held module
  stays manual until the decision has been stock-equivalent that long, and meanwhile the node sends
  what the module's own `safe` says. The entry band corrects the arrival read from a track by
  `arrival_bias` (reading − truth, −0.24 s), and tracks younger than 0.5 s are not read. The rest
  stamp read from a track lags (+0.7 s median, 0.45-3.55 s); `rest_stamp_bias` orders
  `stopped_first` only. **A guard never takes a correction that shortens it** (P2c: the corrected
  fresh-rest guard, 2.3 s, let 2 s-patience cars restart into the go). A go from rest stays latched
  until the car first moves (`go_latch_motion_timeout` 3 s). **Own rest:** Autoware stops before its
  own stop line, which can lie metres before the zone's edge, so the own rest window is wider than
  the other's (`own_rest_window`). **A partner that drops out of the conflict set** without being
  seen to leave keeps the last decision for `mode_hold`; it is not read as clear. Own pose:
  Autoware's `kinematic_state` is base_link (rear axle), so the node shifts it to the centre by
  `own_base_to_centre` from the spec. Autoware takes about 1.4 s from ACTIVATE at rest to motion,
  so a go from rest must stay latched until the car moves.

## Open questions

- The commit point for a lane change inside Autoware's module: where RTC can still deny.
- Whether a stop instant can be shared from UWB range rate (coop 118's idea), which would make
  `stopped_first` a shared fact between equipped cars.
- Taiwan's rules for scooters at all-way stops and narrow passages, for the Taiwan convention file.
