# 075: czp hands a stopped decision to stock on a slow tick

**Severity**: high. A self-contract failure in the middle of a probe handed the car to stock. Stock read the other car
at rest as static and launched against it: the one contact of the issue 074 grid. **Status**: open (fixes on
p2g-launch; a new grid decides). **Found**: 2026-11-20, by 133, in the issue 069/074 grid (NAS `logs/2026 SSv2 on CARLA/2026-11-20
078 issues 069 074 grid left t1.5p5 (133)/grid074`).

## What happens

In run `on_dt0.3_t1.5p5_s17` (B is the ego, A is the arena car, rule t1.5p5), both cars stood at their lines, B 1.4 m
and A 1.0 m from Z (the truth). czp had decided `wait rest_tie_wait` at 1891.2 and, at 1894.2, began the tie's probe
(`creep probe:creep`). Times are sim time (CARLA elapsed + 1000 s), from `decisions.jsonl` and `coop_a.jsonl`:

| t (s) | what |
|---|---|
| 1894.2 | czp: `creep probe:creep`, ACTIVATE, limit 1.0 m/s; B still at rest |
| 1894.9 | the self contract fails, CPU median 0.021 s (it read 0.015-0.018 s since the conflict began at 1886.5); `stock contract`, the probe aborted; the module goes to auto mode and the limit is cleared. The module's status read `safe` (the stale scene, A static) |
| 1894.9-1895.1 | B starts: 0.008, then 0.097 m/s |
| 1895.5 | A's patience fires (A committed) |
| 1896.0 | A starts |
| 1896.75 / 1897.0 | A enters Z at 2.5 m/s / B enters Z at 1.6 m/s: contact |
| 1904.0 | the self contract recovers (median 0.007 s, the conflict over) |

B launched under stock 0.6 s before A's patience fired. In auto mode Autoware ignores RTC commands, so a DEACTIVATE
sent before the handover would not have stopped it.

In run `on_dt-0.3_t1.5p5_s18` the contract failed at 2185.6, at a median of 0.016 s, and recovered only at 2198.5:
12.9 s, the whole conflict. There B went first and nothing collided.

## Cause

Three things together.

1. **The handover.** Issue 066 exempted a committed go from the self contract: a slow but live node keeps it. Every
   other decision still went to stock at once (`act.latch`; `Strategy.cycle` then switches the module to auto mode
   and clears the limit). A stopping decision is exactly where stock is wrong at a mutual rest: it reads the other
   car at rest as static and goes. That is why the probe exists (074 C8 fix (c), 078 P2g).
2. **No margin.** A conflict cycle cost 15-19 ms of CPU against R10's 20 ms bound. Measured on 133 on s17's scene
   (`scripts/coop/czp_cycle_profile.py`, 600 conflict ticks, before the fix), the median was 17.7-20.7 ms and p99
   19.2-35.9 ms, depending on the host's load. 95 % of that time was the trajectory test in `Strategy.candidates`.
   It tested every zone on the ego's approach (about 30 at a Town05 junction) against every segment of the
   trajectory (59), and point-in-polygon on both ends of each segment, every cycle. Having no object around barely
   changed the cost (17.5-32.7 ms).
3. **A recovery threshold under the normal cost.** SelfWindow recovered only below 0.75 x bound (15 ms). That was
   under the conflict cycle's own cost, so once the contract failed it stayed failed until the conflict ended (s18).

The self window's flips also left no trace of their own. A flip appeared only on a decision line, and a stock to
stock tick writes none.

## Fix

- **The hold by contract** (`act.latch`, `cycle.Protocol.step`, `Strategy.cycle`). Say the self contract alone fails
  on a live node (`slow_only`), and the zone's previous decision is czp's own stopping decision: yield, wait,
  wait_second_round, or the probe's creep or watch, and not stock-equivalent. Then the decision becomes a stop with
  the reason `hold: contract`: the same yield or wait_second_round, or a wait (a creep or watch is aborted). The
  adapter keeps the module in manual mode with DEACTIVATE. It adds the stop limit `sqrt(2 b (s_entry - 2))`, whether a
  module covers the zone or not, and it never switches to auto mode. The hold ends in three ways:
  - the contract recovers: the protocol decides again;
  - the conflict set read this cycle is empty: the partner has left the zone, and the car goes to stock;
  - `hold_by_contract_s` (params.yaml, 5 s) passes since the hold began: the car goes to stock on that cycle's
    reading, and is not held again until a decision of czp's comes back.

  A node that is not alive, or any other failed contract, still goes to stock at once (R9). Issue 066's
  committed-go exemption is unchanged. `decisions.jsonl` records the hold as decision `wait`/`yield` with reason
  `hold: contract`, distinct from `stock contract`. On s17 the hold would have stopped B at 1894.9. A left Z at
  1898.7, so the conflict set emptied 3.8 s later, within the 5 s.
- **The margin** (`Strategy.candidates`). The trajectory test runs nearest first and stops once `max_zones` zones
  pass. It tests only the segments whose bounding box meets the zone polygon's. A segment that meets a polygon meets
  its bounding box, so the result is the same (`test_the_trajectory_prefilter_keeps_the_candidates`). On s17's
  scene, on the same host:

  | | conflict CPU p50 | p99 | no object p50 |
  |---|---|---|---|
  | before | 17.7-20.7 ms | 19.2-35.9 ms | 17.5-32.7 ms |
  | after | 0.71-2.27 ms | 1.1-3.3 ms | 0.59-1.16 ms |

  (four alternating runs each, load average 13-30 on 32 cores.)
- **The recovery threshold.** `contracts.yaml`'s self contract now carries `recover: 0.9`, and the node reads it:
  the contract recovers below 18 ms. That is above the old conflict cost's median (17.7 ms on an idle host), so a
  failure no longer sticks even at the old cost, and it is 5-25 times the new cost. The bound stays 20 ms. The
  2 ms between 18 and 20 ms keeps the hysteresis against flapping at the bound (066 F2).
- **The flips.** `SelfWindow.add` returns every flip of `ok` or `alive`. The node logs each one and writes it to
  `decisions.jsonl` as its own line (`event: self_window`, the tick time, the CPU and wall medians, bound and
  recover). The line has `zone: null` and `decision: null`, and `contracts.self` as on a decision line, so the
  readers of decision lines skip or count it.

Tests: `test_steps.py` (the hold for each stopping decision, its three releases, a dead node, another contract and a
stock-equivalent decision going to stock, the committed go unchanged, the flips, the recovery above the conflict
cost) and `test_strategy.py` (manual mode and DEACTIVATE held with the stop limit, then auto mode after the timeout,
which fails on the code before this fix; a dead node to stock at once; the prefilter's candidates).

## What it changes in earlier numbers

The grids' "stock by contract" counted every zone's `stock contract` line, the empty zones' too (s17 wrote three at
1894.9, two with an empty conflict set). The next grid's summary counts a handover only where the zone's previous
decision was not stock, and counts `hold: contract` separately.
