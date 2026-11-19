# 066: czp's self contract hands a committed go to stock when the decision runs slow

**Severity**: high. A crossing car that has been told to go, and is moving into the zone, is handed
back to stock Autoware mid-crossing because its decision took 20 ms, and the other car can go first
into it. **Status**: fixed on p2g-launch (roadmap 078 P2g). **Found**: 2026-11-19, swb 133's P2g
scensim grid (NAS `logs/2026 SSv2 on CARLA/2026-11-19 078 P2g scensim grid (133)`), analysed by 241.

## What happens

On the P2g grid (Town05's unsignalled crossing, Autoware as car B, 64 runs) czp had 2 contacts in
32 runs against stock's 0 in 32. One of them, `on_dt0.3_t1.5p5_s6`, is the self contract's:

```
2642.2  creep   probe:watch        self 0.018 ok
2643.7  go      rest_tie+probe     self 0.018 ok      (the probe done; the car starts from rest)
2645.1  go      round2             self 0.018 ok
2645.4  stock   contract           self 0.020 FAIL    (median of 10 decision cycles > 0.020 s)
2646.3  go      inside             self 0.019 ok
```

At 2645.4 the car was moving under a go from rest, which the crossing row commits on the move
(`commit: on_move`), but a failed contract is never latched or committed over (act.py), so the
decision went to stock for that tick, and the other car went first.

The self contract failed in 219 of 891 logged decision lines, in 25 of 32 runs, after 987e2fd had
already narrowed its reading to the decision's own time (`Strategy.last_cycle`, not the whole tick).

## Cause

Two, both in how the self contract reads the decision's time.

1. **Warm-up.** The window's median is read from the first tick, so the second tick reads one
   sample: the first cycle, which is cold (20-26 ms in an offline replay of the run on 241, then
   0.6 ms). 25 of the 32 runs fail at exactly t0 + 0.1 s. Harmless there (no conflict yet), but noise.
2. **A bound with no margin, on wall time.** After the warm-up, failures appear only in the left
   t1.5p5 runs (17 of 20), 25-50 s into the run, during the long mutual-rest interaction. They never
   appear in the hold or right runs (0 of 12), even at the same host load. The decision's p99 is 18-19 ms even on the
   quiet runs and 20-65 ms in the interaction; the window's median sits at 0.016-0.021 s for
   seconds and crosses the 0.020 bound every few ticks. The time is wall time (`perf_counter`), so
   preemption counts: the host's load was 52-113 (mean 64, max 177), and every process of the
   user ran at nice 19 during the grid.

What turned a flicker into a contact is the act's rule that a failed contract hands the car to
stock at once, whatever the decision held. That rule is right for a node that is not alive; for one
that is merely slow it trades a committed crossing for a policy switch mid-zone.

## Fix

- **F1** The self contract judges a full window only (10 decision cycles); until then it is ok.
- **F2** Hysteresis: the contract fails when the window's median exceeds the bound, and recovers only
  when the median is below 0.75 of the bound and it has failed for at least `mode_hold` (1.5 s).
- **F3** A committed go never changes hands for slowness: a self-contract failure on a live node
  (and no other contract failing) leaves a go that has passed its commit point, or is committed on
  the move, as it is, until the car is out of the zone. A node that is not alive still hands over at
  once, as does any other failed contract.
- **F4** R10's 20 ms judges the decision's CPU time (`time.thread_time`), its window median; wall
  time is judged only against the tick period (100 ms at 10 Hz), as liveness: a window whose wall
  median exceeds the period is a node that is not alive. The record's self value is the CPU median;
  both times are summarised at shutdown (`decision_cpu_p50_ms`, `decision_cpu_p99_ms` beside the wall
  ones).

The second P2g contact (`on_dt-0.3_hold_s2`) is not this issue: the other car was first tracked
10.5 m from the zone (15.8 m in stock's twin run), too late for the car at 8 m/s to stop.
