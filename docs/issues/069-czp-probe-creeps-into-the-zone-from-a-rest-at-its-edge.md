# 069: czp's probe creeps into the zone when the car rests at its edge, and its go meets a patient car's restart

**Severity**: high. After a mutual rest at an unsignalled crossing, czp's tie-break go is meant to
creep about 0.5 m, then hold and watch for 1.5 s so that the other car's start is seen first. When the
car rests about half a metre from the zone, the creep carries it into the zone. The watch never holds
the car, and the go lands at the moment a 5 s-patience car restarts. **Status**: open. Part 1 (the probe's
stop margin) is fixed on p2g-launch; part 2 (the tie's timing) is being screened. **Found**: 2026-11-20, swb 133's P2g rerun (NAS `logs/2026 SSv2 on CARLA/2026-11-19 078 P2g
rerun left t1.5p5 (133)`), analysed by 241.

## What happens

The rerun of P2g's left t1.5p5 cells (seeds 8-11, 40 runs at `3505a08`) has czp 1 contact in 20 and
stock 0 in 20, so G0 fails in cell dt0.3_t1.5p5. Issue 066's counts are 0 and 0. The contact,
`on_dt0.3_t1.5p5_s8`, is not 066's chain: the self contract never failed in that run, and no contract
handed a decision to stock.

czp's record (car B, Autoware) against the arena's car A (`t1.5p5`: gap 1.5 s, patience 5 s), in the
scenario's time:

```
1360.8  wait   ambiguous      map priority 1 against 1
1363.6  yield  contested
1364.4                        A at rest 1.02 m out, B at rest 0.56 m out
1364.9  wait   rest_tie_wait  B's turn; it waits the rest guard (3.0 s)
1367.9  creep  probe:creep    B moves 0.7 s later (Autoware's launch)
1369.4                        A commits (patience, 5.0 s); B 0.39 m out, 0.45 m/s
1369.8  creep  probe:watch    B 0.14 m out at 0.81 m/s: too late to hold
1370.0                        B enters at 0.98 m/s; A starts
1370.2  go     inside         inside: the probe no longer applies
1372.0                        both at rest in contact (B 2.08 m in, A 1.45 m in)
```

A's safety layer braked lightly (brake 0.22-0.33), then stopped acting once A was in the zone itself
(`crossing_game.Policy.step` returns at `u >= 0`), and A drove on at throttle 0.6 into B.

The same overrun is in P2g's grid, in `on_dt0.3_t1.5p5_s7`, without a contact:
- B rested 0.66 m out;
- the creep started 3.6 s after A's rest;
- A committed 5.0 s after its rest, during B's creep;
- B was inside 0.4 s after the watch began.

There, A's safety layer held A at -0.95 m.

Of the probes in both grids that ended in a go, 2 in 8 ended "inside" rather than after the watch.
In both, A's patience restart fell inside B's creep:

| grid | probe ended done | inside | aborted |
|---|---|---|---|
| rerun (grid066) | 3 | 1 | 21 |
| P2g (grid) | 3 | 1 | 10 |

## Cause

Two things add up.

1. **The creep cannot end before the zone from a rest at its edge** (`probe.py`).
   - The creep runs until the car has moved `probe_creep_dist` (0.5 m), or is within 0.35 m of the
     zone. From a rest 0.56 m out, either way the watch begins with the car moving at about 0.8 m/s,
     0.1-0.3 m from the edge.
   - The watch's DEACTIVATE asks Autoware to stop. From 0.8 m/s, with Autoware's lag, that needs more
     than half a metre.
   - So the car enters the zone. The `inside` go is exempt from the probe (078 P2g), and the car is
     committed.
2. **The tie-break's go lands on a 5 s patience.** The go comes at:
   - the rest guard, 3.0 s after the later rest;
   - plus Autoware's launch, about 0.7 s;
   - plus the creep, about 1.4 s.

   That is about 5.1 s, and t1.5p5 restarts 5.0 s after its rest. The restart cannot be seen in time:
   seeing a start takes `see_delay` (1.5 s). Had the watch held B at its line, it would have run from
   about 1369.8 to 1371.3. A's start at 1370.0 is seen at about 1371.5, so even a held watch ends just
   before seeing it.

Stock Autoware rarely meets this here: it went first in 18 of its 20 runs, and had no contact.

## Fix

133's decisions (2026-11-20): part 1 yes; part 2 screened first, both options with numbers; the arena
stays as it is.

1. **czp, the probe.** The creep may only move the car while it can still stop before the zone.
   - The creep is asked to stop after `probe_creep_dist`, or as soon as the zone is within the stop
     margin at the car's speed, whichever comes first.
   - The stop margin is `probe_stop_margin + probe_stop_time x v` = 0.1 m + 1.0 s x v. It is the
     envelope of the 15 creeps stopped in both grids: requested at 0.06-0.20 m/s, they travelled
     0.04-0.25 m (Autoware accelerates about 0.4 s more, then brakes). Above 0.2 m/s it is
     extrapolated.
   - A car resting within the margin at `creep_speed` (1.1 m) does not creep: it watches at rest.
   - An `inside` reached while the probe is in its creep or watch is recorded as `probe_overrun`, so
     that it is counted, not hidden as a plain go.
2. **czp, the tie's timing.** The tie-break's go should not fall on the other car's patience restart,
   but czp does not know that patience. Two options; a kinematic screen (`crossing_game.py`) should
   pick one before code:
   - (a) the watch lasts until a restart from the other's rest + 5 s would have been seen;
   - (b) the priority car goes as soon as the guard ends, with no probe, so that it is in the zone
     well before a 5 s restart.
3. **The arena, not czp.** A's safety layer stops acting once A is in the zone (and
   `ARENA_HOLD_STOP` is False, as documented in 074). So any car of B's that is in the zone when A's
   patience restarts gets hit at low speed. It is the reference adversary for G0, so it stays as it
   is unless 133 decides otherwise; noted here because it decides whether such a meeting is a contact.

## Also seen in the run

- **Decision time.** The decision's CPU p50 was 15.5 ms in this run, against 5-6 ms in the other 19.
  The p99 was 18.75 ms, under R10's 20 ms. The run spent about 20 s after the contact with the two
  cars locked together, alternating `yield turn` and `stock` at the next zone. That stretch is the
  likely source, not checked.
- **No Autoware overlay.** Every run of both grids ran stock Autoware: the ego log says "Autoware
  overlay not built", in all 64 P2g runs and all 40 rerun runs, both arms. 133 saw apport crashes of
  stock nodes (pose_instability_detector, awapi_awiv_adapter, manual_lane_change_handler). The two
  arms are alike in this, so it does not bias G0. The bundle's `HOST_FROM` was meant to supply the
  overlay, but no step of `setup.sh` links it.
