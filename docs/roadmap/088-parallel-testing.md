# 088: Parallel Testing

**Status**: opened 2026-11-18 (user). Step 1 (the measurement) runs on host 133 after 078's grid.

## Goal

Run one suite's scenarios on many headless simulators at once, without conflicts between them,
so that a large suite finishes in a fraction of the time and gives the same verdicts as a run
one scenario at a time.

## What already exists

The isolation is in place; what is missing is the scheduling.

- **Seats** (`scensim seat`, `src/scensim/scensim/seat.py`, `docs/design/projects-and-pilots.md`,
  "Seats"). Seat `n` of a host's table has its own scenario domain and pilot domains, SSv2 port
  (its sensor-release sockets +1, +2), relay port (+5), world server port (+36), web UI ports,
  GPU, and, with `carla:` in the table, its own CARLA port. A seat is held by a `flock` on
  `<seat_locks>/seat-<n>.lock`, reclaimed when its holder dies; a seat whose ports something
  else serves is refused (077 G30).
- **Per-run output**: each run writes under its own run directory (SSv2's
  `output_directory` included), so results do not collide.
- **Headless CARLA**: `-RenderOffScreen` with no `DISPLAY` serves RPC (CLAUDE.md).
- **Simulated time**: every stack runs on its world's clock, so a loaded host slows a run
  rather than breaking it, up to the point where Autoware's own timing fails (below).

## Limits

- **Domains**: 0-57 (the localhost Cyclone profile's DomainGain 1000). At a stride of 3 that is
  19 seats per host; this is not the limit that binds.
- **CPU**: an Autoware ego is ~90 `component_node` processes. Past the core count, Autoware
  misses its deadlines and stops (seen at load 68 on 32 cores), which reads as a scenario
  failure. This is the binding limit, and the measurement must find it.
- **GPU memory**: CARLA ~2 GB per instance at Epic; Autoware's detectors up to ~2 GB
  (full CenterPoint).
- **Isaac**: its world holds ~17.5 GB of RAM; at most one per host. Parallel testing is
  CARLA-first.

## Steps

1. **Measure (133, after 078's grid).** One CARLA suite with an Autoware ego, run on 1, 2, 3 and
   4 seats at once on one host, each seat with its own headless CARLA. Record:
   - throughput (scenarios per hour);
   - load, RAM, VRAM and the I/O of the log disk;
   - each scenario's verdict and key metrics against the single-seat run.

   The same at 1-8 seats without an ego (SSv2 and NPCs only).

   The usable seat count is the largest at which no verdict differs from the single-seat run
   and the ego's control metrics stay within their thresholds.
2. **Dispatcher.** `scensim run <suite> --seats <list>` (name to be settled): a queue that gives
   each free seat the next scenario, starts and supervises the seat's own CARLA (restart on a
   crash, a world load per scenario), admits a seat only when its GPU memory and the host load
   allow, and merges the results into one suite report.
3. **Across hosts.** Lab hosts each run their own seats from a shared queue, results on the
   NAS.
4. **Determinism check.** The same suite twice at the chosen seat count; the verdicts and their
   spread against the single-seat runs (the arena is known not to repeat per seed; 078 P2e).

## Open questions

- Is the per-seat CARLA started by the stack (`carla.root` already starts one) or by the
  dispatcher?
- Do verdicts depend on host load through Autoware's wall-clock parts, and at what load?
- Do T3's comparisons run unchanged on a parallel run's merged report?
