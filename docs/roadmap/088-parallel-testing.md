# 088: Parallel Testing

**Status**: opened 2026-11-18 (user). Step 1 done 2026-11-19 on host 133: 2 ego seats (1.8x), 3 no-ego seats; the dispatcher (step 2) is next.

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

## Step 1 result (133, 2026-11-19)

scensim main bd2d49a5; each seat its own CARLA 0.9.16 (Epic, offscreen; `seats.carla` base
3000, stride 10), started by the stack; each seat ran the whole suite (`scensim run ... --seat
n`, its own `--logs`), seats 30 s apart. The GPU was shared: another user's job held ~22 GB of
the 32 GB card throughout. NAS: `logs/2026 SSv2 on CARLA/2026-11-19 088 seats per host (133)/`.

Autoware ego, `basic`'s `town01` suite (6 scenarios):

| seats | wall | scenarios/h | load mean / peak | RAM peak | VRAM peak | verdicts |
|---|---|---|---|---|---|---|
| 1 | 572 s | 37.8 | 22.6 / 108 | 29.0 GB | 26.0 GB | 6/6 pass (reference) |
| 2 | 623 s | 69.3 | 56.5 / 170 | 40.4 GB | 29.4 GB | 12/12, as the reference |
| 3 | 765 s | 84.7 | 82.6 / 261 | 53.5 GB | 31.2 GB | 17/18: a pedestrian run failed (ego within 0.3 m); acb skipped 34 frames a seat |
| 3, again | 1720 s | - | 42.1 / 253 | 52.5 GB | 31.2 GB | one seat's Autoware failed a composable on CUDA OOM, was still counted ready, and timed out every run |

No ego, `bench` (NPC 10/20/50): 1, 2 and 3 seats all pass with near-identical metrics,
8.2 -> 15.8 -> 22.9 scenarios/h; each seat is paced by SSv2, not the CPU. Not pushed past 3:
a fourth CARLA would not fit beside the other user's job.

Findings for step 2:
- The binding limits are GPU memory (here shared) and CPU: ~3 GB VRAM and ~12 load per ego
  seat. CPU overload changes verdicts before VRAM runs out; acb's skipped frames (7-12 per
  suite at <= 2 seats, 34+ at 3) are a cheap overload signal for admission.
- A pilot whose composable failed to load still counted as ready (`126/127 composables
  loaded`), so the suite spent every run on retries: readiness must fail on a missing
  composable, and admission must check free VRAM.
- With `seats.carla`, each seat needs an explicit `--seat n` (`seat: auto` is refused) and its
  own `--logs`.

## Open questions

- Is the per-seat CARLA started by the stack (`carla.root` already starts one) or by the
  dispatcher?
- Do verdicts depend on host load through Autoware's wall-clock parts, and at what load?
- Do T3's comparisons run unchanged on a parallel run's merged report?
