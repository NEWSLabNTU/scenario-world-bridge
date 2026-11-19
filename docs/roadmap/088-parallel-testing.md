# 088: Parallel Testing

**Status**: opened 2026-11-18 (user). Step 1 done 2026-11-19 on host 133: 2 ego seats (1.8x), 3 no-ego seats. Step 2 (the dispatcher) on branch `088-dispatcher-2`, waiting for its live check. Model revised 2026-11-19 (user): one host, rollouts and many scenarios; several hosts dropped.

## Goal

**The model (user, 2026-11-19).** A user works on one host with the testbench's complete
source tree. A testbench run is one simulator world with any number of autopilots (pilots).
Massive runs come from two needs:

- **rollouts**: one scenario run many times (repetitions, seeds, parameter values), for
  statistics, sweeps or training data;
- **many scenarios**: a suite run as a whole.

Both are served on that one host by several **seats** at once, each seat one simulator plus
its pilots, without conflicts between them, with the same verdicts as one run at a time.
Several hosts are not part of the model.

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
3. **Rollouts.** The dispatcher's queue takes a scenario's rollouts as well as a suite's
   scenarios: `--repeat N`, a seed range, and parameter values (a list or a grid), each rollout
   one queue item with its seed and parameters recorded, the merged report grouping them per
   scenario (counts, rates, spreads). A seat's admission counts what the testbench runs in it:
   each Autoware pilot adds its VRAM and load (a three-pilot testbench is not one ego's seat).
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

## Step 2 design: the dispatcher (118, 2026-11-19)

`scensim run <suite> --seats <list|auto>`, a CARLA testbench only.

- **One worker process per seat.** The dispatcher (the `scensim run` process) expands the
  suite as today, then starts a worker per seat (`python -m scensim.dispatch worker`), each in
  its own session. A worker is the existing runner with one seat: `context(seat=n,
  start=True)` starts the seat's stack (its CARLA from `seats.carla`, started by the stack as
  `carla.root` does; the open question answered: the stack, not the dispatcher), then asks
  for one run at a time on a pipe (JSON lines), runs it with `run_spec`, and answers with the
  run's directory and verdict. A process, not a thread: each worker's SSv2, pilots and stack
  are stopped by its own SIGTERM path (`run.cli`), which only a process main thread has.
- **The queue** is the suite's specs in order (map order per seat is the stack's concern).
  A free worker gets the next spec; a run that ends `error` is retried as the suite's
  `retry:` says, on whichever seat is free next.
- **Admission.** A seat starts its stack only when the host has room for it: free VRAM on its
  GPU (`nvidia-smi`) at least `dispatch.vram_per_seat_mb` (3000, step 1's ~3 GB per ego seat)
  and the 1-minute load plus `dispatch.load_per_seat` (12) at most the core count; seats
  start one at a time, the next only once the last is ready and `dispatch.start_gap` s
  (30) after it, so one seat's start load is seen before the next. A seat that cannot be
  admitted waits, and the suite runs on the seats that were.
- **Overload, per run.** While a run goes, the dispatcher samples the host's 1-minute load;
  when it ends, it reads the seat's acb_bridge `Skipped N CARLA frame(s)` lines written since
  the run started. A run is overloaded when it skipped more than `dispatch.skip_limit` (12)
  frames or the mean load during it passed `dispatch.overload_load` (the core count). An
  overloaded run's verdict is the host's as much as the scenario's: the try stays on disk
  unlinked, the run goes back to the queue once, and one seat fewer takes work from then on
  (never below one; that seat is parked, its stack up and idle). A second overloaded try keeps
  its verdict. Each run's `dispatch.json` holds its seat, skips, load mean and max, and why it
  was overloaded; `summary.csv`'s `overload` column says the last, and `t3.py compare` adds
  it to a changed verdict's line, so an overload failure reads apart from a real one.
- **Retirement.** A seat whose stack does not become ready, whose pilots do not come back
  after a crash (`pilots_broken`), or whose stack pilot's launch did not load a composable
  (issue 064: after an `error` run the worker asks `timeline.missing_composables`, the check
  readiness uses) is retired: its worker stops its stack and ends, its run goes back to the
  queue once, and the report names the seat and why. No seat is restarted within a suite.
- **One report.** Runs go under the host's one log root (`new_run_dir` is safe across
  processes), so `runs list` sees them all; the suite's directory is the usual
  `<logs>/<testbench>/suites/<suite>/`, one link per run, and its `summary.csv` and
  `junit.xml` come from `runs.summarize` as for one seat. `scripts/t3.py compare` reads that
  file, so it runs unchanged. `--resume` skips linked runs as today.
- **Ending.** The dispatcher stops the stacks its workers started when the suite ends (they
  are the seats' own CARLAs; `--keep-stacks` leaves them up). SIGINT or SIGTERM stops every
  worker's group, each through its own stop path, then exits 130 or 143.
- **Refused:** `--seats` with `--seat`, `--shard`, a project not on CARLA, a seat table
  without `seats.carla` (two seats would share one CARLA), a seat named twice or outside the
  table, and `auto` when no seat is free.

### Step 2 live check, first try (133, 2026-11-19)

Branch 088-dispatcher 0b1eb1d, basic's town01 (6 runs) on seats 0 and 1, the host also
loaded by another session's builds: 502 s, 43.0 scenarios/h, 5 passed and
town01_pedestrian failed (the ego within 1 m of the pedestrian), step 1's 3-seat failure.
Seat 0 waited three times on the load before starting; no park, no retirement; nothing was
left running. Mean load 78.4 on 32 cores (peak 225), acb skipped 19 frames on seat 0's stack
over the suite and 7 on seat 1's. The verdict changed under overload and nothing caught it:
the load was checked only at a seat's start, the second seat started while the first was
still loading, and a limit of 20 skips was too lax. Hence the per-run overload above, the
limit of 12, and one seat starting at a time. NAS: `home/2026-11-19 088 dispatcher live (118,
run on 133)/`.

## Open questions

- Is the per-seat CARLA started by the stack (`carla.root` already starts one) or by the
  dispatcher?
- Do verdicts depend on host load through Autoware's wall-clock parts, and at what load?
- Do T3's comparisons run unchanged on a parallel run's merged report?

## Not in scope: several hosts

A coordinator over several lab hosts (ssh, a shared queue, results on the NAS; 241's design
note, 2026-11-19) was dropped when the model was revised: a user works on one host. Kept here
as the option if a farm or a CI cluster ever needs it.
