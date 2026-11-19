# 088: Parallel Testing

**Status**: opened 2026-11-18 (user). Step 1 done 2026-11-19 on host 133: 2 ego seats (1.8x), 3 no-ego seats. Steps 2 (the dispatcher) and 3 (rollouts, admission by pilots) done 2026-11-20: live check on 133 passed, 22 of 22, on main. Step 4 (determinism) done 2026-11-20 on 133: no verdict differs between one seat and two; durations repeat pass to pass. All four steps done. Model revised 2026-11-19 (user): one host, rollouts and many scenarios; several hosts dropped.

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
  GPU (`nvidia-smi`) and the 1-minute load plus the seat's own at most the core count (step 3
  sizes both by the testbench's pilots; step 1 measured ~3 GB and ~12 per ego seat); seats
  start one at a time, the next only once the last is ready and `dispatch.start_gap` s
  (30) after it, so one seat's start load is seen before the next. A seat that cannot be
  admitted waits, and the suite runs on the seats that were.
- **Overload, per run.** While a run goes, the dispatcher samples the host's 1-minute load;
  when it ends, it reads the seat's acb_bridge `Skipped N CARLA frame(s)` lines written since
  the run started. A run is overloaded when it skipped more than `dispatch.skip_limit` (12)
  frames (step 3 adds ticks over budget and CPU pressure, and drops the load during a run). An
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
  Step 4: at two seats no verdict changed, but simulated durations grew, most during
  stack start-up (1-minute load 80-137); step 1's three seats changed one.
- Do T3's comparisons run unchanged on a parallel run's merged report?

## Not in scope: several hosts

A coordinator over several lab hosts (ssh, a shared queue, results on the NAS; 241's design
note, 2026-11-19) was dropped when the model was revised: a user works on one host. Kept here
as the option if a farm or a CI cluster ever needs it.

## Step 3: rollouts and admission by pilots (118, 2026-11-20)

Branch `088-dispatcher-3` (088-dispatcher-2 rebased onto main after 0.2.0, plus this step).

- **Rollouts are queue items.** A suite already expanded to one spec per scenario, grid
  combination and repetition, each with its seed; step 3 adds what a rollout needs and lets
  `--seats` take it:
  - `--seeds A-B`: one run per seed from A to B (repetition k gets seed A + k), over a
    suite's own `seed:`; not with `--repeat`.
  - `--param NAME=[a,b,c]` on a scenario: one run per value (a grid axis, as in a suite;
    before, a scenario took the list as one value).
  - `cases:` in a suite: a list of parameter sets, each run as it is (a list, where
    `parameters:` is a grid); each case times the grid. A name may not be in both, and two
    cases may not be the same; `project check` names either.
  - A scenario (or a directory) run more than once gets a report directory like a suite's,
    `suites/rollouts-<scenario>/`, so `--seats` and `--resume` work for it too.
- **The report groups per scenario.** `runs.summarize` writes, beside `summary.csv`,
  `rollouts.csv` (per scenario and parameter set: runs, passed, failed, error, pass rate, the
  seeds as ranges, the duration's mean, sd, min and max, how many were overloaded) and
  `rollout_metrics.csv` (per group and numeric metric of `result.json`: n, mean, sd, min,
  max). One seat or many, the files are the same.
- **Admission counts the pilots.** A seat's need is its CARLA and SSv2
  (`dispatch.vram_base_mb` 2000, `load_base` 2) plus, for each `kind: autoware` pilot of the
  testbench (of the stack or of a run), `vram_per_autoware_mb` 1000 and `load_per_autoware`
  10. One ego is step 1's 3000 MiB and 12; a three-pilot testbench needs 5000 MiB and 32.
  The `dispatch ...` line names the need. Other pilot kinds count nothing.
- **Overload is judged on the run, not on the load during it** (133, from 243's reruns,
  2026-11-20: the 1-minute load peaked at 90-616 during runs from our own Autoware start-up
  with no harm, two_av at 616 with 0.4 % ticks over budget and no CPU stall; the overruns of
  7.8-11.5 % came from other work on the host). The load is looked at only before a seat
  starts. A run is overloaded when acb_bridge skipped more than `skip_limit` frames, more than
  `tick_overrun_limit` (5) % of its world ticks were over budget (`ticks_over_budget` of
  `frame_count` in its `result.json`), or the host's CPU was fully stalled
  (`/proc/pressure/cpu`, `full`) more than `psi_full_limit` (10) % of its wall time. The 10 %
  is a first value: the live check should look at the stall shares a clean run and a loaded
  one have. `overload_load` is gone; `dispatch.json` records the load (mean, max), the ticks,
  the stall share and the pilot count.
- **Settings by name.** A `dispatch:` key that is not a setting is refused, naming those
  there are (`vram_per_seat_mb`, `load_per_seat` and `overload_load` were only ever on the
  branch).

### Steps 2 and 3 live check (133, 2026-11-20)

Branch `088-dispatcher-3` b9427899, Autoware 1.5.0 ego, seats 0 and 1 (CARLA 3000 and 3010).
22 of 22 runs passed, no error, no overloaded run, no park, no retirement, nothing left
running.

| Part | What | Runs | Wall | Scenarios/h | VRAM peak |
|---|---|---|---|---|---|
| A | basic's `suites/town01.yaml`, seats 0,1 | 6 passed | 465 s | 46.5 | 6872 MiB |
| B | `town01_pedestrian --seeds 1-8`, seats 0,1 | 8 passed | 743 s | 38.8 | 6881 MiB |
| C | the same rollout, seat 0 | 8 passed | 1213 s | 23.7 | 3451 MiB |

- **Rollouts.** B's and C's `rollouts.csv`: pass rate 1.0 both, seeds `1-8`; duration mean
  130.27 s (sd 5.80) on two seats against 124.07 s (sd 1.36) on one.
- **Admission.** B and C each started after the previous part's stacks: seat 0 waited while
  the load fell from 71 (B) and 127 (C) to about 21 on 32 cores; the second seat started
  about 35 s after the first (`start_gap`).
- **Overload signals on a clean run.** acb skipped at most 5 frames (limit 12); at most
  18 of 600 ticks were over budget (3 %, limit 5 %); the CPU full stall share was 0.0 % in
  every run, with the 1-minute load at up to 160 during them. `psi_full_limit` stays at
  10 %: no run here stalled, so there is nothing to tune it against yet.
- **Fixed after it.** Try 1 retired every seat on "project check found problems" (a maps
  revision not in the asset cache): `--seats` now runs the stack's project check once before
  any seat starts and refuses there, naming `scensim assets fetch`. `scensim assets fetch`
  no longer rewrites a lock whose data is unchanged (a lock an older scensim wrote kept its
  header, and the rewrite left the checkout dirty).

NAS: `logs/2026 SSv2 on CARLA/2026-11-20 088 dispatcher steps 2-3 live (133)/`.

## Step 4: determinism (133, 2026-11-20)

scensim main ef9f1803, seats 0 and 1, Autoware 1.5.0 ego, in the order A1 B1 A2 B2: basic's
town01 suite twice (A) and `town01_pedestrian --seeds 1-8` twice (B), each run set against
a one-seat reference (step 1's one-seat suite run; the steps 2-3 check's one-seat rollout,
part C). 28 of 28 runs passed; no verdict differs from the reference (0 of 14 keys); no run
overloaded, nothing left running. 47.2, 38.0, 39.3 and 37.9 scenarios/h (A2 waited for the
earlier parts' load to fall), VRAM peak 6.7 GB, RAM peak 34 GB.

- **Pass to pass** (same run, seat and seed): the simulated duration repeats within mean
  1.7 s (max 5.1 s) on the suite and mean 0.9 s (max 3.0 s) on the rollout, 5 of 8 seeds
  within 0.02 s.
- **Two seats against one**: two seats' runs are longer, mean +5.9 s on both. Most of it is
  each seat's first run, which runs while the stacks start (1-minute load 80-137, against
  about 20 later): town01_ego_drive +13.2 s, town01_engage_state +7.1 s, the rollout's seeds
  1 and 2 +14.7 s and +9.6 s; later runs +1.5 to +6.5 s. One seat shows the same start-up
  effect, smaller (seed 1 at 127.2 s against 123.2-124.2 s).

So contention, mostly at stack start-up, adds simulated time through Autoware's wall-clock
parts (probably the engage timing), reproducibly per seat and seed, without changing a
verdict at two seats. A metric that reads durations or timing should be compared at the same
seat count. NAS: `logs/2026 SSv2 on CARLA/2026-11-20 088 step 4 determinism (133)/`
(`compare-split.txt`).
