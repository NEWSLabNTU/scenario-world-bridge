# 080: keep levels drop the files the judges read

**Severity**: high. A failed run lost the evidence its own judge had scored, so the failure could not be looked
into. **Status**: fixed on branch `issue-080`; it lands on main after 0.2.1 is published. **Found**: 2026-11-20 by
243: an s5 run with a contact kept no `coop_a.jsonl`.

## What happens

`logging.keep` prunes a finished run to a level per verdict (`runs.apply_keep`). Each level keeps a fixed set of
top-level names (`runs.LEVEL_KEEPS`): `result.json`, `judges/`, `contacts/` and the like at `result`, `logs/` at
`standard`, `record/` at `full`. Anything else in the run directory is kept only at `debug`. A `command` pilot that
writes its own log into the run directory (s5's oncoming pilot writes `coop_a.jsonl`, c2b's two pilots
`coop_a.jsonl` and `coop_b.jsonl`) therefore lost it at every level below `debug`. That included failed runs:
s5's `keep` is `{passed: standard, failed: full, error: full}`. The judge (`judges/s5_score.py`) had read the file
and written its record, but nothing was left to check the record against. 243 lost the evidence for an s5 contact
this way.

## Cause

Two gaps in `apply_keep`:

- a failed or error run, or one with a contact, was pruned like any other, to the level `keep` names for its
  verdict;
- the levels know scensim's own files by name, and nothing told them which other files the testbench's judges
  read.

## Fix

- **Failed, error or contact: everything.** A run whose verdict is `failed` or `error`, one a judge found `failed`
  or `error` (it used to keep what that verdict keeps, roadmap 077 G52), and one whose contact report names a
  contact of the ego (`contacts/*.json` with a non-empty `contacts`) keep everything, whatever `keep` says.
  `--capture result` still records only the result level, as asked.
- **What the judges read.** A judge declares the run files it reads in a new key, `reads:` (paths or globs
  relative to the run directory, checked like the other judge keys). A run kept at `standard`, `full` or `debug`
  keeps every top-level entry those patterns name; one kept at `result` does not. The judge was the better fit
  than a testbench-wide list: what a file is kept for sits beside the program that needs it, as `needs:` already
  does for what a run must record. s5's and c2b's judges now name their pilots' logs.
- **Docs.** `docs/reference/project-file.md` (`logging.keep`, the judge's `reads`) and
  `docs/reference/results-and-logs.md` (the other files of a run directory, and the runs that keep everything).

`scensim runs prune --to <level>` is unchanged: it is an explicit command and prunes what it is told.

## Tests

`test_runs.py`: `test_a_failed_run_keeps_everything_whatever_keep_says` (failed and error, with `keep` at `result`
and `standard`), `test_a_run_with_a_contact_keeps_everything` (an empty report does not count),
`test_what_the_judges_read_is_kept_with_the_logs` (kept at standard and full, not at result) and
`test_a_judges_failure_keeps_everything`; `test_judges.py::test_reads_names_the_run_files_a_judge_reads`;
`test_run.py::test_a_judges_reads_are_kept_with_the_logs` (a whole run on the test stack). Each fails on the old
code.
