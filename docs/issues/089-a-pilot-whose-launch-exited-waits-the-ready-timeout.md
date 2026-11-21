# 089: A pilot whose launch had exited left the run waiting out the ready timeout

**Severity**: medium. It cost one night: 24 runs, each waiting 1800 s for a pilot that could
not come up, and nothing said why until each timeout. **Status**: fixed (scensim). **Found**:
2026-11-21, swb 243.

## What happens

A `lifetime: run` pilot's launch exited by itself before it was ready. In this case acb_bridge
exits as it must on a CARLA version mismatch, and that ends the launch. The run kept polling the
pilot's ready check until the ready timeout. Each run of the command did the same, so the night
went by.

A `lifetime: stack` pilot that exits so at stack start is respawned by the keeper. It exits again
on every respawn, and the stack never becomes ready within its timeout.

## Why

The run's wait checked readiness, failed composable loads (064), stalled loads (088), a launch
that fails at once (a missing package, an exception in the launch description) and crash loops.
It never asked whether the pilot's process was still there. The keeper treated a pilot that
exits before it was ever ready like one that crashed after: respawn, for ever.

## Fix

- **A run's pilot**: the run keeps each of its pilots' processes. One that has exited before it
  was ready ends the run `error` at once. The message names the pilot, the exit code and the
  first line of its console that names an error. A "process has died" line counts only when
  nothing better is there:

      pilot follower is not coming up: its launch exited (code 3) before it was ready:
      [acb_bridge-1] [ERROR] [...] [acb_bridge]: CARLA version mismatch: ... (see <console>)

- **A stack's pilot**: a pilot that ends before it was ever ready on its current start is
  counted. After `EXITS_UNREADY` (2) such ends, the keeper stops respawning it and stops the
  stack with that message as its error (event `pilot.failed`). The count starts again once the
  pilot is ready. Exits after it was ready are unchanged and still respawned.
- **The runner waiting for a stack** (`check_stack`, `--wait`): a stack that has stopped with an
  error is refused at once with that error, not at the deadline.

## Test

- `test_run.py`: a run pilot whose launch prints a version-mismatch error and exits 3 ends the
  run `error` within seconds, naming the pilot, the code and the error line. A stopped stack with
  an error is refused at once despite `wait=600`.
- `test_stack.py`: a stack pilot that exits 3 at once is started twice, then the keeper returns
  2, the stack is `stopped`, and its error names the pilot, the code and the error line.
  `first_error` prefers an error line to the "process has died" ones.
