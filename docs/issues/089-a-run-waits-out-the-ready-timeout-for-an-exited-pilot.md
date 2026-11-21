# 089: A run waits out its ready timeout for a pilot whose launch has exited

**Severity**: medium. A suite whose ego cannot start costs its whole ready timeout per run,
and the error does not name the cause. **Status**: fixed (scensim 4e391d4b..6b30e70e, swb 139).
**Found**: 2026-11-21, swb 243, an overnight T3 suite (issue 073, s5 on CARLA 1.0).

## What happens

Each run of the suite started its ego pilot (`lifetime: run`, an Autoware ego on acb). acb's
`acb_bridge` exited 1 within a second of starting:

```
Error: ConfigError("CARLA version mismatch: the server is 1.0 but this binary was built for
CARLA 0.9.16. ...")
INFO [node:/acb_bridge] exited (code 1) and was declared on_exit=Shutdown: shutting down the launch
```

The pilot's play_launch shut the launch down and exited 17 s after the start. The run did not
notice. It went on polling the ego's readiness until its ready timeout, then failed with:

```
pilot ego never became ready (1800 s): [ego-health] not ready: nothing publishes
/api/operation_mode/state after 10s. Start the ego stack first (`just ego-av`) and wait for
'Startup complete'.
```

The suite had 12 shards of 2 runs, each run attempted once more after the error: 24 run
directories, each one 1800 s of waiting, 12 hours for a cause written to the pilot's log in the
first second. The message names neither the exit nor acb's error.

The binary mismatch itself was the harness's: the overnight script put the 1.0 `acb_bridge`
into this workspace's install only, and the ego's overlay ran acb's own install, built for
0.9.16 (CLAUDE.md, "Use just build"). That is fixed in the script. This issue is the run's
side: whatever ends a pilot's launch, the run should say so at once.

## Cause

`run_spec`'s ready loop asks four things of each waiting pilot, in turn: composables missing
(issue 064), loads stalled (issue 088), the launch's own failure line, and crashing nodes
(`CrashWatch`, a node dying `CRASH_LIMIT` times). None of them asks whether the pilot's
process is still alive. A required node that exits once, and takes the launch with it, is
neither a crash loop nor a failed load. So nothing fires, and readiness is polled against a
stack that no longer exists.

## Fix

- **A run's pilot** (`lifetime: run`): the run keeps each of its pilots' launch processes. One
  that has exited before it was ready ends the run `error` at once:

      pilot ego is not coming up: its launch exited (code 1) before it was ready: acb_bridge
      exited 1: Error: ConfigError("CARLA version mismatch: the server is 1.0 but this binary
      was built for CARLA 0.9.16. ...") (see .../logs/pilot-ego/console.log)

  The reason comes from `stack.first_error`. When play_launch names the node that exited
  (`ERROR [node:/acb_bridge] Exited with code 1`, or `exited (code 1) and was declared
  on_exit=Shutdown`), it is that node's own first error line. The node's err is the one
  play_launch's `ERROR Check <dir>` points to, else the newest `<stamp>/node/<name>/err` beside
  the console: acb's reason is there, not in the console. Otherwise it is the console's first
  error line (ros2 launch's `[ERROR]`, play_launch's bare `ERROR` level, a traceback's last
  line, a panic). The "process has died" and "Exited with code" lines are used only when nothing
  better is there.
- **A stack's pilot**: it is not only named in an event, as was wanted. A pilot that ends before
  it was ever ready on its start is counted. After `EXITS_UNREADY` (2) such ends the keeper stops
  respawning it and stops the stack, with the same message as its error (event `pilot.failed`).
  A launch that exits on its own does so again on every respawn. The count starts again once the
  pilot is ready, and exits after ready are respawned as before. A transient failure at start (an
  OOM) now costs the stack after two tries, as 088's `LOAD_RESTARTS` does.
- **The runner waiting for a stack** (`check_stack`, `--wait`): a stack stopped with an error is
  refused at once with that error, not at the deadline.

Tests: `test_run.py` covers a run pilot whose launch prints a version-mismatch error and exits 3
(the run ends `error` within seconds, naming pilot, code and line), and a stopped stack refused
at once despite `wait=600`. `test_stack.py` covers a stack pilot exiting 3 twice (the keeper
returns 2 and the stack's error names it), and `first_error`: on ros2 launch's lines, on
play_launch's `Check` form with the node's err (from this suite's console and err), on the
`on_exit=Shutdown` form with the err beside the console, and with no err. Checked by swb 243 on
the console above: "its launch exited (code 1) before it was ready: acb_bridge exited 1: Error:
ConfigError("CARLA version mismatch ...")".

## How it was confirmed

`/var/tmp/aeon/i073-10-logs/s5/runs/2026-11-20T19-39-50_s5_v10_g5.5_r0` on newslab-server243:

- the pilot's console, `logs/pilot-ego/console.log`, ends at 19:40:07 with play_launch's
  CPU summary, after `acb_bridge` exited (code 1) at 19:39:51;
- `logs/pilot-ego/2026-11-20_19-39-51/node/acb_bridge/err` holds the version-mismatch error;
- `timeline.jsonl` has `pilot.start` for the ego and nothing after it for that pilot;
- the run's error came at the ready timeout, 1800 s after the start, and the same in all 24
  runs of the suite.
