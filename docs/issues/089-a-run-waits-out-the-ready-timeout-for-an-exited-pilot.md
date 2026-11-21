# 089: A run waits out its ready timeout for a pilot whose launch has exited

**Severity**: medium. A suite whose ego cannot start costs its whole ready timeout per run,
and the error does not name the cause. **Status**: open; the fix is handed to swb 139.
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

## Fix (wanted)

A `lifetime: run` pilot whose launch process has exited fails the run at once, naming:

- the exit code of the pilot's launch;
- the first error line of its console after the start (here acb's `CARLA version mismatch`
  line), or the node the launcher names as the one that ended it (`[node:/acb_bridge] exited
  (code 1)`).

A stack's pilot is the keeper's, which respawns it; there the same message belongs in the
keeper's event for the exit, so a pilot that dies at every start is named the same way.

## How it was confirmed

`/var/tmp/aeon/i073-10-logs/s5/runs/2026-11-20T19-39-50_s5_v10_g5.5_r0` on newslab-server243:

- the pilot's console, `logs/pilot-ego/console.log`, ends at 19:40:07 with play_launch's
  CPU summary, after `acb_bridge` exited (code 1) at 19:39:51;
- `logs/pilot-ego/2026-11-20_19-39-51/node/acb_bridge/err` holds the version-mismatch error;
- `timeline.jsonl` has `pilot.start` for the ego and nothing after it for that pilot;
- the run's error came at the ready timeout, 1800 s after the start, and the same in all 24
  runs of the suite.
