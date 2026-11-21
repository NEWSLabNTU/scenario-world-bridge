# 091: scensim_agent's check test fails under host load

**Severity**: low. A timing-dependent test can fail a landing gate for a reason that has nothing
to do with the change under test. **Status**: fixed (scensim, swb 139): an agent defect the check rightly found. **Found**: 2026-11-21,
swb 243, a `just test` landing gate for roadmap 091 W13 to W16 (docs-only commits at the time).

## What happens

`src/scensim_agent/test/test_autoware_agent.py::test_passes_check_on_a_fake_autoware` runs
`scensim-agent check` against a fake Autoware on its own real-time clock (`_RealTimeFake`,
ticking every 0.05 s on a thread). Once, it failed one rule of sixteen:

```
PASS  stop answered within 2.5 s -- OK in 0.00 s
FAIL  stop: STOPPED before the reply -- phase IDLE
FAIL: 15/16 rules
```

The host's load average was about 25 (32 cores) at the time. The same test passed in the two
gates before on the same code, 5 times out of 5 run alone right after, and in the full rerun.
The log was `~/repos/c10/t3-10/landw13-test.log` on newslab-server243, overwritten by the
rerun; the lines above are copied from it.

## Why it can depend on timing

The rule reads the last state the agent pushed before its reply to `stop`
(`scensim_agent/check.py`, `command`: the states received at or before the reply's arrival). If
the agent's state push after the stop and its reply race on separate threads, a loaded host
can order them the other way, and the check sees the state before the stop. This is a guess
from the code, not confirmed.

## Fix

It was the first case: the agent did not always push the new phase before it replied. The
client's push loop (`Client.push_state`) built its state message outside any lock, then sent it.
When the loop was preempted between the two while a command ran, its stale state went out
after the command's own state and before the command's reply. The order was:

1. the loop builds IDLE;
2. `stop` runs and the worker pushes STOPPED;
3. the loop sends its IDLE;
4. the worker sends the reply.

The relay answers the scenario with what it shows, so it would have shown the old phase too. So
the test was racy, not flaky, and the check was right to read the last state before the reply.

The client now builds and sends a state under one lock (`_state_lock`, an RLock), and the
command worker holds it across its state and its reply. A state the loop built before a command
now goes out either before the command's state or after its reply, never between them. The
phase-change log line moved under the lock too, so each transition is logged once. Lock order:
`_state_lock`, then the agent's own lock (inside `state_message`), then `_send_lock`. Nothing
takes them the other way, so the change adds no deadlock.

The test is `test_client_state_order.py`. It forces the interleave with events instead of
waiting for load: without the fix the wire shows `[STOPPED, IDLE, reply]`, the reported
failure; with it, `[IDLE, STOPPED, reply]`. The race is too rare for load alone to show it.
With the check test and busy loops pinned to the same cores (12 loops on 4 cores, then 8 on 1
core), the original test passed 20 of 20 both with and without the fix. The forced test is the
evidence.
