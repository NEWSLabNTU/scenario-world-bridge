# 091: scensim_agent's check test fails under host load

**Severity**: low. A timing-dependent test can fail a landing gate for a reason that has nothing
to do with the change under test. **Status**: open, not investigated. **Found**: 2026-11-21,
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

## Fix (when taken)

Find whether the agent always pushes the new phase before it replies; if it does not, that is
an agent defect the check rightly finds, and the test is not flaky but racy. If it does, the
check should wait for the first state after the reply (within a bound) rather than read the
last one before it.
