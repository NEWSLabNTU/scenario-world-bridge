# 047: A goal handed back by the relay can re-arm after a STOP that came later

**Severity**: medium (rare: a STOP or clear_goal must arrive while a goal handed back to a
re-registered agent is still unanswered; then the next agent of the entity is given a goal that
was stopped, and a stopped car may drive again or its run may end as a lost goal). **Status**:
fixed, main 7b21bec. **Found**: 2026-11-17, swb 241,
`just test` during the 080 rename (a flake in `test_replay.py`); the same race is in main
before the rename (the rename changed only the test file's import).

## What happens

`src/scenario_agent_relay/test/test_replay.py::test_a_stopped_car_or_a_new_episode_is_not_given_the_old_goal`
failed once in a full `just test` with the host at load ~31 on 24 cores
(`/home/aeon/coop-c3/r080/test3.log` on newslab-server-241):

```
>       with pytest.raises(OSError):   # nothing is handed back: a timeout, no command
E       Failed: DID NOT RAISE <class 'OSError'>
src/scenario_agent_relay/test/test_replay.py:121: Failed
----------------------------- Captured stderr call -----------------------------
entity 'ego': agent gone (agent closed the connection)
entity 'ego': its agent registered again; its goal was handed back (set_goal -> OK)
entity 'ego': agent gone (agent closed the connection)
--------------------------- Captured stderr teardown ---------------------------
entity 'ego': agent gone (relay stopping)
entity 'ego': goal lost: its agent registered again (it was STOPPED) and refused its goal
handed back: no reply no agent registered for entity 'ego'
```

Run alone it passed 2 of 3, and the full rerun passed. The test's third agent `c`, which
registers after a STOP, expects nothing within 0.8 s. Instead it was handed the old goal, and
at teardown the relay was still trying to hand that goal back to a STOPPED entity. The 0.8 s
is not the cause: the goal was standing when it should not have been.

## Why

`AgentServer._replay` (`scenario_agent_relay/server.py:307`) runs on its own thread when an
entity with a standing goal re-registers. It sends the speed limit, then `SET_GOAL`, through
`AgentServer.command`, which records an accepted command with `_note` only **after**
`session.command` returns with the agent's reply (`server.py:272-273`). `_note` sets
`_standing[entity]["goal"]` on an OK `SET_GOAL` and clears it on `STOP` or `CLEAR_GOAL`
(`server.py:277-291`).

The test (and the scenario, in a run) sends `STOP` as soon as the agent has answered the
handed-back `SET_GOAL`. Two threads now race:

1. The replay thread: `session.command(SET_GOAL)` has returned, and `_note(SET_GOAL, OK)` is
   next.
2. The command thread: `session.command(STOP)` returns, then `_note(STOP)` clears the goal.

When the replay thread is descheduled between its return and its `_note` (likelier on a loaded
host), `_note(STOP)` runs first and `_note(SET_GOAL)` then restores the goal. The entity is
STOPPED with a goal standing, and its next agent is handed that goal. A `RESET`, `TELEPORTED`
or `EPISODE_END` that clears the standing commands races the same way.

## Fix (design)

- Order the notes by when each command was **sent**, not when its reply arrived. `_note` takes
  the command's sequence number (the id `next(self._ids)` already gives each command) and
  keeps, per entity, the sequence of the last goal-changing note. A `SET_GOAL` whose sequence
  is older than a later `STOP`, `CLEAR_GOAL` or episode boundary is not recorded.
- `_replay` checks before each send that the goal it is handing back is still the standing one
  (it may have been cleared since the agent registered).
- Test: a deterministic version of the race. Hold the replay thread between its `SET_GOAL`
  reply and its note (an event in a test hook), note a `STOP`, release it, and assert that no
  goal stands and that a new agent gets nothing. The existing test keeps its 0.8 s.

## Evidence

- `/home/aeon/coop-c3/r080/test3.log` on newslab-server-241 (the failing `just test`, 2026-11-17).
- `src/scenario_agent_relay/scenario_agent_relay/server.py` at ac5b709: `command` (250-273),
  `_note` (277-291), `_replay` (307-333).

## Fix

- `AgentServer.command` takes the command's sequence number before sending it and hands it to
  `_note`, which keeps per entity the sequence of the last goal note and of the last speed
  note. A note older than the last one of its kind is dropped, so a goal sent before a STOP,
  a clear or an episode boundary no longer stands after it, whichever reply came back first.
- `_replay` checks that the goal (and the speed limit) it read at registration still stands
  before each send; a goal cleared meanwhile is neither handed back nor reported lost.
- Tests (`test_replay.py`): the race held open deterministically through the
  `_before_note` test hook (a STOP noted while the replay thread waits to note its goal), a
  goal cleared before the re-registered agent reports, and the note order on its own.

## Found on the way: a test that reads a state before the first one arrives

Running the relay tests 24 times for the fix, one run failed in
`test_sdk_client.py::test_reset_round_trip_and_state_before_reply` with
`TypeError: 'NoneType' object is not subscriptable`: its `wait_until` polls
`relay.session("ego").state["phase"]`, and `session.state` is None until the agent's first
state arrives, so a poll that comes first raises instead of waiting. A test-only race, not the
relay's; the polls in `test_sdk_client.py` and `test_episodes.py` now read
`(state or {}).get("phase")`.

## Found on the way: a load-sensitive runner test

`src/scensim/test/test_run.py::test_sigterm_stops_what_the_runner_started` failed once in a full
scensim test run on newslab-server-241 at load ~12 (2026-11-18, testing the issue 046 branch,
which does not touch it). It passed run alone and in a rerun of the three runner test files
(85 passed). Load-sensitive, not fixed here; noted so a failure under load is not taken for a
regression.
