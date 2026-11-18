# 048: `testbench check --live` blames the world server when the check tool is missing

**Severity**: low (a misleading message, no wrong verdict: the check cannot run either way).
**Status**: fixed on branch `048-live-check-tool-missing`. **Found**: 2026-11-18, swb 243's
review of 5a3be3c (081 edit 4).

## What happens

`testbench check --live` runs `ros2 run scensim_world_check scensim_world_check` (
`scensim/own_world.py`, `CHECK`): `hello` for the stock world server of a live stack
(`hello_findings`), `hello` then `conformance` for a testbench's own server (`conformance`).
On an install without the `scensim_world_check` package, `ros2 run` exits 1 printing
`Package 'scensim_world_check' not found`. That is not an `OSError`, so:

- `hello_findings` reports `the stack's world server on tcp://127.0.0.1:<port> did not answer
  Hello: Package 'scensim_world_check' not found`, with the fix "restart the stack";
- `conformance` reports the own server as `did not answer Hello within 60 s`, after waiting
  for nothing.

Restarting the stack, or fixing the server, does not help: the tool is missing.

## Fix

A check whose output says `ros2 run` found no package or executable (`Package '...' not
found`, `No executable found`) did not run. Both paths report it as one host finding, the same
as a check that cannot be started at all: "the probe did not run (...): source scensim's
setup.bash (it provides scensim_world_check)". Tests stub the check with a command that exits 1
printing `Package 'scensim_world_check' not found`, for the Hello probe and for conformance.

Also from the review: the fixture world server (`test/fixtures/world_servers/
kinematic_world.py`) despawns with `pop`, so an unknown handle cannot raise inside it (the
shell refuses one before it reaches the world, so this was latent).
