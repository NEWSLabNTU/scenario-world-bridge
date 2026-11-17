# 045: A failed `git submodule status` at a stack's start refused every run on that stack

**Severity**: medium (the run is refused before it starts, exit 3, with a message that blames
the build; restarting the stack is the only way out). **Status**: fixed on scensim-rename
(this commit) and on 081-run-testbench (f9a9ef5): a failed probe is recorded as unknown and
not compared.
**Found**: 2026-11-18, swb 241, the 080 exit run of Tutorial 1 at swb-c3 b163816 (scensim-rename).

## What happens

```
$ ros2 run scensim run scenarios/drive_to_goal.xosc
scensim run: refused: the stack 2026-11-18T03-16-22_seat0 was started from another project
state: the build's submodules (src/autoware_carla_bridge, src/autoware_isaac_bridge, ...,
third_party/cuda_ndt_matcher) differ; restart the stack
```

The stack was started by hand (`ros2 launch scensim_launch scensim.launch.xml`) in
`~/my_first_drive` at 03:16 on a host at load ~40 on 24 cores, and was ready at about 03:26.
Nothing was rebuilt and no submodule moved between the stack's start and the run. The
stack's `stack.json` fingerprint held

```
"build": {"scensim": "src:126f30b5...", "submodules": {}}
```

while the run's fingerprint, computed a few minutes later in the same checkout, listed all
11 submodules. The submodule names in the refusal are exactly the 11 the run found, with
nothing on the stack's side. A second stack, started at 03:29 with the load at ~31,
recorded all 11, and its run was accepted.

## Why

`stack.build_commits()` takes the submodule pins from `results.submodule_pins()`, which ran
`git --no-optional-locks submodule status` through `results._out()` with a 10 s timeout and
returned `{}` when the command failed or timed out. A failed probe was indistinguishable from
a checkout with no submodules, and `fingerprint_diff` compared `{}` against the run's 11 pins
as 11 differences.

The cause of the failure itself is not proven: `_out` swallowed it and the stack's console
(`stack-1.log` below) has no line about it. The same command takes 0.2 s on an idle host. The
stack's start runs it while CARLA, the world server and the bridge are starting on a loaded
host, which makes a 10 s timeout the likely cause.

## Fix

- `results.submodule_pins()` returns `None` (unknown) when the command fails or times out, and
  prints a WARN naming the command, the workspace and why. It returns `None` silently outside a
  git checkout (a release). Its timeout is 30 s.
- `stack.build_commits()` keeps `None` instead of turning it into `{}`.
- `stack.fingerprint_diff()` does not compare the submodules when either side is unknown, and
  says so through its `notes`. `run.check_stack()` prints that note as a warning, or appends
  it to a refusal made for another reason.
- Test: `test_stack.py::test_a_failed_submodule_probe_is_unknown_and_not_a_different_build`.

## Evidence

`/home/aeon/coop-c3/r080/exit/run3/` on newslab-server-241: `stack-1.log` (the first stack's
console), `t1-run.log` before the retry (the refusal), and
`~/swb-logs/my_first_drive/stacks/2026-11-18T03-16-22_seat0/stack.json`.
