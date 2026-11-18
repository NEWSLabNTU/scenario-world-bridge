# 054: `scensim run` does not resolve a suite's `pkg://` scenario

**Severity**: high for testbenches. 081 makes `pkg://<package>/<path>` the way a testbench
names a scenario from another package, and `testbench check` and `colcon build` accept such a
suite. But `scensim run` then refuses it, so no testbench whose scenarios live in a shared
package can run. **Status**: fixed on branch `081-studies-basic`. **Found**: 2026-11-18, swb
241, while converting `scenarios/projects/basic` to a testbench (081 step 4).

## What happens

`scenarios/projects/basic` with its suites naming `pkg://scensim_examples/scenarios/basic/...`
(instead of `../../../src/scensim_examples/...`) passes `scensim testbench check`. Expanding
any of its suites for a run fails:

```
scensim.run.Refused: .../basic/suites/episode_change.yaml: scenario
pkg://scensim_examples/scenarios/basic/town02_episode_change.xosc: no such file or directory
```

## Cause

`testbench.build_findings` leaves a suite's `pkg://` item to the run ("resolved when the
testbench runs (step 3)"). But `run._resolve`, which every suite item, scenario target and
distribution directory goes through, treats any string as a path relative to the project or
the suite. So `pkg://...` becomes `<project>/pkg:/...`. The fixture testbench `tb_good`
(colcon_scensim's tests) names a `pkg://` scenario, but it is only built, never run.

## Fix

`run._resolve` resolves `pkg://<package>/<path>` to `<package's share>/<path>` through the
ament index (`config.pkg_path`, which `find_project`'s `pkg://` start shares). A package
that is not installed is refused by name, with the fix: build it, or source the workspace
that installs it.

## Test

- a suite naming `pkg://<package>/...` of a fixture package on `AMENT_PREFIX_PATH` expands to
  the installed file;
- an uninstalled package is refused, naming the package.
