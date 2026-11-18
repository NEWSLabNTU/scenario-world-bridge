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

## Part 2: a `ground:` file (found 2026-11-18, converting isaac_road_physics)

`ground: {<map>: {surfaces: <file>}}` had the same gap: `ground.problems` and `ground.apply`
took every file as a path in the project. So `pkg://scensim_examples/projects/road_physics/
ground/town01.yaml` was "not a file in the project". Both now resolve `pkg://` through
`config.pkg_path` (`ground.source`), and an uninstalled package is refused by name. A link
into an installed package's share counts as scensim's own (`_ours`), so it is replaced at
the next start and removed when the testbench stops naming it, as a project file's link is.
Test: `test_ground.py::test_a_ground_file_from_another_package`.

## Part 3: the stack's own check (found 2026-11-18, the same day)

The check a stack runs before it starts (`project.check_stack`, through `collect_scenarios`)
took a suite's `pkg://` item as a path in the project too. It found it "no such file or
directory", and the stack refused to start on that finding. So the studies converted that day
(basic, group A, isaac_t2) passed `testbench check`, which leaves `pkg://` to the run, but
their stacks would not start. `collect_scenarios` now resolves `pkg://` through
`config.pkg_path`, as the run does. A package that is not installed is a host finding: build
it, or source the workspace that installs it. Test:
`test_run.py::test_the_stack_check_finds_a_suites_pkg_scenario`.
