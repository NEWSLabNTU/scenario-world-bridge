# 056: A suite's scenario from a fetched asset is reported as a problem of the package

**Severity**: medium. A testbench whose scenarios come with a dataset fails `testbench
check`, and so would fail `colcon build`, on any host that has not fetched it. A build never
has fetched it, since assets live in the host cache and never in `share/<pkg>`. So the
081 table's "clean public example", courses, cannot be built. **Status**: fixed on branch
`056-asset-scenarios`. **Found**: 2026-11-18, swb 241, converting the studies to
testbenches (081 step 4, group D).

## What happens

`scensim testbench check scenarios/projects/courses`, on a checkout without its assets
fetched:

```
problem: suites/haruna.yaml: scenario assets/maps/haruna/scenarios/haruna_down.xosc: no such file or directory
problem: suites/tsukuba.yaml: scenario assets/maps/tsukuba/scenarios/tsukuba_lap.xosc: no such file or directory
problem: scenarios/: no scenario (fix: add one, e.g. scenarios/drive.xosc)
```

All three are reported as the package's problems, not the host's.

## Cause

`project.collect_scenarios` makes a plain finding for any suite item that is not there.
081 step 3 split findings into the package's and the host's (`Finding.host`). A map that
is not fetched is a host finding. A scenario under `assets/` is fetched the same way, but it
was never marked. With no scenario found, the "no scenario" rule fired too.

## Fix

A suite item under `assets/` that is not there is a host finding: "not fetched (fix: scensim
assets fetch)". It no longer counts against the package. The "no scenario" rule holds only
when the testbench names no scenario at all, so a testbench whose scenarios are all fetched
passes the package check.

## Test

`test_testbench.py`: a testbench whose only suite names `assets/maps/<m>/scenarios/x.xosc`,
unfetched, gives no package finding and one host finding naming the fetch. The project check
(`check_project`) no longer adds "no scenario" when a scenario is only unfetched.
