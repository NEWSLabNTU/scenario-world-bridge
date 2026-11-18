# 057: An Isaac ego config's rig leaves the testbench, unchecked, and cannot name a package

**Severity**: medium. A testbench whose Isaac ego config names a rig outside the package (in
another repository's source tree) passes `testbench check`, then breaks wherever that path is
not there: an installed copy, or another checkout. And there was no right way to write it:
the rig belongs to aib, but the config could not name it by `pkg://`. **Status**: fixed on
branch `057-isaac-ego-paths`. **Found**: 2026-11-18, swb 241, converting the studies to
testbenches (081 step 4, group D).

## What happens

courses, drift_lap and speed_ladder name `isaac: {ego: config/ego_drift_mustang.yaml}`. That
config names its rig as:

```
rig: ../../../../src/autoware_isaac_bridge/src/aib/config/drift_mustang.rig.yaml
```

This climbs out of the testbench into the checkout's aib submodule. `scensim testbench check`
gives no finding for it.

## Cause

1. 081's leave and declaration rules (`testbench.reference_findings`) read testbench.yaml and
   the suites. They read `isaac.ego`'s own path, but not the file it names. The ego config's
   `rig:` and `body:` are paths too: aib resolves them from the config's directory.
2. aib reads only paths (environment variables expand), so `pkg://aib/config/...` would fail
   in aib. aib installs the rig under its share (`share/aib/config/`), but nothing could name
   it there.

## Fix

- **The check reads the ego config** that `isaac.ego` names in the package. Its `rig` and
  `body` fall under the same rules: `../` out of the testbench is refused, and a `pkg://`
  package must be declared.
- **scensim resolves `pkg://` before aib sees it.** When the ego config names its rig or body
  by `pkg://<package>/<path>`, the stack hands the world server a copy in the stack's
  directory (`isaac_ego_resolved.yaml`). In that copy those paths, and any other relative
  one, are absolute. A config without `pkg://` is handed over as it is. A package that is not
  installed refuses the stack by name.
- The three studies name aib's rig as `pkg://aib/config/drift_mustang.rig.yaml` and declare
  `aib` (081 step 4, group D).

## Test

- `test_testbench.py`: a `../` rig is refused, and a `pkg://aib` rig must be declared.
- `test_isaac_ego_config.py`: a `pkg://` rig is resolved into the copy, a relative body is made
  absolute, a plain config is passed as it is, and an uninstalled package is refused.
