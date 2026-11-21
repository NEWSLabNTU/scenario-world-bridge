# 090: scensim_isaac_world's tests fail in Isaac's own Python

**Severity**: low. Thirteen of the component's tests fail or cannot import in the Python the
world server runs in, so a run of its suite there is never green and a new failure hides among
them. **Status**: fixed (scensim_isaac_world 19d82da, bf573e8; scensim's `just test` lane), swb 139. **Found**: 2026-11-21, swb 243, roadmap 091 W13 to
W16 (the suite at its baseline: 3 failures, 10 errors, 7 skipped).

## Where they fail, and where not

`just test` here runs the suite in the system Python (`PYTHONNOUSERSITE=1 python3 -m
unittest discover -s src/scensim_isaac_world/test`, numpy 1.x, no pxr): there it is green, the
pxr tests skipped (333 tests, 33 skipped, on 2026-11-21). The thirteen below fail only in
Isaac's venv (`source src/scensim_isaac_world/scripts/isaac_env.sh`, Python 3.12, numpy
2.3.1), which no gate runs.

## The thirteen

Missing modules (the venv lacks them; the tests do not skip):

- `test_course_closed` (7 tests, `ClosedCentreline` and `ClosedRibbonsAndLanelets`): `import
  pyproj` in `setUp`, `ModuleNotFoundError: No module named 'pyproj'`.
- `test_installed`: `import pytest` at module level, so the module does not import.
- `test_rider`: the same.
- `test_walkers`: the same.

numpy 2's scalar repr in messages (a defect in the messages, not only in the tests):

- `test_road_record.Constants.test_every_problem_is_listed`: expects `half_width -1.0 out of
  range`, gets `half_width np.float64(-1.0) out of range`.
- `test_road_record.Stations.test_a_bad_record_lists_every_problem`: expects `half_width 0.0
  out of range`, gets `half_width np.float64(0.0) out of range`.
- `test_road_record.SurfaceClasses.test_a_surface_not_in_the_table_is_refused`: expects
  `runoff_surface 'lava' is not a surface class`, gets `runoff_surface np.str_('lava')`.

The world server runs in this venv, so a user's broken `road_detail.yaml` is reported there with
`np.float64(...)` and `np.str_(...)` in the message.

## Fix

**The thirteen** (scensim_isaac_world 19d82da, bf573e8):

- **The messages.** `road_record` and `road_layer` now turn the value into a Python float or str
  before they format it: "half_width -1.0 out of range", "runoff_surface 'lava' is not a surface
  class". There were three such sites, one more than listed above: `hazard 'cliff' is not one
  of ...` had the same `np.str_` repr. The road_record tests are the check, now passing under
  numpy 2.3.
- **`pyproj`: skipped with its reason, not added to Isaac's setup.** The course's centreline and
  lanelets are built by the system Python (pyproj, lanelet2), and Isaac's Python writes only the
  USD (`scensim_isaac_world.course`). So `test_course_closed` skips there with "no pyproj in this
  Python: the course's centreline and lanelets are built by the system Python (pyproj,
  lanelet2), not Isaac's".
- **`pytest`.** `test_installed` (pytest fixtures, a colcon build) skips without it, naming the
  system Python's pytest as what runs it.
- **`test_rider` and `test_walkers` were never run by any gate.** They are plain `test_`
  functions, which `unittest discover` does not collect. They no longer import pytest:
  `test/_plain.py` gives `approx` (pytest.approx's default tolerances; arrays defer to it) and a
  `load_tests` hook that makes each `test_` function a case. Their 17 tests now run in both
  Pythons, and pass.

Results: in the system Python (ROS sourced), 350 tests OK, 33 skipped (333 before). In Isaac's
venv, 351 tests OK, 15 skipped (3 failures and 10 errors before).

**The lane.** `just test` now also runs the suite in Isaac's venv, in a clean environment
through `scripts/isaac_env.sh`, when `ISAAC_ROOT` names an install. Otherwise it prints
`skipped: scensim_isaac_world's suite in Isaac's Python (ISAAC_ROOT names no Isaac Sim
install)`.
