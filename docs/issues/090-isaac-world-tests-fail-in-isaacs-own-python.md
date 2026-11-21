# 090: scensim_isaac_world's tests fail in Isaac's own Python

**Severity**: low. Thirteen of the component's tests fail or cannot import in the Python the
world server runs in, so a run of its suite there is never green and a new failure hides among
them. **Status**: open, no fix scheduled. **Found**: 2026-11-21, swb 243, roadmap 091 W13 to
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

## Fix (when taken)

The real gap: the world server runs in Isaac's Python, and no gate tests it there. The fix is a
`just test` lane that runs this suite in Isaac's venv when `ISAAC_ROOT` is set, and skips with a
printed note when it is not; it goes in after the thirteen below are fixed.

For the thirteen: the missing modules, skip the tests where the module is absent (`unittest.skipIf` on the
import, as the pxr tests do), or add the modules to the venv's setup. The messages: format the
values as Python scalars (`float(v)`, `str(v)`, or `{v:g}`) where `road_record` builds them.
