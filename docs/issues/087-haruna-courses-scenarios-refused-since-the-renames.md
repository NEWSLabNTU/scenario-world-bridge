# 087: The courses project's scenarios are refused, since the renames

**Severity**: medium. The Haruna and Tsukuba course runs cannot start, and no regression run notices,
because `courses` is not in the T3 manifest. **Status**: the generator is fixed (scensim_isaac_world 310df6c). The dataset revision with the
regenerated scenarios is staged, not yet published, and the project's lock moves when it is. **Found**: 2026-11-21, by 241, rerunning
haruna_down for issue 086.

## What happens

```
$ cd scenarios/projects/courses && ros2 run scensim run suites/haruna.yaml --start-stack --stop-stack
scensim run: refused: project check found problems in the scenario:
problem: .../haruna-tsukuba-courses/fff4cb0b.../haruna/scenarios/haruna_down.xosc: its VehicleCatalog
is $(find-pkg-share sim_vehicle_spec)/catalogs, the cars on one body (CARLA 0.9.16's): ...
```

With the catalog fixed, the check finds a second problem:

```
problem: scenarios/haruna_down.xosc: LogicFile $(env SWB_MAPS)/haruna does not name a project map
(SWB_MAPS was renamed SCENSIM_MAPS with the project) (fix: filepath="$(env SCENSIM_MAPS)/haruna")
```

## Cause

The scenarios come from the dataset `haruna-tsukuba-courses` at fff4cb0b (v2026.11.1), which the
project locks. They were written before two changes that the project check now enforces:

- the rename to scensim (roadmap 080 step 6), which renamed `SWB_MAPS` to `SCENSIM_MAPS`;
- roadmap 083, which makes a scenario name the stack's vehicle catalog, `$(env SCENSIM_VEHICLES)`,
  not the installed one.

A dataset scenario cannot follow either change unless the dataset is republished. The project
check therefore refuses both, by name.

`courses` is not in `scenarios/t3.yaml`, so no T3 run tried it after either change.

## Workaround

Copy the project and the scenario, and fix the two lines in the copy:

```
<VehicleCatalog><Directory path="$(env SCENSIM_VEHICLES)" /></VehicleCatalog>
<LogicFile filepath="$(env SCENSIM_MAPS)/haruna" />
```

`scensim project check` then reports `project: ok`. haruna_down ran that way for 086.

## Fix options

1. Publish a new dataset revision with both lines fixed in both scenarios (`haruna_down.xosc` and
   `tsukuba_lap.xosc`, which has the same two lines), and move the lock to it.
2. Keep the scenarios in the project (`scenarios/projects/courses/scenarios/`), taking only the
   maps from the dataset.

Either way, add `courses` to the T3 manifest, so the next rename is caught.

## Fix

The scenarios come from `scensim_isaac_world.course`'s `scenario` step. Its template named the
installed catalog and the map directory's absolute path. The dataset's `$(env SWB_MAPS)` had been
edited in by hand when it was published. Since 310df6c the step writes `$(env SCENSIM_VEHICLES)` and
`$(env SCENSIM_MAPS)/<map dir's name>`. It also reruns alone, from the centreline the
`centreline` step saved (`course/centreline.npz` and `.json`), because a published course has no raw
tiles. `test/test_course_scenario.py` covers both, and both tests fail before the fix.

Rerun on fff4cb0b, the step reproduces each course's start, goal and drift points exactly. Six files
change:
- `<course>/scenarios/*.xosc`: the two paths, and the generator's name in the header;
- `<course>/surfaces.yaml` and `<course>/course/drift_points.yaml`: the generator's name in a comment.

The README says so, and renames the generator and its repository. `scensim project check`
passes on the regenerated scenarios and refuses fff4cb0b's.

The staged revision, with the README, `NOTICE.md` and `.gitattributes`, is on 241 at
`/var/tmp/aeon/087/stage/rev/`. The changed files and the diffs against fff4cb0b are in the NAS's
`home/087-haruna-tsukuba-courses-staged/`.

To do once it is published:
- move `scenarios/projects/courses/scensim.lock` to the new revision;
- sync `docs/roadmap/080-study/dataset-cards/haruna-tsukuba-courses.md` with its README;
- add `courses` to `scenarios/t3.yaml`.
