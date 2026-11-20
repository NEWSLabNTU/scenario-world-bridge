# 087: The courses project's scenarios are refused, since the renames

**Severity**: medium. The Haruna and Tsukuba course runs cannot start, and no regression run notices,
because `courses` is not in the T3 manifest. **Status**: open. **Found**: 2026-11-21, by 241, rerunning
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
