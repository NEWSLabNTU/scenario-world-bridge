# 061: `scensim testbench new --from` refused a project that still had swb.yaml

**Severity**: medium. A project from before the rename (roadmap 080) could not become a
testbench in one step: the command refused it. The user first had to rename the file and the
old names in its scenarios by hand. **Status**: fixed on main. **Found**: 2026-11-19,
coop 118, migrating its 077-era projects to testbenches.

## What happens

```
$ scensim testbench new coop --from ~/coop/projects/coop
.../coop: no testbench.yaml (fix: a project directory, e.g. pkg://scensim_examples/projects/first_drive)
```

## Cause

`project.source_project` accepted only a directory with `testbench.yaml`. The old names
inside such a project are known elsewhere in scensim, but only as refusals that
`testbench check` reports:

- `swb.yaml` (`config.old_project_file`);
- `$(env SWB_MAPS)` in a LogicFile;
- the `swb.*` scenario properties.

The copy did not apply any of them.

## Fix

`testbench new --from <dir>` now takes a directory that has `swb.yaml` and no
`testbench.yaml`. In the copy, and never in the source:

- `swb.yaml` becomes `testbench.yaml`;
- `$(env SWB_MAPS)` becomes `$(env SCENSIM_MAPS)`;
- `name="swb.<x>"` becomes `name="scensim.<x>"`;
- the installed VehicleCatalog (`$(find-pkg-share sim_vehicle_spec)/catalogs`) becomes
  `$(env SCENSIM_VEHICLES)`, which a testbench must name (roadmap 083);
- `.swb/` holds host settings, so it is not copied, like `.scensim/`.

Each rename is printed as one note per file and per name. Paths that leave the source are
then rewritten as for issue 055.

## Test

`test_testbench_new.py` copies a 077-era project that has all four old names and a `.swb/`.
The copy has `testbench.yaml`, the new names and no `.swb/`. Its suite's `../` path becomes
`pkg://` (issue 055). The command prints a note for each rename. `check_project` gives no
finding about a rename. The source is left unchanged.
