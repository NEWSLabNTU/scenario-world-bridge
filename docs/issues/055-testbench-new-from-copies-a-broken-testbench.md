# 055: `scensim testbench new --from` makes a testbench that is broken or half-filled

**Severity**: medium. The copy looks right but fails `testbench check`, and the user does not
learn why from the command that made it. **Status**: fixed on branch `055-new-from`.
**Found**: 2026-11-18, swb 241, converting `scenarios/projects/basic` to a testbench (081
step 4).

## What happens

`scensim testbench new basic --from scenarios/projects/basic`, run outside a git checkout:

1. **The copied suites point nowhere.** basic's suites name
   `../../../src/scensim_examples/scenarios/basic/...`. That path is relative to the source
   project, and the copy keeps it as it is. From the new directory it names nothing, and 081's
   leave rule refuses it anyway.
2. **The default suite is the first one alphabetically.** basic has four suites and no
   `default:`. The copy got `default: episode_change`, a one-scenario town-load check, not
   its main suite, `town01`. Nothing said so.
3. **package.xml is filled with placeholders.** `<maintainer
   email="maintainer@example.com">Maintainer</maintainer>` and `<license>TODO: License
   declaration</license>`, because this host has no git identity. Nothing said so either.

A fourth gap, found while converting the studies: a `ground:` file under `assets/` that is not
fetched yet (isaac_rough_road's `road_detail`) is reported as a problem of the package. It is
the host's, as issue 056 made a fetched scenario.

## Cause

1. `project.copy_project` copies files byte for byte. A relative path that leaves the source
   project stays relative to a directory the copy is not in.
2. `testbench_cli.new_testbench` writes `default: <first suite>` whenever the source has none.
3. The maintainer comes from `git config`, else a placeholder, and the license is always the
   ROS placeholder. There is no option for either, and no notice.
4. `project.check_project` reports any missing ground file as the package's.

## Fix

- **Paths that leave the source are rewritten** to `pkg://<package>/<path>` when the file they
  name is that package's. The package is the one holding the file in a source tree (the
  nearest `package.xml`) or in an installed share. The rewrite is made only when
  `pkg://<package>/<path>` resolves to the same file in the sourced install. Each such package
  is declared in the new package.xml, and so is every package the copy already names by
  `pkg://`. A path that cannot be rewritten is left as it is and listed in the command's
  output, with the fix.
- **`--default <suite>`** names the default suite. Without it:
  - the source's own `default:` is kept;
  - a single suite, or one named `default`, is taken;
  - otherwise the first is taken, and the output names it, the others and `--default`.
- **`--maintainer "Name <email>"` and `--license <SPDX>`** fill package.xml. Any placeholder
  left is named in the output.
- **A ground file under `assets/`** that is not there is a host finding: "not fetched (fix:
  scensim assets fetch)".

## Test

`test_testbench_new.py`:
- a project whose suite names `../shared/scenarios/x.xosc` in a source package, also installed,
  is copied with `pkg://shared/scenarios/x.xosc` and `<exec_depend>shared</exec_depend>`;
- a path in no package is kept and listed;
- the default choice and its notice;
- the maintainer and license options, and the placeholder notice.

`test_project.py`: an unfetched ground file under `assets/` is a host finding.
