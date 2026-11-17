# The testbench

A **testbench** is a ROS package that holds one use case: `testbench.yaml`, the scenarios
and suites it runs, and anything else it names. It lives in a colcon workspace beside other
testbenches and support packages, and colcon builds it with the `scensim_testbench` build
type. This page describes the package, its build and its rules. The keys of
`testbench.yaml`, of a suite file and of `scensim.lock` are described in
[The project file](project-file.md), whose `swb.yaml` is the same schema under its former
name.

<!-- TODO(exit run): rename project-file.md's subject to testbench.yaml once 081 lands. -->

## The package

```
src/my_testbench/
  package.xml          the manifest: build type and dependencies
  testbench.yaml       the testbench file
  scensim.lock         the assets' exact versions (written by `scensim assets fetch`)
  suites/*.yaml        lists of scenarios; `default:` names the one run by name
  scenarios/*.xosc     the scenarios
```

`scensim testbench new <name>` creates one from a template.
The name must be a valid ROS package name: lower-case letters, digits and underscores.

```xml
<package format="3">
  <name>my_testbench</name>
  <version>0.1.0</version>
  <description>What this testbench tests.</description>
  <maintainer email="you@example.org">You</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>scensim</exec_depend>
  <exec_depend>my_world</exec_depend>     <!-- every package it names, below -->
  <export>
    <build_type>scensim_testbench</build_type>
  </export>
</package>
```

## The build

The build type comes with scensim's `setup.bash`, which must be sourced before `colcon
build`. Without it colcon does not fail: it warns `No task extension to 'build' a
'ros.scensim_testbench' package` and installs nothing, and `scensim run` later refuses the
testbench as not in the sourced workspace. A build of a testbench:

1. installs every file of the package to `share/<package>/`, a copy, or under
   `--symlink-install` a link per file, so that an edit takes effect without a rebuild
   (adding or removing a file needs one). What a host writes at run time (`assets/`,
   `state/`, `runs/`, `.scensim/`) is never installed;
2. registers the package in the ament index, with the version and prefix of the scensim it
   was built over: `scensim run` refuses a testbench built over another scensim, naming the
   rebuild (`cd <workspace> && colcon build`);
3. checks the installed testbench, and fails the package's build on any problem, each
   printed as `testbench check` prints it, with its fix.

The build checks only what holds on any machine: the schema, the files the testbench names,
its scenarios, and the two rules below. What depends on the machine (fetched assets, the
simulator's installation) is checked by `scensim testbench check <name>`, which prints
such a finding as `problem (this host): ...`. A testbench without `scensim.lock` builds,
with the warning `no scensim.lock yet: run scensim assets fetch <name> and commit
src/<name>/scensim.lock`.

## Names of other files and packages

A testbench names its own files by paths relative to itself (a suite's scenarios relative
to the testbench or to the suite's directory), and files of another package as
`pkg://<package>/<path>`. Two rules apply, checked at build:

| Rule | Refused as | Fix |
|---|---|---|
| a `pkg://<package>/...` names a package declared in `package.xml` (`<depend>` or `<exec_depend>`), or the testbench itself | `pkg://other/x.xosc: package other is not declared` | add `<exec_depend>other</exec_depend>` |
| no path leaves the package: neither `../` out of it nor an absolute path (or one starting with `~`) | `../x.xosc: leaves the testbench` | move the file into a package and name it as `pkg://<package>/<path>`, declared in `package.xml` |

The rules make a testbench self-contained: it runs on another machine once its workspace is
built there and its assets are fetched.

## Assets and the lock

`scensim assets fetch <name>` downloads what `assets:` names into the shared cache and
writes `scensim.lock` into the testbench's source directory, which the installed files of a
`--symlink-install` build link to. A workspace built with copies needs the source named:
`scensim assets fetch <name> --source src/<name>`. Commit the lock with the testbench, so
that every machine fetches the same files.

## A world server of your own

`world: {server: [<package>, <executable>, args...]}` runs a world server from a package of
the workspace in place of the stock one: [Your world server](../how-to/your-world-server.md).

## Commands

| Command | Does |
|---|---|
| `scensim testbench new <name>` | creates a testbench in the current directory (`src/`) |
| `scensim testbench list` | the testbenches of the sourced workspaces, with their suites |
| `scensim testbench check <name>` | the build's checks and this host's |
| `scensim run <name>` | the default suite; `<name>/<suite>` another suite; `<name>/scenarios/<file>` one scenario |
| `scensim assets fetch <name>` | the assets, and the lock |
| `scensim stack --list`, `scensim stack <name> --stop` | the running stacks; stop one |

Every command and option: [Commands](commands.md).

<!-- TODO(exit run): check each command's spelling and output against the release. -->
