# Phase 081: Testbenches, a Colcon Package Type, and the User Workspace

A user's work is a colcon workspace: one or more **testbenches** (a ROS package of scenario
files, autopilot assignments and simulator settings, declarative) next to **support
packages** (a world server, an autopilot adapter, a car), built with `colcon build` over
scensim. scensim runs a testbench by its package name. A testbench has its own colcon build
type, `scensim_testbench`, provided by the colcon plugin `colcon-scensim`: a `package.xml`
and a `testbench.yaml`, no CMake.

**Status**: done, landed 2026-11-18 with 080 (scensim main 12e54f1), then a testbench's own world server and `simulator: none` (a53a2cb, ee7bb48) and `testbench check --live` (d6cb0eb, issue 048). Left: the internal study projects under `scenarios/projects/` become testbenches (parked).

**Source**: user, 2026-11-17, in one design conversation:

- "The user would treat swb as a convenient framework. They prepare user working dirs with
  assets and config included." The working directory is self-contained and declarative:
  scenario files, config files and support packages.
- "The work dir can contain additional simulator support package if they'd customize the
  Carla port or run a 3rd party simulator, or have a autopilot package to plug in to an
  actor."
- Support packages are source in the workspace: "My idea is to take colcon convention".
- "Take a more modular design. The swb work dir itself is a ROS package. The user prepare a
  colcon workspace containing support packages and scenario work dir. They may want to host
  many scenario runs." The name: "testbench sounds good to me. You're colcon workspace
  framing matches my idea exactly."
- The build type: of an ament_cmake helper, ament_python, a colcon build type of its own, or
  no build at all, "Go C. It's clean in design" (its own build type).
- "The swb does not provide Autoware. It's the user to provide Autoware build, either from
  source or from our Autoware binaries. It's user's responsibility to provide the autopilot."

**Follows**: 076 and 077, whose project workflow (`swb.yaml`, `project new|check`,
`swb run`, assets by name, `swb.lock`) a testbench keeps, as a package.

## Terms

- **testbench**: a ROS package whose `package.xml` exports
  `<build_type>scensim_testbench</build_type>`, holding `testbench.yaml` and the files it
  names (scenarios, suites, pilot programs, judges, ground files, config). It is what a
  project is today, with `testbench.yaml` for `swb.yaml`.
- **support package**: any other ROS package in the user's workspace that a testbench uses:
  a world server, a pilot (an adapter, a launch file, an agent program), a car, a package of
  shared scenarios. Built by its own build type (ament_cmake, ament_python, ament_cargo).
- **the user workspace**: a colcon workspace of testbenches and support packages, built
  over scensim (an underlay: the prebuilt tarball or a source build) and over the user's
  Autoware when a pilot needs it.

## Design

### The user workspace

```
my_ws/src/
  tb_courses/                 # a testbench
    package.xml               # <exec_depend> on scensim, support packages, scenario packages
    testbench.yaml
    scensim.lock              # pinned dataset commits, committed
    scenarios/ suites/ pilots/ judges/ config/ ground/ reference/
  coop_pilots/                # support package: Python pilots
  my_carla_world/             # support package: a world server
```

The chain: `/opt/ros/<distro>`, scensim, the user's Autoware when used, then
`my_ws/install`. scensim finds a testbench by name through the ament index of the sourced
chain, as `ros2 launch` finds a package.

### The build type (`colcon-scensim`)

A colcon plugin, pure Python, shipped inside the scensim wheel and as `colcon-scensim` on
PyPI (082), and in the source build's documented tools.

- **Build**: installs the package's files to `share/<pkg>/` (links them under
  `--symlink-install`, so an edit to a testbench's files takes effect without a rebuild;
  adding or removing a package needs one), registers
  `share/ament_index/resource_index/scensim_testbenches/<pkg>` and `packages/<pkg>`, and
  writes the package's `local_setup` hooks as colcon does for other types.
- **Check at build**: runs `testbench check` (today's `project check` rules plus the two
  below) on the installed tree; a problem fails that package's build with the same message
  and fix `testbench check` prints. Assets are not checked at build (they are fetched per
  host); the lock's syntax is.
- **No compilation.** A scenario generator (c2b's and s5's `make_scenarios.py`) is either a
  run-time suite step or a support package's executable; step 3 decides.

### Inside a testbench

`testbench.yaml` keeps the project file's schema (`docs/reference/project-file.md`), with:

- **References into other packages** by `pkg://<package>/<path>` (it exists today,
  `config.py:100-118`), each package declared as an `<exec_depend>`. `testbench check`
  refuses a `pkg://` whose package is not declared, and any path that leaves the testbench
  (`../`), naming the `pkg://` form. This replaces today's suites that point at
  `../../../src/swb_examples/...` and `isaac_road_physics`'s ground file in another
  project.
- **Pilots from support packages** by `ros2 run` or `ros2 launch` form (`command:
  [coop_pilots, cutin_pilot, --pair, "{param:pair}"]`, `launch: <pkg> <file>`), not by
  `sys.path` reaches into a checkout (c2b's and s5's pilots today).
- **A world server from a support package** (new): `world: {server: [<pkg>, <executable>]}`.
  Today `world:` refuses any world-server setting. scensim starts it in place of the stock
  one and runs `scensim_world_check conformance` against it in `testbench check --live`.
  Ports, the lease and readiness follow the stock servers' contract; step 5.
- **Writable state outside the package**: runs go to `<logs>/<pkg>/runs/`, host state to the
  host's state directory; an installed testbench is read-only (as today's
  `writable_project` already assumes for package projects).
- **Assets and the lock**: fetched assets live in the host cache and state directory, never
  in `share/<pkg>`. `scensim assets fetch <testbench>` writes `scensim.lock` to the
  testbench's source directory (found through the `--symlink-install` link, or
  `--source <dir>`); an installed copy without its source uses its committed lock.
- Every other key keeps its 076/077 meaning. `swb.yaml` in a testbench is refused by name,
  naming `testbench.yaml` (080's no-shims rule).

### Running

- `scensim run <testbench>[/<suite>|/<scenario>]` resolves `<testbench>` through the ament
  index; `ros2 run scensim run` works the same. A path (`scensim run ./src/tb_courses`)
  works for a testbench not yet built.
- `scensim testbench new <name> [--from <template>|pkg://...]` writes a testbench package
  (the 076 templates: `carla-first-drive`, `multi-pilot`, `isaac`).
- `scensim testbench list` lists the testbenches in the sourced chain, with each one's
  suites.

### Today's studies as testbenches (surveyed 2026-11-17, 73 studies in 077's inventory)

| Today | Problem today | As a testbench |
|---|---|---|
| basic, demo, gap, set050, e2, two_av | Suites reach into other directories (`../../../src/swb_examples/...`, `../../gap/*.xosc`). | Shared scenarios become packages, named by `pkg://` and declared in `package.xml`. |
| isaac_road_physics | Its ground file points into another project. | A dependency on that package, by `pkg://`. |
| c2b, s5 | Their pilots reach into the checkout's `scripts/coop/` with `sys.path`, so they work only inside a checkout. | Those scripts become a support package `coop_pilots`; the generated scenarios follow step 3's decision. |
| courses (Haruna, Tsukuba) | Its scenarios come from a public dataset. | Unchanged: assets declared and locked. A clean public example. |
| drift_lap, speed_ladder | They need autoware-drift, a host-level workspace. | The autopilot is the user's: autoware-drift stays an underlay the user sources, or a package in their workspace; the testbench names it. |
| NTU twin, NTU open roads, Shalun, golf cart | Private maps, data and NAS paths. | Private testbenches in a new private repository the user creates (user, 2026-11-17); scensim ships none of them. |
| an own world server | Refused today. | `world: {server: ...}`, a support package (step 5). |
| an own car | | `vehicles:` names a spec in a support package; its Autoware packages build next to it. |

Found on the way: the committed `swb.lock` of courses, drift_lap and speed_ladder has no
entry for the Isaac USD dataset their `swb.yaml` asks for.

## Steps

Each step is pushed after review, with its tests.

1. **Docs** (133): this doc.
2. **`colcon-scensim` and the testbench type.** The plugin, the ament index entries,
   `--symlink-install`, the check at build; `testbench check` (today's rules plus the
   `pkg://`-declared and no-`../` rules). Tests: a fixture workspace with a testbench, a
   support package and a scenarios package; `colcon build` fails on a bad testbench with the
   check's message.
3. **Finding and running testbenches by name.** `scensim run <testbench>`, `testbench new`,
   `testbench list`, the logs path by package, `swb.yaml` refused by name, `assets fetch`
   writing the lock to the source; the generator decision.
4. **The studies as testbenches.** `scenarios/projects/*` and `src/scensim_examples/projects/*`
   become testbench packages as in the table; `coop_pilots`; the stale locks refreshed;
   t3.yaml follows (the T3 references keep their keys); the private ones moved to the
   user's private repository.
   `coop_pilots` landed (2026-11-19) as scripts/coop installed as data
   (`share/coop_pilots/coop`, on the testbenches' sys.path); a Python package
   (`from coop_pilots import ...`, its 15 modules moved) replaces it once roadmap 078 settles.
5. **A world server from a support package**: `world: {server: ...}`, the conformance check,
   a fixture world-server package, a how-to.
6. **Docs for users** (243): the tutorials on a testbench workspace; a reference page for
   `testbench.yaml` and the build type; "your own world server", "your own pilot" and "your
   own car" as support packages.
7. **Exit runs.**

## Exit criteria

- A workspace with a testbench made by `scensim testbench new` and a support package (a
  `command` pilot) builds with `colcon build --symlink-install`, and `scensim run` runs it on
  CARLA; an edit to a scenario takes effect without a rebuild.
- Tutorials 1 to 5, rewritten for testbenches, pass as written.
- A testbench that names an undeclared package, leaves its directory, or has a schema error
  fails `colcon build` with the check's message.
- Every study in step 4 runs from its testbench with its 077 verdicts; the T3 suites compare
  against the 36e4da4 references.
- check_docs, the tests and check_pins pass.

## Non-goals

- A CMake or Python build file in a testbench (the build type is the design).
- Shipping an autopilot, or private testbenches in any public repository.
- Changing what a project file's keys mean.

## Open, for the builder

- Whether a testbench may depend on another testbench (for shared suites), or only on
  support and scenario packages; the doc allows `pkg://` to any declared package.
