# Phase 080: Distribution, `pip install swb` and Testbenches in a Colcon Workspace

A user with an LTS ROS 2 from apt runs `pip install swb`, then `swb` once; the first command
downloads the prebuilt swb for their ROS release. Their own work is a colcon workspace: one
or more **testbenches** (a ROS package of scenario files, autopilot assignments and simulator
settings, declarative) next to **support packages** (a world server, an autopilot adapter, a
car), built with `colcon build` over swb. swb runs a testbench by its package name. A
source build of swb stays documented: `rosdep install`, then `colcon build` with
colcon-cargo-ros2.

**Status**: planned, 2026-11-17. Nothing is built yet.

**Source**: user, 2026-11-17, in one design conversation:

- "The user would treat swb as a convenient framework. They prepare user working dirs with
  assets and config included. They installed the swb in a convenient way, either by pip,
  npm or something." The working directory is self-contained and declarative: scenario
  files, config files and support packages.
- "The user will install lts ROS." swb fits into it and brings no ROS of its own.
- "The swb does not provide Autoware. It's the user to provide Autoware build, either from
  source or from our Autoware binaries."
- Support packages are source in the workspace, "take colcon convention".
- "Take a more modular design. The swb work dir itself is a ROS package. The user prepare a
  colcon workspace containing support packages and scenario work dir. They may want to host
  many scenario runs." The name for that package: **testbench** (user's choice of the
  proposed names).
- The testbench builds with its own colcon build type, no CMake ("Go C. It's clean in
  design").
- Prebuilt binaries, and a documented source build: "Use colcon-cargo-ros2 and make rosdep
  work".
- Not debs: "What if we provide install/ binaries? User does not need root to install Deb".
  Then: "Tarball plus pip front end", the right release chosen by `pip install swb`, and
  "One time download at the first swb is ok".

**Supersedes**: [070](070-user-experience.md) section B (prebuilt releases) and
[029](029-prebuilt-releases.md), whose measurements and tarball design (relocatable merged
install, vendored Autoware message packages, a CARLA binary per version, a release manifest,
an install-deps list, a setup guard) this phase keeps. **Follows**: 076 and 077 (the project
workflow this phase turns into testbenches), 079 (a stock `ros2 launch` path, the DDS
dependency declared). It does not change v0.1.0.

## Goal

On a clean Ubuntu 22.04 with ROS 2 Humble from apt (and later 24.04 with Jazzy), with the
user's simulator and autopilot already installed (CARLA 0.9.16 and NEWSLab's Autoware 1.5.0
binaries for the exit runs):

```bash
pip install scenario-world-bridge    # the PyPI name, see "Cutting a release"; Ubuntu 24.04: PEP 668 below
swb --version                        # first run: downloads swb for Humble, checks its sha256
sudo rosdep install ...              # the one root step, printed by swb (system packages)
source ~/.local/share/swb/current/setup.bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src && swb testbench new my_first_drive
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
swb run my_first_drive
```

passes, and so do the tutorials rewritten for testbenches. No step but the rosdep line needs
root. The same workspace builds on a source build of swb.

## Terms

- **testbench**: a ROS package whose `package.xml` exports
  `<build_type>swb_testbench</build_type>`, holding `testbench.yaml` and the files it names
  (scenarios, suites, pilot programs, judges, ground files, config). It is what a project is
  today (076, 077), with `testbench.yaml` for `swb.yaml`.
- **support package**: any other ROS package in the user's workspace that a testbench uses:
  a world server, a pilot (an adapter, a launch file, an agent program), a car. Built by its
  own build type (ament_cmake, ament_python, ament_cargo).
- **the user workspace**: a colcon workspace holding testbenches and support packages,
  built over swb (an underlay) and the user's Autoware when a pilot needs it.

## Design

### The user workspace

```
my_ws/src/
  tb_courses/                 # a testbench
    package.xml               # <exec_depend> on swb, support packages, scenario packages
    testbench.yaml
    swb.lock                  # pinned dataset commits, committed
    scenarios/ suites/ pilots/ judges/ config/ ground/ reference/
  coop_pilots/                # support package: Python pilots
  my_carla_world/             # support package: a world server
```

Chain: `/opt/ros/<distro>`, then swb (`~/.local/share/swb/current`, or a source build), then
the user's Autoware when used, then `my_ws/install`. swb finds a testbench by name through
the ament index of the sourced chain, as `ros2 launch` finds a package.

### The testbench build type (`colcon-swb`)

A colcon plugin, pure Python, shipped inside the `swb` wheel and as its own `colcon-swb`
package on PyPI for colcon installs made by pip.

- **Build**: installs the package's files to `share/<pkg>/` (links them under
  `--symlink-install`, so an edit takes effect without a rebuild; adding or removing a
  package needs one), registers `share/ament_index/resource_index/swb_testbenches/<pkg>`
  and `packages/<pkg>`, and writes the package's `local_setup` hooks as colcon does for
  other types.
- **Check at build**: runs `project check`'s rules (renamed `testbench check`) on the
  installed tree; a problem fails the package's build with the same message and fix that
  `testbench check` prints. Asset presence is not checked at build (assets are fetched per
  host); the lock's syntax is.
- **No compilation.** A generator (c2b's and s5's `make_scenarios.py`) runs at run time as
  a suite step, or is a support package's executable; decided in step 3.

### Inside a testbench

`testbench.yaml` keeps the project file's schema (`docs/reference/project-file.md`), with:

- **References into other packages** by `pkg://<package>/<path>` (exists today,
  `config.py:100-118`), each package declared as an `<exec_depend>`; `testbench check`
  refuses a `pkg://` whose package is not declared, and any path that leaves the testbench
  (`../`), naming the `pkg://` form. This replaces today's `../../../src/swb_examples/...`
  suites and `isaac_road_physics`'s ground file in another project.
- **Pilots from support packages** by `ros2 run` or `ros2 launch` form (`command:
  [coop_pilots, cutin_pilot, --pair, "{param:pair}"]`, `launch: <pkg> <file>`), not by
  `sys.path` reaches into a checkout (c2b's and s5's pilots today).
- **A world server from a support package** (new): `world: {server: [<pkg>, <executable>]}`.
  Today `world:` refuses any world-server setting. swb starts it in place of the stock one
  and runs `swb_world_check conformance` against it in `testbench check --live`. The
  details (ports, the lease, readiness) follow the stock servers' contract; step 6.
- **Writable state outside the package**: runs go to `<logs>/<pkg>/runs/` (as today's
  `<logs>/<project>/runs`), host state to the host's state directory; an installed testbench
  is read-only (as `writable_project` already assumes for package projects).
- The schema file and every key keep their 076/077 meanings. `swb.yaml` in a testbench is
  refused by name, naming `testbench.yaml` (no shims, as `SWB_RETIRED_ENV`).

### Running

- `swb run <testbench>[/<suite>|/<scenario>]` resolves `<testbench>` through the ament
  index; `ros2 run swb run` keeps working. A path (`swb run ./src/tb_courses`) works for a
  testbench not yet built.
- `swb testbench new <name> [--from <template>|pkg://...]` writes a testbench (the 076
  templates: `carla-first-drive`, `multi-pilot`, `isaac`).
- `swb testbench list` lists the testbenches in the sourced chain, with each one's suites.

### The release: one tarball per ROS release, and per Autoware release for the vehicle side

029's split stands, by what each side links against:

| Artefact | Built per | Holds |
|---|---|---|
| `swb-sim-<ver>-<distro>-<arch>.tar.zst` | ROS release x arch | the bridge, relay, `swb_launch`, `swb_interfaces`, `swb_world_protocol`, the CARLA world server (a binary per CARLA version, 070 A's launcher), `swb_isaac_world`, the SSv2 fork, the vendored Autoware message packages (029), `swb_examples` |
| `swb-vehicle-<ver>-autoware<rel>-<arch>.tar.zst` | Autoware release x arch | acb, aib, `autoware_sim_vehicle`, `sim_vehicle_spec`, `sim_vehicle_ecu`, the vehicle agent |

Both are a `colcon build --merge-install`, release profile, stripped, without
`--symlink-install`, `build/` and `log/` left out, plus `swb-release.yaml` (side, ROS
release, Autoware release for the vehicle side, arch, every repository's commit, each CARLA
binary's support tier), the `package.xml` files (for `rosdep install --from-paths`),
`LICENSES/` and a `setup.bash` guard that refuses a mismatched `ROS_DISTRO`, arch or, for
the vehicle side, sourced Autoware release, naming both values. A user without Autoware
takes the simulation side only.

**A link check** gates every artefact: each shared object and executable resolves every
symbol (`ldd -r` on a clean image of the target release with only the base, the artefact
and its rosdep packages installed). An Autoware 1.5.0 rebuilt from a local repository was
found on 2026-11-17 to ship a library that uses `diagnostic_updater` without linking it;
that kind of fault fails the release, not a user's first run.

### `pip install swb` picks the release

Each Ubuntu LTS pairs one ROS LTS with one system Python, and apt ROS runs on that Python:

| Ubuntu | ROS 2 | Python |
|---|---|---|
| 22.04 | Humble | 3.10 |
| 24.04 | Jazzy | 3.12 |
| 26.04 | Lyrical | (to be checked in step 8) |

- **`swb`** (pure Python, `py3-none-any`): the front end (`swb install`, `swb run`, the
  testbench commands) and `colcon-swb`. Its metadata requires the release package by
  environment marker:
  `swb-ros-humble == <ver>; python_version == "3.10"`,
  `swb-ros-jazzy == <ver>; python_version == "3.12"`.
- **`swb-ros-<distro>`** (pure Python, small): the release's manifest: each artefact's URL
  (GitHub Releases), size and sha256. No binaries in any wheel (PyPI's 100 MB default, and
  the manylinux policy assumes self-contained wheels; swb links `/opt/ros`).
- **First run**: any `swb` command finds no install for its release and downloads the
  simulation tarball (and, when a sourced Autoware is found, the matching vehicle tarball;
  else on the first pilot that needs it), checks the sha256, unpacks to
  `~/.local/share/swb/<ver>-<distro>/` (`SWB_HOME` overrides), points `current` at it, and
  prints the rosdep line for the system packages that are missing and the `source` line.
  `swb install [--distro <d>] [--autoware <rel>]` does the same on purpose; versions sit
  side by side.
- **Guard**: every start compares the install's ROS release with `$ROS_DISTRO` and refuses a
  mismatch (a source-built Humble on 24.04, say), naming `pip install swb-ros-<distro>`.
- Ubuntu 24.04's system Python refuses `pip install` outside a virtual environment (PEP
  668). The install page gives `pip install --user --break-system-packages swb`, or a venv
  with `--system-site-packages`, as play_launch's install page does. The front end must
  import no ROS module at install, so a `pipx` install also works for the downloader.

### The source build

`git clone --recursive`, `rosdep install --from-paths src --ignore-src -y`, then `colcon build`
with colcon-cargo-ros2 for the Rust packages (both documented today in install.md). rosdep
must resolve everything: every `package.xml` declares its real dependencies (079 step 6's
Cyclone DDS is one gap), and the Autoware message packages come from the vendored repos file
(029) or from the user's Autoware. A source build is an underlay exactly like the tarball.

## Cutting a release

Asked by the user, 2026-11-17: "Prepare a workflow to cut the release on Pypi once the infra
is ready. We'll tag a release at that time." So v0.1.0 is tagged when this workflow first
publishes, not before; the release notes already written for v0.1.0 (docs/release-notes.md)
are its notes, with an install section for pip added then.

**PyPI names** (checked 2026-11-17): `swb` is taken on PyPI (an unrelated soil water balance
package, 5.0.1). Free: `scenario-world-bridge`, `swb-ros-humble`, `swb-ros-jazzy`,
`colcon-swb`. Assumed until the user decides: the distribution is `scenario-world-bridge`
and its command is `swb` (a console-script entry point), so a user types
`pip install scenario-world-bridge`, then `swb`.

### What one release publishes

| Where | What |
|---|---|
| GitHub Release `v<ver>` | `swb-sim-<ver>-humble-x86_64.tar.zst`, `swb-vehicle-<ver>-autoware1.5.0-x86_64.tar.zst` (more rows per release and Autoware release later), `SHA256SUMS`, the notes |
| PyPI | `scenario-world-bridge` (front end + `colcon-swb`), `swb-ros-humble` (the manifest naming the GitHub Release's URLs and sha256), `colcon-swb` (the plugin alone) |
| git | the annotated tag `v<ver>` on the commit the artefacts were built from |

### The workflow (`.github/workflows/release.yml`, written in step 8)

Triggered by `workflow_dispatch` with the version as input, never by a pushed tag, so nothing
publishes by accident; the tag is made by the workflow at the end, on the commit it built.

1. **Preflight** (no secrets): the version matches every `pyproject.toml` and
   `docs/release-notes.md`'s heading; `check_pins`, `check_docs` and the tests pass on that
   commit; the tag does not exist yet.
2. **Build the tarballs**, one job per row of the matrix (`ubuntu-22.04` container with ROS
   Humble from apt; the vehicle side also with Autoware 1.5.0's published packages): the
   release build of step 5, then the link check on a fresh container with only the base,
   the tarball and its rosdep packages.
3. **Build the wheels** (pure Python, once): `scenario-world-bridge`, `colcon-swb`, and per
   ROS release `swb-ros-<distro>` with the manifest written from step 2's sha256 and the
   GitHub Release URLs the release will have.
4. **Smoke test** on a clean `ubuntu-22.04` + Humble container: install the wheels from the
   job's files (`pip install --no-index --find-links`), point the manifest at the job's
   tarballs (a local URL), run the first-run download, the rosdep line, `swb testbench new`,
   `colcon build`, `swb testbench list` and a no-simulator run (`--dry-run`). A run against
   CARLA stays the manual exit run (step 11), done before the workflow is started.
5. **TestPyPI**: publish the wheels to test.pypi.org, then install from there on a clean
   container and repeat step 4's first-run check against a draft GitHub Release holding the
   tarballs.
6. **Approval**: the `pypi` GitHub environment requires the user's review. Nothing past this
   point runs without it (standing rule: publishing needs the user's word).
7. **Publish**: the GitHub Release (from the draft, notes from `docs/release-notes.md`'s
   section), then the wheels to PyPI by trusted publishing (OIDC, no API token stored), then
   the annotated tag `v<ver>` pushed on the built commit.
8. **After**: a clean container installs `scenario-world-bridge` from PyPI and runs `swb
   --version` through the real first-run download; the result goes in the run's summary.

Order matters: tarballs are on the GitHub Release before the manifest wheel is on PyPI, so
no published wheel ever names a missing file.

### What the user sets up once (the infra)

- A PyPI project for each of the three names, with a **trusted publisher** for this
  repository and workflow (`release.yml`, environment `pypi`); the same on TestPyPI
  (environment `testpypi`). PyPI allows a "pending publisher" before the first upload.
- The GitHub environments `testpypi` and `pypi`, with the user as required reviewer on
  `pypi`.
- GitHub-hosted runners are assumed; the SSv2 and Rust builds may need a larger runner or a
  self-hosted one (070 B already allowed for it), measured in step 5.

### Before the first release

The workflow is ready when steps 2, 3, 5, 7 and 8 are done and step 11's exit runs pass. The
v0.1.0 release notes then get an install section (pip first, source build second) and the
version moves to the first release that ships this way; the user decides whether that is
still v0.1.0.

## Steps

Each step is pushed after review, with its tests.

1. **Docs** (133, this doc; the index; 070 B and 029 pointed here).
1b. **The rename to scensim**, before any new package exists. Its own short plan first
   (the GitHub repositories, which the user renames; the ROS packages `swb_*`; the Python
   package and command; the host settings `SWB_*`; the docs, the site and the datasets'
   references), then one change per repository, pins after submodules as always. Old names
   are refused by name, never aliased (no shims). The user's GitHub renames come first;
   GitHub redirects the old URLs, and `.gitmodules` moves to the new ones.
2. **`colcon-swb` and the testbench type.** The plugin, the ament index entries,
   `--symlink-install`, the check at build; `testbench check` (today's `project check`
   rules plus the `pkg://`-declared and no-`../` rules). Tests: a fixture workspace with a
   testbench, a support package and a scenarios package; colcon build fails on a bad
   testbench with the check's message.
3. **swb finds and runs testbenches by name.** `swb run <testbench>`, `testbench new`,
   `testbench list`, the logs path by package, `swb.yaml` refused by name; the generator
   decision (run-time suite step or a support package's executable).
4. **The studies as testbenches.** `scenarios/projects/*` and `src/swb_examples/projects/*`
   become testbench packages; shared scenarios become packages named by `pkg://`; c2b's and
   s5's pilots move from `scripts/coop/` into a `coop_pilots` support package; the stale
   `swb.lock` of courses, drift_lap and speed_ladder (no `carla-0.9.16-isaac-usd` entry,
   found 2026-11-17) refreshed; t3.yaml follows. Private testbenches (NTU, Shalun, the golf
   cart) move to a private repository's workspace; swb ships none of them.
5. **The release build, local.** Both tarballs for Humble on this host's tooling: the
   merged release install, stripping, `swb-release.yaml`, the guard, the vendored message
   packages, the link check on a clean Ubuntu 22.04 image. 029 steps 2-3's measurements
   redone (size, build time).
6. **A world server from a support package**: `world: {server: ...}`, the conformance check,
   a fixture world server package, its docs page.
7. **The pip front end.** `swb`, `swb-ros-humble`, the first-run download, `swb install`,
   the guard, `SWB_HOME`, the rosdep line; the install page (both Ubuntu releases' pip
   forms). Tests: a fake release served locally (sha256 pass and fail, a partial download, a
   mismatched `ROS_DISTRO`).
8. **CI and publishing.** `release.yml` as in "Cutting a release": tarballs, wheels and the
   manifest, a smoke test per artefact on a clean job (029 step 5), TestPyPI, then GitHub
   Releases and PyPI behind the user's approval; the tag made last.
9. **Jazzy**, its own phase (029's note stands: the SSv2 fork and acb are Humble-only; Jazzy
   needs Ubuntu 24.04). This phase makes it one more matrix entry; Lyrical after.
10. **Docs for users** (243): install by pip first, the source build second; the tutorials
    on a testbench workspace; a reference page for `testbench.yaml` and the build type; the
    "own world server" and "own pilot" how-tos as support packages.
11. **Exit runs.**

## Exit criteria

- On a clean Ubuntu 22.04 with ROS 2 Humble from apt: the Goal's commands pass as written,
  with root only for the rosdep line, and tutorials 1 to 5 pass on a testbench workspace.
- The same workspace builds and passes tutorial 1 on a source build of swb.
- A testbench that names an undeclared package, leaves its directory, or has a schema error
  fails `colcon build` with the check's message.
- A mismatched `ROS_DISTRO` is refused at start, naming the release package to install.
- Every artefact passes the link check on a clean image.
- check_docs, the tests and check_pins pass.

## Non-goals

- Debs, a buildfarm release through bloom and rosdistro (bloom has no `ament_cargo`
  support today, and the SSv2 fork cannot be released under upstream's names). colcon2deb
  can make debs from the same build later if asked.
- Docker images, conda or pixi environments.
- Shipping any autopilot: Autoware is the user's, from source or from NEWSLab's binaries.
- Private assets or testbenches in any public artefact.

## Settled after the builder check (133's calls, 2026-11-17; the user may overrule)

- **The simulator is the user's, as the autopilot is.** The exit host has CARLA 0.9.16 (the
  packaged release) and NEWSLab's Autoware 1.5.0 binaries installed beforehand; swb fetches
  neither. A CARLA downloader (029's `csb_setup`) is a later convenience, not in this phase.
- **The vehicle tarball serves NEWSLab's Autoware 1.5.0 binaries.** A user with a source
  build of Autoware builds the vehicle packages (acb, aib, `autoware_sim_vehicle`,
  `sim_vehicle_spec`, `sim_vehicle_ecu`, all public) from source in their workspace, as
  support packages; the guard compares the sourced Autoware's prefix and version with
  `swb-release.yaml`'s and names that path on a mismatch.
- **The wheel's `swb` is a thin launcher.** It imports no ROS; after the first-run download
  it execs the tarball's `swb` (the ROS package `src/swb`, unchanged in place). The wheel
  and the tarball carry the same version, pinned exactly in `swb-ros-<distro>`'s manifest;
  a skew is refused by the launcher, naming both.
- **Assets and the lock.** Fetched assets live in the host cache (as today, `~/.cache/swb`)
  and the host state directory, never in `share/<pkg>`. `swb assets fetch <testbench>`
  writes `swb.lock` to the testbench's source directory (found through the
  `--symlink-install` link, or `--source <dir>`); an installed copy without its source can
  only use its committed lock. The build-time check validates the lock's syntax only.
- **Isaac is shipped, not in the clean-image exit runs.** `swb_isaac_world` and aib go in
  the tarballs; the `isaac` template passes `colcon build`; Isaac runs are checked on a lab
  host with Isaac Sim, as in 077.
- **The Autoware fixes and TensorRT 10.8 belong to NEWSLab's Autoware binaries** (user,
  2026-11-17): the overlay's three packages (`blind_spot`, `pose_instability_detector`,
  `ndt_scan_matcher`) are built into autoware-localrepo's 1.5.0 packages, and its docs name
  the TensorRT 10.8 runtime; swb ships no Autoware part. The developer overlay
  (`just build-autoware-overlay`) stays for source checkouts until those binaries carry the
  fixes. That rebuild must pass the link check too (see "A link check" above).
- **Private testbenches** go to a new private repository the user creates (user,
  2026-11-17), with a workspace holding the NTU, Shalun and golf-cart testbenches.
- **The project's name is scensim** (user, 2026-11-17: "It's no longer a bridge. It becomes a
  framework. I'd like to find a memorizable name"; then "scensim", for SCENario SIMulation,
  from names built on scenario plus simulation). `swb` is taken on PyPI; `scensim` is free
  (checked 2026-11-17, as are `scensim-ros-humble` style names to be checked at step 1b).
  The rename is step 1b, before step 2, so the plugin (`colcon-scensim`), the build type
  (`scensim_testbench`) and the wheels are born under it. Until then this doc says swb.
- **x86_64 only** for now (CARLA has no aarch64 build).
- **Redistribution** (checked 2026-11-17): this repository and every NEWSLabNTU submodule
  in the tarballs are public and Apache-2.0, as is `color_names`. `cuda_ndt_matcher` is
  public with no licence and is built on its own (optional `ndt:=cuda`); it stays out of
  the tarballs until it has one.

## Open, for the builder

- The tarball host (GitHub Releases assumed) and whether artefacts are signed beyond sha256.
- How `swb install` finds the sourced Autoware's release (its `package.xml` versions, or the
  setup path).
- The `testbench.yaml` file name could stay `swb.yaml` if the user prefers; this doc assumes
  the rename.
