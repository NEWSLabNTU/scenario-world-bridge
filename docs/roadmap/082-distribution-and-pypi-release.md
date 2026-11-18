# Phase 082: Distribution, `pip install scensim` and the PyPI Release

A user with an LTS ROS 2 from apt runs `pip install scensim`, then `scensim` once; the first
command downloads the prebuilt scensim for their ROS release, from a tarball of the built
workspace. No step but one rosdep line needs root. A source build stays documented:
`rosdep install`, then `colcon build` with colcon-cargo-ros2. Releases are cut by a
workflow that publishes to GitHub Releases and PyPI behind the user's approval, and makes
the tag last.

**Status**: 0.1.0 released 2026-11-18 (PyPI `scensim`, `scensim-ros-humble`, `colcon-scensim`; GitHub release v0.1.0 on d90ac38; run 44267937009), plus a placeholder `scensim-ros-jazzy` 0.1.0 that refuses clearly on 24.04. Next: 0.2.0 with Jazzy (step 7; the sim set builds on Jazzy, 118's `jazzy-0.2.0`).

**Source**: user, 2026-11-17, in one design conversation:

- "They installed the swb in a convenient way, either by pip, npm or something."
- "The user will install lts ROS." scensim fits into it and brings no ROS of its own.
- "I would provide prebuilt binaries and also document source build method. Use
  colcon-cargo-ros2 and make rosdep work."
- After an apt repository was proposed: "What if we provide install/ binaries? User does not
  need root to install Deb". Then: "Tarball plus pip front end looks good to me. Check how
  users can get the right distribution for their environment. Best if it can be done
  automatically by pip install swb", and "One time download at the first swb is ok".
- "Prepare a workflow to cut the release on Pypi once the infra is ready. We'll tag a
  release at that time." So v0.1.0 is not tagged on its own: it is tagged when this
  workflow first publishes.
- The Autoware fixes and the TensorRT 10.8 runtime go into NEWSLab's Autoware binaries;
  scensim ships no Autoware part.

**Supersedes**: [070](070-user-experience.md) section B and [029](029-prebuilt-releases.md),
whose measurements and tarball design (a relocatable merged install, the vendored Autoware
message packages, a CARLA binary per version behind 070 A's launcher, a release manifest,
an install-deps list, a setup guard) this phase keeps. Debs were weighed: ROS's official
path (bloom, rosdistro, the buildfarm) has no `ament_cargo` support and cannot release the
SSv2 fork under upstream's names; colcon2deb (NEWSLab's) automates ROS's documented local
method and could make debs from the same build later.

## Goal

On a clean Ubuntu 22.04 with ROS 2 Humble from apt (later 24.04 with Jazzy), with the
user's simulator and autopilot already installed (CARLA 0.9.16 and NEWSLab's Autoware 1.5.0
binaries for the exit runs):

```bash
pip install scensim                  # Ubuntu 24.04: see PEP 668 below
scensim --version                    # first run: downloads scensim for Humble, checks its sha256
sudo rosdep install ...              # the one root step, printed by scensim
source ~/.local/share/scensim/current/setup.bash
mkdir -p ~/my_ws/src && cd ~/my_ws/src && scensim testbench new my_first_drive
cd ~/my_ws && colcon build --symlink-install && source install/setup.bash
scensim run my_first_drive
```

## Design

### The artefacts: one tarball per ROS release, and per Autoware release for the vehicle side

| Artefact | Built per | Holds |
|---|---|---|
| `scensim-sim-<ver>-<distro>-<arch>.tar.zst` | ROS release x arch | the bridge, relay, `scensim_launch`, `scensim_interfaces`, `scensim_world_protocol`, the CARLA world server (a binary per CARLA version), `scensim_isaac_world`, the SSv2 fork, the vendored Autoware message packages (029), `scensim_examples`, the run tooling (`scensim`) |
| `scensim-vehicle-<ver>-autoware<rel>-<arch>.tar.zst` | Autoware release x arch | acb, aib, `autoware_sim_vehicle`, `sim_vehicle_spec`, `sim_vehicle_ecu`, the vehicle agent |

Both are a `colcon build --merge-install`, release profile, stripped, without
`--symlink-install`, with `build/` and `log/` left out, plus `scensim-release.yaml` (side,
ROS release, Autoware release for the vehicle side, arch, every repository's commit, each
CARLA binary's support tier), the `package.xml` files (for `rosdep install --from-paths`),
`LICENSES/`, and a `setup.bash` guard that refuses a mismatched `ROS_DISTRO`, arch or, for
the vehicle side, sourced Autoware, naming both values. A user without Autoware takes the
simulation side only. The vehicle tarball serves NEWSLab's Autoware 1.5.0 binaries; a user
with a source build of Autoware builds the vehicle packages (all public) from source in
their workspace, as support packages.

**A link check** gates every artefact: each shared object and executable resolves every
symbol (`ldd -r` on a clean image of the target release with only the base, the artefact
and its rosdep packages). An Autoware 1.5.0 rebuilt from a local repository was found on
2026-11-17 to ship a library that uses `diagnostic_updater` without linking it; that kind of
fault fails the release, not a user's first run. That build was NEWSLab's own
(`autoware-localrepo` 1.5.0-3): its packages failed the check, and 1.5.0-humble-4
(2026-11-18) fixes the link in all five platforms' packages (amd64, amd64-cpu, sbsa,
arm64-cpu, jetpack62) and adds `cuda-cudart-12-4`; each passed `ldd -r` and a link scan
after a clean install. `apt upgrade` does not repair an existing 1.5.0-3 install (the inner
packages keep their versions): uninstall, then install `autoware-full-1-5-0` again.

### `pip install scensim` picks the release

Each Ubuntu LTS pairs one ROS LTS with one system Python, and apt ROS runs on that Python:

| Ubuntu | ROS 2 | Python | Released |
|---|---|---|---|
| 22.04 | Humble | 3.10 | 0.1.0: the simulation side only; an Autoware ego builds the vehicle packages from source (found 2026-11-18: release.yml builds no vehicle tarball, though this table and install.md said it did) |
| 24.04 | Jazzy | 3.12 | 0.2.0: the simulation side only (Autoware 1.5.0 is Humble only); 0.1.0's scensim-ros-jazzy is a placeholder whose commands refuse, naming 22.04 |
| 26.04 | Lyrical | (to be checked) | none |

- **`scensim`** (pure Python, `py3-none-any`): a thin front end and `colcon-scensim`. It
  imports no ROS. Its metadata requires the release package by environment marker:
  `scensim-ros-humble == <ver>; python_version == "3.10"`,
  `scensim-ros-jazzy == <ver>; python_version == "3.12"`.
- **`scensim-ros-<distro>`** (pure Python, small): the release's manifest, each artefact's
  URL on GitHub Releases, size and sha256. No binaries in any wheel (PyPI's 100 MB default;
  manylinux assumes self-contained wheels, and scensim links `/opt/ros`).
- **First run**: any `scensim` command finds no install for its release, downloads the
  simulation tarball (and the vehicle tarball when a sourced Autoware is found, else on the
  first pilot that needs it), checks the sha256, unpacks to
  `~/.local/share/scensim/<ver>-<distro>/` (`SCENSIM_HOME` overrides), points `current` at
  it, and prints the missing system packages' rosdep line and the `source` line. Then the
  front end execs the tarball's `scensim`. `scensim install [--distro <d>] [--autoware
  <rel>]` does the same on purpose; versions sit side by side.
- **Guards**: every start compares the install's ROS release with `$ROS_DISTRO` (a
  source-built Humble on 24.04, say) and refuses a mismatch, naming
  `pip install scensim-ros-<distro>`; the wheel and the tarball carry the same version, and
  a skew is refused, naming both.
- **Ubuntu 24.04** refuses `pip install` outside a virtual environment (PEP 668). The
  install page gives `pip install --user --break-system-packages scensim`, or a venv with
  `--system-site-packages`, as play_launch's install page does; `pipx` works for the front
  end because it imports no ROS.

### The source build

`git clone --recursive`, `rosdep install --from-paths src --ignore-src -y`, then
`colcon build` with colcon-cargo-ros2 for the Rust packages. rosdep must resolve
everything: every `package.xml` declares its real dependencies (079 step 6's Cyclone DDS is
one gap), and the Autoware message packages come from the vendored repos file (029) or the
user's Autoware. A source build is an underlay exactly like the tarball.

## Measured (step 2, 118, 2026-11-17, main e3bfe56, CARLA 0.9.16 only)

The simulation tarball for Humble, built in a clean shell with only `/opt/ros/humble`
sourced: 51 packages (`--packages-up-to` the simulation set), `--merge-install`, cargo
`--release`, `CMAKE_BUILD_TYPE=Release`, `BUILD_TESTING=OFF`; the Autoware message packages
vendored at Autoware 1.5.0's `autoware.repos` pins. Report on the lab NAS, "2026-11-17 082
step 2 tarball measurement".

| | |
|---|---|
| Build, from clean | 382 s wall (48 cores, load 20 to 32) |
| As built | 912.6 MB |
| Stripped (276 ELF files) | 290.4 MB; about 170 MB without SSv2's embree tutorials (73 MB), headers (35 MB) and static libraries (11.5 MB) |
| `tar.zst` | 65.1 MB at `-19` (22.7 s), 101.3 MB at `-3` |
| Relocation | unpacked elsewhere: `ros2 pkg prefix` and the launch's `--show-args` work; RPATH holds only `/opt/ros/humble/lib` |
| Link check (`ldd -r`, 276 files) | on a clean `ros:humble-ros-core` image: 68 fail before rosdep, 1 after (the SSv2 RViz plugin's Ogre libraries, likely an `ldd`-only false positive) |
| rosdep beyond bare Humble | 53 keys, 47 apt packages (41 with `--dependency-types exec`) |

To fix before the release build is scripted (each in step 2, or step 4 for `package.xml`):

1. **colcon-cargo-ros2 0.5.4 breaks under `--merge-install`**: its generated `build.rs`
   links from `install/<package>/lib` (the isolated layout), so the bridge fails with
   `unable to find library -lswb_interfaces__rosidl_typesupport_c`. Worked around with
   `AMENT_PREFIX_PATH=<ws>/install`; to be fixed in colcon-cargo-ros2 (the user's call where
   it is filed).
2. **The SSv2 fork fails a Release build on Ubuntu 22.04**: `traffic_simulator`, Boost
   geometry's `rescale_policy.hpp:46`, `-Werror=maybe-uninitialized` (`just build` sets no
   build type, so it never meets it). Worked around with `-Wno-error=maybe-uninitialized`;
   fix in the fork.
3. **`swb_launch` exec-depends on the vehicle side** (`acb_bridge`, `acb_launch`,
   `aib_launch`, `autoware_sim_profile`, `czp`), so rosdep refuses the simulation tarball
   without `--skip-keys`; `scenario_test_runner`'s `python-yamale-pip` does not resolve
   either. The dependencies split by side (step 4).
4. **Not relocatable**: `swb_isaac_world`'s scripts default `SWB_ISAAC_DIR` to the source
   tree; `openscenario_interpreter`'s CMake exports name a `build/` path.
5. **`--help` is ignored** by `scenario_world_bridge` and `swb_carla_world`: both start
   serving.
6. The tarball leaves out build-only parts (embree's tutorials, static libraries, the CMake
   exports and headers of internal packages), but keeps what a support package builds
   against: the message packages (`swb_interfaces` and the vendored Autoware messages) and
   the world protocol's libraries and headers, since 081's own world server and pilots link
   them.

## Cutting a release

### What one release publishes

| Where | What |
|---|---|
| GitHub Release `v<ver>` | `scensim-sim-<ver>-humble-x86_64.tar.zst`, from 0.2.0 `scensim-sim-<ver>-jazzy-x86_64.tar.zst`, planned, not built yet: `scensim-vehicle-<ver>-autoware<rel>-x86_64.tar.zst` per Autoware release (0.1.0 shipped none), one `SHA256SUMS` for all, the notes |
| PyPI | `scensim` (front end + plugin), `scensim-ros-humble` and from 0.2.0 `scensim-ros-jazzy` (the manifests), `colcon-scensim` (the plugin alone) |
| git | the annotated tag `v<ver>` on the commit the artefacts were built from |

### The workflow (`.github/workflows/release.yml`)

Started by `workflow_dispatch` with the version as input, never by a pushed tag, so nothing
publishes by accident; the workflow makes the tag at the end, on the commit it built.

1. **Preflight** (no secrets): the version matches every `pyproject.toml` and
   `docs/release-notes.md`'s heading; check_pins, check_docs and the tests pass on that
   commit; the tag does not exist yet.
2. **Build the tarballs**, one job per row of the matrix (an `ubuntu-22.04` container with
   ROS Humble from apt; the vehicle side also with NEWSLab's Autoware 1.5.0 packages), then
   the link check on a fresh container with only the base, the tarball and its rosdep
   packages.
3. **Build the wheels** (pure Python, once): `scensim`, `colcon-scensim`, and per ROS release
   `scensim-ros-<distro>` with the manifest written from step 2's sha256 and the URLs the
   release will have.
4. **Smoke test** on a clean `ubuntu-22.04` + Humble container: install the wheels from the
   job's files (`pip install --no-index --find-links`), point the manifest at the job's
   tarballs (a local URL), then the first-run download, the rosdep line, `testbench new`,
   `colcon build`, `testbench list` and a run with `--dry-run`. A run against CARLA is the
   manual exit run, done before the workflow is started.
5. **No TestPyPI stage** (the user, 2026-11-18): the release goes from smoke straight to
   the approval; the first public install is checked by step 8 instead.
6. **Approval**: the `pypi-scensim-ros-humble` GitHub environment (the first upload)
   requires the user's review. Nothing past this point runs without it (standing rule:
   publishing needs the user's word).
7. **Publish**: the GitHub Release (drafted with the tarballs, then made public, notes from
   `docs/release-notes.md`'s section), then the wheels to PyPI by trusted publishing (OIDC,
   no stored token), one job and one environment per project, dependencies first
   (scensim-ros-humble, colcon-scensim, then scensim), then the tag pushed on the built
   commit once all three are up.
8. **After**: a clean container installs `scensim` and `colcon-scensim` from PyPI and runs
   `scensim install` and `scensim --version` through the real first-run download; the result
   goes in the run's summary.

The tarballs are on the GitHub Release before the manifest wheel is on PyPI, so no
published wheel ever names a missing file.

### What the user sets up once (the infra)

- A **trusted publisher** on PyPI for each of the three projects, each with its own GitHub
  environment: PyPI refuses a second pending publisher with the same
  repository, workflow and environment for another project name ("A pending trusted
  publisher matching this configuration has already been registered for a different project
  name"). A pending publisher can be registered before the first upload, which also holds
  the name. Every form takes owner `NEWSLabNTU`, repository `scensim` and workflow
  `release.yml`; the project name and environment differ:

  | PyPI project name | Environment name | Required reviewer | Order |
  |---|---|---|---|
  | `scensim-ros-humble` | `pypi-scensim-ros-humble` | the user | 1 |
  | `scensim-ros-jazzy` | `pypi-scensim-ros-jazzy` | none | 2 (from 0.2.0; 0.1.0's placeholder was published on its own) |
  | `colcon-scensim` | `pypi-colcon-scensim` | none | 2 |
  | `scensim` | `pypi` | none | 3 |

- The same three GitHub environments in the repository (Settings, Environments), with the user
  as required reviewer on `pypi-scensim-ros-humble` only. The dependencies publish first, so
  scensim never stands on PyPI without the scensim-ros-humble it pins: `publish`
  (scensim-ros-humble) waits for that approval and makes the release, with the tarballs,
  public before its upload, then `publish-colcon` (colcon-scensim) and `publish-scensim`
  (scensim) follow without a reviewer, and `tag` tags the release only after all three
  uploads succeed. No TestPyPI publishers or environments are needed (the user,
  2026-11-18).
- The repository variable `SCENSIM_RELEASE_ENABLED` set to `true`. Until then every run
  stops after smoke.
- GitHub-hosted runners are assumed; the SSv2 and Rust builds may need a larger runner or a
  self-hosted one, measured in step 2.

### The first release

The workflow is ready when steps 2 to 6 are done, 080 and 081 have landed, and step 8's
exit runs pass. The release notes (`docs/release-notes.md`, written for v0.1.0 on
2026-11-17) then get an install section, pip first and the source build second; the user
decides whether that release is still v0.1.0.

## Steps

Each step is pushed after review, with its tests.

1. **Docs** (133): this doc.
2. **The release build, local** (118, may start before 080): the simulation tarball for
   Humble (merged release install, stripping, relocation, the link check on a clean image),
   its size and build time, the apt packages rosdep adds to a bare ROS; then the vehicle
   tarball; `scensim-release.yaml` and the guard; the vendored message packages.
3. **The pip front end**: `scensim`, `scensim-ros-humble`, the first-run download,
   `scensim install`, the guards, `SCENSIM_HOME`, the rosdep line. Tests: a fake release
   served locally (sha256 pass and fail, a partial download, a mismatched `ROS_DISTRO`, a
   version skew).
4. **The source build made complete**: every `package.xml`'s dependencies, so `rosdep
   install` alone prepares a clean host; the install page's source section.
5. **`release.yml`** as above, run end to end through smoke (no TestPyPI stage: the user,
   2026-11-18).
6. **Docs for users** (243): the install page, pip first, both Ubuntu releases' pip forms,
   the source build second.
7. **Jazzy**, its own phase, one more matrix row in release.yml (0.2.0); Lyrical after.
   The feasibility build (118, 2026-11-18, ros:jazzy) found 029's note no longer holds:
   the SSv2 fork and acb build on Jazzy. What Jazzy needs: libclang 14 for carla-sys
   (autocxx's bindings drop `const` with 18), `--break-system-packages` and
   `PIP_BREAK_SYSTEM_PACKAGES=1 PIP_IGNORE_INSTALLED=1` for pip and rosdep's pip keys
   (PEP 668), the release package `scensim-ros-jazzy`. The simulation side only: Autoware
   1.5.0 is Humble only, and the front end refuses the vehicle side on Jazzy by name.
8. **Exit runs.**

## Exit criteria

- On a clean Ubuntu 22.04 with ROS 2 Humble from apt: the Goal's commands pass as written,
  with root only for the rosdep line, and tutorial 1 passes on the installed scensim.
- The same workspace builds and passes tutorial 1 on a source build.
- A mismatched `ROS_DISTRO` and a wheel/tarball version skew are refused at start, naming
  the fix.
- Every artefact passes the link check on a clean image.
- `release.yml` has run end to end through smoke on main (no TestPyPI stage: the user,
  2026-11-18).
- check_docs, the tests and check_pins pass.

## Settled (133's calls after a builder check, 2026-11-17; the user may overrule)

- **The simulator is the user's**, as the autopilot is; scensim fetches neither. A CARLA
  downloader (029's `csb_setup`) is a later convenience.
- **Isaac is shipped, not in the clean-image exit runs**: `scensim_isaac_world` and aib go in
  the tarballs; Isaac runs are checked on a lab host with Isaac Sim, as in 077.
- **x86_64 only** for now (CARLA has no aarch64 build).
- **Redistribution** (checked 2026-11-17): this repository and every NEWSLabNTU submodule in
  the tarballs are public and Apache-2.0, as is `color_names`. `cuda_ndt_matcher` is public
  with no licence and stays out of the tarballs until it has one.
- **The Autoware fixes and TensorRT 10.8** belong to NEWSLab's Autoware binaries (user): the
  overlay's three packages built into autoware-localrepo's 1.5.0 set, the TensorRT runtime
  named in its docs. The developer overlay stays for source checkouts until then.

## Non-goals

- Debs and a buildfarm release (above); Docker images; conda or pixi.
- Shipping any autopilot or simulator.
- Private assets or testbenches in any public artefact.

## Open, for the builder

- Whether artefacts are signed beyond sha256.
- How `scensim install` reads the sourced Autoware's release (its `package.xml` versions, or
  the setup path).
