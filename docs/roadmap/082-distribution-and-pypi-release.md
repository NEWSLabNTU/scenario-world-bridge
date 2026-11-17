# Phase 082: Distribution, `pip install scensim` and the PyPI Release

A user with an LTS ROS 2 from apt runs `pip install scensim`, then `scensim` once; the first
command downloads the prebuilt scensim for their ROS release, from a tarball of the built
workspace. No step but one rosdep line needs root. A source build stays documented:
`rosdep install`, then `colcon build` with colcon-cargo-ros2. Releases are cut by a
workflow that publishes to GitHub Releases and PyPI behind the user's approval, and makes
the tag last.

**Status**: planned, 2026-11-17. Nothing is built yet. Follows [080](080-rename-to-scensim.md)
(names) and ships [081](081-testbench-packages.md)'s plugin and templates. Step 2's local
measurement publishes nothing and may run before 080.

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
fault fails the release, not a user's first run. NEWSLab's Autoware binaries, which now
carry the overlay's fixes, pass the same check.

### `pip install scensim` picks the release

Each Ubuntu LTS pairs one ROS LTS with one system Python, and apt ROS runs on that Python:

| Ubuntu | ROS 2 | Python |
|---|---|---|
| 22.04 | Humble | 3.10 |
| 24.04 | Jazzy | 3.12 |
| 26.04 | Lyrical | (to be checked) |

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

## Cutting a release

### What one release publishes

| Where | What |
|---|---|
| GitHub Release `v<ver>` | `scensim-sim-<ver>-humble-x86_64.tar.zst`, `scensim-vehicle-<ver>-autoware1.5.0-x86_64.tar.zst` (more rows per ROS and Autoware release later), `SHA256SUMS`, the notes |
| PyPI | `scensim` (front end + plugin), `scensim-ros-humble` (the manifest), `colcon-scensim` (the plugin alone) |
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
5. **TestPyPI**: publish the wheels to test.pypi.org, install from there on a clean
   container, and repeat step 4's first run against a draft GitHub Release holding the
   tarballs.
6. **Approval**: the `pypi` GitHub environment requires the user's review. Nothing past this
   point runs without it (standing rule: publishing needs the user's word).
7. **Publish**: the GitHub Release (from the draft, notes from `docs/release-notes.md`'s
   section), then the wheels to PyPI by trusted publishing (OIDC, no stored token), then the
   tag pushed on the built commit.
8. **After**: a clean container installs `scensim` from PyPI and runs `scensim --version`
   through the real first-run download; the result goes in the run's summary.

The tarballs are on the GitHub Release before the manifest wheel is on PyPI, so no
published wheel ever names a missing file.

### What the user sets up once (the infra)

- A PyPI project for each of the three names, each with a **trusted publisher** for this
  repository and workflow (`release.yml`, environment `pypi`); the same on TestPyPI
  (environment `testpypi`). A "pending publisher" can be registered before the first upload,
  which also holds the name.
- The GitHub environments `testpypi` and `pypi`, with the user as required reviewer on
  `pypi`.
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
5. **`release.yml`** as above, run end to end against TestPyPI and a draft release.
6. **Docs for users** (243): the install page, pip first, both Ubuntu releases' pip forms,
   the source build second.
7. **Jazzy**, its own phase (029's note stands: the SSv2 fork and acb are Humble-only; Jazzy
   needs Ubuntu 24.04). This phase makes it one more matrix row; Lyrical after.
8. **Exit runs.**

## Exit criteria

- On a clean Ubuntu 22.04 with ROS 2 Humble from apt: the Goal's commands pass as written,
  with root only for the rosdep line, and tutorial 1 passes on the installed scensim.
- The same workspace builds and passes tutorial 1 on a source build.
- A mismatched `ROS_DISTRO` and a wheel/tarball version skew are refused at start, naming
  the fix.
- Every artefact passes the link check on a clean image.
- `release.yml` has run end to end against TestPyPI and a draft GitHub Release.
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
