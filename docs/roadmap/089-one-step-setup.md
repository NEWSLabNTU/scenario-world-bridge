# 089: One-Step Setup

**Status**: done 2026-11-19, in 0.2.0 (the user's call, through 133; opened for 0.2.1).

## Goal

From a fresh Ubuntu with ROS 2 and an autopilot to a first testbench run in three commands:

```bash
pip install --user scensim
scensim setup            # one sudo prompt
source ~/scensim.bash
```

Today the install is nine or more manual steps (`docs/getting-started/install.md`,
`docs/how-to/connect-autoware.md`).

## What cannot be in a pip install, and how setup handles it

- **System packages** (rosdep keys of the prebuilt release, the CycloneDDS sysctl file) need
  sudo. `scensim setup` runs them behind one sudo prompt, or prints the exact commands with
  `--no-sudo`.
- **The car's Autoware packages** must be built against the user's own Autoware.
  `scensim autoware-side build` imports the release's pinned `autoware-side.repos` into
  `~/autoware_side` and builds it against the sourced Autoware.

## What `scensim setup` does

| Today | `scensim setup` |
|---|---|
| `scensim install` | already automatic on the first command; setup runs it explicitly |
| `export PATH=$HOME/.local/bin:$PATH` | warns only when the directory is not on PATH |
| `rosdep init`, `rosdep install` | runs them (one sudo prompt), or prints them (`--no-sudo`) |
| the CycloneDDS sysctl file | same sudo step; optional, skipped with a note |
| writing `~/scensim.bash` | generated from the detected ROS, Autoware and release |
| writing `~/.config/scensim/host.yaml` | detected: `/opt/autoware/*`, a CARLA unpacked under `~`, a fast log disk; never overwrites an existing file without `--force` |
| the Autoware side `vcs import` + `colcon build` | `scensim autoware-side build` (also run by setup when Autoware is sourced) |
| `ros2 run autoware_sim_profile setup_autoware_data` | run by setup when the profile is built |
| downloading CARLA | `scensim carla fetch <version>`, optional |
| checking | `scensim check` at the end, naming each fix |

Every step is idempotent and says what it does before it does it. Nothing is written outside
the user's home except under sudo, and only what the step names.

## Design (step 1)

### Commands

Three new commands of the pip front end, beside `install` and `--version`. Like those, they
import no ROS, and the release's own `scensim` forwards them to the front end
(`FRONT_END` in `src/scensim/scensim/cli.py`). A source build has no front end and refuses
them, naming `just setup`.

```
scensim setup [--no-sudo] [--force] [--autoware <setup.bash>|none]
scensim autoware-side build [--autoware <setup.bash>] [--workspace ~/autoware_side] [--force]
scensim carla fetch 0.9.16 [--dir ~]
```

`autoware` remains the release's command (overlay, tensorrt, where); `autoware-side` is
the front end's, so the two never collide.

### What setup detects

- **ROS**: the release's distro, from the Python version as `install` chooses it, and
  `/opt/ros/<distro>/setup.bash`; missing, it refuses and names the ROS install page.
- **Autoware**: `--autoware`, else the Autoware the terminal has sourced, else the only
  `/opt/autoware/*/setup.bash`. Several and none sourced: refused, naming each, with
  `--autoware`. None: a simulation-only setup (no Autoware side), said once.
- **CARLA**: `carla.root` in host.yaml, else a `~/CARLA_*` holding `CarlaUE4.sh`; none: a
  note naming `scensim carla fetch`.
- **The log disk**: when the home directory is on a spinning disk
  (`/sys/block/<dev>/queue/rotational`), a warning naming `logs:`; setup picks no disk.

### Order

1. Detect, then print the plan: the system step's script and every file it will write.
2. `scensim install` (a no-op when the release is in place).
3. **The system step**, one `sudo bash -c <script>`, the script printed first. Each line is
   skipped when already done, and a step with nothing left asks for no password:
   - `apt-get install` of what is missing among `git python3-rosdep python3-vcstool
     python3-colcon-common-extensions python3-pip`;
   - `rosdep init` when `/etc/ros/rosdep/sources.list.d/` has no list;
   - `rosdep update` as the user (`sudo -u $SUDO_USER -H`), so the cache stays the user's;
   - `rosdep install --from-paths <current>/sim/share --ignore-src -y`, with the PEP 668
     settings on Python 3.11 and later, as `install` prints it today;
   - `/etc/sysctl.d/99-cyclonedds-max.conf` and `sysctl -p` on it, unless the running values
     are already at least the file's. The front end carries the file's text; a test keeps it
     equal to `config/99-cyclonedds-max.conf`.

   `--no-sudo` prints the script and goes on with the user's steps; the check at the end
   names what is still missing.
4. `scensim autoware-side build`, when there is an Autoware.
5. `~/scensim.bash`: ROS, the Autoware, the release's `current/sim/setup.bash`, then
   `~/autoware_side/install/setup.bash` when built (install.md's order). It is written after
   the build, so it never sources a workspace that failed to build.
6. `~/.config/scensim/host.yaml`: `carla.root` and `autoware.setup` as detected.
7. `ros2 run autoware_sim_profile setup_autoware_data`, once (it is idempotent).
8. The check (below).

**An existing file is never overwritten** without `--force`. When `~/scensim.bash` or
host.yaml exists and differs from what setup would write, setup keeps it, prints the
difference (for host.yaml, the detected keys it lacks), and goes on. The check then judges the
file as it is.

### The Autoware side without a download

`scensim autoware-side build` needs no `autoware-side.repos` download. The installed
release's `scensim-release.yaml` already records every repository's commit, so the front end
writes the same file `scripts/release/autoware_side_repos.py` writes, from the
`src/autoware_sim_vehicle` commit there. This also works for 0.2.0's tarballs. Then, in `bash -c`
with ROS and that Autoware sourced:

```
vcs import src < autoware-side.repos
colcon build --packages-up-to tesla_model3_launch tesla_model3_sensor_kit_launch \
    individual_params autoware_sim_profile
```

The profile is always built (seconds, CMake only), so either `autoware_profile` works.
`~/autoware_side/.scensim-built` records the repos text and the Autoware's setup path. When
both match, the build is skipped and setup says so; `--force` rebuilds. A workspace that
holds other packages is refused unless `--workspace` names another directory.

### Not in setup

- CARLA (about 10 GB): `scensim carla fetch 0.9.16`. It downloads CARLA's own release
  tarball, checks it against a size and sha256 that the front end pins (CARLA publishes
  none), and unpacks it to `~/CARLA_0.9.16`. It writes host.yaml with `carla.root` only when
  there is no host.yaml, and otherwise prints the line to add. 0.9.16 only: the release's binaries serve no other.
- TensorRT 10.8.0 for Autoware 1.5.0 (2.6 GB): the release's `scensim autoware tensorrt`,
  which setup recommends for 1.5.0 and does not run.
- Maps: `scensim assets fetch <testbench>`, as Tutorial 1 has it.

### The check

At the end, and as `scensim setup --check` alone (no writes, no sudo). Each line is ok or
names its fix:
- the release is installed, and its `scensim --version` answers in `~/scensim.bash`'s
  environment;
- rosdep reports nothing missing for the sim share (`rosdep check`);
- the sysctl values;
- in `~/scensim.bash`'s environment, `ros2 pkg prefix` finds the car's packages and the
  profile;
- `seat show` answers;
- CARLA: `carla.root` holds `CarlaUE4.sh`, or a note.

Exit 0 when every line is ok, 3 otherwise; `--no-sudo` with system lines left also exits 3.

### Tests

These live in `pypi/scensim/tests` and need no network, sudo or ROS:
- a fake `/opt` (`SCENSIM_ROS_ROOT`, and `SCENSIM_AUTOWARE_ROOT` for `/opt/autoware`);
- `sudo`, `apt-get`, `dpkg-query`, `rosdep`, `vcs`, `colcon`, `sysctl` and `ros2` as stubs
  on `PATH` that record their calls;
- a release installed from a local tarball, as the front end's tests already do.

They check the plan, the script, what is skipped, idempotence (a second setup changes
nothing and asks no password), never overwriting, `--no-sudo`, the Autoware detection cases
and the check's verdicts.

### The clean-machine check (step 5)

It runs in Docker on 118:
- **Ubuntu 22.04**: ROS Humble and NEWSLab's Autoware 1.5.0 debs, the GPU passed through,
  host networking, and `carla.port` set to the host's CARLA. From `pip install --user
  scensim` (a wheel built from the branch, and the release tarball from a local manifest)
  through the three commands to Tutorial 1's first run.
- **Ubuntu 24.04 with Jazzy**: `scensim setup --autoware none`, then `scensim testbench
  check` on Tutorial 1's testbench (no live run: no Autoware for Jazzy is tested).

## Steps

1. Design note (this doc, refined), and the front end's command layout.
2. `scensim setup` and `scensim autoware-side build` in the pip front end, with tests that
   need no network, no sudo and no ROS (faked).
3. `scensim carla fetch`.
4. Docs: install.md and connect-autoware.md lead with the three commands; the manual steps
   stay as a reference section.
5. A clean-machine check: a fresh Ubuntu 22.04 container (and 24.04 for the simulation side)
   from `pip install` to Tutorial 1's first run.

## Progress

- 2026-11-19, branch `089-setup`: steps 2 and 3 in the front end (`setup.py`, `carla.py`),
  13 tests in `pypi/scensim/tests/test_setup.py` (stubs, no network, sudo or ROS). The
  CARLA 0.9.16 size and sha256 were measured on 118's copy of CARLA's tarball
  (8346095504 bytes). The URL is the one CARLA's GitHub release page links,
  `downloads.carlasim.com`; the older `carla-releases.b-cdn.net` answers 403 to 118. That
  server ended two downloads after about 1.06 GB; `carla fetch` resumes (HTTP Range), and a
  full download matched the pinned sha256.
- Step 4: install.md and connect-autoware lead with the three commands.
- Step 5, the clean-machine check on 118 (`~/nas/home/2026-11-19 089 clean machine (118)/`):
  an image of `nvidia/cuda` 12.8 with ROS Humble from apt, TensorRT 10.8 and NEWSLab's
  Autoware 1.5.0 (`autoware-full-1-5-0`, its ML models as `autoware-download-models` puts
  them), nothing of scensim's. As a user with sudo: `pip install --user scensim` (the
  branch's wheels, the release tarball built from the branch), `scensim setup` through a
  terminal, then Tutorial 1 against the host's CARLA. Six runs; each of the first five found a
  gap that a user on a fresh host would have met, and the manual steps had the first three:
  1. sudo resets HOME, so root's `rosdep install` found no index: `ROS_HOME`;
  2. sudo drops `ROS_DISTRO`, so rosdep resolved no ROS key: `--rosdistro`;
  3. a host with ROS and Autoware from apt has no git, which the import needs: installed;
  4. Ubuntu's brotli 1.0.9 with huggingface_hub 2 made `testbench check` refuse the host:
     setup installs `brotli>=1.2` for the user;
  5. Autoware without its ML models failed the first run at the pilot's start: the check
     names them and the downloader.
  Run 6 passed: one sudo prompt, a second setup with nothing to do, and Tutorial 1's
  `drive_to_goal` passed in 68.5 s.
- CI: release.yml's smoke runs `scensim setup` as a user with sudo and nothing sourced (243
  reviewed it), which the root steps with ROS sourced could not catch. Run locally it passed
  in ros:humble and ros:jazzy; it found that `sysctl -p` fails in a container, now not fatal.
- Ubuntu 24.04: the CI step above, run locally in ros:jazzy (setup without Autoware), is the
  simulation-side check; no Jazzy Autoware was run.

## Exit criteria

- On a clean Ubuntu 22.04 with ROS 2 Humble and Autoware 1.5.0, the three commands plus
  Tutorial 1 give a passing first run.
- Every manual step in install.md still works, for users who prefer it.
