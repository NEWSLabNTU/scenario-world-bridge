# 081 and 082 exit runs: the user's journey, step by step

A check list for the exit runs of [081](../081-testbench-packages.md) and
[082](../082-distribution-and-pypi-release.md): every command a user types, from a clean
host to a passed run, with what it must print, its exit code, and which steps need root.
Written by 243 on 2026-11-17 from [the install draft](install-draft.md),
[Tutorial 1's draft](tutorial-1-testbench-draft.md) and the answers in
[questions.md](questions.md). Expected output is a pattern (`...` elides); an exit run
records the actual output beside each row, and any difference is a finding, either in the
program or in the docs.

Columns: **Root** is `yes` for a step that needs sudo, which a user on a shared host asks
an administrator for. **Exit** is the command's exit code.

## 0. The host before the run

Recorded at the start of the run, not checked against expectations:

- Ubuntu 22.04, ROS 2 Humble from apt (`/opt/ros/humble`), `python3 --version` 3.10.
- NEWSLab's Autoware 1.5.0 packages (the revision of 2026-11-17 or later, which passes the
  link check), `/opt/autoware/1.5.0`.
- CARLA 0.9.16 unpacked; `~/.config/scensim/host.yaml` absent.
- No scensim: `command -v scensim` prints nothing, `~/.local/share/scensim` absent,
  `pip list --user | grep scensim` empty.
- The GPU's memory in use and the load (`nvidia-smi`, `uptime`), for the report.

## 1. Install (082)

| # | Root | Command | Expected | Exit |
|---|---|---|---|---|
| 1.1 | no | `python3 -m pip install --user scensim` | `Successfully installed scensim-<ver> scensim-ros-humble-<ver>` | 0 |
| 1.2 | no | `export PATH=$HOME/.local/bin:$PATH` | (nothing) | 0 |
| 1.3 | no | `scensim --version` | `scensim <ver>` and `not installed for ROS humble: run scensim install`; nothing downloaded (`~/.local/share/scensim` still absent) | 0 |
| 1.4 | no | `scensim install` | `scensim <ver> for ROS 2 humble (x86_64): downloaded <~65> MB, sha256 ok`, `installed in ~/.local/share/scensim/<ver>-humble (current)`, then the root block and the `source` line | 0 |
| 1.5 | yes | `sudo rosdep init` | only where never done; else `default sources list file already exists` | 0 or 1 |
| 1.6 | no | `rosdep update` | `updated cache in ...` | 0 |
| 1.7 | yes | the `sudo rosdep install --from-paths ~/.local/share/scensim/current/share --ignore-src -y` line 1.4 printed | apt installs about 47 packages on a bare Humble; `#All required rosdeps installed successfully` | 0 |
| 1.8 | no | write `~/scensim.bash`: ROS, `/opt/autoware/1.5.0/setup.bash`, `~/.local/share/scensim/current/setup.bash`, in that order | (a file) | |
| 1.9 | no | write `~/.config/scensim/host.yaml` with `carla: {root: <CARLA dir>}` | (a file) | |
| 1.10 | no | `source ~/scensim.bash && scensim --version` | `scensim <ver> (ROS humble, ~/.local/share/scensim/<ver>-humble)` | 0 |
| 1.11 | no | `scensim seat show` | the seats, `your seat: auto`, and `launcher: auto -> ros2 (play_launch not on PATH)` | 0 |

**Root count**: steps 1.5 (where needed) and 1.7, nothing else. A step that asks for a
password beyond these is a finding.

## 2. A workspace and a testbench (081)

| # | Root | Command | Expected | Exit |
|---|---|---|---|---|
| 2.1 | no | `mkdir -p ~/my_ws/src && cd ~/my_ws/src` | | 0 |
| 2.2 | no | `scensim testbench new my-first-drive` | refused before writing anything: `my-first-drive is not a valid ROS package name (use my_first_drive)`; no directory created | non-zero |
| 2.3 | no | `scensim testbench new my_first_drive` | `wrote src/my_first_drive (carla-first-drive)` | 0 |
| 2.4 | no | `cat my_first_drive/package.xml` | `<build_type>scensim_testbench</build_type>`; maintainer from `git config` (or a placeholder without it); `<license>TODO: License declaration</license>` | 0 |
| 2.5 | no | `cat my_first_drive/testbench.yaml` | `simulator: carla`, the Town01 asset, the `ego` Autoware pilot, `default: first` | 0 |
| 2.6 | no | `cd ~/my_ws && colcon build --symlink-install` | `Summary: 1 package finished`; a warning that the testbench has no lock yet (no fetch done) | 0 |
| 2.7 | no | `source install/setup.bash && scensim testbench list` | `my_first_drive` with its suites (`first`) | 0 |

## 3. Assets and the first run

| # | Root | Command | Expected | Exit |
|---|---|---|---|---|
| 3.1 | no | `scensim run my_first_drive` (before fetching) | refused: the assets are not fetched, naming `scensim assets fetch my_first_drive` | 3 |
| 3.2 | no | `scensim assets fetch my_first_drive` | `carla-town-autoware-maps: <commit>: 9 file(s), 9 downloaded`, `src/my_first_drive/scensim.lock written` (the source directory, through the symlink) | 0 |
| 3.3 | no | `scensim testbench check my_first_drive` | `testbench: ok` | 0 |
| 3.4 | no | `scensim run my_first_drive` | `stack    my_first_drive on seat 0: started (CARLA, Town01); left up ...`, the map line, `passed   <run id>  .../my_first_drive/runs/...`, `summary: .../suites/first/summary.csv` | 0 |
| 3.5 | no | `scensim runs list my_first_drive` | one row, `passed`, with the `WORKSPACE` column `~/my_ws` | 0 |
| 3.6 | no | `scensim runs show <run id>` | the verdict, duration and timeline | 0 |
| 3.7 | no | `scensim stack --list` | the stack of `my_first_drive` on seat 0 | 0 |

## 4. Edit without a rebuild, and run again

| # | Root | Command | Expected | Exit |
|---|---|---|---|---|
| 4.1 | no | edit `src/my_first_drive/scenarios/drive_to_goal.xosc` (halve the goal's distance) | | |
| 4.2 | no | `scensim run my_first_drive/scenarios/drive_to_goal.xosc` | no stack line (it is up); `passed`; the run's `resolved.yaml` holds the new goal | 0 |
| 4.3 | no | `scensim run my_first_drive --stop-stack` | `passed`, then the stack stopped; `scensim stack --list` empty | 0 |

## 5. Refusals a user can meet

Each must name its fix in one line.

| # | Setup | Command | Expected | Exit |
|---|---|---|---|---|
| 5.1 | a terminal without `~/scensim.bash` sourced | `colcon build` of the workspace, then `scensim run my_first_drive` | colcon does not fail: it warns `No task extension to 'build' a 'ros.scensim_testbench' package` and exits 0 with nothing installed; the run then refuses the unbuilt testbench, naming `source ~/scensim.bash, then colcon build`; the install page's troubleshooting quotes colcon's warning | 0, then non-zero |
| 5.2 | a terminal with Jazzy's setup sourced | any `scensim` command | `scensim is installed for humble, but ROS_DISTRO is jazzy (python <path>)` | non-zero |
| 5.3 | a testbench with `swb.yaml` instead of `testbench.yaml` | `colcon build` | the package fails: `swb.yaml is renamed testbench.yaml` | non-zero |
| 5.4 | a testbench naming `pkg://other/...` without `<exec_depend>other</exec_depend>` | `colcon build` | the package fails, naming the missing `<exec_depend>` | non-zero |
| 5.5 | a scenario path with `../` out of the testbench | `colcon build` | the package fails, naming the `pkg://` form | non-zero |
| 5.6 | `scensim install` of a newer version, the workspace not rebuilt | `scensim run my_first_drive` | refused: built over scensim `<old>`, naming `colcon build` | non-zero |
| 5.7 | a workspace built without `--symlink-install` | `scensim assets fetch my_first_drive` | refused, naming `--source src/my_first_drive` | non-zero |
| 5.8 | `testbench.yaml` without `default:` and two suites | `scensim run <tb>` | refused, listing the suites | non-zero |
| 5.9 | `SWB_LOG_ROOT` exported | any `scensim` command | refused: `SWB_LOG_ROOT is renamed SCENSIM_LOG_ROOT` (080) | non-zero |
| 5.10 | the download cut part way (network dropped during 1.4) | `scensim install` again | the partial file is not kept; `sha256 ok` on the retry | 0 |

## 6. The source build (082 exit criterion 2)

Two hosts: A with Autoware (both sides), B a bare `ros:humble` without Autoware (the
simulation side). Neither passes `--skip-keys`.

| # | Root | Command | Expected | Exit |
|---|---|---|---|---|
| 6.1 | yes | `sudo apt install python3-pip python3-vcstool` | installed (absent from a bare `ros:humble`; rosdep's pip keys need pip) | 0 |
| 6.2 | no | `git clone https://github.com/NEWSLabNTU/scensim.git ~/scensim && cd ~/scensim` | the clone | 0 |
| 6.3A | no | `git submodule update --init --recursive` | every submodule | 0 |
| 6.4A | yes | ROS and Autoware sourced: `rosdep install --from-paths src --ignore-src -y` | every key resolves; `#All required rosdeps installed successfully` | 0 |
| 6.3B | no | `git submodule update --init --recursive` with the simulation-side submodules the page lists, then `vcs import src < sim.repos` | six message repositories imported | 0 |
| 6.3B-Isaac | no | for Isaac Sim also: `git submodule update --init --recursive src/autoware_isaac_bridge` and `touch src/autoware_isaac_bridge/src/aib_launch/COLCON_IGNORE` | `aib` built, `aib_launch` skipped | 0 |
| 6.4B | yes | ROS sourced: `rosdep install --from-paths src --ignore-src -y` | every key resolves (about 28 apt packages and one pip, yamale, on a bare `ros:humble`) | 0 |
| 6.5 | no | `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cargo-args --release` | `Summary: <n> packages finished`, none failed | 0 |
| 6.6 | no | `~/scensim.bash` with `~/scensim/install/setup.bash` in place of the installed one; then sections 2 to 4 again in a new workspace (host A) | the same results | |

## 7. Ubuntu 24.04, before a Jazzy release

| # | Root | Command | Expected | Exit |
|---|---|---|---|---|
| 7.1 | no | `python3 -m pip install --user scensim` | `error: externally-managed-environment` (PEP 668) | non-zero |
| 7.2 | no | `python3 -m pip install --user --break-system-packages scensim` | the front end alone (no `scensim-ros-jazzy`) | 0 |
| 7.3 | no | `scensim install` | `no prebuilt scensim for ROS jazzy yet; build from source: <link>` | non-zero |
| 7.4 | no | the venv form (`python3 -m venv --system-site-packages`, `pip install scensim`) | the same as 7.2 and 7.3 | |

## What the run reports

For each row: the command as typed, the output (the first and last lines where long), the
exit code, the wall time, and pass or finding. Totals: commands typed, root steps, time from
1.1 to the first `passed`, and the disk used (`~/.local/share/scensim`, the cache, the
workspace). A finding names the row and whether the program or the page is wrong.
