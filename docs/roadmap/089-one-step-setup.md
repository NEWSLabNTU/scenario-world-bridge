# 089: One-Step Setup

**Status**: opened 2026-11-19 (user). Target: 0.2.1.

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

## Steps

1. Design note (this doc, refined), and the front end's command layout.
2. `scensim setup` and `scensim autoware-side build` in the pip front end, with tests that
   need no network, no sudo and no ROS (faked).
3. `scensim carla fetch`.
4. Docs: install.md and connect-autoware.md lead with the three commands; the manual steps
   stay as a reference section.
5. A clean-machine check: a fresh Ubuntu 22.04 container (and 24.04 for the simulation side)
   from `pip install` to Tutorial 1's first run.

## Exit criteria

- On a clean Ubuntu 22.04 with ROS 2 Humble and Autoware 1.5.0, the three commands plus
  Tutorial 1 give a passing first run.
- Every manual step in install.md still works, for users who prefer it.
