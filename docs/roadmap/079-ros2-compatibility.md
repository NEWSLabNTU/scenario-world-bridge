# Phase 079: ROS 2 Compatibility, Without Lab Tooling

swb runs on a stock ROS 2 Humble install: ROS 2 is the only requirement, and the simulator,
its version and the autopilot are the user's choices. A user who has ROS 2 Humble, an Autoware
workspace (built or binary; tested with 1.5.0) and CARLA 0.9.16 can install swb, start the stack with `ros2 launch swb_launch swb.launch.xml` and run scenarios
with `ros2 run swb run`. play_launch stays the recommended launcher: when it is installed swb
uses it, and nothing requires it.

**Status**: approved, 2026-11-16 (the roadmap; the schema and the docs ruled in steps 1 and
8). Nothing is built yet.

**Source**: user, 2026-11-16: "Our stack should be ROS compatible and work without my
play_launch." Then: ROS 2 compatibility is its own phase, a campaign; with the warning that a
SIGKILL to `ros2 launch` alone leaves its children running.

**Follows**: 076 (the project workflow: `swb.launch.xml`, `ros2 run swb run`, the tutorials)
and 077 (the migration and test matrix, whose T1-T5 runs are the exit criteria here). It
does not block the current user-experience release unless the user says so.

## Goal

On a host where play_launch is not installed, a fresh clone installs by
[Install](../getting-started/install.md), tutorials 1 to 5 pass as written, and a run stopped
part way leaves no process of its stack behind. On a host with play_launch, the same runs
behave as they do today.

## Audit: what the user path assumes today (2026-11-16, main at baeba03)

| # | Assumption | Where | Outcome |
|---|---|---|---|
| A1 | `play_launch launch` starts the stack, each nested pilot, the world server and SSv2's scenario | swb/stack.py, swb/run.py, swb_launch/launch/stack.launch.py | step 2 |
| A2 | `project check` refuses a host without play_launch 0.30 or later | swb/tools.py (`PLAY_LAUNCH_FIX`) | step 1 |
| A3 | play_launch-only flags: `--web-addr`, `--log-dir`, `--disable-web-ui`, `--disable-diagnostics`, and for Autoware pilots `--container-mode isolated --composable-respawn on-crash` | stack.py, stack.launch.py | step 2 (flags), step 5 (the container behaviour) |
| A4 | Stopping a stack, a pilot or a scenario signals one launcher process; its children follow because play_launch reaps them | stack.py `stop_processes`, run.py, the keeper | step 3 |
| A5 | Logs are play_launch's layout, `node/<name>/{out,err}` | swb/timeline.py, swb/slices, `runs why`, the pilot crash counter | step 4 |
| A6 | The DDS profile is Cyclone's (`rmw_cyclonedds_cpp`), which no package.xml declares, so rosdep never installs it | swb_launch config, seat env | step 6 |
| A7 | Python tools installed by pip only: colcon-cargo-ros2, play_launch, yamale, the CARLA client; xmlschema, shapely, networkx and pygame have apt packages | install.md, carla.md | step 7 |
| A8 | The docs show `play_launch launch` as the way to start a stack | swb.launch.xml's header, T1, T3, T4, install.md | step 8 |
| A9 | `scenario.launch.xml` works around play_launch's serialisation of empty-string parameters | swb_launch/launch/scenario.launch.xml | step 5 |
| A10 | Autoware's fix overlay and TensorRT 10.8 | on the user path for Autoware 1.5.0 binaries: the pilot warns "Autoware overlay not built" without it, and stock pose_instability_detector can abort (issue 027); built by `ros2 run swb autoware overlay` and `ros2 run swb autoware tensorrt` (077 G62), no `just` | optional, for Autoware 1.5.0 binaries; launcher-independent, so no step |
| A11 | `just` | developer docs and recipes only; no user page uses it (checked: getting-started, tutorials, how-to, reference, README) | non-goal |
| A12 | Lab host paths or names in defaults | none on the user path (checked swb, swb_launch); one dataset name in the asset catalogue is a public Hugging Face name | none |

## Scope

In: the user path (`swb.launch.xml`, `ros2 run swb ...`, the project workflow, tutorials 1
to 5 on CARLA), on both launchers.

Non-goals:

- The developer recipes (`just run`, `just ego-av`, `just bg-av`, `just scenario-direct`):
  they keep play_launch.
- Isaac Sim (tutorials 6 and 7): the same launcher code serves it, but its exit criteria are
  a later phase's.
- Parity of play_launch's extras: the web UI, its diagnostics and its I/O monitor have no
  `ros2 launch` counterpart, and the docs say so.
- Autoware's overlay and TensorRT 10.8 (A10), and `just` (A11).

## Steps

1. **The launcher setting** (243, 2026-11-16). The host setting `launcher:` takes `auto`,
   `play_launch` or `ros2`; the default is `auto`, as with `seat: auto`: play_launch when it is
   on `PATH` at 0.30 or later, else ros2. A stack overrides it with the launch argument
   `launcher:=` (arguments beat host settings, as for seat and gpu). `seat show` and
   `project check` print the choice ("launcher: auto -> play_launch 0.30.2", or "-> ros2").
   `project check` no longer refuses a host without play_launch; it refuses only an explicit
   `play_launch` that is missing or older than 0.30, naming the version found and the pip
   line. Documented in configure.md's host settings table and in commands.md.
2. **One argv builder.** One function builds every launcher command (the stack, a nested
   pilot, the world server's own launch, SSv2's scenario launch) for both launchers.
   play_launch-only flags (A3) go only to play_launch; the log directory goes to `ros2 launch`
   as `ROS_LOG_DIR` (step 4). The stack's `swb.launch.xml` works under both (it already
   reads no play_launch option).
3. **Process-group stop.** Every launcher starts in its own process group (`setsid`, as
   today), and is stopped through the group: SIGINT, then SIGTERM, then SIGKILL, each to
   `-pgid`, never to the launcher's pid alone (the user's warning; it is also how a terminal
   job is stopped). The nodes' `PR_SET_PDEATHSIG` stays the backstop. Test: a stop part way
   through a run, then no survivor of any group the run started (`ps`, by process group and
   by `ppid == 1`).
4. **Logs under ros2 launch.** `ROS_LOG_DIR` set to the run's or the stack's log directory,
   and nodes started with `output='both'` where they are `screen` today, so each node writes
   `<dir>/<name>-<pid>-stdout.log` and `-stderr.log`. Only under ros2: play_launch already keeps
   a log per node, and `both` there would write each line twice. The launch files take a
   launch argument `launcher` (which the argv builder of step 2 passes, and swb.launch.xml
   hands on to every nested launch) and choose `output` from it. timeline.py, the slicer, `runs why` and
   the pilot crash counter read both layouts. Tests: the same timeline from each layout.
5. **What `ros2 launch` does natively, checked rather than rebuilt.** The rule: behaviour may
   differ between the launchers only where the docs say so, and never so that a run gets a
   different verdict. Respawn
   (`respawn="true"`) and `on_exit` Shutdown both exist in launch_ros and are kept as they
   are. Each of these gets a decision in this step, by that rule: Autoware's composable
   containers without `--container-mode isolated` (a crashing component then takes its whole
   container down, and the pilot's episode ends instead of the component restarting: the run
   must then end `error`, with the crash named by `runs why`, never `passed` and never a
   misleading `failed`); the empty-string
   parameter workaround (A9), which may be play_launch-only; no web UI for nested launches.
6. **The DDS profile.** swb_launch declares `rmw_cyclonedds_cpp` (an `exec_depend`), so
   rosdep installs it, and `project check` names it when it is missing.
7. **Python tools.** The install page installs from apt what apt has (xmlschema, shapely,
   networkx, pygame), and from pip only what has no apt package (colcon-cargo-ros2, yamale,
   the CARLA client), each said once. apt needs sudo, so the no-sudo path stays beside it
   (`python3 -m pip install --user ...`) for a host with no administrator. play_launch moves to
   an optional section.
8. **Docs** (243, 2026-11-16), after step 9's live runs pass on both launchers. The command
   every reader can run comes first: the tutorials start a stack with `ros2 launch swb_launch
   swb.launch.xml`. Tutorial 1's step 3 adds one short paragraph naming play_launch as the
   recommended alternative (a log per node, the web interface, supervision), with its command
   and a link to the reference; the later tutorials use ros2 launch only, with a one-line
   pointer where the web UI is mentioned. A reference section "Launchers" (in reference/)
   says what each gives: logs, the web UI, respawn, the stop (the process-group stop), and
   where `runs why` and the timeline read from. In install.md, play_launch moves to an
   optional step after the build, "Recommended: play_launch", with its pip line. Sent to 243
   as a patch.
9. **The exit runs** (below), on both launchers, on a quiet GPU.
10. **A regression guard in `just test`**, so that the ros2 path does not rot between
    releases with no simulator to hand: the argv builder's tests for both launchers, and a
    `ros2 launch swb_launch swb.launch.xml --show-args` (and `--print` of a project's
    stack) parse check, also as a step of [Run swb in CI](../how-to/run-in-ci.md).

Order: steps 6 and 7 are small and independent, so they go alongside step 1; the rest in
order. Each step is pushed after review, with its tests.

## Exit criteria

- On a host where play_launch is not on `PATH`: a fresh clone installs by Install; tutorials
  1 to 5 pass as written; `project check` reports `launcher: ros2`.
- A run stopped part way (SIGINT to `ros2 run swb run`, and the stack stopped as the
  tutorials say) leaves no process of the groups it started, and no orphan with `ppid == 1`.
- The same runs with play_launch (`launcher: play_launch`): T1's drive, a two-pilot run
  (T4 or two_av) and the stop part way, with the same verdicts as before and no orphans.
- Under ros2, a component crash that ends the pilot's episode ends the run `error`, and
  `runs why` names the crash (step 5's rule).
- check_docs, the tests (with step 10's guard) and check_pins pass.
