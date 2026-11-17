# Phase 080: The Rename to scensim

swb becomes **scensim** (SCENario SIMulation): a framework that runs driving scenarios in
simulation, with OpenSCENARIO scenarios, the user's autopilot and CARLA or Isaac Sim. The
name changes before [081](081-testbench-packages.md) and [082](082-distribution-and-pypi-release.md)
create any new package, so the colcon plugin, the testbench build type and the wheels are
born under it.

**Status**: planned, 2026-11-17. Nothing is renamed yet. The order of the three phases is
080, then 081 and 082 (082's local measurements may run before, as they publish nothing).

**Source**: user, 2026-11-17: "I'd rename the project too. It's no longer a bridge. It
becomes a framework. I'd like to find a memorizable name." Then, after candidates were
checked: a long meaningful name shortened, "emphasis the scenario plus simulation
capabilities"; "scensim" chosen; "and mainly focus on driving".

## The name

- `scensim`, lowercase everywhere (PyPI, the command, the ROS package prefix, the
  repository). Checked free on PyPI on 2026-11-17: `scensim`, `scensim-ros-humble`,
  `scensim-ros-jazzy`, `colcon-scensim`. `swb` is taken on PyPI (an unrelated package).
- Rejected on the way: `drivebench` (a dataset), `skena` (a published paper), `simario` (an
  R framework for scenario microsimulation), `scenrun` (too close to CARLA's
  scenario_runner), several short words taken on PyPI.
- **Focus: driving.** Cars, their autopilots and the traffic around them; robots and
  pedestrians are other agents the same framework can drive. The docs' one-line description,
  for 243 to polish: "scensim runs driving scenarios in simulation: OpenSCENARIO scenarios,
  your autopilot, CARLA or Isaac Sim."

## What is renamed, and what is not

The rule: names that stand for the framework take `scensim`; names that describe one
component's role stay, because the component is still that thing.

| Kind | Today | After |
|---|---|---|
| This repository | `NEWSLabNTU/scenario-world-bridge` | `NEWSLabNTU/scensim` |
| The world protocol repository | `swb_world_protocol` | `scensim_world_protocol` |
| The world servers' repositories | `swb_carla_world`, `swb_isaac_world` | `scensim_carla_world`, `scensim_isaac_world` |
| ROS packages of the framework | `swb` (the run tooling), `swb_launch`, `swb_agent`, `swb_examples`, `swb_interfaces`, `swb_world_*`, `swb_carla_world*`, `swb_isaac_world` | `scensim`, `scensim_launch`, `scensim_agent`, `scensim_examples`, `scensim_interfaces`, `scensim_world_*`, `scensim_carla_world*`, `scensim_isaac_world` |
| The command | `ros2 run swb run`, `swb ...` | `ros2 run scensim run`, `scensim ...` |
| Files | `swb.yaml`, `swb.lock`, `swb.launch.xml`, `swb_world.proto` | 081 renames `swb.yaml` to `testbench.yaml`; `scensim.lock`, `scensim.launch.xml`, `scensim_world.proto` |
| Host settings | `SWB_*` (about 40, e.g. `SWB_LOG_ROOT`, `SWB_WEB_PORT`, `SWB_MAPS`) | `SCENSIM_*` |
| Host paths | `~/.config/swb`, `~/.cache/swb`, `~/.local/state/swb`, a project's `.swb/`, `/tmp/swb-locks`, `/tmp/swb-seats` | the same with `scensim` |
| ROS services and topics | `/swb/wheels`, `swb_interfaces`' services | `/scensim/wheels`, `scensim_interfaces` |

**Unchanged** (each names a role, not the framework): `scenario_world_bridge` (the bridge
between SSv2 and a world server), `scenario_agent_relay`, `czp`, acb
(`autoware_carla_bridge`), aib (`autoware_isaac_bridge`), `autoware_sim_vehicle`,
`sim_vehicle_spec`, `sim_vehicle_ecu`, `cuda_ndt_matcher`, the SSv2 fork, the Hugging Face
datasets and their names (none carries `swb`; their cards' text is updated).

## Rules

- **No shims.** Every old name is refused by name, never aliased: a `SWB_*` variable makes a
  command refuse to start, naming its `SCENSIM_*` successor (the `SWB_RETIRED_ENV`
  mechanism, `scripts/just_lib.bash`, becomes the list of renamed variables); `swb.yaml` is
  refused naming `testbench.yaml` (081); an old ROS package name is not provided.
- **Host state moves, it is not shared.** On its first run scensim finds a `~/.config/swb`
  or `~/.cache/swb` and prints the `mv` lines to move them; it never reads the old paths.
- **Old runs stay readable.** `runs show` and the slicer read a `result.json` written under
  the old names (its keys are not renamed by this phase; provenance names `swb` commits
  before the rename).
- **Linear history, pins after submodules**, as always: each renamed submodule is pushed
  first, then the pin here.

## Steps

1. **Docs** (133): this doc, 081 and 082; the index; the old 080 text folded into them.
2. **The user renames the four GitHub repositories** (`scenario-world-bridge`,
   `swb_world_protocol`, `swb_carla_world`, `swb_isaac_world`). GitHub redirects the old
   URLs, so clones and `.gitmodules` keep working until step 3 moves them. Outward-facing
   and the user's alone.
3. **`.gitmodules` and every component's `deps/` pins** point at the new URLs; check_pins
   passes.
4. **The world protocol** (`scensim_world_protocol`): packages, crates, the Python shell,
   the proto file and its package name. Both world servers and the bridge follow in the
   same lockstep (the protocol's names are compiled in).
5. **The world servers** (`scensim_carla_world`, `scensim_isaac_world`).
6. **This repository**: the ROS packages, the Python package and command, the launch files,
   the host settings and paths, the justfile, the tests, `SWB_RETIRED_ENV` turned into the
   renamed list.
7. **The other components' references** (acb, aib, `autoware_sim_vehicle`: launch
   arguments, environment names and docs that say swb).
8. **Docs and agent files**: the user docs, the README, the design docs' current text (the
   roadmap's history keeps the old names), the dataset cards. CLAUDE.md needs the user's OK,
   as always.
9. **Exit runs.**

Steps 4 to 7 are one coordinated change across repositories: no pin on main points at a
mix of old and new names. They are prepared on branches and landed together, in pin order.

## Exit criteria

- A fresh clone of `NEWSLabNTU/scensim` with `--recursive`, built from a clean shell as
  CLAUDE.md says, passes `just test` (with check_pins) and check_docs.
- Tutorials 1 and 5 pass as written under the new names (one CARLA run each, as 077's
  release check did); one Isaac basic run passes on a lab host.
- Every `SWB_*` variable and `swb.yaml` is refused with its successor's name; no user doc
  and no installed file names `swb` outside the roadmap's history and old runs.
- The T3 references recorded at 36e4da4 still compare (their keys are unchanged).

## Non-goals

- Renaming the components that describe a role (above).
- Renaming the keys inside `result.json`, the T3 references or the Hugging Face dataset
  names.
- Any change of behaviour: the rename is names only.
