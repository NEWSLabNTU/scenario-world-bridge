# Phase 070: User Experience -- Install, Pick a CARLA, Build and Run the ROS Way

A new user should go from a fresh Ubuntu host to a scenario they wrote, driven by a simple
autopilot or by Autoware, on CARLA or Isaac Sim, without reading our development notes. A
review on 2026-11-01 walked that path and found four frictions:

1. **Nothing prebuilt.** Phase 029 designed release tarballs on 2026-10-13. Only its first
   step (the doc) was done, and its names predate 051: it says `carla_scenario_bridge`
   (which no longer links CARLA), `csb_launch`, `csb_agent`, and one repo instead of
   nine. A user builds 60+ packages, the SSv2 fork and LibCarla included, after
   installing rustup and colcon's cargo plugin.
2. **The CARLA version is a build-time choice.** `swb_carla_world` and `acb_bridge` pin
   `features = ["carla-0916"]`. Another CARLA means a rebuild with `CARLA_VERSION=...`,
   and a binary built for one version refuses a server of another.
3. **The run is not the ROS way.** The README does give `colcon build` and `ros2 launch`,
   but a run is four terminals in two ROS domains, started in order and each waited for.
   The one-command path (`just scenario`) is our shell tooling, not a launch file.
4. **No starter path.** Tutorials 01-04 are CARLA with Autoware only. There is no Isaac
   starter, the simple autopilot (`carla_basic`, CARLA's BasicAgent) is a section deep in
   `agent-interface.md`, and it works on CARLA only. The map pack every run needs has no
   public download (README, Prerequisites).

**Status**: phase doc, 2026-11-02; the three decisions taken the same day. No code yet.

**Source**: user, 2026-11-02: "Check the user experience using our swb stack. ... prepare
prebuilt binary ... turn [the CARLA version] to a runtime choice but still uses
carla-rust ... The user would follow ROS convention ... colcon build and ros2 launch at
once ... starter documents to help users to get started to Carla or Isaac sim. They can
write their scenario and run with simple autopilot or Autoware."

**Takes over**: 029's steps 2-8 (029 is marked "folded into 070"); 068 step 6 (first
run on one page).

## The user's path, as it should be

```bash
# from a release (A + B)
tar xf swb-sim-<ver>-humble-amd64.tar.zst && ./swb/install-deps.sh --apply
source swb/setup.bash
# or from source (C)
git clone --recurse-submodules .../scenario-world-bridge && cd scenario-world-bridge
rosdep install --from-paths src --ignore-src -y && colcon build && source install/setup.bash

# one launch: world server, bridge, relay, the autopilot, the scenario; ends with it
ros2 launch swb_launch run.launch.xml sim:=carla autopilot:=simple \
    scenario:=$(ros2 pkg prefix swb_examples)/share/swb_examples/scenarios/starter/drive_to_goal.xosc
```

Whatever CARLA is serving on port 2000 (0.9.14, 0.9.15, 0.9.16, later 0.10) is the
version used, with no rebuild.

## A. The CARLA version at run time, still on carla-rust

**One process cannot hold two CARLA versions.** carla-sys links LibCarla's client
statically, and every version defines the same C++ symbols (`carla::client::...`), with
different layouts. So "runtime choice" means **one executable per CARLA version, picked at
launch.** Since 046 the CARLA client lives only in the world server (and `acb_bridge` on
the vehicle side), behind the world protocol, so the bridge needs no change at all.

- **Per-version executables.** `swb_carla_world` becomes a library crate with no version
  feature, plus thin binary packages `swb_carla_world_0914`, `_0915`, `_0916` (later
  `_0100`). Each one enables one carla-rust feature and calls the library's `main`. Same
  for `acb_bridge` (`acb_bridge_0916`, ...). carla-sys downloads a prebuilt
  `libcarla_client` per version, so an extra version costs one more Rust compile and link,
  not a LibCarla build.
- **Spike first:** colcon-cargo-ros2 must build each package in its own cargo invocation.
  Otherwise Cargo unifies features across the workspace and two version features fail the
  build. If it does not, the per-version packages live outside the shared Cargo workspace.
  **Spike result (2026-11-02): it does.** colcon-cargo-ros2 0.5.3 runs `cargo build
  --manifest-path <package>/Cargo.toml` per package, so features resolve per package. A
  scratch copy of swb_carla_world split into the library (features `carla-0914/0915/0916`
  forwarded to carla, none on by default) and three thin packages built in one `colcon
  build`, in one Cargo workspace and one target dir: 5 min 46 s cold, debug, three cargo
  builds in parallel. Each binary links its own `libcarla_client` (0.9.14, 0.9.15,
  0.9.16 prebuilts; ~230-260 MB each, debug). Our code compiles for 0.9.14 and 0.9.15 as is.
  Against a live 0.9.16 server, `swb_carla_world_0915` refuses (`CARLA version mismatch:
  the server is 0.9.16 but this binary was built for CARLA 0.9.15`) and retries;
  `_0916` accepts (`matches this build`). So the existing run-time check is the launcher's
  backstop, unchanged.
- **The launcher.** `swb_carla_world` (the name the launch files already use) becomes a
  small Python launcher. It asks the server its version with one msgpack-rpc `version`
  call on the RPC port (what LibCarla's `get_server_version` sends), then `exec`s the
  matching binary with the same arguments.
  - The call carries LibCarla's metadata: `rpc::Client::call` prepends a `Metadata`
    (`MSGPACK_DEFINE_ARRAY(_asynchronous_call)`), so the request is `[0, id, "version",
    [[false]]]`. With empty params the server answers `rpclib: client error C0002: Function
    'version' was called with an invalid number of arguments` (spike, 2026-11-02; the
    `[[false]]` form is from LibCarla's `rpc/Client.h` and `Metadata.h`, its live check
    pending). The
    launcher packs this one request by hand (a dozen bytes), so it needs no msgpack module.
  - `carla_version:=` (launch argument) overrides the probe.
  - No server yet: wait as the world server does today, then probe.
  - No binary for that version: refuse, naming the server's version and the installed
    ones.
  - acb's launch gets the same launcher.
- **What gets built.** By default every version carla-rust supports and our code compiles
  for (0.9.14-0.9.16 today). `colcon build --packages-skip swb_carla_world_0914 ...` drops
  some. The pin check and `just test` cover every built version.
- **The run-time check stays.** Each binary still compares `carla::CARLA_VERSION` with
  the server's on every connect. The launcher only makes the right binary the one that
  runs.

## B. Prebuilt releases (029, brought up to date)

029's design stands: tarballs of the built workspace, split by side, every CARLA version
in the same tarball, no Docker, `.deb` later if asked. What changes after 051 and with A:

- **Names:** `swb-sim-<ver>-<distro>-<arch>.tar.zst` (SSv2 fork, bridge, relay,
  `swb_launch`, `swb_examples`, `swb_interfaces`, `swb_world_protocol`, `swb_carla_world_*`,
  `swb_isaac_world`, the vendored Autoware message packages);
  `swb-vehicle-<ver>-autoware<release>-<arch>.tar.zst` (acb with `acb_bridge_*`, aib,
  `autoware_sim_vehicle`, `sim_vehicle_spec`, `sim_vehicle_ecu`, the vehicle agent); the
  `swb_agent` wheel.
- **A's launcher is 029's launcher.** 029 step 4 planned exactly this selection. A builds
  it for source builds too.
- **Isaac:** `swb_isaac_world` and aib are Python in Isaac's own venv; the tarball carries
  them and `swb_setup isaac` checks the venv.
- CI per 029. GitHub-hosted first, self-hosted on this host if a job outgrows them.
- **Publishing a release needs the user's word** (standing rule). Building and smoke-testing
  one locally does not.

## C. Build and run the ROS way

- **`colcon build` from a clean clone, nothing else**, after rosdep and the README's two
  prerequisites (rustup, `colcon-cargo-ros2`). The acceptance is a fresh clone in a clean
  shell, with no `just`, no `.env`, no `--symlink-install`. Whatever breaks there is fixed
  in the packages, not documented around.
- **`ros2 launch swb_launch run.launch.xml`**, one launch for one run:
  - `sim:=carla|isaac`;
  - `autopilot:=simple|autoware|none` (`none`: a scenario with no ego, or the user's own
    agent registered by hand);
  - `scenario:=`, `map_path:=` (default from the scenario's town and `CARLA_MAPS`);
  - optional `carla_root:=` to start CARLA itself, else it uses the one on
    `carla_port:=2000`.
  - It sets each group's `ROS_DOMAIN_ID` itself (simulation and scenario in one, the
    vehicle stack in another), waits on each part's ready signal in order, and ends when
    the scenario ends (`on_exit` shutdown). It prints where the JUnit verdict is.
  - The four-terminal form stays, as the long-lived, several-scenario way. `run.launch.xml`
    is the first-run way.
- `just` stays the developer's tool. No recipe becomes required for a user.
- **Retire the leftover alias `carla_scenario.launch.xml`** ("kept for one phase", 017),
  per the no-shims rule.

## D. Starter documents

`docs/getting-started/`, two parallel guides with the same five stages, replacing the
README's quick start as the entry point. Tutorials 02-04 stay as the next steps.

| | CARLA | Isaac Sim |
|---|---|---|
| 1. Install | release or source (B, C); CARLA from its release | release or source; Isaac Sim 6.1 pip; the asset set |
| 2. A map | the starter map (decision below) | the same town, as a USD world |
| 3. First run | `run.launch.xml autopilot:=simple` | the same, `sim:=isaac` |
| 4. Your scenario | copy `starter/drive_to_goal.xosc`, move the goal, run | the same file |
| 5. Autoware | `autopilot:=autoware`, same scenario | the same |

- **A simple autopilot on both simulators.** `carla_basic` needs CARLA's Python API, so it
  cannot drive on Isaac. New `swb_agent` adapter `simple_pilot`: route along the lanelet
  map from the scenario's goal, pure pursuit plus a speed PID, publishing Autoware's
  `/control/command/control_cmd` to the vehicle bridge (acb on CARLA, aib on Isaac, which
  both take Autoware's control interface). No Autoware install. It stops for the
  scenario's end, not for traffic: the guide says so and points to Autoware for that.
  `carla_basic` stays as the CARLA-only option that obeys lights.
- `swb_examples/scenarios/starter/`: one scenario per guide stage, run on both simulators
  by the acceptance.
- Every command in the guides runs in CI's smoke test or in the acceptance run. Nothing
  untested goes in.

## Decisions (user, 2026-11-02)

1. **The starter map: TUM's pack, mirrored** (user, 2026-11-02, replacing "our Town01 as a
   release asset"; a team wants to start from it early). TUM's LRZ links are dead (both
   `/dl/` and `/getlink/` 404 on 2026-11-02), and no public mirror was found. TUM's zip,
   byte for byte (SHA-256 `6e582242...3ed4838`, the one `download_maps.sh` always checked;
   313 MB, five towns, LGPL-3.0 with `LICENSE.md` in each town), becomes release `tum-2024-06`
   of a new repository, `NEWSLabNTU/carla-autoware-maps`, with a README crediting TUM. Our
   repairs stay in `download_maps.sh` and run after the download, so the published files are
   TUM's own. Generating a map from CARLA with acb's tools stays a later guide section.
2. **The default build compiles every supported CARLA version** (0.9.14-0.9.16 today), so
   any server works with no rebuild. `--packages-skip` drops some.
3. **`simple_pilot` is added** as described in D, on both simulators.

## Steps

1. [ ] **Docs.** This doc; 029 marked folded in; 068 step 6 pointed here; the index.
   - [x] **Early CARLA guide** (2026-11-02, for a team starting now; user: "docs first").
     `docs/getting-started/carla.md` on today's commands: stage 3 drives with `carla_basic`
     (CARLA's BasicAgent, no Autoware), stage 5 with `ego_av`; the README, the user guide §2,
     tutorial 01 and the docs index point to it. Steps 5, 6 and 8 move it to `run.launch.xml`
     and `simple_pilot`. Tutorial 01's stale `simulation.launch.xml`, empty code block and
     `acb_launch` play_launch line fixed on the way.
   - [x] **`download_maps.sh` downloads again.** The download branch had never rewritten the
     projector (only the local-copy branch did) and named no checksum failure: it now unpacks
     TUM's zip unchanged into `data/tum-carla-autoware-maps/` and goes through the same link,
     projector and repair path; `SWB_MAP_URL` for a mirror. Checked against the real zip over
     local HTTP with the CARLA release's OpenDRIVE: all five towns linked and repaired.
   - [ ] **The mirror published**: repository, release, asset. Then the guide followed
     verbatim on CARLA (stages 1-5) before the team gets the link.
2. [~] **Friction log.** Build half done, below. A fresh clone on this host in a clean shell (no `.env`, no
   `just`), following only the README. Every stumble written down with its fix owner.
   This sets C's list.
3. [~] **A, spike** (2026-11-02, results in A). Three per-version world-server packages
   in one colcon build: pass. The msgpack `version` probe against a live CARLA 0.9.16: the
   first try sent empty params and was refused; the rerun with LibCarla's metadata waits
   on a free GPU.
4. [ ] **A.** Library split, per-version packages for swb_carla_world and acb, the
   launcher with its tests (probe, override, refusal, wait). README "Other CARLA versions"
   rewritten.
5. [ ] **C.** `colcon build` clean-clone fixes from step 2; `run.launch.xml`; the alias
   retired.
6. [ ] **D, code.** `simple_pilot` and its tests; `starter/` scenarios.
7. [ ] **B.** 029 steps 2-7 under the new names: local release build, vendored packages,
   guard and `install-deps.sh`, `swb_setup`, CI dry-run on a pre-release tag.
8. [ ] **D, docs.** The two guides, the README's entry point moved to them.
9. [ ] **Acceptance.**

### Friction log (step 2, 2026-11-02)

A fresh clone of csb main in a clean shell (`env -i`, ROS + Autoware sourced, no `.env`, no
`just`), following only the README's Setup, on newslab-server139.

| # | Where | What happened | Fix |
|---|---|---|---|
| 1 | `rosdep install --from-paths src` | Aborted: `Package name "@CAR@_launch" does not follow naming conventions` (autoware_sim_vehicle's templates). rosdep's crawl honours `CATKIN_IGNORE` only, so it also walked every component's `deps/` copy. Every user's README setup failed here. | **Fixed**: `CATKIN_IGNORE` beside each `COLCON_IGNORE` in acb 776f8d6, aib 0f83aff, autoware_sim_vehicle c037376, swb_carla_world 8417221, swb_isaac_world ef7e6dc. rosdep then lists one missing key, `python3-pygame` (as the README says). |
| 2 | `colcon build` (README's flags) | **Passed**: 66 packages in 7 min 37 s (cargo registry and carla-sys prebuilt already cached on this host; a first build downloads them). 20 SSv2 packages print compiler warnings only. | -- |
| 3 | `colcon build` from the root | Also builds `third_party/cuda_ndt_matcher`, which needs CUDA. It built here; on a host without CUDA it is unchecked. | Check on a CUDA-less host; if it fails, `COLCON_IGNORE` it and keep `just build-cuda-ndt`. |
| 4 | `ros2 launch swb_launch carla_simulation.launch.xml --show-args` | Works. Several arguments say `no description given`. | Describe them (C). |

| 5 | Quick start, terminal 3 (`ros2 launch swb_launch ego_av.launch.xml`) | Died 13 s in: `Failed to convert '<?xml version="1.0" ?> ...' using yaml rules`. Humble's `ros2 launch` tries a parameter value as YAML first, and the generated sensor kit's comments (`<!-- top: lidar, ... -->`) broke the parse. No ego, so the scenario ended `errors="1"` after 17 min. Our runs use play_launch, which never tries YAML, so nothing of ours had seen it. | **Fixed**: autoware_sim_vehicle e77c39e (comments without `: `, a test refusing them). Under plain `ros2 launch` the ego stack then starts all 81 processes. The same commit stops the generator copying c037376's `CATKIN_IGNORE` into each car. |
| 6 | Quick start, terminals 1-2 | CARLA served RPC and the bridge logged `ZMQ server ready` as the README says. | -- |

| 7 | Quick start, "wait for `ego: agent up`" | The relay prints it when the agent registers, while Autoware is still starting: a scenario started then got no answer to `change_to_stop` 30 times (`errors="1"`). Waiting for IDLE is wrong too (IDLE needs the ego spawned, which the scenario does). The cue that works is ours, `scripts/ego_stack_health.py` (ADAPI up, planning publishing), which colcon does not install. | **Fixed**: `scripts/ego_stack_health.py` is now swb_launch's `ego_ready` (`ros2 run swb_launch ego_ready --timeout 600`), named in the README and the tutorial; it defaults its own DDS to the stack's. `run.launch.xml` will wait on it (C). |
| 8 | Quick start, terminal 3 without `CYCLONEDDS_URI` | Autoware never got the lidar (`No InputSource`, `only 0 fresh scan(s) in 10 s`), so align failed and no route came. The README calls the localhost config optional ("only if discovery fails"); on this host it is required, and nothing says so when it is missing. | **Fixed**: the profile moved into swb_launch (`config/`, installed). Every launch a user starts (`carla_simulation`, `isaac_simulation`, `ego_av`, `scenario`, `background_av`, `vehicle_agent`) sets `CYCLONEDDS_URI` (`dds_config:=`, default the environment's, else the installed profile) and `RMW_IMPLEMENTATION` (the environment's, else Cyclone) before it starts anything. Checked from a shell with neither: the bridge gets both under `ros2 launch` and play_launch 0.21.0; a test pins it. |
| 9 | Quick start, terminal 3 under plain `ros2 launch`, 7 and 8 applied | Localization aligned, but Autoware stayed in PLANNING: `mission_planner: waiting odometry`, control commands in 10-15 s gaps. The same terminal under `play_launch launch`, nothing else changed, reached READY and DRIVING. | Find what `ros2 launch` starts differently (step 2, next). |
| 10 | Quick start under play_launch, 7 and 8 applied | Drove, but `failures="1"` at the 180 s timeout: Autoware's control commands still came and went (gaps of 2-4 s) and the ego did not arrive. Our tested path also sources the Autoware overlay (the NDT step fix), the TensorRT 10.8 runtime and the GPU NDT, none of which the README mentions. | A/B against the tested path's extras, one at a time (step 2, next). |

| 11 | Our own path, `just scenario town01_ego_drive`, from the same clean clone with this host's `.env` | `just carla-start` exited 1 with no message. A fresh clone has no maps and nothing says to set `DATA_DIR`, so the first try handed Autoware a missing map. A stale overlay only warns. | `carla-start` says why it failed; `just scenario` refuses a missing map dir and a stale overlay (C). |
| 12 | Our own path, CARLA started by hand, maps and a fresh overlay in place | **Fails**: `the ego's ADAPI or its agent is not ready after 300 s`, nothing publishes `/api/operation_mode/state`, the agent stays UNAVAILABLE. No CARLA scenario had run on main since 2026-10-24. | Root cause: 14 (13 first blamed play_launch, wrongly). |
| 13 | The ego stack alone, no CARLA, `--container-mode isolated` (`just ego-av`'s default) against play_launch's default mode | First measured as a play_launch bug (isolated: no ADAPI in 120 s; default: 5 s). **It was not.** That test had no `RMW_IMPLEMENTATION`, so ~90 isolated processes ran Fast DDS (14). Rerun on Cyclone in domain 7: isolated, ADAPI in 5 s. | **No change**: isolated stays the default (a day's `observable` default reverted). 12 was 14. |
| 14 | Our own path from a shell without direnv (first with `observable` containers, 13) | Bring-up ready, then `Service /api/operation_mode/change_to_stop is not ready after 119 seconds`. The scenario stack ran on Fast DDS (its dds_report: `CYCLONEDDS_URI ... NOT READ`), the relay on Cyclone: they never meet. Only the repo's `.envrc` set `RMW_IMPLEMENTATION`; `run` and `demo-tour` defaulted it, seven other recipes did not. | **Fixed**: every recipe that sets `CYCLONEDDS_URI` defaults `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (a set value wins; `EGO_RMW_IMPLEMENTATION` still overrides the ego's). Then `just scenario town01_ego_drive` from the clean clone **passed** (`failures="0" errors="0"`), everything reaped after. |
| 15 | 13's rerun in domain 77 | Every node died at once: `rmw_create_node: failed to create domain`, the launch shut down. The localhost Cyclone config sets `DomainGain 1000`, so domain 77's ports start at 84410, past 65535; the cause is in each node's `err` only (`resulting port numbers (84410, 84411) are out of range`). Domains above 57 cannot work with it, and nothing says so up front. | **Fixed in the recipes** (`swb_check_domains`, every recipe) and named in the profile's comment and the user guide. The launches cannot check it in XML; `run.launch.xml` will (C). |
| 16 | `download_maps.sh` from the zip with the CARLA release's OpenDRIVE (`SWB_XODR_DIR`) | The towns differ from the ones every run on this host used: those have **no `speed_limit` tags** (Town01 110, Town02 78, Town03 428, Town05 426, Town10 145) and lack the walkway-under-road retags (Town03 13, Town05 12), so this host's runs drove at lanelet2's 50 km/h default. The speed-limit and walkway repairs came on 2026-10-27 (033f7a2, 4b8fed7), after this host's maps were installed (2026-10-21), and were never re-run here. A user gets the fully repaired maps. | The guide's acceptance runs on the downloaded maps. Re-run `download_maps.sh` on this host before comparing a run with the user's. |

The README path is not the tested path: 7-10 are the gap. **And 8-10 ran on Fast DDS**:
Humble's and Autoware's `setup.bash` leave `RMW_IMPLEMENTATION` unset, the quick start never
set it, so the `CYCLONEDDS_URI` those runs exported was never read (found with 14). 9 and 10
are re-measured with the launches setting the transport themselves (8's fix). Seven live runs on 2026-11-02,
each under the quiet-host gate; logs in `/var/tmp/aeon/070/`.

## Acceptance

- From a clean clone in a clean shell: `rosdep install`, `colcon build`, `source`, then
  `ros2 launch swb_launch run.launch.xml` passes `starter/drive_to_goal.xosc`:
  - on CARLA 0.9.16 with `autopilot:=simple` and `autopilot:=autoware`;
  - on Isaac with both.
- The same build, unchanged, against a CARLA 0.9.15 server: the launcher picks the 0.9.15
  binary and the simple run passes. A server version with no binary is refused, both
  versions named.
- From the release tarballs on a host with no Rust and no colcon build: the same runs pass
  (029's acceptance under the new names).
- Each getting-started guide followed verbatim by someone who did not write it (the user,
  or a fresh subagent with only the guide) reaches stage 5.
- The live runs keep the standing gate: load < 12, GPU < 10 %, >= 2.2 GB free for 5 min,
  never CARLA and Isaac together.

## Not in this phase

Jazzy (029's reasoning stands). `.deb`/apt. Docker. CARLA 0.10 itself (068 step 8; A
makes it one more package once it builds). A GUI launcher.
