# UX walkthrough of roadmap 076 (proposed design), by persona

Subject: `docs/roadmap/076-assets-interface-and-docs-by-reader.md` (section A, the interface;
section B, the docs), read against today's user docs (README, getting-started, build your own
stack, domains and DDS, tutorials 1 to 7, user guide, agent interface, scenario authoring) and
`src/swb_launch/launch/stack.launch.py`. Each walkthrough is what a reader would type if 076
shipped as written. "F" marks a friction. Date: 2026-11-14.

## 0. Facts checked for this review

- **The working directory under a launch.** Tested with play_launch 0.20.0 and `ros2 launch`
  (Humble) on a small launch file run from a project directory:
  - A top-level process's cwd is the directory the user started the launch from, under both.
    `os.getcwd()` in a Python launch file also returns it.
  - `$(env PWD)` gives the shell's `PWD` variable, not the real cwd. With
    `env PWD=/stale play_launch launch ...` the argument came out as `/stale` while the
    process ran in the project directory. `PWD` is stale or unset under `env -i`, systemd,
    a Python `subprocess(cwd=...)` and some CI runners. `$(env PWD)` with no default also
    fails to parse when `PWD` is unset.
  - play_launch writes its own `play_log/<time>/` into the cwd **before** it reads the launch
    file. A project would therefore get `play_log/` next to `runs/`, and the entry launch
    cannot move it. Only `--log-dir` on the play_launch command line can.
  - play_launch's web UI is on by default at `127.0.0.1:8080`, so two users who both type the
    design's one-line command on one host both ask for port 8080. This too is a play_launch
    flag that `swb.yaml` cannot set.
  - play_launch's Rust parser rejects some quoting that `ros2 launch` takes
    (`unbalanced quotes in command line`), and it reads `$(...)` inside a `cmd` string as a
    substitution. Any nested argument value such as `pilots:="ego=carla_basic bg_av_1=none"`
    must parse the same way under both front ends.
- **What the stack does today** (`stack.launch.py`): CARLA itself runs apart (`CarlaUE4.sh`).
  The world server runs in the **ego** domain. `acb_bridge` (CARLA's sensors and vehicle
  interface) is part of the default *Autoware* ego launch (`acb_launch
  carla_simulator.launch.xml`), not a separate piece. A scenario started with the stack
  begins at once, before Autoware is ready. That is why the docs raise
  `initialize_duration` to 600 s.
- **What the scenario owns today**: the map (`RoadNetwork/LogicFile`), the entities, each
  entity's car (`CatalogReference` plus `CatalogLocations`) and who drives it (the controller
  name: `Autoware` or `isEgo` for the ego, `agent`, `simulator_autopilot`, or none for SSv2).
  On Isaac the ground (`surfaces.yaml`, `road_detail.yaml`) lives in the map directory and
  saved ruts are written there. The Traffic Manager seed lives in `swb_carla_world.yaml`.

## 1. Newcomer: a student who knows ROS, one GPU workstation, wants Autoware to pass a scenario on CARLA

**Walkthrough**

1. Install ROS 2 Humble, CARLA 0.9.16, Autoware 1.5.0, Rust, `colcon-cargo-ros2` and
   play_launch, then clone the repository and run `colcon build`. This is unchanged by 076
   and still takes about 30 minutes plus downloads.
2. Write the environment file. 076 drops `CARLA_MAPS`, but the student still needs:
   ```bash
   source /opt/autoware/1.5.0/setup.bash
   source ~/scenario-world-bridge/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file://$(ros2 pkg prefix --share swb_launch)/config/cyclonedds-localhost.xml
   ```
   **F1.1** These DDS lines are still needed for every `ros2 topic` and RViz terminal. A
   newcomer meets Cyclone, `CYCLONEDDS_URI` and the "export after Autoware" rule before the
   first drive. 076 does not say whether the entry launch changes this.
3. Start CARLA: `~/CARLA_0.9.16/CarlaUE4.sh -RenderOffScreen -nosound -carla-rpc-port=2000`,
   then wait 1 to 3 minutes. **F1.2** A6 says the entry launch "starts the simulator side".
   Today that means the world server, not CARLA. The acceptance test ("from a clean clone
   ... reach a passed scenario") reads as if CARLA starts too. If it does, where does the
   launch find CARLA (a host setting nobody has named yet), and who stops it? If it does
   not, step 3 is a manual step that 076's "no manual steps" promise leaves out.
4. `ros2 run swb_launch project new ~/first && cd ~/first`.
   **F1.3** Which pilot does the starter use? A2's sample file has two Autoware pilots, one
   `carla_basic` and one `launch` pilot. On one GPU, two Autowares beside CARLA is past the
   comfortable limit, which the docs give as one ego plus one background AV. The starter
   must be the minimal one: one entity, Autoware, Town01. The student also cannot judge
   whether to try `carla_basic` first, as getting-started does today.
5. `ros2 run swb_launch assets fetch`. This downloads Town01 into `~/.cache/swb/` and links
   it into `assets/maps/Town01`. **F1.4** Size and time are not stated, and the cache path
   is not configurable in the design (see F4.5).
6. `play_launch launch swb_launch swb.launch.xml`. The steps that follow:
   - The world server writes the signal table on first load (A7). **F1.5** It writes into
     the map directory, which is a link into the shared cache, so the write lands in the
     cache (see F5.2). Two projects loading Town01 for the first time at once race on the
     same file.
   - The Autoware pilot starts. The first start may build TensorRT engines (3 to 11 minutes
     on a host whose TensorRT is not 10.8.0). **F1.6** The TensorRT 10.8.0 runtime is filled
     today by `just setup-tensorrt`, a `just` recipe, and B6 bans `just` from user docs. No
     ROS-command replacement is named.
   - The scenario. **F1.7** With `end_with_scenario` true by default, does the entry launch
     wait for `ego_ready` before it starts SSv2? Today it does not, and the docs make the
     user raise `initialize_duration`. If 076 keeps that behaviour, the newcomer's first run
     fails with `waited for the Autoware state` while Autoware is still coming up.
7. Read the verdict: `cat runs/<time>/.../result.junit.xml`. **F1.8** Three things are
   missing: a fixed path such as `runs/latest/verdict`, a console line such as
   `PASS town01_ego_drive`, and a launch exit code that says pass or fail.
8. Run it again. The whole stack restarts (Autoware about 20 s to 4 minutes, plus
   `ego_ready`). **F1.9** Nothing tells the student how to keep the stack up and rerun only
   the scenario. That workflow is persona 6's main need.

**Other frictions**: `ls` in the project now shows `play_log/` as well as `runs/` (section 0).
A second terminal with `ros2 topic list` needs `ROS_DOMAIN_ID=1`, but the domain now comes
from `swb.yaml`. 076 names no command that prints a project's domains.

## 2. Adopter: a lab with its own car and its own ROS 2 planner and controller (not Autoware); CARLA first, then its own map, then sharing with a colleague

**Walkthrough: their autopilot on a shipped car**

1. Build their planner in their own colcon workspace and source it after swb's.
2. `project new ~/lab_eval`, then edit `swb.yaml`:
   ```yaml
   simulator: carla
   world: Town01
   assets: [{carla-town-autoware-maps@v2026.11: [Town01]}]
   scenario: scenarios/cut_in.xosc
   pilots:
     ego: {kind: launch, launch: lab_pilot lab_pilot.launch.xml, vehicle: tesla_model3}
   ```
3. Their launch receives `entity relay sim map_path vehicle sim_endpoint`. **F2.1, the
   largest gap for this reader.** On CARLA, the car's sensors and its vehicle interface
   (`/vehicle/status/*`, `/control/command/*`) come from `acb_bridge`, and today
   `acb_bridge` lives inside the Autoware ego launch. With `kind: launch` the adopter must
   start `acb_bridge` themselves, with its sensor config, release sockets, `carla_port`
   and spec hash. That is swb's internals again. Their controller must also speak
   Autoware's control and status messages, and nothing on the user path says so. A5's
   table hides this behind "`sim`: for pilots that need the simulator's side".
4. Their agent. They need `ros_adapter` plus an `agent.yaml`, or an SDK class. **F2.2**
   Does the agent go in `pilots/` and start through `kind: command`, or inside their launch?
   A5 makes a pilot "an autopilot plus the agent" but gives no key for an agent config next
   to a launch. Today `agent:=false` covers this case. 076 does not say whether
   `kind: launch` implies "your launch brings its own agent" or "swb's Autoware agent runs
   beside it".
5. Run it: `play_launch launch swb_launch swb.launch.xml`.

**Their own car**

6. Write `vehicles/lab_car.yaml` (spec v2), then run
   `python3 -m sim_vehicle_spec.gen --spec vehicles/lab_car.yaml ...`. **F2.3** A1 says
   `vehicles/` holds "specs, catalogs", but nothing says how `pilots.ego.vehicle: lab_car`
   resolves to a project-local spec, or whether the entry launch runs the generator itself.
   Neither is the bridge's `vehicle_catalogs` named. The scenario's `CatalogLocations` also
   points at the catalog, so the car is named twice, once in `swb.yaml` and once in the xosc,
   and a mismatch ends in the spec-hash refusal at spawn instead of in `project check`.
   With an Autoware pilot the car also needs `<car>_description` and `_sensor_kit_*`
   packages built with colcon, plus measured pedal maps. Those are code, not project files.
7. **F2.4** Does the spec check apply when the agent is theirs? Today the bridge compares the
   vehicle agent's `spec_hash`. An SDK agent that reports none: refused, or skipped? The
   adopter learns this only from an error at spawn.

**Their own map**

8. **F2.5** `assets:` names only Hugging Face datasets under `jerry73204` (decision 5).
   There is no form for a local map directory or the lab's own dataset, so the adopter's
   map has no way into `assets/maps/`. They will copy it there by hand, and `assets check`
   then flags a file that no manifest lists. CARLA also needs the town itself: a cooked UE
   package or an OpenDRIVE world. That needs `carla/map_origin.yaml` and a signal table,
   and it is the hardest step of the whole persona. 076 says nothing about it.

**Sharing with a colleague on another host**

9. The colleague clones the project, runs `assets fetch` and starts it. Frictions:
   - **F2.6** `domains:` is in `swb.yaml`, a committed file. In the workspace rules (and in
     the user's memory: "args vs env") domains are host settings. A domain chosen on host A
     is copied to host B and collides there.
   - **F2.7** Host paths creep into the shared file. `carla_basic` needs `carla_pythonapi:`
     (a CARLA install path), and a pilot's `args:` may hold absolute paths. 076 does not say
     where per-host values live once `CARLA_MAPS` is gone: `.env`, a `swb.local.yaml`, or
     the environment.
   - **F2.8** Nothing pins the software. The project pins datasets (`@v2026.11`) but not
     the swb release, Autoware, CARLA or the pilot package's version. The colleague's older
     swb may not know a pilot kind, and the run fails late. `project check` should compare
     a `requires:` block.
   - **F2.9** Pilots on another machine. On CARLA the vehicle side may run on another host
     (relay over TCP), but the entry launch starts every pilot locally, under the localhost
     DDS profile. There is no `host:` key and no documented "run this one pilot here"
     command, such as a `pilot.launch.xml project:=... entity:=...`.

## 3. Educator: a lab of 20 students on 2 or 3 shared GPU servers; reproducible assignments, isolation per student, automatic grading

**Walkthrough**

1. Write `assignments/a1/` as a project (swb.yaml, scenarios, a `pilots/` skeleton) and
   publish it in a git repository.
2. Each student: `git clone ...; cd a1; ros2 run swb_launch assets fetch;
   play_launch launch swb_launch swb.launch.xml`.
3. Seven students on one server start at once. Frictions:
   - **F3.1 Domains collide.** Every clone carries the same `domains: {scenario: 9, pilots:
     1}`. A6 "assigns domains (checked 0-57)", but it only checks the range. Nothing stops
     the second student's stack from joining domain 1. Then two `/clock` publishers and two
     Autowares share topics, and the runs break in confusing ways instead of being refused.
     Pilots that "take 1, 2, 3, ... in order" use up blocks that overlap between seats.
   - **F3.2 Ports collide, and 076 does not move them at all.** Each seat moves eight values
     today: `SSV2_PORT` (and +1, +2), `AGENT_PORT`, `WORLD_PORT`, `CARLA_PORT` (and its
     Traffic Manager at +6000), play_launch's web UI (8080 by default), the stack's web
     port, and `SWB_LOG_ROOT`. 076 auto-assigns only domains. The CARLA world server's
     flock refuses a second world server on one CARLA, so each student needs a CARLA of
     their own. There is no seat or slot concept that derives all of these from one number.
   - **F3.3 Resources.** Seven CARLAs (about 2.4 GB RAM and 2 GB VRAM each) and seven
     Autowares (10 to 20 GB RAM each, several GB of VRAM) do not fit one server. The
     educator needs a light pilot (`simple`, which depends on 070 D, not yet done) for most
     labs, a queue for the Autoware ones, and a GPU-memory budget per kind in the docs.
     076 has none of these.
   - **F3.4 The shared cache, per user.** `~/.cache/swb/` is per user: 20 students hold 20
     copies of Town01, and 20 copies of 25 GB of CARLA USD if a lab uses Isaac. 20
     simultaneous Hugging Face downloads may also be rate-limited. The design needs a
     host-wide, read-only cache (an `SWB_CACHE` host setting, group-readable) that the
     educator fills once.
   - **F3.5 /tmp/swb-locks across users.** The CARLA lock directory must be world-writable
     with the sticky bit set. When the lock is held by another user's process, the refusal
     message should name that user.
4. Grading. B2 says "grading with run_suite's JUnit". **F3.6** `run_suite` today takes
   `--relay`, `-- port:=` and a domain, and it runs against a live stack. 076 does not make
   it project-aware. A grader needs a single command: run the teacher's hidden scenarios
   against the student's pilots, `--project student_dir --scenarios hidden/`, with an exit
   code and a stable output path. `project:=` takes one directory, and there is no way to
   combine "scenarios from here, pilots from there".
5. Reproducibility. Same as F2.8: the swb, Autoware and CARLA versions are not pinned, and
   the CARLA quality level (Low crashes on a town change) is a host choice that can break
   one student's runs and not another's.

## 4. Researcher: parameter sweeps (speeds, distances), seeds, many runs overnight, metrics in a table

**Walkthrough**

1. One scenario with parameters. SSv2's YAML scenarios have `ScenarioModifiers`, and
   `run_suite` runs every variant of one file (tutorial 2, section 5).
2. A script:
   ```bash
   for v in 8 11 14; do for d in 20 30 40; do
     play_launch launch swb_launch swb.launch.xml \
         scenario:=scenarios/cut_in.xosc  # how to pass v and d?
   done; done
   ```
   - **F4.1 There is no parameter override.** The entry launch has no
     `scenario_params:="ego_speed=11 gap=30"`. The researcher must template xosc files,
     or learn that only the YAML format sweeps. 076 does not mention `scenario_args`
     (SSv2's own options) at all, so `initialize_duration` and `global_timeout` have no
     place either.
   - **F4.2 Each run pays the start-up cost.** With `end_with_scenario` true, every run
     restarts Autoware and possibly CARLA. Over 200 runs, about 2 to 4 minutes of start-up
     each is 7 to 13 hours of overhead. The researcher needs "a stack up once, N runs
     against it" from the project, which is the long-lived form that 076 pushes
     "underneath".
   - **F4.3 Seeds.** The Traffic Manager seed is in `swb_carla_world.yaml`, the rock and
     road seed in `road_detail.yaml`, and SSv2's randomness in the scenario. There is no
     `seed:=` on the entry launch and no record of the seeds a run used.
   - **F4.4 Metrics.** `runs/<time>/` holds a JUnit verdict and logs. A table needs, per
     run, the parameters, the verdict and its reason, timings, and metrics (minimum gap,
     maximum deceleration, contacts), in one `summary.json`, plus a command that collects
     them into CSV. A8's rosbag is the raw material, not the table. `runs/<time>` names can
     also collide when two runs start in the same second, and the names say nothing about
     the parameters.
   - **F4.5 Disk and logs.** Overnight logs go under `runs/`, inside the project. On the
     group's hosts `/home` is shared and 87 to 94 % full, and one host's home is a spinning
     disk. Today `SWB_LOG_ROOT` moves logs to a fast disk. 076 does not say whether `runs/`
     follows it, and the shared cache has no configurable location either.
   - **F4.6 Failure classes.** An overnight table needs to tell "the scenario failed" from
     "the stack never became ready" from "CARLA crashed". 076's verdict is SSv2's alone.
   - **F4.7 Parallel batches.** Two batches on one host need two full slots (F3.2).

## 5. Isaac user: road-physics experiments (mud, rocks) with their own car

**Walkthrough**

1. Isaac Sim installed (`ISAAC_ROOT`), a GPU with 24 GB and 64 GB of RAM.
2. `swb.yaml`: `simulator: isaac`, `world: Town01`, assets `carla-town-autoware-maps` and
   `carla-0.9.16-isaac-usd` (about 25 GB).
3. Mud. Tutorial 7 copies `town01_mud.yaml` into the map directory as `surfaces.yaml`.
   - **F5.1** Under 076, `assets/maps/Town01` is a link into the shared cache. Copying a
     `surfaces.yaml` there changes the cache for **every project** on Town01, and
     `assets check` then exits 1 ("changed file"). The ground is per experiment, but the
     design puts it inside a shared, read-only asset. The ground should be a project file
     named in `swb.yaml` (for example `ground: {surfaces: worlds/mud.yaml, road_detail:
     worlds/rough.yaml}`), overridable per run (`ground:=...`), so a sweep over surfaces
     is possible. Today the scenario cannot choose the surface map (tutorial 7, section 6).
   - **F5.2** The world server writes saved ruts (`<map dir>/ruts/runs/<time>.npz`) and, on
     CARLA, the signal table into the map directory. Both are writes into the cache.
     They belong under `runs/` and in a per-project or per-host state directory.
4. Their own car. Isaac needs an aib ego config (`rig:`, `body:` as USD), the rig written by
   `aib.gen`, and on Autoware the car's packages. **F5.3** `vehicle: my_car` in `swb.yaml`
   does not show how a project-local USD body and rig are found. `SWB_VEHICLE_USD` points at
   the CARLA set's `vehicles/`; does it now point at the project's? 076's A4 covers
   `SWB_CARLA_USD` only.
5. Start: `play_launch launch swb_launch swb.launch.xml`. **F5.4** The Isaac world server
   preloads for about 7 minutes and holds 17.5 GB of RAM. With `end_with_scenario` true,
   every mud variant costs 7 minutes or more. Here the long-lived stack is required, not
   optional.
6. **F5.5** Isaac carries one ego only. A project with a second pilot on `simulator: isaac`
   should be refused by `project check`, not fail at run time. On Isaac the world server
   also publishes the ego's sensors in **its** domain, so the ego pilot must get the world
   server's domain. "Pilots take 1, 2, 3, ... in order" makes that depend on YAML key order.
7. Telemetry. `/swb/wheels` and `record:=true` (A8) fit this reader well. **F5.6** The world
   name is overloaded. `world: surfaces` (the 13-surface ground) is a map, `world: Town01`
   plus mud is a map plus a ground, and `isaac.launch.xml world:=` today is a `world.yaml`
   path. There are three meanings of "world" next to "world server".

## 6. Scenario author: writes OpenSCENARIO; wants to know what swb supports and to iterate on one scenario without restarting

**Walkthrough**

1. Find what is supported. Today that is `design/scenario-authoring.md` (an explanation
   page) and `design/ssv2-feature-completeness.md`. **F6.1** B1 puts reference pages under
   "generated where possible (launch arguments)", but there is no reference page for the
   supported OpenSCENARIO subset: controller names, refused actions for `agent` and
   `simulator_autopilot`, `swb.*` properties, `csbExpectedContacts`, `RoadCondition` keys.
   That is this reader's main page.
2. Write `scenarios/my.xosc` with `<LogicFile filepath="$(env SWB_MAPS)/Town01"/>`.
   - **F6.2** The map is named twice: in the xosc's `LogicFile` and in `swb.yaml`'s
     `world:`. If they disagree, Autoware gets one map while the bridge loads another, and
     the run fails waiting for Autoware's state, which is today's most common failure. One
     must win: derive `world` from the scenario, keep `world:` for free drive only, and
     have `project check` refuse a mismatch.
   - **F6.3** `SWB_MAPS` exists only inside the entry launch. The same xosc fails in
     `scenario.launch.xml` alone, in `run_suite` against a long-lived stack, and in SSv2's
     own tools. The user's existing scenarios with `$(env CARLA_MAPS)` are now refused by
     name. The refusal should name the file and line and give the replacement text.
3. Poses: `ros2 run swb_launch carla_poses ahead 2 --distance 120` needs CARLA up with that
   town loaded. Fine, but it is not linked from the project flow.
4. Iterate. Edit, run, read the verdict, edit again. **F6.4** The entry launch with a
   scenario ends with it. With `end_with_scenario:=false` the stack stays, but 076 gives
   no command to run the edited scenario against the live stack from the project.
   `scenario.launch.xml` needs `port:=`, a domain and an output directory, which is the
   old path. The author needs `ros2 run swb_launch run scenarios/my.xosc`: it reads
   `./swb.yaml`, finds the live stack, checks that the stack serves this project, and
   writes to `runs/`.
5. **F6.5** A town change in the xosc is a world load, which ends every pilot stack (the
   episode rule). Autoware restarts on the new map only if its `map_path` follows. Under
   076 the pilots get `map_path` from `swb.yaml`'s `world`, so a scenario on Town02 against
   a Town01 project gives a stack restarted on the wrong map.
6. **F6.6** Who drives an entity is said in two places. The xosc controller (`Autoware`,
   `agent`, `simulator_autopilot`, none) and `swb.yaml`'s pilots must agree:
   - A pilot for an entity whose controller is not `agent` (or the ego) sits idle,
     possibly a whole Autoware.
   - An `agent` entity with no pilot fails only when its goal is set.
   - An ego whose controller is named `Autoware` but whose pilot is `carla_basic` reads
     wrong.
   - `simulator_autopilot` is a third "kind" that lives only in the xosc.

   `project check` must cross-check every scenario in `scenarios/`, and the docs need one
   table: controller name, then who drives, then where it is configured.

## 7. Internal consistency of the design

| Question | 076 as written | Problem | Proposal |
|---|---|---|---|
| Who owns the map? | xosc `LogicFile` **and** `swb.yaml` `world` | two sources; a mismatch fails late | scenario wins; `world` only for free drive; check refuses a mismatch |
| Who owns the entities? | xosc; pilots keyed by entity name | fine, if checked | `project check` lists entities per scenario against pilots |
| Who owns each car? | xosc `CatalogReference` **and** `pilots.<e>.vehicle` | two sources; spec-hash refusal at spawn | derive the pilot's car from the catalog entry; `swb.yaml` names catalog locations only (`$(env SWB_CATALOGS)`) |
| Who drives? | xosc controller name **and** `pilots.<e>.kind` | 3 xosc names, 6 kinds, no table | one table; check both directions |
| Long-lived stack or a launch per run? | entry launch per run; long-lived forms "underneath" | CARLA 1-3 min, Autoware 20 s to 11 min, Isaac about 7 min per run | entry launch long-lived by default; a project-aware `run` (one or many scenarios) against it |
| Does the entry launch start CARLA? | "starts the simulator side" | ambiguous; acceptance implies yes | say so; host setting `CARLA_ROOT`; never stop a CARLA it did not start |
| The project's directory | `project:=`, default the cwd | `$(env PWD)` can be stale; play_launch writes `play_log/` there first | resolve with `os.getcwd()` in Python, print the resolved project first; document `--log-dir runs/...` or redirect |
| Domains | in `swb.yaml`, "auto-assigned", range-checked | a committed file holds a host setting; no collision check across projects or users | domains and ports from a **slot** (`SWB_SLOT=n`, host setting), claimed with a flock in `/tmp/swb-locks/slot-<n>`; `swb.yaml` holds none |
| Pilots' domains | "1, 2, 3, ... in order" | Isaac's ego must share the world server's domain; the CARLA world server is in the ego domain today | the ego is always the first domain of the slot; the rest follow by entity name, sorted |
| The `pilots:=` override | `"ego=carla_basic bg_av_1=none"` | only the kind; no args or vehicle; quoting differs between play_launch's parser and ros2 launch | layered files: `with:=ci.yaml` (merged over `swb.yaml`); keep `pilots:=` for the kind only |
| Hosts | all pilots local, localhost DDS | CARLA allows a remote vehicle side; the design cannot express it | `host:` per pilot, or a documented `pilot.launch.xml` to start one pilot elsewhere |
| Assets | HF `jerry73204` datasets only | no local or lab datasets; shared links get written into | `path:` and `hf:<org>/<name>@<rev>` sources; the cache read-only; writes go to `runs/` or a state directory |
| Vocabulary | pilot, agent, autopilot, vehicle side, ego stack, world, world server | seven nouns for three things | one glossary page; "pilot" in user docs, "agent" only for the SDK |

## 8. Consolidated frictions, ranked

Ranked by how many personas each one hits and how hard it blocks. Each line gives the
fix.

1. **Every run restarts the stack** (F1.9, F4.2, F5.4, F6.4). Fix: the entry launch stays
   up by default. Add a project-aware `ros2 run swb_launch run [scenario ...]` (one, many,
   variants) that finds the live stack, refuses a stack started from another project or
   another `swb.yaml` (the stack publishes the project path and a hash), and writes to
   `runs/`. Keep `end_with_scenario:=true` for CI. The entry launch waits for the pilots'
   readiness before it starts SSv2.
2. **Several seats on one host: domains and ports** (F3.1, F3.2, F2.6, F4.7, F3.5). Fix: a
   **slot**. `SWB_SLOT=n` (host setting) derives both domain blocks, every port and the
   web ports. A flock in `/tmp/swb-locks/slot-<n>.lock` (sticky, world-writable) refuses a
   taken slot and names its owner (user, pid, project). `slot:=auto` takes the first free
   one. Take `domains:` out of `swb.yaml`.
3. **The map and the car are named twice, and so is who drives** (F6.2, F6.6, F2.3,
   F6.5). Fix: the scenario owns the map, the entities, the cars and the controller.
   `swb.yaml` owns the simulator, the assets, and per-entity pilot kinds and args.
   `project check` cross-checks every scenario in `scenarios/` against the pilots, the
   fetched maps and the catalogs before anything starts. One table maps each controller
   name to who drives and where that is configured.
4. **A non-Autoware autopilot still needs swb's internals** (F2.1, F2.2, F2.4). Fix: split
   a pilot into a **vehicle side** that swb always starts (acb's sensors and vehicle
   interface on CARLA; Isaac's world server) and an **autopilot** (the user's launch) with
   an **agent** (`agent: ros_adapter pilots/agent.yaml`, `sdk: my_pkg.agent:Cls`, or
   `own`). Document the topics and message types the vehicle side offers. State the
   spec-check rule for agents that report no `spec_hash`.
5. **Writes into shared assets** (F1.5, F5.1, F5.2). Fix: the cache is read-only. The
   signal table, saved ruts and ground files go to a per-host state directory or to
   `runs/`. The ground becomes a project key (`ground:`) with a run override. `assets
   check` ignores, but lists, project-local overlays.
6. **Host settings versus the shared project** (F2.7, F2.8, F4.5, F3.4). Fix: one named
   place for host values (`.env` or `swb.local.yaml`, git-ignored, written by
   `project new`): `CARLA_ROOT`, `SWB_CACHE` (shared, group-readable),
   `SWB_RUNS_ROOT` (a fast disk). Add `requires:` in `swb.yaml` (swb, Autoware, CARLA,
   pilot packages), checked by `project check`.
7. **Nothing for batches** (F4.1, F4.3, F4.4, F4.6). Fix: `params:=` on `run` and the
   entry launch, mapped to SSv2's ParameterDeclarations; a `sweep:` block (or YAML
   modifiers) in the project; `seed:=` that sets the Traffic Manager, road and SSv2 seeds
   and records them; `runs/<id>/summary.json` (parameters, seeds, verdict, failure class,
   timings, metrics); `ros2 run swb_launch runs table > results.csv`. Run ids must be
   unique per run, not per second.
8. **The verdict and the console** (F1.8, F3.6). Fix: `runs/latest` and a one-line
   `PASS/FAIL <scenario>: <reason>` at the end. The launch exits 0 only on a pass.
   `run_suite` (or `run`) takes `--project` and `--scenarios` from different directories,
   for grading.
9. **The project directory under play_launch** (section 0). Fix: resolve the project with
   `os.getcwd()` (or `project:=`), never `$(env PWD)`. Print `project: <abs path>` first.
   Document `--log-dir runs/stack` and `--web-addr` (or have the slot set them through a
   small wrapper such as `ros2 run swb_launch up`), because play_launch writes `play_log/`
   and binds 8080 before the launch file can intervene.
10. **The environment file stays** (F1.1). Fix: ship
    `share/swb_launch/env.bash` (RMW, `CYCLONEDDS_URI`, sourced after Autoware), and
    `ros2 run swb_launch project env` to print the project's slot, domains and ports,
    ready for `ROS_DOMAIN_ID=$(...) ros2 topic list`.
11. **Resource limits per kind** (F1.3, F3.3). Fix: the starter project has one Autoware
    pilot. The docs and `project check` give RAM and VRAM per kind and per simulator
    (CARLA about 2.4 GB and 2 GB; Autoware 10 to 20 GB RAM; Isaac about 17.5 GB RAM and
    24 GB GPU) and warn when a project's sum exceeds the host's. Land `simple` (070 D)
    before the educator page relies on it.
12. **Own maps and datasets** (F2.5). Fix: `assets:` accepts `path:` and any HF repo. A
    how-to guide covers "your map on CARLA": OpenDRIVE or a cooked town, `map_origin.yaml`,
    the signal table.
13. **Isaac rules hidden until run time** (F5.5, F5.3, F5.6). Fix: `project check` refuses
    more than one pilot on Isaac. The ego takes the world server's domain. `SWB_VEHICLE_USD`
    gets the same project-relative treatment as `SWB_CARLA_USD`. "world" means one thing.
14. **Migration** (F6.3). Fix: the `CARLA_MAPS` refusal names the xosc file and line with
    the replacement text. `ros2 run swb_launch project import <dir-of-xosc>` rewrites
    `LogicFile` paths.
15. **Docs and tooling gaps** (F6.1, F1.6, F2.9). Fix: a generated reference for the
    OpenSCENARIO subset swb supports. A ROS command in place of `just setup-tensorrt`. A
    documented single-pilot launch for a pilot on another host.
