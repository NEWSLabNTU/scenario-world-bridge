# Roadmap 076 section A against five real use cases

What this is: each past use case walked through 076's project directory, `swb.yaml`, `assets fetch`, `$(env
SWB_MAPS)`, pilots and `swb.launch.xml`, with the file it would need and the commands, then where the design
holds, where it is awkward and what it lacks. Evidence is what the work actually ran: the justfile recipes,
`run_suite`, `scripts/coop/c3_run.sh` and `c3.py`, `play_log/course_probe/sweep*.sh`,
`spikes/course_probe.py`, the golf cart's `golfcart_twin/launch/twin.launch.py`, and roadmaps 020, 021, 024,
025, 033, 057-074. Written 2026-11-14.

Short verdict: the design fits "one scenario, one stack, started and judged once" well. Almost none of the
real work had that shape. Every study ran a long-lived stack and many runs against it, from its own driver
script with its own seeds, retries, restarts, result layout and host split. The design has no place for any of
that, so each study would still write a `c3_run.sh`. Two smaller structural problems recur: `world:` is per
project while scenarios name their own maps, and `domains:` puts a host setting in a project file.

Friction ids (F1...) are per use case; the changes (C1...) are ranked at the end.

## 1. Autoware safety WG scenarios (AWF use cases, suites, gap pipeline, demo tour)

**What the work needed.** `awf/UC-ACC-001-0001` and `UC-AEB-001-0001`, adapted to Town01 (lanelet ids
remapped, ego box set to the Model 3), each a YAML with `ScenarioModifiers` expanding to 27 and 9 variants
(UC-ACC needs 2258 s). Suites by `run_suite` over a directory, with the relay pre-flight, `--physical-verdict`
(021) and the restart ladder (030). `basic/` spans Town01, 02, 03, 05 and 10. The gap campaign (024, 025)
keeps the stacks up, turns on the bridge's `gap_trace_dir`, runs one generated scenario per genome, then
`gap_oracle` and `physical_verdict`; the search is ask/tell (the next scenario depends on the last outcome),
with confirm (3 runs, 2 agree), bisection and shrink, and infrastructure failures retried and never scored.
The demo tour (033) needs ClearNoon weather, CenterPoint with `max_velocity:=11.11`, chase view, captions, a
screencast and `incident_judge`. 020 also found UC-AEB passes 9/9 with `clustering` and 2/9 with
`centerpoint`: pilot settings are part of what a suite means.

**Under the design.**

```yaml
# awf_project/swb.yaml
simulator: carla
world: Town01
assets:
  - carla-town-autoware-maps@v2026.11: [Town01, Town02, Town03, Town05, Town10HD]
scenario: scenarios/UC-ACC-001-0001_Preceding_vehicle_partial_deceleration.yaml
pilots:
  ego: {kind: autoware, vehicle: tesla_model3, args: {lidar_detection_model: clustering}}
domains: {scenario: 9, pilots: 1}
```
```bash
ros2 run swb_launch assets fetch
play_launch launch swb_launch swb.launch.xml                      # one scenario, then ends
play_launch launch swb_launch swb.launch.xml scenario:=scenarios/UC-AEB-001-0001_Standing-pedestrian.yaml
# a suite: not in the design; today's run_suite needs the stack already up:
play_launch launch swb_launch swb.launch.xml scenario:= end_with_scenario:=false &
ros2 run swb_launch run_suite scenarios/ --output runs/suite --physical-verdict
```

**Works.** One AWF use case from a clean clone: fetch, launch, verdict in `runs/<time>/`. `$(env
SWB_MAPS)/Town01` replaces `$(env CARLA_MAPS)/Town01` cleanly. Pilot args carry `clustering`.

**Frictions.**

- **F1 Map per scenario.** `world: Town01` is project-wide, but each scenario's `LogicFile` names its town,
  and `basic/` mixes five. Today `just scenario` reads the map from the scenario (`_scenario-map`) and
  `carla_up.sh` replaces a stack on another map. The design does not say which wins, and the Autoware pilot
  gets `map_path` once at start. A town change ends every vehicle stack (roadmap 051), so the pilot must come
  back on the new map, which a respawn with the old `map_path` will not do.
- **F2 One run per launch.** `end_with_scenario` defaults to true with a scenario, so every scenario pays
  Autoware's start (minutes; CenterPoint 19 s to 189 s) and CARLA's town load. Suites, the gap search and the
  demo all reuse a stack. With `end_with_scenario:=false` the user leaves the design: `run_suite` takes no
  project, does not know `SWB_MAPS`, and writes where `--output` says, not into `runs/`.
- **F3 No run primitive for drivers.** The gap search is ask/tell: a Python loop that needs `run(genome) ->
  RunResult` (verdict, oracle findings, infrastructure or not). The design gives only a launch file. Variants
  (`ScenarioModifiers`) and their per-variant reset records are SSv2's, which is fine, but a parameter grid
  *outside* the scenario has nowhere to live.
- **F14 Judges and run tooling.** `physical_verdict`, `gap_oracle` (with the bridge's `gap_trace_dir`),
  `incident_judge`, chase view, captions, a screencast, the weather. None has a place in `swb.yaml`; `record`
  (A8) is only a rosbag.
- **F17 Pilots versus the scenario's entities.** `basic/` has an ego in most files and none in the benches;
  `multi_av/` adds `bg_av_1`. Pilots keyed by entity name start for every scenario in the run or none: either
  an idle Autoware for `bg_av_1` (minutes, GPU) or a missing pilot that `run_suite`'s pre-flight catches.

**Changes** (ranked list below): F1, F17 -> C3; F2 -> C1; F3 -> C2; F14 -> C7.

## 2. Mount Haruna drift campaign (057, 058, 060, 061, 065, 071)

**What the work needed.** A car of its own (`drift_mustang`: locked rear axle, ABS/TC/ESC off, 58 deg lock),
variants of it (locked versus open axle), and surfaces (asphalt, snow-packed). A course *generated* from open
data, not fetched: `python3 -m swb_isaac_world.course courses/haruna.yaml data/courses/haruna` (terrain,
ribbon, Lanelet2, point cloud, drift points, `surfaces.yaml`), plus a `carla-usd` link beside the world for
its props (issue 020). 057's first plan was Autoware between bends with `drift_controller` taking the command
near each waypoint, at 100 Hz; through swb the ego took one command per SSv2 frame (20 Hz), so the runs used a
10 ms step and 1/240 s physics. From 058 C9 on, the work left swb. `course_probe.py` runs inside Isaac's
process with no SSv2, relay or ROS. The law, or the live planner's L2 at 50 Hz, is in the physics loop; L1
(IPOPT) runs in its own process. Sweeps were shell files (`play_log/course_probe/sweep2.sh`: tag, entry speed,
`LAW_PARAMS` per run, a result line grepped from `SPIKE-RESULT`). Model-in-the-loop studies
(`play_log/race_planner/drift_study/`) had no simulator at all. 071 asks for routes never tuned on: a held-out
set of courses.

**Under the design.**

```yaml
# haruna/swb.yaml
simulator: isaac
world: haruna                    # not in any dataset: generated
assets:
  - carla-0.9.16-isaac-usd@v2026.11: [props]     # the guardrails and trees
  # the course itself: no way to say "generate from courses/haruna.yaml"
scenario: scenarios/haruna_down.xosc
pilots:
  ego:
    kind: launch
    launch: race_planner race_stack.launch.xml   # autoware-drift; L1 and L2 as two nodes
    vehicle: drift_mustang
    args: {style: S2, multistart: '{"schedule": {"always": false}}'}
domains: {scenario: 9, pilots: 1}
```
```bash
python3 -m swb_isaac_world.course courses/haruna.yaml assets/maps/haruna   # by hand
play_launch launch swb_launch swb.launch.xml
play_launch launch swb_launch swb.launch.xml pilots:="ego=simple"          # the baseline
```

**Works.** The `launch` kind fits a planner plus a controller as two nodes of one launch. `simple` gives a
pure-pursuit baseline. `simulator: isaac` with a fetched props set replaces the hand-made `carla-usd` link.

**Frictions.**

- **F12 Generated and private assets.** The course is the output of a generator with a config, and every
  course in 071's held-out set is another run of it. `assets fetch` only knows pinned Hugging Face sets.
- **F15 Timing and lockstep.** The drift needs control at physics rate, in lockstep. Through the relay and ROS
  a pilot is sampled at SSv2's frame rate; 057 needed `frame_rate` 10 ms and `physics_hz` 240, which are
  launch arguments today (`scenario.launch.xml` `frame_rate`, `isaac.launch.xml` `physics_hz`) and absent from
  `swb.yaml`. Nothing in the pilot contract says "wait for my command each step", so the result depends on
  host load, which is why `course_probe` kept the law in-process.
- **F16 World and vehicle variants.** The sweeps' axes were surfaces, the differential and the style. Surfaces
  live in a scenario's `surfaces.yaml` (056) or the world's; the differential is a spec field. Sweeping it
  means a spec per variant, each regenerated with its own `spec_hash`, or the bridge refuses the ego. There is
  no `vehicle` override, and no simulator-specific option block (`render`, `follow`, `physics_hz`, `worlds`
  preload, the road detail of A7).
- **F3 Sweeps.** `sweep2.sh` is a parameter list over `LAW_PARAMS` with one result line per tag. Under the
  design each line is a launch with a `pilots:` override string, and the result is still grepped out of a log.
- **F20 Harness mode.** The drift's own harness (no SSv2) and model-only studies need the project's assets and
  car resolved (map dir, rig, spec) without the stack. The design only resolves them inside `swb.launch.xml`.

**Changes**: F12, F20 -> C6; F15, F16 -> C8; F3 -> C2.

## 3. The cooperation study (074, `scripts/coop/`)

**What the work needed** (`c3_run.sh`, `c3.py`). Its own CARLA on port 2400 and a chosen GPU
(`-graphicsadapter`, `CUDA_VISIBLE_DEVICES`), Town03 loaded, the stack with `max_velocity:=$VB` and a
perception preset (C3a ground truth, C3b CenterPoint), the spec's physics and `gap_trace_dir`. A gate before
the batch: the west lane change with a rosbag of `/localization/kinematic_state`, and no batch unless the ego
holds `VB`. Then cells (`d0, dv`) x rules (`A_eq`, `A_aw_perm`) x seeds. Per run: a generated `.xosc` for the
cell, a **fresh Autoware** (`restart_ego`: after a scenario the next found Autoware stuck in PLANNING), the
cut-in agent (`cutin_agent.py`, an SDK agent) started with a per-run JSON config (`d0, dv, rule, seed, log
path`) because "the relay passes no scenario parameters to agents", a rosbag of six topics in the ego domain,
then `just scenario`, scoring from the agent's log. Retries: three attempts when the stack failed (14 of 24
runs on 241 stayed INITIALIZING), the failed attempt's dirs renamed `.failed1`. Hosts: 118 took seeds 0-3, 241
the arena's cells (`SEED0`, `CELLS`, `ARENA=0`). `--rescore` re-scores offline. Results went to the NAS
(`dataset/2026-11-13_coop-c3/c3b_118`, `logs/2026 SSv2 on CARLA/2026-11-14 coop .../`). The arena (C2) runs K
pairs in one CARLA without SSv2 for throughput (target 1000 runs/h).

**Under the design.**

```yaml
# coop_c3/swb.yaml
simulator: carla
world: Town03
assets:
  - carla-town-autoware-maps@v2026.11: [Town03]
scenario: runs/gen/c3.xosc       # rewritten per cell by a script
pilots:
  ego: {kind: autoware, vehicle: tesla_model3,
        args: {max_velocity: 6.0, lidar_detection_model: centerpoint}}
  coop_a: {kind: command, command: python3 scripts/cutin_agent.py --config runs/gen/coop_a.json}
domains: {scenario: 9, pilots: 1}
```
```bash
# per run, from a driver that still has to exist:
python3 scripts/gen_cell.py --d0 6 --dv 0 --rule A_eq --seed 1    # writes runs/gen/*
play_launch launch swb_launch swb.launch.xml                      # Autoware starts each time
```

**Works.** The two kinds in one run are exactly A5's example. `command` replaces the hand-made `Popen` of the
agent and the stray agent that kept the entity name. Autoware args carry the perception preset.

**Frictions.**

- **F4 Per-run pilot parameters.** The agent's context is the run's cell and seed. A pilot started once per
  stack cannot get it; a pilot started per run gets only the static `--relay --entity`. The agent protocol
  already has `reset.parameters`, but the relay does not fill it from the scenario.
- **F5 Pilot lifetime and restart.** Fresh Autoware per run, a per-run agent, a long-lived CARLA: three
  lifetimes in one study. The design has one ("a respawned play_launch of its own") and no readiness gate
  (`ego_ready`) before the scenario starts.
- **F3 Batches, seeds, shards, retries.** The grid, the seed split across hosts, the retry with its cause and
  the kept failed attempt, the pre-batch gate, `--rescore`: all in `c3.py`. The design neither provides them
  nor gives a driver an API to build them on.
- **F7 Simulator lifecycle and GPU.** The design says the entry launch "starts the simulator side". It does
  not say whether it starts CARLA, on which port, at which quality, on which GPU, or whether it loads the
  town. `c3_run.sh` needed all five.
- **F6 Domains and ports.** Two batches on two hosts are fine. Two on one host (C2's arenas on ports
  2600-2620) need every port moved. `domains:` in `swb.yaml` is a host setting in a file that is meant to be
  copied between hosts.
- **F14 Recording per pilot.** The bag was in the ego's domain, not the scenario's. `record:` (A8) names "the
  ego's key topics" with no per-pilot topic list.
- **F18 Results.** `runs/<time>/` is one run. The study's unit is a batch: `c3.jsonl` plus
  `<cell>_<rule>_s<seed>/attempt`, each self-describing enough to copy to the NAS and merge with another
  host's. Live logs must stay on NVMe (CLAUDE.md); a project kept on `/home` (a spinning disk on server139) or
  on the NAS would put them on a slow disk.

**Changes**: F4, F5 -> C4; F3, F18 -> C2; F6, F7 -> C5; F14 -> C7.

## 4. The golf cart Isaac twin (NEWSLabNTU/2026-golf-cart, roadmap 073's first user)

**What it needed** (`golfcart_twin`). Its own car: `vehicle/newslab_golfcart.yaml` (spec v2), the rig and body
generated from it (`isaac/newslab_golfcart.rig.yaml`, `body/.../entry.json`), `catalogs/vehicles.xosc` and
`isaac/spec_hash.yaml` from `sim_vehicle_spec.gen --spec`, Autoware's `golfcart_vehicle` description and
sensor kit. Its own Autoware: `golfcart_launch`'s logging simulation with twin-specific choices (clustering
beside CenterPoint, a latched speed cap, no GMSL cameras, its own RViz with the vehicle parameters). The NTU
twin map from `$TWIN_MAP` (not a public dataset), scenarios naming `$(env TWIN_MAP)` and `$(find-pkg-share
golfcart_twin)/catalogs`. Free drive on four routes (`routes.yaml`, `spawn` from a route's start, `twin_go.py
N` sets the goal and engages). Its own DDS profile (`cyclonedds-unicast-lo.xml`: loopback multicast is off on
118), `gpu:=` for Isaac (`--/renderer/activeGpu`). It is a ROS package: `play_launch launch golfcart_twin
twin.launch.py route:=1`, which includes `swb_launch isaac.launch.xml`.

**Under the design.**

```yaml
# golfcart_twin/swb.yaml (in the package source, installed to its share?)
simulator: isaac
world: ntu_twin                 # $TWIN_MAP today; private, not on Hugging Face
assets:
  - carla-0.9.16-isaac-usd@v2026.11: [props]
scenario: scenarios/route1.xosc
pilots:
  ego:
    kind: autoware              # but not swb's Autoware: golfcart_launch's
    vehicle: newslab_golfcart   # where is it? not in sim_vehicle_spec
    launch: golfcart_twin twin_autoware.launch.xml   # is `launch` allowed on kind autoware?
    args: {speed_limit: 2.5, demo_rviz: true}
domains: {scenario: 9, pilots: 1}
```
```bash
cd ~/repos/2026-golf-cart/src/simulation/golfcart_twin
play_launch launch swb_launch swb.launch.xml                 # scenario route1
play_launch launch swb_launch swb.launch.xml scenario:=      # free drive: from where?
```

**Works.** `kind: autoware` plus an Autoware launch of one's own is the "your Autoware" how-to in one line, if
`kind: autoware` takes `launch:`. The vehicle agent, `spec_hash` check and relay come for free, as
`twin.launch.py` gets them today from `isaac.launch.xml`.

**Frictions.**

- **F10 The project's own car.** `vehicle: <name>` resolves in `sim_vehicle_spec`. The cart needs five
  artefacts named (spec, catalog, spec_hash, Isaac `ego.yaml` with rig and body, Autoware `vehicle_model`),
  today five launch arguments (`ego`, `vehicle_catalogs`, `spec_hash_file`, `vehicle_model`, `vehicle`). A1
  lists a `vehicles/` directory with no format.
- **F11 A project that is a package.** The project is installed with colcon and its files are found by
  `find-pkg-share`. The design reads `./swb.yaml` relative to the working directory; `runs/` and `assets/`
  inside an install share are wrong (read-only, wiped by a rebuild). The cart also wants to keep its own entry
  (`twin.launch.py route:=1`) on top of swb's.
- **F12 Private and local maps.** The NTU twin is not a public dataset; scenarios name `$(env TWIN_MAP)`. The
  design retires such env vars from the user path but gives no way to name a map that is only on this host or
  the NAS.
- **F13 Free drive and routes.** "Empty scenario: free drive" (A2), but free drive needs a spawn pose
  (`spawn:=x,y,z,yaw`), and the cart's demo is four named routes with a goal.
- **F9 The pilot's simulator side.** On Isaac the car is inside the world server (aib's rig; the
  `isaac_vehicle` node follows `EGO_ROS_DOMAIN_ID`), not in the pilot's launch. "One domain per pilot" has to
  reach into the world server, which one `ego:=` argument holds today; a second Isaac ego (039) means one ROS
  context per ego in that process.
- **F6 DDS profile and domains.** The cart ships its own Cyclone profile and takes domains and ports from
  `scripts/twin/setup.bash`. The design sets domains but says nothing of the RMW and the profile, which
  roadmap 070 found must match on every process (friction 14).
- **F8 GPU.** `gpu:=` picks Isaac's renderer; the design has no GPU setting.

**Changes**: F10, F11, F12 -> C6; F13 -> C9; F9 -> C4; F6, F8 -> C5.

## 5. Two stacks on one host, several checkouts, the developer's long-lived stack

**What the work needed.** CLAUDE.md: a second checkout moves its whole set in `.env`: `SWB_WEB_PORT` (+0..+5),
`SSV2_PORT` (the sensor-release sockets follow at +1 and +2, on both the world server's and acb's side),
`AGENT_PORT`, `WORLD_PORT`, the three domains; `CARLA_PORT` and its Traffic Manager (+6000) when a second
CARLA runs; the CARLA world server's `flock` per CARLA. Domains at 57 or below (the profile's DomainGain). The
stack (`just run`, web 8084) is a tracked singleton: it outlives sessions (one served scenarios 44 hours later
on code two builds old), must be restarted after `just build`, and `just scenario` attaches to it, starting it
only when absent and refusing a stack of another simulator or replacing one on another map. A separate `just
ego-av` (8082) can be the ego.

**Under the design.**

```yaml
# checkout B, project p2: swb.yaml
simulator: carla
world: Town01
pilots: {ego: {kind: autoware, vehicle: tesla_model3}}
domains: {scenario: 19, pilots: 11}     # moved by hand; still not the ports
```
```bash
play_launch launch swb_launch swb.launch.xml scenario:= end_with_scenario:=false   # stays up
play_launch launch swb_launch scenario.launch.xml scenario:=... port:=$SSV2_PORT    # outside the project
```

**Works.** The domain check (0-57) and the refusal of what is not fetched remove two of the known foot-guns.
One pilot per domain is the right isolation.

**Frictions.**

- **F6 Host settings in the project file.** Domains are host settings today and should stay so: a project
  copied to another host (076's own acceptance) carries checkout A's domains into checkout B's. Ports are not
  in `swb.yaml` at all, so two projects on one host collide on 5555, 5560, 5591, 8080-8085 and the release
  sockets anyway. "Pilots take 1, 2, 3" meets the scenario domain 9 at the ninth pilot and the other
  checkout's at the second.
- **F2 The long-lived stack.** The design's unit is the run. The developer's is the stack: started once, many
  scenarios, rebuilt and restarted deliberately. Without an attach mode the long-lived form is
  `carla.launch.xml` and `scenario.launch.xml`, outside the project (no `SWB_MAPS`, no pilots, no `runs/`).
- **F19 Stale stacks.** An attach must know the running stack is this project's, with these pilots, on these
  binaries. Today that is a port check and a map check. The 0.618 versus 0.051 m/s^2 acb case shows the cost
  of trusting a stale one.
- **F7 Who starts CARLA, who stops it.** `carla_up.sh` starts CARLA and leaves it; `e2e` stops what it
  started; CARLA at Low must never change towns. A per-run launch that starts CARLA pays its start each time;
  one that does not needs it already up.
- **F18 Live logs.** `SWB_LOG_ROOT` exists because live logs on a spinning disk stalled acb's loop. `runs/`
  under the project inherits whatever disk the project is on.

**Changes**: F6, F7 -> C5; F2, F19 -> C1; F18 -> C2.

## Ranked design changes

Ranked by how many use cases they unblock and how much driver code they retire.

**C1. The stack is the unit; runs attach to it.** (F2, F19; use cases 1, 3, 5, and 2 when it comes back to
swb.) `swb.launch.xml` gets `mode:=stack` (simulator side and pilots, no scenario, stays up, respawns) and
`mode:=run` (default when a scenario is given: attach to this project's running stack if one is up, start it
if not, run the scenario, leave the stack up unless this run started it and `end_with_scenario:=true`). The
stack publishes a fingerprint: project path, a hash of the effective `swb.yaml` with overrides, pilot set, and
the install's build time and git SHAs. An attach refuses a mismatch with what differs and the command to
restart it. `run_suite` takes `project:=` and attaches the same way. This is `just scenario`'s behaviour
today, moved into the ROS path.

**C2. A run primitive, a suite file and a batch layout.** (F3, F5, F18; use cases 1, 2, 3.)

- Python: `swb_launch.runs.run_one(project, overrides, run_dir) -> RunResult` (verdict, testcases, judges'
  output, `infrastructure: bool` with the cause, timings). The gap search, double oracle and
  confirm/bisect/shrink build on it.
- Declarative, for grids: a suite file run by `ros2 run swb_launch run_suite suite.yaml`:

  ```yaml
  scenarios: [scenarios/c3_cutin.xosc.j2]        # a template is rendered per cell
  matrix: {d0: [6, 10, 14], dv: [0, -3], rule: [A_eq, A_aw_perm]}
  seeds: 4
  shard: ${SWB_SHARD}                              # "0/2" on 118, "1/2" on 241
  gate: {scenario: scenarios/e2_lane_change_west.xosc, judge: vmax_check}
  retry: {on: infrastructure, max: 3}             # failed attempts kept
  restart: {ego: every-run}                       # run_suite's ladder, per pilot
  ```

- Layout: `<runs root>/<suite>/<case>/attempt-<k>/`, `results.jsonl` (one line per finished case, appended,
  resumable), and `manifest.json` per run (fingerprint, seed, parameters, overrides, host, GPU, the
  repositories' SHAs, `spec_hash`). Runs root is a host setting, `SWB_RUNS_ROOT`, default `<project>/runs`,
  linked from the project when elsewhere, so live logs stay on a fast disk. `ros2 run swb_launch runs export
  <dest>` copies a suite to the NAS layout and merges shards.

**C3. The map comes from the scenario; pilots follow the scenario's entities.** (F1, F17; use cases 1 and 3.)
`world:` becomes the free-drive default only. The entry launch reads each scenario's `LogicFile` and refuses
one whose map is not fetched. On a town change the pilots' supervisors restart them with the new `map_path`,
which the relay already knows (`reset.map`). `assets` may be omitted for maps: `project check` derives them
from the scenarios. Pilots match by entity name, by pattern, or by role (`ego`, `controller: agent`); a pilot
starts only for scenarios that have its entity, and `project check` reports each scenario's unmatched
entities.

**C4. A pilot is a vehicle side plus a driver, with lifetime and per-run parameters.** (F4, F5, F9, F19; use
cases 2, 3, 4.)

- The vehicle side belongs to the simulator package: acb in the pilot's domain on CARLA (with the release
  sockets and the physics verify), aib's rig inside the world server on Isaac (one ROS context per pilot's
  domain). The driver is the autopilot plus the agent. The entry launch wires one domain into both, so `kind:
  launch` never has to know acb or aib.
- `lifetime: stack | run` and `restart: never | on-failure | on-reset-failure | every-run` per pilot; `ready:`
  a check with a timeout (`ego_ready` for Autoware) before the scenario starts.
- The relay fills `reset.parameters` from the entity's controller properties and the scenario's parameters
  (the protocol field exists). A `command` or `launch` pilot also gets `{run_dir}`, `{seed}` and
  `{param:<name>}` substitutions and `SWB_RUN_DIR`, `SWB_SEED`. The cut-in agent's per-run JSON goes away.
- `kind: autoware` takes `launch:` (your Autoware) and `vehicle:`; the `autoware_sim_profile` argument check
  still applies.

**C5. Seats for host settings; simulator start policy.** (F6, F7, F8; use cases 3, 4, 5.) Drop `domains:` from
`swb.yaml`. A host setting `SWB_SEAT=<n>` (env or `.env`) derives every port (SSv2, relay, world server, web
UIs, release sockets, CARLA and its Traffic Manager when swb starts CARLA) and a domain block (scenario domain
= block base, pilots base+1...), kept at 57 or below. A seat lock under `/tmp/swb-locks/` refuses a second
stack on one seat. Explicit env (`SSV2_PORT`, ...) still wins. The run writes `endpoints.yaml` with what it
got, and `ros2 run swb_launch status` lists seats on the host. GPU (`SWB_GPU`, or per part: simulator, pilots)
and the RMW and DDS profile are seat settings too. In `swb.yaml`: `simulator: {name: carla, start: auto |
external, quality: epic}`. `auto` starts CARLA on the seat's port and leaves it up. `project check` refuses
`quality: low` with scenarios on more than one town.

**C6. Projects as packages, the project's own car, assets beyond Hugging Face.** (F10, F11, F12, F20; use
cases 2 and 4.)

- `project:=pkg://<package>` reads `swb.yaml` from a package's share. `assets/` and `runs/` go to
  `$XDG_STATE_HOME/swb/<package>/` (writable, not wiped by colcon). `swb.launch.xml` is includable with
  arguments, so `twin.launch.py route:=1` stays a thin wrapper.
- `vehicles:` names the project's cars: `{spec: vehicles/newslab_golfcart.yaml, autoware: golfcart_vehicle,
  catalog: catalogs/vehicles.xosc, isaac: isaac/ego.yaml}`. `project check` regenerates in memory and refuses
  a stale catalog, rig or `spec_hash`.
- Asset sources: `hf: <set>@<tag>`, `path: <dir>` (private or local, e.g. the NTU twin), and `generate:
  {command: python3 -m swb_isaac_world.course, config: courses/haruna.yaml}`, cached by a hash of the
  generator's version and inputs. A host-local override (`SWB_ASSET_ntu_twin=/mnt/wd/...`) supplies what
  cannot be shared. Scenarios name `$(env SWB_MAPS)/<name>` for every source.
- `ros2 run swb_launch project env` prints the resolved `SWB_MAPS`, `SWB_CARLA_USD`, rig and spec paths for
  harnesses that do not run the stack (`course_probe`, the arena, model studies).

**C7. Judges, tools and recording in the project.** (F14; use cases 1 and 3.) `judges:` post-run programs that
add JUnit testcases to the run (`physical_verdict`, `gap_oracle`, `incident_judge`, a study's own
`vmax_check`); a judge that needs a bridge setting declares it (`gap_oracle` turns on `gap_trace_dir` into the
run directory). `tools:` per-run side processes in a named pilot's or the scenario's domain (chase view,
captions, screencast). `record: {ego: [topics], scenario: [topics]}` per pilot domain. `world_config:` for the
weather and the bridge's and world server's keys.

**C8. Timing, lockstep and simulator options.** (F15, F16; use case 2.) `timing: {frame_rate, physics_hz,
realtime}` in `swb.yaml`, passed to SSv2 and the world server. A pilot option `sync: tick`: the world server
holds each step until that pilot's command for the step arrives, within a budget, and counts the misses in the
run's manifest. Per-simulator blocks (`isaac: {render, follow, worlds, road_detail}`, `carla:
{npc_contact_sensors}`) validated by the simulator package's schema. `world:` as an object `{map, surfaces}`.
Vehicle variants by overlay (`vehicle: drift_mustang{powertrain.differential: open}`), generated with their
own `spec_hash`, so a matrix can sweep them.

**C9. Free drive with a spawn, and named routes.** (F13; use case 4.) `spawn:` (pose) or `routes:` in
`swb.yaml`, `route:=<name>` on the entry launch, and a `goal` tool (`twin_go.py`'s job) that sets the route's
goal and engages through the pilot's agent.

## Not covered by any change above

- The arena (074 C2, K pairs per CARLA without SSv2, for about 1000 runs/h) and the drift's in-process harness
  are throughput and latency choices that SSv2 in the loop cannot match. C6's `project env` lets them share
  the project's assets and car; they stay harnesses.
- Autoware non-determinism (025's confirm needs 2 of 3 runs to agree) is a suite-level `repeat:`/`agree:`
  option of C2, not a design change of its own.
