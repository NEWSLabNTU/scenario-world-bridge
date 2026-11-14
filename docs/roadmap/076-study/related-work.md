# Related work: how comparable tools shape their interface, and what an audit platform needs

Written 2026-11-14 as input to roadmap 076 section A (project directory, `swb.yaml`, `assets
fetch`, pilots, `swb.launch.xml`). Facts marked "(web)" were checked against the sources listed
at the end on 2026-11-14; the rest is from the tools' public documentation as generally known
and should be re-checked before it is quoted in user docs.

## 1. The five questions, tool by tool

For each tool: how a user defines an experiment, plugs in an autopilot, runs batches or
variations, gets results, shares assets, and reproduces a run.

### 1.1 CARLA Leaderboard (1.0, 2.0, 2.1)

- **Experiment**: a routes XML (`routes_*.xml`): per route a town, waypoints and, since 2.0,
  the scenarios placed along it with their parameters. Weather is per route. The "track"
  (SENSORS or MAP) fixes which inputs an agent may ask for. (web)
- **Agent interface**: a Python file whose `get_entry_point()` names a class derived from
  `leaderboard.autoagents.autonomous_agent.AutonomousAgent`; `setup(path_to_conf_file)` sets
  `self.track`, `sensors()` returns a list of sensor dicts (type, pose, fov, id) that the
  evaluator spawns and validates against the track's limits (2.1: cameras 0-8, LiDAR 0-2,
  radar 0-4, GNSS, IMU, speedometer; MAP adds an OpenDRIVE pseudo-sensor), `run_step
  (input_data, timestamp)` returns a `carla.VehicleControl`, `destroy()` cleans up. A ROS
  variant (`ROS1Agent`/`ROS2Agent`) exists for stacks like Autoware. (web)
- **Running**: `leaderboard_evaluator.py --routes --routes-subset --repetitions --track
  --agent --agent-config --checkpoint --resume`, usually via env vars in `run_leaderboard.sh`
  (`TEAM_AGENT`, `TEAM_CONFIG`, `CHECKPOINT_ENDPOINT`, `CHALLENGE_TRACK_CODENAME`).
  `--resume` continues a crashed batch from the checkpoint. No parameter distributions:
  variation is by enumerating routes and repetitions.
- **Results**: a checkpoint JSON, per route `index`, `route_id`, `status` (e.g. "Failed -
  Agent deviated from the route"), `num_infractions`, `infractions` (lists of events by type:
  `collisions_layout`, `collisions_pedestrian`, `collisions_vehicle`, `red_light`,
  `stop_infraction`, `outside_route_lanes`, `min_speed_infractions`,
  `yield_emergency_vehicle_infractions`, `scenario_timeouts`, `route_dev`, `vehicle_blocked`,
  `route_timeout`), `scores` (`score_route`, `score_penalty`, `score_composed`), `meta`
  (`route_length`, `duration_game`, `duration_system`), plus a global record with an
  `entry_status` and sensors used. Driving score = route completion x infraction penalty; 2.x
  penalty P = 1 / (1 + sum c_j n_j) with c = 1.0 pedestrian, 0.70 vehicle, 0.60 static, 0.40
  red light, 0.40 failure to yield, 0.4 scenario timeout, up to 0.4 min speed, 0.25 stop
  sign; off-road subtracts from completion. Shutdown events: route deviation > 30 m, blocked
  180 s sim time, simulation timeout 60 s, route timeout. (web)
- **Assets**: CARLA's own towns; nothing to fetch beyond the CARLA release.
- **Reproduction / submission**: the agent ships as a Docker image built with
  `make_docker.sh` from `Dockerfile.master` or `Dockerfile.ros`, user commands between
  markers, code at `/workspace/team_code`, extra sourcing in `~/agent_sources.sh`; anything
  the user changed in leaderboard or scenario_runner is overwritten. Submission went through
  EvalAI (`evalai push --phase`), statuses SUBMITTED, PARTIALLY_EVALUATED, FINISHED, FAILED.
  The public leaderboard is now permanently closed; the code is kept for local use. (web)
- **Take-away for swb**: the cleanest third-party contract in the field: the evaluator owns
  the simulator, the routes and the scoring; the agent owns only its decision; the sensor
  request is declared and checked; the result is one JSON with fixed infraction names. Its
  weakness is that the agent runs in the evaluator's process (Python import), which swb
  avoids by having a separate process per pilot.

### 1.2 CARLA ScenarioRunner and CARLA Python API patterns

- ScenarioRunner runs Python scenarios (py_trees behaviours), route files and OpenSCENARIO
  1.0 (`--scenario`, `--route`, `--openscenario`, `--agent`, `--reloadWorld`,
  `--repetitions`, `--output`, `--junit`, `--json`, `--file`, `--record`). OpenSCENARIO 1.0
  defines no pass criteria, so ScenarioRunner reuses StopTrigger conditions as criteria (web).
  Results are console tables, JUnit XML and JSON of the criteria; the metrics module computes
  metrics offline from a CARLA recorder log plus a criteria JSON (web).
- CARLA's Python API projects follow one pattern: a client script that sets synchronous mode
  with a fixed delta, seeds the Traffic Manager (`tm.set_random_device_seed`), spawns actors,
  ticks, and destroys what it spawned. Determinism is only claimed for synchronous mode with
  fixed `fixed_delta_seconds` and a seeded Traffic Manager; physics substepping and rendering
  still vary across GPUs.
- **Take-away**: CARLA's own runner already writes JUnit; swb's choice of SSv2's
  `result.junit.xml` is in line. The recorder log as a replayable artefact is worth keeping in
  `runs/<time>/` (CARLA's `start_recorder`), next to the rosbag A8 plans.

### 1.3 Autoware: Evaluator, driving_log_replayer, scenario_simulator_v2, AWSIM

- **Autoware Evaluator (TIER IV Web.Auto)**: a web CI/CD service. Users upload scenarios
  with their maps, group them into suites and catalogs ("a group of suites that were created
  for a specific use case"), run a catalog in parallel against an Autoware build and get a
  pass/fail table with logs. Run from CI by `webauto ci scenario run --project-id
  --scenario-id --scenario-version-id`; scenarios and maps are versioned objects with IDs.
  Access needs a TIER IV account; AWF working groups (ODD WG) ran their scenarios weekly
  there; one AWF update reported 502 of 554 passed (November 2023). (web)
- **scenario_simulator_v2**: `scenario_test_runner` takes `scenario:=` or `workflow:=` (a
  YAML listing scenario paths with an expected outcome, `Expect: success|failure|error`),
  plus `architecture_type`, `sensor_model`, `vehicle_model`, `launch_autoware`,
  `global_frame_rate`, `global_real_time_factor`, `global_timeout`, `output_directory`.
  OpenSCENARIO `ParameterValueDistribution` files are expanded into one run per value set.
  Results: JUnit 5 `result.junit.xml`; a `failure` is the scenario reaching an unexpected
  state (collision, stuck), its message one of "Expected success/failure/error"; an `error`
  is a runtime problem (Autoware down), typed by the exception (`SemanticError`,
  `SyntaxError`, `InternalError`, `UnknownError`). The random test runner writes
  `result.yaml` that replays a failing random case. (web)
- **driving_log_replayer / v2**: one `scenario.yaml` per test (ScenarioFormatVersion,
  ScenarioName, Evaluation with UseCaseName and its Conditions, the rosbag and map paths);
  outputs per dataset a `result.jsonl` (one JSON line per evaluation frame, the last line
  holds the final verdict, which the Evaluator reads) and a `result_bag`. JSONL is the
  documented format for analysis. A CLI runs a directory of `scenario.yaml`s. (web)
- **AWSIM / AWSIM Labs**: a Unity simulator that is a second SSv2 backend over the same
  ZMQ `simulation_interface` swb implements; the user path is "start AWSIM, run
  scenario_test_runner with `launch_autoware:=false`". Assets are a prebuilt binary per
  release plus a map package (lanelet2 + pcd) downloaded by hand.
- **Take-away**: swb already sits in this ecosystem: its scenarios are SSv2's, its verdict is
  SSv2's JUnit, its batch runner (`run_suite`) merges JUnit and writes `suite.json`. What the
  Autoware side adds and swb lacks: versioned objects with IDs (scenario version, map
  version), the expected outcome stored with the scenario (`Expect:`), and a per-frame JSONL
  whose last line is the verdict.

### 1.4 OpenSCENARIO tooling

- **ASAM OpenSCENARIO XML 1.x**: a scenario declares `ParameterDeclarations`; a separate
  **parameter value distribution file** (`ParameterValueDistribution`, handled like a
  catalog) references the scenario and gives Deterministic (single- or multi-parameter:
  ranges with step, value sets) or Stochastic distributions (normal, uniform, Poisson,
  histogram, probability set, user-defined), with `numberOfTestRuns` and `randomSeed` on the
  stochastic ones (web). This is the standard's own answer to "run a batch": the scenario
  stays one file, the variation is another.
- **ASAM OpenSCENARIO DSL (2.x)**: a language (`.osc`) with abstract scenarios, constraints,
  coverage (`cover`, `record`) and KPIs as first-class constructs. The standard itself leaves
  stochastic sampling to tools (migration guide, web) and gives an informative annex on
  parameter distributions.
- **esmini**: the reference player. `esmini --osc x.xosc --headless --fixed_timestep 0.05
  --record sim.dat`, a CSV/dat log and `--param_dist` / `--param_permutation` for
  distribution files; options can sit in a YAML config (`--config_file_path`). `--seed` makes
  stochastic parts repeatable. (web for headless, fixed timestep, record, config file.)
- **Foretify (Foretellix)**: OSC 2 based, coverage-driven verification: a V&V plan of
  metrics, KPIs and checks; tests generated from abstract scenarios; Foretify Manager triages
  runs and reports coverage of the ODD space ("85 % of ODD variations covered"). Simulators
  and the stack are plugged in by adapters (CARLA, NVIDIA, others); V-Suites ship scenario
  libraries per function with built-in KPIs. (web)
- **MathWorks RoadRunner Scenario**: scenario variables set through MATLAB
  (`setScenarioVariable`) or gRPC; `createSimulation`, `Logging="on"`, `SimulationLog`,
  `MaxSimulationTime`; exports OpenSCENARIO 1.x and 2.0; ships Euro NCAP test scenario
  sets and variant generators for AEB car-to-car. Batch loops are user MATLAB code. (web)
- **Take-away**: the parameter distribution file is the standard batch unit; SSv2 already
  expands it, so swb should expose it (a project names a distribution file as readily as a
  scenario) rather than invent a seed sweep in `swb.yaml`.

### 1.5 Other simulation and benchmark stacks

- **Apollo Dreamland / Dreamview**: scenarios edited in a web editor (name, duration, road
  structure, ego behaviour, the metrics to track); 200+ sample scenarios (LogSim from real
  logs, WorldSim hand-made); an automatic grading system with about ten metrics (collision,
  red light, speed limit, out of lane, end of route...). Scenario sets are a cloud object, not
  a file in a repo. (web)
- **SVL / LGSVL** (discontinued 2022): Python API `lgsvl.Simulator(host, port)`,
  `sim.load(scene, seed)`, `add_agent`, `connect_bridge` for Autoware/Apollo; a
  "simulation" was a web object (map, vehicle with sensor JSON, autopilot bridge, cluster)
  and assets came from its web store with content IDs. Its shutdown stranded every asset
  link: an argument for content that lives in open, mirrorable stores (HF, git).
- **MetaDrive / ScenarioNet**: ScenarioNet converts Waymo, nuScenes, nuPlan, Lyft, Argoverse
  into one scenario description (nested dict, four top-level keys, metadata with source and
  frame interval; one `scenario_id.pkl` per scenario) and a database with filter, merge,
  check, split and overlap tools; MetaDrive replays them closed-loop. The database, not the
  scenario, is the unit of sharing. (web)
- **nuPlan devkit**: planner interface `AbstractPlanner` (`initialize(PlannerInitialization)`
  with map, route roadblocks, mission goal; `compute_planner_trajectory(PlannerInput)` with
  iteration, history, traffic lights; returns an 8 s trajectory, tracked by an LQR
  controller). Experiments are Hydra configs (`+simulation=closed_loop_reactive_agents
  planner=... scenario_filter=...`); three modes (open loop, closed loop non-reactive,
  reactive); metrics per scenario into parquet files and an aggregated score (closed loop:
  multiplier metrics such as at-fault collision, drivable area, direction compliance, making
  progress, times weighted TTC, progress, speed limit, comfort in 5:5:4:2), nuBoard for
  viewing; the competition took a Docker image of the planner. (web, partly)
- **Waymax**: JAX, data-driven from WOMD at 10 Hz; agents are functions from state to
  action, batched on accelerators; metrics log divergence, overlap, offroad, wrong way,
  kinematic infeasibility. Determinism is total (pure functions). (web)
- **BARK**: scenario generators (`ConfigurableScenarioGeneration`, ...) produce a benchmark
  database; `BenchmarkRunner` (and a Ray variant) evaluates behaviour models with several
  parameter configs over the database; evaluators return bool/int/real per world state and
  can terminate a run; results into a pandas DataFrame. (web)
- **highway-env**: Gymnasium environments, `env.configure({...})` dict, `reset(seed=)`;
  the experiment is code, the agent is a policy; reproducibility by seed only.
- **NVIDIA SimReady / Omniverse**: SimReady Foundation is a spec over OpenUSD (features,
  capabilities, profiles); `simready-validate` writes `results.json` and can stamp
  `SimReady_Metadata` into the USD; `simready-package` runs pre-validate, build,
  post-validate and writes `com.nvidia.simready.packaging.json`; validation is mandatory to
  publish to a repository. Isaac Sim's Asset Validation extension runs the same rules. (web)
- **Hugging Face Hub as asset store**: `snapshot_download(repo_id, repo_type="dataset",
  revision=...)` takes a branch, a tag or a full 40-character commit hash (short hashes are
  refused); default is the latest revision; the cache keeps one snapshot per commit and a
  `refs` map from names to commits, and resolves offline from it; `allow_patterns` limits
  what is fetched. Tags can be moved, commits cannot. (web)
- **Take-away**: the stacks that grew a third-party evaluation story (Leaderboard, nuPlan)
  all froze three things: a narrow agent API, a fixed metric list with fixed names and
  weights, and a container boundary for submissions. The ones that kept experiments as
  web objects (SVL, Dreamland, Evaluator) are hard to reproduce outside their service.

### 1.6 Safety assessment frameworks and scenario sets

- **UN R157 (ALKS)**: Annex 5 lists test scenarios (lane keeping, following, cut-in,
  cut-out, deceleration of lead vehicle, stationary obstacle, pedestrian, ...) with parameter
  ranges; ConScenD derived concrete ALKS scenarios from highD in OpenSCENARIO/OpenDRIVE (web).
- **Euro NCAP**: test protocols (AEB Car-to-Car: CCRs, CCRm, CCRb, CCFtap; VRU: CPNA, CPFA,
  CBLA, ...) as parameter grids with speed and overlap steps and a points-based assessment;
  Euro NCAP publishes parameterised OpenSCENARIO files for its virtual testing protocol, and
  Vector's `OSC-NCAP-scenarios` repo models them in OpenSCENARIO XML (web). Protocol versions
  change yearly (4.2 in June 2023, 4.3.1 in February 2024), so the version is part of a result.
- **ISO 34502:2022** (scenario-based safety evaluation: from ODD to risk factors to critical
  scenarios to coverage and evaluation), **ISO 34503:2023** (ODD taxonomy and definition
  format), ISO 34501 (vocabulary), 34504 (scenario categorisation). (web)
- **PEGASUS** (German project, 2016-2019): six-layer scenario model, functional / logical /
  concrete scenarios, logical scenarios as parameter spaces sampled for concrete tests; the
  origin of OpenSCENARIO's parameter distribution idea.
- **Safety Pool Scenario Database** (WMG Warwick): 250,000+ scenarios, tagged by ODD,
  behaviour and admin dimensions with ASAM OpenLABEL tags (ODD tags from BSI PAS 1883),
  libraries such as ALKS, exported as OpenSCENARIO 1.1 and OpenDRIVE 1.6, credit-based access
  (web).
- **ASAM OpenODD 1.0.0** (April 2025): machine-readable ODD in YAML, tables or OSC DSL; ISO
  34503 attributes; include/exclude rules (web). **ASAM OpenLABEL**: tags for scenarios and
  labels for sensor data.
- **Take-away**: assessors think in logical scenarios (a parameter space), tags (ODD,
  behaviour) and protocol versions. A scenario catalog an auditor brings will be
  OpenSCENARIO XML with distribution files and OpenLABEL-like tags, and a result must name the
  protocol version it was scored against.

## 2. Patterns swb should adopt

1. **One declared experiment file, overridable per run** (nuPlan's Hydra, esmini's config
   YAML, Leaderboard's env vars): 076's `swb.yaml` plus `name:=value` overrides is this
   pattern. Add one thing: the run writes the fully resolved file (`runs/<time>/resolved.yaml`,
   every default filled, every override applied), so a run is rerunnable from its own
   directory.
2. **Batch = scenario x distribution x repetitions, not a new swb concept**. Accept a
   `ParameterValueDistribution` file wherever a scenario is accepted (SSv2 expands it), and
   `repetitions:` for non-determinism. Record per variant the concrete parameter values, as
   SSv2's testcase names partly do.
3. **Expected outcome next to the scenario** (SSv2 workflow `Expect:`): a scenario that should
   fail (a negative test) is listed as such in `swb.yaml`, so "passed" means "met its
   expectation".
4. **Fixed metric vocabulary with stable names** (Leaderboard infractions, nuPlan metrics):
   swb already measures contacts (`physical_verdict`), the gap oracle and incidents. Give them
   fixed keys (`collision_vehicle`, `collision_pedestrian`, `collision_static`,
   `undeclared_contact`, `red_light`, `off_lane`, `stuck`, `timeout`, `route_completion`) and
   events with sim time and position, beside SSv2's pass/fail. Keep scoring (weights) out of
   swb: an evaluator brings its own.
5. **Declared inputs, checked before the run** (Leaderboard `sensors()` against the track):
   a pilot declares what it consumes (sensors, map, ground truth or not); `project check`
   refuses a pilot whose declaration exceeds what the project allows. This is what makes
   "track" comparisons fair.
6. **Content-addressed assets** (HF commit hashes, SimReady package manifest and validation):
   `swb.yaml` may say `@v2026.11`, but `assets fetch` writes a lock file
   (`swb.lock`: dataset, tag, resolved commit, per-file sha256) and runs read only the lock.
   `assets check` against the lock. Mirrors stay possible because the lock names content, not
   a URL (SVL's lesson).
7. **Per-pilot process and domain isolation** (Leaderboard containers, nuPlan Docker
   submission): 076's one ROS domain per pilot is the right start; make the pilot boundary a
   container-able one, i.e. a pilot needs nothing from the host but the relay address, the
   simulator endpoint and the map dir (all already in A5's contract).
8. **Resume** (Leaderboard `--resume`): `run_suite` should continue a batch from its
   `suite.json` after a crash, skipping finished variants.
9. **Tags on scenarios** (Safety Pool, OpenLABEL, OpenODD): allow scenario tags (ODD,
   behaviour, protocol and version) in the project or in the scenario's `FileHeader`
   properties, carried into results, so an auditor can aggregate by tag.

## 3. Conflicts with the current design

- **Everything is a ROS command** (076 decision 2) vs evaluators, which drive runs from
  CI, Python or a web service. Not fatal: `ros2 run swb_launch run_suite` is scriptable, but
  an evaluator needs a stable exit-code contract and a machine-readable result, not log
  scraping. Document exit codes (0 all passed, 1 some failed, 2 infrastructure error, 3
  refused config) and keep them stable.
- **Long-lived stack singleton** (`just run` stack outliving sessions, respawned ego) vs
  per-run isolation. An evaluator wants each run to start from a known state; swb's stack is
  shared across scenarios and was once found serving runs 44 hours later with stale
  binaries. For audit runs, the entry launch's `end_with_scenario:=true` must be the default,
  and every run must record the stack's build identity (below), so a stale stack is visible
  in the result rather than in a postmortem.
- **Host-wide resources** (one CARLA per port, flock, fixed web ports, domain 0-57 limit,
  GPU) vs many concurrent runs. Parallelism is one CARLA (and one GPU slice) per run; the
  domain allocator in A6 must take a base from the environment so two runs on one host do not
  collide, and the 58-domain ceiling (Cyclone DomainGain) limits pilots x parallel runs.
- **Determinism**: CARLA synchronous mode is repeatable only per machine and GPU, Autoware
  is multi-process and non-deterministic, the relay and agent run on wall clock in places.
  swb cannot promise bit-identical reruns; it should promise "same inputs" (provenance) and
  report variance over `repetitions`. Saying otherwise would mislead an auditor.
- **SSv2's pass/fail is the only verdict today**; Leaderboard-style partial credit (route
  completion, infraction counts) needs swb's own metrics as above. SSv2's JUnit cannot carry
  them without abuse of `system-out`, so a second file is needed.
- **No-shims rule** vs evaluator stability: renaming an argument breaks a third party's
  pipeline. Keep the rule for users, but version the result schema and the run API
  explicitly (`schema: swb.result/1`) so renames happen across a version bump with a
  changelog, not silently.
- **Pilot kinds include `carla_basic` and ground-truth `simple`** which see the world, while
  `autoware` sees sensors: results from different kinds are not comparable unless the input
  class is recorded (pattern 5).

## 4. What an external audit platform needs from swb as a backend

Picture: a third party holds a catalog (OpenSCENARIO with distributions and tags) and N
autopilots; it wants a score table, reproducible and attributable.

**4.1 A run API, machine-readable.** One call: project (or catalog + pilots), output dir,
per-run overrides; returns when done with a documented exit code. Today's
`run_suite --output` is close. Needed on top: `--dry-run` / `project check --json` that lists
every variant the run would execute (scenario, parameter values, pilot per entity), and a
progress stream (JSONL events: variant started, finished, error) so a platform can schedule
and show progress without parsing logs. A plain file protocol is enough; no server.

**4.2 A result schema** (`runs/<id>/result.json`, versioned), per variant:

```yaml
schema: swb.result/1
run_id: <uuid>        # also the directory name
variant: {scenario: cut_in.xosc, parameters: {ego_speed: 13.9, gap: 12.0}, repetition: 2}
expected: success
outcome: passed | failed | error | refused | timeout
reason: "<SSv2 failure message or infrastructure error>"
ssv2_junit: scenario_test_runner/result.junit.xml
metrics: {route_completion: 0.94, min_ttc_s: 1.8, max_decel_mps2: 4.1, duration_sim_s: 41.2,
          duration_wall_s: 63.0, rtf: 0.65}
events:   # fixed vocabulary, sim time and position
  - {t: 23.4, type: collision_vehicle, other: npc_2, impulse: 812.0, x: .., y: ..}
provenance: {...}     # 4.5
artefacts: {rosbag: bag/, carla_recorder: carla.log, logs: logs/}
```

Plus `suite.json` aggregating (exists today, extend), and JUnit kept for CI tools. Scoring
(weights, driving score) stays outside, or ships as a separate, named, versioned scorer
(`leaderboard_2_1`, `euroncap_aeb_c2c_4_3_1`) that reads `result.json`.

**4.3 Determinism, honestly stated.** Synchronous stepping with a fixed step, seeded
everything swb owns (CARLA Traffic Manager, SSv2's random parts, pilots that accept a
seed via the contract: add `seed` to A5's arguments), sim time in every record, and
`repetitions` to measure what remains. Record whether a run met real-time (`rtf`, skipped
frames, the frame-budget events of `failure-and-frame-budget.md`): an audit must be able to
discard runs where the backend, not the autopilot, failed.

**4.4 Isolation.** Per run: fresh simulator world (or a verified reset), fresh pilot
processes, own ROS domains, own output directory, no state carried between runs unless
declared (the restart ladder in `run_suite` records resets: keep that in the result). Per
pilot: the A5 contract as the only coupling, so a pilot can be a container image
(`kind: container`, `image: ...@sha256:...`, run with host networking or a mapped relay
port and the map dir mounted read-only). The Leaderboard and nuPlan both required this for
submissions; an auditor running autopilots it did not build will too. Infrastructure
errors (CARLA crash, world server restart, agent never registered) must be `error`, never
`failed`, so an autopilot is not blamed for the backend.

**4.5 Provenance**, in every `result.json`:

- swb: superproject commit and every submodule pin (`git submodule status`), dirty flag,
  the build's binary hashes (the acb stale-binary incident shows why: 0.618 vs 0.051 m/s^2),
  play_launch version.
- simulator: CARLA version (server-reported), quality level, map and its hash, fixed step;
  or Isaac Sim version and world file hash.
- assets: `swb.lock` content (dataset, commit, per-file sha256).
- vehicle: `spec_hash` (already enforced end to end; surface it).
- pilot per entity: kind, package versions or image digest, its config file hash, input
  class (sensors / map / ground truth); for Autoware, the release plus the overlay pins
  (`scripts/autoware_overlay.bash`) and TensorRT version.
- scenario: file sha256, distribution file sha256, concrete parameters, tags, protocol
  version if any.
- host: CPU, GPU, driver, kernel, load at start (load average 68 once explained a slow run).

**4.6 Containerization of the backend.** Platform operators will want swb itself as an image
(CARLA server image + swb image with ROS, Autoware optional) so they can scale runs on a
cluster. Not needed for 076, but the project directory must not assume host paths (076 A4
already removes them), and the cache (`~/.cache/swb`) must be relocatable by env.

## 5. Recommendations for 076 (ordered)

1. Add `swb.lock` (resolved HF commits + sha256) written by `assets fetch` and read by runs;
   tags in `swb.yaml` are a request, the lock is the truth.
2. Add a versioned `result.json` per variant with fixed metric and event names and a full
   provenance block, beside SSv2's JUnit; keep scoring in separate named scorers.
3. Let `scenario:` take a list, a directory, or an OpenSCENARIO distribution file, with
   `expect:` and `repetitions:`; reuse SSv2's expansion, do not invent a sweep syntax.
4. Extend the pilot contract with `seed` and a declared input class, and add `kind:
   container` (image digest) on top of `launch`/`command`, so third-party autopilots plug in
   without building inside swb's workspace.
5. Make every run self-contained: resolved config written into `runs/<id>/`, the stack's build
   identity checked against the source before the run, infrastructure failures reported as
   `error`, stable exit codes, a JSONL progress stream, and `--resume` for batches.

## Sources

- CARLA Leaderboard 2.1 evaluation criteria: https://leaderboard.carla.org/evaluation_v2_1/
- CARLA Leaderboard 2.0 evaluation criteria: https://leaderboard.carla.org/evaluation_v2_0
- CARLA Leaderboard get started 2.1: https://leaderboard.carla.org/get_started_v2_1/
- CARLA Leaderboard get started 1.0: https://leaderboard.carla.org/get_started_v1_0/
- CARLA Leaderboard submission 2.1: https://leaderboard.carla.org/submit_v2_1/
- CARLA leaderboard repo: https://github.com/carla-simulator/leaderboard
- Leaderboard baseline agents (docker per agent): https://github.com/carla-simulator/leaderboard-agents
- ScenarioRunner agent evaluation: https://scenario-runner.readthedocs.io/en/latest/agent_evaluation/
- ScenarioRunner OpenSCENARIO support: https://scenario-runner.readthedocs.io/en/latest/openscenario_support/
- SSv2 result format: https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/SimulationResultFormat
- SSv2 scenario_test_runner: https://tier4.github.io/scenario_simulator_v2-docs/user_guide/scenario_test_runner/ScenarioTestRunner
- SSv2 random test runner quick start: https://tier4.github.io/scenario_simulator_v2-docs/user_guide/random_test_runner/QuickStart
- Autoware planning evaluation using scenarios (Evaluator): https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/others/planning-evaluation-using-scenarios
- TIER IV L4 V&V / Web.Auto: https://services.tier4.jp/l4vv
- ODD WG issue on Evaluator access: https://gitlab.com/autowarefoundation/autoware-foundation/-/issues/196
- driving_log_replayer_v2 scenario format: https://tier4.github.io/driving_log_replayer_v2/scenario_format/
- driving_log_replayer result format: https://tier4.github.io/driving_log_replayer/result_format
- driving_log_replayer_v2 command: https://tier4.github.io/driving_log_replayer_v2/overview/command
- AWSIM Labs, using OpenSCENARIO: https://autowarefoundation.github.io/AWSIM-Labs/main/GettingStarted/UsingOpenSCENARIO
- ASAM OpenSCENARIO XML parameter distribution file: https://openscenario.asam.net/ASAM_OpenSCENARIO_XML/latest/09_reuse_mechanisms/09_03_parameter_distribution.html
- ParameterValueDistribution class: https://openscenario.asam.net/ASAM_OpenSCENARIO_XML/latest/generated/content/ParameterValueDistribution.html
- OSC DSL parameter distributions annex: https://openscenario.asam.net/ASAM_OpenSCENARIO_DSL/latest/annexes/parameter_distribution.html
- OSC 2.0 migration, parameter distributions: https://www.asam.net/static_downloads/public/asam-openscenario/2.0.0/migration/mg_parameter_distribution.html
- esmini command reference: https://esmini.github.io/command-reference.html
- Foretify platform: https://foretellix.com/?p=19774 ; V-Suites: https://foretellix.com/?p=23292 ; OSC DSL at Foretellix: https://www.foretellix.com/openscenario-2-0/
- RoadRunner Scenario MATLAB functions: https://www.mathworks.com/help/roadrunner-scenario/matlab-functions-for-scenarios.html
- Simulate a RoadRunner scenario using MATLAB: https://www.mathworks.com/help/driving/ug/simulate-roadrunner-scenario-using-matlab.html
- Euro NCAP test scenarios 2026 (MathWorks): https://www.mathworks.com/help/driving/ug/euroncap-test-scenarios-2026.html
- Vector OSC NCAP scenarios: https://awesome.ecosyste.ms/projects/github.com%2Fvectorgrp%2FOSC-NCAP-scenarios
- Euro NCAP AEB C2C protocol 3.0.2: https://cdn.euroncap.com/media/56143/euro-ncap-aeb-c2c-test-protocol-v302.pdf
- Euro NCAP virtual testing supporting protocol: https://euroncap.com/media/85832/euro-ncap-supporting-protocol-safe-driving-crash-avoidance-virtual-testing-v10.pdf
- Apollo simulation platform: https://apollo.auto/platform/simulation.html
- Apollo scenario editor: https://daobook.github.io/apollo/docs/howto/how_to_leverage_scenario_editor.html
- nuPlan paper: https://arxiv.org/abs/2106.11810 ; closed-loop score breakdown: https://arxiv.org/html/2306.07962v1
- nuPlan devkit: https://github.com/motional/nuplan-devkit
- Waymax: https://waymo.com/research/waymax/
- ScenarioNet: https://arxiv.org/pdf/2306.12241 ; docs: https://scenarionet.readthedocs.io/en/latest/example.html
- MetaDrive: https://github.com/metadriverse/metadrive
- BARK: https://arxiv.org/pdf/2003.02604 ; runtime docs: https://bark-simulator.readthedocs.io/en/latest/runtime.html
- highway-env: https://highway-env.farama.org/
- SVL Python API (archived): https://github.com/lgsvl/PythonAPI
- SimReady Foundation: https://nvidia.github.io/simready-foundation/2026.08.0/guides/getting_started.html
- SimReady validation workflow: https://nvidia.github.io/simready-foundation/2026.06.0/guides/validate_workflow.html
- simready-package: https://pypi.org/project/simready-package/
- Isaac Sim asset validation: https://docs.isaacsim.omniverse.nvidia.com/6.2.0/robot_setup/asset_validation.html
- Hugging Face Hub downloads (revision, cache): https://huggingface.co/docs/huggingface_hub/guides/download
- Safety Pool scenarios: https://safetypooldb.document360.io/docs/scenarios ; Warwick release: https://warwick.ac.uk/news/pressreleases/worlds_largest_scenario
- ConScenD (R157 ALKS concrete scenarios): https://arxiv.org/pdf/2103.09772
- ISO 34502 fact sheet: https://www.prostep.org/mediathek/fact-sheets/iso-34502-test-scenarios-for-automated-driving-systems-scenario-based-safety-evaluation-framework
- ISO 34503:2023: https://standards.iteh.ai/catalog/standards/iso/62985aaa-27f4-4e00-b868-4361ee5b14c7/iso-34503-2023
- ASAM OpenODD: https://asam.net/standards/detail/openodd/
- PEGASUS project: https://www.pegasusprojekt.de/en/
- UN R157: https://unece.org/transport/documents/2021/03/standards/un-regulation-no-157-automated-lane-keeping-systems-alks
