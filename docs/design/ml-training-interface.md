# Building blocks for learned policies: run, sample, batch, evaluate (draft)

Status: draft for review, revision 2, 2026-11-20. Not a roadmap phase yet.

Source: a RoboRacer team (TUM) gave feedback after a scensim presentation on 2026-11-20. They
train and deploy RL and ML policies and know simulation well. Their pain points:

- training seeds many simulation runs, and each world is set up by hand;
- data crawls between the autopilot, the simulator and the training infrastructure;
- they need ROS 2 Jazzy as well as Humble.

Revision 2 follows the user's review of the first draft:

- no special-purpose `training:` section: randomness lives in the scenario file and in the
  autopilot;
- the design is organised as layers of application;
- batching uses compiled scenarios (option b);
- evaluation is covered as well as training: real target platforms, several actors on
  different ML stacks, vision under varied scenes, V2X;
- scensim provides building blocks that users combine; it does not cover every use case.

## 1. Decisions

| # | Decision | From |
|---|---|---|
| D1 | scensim provides building blocks and interfaces; users compose them. No learner, experiment tracker, model compiler or perception metric library of our own | user |
| D2 | Adopt existing technology where it fits: Isaac Lab's batched simulation, NITROS, NCCL, TensorRT, perception_eval, CARLA 1.0's V2X sensors | user |
| D3 | No patches to official ROS; Humble and Jazzy | user |
| D4 | No training-specific specification. A scenario file carries its own sampling (OpenSCENARIO `ParameterValueDistribution`, which SSv2 already interprets); the autopilot carries its own randomness, seeded per run | user |
| D5 | Three layers: L1 simulator plus autopilot; L2 one sampled run of a scenario; L3 many managed runs | user |
| D6 | L3's batched executor compiles scenarios into a batched simulation (Isaac Lab style). Scenarios outside the compilable subset run one per seat, as today | user (option b) |
| D7 | One interface across simulators: Isaac Sim and CARLA 1.0 (and 0.9.16) behind the same blocks, each at its own throughput | user |
| D8 | Deployment and evaluation on the real target: the user compiles the model; scensim runs it through the pilot interface, on the host or on a board | user |
| D9 | Gradients stay in the learner; NCCL between GPUs is the user's. The reward is the learner's: scensim streams observations and events, not rewards | finding, and D1 |
| D10 | V2X is optional and attaches as a block; nothing requires it | user (2026-11-15 rule) |

## 2. The layers

```mermaid
flowchart TB
  classDef l1 fill:#EAF1FE,stroke:#2F6FC9,color:#222
  classDef l2 fill:#E8F5E9,stroke:#2E7D32,color:#222
  classDef l3 fill:#F3ECFA,stroke:#7A3FB0,color:#222
  classDef user fill:#FFF3E0,stroke:#A85F00,color:#222

  subgraph L3["L3: many managed runs"]
    PLAN["sampling plan<br/>suite: distribution, N, seed,<br/>adaptive sampler"]:::l3
    EXEC["executor<br/>seats (one run per stack)<br/>or batch (compiled, many per process)"]:::l3
    DATA["data plane<br/>in-process tensors, GPU side channel,<br/>NITROS, ROS"]:::l3
    CONS["consumers<br/>report, dataset recorder,<br/>learner, dashboard"]:::user
    PLAN --> EXEC --> DATA --> CONS
  end

  subgraph L2["L2: one sampled run"]
    SCN["scenario file<br/>parameters + distribution + seed"]:::l2
    RUN["run<br/>sample, seed to world and pilots,<br/>judges, trace, provenance"]:::l2
    SCN --> RUN
  end

  subgraph L1["L1: simulator plus autopilot"]
    W["world<br/>field, car from spec,<br/>sensors, clock"]:::l1
    P["pilots<br/>Autoware, E2E model, RL policy,<br/>board, own"]:::l1
    W <--> P
  end

  EXEC -->|"runs samples"| RUN
  RUN -->|"sets up"| W
```

**L1, simulator plus autopilot.** A field, a car and one or more pilots, with no scenario.
The world steps under a clock (lockstep, or paced to wall time for a board), publishes
sensors and accepts commands. This is what `just run` does today, and a policy developer
uses it as a single environment.

**L2, one sampled run.**

- A scenario file states the setup, the actors, their pilots and the sampling: parameter
  ranges and distributions, and the number of runs (SSv2's `Stochastic` with `numberOfTestRuns`
  and `randomSeed`, or `Deterministic` sets and ranges).
- One run is one sample: a concrete parameter set plus a run seed. The run seed reaches the
  scenario sample, the simulator's own randomness (traffic, sensor noise, weather variation),
  and each pilot (as `{seed}` in its arguments, or as an SSv2 controller property).
- The run writes `result.json`, judge verdicts, a trace and its provenance, which names the
  sample. Replaying a sample reproduces the run.

**L3, many runs.**

- *The sampling plan* is a suite: which scenarios and distributions, how many runs, a base seed,
  and optionally a sampler that picks the next sample from earlier results (as in the gap
  campaign).
- *The executor* places runs. `seats` runs each sample on its own stack, as today, with full
  scenario semantics. `batch` compiles samples into one batched simulation (section 5).
- *The data plane* moves sensor data and events to the consumers.
- *The consumers* are the user's: a report, a dataset recorder, a learner. Training is L3 with
  a learner as consumer and a learning policy as pilot; it needs no section of its own.

Rollouts: when a run inside a batch ends, the executor draws the next sample from the same
plan and resets that slot. The scenario file needs no new syntax for it.

## 3. The blocks

Each block has one job and a stable interface. Users compose them in a testbench.

| Block | What it does | Exists today | New |
|---|---|---|---|
| **World** | a simulator behind the world protocol; field, weather, lights, time | world servers (CARLA, Isaac), `set_weather` | scene parameters settable from a scenario sample (section 6.3) |
| **Car** | the car from one spec, per simulator body | spec v2, bodies, rigs | rig from spec on Isaac (091 W5) |
| **Pilot** | anything that drives an entity: Autoware, a launch file, a program, an exported model, a board | pilots of six kinds, agent protocol, relay | the *model pilot* (an engine runner), the *board pilot* (section 6.1) |
| **Sensor stream** | sensors from the car's spec to a pilot | vehicle bridges publish ROS topics | GPU tensors in process; a GPU side channel or NITROS between processes; a compressed stream to a board |
| **Truth stream** | ground truth per step: objects with boxes and occlusion, lanes, signals, contacts, route progress | parts, in judges and traces | one published stream, per step, which any block can read |
| **Clock** | how time advances | lockstep in simulated time | paced mode for boards (simulated time held to wall time, with the lag recorded) |
| **Scenario** | setup, actors, sampling | SSv2, distributions, suites | the compilable subset for `batch` |
| **Judge** | a verdict or metric from a run | judges, `result.json` | adapters for third-party scorers (perception_eval, Bench2Drive-style driving score) |
| **Attachment** | any extra process tied to a run's lifetime, clock and seed: a recorder, an evaluator, a V2X stack, a network simulator | the tick gate's followers, the episode files, the gap trace | a declared attachment with the entity-to-role map and start and stop per run (section 6.4) |
| **Executor** | places runs | seats, suites, retries | `batch` (section 5) |
| **Record** | provenance, traces, datasets | runs layout, provenance, logs | a dataset writer over the sensor and truth streams |

## 4. Real evaluation pipelines, and the blocks they use

The examples were read for the blocks they need. The gaps they expose are work items.

| Pipeline | What it is | Blocks | Gap |
|---|---|---|---|
| **Bench2Drive** ([NeurIPS 2024](https://arxiv.org/abs/2406.03877)) | closed-loop evaluation of end-to-end models on CARLA: 44 interactive scenarios across locations and weathers, 220 routes, a driving score and success rate per ability | scenario sampling over town and weather, model pilot, judge | a driving-score judge adapter; weather and location as sample parameters |
| **CARLA Leaderboard and Roach** ([ICCV 2021](https://arxiv.org/abs/2108.08265)) | route-based evaluation of a sensor agent; an RL expert trained on a bird's-eye view, then imitated | pilot, truth stream (BEV), record (data for imitation) | BEV rendered from the truth stream |
| **driving_log_replayer v2** ([docs](https://tier4.github.io/driving_log_replayer_v2/overview)) | Autoware's evaluator: perception scored against annotations by perception_eval, or without annotations from online metrics; planning and control from evaluator metric topics | judge adapter, truth stream as the annotation | perception scoring against simulated truth: perception_eval reading our truth stream |
| **OpenCDA** ([arXiv 2107.06260](https://arxiv.org/abs/2107.06260)) | cooperative driving on CARLA with SUMO traffic: V2X, cooperative perception, platooning; collects V2X perception data (OPV2V) | attachment (V2X), truth stream, record | V2X stacks and their network simulators attach; non-commercial licence, so a reference, not a dependency |
| **Board in the loop** (e.g. [MMRHP](https://arxiv.org/html/2510.18371v1); our NXP board with SSv2) | the stack on the target board; sensors injected; latency measured; Autoware's tracking broke past about 40 ms of injected latency in that study | board pilot, paced clock, sensor stream to the board | the paced clock; a latency record per command; sensor compression |
| **Wheeled Lab** ([CoRL 2025](https://arxiv.org/abs/2502.07380)) and **RoboRacer Arena** ([arXiv 2608.23040](https://arxiv.org/pdf/2608.23040)) | small cars trained on Isaac Lab or Isaac Sim with heavy randomisation; residual RL over Pure Pursuit; deployed zero-shot | batch executor, scenario sampling, model pilot | residual actions over a named base controller; observation history recorded with the model |
| **GPUDrive** ([ICLR 2025](https://arxiv.org/html/2408.01584v3)) | multi-agent driving on recorded scenes, about a million steps per second | batch executor | several learning actors per environment (later) |

## 5. The batch executor (option b)

The executor compiles a scenario sample into a slot of a batched simulation: N clones of the
field in one Isaac process, stepped together on the GPU (Isaac Lab's model).

- **The subset that compiles.** Start poses, routes, speeds, NPC routes with constant or
  profiled speed, simple distance and time triggers, weather and scene parameters (section
  6.3), and the end conditions. A scenario outside the subset is refused for `batch` by name,
  with the first construct that does not compile, and runs on `seats` instead.
- **Pilots in a batch.** A batched pilot steps all slots at once: an RL policy or an exported
  model on the GPU. Autoware and other per-process stacks do not batch; they run on `seats`.
- **The interface for learners.** A batched run is also a Gymnasium vector environment:
  observations as tensors (DLPack across PyTorch, JAX, Warp), actions in, per-step events out
  (contacts, route progress, judge states, episode ends). The learner computes its reward
  from them (D9).
- **Same car, same component.** The rig comes from the car's spec, and the 091 control
  component runs inside the physics step, in batched form.
- **Provenance per slot.** Each slot records its sample and seed, so any slot's episode can be
  replayed as an L2 run on `seats`, with full SSv2 semantics. That is how a surprising training
  episode becomes a test.

## 6. Evaluation building blocks in detail

### 6.1 The real target platform

A *board pilot* runs the stack on the target (a Jetson, an NXP board) and talks to the run over
the network, through the agent protocol's relay, which already crosses domains.

- **Clock.** A board runs in wall time. The paced clock holds simulated time to wall time and
  records the lag. A run whose lag exceeds its budget is marked as such, not silently
  stretched.
- **Sensors.** Sensors are encoded for the link (compressed images, decimated lidar) at rates
  the board declares.
- **Latency.** Every command is stamped, so latency per command, from sensor to actuation, is
  part of the record.

### 6.2 Several actors on different ML stacks

A scenario names a pilot per entity: Autoware for one car, an end-to-end vision model for
another, an RL policy for a third, a board for a fourth.

- **Seats.** Seats isolate each pilot's ROS domain, as today (`multi_av`).
- **Sensors.** Each pilot gets the sensors of its own car's spec.
- **Judges.** Judges score each actor separately, and the run records which pilot drove which
  entity.
- **Transport.** Where pilots on one host consume the same camera data, they share it on the
  GPU (NITROS or the side channel) rather than each taking a copy.

### 6.3 Vision under varied scenes

Scene parameters become scenario parameters, so a distribution sweeps them. They cover
weather, sun angle and time of day, wet roads, fog, lighting presets, props and surfaces, and,
on Isaac, variants of the authored stage.

- **How they reach the world.** The world server applies them at the start of a run (the
  weather path exists).
- **Two kinds of scoring.**
  - *Closed-loop:* the run's judges.
  - *Perception:* the pilot's detections against the truth stream, through perception_eval, the
    scorer Autoware's own evaluator uses.

### 6.4 V2X, as an attachment

An attachment is a process declared in the testbench and tied to a run. It receives the run's
seed, the entity-to-role map, and start and stop at the run's edges. It follows the run's clock
as a follower of the world server's tick gate (`SCENSIM_TICK_GATE`, which already takes
several followers). It reads the truth stream and writes into the run's record. Nothing in a
run requires V2X, and a scenario with V2X also runs without it.

A neighbouring lab project gives the requirements. It runs shared perception and junction
space leases over a simulated 5G network, which ns-3 5G-LENA and a ray tracer model.

- **What it exchanges.** Each car runs a node that reads its own stack's state (pose, tracked
  objects) and UWB ranges, and sends its own packets every 100 ms (state, tracks, leases), with
  evidence patches every 200 ms. The packets travel over Zenoh through the network simulator.
  A roadside node grants leases from the junction's lights.
- **What it needs from the simulator.** Every vehicle's true pose each frame, by role name, to
  drive the network simulator's mobility and line of sight. True ranges for the UWB model.
  Truth logged per run for scoring. The cars' own state still comes from each stack, never
  from truth.
- **Clock.** Simulated time in lockstep, with the network simulator, the junction coordinator
  and the ray tracer as tick-gate followers. Zenoh's keep-alives and TCP still run on wall
  time, so the record states it.
- **Lifetime.** The nodes start fresh per run and stop after the verdict. The network
  simulator and gateways live for a lab (one town). Seeds go to the network simulator and the
  UWB noise.
- **What it asks of scensim**, in its order:
  1. the attachment hook (entity-to-role map, start and stop per run);
  2. the tick gate with several followers, kept as it is;
  3. per-run recorded truth (poses, actor ids, frame times) in the run directory.

  It needs no channel model from us. CARLA 1.0's V2X sensor would serve only as a quick
  baseline.

So the attachment block (B6) and the truth stream (B3) cover V2X. A channel model is not a
scensim block.

## 7. Transport

| Data | Between | Mechanism |
|---|---|---|
| observations, actions, events in a batch | within one process | tensors; nothing to transport |
| gradients and parameters | the user's learners | NCCL |
| camera frames to a pilot process on the same host | process to process | NITROS where the stack uses Isaac ROS (3.2 on Humble, 4.x on Jazzy); a CUDA IPC side channel otherwise; ROS topics as the fallback |
| sensors to a board | host to board | compressed stream over the relay |
| commands, states, scenario control | everywhere | ROS topics and the world protocol |

Notes:

- **NITROS.** It is zero-copy only within one process. Between processes it uses the NITROS
  Bridge. Isaac ROS 4.5 dropped the GXF implementation, so the 4.x API has to be pinned.
- **ROS Lyrical.** Lyrical's `rosidl::Buffer` with its CUDA backend does the same job
  upstream. It is not in Humble or Jazzy, and D3 rules out backporting it. When a distro has
  it, our side channel steps aside.

## 8. Not in scope

- learners, RL algorithms and experiment tracking;
- model export and compilation;
- our own perception metrics (perception_eval and others are adopted);
- differentiable simulation;
- patching ROS;
- a V2X stack or a channel model of our own.

## 9. Open questions

1. **Isaac Lab and our stage.** Which Isaac Lab release fits Isaac Sim 6.x, and can it clone an
   authored stage with our session layer (091 W13)?
2. **The compilable subset.** Start poses and NPC routes first; which triggers next?
3. **The batched control component.** A vectorised `step` over slots: a second method, or a
   second component?
4. **The paced clock's budget.** How much lag may a board run carry before its verdict is void?
5. **The truth stream's form.** Is it a ROS topic set, the world protocol, or both? It must be
   cheap enough to publish every step.
6. **Attachments that outlive a run.** A network simulator lives for a lab (one town), its
   nodes for a run: does an attachment declare its lifetime (run, lab), as pilots do?

## 10. Work items (sketch, for a roadmap phase once approved)

| Item | What | Layer | Depends on |
|---|---|---|---|
| B1 | Seeds end to end: run seed to sample, world and pilots; provenance names the sample; replay a sample | L2 | none |
| B2 | Scene parameters as scenario parameters (weather, light, surfaces, stage variants) | L2 | B1 |
| B3 | The truth stream | L1 | none |
| B4 | The model pilot (TensorRT, ONNX Runtime) | L1 | none |
| B5 | Judge adapters: perception_eval against the truth stream; a driving-score judge | L2 | B3 |
| B6 | Attachments: declaration, lifetime, clock, seed, record | L2 | B1 |
| B7 | The board pilot and the paced clock, with latency records | L1 | B4 |
| B8 | The batch executor: compiled subset, slots, rollouts, vector interface, replay on seats | L3 | B1, 091 W5 |
| B9 | GPU transport between processes: NITROS profile, CUDA IPC side channel | L3 | B4 |
| B10 | Jazzy for the whole set (with 082 step 7) | all | 082 |
| B11 | Worked examples: a RoboRacer policy trained in a batch and evaluated on seats; a vision stack swept over weather with perception scored; two stacks in one scenario; V2X attached | all | the above |
