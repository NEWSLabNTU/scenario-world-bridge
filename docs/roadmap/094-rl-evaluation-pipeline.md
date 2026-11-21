# Phase 094: A real RL pipeline, measured

Train real policies end to end on phase 092's components, and measure two things: the
**throughput** of the pipeline, and the **learning efficiency** of training on it. Then
evaluate the trained policies on seats, where an autopilot is evaluated today. The pipeline
is built from a working reference project, not invented.

**Status**: planned 2026-11-21 (user: "After you finish the infra, build and run a real RL
pipeline for evaluation to measure the throughput and the RL learning efficiency. I suggest
to find a working project or take it for ref."); revised the same day to run stock
WheeledLab and ours side by side (user: "run both stock WheelLab and ours for comparison.
Measure the efficiency on both sides."). Starts when 092's K2 (the Isaac Lab adapter)
reaches K2.2; steps P0 and P1 need no scensim component and start earlier.

**Depends on**: 092 S1 (sample record), S2 (compilable subset), S3 (batched control
component), S4/K4 (truth stream), K1 (sampler), K2 (Isaac Lab adapter), K3 (replay), K7
(coverage report); 091's RoboRacer field and car.

## References

| Project | What it is | What we take |
|---|---|---|
| [WheeledLab](https://github.com/UWRobotLearning/WheeledLab) ([arXiv 2502.07380](https://arxiv.org/abs/2502.07380)) | Isaac Lab tasks for 1/10 wheeled robots (MuSHR, and an F1TENTH drift task without a published curve), drifting, elevation and visual navigation, trained massively parallel and run on real cars | **the reference pipeline**: the stock side of every comparison: its tasks, rewards, PPO settings and learning curves, ported to Isaac Lab 3.0.0 / Isaac Sim 6.2, against the same task built on our components |
| [CaRL](https://github.com/autonomousvision/CaRL) ([arXiv 2504.17838](https://arxiv.org/abs/2504.17838), CoRL 2025) | PPO planning policies in CARLA and nuPlan with a reward built on route completion alone, scaled to 300 M samples on one 8-GPU node | **the driving reward and the evaluation discipline**: route completion with multiplicative infraction penalties, which keeps PPO stable at large mini-batches; scores on a fixed route benchmark |
| [GPUDrive](https://arxiv.org/abs/2408.01584) (ICLR 2025) | multi-agent driving on Madrona, over 1 M simulator steps/s; PPO at about 30 M steps of experience per hour on a consumer GPU | **a throughput reference** for what training (not only simulation) reaches on one GPU |
| [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) | CPU lidar racing environment with PPO baselines | **a low-fidelity baseline**: the same lidar task on a 2D simulator, for the fidelity/throughput trade-off |

## Two sides, measured the same way

Every task runs on **both sides**, and both are measured with the same harness:

- **Stock**: WheeledLab with the smallest port to Isaac Lab 3.0.0 and Isaac Sim 6.2, on the
  NEWSLabNTU fork (branch `isaaclab-3.0`, upstream's base commit named). Its unmodified form
  was dropped on 2026-11-21: Isaac Sim 4.5 does not support the RTX 5090, and Isaac Sim 5.0 and
  5.1 crash at renderer start on driver 610 (241), so the only host able to run it was 139, and
  the user chose to move every host to 6.2 and remove 5.x instead (user: "switch to Isaac Sim
  6.2 on all agents. Remove old 6.1 and 5.0"). The port's every change is listed in the fork,
  so the version's effect is described, not measured.
- **Ours**: the same task built from scensim's components: the car as a spec and its rig,
  the field, the episode as a compiled scenario with sampled records, the control component
  through the batched form, truth per slot, through K2's environment. Rewards, observations,
  actions and the PPO settings are copied from stock.

Fairness rules, for every comparison:

- the same host and GPU, checked idle of other jobs before each run; the same driver;
- the same number of environments, iterations, seeds, PPO settings and network;
- logging reduced to the same scalars on both sides; the first iterations (warm-up,
  compilation) reported apart, not in the rates;
- the car's values equal on both sides: ours uses a spec written from stock's MuSHR asset
  (geometry, mass, wheels, actuators), and any value stock lacks is named, not invented;
- runs alternated between sides, not all of one then all of the other.

Which engine "ours" runs on shapes every rate comparison (092 K2.2, batch-executor.md section
5). Engine A, N copies of aib's PhysX vehicle (seats' car), is CPU physics by nature: PhysX's
vehicle constraints are not GPU-compatible, and its direct-GPU pipeline refuses them. Only
engine B, an articulation car like WheeledLab's, runs on the GPU pipeline. So a throughput
comparison with stock is B against stock; A is reported as what seats fidelity costs, never
as ours' rate.

## Measures

The same on both sides, from one harness (P0):

- **Throughput**: simulator steps/s, and training samples/s end to end, split by stage
  (physics, control component, scenario conditions, resets, observation and transfer, policy
  inference, PPO update); at N = 256, 1024 and 4096 environments; peak GPU memory.
- **Learning efficiency**: samples and wall-clock time to reach a fixed threshold (set from
  stock's published result before training), the final return and task metric, the learning
  curve's area; at least 5 seeds per side, reported with their spread.
- **Cross-evaluation**: each side's trained policy run in the other side's environment, so a
  difference in learning can be told apart from a difference in the task.
- **Evaluation on seats**: every trained policy, from both sides, run on seats (the Isaac
  world server with SSv2, and CARLA by replay) on sampled scenarios, with the coverage
  report's per-cell rates. The gap between batch and seats is a result in itself.

Every number names its hardware, driver, releases and seeds.

## Steps

| Step | What | Owner | Status |
|---|---|---|---|
| P0 | The measurement harness: stage timers around the environment step and the runner's phases, GPU memory, and the learning curve, written to one CSV schema; applied unchanged to stock and to ours. Its own overhead measured and reported | 241 | open |
| P1 | Stock: the MuSHR drift task (rsl_rl PPO, 1024 environments, the paper's headline task), ported (the NEWSLabNTU fork, branch `isaaclab-3.0`, upstream's base commit named), trained to its published result over 5 seeds; the pass bar fixed from the paper before training. F1TENTH drift afterwards as an extra | 241 | open |
| P2 | Ours, the same task: a MuSHR spec from stock's asset, the field, the drift episode as a compiled scenario with sampled starts, the control component, truth, through K2's environment; stock's rewards and PPO settings. Trained over 5 seeds | | open |
| P3 | The comparison: throughput by stage, learning efficiency and cross-evaluation, stock (ported) and ours, with every difference in cost explained by stage | | open |
| P4 | A driving task from the case studies (for example the lead car braking, or a walker crossing) on Town01's Isaac stage, with a CaRL-style reward and the other road users on the batched behaviour. Its stock side is a hand-written Isaac Lab environment of the same task, in Isaac Lab's own style; both measured as in P3 | | open |
| P5 | Evaluation on seats: the policies from P1 to P4, both sides, exported as pilots and run on sampled scenarios on Isaac with SSv2 and on CARLA by replay; the coverage report per cell; the batch-to-seats gap | | open |
| P6 | Write-up: one table each for throughput, learning efficiency, cross-evaluation and evaluation on seats, stock against ours, with GPUDrive and f1tenth_gym as outside references; what our components cost and what they buy (identity, replay, cross-fidelity evaluation) | | open |

## Exit

Stock, ported, reproduces its published result. Ours trains the same task to
the same threshold. Throughput and learning efficiency are reported for both sides with the
same harness, and every cost difference is attributed to a stage. P4 repeats the comparison
on a driving task. P5 runs every policy on seats on both simulators with a coverage report.

## Non-goals

- A training framework of our own: training stays in Isaac Lab and a published RL library
  (rsl_rl, skrl or rl_games, whichever the reference uses).
- New algorithms. The point is to measure the pipeline, not to beat a benchmark.
