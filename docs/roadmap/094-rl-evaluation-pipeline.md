# Phase 094: A real RL pipeline, measured

Train real policies end to end on phase 092's components, and measure two things: the
**throughput** of the pipeline, and the **learning efficiency** of training on it. Then
evaluate the trained policies on seats, where an autopilot is evaluated today. The pipeline
is built from a working reference project, not invented.

**Status**: planned 2026-11-21 (user: "After you finish the infra, build and run a real RL
pipeline for evaluation to measure the throughput and the RL learning efficiency. I suggest
to find a working project or take it for ref."). Starts when 092's K2 (the Isaac Lab adapter)
reaches K2.2; step P1 needs no scensim component and may start earlier.

**Depends on**: 092 S1 (sample record), S2 (compilable subset), S3 (batched control
component), S4/K4 (truth stream), K1 (sampler), K2 (Isaac Lab adapter), K3 (replay), K7
(coverage report); 091's RoboRacer field and car.

## References

| Project | What it is | What we take |
|---|---|---|
| [WheeledLab](https://github.com/UWRobotLearning/WheeledLab) ([arXiv 2502.07380](https://arxiv.org/abs/2502.07380)) | Isaac Lab tasks for 1/10 wheeled robots (MuSHR, and an F1TENTH drift task without a published curve), drifting, elevation and visual navigation, trained massively parallel and run on real cars | **the reference pipeline**: its tasks, rewards, PPO settings and learning curves are the baseline we reproduce and then rebuild on our components. Targets Isaac Lab 2.0.2 / Isaac Sim 4.5, so it needs a port to Isaac Lab 3.0 / Isaac Sim 6.1 |
| [CaRL](https://github.com/autonomousvision/CaRL) ([arXiv 2504.17838](https://arxiv.org/abs/2504.17838), CoRL 2025) | PPO planning policies in CARLA and nuPlan with a reward built on route completion alone, scaled to 300 M samples on one 8-GPU node | **the driving reward and the evaluation discipline**: route completion with multiplicative infraction penalties, which keeps PPO stable at large mini-batches; scores on a fixed route benchmark |
| [GPUDrive](https://arxiv.org/abs/2408.01584) (ICLR 2025) | multi-agent driving on Madrona, over 1 M simulator steps/s; PPO at about 30 M steps of experience per hour on a consumer GPU | **a throughput reference** for what training (not only simulation) reaches on one GPU |
| [f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) | CPU lidar racing environment with PPO baselines | **a low-fidelity baseline**: the same lidar task on a 2D simulator, for the fidelity/throughput trade-off |

## Measures

- **Throughput**: simulator steps/s, and training samples/s end to end (policy inference,
  physics, control component, conditions, resets, transfer), each split by stage; at N = 256,
  1024 and 4096 environments; GPU memory at peak. Hardware, driver and every release named.
- **Learning efficiency**: samples and wall-clock time to reach a fixed success threshold
  (for example 90 % of the reference's final return or success rate); the final success rate;
  the learning curve's area; each over at least 5 training seeds, reported with their spread.
- **Evaluation**: the trained policy run on seats (the Isaac world server with SSv2, and CARLA
  through cross-fidelity replay) on sampled scenarios, with the coverage report's per-cell
  rates. The gap between batch success and seats success is a result in itself.

## Steps

| Step | What | Owner | Status |
|---|---|---|---|
| P1 | Reproduce the reference: upstream WheeledLab ported to Isaac Lab 3.0 / Isaac Sim 6.1 on one host, the MuSHR drift task (rsl_rl PPO, 1024 environments, its paper's headline task) trained to its published result, the pass bar fixed from the paper before training; F1TENTH drift afterwards as an extra; record its throughput and learning curve as the baseline. Any port patches stay in a NEWSLabNTU fork, not in scensim | | open |
| P2 | The same task on scensim's components: the field and car from 091 (spec, rig, the user's control component through S3), the episode from a compiled scenario with sampled records (S1, S2, K1), truth from S4, through K2's environment; rewards and the PPO settings as P1's. Compare throughput and learning efficiency with P1 at equal settings; every difference in cost explained by stage | | open |
| P3 | A driving task from the case studies: a scenario family (for example the lead car braking, or a walker crossing) trained with a CaRL-style reward on Town01's Isaac stage, the other road users on K2.1's batched behaviour; learning efficiency over seeds | | open |
| P4 | Evaluation on seats: P2's and P3's policies exported as pilots (the agent SDK), run on sampled scenarios on Isaac with SSv2 and on CARLA by replay (K3); the coverage report (K7) per cell; the batch-to-seats gap per row of S2's table | | open |
| P5 | Write-up: one table each for throughput, learning efficiency and evaluation, against the references; what our components cost and what they buy (identity, replay, cross-fidelity evaluation) | | open |

## Exit

P1 reproduces the reference within its reported spread. P2 and P3 train to the threshold,
with throughput and learning efficiency reported against P1 and the references. P4 runs the
trained policies on seats on both simulators with a coverage report. Every number names its
hardware, releases and seeds.

## Non-goals

- A training framework of our own: training stays in Isaac Lab and a published RL library
  (rsl_rl, skrl or rl_games, whichever the reference uses).
- New algorithms. The point is to measure the pipeline, not to beat a benchmark.
