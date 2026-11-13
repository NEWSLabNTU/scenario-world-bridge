# Survey: Cooperation Strategies Among Autonomous Vehicles, and What swb Can Measure

This study asks how to evaluate the ways autonomous vehicles decide among each other in
multi-vehicle traffic, with cooperation strategies in focus, and whether swb can estimate the
payoffs of those strategies and discover new ones. It looks for a method that keeps game theory's
explanations and takes learning's scale.

**Source**: user direction 2026-11-13: "Study the ways to evaluate the ways to enable vehicles to
drive autonomously in a multiple vehicle environment. Focus cooperation strategies. See if our swb
can help exploiting payoffs and discover strategies." The user's sketch: car A cuts into car B's
lane; A takes the lane now or holds, B yields or holds the lane. The 2 x 2 profile is easy, but
speed and distance matter, which makes the equilibrium hard to model; with more vehicles the
strategy space explodes. RL scales but generalises poorly and does not explain itself. The aim is
to combine theory and learning.

Literature checked by web search on 2026-11-13; claims not confirmed against a primary source are
marked UNVERIFIED.

## The problem in the user's terms

The 2 x 2 game {A: go now, wait} x {B: yield, hold} has a different payoff table at every
continuous context: the gap between the cars, their speeds, and how much road A has left. Three
things make it hard:

1. **Continuous context.** The equilibrium is not one profile but a map over context: in some
   (gap, speed difference) regions "A goes, B yields" is the only equilibrium, in others "A waits,
   B holds", and in a band between them both are equilibria (a coordination game). Most
   incidents and deadlocks live in that band.
2. **Time.** "Wait" is not one action. It is a policy (wait until the gap reaches g, then go), so
   the real strategies are contingent plans, and the one-shot table is only their summary.
3. **N vehicles.** Joint profiles grow as k^N, and an equilibrium of the whole is not the union of
   the pairwise ones.

## What the literature offers

### Game-theoretic interaction models

| approach | key works | scales to N | continuous state | explainable | needs |
|---|---|---|---|---|---|
| Stackelberg / hierarchical | Sadigh et al. RSS 2016 [1]; Fisac et al. ICRA 2019 [2] | poor (pairwise) | yes | high | a model |
| level-k / cognitive hierarchy | Li, Kolmanovsky et al. TCST 2018 [3]; Tian et al. T-ITS 2020 [4]; GameFormer ICCV 2023 [5] | fair (by level) | yes | medium-high | data or sim |
| dynamic games, local Nash / GNE | iLQGames ICRA 2020 [6]; ALGAMES RSS 2020 [7]; Wang et al. RSS 2019 [8] | about 3-10, local | yes | medium (the costs) | a differentiable model |
| potential games | Bhatt & Mehr 2022, 2023 [9]; Liu, Kolmanovsky et al. 2022 [10] | good | yes | high | near-symmetric couplings |
| mean-field games | Huang et al. ITSC 2019 [11] | very good | yes | medium | exchangeable agents |
| incomplete information | contingency games RA-L 2024 [12]; Bayesian games 2024 [13] | moderate | yes | high | intent priors |
| inverse games | Peters et al. RSS 2021 [14]; Liu, Peters, Alonso-Mora RA-L 2023 [15]; Bayesian inverse games 2024 [16] | moderate | yes | high (fitted weights) | trajectories |
| social value orientation (SVO) | Schwarting et al. PNAS 2019 [17] | any | yes | high (one angle per car) | online estimation |

Three ideas carry over directly to the sketch:

- **Fisac's split [2]**: a coarse strategic game on simple dynamics, whose value becomes the
  terminal cost of a short-horizon planner on full dynamics. Go/wait x yield/hold is the
  strategic layer; the continuous speed and gap live in the tactical layer.
- **SVO [17]**: each car weighs its own utility against the other's by one angle. It is a
  one-number driving style, so a natural strategy axis that a person can read.
- **Potential games [9, 10]**: when the interaction has a potential function, a Nash equilibrium
  is an optimum of one function, which is cheap to compute for N cars and easy to explain. The
  catch is that real couplings are only near-potential.

### Learning and theory together

- **Empirical game-theoretic analysis (EGTA)**, Wellman, Tuyls, Greenwald, JAIR 2025 [18]: build
  the game from a simulator. Choose a restricted set of strategies, estimate each profile's payoff
  from noisy simulation runs, solve the estimated game, and decide which profiles to simulate next.
  This is the role swb would play.
- **PSRO**, Lanctot et al. NeurIPS 2017 [19]: EGTA whose strategy set grows by RL best responses to
  the current equilibrium mixture. RL is only the oracle that proposes strategies; the game solver
  decides what the population plays. This answers the explosion: the set grows only by strategies
  that beat the current equilibrium.
- **alpha-Rank**, Omidshafiei et al. Sci. Rep. 2019 [20]: ranks strategies by evolutionary
  dynamics, polynomial in the number of profiles, no Nash computation. Fits ranking driving styles
  or stack configurations.
- **alpha-RACER**, Kalaria, Maheshwari, Sastry, L4DC 2025 [21]: learns a near-potential function
  offline from simulated game data, then maximises it online for an approximate Nash equilibrium
  (three-car racing). The simulator fits the payoffs, the theory gives the equilibria: the closest
  precedent for this proposal.
- **Differentiable game layers**: Geiger & Straehle AAAI 2021 [22] (a network infers preferences,
  an implicit layer solves a potential game); KLGame RSS 2024 [23] (Nash regularised toward a
  learned policy, one knob from task-optimal to human-like).
- **EGTA or PSRO for driving is thin.** The closest are a proposal [24] whose strategies are driving
  styles, a thesis on empirical game models for AD [25], and TrafficGamer [26] (a generative world
  model fine-tuned by a game-theoretic oracle). None runs production stacks.
- **LLM negotiation** (CoLMDriver ICCV 2025 [27]): readable V2V messages, no guarantee; the
  explanation is the transcript, not a model.

### Evaluating multi-vehicle interaction

- **Per-run metrics**: each car's progress and delay, safety surrogates (minimum TTC, PET, DRAC;
  no single surrogate is good on every dimension), comfort (jerk, peak deceleration), and physical
  contact. **Population metrics**: social welfare (sum of utilities), throughput, the fairness gap
  between the cars, MOBIL's politeness factor [28] as a reference style.
- **Rare-event and adversarial testing**: adaptive importance sampling against a full stack
  (O'Kelly et al. NeurIPS 2018 [29]); AdvSim [30], STRIVE [31], KING [32]; D2RL (Feng et al. Nature
  2023 [33]) trains on a densified MDP so that evaluation needs far fewer miles.
- **Benchmarks**: nuPlan, the Waymo Sim Agents challenge, Waymax (whose authors found RL
  overfitting to the simulated agents), InterSim, MetaDrive, SMARTS, Bench2Drive (CARLA, 44
  interactive scenarios) [34]. All drive policy abstractions or log replay, not several full stacks.
- **Several AV stacks meeting**: DoppelTest (ICSE 2023 [35]) puts several instances of the same
  Apollo in one scene and finds 123 violations by a genetic search; STCLocker (2025 [36]) adds a
  black-box wait-for-graph deadlock oracle. Mixed autonomy: Flow (T-RO 2022 [37]); controller
  heterogeneity (Le Maitre & Prorok 2020 [38]) found the spread of target speeds dominates.
  **No paper found pits two different full production stacks, or two configurations of one,
  against each other game-theoretically.**

## A method that combines the two

Three layers, each explainable at its own level, with learning only where theory cannot reach.

**1. Strategies are policies with readable parameters, not instantaneous actions.** A's strategy
is "merge when the gap to B exceeds g_A, at acceleration a_A"; B's is "on a cut-in signal, open
the gap (yield) or keep following distance d_B (hold)", or one SVO angle per car. Speed and distance
leave the strategy space and become the context in which strategies are evaluated. The game is then
small and fixed (a handful of styles per car), and the continuous part is the context.

**2. The payoff map is measured, not assumed (EGTA).** For each strategy profile, a surrogate
model (a Gaussian process, say) predicts each car's payoff vector as a function of the context
(gap, speed difference, road left). Simulation runs are chosen where they change the answer most:
near the boundaries where the equilibrium changes, and where the two cars' best responses
disagree. The result is an **equilibrium map** over the context, with confidence: where "A goes, B
yields" is stable, where the game is a coordination game, and where every profile ends in contact
or in a deadlock. That map is the explanation: "at a 12 m gap and 3 m/s closing, A waiting and B
holding is the only equilibrium, because..."

**3. Theory is fitted to the measurements, and the fit is tested.** Fit an interpretable model to
the measured payoffs and trajectories: SVO angles, potential-game weights (as alpha-RACER [21]),
or iLQGames cost weights by inverse game [15, 16]. Where it fits, it predicts outside the sampled
contexts, and the weights say why the cars behave as they do. Where it does not fit, the stack is
not playing any equilibrium of that family: a finding in itself (a frozen pair, a hidden priority
rule).

**N vehicles.** Do not build the joint game. (a) Decompose into the interaction graph: most cars
interact pairwise or in small cliques at any one time, so the game is a set of local games coupled
through shared cars. (b) Where the couplings are near-potential, fit one potential (alpha-RACER) and
solve N cars by one optimisation. (c) Grow the strategy population by PSRO: an RL best response,
trained in a cheap simulator, proposes a new style; it enters only if it beats the current
equilibrium mixture in the measured game, and its parameters are then fitted back to the readable
family where possible. RL's generalisation problem is checked, not assumed away: every proposed
strategy is evaluated on contexts and maps it did not train on.

## Car models, and what a car does not know about the other

User direction 2026-11-13: the method must adapt to any car model, under a practical
assumption: a car does not know the other vehicle's kinematic or dynamic properties. "The strategy
and payoff space can be general to any car models. The car model restricts the set of strategies
and outcomes."

### One general game, the car model as a constraint

- **The strategy space is the interaction's, not the car's.** A (cutting in): the gap g at which it
  starts, the merge duration T, the target speed v*. B (in the lane): the time headway h it keeps,
  its response to the cut-in (open the gap, keep it, close it) and how fast. Or, one level down,
  the speed and lateral profiles over time. The same space for a sedan, a truck and a golf cart.
- **The payoff space is the outcomes'.** Who goes first, each car's delay, minimum TTC and PET, the
  peak deceleration and jerk used, contact and its severity. Severity depends on mass and speed, so
  the outcome carries the physics and the payoff function stays one.
- **A car model is a feasible set.** Its capability theta (acceleration and braking limits, top
  speed, lateral acceleration and curvature, actuator lag, size) cuts the space: car i can execute
  only S intersected with F(theta_i), and only the outcomes those strategies reach. The golf cart
  (2.5 m/s ceiling) has no "go now" ahead of a fast B at a short gap; a truck's braking removes the
  outcomes where B yields late. A larger capability gives a superset of strategies, so the
  equilibrium regions should move predictably as theta grows; how far they move is a result.
- **F is the car's and its stack's.** The autopilot limits the car below its physics (Autoware's
  own speed, acceleration and jerk settings), so F = physical limits intersected with the stack's.
  Both are measurable.

A new car model is then a new F and one measurement of it, not a new game: the payoff model over
the general strategies is reused, and only the boundaries move.

### Incomplete information: the other car's F is a belief

A car sees the other's pose, velocity, size and class, not its theta. So the game is Bayesian with
the type being the other car's capability:

1. **Prior from what the sensors show**: a class and size give a prior over limits (a car this size
   brakes at least so hard). RSS [42] assumes such bounds as fixed worst cases, which is the
   assumption this study drops; used per class it is the conservative fallback.
2. **Belief from observed motion**: every observed acceleration is a lower bound on capability (a
   6 m/s^2 stop proves at least 6). Zhou et al. [39] estimate other cars' acceleration bounds online
   for forced merging; LUCIDGames [40] runs an unscented Kalman filter over others' parameters inside
   a game MPC, where theta could hold limits instead of costs; constraint-learning inverse games [41]
   recover constraint parameters, and capability limits are constraints.
3. **Decisions over the belief**: robust (the worst case over the belief's support), chance
   constrained (CCILQGames [43]), or confidence-aware (more conservative when the model predicts
   badly, [44]).
4. **Probing**: the cut-in is itself a probe. Nudging toward the lane reveals how B responds;
   Sadigh et al. [45] probe a driver's style so, and dual control [46] trades probing against
   performance under a safety shield. Here it would probe capability, not style.

**What the literature lacks** (follow-up search, 2026-11-13): no driving game whose Bayesian type is
the other car's dynamics or capability; types are objectives, intent, rationality or
aggressiveness. No class-aware filter (car, truck, bus, cart) with class-specific limits feeding an
interaction planner. No policy conditioned on the *other* agent's inferred dynamics (RMA [47] infers
the ego's own; DynaDreamer [48] transfers across chassis from microcars to buses, single-agent). No
quantitative study of how equilibria shift with the vehicle mix. The pieces exist apart; the
combination is open.

### Measuring the cost of not knowing

The simulator knows every car's true theta; the cars do not. swb can therefore score both what
happened and what would have happened with knowledge:

- **Misbelief regret**: J(true type | belief) - J(true type | true type), per car and run.
- **Value of information**: its expectation over the type distribution, i.e. what knowing the
  other's capability is worth, and so what a V2X capability broadcast would be worth.
- **The type-confusion matrix**: assumed class x actual class, with contact rate, minimum TTC and
  delay in each cell. No AV paper reports it; in swb it is a sweep over two catalog entries and one
  belief parameter.
- The game-to-real gap of Ferguson et al. [49] (utility obtained less utility expected under the
  agent's own model) is the same quantity in general games.

## Taming the strategy space

User direction 2026-11-13 named three ways in: (1) start from general strategy combinations over
all cars and refine the dominant ones; (2) a learned proposer of strategies, per car or for several,
validated with care because it is framed by its training data; (3) a car watches the other, whether
it slows or keeps its speed as time moves on, and updates its belief. Each fits a different layer.

### 1. Combinations, refined: exploit structure, never enumerate

The joint space is k^N only if every car's payoff depends on every other car's strategy. In
traffic it rarely does.

- **Locality: graphical games** [50]. A car's payoff depends on the cars it interacts with (its
  lane, the next lane, within a few seconds), so the game is a set of small local games coupled
  through shared cars. On a tree-shaped interaction graph an equilibrium is found in polynomial
  time. The cost grows with the largest group, not with N.
- **Anonymity: anonymous games** [51]. Cars of one class playing one strategy are interchangeable:
  a payoff depends on how many cars play each strategy, not which. Profiles drop from k^N to count
  vectors, about N^(k-1).
- **Double oracle** [52]. Start from a few strategies, solve, add only the best response to the
  current equilibrium, repeat. Equilibria usually have small support, so the set stays small.
  Refining the dominant strategies is this loop made systematic; PSRO [19] is its learned form.

### 2. A learned proposer: it proposes, something independent decides

The risk is real: a proposer knows only its training distribution. It is contained if the proposer
never decides:

- **Propose, then verify.** As in PSRO, the proposer only adds candidates; the game solver over
  measured payoffs decides what is played. A bad proposer then costs time, not correctness.
- **Exploitability is the validation.** Against a proposed equilibrium, search hard for any car's
  profitable deviation: 025's adaptive search and an untrained random search, both independent of
  the proposer. A gain near zero means the equilibrium holds, and needs no trust in the training
  data; a gain names exactly what the proposer missed.
- **Exact ground truth at small N.** For two or three cars, 1. computes the equilibria exactly; the
  proposer must reproduce them before it is trusted at larger N.
- **Held-out conditions.** Contexts, maps and car models the proposer did not train on; and its
  proposals are always mixed with untrained exploration, so the search cannot be trapped in its
  blind spot.

### 3. Observe and update: strategies become short reactive rules

This changes what a strategy is, and so shrinks the space most:

- **A strategy is a rule over the belief, not a full-horizon plan**: "if B slows, go; if B holds,
  wait one more second; past gap g, abort". Each decision is among two or three actions over a few
  seconds, and the belief carries the rest. Strategies become low-dimensional parameter vectors
  (the merge threshold, the wait, how much evidence before going), searched by Bayesian
  optimisation or 025's search rather than enumerated.
- **One belief over intent and capability.** Slowing is ambiguous: a yield, a car that cannot go
  faster, or a reaction to something else. The rate, the timing and the context tell them apart: a
  capability limit shows as a plateau, a yield as a deceleration that starts when A signals.
- **The other car watches too.** An action is also a signal: A's nudge toward the lane probes B and
  tells B. Model it to one or two levels (level-k [3, 4]), not to infinity.
- **Mutual waiting.** Both cars wait to learn more and neither moves: the frozen robot, or a
  deadlock. It needs a tie-break (a time limit, a priority rule, a deliberate probe), and swb can
  find where it happens by sweeping contexts.

### Together

| layer | what runs | idea |
|---|---|---|
| in the car, live | belief update and a short-horizon reactive choice among a few actions | 3 |
| offline, analysis | local games over the interaction graph, anonymity per car class, double oracle | 1 |
| offline, scale | a learned proposer feeding the double oracle; exploitability measured in swb; exact agreement at N = 2-3 first | 2 |

swb's part: the exact ground truth at small N with real stacks, the exploitability check (the search
for profitable deviations against any proposed equilibrium), and the deadlock finder.

## What swb brings

| needed for the method | swb today |
|---|---|
| several AVs in one world, each a real stack | CARLA: the ego plus background AVs, each an Autoware stack in its own ROS domain, reached through the agent relay (`multi_av/town01_two_av.xosc`, `just two-av`, `just bg-av`). Isaac: open (roadmap 039) |
| a strategy plugged into a car | `swb_agent.Agent`: any Python policy drives a car; Autoware's RTC (operator approval at intersection, lane change and other modules) is reachable through `on_cooperate` / `on_cooperate_auto`, which is a go-now / wait control on a real stack |
| contexts as parameters | OpenSCENARIO parameters and SSv2's parameter sweeps; roadmap 025's genomes, including a cut-in family (gap 5-40 m, merge length 6-30 m, speed 2-10 m/s, acceleration 1-5 m/s^2) |
| choosing the next runs | 025's ask/tell search (`scripts/gap_search.py`: corpus, archive, Exp3, Thompson sampling; tested offline) and its exploit pipeline (bisecting to a boundary); the same machinery can sample an equilibrium boundary instead of a gap boundary |
| payoffs from physics | the per-frame trace (024: every entity's pose and velocity), contacts with impulse (021), JUnit verdicts; TTC, PET, delay and comfort follow from the trace |
| car models, with true capabilities | `sim_vehicle_spec`: one spec per car (mass, size, brake torque and lag, motor limits and lag, steering limit and lag); three cars ship (`tesla_model3`, `drift_mustang`, `newslab_golfcart`), chosen per entity from the catalog. `just vehicle-check` measures each car's dynamics, so F(theta) is measured, not assumed |
| two simulators | CARLA 0.9.16 and Isaac Sim 6.1 behind one world protocol: a payoff map measured on both says whether it depends on the simulator |

What it adds that the literature lacks: payoffs measured on **full production stacks** (perception,
localization, planning, control), with physical contacts, rather than on policy abstractions. The
heterogeneous meeting of two different stacks or configurations, which the survey found no work on,
is a configuration change in swb, not a new simulator.

**Gaps to close first:**

- **Cost.** One run is 1-3 minutes with a real Autoware ego, so a few hundred a night per
  pipeline, and two stacks on one host cost about twice the CPU and GPU. Payoff maps need far more
  points than that. Hence fidelities: (1) a kinematic simulator in Python for the theory and for
  RL best responses; (2) swb with light agents (a `swb_agent` policy on ground-truth localization,
  no perception) for broad maps; (3) swb with full Autoware stacks to validate the boundaries the
  lighter layers found. Parallel pipelines (roadmap 026) multiply the third.
- **Strategy knobs on Autoware.** Which of Autoware 1.5's parameters make a car "yield" or "hold"
  for a cut-in, beyond RTC's timing, is not established; obstacle cruise's following distance and
  the lane change module's safety margins are candidates. To be measured before they are called
  strategies.
- **The map.** A cut-in needs two lanes in one direction; Town01 has one each way (025's i7 pulls
  out from the shoulder instead). Town04 or Town05 has multi-lane roads.
- **Isaac.** Several Autoware stacks on Isaac (039) is open; CARLA carries the first experiments.
- **SSv2.** SSv2 drives one scenario ego; the other AVs are entities with the `agent` controller,
  reached through the relay. Whether every SSv2 condition sees them as it sees the ego is not
  checked.

## A first experiment

The user's own 2 x 2 cut-in, end to end, before any N-vehicle work:

1. **Theory baseline.** The cut-in in a kinematic model: A's (g_A, a_A), B's yield or hold;
   payoffs from progress, minimum TTC and contact. Compute the equilibrium map over (gap, speed
   difference) analytically or by grid. This is the prediction.
2. **swb, light agents.** The same game on Town04 or Town05 in CARLA, A and B as `swb_agent`
   policies with the same parameters; the context swept by 025's search, aimed at the equilibrium
   boundaries. Compare the measured map with the predicted one.
3. **swb, Autoware.** B a full Autoware stack, A a light agent or a second Autoware with RTC timing
   as its go-now / wait; then both Autoware. Where the measured map leaves the predicted one is
   where the real stack behaves outside the model.
4. **Fit and explain.** SVO angles and potential-game weights fitted to steps 2 and 3; the fit's
   residuals reported.
5. **Car models.** Steps 1-3 repeated with A and B drawn from the catalog (Model 3, golf cart, a
   heavier car when one is specified): the equilibrium map per pair, and how it moves with theta
   against what F(theta) predicts.
6. **Hidden capability.** Each car's policy gets a belief over the other's class and limits instead
   of the truth, from a class prior refined by observed motion. The type-confusion matrix and the
   misbelief regret per context; then probing (A's nudge) against passive estimation.
7. **Then N = 3-4** on a merge: interaction-graph decomposition and one PSRO round with an RL best
   response trained in step 1's model and validated in steps 2 and 3.

Outputs: the equilibrium map with confidence, per car-model pair; the fitted readable model and
where it fails; the cost of not knowing the other car's capability; and for step 3 the cross-play
payoffs of two stack configurations. None of these is reported for full stacks in published work.

## References

Verified by title, authors, venue and URL on 2026-11-13 unless marked.

1. Sadigh, Sastry, Seshia, Dragan, "Planning for Autonomous Cars that Leverage Effects on Human Actions", RSS 2016. https://roboticsproceedings.org/rss12/p29.html
2. Fisac et al., "Hierarchical Game-Theoretic Planning for Autonomous Vehicles", ICRA 2019. https://arxiv.org/abs/1810.05766
3. Li, Kolmanovsky et al., "Game Theoretic Modeling of Driver and Vehicle Interactions for Verification and Validation of Autonomous Vehicle Control Systems", IEEE TCST 26(5), 2018. https://core.ac.uk/download/154803307.pdf
4. Tian, Li, Kolmanovsky, Yildiz, Girard, IEEE T-ITS 2020. https://arxiv.org/abs/1910.07141
5. Huang, Liu, Lv, "GameFormer", ICCV 2023. https://arxiv.org/abs/2303.05760
6. Fridovich-Keil, Ratner, Peters, Dragan, Tomlin, "Efficient Iterative Linear-Quadratic Approximations for Nonlinear Multi-Player General-Sum Differential Games", ICRA 2020. https://arxiv.org/abs/1909.04694
7. Le Cleac'h, Schwager, Manchester, "ALGAMES", RSS 2020. https://arxiv.org/abs/1910.09713
8. Wang, Wang, Talbot, Gerdes, Schwager, "Game Theoretic Planning for Self-Driving Cars in Competitive Scenarios", RSS 2019. https://roboticsproceedings.org/rss15/p48.html
9. Bhatt, Mehr, constrained dynamic potential games, 2022, https://arxiv.org/abs/2206.08963; weighted version, 2023, https://arxiv.org/abs/2308.05876
10. Liu, Kolmanovsky et al., "Potential Game Based Decision-Making Frameworks for Autonomous Driving", 2022. https://arxiv.org/abs/2201.06157
11. Huang, Di, Du, Chen, mean-field game for AV traffic, ITSC 2019. https://arxiv.org/abs/1906.01554
12. Peters et al., "Contingency Games for Multi-Agent Interaction", RA-L 9(3), 2024. https://arxiv.org/abs/2304.05483
13. Huang et al., Bayesian game with intent as type, 2024. https://arxiv.org/abs/2409.13993
14. Peters et al., "Inferring Objectives in Continuous Dynamic Games from Noise-Corrupted Partial State Observations", RSS 2021. https://arxiv.org/abs/2106.03611
15. Liu, Peters, Alonso-Mora, "Learning to Play Trajectory Games Against Opponents with Unknown Objectives", RA-L 8(7), 2023. https://arxiv.org/abs/2211.13779
16. Liu, Peters, Alonso-Mora, Topcu, Fridovich-Keil, "Auto-Encoding Bayesian Inverse Games", 2024 (venue UNVERIFIED). https://arxiv.org/abs/2402.08902
17. Schwarting, Pierson, Alonso-Mora, Karaman, Rus, "Social behavior for autonomous vehicles", PNAS 116(50), 2019. https://doi.org/10.1073/pnas.1820676116
18. Wellman, Tuyls, Greenwald, "Empirical Game-Theoretic Analysis: A Survey", JAIR 82, 2025. https://arxiv.org/abs/2403.04018
19. Lanctot et al., "A Unified Game-Theoretic Approach to Multiagent Reinforcement Learning", NeurIPS 2017. https://arxiv.org/abs/1711.00832
20. Omidshafiei et al., "alpha-Rank", Scientific Reports 9:9937, 2019. https://arxiv.org/abs/1903.01373
21. Kalaria, Maheshwari, Sastry, "alpha-RACER", L4DC 2025, PMLR 283. https://proceedings.mlr.press/v283/kalaria25a.html
22. Geiger, Straehle, "Learning Game-Theoretic Models of Multiagent Trajectories Using Implicit Layers", AAAI 2021. https://arxiv.org/abs/2008.07303
23. Lidard, Hu et al., "Blending Data-Driven Priors in Dynamic Games", RSS 2024. https://arxiv.org/abs/2402.14174
24. Wang, Yang, EGTA for non-collaborative navigation (proposal), 2021. https://arxiv.org/abs/2109.14755
25. Sarkar, empirical game models for autonomous driving, PhD thesis, University of Waterloo. https://uwspace.uwaterloo.ca/handle/10012/18751
26. Qiao et al., "TrafficGamer", 2024. https://arxiv.org/abs/2408.15538
27. "CoLMDriver", ICCV 2025. https://arxiv.org/abs/2503.08683
28. Kesting, Treiber, Helbing, MOBIL, Transportation Research Record 2007 (author list UNVERIFIED).
29. O'Kelly et al., "Scalable End-to-End Autonomous Vehicle Testing via Rare-event Simulation", NeurIPS 2018. https://arxiv.org/abs/1811.00145
30. Wang et al., "AdvSim", CVPR 2021. https://arxiv.org/abs/2101.06549
31. Rempe et al., "STRIVE", CVPR 2022. https://arxiv.org/abs/2112.05077
32. Hanselmann et al., "KING", ECCV 2022. https://github.com/autonomousvision/king
33. Feng et al., "Dense reinforcement learning for safety validation of autonomous vehicles", Nature 615:620, 2023. https://doi.org/10.1038/s41586-023-05732-2
34. nuPlan https://arxiv.org/abs/2106.11810; Waymo Sim Agents https://arxiv.org/abs/2305.12032; Waymax https://arxiv.org/abs/2310.08710; InterSim https://arxiv.org/abs/2210.14413; MetaDrive https://arxiv.org/abs/2109.12674; SMARTS https://arxiv.org/abs/2010.09776; Bench2Drive https://arxiv.org/abs/2406.03877
35. Huai et al., "DoppelTest", ICSE 2023. https://ics.uci.edu/~yhuai/DoppelTest
36. "STCLocker", 2025. https://arxiv.org/abs/2506.23995
37. Wu, Kreidieh, Parvate, Vinitsky, Bayen, "Flow", IEEE T-RO 38(2), 2022. https://arxiv.org/abs/1710.05465
38. Le Maitre, Prorok, "Effects of Controller Heterogeneity on Autonomous Vehicle Traffic", ITSC 2020. https://arxiv.org/abs/2005.04995
39. Zhou, Gao, Olofsson, Frisk, "Uncertainty-Aware Decision-Making and Planning for Autonomous Forced Merging", CDC 2024. https://arxiv.org/abs/2410.20514
40. Le Cleac'h, Schwager, Manchester, "LUCIDGames", RA-L 6(3), 2021. https://arxiv.org/abs/2011.08152
41. Zhang, Chiu, Chou, "Constraint Learning in Multi-Agent Dynamic Games from Demonstrations of Local Nash Interactions", 2025. https://arxiv.org/abs/2508.19945
42. Shalev-Shwartz, Shammah, Shashua, "On a Formal Model of Safe and Scalable Self-driving Cars" (RSS), 2017. https://arxiv.org/abs/1708.06374
43. Zhong, Shimizu, Chen, "CCILQGames", RA-L 2023. https://arxiv.org/abs/2203.01222
44. Tian, Sun, Bajcsy, Tomizuka, Dragan, confidence-aware game-theoretic human models with a reachability safety monitor (venue UNVERIFIED). https://arxiv.org/abs/2109.14700
45. Sadigh, Sastry, Seshia, Dragan, "Information Gathering Actions Over Human Internal State", IROS 2016. https://doi.org/10.1109/IROS.2016.7759036
46. Hu, Isele, Bae, Fisac, "Active Uncertainty Reduction for Safe and Efficient Interaction Planning: A Shielding-Aware Dual Control Approach", IJRR 43(9), 2024. https://arxiv.org/abs/2302.00171
47. Kumar, Fu, Pathak, Malik, "RMA: Rapid Motor Adaptation for Legged Robots", RSS 2021. https://arxiv.org/abs/2107.04034
48. Wang, ..., Lv, "DynaDreamer", 2026. https://arxiv.org/abs/2607.13410
49. Ferguson, Maheshwari, Wu, Sastry, "Game-to-Real Gap", 2026. https://arxiv.org/abs/2601.16367

Cited from memory, not re-checked by search:

50. Kearns, Littman, Singh, "Graphical Models for Game Theory", UAI 2001.
51. Daskalakis, Papadimitriou, "Computing Equilibria in Anonymous Games", FOCS 2007.
52. McMahan, Gordon, Blum, "Planning in the Presence of Cost Functions Controlled by an Adversary" (double oracle), ICML 2003.
