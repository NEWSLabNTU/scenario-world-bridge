# Survey: Cooperation Strategies Among Autonomous Vehicles, and What scensim Can Measure

This study asks how to evaluate the ways autonomous vehicles decide among each other in
multi-vehicle traffic, with cooperation strategies in focus, and whether scensim can estimate the
payoffs of those strategies and discover new ones. It looks for a method that keeps game theory's
explanations and takes learning's scale.

**Source**: user direction 2026-11-13: "Study the ways to evaluate the ways to enable vehicles to
drive autonomously in a multiple vehicle environment. Focus cooperation strategies. See if our scensim
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
  This is the role scensim would play.
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

The simulator knows every car's true theta; the cars do not. scensim can therefore score both what
happened and what would have happened with knowledge:

- **Misbelief regret**: J(true type | belief) - J(true type | true type), per car and run.
- **Value of information**: its expectation over the type distribution, i.e. what knowing the
  other's capability is worth, and so what a V2X capability broadcast would be worth.
- **The type-confusion matrix**: assumed class x actual class, with contact rate, minimum TTC and
  delay in each cell. No AV paper reports it; in scensim it is a sweep over two catalog entries and one
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
  deadlock. It needs a tie-break (a time limit, a priority rule, a deliberate probe), and scensim can
  find where it happens by sweeping contexts.

### Together

| layer | what runs | idea |
|---|---|---|
| in the car, live | belief update and a short-horizon reactive choice among a few actions | 3 |
| offline, analysis | local games over the interaction graph, anonymity per car class, double oracle | 1 |
| offline, scale | a learned proposer feeding the double oracle; exploitability measured in scensim; exact agreement at N = 2-3 first | 2 |

scensim's part: the exact ground truth at small N with real stacks, the exploitability check (the search
for profitable deviations against any proposed equilibrium), and the deadlock finder.

## Theory formulation

User direction 2026-11-13: explore the theory formulation. This section states the game the rest of
the document measures, in the user's terms: a general strategy and payoff space, the car model as a
restriction, the other car's capability unknown, beliefs updated from observed behaviour. The
propositions marked as such are standard results or direct consequences; the ones marked
*hypothesis* are for scensim to test.

### Primitives

- **Agents** $i \in N$, the cars in an interaction.
- **Type** $t_i = (\theta_i, \varphi_i)$: the **capability** $\theta_i$ (acceleration and braking
  bounds, top speed, lateral acceleration and curvature bounds, actuator lag, size) and the
  **preference** $\varphi_i$ (the weights of its payoff; an SVO angle in the simplest case). The
  ego knows its own type; another car's is private.
- **Dynamics** $x_{k+1} = f(x_k, u_k)$ over the joint state, each car's control constrained by its
  capability, $u_i \in U(\theta_i)$. The car's stack sits inside $f$ and $U$: Autoware turns a
  high-level action into controls, and its own limits make $U(\theta_i)$ the physical bounds
  intersected with the stack's.
- **Observation** $o_{i,k} = h_i(x_{k-d}) + \text{noise}$ at the perception rate (10 Hz) with delay
  $d$: the others' pose, velocity, size and class, never their type.
- **Prior** $b_{i,0}(t_j \mid c_j)$ from the observed class and size $c_j$, and from a V2X broadcast
  when there is one.
- **Outcome** $\omega$ of an episode: the order of passage, each car's delay, minimum TTC and PET,
  peak deceleration and jerk, contact and its impulse.
- **Payoff** $J_i(\omega;\varphi_i)$: one function of the outcome for every car model. With an SVO
  angle, $J_i = \cos\varphi_i\,U_i(\omega) + \sin\varphi_i\,U_{-i}(\omega)$, where $U$ is a car's
  own utility (progress, comfort, safety).

### Strategies: general rules, restricted by the car model

A strategy is a **reactive rule** over high-level actions, not a full-horizon plan:

$$\pi_i(a \mid b_i, s_i;\ \psi_i), \qquad a \in A_i,$$

where $A_i$ is a handful of actions (A: go, wait, abort; B: yield, hold, close), $s_i$ the local
state (gap, speeds, distance left), $b_i$ the belief about the others and $\psi_i \in \Psi$ the
rule's parameters (merge threshold, wait, evidence needed). $\Psi$ is the same for every car
model. The car model enters as a **type-dependent strategy set**: a rule is executable by type
$\theta$ if the controls it induces stay in $U(\theta)$ over the contexts considered,

$$\Psi(\theta_i) = \{\psi \in \Psi : u_i(\psi, \cdot) \in U(\theta_i)\} \subseteq \Psi,$$

and the reachable outcomes shrink with it. A larger capability gives a larger set:
$U(\theta) \subseteq U(\theta') \Rightarrow \Psi(\theta) \subseteq \Psi(\theta')$.

### Beliefs from observed behaviour

$$b_{i,k}(t_j) \;\propto\; P(o_{i,1:k} \mid t_j, \pi_j)\; b_{i,0}(t_j).$$

Two kinds of evidence, with different logic:

- **Capability evidence is set-valued, but soft.** An observed deceleration of $6\ \text{m/s}^2$
  says the braking bound is at least 6, whatever the strategy; this part needs no model of the
  other's strategy. But the acceleration is differentiated twice out of 10 Hz tracks, so the bound
  is a likelihood with a noise model, not a hard cut, or one noisy sample prunes the true type.
  And cars seldom show their limits in normal traffic, so most of the capability belief is the
  class prior; observed motion mostly rules out the weakest types (roadmap 074 review).
- **Intent evidence is strategic.** Whether a deceleration means "yields" depends on the strategy
  the other is believed to play, so beliefs and strategies must be consistent: the likelihood uses
  $\pi_j$, which is itself an equilibrium object.

### Solution concept

The exact object is a **perfect Bayesian equilibrium** of a dynamic game with type-dependent
strategy sets: each car's rule a best response to the others' given its belief, beliefs updated by
Bayes' rule on the equilibrium path. Intractable in general; three restrictions make it usable:

1. **Restricted rules**: $\psi$ low-dimensional, so the game is over parameters, evaluated by
   simulation: an empirical Bayesian game.
2. **Approximate equilibrium**: an $\varepsilon$-Bayes-Nash equilibrium, with $\varepsilon$ measured as
   exploitability (below), not assumed.
3. **Safety as a constraint, not a payoff term**: each rule must satisfy
   $P_{b_i}(\text{contact}) \le \delta$ under its own belief, which makes the game a generalized Nash
   problem whose feasible sets depend on the others' strategies. The shield (AEB, RSS) is the hard
   floor under it.

The result is an **equilibrium map** $E(c, t)$ over the context $c$ (gap, speeds) and the types: the
object scensim measures.

### The two-car cut-in: canonical forms

**One-shot, at a fixed context: chicken, only where the moves are simultaneous.** Go/go risks
contact, wait/wait costs time, and the two asymmetric profiles (one goes, one yields) are both
equilibria. But the cut-in is sequential: B sees A's lane change and replies, and a threat to hold
against a car that already went is not credible if yielding is then better for B. With B's reply
sequentially rational the chicken band of a simultaneous model disappears (roadmap 074, C1 after
its review). Chicken stays where something makes the choices simultaneous: a commitment that has to
be made before the other's move is visible (B's courtesy before any signal; A's prepare phase),
latency longer than the window, or uncertainty about the other's type. There, which equilibrium
is played is an **equilibrium selection** problem, not a computation problem. With B's type unknown, A's best response
is a **threshold rule**. Let $p = P_{b_A}(\text{B yields})$, and $U_s$, $U_c$, $U_w$ A's payoffs if it
goes and B yields, if it goes and B holds, and if it waits. Then A goes if and only if

$$p \;>\; \kappa(c, \theta) = \frac{U_w - U_c}{U_s - U_c}.$$

The threshold is explainable ("A went because $P(\text{yield}) = 0.7 > \kappa = 0.55$"), and the
capabilities enter through $U_c$: the cost of going against a car that holds depends on how hard
both can brake. With safety as a constraint rather than a payoff, going against a hold that ends
in contact is allowed only when $P(\text{hold}) \le \delta$, so A goes iff
$p > \max(\kappa, 1-\delta)$; otherwise a large payoff gain could buy a contact risk.

**In time: a war of attrition.** "Wait" is a choice of when to concede. With private types
(capability, patience), the classical result is that equilibrium concession times are monotone in
type, and **deadlock is an equilibrium outcome**: two patient cars both wait. A deadlock is then not
a bug of one stack but a property of the game, and its cure is outside the two cars: a tie-break.

**Signals.** A's lateral nudge is a costly signal (it costs risk), B's early deceleration a signal
back. Where the cost differs by type, separating equilibria exist and the nudge is informative; the
turn indicator is cheap talk.

**Conventions as correlation devices.** Right-of-way rules, traffic lights and a V2X negotiation
act as correlation devices: they pick one of the asymmetric equilibria for both cars, a correlated
equilibrium. In this view, cooperation is mostly **equilibrium selection**, and the question of
which convention to use becomes mechanism design.

### Capability in the game: two hypotheses for scensim

- **Single-agent monotonicity holds.** Against fixed others, more capability never hurts:
  $\Psi(\theta) \subseteq \Psi(\theta')$, so the best response over the larger set is at least as
  good.
- *Hypothesis*: **in the game it can fail.** In chicken, inability is commitment: a car that
  visibly cannot brake hard (a loaded truck), or cannot accelerate (the golf cart), is yielded to
  or waited for, so a lower capability can earn a higher equilibrium payoff. Whether the stacks'
  equilibrium map shows it, per car-model pair, is a measurement.
- *Hypothesis*: **misbelief is asymmetric.** Overestimating the other's braking (believing it can
  yield when it cannot) costs contact; underestimating it costs delay. The type-confusion matrix
  should be lopsided, which argues for pessimistic priors on capability and neutral ones on
  intent.

### N cars: pair terms, composition, residual, mean field

The two-car games above are what scensim can measure cheaply (C2a: thousands of runs per hour). An
N-car scene is built from them, and the error of doing so is measured rather than assumed.

**Locality.** At step $k$ car $i$'s conflict set $K_i(k)$ is the cars whose reachable sets meet
$i$'s within the horizon, sorted into slots: lead, rear, the four adjacent-lane neighbours (front
and rear on each side), crossing, oncoming. Its payoff depends on $K_i$ only, a graphical game
(Kearns, Littman, Singh [37]). Naturalistic data says $K_i$ is small: multi-vehicle interaction
events in INTERACTION involve 3.2 vehicles on average and 7 at most, WOMD and Lyft at most 5
[38]; at INTERACTION's intersections 10-17 cars are in the scene but about 5 or fewer are in
conflict [39]. So exact games up to $|K_i| = 4$-$5$ cover most events, and the slots bound the
local structure that a learned model must generalize over (Yehudai et al. [40]: size
generalization holds only when local neighbourhoods match).

**Pair terms.** $J^{(2)}(\psi_i, \psi_j; c_{ij}, t_i, t_j)$ is the measured two-car game for the
pair's slot, relative context and types: the cut-in (C2a, the double oracle), the left turn (U1),
car following. Each pair term splits into the measures it is made of: risk $\rho_{ij}$
(post-encroachment or time-to-collision shortfall, contact), delay $d_{ij}$, peak braking $b_{ij}$.

**Composition.** Different measures compose differently, and the rule is part of the model:

| measure | rule | why | precedent |
|---|---|---|---|
| risk | $\rho_i = \max_{j \in K_i} \rho_{ij}$ (safe iff every pair is safe); upper bound $\sum_j \rho_{ij}$ | a car is hit by one other car at a time; the sum is the union bound | RSS [41]; pairwise CBF union bound [42] |
| delay | max-plus along the leader-follower DAG: $d_j = \max(d_j^{\text{own}}, d_{\text{lead}(j)} - s_j)$, $s_j$ the slack its headway absorbs | a yield propagates down the queue (U1's chain externality) | FJMP's DAG factorization [43], M2I [44] |
| braking | $b_i = \max_j b_{ij}$ | peak, not total | |
| welfare | $C = \sum_i$ of the composed own costs | as for two cars | |

The composed model $\hat J^{\text{comp}}_i = \mathrm{Agg}_{j \in K_i} J^{(2)}$ is the prediction;
pairwise sums of social forces are known to fail in dense crowds (Moussaïd et al. [45]), which
is why the aggregation is per measure and tested.

**Higher-order residual.** Run the N-car game in the arena and in scensim and measure
$r_i = J_i - \hat J^{\text{comp}}_i$ per profile and context, with its bootstrap interval. Three
outcomes: $r$ within noise (pairs suffice: report the variance the pairs explain); $r$ systematic
but smooth in the slot features (learn it: $J_i = \hat J^{\text{comp}}_i + g_\theta(\{x_j\}_{j \in
K_i})$, $g$ permutation-invariant per slot class, a Deep Sets or attention model; theory plus a
learned residual as in PERL [46] and physics-informed car following [47]); $r$ large and
irregular (the scene is a game of its own: solve it exactly). Cell-migration work fits pair plus
three-body terms to trajectories and finds the three-body term a small correction in one of two
cell lines [48]; that is the template, and nobody has done it for driving stacks.

**Growing N by error.** Start from pairs. For each context family run $N = 3$; where the residual
interval excludes zero and exceeds the tolerance $\delta$ (the D2 bar), keep the three-car game
explicit and try $N = 4$ there only. Stop when $N+1$ changes the payoffs by less than $\delta$.
The cost grows with the number of contexts that need it, not with $N$.

**Equilibria.** With symmetric pair terms and self terms the game is a potential game: a pure
equilibrium exists and best response converges (Liu et al. [10]). Right of way, truck against car
and capability types make pairs asymmetric; then the near-potential fit above applies, and the
fit's residual bounds the distance to an equilibrium. Pairwise games can be over-cautious and
deadlock [10]; the arena's "after you" deadlock (910 runs in C2a) is that.

**Mean-field regime.** Past the conflict set, cars matter through aggregates. Near field exact,
far field mean field:

$$J_i = J_i^{\text{near}}(\psi_i, \psi_{K_i}; c) + J^{\text{far}}(\psi_i; \rho, \bar v, \mu_\psi)$$

with $\rho$ the density, $\bar v$ the mean speed and $\mu_\psi$ the population's rule mix,
in equilibrium with the far field's own response (a mean-field game, Lasry-Lions [49],
Huang-Malhamé-Caines [50]; for traffic, a driver utility calibrated to the fundamental diagram
[51], AV speed control [11], lane changes [52], mixed human and AV populations [53]). One AV among
many is a major agent among minor ones [54]; mean-field RL replaces the pair sum by an interaction
with the neighbours' mean action [55], which is the bridge between the two halves. The
finite-$N$ error is $O(1/\sqrt N)$ [54]; the one empirical threshold found is $N \gtrsim 30$, for
routing [56], none for merges or junctions. Mean field fails at small $N$, under strong asymmetric
coupling (mixed-autonomy routing's price of anarchy is unbounded [57]) and in dense crowds. For
scooter swarms the far field is a porous-flow density [58].

Hybrids of near and far exist in traffic *simulation* (micro-macro coupling at an interface) and
in generic MARL (attention over near neighbours plus a mean field), but no game model has an exact
conflict-set game plus a mean-field remainder, and there is no error bound for the split. scensim can
measure that error directly: the same scene with the far field explicit (all cars as agents) and
as a density, at increasing $N$.

**What is new here** (searched 2026-11-13, abstracts only): a measured higher-order residual on a
full AV stack; payoff composition rules compared on measured multi-car risk and delay; a
near-field game with a mean-field remainder for traffic and its validity range; a residual learned
on top of a game-theoretic payoff model.

### Measures

| measure | definition | what it says |
|---|---|---|
| exploitability | $\varepsilon(\pi) = \max_i \max_{\pi_i'} [J_i(\pi_i', \pi_{-i}) - J_i(\pi)]$ | how far a profile is from equilibrium; found by search, so independent of how the profile was made, and a lower bound: reported with the search's budget |
| misbelief regret | $R_i = J_i(\pi_i^{b}; t) - J_i(\pi_i^{t}; t)$ | what acting on the belief instead of the truth cost |
| value of information | $\mathbb{E}_t[R_i]$ | what knowing the other's type is worth, so what a V2X broadcast is worth |
| social cost | $C = -\sum_i U_i \ge 0$ over everyone in the trace, own utilities $U$ (not the SVO-weighted $J$, which would count the others twice); for comparisons across car models in common units (vehicle-seconds of delay), with safety as a constraint | |
| price of anarchy, of stability | $C(\text{worst eq.}) / C(\text{opt})$, $C(\text{best eq.}) / C(\text{opt})$, both $\ge 1$; the losses $C(\text{eq.}) - C(\text{opt})$ when $C(\text{opt})$ is near 0 | what selfish rules cost against coordination; what a good convention can recover |

Per run, scensim and the arena record the same raw measures from truth (`scripts/coop/measures.py`),
each defined over time rather than samples so that the arena's 0.05 s ticks and scensim's 0.1 s logs
agree, and none scaled by the run's length:

| kind | measures |
|---|---|
| safety | contact and the closing speed at impact; minimum TTC and time headway; the deceleration rate to avoid a crash (DRAC, closing²/2·gap) and the capability margin $1 - \text{DRAC}/b_{max}$; time-exposed and time-integrated TTC below 3 s (TET, TIT); PET for crossings |
| efficiency | lost progress against free flow over a fixed horizon; stops (below 0.1 m/s for 1 s or more) and seconds stopped; time to complete the manoeuvre |
| comfort | peak braking and peak jerk, both as slopes over 0.5 s |
| latency | event times: A's lateral move, each car's first braking, A's decision; from them B's reaction to A's move and A's reaction to B's yield |

Latency is a property of the car, so it enters the game as a type parameter, and its cost shows
in the outcome; the measured reaction times check that the agents react as their type says.
The utility combines a few of these (lost progress, the TTC and headway shortfall, peak braking,
contact); the rest are reported beside it, so that safety can also be applied as a constraint
(drop profiles whose contact rate or margin fails a bound, then compare on efficiency) and the
equilibrium map can be checked for how much it depends on the weights.

### From the theory to the measurement

scensim estimates $\hat J_i(\psi, c, t)$ from runs: a mean per profile, context and type pair with its
uncertainty, a surrogate (a Gaussian process) over $c$, and runs placed where the equilibrium map
changes. A profile is reported as an $\varepsilon$-equilibrium with confidence $1-\alpha$ from the
estimates' bounds, as in EGTA's statistical results [18]. What is new against the literature: the
type is the capability, the strategy sets depend on it, and the dynamics include the real stack.

## Multi-vehicle cases from real traffic

User direction 2026-11-13: the formulation on multi-vehicle cases (highway, night market, urban).
Literature checked by web search the same day (a third round); unchecked items are marked.

### What changes with many road users

- **Heterogeneous classes.** Road users are cars, trucks, buses, scooters, bicycles and pedestrians.
  A class is a prior over the type: a scooter is light, quick to turn, slow to brake hard and has
  high agility, and a pedestrian has almost no momentum and a large lateral freedom.
- **An open system.** Agents arrive and leave, so the game is a stochastic game with entry over a
  time-varying interaction graph $G_k$, not a fixed $N$.
- **Externalities beyond the graph.** A courtesy yield on a highway is local, but its cost
  travels upstream as a shockwave to cars that never interacted with the merger. The game can be
  solved locally, but the welfare must be measured over everyone the trace holds.
- **Three structural regimes**, each with its own solver:

| regime | where | structure | solver |
|---|---|---|---|
| few agents, tight coupling | merge gap, 4-way stop, roundabout entry, left turn | a Bayesian game over rules among 2-4 cars | the empirical game of this document, exactly |
| chains | lanes, queues, an oncoming stream | leader-follower pairs along a lane; a follower's choice depends on its leader's | leader-follower decomposition; externality propagation along the chain |
| populations | scooter swarm at a light, night-market crowd | many interchangeable agents; a payoff depends on the distribution, not on who | population or mean-field game: one AV against a distribution |

- **Conventions differ per setting**: a zipper sign, first-come order at a stop, the two-stage left
  box, or none at all in a night market. Each is a correlation device that selects among
  equilibria, and its effect is measurable.

### Highway: on-ramp, weaving, lane drop

- **Players**: the merging car and the cars in the target lane, a chain. Zhang, Han, Grammatico [53]
  treat the ego against the surrounding cars as one player that picks a gap, with a branch MPC
  hedging whether the others play Nash or Stackelberg: an N > 2 setting kept small.
- **Strategies**: the merger's gap choice and timing (which gap, when); each target-lane car's
  yield or hold, and its headway.
- **Lane drop**: an N-player alternating-priority game. The zipper is socially efficient, but a late
  merger can look like it defects ("jumping the queue"). No game model of it was found. Field
  evidence is weak: Kentucky 2017 found minor, not significant gains [54]; Iowa 2024 found better
  operations but widely varying driver compliance [55]. Read as a game, the zipper is a norm
  equilibrium that depends on familiarity: price of anarchy without it, of stability with it.
- **Phenomena to test**: gap acceptance by speed difference and by class (a truck merging); the
  courtesy yield's upstream shockwave (simulation evidence only so far, [56]).
- **Data**: highD, exiD, INTERACTION [57-59].
- **scensim**: CARLA's highway towns (Town04, Town06) have ramps, but scensim has no Autoware map of either
  (roadmap 074; Town06 is not in the 0.9.16 binary here), which SSv2 needs; the target-lane traffic can be SSv2's
  NPCs plus background AVs; the upstream cost is read from the trace.

### Urban: 4-way stop, unprotected left, roundabout, Taiwan's scooters

- **Conflict-zone structure.** Cars interact through the conflict zones of an intersection, so the
  game is a scheduling game: the order of passage through each zone. An equilibrium is an order; a
  **deadlock is a cycle in the wait-for graph** (STCLocker's oracle [36]).
- **4-way stop**: symmetric arrivals have no deterministic tie-break, so randomisation, auctions
  (GAMEPLAN [60]) or backing off are needed. First-come order is the convention. Li et al. [61] model
  several concurrent leader-follower pairs at an uncontrolled intersection, scaling to N. Data:
  INTERACTION's multi-way stops [59].
- **Unprotected left against an oncoming queue**: the oncoming cars are a chain. A follower cannot
  yield unless its leader does, so A plays against the chain, not against one car. Two-player models
  exist (bounded rationality, a quantal response equilibrium [62]; a Bayesian game [13]); none for the
  queue was found.
- **Roundabout entry**: level-k with an online estimate of the other driver's type [63]; a
  Stackelberg-coalition model for CAVs [64]. Data: rounD [58], INTERACTION.
- **Taiwan's scooters**:
  - The swarm at a light is a population: scooters fill the waiting box and discharge ahead of the
    cars. A Thai study found a motorcycle storage zone raises cars' start-up lost time linearly in
    the motorcycles queued [65].
  - Filtering between car lanes is a game between the scooter's choice of a gap and the car's
    lateral position.
  - The two-stage left box is a convention trading the scooter's delay for fewer conflicts.
  - Seepage and porous-medium models exist [66]; **no game model of any of these was found.**
  - **HetroD** [67] is drone data from six Taiwanese urban sites, about 70 % vulnerable road users,
    with hook turns, lane splitting and aggressive overtaking, maps and signal states (licence
    unverified). It is the dataset to fit scooter types and payoffs on.
- **scensim**: Town maps for the generic cases; the NTU twin (072, 059) for Taiwan's campus roads, where
  scooters are common. A swarm needs many scooter NPCs. SSv2 scripts its NPCs rather than letting
  them react, so a population that responds to the AV needs a reactive NPC controller. That
  controller does not exist in scensim.

### Night market: shared space at walking speed

- **Players**: one vehicle, sometimes a scooter or two, against a crowd: a population of pedestrians
  of varying attention and patience, with stalls as obstacles. There are no lanes and no
  convention.
- **Formulation**: a repeated sequential chicken (Fox, Camara [68]) between the vehicle and each
  pedestrian ahead of it. The vehicle's strategy is its assertiveness: its creep speed and the
  minimum gap it holds. A pedestrian's best response is to step in front or to wait, by gap
  acceptance. Johora and Müller [69] layer this: trajectory planning, social force for free flow, and
  a game at each conflict.
- **Equilibrium phenomena**: a vehicle that is too risk-averse is exploited and freezes, because
  pedestrians cross with impunity in front of a car that always yields (Millard-Ball's argument
  [70], popular-press source). A vehicle that is too assertive has unacceptable risk. The design
  question is the creep speed and gap that keep throughput above zero with contact near zero.
- **Capability** matters differently here: what counts is the stopping distance at walking speed,
  the precision of a crawl, and the pedestrians' belief that the car can stop. Beliefs about
  pedestrians cover attention (a phone), children and stall operators.
- **Data**: Hamburg-Bergedorf shared space [71], DUT and CITR [72]. Nothing is at night-market
  density. Golchoubian et al. [73] train a crowd-navigating AV on Bergedorf data (not a game).
- **scensim**: not ready. There is no map, SSv2's pedestrians are scripted, not reactive, and Autoware's
  perception of a dense crowd is untested. The first step is offline: fit the sequential chicken on
  Bergedorf or DUT, then add a social-force crowd controller as a world-side NPC driver.

### Summary

| setting | regime | start from | data | phenomenon | scensim today |
|---|---|---|---|---|---|
| on-ramp, weaving | chain + small game | matrix game + branch MPC [53] | exiD, highD, INTERACTION | gap choice, courtesy shockwave | needs a Town04 map (031) |
| lane drop | N-player alternating priority (new) | none | highD, own sweeps | zipper vs early merge, defection | needs a lane-drop site |
| 4-way stop | small game, scheduling | concurrent leader-follower [61]; auction [60] | INTERACTION | symmetric deadlock, tie-breaks | needs an unsignalled junction |
| left turn vs queue | chain | QRE left turn [62], extended to a queue | INTERACTION, WOMD, SinD | gap acceptance vs queue length | ready (Town05, Town10) |
| roundabout | small game | adaptive level-k [63] | rounD, INTERACTION | entry assertiveness | ready (Town03) |
| Taiwan scooters | population | seepage models [66] + a game layer (new) | HetroD | swarm discharge, two-stage box, filtering | needs reactive scooter NPCs |
| night market | population, shared space | sequential chicken [68] + GSFM [69] | Bergedorf, DUT | frozen vehicle vs risk | needs a map and a reactive crowd |

The gaps in the literature are the lane-drop game, the left turn against a queue, any game model of
scooter traffic, and shared space at night-market density. scensim reaches the left turn now; the lane
drop needs a site on a mapped town, and the on-ramp a Town04 map (roadmap 074). The other
two need reactive populations.

## A per-vehicle strategy layer, on Autoware

User direction 2026-11-13: explore the per-vehicle strategy design and architecture, mindful of
message rates and practical deployment, with Autoware as the reference. The numbers below are
Autoware 1.5.0's shipped configuration (`/opt/autoware/1.5.0/share/autoware_launch/config`); no
live rates were measured (the stack on 133 was idle).

### Autoware already holds a belief about the other car: a fixed one

| where | parameter | default | what it assumes about the other car |
|---|---|---|---|
| lane change, safety check (prepare / lane changing / cancel) | `expected_rear_deceleration` | -1.0 / -2.0 m/s^2 | how hard the car behind brakes for us |
| lane change | `rear_vehicle_reaction_time` | 1.0-2.0 s | its reaction time |
| obstacle cruise | `min_object_accel_for_rss` | -1.0 m/s^2 | how hard the car ahead may brake |
| obstacle cruise | `idling_time` | 2.0 s | how long until the car ahead's braking is noticed |
| map-based prediction | `prediction_time_horizon.vehicle`, `prediction_sampling_delta_time` | 15 s, 0.5 s | |

One number for a truck, a golf cart and a sports car. The method of this study replaces these
constants with a per-object belief. RTC, Autoware's approval gate on behaviour modules, is off for
lane change in the default configuration (`enable_rtc: false` in `scene_module_manager.param.yaml`);
using it as the go-now / wait control means turning it on.

### The rates it must live with

| stage | rate | note |
|---|---|---|
| lidar | 10 Hz on a real car (20 Hz in our simulated egos) | |
| tracking and prediction | 10 Hz | pipeline delay about 0.1-0.3 s (not measured here) |
| behavior path planner | 10 Hz (`planning_hz`) | lane change budget 50 ms a cycle (`time_limit`) |
| lane change prepare phase | 2-4 s | cancel judged per 1 s, 5 unsafe cycles before it acts |
| vehicle command gate | 10 Hz | |
| control | 33 Hz (`ctrl_period` 0.03 s) | |
| V2X | BSM / CAM about 10 Hz; manoeuvre coordination event-driven | about 0.1 s latency and loss (from memory, not checked) |

The consequence: telling a yield from a hold takes about 0.5-1.5 s of filtered acceleration
(Autoware itself waits 2 s to judge that the car ahead brakes), and the lane change decides within
2-4 s. The belief has one to three updates' worth of evidence before the decision is due; this is
why probing pays, since a nudge makes the other car show itself sooner.

### Two nodes between prediction and planning, the safety layers untouched

```
lidar 10Hz -> tracking -> prediction 10Hz --+-----------------------------> planning 10Hz -> control 33Hz
                                            |                                 ^      ^
                     (V2X BSM, 10 Hz) ------+                                 |      |
                                            v                                 |      |
                                [interaction_belief, 10 Hz]                   |      |
                       per object: class prior -> capability and intent       |      |
                                            v                                 |      |
                                [interaction_strategy, 2-5 Hz, dwell >= 1 s] -+      |
                       a few reactive rules scored by the offline payoff table       |
                       out: go / wait (RTC), a speed limit, belief-shaped     -------+
                            predictions or per-object safety bounds
```

1. **`interaction_belief`, 10 Hz** (with tracking). Only the objects in the interaction set (same
   or next lane, within a few seconds). Prior from class and size, and a V2X capability broadcast
   when there is one; updated from observed acceleration, speed plateaus (a capability cap) and a
   deceleration that starts when the ego signals (a yield). A few hypotheses per object (interacting
   multiple models or particles). About 100 B per object: 20 objects are about 20 KB/s.
2. **`interaction_strategy`, 2-5 Hz, minimum dwell 1 s.** Slower than the planner on purpose: a
   lane change that switches often chatters and cancels. Candidates: go now, wait tau, abort (A);
   yield, hold (B). Scored by the payoff table built offline in scensim, indexed by the ego's car class,
   the belief about the other and the context (gap, speeds): a lookup of microseconds. An online
   game solve for a small group (iLQGames-class, under 50 ms warm for three cars) also fits the
   budget. Each decision is logged with its belief, the candidates' payoffs and the choice; the log
   is the explanation.
3. **Into Autoware**, from least to most invasive:
   - existing interfaces, no fork: go / wait through RTC (with `enable_rtc` on) or the
     `external_request_lane_change` module (already in scensim's preset); yield / hold through the
     AD API's velocity limit;
   - belief-shaped predictions: a filter edits the predicted paths of objects whose belief is
     pessimistic, the planner unchanged. Lane change reads only the most confident path
     (`use_all_predicted_paths: false`), so several hypotheses on one object are ignored unless that
     changes;
   - per-object safety bounds: `expected_rear_deceleration` and the like taken per object from the
     belief. A code change in lane change and obstacle cruise, which the Autoware overlay
     (roadmap 042) carries as a fork per package. The cleanest, and the most invasive.

### Deployment rules

- **The shield stays.** AEB, obstacle stop and the RSS checks are never bypassed; the layer only
  picks among options the planner already finds safe, or makes the car more conservative.
- **The fallback is stock Autoware.** With the belief node dead or unsure, the constants above
  apply, as today.
- **V2X is untrusted.** A broadcast capability is a prior, checked against observed motion;
  messages are lost or late, so the layer must work on observation alone.
- **Own capability is known** (the vehicle spec, `vehicle_info`), others' only through the belief.
  The same node runs on every car model; only the ego's limits and the table's index change.
- **It fits an ECU**: a table lookup and a small filter per interacting object.

### How scensim tests it

- The same nodes run inside every ego stack in scensim, so the experiments run deployment code.
- One ablation on the same scenarios: stock constants, belief, belief and V2X. The differences are
  the value of information, measured.
- At deployment rates: 10 Hz perception with a realistic delay, not ground truth at 20 Hz, or the
  belief converges faster than it would on the road.
- Open: the real perception delay on our stack; whether lane change and obstacle cruise take
  parameter changes at run time.

## What scensim brings

| needed for the method | scensim today |
|---|---|
| several AVs in one world, each a real stack | CARLA: the ego plus background AVs, each an Autoware stack in its own ROS domain, reached through the agent relay (`multi_av/town01_two_av.xosc`, `just two-av`, `just bg-av`). Isaac: open (roadmap 039) |
| a strategy plugged into a car | `scensim_agent.Agent`: any Python policy drives a car; Autoware's RTC (operator approval at intersection, lane change and other modules) is reachable through `on_cooperate` / `on_cooperate_auto`, which is a go-now / wait control on a real stack once the module's `enable_rtc` is on (Autoware 1.5's default config has it off for lane change) |
| contexts as parameters | OpenSCENARIO parameters and SSv2's parameter sweeps; roadmap 025's genomes, including a cut-in family (gap 5-40 m, merge length 6-30 m, speed 2-10 m/s, acceleration 1-5 m/s^2) |
| choosing the next runs | 025's ask/tell search (`scripts/gap_search.py`: corpus, archive, Exp3, Thompson sampling; tested offline) and its exploit pipeline (bisecting to a boundary); the same machinery can sample an equilibrium boundary instead of a gap boundary |
| payoffs from physics | the per-frame trace (024: every entity's pose and velocity), contacts with impulse (021), JUnit verdicts; TTC, PET, delay and comfort follow from the trace |
| car models, with true capabilities | `sim_vehicle_spec`: one spec per car (mass, size, brake torque and lag, motor limits and lag, steering limit and lag); three cars ship (`tesla_model3`, `drift_mustang`, `newslab_golfcart`), chosen per entity from the catalog. `just vehicle-check` measures each car's dynamics, so F(theta) is measured, not assumed |
| two simulators | CARLA 0.9.16 and Isaac Sim 6.1 behind one world protocol: a payoff map measured on both says whether it depends on the simulator |

What it adds that the literature lacks: payoffs measured on **full production stacks** (perception,
localization, planning, control), with physical contacts, rather than on policy abstractions. The
heterogeneous meeting of two different stacks or configurations, which the survey found no work on,
is a configuration change in scensim, not a new simulator.

**Gaps to close first:**

- **Cost.** One run is 1-3 minutes with a real Autoware ego, so a few hundred a night per
  pipeline, and two stacks on one host cost about twice the CPU and GPU. Payoff maps need far more
  points than that. Hence fidelities: (1) a kinematic simulator in Python for the theory and for
  RL best responses; (2) scensim with light agents (a `scensim_agent` policy on ground-truth localization,
  no perception) for broad maps; (3) scensim with full Autoware stacks to validate the boundaries the
  lighter layers found. Parallel pipelines (roadmap 026) multiply the third.
- **Strategy knobs on Autoware.** Which of Autoware 1.5's parameters make a car "yield" or "hold"
  for a cut-in, beyond RTC's timing, is not established; obstacle cruise's following distance and
  the lane change module's safety margins are candidates. To be measured before they are called
  strategies.
- **The map.** A cut-in needs two lanes in one direction; Town01 has one each way (025's i7 pulls
  out from the shoulder instead). Town05 has multi-lane roads and an Autoware map; Town04 has no
  map here.
- **Isaac.** Several Autoware stacks on Isaac (039) is open; CARLA carries the first experiments.
- **SSv2.** SSv2 drives one scenario ego; the other AVs are entities with the `agent` controller,
  reached through the relay. Whether every SSv2 condition sees them as it sees the ego is not
  checked.

## A first experiment

The user's own 2 x 2 cut-in, end to end, before any N-vehicle work:

1. **Theory baseline.** The cut-in in a kinematic model: A's (g_A, a_A), B's yield or hold;
   payoffs from progress, minimum TTC and contact. Compute the equilibrium map over (gap, speed
   difference) analytically or by grid. This is the prediction.
2. **scensim, light agents.** The same game on Town05 in CARLA, A and B as `scensim_agent`
   policies with the same parameters; the context swept by 025's search, aimed at the equilibrium
   boundaries. Compare the measured map with the predicted one.
3. **scensim, Autoware.** B a full Autoware stack, A a light agent or a second Autoware with RTC timing
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
37. Kearns, Littman, Singh, "Graphical Models for Game Theory", UAI 2001 (UNVERIFIED, from memory).
38. InterHub, multi-vehicle interaction events across datasets, 2024. https://arxiv.org/abs/2411.18302
39. Transferable behaviour prediction at INTERACTION intersections, 2022. https://arxiv.org/abs/2202.05140
40. Yehudai, Fetaya, Meirom, Chechik, Maron, "From Local Structures to Size Generalization in Graph Neural Networks", ICML 2021 (author list UNVERIFIED).
41. Shalev-Shwartz, Shammah, Shashua, "On a Formal Model of Safe and Scalable Self-driving Cars", 2017. https://arxiv.org/abs/1708.06374
42. TTC-based barrier functions for stochastic systems, 2026. https://arxiv.org/abs/2609.17347
43. Rowe et al., "FJMP: Factorized Joint Multi-Agent Motion Prediction", CVPR 2023.
44. Sun et al., "M2I: From Factored Marginal Trajectory Prediction to Interactive Prediction", CVPR 2022.
45. Moussaïd, Helbing, Theraulaz, "How simple rules determine pedestrian behavior and crowd disasters", PNAS 108(17), 2011.
46. Long et al., "PERL: physics-enhanced residual learning" for car following, 2023. https://arxiv.org/abs/2309.15284
47. Mo, Shi, Di, physics-informed deep learning for car following, TR-C 2021. https://arxiv.org/abs/2012.13376
48. Pair and three-body interactions fitted to cell trajectories, Physical Review Research 2025 (UNVERIFIED citation details).
49. Lasry, Lions, "Mean field games", Japanese Journal of Mathematics 2, 2007 (UNVERIFIED, from memory).
50. Huang, Malhamé, Caines, Nash certainty equivalence, Communications in Information and Systems 6(3), 2006 (UNVERIFIED, from memory).
51. Chevalier, Le Ny, Malhamé, "A micro-macro traffic model based on mean-field games", ACC 2015.
52. Festa, Göttlich, "A mean field game approach for multi-lane traffic management", 2017. https://arxiv.org/abs/1711.04116
53. Multi-class Nash mean-field games for mixed human and AV traffic, TR-B 2025. https://arxiv.org/abs/2306.13543
54. Nourian, Caines, "ε-Nash mean field game theory for nonlinear stochastic dynamical systems with major and minor agents", SICON 2013. https://arxiv.org/abs/1209.5684
55. Yang et al., "Mean Field Multi-Agent Reinforcement Learning", ICML 2018. https://arxiv.org/abs/1802.05438
56. Cabannes et al., "Solving N-player dynamic routing games with congestion: a mean field approach", AAMAS 2022. https://arxiv.org/abs/2110.11943
57. Lazar, Coogan, Pedarsani, routing with mixed autonomy, unbounded price of anarchy, 2018-2019 (venue UNVERIFIED).
58. Nair, Mahmassani, Miller-Hooks, "A porous flow approach to modeling heterogeneous traffic in disordered systems", TR-B 45(9), 2011.
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

53. Zhang, Han, Grammatico, lane merging as a matrix game with branch MPC, 2023. https://arxiv.org/abs/2311.14916
54. Kentucky zipper merge field study, 2017. https://rosap.ntl.bts.gov/view/dot/35694/dot_35694_DS1.pdf
55. Iowa zipper merge study, 2024. https://rosap.ntl.bts.gov/view/dot/80870
56. Repeated-game freeway lane-change model, 2020. https://arxiv.org/abs/2002.02529
57. Krajewski et al., highD, 2018. https://arxiv.org/abs/1810.05642
58. exiD, inD, rounD (LevelX; original venues UNVERIFIED). https://www.exid-dataset.com
59. Zhan et al., INTERACTION dataset, 2019. https://arxiv.org/abs/1910.03088
60. GAMEPLAN, 2021. https://arxiv.org/abs/2109.01896
61. Li, Yao, Kolmanovsky, Atkins, Girard, "Game-Theoretic Modeling of Multi-Vehicle Interactions at Uncontrolled Intersections", 2019. https://arxiv.org/abs/1904.05423
62. Quantal response equilibrium model of the unprotected left turn, 2025. https://arxiv.org/abs/2507.03002
63. Tian et al., adaptive game-theoretic roundabout entry, 2018. https://arxiv.org/abs/1810.00829
64. Stackelberg and grand coalition for CAVs at an unsignalized roundabout, 2021. https://arxiv.org/abs/2103.07910
65. Motorcycle queue storage zones and cars' start-up lost time, Thailand (EASTS). https://www.jstage.jst.go.jp/article/easts/6/0/6_0_137/_article
66. Agarwal, Lämmel, seepage queue model, https://depositonce.tu-berlin.de//handle/11303/7627; Gashaw, Goatin, Härri, porous-medium model of two-wheelers, https://www.eurecom.edu/en/node/4902956
67. HetroD, Taiwanese urban drone dataset, 2026 (licence UNVERIFIED). https://hetrod-data.github.io/HetroD/
68. Camara, Fox, Sequential Chicken (Fox et al. 2018 venue UNVERIFIED). https://eprints.whiterose.ac.uk/159638/1/Camara%26Fox2020game_UK_RAS_2020.pdf
69. Johora, Müller, "Modeling Interactions of Multimodal Road Users in Shared Spaces", ITSC 2018. https://arxiv.org/abs/2107.02083
70. Millard-Ball, "Pedestrians, Autonomous Vehicles, and Cities", JPER 2018 (paper UNVERIFIED; popular-press summary). https://www.scientificamerican.com/article/how-pedestrians-will-defeat-autonomous-vehicles/
71. Pascucci et al., Hamburg-Bergedorf shared space data. https://doi.org/10.24355/dbbs.084-202111081809-0
72. Yang, Li, Redmill, Özgüner, DUT and CITR, IV 2019. https://arxiv.org/abs/1902.00487
73. Golchoubian et al., uncertainty-aware DRL for an AV in a shared-space crowd, T-IV 2024. https://arxiv.org/abs/2405.13969

Cited from memory, not re-checked by search:

50. Kearns, Littman, Singh, "Graphical Models for Game Theory", UAI 2001.
51. Daskalakis, Papadimitriou, "Computing Equilibria in Anonymous Games", FOCS 2007.
52. McMahan, Gordon, Blum, "Planning in the Presence of Cost Functions Controlled by an Adversary" (double oracle), ICML 2003.

## The protocol framework: related work (2026-11-15)

Roadmap 074's study is reframed as **a protocol for a shared resource**: a conflict zone is a
critical section, the cars are processes, and the study's layers are (1) a specification (a
rulebook of ranked rules plus STL robustness; S0's risk ρ is minus the robustness of
"always (TTC ≥ 3 ∧ headway ≥ 1 ∧ PET ≥ 1.5 ∧ clearance ≥ 1)"), (2) safety (reachability,
barrier functions, RSS as the feasible set), (3) coordination (distributed mutual exclusion with
symmetry breaking, tolerant to mixed traffic in the BAR sense, verified for safety and
liveness), (4) incentives (correlated equilibrium, ESS, cheap talk as one check inside the
protocol), (5) flow (max-plus delay composition, string stability, LWR, bottleneck capacity),
(6) execution (contingency MPC, legible motion as the probe, intent messages) and (7)
validation (conformal prediction for arena-vs-real agreement, falsification). This section
lists the closest work per layer, then who has combined layers, then the gap. Game-theoretic
planning (Sadigh, Fisac, Wang, Schwarting, potential games), EGTA/PSRO, rare-event testing
with O'Kelly/D2RL, and DoppelTest are already covered above and are only pointed to here
("survey [n]" is the numbering of the references above; [F*] is this section's list).

Checked by web search and Crossref on 2026-11-15; anything not confirmed against a primary
record is marked UNVERIFIED.

### 1. Specification: rulebooks and STL

- **Rulebooks** [F1]: rules as violation metrics in a pre-order by priority; planning picks the
  trajectory that is lexicographically least violating. Motional followed with a hazard-based
  SOTIF argument over rulebooks [F2] and with "The Reasonable Crowd" [F3], 92 scenarios ranked by
  people and fitted by a 14-rule rulebook. Minimum-violation planning [F4] is the theory under
  it. Ours uses a rulebook to *score* runs of two full stacks against each other, not to plan.
- **STL robustness** [F5] gives the signed margin we use for ρ; differentiable STL (STLCG [F6])
  and rule hierarchies turned into a receding-horizon reward [F7] put it inside a planner.
- **Formalized traffic rules**: German interstate [F8] and intersection [F9] rules in temporal
  logic (TUM, CommonRoad [F10]), checked on recorded traffic; Isabelle/HOL monitors [F11]. These
  formalize *the law*; our S0 formula is a safety-surrogate envelope, not a traffic rule, and
  the right-of-way rules of [F9] are a ready source for the convention table.

### 2. Safety: the feasible set

- **HJ reachability** [F12] and **CBF-QPs** [F13] are the standard safety filters; RSS is
  survey [41]. Online verification of every planned trajectory with a fail-safe fallback,
  under "legal safety" (others obey the rules) [F14], is the closest to Autoware's shield.
- **Reachability inside interaction planning**: Leung et al. [F15] and Wang, Leung, Pavone
  [F16] give planner and controller one HJ notion of safety in multi-agent driving. Ours
  takes the shield as given (AEB, obstacle stop) and uses the feasible set only to prune
  strategies, which is the standard role.

### 3. Coordination: mutual exclusion, symmetry, faults, verification

- **Intersections as mutual exclusion.** Wu, Zhang, Luo, Cao [F17] state intersection control
  over V2V as a new variant of distributed mutual exclusion and give algorithms for it; this is
  our framing, a decade earlier. Dresner and Stone's AIM [F18] is the centralized reservation
  version (it emulates a light or stop sign, and has a human-driver extension). Azimi et al.
  [F19] and Aoki and Rajkumar [F20, F21] give V2V protocols with deadlock-freedom arguments;
  [F21] handles mixed human and automated traffic by letting human drivers follow the lights.
- **Knowledge-based intersection protocols** (Alpturer, Halpern, van der Meyden, DISC 2024
  [F22]): a global policy compiled into protocols vehicles run, proven safe, live and optimal,
  also with faulty (lossy) and legacy (limited-communication) vehicles. The closest
  distributed-computing treatment of our layer 3; it is abstract, no vehicle dynamics.
- **Symmetry breaking.** Angluin [F23] is the root impossibility for anonymous identical
  processes. In driving: the symmetric four-way case deadlocks until one car probes forward
  (Li et al., survey [61]); Google's prototypes waited at four-way stops while others edged out,
  and were tuned in 2015 to creep, a probe as a convention [F24] (press, not a paper). Learned
  conventions [F25] show self-play picks a convention the real population does not use. Social
  laws [F26] are the AI version of "design the convention off-line".
- **Byzantine, rational and mixed participants.** BAR [F27] and (k,t)-robust equilibria [F28]
  are the distributed-systems models. In vehicles: Byzantine consensus for connected vehicles
  [F29], an intersection game with "malicious" cars that assume priority [F30], and a single
  spoofing vehicle that raised delay 23.4 % under the US DOT's I-SIG signal control [F31]. No
  vehicle work found uses the BAR model itself; mixed traffic is usually "humans follow lights"
  [F21] or "legacy vehicles" [F22].
- **Verified protocols.** KeYmaera proofs of distributed ACC for any number of cars on a straight
  highway [F32], a two-car intersection [F33], its efficiency cost under V2V loss [F34], and
  ground-robot obstacle avoidance with liveness [F35]. Spatial logics (MLSL) for lane-change
  safety [F36, F37] and urban crossing controllers verified in UPPAAL for liveness and fairness
  ("no queue jumping") [F38, F39]. UPPAAL-SMC on an intersection manager [F40]; Gwendolen plus
  UPPAAL on platoon join/leave [F41]. No TLA+ work on vehicle protocols was found. All verify
  a model of a protocol, none a running production stack.

### 4. Incentives

- **Correlated equilibrium at intersections**: a manager samples an optimal CE of a graphical
  game [F42]; CE trajectory planning at V2I intersections [F43]. Both make the recommendation a
  central device; ours makes it a convention keyed on an observable role, which needs no device.
- **Cheap talk and signalling between road users**: non-cooperative models of whether to signal
  [F44, F45] find selective disclosure pays, consistent with C6 (intent pays, capability does not).
- **Evolutionary analysis**: real lane changes (Waymo Open Motion) classified as Stag Hunt or
  Prisoner's Dilemma in 4-11 % of events, with an EGT simulation [F46]. No paper found applies
  Selten's result (only pure, role-keyed strategies are ESS in asymmetric contests) to yielding
  conventions; that use is ours, as far as we can tell.
- Game-theoretic AV planning itself: survey [1, 2, 8, 10, 17].

### 5. Flow

- **Max-plus** is the algebra of timed event graphs [F47]. In traffic it is used for signal
  switching sequences (De Schutter's group, UNVERIFIED citation detail) and, as min-plus, for
  LWR network models [F48]. No paper found writes the delay of a queue of automated cars
  crossing a conflict zone as a max-plus recursion, as our N-car composition does; the
  recursion itself is textbook.
- **Capacity of slot-based and reservation intersections**: queueing analysis of slot-based
  crossings against lights [F49]; reservation control can be worse than signals on asymmetric
  networks under route choice [F50]. These are the "a convention does not make capacity" check.
- **String stability** [F51] and LWR / cell transmission [F52, F53] are standard; Stern et al.
  [F54] measured one automated car damping stop-and-go waves in a real ring.

### 6. Execution

- **Contingency planning**: multiple contingency paths over predicted obstacle modes [F55];
  contingency MPC with a second, emergency horizon [F56]; contingency games survey [12]. Our
  "commit, hold a branch" is the same structure.
- **Legibility**: Dragan's legibility vs predictability [F57]; legible MPC for highway lane
  changes [F58]; a VR study that lateral offsets make lane-change intent legible but read as
  aggressive [F59] (venue UNVERIFIED). Using a legible nudge as a *probe* whose response is
  measured is the information-gathering idea of survey [45].

### 7. Validation

- **Conformal prediction**: planning with conformal prediction regions (in CARLA) [F60];
  conformal assurance for warning systems from simulation [F61]; reachability under a bounded
  sim-to-real distribution shift [F62]. Using conformal intervals for *arena-vs-stack agreement*
  (D2) is an application of these, not a new method.
- **Falsification / stress testing**: the black-box validation survey [F63] and adaptive stress
  testing [F64]; Scenic/VerifAI's sim-to-track study [F65] measured the sim-to-real gap
  systematically, the closest precedent for D2.

### Standards and industry

- **SAE J3216** (2021) [F66] defines cooperation classes: A status-sharing, B intent-sharing,
  C agreement-seeking, D prescriptive. Our convention plus intent message is class B used to
  reach a class C outcome without a negotiation round, which is why the probe is needed.
- **ETSI ITS**: the Manoeuvre Coordination Service pre-standardization study TR 103 578 V2.1.1
  (2024) [F67] has agreement-seeking and prescriptive concepts, the specification TS 103 561 is
  in work (status UNVERIFIED); the Collective Perception Service is TS 103 324 V2.1.1 (2023)
  [F68], class A.
- **Safety standards**: ISO 21448 (SOTIF) [F69] treats other road users' behaviour as triggering
  conditions of the ego's functional insufficiency, not as a protocol partner (our reading);
  ISO 34502:2022 [F70] is scenario-based evaluation for highway ADS and excludes traffic flow;
  UL 4600 Ed. 3 (2023) [F71] is a safety-case standard whose scope includes platoons. None
  specifies how an intent message changes the ego's obligations.

### Combinations

| who | layers combined | what is missing for this study |
|---|---|---|
| Loos, Platzer [F32-F34] | 2 + 3, proven for N cars | straight road or one intersection; no incentives; model, not a stack |
| Hilscher, Olderog, Schwammberger [F36-F39] | 1 (spatial logic) + 3 (UPPAAL safety, liveness, fairness) | abstract cars; no measurement |
| Wu et al. [F17]; Aoki, Rajkumar [F20, F21] | 3 + 5 (throughput), mixed traffic in [F21] | no game, no formal proof of the mixed case, simulated vehicles |
| Alpturer, Halpern, van der Meyden [F22] | 3 with faults and legacy vehicles | no dynamics, no incentives |
| Pruekprasert et al. [F30] | 4 + 3 (priority games with rule-breakers) | small models, no shield |
| Adkins et al. [F42]; Wang, Yi, Hong [F43] | 4 (CE) + 3 (a central manager) | needs infrastructure; no role-keyed convention |
| Leung, Wang, Pavone [F15, F16]; survey [44, 46] | 2 + 4 (reachability in interaction games) | two cars, no protocol |
| Veer et al. [F7]; Collin et al. [F2] | 1 + 6 (rules in the planner) | single ego |
| Fremont et al. [F65] | 7 + 1 (formal scenarios, sim-to-track) | single stack, no interaction protocol |
| Yu et al. 2026 [F72] | 3 (RSU coordination) measured against Autoware in CARLA | Autoware is the baseline (it showed "repeated mutual yielding near conflict points"); learned controller, no formal layer |

Nobody found combines more than two or three layers, and none of the combinations above takes a
full production stack as the process.

### Gap

1. **The processes are full Autoware stacks.** Every verified or game-theoretic protocol above
   runs on a model car or a planner abstraction; the nearest measured work uses Autoware as a
   baseline [F72] or as the host of one cooperative planner [F73]. Measuring a convention's
   safety and liveness on several complete stacks, with the shield underneath, is new.
2. **Mixed traffic as BAR.** Stock Autoware (reads no message), a convention-following stack and
   an RL deviator map onto altruistic/obedient, Byzantine-to-the-convention and rational
   participants. Faulty or legacy vehicles [F22] and malicious ones [F30, F31] exist separately;
   the three together, with "never worse than stock against a non-complier" (D7) as the
   requirement, were not found.
3. **Conventions measured against a real stack.** Symmetry breaking by role or randomness is
   theory [F23] and anecdote [F24]; ESS arguments for role-keyed yielding were not found applied
   to driving. Measuring which convention is stable, with an arena model checked against the
   stack by conformal intervals, is the study's own.

The gap is small in theory, wide in measurement. Each layer is standard; the framing of an
intersection as distributed mutual exclusion is in [F17] (2015), safety and liveness proofs
of such protocols exist [F22, F32-F39], and CE coordination at intersections exists [F42, F43].
What is new is putting them in one protocol whose processes are real stacks, and measuring it.

**Standard practice vs new, per layer.** Standard: 1 (rulebooks, STL robustness), 2 (the
shield, RSS, reachability), 6 (contingency MPC, intent messages per J3216 class B), 7
(falsification; conformal regions). Established in research, not in practice: 3's protocol
proofs (UPPAAL, KeYmaera), 4's CE at intersections, 5's capacity analyses. New, as far as
found: the BAR framing of mixed traffic, ESS-based convention selection, max-plus delay
composition for a queue of automated cars at a conflict zone, legible motion used as a probe
whose response is measured, and conformal intervals for arena-vs-stack agreement.

### References (protocol framework)

Verified by Crossref, the publisher or arXiv on 2026-11-15 unless marked.

- F1. Censi, Slutsky, Wongpiromsarn, Yershov, Pendleton, Fu, Frazzoli, "Liability, Ethics, and Culture-Aware Behavior Specification using Rulebooks", ICRA 2019, pp. 8536-8542. https://doi.org/10.1109/ICRA.2019.8794364 (arXiv 1902.09355)
- F2. Collin, Bilka, Pendleton, Duintjer Tebbens, "Safety of the Intended Driving Behavior Using Rulebooks", IEEE IV 2020, pp. 136-143. https://doi.org/10.1109/IV47402.2020.9304588
- F3. Helou et al., "The Reasonable Crowd: Towards evidence-based and interpretable models of driving behavior", IROS 2021, pp. 6708-6715. https://doi.org/10.1109/IROS51168.2021.9635960 (arXiv 2107.13507)
- F4. Wongpiromsarn, Slutsky, Frazzoli, Topcu, "Minimum-Violation Planning for Autonomous Systems: Theoretical and Practical Considerations", ACC 2021, pp. 4866-4872. https://doi.org/10.23919/ACC50511.2021.9483174
- F5. Donzé, Maler, "Robust Satisfaction of Temporal Logic over Real-Valued Signals", FORMATS 2010, LNCS 6246, pp. 92-106. https://doi.org/10.1007/978-3-642-15297-9_9
- F6. Leung, Aréchiga, Pavone, "Back-Propagation Through Signal Temporal Logic Specifications: Infusing Logical Structure into Gradient-Based Methods", WAFR 2020; IJRR 42(6), 2023. https://arxiv.org/abs/2008.00097
- F7. Veer, Leung, Cosner, Chen, Karkus, Pavone, "Receding Horizon Planning with Rule Hierarchies for Autonomous Vehicles", ICRA 2023, pp. 1507-1513. https://doi.org/10.1109/ICRA48891.2023.10160622 (arXiv 2212.03323)
- F8. Maierhofer, Rettinger, Mayer, Althoff, "Formalization of Interstate Traffic Rules in Temporal Logic", IEEE IV 2020, pp. 752-759. https://doi.org/10.1109/IV47402.2020.9304549
- F9. Maierhofer, Moosbrugger, Althoff, "Formalization of Intersection Traffic Rules in Temporal Logic", IEEE IV 2022. https://doi.org/10.1109/IV51971.2022.9827153
- F10. Althoff, Koschi, Manzinger, "CommonRoad: Composable Benchmarks for Motion Planning on Roads", IEEE IV 2017, pp. 719-726. https://doi.org/10.1109/IVS.2017.7995802
- F11. Rizaldi, Keinholz, Huber, Feldle, Immler, Althoff, Hilgendorf, Nipkow, "Formalising and Monitoring Traffic Rules for Autonomous Vehicles in Isabelle/HOL", iFM 2017, LNCS, pp. 50-66 (last two authors from memory). https://doi.org/10.1007/978-3-319-66845-1_4
- F12. Bansal, Chen, Herbert, Tomlin, "Hamilton-Jacobi Reachability: A Brief Overview and Recent Advances", CDC 2017 tutorial. https://arxiv.org/abs/1709.07523
- F13. Ames, Xu, Grizzle, Tabuada, "Control Barrier Function Based Quadratic Programs for Safety Critical Systems", IEEE TAC 62(8), 2017, pp. 3861-3876. https://arxiv.org/abs/1609.06408
- F14. Pek, Manzinger, Koschi, Althoff, "Using online verification to prevent autonomous vehicles from causing accidents", Nature Machine Intelligence 2(9), 2020, pp. 518-528. https://doi.org/10.1038/s42256-020-0225-y
- F15. Leung, Schmerling, Zhang, Chen, Talbot, Gerdes, Pavone, "On infusing reachability-based safety assurance within planning frameworks for human-robot vehicle interactions", IJRR 39, 2020, pp. 1326-1345. https://doi.org/10.1177/0278364920950795
- F16. Wang, Leung, Pavone, "Infusing Reachability-Based Safety into Planning and Control for Multi-agent Interactions", IROS 2020, pp. 6252-6259. https://arxiv.org/abs/2008.00067
- F17. Wu, Zhang, Luo, Cao, "Distributed Mutual Exclusion Algorithms for Intersection Traffic Control", IEEE TPDS 26(1), 2015. https://research.polyu.edu.hk/en/publications/distributed-mutual-exclusion-algorithms-for-intersection-traffic-/
- F18. Dresner, Stone, "A Multiagent Approach to Autonomous Intersection Management", JAIR 31, 2008, pp. 591-656. https://doi.org/10.1613/jair.2502
- F19. Azimi, Bhatia, Rajkumar, Mudalige, "Reliable Intersection Protocols Using Vehicular Networks", ICCPS 2013, pp. 1-10. https://dblp.org/rec/conf/iccps/AzimiBRM13
- F20. Aoki, Rajkumar, "A Configurable Synchronous Intersection Protocol for Self-Driving Vehicles", RTCSA 2017, pp. 1-11. https://doi.org/10.1109/RTCSA.2017.8046306
- F21. Aoki, Rajkumar, "V2V-based Synchronous Intersection Protocols for Mixed Traffic of Human-Driven and Self-Driving Vehicles", RTCSA 2019, pp. 1-11. https://doi.org/10.1109/RTCSA.2019.8864572
- F22. Alpturer, Halpern, van der Meyden, "A Knowledge-Based Analysis of Intersection Protocols", DISC 2024 (full version). https://arxiv.org/abs/2408.09499
- F23. Angluin, "Local and Global Properties in Networks of Processors", STOC 1980, pp. 82-93. https://doi.org/10.1145/800141.804655
- F24. Google self-driving prototypes tuned to creep at stop signs, Fortune 2015-09-29 citing the Wall Street Journal; AP 2014 on waiting at four-way stops (press, not peer-reviewed). https://fortune.com/2015/09/29/google-self-driving-cars-humans
- F25. Lerer, Peysakhovich, "Learning Existing Social Conventions via Observationally Augmented Self-Play", AIES 2019. https://arxiv.org/abs/1806.10071
- F26. Shoham, Tennenholtz, "On social laws for artificial agent societies: off-line design", Artificial Intelligence 73, 1995, pp. 231-252. https://doi.org/10.1016/0004-3702(94)00007-N
- F27. Aiyer, Alvisi, Clement, Dahlin, Martin, Porth, "BAR Fault Tolerance for Cooperative Services", SOSP 2005, pp. 45-58. https://www.cs.utexas.edu/~lorenzo/papers/sosp05.pdf
- F28. Abraham, Dolev, Gonen, Halpern, "Distributed Computing Meets Game Theory: Robust Mechanisms for Rational Secret Sharing and Multiparty Computation", PODC 2006, pp. 53-62. https://www.cs.huji.ac.il/~dolev/pubs/dg53.pdf
- F29. Liu, Lin, Kang, Shiraishi, Blough, "A Byzantine-Tolerant Distributed Consensus Algorithm for Connected Vehicles Using Proof-of-Eligibility", MSWiM 2019, pp. 225-234. https://doi.org/10.1145/3345768.3355910 (application details UNVERIFIED)
- F30. Pruekprasert, Zhang, Dubut, Huang, Kishida, "Decision Making for Autonomous Vehicles at Unsignalized Intersection in Presence of Malicious Vehicles", ITSC 2019. https://arxiv.org/abs/1904.10158
- F31. Chen, Yin, Feng, Mao, Liu, "Exposing Congestion Attack on Emerging Connected Vehicle based Traffic Signal Control", NDSS 2018. https://doi.org/10.14722/ndss.2018.23222
- F32. Loos, Platzer, Nistor, "Adaptive Cruise Control: Hybrid, Distributed, and Now Formally Verified", FM 2011, LNCS 6664, pp. 42-56. https://logic.kastel.kit.edu/ap/pub/dccs.pdf
- F33. Loos, Platzer, "Safe Intersections: At the Crossing of Hybrid Systems and Verification", ITSC 2011, pp. 1181-1186. https://doi.org/10.1109/ITSC.2011.6083138
- F34. Loos, Witmer, Steenkiste, Platzer, "Efficiency Analysis of Formally Verified Adaptive Cruise Controllers", ITSC 2013, pp. 1565-1570. https://doi.org/10.1109/ITSC.2013.6728453
- F35. Mitsch, Ghorbal, Vogelbacher, Platzer, "Formal verification of obstacle avoidance and navigation of ground robots", IJRR 36(12), 2017, pp. 1312-1340. https://doi.org/10.1177/0278364917733549
- F36. Hilscher, Linker, Olderog, Ravn, "An Abstract Model for Proving Safety of Multi-lane Traffic Manoeuvres", ICFEM 2011, LNCS 6991, pp. 404-419. https://doi.org/10.1007/978-3-642-24559-6_28 (its reservation/claim mechanism described from memory)
- F37. Bochmann, Hilscher, Linker, Olderog, "Synthesizing and verifying controllers for multi-lane traffic maneuvers", Formal Aspects of Computing 29, 2017, pp. 583-600. https://doi.org/10.1007/s00165-017-0424-4
- F38. Schwammberger, "Introducing Liveness into Multi-lane Spatial Logic lane change controllers using UPPAAL", EPTCS 269, 2018, pp. 17-31. https://doi.org/10.4204/EPTCS.269.3
- F39. Schwammberger, "Proving properties of autonomous car manoeuvres in urban traffic", it - Information Technology 63(4), 2021, pp. 253-263. https://doi.org/10.1515/itit-2021-0007
- F40. Chouhan, Banda, "Formal Verification of Heuristic Autonomous Intersection Management Using Statistical Model Checking", Sensors 20(16):4506, 2020. https://doi.org/10.3390/s20164506
- F41. Kamali, Dennis, McAree, Fisher, Veres, "Formal verification of autonomous vehicle platooning", Science of Computer Programming 148, 2017, pp. 88-106. https://doi.org/10.1016/j.scico.2017.05.006
- F42. Adkins, Mount, Zhang, "A Game-Theoretic Approach for Minimizing Delays in Autonomous Intersections", Traffic and Granular Flow '17, Springer 2019, pp. 131-139. https://doi.org/10.1007/978-3-030-11440-4_16
- F43. Wang, Yi, Hong, "Multi-Vehicle Trajectory Planning at V2I-Enabled Intersections Based on Correlated Equilibrium", RA-L 9, 2024, pp. 8346-8353. https://doi.org/10.1109/LRA.2024.3444715
- F44. Bitar, Crusat, Watling, "Modelling Implicit and Explicit Communication Between Road Users from a Non-Cooperative Game-Theoretic Perspective: An Exploratory Study", VEHITS 2024, pp. 456-464. https://doi.org/10.5220/0012729000003702
- F45. Bitar, Solernou Crusat, Romano, Watling, "To Signal or Not to Signal? A Non-cooperative Game-Theoretic Approach to Discretionary Communication Between Road Users", SN Computer Science, 2025. https://doi.org/10.1007/s42979-025-04533-w
- F46. Chung, Talebpour, Hamdar, "Characterizing Lane-Changing Behavior in Mixed Traffic", 2025 (preprint). https://arxiv.org/abs/2512.07219
- F47. Baccelli, Cohen, Olsder, Quadrat, "Synchronization and Linearity: An Algebra for Discrete Event Systems", Wiley 1992 (free web edition 2001).
- F48. Farhi, Haj-Salem, Lebacque, "An algebraic approach for modeling and simulation of road traffic networks", 2014 (preprint). https://arxiv.org/abs/1406.6929
- F49. Tachet, Santi, Sobolevsky, Reyes-Castro, Frazzoli, Helbing, Ratti, "Revisiting Street Intersections Using Slot-Based Systems", PLoS ONE 11(3):e0149607, 2016. https://doi.org/10.1371/journal.pone.0149607
- F50. Levin, Boyles, Patel, "Paradoxes of reservation-based intersection controls in traffic networks", Transportation Research Part A 90, 2016, pp. 14-25. https://doi.org/10.1016/j.tra.2016.05.013
- F51. Ploeg, van de Wouw, Nijmeijer, "Lp String Stability of Cascaded Systems: Application to Vehicle Platooning", IEEE TCST 22(2), 2014, pp. 786-793.
- F52. Lighthill, Whitham, "On kinematic waves II: A theory of traffic flow on long crowded roads", Proc. Royal Society A 229(1178), 1955 (start page 281 or 317 per source); Richards, "Shock waves on the highway", Operations Research 4, 1956.
- F53. Daganzo, "The cell transmission model: A dynamic representation of highway traffic consistent with the hydrodynamic theory", Transportation Research Part B 28(4), 1994, pp. 269-287.
- F54. Stern et al., "Dissipation of stop-and-go waves via control of autonomous vehicles: Field experiments", Transportation Research Part C 89, 2018, pp. 205-221. https://doi.org/10.1016/j.trc.2018.02.005
- F55. Hardy, Campbell, "Contingency Planning Over Probabilistic Obstacle Predictions for Autonomous Road Vehicles", IEEE T-RO 29, 2013, pp. 913-929. https://doi.org/10.1109/TRO.2013.2254033
- F56. Alsterda, Brown, Gerdes, "Contingency Model Predictive Control for Automated Vehicles", ACC 2019, pp. 717-722. https://doi.org/10.23919/ACC.2019.8815260 (arXiv 1903.08818)
- F57. Dragan, Lee, Srinivasa, "Legibility and Predictability of Robot Motion", HRI 2013, pp. 301-308. https://doi.org/10.1109/HRI.2013.6483603
- F58. Brüdigam, Ahmic, Leibold, Wollherr, "Legible Model Predictive Control for Autonomous Driving on Highways", IFAC-PapersOnLine 51, 2018, pp. 215-221. https://doi.org/10.1016/j.ifacol.2018.11.016
- F59. Lee, Britten, Block, Pandya, Jung, Schmitt, "Coming In! Communicating Lane Change Intent in Autonomous Vehicles", HRI 2023 companion (venue UNVERIFIED). https://motional.com/news/coming-communicating-lane-change-intent-autonomous-vehicles
- F60. Lindemann, Cleaveland, Shim, Pappas, "Safe Planning in Dynamic Environments using Conformal Prediction", RA-L 8(8), 2023. https://arxiv.org/abs/2210.10254
- F61. Luo, Zhao, Kuck, Ivanovic, Savarese, Schmerling et al., "Sample-Efficient Safety Assurances Using Conformal Prediction", WAFR 2022; IJRR 43, 2024, pp. 1409-1424. https://doi.org/10.1177/02783649231221580
- F62. Hashemi, Lindemann, Deshmukh, "Statistical Reachability Analysis of Stochastic Cyber-Physical Systems Under Distribution Shift", IEEE TCAD 43, 2024, pp. 4250-4261. https://doi.org/10.1109/TCAD.2024.3438072
- F63. Corso, Moss, Koren, Lee, Kochenderfer, "A Survey of Algorithms for Black-Box Safety Validation of Cyber-Physical Systems", JAIR 72, 2021, pp. 377-428. https://doi.org/10.1613/jair.1.12716
- F64. Koren, Alsaif, Lee, Kochenderfer, "Adaptive Stress Testing for Autonomous Vehicles", IEEE IV 2018. https://arxiv.org/abs/1902.01909
- F65. Fremont, Kim, Pant, Seshia et al., "Formal Scenario-Based Testing of Autonomous Vehicles: From Simulation to the Real World", ITSC 2020. https://doi.org/10.1109/ITSC45102.2020.9294368 (arXiv 2003.07739)
- F66. SAE J3216_202107, "Taxonomy and Definitions for Terms Related to Cooperative Driving Automation for On-Road Motor Vehicles" (first issued 2020-05). https://saemobilus.sae.org/standards/j3216_202005-taxonomy-definitions-terms-related-cooperative-driving-automation-road-motor-vehicles
- F67. ETSI TR 103 578 V2.1.1 (2024-04), "Intelligent Transport Systems; Vehicular Communication; Informative report for the Manoeuvre Coordination Service" (title wording UNVERIFIED; content per the catalogue and secondary papers). https://www.etsi.org/deliver/etsi_tr/103500_103599/103578/02.01.01_60/
- F68. ETSI TS 103 324 V2.1.1 (2023-06), "Intelligent Transport Systems; Vehicular Communication; Basic Set of Applications; Collective Perception Service; Release 2". https://www.etsi.org/deliver/etsi_ts/103300_103399/103324/02.01.01_60/
- F69. ISO 21448:2022, "Road vehicles -- Safety of the intended functionality" (not fetched; cited from the standard's catalogue title).
- F70. ISO 34502:2022, "Road vehicles -- Test scenarios for automated driving systems -- Scenario based safety evaluation framework". https://www.iso.org/standard/78951.html
- F71. ANSI/UL 4600, "Standard for Evaluation of Autonomous Products", Ed. 3, 2023-03-17. https://ulse.org/ul-standards-engagement/autonomous-vehicle-technology/autonomous-trucking-incorporated-third
- F72. Yu, Wang, Li, Yu, Saad, Sakaguchi, "How Roadside Units Enhance Intersection Safety? Cooperative Autonomous Driving System Design and A Proof of Concept", 2026 (preprint; Autoware baseline in CARLA and on real vehicles). https://arxiv.org/abs/2608.09144
- F73. Hirata, Tsukada, Okumura, Tamura, Ochiai, Défago, "Roadside-assisted Cooperative Planning using Future Path Sharing for Autonomous Driving", VTC2021-Fall (Autoware + LGSVL). https://arxiv.org/abs/2108.04629
