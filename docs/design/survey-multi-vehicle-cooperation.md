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

## Theory formulation

User direction 2026-11-13: explore the theory formulation. This section states the game the rest of
the document measures, in the user's terms: a general strategy and payoff space, the car model as a
restriction, the other car's capability unknown, beliefs updated from observed behaviour. The
propositions marked as such are standard results or direct consequences; the ones marked
*hypothesis* are for swb to test.

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
object swb measures.

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

### Capability in the game: two hypotheses for swb

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

The two-car games above are what swb can measure cheaply (C2a: thousands of runs per hour). An
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

**Higher-order residual.** Run the N-car game in the arena and in swb and measure
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
conflict-set game plus a mean-field remainder, and there is no error bound for the split. swb can
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

Per run, swb and the arena record the same raw measures from truth (`scripts/coop/measures.py`),
each defined over time rather than samples so that the arena's 0.05 s ticks and swb's 0.1 s logs
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

swb estimates $\hat J_i(\psi, c, t)$ from runs: a mean per profile, context and type pair with its
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
- **swb**: CARLA's highway towns (Town04, Town06) have ramps, but swb has no Autoware map of either
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
- **swb**: Town maps for the generic cases; the NTU twin (072, 059) for Taiwan's campus roads, where
  scooters are common. A swarm needs many scooter NPCs. SSv2 scripts its NPCs rather than letting
  them react, so a population that responds to the AV needs a reactive NPC controller. That
  controller does not exist in swb.

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
- **swb**: not ready. There is no map, SSv2's pedestrians are scripted, not reactive, and Autoware's
  perception of a dense crowd is untested. The first step is offline: fit the sequential chicken on
  Bergedorf or DUT, then add a social-force crowd controller as a world-side NPC driver.

### Summary

| setting | regime | start from | data | phenomenon | swb today |
|---|---|---|---|---|---|
| on-ramp, weaving | chain + small game | matrix game + branch MPC [53] | exiD, highD, INTERACTION | gap choice, courtesy shockwave | needs a Town04 map (031) |
| lane drop | N-player alternating priority (new) | none | highD, own sweeps | zipper vs early merge, defection | needs a lane-drop site |
| 4-way stop | small game, scheduling | concurrent leader-follower [61]; auction [60] | INTERACTION | symmetric deadlock, tie-breaks | needs an unsignalled junction |
| left turn vs queue | chain | QRE left turn [62], extended to a queue | INTERACTION, WOMD, SinD | gap acceptance vs queue length | ready (Town05, Town10) |
| roundabout | small game | adaptive level-k [63] | rounD, INTERACTION | entry assertiveness | ready (Town03) |
| Taiwan scooters | population | seepage models [66] + a game layer (new) | HetroD | swarm discharge, two-stage box, filtering | needs reactive scooter NPCs |
| night market | population, shared space | sequential chicken [68] + GSFM [69] | Bergedorf, DUT | frozen vehicle vs risk | needs a map and a reactive crowd |

The gaps in the literature are the lane-drop game, the left turn against a queue, any game model of
scooter traffic, and shared space at night-market density. swb reaches the left turn now; the lane
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
   yield, hold (B). Scored by the payoff table built offline in swb, indexed by the ego's car class,
   the belief about the other and the context (gap, speeds): a lookup of microseconds. An online
   game solve for a small group (iLQGames-class, under 50 ms warm for three cars) also fits the
   budget. Each decision is logged with its belief, the candidates' payoffs and the choice; the log
   is the explanation.
3. **Into Autoware**, from least to most invasive:
   - existing interfaces, no fork: go / wait through RTC (with `enable_rtc` on) or the
     `external_request_lane_change` module (already in swb's preset); yield / hold through the
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

### How swb tests it

- The same nodes run inside every ego stack in swb, so the experiments run deployment code.
- One ablation on the same scenarios: stock constants, belief, belief and V2X. The differences are
  the value of information, measured.
- At deployment rates: 10 Hz perception with a realistic delay, not ground truth at 20 Hz, or the
  belief converges faster than it would on the road.
- Open: the real perception delay on our stack; whether lane change and obstacle cruise take
  parameter changes at run time.

## What swb brings

| needed for the method | swb today |
|---|---|
| several AVs in one world, each a real stack | CARLA: the ego plus background AVs, each an Autoware stack in its own ROS domain, reached through the agent relay (`multi_av/town01_two_av.xosc`, `just two-av`, `just bg-av`). Isaac: open (roadmap 039) |
| a strategy plugged into a car | `swb_agent.Agent`: any Python policy drives a car; Autoware's RTC (operator approval at intersection, lane change and other modules) is reachable through `on_cooperate` / `on_cooperate_auto`, which is a go-now / wait control on a real stack once the module's `enable_rtc` is on (Autoware 1.5's default config has it off for lane change) |
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
2. **swb, light agents.** The same game on Town05 in CARLA, A and B as `swb_agent`
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
