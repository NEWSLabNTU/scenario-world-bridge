# Phase 093: Scenario description and the scenario-to-world compiler (design exploration)

Explore how a scenario is described for vehicle risk studies, and how the compiler turns
that description into a world on each target, through worked case studies. This phase
produces designs and specification drafts, not product code. Phase 092's specifications (S1
the sample record, S2 the compilable subset) and components (K1 the sampler, K2 the Isaac Lab
adapter, K3 replay) are built from what this phase settles.

**Status**: started 2026-11-20 (user: "Have a phase on the design exploration of the scenario
to world compiler. Explore the scenario description design on risk and case studies").

**Source**: phase 092's design and its two studies,
[survey-risk-categories.md](../design/survey-risk-categories.md) and
[survey-vehicle-risk-taxonomy.md](../design/survey-vehicle-risk-taxonomy.md). Decisions
already made by the user:

- scenario and world are separate, joined by the compiler (option A);
- the purposes are a realistic risk rate (A) and failure-mode search (B);
- the realistic base is fitted from public highway and intersection datasets;
- scensim provides components and specifications, not a framework.

**Design**: [scenario-description.md](../design/scenario-description.md).

## Questions

1. **Expressiveness.** What must a scenario say to describe a risk case, encounter-relative
   and realistic, for each of the eight risk families? How much of it does OpenSCENARIO 1.3
   already carry, and what needs an extension point (`UserDefinedDistribution`,
   `UserDefinedAction`, `ObjectController` properties)?
2. **The world interface.** Which anchors (lanes, regions, conflict points) and knobs
   (surfaces, weather, map variant, props) does a world expose, and how does a scenario bind
   to them by name?
3. **Behaviour models.** How does a scenario name a behaviour model for another road user,
   and where does it run on each target: a batched model in Isaac Lab, an SSv2 controller, or
   a pilot on seats?
4. **Sampling in the description.** How does a scenario name its joint base, constraints,
   proposal (importance sampling) and search strategy, and what lands in the sample record?
5. **Compilation.** For each case study: what compiles to an Isaac Lab batch, what only runs
   on seats, and why. This becomes the S2 draft.
6. **Coverage.** How do category ids and cells attach to a scenario, so the coverage report
   can count them?

## Method

Write case studies across the risk families, each in four parts:

1. the functional description;
2. the logical scenario (OpenSCENARIO with its distribution);
3. a sample record;
4. the compiled form on each target, with the gaps found.

Then generalise the cross-cutting designs from them.

## Work items

| Item | What | Owner | Status |
|---|---|---|---|
| D1 | Case studies: a walker dart-out from occlusion (R2), the lead car braking in a curve on a wet road (R1 × R5 × R4), a stale map (R3), a sensor dropout during a cut-in (R6 × R1), weather onset mid-run (R8 × R4) | 133 | in progress |
| D2 | The world interface: anchors and knobs, their declaration in a world, binding by name, the check | 133 | in progress |
| D3 | Behaviour models: naming, parameters, and where each runs per target | 133 | open |
| D4 | Sampling in the description: base, constraints, proposal, search; the sample record's fields (feeds 092 S1) | 133 | open |
| D5 | The compilable subset, drafted from the case studies (feeds 092 S2) | 133 | open |
| D6 | Coverage tagging: category ids and cells in the scenario | 133 | open |
| D7 | Review: the drafts checked against two more case studies not used to write them | swb 243 | done 2026-11-21: CS6 (a cut-out reveals a stopped car, R1) and CS7 (a cyclist overtaken at an intersection exit under low sun, R2 × R4), scenario-description.md section 2a and `src/scensim_sampler/examples/cases/cs6_*`, `cs7_*`; the exit is not met: nine constructs or rules the drafts lack, first among them that sampled knob values never reach the world |

## Exit

The design note covers D1 to D6. 092's S1 and S2 can be written from it without new design
questions. D7's two fresh case studies are expressible with no construct the drafts lack.
