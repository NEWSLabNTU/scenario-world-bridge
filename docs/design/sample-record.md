# The sample record (specification, version 1)

A sample record says what one drawn scenario is. It holds enough to run the sample again on any
executor that accepts it, and, across many records, enough to audit a rate computed from them.
It is the record of [ml-training-interface.md](ml-training-interface.md)'s C2 and of CS1 in
[scenario-description.md](scenario-description.md). scensim's sampler (`scensim_sampler`,
`scensim-sample draw`) writes it; another tool may write or read it from this page alone.

- **The schema** (JSON Schema, draft 2020-12) is
  [sample-record-1.json](../../src/scensim_sampler/scensim_sampler/schema/sample-record-1.json).
- **The checker** is `scensim-sample check` (`scensim_sampler.check`). It applies the schema's
  rules and the ones a schema cannot state (section 3), and names every problem.

## 1. The record

One record is a JSON object. A file holds one per line (`.jsonl`), or YAML documents.

```yaml
record: 1
id: 3f0c...                       # section 2
scenario: {path: walker_dartout.xosc, sha256: 13bc...}
distribution: {path: walker_dartout_dist.xosc, sha256: 6a1e...}   # or null
category: R2.walker_crossing.dart_out
cell: {illumination: day, surface: dry, occluder: van}
values: {ego_speed: 9.1, van_offset: 1.6, walker_type: child, walk_speed: 1.9,
         ttc_entry: 1.8, behaviour: run, trigger_distance: 16.38}
derived: [trigger_distance]
seeds: {scenario: 1610840759, world: 2840832722,
        pilots: {ego: 3667288858, van: 1759773357, walker: 3492716762}}
base: {id: models/walker_dartout.yaml, sha256: f6ea...}            # or null
proposal: {id: near_ttc_short, sha256: 8ea6...}                    # or null
weight: 0.037
log_weight: -3.297
search: null
world: {name: town_street, hash: "stage:sha256:..."}               # optional until bound
car: {name: tesla_model3, spec_hash: 9c1d...}                      # optional until bound
sampler: {name: scensim_sampler, version: 0.1.0, plugins: [importance], run_seed: 41,
          index: 12, env: {python: 3.10.12, numpy: 1.21.5, platform: Linux-x86_64}}
```

| Field | Type | Meaning |
|---|---|---|
| `record` | `1` | the format's version |
| `id` | 64 hex | the record's identity (section 2) |
| `scenario` | `{path, sha256}` | the scenario file the values are for. `sha256` is over the file's bytes; `path` is where it was, for a reader, and is not part of the identity |
| `distribution` | `{path, sha256}` or null | the ParameterValueDistribution file it was drawn from; null when there was none (a scenario with a model file) |
| `category` | string | `R<family>.<category>.<subtype>`, from the scenario's `FileHeader` property `scensim.category`. The family is R1 to R8 of [survey-vehicle-risk-taxonomy.md](survey-vehicle-risk-taxonomy.md); category and subtype are lowercase words (`[a-z0-9_]`) |
| `cell` | `{attribute: value}` | the condition cell. Its keys are exactly the scenario's `scensim.cell_attributes`; each value is drawn, mapped by the model's `cell`, or fixed by the scenario's `scensim.cell.<attribute>` |
| `values` | `{parameter: value}` | the concrete values, each a parameter the scenario declares, in its declared type (double: number; integer types: integer; boolean: true or false; string and dateTime: string). A parameter not listed keeps the scenario's default |
| `derived` | list of names | the values computed from others rather than drawn (CS1's trigger distance, `ego_speed × ttc_entry`) |
| `seeds` | `{scenario, world, pilots: {entity: seed}}` | integers in [0, 2^32). The scenario's seed is for the scenario engine; the world's for the world server (spawn jitter, weather noise); each pilot's for that entity's pilot or behaviour model |
| `base` | `{id, sha256}` or null | the fitted model the sample's realism rests on (its file's header names dataset, version and licence). Null: the distribution file's standard distributions, or a deterministic set, are the base |
| `proposal` | `{id, sha256}` or null | the distribution it was drawn from instead of the base (purpose A, a rate) |
| `weight` | number ≥ 0 | base density / proposal density at `values`; 1 without a proposal |
| `log_weight` | number | its natural logarithm, which does not underflow; 0 without a proposal |
| `search` | `{id, generation}` or null | the adaptive search that proposed it (purpose B); reserved, the sampler writes null |
| `world` | `{name, hash}` | the world it ran in. Written by the executor that binds it (the field is optional in a drawn record, required by `check --bound`) |
| `car` | `{name, spec_hash}` | the car spec it ran with, by `sim_vehicle_spec`'s `spec_hash`; bound as `world` is |
| `sampler` | `{name, version, plugins, run_seed, index, env}` | what drew it: the plug-ins in order, the run's seed, the sample's index in the run, and `env` `{python, numpy, platform}`, the host's versions, to trace a record that does not redraw the same (numpy's streams may change between versions; libm may differ in the last bit) |

## 2. Identity

`id` is the SHA-256 (lowercase hex) of the record's canonical form, taken without `id`, without
`scenario.path` and `distribution.path`, and without `sampler.env`. Where a file was is not what
it was (its `sha256` is), and the host that drew a sample is not what was drawn. Nothing else is
left out: a parameter named `path` is a value like any other. The canonical form is JSON with keys sorted, no spaces (`,` and `:` as separators), ASCII
only, and no NaN or infinity. In Python:

```python
json.dumps(body, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
```

Two identical draws therefore share an id, and binding `world` and `car` makes a new one. A
record that was edited after it was drawn no longer matches its id, and the checker says so.

## 3. Rules a schema cannot state

The checker enforces these as well as the schema:

- `id` is the hash of section 2.
- `weight` = exp(`log_weight`) (relative 1e-9). A weight that has underflowed to 0 with a
  very negative `log_weight` is valid.
- Without a proposal, `weight` is 1 and `log_weight` is 0. With one, `base` is not null.
- Every name in `derived` is in `values`.

With `--scenario` (the scenario file), it also checks:

- `scenario.sha256` is the file's hash, and `category` is its `scensim.category`.
- The `cell` keys are its `scensim.cell_attributes`.
- Every `values` key is a declared parameter, in its declared type.
- Every `seeds.pilots` key is one of its entities.

## 4. Seeds

A run has one seed, S. Sample i takes numpy's `SeedSequence(S, spawn_key=(i,))` and spawns four
streams from it: the draw, the scenario's seed, the world's seed and the pilots' seeds (one
value per entity, in the scenario's order). A sample therefore does not depend on how many
others were drawn, and `sampler.run_seed` with `sampler.index` redraws it. Another sampler need
not use this scheme: a record's seeds are what an executor uses, however they were made. A
deterministic file's run seed defaults to 0.

scensim_sampler gives the same records, bit for bit, under numpy 1.x and 2.x. The test suite
checks a digest of 50 draws in both Pythons the workspace has.

## 5. Weights and rates

A proposal's weights are density ratios that are not normalised. A rate is estimated
**self-normalised**:

  p̂ = Σ w_i f_i / Σ w_i

where f_i is 1 for a failure and 0 otherwise, per cell. Three reasons:

- **Constraints.** They truncate both the base and the proposal, and their normalising constants
  are not computed. Under self-normalisation, the constant ratio between them cancels.
- **Rejected draws.** They are not recorded, and they cancel the same way.
- **Log weights.** Sum them in log space (log-sum-exp); a product of small densities underflows.

A draw where the base's density is 0 (outside its range, say) is rejected, not recorded with
weight 0. That truncates the proposal to the base's support, which is a constant factor and
cancels as above. **Do not use the weights unnormalised.** A plain mean of w_i f_i, for a
rare-event probability say, is biased by these constants.

## 6. The model file and the proposal file

These are the formats of `scensim.sampler:joint` (a fitted base) and `scensim.sampler:importance`
(a proposal). The annotated example is
[walker_dartout.yaml](../../src/scensim_sampler/examples/walker_dartout/models/walker_dartout.yaml).
Its numbers are invented for illustration, not fitted from data.

```yaml
model: 1
header: {name: ..., dataset: ..., version: ..., licence: ...}   # all four required
knobs: {speed_limit: 13.9}            # world knobs' defaults
variables:                            # drawn in order; later ones may read earlier ones
  - {name: walker_type, categorical: {adult: 0.6, child: 0.25, runner: 0.15}}
  - name: walk_speed
    given: walker_type                # a discrete earlier variable
    cases: {adult: {normal: {mean: 1.4, sd: 0.2}, range: [0.6, 2.4]}, ...}
  - names: [ego_speed, van_offset]    # drawn together
    mixture: [{weight: 0.7, mean: [8.3, 2.0], cov: [[1.6, 0.2], [0.2, 0.5]]}, ...]
  - {name: ttc_entry, lognormal: {mean: "2.6 if walker_type == 'adult' else 2.0", sd: 0.8}}
derived: {trigger_distance: "ego_speed * ttc_entry"}
constraints: ["ego_speed <= 1.2 * speed_limit"]
cell: {illumination: {parameter: illumination}, occluder: van, light: {expr: "..."}}
```

**Distributions.** Exactly one of these keys, and `range: [low, high]` where it applies:

| Key | Arguments | `range` |
|---|---|---|
| `normal` | `{mean, sd}` | yes |
| `lognormal` | `{mean, sd}`, of the distribution itself | yes |
| `uniform` | `{low, high}` | no |
| `poisson` | `{mean}` | yes |
| `categorical` | `{value: weight}` | no |
| `histogram` | `[[low, high, weight], ...]` | no |
| `constant` | a value | no |

A range truncates the distribution, and its density is renormalised. Any numeric argument may
be an expression of earlier values.

**Expressions** are parsed and walked, never passed to `eval`. They allow:

- arithmetic and comparisons (chained included);
- `and`, `or`, `not`;
- `a if c else b`;
- `in` over a literal list;
- `abs min max round sqrt exp log floor ceil sin cos tan atan2 int float str`;
- the constants `pi` and `g`;
- names: the values drawn so far, the derived values, and the world knobs.

An integer power with an exponent above 1000, or a string repeated past 10,000 characters, is
refused, so a typo cannot hang the sampler; a float power that overflows is an error.

**A variable a scenario does not declare** may be one of its cell attributes, which the record
puts under `cell`. A derived value that is neither a parameter nor a cell attribute is an
intermediate: it is used, and not recorded. Any other undeclared name is refused.

**A proposal file** replaces some of its base's variables. Each replacement has the same name
(or the same group of names) and the same discreteness; the others are drawn as the base draws
them. The base's derived values, constraints and cell apply.

```yaml
proposal: 1
name: near_ttc_short
base: ../models/walker_dartout.yaml   # relative to this file
variables:
  - {name: ttc_entry, lognormal: {mean: 1.2, sd: 0.5}, range: [0.3, 8.0]}
```

`independent: 1` instead of `model: 1` is the `independent` plug-in's file. It needs no header,
and has no `given` and no mixture.

## 7. Plug-ins

A `UserDefinedDistribution type="scensim.sampler:<name>"` names a plug-in, found in the Python
entry point group `scensim.sampler`. The element's text is the plug-in's content, usually a file
path relative to the distribution file. One model may be named under several of the parameters
it draws; it is loaded once. Two distributions giving one parameter are refused.

The factory is called as `factory(content, base_dir=..., parameter=..., scenario=...)` and
returns a `scensim_sampler.plugins.Plugin`:

| Member | Meaning |
|---|---|
| `names` | the parameters (or cell attributes) it assigns |
| `draw(rng, index, values)` | one draw; `values` holds what earlier plug-ins drew, plus the knobs |
| `deterministic`, `rows()` | a set to enumerate: Deterministic takes every row, Stochastic row `index mod len` |
| `base`, `proposal` | `{id, sha256}` of its files, for the record |
| `log_weight(values)` | log(base / proposal); `-inf` rejects the draw |
| `violations`, `derived`, `cell`, `knobs` | constraints failed, derived values, cell values, knob defaults |

Deterministic plug-ins (t-wise rows) go first, so later ones may condition on their factors.

## 8. Coverage

`scensim-sample coverage <runs | outcomes.jsonl> [--catalogue c.yaml] [--failure
verdict|contact|either] [-t N] [--json coverage.json]` reports what a set of sampled runs
covers. `scensim runs coverage` runs the same over a testbench's runs. It reads sample records
with their outcomes:

- **scensim's runs:** `sample.json` and `result.json`;
- **any other runs:** a JSONL of `{record, outcome: {failed, contact, error,
  seeds_not_applied}}`.

It needs no simulator. Its layers are those of
[survey-vehicle-risk-taxonomy.md](survey-vehicle-risk-taxonomy.md), section 4.

1. **Categories.** Each catalogue category is covered or not, with its runs, and the uncovered
   ones are named. It also lists categories that are run but not in the catalogue, and the
   families R1 to R8 that have no category at all.
2. **Conditions.** Per category: the t-wise coverage of its cell attributes' levels (covered
   t-tuples of all), each missing tuple, and each run whose cell lies outside the declared
   levels. Without a catalogue, the levels are the ones observed, and the report says so.
3. **Criticality.** Per category and cell: runs, errors, judged runs, failures, and the failure
   rate with a 95 % interval.
   - **Unweighted:** a Wilson interval. With no failures, its upper end is the bound, close to
     the rule of three.
   - **Drawn from a proposal:** the self-normalised estimate from the log weights (section 5),
     with a Wilson interval on the effective count `(Σw)² / Σw²`, marked approximate.
   - **Mixed:** a cell that mixes weighted and unweighted runs, or several proposals or bases,
     gets no rate, and says why. They are not blended.

Some runs are counted apart and listed, never pooled into the rates:

- **Errors.**
- **Runs with seeds not applied** (`result.json`'s `sample.unbound`): they are not the sample
  their record names.
- **Replays:** each is the same sample again.

**The catalogue** is the study's own declaration of what it means to cover:

```yaml
catalogue: 1
categories: [R2.walker_crossing.dart_out, R1.rear_end.lead_brakes]
cells: {R2.walker_crossing.dart_out: {illumination: [day, night], surface: [dry, wet]}}
t: 2
exposure: {R2.walker_crossing.dart_out: 0.12, R1.rear_end.lead_brakes: 0.29}   # optional
```

**`exposure`** gives each category's weight, from crash or exposure frequencies for the study's
region. Only when it is given does the report add an overall rate, `Σ e_c r_c / Σ e_c`, with an
approximate interval. The rate is refused when a weighted category has no rate. The weights are
the study's input, not scensim's. Whose statistics to use (a country's crash typology, say) is
the study's choice, and the report prints no overall number without one.

`coverage.json` has the schema `scensim.coverage/1`: the three layers, as printed, with every
missing tuple. Failure clusters, the reference driver's classes (avoidable or not) and the
sample density near a boundary are not here yet. Each needs a judge or a metric that the runs do
not record yet.

## 9. What is not here yet

- **Adaptive search (purpose B).** `search` is reserved for it.
- **Binding `world` and `car`.** The executors do this (roadmap 092 K2, K3).
- **More than one fitted base per scenario.** A version 1 record names one, and the sampler
  refuses two.
