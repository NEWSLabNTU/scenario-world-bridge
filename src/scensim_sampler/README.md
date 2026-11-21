# scensim_sampler

OpenSCENARIO parameter distributions in, concrete parameter sets out, each with its **sample
record**: the scenario's hash, its risk category and cell, the values, seeds for the scenario,
the world and each pilot, the base model, the proposal and the weight. A record is enough to
run the sample again on any executor, and many of them are enough to audit a rate. The record
is specified in [docs/design/sample-record.md](../../docs/design/sample-record.md), with a JSON
Schema at `scensim_sampler/schema/sample-record-1.json`.

Pure Python (numpy, PyYAML). No ROS, no simulator. It runs under numpy 1.x and 2.x, and a seed
gives the same records, bit for bit, in either.

## Use it alone

```bash
pip install ./src/scensim_sampler        # or: just build, in this workspace
cd src/scensim_sampler/examples/walker_dartout
scensim-sample draw walker_dartout_dist.xosc -n 100 --seed 7 -o samples.jsonl
scensim-sample check samples.jsonl --scenario walker_dartout.xosc
```

From Python, with your own simulator (the adoption cost: three lines):

```python
from scensim_sampler import Sampler

for r in Sampler.from_file("walker_dartout_dist.xosc").draw(n=100, seed=7).records:
    my_simulator.run("walker_dartout.xosc", r["values"], seed=r["seeds"]["world"])
```

## What it reads

A `ParameterValueDistribution` file, as OpenSCENARIO 1.3 defines it:

| | Elements |
|---|---|
| Deterministic | `DistributionSet`, `DistributionRange`, `ValueSetDistribution`; several are combined as their product |
| Stochastic | `ProbabilityDistributionSet`, `NormalDistribution`, `LogNormalDistribution`, `UniformDistribution`, `PoissonDistribution`, `Histogram`, each with its `Range` where it takes one; `numberOfTestRuns`, `randomSeed` |
| Both | `UserDefinedDistribution type="scensim.sampler:<name>"`: a plug-in (below) |

SSv2 does not read `ValueSetDistribution` or `LogNormalDistribution`; this sampler does. A
`LogNormalDistribution`'s `expectedValue` and `variance` are read as the mean and variance of
the distribution itself, not of its logarithm.

Or a scenario with a model file: `scensim-sample draw walker_dartout.xosc --model
models/walker_dartout.yaml`.

The scenario's `FileHeader` names its category and cell attributes:

```xml
<Property name="scensim.category" value="R2.walker_crossing.dart_out"/>
<Property name="scensim.cell_attributes" value="illumination,surface,occluder"/>
<Property name="scensim.cell.occluder" value="van"/>   <!-- a cell value the scenario fixes -->
```

## Plug-ins

| `scensim.sampler:` | Content | Draws |
|---|---|---|
| `joint` | a model file | several parameters together, dependencies kept: a variable conditioned on an earlier discrete one (`given`, `cases`), a distribution parameter that is an expression of earlier values, a Gaussian mixture over a group; then derived values, constraints and the cell |
| `independent` | a model file with `independent: 1` | one distribution per parameter, from YAML instead of XML |
| `importance` | a proposal file | from the proposal, weighted base / proposal (also `--proposal`) |
| `twise` | `{t: 2, factors: {name: [levels]}}`, inline or a file | a covering array: every combination of the levels of any t factors appears in at least one row. Under Stochastic, run i takes row i mod rows |

A package adds its own by declaring an entry point in the group `scensim.sampler`. It names a
factory, called as `factory(content, base_dir=..., parameter=..., scenario=...)`, that returns
a `scensim_sampler.plugins.Plugin`. `scensim-sample plugins` lists those found.

## The model file

The annotated example is `examples/walker_dartout/models/walker_dartout.yaml`. Its numbers are
**invented for illustration, not fitted from data**, and its header says so.

- **`header`.** A fitted model must give `name`, `dataset`, `version` and `licence`. A model
  without them is refused.
- **`knobs`.** Defaults for world knobs the constraints read; `--knob name=value` overrides one.
- **`constraints`.** Each draw that fails one is drawn again. The CLI's summary counts the
  rejections, by constraint.
- **Expressions.** Arithmetic, comparisons, `and`/`or`/`not`, `a if c else b`, `in` over a
  literal list, and a few math functions. They are parsed and walked, never passed to `eval`.

## Weights

A record drawn from a proposal carries `weight = base / proposal` and its logarithm
`log_weight`; a small weight underflows a float, its logarithm does not. The weights are not
normalised. A rate is estimated self-normalised, `sum(w * f) / sum(w)`, which is also what
makes constraints (which truncate base and proposal alike) cancel.
