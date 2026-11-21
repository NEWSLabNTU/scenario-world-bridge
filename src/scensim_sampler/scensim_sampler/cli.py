"""scensim-sample: draw sample records, check them, list the plug-ins.

    scensim-sample draw walker_dartout_dist.xosc -n 100 --seed 7 -o samples.jsonl
    scensim-sample draw walker_dartout.xosc --model models/walker_dartout.yaml -n 100 --seed 7
    scensim-sample draw walker_dartout_dist.xosc --proposal proposals/near_ttc_short.yaml ...
    scensim-sample check samples.jsonl --scenario walker_dartout.xosc
    scensim-sample plugins
    scensim-sample coverage runs/ --catalogue catalogue.yaml --json coverage.json
"""
import argparse
import json
import sys

import yaml

from . import plugins
from .dist import SpecError
from .expr import ExprError
from .record import check, read
from .sampler import Sampler, SamplingError
from .xosc import Scenario


def _knob(text):
    name, sep, value = text.partition("=")
    if not sep or not name.isidentifier():
        raise argparse.ArgumentTypeError(f"a knob is name=value, not {text!r}")
    try:
        v = yaml.safe_load(value)
    except yaml.YAMLError:
        v = value
    return name, v


def draw(a):
    s = Sampler.from_file(a.file, model=a.model, proposal=a.proposal, knobs=dict(a.knob))
    batch = s.draw(n=a.n, seed=a.seed)
    out = sys.stdout if a.output == "-" else open(a.output, "w")
    try:
        if a.format == "yaml":
            yaml.safe_dump_all(batch.records, out, sort_keys=False)
        else:
            for r in batch.records:
                out.write(json.dumps(r) + "\n")
    finally:
        if out is not sys.stdout:
            out.close()
    summ = batch.summary()
    why = "".join(f"\n  {n} x {w}" for w, n in sorted(summ["rejected_why"].items(),
                                                     key=lambda x: -x[1]))
    print(f"scensim-sample: {summ['records']} records to {a.output}, {summ['rejected']} "
          f"draws rejected{why}", file=sys.stderr)
    return 0


def check_cmd(a):
    scenario = Scenario(a.scenario) if a.scenario else None
    bad = 0
    total = 0
    for path in a.records:
        for i, r in enumerate(read(path)):
            total += 1
            for p in check(r, scenario=scenario, bound=a.bound):
                bad += 1
                print(f"{path}: record {i}: {p}")
    print(f"scensim-sample: {total} records, {bad} problems", file=sys.stderr)
    return 1 if bad else 0


def coverage_cmd(a):
    from . import coverage as cov  # noqa: PLC0415
    items = []
    runs = [p for p in a.inputs if not p.endswith(".jsonl")]
    for p in a.inputs:
        if p.endswith(".jsonl"):
            items += cov.outcomes_from_jsonl(p)
    if runs:
        items += cov.outcomes_from_runs(runs)
    if not items:
        raise cov.CoverageError("no sample records in " + " ".join(a.inputs))
    cat = cov.load_catalogue(a.catalogue) if a.catalogue else None
    rep = cov.report(items, cat, failure=a.failure, t=a.t)
    sys.stdout.write(cov.format_report(rep))
    if a.json:
        with open(a.json, "w") as f:
            json.dump(rep, f, indent=2)
            f.write("\n")
        print(a.json)
    return 0


def list_plugins(a):
    for name, target in sorted(plugins.known().items()):
        print(f"{plugins.PREFIX}{name}  {target}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="scensim-sample", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("draw", help="draw sample records")
    d.add_argument("file", help="a ParameterValueDistribution .xosc, or a scenario with --model")
    d.add_argument("-n", type=int, help="samples (default: numberOfTestRuns; deterministic: "
                                        "every row)")
    d.add_argument("--seed", type=int, help="the run seed (default: randomSeed)")
    d.add_argument("--model", help="a model file, for a scenario without a distribution file")
    d.add_argument("--proposal", help="a proposal file: draw from it, weighted against its base")
    d.add_argument("--knob", type=_knob, action="append", default=[],
                   help="name=value: a world knob the constraints and the cell may read")
    d.add_argument("-o", "--output", default="-", help="the records (default: stdout)")
    d.add_argument("--format", choices=("jsonl", "yaml"), default="jsonl")
    d.set_defaults(func=draw)
    c = sub.add_parser("check", help="check sample records")
    c.add_argument("records", nargs="+", help=".jsonl or YAML files")
    c.add_argument("--scenario", help="the scenario the records must match")
    c.add_argument("--bound", action="store_true", help="world and car are required")
    c.set_defaults(func=check_cmd)
    v = sub.add_parser("coverage", help="the coverage report: categories, conditions, rates")
    v.add_argument("inputs", nargs="+", help="run directories, directories of runs, or .jsonl "
                                             "files of {record, outcome}")
    v.add_argument("--catalogue", help="the categories, cell levels, t and exposure the study "
                                       "declares")
    v.add_argument("--failure", choices=("verdict", "contact", "either"), default="verdict",
                   help="what counts as a failure (default: the run's verdict)")
    v.add_argument("-t", type=int, help="t of the t-wise condition coverage (default: the "
                                        "catalogue's, else 2)")
    v.add_argument("--json", help="also write the report here (scensim.coverage/1)")
    v.set_defaults(func=coverage_cmd)
    p = sub.add_parser("plugins", help="the sampler plug-ins found")
    p.set_defaults(func=list_plugins)
    a = ap.parse_args(argv)
    try:
        return a.func(a)
    except (SpecError, SamplingError, ExprError, plugins.PluginError, ValueError) as e:
        print(f"scensim-sample: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
