"""The model file: variables drawn in order, each a distribution (scensim_sampler.dist), one
conditioned on an earlier discrete value (`given` with `cases`), or a Gaussian mixture over a
group; then derived values, constraints and the cell. The format is in docs/design/sample-record.md
("The model file").

    model: 1
    header: {name: walker_dartout, dataset: ..., version: ..., licence: ...}
    variables:
      - {name: walker_type, categorical: {adult: 0.6, child: 0.25, runner: 0.15}}
      - name: walk_speed
        given: walker_type
        cases:
          adult: {normal: {mean: 1.4, sd: 0.2}, range: [0.5, 2.5]}
          child: {normal: {mean: 1.6, sd: 0.3}, range: [0.5, 3.0]}
          runner: {lognormal: {mean: 3.0, sd: 0.5}}
      - names: [ego_speed, van_offset]
        mixture:
          - {weight: 1.0, mean: [8.3, 2.0], cov: [[2.0, 0.0], [0.0, 0.4]]}
    knobs: {speed_limit: 13.9}   # world knobs' defaults; the sampler's knobs override them
    derived: {trigger_distance: "ego_speed * ttc_entry"}
    constraints: ["ego_speed <= 1.2 * 13.9"]
    cell: {illumination: {expr: "'night' if hour < 6 else 'day'"}, occluder: van}
"""
import hashlib
import math
from pathlib import Path

import yaml

from .. import dist
from ..dist import SpecError
from ..expr import Expr, ExprError

HEADER_REQUIRED = ("name", "dataset", "version", "licence")


def read_yaml(path):
    path = Path(path)
    try:
        data = path.read_bytes()
    except OSError as e:
        raise SpecError(f"{path}: {e.strerror}") from None
    try:
        doc = yaml.safe_load(data)
    except yaml.YAMLError as e:
        raise SpecError(f"{path}: not YAML ({e})") from None
    return doc, hashlib.sha256(data).hexdigest()


def _expr(text, where):
    try:
        return Expr(text, where)
    except ExprError as e:
        raise SpecError(str(e)) from None


class Var:
    def __init__(self, name, d, where):
        self.names = (name,)
        self.name, self.dist, self.discrete = name, d, d.discrete

    def sample(self, rng, env):
        return {self.name: self.dist.sample(rng, env)}

    def logpdf(self, values):
        return self.dist.logpdf(values[self.name], values)


class Given:
    def __init__(self, name, given, cases, where):
        if not isinstance(cases, dict) or not cases:
            raise SpecError(f"{where}: given needs cases: {{value: distribution}}")
        self.names = (name,)
        self.name, self.given = name, given
        self.cases = {str(k): dist.spec(v, f"{where}.cases[{k!r}]") if isinstance(v, dict)
                      else _bad(f"{where}.cases[{k!r}]: a distribution mapping")
                      for k, v in cases.items()}
        kinds = {c.discrete for c in self.cases.values()}
        if len(kinds) != 1:
            raise SpecError(f"{where}: the cases mix discrete and continuous distributions")
        self.discrete = kinds.pop()
        self.where = where

    def _case(self, env):
        if self.given not in env:
            raise SpecError(f"{self.where}: given {self.given}, which is not drawn before it")
        key = str(env[self.given])
        if key not in self.cases:
            raise SpecError(f"{self.where}: no case for {self.given} = {key!r} (cases: "
                            f"{', '.join(self.cases)})")
        return self.cases[key]

    def sample(self, rng, env):
        return {self.name: self._case(env).sample(rng, env)}

    def logpdf(self, values):
        return self._case(values).logpdf(values[self.name], values)


class Group:
    def __init__(self, names, components, where):
        self.mixture = dist.Mixture(names, components, where)
        self.names = tuple(names)
        self.discrete = False

    def sample(self, rng, env):
        return self.mixture.sample(rng, env)

    def logpdf(self, values):
        return self.mixture.logpdf(values, values)


def _bad(msg):
    raise SpecError(msg)


def entry(e, where):
    if not isinstance(e, dict):
        raise SpecError(f"{where}: a variable is a mapping")
    if "names" in e:
        extra = set(e) - {"names", "mixture"}
        if extra or "mixture" not in e:
            raise SpecError(f"{where}: a group is {{names, mixture}}"
                            + (f"; unknown {sorted(extra)}" if extra else ""))
        return Group(e["names"], e["mixture"], where)
    name = e.get("name")
    if not isinstance(name, str) or not name.isidentifier():
        raise SpecError(f"{where}: name is a parameter name, not {name!r}")
    where = f"{where} ({name})"
    if "given" in e:
        extra = set(e) - {"name", "given", "cases"}
        if extra:
            raise SpecError(f"{where}: unknown {sorted(extra)} (a conditional is "
                            f"{{name, given, cases}})")
        return Given(name, e["given"], e.get("cases"), where)
    extra = set(e) - {"name", "range"} - set(dist.KINDS)
    if extra:
        raise SpecError(f"{where}: unknown {sorted(extra)}")
    return Var(name, dist.spec(e, where), where)


class Model:
    """A model file. `kind`: "model" (a fitted base: header with name, dataset, version and
    licence required), "independent" (no header needed) or "proposal" (scensim_sampler.plugins.
    importance builds one from its file)."""

    TOP = {"model": {"model", "header", "knobs", "variables", "derived", "constraints", "cell"},
           "independent": {"independent", "knobs", "variables", "derived", "constraints",
                           "cell"}}

    def __init__(self, doc, where, sha256=None, kind="model"):
        self.where, self.sha256, self.kind = where, sha256, kind
        if not isinstance(doc, dict):
            raise SpecError(f"{where}: a {kind} file is a mapping")
        if doc.get(kind) != 1:
            raise SpecError(f"{where}: `{kind}: 1` (the format's version) is required")
        extra = set(doc) - self.TOP[kind]
        if extra:
            raise SpecError(f"{where}: unknown {sorted(extra)} (a {kind} file has "
                            f"{', '.join(sorted(self.TOP[kind]))})")
        self.header = doc.get("header") or {}
        if kind == "model":
            if not isinstance(self.header, dict):
                raise SpecError(f"{where}: header is a mapping")
            missing = [k for k in HEADER_REQUIRED
                       if not isinstance(self.header.get(k), str) or not self.header[k].strip()]
            if missing:
                raise SpecError(f"{where}: header needs {', '.join(missing)}: a fitted model "
                                f"names its data, the data's version and its licence")
        variables = doc.get("variables")
        if not isinstance(variables, list) or not variables:
            raise SpecError(f"{where}: variables is a non-empty list")
        self.entries = [entry(e, f"{where}: variables[{i}]") for i, e in enumerate(variables)]
        self._index()
        self.derived = self._derived(doc.get("derived") or {})
        cons = doc.get("constraints") or []
        if not isinstance(cons, list):
            raise SpecError(f"{where}: constraints is a list of expressions")
        self.constraints = [_expr(c, f"{where}: constraints[{i}]") for i, c in enumerate(cons)]
        self.cell_spec = self._cell(doc.get("cell") or {})
        self.knobs = doc.get("knobs") or {}
        if not isinstance(self.knobs, dict) or not all(
                isinstance(k, str) and isinstance(v, (int, float, str, bool))
                for k, v in self.knobs.items()):
            raise SpecError(f"{where}: knobs is {{name: value}}: the world knobs' defaults")

    def _index(self):
        self.names = []
        for e in self.entries:
            for n in e.names:
                if n in self.names:
                    raise SpecError(f"{self.where}: {n} is drawn twice")
                self.names.append(n)

    def _derived(self, d):
        if not isinstance(d, dict):
            raise SpecError(f"{self.where}: derived is {{name: expression}}")
        out = {}
        for k, v in d.items():
            if k in self.names:
                raise SpecError(f"{self.where}: derived {k} is also drawn")
            out[k] = _expr(v, f"{self.where}: derived.{k}")
        return out

    def _cell(self, c):
        if not isinstance(c, dict):
            raise SpecError(f"{self.where}: cell is {{attribute: value | {{expr}} | "
                            f"{{parameter}}}}")
        out = {}
        for k, v in c.items():
            at = f"{self.where}: cell.{k}"
            if isinstance(v, dict):
                if set(v) == {"expr"}:
                    out[k] = _expr(v["expr"], at)
                elif set(v) == {"parameter"} and isinstance(v["parameter"], str):
                    out[k] = _expr(v["parameter"], at)
                else:
                    raise SpecError(f"{at}: a value, {{expr: ...}} or {{parameter: name}}")
            elif isinstance(v, (str, int, float, bool)):
                out[k] = v
            else:
                raise SpecError(f"{at}: a value, {{expr: ...}} or {{parameter: name}}")
        return out

    def replace(self, entries):
        """A copy with some entries replaced (a proposal), each by the same names."""
        new = object.__new__(Model)
        new.__dict__.update(self.__dict__)
        by = {tuple(e.names): e for e in entries}
        own = {tuple(e.names) for e in self.entries}
        for names in by:
            if names not in own:
                raise SpecError(f"the proposal draws {', '.join(names)}, which the base "
                                f"{self.where} does not draw as one variable or group")
        new.entries = [by.get(tuple(e.names), e) for e in self.entries]
        for old, e in zip(self.entries, new.entries):
            if old.discrete != e.discrete:
                raise SpecError(f"the proposal makes {', '.join(e.names)} "
                                f"{'discrete' if e.discrete else 'continuous'}; the base's is not")
        return new

    def sample(self, rng, values):
        env = dict(values)
        for e in self.entries:
            env.update(e.sample(rng, env))
        return {n: env[n] for n in self.names}

    def logpdf(self, values):
        total = 0.0
        for e in self.entries:
            lp = e.logpdf(values)
            if lp == -math.inf:
                return lp
            total += lp
        return total

    def eval_derived(self, values):
        env = dict(values)
        for k, x in self.derived.items():
            env[k] = x(env)
        return {k: env[k] for k in self.derived}

    def violations(self, values):
        return [c.text for c in self.constraints if not c(values)]

    def eval_cell(self, values):
        return {k: (v(values) if isinstance(v, Expr) else v) for k, v in self.cell_spec.items()}
