"""OpenSCENARIO in: a scenario's parameters, header properties and entities, and a
ParameterValueDistribution file's distributions, as plug-ins (scensim_sampler.plugins).

Read with xml.etree; only what the sampler needs is parsed. Supported: Deterministic
(DistributionSet, DistributionRange, ValueSetDistribution, UserDefinedDistribution) and
Stochastic (ProbabilityDistributionSet, Normal-, LogNormal-, Uniform-, PoissonDistribution,
Histogram, UserDefinedDistribution), numberOfTestRuns and randomSeed. SSv2 itself does not
read ValueSetDistribution (commented out) or LogNormalDistribution (no case for it); both are
read here.
"""
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from . import dist, plugins
from .dist import SpecError
from .plugins.independent import Independent

INTEGER = ("integer", "int", "unsignedInt", "unsignedShort")
TYPES = INTEGER + ("double", "string", "boolean", "dateTime")


def cast(ptype, v, where):
    """A value as its parameter's declared type; a lossy cast is refused."""
    if ptype == "double":
        if isinstance(v, bool):
            raise SpecError(f"{where}: {v!r} is not a double")
        try:
            x = float(v)
        except (TypeError, ValueError):
            raise SpecError(f"{where}: {v!r} is not a double") from None
        if not math.isfinite(x):
            raise SpecError(f"{where}: {v!r} is not finite")
        return x
    if ptype in INTEGER:
        if isinstance(v, bool):
            raise SpecError(f"{where}: {v!r} is not an integer")
        if isinstance(v, str):
            try:
                v = float(v) if any(c in v for c in ".eE") else int(v)
            except ValueError:
                raise SpecError(f"{where}: {v!r} is not an integer") from None
        if isinstance(v, float):
            if not v.is_integer():
                raise SpecError(f"{where}: {v} is not an integer (an integer parameter takes "
                                f"a discrete distribution)")
            v = int(v)
        if not isinstance(v, int) or (ptype != "integer" and ptype != "int" and v < 0):
            raise SpecError(f"{where}: {v!r} is not a valid {ptype}")
        return v
    if ptype == "boolean":
        if isinstance(v, bool):
            return v
        if str(v).lower() in ("true", "false"):
            return str(v).lower() == "true"
        raise SpecError(f"{where}: {v!r} is not a boolean")
    return str(v)


def _attr(el, name, where):
    v = el.get(name)
    if v is None:
        raise SpecError(f"{where}: {el.tag} needs {name}")
    return v


def _float(el, name, where):
    v = _attr(el, name, where)
    try:
        return float(v)
    except ValueError:
        raise SpecError(f"{where}: {el.tag}.{name} {v!r} is not a number "
                        f"(parameter references are not resolved here)") from None


def _range(el, where, required=False):
    r = el.find("Range")
    if r is None:
        if required:
            raise SpecError(f"{where}: {el.tag} needs a Range")
        return None
    return [_float(r, "lowerLimit", where), _float(r, "upperLimit", where)]


def _read(path):
    path = Path(path)
    try:
        data = path.read_bytes()
    except OSError as e:
        raise SpecError(f"{path}: {e.strerror}") from None
    try:
        root = ET.fromstring(data)
    except ET.ParseError as e:
        raise SpecError(f"{path}: not XML ({e})") from None
    if root.tag != "OpenSCENARIO":
        raise SpecError(f"{path}: not OpenSCENARIO (root {root.tag})")
    return root, hashlib.sha256(data).hexdigest()


class Scenario:
    """A scenario file: `parameters` {name: (type, default)}, `properties` (FileHeader),
    `category`, `cell_attributes`, `entities`, `sha256`."""

    def __init__(self, path):
        self.path = Path(path)
        root, self.sha256 = _read(self.path)
        self.parameters = {}
        decl = root.find("ParameterDeclarations")
        for p in (decl if decl is not None else []):
            name, ptype = p.get("name"), p.get("parameterType")
            if not name or ptype not in TYPES:
                raise SpecError(f"{path}: ParameterDeclaration {name!r} has parameterType "
                                f"{ptype!r} (one of {', '.join(TYPES)})")
            self.parameters[name] = (ptype, p.get("value"))
        header = root.find("FileHeader")
        self.properties = {}
        if header is not None:
            for prop in header.findall("Properties/Property"):
                self.properties[prop.get("name")] = prop.get("value")
        self.category = self.properties.get("scensim.category")
        attrs = self.properties.get("scensim.cell_attributes") or ""
        self.cell_attributes = [a.strip() for a in attrs.split(",") if a.strip()]
        self.entities = [o.get("name") for o in root.findall("Entities/ScenarioObject")
                         if o.get("name")]
        self.is_distribution = root.find("ParameterValueDistribution") is not None

    def fixed_cell(self):
        """Cell values the scenario fixes: FileHeader `scensim.cell.<attribute>`."""
        return {a: self.properties["scensim.cell." + a] for a in self.cell_attributes
                if "scensim.cell." + a in self.properties}

    def cast(self, name, v):
        return cast(self.parameters[name][0], v, f"{self.path.name}: {name}")


class Distribution:
    """A ParameterValueDistribution file: `scenario` (the ScenarioFile, relative to this
    file), `stochastic` (bool), `runs` (numberOfTestRuns), `seed` (randomSeed or None),
    `plugins` in order, `sha256`."""

    def __init__(self, path):
        self.path = Path(path)
        root, self.sha256 = _read(self.path)
        pvd = root.find("ParameterValueDistribution")
        if pvd is None:
            raise SpecError(f"{path}: no ParameterValueDistribution")
        sf = pvd.find("ScenarioFile")
        if sf is None or not sf.get("filepath"):
            raise SpecError(f"{path}: ParameterValueDistribution needs ScenarioFile filepath")
        self.scenario = Scenario(self.path.parent / sf.get("filepath"))
        if self.scenario.is_distribution:
            raise SpecError(f"{path}: its ScenarioFile is another distribution file")
        det, sto = pvd.find("Deterministic"), pvd.find("Stochastic")
        if (det is None) == (sto is None):
            raise SpecError(f"{path}: exactly one of Deterministic and Stochastic")
        self.stochastic = sto is not None
        self.runs, self.seed = None, None
        self.plugins = []
        self._models = {}
        if self.stochastic:
            self.runs = int(_attr(sto, "numberOfTestRuns", str(path)))
            seed = sto.get("randomSeed")
            if seed is not None:
                s = float(seed)
                if not (s >= 0 and s.is_integer()):
                    raise SpecError(f"{path}: randomSeed {seed} is not an integer >= 0")
                self.seed = int(s)
            self._stochastic(sto)
        else:
            self._deterministic(det)
        self._check_names()

    def _param(self, name, where):
        if name not in self.scenario.parameters \
                and name not in self.scenario.cell_attributes:
            raise SpecError(f"{where}: {name} is not a parameter of {self.scenario.path.name} "
                            f"nor one of its cell attributes")
        return name

    def _typed(self, name, v, where):
        return self.scenario.cast(name, v) if name in self.scenario.parameters else v

    def _user(self, el, name, where):
        content = (el.text or "").strip()
        key = (el.get("type"), content)
        if key in self._models:  # one model named under several of its parameters
            return None
        p = plugins.load(_attr(el, "type", where), content, base_dir=self.path.parent,
                         parameter=name, scenario=self.scenario)
        self._models[key] = p
        return p

    def _stochastic(self, sto):
        independent = Independent({})
        for sd in sto.findall("StochasticDistribution"):
            name = _attr(sd, "parameterName", str(self.path))
            where = f"{self.path}: {name}"
            self._param(name, where)
            kids = list(sd)
            if len(kids) != 1:
                raise SpecError(f"{where}: StochasticDistribution holds one distribution")
            el = kids[0]
            if el.tag == "UserDefinedDistribution":
                p = self._user(el, name, where)
                if p is not None:
                    self.plugins.append(p)
                continue
            if not self.plugins or self.plugins[-1] is not independent:
                if independent in self.plugins:
                    independent = Independent({})
                self.plugins.append(independent)
            independent.add(name, self._standard(el, name, where))

    def _standard(self, el, name, where):
        t = el.tag
        if t == "NormalDistribution":
            return dist.Normal(_float(el, "expectedValue", where),
                               math.sqrt(_float(el, "variance", where)), where, _range(el, where))
        if t == "LogNormalDistribution":
            return dist.LogNormal(_float(el, "expectedValue", where),
                                  math.sqrt(_float(el, "variance", where)), where,
                                  _range(el, where))
        if t == "UniformDistribution":
            lo, hi = _range(el, where, required=True)
            return dist.Uniform(lo, hi, where)
        if t == "PoissonDistribution":
            return dist.Poisson(_float(el, "expectedValue", where), where, _range(el, where))
        if t == "Histogram":
            bins = []
            for b in el.findall("Bin"):
                lo, hi = _range(b, where, required=True)
                bins.append([lo, hi, _float(b, "weight", where)])
            return dist.Histogram(bins, where)
        if t == "ProbabilityDistributionSet":
            weights = {}
            for e in el.findall("Element"):
                v = self._typed(name, _attr(e, "value", where), where)
                weights[v] = weights.get(v, 0.0) + _float(e, "weight", where)
            return dist.Categorical(weights, where)
        raise SpecError(f"{where}: {t} is not a stochastic distribution")

    def _deterministic(self, det):
        for d in det:
            if d.tag == "DeterministicSingleParameterDistribution":
                name = _attr(d, "parameterName", str(self.path))
                where = f"{self.path}: {name}"
                self._param(name, where)
                kids = list(d)
                if len(kids) != 1:
                    raise SpecError(f"{where}: one distribution per parameter")
                el = kids[0]
                if el.tag == "UserDefinedDistribution":
                    p = self._user(el, name, where)
                    if p is not None:
                        if not p.deterministic:
                            raise SpecError(f"{where}: {p.name} draws at random; it belongs "
                                            f"under Stochastic")
                        self.plugins.append(p)
                    continue
                self.plugins.append(Rows(self._single(el, name, where)))
            elif d.tag == "DeterministicMultiParameterDistribution":
                where = f"{self.path}: ValueSetDistribution"
                vsd = d.find("ValueSetDistribution")
                if vsd is None:
                    raise SpecError(f"{where}: DeterministicMultiParameterDistribution holds "
                                    f"a ValueSetDistribution")
                rows = []
                for s in vsd.findall("ParameterValueSet"):
                    row = {}
                    for a in s.findall("ParameterAssignment"):
                        n = self._param(_attr(a, "parameterRef", where), where)
                        row[n] = self._typed(n, _attr(a, "value", where), where)
                    rows.append(row)
                if not rows or len({tuple(sorted(r)) for r in rows}) != 1:
                    raise SpecError(f"{where}: every ParameterValueSet assigns the same "
                                    f"parameters")
                self.plugins.append(Rows(rows))
            else:
                raise SpecError(f"{self.path}: {d.tag} is not a deterministic distribution")

    def _single(self, el, name, where):
        if el.tag == "DistributionSet":
            vals = [self._typed(name, _attr(e, "value", where), where)
                    for e in el.findall("Element")]
            if not vals:
                raise SpecError(f"{where}: DistributionSet is empty")
            return [{name: v} for v in vals]
        if el.tag == "DistributionRange":
            step = _float(el, "stepWidth", where)
            lo, hi = _range(el, where, required=True)
            if not step > 0 or hi < lo:
                raise SpecError(f"{where}: DistributionRange needs stepWidth > 0 and "
                                f"lowerLimit <= upperLimit")
            n = int(math.floor((hi - lo) / step + 1e-9)) + 1
            return [{name: self._typed(name, round(lo + i * step, 12), where)}
                    for i in range(n)]
        raise SpecError(f"{where}: {el.tag} is not a deterministic distribution")

    def _check_names(self):
        seen = {}
        for p in self.plugins:
            for n in getattr(p, "names", ()):
                if n in seen and seen[n] is not p:
                    raise SpecError(f"{self.path}: {n} is given by two distributions")
                seen[n] = p


class Rows(plugins.Plugin):
    """Deterministic rows of the standard elements."""
    name = "deterministic"
    deterministic = True

    def __init__(self, rows):
        self._rows = rows
        self.names = tuple(rows[0])

    def rows(self):
        return self._rows
