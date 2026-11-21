"""The distributions: each draws a value and gives its log density (log probability for a
discrete one), truncated to a range when it has one. A parameter of a distribution may be an
expression over the values drawn before it, which is how a model conditions one variable on
another (`mean: "1.2 + 0.05 * ego_speed"`).

The model files spell them as YAML (`spec()`); the OpenSCENARIO elements map onto the same
classes (scensim_sampler.xosc).
"""
import math

import numpy as np

from .expr import Expr, ExprError

MAX_TRIES = 10000  # a truncated draw: rejection, refused when the range holds too little


class SpecError(ValueError):
    pass


def _param(v, where):
    if isinstance(v, bool):
        raise SpecError(f"{where}: a number or an expression, not {v!r}")
    if isinstance(v, (int, float)):
        return float(v)
    if isinstance(v, str):
        try:
            return Expr(v, where)
        except ExprError as e:
            raise SpecError(str(e)) from None
    raise SpecError(f"{where}: a number or an expression, not {v!r}")


def _value(p, env):
    return float(p(env)) if isinstance(p, Expr) else p


def _phi(z):
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


class Dist:
    discrete = False
    kind = "?"

    def __init__(self, where, rng_range=None):
        self.where = where
        self.range = None
        if rng_range is not None:
            if (not isinstance(rng_range, (list, tuple)) or len(rng_range) != 2
                    or any(isinstance(x, bool) or not isinstance(x, (int, float))
                           for x in rng_range) or rng_range[0] > rng_range[1]):
                raise SpecError(f"{where}: range is [low, high] with low <= high, "
                                f"not {rng_range!r}")
            self.range = (float(rng_range[0]), float(rng_range[1]))

    def inside(self, x):
        return self.range is None or self.range[0] <= x <= self.range[1]

    def sample(self, rng, env):
        for _ in range(MAX_TRIES):
            x = self._sample(rng, env)
            if self.inside(x):
                return x
        raise SpecError(f"{self.where}: {MAX_TRIES} draws, none inside range {list(self.range)} "
                        f"(the range holds almost none of the distribution)")

    def logpdf(self, x, env):
        if not self.inside(x):
            return -math.inf
        lp = self._logpdf(x, env)
        if self.range is not None and lp > -math.inf:
            z = self._mass(env)
            if z <= 0:
                return -math.inf
            lp -= math.log(z)
        return lp

    def _mass(self, env):  # the untruncated distribution's mass inside the range
        return 1.0


class Normal(Dist):
    kind = "normal"

    def __init__(self, mean, sd, where, rng_range=None):
        super().__init__(where, rng_range)
        self.mean, self.sd = _param(mean, f"{where}.mean"), _param(sd, f"{where}.sd")

    def _ms(self, env):
        m, s = _value(self.mean, env), _value(self.sd, env)
        if not s > 0:
            raise SpecError(f"{self.where}: sd must be > 0, is {s}")
        return m, s

    def _sample(self, rng, env):
        m, s = self._ms(env)
        return float(rng.normal(m, s))

    def _logpdf(self, x, env):
        m, s = self._ms(env)
        return -0.5 * ((x - m) / s) ** 2 - math.log(s * math.sqrt(2 * math.pi))

    def _mass(self, env):
        m, s = self._ms(env)
        return _phi((self.range[1] - m) / s) - _phi((self.range[0] - m) / s)


class LogNormal(Dist):
    """mean and sd of the distribution itself (not of its logarithm), as OpenSCENARIO's
    expectedValue and variance of a LogNormalDistribution are read here."""
    kind = "lognormal"

    def __init__(self, mean, sd, where, rng_range=None):
        super().__init__(where, rng_range)
        self.mean, self.sd = _param(mean, f"{where}.mean"), _param(sd, f"{where}.sd")

    def _mu_sigma(self, env):
        m, s = _value(self.mean, env), _value(self.sd, env)
        if not (m > 0 and s > 0):
            raise SpecError(f"{self.where}: a log-normal needs mean > 0 and sd > 0, "
                            f"has {m} and {s}")
        s2 = math.log(1.0 + (s / m) ** 2)
        return math.log(m) - s2 / 2, math.sqrt(s2)

    def _sample(self, rng, env):
        mu, sigma = self._mu_sigma(env)
        return float(rng.lognormal(mu, sigma))

    def _logpdf(self, x, env):
        if x <= 0:
            return -math.inf
        mu, sigma = self._mu_sigma(env)
        return (-math.log(x * sigma * math.sqrt(2 * math.pi))
                - (math.log(x) - mu) ** 2 / (2 * sigma ** 2))

    def _mass(self, env):
        mu, sigma = self._mu_sigma(env)
        lo, hi = self.range
        cdf = (lambda v: 0.0 if v <= 0 else _phi((math.log(v) - mu) / sigma))
        return cdf(hi) - cdf(lo)


class Uniform(Dist):
    kind = "uniform"

    def __init__(self, low, high, where):
        super().__init__(where)
        self.low, self.high = _param(low, f"{where}.low"), _param(high, f"{where}.high")

    def _lh(self, env):
        lo, hi = _value(self.low, env), _value(self.high, env)
        if not hi > lo:
            raise SpecError(f"{self.where}: uniform needs high > low, has [{lo}, {hi}]")
        return lo, hi

    def _sample(self, rng, env):
        lo, hi = self._lh(env)
        return float(rng.uniform(lo, hi))

    def _logpdf(self, x, env):
        lo, hi = self._lh(env)
        return -math.log(hi - lo) if lo <= x <= hi else -math.inf


class Poisson(Dist):
    kind = "poisson"
    discrete = True

    def __init__(self, mean, where, rng_range=None):
        super().__init__(where, rng_range)
        self.mean = _param(mean, f"{where}.mean")

    def _lam(self, env):
        lam = _value(self.mean, env)
        if not lam > 0:
            raise SpecError(f"{self.where}: a Poisson needs mean > 0, has {lam}")
        return lam

    def _sample(self, rng, env):
        return int(rng.poisson(self._lam(env)))

    def _logpdf(self, x, env):
        if int(x) != x or x < 0:
            return -math.inf
        lam = self._lam(env)
        return x * math.log(lam) - lam - math.lgamma(x + 1)

    def _mass(self, env):
        lam = self._lam(env)
        lo = max(0, math.ceil(self.range[0]))
        hi = min(math.floor(self.range[1]), int(lam + 40 * math.sqrt(lam) + 40))
        return sum(math.exp(k * math.log(lam) - lam - math.lgamma(k + 1))
                   for k in range(lo, hi + 1))


class Categorical(Dist):
    kind = "categorical"
    discrete = True

    def __init__(self, weights, where):
        super().__init__(where)
        if not isinstance(weights, dict) or not weights:
            raise SpecError(f"{where}: categorical is a mapping {{value: weight}}, "
                            f"not {weights!r}")
        self.values = list(weights)
        w = [_param(v, f"{where}[{k!r}]") for k, v in weights.items()]
        if any(not isinstance(x, float) or x < 0 for x in w) or sum(w) <= 0:
            raise SpecError(f"{where}: categorical weights are numbers >= 0, not all 0")
        total = sum(w)
        self.p = [x / total for x in w]

    def _sample(self, rng, env):
        return self.values[int(rng.choice(len(self.values), p=self.p))]

    def _logpdf(self, x, env):
        for v, p in zip(self.values, self.p):
            if v == x or str(v) == str(x):
                return math.log(p) if p > 0 else -math.inf
        return -math.inf


class Histogram(Dist):
    kind = "histogram"

    def __init__(self, bins, where):
        super().__init__(where)
        ok = (isinstance(bins, list) and bins
              and all(isinstance(b, (list, tuple)) and len(b) == 3 for b in bins))
        if not ok:
            raise SpecError(f"{where}: histogram bins are [[low, high, weight], ...]")
        self.bins = [(float(a), float(b), float(w)) for a, b, w in bins]
        if any(not b > a or w < 0 for a, b, w in self.bins):
            raise SpecError(f"{where}: each bin needs high > low and weight >= 0")
        total = sum(w for _, _, w in self.bins)
        if total <= 0:
            raise SpecError(f"{where}: the bins' weights are all 0")
        self.p = [w / total for _, _, w in self.bins]

    def _sample(self, rng, env):
        a, b, _ = self.bins[int(rng.choice(len(self.bins), p=self.p))]
        return float(rng.uniform(a, b))

    def _logpdf(self, x, env):
        d = sum(p / (b - a) for (a, b, _), p in zip(self.bins, self.p) if a <= x < b)
        if d == 0 and x == self.bins[-1][1]:
            a, b, _ = self.bins[-1]
            d = self.p[-1] / (b - a)
        return math.log(d) if d > 0 else -math.inf


class Constant(Dist):
    kind = "constant"
    discrete = True

    def __init__(self, value, where):
        super().__init__(where)
        self.value = value

    def _sample(self, rng, env):
        return self.value

    def _logpdf(self, x, env):
        return 0.0 if x == self.value or str(x) == str(self.value) else -math.inf


class Mixture:
    """A Gaussian mixture over several continuous variables, drawn together."""
    kind = "mixture"
    discrete = False

    def __init__(self, names, components, where):
        if not (isinstance(names, list) and names and all(isinstance(n, str) for n in names)):
            raise SpecError(f"{where}: names is a list of variable names")
        if not (isinstance(components, list) and components):
            raise SpecError(f"{where}: mixture is a list of {{weight, mean, cov}}")
        self.names, self.where = names, where
        k = len(names)
        self.w, self.mean, self.cov, self.chol, self.logdet = [], [], [], [], []
        for i, c in enumerate(components):
            at = f"{where}.mixture[{i}]"
            if not isinstance(c, dict) or set(c) != {"weight", "mean", "cov"}:
                raise SpecError(f"{at}: a component is {{weight, mean, cov}}")
            m = np.asarray(c["mean"], dtype=float)
            s = np.asarray(c["cov"], dtype=float)
            if m.shape != (k,) or s.shape != (k, k) or not np.allclose(s, s.T):
                raise SpecError(f"{at}: mean has {k} entries and cov is a symmetric "
                                f"{k}x{k} matrix (names: {', '.join(names)})")
            try:
                ch = np.linalg.cholesky(s)
            except np.linalg.LinAlgError:
                raise SpecError(f"{at}: cov is not positive definite") from None
            self.w.append(float(c["weight"]))
            self.mean.append(m), self.cov.append(s), self.chol.append(ch)
            self.logdet.append(2.0 * math.fsum(math.log(float(ch[j, j])) for j in range(k)))
        if any(w < 0 for w in self.w) or sum(self.w) <= 0:
            raise SpecError(f"{where}: mixture weights are numbers >= 0, not all 0")
        total = sum(self.w)
        self.w = [w / total for w in self.w]

    def sample(self, rng, env):
        i = int(rng.choice(len(self.w), p=self.w))
        z = [float(v) for v in rng.standard_normal(len(self.names))]
        m, ch = self.mean[i], self.chol[i]
        # summed in a fixed order, not by BLAS, so a seed gives the same bits on every host
        return {n: float(m[j]) + math.fsum(float(ch[j, k]) * z[k] for k in range(j + 1))
                for j, n in enumerate(self.names)}

    def logpdf(self, values, env):
        x = np.array([values[n] for n in self.names], dtype=float)
        k = len(self.names)
        terms = []
        for w, m, ch, ld in zip(self.w, self.mean, self.chol, self.logdet):
            if w > 0:
                # d' S^-1 d = |L^-1 d|^2, L^-1 d by forward substitution in a fixed order
                d = [float(x[j]) - float(m[j]) for j in range(k)]
                y = []
                for j in range(k):
                    y.append((d[j] - math.fsum(float(ch[j, i]) * y[i] for i in range(j)))
                             / float(ch[j, j]))
                q = math.fsum(v * v for v in y)
                terms.append(math.log(w) - 0.5 * (k * math.log(2 * math.pi) + ld + q))
        top = max(terms)
        return top + math.log(sum(math.exp(t - top) for t in terms))


KINDS = ("normal", "lognormal", "uniform", "poisson", "categorical", "histogram", "constant")


def spec(d, where):
    """One distribution from its YAML mapping: exactly one of KINDS, and `range` where the kind
    takes one. Other keys of the mapping (`name`, `given`) are the caller's."""
    kinds = [k for k in KINDS if k in d]
    if len(kinds) != 1:
        raise SpecError(f"{where}: exactly one of {', '.join(KINDS)}, has "
                        f"{', '.join(kinds) or 'none'}")
    kind = kinds[0]
    a = d[kind]
    r = d.get("range")
    if kind in ("normal", "lognormal"):
        if not isinstance(a, dict) or set(a) != {"mean", "sd"}:
            raise SpecError(f"{where}.{kind}: {{mean, sd}}")
        return (Normal if kind == "normal" else LogNormal)(a["mean"], a["sd"], where, r)
    if r is not None and kind != "poisson":
        raise SpecError(f"{where}: range applies to normal, lognormal and poisson, not {kind}")
    if kind == "uniform":
        if not isinstance(a, dict) or set(a) != {"low", "high"}:
            raise SpecError(f"{where}.uniform: {{low, high}}")
        return Uniform(a["low"], a["high"], where)
    if kind == "poisson":
        if not isinstance(a, dict) or set(a) != {"mean"}:
            raise SpecError(f"{where}.poisson: {{mean}}")
        return Poisson(a["mean"], where, r)
    if kind == "categorical":
        return Categorical(a, where)
    if kind == "histogram":
        return Histogram(a, where)
    return Constant(a, where)
