"""The restricted expression, and the distributions: moments, truncation, densities."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scensim_sampler import dist  # noqa: E402
from scensim_sampler.dist import SpecError  # noqa: E402
from scensim_sampler.expr import Expr, ExprError  # noqa: E402

N = 20000


def draws(d, env=None, seed=1):
    rng = np.random.Generator(np.random.PCG64(seed))
    return np.array([d.sample(rng, env or {}) for _ in range(N)], dtype=float)


class Expressions(unittest.TestCase):
    def test_arithmetic_names_functions_and_if_else(self):
        self.assertEqual(Expr("a * b + sqrt(16)")({"a": 2, "b": 3}), 10.0)
        self.assertEqual(Expr("'night' if hour < 6 else 'day'")({"hour": 3}), "night")
        self.assertTrue(Expr("x in ('a', 'b') and not y")({"x": "a", "y": False}))
        self.assertTrue(Expr("1 < x <= 3")({"x": 3}))
        self.assertAlmostEqual(Expr("0.8 * g")({}), 0.8 * 9.80665)

    def test_anything_else_is_refused_by_name(self):
        for text in ("__import__('os')", "x.attr", "x[0]", "lambda: 1", "open('f')",
                     "[i for i in x]", "f(x=1)"):
            with self.assertRaises(ExprError, msg=text):
                Expr(text)

    def test_a_runaway_power_or_repetition_is_refused_not_computed(self):
        with self.assertRaisesRegex(ExprError, "exponent 10000000000"):
            Expr("10 ** 10 ** 10")({})
        with self.assertRaisesRegex(ExprError, "OverflowError"):
            Expr("10.0 ** 1000")({})
        with self.assertRaisesRegex(ExprError, "repeated 10000000000 times"):
            Expr("'x' * 10 ** 10")({})
        self.assertEqual(Expr("2 ** 10 + len_ * 0")({"len_": 1}), 1024)
        self.assertEqual(Expr("'ab' * 2")({}), "abab")

    def test_an_unknown_name_lists_the_known_ones(self):
        with self.assertRaisesRegex(ExprError, r"speed is not known here \(known: a, b\)"):
            Expr("speed * 2")({"a": 1, "b": 2})


class Moments(unittest.TestCase):
    def test_normal(self):
        x = draws(dist.Normal(3.0, 0.5, "t"))
        self.assertAlmostEqual(x.mean(), 3.0, delta=0.02)
        self.assertAlmostEqual(x.std(), 0.5, delta=0.02)

    def test_lognormal_takes_the_distributions_own_mean_and_sd(self):
        x = draws(dist.LogNormal(2.0, 0.6, "t"))
        self.assertAlmostEqual(x.mean(), 2.0, delta=0.03)
        self.assertAlmostEqual(x.std(), 0.6, delta=0.03)

    def test_uniform(self):
        x = draws(dist.Uniform(1.0, 3.0, "t"))
        self.assertTrue(x.min() >= 1.0 and x.max() <= 3.0)
        self.assertAlmostEqual(x.mean(), 2.0, delta=0.02)

    def test_poisson(self):
        x = draws(dist.Poisson(4.0, "t"))
        self.assertAlmostEqual(x.mean(), 4.0, delta=0.05)
        self.assertAlmostEqual(x.var(), 4.0, delta=0.15)

    def test_categorical(self):
        d = dist.Categorical({"a": 1, "b": 3}, "t")
        rng = np.random.Generator(np.random.PCG64(2))
        share = sum(d.sample(rng, {}) == "b" for _ in range(N)) / N
        self.assertAlmostEqual(share, 0.75, delta=0.01)

    def test_histogram(self):
        x = draws(dist.Histogram([[0, 1, 1], [1, 3, 3]], "t"))
        self.assertAlmostEqual((x < 1).mean(), 0.25, delta=0.01)
        self.assertTrue(x.min() >= 0 and x.max() <= 3)


class Truncation(unittest.TestCase):
    def test_a_range_truncates_and_the_density_is_renormalised(self):
        d = dist.Normal(0.0, 1.0, "t", [0.0, 10.0])
        x = draws(d)
        self.assertGreaterEqual(x.min(), 0.0)
        # half-normal: density at 0 is 2 / sqrt(2 pi)
        self.assertAlmostEqual(math.exp(d.logpdf(0.0, {})), 2 / math.sqrt(2 * math.pi), 6)
        self.assertEqual(d.logpdf(-0.1, {}), -math.inf)

    def test_each_density_integrates_to_one(self):
        cases = [dist.Normal(1, 2, "t", [-1, 4]), dist.LogNormal(2, 1, "t", [0.5, 6]),
                 dist.Uniform(1, 3, "t"), dist.Histogram([[0, 1, 1], [1, 3, 3]], "t")]
        for d in cases:
            xs = np.linspace(-5, 12, 170001)
            p = np.exp([d.logpdf(float(x), {}) for x in xs])
            self.assertAlmostEqual(float(np.sum(p) * (xs[1] - xs[0])), 1.0, delta=2e-3,
                                   msg=d.kind)
        pz = dist.Poisson(3, "t", [1, 5])
        self.assertAlmostEqual(sum(math.exp(pz.logpdf(k, {})) for k in range(0, 30)), 1.0, 9)

    def test_a_range_that_holds_almost_nothing_is_refused(self):
        d = dist.Normal(0.0, 0.01, "far", [5.0, 6.0])
        with self.assertRaisesRegex(SpecError, "far: 10000 draws, none inside range"):
            d.sample(np.random.Generator(np.random.PCG64(0)), {})


class Conditioning(unittest.TestCase):
    def test_a_parameter_may_be_an_expression_of_earlier_values(self):
        d = dist.Normal("1.0 + 0.5 * speed", 0.1, "t")
        x = draws(d, {"speed": 4.0})
        self.assertAlmostEqual(x.mean(), 3.0, delta=0.01)

    def test_mixture_draws_together_and_its_density_is_the_weighted_sum(self):
        m = dist.Mixture(["a", "b"], [{"weight": 1, "mean": [0, 0], "cov": [[1, 0.8], [0.8, 1]]}],
                         "t")
        rng = np.random.Generator(np.random.PCG64(3))
        xy = np.array([[v["a"], v["b"]] for v in (m.sample(rng, {}) for _ in range(N))])
        self.assertAlmostEqual(float(np.corrcoef(xy.T)[0, 1]), 0.8, delta=0.02)
        det = 1 - 0.64
        self.assertAlmostEqual(m.logpdf({"a": 0.0, "b": 0.0}, {}),
                               -math.log(2 * math.pi) - 0.5 * math.log(det), 9)

    def test_spec_refuses_a_malformed_mapping_by_name(self):
        with self.assertRaisesRegex(SpecError, "exactly one of"):
            dist.spec({"normal": {"mean": 1, "sd": 1}, "uniform": {"low": 0, "high": 1}}, "v")
        with self.assertRaisesRegex(SpecError, r"v.normal: \{mean, sd\}"):
            dist.spec({"normal": {"mu": 1, "sd": 1}}, "v")
        with self.assertRaisesRegex(SpecError, "range applies to"):
            dist.spec({"uniform": {"low": 0, "high": 1}, "range": [0, 1]}, "v")


if __name__ == "__main__":
    unittest.main()
