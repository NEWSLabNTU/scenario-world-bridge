"""Plug-ins found by entry point, the t-wise covering array, and the command line."""
import io
import itertools
import json
import subprocess
import sys
import tempfile
import textwrap
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from _files import EXAMPLES, Dir  # noqa: E402
from scensim_sampler import Sampler, plugins  # noqa: E402
from scensim_sampler.cli import main  # noqa: E402
from scensim_sampler.plugins.twise import covering_array  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


class EntryPoints(unittest.TestCase):
    """A package outside scensim adds a plug-in by declaring it in the group."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = Path(self.tmp.name)
        (d / "seven_plugin.py").write_text(textwrap.dedent("""
            from scensim_sampler.plugins import Plugin

            class Seven(Plugin):
                def __init__(self, content, *, base_dir, parameter, scenario):
                    self.names = (parameter,)
                    self.value = float(content)

                def draw(self, rng, index, values):
                    return {self.names[0]: self.value}
        """))
        info = d / "seven_plugin-1.0.dist-info"
        info.mkdir()
        (info / "METADATA").write_text("Metadata-Version: 2.1\nName: seven-plugin\nVersion: 1.0\n")
        (info / "entry_points.txt").write_text("[scensim.sampler]\nseven = seven_plugin:Seven\n")
        sys.path.insert(0, str(d))
        self.d = Dir()

    def tearDown(self):
        sys.path.remove(self.tmp.name)
        sys.modules.pop("seven_plugin", None)
        self.tmp.cleanup()
        self.d.close()

    def test_an_installed_plugin_is_found_and_drawn(self):
        self.assertEqual(plugins.known()["seven"], "seven_plugin:Seven")
        p = self.d.dist("""<Stochastic numberOfTestRuns="2" randomSeed="1">
          <StochasticDistribution parameterName="speed">
            <UserDefinedDistribution type="scensim.sampler:seven">7.5</UserDefinedDistribution>
          </StochasticDistribution></Stochastic>""")
        b = Sampler.from_file(p).draw()
        self.assertEqual([r["values"]["speed"] for r in b.records], [7.5, 7.5])
        self.assertEqual(b.records[0]["sampler"]["plugins"], ["seven"])

    def test_a_type_outside_the_group_is_refused(self):
        p = self.d.dist("""<Stochastic numberOfTestRuns="2" randomSeed="1">
          <StochasticDistribution parameterName="speed">
            <UserDefinedDistribution type="other:seven">7.5</UserDefinedDistribution>
          </StochasticDistribution></Stochastic>""")
        with self.assertRaisesRegex(ValueError, r"not a scensim sampler \(scensim.sampler:"):
            Sampler.from_file(p)


def covered(rows, factors, t):
    names = list(factors)
    for combo in itertools.combinations(names, t):
        need = set(itertools.product(*(factors[n] for n in combo)))
        have = {tuple(r[n] for n in combo) for r in rows}
        if need - have:
            return False
    return True


class TWise(unittest.TestCase):
    def test_every_t_combination_is_covered_in_fewer_rows_than_the_product(self):
        f = {"a": [1, 2, 3], "b": ["x", "y", "z"], "c": [True, False], "d": [0, 1, 2]}
        for t in (1, 2, 3):
            rows = covering_array(f, t)
            self.assertTrue(covered(rows, f, t), t)
        self.assertEqual(len(covering_array(f, 1)), 3)
        self.assertLessEqual(len(covering_array(f, 2)), 12)
        self.assertLess(len(covering_array(f, 2)), 3 * 3 * 2 * 3)
        self.assertEqual(len(covering_array(f, 4)), 3 * 3 * 2 * 3)

    def test_t_is_a_parameter_and_under_stochastic_runs_cycle_the_rows(self):
        d = Dir(cells="surface,light", fixed="")
        try:
            p = d.dist("""<Stochastic numberOfTestRuns="9" randomSeed="1">
              <StochasticDistribution parameterName="surface">
                <UserDefinedDistribution type="scensim.sampler:twise">
t: 2
factors: {surface: [dry, wet, icy], light: [day, night], kind: [car, truck]}
                </UserDefinedDistribution>
              </StochasticDistribution>
              <StochasticDistribution parameterName="speed">
                <UniformDistribution><Range lowerLimit="5" upperLimit="15"/></UniformDistribution>
              </StochasticDistribution></Stochastic>""")
            b = Sampler.from_file(p).draw()
            cells = [(r["cell"]["surface"], r["cell"]["light"]) for r in b.records]
            self.assertEqual(len(set(cells)), 6)
            self.assertEqual(b.records[0]["sampler"]["plugins"], ["twise", "independent"])
            self.assertTrue(all(5 <= r["values"]["speed"] <= 15 for r in b.records))
        finally:
            d.close()


class CommandLine(unittest.TestCase):
    def run_cli(self, *args):
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            rc = main(list(args))
        return rc, out.getvalue(), err.getvalue()

    def test_draw_writes_records_and_check_passes_them(self):
        with tempfile.TemporaryDirectory() as t:
            out = Path(t) / "s.jsonl"
            rc, _, err = self.run_cli("draw", str(EXAMPLES / "walker_dartout_dist.xosc"),
                                      "-n", "20", "--seed", "3", "-o", str(out))
            self.assertEqual(rc, 0, err)
            self.assertIn("20 records", err)
            self.assertEqual(len(out.read_text().splitlines()), 20)
            rc, _, err = self.run_cli("check", str(out), "--scenario",
                                      str(EXAMPLES / "walker_dartout.xosc"))
            self.assertEqual(rc, 0, err)
            self.assertIn("20 records, 0 problems", err)
            r = json.loads(out.read_text().splitlines()[0])
            r["values"]["ego_speed"] = 0.0
            bad = Path(t) / "bad.jsonl"
            bad.write_text(json.dumps(r) + "\n")
            rc, so, _ = self.run_cli("check", str(bad))
            self.assertEqual(rc, 1)
            self.assertIn("record 0: id: not the hash", so)

    def test_yaml_output_proposal_and_knobs(self):
        with tempfile.TemporaryDirectory() as t:
            out = Path(t) / "s.yaml"
            rc, _, err = self.run_cli(
                "draw", str(EXAMPLES / "walker_dartout_dist.xosc"), "-n", "5", "--seed", "1",
                "--proposal", str(EXAMPLES / "proposals" / "near_ttc_short.yaml"),
                "--knob", "speed_limit=10", "--format", "yaml", "-o", str(out))
            self.assertEqual(rc, 0, err)
            rc, _, err = self.run_cli("check", str(out))
            self.assertEqual(rc, 0, err)

    def test_a_bad_input_is_one_line_and_exit_2(self):
        rc, _, err = self.run_cli("draw", str(EXAMPLES / "walker_dartout.xosc"), "-n", "1")
        self.assertEqual(rc, 2)
        self.assertIn("declares no distribution: give a ParameterValueDistribution file", err)

    def test_the_module_runs_as_a_program(self):
        r = subprocess.run([sys.executable, "-m", "scensim_sampler.cli", "plugins"], cwd=ROOT,
                           capture_output=True, text=True, check=True)
        self.assertIn("scensim.sampler:joint", r.stdout)


if __name__ == "__main__":
    unittest.main()
