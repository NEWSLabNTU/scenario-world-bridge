"""Knobs bound by property, cells computed from values (093 D7 gap 1; scenario-description.md
sections 3 and 7): the binding's parse and refusals, a cell from the record's values through
the bindings, a labelled-only attribute refused when drawing and when checking, and derived
cell attributes allowed only when they read parameters, transitively."""
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scensim_sampler import Sampler  # noqa: E402
from scensim_sampler.dist import SpecError  # noqa: E402
from scensim_sampler.record import check  # noqa: E402
from scensim_sampler.xosc import Scenario  # noqa: E402

SCEN = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-11-21T00:00:00" author="t" description="t">
    <Properties>
      <Property name="scensim.category" value="R1.rear_end.lead_brakes"/>
      <Property name="scensim.cell_attributes" value="{cells}"/>
      {props}
    </Properties>
  </FileHeader>
  <ParameterDeclarations>
    <ParameterDeclaration name="speed" parameterType="double" value="10"/>
    <ParameterDeclaration name="light" parameterType="string" value="day"/>
    <ParameterDeclaration name="road" parameterType="string" value="dry"/>
  </ParameterDeclarations>
  <Entities><ScenarioObject name="ego"/></Entities>
  <Storyboard/>
</OpenSCENARIO>
"""
DIST = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-11-21T00:00:00" author="t" description="t"/>
  <ParameterValueDistribution>
    <ScenarioFile filepath="s.xosc"/>
    <Stochastic numberOfTestRuns="40" randomSeed="3">
      <StochasticDistribution parameterName="speed">
        <UserDefinedDistribution type="scensim.sampler:joint">m.yaml</UserDefinedDistribution>
      </StochasticDistribution>
    </Stochastic>
  </ParameterValueDistribution>
</OpenSCENARIO>
"""
MODEL = """
model: 1
knobs: {{limit: 13.9}}
header: {{name: t, dataset: invented, version: "1", licence: CC0}}
variables:
  - {{name: speed, uniform: {{low: 5, high: 15}}}}
  - {{name: light, categorical: {{day: 0.5, night: 0.5}}}}
  - {{name: road, categorical: {{dry: 0.5, wet: 0.5}}}}
{derived}
"""


class Tmp:
    def __init__(self, cells="light,road", props="", derived=""):
        self.d = tempfile.TemporaryDirectory()
        p = Path(self.d.name)
        (p / "s.xosc").write_text(SCEN.format(cells=cells, props=props))
        (p / "d.xosc").write_text(DIST)
        (p / "m.yaml").write_text(textwrap.dedent(MODEL.format(derived=derived)))
        self.path = p

    def __enter__(self):
        return self.path

    def __exit__(self, *a):
        self.d.cleanup()


BIND = ('<Property name="scensim.knob.illumination" value="$light"/>'
        '<Property name="scensim.knob.surface" value="$road"/>')


class Knobs(unittest.TestCase):
    def test_a_binding_names_a_declared_parameter(self):
        with Tmp(props=BIND) as d:
            self.assertEqual(Scenario(d / "s.xosc").knobs, {"illumination": "light",
                                                            "surface": "road"})
        for bad in ('<Property name="scensim.knob.surface" value="wet"/>',
                    '<Property name="scensim.knob.surface" value="$nope"/>'):
            with Tmp(props=bad) as d, self.assertRaisesRegex(SpecError, "bound to a declared"):
                Scenario(d / "s.xosc")

    def test_the_cell_is_computed_from_the_values(self):
        with Tmp(cells="illumination,surface,speed", props=BIND) as d:
            b = Sampler.from_file(d / "d.xosc").draw(n=40)
            sc = Scenario(d / "s.xosc")
            for r in b.records:
                v = r["values"]
                self.assertEqual(r["cell"], {"illumination": v["light"], "surface": v["road"],
                                             "speed": v["speed"]})
                self.assertEqual(check(r, scenario=sc), [])
            seen = {(r["cell"]["illumination"], r["cell"]["surface"]) for r in b.records}
            self.assertEqual(len(seen), 4)   # night and wet are drawn, and reach the cell

    def test_a_labelled_cell_is_refused_when_drawing_and_when_checking(self):
        with Tmp(cells="weather", props=BIND) as d:
            with self.assertRaisesRegex(SpecError, "'weather' is neither a bound knob"):
                Sampler.from_file(d / "d.xosc").draw(n=1)
        with Tmp(cells="illumination", props=BIND) as d:
            r = Sampler.from_file(d / "d.xosc").draw(n=1).records[0]
            sc = Scenario(d / "s.xosc")
            other = "day" if r["values"]["light"] == "night" else "night"
            forged = dict(r, cell={"illumination": other})
            self.assertTrue(any("but light is" in p for p in check(forged, scenario=sc)))

    def test_a_derived_cell_reads_parameters_only_transitively(self):
        ok = 'derived: {glare: "speed * 2", level: "glare + 1"}'
        with Tmp(cells="level", derived=ok) as d:
            r = Sampler.from_file(d / "d.xosc").draw(n=1).records[0]
            self.assertEqual(r["cell"]["level"], r["values"]["speed"] * 2 + 1)
        # a value the scenario does not declare (a world default the model assumes, not bound
        # by the scenario) is latent to the cell
        latent = 'derived: {hidden: "limit * 2", level: "hidden + 1"}'
        with Tmp(cells="level", derived=latent) as d:
            with self.assertRaisesRegex(SpecError, "'level' is neither"):
                Sampler.from_file(d / "d.xosc").draw(n=1)


if __name__ == "__main__":
    unittest.main()
