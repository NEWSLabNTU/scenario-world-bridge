"""Scenario and distribution files written into a temporary directory for a test."""
import tempfile
from pathlib import Path

EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "walker_dartout"

SCENARIO = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-11-21T00:00:00" author="t" description="t">
    <Properties>
      <Property name="scensim.category" value="R1.rear_end.lead_brakes"/>
      <Property name="scensim.cell_attributes" value="{cells}"/>
      {fixed}
    </Properties>
  </FileHeader>
  <ParameterDeclarations>
    <ParameterDeclaration name="speed" parameterType="double" value="10"/>
    <ParameterDeclaration name="gap" parameterType="double" value="2"/>
    <ParameterDeclaration name="lanes" parameterType="unsignedInt" value="2"/>
    <ParameterDeclaration name="kind" parameterType="string" value="car"/>
    <ParameterDeclaration name="wet" parameterType="boolean" value="false"/>
  </ParameterDeclarations>
  <Entities><ScenarioObject name="ego"/><ScenarioObject name="lead"/></Entities>
  <Storyboard/>
</OpenSCENARIO>
"""

DIST = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-11-21T00:00:00" author="t" description="t"/>
  <ParameterValueDistribution>
    <ScenarioFile filepath="s.xosc"/>
    {body}
  </ParameterValueDistribution>
</OpenSCENARIO>
"""


class Dir:
    def __init__(self, cells="surface", fixed='<Property name="scensim.cell.surface" value="dry"/>'):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        self.write("s.xosc", SCENARIO.format(cells=cells, fixed=fixed))

    def write(self, name, text):
        p = self.path / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
        return p

    def dist(self, body, name="d.xosc"):
        return self.write(name, DIST.format(body=body))

    def close(self):
        self.tmp.cleanup()
