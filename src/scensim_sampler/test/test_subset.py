"""The compilable subset (roadmap 092 S2, docs/design/compilable-subset.md): the five case
studies' verdicts, a refusal per seats and derived row, the XML path of a refusal, the ego as
the batch's policy or refused, distributions read through, and SSv2's own example scenarios
measured (skipped without the submodule)."""
import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scensim_sampler import subset  # noqa: E402
from scensim_sampler.cli import main  # noqa: E402

CASES = Path(__file__).resolve().parents[1] / "examples" / "cases"
SSV2 = Path(__file__).resolve().parents[3] / "src" / "scenario_simulator_v2"

BASE = """<?xml version="1.0"?>
<OpenSCENARIO>
  <ParameterDeclarations>{params}</ParameterDeclarations>
  <Entities>
    <ScenarioObject name="ego"><Vehicle name="c"/><ObjectController><Controller name="ego">
      <Properties><Property name="isEgo" value="true"/></Properties></Controller>
    </ObjectController></ScenarioObject>
    <ScenarioObject name="cone"><MiscObject name="cone"/></ScenarioObject>
    {entities}
  </Entities>
  <Storyboard>
    <Init><Actions>{init}</Actions></Init>
    <Story name="s"><Act name="a"><ManeuverGroup name="g"><Actors/><Maneuver name="m">
      <Event name="e"><Action name="x">{action}</Action>
        <StartTrigger><ConditionGroup><Condition name="c">{condition}</Condition>
        </ConditionGroup></StartTrigger></Event></Maneuver></ManeuverGroup></Act></Story>
    <StopTrigger/>
  </Storyboard>
</OpenSCENARIO>
"""
SPEED = ("<PrivateAction><LongitudinalAction><SpeedAction><SpeedActionDynamics/>"
         "<SpeedActionTarget><AbsoluteTargetSpeed value='3'/></SpeedActionTarget></SpeedAction>"
         "</LongitudinalAction></PrivateAction>")
TIME = "<ByValueCondition><SimulationTimeCondition value='1' rule='greaterThan'/></ByValueCondition>"


def scenario(d, params="", entities="", init="", action=SPEED, condition=TIME, name="s.xosc"):
    p = Path(d) / name
    p.write_text(BASE.format(params=params, entities=entities, init=init, action=action,
                             condition=condition))
    return p


class CaseStudies(unittest.TestCase):
    def verdict(self, name, ego="policy"):
        return subset.check(CASES / f"{name}.xosc", ego=ego)

    def test_cs1_compiles_with_its_trigger_derived_and_refuses_the_ttc_form(self):
        v = self.verdict("cs1_walker_dartout")
        self.assertTrue(v.compiles)
        self.assertEqual(v.conditions, ["behaviour model scensim.behaviour:gap_acceptance_walker:"
                                        " a batched form of the model"])
        t = self.verdict("cs1_walker_dartout_ttc")
        r = t.refusals[0]
        self.assertEqual((r["class"], r["construct"]), ("derived", "TimeToCollisionCondition"))
        self.assertIn("DistanceCondition on a parameter the sampler derives", r["fix"])
        self.assertTrue(r["at"].startswith("/OpenSCENARIO/Storyboard/Story[@name='main']/"
                                           "Act[@name='act']/ManeuverGroup[@name='dart']/"))
        self.assertTrue(r["at"].endswith("/EntityCondition/TimeToCollisionCondition"))

    def test_cs2_compiles_if_the_slot_has_a_camera_for_night(self):
        v = self.verdict("cs2_lead_brakes_wet_night")
        self.assertTrue(v.compiles)
        self.assertIn("world knob illumination=night: a slot camera (illumination), and its "
                      "render time", v.conditions)
        self.assertFalse(any("surface" in c for c in v.conditions))   # a material: batch

    def test_cs3_compiles_with_the_ego_as_policy_and_is_refused_with_its_autopilot(self):
        v = self.verdict("cs3_stale_map")
        self.assertTrue(v.compiles)
        self.assertTrue(v.line().startswith("compiles with the ego replaced by the batch's "
                                            "policy; the scenario's own autopilot does not "
                                            "run in a batch"))
        self.assertIn(subset.PILOT_PARAM, v.notes)
        s = self.verdict("cs3_stale_map", ego="seats")
        self.assertEqual(s.refusals[0]["construct"], "the ego's autopilot")

    def test_cs4_compiles_with_the_fault_as_a_mask(self):
        v = self.verdict("cs4_lidar_dropout_cutin")
        self.assertTrue(v.compiles)
        self.assertEqual(v.counts.get("batch", 0) >= 1, True)

    def test_cs5_compiles_only_with_a_fog_renderer(self):
        v = self.verdict("cs5_fog_mid_route")
        self.assertTrue(v.compiles)
        self.assertTrue(any(c.startswith("Fog: a slot camera, and a renderer with fog")
                            for c in v.conditions))


class Rows(unittest.TestCase):
    def refused(self, construct, **kw):
        with tempfile.TemporaryDirectory() as d:
            v = subset.check(scenario(d, **kw))
        self.assertFalse(v.compiles, v.line())
        self.assertEqual(v.refusals[0]["construct"], construct, v.line())
        return v

    def test_the_plain_scenario_compiles_unconditionally(self):
        with tempfile.TemporaryDirectory() as d:
            v = subset.check(scenario(d))
        self.assertTrue(v.compiles)
        self.assertEqual(v.conditions, [])

    def test_each_seats_row_is_refused_by_name(self):
        for construct, action in (
                ("AddEntityAction", "<GlobalAction><EntityAction entityRef='x'><AddEntityAction/>"
                                    "</EntityAction></GlobalAction>"),
                ("ParameterAction", "<GlobalAction><ParameterAction parameterRef='p'><SetAction "
                                    "value='1'/></ParameterAction></GlobalAction>"),
                ("ControllerAction", "<PrivateAction><ControllerAction><AssignControllerAction/>"
                                     "</ControllerAction></PrivateAction>"),
                ("VisibilityAction", "<PrivateAction><VisibilityAction graphics='false'/>"
                                     "</PrivateAction>"),
                ("CustomCommandAction V2ITrafficSignalStateAction(1, red)",
                 "<UserDefinedAction><CustomCommandAction type='V2ITrafficSignalStateAction(1, "
                 "red)'/></UserDefinedAction>")):
            with self.subTest(construct):
                v = self.refused(construct, action=action)
                self.assertEqual(v.refusals[0]["class"], "seats")
        udvc = ("<ByValueCondition><UserDefinedValueCondition name='{}' value='1' "
                "rule='equalTo'/></ByValueCondition>")
        for name in ("ego.currentState", "/count_up"):
            self.refused(f"UserDefinedValueCondition {name}", condition=udvc.format(name))
        with tempfile.TemporaryDirectory() as d:   # SSv2's built-in function: per slot
            for name in ("RelativeHeadingCondition(ego)", "RelativeHeadingCondition(ego, 34507, 50)"):
                self.assertTrue(subset.check(scenario(d, condition=udvc.format(name))).compiles)

    def test_a_parameter_set_before_the_run_is_fixed_for_the_slot(self):
        init = ("<GlobalAction><ParameterAction parameterRef='p'><SetAction value='1'/>"
                "</ParameterAction></GlobalAction>")
        with tempfile.TemporaryDirectory() as d:
            self.assertTrue(subset.check(scenario(d, init=init)).compiles)

    def test_a_second_autopilot_and_an_agent_pilot_are_seats_only(self):
        ent = ("<ScenarioObject name='npc'><Vehicle name='c'/><ObjectController><Controller "
               "name='Autoware'/></ObjectController></ScenarioObject>")
        self.assertEqual(self.refused("2 egos (ego, npc)", entities=ent).refusals[0]["class"],
                         "seats")
        ent = ("<ScenarioObject name='npc'><Vehicle name='c'/><ObjectController><Controller "
               "name='agent'/></ObjectController></ScenarioObject>")
        v = self.refused("controller agent", entities=ent)
        # an entity made an ego in Init (as SSv2's examples do) is an ego too
        init = ("<Private entityRef='npc'><PrivateAction><ControllerAction><AssignControllerAction>"
                "<Controller name='x'><Properties><Property name='isEgo' value='true'/>"
                "</Properties></Controller></AssignControllerAction></ControllerAction>"
                "</PrivateAction></Private>")
        ent2 = "<ScenarioObject name='npc'><Vehicle name='c'/></ScenarioObject>"
        self.refused("2 egos (ego, npc)", entities=ent2, init=init)
        # an external agent pilot assigned to a non-ego in Init stays seats
        agent = ("<Private entityRef='npc'><PrivateAction><ControllerAction><AssignControllerAction>"
                 "<Controller name='agent'/></AssignControllerAction></ControllerAction>"
                 "</PrivateAction></Private>")
        self.assertEqual(self.refused("ControllerAction", entities=ent2, init=agent)
                         .refusals[0]["class"], "seats")
        self.assertEqual(v.refusals[0]["class"], "seats")

    def test_an_unlisted_element_is_refused_not_admitted(self):
        v = self.refused("AppearanceAction", action="<PrivateAction><AppearanceAction>"
                                                   "<AnimationAction/></AppearanceAction>"
                                                   "</PrivateAction>")
        self.assertEqual(v.refusals[0]["class"], "unlisted")

    def test_an_unknown_command_is_refused_and_the_known_ones_compile(self):
        for t, ok in (("exitSuccess", True), ("exitFailure", True), ("scensim.fault", True),
                      ("echo hello", False), ("", False)):
            with self.subTest(t), tempfile.TemporaryDirectory() as d:
                a = f"<UserDefinedAction><CustomCommandAction type='{t}'/></UserDefinedAction>"
                self.assertEqual(subset.check(scenario(d, action=a)).compiles, ok)

    def test_a_distribution_is_read_through_to_its_scenario(self):
        with tempfile.TemporaryDirectory() as d:
            scenario(d, action="<PrivateAction><VisibilityAction/></PrivateAction>")
            dist = Path(d) / "dist.xosc"
            dist.write_text("<OpenSCENARIO><ParameterValueDistribution><ScenarioFile "
                            "filepath='s.xosc'/></ParameterValueDistribution></OpenSCENARIO>")
            v = subset.check(dist)
        self.assertEqual(v.refusals[0]["construct"], "VisibilityAction")
        self.assertTrue(v.path.endswith("s.xosc"))

    def test_ssv2_yaml_scenario_format_is_read_as_the_same_tree(self):
        y = ("OpenSCENARIO:\n  Entities:\n    ScenarioObject:\n      - name: npc\n"
             "        Vehicle: {name: c}\n  Storyboard:\n    Init:\n      Actions: {}\n"
             "    Story:\n      - name: s\n        Act:\n          - name: a\n"
             "            ManeuverGroup:\n              - name: g\n                Maneuver:\n"
             "                  - name: m\n                    Event:\n"
             "                      - name: e\n                        Action:\n"
             "                          - name: x\n                            PrivateAction:\n"
             "                              - VisibilityAction: {graphics: false}\n")
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "s.yaml"
            p.write_text(y)
            v = subset.check(p)
        self.assertEqual(v.refusals[0]["construct"], "VisibilityAction")
        self.assertIn("Story[@name='s']/Act[@name='a']", v.refusals[0]["at"])


class Cli(unittest.TestCase):
    def test_exit_codes_lines_and_json(self):
        out = io.StringIO()
        with tempfile.TemporaryDirectory() as d, redirect_stdout(out):
            j = Path(d) / "v.json"
            rc = main(["compilable", str(CASES / "cs3_stale_map.xosc"),
                       str(CASES / "cs1_walker_dartout_ttc.xosc"), "--all", "--json", str(j)])
            doc = json.loads(j.read_text())
        self.assertEqual(rc, 1)
        self.assertIn("1 of 2 compile", out.getvalue())
        self.assertEqual(doc["format"], "scensim.compilable/1")
        self.assertEqual([s["compiles"] for s in doc["scenarios"]], [True, False])
        with redirect_stdout(io.StringIO()):
            self.assertEqual(main(["compilable", str(CASES / "cs4_lidar_dropout_cutin.xosc")]), 0)
            self.assertEqual(main(["compilable", "--ego", "seats",
                                   str(CASES / "cs4_lidar_dropout_cutin.xosc")]), 1)


@unittest.skipUnless((SSV2 / "test_runner").is_dir(), "no scenario_simulator_v2 submodule")
class Ssv2Examples(unittest.TestCase):
    def test_every_example_is_read_and_has_a_verdict(self):
        files = sorted((SSV2 / "test_runner/scenario_test_runner/scenario").glob("*.yaml")) + \
            sorted((SSV2 / "openscenario/openscenario_interpreter/example").glob("*.xosc"))
        verdicts = [subset.check(f) for f in files]
        s = subset.summary(verdicts)
        self.assertEqual(s["scenarios"], len(files))
        self.assertGreater(s["scenarios"], 50)
        self.assertEqual(s["compile"] + s["refused"], s["scenarios"])
        unlisted = [r for v in verdicts for r in v.refusals if r["class"] == "unlisted"]
        self.assertEqual(unlisted, [], "an element SSv2's examples use has no row")


if __name__ == "__main__":
    unittest.main()
