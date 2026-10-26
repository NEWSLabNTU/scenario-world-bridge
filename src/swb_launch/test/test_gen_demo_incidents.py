"""Unit tests for scripts/gen_demo_incidents.py, the demo tour's incident templates (roadmap 033
step 4d). Generates every template from the repo's sites file and from a step 4b-shaped one
(one parked car, no speeds), and checks what the 40 km/h tour relies on. No ROS:

    python3 -m pytest src/swb_launch/test/test_gen_demo_incidents.py
"""

import copy
import importlib.machinery
import importlib.util
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

_SCRIPT = Path(__file__).resolve().parents[3] / "scripts" / "gen_demo_incidents.py"
_loader = importlib.machinery.SourceFileLoader("gen_demo_incidents", str(_SCRIPT))
_spec = importlib.util.spec_from_loader("gen_demo_incidents", _loader)
gen = importlib.util.module_from_spec(_spec)
_loader.exec_module(gen)

SITES = yaml.safe_load(gen.SITES.read_text())


def parse(i, sites=SITES):
    return ET.fromstring(gen.template(i, sites))


def objects(root):
    return {o.get("name"): o for o in root.iter("ScenarioObject")}


class GenDemoIncidents(unittest.TestCase):
    def test_every_incident_generates(self):
        self.assertEqual(sorted(SITES["incidents"]), [1, 2, 3, 4, 5, 6, 7, 8])
        for i in range(1, 9):
            root = parse(i)
            names = [g.get("name") for g in root.iter("ManeuverGroup")]
            self.assertIn("ego_group", names, i)
            # every ParameterRef is declared
            decl = {p.get("name") for p in root.iter("ParameterDeclaration")}
            refs = {e.get("parameterRef") for e in root.iter() if e.get("parameterRef")}
            self.assertLessEqual(refs, decl, i)

    def test_ego_max_speed_above_cap(self):
        for i in range(1, 9):
            perf = objects(parse(i))["ego"].find("Vehicle/Performance")
            self.assertEqual(float(perf.get("maxSpeed")), gen.EGO_MAX_SPEED)
            self.assertGreater(gen.EGO_MAX_SPEED, 11.11)

    def test_parked_vehicles_are_misc_objects(self):
        obj = objects(parse(2))
        for name, bp in (("i2_van", "vehicle.mercedes.sprinter"), ("i2_parked", "vehicle.tesla.model3")):
            misc = obj[name].find("MiscObject")
            self.assertIsNotNone(misc, name)
            self.assertEqual(misc.get("model3d"), bp)
            dims = misc.find("BoundingBox/Dimensions")
            self.assertAlmostEqual(float(dims.get("length")), gen.BLUEPRINT_BOX[bp][0])
        self.assertIsNotNone(obj["i2_ped"].find("Pedestrian"))
        # no speed action on a misc object
        root = parse(2)
        grp = next(g for g in root.iter("ManeuverGroup") if g.get("name") == "i2_parked")
        self.assertEqual(list(grp.iter("SpeedAction")), [])

    def test_step4b_sites_still_generate(self):
        old = copy.deepcopy(SITES["incidents"])
        del old[2]["van"]
        for k in ("lead_mps", "brake_gap_m"):
            old[1].pop(k)
        old[6].pop("cyclist_mps")
        obj = objects(parse(2, {"incidents": old}))
        self.assertNotIn("i2_van", obj)
        self.assertIn("i2_parked", obj)
        text = gen.template(1, {"incidents": old})
        self.assertIn('value="3.0"', text)  # the step 4b lead speed

    def test_scalars_from_the_sites_file(self):
        s = copy.deepcopy(SITES)
        s["incidents"][1]["lead_mps"] = 7.25
        s["incidents"][4]["oncoming_mps"] = 6.75
        self.assertIn('<AbsoluteTargetSpeed value="7.25" />', gen.template(1, s))
        self.assertIn('<AbsoluteTargetSpeed value="6.75" />', gen.template(4, s))

    def test_incident_7_pulls_out_from_the_shoulder(self):
        root = parse(7)
        S = SITES["incidents"][7]
        self.assertIsNotNone(objects(root)["i7_cut"].find("Vehicle"))
        grp = next(g for g in root.iter("ManeuverGroup") if g.get("name") == "i7")
        events = {e.get("name") for e in grp.iter("Event")}
        self.assertLessEqual({"i7_start", "i7_pull_out", "i7_pull_out_done", "i7_on_lane",
                              "i7_end", "i7_watchdog"}, events)
        # out of the shoulder (y -334.76) into the ego's lane (y -330.47)
        path = next(a for a in grp.iter("Action") if a.get("name") == "i7_pull_out")
        ys = [float(w.get("y")) for w in path.iter("WorldPosition")]
        self.assertEqual(ys, [S["cut_spawn"]["y"], S["cut_in_lane"]["y"], S["cut_settle"]["y"]])
        self.assertGreater(ys[1] - ys[0], 4.0)
        # the watchdog waits while the trajectory runs
        wd = next(e for e in grp.iter("Event") if e.get("name") == "i7_watchdog")
        refs = {c.get("parameterRef") for c in wd.iter("ParameterCondition")}
        self.assertIn("i7_busy", refs)

    def test_incident_8_is_the_red_light_and_a_closing_car(self):
        root = parse(8)
        names = [g.get("name") for g in root.iter("ManeuverGroup")]
        self.assertEqual(names, ["ego_group", "i8", "i8_rear"])
        # i3's red light under its own prefix, same signal
        sig = {a.get("name") for a in root.iter("TrafficSignalStateAction")}
        self.assertEqual(sig, {str(SITES["incidents"][8]["signal"])})
        self.assertIn("i8_red", {e.get("name") for e in root.iter("Event")})
        self.assertNotIn("i3_red", {e.get("name") for e in root.iter("Event")})
        rear = objects(root)["i8_rear"]
        self.assertIsNotNone(rear.find("Vehicle"))
        self.assertEqual(rear.find(".//Controller/Properties").findall("Property"), [])  # not blind

    def test_incident_6_installed(self):
        with tempfile.TemporaryDirectory() as d:
            for i in range(1, 9):
                (Path(d) / gen.FILES[i]).write_text(gen.template(i, SITES))
            self.assertTrue((Path(d) / "town01_i6_cyclist_inside.xosc").exists())
        self.assertTrue((gen.DEFAULT_OUT / "town01_i6_cyclist_inside.xosc").exists())


if __name__ == "__main__":
    unittest.main()
