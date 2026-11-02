"""ego_ready and the launches agree on one DDS transport (roadmap 070, frictions 8 and 14)."""

import importlib.machinery
import importlib.util
import os
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock

_PKG = Path(__file__).resolve().parents[1]
_SCRIPTS = _PKG / "scripts"

# The launches a user starts on their own; each must set the transport for what it starts.
TOP_LAUNCHES = [
    "carla_simulation",
    "isaac_simulation",
    "ego_av",
    "scenario",
    "background_av",
    "vehicle_agent",
]


def _load(name):
    loader = importlib.machinery.SourceFileLoader(name, str(_SCRIPTS / name))
    spec = importlib.util.spec_from_loader(name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class TransportDefault(unittest.TestCase):
    def setUp(self):
        self.ego_ready = _load("ego_ready")

    def test_an_empty_environment_gets_cyclone_and_the_profile(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.ego_ready.use_the_stack_transport()
            self.assertEqual(os.environ["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
            uri = os.environ["CYCLONEDDS_URI"]
            self.assertTrue(uri.startswith("file://"), uri)
            self.assertTrue(Path(uri[len("file://"):]).is_file(), uri)
            self.assertTrue(uri.endswith("cyclonedds-localhost.xml"), uri)

    def test_what_the_environment_names_wins(self):
        env = {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "CYCLONEDDS_URI": "file:///x.xml"}
        with mock.patch.dict(os.environ, env, clear=True):
            self.ego_ready.use_the_stack_transport()
            self.assertEqual(os.environ["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
            self.assertEqual(os.environ["CYCLONEDDS_URI"], "file:///x.xml")


class LaunchesSetTheTransport(unittest.TestCase):
    def test_each_top_launch_sets_rmw_and_profile_before_anything_starts(self):
        for name in TOP_LAUNCHES:
            with self.subTest(launch=name):
                root = ET.parse(_PKG / "launch" / f"{name}.launch.xml").getroot()
                kids = list(root)
                envs = {e.get("name"): i for i, e in enumerate(kids) if e.tag == "set_env"}
                self.assertIn("RMW_IMPLEMENTATION", envs)
                self.assertIn("CYCLONEDDS_URI", envs)
                self.assertIn("rmw_cyclonedds_cpp", kids[envs["RMW_IMPLEMENTATION"]].get("value"))
                first_start = min(
                    (i for i, e in enumerate(kids) if e.tag in ("node", "include", "group", "executable")),
                    default=len(kids),
                )
                self.assertLess(max(envs.values()), first_start)


if __name__ == "__main__":
    unittest.main()
