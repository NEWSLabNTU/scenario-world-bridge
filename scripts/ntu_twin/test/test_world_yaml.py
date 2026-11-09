"""world.yaml: what aib's runner and swb's world server read. Its `usd:` is relative to the map
dir (the one holding isaac/), so a world copied or mounted elsewhere still loads."""
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import world_usd  # noqa: E402


def test_world_yaml_names_its_usd_relative_to_the_map_dir():
    doc = yaml.safe_load(world_usd.world_yaml_text({"name": "coss"}, (0.0, 0.0, 0.0)))
    assert doc["usd"] == "isaac/world.usdc"
    assert doc["map_origin"] == [0.0, 0.0, 0.0]
    assert doc["ground_paths"] == ["/World/env/Tiles"]


def test_world_yaml_says_which_site_it_is():
    text = world_usd.world_yaml_text({"name": "coss"}, (0.0, 0.0, 0.0))
    assert "site coss" in text and "NTU" not in text
