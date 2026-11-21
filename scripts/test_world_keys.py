"""The world file's keys: the bridge's and the Isaac world server's lists agree (roadmap 091).

Both parse a map's `isaac/world.yaml`. The bridge refuses a key it does not know
(`WorldFile`, deny_unknown_fields), so a key the world server reads but the bridge does not
know refuses every map that uses it, as `authored` did until roadmap 091 W17's exit run. This
reads both lists from the sources, with no build: the struct's fields and serde aliases, and
scensim_isaac_world's `names.WORLD_KEYS`.
"""
import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "src/scenario_world_bridge/src/backend/remote/mod.rs"
NAMES = ROOT / "src/scensim_isaac_world/scensim_isaac_world/names.py"


def bridge_keys():
    text = BRIDGE.read_text()
    body = re.search(r"pub struct WorldFile \{(.*?)\n\}", text, re.S)
    assert body, f"{BRIDGE}: no `pub struct WorldFile`"
    keys = set(re.findall(r"^\s*pub (\w+):", body.group(1), re.M))
    keys |= set(re.findall(r'#\[serde\(alias = "(\w+)"\)\]', body.group(1)))
    return keys


def server_keys():
    tree = ast.parse(NAMES.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, "id", None) == "WORLD_KEYS" for t in node.targets):
            return set(ast.literal_eval(node.value))
    raise AssertionError(f"{NAMES}: no WORLD_KEYS")


def test_the_bridge_and_the_isaac_world_server_take_the_same_keys():
    b, s = bridge_keys(), server_keys()
    assert b == s, (f"world.yaml keys differ: the bridge only {sorted(b - s)}, the Isaac world "
                    f"server only {sorted(s - b)} ({BRIDGE.relative_to(ROOT)}, "
                    f"{NAMES.relative_to(ROOT)})")


def test_the_bridge_list_is_read():
    assert {"world", "usd", "ground_paths", "map_origin"} <= bridge_keys()
