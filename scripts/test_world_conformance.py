"""T1, world-server conformance (roadmap 077, item 56): `scensim_world_check conformance` against
a running world server, passing on its exit code and its output.

Opt-in, since the check takes the server over and moves bodies in its world (never during a
scenario): it runs only when SCENSIM_WORLD_CHECK names the simulator, and skips otherwise, so
`just test` stays simulator-free.

    SCENSIM_WORLD_CHECK=carla SCENSIM_WORLD_MAP=<map dir> python3 -m pytest -q scripts/test_world_conformance.py
    SCENSIM_WORLD_CHECK=isaac python3 -m pytest -q scripts/test_world_conformance.py

SCENSIM_WORLD_ENDPOINT is the server's endpoint (default tcp://127.0.0.1:<port>, the port
WORLD_PORT, else 5591, for carla; ISAAC_PORT, else 5590, for isaac). SCENSIM_WORLD_MAP is the map directory to load (a
CARLA town needs one: its Autoware map, e.g. a project's assets/maps/Town01; with its
traffic_lights.yaml the signals capability is checked too); Isaac without it loads its
default world. No bridge may own the server: stop the stack's bridge, or start the world
server on its own (`ros2 launch scensim_carla_world scensim_carla_world.launch.xml port:=...
carla_port:=...`, or the Isaac world server).
"""
import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PORT = {"carla": ("WORLD_PORT", "5591"), "isaac": ("ISAAC_PORT", "5590")}
SUMMARY = re.compile(r"^(\d+) passed, (\d+) failed$", re.M)


def _tool():
    built = ROOT / "install/scensim_world_check/lib/scensim_world_check/scensim_world_check"
    return str(built) if built.exists() else shutil.which("scensim_world_check")


def _target():
    sim = os.environ.get("SCENSIM_WORLD_CHECK", "")
    if not sim:
        pytest.skip("SCENSIM_WORLD_CHECK unset (carla or isaac): needs a running world server")
    if sim not in DEFAULT_PORT:
        pytest.fail(f"SCENSIM_WORLD_CHECK={sim!r}: carla or isaac")
    var, port = DEFAULT_PORT[sim]
    endpoint = os.environ.get("SCENSIM_WORLD_ENDPOINT",
                              f"tcp://127.0.0.1:{os.environ.get(var, port)}")
    return sim, endpoint, os.environ.get("SCENSIM_WORLD_MAP")


def test_the_world_server_passes_conformance():
    sim, endpoint, world_map = _target()
    tool = _tool()
    assert tool, "scensim_world_check not built (colcon build of scensim_world_check)"
    if sim == "carla":
        assert world_map, "SCENSIM_WORLD_MAP: a CARLA town needs its map directory"
    # answering at all first, so a server that is not up reads as that, not as a failed check
    hello = subprocess.run([tool, "hello", endpoint, "--wait", "60"], capture_output=True,
                           text=True, timeout=90)
    assert hello.returncode == 0, f"no world server at {endpoint}: {hello.stdout}{hello.stderr}"
    cmd = [tool, "conformance", endpoint] + (["--map", world_map] if world_map else [])
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    out = r.stdout + r.stderr
    print(out)
    fails = [line for line in out.splitlines() if line.lstrip().startswith("FAIL")]
    summary = SUMMARY.search(out)
    assert r.returncode == 0 and not fails, f"exit {r.returncode}; " + "\n".join(fails or [out[-2000:]])
    assert summary and int(summary.group(1)) > 0 and summary.group(2) == "0", out[-2000:]
