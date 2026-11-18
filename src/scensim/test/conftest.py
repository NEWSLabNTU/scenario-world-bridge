"""Test support for scensim.

Makes `scensim` importable from the repository root before a colcon install, and serves a fake
Hugging Face Hub (http.server on localhost) with host settings pointing at it.
"""
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import threading
import urllib.parse

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


class FakeHub:
    """Datasets as ``{repo: {"refs": {"main": c, "tags": {name: c}}, "commits": {c: ...}}}``,
    each commit ``{"files": {path: bytes}, "lastModified": iso}``."""

    def __init__(self):
        self.repos = {}
        self.downloads = []
        self._n = 0

    def commit(self, repo, files, tag=None, branch="main", when=None, manifest=False):
        self._n += 1
        sha = hashlib.sha1(f"{repo}{self._n}".encode()).hexdigest()
        files = {k: (v.encode() if isinstance(v, str) else v) for k, v in files.items()}
        if manifest:
            lines = [f"{hashlib.sha256(b).hexdigest()}  {p}" for p, b in sorted(files.items())]
            files["MANIFEST.sha256"] = ("\n".join(lines) + "\n").encode()
        r = self.repos.setdefault(repo, {"refs": {"tags": {}}, "commits": {}})
        r["commits"][sha] = {"files": files,
                             "lastModified": when or f"2026-11-{self._n:02d}T00:00:00.000Z"}
        if branch:
            r["refs"][branch] = sha
        if tag:
            r["refs"]["tags"][tag] = sha
        return sha

    def _rev(self, repo, rev):
        r = self.repos.get(repo)
        if r is None:
            return None
        branches = {k: v for k, v in r["refs"].items() if k != "tags"}
        return (r["refs"]["tags"].get(rev) or branches.get(rev)
                or (rev if rev in r["commits"] else None))

    def handle(self, path):
        """``(status, body bytes)`` for a GET."""
        path = urllib.parse.unquote(path)
        if path.startswith("/api/datasets/"):
            rest = path[len("/api/datasets/"):]
            parts = rest.split("/")
            repo = "/".join(parts[:2])
            r = self.repos.get(repo)
            if r is None:
                return 404, b"{}"
            if parts[2] == "refs":
                tags = [{"name": t, "ref": f"refs/tags/{t}", "targetCommit": c}
                        for t, c in r["refs"]["tags"].items()]
                return 200, json.dumps({"branches": [], "tags": tags}).encode()
            if parts[2] == "revision":
                sha = self._rev(repo, "/".join(parts[3:]))
                if sha is None:
                    return 404, b"{}"
                c = r["commits"][sha]
                body = {"sha": sha, "lastModified": c["lastModified"],
                        "siblings": [{"rfilename": f} for f in sorted(c["files"])]}
                return 200, json.dumps(body).encode()
        if path.startswith("/datasets/"):
            parts = path[len("/datasets/"):].split("/")
            repo, sha, rel = "/".join(parts[:2]), parts[3], "/".join(parts[4:])
            c = self.repos.get(repo, {}).get("commits", {}).get(sha)
            if c is None or rel not in c["files"]:
                return 404, b""
            self.downloads.append((repo, rel))
            return 200, c["files"][rel]
        return 404, b""


@pytest.fixture
def hub():
    fake = FakeHub()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            status, body = fake.handle(self.path)
            self.send_response(status)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    fake.url = f"http://127.0.0.1:{server.server_address[1]}"
    yield fake
    server.shutdown()
    server.server_close()


@pytest.fixture
def host(tmp_path, monkeypatch, hub):
    """Host settings: the cache under tmp, the Hub the fake one."""
    user, system = tmp_path / "user", tmp_path / "system"
    user.mkdir()
    system.mkdir()
    monkeypatch.setenv("SCENSIM_CONFIG_HOME", str(user))
    monkeypatch.setenv("SCENSIM_SYSTEM_CONFIG", str(system))
    cache = tmp_path / "cache"
    (user / "host.yaml").write_text(yaml.safe_dump({"cache": str(cache), "hub_url": hub.url}))
    return {"cache": cache, "hub": hub, "tmp": tmp_path}


TOWN = {
    "Town01/lanelet2_map.osm": "<osm town01/>",
    "Town01/pointcloud_map.pcd": "pcd01",
    "Town01/isaac/world.yaml": "usd: x\n",
    "Town05/lanelet2_map.osm": "<osm town05/>",
    "README.md": "# maps\n",
}


@pytest.fixture
def maps_hub(hub):
    """jerry73204/carla-town-autoware-maps, tagged v1, with a manifest."""
    hub.commit("jerry73204/carla-town-autoware-maps", TOWN, tag="v1", manifest=True)
    hub.commit("jerry73204/carla-0.9.16-isaac-usd",
               {"Town01/Town01_physics.usda": "usda", "vehicles/vehicles.json": "{}",
                "Town02/Town02.usd": "usd2"}, tag="2026-10-23", manifest=True)
    return hub


@pytest.fixture(autouse=True)
def bridge_up(monkeypatch):
    """No bridge in these tests: a keeper takes its world server as up (the tests of the gate
    patch it back)."""
    from scensim import stack
    monkeypatch.setattr(stack, "bridge_serving", lambda rec, timeout=0.5: True)


@pytest.fixture(autouse=True)
def carla_serving(monkeypatch):
    """No CARLA in these tests: a stack whose host settings name no CARLA (no root, the
    default start mode) takes the port as external, as before G39 refused it; a test that
    sets either gets the real decision."""
    from scensim import simulator, stack

    def decide(carla, mode=None, probe_fn=simulator.probe):
        if mode is None and not (carla or {}).get("root") and \
                (carla or {}).get("start", "auto") == "auto":
            mode = "external"
        return simulator.decide(carla, mode, probe_fn)
    monkeypatch.setattr(stack, "CARLA_DECIDE", decide)


@pytest.fixture(autouse=True)
def no_isaac_root(monkeypatch, tmp_path_factory, request):
    """The host's ISAAC_ROOT is not the tests' (scensim.isaac reads it when isaac.root is unset).
    An Isaac project needs a root (077 G76): outside test_isaac, which tests the check itself,
    ISAAC_ROOT is an empty install of the tests' own and the message bindings are not checked."""
    monkeypatch.delenv("ISAAC_ROOT", raising=False)
    if request.module.__name__ == "test_isaac":
        return
    from scensim import isaac
    root = tmp_path_factory.mktemp("isaac-sim")
    for n in isaac.INSTALL:
        (root / n).parent.mkdir(parents=True, exist_ok=True)
        (root / n).write_text("") if n.endswith("activate") else (root / n).mkdir()
    monkeypatch.setenv("ISAAC_ROOT", str(root))
    monkeypatch.setattr(isaac, "binding_findings", lambda host, r, prefixes=None: [])


PKGS = ("autoware_vehicle_msgs", "autoware_control_msgs", "autoware_perception_msgs")


def fake_isaac(tmp, monkeypatch, built=PKGS, autoware_msgs=("_traffic_light_group.py",)):
    """An Isaac install under ``tmp/isaac-sim`` with ``built`` message packages in msgs/, aib's
    build script on AMENT_PREFIX_PATH (default list PKGS), and an Autoware install under
    ``tmp/autoware`` whose packages hold ``autoware_msgs`` message modules. Returns
    ``(isaac root, autoware setup.bash)``."""
    root, aw, ament = tmp / "isaac-sim", tmp / "autoware", tmp / "ament"
    (root / "venv" / "bin").mkdir(parents=True, exist_ok=True)
    (root / "venv" / "bin" / "activate").write_text("")   # what install_isaac.sh makes
    (root / "venv" / "lib" / "python3.12" / "site-packages" / "isaacsim").mkdir(
        parents=True, exist_ok=True)
    script = ament / "share" / "aib" / "scripts" / "build_msgs.sh"
    script.parent.mkdir(parents=True, exist_ok=True)
    script.write_text("#!/usr/bin/env bash\nPKGS=(\"$@\")\n"
                      f"[ ${{#PKGS[@]}} -gt 0 ] || PKGS=({' '.join(PKGS)})\n")
    (aw / "lib").mkdir(parents=True, exist_ok=True)
    (aw / "setup.bash").write_text("")
    (root / "msgs" / "lib").mkdir(parents=True, exist_ok=True)
    for pkg in PKGS:
        src = aw / "local" / "lib" / "python3.10" / "dist-packages" / pkg / "msg"
        src.mkdir(parents=True, exist_ok=True)
        for m in autoware_msgs:
            (src / m).write_text("")
        for ts in ("generator_c", "typesupport_c", "typesupport_fastrtps_c",
                   "typesupport_introspection_c"):
            (aw / "lib" / f"lib{pkg}__rosidl_{ts}.so").write_text("")
    for pkg in built:
        b = root / "msgs" / pkg
        (b / "msg").mkdir(parents=True, exist_ok=True)
        (b / "__init__.py").write_text("")
        (b / f"lib{pkg}__rosidl_generator_py.so").write_text("")
        for m in autoware_msgs:
            (b / "msg" / m).write_text("")
        for ts in ("generator_c", "typesupport_c", "typesupport_fastrtps_c",
                   "typesupport_introspection_c"):
            (root / "msgs" / "lib" / f"lib{pkg}__rosidl_{ts}.so").write_text("")
    # what the stack needs of the ROS install itself (project check asks the index)
    index = ament / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / "rmw_cyclonedds_cpp").write_text("")
    # and the stock cars' Autoware packages an autoware pilot needs (081 step 3)
    for car in ("tesla_model3", "lincoln_mkz_2020"):
        for kind in ("description", "launch"):
            (index / f"{car}_{kind}").write_text("")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(ament))
    return root, aw / "setup.bash"


@pytest.fixture(autouse=True)
def play_launch_ok(monkeypatch, tmp_path_factory):
    """A play_launch at the floor first on PATH, for the keepers the tests start as child
    processes too (it only answers --version; the tests of the floor give their own)."""
    from scensim import tools
    d = tmp_path_factory.mktemp("bin")
    exe = d / "play_launch"
    floor = ".".join(map(str, tools.PLAY_LAUNCH_MIN))
    exe.write_text(f"#!/bin/sh\necho 'play_launch {floor} (fake)'\n")
    exe.chmod(0o755)
    monkeypatch.setenv("PATH", f"{d}:" + __import__("os").environ.get("PATH", ""))


@pytest.fixture(autouse=True)
def gpu_unknown(monkeypatch):
    """No GPU query in tests (the tests of the note give their own state)."""
    from scensim import tools
    monkeypatch.setattr(tools, "gpu_state", lambda index: None)


@pytest.fixture(autouse=True)
def pins_kept(monkeypatch):
    """The test host's checkout is not under test (the tests of the drift give their own)."""
    from scensim import tools
    monkeypatch.setattr(tools, "submodule_drift", lambda workspace: [])


@pytest.fixture(autouse=True)
def agent_modules_present(monkeypatch, tmp_path_factory):
    """CARLA's agents' modules taken as importable (the tests of G38 give their own), in
    this process and in the stacks the tests start: a ``python3`` first on PATH that answers
    the check's ``-c "import shapely"`` and ``networkx`` itself and runs everything else on
    the real one (a host without them failed every stack test, swb 118 2026-11-16)."""
    from scensim import tools
    monkeypatch.setattr(tools, "missing_modules", lambda modules, python="python3": [])
    d = tmp_path_factory.mktemp("agent_python")
    w = d / "python3"
    w.write_text("#!/bin/sh\n"
                 'case "$1 $2" in "-c import shapely"|"-c import networkx") exit 0;; esac\n'
                 f'exec {sys.executable} "$@"\n')
    w.chmod(0o755)
    monkeypatch.setenv("PATH", f"{d}:{os.environ.get('PATH', '')}")


@pytest.fixture(autouse=True)
def no_host_autoware_side(monkeypatch, request):
    """This host's car Autoware packages and Autoware are not the tests': outside
    test_autoware_side, which tests the check itself, an autoware pilot's Autoware side is
    taken as in place and checked against nothing."""
    if request.module.__name__ == "test_autoware_side":
        return
    from scensim import autoware_side
    monkeypatch.setattr(autoware_side, "assess", lambda pilots, **kw: ([], []))
