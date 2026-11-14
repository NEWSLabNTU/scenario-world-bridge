"""Test support: `swb` importable from the repository root (before a colcon install), and a fake
Hugging Face Hub (http.server on localhost) with host settings pointing at it."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import hashlib
import json
import threading
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
import yaml


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
    monkeypatch.setenv("SWB_CONFIG_HOME", str(user))
    monkeypatch.setenv("SWB_SYSTEM_CONFIG", str(system))
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
