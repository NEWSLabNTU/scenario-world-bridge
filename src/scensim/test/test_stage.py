"""An authored Isaac map's stage: the map check's wiring and the run's record (roadmap 091
W15, W16). The check itself is scensim_isaac_world's (``stage_record``), tested there."""
import json
import subprocess
from pathlib import Path

from scensim import stage


def _map(project, name, authored=True):
    d = Path(project) / "assets" / "maps" / name / "isaac"
    d.mkdir(parents=True)
    (d / "world.yaml").write_text("usd: field.usda\n" + ("authored: true\n" if authored else ""))
    return d / "world.yaml"


def _ws(tmp_path):
    ws = tmp_path / "ws"
    (ws / "src" / "scensim_isaac_world").mkdir(parents=True)
    (ws / "src" / "scensim_world_protocol" / "scensim_world_server").mkdir(parents=True)
    return ws


def test_authored_maps_only(tmp_path):
    _map(tmp_path, "lab")
    _map(tmp_path, "Town01", authored=False)
    assert [m for m, _ in stage.authored_maps(tmp_path, {})] == ["lab"]
    assert stage.authored_maps(tmp_path, {"maps": ["Town01"]}) == []


def test_no_authored_map_no_check_no_note(tmp_path, monkeypatch):
    _map(tmp_path, "Town01", authored=False)
    monkeypatch.setattr(stage, "pxr_python", lambda *a: (_ for _ in ()).throw(AssertionError))
    assert stage.map_findings(tmp_path, {}, {}, ws=_ws(tmp_path)) == []
    assert stage.notes(tmp_path) == []


def test_skipped_says_why(tmp_path, monkeypatch):
    _map(tmp_path, "lab")
    monkeypatch.setattr(stage, "pxr_python", lambda host, env: (None, ["/isaac/python", "python3"]))
    assert stage.map_findings(tmp_path, {}, {}, ws=_ws(tmp_path)) == []
    [note] = stage.notes(tmp_path)
    assert "lab" in note and "skipped" in note and "no Python with pxr" in note
    assert "/isaac/python, python3" in note


def test_problems_refuse_naming_the_map(tmp_path, monkeypatch):
    wy = _map(tmp_path, "lab")
    monkeypatch.setattr(stage, "pxr_python", lambda host, env: ("py", ["py"]))
    seen = {}

    def run(cmd, **kw):
        seen["cmd"], seen["env"] = cmd, kw["env"]
        out = {"map": "lab", "authored": True, "stage": "x",
               "problems": ["x: metersPerUnit is 0.01, not 1 (the field is in metres)"]}
        return subprocess.CompletedProcess(cmd, 0, json.dumps(out) + "\n", "")

    monkeypatch.setattr(stage.subprocess, "run", run)
    [f] = stage.map_findings(tmp_path, {}, {}, ws=_ws(tmp_path))
    assert f.refused and f.file == "assets/maps/lab/isaac/world.yaml"
    assert "metersPerUnit is 0.01" in f.what
    assert seen["cmd"][:4] == ["py", "-m", "scensim_isaac_world.stage_record", "map"]
    assert seen["cmd"][4] == str(wy) and seen["cmd"][-1] == str(tmp_path / "state")
    assert "scensim_world_server" in seen["env"]["PYTHONPATH"]
    assert stage.notes(tmp_path) == []


def test_a_crashed_check_refuses(tmp_path, monkeypatch):
    _map(tmp_path, "lab")
    monkeypatch.setattr(stage, "pxr_python", lambda host, env: ("py", ["py"]))
    monkeypatch.setattr(stage.subprocess, "run", lambda cmd, **kw: subprocess.CompletedProcess(
        cmd, 1, "", "Traceback ...\nImportError: no pxr\n"))
    [f] = stage.map_findings(tmp_path, {}, {}, ws=_ws(tmp_path))
    assert f.refused and "ImportError: no pxr" in f.what


def test_provenance(tmp_path):
    _map(tmp_path, "lab")
    _map(tmp_path, "Town01", authored=False)
    rec = {"simulator": "isaac", "map": "lab"}
    assert "not written" in stage.provenance(tmp_path, rec)["missing"]
    d = tmp_path / "state" / "lab"
    d.mkdir(parents=True)
    (d / "stage.json").write_text(json.dumps({"root": "field.usda", "sha256": "ab"}))
    assert stage.provenance(tmp_path, rec) == {"root": "field.usda", "sha256": "ab"}
    assert stage.provenance(tmp_path, {"simulator": "isaac", "map": "Town01"}) is None
    assert stage.provenance(tmp_path, {"simulator": "carla", "map": "lab"}) is None
