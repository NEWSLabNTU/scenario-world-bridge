"""swb.tools (roadmap 077, G26): play_launch's floor is checked before a stack starts."""
from swb import tools


def test_at_or_above_the_floor_is_fine():
    assert tools.findings(lambda: ((0, 30, 0), "/x/play_launch")) == []
    assert tools.findings(lambda: ((0, 35, 2), "/x/play_launch")) == []


def test_an_older_play_launch_is_refused_naming_both_versions():
    [f] = tools.findings(lambda: ((0, 18, 3), "/x/play_launch"))
    assert f.refused and "play_launch 0.18.3 (/x/play_launch) is older than 0.30.0" in f.what
    assert "play_launch>=0.30.0" in f.fix


def test_none_on_path_is_refused():
    [f] = tools.findings(lambda: (None, None))
    assert f.refused and "not on PATH" in f.what


def test_an_unparsed_version_is_a_problem_not_a_refusal():
    [f] = tools.findings(lambda: (None, "/x/play_launch"))
    assert not f.refused and "does not parse" in f.what


def test_the_version_line_parses():
    out = "play_launch 0.34.0 (ROS humble, v0.34.0, rlm v0.3.18)\n"
    assert tools.parse_version(out) == (0, 34, 0)
    assert tools.parse_version("something else") is None



def test_gpu_note_warns_for_too_little_free_memory_or_a_busy_card():
    """roadmap 077, G27: 241's gpu 0 had another user's job (7.4 GB, 85 %); nothing said so."""
    assert tools.gpu_note(0, "carla", (2000, 32607, 10)) == ""
    n = tools.gpu_note(0, "isaac", (28207, 32607, 91))
    assert n.startswith("GPU 0 (27.5 of 31.8 GB in use)")
    assert "4.3 GB free, isaac needs about 9.8" in n and "91 % busy" in n
    assert "gpu:" in n
    assert tools.gpu_note(1, "carla", (7400, 24000, 85)) .endswith(
        "another card") and "busy" in tools.gpu_note(1, "carla", (7400, 24000, 85))
    assert tools.gpu_note(0, "carla", None) == ""



def _git(*a, cwd):
    import subprocess
    subprocess.run(["git", "-c", "user.email=t@t", "-c", "user.name=t",
                    "-c", "protocol.file.allow=always", *a], cwd=cwd, check=True,
                   capture_output=True)


def test_submodule_drift_finds_a_nested_checkout_off_its_pin(tmp_path):
    """roadmap 077, G31: a component's deps/ left on an older commit by a non-recursive
    update is named with both commits."""
    import importlib
    real = importlib.reload(importlib.import_module("swb.tools"))   # undo the autouse stub
    lib = tmp_path / "lib"
    lib.mkdir()
    _git("init", "-q", cwd=lib)
    (lib / "f").write_text("1")
    _git("add", "f", cwd=lib)
    _git("commit", "-qm", "one", cwd=lib)
    comp = tmp_path / "comp"
    comp.mkdir()
    _git("init", "-q", cwd=comp)
    _git("submodule", "add", "-q", str(lib), "deps/lib", cwd=comp)
    _git("commit", "-qm", "pin lib", cwd=comp)
    ws = tmp_path / "ws"
    ws.mkdir()
    _git("init", "-q", cwd=ws)
    _git("submodule", "add", "-q", str(comp), "src/comp", cwd=ws)
    _git("commit", "-qm", "pin comp", cwd=ws)
    _git("submodule", "update", "--init", "--recursive", "-q", cwd=ws)
    assert real.submodule_drift(ws) == []
    # the library moves on, and the component pins the new commit; the workspace takes the
    # component's new pin without --recursive: its deps/lib stays on the old commit
    (lib / "f").write_text("2")
    _git("commit", "-qam", "two", cwd=lib)
    _git("-C", "deps/lib", "pull", "-q", "origin", "HEAD", cwd=comp)
    _git("commit", "-qam", "bump lib", cwd=comp)
    _git("-C", "src/comp", "pull", "-q", "origin", "HEAD", cwd=ws)
    _git("commit", "-qam", "bump comp", cwd=ws)
    drift = real.submodule_drift(ws)
    assert [d[0] for d in drift] == ["src/comp/deps/lib"]
    path, have, pinned = drift[0]
    assert have and pinned and have != pinned
    note = real.drift_note(drift)
    assert f"src/comp/deps/lib at {have[:7]} (pinned {pinned[:7]})" in note
    assert "--recursive" in note
    assert real.drift_note([]) == ""



def test_a_carla_basic_pilot_without_shapely_or_networkx_is_refused():
    """roadmap 077, G38: 241's carla_basic pilots crash-looped on ModuleNotFoundError."""
    s = {"pilots": {"lead": {"kind": "carla_basic"}, "ego": {"kind": "autoware"}}}
    [f] = tools.agent_findings(s, missing_fn=lambda mods: ["shapely", "networkx"])
    assert f.refused and "cannot import shapely, networkx" in f.what and "lead" in f.what
    assert "python3-shapely" in f.fix and "pip install --user shapely networkx" in f.fix
    assert tools.agent_findings(s, missing_fn=lambda mods: []) == []
    assert tools.agent_findings({"pilots": {"ego": {"kind": "simple"}}},
                                missing_fn=lambda mods: ["shapely"]) == []


def test_missing_modules_imports_each(tmp_path, monkeypatch):
    import sys
    monkeypatch.undo()   # the autouse stub of missing_modules
    assert tools.missing_modules(["json", "surely_not_a_module_x"], sys.executable) == \
        ["surely_not_a_module_x"]
