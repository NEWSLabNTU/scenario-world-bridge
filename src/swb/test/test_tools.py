"""swb.tools: the launcher (roadmap 079: `launcher: auto | play_launch | ros2`), and
play_launch's floor when it is the one asked for (roadmap 077, G26)."""
from swb import tools

NEW = lambda: ((0, 35, 2), "/x/play_launch")  # noqa: E731
OLD = lambda: ((0, 18, 3), "/x/play_launch")  # noqa: E731
NONE = lambda: (None, None)  # noqa: E731
GARBLED = lambda: (None, "/x/play_launch")  # noqa: E731


def test_auto_takes_play_launch_at_or_above_the_floor_else_ros2():
    assert tools.launcher("auto", lambda: ((0, 30, 0), "/x/play_launch"))[0] == "play_launch"
    assert tools.launcher("auto", NEW) == ("play_launch", "play_launch 0.35.2")
    assert tools.launcher("auto", OLD) == ("ros2", "play_launch 0.18.3 is older than 0.30.0")
    assert tools.launcher("auto", NONE) == ("ros2", "play_launch not on PATH")
    assert tools.launcher("auto", GARBLED)[0] == "ros2"
    assert tools.launcher(None, NEW)[0] == "play_launch"  # unset: auto
    assert tools.launcher("ros2", NEW) == ("ros2", "set")
    assert tools.launcher("play_launch", NONE)[0] == "play_launch"  # findings refuses it


def test_the_line_project_check_and_seat_show_print():
    assert tools.launcher_line("auto", NEW) == "launcher: auto -> play_launch 0.35.2"
    assert tools.launcher_line("auto", NONE) == "launcher: auto -> ros2 (play_launch not on PATH)"
    assert tools.launcher_line("ros2", NEW) == "launcher: ros2"
    assert tools.launcher_line("play_launch", NEW) == "launcher: play_launch (play_launch 0.35.2)"


def test_without_play_launch_auto_and_ros2_are_fine():
    for setting in ("auto", None, "ros2"):
        for fn in (NEW, OLD, NONE, GARBLED):
            assert tools.findings(setting, fn) == []


def test_an_explicit_play_launch_that_is_missing_or_old_is_refused():
    [f] = tools.findings("play_launch", OLD)
    assert f.refused and "play_launch 0.18.3 (/x/play_launch) is older than 0.30.0" in f.what
    assert "play_launch>=0.30.0" in f.fix and "launcher: auto" in f.fix
    [f] = tools.findings("play_launch", NONE)
    assert f.refused and "not on PATH" in f.what
    [f] = tools.findings("play_launch", GARBLED)
    assert not f.refused and "does not parse" in f.what
    assert tools.findings("play_launch", NEW) == []


def test_an_unknown_launcher_is_refused():
    [f] = tools.findings("roslaunch", NEW)
    assert f.refused and "'roslaunch' is not one of auto, play_launch, ros2" in f.what


def test_cyclone_dds_missing_is_refused_with_its_install():
    assert tools.rmw_findings(lambda: True) == []
    [f] = tools.rmw_findings(lambda: False)
    assert f.refused and "rmw_cyclonedds_cpp is not installed" in f.what
    assert "ros-humble-rmw-cyclonedds-cpp" in f.fix


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


def test_a_centerpoint_pilot_without_autowares_tensorrt_is_warned(tmp_path):
    """G54: CenterPoint's engines rebuilt at every start on a host's other TensorRT."""
    from swb import tools
    (tmp_path / "suites").mkdir()
    settings = {"pilots": {"ego": {"kind": "autoware", "args": {}},
                           "lead": {"kind": "carla_basic"}}}
    assert tools.tensorrt_note(tmp_path, settings, {}) == ""
    (tmp_path / "suites" / "cp.yaml").write_text(
        "pilot_args: {ego: {lidar_detection_model: centerpoint}}\n")
    assert tools.centerpoint_pilots(tmp_path, settings) == ["ego"]
    note = tools.tensorrt_note(tmp_path, settings, {})
    assert "ego run CenterPoint" in note and "tensorrt_lib is not set" in note
    lib = tmp_path / "trt"
    lib.mkdir()
    assert "no libnvinfer.so.10" in tools.tensorrt_note(
        tmp_path, settings, {"autoware": {"tensorrt_lib": str(lib)}})
    (lib / "libnvinfer.so.10").write_text("")
    assert tools.tensorrt_note(tmp_path, settings, {"autoware": {"tensorrt_lib": str(lib)}}) == ""
    settings["pilots"]["ego"]["args"] = {"lidar_detection_model": "centerpoint/centerpoint_tiny"}
    (tmp_path / "suites" / "cp.yaml").unlink()
    assert tools.centerpoint_pilots(tmp_path, settings) == ["ego"]
