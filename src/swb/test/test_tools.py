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
