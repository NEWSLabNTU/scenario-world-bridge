"""Issue 055: `scensim testbench new --from` makes a testbench that checks clean, or says what
is left: paths that left the source as pkg://, the default suite, the maintainer and license."""
from pathlib import Path

import pytest

from scensim import config, project, testbench_cli
from scensim import testbench as tbmod

XOSC = (project.TEMPLATES / "carla-first-drive" / "scenarios" / "drive_to_goal.xosc").read_text()
TB = "simulator: carla\nassets: []\npilots:\n  ego: {kind: autoware, vehicle: tesla_model3}\n"


@pytest.fixture
def tree(tmp_path, monkeypatch):
    """A checkout: a source package `shared` with scenarios, installed (a symlink install), and
    a study project whose suites reach it with ../ and reach a directory in no package."""
    pytest.importorskip("ament_index_python")
    co = tmp_path / "co"
    shared = co / "src" / "shared"
    (shared / "scenarios").mkdir(parents=True)
    (shared / "scenarios" / "x.xosc").write_text(XOSC)
    (shared / "package.xml").write_text("<package format='3'><name>shared</name></package>")
    loose = co / "loose"
    loose.mkdir()
    (loose / "y.xosc").write_text(XOSC)
    prefix = tmp_path / "install"
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True)
    (index / "shared").write_text("")
    (prefix / "share" / "shared" / "scenarios").mkdir(parents=True)
    (prefix / "share" / "shared" / "scenarios" / "x.xosc").symlink_to(shared / "scenarios" / "x.xosc")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    study = co / "studies" / "basic"
    (study / "suites").mkdir(parents=True)
    (study / "testbench.yaml").write_text(TB)
    (study / "suites" / "main.yaml").write_text(
        "scenarios:\n  - ../../src/shared/scenarios/x.xosc\n  - ../../src/shared/scenarios/\n")
    (study / "suites" / "aux.yaml").write_text("scenarios:\n  - ../../loose/y.xosc\n")
    out = tmp_path / "ws"
    out.mkdir()
    monkeypatch.chdir(out)
    monkeypatch.setattr(testbench_cli, "_git", lambda key: "")
    return {"study": study, "out": out}


def test_paths_that_leave_the_source_become_pkg_and_are_declared(tree, capsys):
    assert testbench_cli.main(["new", "basic", "--from", str(tree["study"]),
                               "--default", "main"]) == 0
    d = tree["out"] / "basic"
    assert (d / "suites" / "main.yaml").read_text() == (
        "scenarios:\n  - pkg://shared/scenarios/x.xosc\n  - pkg://shared/scenarios/\n")
    assert "<exec_depend>shared</exec_depend>" in (d / "package.xml").read_text()
    # in no package: kept, and named in the output
    assert (d / "suites" / "aux.yaml").read_text() == "scenarios:\n  - ../../loose/y.xosc\n"
    out = capsys.readouterr().out
    assert "note: suites/aux.yaml: ../../loose/y.xosc leaves the testbench" in out
    assert config.read_yaml(d / "testbench.yaml")["default"] == "main"
    refs = [str(f) for f in tbmod.reference_findings(d, d / "package.xml")]
    assert refs == [r for r in refs if "loose" in r] and len(refs) == 1


def test_the_default_suite_is_said_when_it_is_a_guess(tree, capsys):
    assert testbench_cli.main(["new", "basic", "--from", str(tree["study"])]) == 0
    out = capsys.readouterr().out
    assert "note: default: aux, the first of 2 suites (aux, main): name another with --default" in out
    assert testbench_cli.main(["new", "other", "--from", str(tree["study"]),
                               "--default", "nope"]) == 3
    assert not (tree["out"] / "other").exists()          # refused: nothing left behind


def test_the_maintainer_and_license_are_options_and_placeholders_are_said(tree, capsys):
    assert testbench_cli.main(["new", "basic", "--from", str(tree["study"]), "--default", "main"]) == 0
    assert "note: package.xml: the maintainer and license are placeholders" in capsys.readouterr().out
    assert testbench_cli.main(["new", "basic2", "--from", str(tree["study"]), "--default", "main",
                               "--maintainer", "Ada Lovelace <ada@example.org>",
                               "--license", "Apache-2.0"]) == 0
    xml = (tree["out"] / "basic2" / "package.xml").read_text()
    assert '<maintainer email="ada@example.org">Ada Lovelace</maintainer>' in xml
    assert "<license>Apache-2.0</license>" in xml
    assert "placeholder" not in capsys.readouterr().out
    assert testbench_cli.main(["new", "basic3", "--maintainer", "nobody"]) == 3


def test_an_unfetched_ground_file_is_the_hosts(tmp_path):
    d = tmp_path / "tb"
    (d / "scenarios").mkdir(parents=True)
    (d / "scenarios" / "a.xosc").write_text(XOSC)
    (d / "testbench.yaml").write_text(
        "simulator: isaac\nassets: []\npilots:\n  ego: {kind: none}\n"
        "ground: {Town01: {road_detail: assets/stages/road_detail.yaml}}\n")
    ground = [f for f in project.check_project(d) if "ground" in f.what]
    assert [(f.what, f.host) for f in ground] == [
        ("ground.Town01.road_detail: assets/stages/road_detail.yaml is not fetched", True)]
