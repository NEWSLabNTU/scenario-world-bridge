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


def test_a_project_of_swb_yaml_is_copied_with_the_renames_said(tree, capsys):
    """Issue 061: a 077-era project (swb.yaml) is a source: its file, its maps variable, its
    swb.* properties and the installed catalog are renamed, each said; .swb/ is the host's."""
    old = tree["study"].parent / "old"
    (old / "scenarios").mkdir(parents=True)
    (old / "suites").mkdir()
    (old / "swb.yaml").write_text(TB)
    (old / ".swb").mkdir()
    (old / ".swb" / "local.yaml").write_text("seat: 3\n")
    x = XOSC.replace("$(env SCENSIM_MAPS)", "$(env SWB_MAPS)").replace(
        project.VEHICLES_CATALOG, project.INSTALLED_CATALOG)
    x = x.replace('<Property name="maxJerk"',
                  '<Property name="swb.spec_hash" value="1" /><Property name="maxJerk"', 1)
    assert "swb.spec_hash" in x
    (old / "scenarios" / "a.xosc").write_text(x)
    (old / "suites" / "main.yaml").write_text(
        "scenarios:\n  - scenarios/a.xosc\n  - ../../src/shared/scenarios/x.xosc\n")
    assert testbench_cli.main(["new", "migrated", "--from", str(old)]) == 0
    d = tree["out"] / "migrated"
    assert (d / "testbench.yaml").is_file() and not (d / "swb.yaml").exists()
    assert not (d / ".swb").exists()
    text = (d / "scenarios" / "a.xosc").read_text()
    assert "SWB_MAPS" not in text and "$(env SCENSIM_MAPS)" in text
    assert 'name="scensim.spec_hash"' in text and "swb." not in text
    assert project.INSTALLED_CATALOG not in text and project.VEHICLES_CATALOG in text
    assert "pkg://shared/scenarios/x.xosc" in (d / "suites" / "main.yaml").read_text()   # 055
    out = capsys.readouterr().out
    assert "note: swb.yaml renamed testbench.yaml" in out
    assert "note: scenarios/a.xosc: SWB_MAPS renamed: $(env SCENSIM_MAPS)" in out
    assert "note: scenarios/a.xosc: the swb.* scenario properties renamed" in out
    assert "note: scenarios/a.xosc: the installed VehicleCatalog renamed" in out
    findings = [f for f in project.check_project(d)
                if "renamed" in f.what or "swb" in f.what or "VehicleCatalog" in f.what]
    assert findings == []
    # the source is untouched
    assert (old / "swb.yaml").is_file() and "SWB_MAPS" in (old / "scenarios" / "a.xosc").read_text()
