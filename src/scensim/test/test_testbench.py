"""The testbench check at build (roadmap 081 step 2): the package's own findings and 081's two
reference rules, on a testbench tree like colcon-scensim installs (share/<pkg>/)."""
import json
from pathlib import Path

from scensim import testbench

TEMPLATE = Path(__file__).resolve().parents[1] / "scensim" / "templates" / "carla-first-drive"


def make(tmp_path, deps=(), suite=None, extra="", name="tb"):
    tb = tmp_path / name
    (tb / "scenarios").mkdir(parents=True)
    (tb / "scenarios" / "drive.xosc").write_text(
        (TEMPLATE / "scenarios" / "drive_to_goal.xosc").read_text())
    (tb / "testbench.yaml").write_text(
        (TEMPLATE / "swb.yaml").read_text().replace("@PROJECT@", name) + extra)
    depends = "".join(f"<exec_depend>{d}</exec_depend>" for d in deps)
    (tb / "package.xml").write_text(
        f'<?xml version="1.0"?><package format="3"><name>{name}</name><version>0.0.1</version>'
        f"<description>t</description><maintainer email='m@e.com'>m</maintainer>"
        f"<license>Apache-2.0</license>{depends}<export><build_type>scensim_testbench"
        f"</build_type></export></package>")
    if suite is not None:
        (tb / "suites").mkdir()
        (tb / "suites" / "s.yaml").write_text("scenarios:\n" + "".join(f"  - {s}\n" for s in suite))
    return tb


def findings(tb):
    return [str(f) for f in testbench.build_findings(tb, tb / "package.xml")]


def test_a_template_testbench_has_no_findings_before_any_asset_is_fetched(tmp_path):
    # its map is not fetched on this host: a host finding, not the package's
    assert findings(make(tmp_path)) == []


def test_pkg_references_name_declared_packages_or_the_testbench_itself(tmp_path):
    tb = make(tmp_path, deps=["shared"], suite=["pkg://shared/scenarios/a.xosc",
                                                 "pkg://tb/scenarios/drive.xosc",
                                                 "pkg://other/x.xosc"])
    got = findings(tb)
    assert got == ["problem: suites/s.yaml: pkg://other/x.xosc: package other is not declared "
                   "(fix: add <exec_depend>other</exec_depend> to package.xml)"]


def test_a_path_out_of_the_testbench_is_refused_one_inside_is_not(tmp_path):
    # the second is relative to suites/, as scensim takes it when only that one exists
    tb = make(tmp_path, suite=["scenarios/../scenarios/drive.xosc",
                               "../scenarios/drive.xosc",
                               "../elsewhere/x.xosc"])
    got = findings(tb)
    assert len(got) == 1 and "suites/s.yaml: ../elsewhere/x.xosc: leaves the testbench" in got[0]
    assert "pkg://<package>/<path>" in got[0]


def test_schema_findings_name_testbench_yaml(tmp_path):
    got = findings(make(tmp_path, extra="colour: red\n"))
    assert got and got[0].startswith("problem: testbench.yaml: unknown key colour")


def test_a_swb_yaml_project_is_refused_naming_its_successor(tmp_path):
    tb = make(tmp_path)
    (tb / "testbench.yaml").rename(tb / "swb.yaml")
    got = testbench.build_findings(tb, tb / "package.xml")
    assert len(got) == 1 and got[0].refused
    assert "a testbench's file is testbench.yaml, not swb.yaml" in got[0].what
    assert got[0].fix == "git mv swb.yaml testbench.yaml"


def test_notes_say_no_lock_and_that_assets_are_not_checked(tmp_path):
    tb = make(tmp_path)
    notes = testbench.build_notes(tb)
    assert any("no scensim.lock" in n for n in notes)
    assert any("assets are not checked at build" in n for n in notes)
    (tb / "scensim.lock").write_text("datasets: {}\n")
    assert not any("no scensim.lock" in n for n in testbench.build_notes(tb))


def test_built_over_is_json_with_the_three_keys():
    assert set(json.loads(json.dumps(testbench.built_over()))) == {"scensim", "prefix", "build"}
