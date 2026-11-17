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
    assert got[0].what == "a testbench's file is testbench.yaml, not swb.yaml"
    assert got[0].fix == "git mv swb.yaml testbench.yaml"


def test_the_only_note_is_a_missing_lock(tmp_path):
    tb = make(tmp_path)
    assert testbench.build_notes(tb) == [
        "no scensim.lock yet: run `scensim assets fetch tb` and commit src/tb/scensim.lock"]
    (tb / "scensim.lock").write_text("datasets: {}\n")
    assert testbench.build_notes(tb) == []


def test_an_absolute_path_leaves_the_testbench_a_ros_topic_is_no_path(tmp_path):
    tb = make(tmp_path, suite=["/home/me/x.xosc"],
              extra="vehicles: [/home/me/car.yaml]\n"
                    "logging: {topics: [/vehicle/status/velocity_status]}\n")
    got = findings(tb)
    assert "problem: suites/s.yaml: /home/me/x.xosc: leaves the testbench" in "\n".join(got)
    assert "problem: testbench.yaml: /home/me/car.yaml: leaves the testbench" in "\n".join(got)
    assert not any("velocity_status" in g for g in got), got


def test_a_symlink_installed_share_keeps_its_relative_paths_inside(tmp_path):
    """243's review: under --symlink-install every installed file is a link into the source;
    a suite's ../scenarios/... is still inside the package."""
    src = make(tmp_path, suite=["../scenarios/drive.xosc", "scenarios/drive.xosc"])
    share = tmp_path / "install" / "share" / "tb"
    for f in sorted(src.rglob("*")):
        if f.is_file():
            (share / f.relative_to(src)).parent.mkdir(parents=True, exist_ok=True)
            (share / f.relative_to(src)).symlink_to(f)
    assert findings(share) == []


def test_the_host_findings_are_project_pys_own_wording():
    """HOST_FINDINGS names project.py's messages exactly: a change there fails here."""
    src = (Path(testbench.__file__).with_name("project.py")).read_text()
    assert 'f"map {name} is not fetched (assets/maps/{name})"' in src
    host = testbench.Finding("scenarios/a.xosc", "map Town01 is not fetched (assets/maps/Town01)")
    pkg = testbench.Finding("testbench.yaml", "assets: Town01 is not fetched yet, so ...")
    assert testbench.split_findings([host, pkg]) == ([pkg], [host])


def test_built_over_is_json_with_the_three_keys():
    assert set(json.loads(json.dumps(testbench.built_over()))) == {"scensim", "prefix", "build"}


def test_a_world_server_from_a_package_is_a_testbench_key(tmp_path):
    """081 step 5: world.server is a package's executable; the stock server's keys beside it
    are refused."""
    assert findings(make(tmp_path, extra="world: {server: [my_world, my_server]}\n")) == []
    got = findings(make(tmp_path, name="tb2",
                        extra="world: {server: [my_world, my_server, --port, '1'], "
                              "config_file: c.yaml}\n"))
    assert got == [
        "problem: testbench.yaml: world.server: --port is scensim's to give (the seat's world "
        "port): drop it",
        "problem: testbench.yaml: world.config_file: is the stock CARLA server's; with "
        "world.server, your server takes its own settings as its arguments"]
