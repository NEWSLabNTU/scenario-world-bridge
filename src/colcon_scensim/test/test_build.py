"""colcon-scensim end to end: a real ``colcon build`` of the fixture workspace
(test/fixtures/ws: two testbenches, one bad, one still a swb.yaml project, a scenarios
package and a support package), with the plugin found only through PYTHONPATH, as scensim's
setup.bash provides it (roadmap 081, 133's decision 7).

Needs ROS 2 (/opt/ros/humble), colcon with colcon-ros, and pip; skipped without them.
"""
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
PLUGIN = HERE.parent
SCENSIM_SRC = PLUGIN.parent / "scensim"          # the scensim package's source, for its check
FIXTURE = HERE / "fixtures" / "ws"
ROS = Path("/opt/ros/humble/setup.bash")

pytestmark = pytest.mark.skipif(
    not ROS.is_file() or shutil.which("colcon") is None,
    reason="needs ROS 2 Humble and colcon")


@pytest.fixture(scope="session")
def plugin_dir(tmp_path_factory):
    """The plugin installed as a distribution (its entry points) in a directory of its own."""
    d = tmp_path_factory.mktemp("plugin")
    subprocess.run([sys.executable, "-m", "pip", "install", "-q", "--no-deps",
                    "--disable-pip-version-check", "--target", str(d), str(PLUGIN)],
                   check=True)
    return d


@pytest.fixture
def ws(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / "ws")
    return tmp_path / "ws"


def colcon(ws, plugin_dir, *args, plugin=True):
    path = ([str(plugin_dir)] if plugin else []) + [str(SCENSIM_SRC)]
    script = (f"source {ROS} && export PYTHONPATH={':'.join(path)}:$PYTHONPATH && "
              f"colcon build {' '.join(args)}")
    return subprocess.run(["bash", "-c", script], cwd=ws, capture_output=True, text=True,
                          env={k: v for k, v in os.environ.items() if k != "PYTHONPATH"})


def test_a_testbench_installs_indexes_and_says_what_it_was_built_over(ws, plugin_dir):
    r = colcon(ws, plugin_dir, "--packages-select", "tb_good")
    assert r.returncode == 0, r.stdout + r.stderr
    share = ws / "install" / "tb_good" / "share" / "tb_good"
    for f in ("package.xml", "testbench.yaml", "scenarios/drive_to_goal.xosc",
              "suites/smoke.yaml"):
        assert (share / f).is_file() and not (share / f).is_symlink(), f
    for host_dir in ("assets", "runs"):   # the host writes these at run time
        assert not (share / host_dir).exists(), host_dir
    index = ws / "install" / "tb_good" / "share" / "ament_index" / "resource_index"
    assert (index / "packages" / "tb_good").is_file()
    over = json.loads((index / "scensim_testbenches" / "tb_good").read_text())
    assert set(over) == {"scensim", "prefix", "build"}
    assert (share / "hook" / "ament_prefix_path.dsv").is_file()
    assert (share / "package.dsv").is_file()                 # colcon's local_setup pieces
    out = r.stdout + r.stderr
    assert "no scensim.lock" in out and "assets are not checked at build" in out


def test_symlink_install_links_each_file_so_an_edit_needs_no_rebuild(ws, plugin_dir):
    r = colcon(ws, plugin_dir, "--symlink-install", "--packages-select", "tb_good")
    assert r.returncode == 0, r.stdout + r.stderr
    installed = ws / "install" / "tb_good" / "share" / "tb_good" / "scenarios" / "drive_to_goal.xosc"
    source = ws / "src" / "tb_good" / "scenarios" / "drive_to_goal.xosc"
    assert installed.is_symlink() and installed.resolve() == source.resolve()
    source.write_text(source.read_text() + "<!-- edited -->\n")
    assert installed.read_text().endswith("<!-- edited -->\n")


def test_a_bad_testbench_fails_its_build_with_the_checks_messages(ws, plugin_dir):
    r = colcon(ws, plugin_dir, "--packages-select", "tb_bad")
    out = r.stdout + r.stderr
    assert r.returncode != 0
    assert "1 package failed: tb_bad" in out
    assert "testbench.yaml: unknown key colour" in out
    assert ("suites/reach.yaml: pkg://shared_scenarios/scenarios/shared_drive.xosc: package "
            "shared_scenarios is not declared (fix: add <exec_depend>shared_scenarios"
            "</exec_depend> to package.xml)") in out
    assert "suites/reach.yaml: ../../tb_good/scenarios/drive_to_goal.xosc: leaves the testbench" in out
    assert "the testbench check found 3 problem(s)" in out


def test_a_swb_yaml_project_is_refused_naming_testbench_yaml(ws, plugin_dir):
    r = colcon(ws, plugin_dir, "--packages-select", "tb_swb")
    out = r.stdout + r.stderr
    assert r.returncode != 0
    assert "a testbench's file is testbench.yaml, not swb.yaml" in out
    assert "git mv swb.yaml testbench.yaml" in out


def test_the_whole_workspace_builds_with_its_support_packages(ws, plugin_dir):
    r = colcon(ws, plugin_dir, "--packages-up-to", "tb_good")
    assert r.returncode == 0, r.stdout + r.stderr
    assert "3 packages finished" in r.stdout


def test_without_the_plugin_on_the_path_colcon_does_not_build_a_testbench(ws, plugin_dir):
    """243's item 7: colcon's own Python must see the plugin (scensim's setup.bash). Without
    it colcon only warns and exits 0 with nothing installed, so a build "passes" without its
    testbenches; `scensim run` must refuse a testbench missing from the index (step 3)."""
    r = colcon(ws, plugin_dir, "--packages-select", "tb_good", plugin=False)
    assert r.returncode == 0                          # colcon's behaviour, not ours
    assert "No task extension to 'build' a 'ros.scensim_testbench' package" in r.stdout + r.stderr
    assert not (ws / "install" / "tb_good" / "share" / "tb_good" / "testbench.yaml").exists()
