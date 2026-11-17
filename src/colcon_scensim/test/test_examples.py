"""The shipped examples and templates as testbenches (roadmap 081 step 4): each one, made with
`scensim testbench new` (``--from`` an example, ``--template`` a template) in a fresh
workspace, builds with colcon and passes `scensim testbench check` but for this host's
findings (assets not fetched, Autoware or Isaac not installed).

Needs ROS 2 (/opt/ros/humble), colcon with colcon-ros, and pip; skipped without them.
"""
import os
import subprocess
import sys
from pathlib import Path

import pytest

from test_build import PLUGIN, ROS, SCENSIM_SRC, colcon, plugin_dir, pytestmark  # noqa: F401

sys.path.insert(0, str(SCENSIM_SRC))
from scensim import project as projectmod  # noqa: E402
from scensim import testbench_cli  # noqa: E402

EXAMPLES = PLUGIN.parent / "scensim_examples" / "projects"
EXAMPLE_NAMES = sorted(p.name for p in EXAMPLES.iterdir()
                       if (p / "testbench.yaml").is_file())
TEMPLATE_NAMES = projectmod.templates()


def test_every_example_and_template_is_listed():
    # a new example or template is tested without editing this file; these are today's
    assert EXAMPLE_NAMES == ["first_drive", "isaac_first_drive", "multi_pilot",
                             "road_physics", "safety_suite", "simple_drive"]
    assert TEMPLATE_NAMES == ["carla-first-drive", "isaac", "multi-pilot"]


@pytest.fixture(scope="module")
def built(tmp_path_factory, plugin_dir):  # noqa: F811
    """One workspace holding every example and template as a testbench, built once."""
    ws = tmp_path_factory.mktemp("examples") / "ws"
    src = ws / "src"
    src.mkdir(parents=True)
    names = {}
    for x in EXAMPLE_NAMES:
        names[f"ex_{x}"] = testbench_cli.new_testbench(f"ex_{x}", src, src=str(EXAMPLES / x))
    for t in TEMPLATE_NAMES:
        n = "tpl_" + t.replace("-", "_")
        names[n] = testbench_cli.new_testbench(n, src, template=t)
    r = colcon(ws, plugin_dir, "--symlink-install")
    return ws, names, r


def test_the_workspace_of_every_example_and_template_builds(built):
    ws, names, r = built
    assert r.returncode == 0, r.stdout + r.stderr
    assert f"{len(names)} packages finished" in r.stdout, r.stdout
    for n in names:
        assert (ws / "install" / n / "share" / n / "testbench.yaml").is_file(), n


@pytest.mark.parametrize("name", [f"ex_{x}" for x in EXAMPLE_NAMES]
                         + ["tpl_" + t.replace("-", "_") for t in TEMPLATE_NAMES])
def test_each_copy_is_a_self_contained_package(built, name):
    ws, names, _ = built
    dest = names[name]
    for f in ("package.xml", "testbench.yaml", "README.md"):
        assert (dest / f).is_file(), f
    assert not (dest / ".gitignore").exists() and not (dest / "gitignore").exists()
    # nothing names a host's checkout (the build checks that no path leaves the package)
    for f in dest.rglob("*"):
        if f.is_file() and f.suffix in (".yaml", ".xosc", ".py", ".md"):
            text = f.read_text()
            assert "/home/" not in text and "/mnt/" not in text, f


@pytest.mark.parametrize("name", [f"ex_{x}" for x in EXAMPLE_NAMES]
                         + ["tpl_" + t.replace("-", "_") for t in TEMPLATE_NAMES])
def test_each_passes_testbench_check_but_for_this_hosts_findings(built, name, tmp_path):
    ws, _, _ = built
    script = (f"source {ROS} && source install/setup.bash && "
              f"export PYTHONPATH={SCENSIM_SRC}:$PYTHONPATH && "
              f"{sys.executable} -c 'import sys; from scensim.cli import main; "
              f"sys.exit(main([\"testbench\", \"check\", \"{name}\"]))'")
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "ISAAC_ROOT")}
    env["HOME"] = str(tmp_path)                     # no host settings, no fetched assets
    r = subprocess.run(["bash", "-c", script], cwd=ws, capture_output=True, text=True, env=env)
    lines = [ln for ln in (r.stdout + r.stderr).splitlines() if ln.strip()]
    package = [ln for ln in lines if ln.startswith("problem") and "(this host)" not in ln]
    assert not package, "\n".join(lines)
    assert r.returncode in (0, 3), "\n".join(lines)   # 3: this host's findings only
    assert any("scensim assets fetch " + name in ln for ln in lines), "\n".join(lines)


def test_each_example_names_its_default_suite_itself(built):
    # testbench new adds `default:` only when the example has none; a shipped default keeps
    # the suite a reader is told about (safety_suite's nightly, not centerpoint by name)
    ws, names, _ = built
    want = {"first_drive": "all", "multi_pilot": "cut_in_seeds", "safety_suite": "nightly",
            "simple_drive": "all", "isaac_first_drive": "first", "road_physics": "first"}
    for x, suite in want.items():
        text = (names[f"ex_{x}"] / "testbench.yaml").read_text()
        assert f"default: {suite} " in text, x
        assert text.count("default:") == 1, x
