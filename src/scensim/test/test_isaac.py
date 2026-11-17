"""scensim.isaac (roadmap 077, G25): the Isaac message bindings are checked before a stack starts,
from aib's build script's own package list, and a missing or stale one is refused by name."""
import os
from pathlib import Path

import pytest
import yaml

from conftest import PKGS, fake_isaac
from scensim import isaac, project, stack

AIB_SCRIPT = (Path(__file__).resolve().parents[2] / "autoware_isaac_bridge" / "src" / "aib"
              / "scripts" / "build_msgs.sh")


def host_of(root, setup):
    return {"isaac": {"root": str(root)}, "autoware": {"setup": str(setup)}}


@pytest.mark.skipif(not AIB_SCRIPT.is_file(), reason="aib submodule not checked out")
def test_the_package_list_is_aibs_build_scripts():
    pkgs = isaac.required_packages(AIB_SCRIPT)
    assert "autoware_perception_msgs" in pkgs   # the one 241's run lacked (2026-11-15)
    assert {"autoware_vehicle_msgs", "autoware_control_msgs"} <= set(pkgs)


def test_a_built_install_has_no_problem(tmp_path, monkeypatch):
    root, setup = fake_isaac(tmp_path, monkeypatch)
    assert isaac.findings(host_of(root, setup)) == []


def test_a_missing_package_is_refused_by_name(tmp_path, monkeypatch):
    root, setup = fake_isaac(tmp_path, monkeypatch, built=PKGS[:2])
    [f] = isaac.findings(host_of(root, setup))
    assert f.refused and "autoware_perception_msgs is not built" in f.what
    assert str(root / "msgs") in f.what and f.fix == "scensim isaac msgs"


def test_a_package_built_from_an_older_autoware_is_stale(tmp_path, monkeypatch):
    root, setup = fake_isaac(tmp_path, monkeypatch)
    newer = (tmp_path / "autoware" / "local" / "lib" / "python3.10" / "dist-packages"
             / "autoware_perception_msgs" / "msg" / "_predicted_objects.py")
    newer.write_text("")
    [f] = isaac.findings(host_of(root, setup))
    assert f.refused and "autoware_perception_msgs is stale: lacks msg/predicted_objects" in f.what


def test_a_library_older_than_the_install_is_stale(tmp_path, monkeypatch):
    root, setup = fake_isaac(tmp_path, monkeypatch)
    lib = root / "msgs" / "lib" / "libautoware_control_msgs__rosidl_typesupport_c.so"
    os.utime(lib, (1, 1))
    [f] = isaac.findings(host_of(root, setup))
    assert "autoware_control_msgs is stale" in f.what and "older than" in f.what


def test_no_root_and_a_wrong_root_are_refused(tmp_path, monkeypatch):
    """077 G75/G76: no isaac.root and no ISAAC_ROOT is refused (no lab default), and so is a
    root without install_isaac.sh's venv and Isaac Sim in it."""
    monkeypatch.delenv("ISAAC_ROOT", raising=False)
    [f] = isaac.findings({"isaac": {"root": ""}})
    assert f.refused and f.what.startswith("isaac.root is not set (nor ISAAC_ROOT)")
    assert f.fix == "set isaac.root in ~/.config/scensim/host.yaml (or export ISAAC_ROOT)"
    [f] = isaac.findings({"isaac": {"root": str(tmp_path / "nowhere")}})
    assert f.refused and "not an Isaac Sim install (no venv/bin/activate or " in f.what
    root, setup = fake_isaac(tmp_path, monkeypatch)
    (root / "venv" / "lib" / "python3.12" / "site-packages" / "isaacsim").rmdir()
    [f] = isaac.findings(host_of(root, setup))
    assert f.what.endswith("(no venv/lib/python3.12/site-packages/isaacsim)")


def test_project_check_refuses_an_isaac_project_without_a_root(host, maps_hub, monkeypatch):
    """118's fresh host (077 G76): `project check` printed ok with isaac.root unset."""
    d = _isaac_project(host, monkeypatch, built=PKGS)
    (d / ".scensim" / "local.yaml").write_text(yaml.safe_dump({"isaac": {"root": ""}}))
    monkeypatch.delenv("ISAAC_ROOT", raising=False)
    found = [f for f in project.check_project(str(d)) if "isaac.root" in f.what]
    assert len(found) == 1 and found[0].refused
    with pytest.raises(stack.StackRefused, match="isaac.root is not set"):
        stack.plan(project_arg=str(d))


def test_the_environment_root_counts_when_the_setting_is_unset(tmp_path, monkeypatch):
    root, setup = fake_isaac(tmp_path, monkeypatch, built=())
    monkeypatch.setenv("ISAAC_ROOT", str(root))
    found = isaac.findings({"isaac": {"root": ""}, "autoware": {"setup": str(setup)}})
    assert len(found) == len(PKGS)


def test_build_passes_only_the_missing_packages(tmp_path, monkeypatch):
    root, setup = fake_isaac(tmp_path, monkeypatch, built=PKGS[:2])
    calls = []
    monkeypatch.setattr(isaac.subprocess, "call", lambda argv, env: calls.append((argv, env)) or 0)
    assert isaac.build(host_of(root, setup)) == 0
    [(argv, env)] = calls
    assert argv[1].endswith("share/aib/scripts/build_msgs.sh")
    assert argv[2:] == ["autoware_perception_msgs"]
    assert env["ISAAC_ROOT"] == str(root) and env["AUTOWARE_PREFIX"] == str(setup.parent)


def _isaac_project(host, monkeypatch, built):
    from test_stack import make   # the stack tests' project maker
    tmp = host["tmp"]
    (tmp / "system" / "host.yaml").write_text(yaml.safe_dump(
        {"seat_locks": str(tmp / "locks"), "logs": str(tmp / "logs"), "seat": 2}))
    d = make(host, template="isaac")
    root, setup = fake_isaac(tmp, monkeypatch, built=built)
    (d / ".scensim").mkdir(exist_ok=True)
    (d / ".scensim" / "local.yaml").write_text(yaml.safe_dump(host_of(root, setup)))
    return d


def test_project_check_and_the_stack_refuse_up_front(host, maps_hub, monkeypatch):
    d = _isaac_project(host, monkeypatch, built=PKGS[:2])
    found = [f for f in project.check_project(str(d)) if "Isaac message" in f.what]
    assert len(found) == 1 and "autoware_perception_msgs" in found[0].what
    with pytest.raises(stack.StackRefused, match="autoware_perception_msgs is not built"):
        stack.plan(project_arg=str(d))
