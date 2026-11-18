"""The car's Autoware packages in the user's workspace, checked against the sourced Autoware
(autoware_side): missing or out of range refuses; supported but untested warns."""
from pathlib import Path

from scensim import autoware_side, project

PILOTS = {"ego": {"kind": "autoware", "vehicle": "tesla_model3"},
          "npc": {"kind": "simple", "vehicle": "tesla_model3"}}
EXPORT = ('<export><scensim_autoware supported="1.5.0..2.1.0" '
          'tested="1.5.0/humble 2.0.0/humble 2.1.0/humble"/></export>')


def pkg(root: Path, name: str, version="0.0.0", export=EXPORT) -> Path:
    d = root / name
    d.mkdir(parents=True)
    (d / "package.xml").write_text(f"<package><name>{name}</name><version>{version}</version>"
                                   f"{export}</package>")
    return d


def shares(root: Path, *names, **kw):
    have = {n: pkg(root, n, **kw.get(n, {})) for n in names}
    return lambda n: have.get(n)


def test_the_pilot_needs_its_vehicle_and_sensor_kit_launches():
    assert autoware_side.required({"vehicle": "golf_cart"}) == ["golf_cart_launch",
                                                                "golf_cart_sensor_kit_launch"]
    assert autoware_side.required({"args": {"sensor_model": "kit_b"}}) == [
        "tesla_model3_launch", "kit_b_launch"]


def test_missing_packages_are_refused_with_the_import_fix(tmp_path):
    share = shares(tmp_path, "tesla_model3_launch")
    env = {"AMENT_PREFIX_PATH": "/opt/autoware/1.5.0", "ROS_DISTRO": "humble"}
    [f] = project.autoware_side_problems({"pilots": PILOTS}, env=env, share=share)
    assert f.refused and f.host
    assert "tesla_model3_sensor_kit_launch not found" in f.what
    assert "vcs import src < autoware-side.repos" in f.fix and "connect-autoware" in f.fix


def test_a_tested_release_passes_silently(tmp_path):
    share = shares(tmp_path, "tesla_model3_launch", "tesla_model3_sensor_kit_launch")
    for rel in ("1.5.0", "2.0.0", "2.1.0"):
        env = {"AMENT_PREFIX_PATH": f"/ws/install:/opt/autoware/{rel}", "ROS_DISTRO": "humble"}
        assert autoware_side.assess(PILOTS, env=env, share=share) == ([], [])


def test_a_release_outside_the_range_is_refused_naming_both(tmp_path):
    share = shares(tmp_path, "tesla_model3_launch", "tesla_model3_sensor_kit_launch")
    env = {"AMENT_PREFIX_PATH": "/opt/autoware/1.4.0", "ROS_DISTRO": "humble"}
    problems, _ = autoware_side.assess(PILOTS, env=env, share=share)
    assert len(problems) == 2
    where, what, fix = problems[0]
    assert where == "pilots.ego" and "supports Autoware 1.5.0 to 2.1.0" in what
    assert "1.4.0" in what and "install path" in what


def test_supported_but_untested_warns_and_does_not_refuse(tmp_path):
    """2.1.0 on Jazzy: inside the range, not tested yet."""
    share = shares(tmp_path, "tesla_model3_launch", "tesla_model3_sensor_kit_launch")
    env = {"AMENT_PREFIX_PATH": "/opt/autoware/2.1.0", "ROS_DISTRO": "jazzy"}
    problems, notes = autoware_side.assess(PILOTS, env=env, share=share)
    assert problems == [] and len(notes) == 2
    assert "2.1.0 on ROS 2 jazzy but has not been tested" in notes[0]
    assert "2.1.0 on humble" in notes[0]


def test_a_users_own_build_is_told_by_autoware_launch(tmp_path):
    names = ("tesla_model3_launch", "tesla_model3_sensor_kit_launch")
    for launch, want in (("0.48.0", ("1.5.0",)), ("0.52.0", ("1.9.0",)),
                         ("0.53.0", ("2.0.0", "2.1.0"))):
        root = tmp_path / launch
        share = shares(root, *names, "autoware_launch", autoware_launch={"version": launch, "export": ""})
        env = {"AMENT_PREFIX_PATH": "/home/u/autoware/install", "ROS_DISTRO": "humble"}
        assert autoware_side.sourced_releases(env, share)[0] == want
        problems, notes = autoware_side.assess(PILOTS, env=env, share=share)
        assert problems == []
        # 1.9.0 is in the range but untested; 2.0.0 or 2.1.0 is tested either way
        assert bool(notes) == (want == ("1.9.0",))


def test_no_autoware_or_no_export_is_a_note_not_a_refusal(tmp_path):
    share = shares(tmp_path, "tesla_model3_launch", "tesla_model3_sensor_kit_launch")
    problems, notes = autoware_side.assess(PILOTS, env={"ROS_DISTRO": "humble"}, share=share)
    assert problems == [] and "cannot be told" in notes[0]
    bare = shares(tmp_path / "b", "tesla_model3_launch", "tesla_model3_sensor_kit_launch",
                  tesla_model3_launch={"export": ""}, tesla_model3_sensor_kit_launch={"export": ""})
    problems, notes = autoware_side.assess(
        PILOTS, env={"AMENT_PREFIX_PATH": "/opt/autoware/1.5.0", "ROS_DISTRO": "humble"},
        share=bare)
    assert problems == [] and all("declares no supported Autoware" in n for n in notes)
