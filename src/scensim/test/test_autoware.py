"""``ros2 run scensim autoware``: the overlay and the TensorRT runtime without just (077, G62)."""
from pathlib import Path

from scensim import autoware


def test_defaults_are_the_workspaces_the_pilot_reads(tmp_path):
    d = autoware.dirs({}, tmp_path)
    assert d["overlay"] == tmp_path / "third_party" / "autoware_overlay"
    assert d["tensorrt"] == tmp_path / "third_party" / "tensorrt" / "lib"
    assert d["setup"] == Path(autoware.DEFAULT_SETUP)


def test_host_settings_win(tmp_path):
    d = autoware.dirs({"autoware": {"overlay": "~/ov", "tensorrt_lib": "/t/lib",
                                    "setup": "/a/setup.bash"}}, tmp_path)
    assert d["overlay"] == Path.home() / "ov" and d["tensorrt"] == Path("/t/lib")
    assert d["setup"] == Path("/a/setup.bash")


def test_the_commands_run_the_workspaces_scripts(tmp_path):
    d = autoware.dirs({}, tmp_path)
    ov = autoware.command("overlay", tmp_path, d)[2]
    assert f'source "{tmp_path}/scripts/autoware_overlay.bash"' in ov
    assert f'scensim_autoware_overlay_build "{d["overlay"]}" "{d["setup"]}"' in ov
    trt = autoware.command("tensorrt", tmp_path, d)[2]
    assert f'scensim_tensorrt_setup "{d["tensorrt"]}"' in trt


def test_a_venv_is_kept_off_the_builds_path(monkeypatch):
    monkeypatch.setenv("PATH", "/x/.venv/bin:/usr/bin")
    assert autoware.clean_env()["PATH"] == "/usr/bin"


def test_the_release_is_the_setups_version_directory():
    assert autoware.release_of("/opt/autoware/2.1.0/setup.bash") == "2.1.0"
    assert autoware.release_of("/opt/autoware/1.5.0/setup.bash") == "1.5.0"
    assert autoware.release_of("/srv/aw/2.0.0/install/setup.bash") == "2.0.0"
    assert autoware.release_of("/opt/ros/humble/setup.bash") is None


def test_a_pilots_release_follows_the_host_and_refuses_another():
    import pytest
    host = {"autoware": {"setup": "/opt/autoware/2.1.0/setup.bash"}}
    assert autoware.pilot_release(host, "pilots.ego", {"x": 1}) == \
        {"x": 1, "autoware_release": "2.1.0"}
    assert autoware.pilot_release(host, "pilots.ego", {"autoware_release": "2.1.0"}) == \
        {"autoware_release": "2.1.0"}
    with pytest.raises(ValueError, match="pilots.ego: autoware_release:=1.5.0, but the host's "
                       "Autoware is 2.1.0"):
        autoware.pilot_release(host, "pilots.ego", {"autoware_release": "1.5.0"})
    # an Autoware in the ROS prefix names no release: the pilot's own choice stands
    ros = {"autoware": {"setup": "/opt/ros/humble/setup.bash"}}
    assert autoware.pilot_release(ros, "p", {"autoware_release": "2.0.0"}) == \
        {"autoware_release": "2.0.0"}


def _built_against(ws, rel, release):
    f = ws / rel / "install" / "setup.bash"
    f.parent.mkdir(parents=True)
    f.write_text('COLCON_CURRENT_PREFIX="/opt/ros/humble"\n'
                 f'COLCON_CURRENT_PREFIX="/opt/autoware/{release}"\n'
                 'COLCON_CURRENT_PREFIX="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)"\n')
    return f


def test_a_workspaces_autoware_is_its_setups_underlay(tmp_path):
    f = _built_against(tmp_path, ".", "1.5.0")
    assert autoware.chained_release(f) == "1.5.0"
    assert autoware.chained_release(tmp_path / "none.bash") is None


def test_a_workspace_built_against_another_release_is_named_with_the_fix(tmp_path):
    acb = _built_against(tmp_path, "src/autoware_carla_bridge", "2.1.0")
    own = _built_against(tmp_path, ".", "1.5.0")
    host = {"autoware": {"setup": "/opt/autoware/2.1.0/setup.bash"}}
    [why] = autoware.install_problems(host, tmp_path)
    assert why.startswith(f"{own} was built against Autoware 1.5.0 and sources it, but "
                          "autoware.setup is Autoware 2.1.0 (/opt/autoware/2.1.0/setup.bash)")
    assert "rebuild with AUTOWARE_SETUP=/opt/autoware/2.1.0/setup.bash" in why
    assert str(acb) not in why
    # with autoware.setup at 1.5.0 it is acb's build that is named
    [why] = autoware.install_problems({"autoware": {"setup": "/opt/autoware/1.5.0/setup.bash"}},
                                      tmp_path)
    assert why.startswith(f"{acb} was built against Autoware 2.1.0")
    assert autoware.install_problems({"autoware": {"setup": "/opt/ros/humble/setup.bash"}},
                                     tmp_path) == []
    assert autoware.install_problems(host, None) == []
