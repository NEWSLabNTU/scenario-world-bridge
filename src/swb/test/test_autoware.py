"""``ros2 run swb autoware``: the overlay and the TensorRT runtime without just (077, G62)."""
from pathlib import Path

from swb import autoware


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
    assert f'swb_autoware_overlay_build "{d["overlay"]}" "{d["setup"]}"' in ov
    trt = autoware.command("tensorrt", tmp_path, d)[2]
    assert f'swb_tensorrt_setup "{d["tensorrt"]}"' in trt


def test_a_venv_is_kept_off_the_builds_path(monkeypatch):
    monkeypatch.setenv("PATH", "/x/.venv/bin:/usr/bin")
    assert autoware.clean_env()["PATH"] == "/usr/bin"
