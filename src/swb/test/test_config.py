from pathlib import Path

import pytest

from swb import config


@pytest.fixture
def dirs(tmp_path, monkeypatch):
    user, system = tmp_path / "user", tmp_path / "system"
    user.mkdir()
    system.mkdir()
    monkeypatch.setenv("SWB_CONFIG_HOME", str(user))
    monkeypatch.setenv("SWB_SYSTEM_CONFIG", str(system))
    return user, system, tmp_path


def test_defaults(dirs):
    s = config.host_settings()
    assert s["seat"] == "auto"
    assert s["seats"]["domains"] == {"base": 1, "stride": 8}


def test_precedence(dirs):
    user, system, tmp = dirs
    (system / "host.yaml").write_text("seat: 1\nseats: {count: 3}\n")
    (user / "host.yaml").write_text("seat: 2\n")
    proj = tmp / "p"
    (proj / ".swb").mkdir(parents=True)
    (proj / ".swb" / "local.yaml").write_text("seat: 4\n")
    assert config.host_settings()["seat"] == 2
    assert config.host_settings()["seats"]["count"] == 3
    assert config.host_settings(proj)["seat"] == 4
    assert config.host_settings(proj, {"seat": 5})["seat"] == 5
    # a None override keeps the file's
    assert config.host_settings(proj, {"seat": None})["seat"] == 4


def test_find_project_walks_up(dirs):
    _, _, tmp = dirs
    (tmp / "proj" / "scenarios").mkdir(parents=True)
    (tmp / "proj" / "swb.yaml").write_text("simulator: carla\n")
    assert config.find_project(tmp / "proj" / "scenarios") == (tmp / "proj").resolve()


def test_find_project_missing(dirs):
    _, _, tmp = dirs
    with pytest.raises(config.ConfigError, match="project new"):
        config.find_project(tmp)


def test_bad_yaml_names_file(dirs):
    user, _, _ = dirs
    (user / "host.yaml").write_text("seat: [\n")
    with pytest.raises(config.ConfigError, match="host.yaml"):
        config.host_settings()


def test_write_user_setting(dirs):
    user, _, _ = dirs
    (user / "host.yaml").write_text("gpu: 1\n")
    config.write_user_setting("seat", 3)
    s = config.host_settings()
    assert s["seat"] == 3 and s["gpu"] == 1


def test_a_package_project_is_read_only_and_named_with_its_copy(tmp_path, monkeypatch):
    """roadmap 077, G15: a project inside an installed package's share is copied to run."""
    from swb import config
    prefix = tmp_path / "install" / "golfcart_twin"
    proj = prefix / "share" / "golfcart_twin" / "projects" / "route1"
    proj.mkdir(parents=True)
    monkeypatch.setenv("AMENT_PREFIX_PATH", f"/nowhere:{prefix}")
    assert config.package_project(proj) == "golfcart_twin"
    assert config.package_project(tmp_path / "mine") == ""
    try:
        config.writable_project(proj)
        raise AssertionError("not refused")
    except config.ConfigError as e:
        assert "--from pkg://golfcart_twin/projects/route1" in str(e)
    assert config.writable_project(tmp_path / "mine") == tmp_path / "mine"


def test_the_dds_profile_is_a_host_setting():
    from swb import config, seat
    assert config.DEFAULTS["dds"] == {"profile": ""}
    assert seat.cyclone_profile({"dds": {"profile": "~/unicast-lo.xml"}}) == \
        Path("~/unicast-lo.xml").expanduser()
    assert seat.cyclone_profile({"dds": {"profile": ""}}).name == seat.CYCLONE_PROFILE
