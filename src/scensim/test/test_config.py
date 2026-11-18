from pathlib import Path

import pytest

from scensim import config


@pytest.fixture
def dirs(tmp_path, monkeypatch):
    user, system = tmp_path / "user", tmp_path / "system"
    user.mkdir()
    system.mkdir()
    monkeypatch.setenv("SCENSIM_CONFIG_HOME", str(user))
    monkeypatch.setenv("SCENSIM_SYSTEM_CONFIG", str(system))
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
    (proj / ".scensim").mkdir(parents=True)
    (proj / ".scensim" / "local.yaml").write_text("seat: 4\n")
    assert config.host_settings()["seat"] == 2
    assert config.host_settings()["seats"]["count"] == 3
    assert config.host_settings(proj)["seat"] == 4
    assert config.host_settings(proj, {"seat": 5})["seat"] == 5
    # a None override keeps the file's
    assert config.host_settings(proj, {"seat": None})["seat"] == 4


def test_find_project_walks_up(dirs):
    _, _, tmp = dirs
    (tmp / "proj" / "scenarios").mkdir(parents=True)
    (tmp / "proj" / "testbench.yaml").write_text("simulator: carla\n")
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
    from scensim import config
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
    from scensim import config, seat
    assert config.DEFAULTS["dds"] == {"profile": ""}
    assert seat.cyclone_profile({"dds": {"profile": "~/unicast-lo.xml"}}) == \
        Path("~/unicast-lo.xml").expanduser()
    assert seat.cyclone_profile({"dds": {"profile": ""}}).name == seat.CYCLONE_PROFILE


def test_a_variable_of_the_old_name_is_refused_naming_its_successor(dirs, monkeypatch):
    """roadmap 080: SWB_* are not read; every one is named with its SCENSIM_* successor."""
    monkeypatch.setenv("SWB_LOG_ROOT", "/x")
    monkeypatch.setenv("SWB_WEB_PORT", "8080")
    with pytest.raises(config.ConfigError) as e:
        config.host_settings()
    assert "SWB_LOG_ROOT was renamed" in str(e.value) and "set SCENSIM_LOG_ROOT" in str(e.value)
    assert "set SCENSIM_WEB_PORT" in str(e.value)


def test_an_old_directory_is_not_read_and_its_move_is_printed(tmp_path, monkeypatch, capsys):
    """roadmap 080: ~/.config/swb (and the cache, the state, a project's .swb/) are not read;
    the first command prints the mv, once per directory."""
    monkeypatch.delenv("SCENSIM_CONFIG_HOME", raising=False)
    monkeypatch.setattr(config.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(config, "_hinted", set())
    (tmp_path / ".config" / "swb").mkdir(parents=True)
    (tmp_path / ".config" / "swb" / "host.yaml").write_text("seat: 3\n")
    proj = tmp_path / "p"
    (proj / ".swb").mkdir(parents=True)
    assert config.host_settings(proj)["seat"] == "auto"
    config.host_settings(proj)
    err = capsys.readouterr().err
    assert err.count(f"mv {tmp_path}/.config/swb {tmp_path}/.config/scensim") == 1
    assert f"mv {proj}/.swb {proj}/.scensim" in err
    (tmp_path / ".config" / "scensim").mkdir()
    assert (tmp_path / ".config" / "swb", tmp_path / ".config" / "scensim") not in config.old_dirs(proj)


def test_an_old_directory_is_hinted_once_not_on_every_command(tmp_path, monkeypatch, capsys):
    """issue 058: the hint leaves a marker in the old directory, so a later process (the next
    command) does not repeat it; the new directory is not made, so the printed mv renames."""
    monkeypatch.delenv("SCENSIM_CONFIG_HOME", raising=False)
    monkeypatch.setattr(config.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(config, "_hinted", set())
    (tmp_path / ".cache" / "swb").mkdir(parents=True)
    config.host_settings()
    assert "mv " + str(tmp_path / ".cache" / "swb") in capsys.readouterr().err
    assert (tmp_path / ".cache" / "swb" / config.HINTED_MARKER).is_file()
    assert not (tmp_path / ".cache" / "scensim").exists()
    monkeypatch.setattr(config, "_hinted", set())   # the next command: a new process
    config.host_settings()
    assert "old name" not in capsys.readouterr().err


def test_an_unwritable_old_directory_is_hinted_once_per_process(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv("SCENSIM_CONFIG_HOME", raising=False)
    monkeypatch.setattr(config.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(config, "_hinted", set())
    old = tmp_path / ".local" / "state" / "swb"
    old.mkdir(parents=True)
    old.chmod(0o500)
    try:
        config.host_settings()
        config.host_settings()
        assert capsys.readouterr().err.count("mv " + str(old)) == 1
    finally:
        old.chmod(0o700)
