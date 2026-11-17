"""carla_poses finds CARLA where scensim does, and takes --host/--port on either side of the
subcommand (roadmap 077, G63). No CARLA needed: nothing here connects."""

import importlib.machinery
import importlib.util
from pathlib import Path

import pytest

_SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def _load():
    loader = importlib.machinery.SourceFileLoader("carla_poses", str(_SCRIPTS / "carla_poses"))
    spec = importlib.util.spec_from_loader("carla_poses", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


@pytest.fixture
def poses():
    return _load()


@pytest.fixture
def host_dir(tmp_path, monkeypatch):
    """Host settings in a temporary user and system config, no project in the working dir."""
    user, system = tmp_path / "user", tmp_path / "system"
    user.mkdir()
    system.mkdir()
    monkeypatch.setenv("SCENSIM_CONFIG_HOME", str(user))
    monkeypatch.setenv("SCENSIM_SYSTEM_CONFIG", str(system))
    (system / "host.yaml").write_text(f"seat_locks: {tmp_path / 'locks'}\n")
    monkeypatch.chdir(tmp_path)
    return user


def endpoint(poses, argv):
    return poses.carla_endpoint(poses.parser().parse_args(argv))


@pytest.mark.parametrize("argv", [
    ["--host", "h", "--port", "2700", "at", "1", "2"],
    ["at", "1", "2", "--host", "h", "--port", "2700"],
    ["--host", "h", "at", "--port", "2700", "1", "2"],
    ["ahead", "3", "--port", "2700", "--host", "h", "--distance", "5"],
    ["--port", "2700", "--host", "h", "spawn-points"],
])
def test_host_and_port_before_or_after_the_subcommand(poses, host_dir, argv):
    assert endpoint(poses, argv) == ("h", 2700)


def test_defaults_are_the_host_settings(poses, host_dir):
    assert endpoint(poses, ["spawn-points"]) == ("localhost", 2000)
    (host_dir / "host.yaml").write_text("carla: {host: carla-box, port: 2600}\n")
    assert endpoint(poses, ["spawn-points"]) == ("carla-box", 2600)
    assert endpoint(poses, ["spawn-points", "--port", "2000"]) == ("carla-box", 2000)


def test_a_project_local_file_wins(poses, host_dir, tmp_path, monkeypatch):
    (host_dir / "host.yaml").write_text("carla: {port: 2600}\n")
    proj = tmp_path / "proj"
    (proj / ".scensim").mkdir(parents=True)
    (proj / "testbench.yaml").write_text("simulator: carla\n")
    (proj / ".scensim" / "local.yaml").write_text("carla: {port: 2700}\n")
    monkeypatch.chdir(proj)
    assert endpoint(poses, ["at", "1", "2"]) == ("localhost", 2700)


def test_the_seat_table_gives_the_seat_its_carla(poses, host_dir):
    (host_dir / "host.yaml").write_text(
        "seat: 2\nseats: {domains: {base: 1, stride: 8}, ports: {base: 5555, stride: 100},\n"
        "  web: {base: 8080, stride: 10}, count: 3, carla: {base: 2700, stride: 10}}\n")
    assert endpoint(poses, ["spawn-points"]) == ("localhost", 2720)
    assert endpoint(poses, ["spawn-points", "--port", "2000"]) == ("localhost", 2000)


def test_an_auto_seat_with_no_stack_of_yours_is_refused(poses, host_dir):
    (host_dir / "host.yaml").write_text(
        "seats: {domains: {base: 1, stride: 8}, ports: {base: 5555, stride: 100},\n"
        "  web: {base: 8080, stride: 10}, count: 3, carla: {base: 2700, stride: 10}}\n")
    with pytest.raises(SystemExit, match="seat take"):
        endpoint(poses, ["spawn-points"])
    assert endpoint(poses, ["spawn-points", "--port", "2710"]) == ("localhost", 2710)


def test_the_auto_seat_a_stack_of_yours_holds(poses, host_dir):
    from scensim import config, seat
    (host_dir / "host.yaml").write_text(
        "seats: {domains: {base: 1, stride: 8}, ports: {base: 5555, stride: 100},\n"
        "  web: {base: 8080, stride: 10}, count: 3, carla: {base: 2700, stride: 10}}\n")
    seats = seat.table(config.host_settings())
    with seat.acquire(seats[1], "p"):
        assert endpoint(poses, ["spawn-points"]) == ("localhost", 2710)
