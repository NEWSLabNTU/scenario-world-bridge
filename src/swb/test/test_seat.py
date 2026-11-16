import json
import os
import stat
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest
import yaml

from swb import config, seat
from swb.config import ConfigError

SWB_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def env(tmp_path, monkeypatch):
    user, system, locks = tmp_path / "user", tmp_path / "system", tmp_path / "locks"
    user.mkdir()
    system.mkdir()
    monkeypatch.setenv("SWB_CONFIG_HOME", str(user))
    monkeypatch.setenv("SWB_SYSTEM_CONFIG", str(system))
    (system / "host.yaml").write_text(f"seat_locks: {locks}\n")
    monkeypatch.chdir(tmp_path)   # no swb.yaml here: no project
    return user, system, locks


def settings(**seats_over):
    s = config.merge(config.DEFAULTS, {"seats": seats_over})
    return s


def test_default_table():
    seats = seat.table(config.DEFAULTS)
    assert len(seats) == 7
    s2 = seats[2]
    assert s2.scenario_domain == 17
    assert s2.pilot_domains == [18, 19, 20, 21, 22, 23, 24]
    assert s2.ssv2_port == 5755
    assert s2.release_ports == (5756, 5757)
    assert s2.relay_port == 5760
    assert s2.world_port == 5791
    assert s2.web_port == 8100 and s2.web_port_run == 8101
    assert s2.gpu == 0
    assert seats[-1].pilot_domains[-1] == 56


def test_gpus_and_override():
    s = settings(gpus=[0, 0, 0, 1, 1, 1, 1])
    assert [x.gpu for x in seat.table(s)] == [0, 0, 0, 1, 1, 1, 1]
    s["gpu"] = 2
    assert {x.gpu for x in seat.table(s)} == {2}


@pytest.mark.parametrize("over, key", [
    ({"count": 8}, "seats.domains"),                       # 1 + 8*8 - 1 = 64 > 57
    ({"domains": {"base": 0, "stride": 1}}, "seats.domains.stride"),
    ({"domains": {"base": -1, "stride": 8}}, "seats.domains.base"),
    ({"ports": {"base": 5555, "stride": 36}}, "seats.ports.stride"),
    ({"ports": {"base": 65500, "stride": 100}}, "seats.ports"),
    ({"web": {"base": 8080, "stride": 1}}, "seats.web.stride"),
    ({"web": {"base": 5560, "stride": 10}}, "seats.web overlaps"),
    ({"gpus": [0, 1]}, "seats.gpus"),
    ({"count": 0}, "seats.count"),
    ({"count": "7"}, "seats.count"),
])
def test_bad_tables_refused(over, key):
    with pytest.raises(ConfigError) as e:
        seat.table(settings(**over), "/etc/swb/host.yaml")
    assert key in str(e.value)
    assert "/etc/swb/host.yaml" in str(e.value)


def test_source_names_the_file(env):
    user, system, _ = env
    assert seat.seats_source() == "defaults"
    (system / "host.yaml").write_text("seats: {count: 2}\n")
    assert seat.seats_source() == str(system / "host.yaml")
    (user / "host.yaml").write_text("seats: {count: 3}\n")
    assert seat.seats_source() == str(user / "host.yaml")


def test_resolve_int_and_auto(env):
    _, _, locks = env
    s = config.host_settings()
    s["seat"] = 3
    assert seat.resolve(s).index == 3
    s["seat"] = "4"
    assert seat.resolve(s).index == 4
    s["seat"] = 9
    with pytest.raises(ConfigError):
        seat.resolve(s)
    s["seat"] = "auto"
    seats = seat.table(s)
    with seat.acquire(seats[0], "p"), seat.acquire(seats[1], "p"):
        assert seat.resolve(s).index == 2
    assert seat.resolve(s).index == 0


def test_lock_dir_is_sticky_world_writable(env):
    _, _, locks = env
    s = seat.table(config.host_settings())[0]
    with seat.acquire(s, "/proj") as lk:
        assert lk.held
        assert stat.S_IMODE(os.stat(locks).st_mode) == 0o1777
        info = json.loads(Path(locks / "seat-0.lock").read_text())
        assert set(info) == {"user", "pid", "host", "project", "since"}
        assert info["pid"] == os.getpid() and info["project"] == "/proj"
        assert seat.holder(0, locks)["pid"] == os.getpid()
        with pytest.raises(seat.SeatTaken) as e:
            seat.acquire(s, "/other")
        assert str(os.getpid()) in str(e.value)
    assert not lk.held
    assert seat.holder(0, locks) is None


def test_holder_in_other_process(env):
    _, _, locks = env
    code = textwrap.dedent(f"""
        import sys, time
        sys.path.insert(0, {str(SWB_ROOT)!r})
        from swb import seat, config
        s = seat.table(config.host_settings())[1]
        lk = seat.acquire(s, "child")
        print("held", flush=True)
        sys.stdin.readline()
    """)
    p = subprocess.Popen([sys.executable, "-c", code], stdin=subprocess.PIPE,
                         stdout=subprocess.PIPE, text=True, env=os.environ.copy())
    try:
        assert p.stdout.readline().strip() == "held"
        h = seat.holder(1, locks)
        assert h["pid"] == p.pid and h["project"] == "child"
        assert seat.main(["take", "1"]) == 0   # same user: allowed
    finally:
        p.stdin.write("\n")
        p.stdin.flush()
        p.wait(10)
    # the holder died: the lock is free again, its file reclaimed
    assert seat.holder(1, locks) is None
    with seat.acquire(seat.table(config.host_settings())[1], "me"):
        assert seat.holder(1, locks)["pid"] == os.getpid()


def test_stale_file_with_dead_pid_is_reclaimed(env):
    _, _, locks = env
    locks.mkdir()
    dead = subprocess.Popen([sys.executable, "-c", "pass"])
    dead.wait()
    (locks / "seat-2.lock").write_text(json.dumps(
        {"user": "someone", "pid": dead.pid, "host": "h", "project": "x", "since": "t"}))
    assert seat.holder(2, locks) is None
    with seat.acquire(seat.table(config.host_settings())[2], "mine") as lk:
        assert lk.info["project"] == "mine"


def test_take_writes_user_file_and_holds_nothing(env, capsys):
    user, _, locks = env
    assert seat.main(["take", "3"]) == 0
    assert yaml.safe_load((user / "host.yaml").read_text())["seat"] == 3
    out = capsys.readouterr().out
    assert "seat 3" in out and "5855" in out and "5891" in out
    assert seat.holder(3, locks) is None


def test_take_auto_skips_held(env):
    user, _, locks = env
    seats = seat.table(config.host_settings())
    with seat.acquire(seats[0], "p"):
        assert seat.main(["take", "auto"]) == 0
    assert yaml.safe_load((user / "host.yaml").read_text())["seat"] == 1


def test_take_refuses(env, monkeypatch):
    user, _, locks = env
    assert seat.main(["take", "99"]) == 3
    assert seat.main(["take", "x"]) == 3
    seats = seat.table(config.host_settings())
    with seat.acquire(seats[2], "p"):
        monkeypatch.setattr(seat.getpass, "getuser", lambda: "another_user")
        assert seat.main(["take", "2"]) == 3
    assert not (user / "host.yaml").exists()


def test_invalid_table_refused_by_command(env, capsys):
    user, _, _ = env
    (user / "host.yaml").write_text("seats: {count: 20}\n")
    assert seat.main(["show"]) == 3
    err = capsys.readouterr().err
    assert str(user / "host.yaml") in err and "seats.domains" in err


def test_show(env, capsys):
    user, _, _ = env
    (user / "host.yaml").write_text("seat: 1\n")
    seats = seat.table(config.host_settings())
    with seat.acquire(seats[4], "/some/proj"):
        assert seat.main(["show"]) == 0
    lines = capsys.readouterr().out.splitlines()
    rows = [ln for ln in lines if ln.startswith("seat ")]
    assert len(rows) == 7
    assert "<- yours" in rows[1] and "free" in rows[1]
    assert "held by" in rows[4] and "/some/proj" in rows[4]
    assert "free" in rows[0] and "yours" not in rows[0]


def test_release(env, capsys):
    user, _, _ = env
    assert seat.main(["take", "2"]) == 0
    assert seat.main(["release"]) == 0
    assert yaml.safe_load((user / "host.yaml").read_text())["seat"] == "auto"
    seats = seat.table(config.host_settings())
    with seat.acquire(seats[5], "p"):
        assert seat.main(["release"]) == 0
    assert "still held by your stack" in capsys.readouterr().out


def test_env(env, capsys):
    user, _, _ = env
    assert seat.main(["env", "scenario"]) == 3      # auto, nothing held
    (user / "host.yaml").write_text("seat: 2\n")
    capsys.readouterr()
    assert seat.main(["env", "scenario"]) == 0
    out = capsys.readouterr().out.strip()
    assert out.startswith("export ROS_DOMAIN_ID=17 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp "
                          "CYCLONEDDS_URI=file://")
    assert out.endswith("/config/cyclonedds-localhost.xml")
    assert seat.main(["env", "pilot:0"]) == 0
    assert "ROS_DOMAIN_ID=18 " in capsys.readouterr().out
    assert seat.main(["env", "6"]) == 0
    assert "ROS_DOMAIN_ID=24 " in capsys.readouterr().out
    assert seat.main(["env", "pilot:7"]) == 3
    assert seat.main(["env", "ego"]) == 3


def test_env_auto_uses_held_seat(env, capsys):
    seats = seat.table(config.host_settings())
    with seat.acquire(seats[3], "p"):
        assert seat.main(["env", "scenario"]) == 0
    assert "ROS_DOMAIN_ID=25 " in capsys.readouterr().out


def test_cyclone_profile_without_ament(monkeypatch):
    import builtins
    real = builtins.__import__

    def no_ament(name, *a, **k):
        if name.startswith("ament_index_python"):
            raise ImportError(name)
        return real(name, *a, **k)
    monkeypatch.setattr(builtins, "__import__", no_ament)
    assert seat.cyclone_profile().name == "cyclonedds-localhost.xml"


def test_bad_argument_exit_code():
    assert seat.main([]) == 3
    assert seat.main(["bogus"]) == 3


def test_seat_table_gives_each_seat_a_carla(tmp_path):
    from swb import config as cfg
    s = cfg.merge(cfg.DEFAULTS, {"seats": {"count": 3, "carla": {"base": 2000, "stride": 10}}})
    seats = seat.table(s)
    assert [x.carla_port for x in seats] == [2000, 2010, 2020]
    assert seat.table(cfg.DEFAULTS)[0].carla_port is None
    bad = cfg.merge(cfg.DEFAULTS, {"seats": {"carla": {"base": 5555, "stride": 10}}})
    with pytest.raises(cfg.ConfigError, match="seats.carla overlaps"):
        seat.table(bad)
    with pytest.raises(cfg.ConfigError, match="seats.carla.stride"):
        seat.table(cfg.merge(cfg.DEFAULTS, {"seats": {"carla": {"base": 2000, "stride": 2}}}))


def _listen(port=0):
    """A listener on ``port``; 0 lets the kernel pick one (read it from getsockname)."""
    import socket
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen()
    return s


def _foreign_web_stack(env, tries=20):
    """Settings whose seat 0 web UI port is a listener the kernel picked (port 0), with
    every other port of seats 0 and 1 free; and that listener.

    Fixed ports are flaky: the kernel's picks and outgoing connections come from the
    ephemeral range (32768-60999 on many hosts), so any fixed port there may be taken by
    the time the test binds it. The listener's port is the kernel's own pick, the SSv2,
    relay and world server ports sit below the ephemeral range (only a listener can hold
    them), and seat 1's web UI port, the one other port in that range, is checked free;
    a layout whose check fails is dropped and another port picked."""
    for _ in range(tries):
        sock = _listen()
        web = sock.getsockname()[1]
        st = config.merge(settings(ports={"base": 20055, "stride": 100},
                                   web={"base": web, "stride": 10}),
                          {"seat_locks": str(env[2])})
        s0, s1 = seat.table(st)[:2]
        if not seat.busy_ports(s0, web=False) and not seat.busy_ports(s1):
            return st, sock
        sock.close()
    pytest.skip(f"no free seat layout in {tries} tries (ports 20055-20291 held?)")


def test_auto_skips_a_seat_whose_ports_a_foreign_stack_holds(env):
    """roadmap 077, G30: a stack outside the seat table (a checkout's `just run` on its own
    SWB_WEB_PORT) holds seat 0's numbers without its lock; auto takes seat 1 instead."""
    st, sock = _foreign_web_stack(env)
    s0 = seat.table(st)[0]
    try:
        assert s0.web_port == sock.getsockname()[1]
        assert seat.busy_ports(s0) == [("web UI", s0.web_port)]
        assert seat.busy_ports(s0, web=False) == []
        got = seat.resolve(st, usable=lambda s: not seat.busy_ports(s))
        assert got.index == 1
        assert seat.resolve(st).index == 0   # without the check, as seat show resolves
        msg = seat.describe_busy(s0, seat.busy_ports(s0))
        assert f"web UI {s0.web_port}" in msg and "seat take auto" in msg
    finally:
        sock.close()
    assert ("web UI", s0.web_port) not in seat.busy_ports(s0)


def test_carla_endpoint(env, capsys):
    user, _, _ = env
    assert seat.carla_endpoint(config.host_settings()) == ("localhost", 2000)
    (user / "host.yaml").write_text("carla: {host: box, port: 2600}\n")
    assert seat.carla_endpoint(config.host_settings()) == ("box", 2600)
    table = "seats: {count: 3, carla: {base: 2700, stride: 10}}\n"
    (user / "host.yaml").write_text("carla: {host: box}\nseat: 2\n" + table)
    assert seat.carla_endpoint(config.host_settings()) == ("box", 2720)
    (user / "host.yaml").write_text(table)
    with pytest.raises(ConfigError, match="seat take"):
        seat.carla_endpoint(config.host_settings())
    with seat.acquire(seat.table(config.host_settings())[1], "p"):
        assert seat.carla_endpoint(config.host_settings()) == ("localhost", 2710)
