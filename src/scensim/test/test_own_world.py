"""A world server from a support package (roadmap 081 step 5): world.server's rules, the
command scensim starts it with, and the conformance report as findings."""
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

from scensim import own_world


def test_a_server_is_a_package_an_executable_and_its_own_arguments():
    assert own_world.server_problems({"server": ["my_world", "my_server", "--fast"]}) == []
    assert own_world.server_problems({}) == []
    assert own_world.server_argv(["my_world", "my_server", "--fast"], 5599) == [
        "ros2", "run", "my_world", "my_server", "--fast", "--port", "5599"]


def test_a_bad_server_is_refused_by_what_is_wrong():
    assert own_world.server_problems({"server": "my_world my_server"}) == [
        ("world.server", "is not [<package>, <executable>, args...] "
                         "(e.g. [my_world, my_world_server])")]
    assert own_world.server_problems({"server": ["my-world", "x"]}) == [
        ("world.server", "my-world: not a package name")]
    got = own_world.server_problems({"server": ["w", "s", "--port", "1"]})
    assert got == [("world.server", "--port is scensim's to give (the seat's world port): drop it")]
    got = own_world.server_problems({"server": ["w", "s"], "world_config_file": "c.yaml"})
    assert got[0][0] == "world.world_config_file" and "stock CARLA server's" in got[0][1]
    # config_file is the bridge's (bridge_config.yaml): it stays beside a server of one's own
    assert own_world.server_problems({"server": ["w", "s"], "config_file": "c.yaml"}) == []


def test_a_server_that_never_comes_up_is_one_finding(tmp_path):
    # `false` as the check: hello fails; the server (`ros2 run` of nothing) never answers
    got = own_world.conformance_findings(["no_such_pkg", "no_exe"], 5599, check=["false"],
                                         start_timeout=1)
    assert len(got) == 1 and got[0].file == "world.server"
    assert "no_such_pkg no_exe" in got[0].what
    assert got[0].fix.startswith("see 'Your world server in a testbench' in scensim's docs")


def test_each_failed_conformance_check_is_a_finding(tmp_path):
    """A fake check: hello passes, conformance fails two checks."""
    fake = tmp_path / "check.py"
    fake.write_text(
        "import sys\n"
        "if sys.argv[1] == 'hello':\n    sys.exit(0)\n"
        "print('  PASS  hello')\nprint('  FAIL  spawn: no body')\n"
        "print('  FAIL  step: pose not kept')\nprint('1 passed, 2 failed')\nsys.exit(1)\n")
    got = own_world.conformance_findings(["w", "s"], 5599, check=[sys.executable, str(fake)],
                                         start_timeout=1)
    assert [f.what for f in got] == ["conformance: spawn: no body",
                                     "conformance: step: pose not kept"]


def test_simulator_none_needs_a_server_and_no_stock_vehicle_side():
    ok = {"simulator": "none", "world": {"server": ["w", "s"]},
          "pilots": {"ego": {"kind": "launch", "launch": "p d.launch.xml"}}}
    assert own_world.simulator_problems(ok) == []
    assert own_world.simulator_problems({"simulator": "carla"}) == []
    got = own_world.simulator_problems({"simulator": "none",
                                        "pilots": {"ego": {"kind": "autoware"}}})
    assert [w for w, _ in got] == ["simulator", "pilots.ego"]
    assert "none needs world.server" in got[0][1]
    assert "kind autoware drives through CARLA's or Isaac's vehicle side" in got[1][1]


def test_world_server_and_simulator_none_are_world_rules():
    """The world rules (scensim.world, every project check) know world.server and
    simulator: none, so a project and a testbench refuse the same things."""
    from scensim import world
    ok = {"simulator": "none", "world": {"server": ["my_world", "my_server"]}}
    assert world.problems(ok, "none") == []
    assert world.problems({"world": {"server": ["my_world", "my_server"], "weather": "WetNoon"}},
                          "carla") == []
    assert world.problems({"simulator": "none"}, "none") == [
        ("simulator", "none needs world.server: your world server, which owns the simulator "
                      "(world: {server: [<package>, <executable>]})")]
    assert ("world.world_config_file", "is the stock CARLA server's; with world.server, your "
            "server takes its own settings as its arguments") in world.problems(
        {"world": {"server": ["a", "b"], "world_config_file": "c.yaml"}}, "carla")


# --- the real check against fixture servers (081 edit 4) -------------------------------------
FIXTURES = Path(__file__).parent / "fixtures" / "world_servers"
SHELL = Path(__file__).resolve().parents[2] / "scensim_world_protocol" / "scensim_world_server"


def _real_check():
    """scensim_world_check from the sourced workspace, and an environment whose Python finds
    the scensim_world_server shell (installed, or the protocol submodule's source); skipped
    without either (no build of the protocol, or no pyzmq)."""
    try:
        from ament_index_python.packages import get_package_prefix
        exe = Path(get_package_prefix("scensim_world_check")) / "lib" / "scensim_world_check" \
            / "scensim_world_check"
    except Exception:  # noqa: BLE001 (not built, or no ROS)
        pytest.skip("scensim_world_check is not in the sourced workspace")
    if not exe.is_file():
        pytest.skip(f"{exe}: not built")
    env = dict(os.environ)
    if SHELL.is_dir():
        env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(SHELL), env.get("PYTHONPATH")]))
    probe = subprocess.run([sys.executable, "-c", "import zmq, scensim_world_server.server"],
                           env=env, capture_output=True)
    if probe.returncode != 0:
        pytest.skip("the scensim_world_server shell or pyzmq is not importable")
    return [str(exe)], env


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _fixture(*flags):
    return [sys.executable, str(FIXTURES / "kinematic_world.py"), *flags]


def test_a_conforming_world_server_passes_the_real_check(tmp_path):
    check, env = _real_check()
    got = own_world.conformance_findings(["fixture_world", "kinematic_world"], _free_port(),
                                         check=check, env=env, start_timeout=20,
                                         command=_fixture(), log=tmp_path / "server.log")
    assert got == [], [str(f) for f in got]


def test_a_world_server_that_breaks_the_protocol_fails_it_by_name(tmp_path):
    check, env = _real_check()
    got = own_world.conformance_findings(["fixture_world", "kinematic_world"], _free_port(),
                                         check=check, env=env, start_timeout=20,
                                         command=_fixture("--broken"),
                                         log=tmp_path / "server.log")
    assert got and all(f.file == "world.server" and f.what.startswith("conformance: ")
                       for f in got), [str(f) for f in got]


def test_a_live_stacks_world_server_is_probed_with_hello(tmp_path):
    check, env = _real_check()
    port = _free_port()
    assert own_world.hello_findings(port, check=check, env=env) != []   # nothing there
    proc = subprocess.Popen(_fixture("--port", str(port)), env=env, start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        assert own_world.hello_findings(port, check=check, env=env) == []
    finally:
        own_world._stop(proc)


def test_hello_findings_name_the_endpoint_and_the_server_says_why():
    got = own_world.hello_findings(5599, check=[sys.executable, "-c",
                                                "import sys; print('no answer'); sys.exit(1)"])
    assert len(got) == 1 and got[0].host
    assert "tcp://127.0.0.1:5599 did not answer Hello: no answer" in got[0].what


def test_the_server_command_can_be_given():
    assert own_world.server_argv(["w", "s", "--x"], 7, command=["python3", "f.py"]) == [
        "python3", "f.py", "--port", "7"]


# --- issue 048: the check tool missing is a host finding, not the server's ----------------
NO_PACKAGE = [sys.executable, "-c",
              "import sys; print(\"Package 'scensim_world_check' not found\"); sys.exit(1)"]


def test_a_missing_check_tool_is_not_blamed_on_the_stacks_server():
    got = own_world.hello_findings(5599, check=NO_PACKAGE)
    assert len(got) == 1 and got[0].host and got[0].file == "world"
    assert "the probe did not run (Package 'scensim_world_check' not found)" in got[0].what
    assert "did not answer" not in got[0].what
    assert got[0].fix == own_world.NO_TOOL_FIX


def test_a_missing_check_tool_is_not_blamed_on_an_own_server(tmp_path):
    started = time.time()
    got = own_world.conformance_findings(
        ["w", "s"], 5599, check=NO_PACKAGE, start_timeout=60,
        command=[sys.executable, "-c", "import time; time.sleep(30)"])
    assert time.time() - started < 20       # no wait for a Hello that cannot be asked
    assert len(got) == 1 and got[0].host and got[0].file == "world.server"
    assert "the probe did not run (Package 'scensim_world_check' not found)" in got[0].what
    assert got[0].fix == own_world.NO_TOOL_FIX


def test_a_check_tool_that_cannot_start_is_the_same_finding():
    got = own_world.conformance_findings(["w", "s"], 5599, check=["/nonexistent/check"],
                                         command=[sys.executable, "-c", "pass"])
    assert len(got) == 1 and got[0].host and "the probe did not run" in got[0].what


def test_the_fixture_world_despawns_an_unknown_handle_without_raising():
    _, env = _real_check()          # skips where the shell is not importable
    code = ("import sys; sys.path.insert(0, sys.argv[1]); import kinematic_world as k; "
            "w = k.KinematicWorld(); w.despawn(99); print('ok')")
    r = subprocess.run([sys.executable, "-c", code, str(FIXTURES)], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0 and r.stdout.strip() == "ok", r.stderr
