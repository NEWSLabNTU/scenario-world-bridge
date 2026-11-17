"""A world server from a support package (roadmap 081 step 5): world.server's rules, the
command scensim starts it with, and the conformance report as findings."""
import sys

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
    got = own_world.server_problems({"server": ["w", "s"], "config_file": "c.yaml"})
    assert got[0][0] == "world.config_file" and "stock CARLA server's" in got[0][1]


def test_a_server_that_never_comes_up_is_one_finding(tmp_path):
    # `false` as the check: hello fails; the server (`ros2 run` of nothing) never answers
    got = own_world.conformance_findings(["no_such_pkg", "no_exe"], 5599, check=["false"],
                                         start_timeout=1)
    assert len(got) == 1 and got[0].file == "world.server"
    assert "no_such_pkg no_exe" in got[0].what
    assert got[0].fix.startswith("docs/how-to/your-world-server.md")


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
