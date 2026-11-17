"""scensim.launcher (roadmap 079, step 2): one builder for every launcher command line, for both
launchers; play_launch-only flags go to play_launch alone, `ros2 launch` logs by ROS_LOG_DIR."""
import pytest

from scensim import launcher


def test_play_launch_nested_has_no_web_ui_no_diagnostics_and_its_flags():
    cmd, env = launcher.argv("play_launch", ["scensim_launch", "scenario.launch.xml"],
                             {"scenario": "/p/x.xosc", "port": 5555}, "/logs/scenario",
                             play_launch_flags=launcher.ISOLATED_CONTAINERS)
    assert cmd == ["play_launch", "launch", "--disable-web-ui", "--disable-diagnostics",
                   "--log-dir", "/logs/scenario", "--container-mode", "isolated",
                   "--composable-respawn", "on-crash", "scensim_launch", "scenario.launch.xml",
                   "scenario:=/p/x.xosc", "port:=5555"]
    assert env == {}


def test_ros2_launch_takes_none_of_play_launchs_flags_and_logs_by_ros_log_dir():
    cmd, env = launcher.argv("ros2", ["scensim_launch", "scenario.launch.xml"],
                             {"scenario": "/p/x.xosc", "flag": True, "empty": None},
                             "/logs/scenario", web_port=8080,
                             play_launch_flags=launcher.ISOLATED_CONTAINERS, diagnostics=True)
    assert cmd == ["ros2", "launch", "scensim_launch", "scenario.launch.xml",
                   "scenario:=/p/x.xosc", "flag:=true", "empty:="]
    assert env == {"ROS_LOG_DIR": "/logs/scenario"}


def test_the_stack_under_play_launch_has_its_web_ui_and_keeps_diagnostics():
    cmd, _ = launcher.argv("play_launch", ["scensim_launch", "scensim.launch.xml"], {"seat": 1},
                           "/logs/x", web_port=8090, diagnostics=True)
    assert cmd[:6] == ["play_launch", "launch", "--web-addr", "127.0.0.1:8090", "--log-dir",
                       "/logs/x"]
    assert "--disable-diagnostics" not in cmd and cmd[-1] == "seat:=1"


def test_a_path_target_and_words_pass_through():
    cmd, _ = launcher.argv("ros2", ["/pilots/ego.launch.xml"], ["entity:=ego", "rviz:=true"])
    assert cmd == ["ros2", "launch", "/pilots/ego.launch.xml", "entity:=ego", "rviz:=true"]


def test_command_carries_the_environment_as_an_env_prefix():
    assert launcher.command("ros2", ["a.launch.py"], None, "/l") == [
        "env", "ROS_LOG_DIR=/l", "ros2", "launch", "a.launch.py"]
    assert launcher.command("play_launch", ["a.launch.py"])[:2] == ["play_launch", "launch"]


def test_an_unknown_launcher_is_an_error_and_an_old_record_is_play_launch():
    with pytest.raises(ValueError):
        launcher.argv("roslaunch", ["a.launch.py"])
    assert launcher.of({}) == "play_launch" and launcher.of(None) == "play_launch"
    assert launcher.of({"launcher": "ros2"}) == "ros2"
    assert launcher.of({"launcher": "bogus"}) == "play_launch"


def test_a_pilot_under_each_launcher(tmp_path):
    from scensim import stack
    rec = {"pilots_dir": str(tmp_path), "pilot_contract": {"map_path": "/m"}}
    p = {"entity": "lead", "kind": "carla_basic", "overlays": None}
    for name, head in (("play_launch", ["play_launch", "launch"]),
                       ("ros2", ["env", f"ROS_LOG_DIR={tmp_path / 'log'}", "ros2", "launch"])):
        argv = stack.pilot_argv(dict(rec, launcher=name), p, {}, tmp_path / "log")
        assert argv[:len(head)] == head
        assert "entity:=lead" in argv and "map_path:=/m" in argv
        assert ("--container-mode" in argv) is False  # carla_basic: no containers either way
