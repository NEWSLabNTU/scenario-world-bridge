"""Roadmap 079 step 5 (A9): an empty-string parameter starts its node under either launcher.
play_launch once wrote it as `-p name:=`, which rcl rejects; 0.30 passes a params file, as
ros2 launch does, and swb's launch files carry no workaround for it any more. A real launch
of demo_nodes_cpp's talker, stopped through its process group; skipped without the tools."""
import os
import shutil
import signal
import subprocess
import time

import pytest

LAUNCH = """<launch>
  <node pkg="demo_nodes_cpp" exec="talker" name="a9_talker" output="screen">
    <param name="empty" value=""/>
  </node>
</launch>
"""


def _have_talker():
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix("demo_nodes_cpp")
        return True
    except Exception:
        return False


@pytest.mark.parametrize("launcher", ["ros2", "play_launch"])
def test_an_empty_string_parameter_starts_its_node(tmp_path, launcher):
    if shutil.which(launcher) is None or not _have_talker():
        pytest.skip(f"{launcher} or demo_nodes_cpp not installed")
    f = tmp_path / "a9.launch.xml"
    f.write_text(LAUNCH)
    log_dir = tmp_path / "logs"
    cmd = (["ros2", "launch", str(f)] if launcher == "ros2" else
           ["play_launch", "launch", "--disable-web-ui", "--log-dir", str(log_dir), str(f)])
    env = dict(os.environ, ROS_LOG_DIR=str(log_dir), ROS_LOCALHOST_ONLY="1",
               ROS_DOMAIN_ID=str(40 + os.getpid() % 15))
    out = tmp_path / "console.log"
    with open(out, "w") as fh:
        p = subprocess.Popen(cmd, env=env, stdout=fh, stderr=subprocess.STDOUT,
                             start_new_session=True)
    try:
        deadline = time.time() + 30
        seen = False
        while time.time() < deadline and not seen:
            time.sleep(0.5)
            texts = [out.read_text(errors="replace")]
            texts += [x.read_text(errors="replace") for x in log_dir.rglob("err")]
            seen = any("Publishing: 'Hello World" in t for t in texts)
            assert p.poll() is None or seen, texts[0][-2000:]
        assert seen, out.read_text(errors="replace")[-2000:]
    finally:
        for sig, wait in ((signal.SIGINT, 15), (signal.SIGKILL, 5)):
            if p.poll() is not None:
                break
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            try:
                p.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                pass


INNER = """from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=["sleep", "600"], name="node", respawn=True,
                                             respawn_delay=1.0, output="screen")])
"""

OUTER = """import importlib.util
from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown

spec = importlib.util.spec_from_file_location("stack_launch_py", {stack!r})
stack = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stack)


def generate_launch_description():
    stack.LAUNCHER["name"] = "ros2"
    keeper = ("import signal, sys, time; "
              "signal.signal(signal.SIGINT, lambda *a: sys.exit(0)); time.sleep(600)")
    return LaunchDescription([
        stack.nested("inner", [{inner!r}], {{}}, {domain}, {log_dir!r}, respawn=True),
        ExecuteProcess(cmd=["python3", "-c", keeper], name="keeper", on_exit=Shutdown()),
    ])
"""


def _ignore_sigint():
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def _group(pgid):
    out = []
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                if os.getpgid(int(d)) == pgid:
                    out.append(int(d))
            except ProcessLookupError:
                pass
    return out


def test_a_nested_ros2_launch_stops_on_the_groups_sigint_when_it_was_ignored(tmp_path):
    """Exit criterion 1, F1: a stack started in the background of a script has SIGINT
    ignored, and every `ros2 launch` below it inherited that, so the group's SIGINT (Ctrl-C)
    ended the nodes and not their nested launch, which respawned them past the stack's end.
    stack.nested() gives each nested launch the default dispositions back."""
    if shutil.which("ros2") is None:
        pytest.skip("ros2 not installed")
    inner = tmp_path / "inner.launch.py"
    inner.write_text(INNER)
    outer = tmp_path / "outer.launch.py"
    stack_py = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "launch",
                            "stack.launch.py")
    outer.write_text(OUTER.format(stack=os.path.abspath(stack_py), inner=str(inner),
                                  domain=40 + os.getpid() % 15, log_dir=str(tmp_path / "in")))
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / "out"), ROS_LOCALHOST_ONLY="1")
    with open(tmp_path / "console.log", "w") as fh:
        p = subprocess.Popen(["ros2", "launch", str(outer)], env=env, stdout=fh,
                             stderr=subprocess.STDOUT, start_new_session=True,
                             preexec_fn=_ignore_sigint)
    try:
        console = tmp_path / "in" / "console.log"
        deadline = time.time() + 30
        while time.time() < deadline and "process started" not in (
                console.read_text(errors="replace") if console.exists() else ""):
            time.sleep(0.5)
        assert "process started" in console.read_text(errors="replace")
        os.killpg(p.pid, signal.SIGINT)
        deadline = time.time() + 20
        while time.time() < deadline and (p.poll() is None or _group(p.pid)):
            time.sleep(0.5)
        left = _group(p.pid) if p.poll() is not None else [p.pid]
        assert not left, (left, console.read_text(errors="replace")[-2000:])
    finally:
        if _group(p.pid):
            os.killpg(p.pid, signal.SIGKILL)
        p.wait(timeout=10)
