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
