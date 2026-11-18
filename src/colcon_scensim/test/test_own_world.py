"""A world server from a support package, live (roadmap 081 step 5): the fixture package
fixture_world, built by colcon in the fixture workspace, passes the world protocol's
conformance check when scensim starts it as `world: {server: [fixture_world, fake_world]}`
would, and scensim_launch's world_server.launch.py brings it up on the port it is given.

Needs ROS 2 and colcon (as test_build), the conformance check (`scensim_world_check`: set
SCENSIM_WORLD_CHECK to its binary, else `ros2 run scensim_world_check scensim_world_check` must
work) and the world protocol's Python shell (SCENSIM_WORLD_SERVER_PY: the directory holding the
`scensim_world_server` package; default: this repository's submodule), with pyzmq importable
(SCENSIM_TEST_PYDEPS: extra directories for PYTHONPATH, e.g. a `pip install --target` of pyzmq).
Skipped without them.
"""
import os
import shutil
import signal
import socket
import subprocess
import time
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SCENSIM_SRC = REPO / "src" / "scensim"
LAUNCH = REPO / "src" / "scensim_launch" / "launch" / "world_server.launch.py"
FIXTURE = HERE / "fixtures" / "ws"
# the sourced distro's ROS 2 (ROS_DISTRO), Humble when none is sourced
ROS = Path(f"/opt/ros/{os.environ.get('ROS_DISTRO') or 'humble'}/setup.bash")
SHELL_PY = Path(os.environ.get("SCENSIM_WORLD_SERVER_PY",
                               REPO / "src" / "scensim_world_protocol" / "scensim_world_server"))
CHECK = os.environ.get("SCENSIM_WORLD_CHECK")
PYDEPS = os.environ.get("SCENSIM_TEST_PYDEPS", "")
PYPATH = ":".join(p for p in (PYDEPS, str(SHELL_PY), str(SCENSIM_SRC)) if p)


def _has_zmq():
    env = dict(os.environ, PYTHONPATH=PYPATH)
    return subprocess.run(["python3", "-c", "import zmq"], env=env,
                          capture_output=True).returncode == 0

pytestmark = pytest.mark.skipif(
    not ROS.is_file() or shutil.which("colcon") is None
    or not (SHELL_PY / "scensim_world_server").is_dir()
    or not (CHECK and Path(CHECK).is_file()) or not _has_zmq(),
    reason="needs ROS 2, colcon, SCENSIM_WORLD_CHECK, the protocol's Python shell and pyzmq")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def ws(tmp_path_factory):
    ws = tmp_path_factory.mktemp("own_world") / "ws"
    shutil.copytree(FIXTURE, ws)
    r = subprocess.run(["bash", "-c", f"source {ROS} && colcon build --packages-select "
                                      "fixture_world"], cwd=ws, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    return ws


def ws_command(ws, script):
    return ["bash", "-c", f"source {ROS} && source {ws}/install/setup.bash && "
                          f"export PYTHONPATH={PYPATH}:$PYTHONPATH && {script}"]


def in_ws(ws, script, timeout=180):
    return subprocess.run(ws_command(ws, script), cwd=ws, capture_output=True, text=True,
                          timeout=timeout)


def test_a_support_packages_world_server_passes_conformance(ws):
    port = free_port()
    r = in_ws(ws, "python3 -c \"from scensim import own_world; "
                  f"ok, report = own_world.conformance(['fixture_world', 'fake_world'], {port}, "
                  f"check=['{CHECK}'], start_timeout=60); print(report); "
                  "raise SystemExit(0 if ok else 1)\"")
    assert r.returncode == 0, r.stdout + r.stderr
    assert " 0 failed" in r.stdout
    assert "FAIL" not in r.stdout


def test_the_launch_file_starts_it_on_the_port_it_is_given(ws):
    port = free_port()
    log = open(ws / "launch.log", "w")
    launch = subprocess.Popen(
        ws_command(ws, f"exec ros2 launch {LAUNCH} package:=fixture_world "
                       f"executable:=fake_world port:={port}"),
        cwd=ws, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        r = subprocess.run([CHECK, "hello", f"tcp://127.0.0.1:{port}", "--wait", "60"],
                           capture_output=True, text=True, timeout=120)
    finally:
        for sig in (signal.SIGINT, signal.SIGKILL):   # its whole group: launch and server
            try:
                os.killpg(launch.pid, sig)
            except ProcessLookupError:
                break
            for _ in range(50):
                if launch.poll() is not None:
                    break
                time.sleep(0.2)
        launch.wait()
        log.close()
    assert r.returncode == 0, r.stdout + r.stderr + (ws / "launch.log").read_text()
    assert ": up, fake" in r.stdout
