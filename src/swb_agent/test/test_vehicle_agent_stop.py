"""077 G72: the vehicle agent ends on SIGINT and on SIGTERM, exit 0, in well under
play_launch's 2 s + 5 s to SIGKILL, and without aborting on its way out ("terminate called
without an active exception"). A real process with a live rclpy context, as a launch runs it."""
import os
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest

vehicle_agent = pytest.importorskip("swb_agent.adapters.vehicle_agent")


def _free_port():
    with socket.socket() as s:   # nothing listens there once it is closed: the relay is away
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM])
def test_the_agent_ends_cleanly_on_a_stop_signal(tmp_path, sig):
    env = dict(os.environ, ROS_DOMAIN_ID="57", ROS_LOG_DIR=str(tmp_path))
    log = tmp_path / "agent.log"
    with open(log, "w") as out:
        p = subprocess.Popen([sys.executable, "-m", "swb_agent.adapters.vehicle_agent",
                              "--ros-args", "-p", f"relay:=tcp://127.0.0.1:{_free_port()}"],
                             env=env, stdout=out, stderr=subprocess.STDOUT)
    try:
        deadline = time.time() + 60
        # up: its node, its executor, its step thread, and the relay client retrying
        while "retrying" not in log.read_text() and time.time() < deadline:
            assert p.poll() is None, log.read_text()
            time.sleep(0.2)
        time.sleep(1.0)
        t0 = time.time()
        p.send_signal(sig)
        rc = p.wait(timeout=vehicle_agent.CLOSE_BUDGET + 2.0)
        took = time.time() - t0
    finally:
        if p.poll() is None:
            p.kill()
    text = log.read_text()
    assert rc == 0, text
    assert took < vehicle_agent.CLOSE_BUDGET + 1.0
    assert "terminate called" not in text and "Traceback" not in text, text


def test_a_stuck_step_costs_the_budget_and_no_more():
    stuck = threading.Event()
    done = []
    t0 = time.time()
    ok = vehicle_agent.close_in_order([("fine", lambda: done.append("fine")),
                                       ("stuck", lambda: stuck.wait(30)),
                                       ("never", lambda: done.append("never"))], budget=0.5)
    assert ok is False and done == ["fine"]
    assert time.time() - t0 < 1.5


def test_a_failing_step_does_not_stop_the_ones_after_it():
    done = []

    def boom():
        raise RuntimeError("context is invalid")

    assert vehicle_agent.close_in_order([("boom", boom), ("next", lambda: done.append(1))])
    assert done == [1]


def test_the_close_budget_is_well_under_play_launchs_kill():
    assert vehicle_agent.CLOSE_BUDGET + vehicle_agent.JOIN_TIMEOUT < 5.0


def test_a_run_that_raises_still_closes_in_order(tmp_path):
    """An exception out of the run (a relay protocol error, an adapter bug) goes through the
    same close: exit 1, the traceback once, no abort on the way out."""
    code = ("import sys, swb_agent.adapters.vehicle_agent as v\n"
            "def boom(node, stop):\n"
            "    raise RuntimeError('relay protocol error')\n"
            "v.run_relay = boom\n"
            "v.main()\n")
    env = dict(os.environ, ROS_DOMAIN_ID="57", ROS_LOG_DIR=str(tmp_path))
    p = subprocess.run([sys.executable, "-c", code, "--ros-args", "-p",
                        f"relay:=tcp://127.0.0.1:{_free_port()}"], env=env,
                       capture_output=True, text=True, timeout=60)
    text = p.stdout + p.stderr
    assert p.returncode == 1, text
    assert text.count("Traceback") == 1 and "relay protocol error" in text
    assert "the agent failed" in text and "terminate called" not in text, text
