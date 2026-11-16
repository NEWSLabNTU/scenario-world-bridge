"""stack.stop_processes (roadmap 079, step 3): a launcher is stopped through its process
group, and the stop is over only when nothing of the group is left: `ros2 launch` may exit
on SIGINT before its children, which must not survive it."""
import os
import sys
import time

from swb import stack

# a launcher that exits at once on SIGINT, leaving a child that ignores SIGINT and SIGTERM
LEADER = r"""
import os, signal, subprocess, sys, time
child = subprocess.Popen([sys.executable, "-c",
    "import signal, time\n"
    "signal.signal(signal.SIGINT, signal.SIG_IGN)\n"
    "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
    "open(%r, 'w').write(str(__import__('os').getpid()))\n"
    "time.sleep(600)\n" % sys.argv[1]])
signal.signal(signal.SIGINT, lambda *_: os._exit(0))
time.sleep(600)
"""


def test_a_child_the_leader_leaves_behind_is_stopped(tmp_path):
    pidfile = tmp_path / "child.pid"
    proc = stack.start_process([sys.executable, "-c", LEADER, str(pidfile)], dict(os.environ),
                               tmp_path / "log")
    deadline = time.time() + 20
    while not (pidfile.exists() and pidfile.read_text()) and time.time() < deadline:
        time.sleep(0.05)
    child = int(pidfile.read_text())
    assert os.getpgid(child) == proc.pid
    t0 = time.time()
    stack.stop_processes([proc], grace=1.0)
    assert proc.poll() is not None
    assert not stack._group_alive(proc.pid)
    assert not os.path.exists(f"/proc/{child}") or open(f"/proc/{child}/stat").read().split()[2] == "Z"
    assert time.time() - t0 < 1.0 + 10.0 + 5.0 + 2.0  # SIGINT, SIGTERM, SIGKILL


def test_a_group_already_gone_is_nothing_to_stop(tmp_path):
    proc = stack.start_process([sys.executable, "-c", "pass"], dict(os.environ), tmp_path / "l")
    proc.wait()
    t0 = time.time()
    stack.stop_processes([proc], grace=5.0)
    assert time.time() - t0 < 1.0


def test_a_launcher_gets_sigint_at_its_default_whatever_started_swb(tmp_path):
    """`swb run ... &` from a script ignores SIGINT, and an ignored signal survives exec:
    `ros2 launch` would then install no handler and ignore the stop's SIGINT."""
    import signal
    out = tmp_path / "disposition"
    code = ("import signal; open(%r, 'w').write(str(signal.getsignal(signal.SIGINT)))"
            % str(out))
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        for kw in ({}, {"die_with_parent": True}):
            proc = stack.start_process([sys.executable, "-c", code], dict(os.environ),
                                       tmp_path / "log", **kw)
            proc.wait(timeout=20)
            assert out.read_text() == str(signal.default_int_handler)
    finally:
        signal.signal(signal.SIGINT, previous)
