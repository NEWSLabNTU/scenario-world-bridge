"""stack.stop_processes (roadmap 079, step 3): a launcher is stopped through its process
group, and the stop is over only when nothing of the group is left: `ros2 launch` may exit
on SIGINT before its children, which must not survive it."""
import os
import sys
import time

from scensim import stack

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
    """`scensim run ... &` from a script ignores SIGINT, and an ignored signal survives exec:
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


# a keeper: on SIGINT it "stops its pilots and CARLA" (sleeps), says stack.stop, exits
KEEPER = r"""
import json, signal, sys, time
d = sys.argv[1]
def stop(*_):
    time.sleep(float(sys.argv[2]))
    if sys.argv[3] == "says":
        open(d + "/events.jsonl", "a").write(json.dumps({"event": "stack.stop"}) + "\n")
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
open(d + "/events.jsonl", "a").write(json.dumps({"event": "stack.ready"}) + "\n")
time.sleep(600)
"""


def _keeper(tmp_path, delay, says="says"):
    (tmp_path / "proc").mkdir(exist_ok=True)
    proc = stack.start_process([sys.executable, "-c", KEEPER, str(tmp_path), str(delay), says,
                                "scensim.stack", "keep"], dict(os.environ), tmp_path / "klog")
    deadline = time.time() + 20
    while not (tmp_path / "events.jsonl").exists() and time.time() < deadline:
        time.sleep(0.05)
    return proc, {"pid": proc.pid, "proc": str(tmp_path / "proc"), "stack_id": "s"}


def test_the_keeper_is_asked_first_and_its_stop_is_waited_for(tmp_path):
    proc, rec = _keeper(tmp_path, 2.0)
    stack.event(tmp_path, "stack.stopping")
    t0 = time.time()
    assert stack.stop_keeper(rec, timeout=20) is True
    assert time.time() - t0 >= 2.0   # it waited for the keeper's own stop
    assert not stack.stop_incomplete(tmp_path)
    proc.wait(timeout=10)


def test_a_stop_the_keeper_did_not_finish_is_incomplete(tmp_path):
    proc, rec = _keeper(tmp_path, 30.0)
    stack.event(tmp_path, "stack.stopping")
    assert stack.stop_keeper(rec, timeout=1.0) is False
    assert stack.stop_incomplete(tmp_path)
    stack.stop_processes([proc], grace=0.5)


def test_a_pid_that_is_not_a_keeper_is_never_signalled(tmp_path):
    other = stack.start_process([sys.executable, "-c", "import time; time.sleep(30)"],
                                dict(os.environ), tmp_path / "o")
    try:
        assert stack.stop_keeper({"pid": other.pid, "proc": str(tmp_path / "proc")}) is True
        assert other.poll() is None
    finally:
        stack.stop_processes([other], grace=0.5)
