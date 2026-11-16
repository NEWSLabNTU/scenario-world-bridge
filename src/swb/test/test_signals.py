"""SIGINT inherited ignored (a script's `cmd &` job): `swb run` still stops on it (077 before
the RC, the 079 F1 class; 118's ladder ignored its group's SIGINT for 5 minutes)."""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from swb import signals

SRC = str(Path(__file__).resolve().parents[1])

# swb run's cli around a main that waits, as a run waits on its stack and SSv2
CHILD = """
import sys, time
from swb import run
def main(argv=None):
    print("waiting", flush=True)
    time.sleep(60)
    return 0
run.main = main
sys.exit(run.cli([]))
"""


def _ignore_sigint():
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # as a non-interactive shell's `&` job


def test_take_sigint_replaces_an_inherited_ignore():
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        assert signals.take_sigint() is signal.SIG_IGN
        assert signal.getsignal(signal.SIGINT) is signal.default_int_handler
    finally:
        signal.signal(signal.SIGINT, previous)


def test_swb_run_stops_on_sigint_inherited_ignored():
    env = dict(os.environ, PYTHONPATH=SRC + os.pathsep + os.environ.get("PYTHONPATH", ""))
    p = subprocess.Popen([sys.executable, "-c", CHILD], env=env, preexec_fn=_ignore_sigint,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert p.stdout.readline().strip() == "waiting"
        with open(f"/proc/{p.pid}/status") as f:   # the child's own record: SIGINT taken
            sigign = int(next(x for x in f if x.startswith("SigIgn:")).split()[1], 16)
        assert not sigign & (1 << (signal.SIGINT - 1))
        t0 = time.time()
        p.send_signal(signal.SIGINT)
        code = p.wait(timeout=10)
    finally:
        if p.poll() is None:
            p.kill()
    assert code == 130 and time.time() - t0 < 10
    assert "swb run: interrupted; what this run started is stopped" in p.stderr.read()
