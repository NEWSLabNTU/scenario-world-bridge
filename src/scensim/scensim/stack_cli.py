"""``scensim stack <testbench> [--stop] | --list`` (roadmap 081 step 3): a testbench's stack
by hand. ``scensim run`` starts one when none serves the testbench and leaves it up; this
starts one without a run, stops one (the keeper first, then its launch's process group), or
lists the live stacks of this host's log root.
"""
import argparse
import os
import signal
import sys
import time
from pathlib import Path

from scensim import config
from scensim import run as runmod
from scensim import stack as stackmod

STOP_GRACE = 20.0


def live_stacks(logs) -> list:
    """``(project name, record)`` of every live stack under the log root."""
    out = []
    root = Path(logs).expanduser()
    for f in sorted(root.glob(f"*/seat-*-current/{stackmod.STACK_JSON}")):
        try:
            rec = stackmod.read_json(f)
        except (OSError, ValueError):
            continue
        if runmod.stack_alive(rec):
            out.append((f.parent.parent.name, rec))
    return out


def _gone(pgid) -> bool:
    try:
        os.killpg(pgid, 0)
        return False
    except ProcessLookupError:
        return True
    except PermissionError:
        return False


def stop_live(rec: dict, grace=STOP_GRACE) -> bool:
    """Stop a live stack: its keeper (pilots and CARLA in order), then its launch's process
    group (the launch runs in a session of its own). True when it is gone."""
    pid = rec.get("pid")
    try:
        pgid = os.getpgid(pid) if pid else None
    except ProcessLookupError:
        pgid = None
    stackmod.stop_keeper(rec)
    if not pgid or pgid == os.getpgid(0):
        return True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        end = time.time() + grace
        while time.time() < end:
            if _gone(pgid):
                return True
            time.sleep(0.5)
    return _gone(pgid)


def _line(name, rec) -> str:
    return (f"{name:24} seat {rec['seat']['seat']}  {rec.get('stack_id')}  "
            f"{rec.get('state')}  (keeper pid {rec.get('pid')})")


def main(argv=None, out=None) -> int:
    out = out or sys.stdout
    ap = argparse.ArgumentParser(prog="scensim stack", description=__doc__.split("\n")[0])
    ap.add_argument("testbench", nargs="?", help="a testbench by name, or its directory")
    what = ap.add_mutually_exclusive_group()
    what.add_argument("--stop", action="store_true", help="stop its stack")
    what.add_argument("--list", action="store_true", help="the live stacks (of the testbench)")
    ap.add_argument("--seat", help="the stack's seat (default: the host settings' seat:)")
    ap.add_argument("--logs", help="the log root (default: the host settings' logs:)")
    ap.add_argument("--stack-timeout", type=float,
                    help="seconds a started stack has to become ready")
    try:
        args = ap.parse_args(argv)
    except SystemExit as e:
        return 0 if e.code == 0 else runmod.EXIT_REFUSED
    over = {"logs": args.logs} if args.logs else {}
    try:
        host = config.host_settings(None, over)
    except config.ConfigError as e:
        sys.stderr.write(f"scensim stack: refused: {e}\n")
        return runmod.EXIT_REFUSED
    if args.list or not args.testbench:
        if not args.list:
            sys.stderr.write("scensim stack: name a testbench, or give --list\n")
            return runmod.EXIT_REFUSED
        rows = live_stacks(host["logs"])
        if args.testbench:
            rows = [r for r in rows if r[0] == Path(args.testbench).name]
        for name, rec in rows:
            out.write(_line(name, rec) + "\n")
        if not rows:
            out.write("no live stack\n")
        return runmod.EXIT_OK
    try:
        found = runmod.open_testbench(args.testbench, args.logs)
        if found is None:
            raise runmod.Refused(f"{args.testbench}: not a testbench (a name in the sourced "
                                 "workspace, or a directory holding testbench.yaml)")
        tb, project, _ = found
        tbinfo = {"name": tb.name}
        if args.stop:
            try:
                rec = runmod.find_stack(project, config.host_settings(project, dict(
                    over, **({"seat": args.seat} if args.seat else {}))))
            except runmod.NoStack:
                out.write(f"stack    {tbinfo['name']}: none is up\n")
                return runmod.EXIT_OK
            ok = stop_live(rec)
            out.write(f"stack    {tbinfo['name']} on seat {rec['seat']['seat']}: "
                      f"{'stopped' if ok else 'did not stop: see its console'}\n")
            return runmod.EXIT_OK if ok else runmod.EXIT_INFRA
        ctx = runmod.context(str(project), args.seat, args.logs, start=True,
                             stack_timeout=args.stack_timeout, out=out)
        rec = ctx.stack
        how = "started" if ctx.started is not None else "already up"
        out.write(f"stack    {tbinfo['name']} on seat {rec['seat']['seat']}: {how}; "
                  f"stop it with: scensim stack {tbinfo['name']} --stop\n")
        return runmod.EXIT_OK
    except runmod.Refused as e:
        sys.stderr.write(f"scensim stack: refused: {e}\n")
        return runmod.EXIT_REFUSED
    except runmod.StackFailed as e:
        sys.stderr.write(f"scensim stack: error: {e}\n")
        return runmod.EXIT_INFRA


if __name__ == "__main__":
    sys.exit(main())
