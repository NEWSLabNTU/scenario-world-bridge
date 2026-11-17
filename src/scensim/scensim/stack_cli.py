"""``scensim stack <testbench> [--foreground|--stop] | --list``: a testbench's stack
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


def foreground_argv(tb, project: Path, host: dict, seat, log_dir: Path) -> list:
    """The stack as a user would start it by hand, on ``seat``: ``testbench:=<name>`` for an
    installed testbench (its work directory is resolved by the launch, never typed),
    ``project:=<dir>`` for one given as a directory."""
    from scensim import launcher as launchermod, tools as toolsmod
    args = {"testbench": tb.name} if tb.installed else {"project": project}
    args.update(seat=seat.index, logs=host["logs"])
    name = toolsmod.launcher(host.get("launcher"))[0]
    return launchermod.command(name, ["scensim_launch", "scensim.launch.xml"], args, log_dir,
                               web_port=seat.web_port, diagnostics=True)


def run_foreground(argv, env, out, popen=None, grace=STOP_GRACE) -> int:
    """The stack in this terminal, in a process group of its own; Ctrl-C (or SIGTERM) stops
    that group: SIGINT (the keeper stops the pilots and CARLA in order), then SIGTERM, then
    SIGKILL, ``grace`` s apart."""
    import subprocess
    proc = (popen or subprocess.Popen)(argv, env=env, start_new_session=True)
    previous = signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        return proc.wait()
    except KeyboardInterrupt:
        out.write("stack    stopping (Ctrl-C): the keeper first, then the launch\n")
        out.flush()
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=grace)
                break
            except Exception:  # noqa: BLE001 (TimeoutExpired: the next signal)
                continue
        return 130
    finally:
        signal.signal(signal.SIGTERM, previous)


def _line(name, rec) -> str:
    return (f"{name:24} seat {rec['seat']['seat']}  {rec.get('stack_id')}  "
            f"{rec.get('state')}  (keeper pid {rec.get('pid')})")


def main(argv=None, out=None) -> int:
    out = out or sys.stdout
    ap = argparse.ArgumentParser(prog="scensim stack", description="A testbench's stack by hand: start it (and leave it up), run it in this terminal, stop it, or list the live ones.")
    ap.add_argument("testbench", nargs="?", help="a testbench by name, or its directory")
    what = ap.add_mutually_exclusive_group()
    what.add_argument("--stop", action="store_true", help="stop its stack")
    what.add_argument("--list", action="store_true", help="the live stacks (of the testbench)")
    what.add_argument("--foreground", action="store_true",
                      help="run the stack in this terminal, its log here; Ctrl-C stops it")
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
        if args.foreground:
            host = config.host_settings(project, dict(
                over, **({"seat": args.seat} if args.seat not in (None, "") else {})))
            try:
                rec = runmod.find_stack(project, host)
                out.write(f"stack    {tbinfo['name']} is already up on seat "
                          f"{rec['seat']['seat']}: stop it first (scensim stack "
                          f"{tbinfo['name']} --stop)\n")
                return runmod.EXIT_REFUSED
            except runmod.NoStack:
                pass
            seat, log_dir = runmod.prepare_stack(project, host)
            argv = foreground_argv(tb, project, host, seat, log_dir)
            out.write(f"stack    {tbinfo['name']} on seat {seat.index} in the foreground "
                      f"(web UI on port {seat.web_port}); Ctrl-C stops it\n")
            out.flush()
            return run_foreground(argv, runmod.stack_env(seat, host=host), out)
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
