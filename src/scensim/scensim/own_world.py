"""A world server from a support package (roadmap 081 step 5).

A testbench names its own world server, from a package in the user's workspace, in the
``ros2 run`` form:

    world:
      server: [my_world, my_world_server, --some-flag]     # <package>, <executable>, args...

scensim then starts that executable in place of the stock one (``scensim_carla_world`` or
``scensim_isaac_world``), in the stack's ego domain, and drives it over the world protocol
on the seat's world port, as it drives the stock servers. Every other ``world:`` key is the
stock CARLA server's and is refused beside ``server``.

**The contract** (what such a server must do; the how-to "Your world server in a
testbench", docs/how-to/your-world-server.md):

- take ``--port <port>`` (appended after its own arguments; also in ``SCENSIM_WORLD_PORT``)
  and bind the world protocol's REP socket on ``127.0.0.1:<port>`` at once, before any long
  start (the Python shell's ``bind``, the Rust shell's ``serve``);
- answer ``Hello`` while it starts, ready or not (the shells do), so ``scensim_world_check
  hello`` and the stack's readiness wait see it come up;
- end with its parent (``lifetime.die_with_parent()``) and on SIGINT/SIGTERM;
- pass ``scensim_world_check conformance`` (``testbench check --live`` runs it).

This module holds the key's rules and the live conformance run; starting the server in the
stack (``stack.py``, ``stack.launch.py``'s ``world_server`` argument and
``scensim_launch``'s ``world_server.launch.py``) is wired by 241's change.
"""
import os
import re
import signal
import subprocess
import time

from scensim.findings import Finding

KEY = "server"
NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
# the stock CARLA server's keys: they configure scensim_carla_world, not a server of one's own
STOCK_ONLY = ("config_file", "world_config_file")
CHECK = ["ros2", "run", "scensim_world_check", "scensim_world_check"]
HOST = "127.0.0.1"
# simulator: none -- scensim starts no simulator; the testbench's world server owns its own
NONE = "none"
# pilot kinds that ride a stock simulator's vehicle side (acb on CARLA, aib on Isaac)
NEEDS_VEHICLE_SIDE = ("autoware", "carla_basic")


def server_problems(world) -> list:
    """``(where, message)`` for ``world.server`` (empty: none, or a good one)."""
    if not isinstance(world, dict) or KEY not in world:
        return []
    server = world[KEY]
    where = f"world.{KEY}"
    if (not isinstance(server, list) or len(server) < 2
            or not all(isinstance(a, str) and a for a in server)):
        return [(where, "is not [<package>, <executable>, args...] "
                        "(e.g. [my_world, my_world_server])")]
    out = []
    pkg, exe = server[0], server[1]
    if not NAME.fullmatch(pkg):
        out.append((where, f"{pkg}: not a package name"))
    if not NAME.fullmatch(exe):
        out.append((where, f"{exe}: not an executable name"))
    if any(a == "--port" or a.startswith("--port=") for a in server[2:]):
        out.append((where, "--port is scensim's to give (the seat's world port): drop it"))
    for k in STOCK_ONLY:
        if k in world:
            out.append((f"world.{k}", f"is the stock CARLA server's; with world.{KEY}, your "
                                      "server takes its own settings as its arguments"))
    return out


def simulator_problems(settings) -> list:
    """``(where, message)`` for ``simulator: none`` (133, 2026-11-17): no simulator started,
    the world server named by ``world.server`` owns one. So ``server`` is required, and no
    pilot may need a stock simulator's vehicle side."""
    if not isinstance(settings, dict) or settings.get("simulator") != NONE:
        return []
    out = []
    world = settings.get("world")
    if not isinstance(world, dict) or KEY not in world:
        out.append(("simulator", f"{NONE} needs world.{KEY}: your world server, which owns "
                                 "the simulator (world: {server: [<package>, <executable>]})"))
    for name, pilot in (settings.get("pilots") or {}).items():
        kind = pilot.get("kind") if isinstance(pilot, dict) else None
        if kind in NEEDS_VEHICLE_SIDE:
            out.append((f"pilots.{name}", f"kind {kind} drives through CARLA's or Isaac's "
                                          f"vehicle side, which simulator: {NONE} does not "
                                          "start: use kind launch or command (your vehicle "
                                          "side), or kind none (no pilot)"))
    return out


def server_argv(server, port, command=None) -> list:
    """The command that starts ``server`` on ``port``: ``ros2 run`` of it, or ``command``
    (the tests' fixtures, which are no ROS package)."""
    head = list(command) if command else ["ros2", "run", server[0], server[1], *server[2:]]
    return [*head, "--port", str(port)]


def endpoint(port) -> str:
    return f"tcp://{HOST}:{port}"


def _stop(proc, grace=10.0):
    """The server's whole process group: SIGINT, then SIGTERM, then SIGKILL."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        end = time.time() + grace
        while time.time() < end:
            proc.poll()
            try:
                os.killpg(proc.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.2)


def conformance(server, port, *, check=None, start_timeout=60.0, env=None, log=None,
                command=None):
    """Start ``server`` on ``port``, wait for its ``Hello``, run the conformance check against
    it, stop it. ``(passed, report)``: the check's output, or why it did not run.
    ``check``: the ``scensim_world_check`` command (default ``ros2 run``); ``command``: the
    server's (``server_argv``)."""
    check = list(check or CHECK)
    run_env = dict(os.environ if env is None else env, SCENSIM_WORLD_PORT=str(port))
    logf = open(log, "w") if log else None
    try:
        proc = subprocess.Popen(server_argv(server, port, command), env=run_env,
                                stdout=logf or subprocess.DEVNULL, stderr=subprocess.STDOUT,
                                start_new_session=True)
    except OSError as e:
        if logf:
            logf.close()
        return False, f"{' '.join(server_argv(server, port, command))}: {e}"
    try:
        hello = subprocess.run(check + ["hello", endpoint(port), "--wait", str(start_timeout),
                                        "--quiet"], env=run_env, capture_output=True, text=True)
        if hello.returncode != 0:
            ended = proc.poll()
            why = (f"ended ({ended}) before it answered" if ended is not None
                   else f"did not answer Hello within {start_timeout:.0f} s")
            return False, (f"{server[0]} {server[1]} {why} on {endpoint(port)}"
                           f"{f' (its log: {log})' if log else ''}\n{hello.stdout}{hello.stderr}")
        r = subprocess.run(check + ["conformance", endpoint(port)], env=run_env,
                           capture_output=True, text=True)
        return r.returncode == 0, r.stdout + r.stderr
    finally:
        _stop(proc)
        if logf:
            logf.close()


def conformance_findings(server, port, **kw) -> list:
    """``conformance`` as findings: one per failed check, or one for a server that never came
    up. Empty: it passed."""
    passed, report = conformance(server, port, **kw)
    if passed:
        return []
    where = f"world.{KEY}"
    fails = [ln.strip()[len("FAIL"):].strip() for ln in report.splitlines()
             if ln.strip().startswith("FAIL")]
    fix = "see 'Your world server in a testbench' in scensim's docs: what a world server must do"
    if not fails:
        return [Finding(where, report.strip().splitlines()[0] if report.strip()
                        else "the conformance check failed", fix)]
    return [Finding(where, f"conformance: {f}", fix) for f in fails]


def hello_findings(port, *, check=None, env=None) -> list:
    """The stock world server of a live stack, on ``port``, probed with ``Hello`` (changes
    nothing; safe beside a run). A full conformance run is not possible there: the stack's
    bridge owns the server. Empty: it answered with this protocol version."""
    check = list(check or CHECK)
    try:
        r = subprocess.run(check + ["hello", endpoint(port), "--wait", "5"],
                           env=dict(os.environ if env is None else env),
                           capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired) as e:
        return [Finding("world", f"the stack's world server on {endpoint(port)}: the probe "
                                 f"did not run ({e})", "source scensim's setup.bash (it "
                                 "provides scensim_world_check)", host=True)]
    if r.returncode == 0:
        return []
    said = (r.stdout + r.stderr).strip().splitlines()
    return [Finding("world", f"the stack's world server on {endpoint(port)} did not answer "
                             "Hello" + (f": {said[-1]}" if said else ""),
                    "see the stack's world server log (scensim stack <testbench> --list "
                    "names the stack); restart the stack", host=True)]
