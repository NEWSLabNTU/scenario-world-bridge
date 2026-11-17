"""A stack node in a crash loop before the stack is ready (roadmap 077 G77).

The stack's world server and simulation side are nested launches whose nodes respawn
(``respawn="true"``): a world server that dies at its start is started again, without end. On
118's fresh host one exited 1 with the same line 180 times in 1800 s, and another segfaulted at
Kit's renderer init every ~14 s, while ``scensim run`` said only "not ready within 1800 s
(starting)". This reads each node's exits from the stack's launch logs, under either launcher
(roadmap 079: the answer must not differ by launcher):

- play_launch: each ``<stamp>/play_launch.log``'s ``[node:/<name>] Exited with code N`` and
  ``Exited without code`` lines (always logged; its DEBUG ``State event: Exited {...,
  signal: Some(S)}`` lines, when there, give the signal of an exit without a code); the
  dead instance's stderr is ``node/<name>/err.<k>`` for its k-th exit (play_launch >= 0.18.3
  keeps it, CLAUDE.md);
- ``ros2 launch``: a ``console.log`` whose ``[ERROR] [<exe>-<n>]: process has died [pid P,
  exit code N, ...]`` lines name each death; the process's own lines are prefixed ``[<exe>-<n>]
  `` (a respawned process keeps its name).

Both for the stack's own launch (its log directory, beside the console ``scensim run`` gives it:
the nested ``simulator`` and ``bridge`` launches are its nodes, and a nested launch that
cannot even parse exits 1 at once, again and again) and for the nested launches' own
directories under the stack's ``proc/`` (the world server, the bridge, the relay).

Every exit counts (an exit code or a signal: 1, 139, -11 alike), since the stack is not ready
yet: nothing it runs has been healthy. ``LIMIT`` (5) exits of one node end the start, naming
the node, how it exited, its last stderr line and, when Kit wrote one, its crash dump.
"""
import re
import signal
from pathlib import Path

LIMIT = 5   # a stack node's exits before the stack is ready, before `scensim run` gives up
SIDES = ("simulator", "bridge")   # stack.launch.py's nested launches under the stack's proc/

_PL_EXITED = re.compile(r"\[node:/?([^\]]+)\] Exited (?:with code (-?\d+)|without code)")
_PL_STATE = re.compile(r'State event: Exited \{ name: "node:/?([^"]+)", exit_code: '
                       r'(?:Some\((-?\d+)\)|None), signal: (?:Some\((\d+)\)|None)')
_ERRORISH = re.compile(r"error|Error|ERROR|Traceback|FATAL|fatal|Segmentation|Abort|refused")
_R2_DIED = re.compile(r"^\[ERROR\] \[([^\]\s]+-\d+)\]: process has died \[pid \d+, "
                      r"exit code (-?\d+)")
_R2_SHUTDOWN = re.compile(r"^\[(?:INFO|WARNING|ERROR)\] \[launch\]: (?:user interrupted with "
                          r"ctrl-c|.*\bshutting down\b)")
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
# Kit's crash reporter: "... crash dump ... /path/<uuid>.dmp" (its dumps' directory is named)
_DUMP = re.compile(r"(/\S+\.dmp)\b")


def how(code=None, sig=None) -> str:
    """'exited 1', 'exited 139 (SIGSEGV)', 'exited -11 (SIGSEGV)', 'killed by signal 9
    (SIGKILL)': a shell's 128+N and ros2 launch's -N name the signal too."""
    if sig is not None:
        return f"killed by signal {sig} ({_signame(sig)})"
    if code is None:
        return "exited without a code"
    n = -code if code < 0 else code - 128 if 128 < code < 160 else 0
    return f"exited {code} ({_signame(n)})" if n else f"exited {code}"


def _signame(n: int) -> str:
    try:
        return signal.Signals(n).name
    except ValueError:
        return f"signal {n}"


def _last_line(text: str) -> str:
    """The last line that reads as an error among the last ten, else the last line: a
    hint printed after the error (play_launch's `--parser python` one) is not the cause."""
    lines = [x.strip() for x in _ANSI.sub("", text).splitlines() if x.strip()]
    for line in reversed(lines[-10:]):
        if _ERRORISH.search(line):
            return line
    return lines[-1] if lines else ""


def _dump(text: str) -> str:
    m = None
    for m in _DUMP.finditer(text):
        pass
    return str(Path(m.group(1)).parent) if m else ""


def _read(path: Path) -> str:
    try:
        return Path(path).read_text(errors="replace")
    except OSError:
        return ""


def _play_launch(launch_dir: Path) -> list:
    out = []
    for log in sorted(Path(launch_dir).glob("*/play_launch.log")):
        if log.parent.is_symlink():
            continue   # `latest`: one of the stamped directories again
        text = _read(log)
        signals = {}   # node -> the signals of its exits without a code, in order
        for m in _PL_STATE.finditer(text):
            if m.group(2) is None:
                signals.setdefault(m.group(1), []).append(int(m.group(3)) if m.group(3)
                                                          else None)
        seen, nocode = {}, {}
        for m in _PL_EXITED.finditer(text):
            name = m.group(1)
            seen[name] = k = seen.get(name, 0) + 1
            sig = None
            if m.group(2) is None:
                nocode[name] = j = nocode.get(name, 0) + 1
                sigs = signals.get(name, [])
                sig = sigs[j - 1] if j <= len(sigs) else None
            node_dir = log.parent / "node" / name.rsplit("/", 1)[-1]
            # play_launch renames the dead instance's err to err.<k> at its respawn; until
            # then (the last exit) it is still `err`
            suffix = f".{k}" if (node_dir / f"err.{k}").exists() else ""
            err = _read(node_dir / f"err{suffix}")
            out.append({"node": name, "code": int(m.group(2)) if m.group(2) else None,
                        "signal": sig, "last": _last_line(err),
                        "dump": _dump(err + _read(node_dir / f"out{suffix}")),
                        "log": str(node_dir / f"err{suffix}")})
    return out


def _ros2(console: Path) -> list:
    out, last, dumps = [], {}, {}
    for line in _ANSI.sub("", _read(console)).splitlines():
        if _R2_SHUTDOWN.match(line):
            break   # the stop's deaths, not the node's
        m = _R2_DIED.match(line)
        if m:
            name = m.group(1)
            out.append({"node": re.sub(r"-\d+$", "", name), "code": int(m.group(2)),
                        "signal": None, "last": last.get(name, ""),
                        "dump": dumps.get(name, ""), "log": str(console)})
            continue
        if line.startswith("["):
            name, _, rest = line[1:].partition("] ")
            if rest and re.fullmatch(r"\S+-\d+", name):
                if rest.strip():
                    last[name] = rest.strip()
                d = _dump(rest)
                if d:
                    dumps[name] = d
    return out


def exits(proc: Path = None, launch_dir: Path = None) -> list:
    """Each stack node's exits, in order per node: ``{node, code, signal, last, dump, log}``,
    from whichever layout each directory holds: the stack's own launch (``launch_dir``, whose
    console.log is ros2 launch's screen) and each nested side under ``proc``."""
    out = []
    dirs = [Path(launch_dir)] if launch_dir else []
    dirs += [Path(proc) / side for side in SIDES] if proc else []
    for d in dirs:
        out += _play_launch(d)
        if (d / "console.log").is_file():
            out += _ros2(d / "console.log")
    return out


def looping(proc: Path = None, limit: int = LIMIT, launch_dir: Path = None):
    """The message for the first node with ``limit`` exits, else None."""
    count = {}
    for e in exits(proc, launch_dir):
        count[e["node"]] = n = count.get(e["node"], 0) + 1
        if n >= limit:
            msg = (f"stack node {e['node']} {how(e['code'], e['signal'])} {n} times before the "
                   f"stack was ready (a crash loop: its launch respawns it)")
            if e["last"]:
                msg += f"; its last stderr line: {e['last']}"
            if e["dump"]:
                msg += f"; crash dump in {e['dump']}"
            return msg + f" ({e['log']})"
    return None
