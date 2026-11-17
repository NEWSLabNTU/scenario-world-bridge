"""A run's ``timeline.jsonl``: one JSON object per line, ordered by wall time.

docs/design/projects-and-pilots.md, "Logs". Built from the runner's own events (run start and
end, pilot ready, result events), SSv2's milestones found in the scenario's logs, and every log
line at WARN or above in the slices. Each entry: ``{t_wall, t_sim, group, process, event, ...}``
with ``t_wall`` in seconds since the epoch. A log line without a time of its own (LibCarla's
``WARNING: ...``) takes the previous line's, else the slice's start, and says so
(``t_inferred``).

Both launchers' layouts are read (roadmap 079, step 4), one source per process, so a run's
timeline does not differ by launcher:

- play_launch: ``<stamp>/node/<name>/{out,err}`` (``load_node/<name>/`` for a component in
  its own process) per process; its own lines from ``console.log``, not ``play_launch.log``
  beside it, which repeats them.
- ``ros2 launch`` (ROS_LOG_DIR=<dir>): the launch's screen, ``console.log``, for every
  process: each of its lines ``[<name>-<n>] <line>`` is that process's, the rest the
  launch's own. rcl's ``<exe>_<pid>_<epoch ms>.log`` files and ``<stamp-host-pid>/
  launch.log`` hold copies of what the screen shows (every rcl line of a stack's pilot was
  in its console), but not what a process prints itself (Rust tracing, tracebacks), so
  they are read only where no console.log is beside them.

Process names are the same from both: no ``-<n>``, ``_<pid>_<epoch ms>`` or ``.py``; a
component's lines in a ``component_container*`` take its logger's last name, as play_launch
names its process.
"""
import datetime as _dt
import json
import re
from pathlib import Path

LEVELS = {"DEBUG": 10, "INFO": 20, "WARN": 30, "WARNING": 30, "ERROR": 40, "FATAL": 50,
          "CRITICAL": 50}

_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
# rclcpp/rclpy: [WARN] [1792452164.527513971] [node.name]: message
# (optionally after a ros2-launch prefix "[proc-1] ")
_ROS = re.compile(r"^(?:\[[^\]]+\]\s+)?\[(DEBUG|INFO|WARN|ERROR|FATAL)\]\s+\[(\d+(?:\.\d+)?)\]"
                  r"\s+\[([^\]]*)\]:\s?(.*)$")
# play_launch and other Rust tracing: 2026-10-19T23:22:41.612593Z  WARN target: message;
# Python logging: 2026-10-19 23:22:41,612 WARNING name: message
_ISO = re.compile(r"^(\d{4}-\d\d-\d\d[T ]\d\d:\d\d:\d\d(?:[.,]\d+)?(?:Z|[+-]\d\d:?\d\d)?)\s+"
                  r"\[?(TRACE|DEBUG|INFO|WARN|WARNING|ERROR|FATAL|CRITICAL)\]?\s+"
                  r"(?:([\w.:<>\-/]+):\s)?(.*)$")
# no time: LibCarla's "WARNING: ...", "ERROR: ..."
_BARE = re.compile(r"^(WARNING|WARN|ERROR|FATAL|CRITICAL):\s+(.*)$")
# ros2 launch's own: "[ERROR] [rviz2-32]: process has died [...]", "[INFO] [launch]: ..."
_LAUNCH = re.compile(r"^\[(DEBUG|INFO|WARNING|WARN|ERROR|FATAL)\]\s+\[([^\]]+)\]:\s?(.*)$")
# ros2 launch's screen: "[<name>-<n>] <the process's line>"
_PREFIX = re.compile(r"^\[([^\]\s]+)-(\d+)\]\s?(.*)$")
# launch.log's lines start with the epoch time they were logged at
_EPOCH = re.compile(r"^(\d{9,}\.\d+)\s+(?=\[)(.*)$")
# rcl's per-process log file: <exe>_<pid>_<epoch ms>.log
_RCL_FILE = re.compile(r"^(.+)_\d+_\d{13}$")
_LAUNCH_NUM = re.compile(r"^(.+)-\d+$")
# ros2 launch's own note of a process that ended other than cleanly, and of its shutdown
# beginning: Ctrl-C, a required process's exit ("... was required: shutting down launched
# system") or its other "shutting down" lines, logger [launch] exactly. Both at the start
# of a line: a nested launch's lines come prefixed ("[<name>-<n>] [WARNING] [launch]: ..."),
# and its stop and deaths are its own, as play_launch sees only the nested launch process.
# The crash counter (run.CrashWatch) ignores deaths after a shutdown began: Humble's rviz2
# (-6), Autoware's component containers and autoware_manual_lane_change_handler_node (-11)
# die of the SIGINT, as seen live 2026-11-16. The timeline keeps them, so a crash on exit
# stays visible.
DIED = re.compile(r"^\[ERROR\] \[([^\]\s]+-\d+)\]: process has died \[pid \d+, exit code (-?\d+)")
SHUTDOWN = re.compile(r"^\[(?:INFO|WARNING|ERROR)\] \[launch\]: (?:user interrupted with ctrl-c"
                      r"|.*\bshutting down\b)")
# A launch's first line: a console a respawned launch appends to holds one instance after
# another, each with its own stop and deaths
LAUNCH_START = re.compile(r"^\[INFO\] \[launch\]: All log files can be found below ")

# SSv2 milestones: (logger suffix or None, pattern, event)
_MILESTONES = [
    ("scenario_test_runner", re.compile(r"^Run (\S+) \((\d+) of (\d+)\)"), "ssv2.run"),
    ("simulation_clock", re.compile(r"^ROS time now follows the simulator"), "ssv2.clock"),
    (None, re.compile(r"^Localization reached the initial pose"), "ssv2.localized"),
    ("openscenario_interpreter", re.compile(r"^Passed$"), "ssv2.passed"),
    ("openscenario_interpreter", re.compile(r"^SimulationFailure: "), "ssv2.failed"),
    ("openscenario_interpreter", re.compile(r"^\w*Error: "), "ssv2.error"),
    ("scenario_test_runner", re.compile(r"^Shutdown interpreter"), "ssv2.shutdown"),
]


def parse_iso(s: str):
    """Seconds since the epoch for an ISO 8601 time; None if not one. A time with no zone is
    this host's local time: Python logging's ``asctime`` (the Isaac world server's lines) is
    written so, and the timeline is built on the host that wrote the logs. Read as UTC, those
    lines landed 28800 s late on a UTC+8 host (roadmap 077, G69)."""
    s = s.strip().replace(",", ".")
    if s.endswith("Z"):
        s = s[:-1] + "+00:00"
    m = re.match(r"^(.*\.\d{6})\d+(.*)$", s)  # Python takes at most microseconds
    if m:
        s = m.group(1) + m.group(2)
    try:
        t = _dt.datetime.fromisoformat(s)
    except ValueError:
        return None
    return t.timestamp()  # naive: local time


def to_epoch(t):
    if t is None or isinstance(t, bool):
        return None
    if isinstance(t, (int, float)):
        return float(t)
    if isinstance(t, str):
        try:
            return float(t)
        except ValueError:
            return parse_iso(t)
    return None


def parse_line(line: str):
    """``(level, t_wall or None, logger or None, message)`` for a recognisable log line, else
    None. Levels are normalised (WARNING -> WARN, CRITICAL -> FATAL)."""
    line = _ANSI.sub("", line).rstrip("\r\n")
    m = _ROS.match(line)
    if m:
        return m.group(1), float(m.group(2)), m.group(3), m.group(4).strip()
    m = _ISO.match(line)
    if m:
        return _norm(m.group(2)), parse_iso(m.group(1)), m.group(3), m.group(4).strip()
    m = _BARE.match(line.strip())
    if m:
        return _norm(m.group(1)), None, None, m.group(2).strip()
    m = _LAUNCH.match(line)
    if m:
        return _norm(m.group(1)), None, m.group(2), m.group(3).strip()
    return None


def process_name(name: str) -> str:
    """A process's name as both launchers give it: without ros2 launch's ``-<n>`` (some of
    play_launch's node directories carry one too), rcl's ``_<pid>_<epoch ms>``, ``.py``."""
    m = _RCL_FILE.match(name)
    if m:
        name = m.group(1)
    m = _LAUNCH_NUM.match(name)
    if m:
        name = m.group(1)
    return name[:-3] if name.endswith(".py") else name


def split_prefix(line: str):
    """``(process or None, rest)``: ros2 launch's ``[<name>-<n>] `` taken off a line."""
    m = _PREFIX.match(line)
    if m:
        return process_name(m.group(1)), m.group(3)
    return None, line


def _norm(level):
    level = level.upper()
    return {"WARNING": "WARN", "CRITICAL": "FATAL"}.get(level, level)


def milestone(logger, message):
    for suffix, pat, event in _MILESTONES:
        if suffix is not None and not (logger or "").endswith(suffix):
            continue
        if pat.search(message):
            return event
    return None


def _process_of(path: Path, base: Path) -> str:
    """The process a log file belongs to: play_launch's ``node/<name>/{out,err}`` gives the
    name; a slice ``<group>/<process>.log`` its stem."""
    rel = path.relative_to(base).parts
    if path.name in ("out", "err", "stdout", "stderr") and len(rel) >= 2:
        return process_name(rel[-2])
    return process_name(path.stem) if path.suffix == ".log" else path.name


def _line(raw, process):
    """``(process, parse_line result)`` for one raw line of ``process``'s file: a ros2
    launch prefix names the process; launch.log's epoch time stands in for a line's own."""
    raw = _ANSI.sub("", raw).rstrip("\r\n")
    stamp = None
    m = _EPOCH.match(raw)
    if m:
        stamp, raw = float(m.group(1)), m.group(2)
    who, rest = split_prefix(raw)
    p = parse_line(rest)
    if p is None:
        return process, None
    level, t, logger, msg = p
    if who is None and t is None and logger and _LAUNCH_NUM.match(logger):
        who = process_name(logger)  # launch's own line about a process: "process has died"
    who = who or process
    if who.startswith("component_container") and logger and t is not None:
        who = logger.rsplit(".", 1)[-1]  # a component: play_launch's name for its process
    return who, (level, t if t is not None else stamp, logger, msg)


def scan(files, group, base, default_t=None, min_level="WARN", milestones=True) -> list:
    """Timeline entries for log files: milestones and lines at ``min_level`` or above."""
    out = []
    floor = LEVELS[min_level]
    for path in files:
        process = _process_of(path, base)
        last_t = default_t
        try:
            fh = open(path, "r", errors="replace")
        except OSError:
            continue
        with fh:
            for line in fh:
                who, p = _line(line, process)
                if p is None:
                    continue
                level, t, logger, msg = p
                inferred = t is None
                if t is None:
                    t = last_t
                else:
                    last_t = t
                ev = milestone(logger, msg) if milestones else None
                if ev is None and LEVELS.get(level, 0) < floor:
                    continue
                e = {"t_wall": t, "t_sim": None, "group": group, "process": who,
                     "event": ev or f"log.{level.lower()}", "level": level, "message": msg}
                if logger:
                    e["logger"] = logger
                if inferred:
                    e["t_inferred"] = True
                out.append(e)
    return out


def _files(d: Path):
    return sorted(p for p in d.rglob("*") if p.is_file()) if d and d.is_dir() else []


_SKIP = {"cmdline", "pid", "status", "metadata.json", "metrics.csv", "run_info.json",
         "startup_timeline.json", "system_stats.csv", "diagnostics.csv", "health.jsonl",
         "dds_report.txt"}


def _log_files(d: Path):
    data = (".yaml", ".json", ".csv", ".jsonl")
    return sources([p for p in _files(d) if p.name not in _SKIP and p.suffix not in data])


def sources(files) -> list:
    """``files`` less the copies of a console.log beside them (module doc): rcl's per-process
    files in its directory, and the launch.log or play_launch.log one directory down."""
    consoles = {p.parent for p in files if p.name == "console.log"}
    out = []
    for p in files:
        if p.parent in consoles and p.suffix == ".log" and _RCL_FILE.match(p.stem):
            continue
        if p.name in ("launch.log", "play_launch.log") and p.parent.parent in consoles:
            continue
        out.append(p)
    return out


def run_event(e: dict) -> dict:
    """A runner event, normalised: ``event`` (or a result event's ``type``), ``t_wall`` as
    epoch seconds, ``group`` default ``run``."""
    out = dict(e)
    out["event"] = out.pop("event", None) or out.get("type") or "run.event"
    out["t_wall"] = to_epoch(out.get("t_wall"))
    out.setdefault("t_sim", None)
    out.setdefault("group", "run")
    if out["event"] == "stack.error":
        out.setdefault("level", "ERROR")
    elif out["event"] == "pilot.warning":  # a display-only process's crash (079 step 5)
        out.setdefault("level", "WARN")
    return out


def build(run_events=(), slices_dir=None, scenario_dir=None, default_t=None) -> list:
    """The timeline: ``run_events`` (dicts from the runner), the slices under ``slices_dir``
    (``<group>/<process>.log``; group ``scenario`` may be among them) and SSv2's own log
    directory ``scenario_dir`` (play_launch's ``node/<name>/{out,err}`` or plain files, group
    ``scenario``). ``default_t``: the time for log lines before any timed line (the run's
    start)."""
    tl = [run_event(e) for e in run_events]
    if default_t is None:
        times = [e["t_wall"] for e in tl if e["t_wall"] is not None]
        default_t = min(times) if times else None
    if slices_dir is not None:
        slices_dir = Path(slices_dir)
        if slices_dir.is_dir():
            for g in sorted(p for p in slices_dir.iterdir() if p.is_dir()):
                tl += scan(_log_files(g), g.name, g, default_t)
    if scenario_dir is not None:
        d = Path(scenario_dir)
        tl += scan(_log_files(d), "scenario", d, default_t)
    return order(tl)


def order(tl) -> list:
    """Stable by wall time; entries without one go last."""
    return sorted(tl, key=lambda e: (e.get("t_wall") is None, e.get("t_wall") or 0.0))


def write(tl, path) -> Path:
    path = Path(path)
    with open(path, "w") as f:
        for e in tl:
            f.write(json.dumps(e, sort_keys=False) + "\n")
    return path


def read(path) -> list:
    out = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if line:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return out


def is_problem(e, min_level="WARN") -> bool:
    return LEVELS.get(str(e.get("level", "")).upper(), 0) >= LEVELS[min_level]


# play_launch's notes on capabilities its helpers lack: never a run's reason (roadmap 077, G67)
_NOISE = re.compile(r"cap_sys_ptrace|\bgetcap\b|\bsetcap\b")
# scensim's own processes (the bridge and the relay, the world servers), and what they say when
# they refuse a scenario or fail it: "Cannot spawn 'x' ...", "... refused", "... failed", and
# a world server that stopped answering ("... did not answer Step within 10 s"), and its
# watchdog ending the blocked request ("... past its deadline ...", G70)
_SWB_GROUPS = {"bridge", "relay", "simulator"}
_REFUSAL = re.compile(r"\bCannot spawn\b|\brefus(?:e|es|ed)\b|\bfailed\b|\bdid not answer\b"
                      r"|\bpast its deadline\b", re.I)


def is_noise(e) -> bool:
    return bool(_NOISE.search(str(e.get("message") or "")))


def is_scensim_refusal(e) -> bool:
    return e.get("group") in _SWB_GROUPS and bool(_REFUSAL.search(str(e.get("message") or "")))


def first_problem(timeline, min_level="WARN"):
    """The first entry at ``min_level`` or above (a WARN/ERROR log line, a stack.error), but
    not play_launch's capability notes."""
    for e in timeline:
        if is_problem(e, min_level) and not is_noise(e):
            return e
    return None


def why(timeline, min_level="WARN"):
    """The entry that best says why a run ended as it did (`runs why`): the first refusal or
    failure that scensim's own processes logged at ``min_level`` or above, which outranks SSv2's
    generic error that follows it (a SemanticError naming no cause; roadmap 077, G67), else
    the first problem."""
    for e in timeline:
        if is_problem(e, min_level) and is_scensim_refusal(e):
            return e
    return first_problem(timeline, min_level)
