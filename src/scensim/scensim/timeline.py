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
import os
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

# A composable that did not load (issue 064): its container runs and the pilot's other topics
# come up, so a readiness check on topics passes while a node is missing (088 on swb 133: a
# CUDA OOM in occupancy_grid_map_node; every run then waited for an engage that never came).
# play_launch says so per composable (LOAD_FAILED) and at the end of its start
# (STARTUP_SUMMARY, the authority once there); ros2 launch through launch_ros.
PLAY_LAUNCH_START = re.compile(r"\bStep 1/\d+: Parsing launch file")
_LOAD_FAILED = re.compile(r"LOAD_FAILED for 'composable:([^']+)': (.*)$")
_STARTUP_SUMMARY = re.compile(r"\bSTARTUP_SUMMARY (\{.*\})\s*$")
_ROS_LOAD_FAILED = re.compile(r"Failed to load node '([^']+)' of type '[^']+' in container "
                              r"'([^']+)': (.*)$")


def _console_text(path, offset=0) -> str:
    """A console from byte ``offset`` on (a pilot's restart appends to it: what came before the
    restart is the previous launch's), ANSI colours removed; "" when unreadable."""
    try:
        with open(path, "rb") as f:
            f.seek(offset)
            return _ANSI.sub("", f.read().decode(errors="replace"))
    except OSError:
        return ""


def missing_composables(path, offset=0) -> str:
    """The composables a pilot's launch (its console at ``path``) declared and did not load,
    in its current instance (after its last start: a respawned launch appends; ``offset``: the
    console's size when the pilot was last started), each with why; "" when none or when the
    console says nothing of it yet."""
    text = _console_text(path, offset)
    lines = text.splitlines()
    start = 0
    for i, line in enumerate(lines):
        if LAUNCH_START.match(line) or PLAY_LAUNCH_START.search(line):
            start = i
    failed, summary = {}, None
    for line in lines[start:]:
        m = _LOAD_FAILED.search(line)
        if m:
            failed[m.group(1)] = m.group(2).strip()
            continue
        m = _ROS_LOAD_FAILED.search(line)
        if m:
            name = m.group(1) if m.group(1).startswith("/") else f"{m.group(2)}/{m.group(1)}"
            failed[name] = m.group(3).strip()
            continue
        m = _STARTUP_SUMMARY.search(line)
        if m:
            try:
                summary = json.loads(m.group(1))
            except ValueError:
                pass
    if summary is not None:
        # the summary is play_launch's last word on this start: one LOAD_FAILED a retry then
        # loaded is not missing, and one the summary names is, with the reason found above
        missing = {}
        for short in summary.get("composable_shortfall") or []:
            for name in short.get("missing") or []:
                missing[name] = failed.get(name) or (
                    f"not loaded in {short.get('container')}, {short.get('loaded')} of "
                    f"{short.get('declared')}")
        failed = missing
    if not failed:
        return ""
    return "; ".join(f"{name} ({why[:200]})" for name, why in failed.items())

# A composable load that never ends (issue 088): launch_ros (Humble) asks a container to load its
# composables one request at a time and waits for each reply with no time limit. When a reply is
# lost (the container's "failed to send response to .../_container/load_node"), the launch waits
# for ever, the composables after it are never requested, and nothing failed, so 064's check sees
# nothing. Under ros2 launch the console has both sides: the container logs each load it takes
# ("Instantiate class: ..."), launch_ros each reply it got ("Loaded node '<n>' in container '<c>'",
# or "Failed to load node"). A container with more loads taken than answered has one outstanding.
# play_launch (0.35) loads through its own container, over a private channel or LoadNode, and
# logs no reply per load; a load whose reply it lost it says so ("LoadNode response lost for
# composable:<n> -- deferring to ComponentEvent"), and every 10 s how many are loaded ("Startup:
# ... composable <a>/<b> loaded (<k> pending)"). With its container frozen it waited 12 minutes
# so, its busy_budget (600 s) notwithstanding, and never wrote LOAD_FAILED or STARTUP_SUMMARY (139,
# 2026-11-21): nothing for 064's check. So the same rule reads those lines.
_INSTANTIATE = re.compile(r"^\[([^\]]+)\]\s+\[INFO\]\s+\[(\d+(?:\.\d+)?)\]\s+\[([^\]]+)\]:\s+"
                          r"Instantiate class: (.*)$")
_LOADED = re.compile(r"Loaded node '[^']+' in container '([^']+)'")
_PL_TIME = re.compile(r"^(\d{4}-\d\d-\d\dT\S+Z)\s")
_PL_LOST = re.compile(r"\bWARN container:(\S+): LoadNode response lost for composable:(\S+)")
_PL_PROGRESS = re.compile(r"\bINFO Startup: .*\bcomposable (\d+)/(\d+) loaded \((\d+) pending\)")
_PL_DONE = re.compile(r"\bStartup complete\b|\bSTARTUP_SUMMARY ")
_LOST_REPLY = re.compile(r"^\[[^\]]+\]\s+\[WARN\]\s+\[(\d+(?:\.\d+)?)\]\s+\[[^\]]+\]:\s+"
                         r"failed to send response to (\S+)/_container/load_node\b")
LOST_GRACE = 30.0   # s after a lost reply: the launch will not get it; a little slack for ordering
PL_LOST_GRACE = 120.0  # s under play_launch: beyond its own retry (60 s), which may still recover
_PID = re.compile(r"^\[INFO\] \[([^\]]+)\]: process started with pid \[(\d+)\]")
BUSY_CPU = 0.2      # s of CPU in BUSY_SAMPLE s: a container still constructing (a TensorRT build, 3-11
BUSY_SAMPLE = 2.0   # minutes on a loaded host before 10.8's cache) is busy; a blocked one is idle (243;
                    # 2 s, not one quiet half-second of a build that waits on the GPU)
LOAD_STALL = 600.0  # s a load may stay unanswered without a lost reply logged (065: a TensorRT build
                    # takes up to 240 s inside a constructor, a first CenterPoint start 189 s)


def _cpu_s(pid: int):
    """utime + stime of process ``pid`` in seconds, None if it is gone."""
    try:
        f = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(f[11]) + int(f[12])) / os.sysconf("SC_CLK_TCK")
    except (OSError, IndexError, ValueError):
        return None


def _cpu_busy(pid: int) -> bool:
    """The process used at least BUSY_CPU s of CPU over BUSY_SAMPLE s."""
    import time as _time
    a = _cpu_s(pid)
    _time.sleep(BUSY_SAMPLE)
    b = _cpu_s(pid)
    return a is not None and b is not None and b - a >= BUSY_CPU


def _container(name: str) -> str:
    """One spelling for a container: a logger (``a.b.c``), a launch's name (``/a/b/c`` or the
    relative ``c``) or a service prefix."""
    return name.strip("/").replace("/", ".")


def stalled_loads(path, now=None, lost_grace=None, stall=None, since=None, offset=0) -> str:
    """The containers of a pilot's launch (its console at ``path``, ros2 launch) with a composable
    load that never ended, in its current instance: a load whose reply the container failed to send
    (after ``lost_grace`` s), or one outstanding ``stall`` s (timed from the container's last load);
    each with its counts and what was loaded there. "" when none, or under play_launch. ``now``: wall
    time, default the clock. ``since``: the pilot's (re)start, wall time; lines stamped before it are
    a previous launch's (a restarted pilot's console keeps them until its new launch writes);
    ``offset``: read the console from there (its size at the pilot's last start)."""
    import time as _time
    now = _time.time() if now is None else now
    stall = LOAD_STALL if stall is None else stall
    lines = _console_text(path, offset).splitlines()
    start, play_launch = 0, False
    for i, line in enumerate(lines):
        if LAUNCH_START.match(line):
            start, play_launch = i, False
        elif PLAY_LAUNCH_START.search(line):
            start, play_launch = i, True
    if play_launch:
        return _pl_stalled(lines[start:], now, PL_LOST_GRACE if lost_grace is None else lost_grace,
                           stall, since)
    lost_grace = LOST_GRACE if lost_grace is None else lost_grace
    taken, answered, lost, pids, procs = {}, {}, {}, {}, {}
    for line in lines[start:]:
        m = _PID.match(line)
        if m:
            pids[m.group(1)] = int(m.group(2))
            continue
        m = _INSTANTIATE.match(line)
        if m:
            if since is None or float(m.group(2)) >= since:
                taken.setdefault(_container(m.group(3)), []).append((float(m.group(2)), m.group(4)))
                procs[_container(m.group(3))] = m.group(1)
            continue
        m = _LOADED.search(line) or _ROS_LOAD_FAILED.search(line)
        if m:
            c = _container(m.group(1) if m.re is _LOADED else m.group(2))
            answered[c] = answered.get(c, 0) + 1
            continue
        m = _LOST_REPLY.match(line)
        if m and (since is None or float(m.group(1)) >= since):
            lost.setdefault(_container(m.group(2)), float(m.group(1)))
    out = []
    for c, loads in taken.items():
        n = answered.get(c, 0)
        if n >= len(loads):
            continue
        # which load is unanswered is not known: a container's lines reach the console late, after
        # replies to later loads (088: glog_component's reply was the lost one, logged before
        # behavior_path_planner's load whose reply came). Its age is at least the last load's.
        last = max(t for t, _ in loads)
        names = ", ".join(sorted({re.sub(r"^rclcpp_components::NodeFactoryTemplate<(.*)>$", r"\1", k)
                                  .rsplit("::", 1)[-1] for _, k in loads}))
        what = f"{n} of {len(loads)} loads answered; loaded there: {names}"
        if c in lost and now - lost[c] >= lost_grace:
            out.append(f"/{c.replace('.', '/')} (its reply to the launch's load_node was lost "
                       f"{now - lost[c]:.0f} s ago; {what})")
        elif now - last >= stall and not (procs.get(c) in pids and _cpu_busy(pids[procs[c]])):
            # no lost reply: only a container that does nothing (a constructor still building
            # burns a core; it answers when the build ends)
            out.append(f"/{c.replace('.', '/')} (a load unanswered for at least {now - last:.0f} s; "
                       f"{what})")
    return "; ".join(out)


def _pl_stalled(lines, now, lost_grace, stall, since) -> str:
    """stalled_loads under play_launch: its loads still pending, the loaded count unchanged for
    ``lost_grace`` s after a lost LoadNode reply or for ``stall`` s without one; "" once its start
    is complete (or summarised: issue 064's check)."""
    lost = {}  # container -> [(t, composable)]
    progress = None  # (t when the loaded count last changed, loaded, total, pending)
    for line in lines:
        m = _PL_TIME.match(line)
        t = parse_iso(m.group(1)) if m else None
        if t is None or (since is not None and t < since):
            continue
        if _PL_DONE.search(line):
            return ""
        m = _PL_LOST.search(line)
        if m:
            lost.setdefault(m.group(1), []).append((t, m.group(2)))
            continue
        m = _PL_PROGRESS.search(line)
        if m:
            loaded, total, pending = int(m.group(1)), int(m.group(2)), int(m.group(3))
            if progress is None or progress[1] != loaded:
                progress = (t, loaded, total, pending)
            else:
                progress = (progress[0], loaded, total, pending)
    if progress is None or progress[3] == 0:
        return ""
    t_same, loaded, total, pending = progress
    what = f"{loaded} of {total} composables loaded, {pending} pending"
    out = []
    for c, ls in lost.items():
        t_lost = max(t_same, max(t for t, _ in ls))  # unchanged since the reply was lost
        if now - t_lost >= lost_grace:
            names = ", ".join(sorted({n for _, n in ls}))
            out.append(f"{c} (its LoadNode reply for {names} was lost {now - min(t for t, _ in ls):.0f} s "
                       f"ago; {what}, none loaded for {now - t_same:.0f} s)")
    if not out and now - t_same >= stall:
        out.append(f"{', '.join(lost) or 'its containers'} ({what}, none loaded for {now - t_same:.0f} s)")
    return "; ".join(out)


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
