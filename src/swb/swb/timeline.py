"""A run's ``timeline.jsonl``: one JSON object per line, ordered by wall time.

docs/design/projects-and-pilots.md, "Logs". Built from the runner's own events (run start and
end, pilot ready, result events), SSv2's milestones found in the scenario's logs, and every log
line at WARN or above in the slices. Each entry: ``{t_wall, t_sim, group, process, event, ...}``
with ``t_wall`` in seconds since the epoch. A log line without a time of its own (LibCarla's
``WARNING: ...``) takes the previous line's, else the slice's start, and says so
(``t_inferred``).
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
    return None


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
        return rel[-2]
    return path.stem if path.suffix == ".log" else path.name


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
                p = parse_line(line)
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
                e = {"t_wall": t, "t_sim": None, "group": group, "process": process,
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
    return [p for p in _files(d) if p.name not in _SKIP and p.suffix not in data]


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
# swb's own processes (the bridge and the relay, the world servers), and what they say when
# they refuse a scenario or fail it: "Cannot spawn 'x' ...", "... refused", "... failed", and
# a world server that stopped answering ("... did not answer Step within 10 s"), and its
# watchdog ending the blocked request ("... past its deadline ...", G70)
_SWB_GROUPS = {"bridge", "relay", "simulator"}
_REFUSAL = re.compile(r"\bCannot spawn\b|\brefus(?:e|es|ed)\b|\bfailed\b|\bdid not answer\b"
                      r"|\bpast its deadline\b", re.I)


def is_noise(e) -> bool:
    return bool(_NOISE.search(str(e.get("message") or "")))


def is_swb_refusal(e) -> bool:
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
    failure that swb's own processes logged at ``min_level`` or above, which outranks SSv2's
    generic error that follows it (a SemanticError naming no cause; roadmap 077, G67), else
    the first problem."""
    for e in timeline:
        if is_problem(e, min_level) and is_swb_refusal(e):
            return e
    return first_problem(timeline, min_level)
