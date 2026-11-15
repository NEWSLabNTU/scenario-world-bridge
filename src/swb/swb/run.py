"""``ros2 run swb run <scenario|suite>``: runs and suites against the project's live stack.

docs/design/projects-and-pilots.md, "Runs" and "Suites". The stack is found through
``<logs>/<project>/seat-<n>-current/stack.json``; a run refuses (3) a stack that is missing,
not running, not ready, or whose fingerprint differs from the project's (naming what).

Each run on its scenario's map: when the stack's pilots are on another (or on none yet),
the runner asks the stack's keeper to move them (``switch-request.json``, answered in
stack.json; swb.stack) and waits for the answer; a map that is not fetched, or not one of the
stack's, is refused. A run on the stack's map restarts nothing. A suite's runs are ordered by
map (the stack's current map first, then by first appearance; stable within a map), after
``--shard`` picks its share, so that shards agree across hosts.

Each run: ``runs/<id>/`` (``<project>/runs`` links to ``<logs>/<project>/runs``),
``resolved.yaml``; ``lifetime: run`` pilots started (a nested play_launch each, in its
domain) and every pilot's readiness waited for; the stack's log sizes noted; SSv2
(``scenario.launch.xml`` as a nested play_launch in the scenario domain, parameters as its
``override_parameters``); its end waited for (``timeout``); the slices cut, the timeline
built, ``result.json`` written, ``logging.keep`` applied, one line printed.

A suite file: ``scenarios`` (files, directories), ``parameters`` (a grid: every combination),
``distribution`` (OpenSCENARIO ParameterValueDistribution files, handed to SSv2 as they are,
one run each), ``repetitions``, ``seed`` (run k gets seed + k), ``expect`` (scenario ->
verdict), ``retry`` (``{on: error, times: n}``), ``pilot_args`` (entity -> launch arguments
for its ``lifetime: run`` pilot), ``logging``. A suite's runs are linked from
``<logs>/<project>/suites/<suite>[_shard-i-of-n]/``, which gets ``summary.csv`` and a merged
``junit.xml``; ``--resume`` skips the runs linked there with a result.

``--start-stack``: when the project has no live stack, the runner starts one (play_launch of
``swb_launch swb.launch.xml`` for the project, on the first free seat, in its own session,
its console under ``<logs>/<project>/stack-launches/``), waits for it to be ready, and
leaves it up after the runs. ``--stop-stack``: the same, and the stack it started is stopped
after the runs (SIGINT to its process group, then SIGTERM, SIGKILL). A stack already up is
used and never stopped; one that differs from the project is refused, never replaced.

Exit codes: 0 every run as expected (passed, or the suite's ``expect``), 1 a scenario failed,
2 an infrastructure error, 3 refused.
"""
import argparse
import dataclasses
import fcntl
import itertools
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from swb import budget as budgetmod, config, judges as judgesmod, project as projectmod, results
from swb import record as recordmod, runs as runsmod, slices, stack as stackmod, timeline
from swb import episodes as episodesmod, frames as framesmod, ssv2 as ssv2mod
from swb.findings import ros_args

EXIT_OK, EXIT_FAILED, EXIT_INFRA, EXIT_REFUSED = 0, 1, 2, 3
DEFAULT_TIMEOUT = 400          # scenario.launch.xml's global_timeout
END_MARGIN = 120               # past global_timeout before the runner stops SSv2 itself
READY_TIMEOUT = 600
SWITCH_TIMEOUT = 180           # the keeper stopping and starting the pilots (not their readiness)
JUNIT = Path("scenario_test_runner") / "result.junit.xml"
DEFAULT_LOGGING = {"capture": "standard", "keep": {"passed": "result", "failed": "full",
                                                   "error": "full"}}


class Refused(Exception):
    pass


class NoStack(Refused):
    """No live stack of the project (``--start-stack`` starts one)."""


class StackFailed(Exception):
    """A stack the runner started did not come up (an infrastructure error, exit 2)."""


# --- run specs ------------------------------------------------------------------------------

@dataclass
class RunSpec:
    scenario: Path
    parameters: dict = field(default_factory=dict)
    repetition: int = 0
    seed: int = None
    expect: str = None
    pilot_args: dict = field(default_factory=dict)
    logging: dict = field(default_factory=dict)
    distribution: bool = False
    timeout: float = None
    ssv2: dict = field(default_factory=dict)   # the suite's and its item's (swb.ssv2)

    @property
    def key(self) -> str:
        parts = [_token(self.scenario.stem)]
        parts += [f"{_token(k)}-{_token(v)}" for k, v in self.parameters.items()]
        return "_".join(parts) + f"_r{self.repetition}"


def _token(v) -> str:
    return re.sub(r"[^A-Za-z0-9_.+-]+", "-", str(v)).strip("-") or "x"


def is_suite(path: Path) -> bool:
    if path.suffix not in (".yaml", ".yml"):
        return False
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError):
        return False
    return isinstance(data, dict) and "OpenSCENARIO" not in data


def _resolve(item, project: Path, base: Path) -> Path:
    p = Path(str(item)).expanduser()
    if p.is_absolute():
        return p
    for root in (project, base):
        if (root / p).exists():
            return (root / p).resolve()
    return (project / p).resolve()


def _rel(project: Path, p: Path) -> str:
    try:
        return Path(p).resolve().relative_to(project.resolve()).as_posix()
    except ValueError:
        return str(p)


def grid(parameters: dict) -> list:
    """Every combination of a ``parameters:`` mapping, in its order; [{}] when empty."""
    if not parameters:
        return [{}]
    names = list(parameters)
    values = [v if isinstance(v, list) else [v] for v in parameters.values()]
    return [dict(zip(names, combo)) for combo in itertools.product(*values)]


def _expect_for(expect: dict, project: Path, scenario: Path):
    rel = _rel(project, scenario)
    for k, v in (expect or {}).items():
        k = str(k)
        if rel == k or rel.endswith("/" + k) or rel == f"scenarios/{k}":
            return v
    return None


def expand(target: Path, project: Path, repeat: int = None, params: dict = None) -> tuple:
    """``(specs, suite settings or None)`` for a scenario file or a suite file."""
    target = Path(target)
    if not target.is_absolute():
        target = _resolve(target, project, Path.cwd())
    if not target.exists():
        raise Refused(f"{target}: no such file")
    if target.is_dir():
        files = projectmod._scenarios_in(target)
        if not files:
            raise Refused(f"{target}: no scenario in it")
        reps = repeat or 1
        return [RunSpec(f, dict(params or {}), k) for f in files for k in range(reps)], None
    if not is_suite(target):
        return [RunSpec(target, dict(params or {}), k) for k in range(repeat or 1)], None

    try:
        suite = config.read_yaml(target)
    except config.ConfigError as e:
        raise Refused(str(e)) from None
    bad = [k for k in suite if k not in projectmod.SUITE_KEYS]
    if bad:
        raise Refused(f"{target}: unknown key {bad[0]} (use only {', '.join(projectmod.SUITE_KEYS)})")
    base = target.parent
    items, bad = projectmod.suite_items(suite.get("scenarios") or [])
    if bad:
        raise Refused(f"{target}: {bad[0]}")
    probs = ssv2mod.problems(suite.get("ssv2"))
    if probs:
        raise Refused(f"{target}: {probs[0][0]}: {probs[0][1]}")
    listed = [path for path, _ in items]
    own_ssv2 = {}
    dists = suite.get("distribution") or []
    dists = [dists] if isinstance(dists, str) else list(dists)
    if not listed and not dists:
        raise Refused(f"{target}: no scenarios: or distribution:")
    files = []
    for item, own in items:
        p = _resolve(item, project, base)
        found = projectmod._scenarios_in(p) if p.is_dir() else [p] if p.is_file() else None
        if found is None:
            raise Refused(f"{target}: scenario {item}: no such file or directory")
        files += found
        for f in found:
            own_ssv2[str(f)] = ssv2mod.merge(suite.get("ssv2"), own)
    reps = repeat or suite.get("repetitions") or 1
    if not isinstance(reps, int) or reps < 1:
        raise Refused(f"{target}: repetitions must be a positive integer, not {reps!r}")
    seed = suite.get("seed")
    if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool)):
        raise Refused(f"{target}: seed must be an integer, not {seed!r}")
    parameters = suite.get("parameters") or {}
    if not isinstance(parameters, dict):
        raise Refused(f"{target}: parameters: a mapping name -> [values]")
    combos = grid({**parameters, **(params or {})})
    expect = suite.get("expect") or {}
    for v in expect.values():
        if v not in results.VERDICTS:
            raise Refused(f"{target}: expect: {v!r} is not one of {', '.join(results.VERDICTS)}")
    pilot_args = suite.get("pilot_args") or {}
    logging = suite.get("logging") or {}
    specs = []
    for f in files:
        for combo in combos:
            for k in range(reps):
                specs.append(RunSpec(f, combo, k, None if seed is None else seed + k,
                                     _expect_for(expect, project, f), pilot_args, logging,
                                     ssv2=dict(own_ssv2.get(str(f), {}))))
    for d in dists:
        p = _resolve(d, project, base)
        if not p.is_file():
            raise Refused(f"{target}: distribution {d}: no such file")
        for k in range(reps):
            specs.append(RunSpec(p, {}, k, None if seed is None else seed + k,
                                 _expect_for(expect, project, p), pilot_args, logging,
                                 distribution=True, ssv2=ssv2mod.merge(suite.get("ssv2"))))
    return specs, suite


def shard(specs: list, text: str) -> list:
    """``i/n`` (1-based): every n-th spec from the i-th."""
    m = re.fullmatch(r"\s*(\d+)\s*/\s*(\d+)\s*", text or "")
    if not m:
        raise Refused(f"--shard {text!r}: give i/n, e.g. 1/3")
    i, n = int(m.group(1)), int(m.group(2))
    if n < 1 or not 1 <= i <= n:
        raise Refused(f"--shard {text}: i must be 1-{n}")
    return specs[i - 1::n]


# --- the stack ------------------------------------------------------------------------------

def find_stack(project: Path, host: dict) -> dict:
    """This project's live, ready stack on this seat (its stack.json), or Refused naming why."""
    name = project.resolve().name
    root = stackmod.project_root(host["logs"], name)
    choice = host.get("seat")
    links = []
    if choice not in (None, "auto"):
        links = [stackmod.current_link(host["logs"], name, int(choice))]
    elif root.is_dir():
        links = sorted(root.glob("seat-*-current"))
    live, seen = [], []
    for link in links:
        f = link / stackmod.STACK_JSON
        if not f.is_file():
            continue
        try:
            rec = stackmod.read_json(f)
        except (OSError, ValueError):
            continue
        seen.append(rec)
        if stack_alive(rec):
            live.append(rec)
    if not live:
        where = f"seat {choice}" if choice not in (None, "auto") else "any seat"
        hint = ("start it: cd " + str(project) + " && play_launch launch swb_launch swb.launch.xml")
        hint += " (or give --start-stack)"
        if seen:
            r = seen[0]
            raise NoStack(f"the stack {r.get('stack_id')} on seat {r['seat']['seat']} is not "
                          f"running (state {r.get('state')}); {hint}")
        raise NoStack(f"no stack of project {name} on {where} ({root}); {hint}")
    if len(live) > 1:
        raise Refused("stacks of this project on seats "
                      + ", ".join(str(r["seat"]["seat"]) for r in live)
                      + ": give --seat <n> (or seat: in .swb/local.yaml)")
    return live[0]


def stack_alive(rec: dict) -> bool:
    """The keeper that wrote ``rec`` still holds its seat."""
    from swb import seat as seatmod
    if rec.get("state") == stackmod.STOPPED or not rec.get("pid"):
        return False
    h = seatmod.holder(rec["seat"]["seat"], rec.get("lock_dir"))
    return bool(h) and h.get("pid") == rec["pid"] and seatmod.pid_alive(rec["pid"])


def check_stack(rec: dict, project: Path, wait: float = 0.0, reread=None) -> dict:
    """Refused unless ``rec``'s fingerprint is the project's and it is (or within ``wait``
    seconds becomes) ready."""
    diff = stackmod.fingerprint_diff(rec.get("fingerprint") or {}, stackmod.fingerprint(project))
    if diff:
        raise Refused(f"the stack {rec.get('stack_id')} was started from another project "
                      "state: " + "; ".join(diff) + " differ; restart the stack")
    deadline = time.time() + wait
    while rec.get("state") != stackmod.READY:
        if time.time() >= deadline:
            waiting = [p["entity"] for p in rec.get("pilots", [])
                       if p.get("lifetime", "stack") == "stack" and not p.get("ready_at")]
            raise Refused(f"the stack {rec.get('stack_id')} is not ready (state "
                          f"{rec.get('state')}" + (f"; waiting for {', '.join(waiting)}"
                                                   if waiting else "") + "); try --wait 600")
        time.sleep(1.0)
        rec = (reread or (lambda r: stackmod.read_json(
            Path(r["proc"]).parent / stackmod.STACK_JSON)))(rec)
    return rec


# --- a stack the runner starts (--start-stack, --stop-stack) --------------------------------

STACK_START_TIMEOUT = 1800.0   # a first Autoware start may build TensorRT engines for minutes
STACK_STOP_GRACE = 120.0       # SIGINT to the launch; the keeper stops pilots and CARLA first


@dataclass
class StartedStack:
    """A stack this runner started: its play_launch, in its own session."""
    proc: object
    argv: list
    log: Path
    seat: int

    def stop(self, grace=STACK_STOP_GRACE):
        stackmod.stop_processes([self.proc], grace)


def stack_argv(project: Path, host: dict, seat: int, log_dir: Path) -> list:
    """The stack as the user starts it: play_launch of swb.launch.xml for ``project``, on
    ``seat``, its web UI on the seat's port."""
    from swb import seat as seatmod
    s = seatmod.table(host, seatmod.seats_source(project))[seat]
    argv = ["play_launch", "launch", "--web-addr", f"127.0.0.1:{s.web_port}", "--log-dir",
            str(log_dir), "swb_launch", "swb.launch.xml", f"project:={project}", f"seat:={seat}",
            f"logs:={host['logs']}"]
    return argv


def start_stack(project: Path, host: dict, command=None, out=None) -> StartedStack:
    """Start the project's stack (``command(project, host, seat, log_dir) -> argv``, default
    ``stack_argv``) in its own session, its console in ``<logs>/<project>/stack-launches/``."""
    from swb import seat as seatmod
    try:
        s = seatmod.resolve(host, seatmod.seats_source(project),
                            usable=lambda x: not seatmod.busy_ports(x))
    except (config.ConfigError, seatmod.SeatTaken) as e:
        raise Refused(f"no seat for a stack: {e}") from None
    if seatmod.holder(s.index, s.lock_dir) is not None:
        raise Refused(f"seat {s.index} is taken: "
                      f"{seatmod.describe_holder(seatmod.holder(s.index, s.lock_dir))}")
    busy = seatmod.busy_ports(s)
    if busy:
        raise Refused(seatmod.describe_busy(s, busy))
    root = stackmod.project_root(host["logs"], project.resolve().name) / "stack-launches"
    log_dir, n = root / stackmod.now_id(), 1
    while True:
        try:
            log_dir.mkdir(parents=True)
            break
        except FileExistsError:
            n += 1
            log_dir = root / f"{stackmod.now_id()}-{n}"
    argv = (command or stack_argv)(project, host, s.index, log_dir)
    try:
        proc = stackmod.start_process(argv, dict(os.environ), log_dir / "console.log",
                                      cwd=str(project))
    except OSError as e:
        raise StackFailed(f"the stack did not start ({argv[0]}): {e}") from None
    (out or sys.stdout).write(f"stack    starting on seat {s.index} (pid {proc.pid}, console "
                              f"{log_dir / 'console.log'})\n")
    (out or sys.stdout).flush()
    return StartedStack(proc=proc, argv=argv, log=log_dir / "console.log", seat=s.index)


def _tail(path: Path, n=15) -> str:
    try:
        return "\n".join(Path(path).read_text(errors="replace").splitlines()[-n:])
    except OSError:
        return ""


def wait_started(started: StartedStack, project: Path, host: dict, timeout: float) -> dict:
    """The started stack's stack.json once it is ready; StackFailed (or Refused, when the
    launch refused the stack) when it ends first or ``timeout`` passes."""
    h = dict(host, seat=started.seat)
    deadline = time.time() + timeout
    while True:
        code = started.proc.poll()
        if code is not None:
            tail = _tail(started.log)
            msg = (f"the stack ended ({code}) before it was ready; {started.log}:\n{tail}")
            raise (Refused if "is refused (3)" in tail else StackFailed)(msg)
        try:
            rec = find_stack(project, h)
        except NoStack:
            rec = None
        if rec is not None and rec.get("state") == stackmod.READY:
            return rec
        if time.time() >= deadline:
            state = rec.get("state") if rec else "no stack.json yet"
            raise StackFailed(f"the stack was not ready within {timeout:.0f} s ({state}); "
                              f"{started.log}")
        time.sleep(0.2)


# --- one run --------------------------------------------------------------------------------

def runs_dir(project: Path, host: dict) -> Path:
    """``<logs>/<project>/runs``, with ``<project>/runs`` linked to it when it can be."""
    d = stackmod.project_root(host["logs"], project.resolve().name) / "runs"
    d.mkdir(parents=True, exist_ok=True)
    link = project / "runs"
    try:
        if not link.exists() and not link.is_symlink():
            os.symlink(d, link)
    except OSError:
        pass
    return d


def new_run_dir(root: Path, key: str) -> Path:
    base = f"{stackmod.now_id()}_{key}"
    d = root / base
    n = 1
    while True:
        try:
            d.mkdir(parents=True)
            return d
        except FileExistsError:
            n += 1
            d = root / f"{base}-{n}"


def scenario_argv(rec: dict, spec: RunSpec, run_dir: Path, scenario: Path) -> list:
    """SSv2 as a nested play_launch of scenario.launch.xml, for this run."""
    args = {"scenario": str(scenario), "port": str(rec["ports"]["ssv2"]),
            "output_directory": str(run_dir / "logs" / "ssv2")}
    # the run settings (swb.ssv2: swb.yaml's, the suite's, the item's, the call's)
    args.update(ssv2mod.launch_args(spec.ssv2 or {}))
    args["global_timeout"] = str(int(spec.timeout or DEFAULT_TIMEOUT))
    ego = next((p for p in rec.get("pilots", []) if p["entity"] == "ego"), None)
    if ego and ego.get("vehicle"):
        args["vehicle_model"] = ego["vehicle"]
    if spec.parameters and not spec.distribution:
        args["override_parameters"] = json.dumps({k: str(v) for k, v in spec.parameters.items()})
    return (["play_launch", "launch", "--disable-web-ui", "--log-dir",
             str(run_dir / "logs" / "scenario"), "swb_launch", "scenario.launch.xml"]
            + [f"{k}:={v}" for k, v in args.items()])


def pilot_argv(rec: dict, p: dict, entry: dict, log_dir: Path) -> list:
    """A ``lifetime: run`` pilot as a nested play_launch of its kind's launch (as the keeper
    starts a stack pilot: swb.stack.pilot_argv)."""
    return stackmod.pilot_argv(rec, p, entry, log_dir)


def _start(argv, env, log: Path, cwd=None):
    return stackmod.start_process(argv, env, log, cwd=cwd)


def _stop(proc, grace=stackmod.STOP_GRACE):
    stackmod.stop_processes([proc], grace)


def host_sample() -> dict:
    out = {"t_wall": time.time()}
    try:
        out["load"] = list(os.getloadavg())
    except OSError:
        pass
    try:
        du = shutil.disk_usage("/")
        out["disk_free_gb"] = round(du.free / 1e9, 1)
    except OSError:
        pass
    return out


@dataclass
class Context:
    project: Path
    host: dict
    stack: dict
    runs_root: Path
    workspace: Path = None
    ready_timeout: float = READY_TIMEOUT
    scenario_command: object = None   # callable(rec, spec, run_dir, scenario) -> argv (tests)
    pilot_command: object = None      # callable(rec, pilot, entry, log_dir) -> argv (tests)
    out: object = None
    started: StartedStack = None      # the stack this runner started (--start-stack)
    turn: object = None               # the stack's run lock while this process runs (take_turn)


def scenario_map(path: Path, sc=None):
    """The project map a scenario names (a distribution: its ScenarioFile's), or None."""
    try:
        sc = sc or projectmod.read_scenario(path)
        if sc.distribution_of is not None:
            if "$(" in sc.distribution_of:
                return None
            target = Path(sc.distribution_of)
            if not target.is_absolute():
                target = Path(path).parent / target
            sc = projectmod.read_scenario(target)
    except (ValueError, OSError):
        return None
    return stackmod.map_of(sc)


def order_by_map(specs: list, current=None) -> list:
    """The specs grouped by map: the stack's current map first, then the maps in order of
    first appearance; stable within a map (and specs with no map keep their group's place)."""
    cache, groups = {}, {}
    for s in specs:
        key = str(s.scenario)
        if key not in cache:
            cache[key] = scenario_map(s.scenario)
        groups.setdefault(cache[key], []).append(s)
    order = list(groups)
    if current is not None and current in groups:
        order.remove(current)
        order.insert(0, current)
    return [s for m in order for s in groups[m]]


def ensure_map(ctx: Context, name: str, timeout: float = SWITCH_TIMEOUT):
    """The stack's pilots on map ``name``: nothing when they are; else the keeper is asked to
    move them and the answer waited for. Refused when the stack cannot serve the map; an
    infrastructure message (str) when the switch failed; None when done. ``ctx.stack``
    becomes the stack.json after the switch."""
    rec = ctx.stack
    if rec.get("map") == name:
        return None
    problem = stackmod.switch_problem(rec, name)
    if problem:
        raise Refused(problem)
    stack_dir = Path(rec["proc"]).parent
    out = ctx.out or sys.stdout
    t0 = time.time()
    rid = stackmod.request_switch(stack_dir, name)
    while True:
        try:
            cur = stackmod.read_json(stack_dir / stackmod.STACK_JSON)
        except (OSError, ValueError):
            cur = {}
        sw = cur.get("switch") or {}
        if sw.get("id") == rid and sw.get("state") in (stackmod.DONE, stackmod.FAILED):
            break
        if cur and not stack_alive(cur):
            return f"the stack {rec.get('stack_id')} stopped while switching to map {name}"
        if time.time() - t0 >= timeout:
            return (f"the stack {rec.get('stack_id')} did not move its pilots to map {name} "
                    f"within {timeout:.0f} s")
        time.sleep(0.1)
    if sw["state"] == stackmod.FAILED:
        return f"the stack could not switch to map {name}: {sw.get('error')}"
    ctx.stack = cur
    out.write(f"map      {name}  (pilots restarted from {rec.get('map') or 'none'} in "
              f"{time.time() - t0:.1f} s)\n")
    out.flush()
    return None


CRASH_LIMIT = 5   # a node's crashes, while its pilot is not ready, before the run gives up
_CRASH = re.compile(r"\[node:(\S+)\] Exited .*\(crash (\d+) since last healthy run\)")


def pilot_console(rec: dict, p: dict, logs: Path) -> Path:
    """Where a pilot's play_launch writes: the run's logs for a per-run pilot, else the
    stack's proc/."""
    if p.get("lifetime") == "run":
        return logs / f"pilot-{p['entity']}" / "console.log"
    return Path(rec["proc"]) / f"pilot-{p['entity']}" / "console.log"


class CrashWatch:
    """Counts play_launch's "crash N since last healthy run" lines a pilot's console gains
    after the watch starts (crashes before this run do not count)."""

    def __init__(self, path: Path, limit=CRASH_LIMIT):
        self.path, self.limit = Path(path), limit
        try:
            self.offset = self.path.stat().st_size
        except OSError:
            self.offset = 0

    def crashing(self):
        """``(node, crashes)`` once a node reached the limit, else None."""
        try:
            with open(self.path, "rb") as f:
                f.seek(self.offset)
                text = f.read().decode("utf-8", "replace")
        except OSError:
            return None
        worst = None
        for m in _CRASH.finditer(text):
            n = int(m.group(2))
            if n >= self.limit and (worst is None or n > worst[1]):
                worst = (m.group(1), n)
        return worst


def logging_for(*layers) -> dict:
    """The defaults, then each layer (the project's, the suite's or the call's); a layer's
    ``keep`` replaces the one before it whole (``keep: {}`` keeps everything)."""
    lg = config.merge(DEFAULT_LOGGING, {})
    for layer in layers:
        if not layer:
            continue
        lg = config.merge(lg, {k: v for k, v in layer.items() if k != "keep"})
        if "keep" in layer:
            lg["keep"] = dict(layer["keep"] or {})
    return lg


RUN_LOCK = "run.lock"


def take_turn(ctx: Context):
    """One runner at a time on a stack: two SSv2s in one ROS domain drive each other's nodes
    (their names are the same), so a second run is refused, naming the one running. Held
    from a command's first run until the command ends (a suite keeps its turn)."""
    if getattr(ctx, "turn", None) is not None:
        return
    stack_dir = Path(ctx.stack["proc"]).parent
    path = stack_dir / RUN_LOCK
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        try:
            holder = json.loads(os.pread(fd, 4096, 0).decode() or "{}")
        except (OSError, ValueError):
            holder = {}
        os.close(fd)
        raise Refused(f"another run is going on stack {ctx.stack.get('stack_id')} (pid "
                      f"{holder.get('pid', '?')}, {holder.get('target', '?')}, since "
                      f"{holder.get('since', '?')}); one run at a time per stack: wait for it,"
                      " or start a stack of your own on another seat") from None
    info = {"pid": os.getpid(), "target": " ".join(sys.argv[1:])[:200],
            "since": time.strftime("%Y-%m-%dT%H:%M:%S")}
    os.ftruncate(fd, 0)
    os.pwrite(fd, json.dumps(info).encode(), 0)
    os.set_inheritable(fd, False)
    ctx.turn = fd


def release_turn(ctx: Context):
    """Give the stack's run lock back (the command or run_one ends)."""
    fd = getattr(ctx, "turn", None) if ctx is not None else None
    if fd is not None:
        ctx.turn = None
        try:
            os.close(fd)
        except OSError:
            pass


def run_spec(ctx: Context, spec: RunSpec, attempt: int = 0) -> results.RunResult:
    """One run, end to end, into its own run directory."""
    take_turn(ctx)
    rec, project = ctx.stack, ctx.project
    psettings = config.project_settings(project)
    lg = logging_for(psettings.get("logging"), spec.logging)
    # SSv2's run settings: swb.yaml's ssv2:, then the spec's (suite, item, call), then
    # --timeout / overrides["timeout"]
    settings = ssv2mod.merge(psettings.get("ssv2"), spec.ssv2)
    if spec.timeout:
        settings["global_timeout"] = spec.timeout
    spec = dataclasses.replace(spec, ssv2=settings,
                               timeout=settings.get("global_timeout") or spec.timeout)
    managed = ssv2mod.managed(settings)
    sc_rel = _rel(project, spec.scenario)
    infra, events, run_events = [], [], []

    # the scenario's entities against the stack's pilots (refused before any run directory)
    try:
        sc = projectmod.read_scenario(spec.scenario)
    except (ValueError, OSError) as e:
        sc = None
        infra.append(f"the scenario cannot be read: {e}")
    variants = sc.variants if sc is not None else 1
    pilots = []
    if sc is not None and sc.distribution_of is None:
        by_entity = {p["entity"]: p for p in rec.get("pilots", [])}
        for ent in sc.entities:
            if not ent.external or (ent.ego and not managed):
                continue   # managed_ego: false: SSv2 hands its ego to no pilot
            p = by_entity.get(ent.name)
            if p is None:
                raise Refused(f"entity {ent.name} of {sc_rel} has no pilot in the stack "
                              f"{rec.get('stack_id')} (its pilots: {', '.join(by_entity) or 'none'})")
            pilots.append(p)
    else:
        pilots = list(rec.get("pilots", []))
        if not managed:
            pilots = [p for p in pilots if p["entity"] != "ego"]

    # the stack's pilots on the scenario's map (a switch when they are on another)
    m = scenario_map(spec.scenario, sc) if sc is not None else None
    if m:
        msg = ensure_map(ctx, m)
        if msg:
            infra.append(msg)
        rec = ctx.stack
        by_entity = {p["entity"]: p for p in rec.get("pilots", [])}
        pilots = [by_entity.get(p["entity"], p) for p in pilots]
    elif sc is not None and not rec.get("map") and pilots:
        raise Refused(f"{sc_rel} names no project map ($(env SWB_MAPS)/<map>) and the stack's "
                      "pilots are on none yet: start the stack with map:=")

    run_dir = new_run_dir(ctx.runs_root, spec.key)
    run_id = run_dir.name
    logs = run_dir / "logs"
    logs.mkdir()
    t0 = time.time()
    run_events.append({"t_wall": t0, "event": "run.start", "run_id": run_id})
    sub = {"run_dir": str(run_dir), "seed": spec.seed if spec.seed is not None else 0,
           "run_id": run_id, "params": spec.parameters}
    run_pilots = []
    for p in pilots:
        if p.get("lifetime") != "run":
            continue
        b = stackmod.Bound(p["entity"], p.get("key") or p["entity"], dict(p.get("spec") or {}),
                           p["domain"])
        extra = (spec.pilot_args or {}).get(p["entity"]) or {}
        if extra:
            b.spec["args"] = {**(b.spec.get("args") or {}), **extra}
        try:
            entry = stackmod.pilot_entry(b, project, sub)
        except stackmod.StackRefused as e:
            shutil.rmtree(run_dir, ignore_errors=True)
            raise Refused(str(e)) from None
        run_pilots.append((p, entry))

    scenario_file = spec.scenario
    ssv2_argv = (ctx.scenario_command or scenario_argv)(rec, spec, run_dir, scenario_file)
    resolved = {
        "run_id": run_id,
        "attempt": attempt,
        "project": str(project),
        "scenario": {"file": sc_rel, "path": str(spec.scenario),
                     "sha256": results.file_sha256(spec.scenario),
                     "parameters": spec.parameters, "repetition": spec.repetition,
                     "seed": spec.seed, "distribution": spec.distribution,
                     "expect": spec.expect},
        "stack": {k: rec.get(k) for k in ("stack_id", "simulator", "map", "seat", "domains",
                                           "ports", "fingerprint")},
        "pilots": {p["entity"]: {"kind": p["kind"], "lifetime": p.get("lifetime", "stack"),
                                 "domain": p["domain"]} for p in pilots},
        "run_pilots": {p["entity"]: e for p, e in run_pilots},
        "logging": lg,
        "ssv2": {"argv": ssv2_argv, "variants": variants, "settings": settings,
                 "timeout_s": (spec.timeout or DEFAULT_TIMEOUT) * variants + END_MARGIN},
        "env": {**(rec.get("env") or {}), "ROS_DOMAIN_ID": str(rec["domains"]["scenario"])},
    }
    (run_dir / "resolved.yaml").write_text(yaml.safe_dump(resolved, sort_keys=False))

    capture = lg.get("capture") or "standard"
    if capture in ("full", "debug"):
        samples = [host_sample()]

    procs, recorders = [], []
    ssv2 = None
    try:
        for p, entry in (run_pilots if not infra else []):
            log_dir = logs / f"pilot-{p['entity']}"
            argv = (ctx.pilot_command or pilot_argv)(rec, p, entry, log_dir)
            if argv:
                procs.append(_start(argv, stackmod.ros_env(rec, p["domain"]),
                                    log_dir / "console.log", cwd=str(project)))
                run_events.append({"t_wall": time.time(), "event": "pilot.start",
                                   "entity": p["entity"]})
        # every pilot ready (none waited for when the stack already failed the run); a pilot
        # whose node keeps crashing fails the run now, not at the readiness timeout
        deadline = time.time() + ctx.ready_timeout
        waiting = list(pilots) if not infra else []
        watches = {p["entity"]: CrashWatch(pilot_console(rec, p, logs)) for p in waiting}
        why = {p["entity"]: [] for p in waiting}
        while waiting:
            for p in list(waiting):
                if stackmod.check_ready(p.get("ready"), stackmod.ros_env(rec, p["domain"]),
                                        why=why[p["entity"]]):
                    waiting.remove(p)
                    run_events.append({"t_wall": time.time(), "event": "pilot.ready",
                                       "entity": p["entity"]})
                    continue
                crashing = watches[p["entity"]].crashing()
                if crashing:
                    node, n = crashing
                    waiting.remove(p)
                    infra.append(f"pilot {p['entity']} is not coming up: its node {node} crashed"
                                 f" {n} times (see {watches[p['entity']].path.parent})")
            if waiting and time.time() >= deadline:
                for p in waiting:
                    last = why[p["entity"]]
                    infra.append(f"pilot {p['entity']} never became ready "
                                 f"({ctx.ready_timeout:.0f} s)"
                                 + (f": {last[0]}" if last else ""))
                break
            if waiting:
                time.sleep(2.0)

        proc_dir = Path(rec["proc"])
        mark0 = slices.LogMark.take(proc_dir)
        episodes = episodesmod.Mark.take(rec.get("episodes") or {})
        t_start = time.time()
        if not infra:
            for name, domain, rargv in recordmod.plan(run_dir, capture, pilots,
                                                      rec["domains"]["scenario"],
                                                      lg.get("topics")):
                try:
                    recorders.append(_start(rargv, stackmod.ros_env(rec, domain),
                                            logs / "record" / f"{name}.log", cwd=str(run_dir)))
                    run_events.append({"t_wall": time.time(), "event": "record.start",
                                       "name": name, "domain": domain})
                except OSError as e:
                    run_events.append({"t_wall": time.time(), "event": "record.error",
                                       "name": name, "message": str(e)})
            env = stackmod.ros_env(rec, rec["domains"]["scenario"])
            try:
                ssv2 = _start(ssv2_argv, env, logs / "scenario" / "console.log", cwd=str(project))
            except OSError as e:
                ssv2 = None
                infra.append(f"SSv2 did not start: {e}")
            if ssv2 is not None:
                # SSv2's global_timeout is per variant; a distribution runs them all in one go
                limit = (spec.timeout or DEFAULT_TIMEOUT) * variants + END_MARGIN
                try:
                    code = ssv2.wait(timeout=limit)
                except subprocess.TimeoutExpired:
                    _stop(ssv2)
                    code = None
                    infra.append(f"SSv2 did not end within {limit:.0f} s; stopped")
                run_events.append({"t_wall": time.time(), "event": "ssv2.exit", "code": code})
        t_end = time.time()
        mark1 = slices.LogMark.take(proc_dir)
    finally:
        # SSv2 first: interrupted, it would go on talking to the stack's bridge and join the
        # next run's scenario as a second client. Then the recorders, on SIGINT, so each bag
        # is closed before its topics go away
        if ssv2 is not None and ssv2.poll() is None:
            _stop(ssv2)
        stackmod.stop_processes(recorders)
        for proc in procs:
            _stop(proc)

    if capture != "result":
        slices.cut(mark0, mark1, logs)
    # the bridge's gap traces and contact reports of this run, into its directory
    if ssv2 is not None:
        for kind, moved in episodesmod.collect(episodes, run_dir).items():
            run_events.append({"t_wall": time.time(), "event": f"{kind}.collected",
                               "files": [m.name for m in moved]})
    frame_metrics = framesmod.from_marks(mark0, mark1)
    junit_src = logs / "ssv2" / JUNIT
    junit = run_dir / "junit.xml"
    if junit_src.is_file():
        shutil.copyfile(junit_src, junit)
    verdict, reason = results.verdict_from(junit if junit.is_file() else None, infra)
    for msg in infra:
        events.append({"t_sim": None, "t_wall": time.time(), "type": results.STACK_ERROR,
                       "message": msg})
    run_events.append({"t_wall": time.time(), "event": "run.end", "verdict": verdict})
    tl = timeline.build(run_events + [dict(e, event=e["type"]) for e in events],
                        slices_dir=logs, default_t=t0)
    timeline.write(tl, run_dir / "timeline.jsonl")
    if capture in ("full", "debug"):
        samples.append(host_sample())
        (run_dir / "host.json").write_text(json.dumps(samples, indent=2) + "\n")

    prov = results.provenance(ctx.workspace or stackmod.workspace_root() or project, project)
    prov.update(simulator=rec.get("simulator"), stack_id=rec.get("stack_id"),
                spec_hash=rec.get("spec_hash") or {})
    result = results.RunResult(
        run_id=run_id, verdict=verdict, reason=reason,
        scenario={"file": sc_rel, "sha256": resolved["scenario"]["sha256"],
                  "parameters": {k: v for k, v in spec.parameters.items()},
                  "repetition": spec.repetition, "seed": spec.seed},
        ssv2=results.ssv2_block(junit if junit.is_file() else None),
        events=events,
        metrics={"duration_s": round(t_end - t_start, 3), **frame_metrics},
        pilots={p["entity"]: {"kind": p["kind"], "vehicle": p.get("vehicle"),
                              "inputs": p.get("inputs") or [],
                              "lifetime": p.get("lifetime", "stack")} for p in pilots},
        provenance=prov)
    result.write(run_dir / "result.json")
    judgesmod.run_all(config.project_settings(project).get("judges"), run_dir, project, verdict)
    if capture == "result":
        runsmod.downgrade(run_dir, "result")
    runsmod.apply_keep(run_dir, verdict, lg.get("keep") or {})
    return result


def outcome(result: results.RunResult, expect=None) -> int:
    """The exit code a run contributes."""
    if expect is not None:
        return EXIT_OK if result.verdict == expect else (
            EXIT_INFRA if result.verdict == results.ERROR else EXIT_FAILED)
    return {results.PASSED: EXIT_OK, results.FAILED: EXIT_FAILED}.get(result.verdict, EXIT_INFRA)


def line(result: results.RunResult, run_dir: Path, expect=None) -> str:
    tag = result.verdict + (f" (expected {expect})" if expect else "")
    why = f"  {result.reason}" if result.reason else ""
    judged = judgesmod.summary(run_dir)
    jud = ("  judges: " + " ".join(f"{k}={v}" for k, v in judged.items())) if judged else ""
    return f"{tag:<8} {result.run_id}  {run_dir}{jud}{why}"


# --- the command ----------------------------------------------------------------------------

def context(project_arg=None, seat=None, logs=None, wait=0.0, start=False, stack_command=None,
            stack_timeout=None, **kw) -> Context:
    """The project, its host settings and its live stack. ``start``: when the project has no
    live stack, start one (``start_stack``) and wait for it (``stack_timeout``, else ``wait``
    when given, else STACK_START_TIMEOUT); ``Context.started`` is then that stack, which the
    caller stops or keeps. A stack that is up but differs from the project is refused, never
    replaced."""
    try:
        project = config.find_project(project_arg or None)
        over = {}
        if seat not in (None, ""):
            over["seat"] = seat
        if logs:
            over["logs"] = logs
        host = config.host_settings(project, over)
    except config.ConfigError as e:
        raise Refused(str(e)) from None
    started = None
    try:
        rec = find_stack(project, host)
    except NoStack:
        if not start:
            raise
        started = start_stack(project, host, stack_command, kw.get("out"))
        try:
            rec = wait_started(started, project, host,
                               stack_timeout or (wait or None) or STACK_START_TIMEOUT)
        except BaseException:
            started.stop()   # half up is of no use to anyone; it was ours
            raise
    try:
        rec = check_stack(rec, project, wait)
    except BaseException:
        if started is not None:
            started.stop()
        raise
    return Context(project=project, host=host, stack=rec, runs_root=runs_dir(project, host),
                   started=started, **kw)


def _parse_params(items) -> dict:
    out = {}
    for it in items or []:
        k, sep, v = it.partition("=")
        if not sep or not k:
            raise Refused(f"--param {it!r}: give name=value")
        try:
            out[k] = yaml.safe_load(v)
        except yaml.YAMLError:
            out[k] = v
    return out


def run_suite(ctx: Context, specs, suite_dir: Path = None, resume=False, retry=None) -> int:
    out = ctx.out or sys.stdout
    retry = retry or {}
    times = int(retry.get("times") or 0) if (retry.get("on") or "error") == "error" else 0
    worst = EXIT_OK
    done = {}
    if suite_dir is not None:
        suite_dir.mkdir(parents=True, exist_ok=True)
        for link in suite_dir.iterdir():
            if not link.is_symlink():
                continue
            if resume and (link / "result.json").is_file():
                done[link.name] = link
            else:
                link.unlink()
    todo = [sp for sp in specs if sp.key not in done]
    if todo:
        lg = logging_for(config.project_settings(ctx.project).get("logging"), todo[0].logging)
        try:
            # retries are not budgeted: they replace a run whose logs keep prunes
            problem = budgetmod.check(lg.get("budget"), len(todo), ctx.runs_root)
        except ValueError as e:
            problem = f"logging.budget: {e}"
        if problem:
            raise Refused(problem)
    for spec in specs:
        if spec.key in done:
            r = runsmod.load(done[spec.key]) or {}
            out.write(f"skipped  {spec.key}  (resume: {r.get('verdict')} in "
                      f"{os.path.realpath(done[spec.key])})\n")
            verdict = r.get("verdict")
            if verdict in results.VERDICTS:
                worst = max(worst, outcome(results.RunResult(spec.key, verdict), spec.expect))
            continue
        for attempt in range(times + 1):
            res = run_spec(ctx, spec, attempt)
            d = ctx.runs_root / res.run_id
            out.write(line(res, d, spec.expect) + "\n")
            out.flush()
            if res.verdict != results.ERROR:
                break
        if suite_dir is not None:
            link = suite_dir / spec.key
            if link.is_symlink():
                link.unlink()
            os.symlink(d, link)
        worst = max(worst, outcome(res, spec.expect))
    if suite_dir is not None:
        csv_path, junit_path = runsmod.summarize(suite_dir)
        out.write(f"summary: {csv_path}\n")
    return worst


def main(argv=None, out=None, stack_command=None) -> int:
    out = out or sys.stdout
    ap = argparse.ArgumentParser(prog="ros2 run swb run", description=__doc__.split("\n")[0])
    ap.add_argument("target", help="a scenario (.xosc or TIER IV .yaml), a directory, or a suite")
    ap.add_argument("--repeat", type=int, help="repetitions (a suite's repetitions: otherwise)")
    ap.add_argument("--shard", help="i/n: every n-th run from the i-th (1-based)")
    ap.add_argument("--resume", action="store_true", help="skip a suite's runs with a result")
    ap.add_argument("--logs", help="the log root (default: the host settings' logs:)")
    ap.add_argument("--capture", choices=runsmod.LEVELS,
                    help="record at this level and keep it whatever the verdict "
                         "(full: + rosbags of the key topics; debug: + every topic)")
    ap.add_argument("--project", help="the project (default: found from the working dir)")
    ap.add_argument("--seat", help="the stack's seat (default: the host settings' seat:)")
    ap.add_argument("--param", action="append", metavar="NAME=VALUE",
                    help="a scenario parameter (repeatable; with a suite, one more grid axis)")
    ap.add_argument("--timeout", type=float, help=f"SSv2's global_timeout, s ({DEFAULT_TIMEOUT})")
    ap.add_argument("--wait", type=float, default=0.0,
                    help="seconds to wait for the stack to become ready (default: refuse now)")
    ap.add_argument("--ready-timeout", type=float, default=READY_TIMEOUT,
                    help=f"seconds a run waits for its pilots' readiness ({READY_TIMEOUT})")
    ap.add_argument("--start-stack", action="store_true",
                    help="start the project's stack when none is up, and leave it up")
    ap.add_argument("--stop-stack", action="store_true",
                    help="as --start-stack, and stop the stack it started after the runs "
                         "(one-shot: scripts, CI); a stack that was already up stays")
    ap.add_argument("--stack-timeout", type=float,
                    help=f"seconds a started stack has to become ready ({STACK_START_TIMEOUT:.0f})")
    try:
        args = ap.parse_args(ros_args(sys.argv[1:] if argv is None else argv))
    except SystemExit as e:
        return EXIT_OK if e.code == 0 else EXIT_REFUSED
    if args.repeat is not None and args.repeat < 1:
        sys.stderr.write("swb run: --repeat must be at least 1\n")
        return EXIT_REFUSED
    return _main(args, out, stack_command)


def _main(args, out, stack_command=None) -> int:
    ctx = None
    try:
        ctx = context(args.project, args.seat, args.logs, args.wait,
                      start=args.start_stack or args.stop_stack, stack_command=stack_command,
                      stack_timeout=args.stack_timeout, ready_timeout=args.ready_timeout,
                      out=out)
        target = Path(args.target)
        specs, suite = expand(target, ctx.project, args.repeat, _parse_params(args.param))
        if args.timeout:
            for s in specs:
                s.timeout = args.timeout
        if args.capture:
            for s in specs:
                s.logging = {**(s.logging or {}), "capture": args.capture, "keep": {}}
        if args.shard:
            specs = shard(specs, args.shard)
        specs = order_by_map(specs, ctx.stack.get("map"))
        suite_dir = None
        if suite is not None:
            name = _token(Path(args.target).stem)
            if args.shard:
                name += "_shard-" + args.shard.replace("/", "-of-").replace(" ", "")
            suite_dir = stackmod.project_root(ctx.host["logs"], ctx.project.resolve().name) \
                / "suites" / name
        elif args.resume:
            raise Refused("--resume is for suites (their runs are linked by name)")
        return run_suite(ctx, specs, suite_dir, args.resume, (suite or {}).get("retry"))
    except Refused as e:
        sys.stderr.write(f"swb run: refused: {e}\n")
        return EXIT_REFUSED
    except StackFailed as e:
        sys.stderr.write(f"swb run: error: {e}\n")
        return EXIT_INFRA
    finally:
        release_turn(ctx)
        started = ctx.started if ctx is not None else None
        if started is not None:
            if args.stop_stack:
                out.write(f"stack    stopping the stack this run started (pid "
                          f"{started.proc.pid})\n")
                out.flush()
                started.stop()
            else:
                out.write(f"stack    left up (pid {started.proc.pid}, seat {started.seat}); "
                          f"stop it with: kill -INT -{started.proc.pid}\n")
                out.flush()


class Terminated(KeyboardInterrupt):
    """SIGTERM, as an interrupt: the runner's finally blocks stop what it started (its run's
    pilots and SSv2, a stack it started with --stop-stack) instead of leaving them behind."""


def _on_term(signum, frame):
    raise Terminated()


def cli(argv=None) -> int:
    """The command line: main() with SIGTERM turned into an interrupt."""
    previous = signal.signal(signal.SIGTERM, _on_term)
    try:
        return main(argv)
    except KeyboardInterrupt:
        sys.stderr.write("swb run: interrupted; what this run started is stopped\n")
        return 130
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    sys.exit(cli())
