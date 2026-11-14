"""The stack's plan and its keeper: what ``swb.launch.py`` starts, and ``stack.json``.

docs/design/projects-and-pilots.md, "The stack". ``plan()`` runs inside the launch (at its
parse): it finds the project, reads the host settings, picks the seat (refusing a taken one),
runs ``project check`` (refusing on a problem), binds the project's ``pilots:`` to the
entities its scenarios drive from outside, gives each a domain of the seat, and turns all of
it into ``stack.launch.py``'s arguments and the environment every process gets
(``SWB_MAPS``, ``SWB_CARLA_USD``, the DDS profile).

The seat's lock must live as long as the stack, and a launch file's parse may happen in a
process that does not (play_launch parses apart from what it runs). So the launch starts a
small **keeper** process, ``python3 -m swb.stack keep <stack dir>``, declared required: it
takes the seat's flock, records its pid in ``stack.json``, links
``<logs>/<project>/seat-<n>-current`` to the stack, watches each stack pilot's readiness
(``autoware``: ``ros2 run swb_launch ego_ready`` in the pilot's domain; a ``ready:`` command
of the project's; else started is ready), sets ``state: ready``, and on SIGTERM (or its
parent's death) sets ``state: stopped`` and lets the seat go. A keeper that cannot take the
seat exits, and the launch with it.

The keeper also **runs the stack's pilots** (``lifetime: stack``), each a nested play_launch
of its kind's launch in its domain, respawned 5 s after it ends (an episode end: a world load
ends acb_bridge), so that it can restart them with another ``map_path``. A stack serves
several maps (``maps:`` of swb.yaml, else every map the project's scenarios name); its pilots
run on one at a time, ``map`` in stack.json. On CARLA they start lazily, for the first run's
map, unless ``map:=`` names one; on Isaac the world is the stack's (``world:=``), so the
first map is the stack's for its lifetime.

A runner asks for another map by writing ``switch-request.json`` (``{id, map}``) into the
stack's directory; the keeper sees a new id within 0.1 s, stops the pilots, records the map
in stack.json (``map``, ``map_path``, the pilots' contract), starts them again with the new
``map_path`` and answers in stack.json's ``switch`` (``{id, map, state: done|failed,
error}``). The run then waits for its pilots' readiness as every run does; the world server
loads the town on SSv2's Initialize.

The simulator (swb.simulator): on CARLA, ``simulator_start`` (``auto``, ``start``,
``external``) decides at the plan whether the stack starts CARLA itself; a started CARLA is
the keeper's child, waited for (its RPC) before the pilots start and the stack is ready, and
stopped with the stack. An external CARLA is never started or stopped. On Isaac the world
server starts Isaac. stack.json records which (``simulator_start``: ``started``,
``external``, ``world_server``; ``carla``: the details).
"""
import argparse
import ctypes
import datetime
import hashlib
import json
import os
import shlex
import signal
import socket
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from swb import config, ground as groundmod, project as projectmod, results, seat as seatmod
from swb import simulator as simmod
from swb.config import ConfigError

SCHEMA = "swb.stack/1"
STARTING, READY, STOPPED = "starting", "ready", "stopped"
STACK_JSON = "stack.json"
EVENTS = "events.jsonl"
SWITCH_REQUEST = "switch-request.json"
SWITCHING, DONE, FAILED = "switching", "done", "failed"
RESPAWN_DELAY = 5.0   # as stack.launch.py's nested launches
STOP_GRACE = 20.0     # SIGINT, then SIGTERM, then SIGKILL

# kind -> its launch under swb_launch's launch/pilots/ (stack.launch.py's PILOT_KINDS)
PILOT_FILES = {"autoware": "autoware.launch.xml", "carla_basic": "carla_basic.launch.py",
               "launch": "launch.launch.py", "command": "command.launch.py", "none": None}
# project pilot keys that are not launch arguments
NOT_ARGS = ("kind", "lifetime", "ready", "inputs", "args")
EGO_READY = ["ros2", "run", "swb_launch", "ego_ready", "--timeout", "10"]
DEFAULT_VEHICLE = "tesla_model3"
SIDED_KINDS = ("autoware",)  # kinds that always bring a vehicle side


class StackRefused(RuntimeError):
    """The stack must not start; ``findings`` (when from ``project check``) say why."""

    def __init__(self, msg, findings=()):
        self.findings = list(findings)
        super().__init__(msg)


# --- small parts ----------------------------------------------------------------------------

def now_id(t=None) -> str:
    return (t or datetime.datetime.now()).strftime("%Y-%m-%dT%H-%M-%S")


def iso_now() -> str:
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def _sha(path: Path):
    return results.file_sha256(path) if Path(path).is_file() else None


def _canon(value) -> str:
    return "sha256:" + hashlib.sha256(
        json.dumps(value, sort_keys=True, default=str).encode()).hexdigest()


def workspace_root():
    """The swb checkout this code runs from (a symlink install resolves into it), or None."""
    here = Path(__file__).resolve()
    for d in here.parents:
        if (d / ".git").exists() and (d / "src" / "swb").is_dir():
            return d
    return None


def source_id(ws) -> str:
    """The code the stack runs, not the commit that holds it: the tree of the checkout's
    ``src/`` (``src:<tree>``), plus a hash of its uncommitted changes there (``+<sha>``). A
    commit of the same tree, or an edit of the docs, leaves it as it was."""
    def git(*a):
        try:
            return subprocess.run(["git", *a], cwd=str(ws), capture_output=True, text=True,
                                  timeout=20).stdout
        except (OSError, subprocess.TimeoutExpired):
            return ""
    tree = git("rev-parse", "HEAD:src").strip()
    if not tree:
        return None
    diff = git("diff", "HEAD", "--ignore-submodules=all", "--", "src")
    return f"src:{tree}" + (f"+{hashlib.sha256(diff.encode()).hexdigest()[:12]}" if diff else "")


def build_commits(workspace=None) -> dict:
    ws = workspace if workspace is not None else workspace_root()
    if ws is None:
        return {"swb": None, "submodules": {}}
    p = results.provenance(ws)
    return {"swb": source_id(ws) or p.get("swb"), "submodules": p.get("submodules") or {}}


def fingerprint(project: Path, settings: dict = None, build: dict = None) -> dict:
    """What a run must agree with: the project file, the lock, the pilots, the build."""
    project = Path(project)
    settings = settings if settings is not None else config.project_settings(project)
    return {
        "swb_yaml": _sha(project / config.PROJECT_FILE),
        "swb_lock": _sha(project / config.LOCK_FILE),
        "pilots": _canon(settings.get("pilots") or {}),
        "build": build if build is not None else build_commits(),
    }


def fingerprint_diff(stack_fp: dict, project_fp: dict) -> list:
    """One phrase per part that differs, naming both sides."""
    names = {"swb_yaml": config.PROJECT_FILE, "swb_lock": config.LOCK_FILE,
             "pilots": "pilots:", "build": "the build"}
    out = []
    for k, name in names.items():
        a, b = (stack_fp or {}).get(k), (project_fp or {}).get(k)
        if a == b:
            continue
        if k == "build":
            a, b = a or {}, b or {}
            if a.get("swb") != b.get("swb"):
                out.append(f"the build (stack swb {a.get('swb')}, now {b.get('swb')})")
            subs = sorted(p for p in set(a.get("submodules") or {}) | set(b.get("submodules") or {})
                          if (a.get("submodules") or {}).get(p) != (b.get("submodules") or {}).get(p))
            if subs:
                out.append("the build's submodules (" + ", ".join(subs) + ")")
            continue
        out.append(f"{name} (stack {_short(a)}, project {_short(b)})")
    return out


def _short(h):
    if h is None:
        return "none"
    return h[:19] if isinstance(h, str) else str(h)


def map_of(sc) -> str:
    lf = sc.logic_file or ""
    if lf.startswith(projectmod.MAPS_PREFIX):
        return lf[len(projectmod.MAPS_PREFIX):].strip("/")
    return None


def substitute(value, ctx: dict, where: str):
    """``{run_dir}``, ``{seed}``, ``{run_id}``, ``{param:<name>}`` in strings, lists, dicts.
    A ``{param:x}`` the context lacks raises StackRefused naming ``where``."""
    if isinstance(value, dict):
        return {k: substitute(v, ctx, where) for k, v in value.items()}
    if isinstance(value, list):
        return [substitute(v, ctx, where) for v in value]
    if not isinstance(value, str):
        return value
    out, i = [], 0
    while i < len(value):
        j = value.find("{", i)
        if j < 0:
            out.append(value[i:])
            break
        k = value.find("}", j)
        if k < 0:
            out.append(value[i:])
            break
        name = value[j + 1:k]
        out.append(value[i:j])
        if name.startswith("param:"):
            p = name[len("param:"):]
            params = ctx.get("params") or {}
            if p not in params:
                raise StackRefused(f"{where}: {{param:{p}}} but the run has no parameter {p}"
                                   + ("" if ctx.get("run_id") else
                                      " (a lifetime: stack pilot has no run; lifetime: run)"))
            out.append(str(params[p]))
        elif name in ("run_dir", "seed", "run_id"):
            v = ctx.get(name)
            out.append("" if v is None else str(v))
        else:
            out.append(value[j:k + 1])
        i = k + 1
    return "".join(out)


def _absolute_words(words, project: Path):
    """A command's words that name a file of the project, made absolute (the stack's
    processes do not run in the project directory when it came from project:=)."""
    out = []
    for w in words:
        w = str(w)
        if w and not w.startswith(("-", "/")) and (project / w).exists():
            out.append(str((project / w).resolve()))
        else:
            out.append(w)
    return out


# --- binding pilots -------------------------------------------------------------------------

@dataclass
class Bound:
    entity: str
    key: str                      # the swb.yaml pilots: key that matched
    spec: dict
    domain: int = None
    scenarios: list = field(default_factory=list)

    @property
    def kind(self):
        return self.spec.get("kind")

    @property
    def lifetime(self):
        return self.spec.get("lifetime") or "stack"

    def sided(self) -> bool:
        if self.kind in SIDED_KINDS:
            return True
        vs = (self.spec.get("args") or {}).get("vehicle_side", self.spec.get("vehicle_side"))
        if self.kind == "launch":
            return str(vs if vs is not None else "true").lower() == "true"
        if self.kind == "command":
            return str(vs if vs is not None else "false").lower() == "true"
        return False


def scan_scenarios(project: Path):
    """``[(path, Scenario)]`` of the project (scenarios/ and the suites'), unreadable ones
    left out (``project check`` names them)."""
    files, _ = projectmod.collect_scenarios(project)
    out = []
    for f in files:
        try:
            out.append((f, projectmod.read_scenario(f)))
        except (ValueError, OSError):
            continue
    return out


def bind_pilots(settings: dict, scenarios) -> list:
    """One Bound per externally driven entity of the scenarios that a pilot matches; ``ego``
    first, then by name."""
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    found = {}
    for path, sc in scenarios:
        for ent in sc.entities:
            if not ent.external:
                continue
            key = projectmod.pilot_for(pilots, ent.name)
            if key is None:
                continue
            b = found.get(ent.name)
            if b is None:
                b = found[ent.name] = Bound(ent.name, key, dict(pilots[key] or {}))
            b.scenarios.append(str(path))
    return sorted(found.values(), key=lambda b: (b.entity != "ego", b.entity))


def assign_domains(bound: list, s: seatmod.Seat, simulator: str):
    """Pilot domains of the seat, in order; on Isaac the one pilot with a vehicle side takes
    the first (the world server's rig publishes there)."""
    pool = list(s.pilot_domains)
    if len(bound) > len(pool):
        raise StackRefused(f"{len(bound)} pilots ({', '.join(b.entity for b in bound)}) but seat "
                           f"{s.index} has {len(pool)} pilot domains (seats.domains.stride - 1)")
    order = list(bound)
    if simulator == "isaac":
        sided = [b for b in bound if b.sided()]
        if len(sided) > 1:
            raise StackRefused("on Isaac one pilot has a vehicle side (one ego): "
                               + ", ".join(b.entity for b in sided))
        order = sided + [b for b in bound if b not in sided]
    for b, d in zip(order, pool):
        b.domain = d
    return bound


def pilot_entry(b: Bound, project: Path, ctx: dict) -> dict:
    """A bound pilot as stack.launch.py's ``pilots:`` takes it (and the runner, for ``lifetime:
    run``): entity, kind, domain, launch arguments, substitutions applied."""
    where = f"pilots.{b.key} ({b.entity})"
    spec = substitute(b.spec, ctx, where)
    entry = {"entity": b.entity, "kind": b.kind, "domain": b.domain}
    for k, v in spec.items():
        if k in NOT_ARGS:
            continue
        if k == "run" and isinstance(v, list):
            v = _absolute_words(v, project)
        entry[k] = v
    args = spec.get("args") or {}
    if args:
        entry["args"] = dict(args)
    entry.setdefault("seed", ctx.get("seed") if ctx.get("seed") is not None else 0)
    if ctx.get("run_dir"):
        entry.setdefault("run_dir", ctx["run_dir"])
    return entry


def ready_command(spec: dict):
    """The pilot's readiness check as argv, or None when started is ready."""
    r = spec.get("ready")
    if r in (None, "", "default"):
        return list(EGO_READY) if spec.get("kind") == "autoware" else None
    if r in ("started", False, "none"):
        return None
    if isinstance(r, list):
        return [str(x) for x in r]
    return shlex.split(str(r))


def spec_hash_of(vehicle):
    """The car's spec_hash from its description package, or None."""
    if not vehicle:
        return None
    try:
        from ament_index_python.packages import get_package_share_directory
        import yaml
        p = Path(get_package_share_directory(f"{vehicle}_description")) / "config" / "spec_hash.yaml"
        return (yaml.safe_load(p.read_text()) or {}).get("spec_hash")
    except Exception:
        return None


def swb_launch_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("swb_launch")
    except Exception:
        return str(Path(__file__).resolve().parents[2] / "swb_launch")


# --- maps -----------------------------------------------------------------------------------

def map_dir(project, name) -> Path:
    return Path(project) / "assets" / "maps" / name


def served_maps(settings: dict, scenarios) -> list:
    """The maps a stack serves: swb.yaml's ``maps:``, else every map the scenarios name."""
    listed = settings.get("maps")
    if isinstance(listed, list) and listed:
        return [str(m) for m in listed]
    return sorted({m for _p, sc in scenarios if (m := map_of(sc))})


def contract_for_map(contract: dict, project, sim, name) -> dict:
    """The pilots' common contract with the map's parts: ``map_path`` and, on CARLA, the
    state dir's signal table."""
    out = {k: v for k, v in contract.items()
           if k not in ("map_path", "traffic_light_state_path")}
    if name:
        out["map_path"] = str(map_dir(project, name))
        if sim == "carla":
            out["traffic_light_state_path"] = str(
                Path(project) / "state" / name / "carla" / "traffic_lights.yaml")
    return out


def switch_problem(rec: dict, name: str):
    """Why the stack of ``rec`` cannot move its pilots to map ``name`` (None: it can)."""
    if name == rec.get("map"):
        return None
    if not map_dir(rec["project"], name).is_dir():
        return (f"map {name} is not fetched ({map_dir(rec['project'], name)}); "
                f"{projectmod.FETCH}")
    if rec.get("simulator") == "isaac":
        return (f"the scenario is on map {name}, the stack's Isaac world on {rec.get('map')} "
                f"(the world server loads it at start): restart the stack with map:={name}")
    if name not in (rec.get("maps") or []):
        return (f"map {name} is not one of the stack's maps ({', '.join(rec.get('maps') or [])});"
                " add it to maps: in swb.yaml and restart the stack")
    return None


def _named(text) -> dict:
    out = {}
    for tok in shlex.split(text or ""):
        k, sep, v = tok.partition(":=")
        if sep and k:
            out[k] = v
    return out


# --- the plan -------------------------------------------------------------------------------

@dataclass
class Plan:
    project: Path
    name: str
    simulator: str
    seat: seatmod.Seat
    stack_id: str
    stack_dir: Path
    map: str
    map_path: str
    env: dict
    launch_args: dict
    record: dict
    pilots: list
    host: dict

    @property
    def stack_json(self) -> Path:
        return self.stack_dir / STACK_JSON


def project_root(logs: str, name: str) -> Path:
    return Path(logs).expanduser() / name


def current_link(logs: str, name: str, index: int) -> Path:
    return project_root(logs, name) / f"seat-{index}-current"


def plan(project_arg=None, seat_arg=None, simulator=None, map_name=None, logs=None, gpu=None,
         stack_args=None, check=projectmod.check_project, now=None, simulator_start=None,
         probe=simmod.probe) -> Plan:
    """Everything the stack is, before anything starts; StackRefused says why it must not."""
    try:
        project = config.find_project(project_arg or None)
    except ConfigError as e:
        raise StackRefused(str(e)) from None
    overrides = {}
    if seat_arg not in (None, ""):
        overrides["seat"] = seat_arg
    if logs:
        overrides["logs"] = logs
    if gpu not in (None, ""):
        overrides["gpu"] = int(gpu)
    try:
        host = config.host_settings(project, overrides)
        psettings = config.project_settings(project)
    except ConfigError as e:
        raise StackRefused(str(e)) from None

    if check is not None:
        findings = check(project)
        if findings:
            raise StackRefused("project check found problems:\n" +
                               "\n".join(str(f) for f in findings), findings)

    sim = simulator or psettings.get("simulator")
    if sim not in projectmod.SIMULATORS:
        raise StackRefused(f"simulator {sim!r}: this stack starts carla or isaac")
    if simulator_start not in (None, "") and simulator_start not in simmod.MODES:
        raise StackRefused(f"simulator_start {simulator_start!r}: one of "
                           + ", ".join(simmod.MODES))

    try:
        source = seatmod.seats_source(project)
        s = seatmod.resolve(host, source)
    except (ConfigError, seatmod.SeatTaken) as e:
        raise StackRefused(str(e)) from None
    h = seatmod.holder(s.index, s.lock_dir)
    if h is not None:
        raise StackRefused(f"seat {s.index} is taken: {seatmod.describe_holder(h)}")

    scenarios = scan_scenarios(project)
    maps = served_maps(psettings, scenarios)
    if map_name and map_name not in maps:
        maps.append(map_name)
    if not maps:
        raise StackRefused("no scenario names a map ($(env SWB_MAPS)/<map>); give map:= "
                           "or maps: in swb.yaml")
    for m in maps:
        if not map_dir(project, m).is_dir():
            raise StackRefused(f"map {m} is not fetched ({map_dir(project, m)}); "
                               f"{projectmod.FETCH}")
    # CARLA: the pilots start for the first run's map unless map:= names one; Isaac: the
    # world server loads its world at start, so the stack has a map from the beginning
    chosen = map_name or (maps[0] if sim == "isaac" else None)
    map_path = map_dir(project, chosen) if chosen else None

    bound = assign_domains(bind_pilots(psettings, scenarios), s, sim)
    stack_pilots = [b for b in bound if b.lifetime == "stack"]
    ctx = {"seed": 0, "run_dir": "", "run_id": "", "params": {}}
    entries = []
    for b in stack_pilots:
        entries.append(pilot_entry(b, project, ctx))

    name = project.resolve().name
    stack_id = f"{now_id(now)}_seat{s.index}"
    stack_dir = project_root(host["logs"], name) / "stacks" / stack_id
    proc = stack_dir / "proc"

    usd = project / "assets" / "carla-usd"
    env = {
        "SWB_MAPS": str(project / "assets" / "maps"),
        "SWB_CARLA_USD": str(usd),
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "CYCLONEDDS_URI": f"file://{seatmod.cyclone_profile()}",
    }
    if host.get("gpu") is not None or (host.get("seats") or {}).get("gpus") is not None:
        env["CUDA_VISIBLE_DEVICES"] = str(s.gpu)

    ego_domain = s.pilot_domains[0]
    vehicle = next((b.spec.get("vehicle") for b in bound if b.spec.get("vehicle")),
                   DEFAULT_VEHICLE)
    carla = host.get("carla") or {}
    carla_run = None
    if sim == "carla":
        try:
            carla_run = simmod.decide(carla, simulator_start or None, probe)
        except simmod.SimulatorRefused as e:
            raise StackRefused(str(e)) from None
    args = {
        "sim": sim,
        "ego_launch": "none",
        "ego_domain": str(ego_domain),
        "scenario_domain": str(s.scenario_domain),
        "ssv2_port": str(s.ssv2_port),
        "agent_port": str(s.relay_port),
        "world_port": str(s.world_port),
        "carla_host": str(carla.get("host") or ""),
        "carla_port": str(carla.get("port") or ""),
        "log_dir": str(proc),
        "map_path": str(map_path or ""),
        "vehicle_model": vehicle,
        "nest_side": "true",
        # every stack process on the profile the runs use (a sourced Autoware exports its own)
        "dds_config": f"file://{seatmod.cyclone_profile()}",
        # the bridge reads a map's signal table from the same state dir first (both
        # simulators: an Isaac world finds its lamps by the table CARLA's server wrote)
        "side_args": f"state_dir:={project / 'state'}",
    }
    # the world server writes what it makes for a map (CARLA's signal table, Isaac's ruts
    # and surfaces) into the project's state dir, never into the fetched (read-only) map
    if sim == "carla":
        args["sim_args"] = f"state_dir:={project / 'state'}"
    if sim == "isaac":
        sided = any(b.sided() for b in bound)
        args["sim_args"] = (f"world:={map_path} ego:={'default' if sided else 'none'}"
                            f" state_dir:={project / 'state'}")
    for k, v in (stack_args or {}).items():
        args[k] = v

    common = {"relay": f"tcp://localhost:{s.relay_port}", "vehicle": vehicle, "sim": sim,
              "use_sim_time": "true"}
    if sim == "carla":
        common.update(carla_host=args["carla_host"], carla_port=args["carla_port"],
                      release_notify_endpoint=f"tcp://127.0.0.1:{s.release_ports[0]}",
                      release_ack_endpoint=f"tcp://127.0.0.1:{s.release_ports[1]}")
        # as stack.launch.py: the spec's physics read back when the world server applies it
        # carla_basic's `agents` package: CARLA's PythonAPI under the host's carla.root
        root = str(carla.get("root") or "")
        if root:
            common["carla_pythonapi"] = os.path.join(os.path.expanduser(root), "PythonAPI",
                                                     "carla")
        physics = _named(args.get("sim_args")).get("vehicle_physics", "none")
        if physics and physics != "none":
            common.update(carla_physics=physics, carla_physics_mode="verify")
    common = contract_for_map(common, project, sim, chosen)

    pilots = []
    for b in bound:
        pilots.append({"entity": b.entity, "key": b.key, "kind": b.kind, "lifetime": b.lifetime,
                       "domain": b.domain, "vehicle": b.spec.get("vehicle"),
                       "inputs": b.spec.get("inputs") or [],
                       "ready": ready_command(b.spec), "spec": b.spec,
                       "ready_at": None})
    numbers = s.numbers()
    record = {
        "schema": SCHEMA,
        "stack_id": stack_id,
        "project": str(project),
        "project_name": name,
        "simulator": sim,
        "map": chosen,
        "map_path": str(map_path) if map_path else None,
        "maps": maps,
        "simulator_start": carla_run["owner"] if carla_run else simmod.WORLD_SERVER,
        "carla": carla_run,
        "switch": None,
        "fingerprint": fingerprint(project, psettings),
        "seat": numbers,
        "lock_dir": s.lock_dir,
        "domains": {"scenario": s.scenario_domain, "ego": ego_domain,
                    "pilots": {b.entity: b.domain for b in bound}},
        "ports": {"ssv2": s.ssv2_port, "release": list(s.release_ports),
                  "relay": s.relay_port, "world": s.world_port, "web": s.web_port,
                  "web_run": s.web_port_run},
        "env": env,
        "pilots": pilots,
        "pilot_entries": entries,
        "pilot_contract": common,
        "pilots_dir": str(Path(swb_launch_share()) / "launch" / "pilots"),
        "spec_hash": {b.entity: h for b in bound if (h := spec_hash_of(b.spec.get("vehicle")))},
        "proc": str(proc),
        "launch_args": args,
        "pid": None,
        "state": STARTING,
        "started": iso_now(),
        "ready_at": None,
        "host": socket.gethostname(),
    }
    return Plan(project=project, name=name, simulator=sim, seat=s, stack_id=stack_id,
                stack_dir=stack_dir, map=chosen, map_path=str(map_path) if map_path else None,
                env=env,
                launch_args=args, record=record, pilots=pilots, host=host)


# --- stack.json -----------------------------------------------------------------------------

def write_json(path: Path, data: dict) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + f".{os.getpid()}.tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(path)
    return path


def read_json(path: Path) -> dict:
    return json.loads(Path(path).read_text())


def write_plan(p: Plan) -> Path:
    """The stack's directory and its stack.json (state starting); the project's ground files
    linked into its state dir (Isaac reads them there); nothing else is linked yet."""
    (p.stack_dir / "proc").mkdir(parents=True, exist_ok=True)
    project = Path(p.record["project"])
    try:
        groundmod.apply(project, config.project_settings(project).get("ground"))
    except groundmod.GroundError as e:
        raise StackRefused(str(e)) from None
    return write_json(p.stack_json, p.record)


def event(stack_dir: Path, name: str, **kw):
    with open(Path(stack_dir) / EVENTS, "a") as f:
        f.write(json.dumps({"t_wall": time.time(), "event": name, **kw}) + "\n")


def link_current(stack_dir: Path, record: dict) -> Path:
    link = Path(stack_dir).parent.parent / f"seat-{record['seat']['seat']}-current"
    tmp = link.with_name(link.name + f".{os.getpid()}.tmp")
    if tmp.is_symlink() or tmp.exists():
        tmp.unlink()
    os.symlink(os.path.relpath(stack_dir, link.parent), tmp)
    tmp.replace(link)
    return link


def seat_of(record: dict) -> seatmod.Seat:
    n = dict(record["seat"])
    return seatmod.Seat(index=n["seat"], scenario_domain=n["scenario_domain"],
                        pilot_domains=n["pilot_domains"], ssv2_port=n["ssv2_port"],
                        release_ports=tuple(n["release_ports"]), relay_port=n["relay_port"],
                        world_port=n["world_port"], web_port=n["web_port"],
                        web_port_run=n["web_port_run"], gpu=n["gpu"],
                        lock_dir=record.get("lock_dir") or config.DEFAULTS["seat_locks"])


def ros_env(record: dict, domain) -> dict:
    env = dict(os.environ)
    env.update(record.get("env") or {})
    env["ROS_DOMAIN_ID"] = str(domain)
    return env


def check_ready(cmd, env, timeout=60) -> bool:
    if not cmd:
        return True
    try:
        return subprocess.run(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              timeout=timeout).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


# --- pilot processes ------------------------------------------------------------------------

def pilot_argv(rec: dict, p: dict, entry: dict, log_dir: Path) -> list:
    """A pilot as a nested play_launch of its kind's launch: the stack's contract (map_path
    of the stack's current map), then the entry's own arguments; None for kind none."""
    name = PILOT_FILES.get(p["kind"])
    if name is None:
        return None
    args = dict(rec.get("pilot_contract") or {})
    args["entity"] = p["entity"]
    for k, v in entry.items():
        if k in ("entity", "kind", "domain", "args"):
            continue
        args[k] = v
    args.update(entry.get("args") or {})
    words = []
    for k, v in args.items():
        if isinstance(v, bool):
            v = "true" if v else "false"
        elif isinstance(v, (list, tuple)):
            v = shlex.join(str(x) for x in v)
        v = "" if v is None else str(v)
        if v != "":
            words.append(f"{k}:={v}")
    extra = ([] if p["kind"] == "carla_basic"
             else ["--container-mode", "isolated", "--composable-respawn", "on-crash"])
    return (["play_launch", "launch", "--disable-web-ui", "--log-dir", str(log_dir), *extra,
             os.path.join(rec["pilots_dir"], name)] + words)


def start_process(argv, env, log: Path, cwd=None, die_with_parent=False):
    """``argv`` in its own session (its group is signalled as one), output to ``log``."""
    log.parent.mkdir(parents=True, exist_ok=True)
    fh = open(log, "ab")
    try:
        return subprocess.Popen(argv, env=env, stdout=fh, stderr=subprocess.STDOUT, cwd=cwd,
                                start_new_session=True,
                                preexec_fn=_die_with_parent if die_with_parent else None)
    finally:
        fh.close()


def stop_processes(procs, grace=STOP_GRACE):
    """SIGINT to every process group, then SIGTERM, then SIGKILL, waiting for all at once."""
    live = [p for p in procs if p is not None and p.poll() is None]
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        if not live:
            return
        for p in live:
            try:
                os.killpg(p.pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        deadline = time.time() + wait
        while time.time() < deadline:
            live = [p for p in live if p.poll() is None]
            if not live:
                return
            time.sleep(0.05)


class Pilots:
    """The stack's pilots as the keeper's children: started, respawned, stopped."""

    def __init__(self, stack_dir: Path, command=None, respawn_delay=RESPAWN_DELAY):
        self.stack_dir = Path(stack_dir)
        self.command = command or pilot_argv
        self.respawn_delay = respawn_delay
        self.procs = {}   # entity -> Popen
        self.due = {}     # entity -> wall time of its (re)start

    @staticmethod
    def stack_pilots(rec):
        entries = {e["entity"]: e for e in rec.get("pilot_entries") or []}
        return [(p, entries[p["entity"]]) for p in rec.get("pilots", [])
                if p.get("lifetime", "stack") == "stack" and p["entity"] in entries]

    def _start(self, rec, p, entry):
        log_dir = Path(rec["proc"]) / f"pilot-{p['entity']}"
        argv = self.command(rec, p, entry, log_dir)
        if not argv:
            return
        try:
            proc = start_process(argv, ros_env(rec, p["domain"]), log_dir / "console.log",
                                 cwd=rec.get("project"), die_with_parent=True)
        except OSError as e:
            event(self.stack_dir, "pilot.error", entity=p["entity"], error=str(e))
            print(f"swb stack: pilot {p['entity']} did not start: {e}", file=sys.stderr,
                  flush=True)
            self.due[p["entity"]] = time.time() + self.respawn_delay
            return
        self.procs[p["entity"]] = proc
        p["pid"] = proc.pid
        p["started_at"] = iso_now()
        p["map"] = rec.get("map")
        event(self.stack_dir, "pilot.start", entity=p["entity"], pid=proc.pid,
              map=rec.get("map"))

    def start_all(self, rec):
        self.due.clear()
        for p, entry in self.stack_pilots(rec):
            self._start(rec, p, entry)

    def stop_all(self, rec=None):
        self.due.clear()
        procs = list(self.procs.values())
        self.procs.clear()
        stop_processes(procs)
        for p in (rec or {}).get("pilots", []):
            p["pid"] = None

    def supervise(self, rec) -> bool:
        """Respawn what ended; True when stack.json should be written."""
        changed = False
        now = time.time()
        by_entity = dict((p["entity"], (p, e)) for p, e in self.stack_pilots(rec))
        for entity, proc in list(self.procs.items()):
            code = proc.poll()
            if code is None:
                continue
            del self.procs[entity]
            self.due[entity] = now + self.respawn_delay
            p = by_entity[entity][0]
            p["pid"] = None
            p["restarts"] = int(p.get("restarts") or 0) + 1
            event(self.stack_dir, "pilot.exit", entity=entity, code=code)
            print(f"swb stack: pilot {entity} ended ({code}); respawn in "
                  f"{self.respawn_delay:.0f} s", flush=True)
            changed = True
        for entity, at in list(self.due.items()):
            if now >= at and entity in by_entity:
                del self.due[entity]
                self._start(rec, *by_entity[entity])
                changed = True
        return changed


# --- the map switch -------------------------------------------------------------------------

def request_switch(stack_dir: Path, name: str, request_id: str = None) -> str:
    """Ask the keeper of ``stack_dir`` to move its pilots to map ``name``; the request's id."""
    rid = request_id or f"{os.getpid()}-{time.time_ns()}"
    write_json(Path(stack_dir) / SWITCH_REQUEST,
               {"id": rid, "map": name, "requested": iso_now(), "by": os.getpid()})
    return rid


def read_request(stack_dir: Path):
    try:
        req = read_json(Path(stack_dir) / SWITCH_REQUEST)
    except (OSError, ValueError):
        return None
    return req if isinstance(req, dict) and req.get("id") and req.get("map") else None


def switch_map(stack_dir: Path, rec: dict, req: dict, pilots: Pilots) -> bool:
    """Carry out ``req`` (``{id, map}``) on the stack of ``rec``: stop the pilots, record the
    map, start them on it; the answer in ``rec["switch"]``, written to stack.json."""
    path = Path(stack_dir) / STACK_JSON
    name = req["map"]
    sw = {"id": req["id"], "map": name, "from": rec.get("map"), "state": SWITCHING,
          "requested": req.get("requested"), "started": iso_now()}
    rec["switch"] = sw
    problem = switch_problem(rec, name)
    if problem:
        sw.update(state=FAILED, error=problem, ended=iso_now())
        write_json(path, rec)
        event(stack_dir, "map.switch_failed", map=name, error=problem)
        print(f"swb stack: no switch to map {name}: {problem}", file=sys.stderr, flush=True)
        return False
    if name == rec.get("map") and pilots.procs:
        sw.update(state=DONE, ended=iso_now())
        write_json(path, rec)
        return True
    write_json(path, rec)
    event(stack_dir, "map.switch", map=name, previous=rec.get("map"))
    print(f"swb stack: pilots to map {name} (from {rec.get('map') or 'none'})", flush=True)
    pilots.stop_all(rec)
    rec["map"] = name
    rec["map_path"] = str(map_dir(rec["project"], name))
    rec["pilot_contract"] = contract_for_map(rec.get("pilot_contract") or {}, rec["project"],
                                             rec.get("simulator"), name)
    for p in rec.get("pilots", []):
        if p.get("lifetime", "stack") == "stack":
            p["ready_at"] = None
    pilots.start_all(rec)
    sw.update(state=DONE, ended=iso_now())
    write_json(path, rec)
    event(stack_dir, "map.switched", map=name)
    return True


# --- the keeper -----------------------------------------------------------------------------

def _die_with_parent():
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        libc.prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except OSError:
        pass


def _waiting(rec):
    if not rec.get("map"):
        return []
    return [p for p in rec["pilots"]
            if p.get("lifetime", "stack") == "stack" and not p.get("ready_at")]


def keep(stack_dir, poll=5.0, once=False, pilot_command=None,
         respawn_delay=RESPAWN_DELAY) -> int:
    """Hold the seat for the stack in ``stack_dir``; run its pilots on the stack's map and
    move them to another on request; mark it ready, and stopped at the end.

    ``once``: return right after the stack is ready (tests). ``pilot_command``:
    ``(rec, pilot, entry, log_dir) -> argv`` in place of ``pilot_argv`` (tests)."""
    stack_dir = Path(stack_dir)
    path = stack_dir / STACK_JSON
    rec = read_json(path)
    s = seat_of(rec)
    try:
        lock = seatmod.acquire(s, rec.get("project"), s.lock_dir)
    except seatmod.SeatTaken as e:
        rec["state"] = STOPPED
        rec["error"] = str(e)
        write_json(path, rec)
        print(f"swb stack: {e}", file=sys.stderr)
        return 3
    stop = {"now": False}

    def on_signal(*_):
        stop["now"] = True

    previous = {sig: signal.signal(sig, on_signal) for sig in (signal.SIGTERM, signal.SIGINT)}
    rec["pid"] = os.getpid()
    # a request left by an earlier stack in this directory is not this one's
    old = read_request(stack_dir)
    if old is not None and not rec.get("switch"):
        rec["switch"] = {"id": old["id"], "map": old["map"], "state": FAILED,
                         "error": "requested before this keeper started"}
    write_json(path, rec)
    link_current(stack_dir, rec)
    event(stack_dir, "stack.start", stack_id=rec["stack_id"], pid=os.getpid())
    print(f"swb stack {rec['stack_id']}: seat {s.index}; scenario domain {s.scenario_domain}, "
          f"SSv2 {s.ssv2_port}, relay {s.relay_port}, world server {s.world_port}; "
          f"web UI: pass --web-addr 127.0.0.1:{s.web_port}; pilots on "
          f"{rec.get('map') or 'the first run' + chr(39) + 's map'}", flush=True)
    pilots = Pilots(stack_dir, pilot_command, respawn_delay)
    carla = None
    code = 0
    try:
        if rec.get("simulator_start") == simmod.STARTED:
            carla = simmod.Carla(rec["carla"], rec["proc"], respawn_delay=respawn_delay,
                                 events=lambda n, **kw: event(stack_dir, n, **kw))
            c = rec["carla"]
            # the plan saw the port free; something may have taken it since
            taken, _v = simmod.probe(c["host"], c["port"])
            if taken != simmod.FREE:
                c.update(state=simmod.FAILED, error=f"{c['host']}:{c['port']} was taken after "
                         "the plan (another CARLA?); restart the stack")
                carla = None
            else:
                carla.start()
                print(f"swb stack: CARLA started on {c['host']}:{c['port']} (pid {c['pid']}, "
                      f"log {c['log']}); waiting for its RPC, up to {c['ready_timeout']:.0f} s",
                      flush=True)
            write_json(path, rec)
        pilots_on = False
        next_check = 0.0
        while not stop["now"]:
            if carla is not None and carla.poll():
                write_json(path, rec)
                if carla.ready:
                    print(f"swb stack: CARLA serves RPC ({rec['carla'].get('version')})",
                          flush=True)
            c = rec.get("carla") or {}
            if rec.get("simulator_start") == simmod.STARTED and c.get("state") == simmod.FAILED:
                rec["error"] = c.get("error")
                print(f"swb stack: {c.get('error')}", file=sys.stderr, flush=True)
                code = 2
                break
            sim_ready = carla is None or carla.ready
            if not sim_ready and rec.get("state") == READY:
                rec["state"] = STARTING   # a started CARLA died; respawned, waited for again
                write_json(path, rec)
            if not pilots_on and sim_ready:
                pilots_on = True
                if rec.get("map") and not pilots.procs:
                    pilots.start_all(rec)
                    write_json(path, rec)
            req = read_request(stack_dir)
            if req is not None and req["id"] != (rec.get("switch") or {}).get("id"):
                switch_map(stack_dir, rec, req, pilots)
                next_check = 0.0
            if pilots.supervise(rec):
                write_json(path, rec)
            waiting = _waiting(rec)
            if waiting and pilots_on and time.time() >= next_check:
                for p in waiting:
                    if stop["now"]:
                        break
                    if check_ready(p.get("ready"), ros_env(rec, p["domain"])):
                        p["ready_at"] = iso_now()
                        event(stack_dir, "pilot.ready", entity=p["entity"], map=rec.get("map"))
                        print(f"swb stack: pilot {p['entity']} ready on {rec.get('map')}",
                              flush=True)
                write_json(path, rec)
                next_check = time.time() + poll
                waiting = _waiting(rec)
            if not waiting and sim_ready and rec.get("state") != READY:
                rec["state"] = READY
                rec["ready_at"] = iso_now()
                write_json(path, rec)
                event(stack_dir, "stack.ready")
                print(f"swb stack {rec['stack_id']}: ready", flush=True)
                if once:
                    return 0
            time.sleep(0.1)
    finally:
        pilots.stop_all(rec)
        if carla is not None:
            carla.stop()   # the stack's own CARLA only; an external one is never touched
            write_json(path, rec)
        if not once or code:
            rec["state"] = STOPPED
            rec["stopped"] = iso_now()
            write_json(path, rec)
            event(stack_dir, "stack.stop")
        lock.release()
        for sig, h in previous.items():
            signal.signal(sig, h)
    return code


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m swb.stack")
    sub = ap.add_subparsers(dest="cmd", required=True)
    k = sub.add_parser("keep", help="hold the seat for a stack, run its pilots, track readiness")
    k.add_argument("stack_dir")
    k.add_argument("--poll", type=float, default=5.0)
    k.add_argument("--respawn-delay", type=float, default=RESPAWN_DELAY)
    k.add_argument("--pilot-wrap", metavar="PROGRAM",
                   help="run each pilot's command line under PROGRAM (tests, debugging)")
    a = ap.parse_args(argv)
    if a.cmd == "keep":
        _die_with_parent()
        command = None
        if a.pilot_wrap:
            wrap = shlex.split(a.pilot_wrap)

            def command(rec, p, entry, log_dir):
                argv = pilot_argv(rec, p, entry, log_dir)
                return wrap + argv if argv else None
        return keep(a.stack_dir, poll=a.poll, pilot_command=command,
                    respawn_delay=a.respawn_delay)
    return 3


if __name__ == "__main__":
    sys.exit(main())
