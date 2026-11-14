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

Not in this step: the simulator's own start (``simulator_start``: CARLA is external, as
before; Isaac's world server is part of the stack), lazy pilots (stack pilots start with the
stack), ``end_with:=run``.
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

from swb import config, project as projectmod, results, seat as seatmod
from swb.config import ConfigError

SCHEMA = "swb.stack/1"
STARTING, READY, STOPPED = "starting", "ready", "stopped"
STACK_JSON = "stack.json"
EVENTS = "events.jsonl"

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


def build_commits(workspace=None) -> dict:
    ws = workspace if workspace is not None else workspace_root()
    if ws is None:
        return {"swb": None, "submodules": {}}
    p = results.provenance(ws)
    return {"swb": p.get("swb"), "submodules": p.get("submodules") or {}}


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
         stack_args=None, check=projectmod.check_project, now=None) -> Plan:
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

    try:
        source = seatmod.seats_source(project)
        s = seatmod.resolve(host, source)
    except (ConfigError, seatmod.SeatTaken) as e:
        raise StackRefused(str(e)) from None
    h = seatmod.holder(s.index, s.lock_dir)
    if h is not None:
        raise StackRefused(f"seat {s.index} is taken: {seatmod.describe_holder(h)}")

    scenarios = scan_scenarios(project)
    maps = sorted({m for _p, sc in scenarios if (m := map_of(sc))})
    if map_name:
        chosen = map_name
    elif maps:
        chosen = maps[0]
    else:
        raise StackRefused("no scenario names a map ($(env SWB_MAPS)/<map>); give map:=")
    map_path = project / "assets" / "maps" / chosen
    if not map_path.is_dir():
        raise StackRefused(f"map {chosen} is not fetched ({map_path}); {projectmod.FETCH}")

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
        "map_path": str(map_path),
        "vehicle_model": vehicle,
        "nest_side": "true",
        "pilots": json.dumps(entries),
    }
    if sim == "isaac":
        sided = any(b.sided() for b in bound)
        args["sim_args"] = f"world:={map_path} ego:={'default' if sided else 'none'}"
    for k, v in (stack_args or {}).items():
        args[k] = v

    common = {"relay": f"tcp://localhost:{s.relay_port}", "map_path": str(map_path),
              "vehicle": vehicle, "sim": sim, "use_sim_time": "true"}
    if sim == "carla":
        common.update(carla_host=args["carla_host"], carla_port=args["carla_port"],
                      release_notify_endpoint=f"tcp://127.0.0.1:{s.release_ports[0]}",
                      release_ack_endpoint=f"tcp://127.0.0.1:{s.release_ports[1]}")

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
        "maps": maps,
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
                stack_dir=stack_dir, map=chosen, map_path=str(map_path), env=env,
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
    """The stack's directory and its stack.json (state starting); nothing is linked yet."""
    (p.stack_dir / "proc").mkdir(parents=True, exist_ok=True)
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


# --- the keeper -----------------------------------------------------------------------------

def _die_with_parent():
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        libc.prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except OSError:
        pass


def keep(stack_dir, poll=5.0, once=False) -> int:
    """Hold the seat for the stack in ``stack_dir``; mark it ready; mark it stopped at the end.

    ``once``: return right after the stack is ready (tests)."""
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
    write_json(path, rec)
    link_current(stack_dir, rec)
    event(stack_dir, "stack.start", stack_id=rec["stack_id"], pid=os.getpid())
    print(f"swb stack {rec['stack_id']}: seat {s.index}; scenario domain {s.scenario_domain}, "
          f"SSv2 {s.ssv2_port}, relay {s.relay_port}, world server {s.world_port}; "
          f"web UI: pass --web-addr 127.0.0.1:{s.web_port}", flush=True)
    try:
        waiting = [p for p in rec["pilots"] if p.get("lifetime", "stack") == "stack"]
        while waiting and not stop["now"]:
            for p in list(waiting):
                if check_ready(p.get("ready"), ros_env(rec, p["domain"])):
                    p["ready_at"] = iso_now()
                    waiting.remove(p)
                    event(stack_dir, "pilot.ready", entity=p["entity"])
                    print(f"swb stack: pilot {p['entity']} ready", flush=True)
            write_json(path, rec)
            if waiting:
                for _ in range(int(poll * 10)):
                    if stop["now"]:
                        break
                    time.sleep(0.1)
        if not stop["now"]:
            rec["state"] = READY
            rec["ready_at"] = iso_now()
            write_json(path, rec)
            event(stack_dir, "stack.ready")
            print(f"swb stack {rec['stack_id']}: ready", flush=True)
            if once:
                return 0
        while not stop["now"]:
            time.sleep(0.5)
    finally:
        if not once:
            rec = read_json(path)
            rec["state"] = STOPPED
            rec["stopped"] = iso_now()
            write_json(path, rec)
            event(stack_dir, "stack.stop")
            lock.release()
        else:
            lock.release()
        for sig, h in previous.items():
            signal.signal(sig, h)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m swb.stack")
    sub = ap.add_subparsers(dest="cmd", required=True)
    k = sub.add_parser("keep", help="hold the seat for a stack and track its readiness")
    k.add_argument("stack_dir")
    k.add_argument("--poll", type=float, default=5.0)
    a = ap.parse_args(argv)
    if a.cmd == "keep":
        _die_with_parent()
        return keep(a.stack_dir, poll=a.poll)
    return 3


if __name__ == "__main__":
    sys.exit(main())
