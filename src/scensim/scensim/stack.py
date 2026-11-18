"""The stack's plan and its keeper: what ``scensim.launch.py`` starts, and ``stack.json``.

docs/design/projects-and-pilots.md, "The stack". ``plan()`` runs inside the launch (at its
parse): it finds the project, reads the host settings, picks the seat (refusing a taken one),
runs ``project check`` (refusing on a problem), binds the project's ``pilots:`` to the
entities its scenarios drive from outside, gives each a domain of the seat, and turns all of
it into ``stack.launch.py``'s arguments and the environment every process gets
(``SCENSIM_MAPS``, ``SCENSIM_CARLA_USD``, the DDS profile).

The seat's lock must live as long as the stack, and a launch file's parse may happen in a
process that does not (play_launch parses apart from what it runs). So the launch starts a
small **keeper** process, ``python3 -m scensim.stack keep <stack dir>``, declared required: it
takes the seat's flock, records its pid in ``stack.json``, links
``<logs>/<project>/seat-<n>-current`` to the stack, watches each stack pilot's readiness
(``autoware``: ``ros2 run scensim_launch ego_ready`` in the pilot's domain; a ``ready:`` command
of the project's; else started is ready), sets ``state: ready``, and on SIGTERM (or its
parent's death) sets ``state: stopped`` and lets the seat go. A keeper that cannot take the
seat exits, and the launch with it.

The keeper also **runs the stack's pilots** (``lifetime: stack``), each a nested play_launch
of its kind's launch in its domain, respawned 5 s after it ends (an episode end: a world load
ends acb_bridge), so that it can restart them with another ``map_path``. A stack serves
several maps (``maps:`` of testbench.yaml, else every map the project's scenarios name); its pilots
run on one at a time, ``map`` in stack.json. On CARLA they start lazily, for the first run's
map, unless ``map:=`` names one; on Isaac the world is the stack's (``world:=``), so the
first map is the stack's for its lifetime.

A runner asks for another map by writing ``switch-request.json`` (``{id, map}``) into the
stack's directory; the keeper sees a new id within 0.1 s, stops the pilots, records the map
in stack.json (``map``, ``map_path``, the pilots' contract), starts them again with the new
``map_path`` and answers in stack.json's ``switch`` (``{id, map, state: done|failed,
error}``). On CARLA it has the bridge load the new town in between (``scensim.load_map``, the
bridge's ``~/load_map``), so the pilots start on it; when that fails the town loads on SSv2's
Initialize, one episode later. The run then waits for its pilots' readiness as every run does.

The simulator (scensim.simulator): on CARLA, ``simulator_start`` (``auto``, ``start``,
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

from scensim import config, ground as groundmod, project as projectmod, results, seat as seatmod
from scensim import episodes as episodesmod, judges as judgesmod, simulator as simmod
from scensim import own_world
from scensim import overlays as overlaysmod, vehicles as vehiclesmod, world as worldmod
from scensim import launcher as launchermod, tools as toolsmod
from scensim import autoware as autowaremod
from scensim.config import ConfigError

SCHEMA = "scensim.stack/1"
STARTING, READY, STOPPING, STOPPED = "starting", "ready", "stopping", "stopped"
STACK_JSON = "stack.json"
EVENTS = "events.jsonl"
SWITCH_REQUEST = "switch-request.json"
SWITCHING, DONE, FAILED = "switching", "done", "failed"
WEATHER_PENDING, WEATHER_APPLIED, WEATHER_FAILED = "pending", "applied", "failed"
WEATHER_RETRY = 5.0       # s between set_weather attempts while the bridge does not serve it
WEATHER_DEADLINE = 600.0  # s of attempts after the simulator is up before the stack fails
RESPAWN_DELAY = 5.0   # as stack.launch.py's nested launches
STOP_GRACE = 20.0     # SIGINT, then SIGTERM, then SIGKILL

# kind -> its launch under scensim_launch's launch/pilots/ (stack.launch.py's PILOT_KINDS)
PILOT_FILES = {"autoware": "autoware.launch.xml", "carla_basic": "carla_basic.launch.py",
               "simple": "simple.launch.py", "launch": "launch.launch.py",
               "command": "command.launch.py", "none": None}
# project pilot keys that are not launch arguments
NOT_ARGS = ("kind", "lifetime", "ready", "inputs", "args")
EGO_READY = ["ros2", "run", "scensim_launch", "ego_ready", "--timeout", "10"]
DEFAULT_VEHICLE = "tesla_model3"
SIDED_KINDS = ("autoware", "simple")  # kinds that always bring a vehicle side
# kinds that drive on the car's true pose: on Isaac the world server's ego config turns aib's
# ground truth on for them (on CARLA acb always publishes /carla/ground_truth/odom)
GROUND_TRUTH_KINDS = ("simple",)
ISAAC_GROUND_TRUTH_TOPIC = "/ground_truth/odom"  # simple_pilot's default on Isaac
ISAAC_EGO = "isaac_ego.yaml"  # the stack's ego config, in its directory, when it needs one


# Launch arguments the project interface replaced (roadmap 076 step 10), each with what to give
# instead. Refused by name, never aliased (as autoware_sim_profile's check_launch_args does for
# the profile's renamed arguments): a launch drops an undeclared name without a word, and as
# stack_args they would start an ego beside the keeper's pilots or move the stack's map under
# it. stack.launch.py keeps ego_launch, ego_launch_args and agent for the developers' `just
# run` (carla.launch.xml, isaac.launch.xml); a project never reaches them.
RETIRED_ARGS = {
    "ego_launch": "a pilot in testbench.yaml: `pilots: {ego: {kind: autoware}}`; your own launch "
                  "as `kind: launch` with `launch: <package> <file>`; none as `kind: none`",
    "ego_launch_args": "the pilot's `args:` in testbench.yaml "
                       "(`pilots: {ego: {kind: autoware, args: {max_velocity: 11.11}}}`)",
    "agent": "`agent: own` on a `kind: launch` pilot in testbench.yaml",
    "map_path": "map:=<map>, one of the project's maps (assets/maps/<map>)",
}


def retired_args(names) -> list:
    """One line per retired launch argument among ``names``: what to give instead."""
    return [f"{n}:= is retired; give {RETIRED_ARGS[n]}. The stack is "
            f"`play_launch launch scensim_launch scensim.launch.xml` in the project"
            for n in sorted(set(names)) if n in RETIRED_ARGS]


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
    """The scensim checkout this code runs from (a symlink install resolves into it), or None."""
    here = Path(__file__).resolve()
    for d in here.parents:
        if (d / ".git").exists() and (d / "src" / "scensim").is_dir():
            return d
    return None


def source_id(ws) -> str:
    """The code the stack runs, not the commit that holds it: the tree of the checkout's
    ``src/`` (``src:<tree>``), plus a hash of its uncommitted changes there (``+<sha>``). A
    commit of the same tree, or an edit of the docs, leaves it as it was."""
    def git(*a):
        try:
            # --no-optional-locks: a killed `git diff` must not leave index.lock behind
            return subprocess.run(["git", "--no-optional-locks", *a], cwd=str(ws),
                                  capture_output=True, text=True,
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
        return {"scensim": None, "submodules": {}}
    p = results.provenance(ws)
    # None: the probe failed (unknown), which fingerprint_diff does not compare
    return {"scensim": source_id(ws) or p.get("swb"), "submodules": p.get("submodules")}


def fingerprint(project: Path, settings: dict = None, build: dict = None) -> dict:
    """What a run must agree with: the project file, the lock, the pilots, the build."""
    project = Path(project)
    settings = settings if settings is not None else config.project_settings(project)
    return {
        "testbench_yaml": _sha(project / config.PROJECT_FILE),
        "scensim_lock": _sha(project / config.LOCK_FILE),
        "pilots": _canon(settings.get("pilots") or {}),
        "build": build if build is not None else build_commits(),
    }


def fingerprint_diff(stack_fp: dict, project_fp: dict, notes: list = None) -> list:
    """One phrase per part that differs, naming both sides. The build's submodules unknown on
    either side (their probe failed) are not compared; ``notes`` gets a phrase saying so."""
    names = {"testbench_yaml": config.PROJECT_FILE, "scensim_lock": config.LOCK_FILE,
             "pilots": "pilots:", "build": "the build"}
    out = []
    for k, name in names.items():
        a, b = (stack_fp or {}).get(k), (project_fp or {}).get(k)
        if a == b:
            continue
        if k == "build":
            a, b = a or {}, b or {}
            if a.get("scensim") != b.get("scensim"):
                out.append(f"the build (stack scensim {a.get('scensim')}, now {b.get('scensim')})")
            sa, sb = a.get("submodules"), b.get("submodules")
            if sa is None or sb is None:
                if notes is not None and (sa or sb):
                    side = "the stack's" if sa is None else "this run's"
                    notes.append(f"the build's submodules were not compared (unknown on "
                                 f"{side} side: its `git submodule status` failed)")
                continue
            subs = sorted(p for p in set(sa) | set(sb) if sa.get(p) != sb.get(p))
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
    """``{run_dir}``, ``{seed}``, ``{run_id}``, ``{map_path}``, ``{param:<name>}``,
    ``{workspace:<name>}`` (the host's ``workspaces:``, G60) in strings,
    lists, dicts. A ``{param:x}`` the context lacks raises StackRefused naming ``where``; so
    does ``{map_path}`` outside a run (a ``lifetime: stack`` pilot outlives a map switch)."""
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
        elif name == "map_path":
            if not ctx.get("map_path"):
                raise StackRefused(f"{where}: {{map_path}} is the run's map; a lifetime: stack "
                                   "pilot outlives a map switch (lifetime: run)")
            out.append(str(ctx["map_path"]))
        elif name.startswith("workspace:"):
            w = name[len("workspace:"):]
            dirs = ctx.get("workspaces") or {}
            if w not in dirs:
                raise StackRefused(f"{where}: {{workspace:{w}}} but this host names no "
                                   f"workspace {w} (workspaces: {{{w}: <dir>}} in "
                                   f"{overlaysmod.HOST_FILE})")
            out.append(str(dirs[w]))
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
    key: str                      # the testbench.yaml pilots: key that matched
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


def own_car_args(car: dict, kind: str, sim: str) -> dict:
    """The launch arguments that give a pilot an own car's generated files: on CARLA the
    vehicle side's sensors and, without Autoware, its spec_hash in place of the car's
    Autoware description package; the simple pilot its spec (wheelbase, steering)."""
    a = {}
    if sim == "carla":
        a["vehicle_config"] = car["carla_sensors"]
    if kind != "autoware":
        a["autoware_spec_hash_file"] = car["spec_hash_file"]
        a["spec_hash_file"] = car["spec_hash_file"]
        maps = (car.get("pedal_maps") or {}).get("dir")
        if sim == "carla" and maps:
            a["vehicle_maps_dir"] = maps   # else the vehicle side looks in <car>_description
    if kind == "simple":
        a["spec"] = car["spec"]
        a["description"] = car["description"]
    return a


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


REGISTERED_KINDS = ("carla_basic", "simple", "command", "launch")


def ready_command(spec: dict, entity: str = None, relay: str = None):
    """The pilot's readiness check as argv, or None when started is ready. By default an
    autoware pilot is ready when its Autoware is (ego_ready), and a carla_basic, simple,
    command or launch pilot when its agent has registered the entity with the relay
    (``registered``):
    a pilot that crashes at start is never ready, and the run says so."""
    r = spec.get("ready")
    if r in (None, "", "default"):
        if spec.get("kind") == "autoware":
            return list(EGO_READY)
        if spec.get("kind") in REGISTERED_KINDS and entity and relay:
            return [sys.executable, "-m", "scensim.stack", "registered", "--relay", relay, entity]
        return None
    if r == "registered":
        return [sys.executable, "-m", "scensim.stack", "registered", "--relay", relay, entity]
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


def aib_ego_config() -> Path:
    """aib's default ego config (`config/ego.yaml`, what the Isaac world server's `ego:=default`
    is): from aib's share, else this workspace's source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("aib")) / "config" / "ego.yaml"
        if p.is_file():
            return p
    except Exception:
        pass
    ws = workspace_root() or Path("/nonexistent")
    return ws / "src" / "autoware_isaac_bridge" / "src" / "aib" / "config" / "ego.yaml"


def isaac_ego_with_ground_truth(base: Path = None) -> dict:
    """aib's ego config ``base`` (default: aib_ego_config()) with its paths made absolute and
    the ground truth on (Odometry of base_link in the map on ISAAC_GROUND_TRUTH_TOPIC): the
    ego config of an Isaac stack whose ego pilot drives on the true pose (GROUND_TRUTH_KINDS)."""
    import yaml
    base = Path(base) if base else aib_ego_config()
    try:
        doc = yaml.safe_load(base.read_text()) or {}
    except OSError as e:
        raise StackRefused(f"aib's ego config {base}: {e.strerror}; a simple pilot on Isaac "
                           "needs it (aib built and sourced)") from None
    for key in ("rig", "body"):
        if doc.get(key):
            p = Path(os.path.expandvars(str(doc[key])))
            doc[key] = str(p if p.is_absolute() else (base.parent / p).resolve())
    doc["ground_truth"] = {**(doc.get("ground_truth") or {}), "topic": ISAAC_GROUND_TRUTH_TOPIC}
    return doc


def scensim_launch_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("scensim_launch")
    except Exception:
        return str(Path(__file__).resolve().parents[2] / "scensim_launch")


# --- maps -----------------------------------------------------------------------------------

def map_dir(project, name) -> Path:
    return Path(project) / "assets" / "maps" / name


def served_maps(settings: dict, scenarios) -> list:
    """The maps a stack serves: testbench.yaml's ``maps:``, else every map the scenarios name."""
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
    if rec.get("simulator") == "isaac" and name not in (rec.get("preload") or []):
        return (f"the scenario is on map {name}, the stack's Isaac world on {rec.get('map')} "
                f"(the world server loads its worlds at start): restart the stack with "
                f"map:={name}, or add {name} to isaac.preload in testbench.yaml")
    if name not in (rec.get("maps") or []):
        return (f"map {name} is not one of the stack's maps ({', '.join(rec.get('maps') or [])});"
                " add it to maps: in testbench.yaml and restart the stack")
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


def isaac_kit_args(given: str, gpu) -> str:
    """Kit's renderer on the CUDA card: CUDA_VISIBLE_DEVICES does not reach Vulkan, which
    still sees every card, so the renderer would pick its own (GPU 0) while CUDA, the RTX
    lidar's interop and PhysX, ran on ``gpu``; the sensors then never publish (roadmap 077,
    swb 241: "ndt No InputSource"). Appended after the user's own, so this one wins."""
    pin = f"--/renderer/multiGpu/enabled=false --/renderer/activeGpu={gpu}"
    return f"{given} {pin}".strip()

def plan(project_arg=None, seat_arg=None, simulator=None, map_name=None, logs=None, gpu=None,
         stack_args=None, check=projectmod.check_stack, now=None, simulator_start=None,
         probe=None, launcher=None) -> Plan:
    """Everything the stack is, before anything starts; StackRefused says why it must not."""
    retired = retired_args(stack_args or {})
    if retired:
        raise StackRefused("stack_args: " + "; ".join(retired))
    try:
        project = config.writable_project(config.find_project(project_arg or None))
    except ConfigError as e:
        raise StackRefused(str(e)) from None
    overrides = {}
    if seat_arg not in (None, ""):
        overrides["seat"] = seat_arg
    if logs:
        overrides["logs"] = logs
    if gpu not in (None, ""):
        overrides["gpu"] = int(gpu)
    if launcher not in (None, ""):
        overrides["launcher"] = launcher  # the launch argument beats the host setting
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
    world = psettings.get("world") if isinstance(psettings.get("world"), dict) else {}
    # world.server: the user's world server in place of the stock one (scensim.own_world);
    # simulator: none, only with it, starts no simulator (that server owns one)
    server = world.get(own_world.KEY)
    if sim not in projectmod.SIMULATORS and not (sim == own_world.NONE and server):
        raise StackRefused(f"simulator {sim!r}: this stack starts carla or isaac, or none with "
                           f"world.{own_world.KEY} (your world server)")
    if simulator_start not in (None, "") and simulator_start not in simmod.MODES:
        raise StackRefused(f"simulator_start {simulator_start!r}: one of "
                           + ", ".join(simmod.MODES))

    try:
        source = seatmod.seats_source(project)
        s = seatmod.resolve(host, source,
                            usable=lambda x: not seatmod.busy_ports(x, web=False))
    except (ConfigError, seatmod.SeatTaken) as e:
        raise StackRefused(str(e)) from None
    h = seatmod.holder(s.index, s.lock_dir)
    if h is not None:
        raise StackRefused(f"seat {s.index} is taken: {seatmod.describe_holder(h)}")
    # the web UI's port is play_launch's by now, so only the ports the stack binds later
    busy = seatmod.busy_ports(s, web=False)
    if busy:
        raise StackRefused(seatmod.describe_busy(s, busy))

    scenarios = scan_scenarios(project)
    maps = served_maps(psettings, scenarios)
    if map_name and map_name not in maps:
        maps.append(map_name)
    isaac = psettings.get("isaac") if isinstance(psettings.get("isaac"), dict) else {}
    preload = [str(m) for m in (isaac.get("preload") or [])] if sim == "isaac" else []
    maps += [m for m in preload if m not in maps]
    if not maps:
        raise StackRefused("no scenario names a map ($(env SCENSIM_MAPS)/<map>); give map:= "
                           "or maps: in testbench.yaml")
    for m in maps:
        if not map_dir(project, m).is_dir():
            raise StackRefused(f"map {m} is not fetched ({map_dir(project, m)}); "
                               f"{projectmod.FETCH}")
    # CARLA: the pilots start for the first run's map unless map:= names one; Isaac: the
    # world server loads its world at start, so the stack has a map from the beginning
    chosen = map_name or (maps[0] if sim == "isaac" else None)
    map_path = map_dir(project, chosen) if chosen else None

    bound = assign_domains(bind_pilots(psettings, scenarios), s, sim)
    # the project's own cars (vehicles:), generated into state/vehicles/; their pilots get
    # the car's files (roadmap 077, G5)
    try:
        cars = vehiclesmod.generate(project, psettings) if psettings.get("vehicles") else {}
    except vehiclesmod.VehicleError as e:
        raise StackRefused(str(e)) from None
    merged_catalog = cars.pop("__catalog__", None)
    for b in bound:
        car = cars.get(b.spec.get("vehicle"))
        if car is not None:
            b.spec["args"] = {**own_car_args(car, b.kind, sim), **(b.spec.get("args") or {})}
        if b.kind == "autoware":
            # the workspaces the pilot sources were built against that same release
            mixed = autowaremod.install_problems(host, workspace_root())
            if mixed:
                raise StackRefused("; ".join(mixed))
            # the profile runs the host's Autoware release (autoware.setup's directory)
            try:
                b.spec["args"] = autowaremod.pilot_release(
                    host, f"pilots.{b.key} ({b.entity})", b.spec.get("args") or {})
            except ValueError as e:
                raise StackRefused(str(e)) from None
    stack_pilots = [b for b in bound if b.lifetime == "stack"]
    workspaces = {k: str(v) for k, v in overlaysmod.host_workspaces(host).items()}
    ctx = {"seed": 0, "run_dir": "", "run_id": "", "params": {}, "workspaces": workspaces}
    entries = []
    for b in stack_pilots:
        entries.append(pilot_entry(b, project, ctx))

    name = project.resolve().name
    stack_id = f"{now_id(now)}_seat{s.index}"
    stack_dir = project_root(host["logs"], name) / "stacks" / stack_id
    proc = stack_dir / "proc"

    usd = project / "assets" / "carla-usd"
    env = {
        "SCENSIM_MAPS": str(project / "assets" / "maps"),
        "SCENSIM_CARLA_USD": str(usd),
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "CYCLONEDDS_URI": f"file://{seatmod.cyclone_profile(host)}",
        # the scenarios' vehicle catalog: the project's merged one, else the installed one
        vehiclesmod.ENV: str(vehiclesmod.catalog_dir(project)),
    }
    # Isaac's world server finds every car's body (the ego's and the NPCs') under
    # SCENSIM_VEHICLE_USD, by default a host path: the project's fetched USD set has them
    # (roadmap 077, G36; without, every car is a box to the sensors)
    if (usd / "vehicles" / "vehicles.json").is_file():
        env["SCENSIM_VEHICLE_USD"] = str(usd / "vehicles")
    if host.get("gpu") is not None or (host.get("seats") or {}).get("gpus") is not None:
        env["CUDA_VISIBLE_DEVICES"] = str(s.gpu)
        if sim == "isaac":
            env["ISAAC_KIT_ARGS"] = isaac_kit_args(os.environ.get("ISAAC_KIT_ARGS", ""), s.gpu)
    # Isaac's world server finds its install by ISAAC_ROOT; the host settings' isaac.root
    isaac_root = str((host.get("isaac") or {}).get("root") or "")
    if sim == "isaac" and isaac_root:
        env["ISAAC_ROOT"] = os.path.expanduser(isaac_root)

    ego_domain = s.pilot_domains[0]
    vehicle = next((b.spec.get("vehicle") for b in bound if b.spec.get("vehicle")),
                   DEFAULT_VEHICLE)
    carla = dict(host.get("carla") or {})
    if s.carla_port:
        carla["port"] = s.carla_port  # the seat table gives each seat its own CARLA
    carla_run = None
    if sim == "carla":
        try:
            carla_run = CARLA_DECIDE(carla, simulator_start or None, probe or simmod.probe)
        except simmod.SimulatorRefused as e:
            raise StackRefused(str(e)) from None
    args = {
        "sim": sim,
        "ego_launch": "none",
        "launcher": toolsmod.launcher(host.get("launcher"))[0],
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
        "dds_config": f"file://{seatmod.cyclone_profile(host)}",
    }
    if merged_catalog:
        args["vehicle_catalogs"] = merged_catalog
    # the bridge reads a map's signal table from the same state dir first (both simulators:
    # an Isaac world finds its lamps by the table CARLA's server wrote); the gap trace and
    # the contact reports go to the stack's episodes/, which each run empties into its own
    # directory (scensim.episodes: the bridge reads both directories once, at start)
    gap_trace = bool((psettings.get("logging") or {}).get("gap_trace")) or \
        "gap_trace" in judgesmod.needed(psettings.get("judges"))
    contacts = worldmod.contacts_mode(world)
    episode_dirs = episodesmod.stack_dirs(stack_dir, gap_trace, contacts is None)
    side = {"state_dir": str(project / "state")}
    if episode_dirs[episodesmod.GAP_TRACE]:
        side["gap_trace_dir"] = episode_dirs[episodesmod.GAP_TRACE]
    if episode_dirs[episodesmod.CONTACTS]:
        side["contact_report_dir"] = episode_dirs[episodesmod.CONTACTS]
    side.update(worldmod.side_args(project, world))
    # the world server writes what it makes for a map (CARLA's signal table, Isaac's ruts
    # and surfaces) into the project's state dir, never into the fetched (read-only) map
    sim_side = {"state_dir": str(project / "state")}
    isaac_ego = None
    if server:
        # the user's server takes its own settings as its arguments (world.server); the stock
        # server's (state_dir, the Isaac world and ego, CARLA's spec physics) are not its
        sim_side = {}
    elif sim == "isaac":
        sided = [b for b in bound if b.sided()]
        ego = "default" if sided else "none"
        # a pilot of a ground-truth kind, or one that says it drives on the true pose
        # (inputs: [ground_truth], e.g. a launch pilot running simple_pilot; roadmap 077)
        if any(b.kind in GROUND_TRUTH_KINDS or "ground_truth" in (b.spec.get("inputs") or [])
               for b in sided):
            isaac_ego = {"path": str(stack_dir / ISAAC_EGO),
                         "config": isaac_ego_with_ground_truth()}
            ego = isaac_ego["path"]
        first = [str(map_path)] + [str(map_dir(project, m)) for m in preload if m != chosen]
        sim_side = {"world": ",".join(first), "ego": ego, **sim_side}
    if not server:
        sim_side.update(worldmod.sim_args(project, sim, world, isaac))
    ego_car = cars.get(vehicle) if vehicle else None
    if server:
        pass
    elif sim == "carla" and ego_car is not None:
        # the world server applies an own car's physics to it, on every car of its model
        # (the agents' too); the pilot's vehicle bridge then checks them against the spec
        sim_side["vehicle_physics"] = ego_car["carla_physics"]
    elif sim == "carla" and vehicle:
        # an installed car runs its spec's physics too (077 G55: it ran CARLA's blueprint's,
        # so a project's Model 3 was not the car the spec describes);
        # stack_args:="sim_args:='vehicle_physics:=none'" keeps the blueprint's
        physics = vehiclesmod.installed_physics(vehicle)
        if physics is None:
            raise StackRefused(f"vehicle {vehicle}: sim_vehicle_spec has no generated "
                               f"{vehicle}/carla_physics.yaml; give stack_args:=\"sim_args:="
                               f"'vehicle_physics:=<its carla_physics.yaml>'\" (or none for "
                               "the blueprint's own)")
        sim_side["vehicle_physics"] = str(physics)
    # stack_args (the developers'): side_args and sim_args merge into scensim's, by name; a name
    # scensim owns is refused (they used to replace scensim's whole text, state_dir with it)
    user = dict(stack_args or {})
    if server and user.get("sim_args"):
        raise StackRefused(f"stack_args sim_args: with world.{own_world.KEY} your world server "
                           "takes its arguments from world.server itself ([<package>, "
                           "<executable>, args...])")
    try:
        side = worldmod.merge_user(side, worldmod.named(user.pop("side_args", ""), "side_args"),
                                   worldmod.SIDE_OWNED, "stack_args side_args")
        sim_side = worldmod.merge_user(sim_side,
                                       worldmod.named(user.pop("sim_args", ""), "sim_args"),
                                       worldmod.SIM_OWNED.get(sim, {}), "stack_args sim_args")
    except ValueError as e:
        raise StackRefused(str(e)) from None
    args["side_args"] = worldmod.joined(side)
    args["sim_args"] = worldmod.joined(sim_side)
    if server:
        args["world_server"] = f"{server[0]} {server[1]}"
        args["world_server_args"] = shlex.join(str(a) for a in server[2:])
    for k, v in user.items():
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
                       "ready": ready_command(b.spec, b.entity,
                                              f"tcp://localhost:{s.relay_port}"),
                       "spec": b.spec,
                       "overlays": overlaysmod.resolved(project, b.spec, host),
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
        "preload": preload,
        "episodes": episode_dirs,
        "weather": ({"parameters": worldmod.weather_parameters(world), "state": WEATHER_PENDING}
                    if worldmod.weather_parameters(world) else None),
        "simulator_start": carla_run["owner"] if carla_run else simmod.WORLD_SERVER,
        # the launcher of this stack's nested launches, resolved (roadmap 079)
        "launcher": toolsmod.launcher(host.get("launcher"))[0],
        "carla": carla_run,
        "switch": None,
        "isaac_ego": isaac_ego,
        "world_server": list(server) if server else None,
        "fingerprint": fingerprint(project, psettings),
        "autoware_env": autoware_env(host, workspace_root()),
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
        "workspaces": workspaces,
        "pilot_contract": common,
        "pilots_dir": str(Path(scensim_launch_share()) / "launch" / "pilots"),
        "spec_hash": {b.entity: h for b in bound
                      if (h := (cars.get(b.spec.get("vehicle")) or {}).get("spec_hash")
                          or spec_hash_of(b.spec.get("vehicle")))},
        "vehicles": {n: {k: c.get(k) for k in ("spec", "spec_hash", "catalog", "pedal_maps")}
                     for n, c in cars.items()},
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
    for d in (p.record.get("episodes") or {}).values():
        if d:
            Path(d).mkdir(parents=True, exist_ok=True)
    project = Path(p.record["project"])
    try:
        groundmod.apply(project, config.project_settings(project).get("ground"))
    except groundmod.GroundError as e:
        raise StackRefused(str(e)) from None
    ego = p.record.get("isaac_ego")
    if ego:  # the Isaac world server's ego config, ground truth on (a simple pilot)
        import yaml
        Path(ego["path"]).write_text(yaml.safe_dump(ego["config"], sort_keys=False))
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
                        lock_dir=record.get("lock_dir") or config.DEFAULTS["seat_locks"],
                        carla_port=n.get("carla_port"))


def ros_env(record: dict, domain) -> dict:
    env = dict(os.environ)
    env.update(record.get("env") or {})
    env["ROS_DOMAIN_ID"] = str(domain)
    return env


def is_registered(relay: str, entity: str, timeout=5.0) -> bool:
    """An agent has registered ``entity`` with the relay at ``relay`` (tcp://host:port)."""
    try:
        from scensim_agent.commander import Commander, CommanderError
    except ImportError:
        return False
    try:
        with Commander(relay, name="scensim-ready", timeout=timeout) as c:
            reply = c.query(entity)
    except (CommanderError, OSError, ValueError):
        return False
    return bool((reply or {}).get("registered"))


def check_ready(cmd, env, timeout=60, why: list = None) -> bool:
    """The readiness check passed. With ``why``, its last output line (the reason it gives
    for not being ready) replaces ``why``'s contents."""
    if not cmd:
        return True
    try:
        p = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        reason, ok = f"its check did not answer in {timeout} s", False
    except (OSError, subprocess.SubprocessError) as e:
        reason, ok = f"its check did not run: {e}", False
    else:
        ok = p.returncode == 0
        lines = [l.strip() for l in (p.stdout + "\n" + p.stderr).splitlines()
                 if l.strip() and "[ros2run]" not in l and "deprecated element" not in l]
        reason = lines[-1] if lines else f"its check exited {p.returncode}"
    if why is not None:
        why[:] = [reason]
    return ok


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
    extra = () if p["kind"] == "carla_basic" else launchermod.ISOLATED_CONTAINERS
    argv = launchermod.command(launchermod.of(rec), [os.path.join(rec["pilots_dir"], name)],
                               words, log_dir, play_launch_flags=extra)
    extra = overlaysmod.flags(p.get("overlays"))
    if p["kind"] == "autoware":
        return autoware_env_argv(rec.get("autoware_env"), argv, extra)
    if extra:
        return ["bash", str(OVERLAY_ENV), *extra, "--", *argv]
    return argv


OVERLAY_ENV = Path(__file__).resolve().parent / "overlay_env.bash"
AUTOWARE_ENV = Path(__file__).resolve().parent / "autoware_env.bash"
CARLA_DECIDE = simmod.decide   # how plan() settles the stack's CARLA (tests wrap it)


def autoware_env(host: dict, workspace) -> dict:
    """The autoware pilot's environment from the host settings (``autoware:``), or None when
    the scensim checkout is not known (the pilot then runs in the stack's environment)."""
    if workspace is None:
        return None
    a = host.get("autoware") or {}
    exp = lambda v: os.path.expanduser(str(v)) if v else ""  # noqa: E731
    return {"workspace": str(workspace), "setup": exp(a.get("setup")),
            "overlay": exp(a.get("overlay")),
            "use_overlay": "true" if a.get("use_overlay", True) else "false",
            "cuda_ndt": exp(a.get("cuda_ndt")), "tensorrt_lib": exp(a.get("tensorrt_lib"))}


def autoware_env_argv(env: dict, argv: list, extra: list = ()) -> list:
    """``argv`` under autoware_env.bash: Autoware, the overlay, CUDA NDT, the installs, the
    pilot's own workspaces (``extra``: ``--overlay <setup.bash>`` pairs, G60) and TensorRT
    10.8 sourced first, as the developer recipes do. Without Autoware's environment the
    pilot's workspaces still come first (overlay_env.bash)."""
    if not env or not env.get("setup"):
        return ["bash", str(OVERLAY_ENV), *extra, "--", *argv] if extra else argv
    return ["bash", str(AUTOWARE_ENV), env["workspace"], env["setup"], env["overlay"],
            env["use_overlay"], env["cuda_ndt"], env["tensorrt_lib"], *extra, "--", *argv]


def start_process(argv, env, log: Path, cwd=None, die_with_parent=False):
    """``argv`` in its own session (its group is signalled as one), output to ``log``."""
    log.parent.mkdir(parents=True, exist_ok=True)
    fh = open(log, "ab")
    try:
        return subprocess.Popen(argv, env=env, stdout=fh, stderr=subprocess.STDOUT, cwd=cwd,
                                start_new_session=True,
                                preexec_fn=_child_with_death_signal if die_with_parent
                                else default_signals)
    finally:
        fh.close()


def stop_processes(procs, grace=STOP_GRACE):
    """SIGINT to every process group, then SIGTERM, then SIGKILL, waiting for all at once.
    A group is stopped when none of it is left, not when its leader exits: `ros2 launch`
    can exit on SIGINT before its children do (roadmap 079, step 3), and what it leaves in
    the group gets the next signal. The leader is polled every 50 ms; the group, a scan of
    /proc, only once its leader is gone and at most every GROUP_POLL."""
    procs = [p for p in procs if p is not None]
    checked = {}  # pid -> when the group was last scanned

    def alive(p):
        if p.poll() is None:
            return True
        now = time.time()
        if now - checked.get(p.pid, 0.0) < GROUP_POLL:
            return True
        checked[p.pid] = now
        return _group_alive(p.pid)

    live = [p for p in procs if alive(p)]
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 10.0), (signal.SIGKILL, 5.0)):
        if not live:
            return
        for p in live:
            try:
                os.killpg(p.pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        checked.clear()  # after a signal, look again at once
        deadline = time.time() + wait
        while time.time() < deadline:
            live = [p for p in live if alive(p)]
            if not live:
                return
            time.sleep(0.05)


GROUP_POLL = 0.5  # s between scans of /proc for what a leader left in its group


KEEPER_STOP_TIMEOUT = 90.0  # s: the keeper's own stop, pilots (up to 35 s) then CARLA (35 s)


def _is_keeper(pid) -> bool:
    """``pid`` is a running keeper (`python3 -m scensim.stack keep <dir>`), not a pid used again."""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            argv = f.read().split(b"\0")
    except OSError:
        return False
    return b"scensim.stack" in argv and b"keep" in argv


def _events_after(stack_dir: Path, offset: int):
    try:
        with open(Path(stack_dir) / EVENTS, "rb") as f:
            f.seek(offset)
            text = f.read().decode("utf-8", "replace")
    except OSError:
        return []
    out = []
    for line in text.splitlines():
        try:
            out.append(json.loads(line).get("event"))
        except ValueError:
            pass
    return out


def stop_keeper(rec: dict, timeout=KEEPER_STOP_TIMEOUT) -> bool:
    """Ask a stack's keeper to stop (SIGINT to its pid) and wait for its ``stack.stop``, so
    that it stops the pilots and the stack's CARLA in order before its launch goes:
    play_launch's own shutdown gives a node 2 s, then SIGKILLs it 5 s later (roadmap 079,
    step 4). True when the keeper said ``stack.stop`` or was already gone."""
    pid = (rec or {}).get("pid")
    stack_dir = Path(rec["proc"]).parent if rec and rec.get("proc") else None
    if not pid or stack_dir is None or not _is_keeper(pid):
        return True
    try:
        offset = (stack_dir / EVENTS).stat().st_size
    except OSError:
        offset = 0
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        return True
    deadline = time.time() + timeout
    while time.time() < deadline:
        if "stack.stop" in _events_after(stack_dir, offset):
            return True
        if not _is_keeper(pid):
            return "stack.stop" in _events_after(stack_dir, offset)
        time.sleep(0.2)
    return False


def stop_incomplete(stack_dir: Path) -> bool:
    """The stack's last ``stack.stopping`` has no ``stack.stop`` after it: its keeper was
    ended part way through its stop (its pilots and CARLA left to their backstops)."""
    events = _events_after(stack_dir, 0)
    if "stack.stopping" not in events:
        return False
    last = len(events) - 1 - events[::-1].index("stack.stopping")
    return "stack.stop" not in events[last + 1:]


def _group_alive(pgid) -> bool:
    """Any process left in group ``pgid`` (zombies excluded: init reaps them)."""
    try:
        os.killpg(pgid, 0)  # cheap: nothing of the group, not even a zombie
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
        except OSError:
            continue
        if fields[0] != "Z" and int(fields[2]) == pgid:
            return True
    return False


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
            print(f"scensim stack: pilot {p['entity']} did not start: {e}", file=sys.stderr,
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
            print(f"scensim stack: pilot {entity} ended ({code}); respawn in "
                  f"{self.respawn_delay:.0f} s", flush=True)
            changed = True
        for entity, at in list(self.due.items()):
            if now >= at and entity in by_entity:
                del self.due[entity]
                self._start(rec, *by_entity[entity])
                changed = True
        return changed


# --- the map switch -------------------------------------------------------------------------

def request_switch(stack_dir: Path, name: str, request_id: str = None,
                   restart: bool = False) -> str:
    """Ask the keeper of ``stack_dir`` to move its pilots to map ``name`` (``restart``: start
    them again even when they are on it); the request's id."""
    rid = request_id or f"{os.getpid()}-{time.time_ns()}"
    write_json(Path(stack_dir) / SWITCH_REQUEST,
               {"id": rid, "map": name, "requested": iso_now(), "by": os.getpid(),
                **({"restart": True} if restart else {})})
    return rid


def read_request(stack_dir: Path):
    try:
        req = read_json(Path(stack_dir) / SWITCH_REQUEST)
    except (OSError, ValueError):
        return None
    return req if isinstance(req, dict) and req.get("id") and req.get("map") else None


# The bridge's town load before the pilots restart (roadmap 077, G9). The runner waits
# SWITCH_TIMEOUT (180 s) for the whole switch: the pilots' stop (up to STOP_GRACE), the
# service's appearance and the load must fit in it. A Town load at Epic takes 20-70 s.
PRELOAD_WAIT = 10.0       # for /scenario_world_bridge/load_map to appear
PRELOAD_TIMEOUT = 120.0   # for its answer
LOADED, NOT_LOADED, SKIPPED = "loaded", "failed", "skipped"


def preload_command(rec: dict, name: str) -> list:
    """``python3 -m scensim.load_map`` for map ``name`` of the stack of ``rec``."""
    return [sys.executable, "-m", "scensim.load_map", str(map_dir(rec["project"], name)),
            "--wait", str(PRELOAD_WAIT), "--timeout", str(PRELOAD_TIMEOUT)]


def bridge_serving(rec: dict, timeout=0.5) -> bool:
    """Something listens on the seat's SSv2 port: the bridge binds it only once a world
    server has taken it as its owner."""
    try:
        with socket.create_connection(("127.0.0.1", int(seat_of(rec).ssv2_port)),
                                      timeout=timeout):
            return True
    except (OSError, ValueError, KeyError, TypeError):
        return False


def preload_map(rec: dict, name: str) -> dict:
    """On CARLA, have the bridge load map ``name``'s town now (``~/load_map``, in the
    scenario domain), so the pilots started next begin on it and SSv2's Initialize finds it
    loaded; ``{state: loaded|failed|skipped, message|error}`` for the switch's answer. A
    failure keeps the old way: the town loads at Initialize, one extra episode."""
    if rec.get("simulator") != "carla":
        return {"state": SKIPPED, "message": f"{rec.get('simulator')} loads its world itself"}
    if not bridge_serving(rec):
        return {"state": SKIPPED, "message": "no bridge serves the seat's SSv2 port yet; the "
                "town loads at SSv2's Initialize"}
    domain = (rec.get("domains") or {}).get("scenario")
    t0 = time.time()
    try:
        p = subprocess.run(preload_command(rec, name), env=ros_env(rec, domain),
                           capture_output=True, text=True,
                           timeout=PRELOAD_WAIT + PRELOAD_TIMEOUT + 15)
    except subprocess.TimeoutExpired:
        return {"state": NOT_LOADED, "error": "scensim.load_map did not end in time"}
    except OSError as e:
        return {"state": NOT_LOADED, "error": f"scensim.load_map: {e}"}
    answer = {}
    for line in reversed(p.stdout.splitlines()):
        try:
            answer = json.loads(line)
            break
        except ValueError:
            continue
    took = round(time.time() - t0, 1)
    if p.returncode == 0 and answer.get("success"):
        return {"state": LOADED, "message": answer.get("message", ""), "seconds": took}
    err = p.stderr.strip().splitlines()
    why = answer.get("message") or (err[-1] if err else f"scensim.load_map exited {p.returncode}")
    return {"state": NOT_LOADED, "error": why, "seconds": took}


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
        print(f"scensim stack: no switch to map {name}: {problem}", file=sys.stderr, flush=True)
        return False
    if name == rec.get("map") and pilots.procs and not req.get("restart"):
        sw.update(state=DONE, ended=iso_now())
        write_json(path, rec)
        return True
    write_json(path, rec)
    event(stack_dir, "map.switch", map=name, previous=rec.get("map"))
    print(f"scensim stack: pilots to map {name} (from {rec.get('map') or 'none'})", flush=True)
    pilots.stop_all(rec)
    # the town first, so the pilots start on it (G9); a failure is recorded, not fatal
    sw["preload"] = preload = preload_map(rec, name)
    event(stack_dir, "map.preload", map=name, **preload)
    print(f"scensim stack: town for {name}: {preload['state']} "
          f"({preload.get('message') or preload.get('error')})",
          file=sys.stderr if preload["state"] == NOT_LOADED else sys.stdout, flush=True)
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

def default_signals():
    """In a child before exec: SIGINT, SIGTERM and SIGQUIT at their defaults. An ignored
    signal stays ignored across exec, and a command started in the background of a
    non-interactive shell (`cmd &` in a script) has SIGINT ignored; `ros2 launch` then
    installs no SIGINT handler, so the group's SIGINT stops its nodes (which set their own)
    but not the launch, which respawns them (roadmap 079, step 3). A stop by SIGINT must
    reach every launcher whatever shell started scensim."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGQUIT):
        signal.signal(sig, signal.SIG_DFL)


def _child_with_death_signal():
    default_signals()
    _die_with_parent()


def _die_with_parent():
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        libc.prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG
    except OSError:
        pass


def set_weather(rec: dict, timeout: float = 20.0):
    """The bridge's set_weather with the project's weather parameters, in the scenario
    domain: ``(ok, message)``, ok None when the bridge did not answer (not up yet)."""
    w = rec.get("weather") or {}
    argv = worldmod.weather_argv(w.get("parameters") or {})
    try:
        p = subprocess.run(argv, env=ros_env(rec, rec["domains"]["scenario"]),
                           capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, str(e) or "no answer"
    return worldmod.weather_answer(p.stdout)


class Weather:
    """The project's weather parameters, set once the bridge answers (before the stack is
    ready). A preset needs none of this: it is the bridge's ``weather`` argument."""

    def __init__(self, rec, call=None):
        self.call = call or set_weather
        self.next = 0.0
        self.since = None

    def pending(self, rec) -> bool:
        w = rec.get("weather")
        return bool(w) and w.get("state") == WEATHER_PENDING

    def step(self, rec) -> bool:
        """One attempt when due; True when the record changed. A bridge that answers
        ``success: false`` is asked again (it may not hold the world server yet) until
        WEATHER_DEADLINE; then the weather is ``failed`` and the stack with it."""
        now = time.time()
        if not self.pending(rec) or now < self.next:
            return False
        self.since = self.since or now
        ok, msg = self.call(rec)
        w = rec["weather"]
        w["attempts"] = int(w.get("attempts") or 0) + 1
        if ok:
            w.update(state=WEATHER_APPLIED, applied_at=iso_now(), message=msg)
            return True
        w["message"] = msg
        if now - self.since >= WEATHER_DEADLINE:
            w["state"] = WEATHER_FAILED
            rec["error"] = (f"the weather parameters were not set within {WEATHER_DEADLINE:.0f}"
                            f" s: {msg or 'the bridge did not answer'} (world.weather in "
                            "testbench.yaml: the world server's parameter names)")
        self.next = now + WEATHER_RETRY
        return True


def _waiting(rec):
    if not rec.get("map"):
        return []
    return [p for p in rec["pilots"]
            if p.get("lifetime", "stack") == "stack" and not p.get("ready_at")]


class _Tee:
    """A text stream that also writes, line by line, to a second one (best effort)."""

    def __init__(self, own, other):
        self.own, self.other = own, other

    def write(self, text):
        n = self.own.write(text)
        try:
            self.other.write(text)
            self.other.flush()
        except (OSError, ValueError):
            pass
        return n

    def flush(self):
        self.own.flush()

    def __getattr__(self, name):
        return getattr(self.own, name)


def tee_to_launcher(ppid=None, proc=Path("/proc")):
    """The keeper's lines on its launcher's terminal too (roadmap 077, G41). play_launch keeps
    a node's output in the node's log (``node/stack_keeper/out``) and shows only its own, so a
    reader of ``play_launch launch ... scensim.launch.xml`` saw none of seat, CARLA, pilot and
    ready: the keeper also writes them to its parent's (play_launch's) stdout, a terminal or
    the console log ``--start-stack`` gives it. Nothing when that is the keeper's own stdout
    already (a launcher that passes output through) or cannot be opened. Returns the stream
    opened, else None."""
    ppid = ppid or os.getppid()
    target = proc / str(ppid) / "fd" / "1"
    try:
        theirs = os.stat(target)
        mine = os.fstat(sys.stdout.fileno())
        if (theirs.st_dev, theirs.st_ino) == (mine.st_dev, mine.st_ino):
            return None
        other = open(target, "a", buffering=1)
    except (OSError, ValueError):
        return None
    sys.stdout = _Tee(sys.stdout, other)
    sys.stderr = _Tee(sys.stderr, other)
    return other


def launcher_web_addr(ppid=None, proc=Path("/proc")):
    """The ``--web-addr`` the keeper's launcher (play_launch) was given, else None."""
    try:
        words = (proc / str(ppid or os.getppid()) / "cmdline").read_bytes().split(b"\0")
    except OSError:
        return None
    words = [w.decode(errors="replace") for w in words]
    for i, w in enumerate(words):
        if w == "--web-addr" and i + 1 < len(words):
            return words[i + 1]
        if w.startswith("--web-addr="):
            return w.split("=", 1)[1]
    return None


def keep(stack_dir, poll=5.0, once=False, pilot_command=None,
         respawn_delay=RESPAWN_DELAY, weather_call=None, world_probe=None) -> int:
    """Hold the seat for the stack in ``stack_dir``; run its pilots on the stack's map and
    move them to another on request; set the project's weather parameters; mark it ready,
    and stopped at the end.

    ``once``: return right after the stack is ready (tests). ``pilot_command``:
    ``(rec, pilot, entry, log_dir) -> argv`` in place of ``pilot_argv`` (tests).
    ``weather_call``: ``(rec) -> (ok, message)`` in place of ``set_weather`` (tests).
    ``world_probe``: ``(rec) -> bool`` in place of ``bridge_serving`` (tests): the stack is
    ready only once it is true."""
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
        print(f"scensim stack: {e}", file=sys.stderr)
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
    print(f"scensim stack {rec['stack_id']}: seat {s.index}; scenario domain {s.scenario_domain}, "
          f"SSv2 {s.ssv2_port}, relay {s.relay_port}, world server {s.world_port}; "
          + (f"web UI http://{web}; " if (web := launcher_web_addr()) else
             "no web UI (ros2 launch); " if launchermod.of(rec) == "ros2" else
             f"web UI: pass --web-addr 127.0.0.1:{s.web_port}; ")
          + "pilots on "
          f"{rec.get('map') or 'the first run' + chr(39) + 's map'}", flush=True)
    pilots = Pilots(stack_dir, pilot_command, respawn_delay)
    weather = Weather(rec, weather_call)
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
                print(f"scensim stack: CARLA started on {c['host']}:{c['port']} (pid {c['pid']}, "
                      f"log {c['log']}); waiting for its RPC, up to {c['ready_timeout']:.0f} s",
                      flush=True)
            write_json(path, rec)
        pilots_on = False
        next_check = 0.0
        # the bridge binds SSv2's port only once a world server has taken it as its owner;
        # an Isaac world server answers only after its worlds are loaded, minutes after it
        # starts, and a run started before that fails at SSv2's SIMULATOR_INITIALIZE_TIMEOUT
        serving = world_probe or bridge_serving
        world_up, next_world = False, 0.0
        while not stop["now"]:
            if carla is not None and carla.poll():
                write_json(path, rec)
                if carla.ready:
                    print(f"scensim stack: CARLA serves RPC ({rec['carla'].get('version')})",
                          flush=True)
            c = rec.get("carla") or {}
            if rec.get("simulator_start") == simmod.STARTED and c.get("state") == simmod.FAILED:
                rec["error"] = c.get("error")
                print(f"scensim stack: {c.get('error')}", file=sys.stderr, flush=True)
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
                        print(f"scensim stack: pilot {p['entity']} ready on {rec.get('map')}",
                              flush=True)
                write_json(path, rec)
                next_check = time.time() + poll
                waiting = _waiting(rec)
            if sim_ready and weather.step(rec):
                write_json(path, rec)
                w = rec["weather"]
                if w["state"] == WEATHER_APPLIED:
                    event(stack_dir, "weather.applied", parameters=w["parameters"])
                    print(f"scensim stack: weather parameters set ({w['parameters']})", flush=True)
                elif w["state"] == WEATHER_FAILED:
                    print(f"scensim stack: {rec['error']}", file=sys.stderr, flush=True)
                    code = 2
                    break
            if sim_ready and not world_up and time.time() >= next_world:
                world_up = serving(rec)
                next_world = time.time() + min(poll, 1.0)
                if world_up:
                    event(stack_dir, "world.ready")
                    print("scensim stack: the bridge serves SSv2 (its world server is up)",
                          flush=True)
            if not waiting and sim_ready and world_up and rec.get("state") != READY \
                    and not weather.pending(rec):
                rec["state"] = READY
                rec["ready_at"] = iso_now()
                write_json(path, rec)
                event(stack_dir, "stack.ready")
                print(f"scensim stack {rec['stack_id']}: ready", flush=True)
                if once:
                    return 0
            time.sleep(0.1)
    finally:
        if not once or code:
            # said first: play_launch gives the keeper seconds before SIGKILL, less than the
            # pilots' grace, and a record left saying ready outlives the stack
            rec["state"] = STOPPING
            write_json(path, rec)
            event(stack_dir, "stack.stopping")
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
    ap = argparse.ArgumentParser(prog="python3 -m scensim.stack")
    sub = ap.add_subparsers(dest="cmd", required=True)
    k = sub.add_parser("keep", help="hold the seat for a stack, run its pilots, track readiness")
    k.add_argument("stack_dir")
    k.add_argument("--poll", type=float, default=5.0)
    k.add_argument("--respawn-delay", type=float, default=RESPAWN_DELAY)
    k.add_argument("--pilot-wrap", metavar="PROGRAM",
                   help="run each pilot's command line under PROGRAM (tests, debugging)")
    r = sub.add_parser("registered", help="exit 0 when an agent has registered ENTITY with "
                       "the relay (a pilot's default readiness)")
    r.add_argument("entity")
    r.add_argument("--relay", required=True, help="tcp://host:port")
    a = ap.parse_args(argv)
    if a.cmd == "registered":
        return 0 if is_registered(a.relay, a.entity) else 1
    if a.cmd == "keep":
        _die_with_parent()
        try:
            launcher = launchermod.of(read_json(Path(a.stack_dir) / STACK_JSON))
        except (OSError, ValueError):
            launcher = "play_launch"
        if launcher == "play_launch":
            # ros2 launch shows its processes' output already ([stack_keeper-1] ...): a tee
            # there would print each line twice (079)
            tee_to_launcher()
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
