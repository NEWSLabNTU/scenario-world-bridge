"""``scensim project new|check``: a starter project, and a project checked before it runs.

docs/design/projects-and-pilots.md, "The project file" and "Ownership". The scenario owns its
map, entities, cars and which entities an external pilot drives (a ``Controller`` named
``agent`` or ``Autoware``, or one with the property ``isEgo``); the project file binds a pilot
to each such entity by name (or an fnmatch pattern).

``check_project(path)`` is what the stack calls before it starts; it returns findings, each
printed as ``problem: <file>: <what> (fix: ...)``.
"""
import argparse
import fnmatch
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from scensim import assets, budget as budgetmod, config, ground as groundmod, isaac as isaacmod
from scensim import judges as judgesmod, tools as toolsmod, vehicles as vehiclesmod
from scensim import overlays as overlaysmod, profile as profilemod, record as recordmod
from scensim import ssv2 as ssv2mod, world as worldmod
from scensim.findings import REFUSED, Finding, add_project_option, exit_code, on_host, ros_args

TEMPLATES = Path(__file__).resolve().parent / "templates"
DEFAULT_TEMPLATE = "carla-first-drive"
# a template's files whose names a package cannot carry as they are
RENAMES = {"gitignore": ".gitignore"}
# what `new --from` leaves behind: made on each host, never part of a project's source
NOT_COPIED = ("assets", "state", "runs", ".scensim", "play_log")

TOP_KEYS = ("simulator", "assets", "maps", "pilots", "vehicles", "ground", "logging", "judges",
            "world", "isaac", "ssv2", "default")
REQUIRED = ("simulator", "pilots", "assets")
SIMULATORS = ("carla", "isaac")
KINDS = ("autoware", "carla_basic", "simple", "launch", "command", "none")
LATER_KINDS = ("container",)
CARLA_ONLY = ("carla_basic",)
COMMON_PILOT_KEYS = ("kind", "vehicle", "args", "lifetime", "ready", "inputs", "rviz")
KIND_KEYS = {
    "autoware": ("autoware_launch", "overlays"),
    "launch": ("launch", "agent", "overlays"),
    "command": ("run",),
}
KIND_REQUIRED = {"launch": "launch", "command": "run"}
LIFETIMES = ("stack", "run")
INPUTS = ("sensors", "map", "ground_truth")
LEVELS = ("result", "standard", "full", "debug")
VERDICTS = ("passed", "failed", "error")
LOGGING_KEYS = ("capture", "keep", "budget", "topics", "gap_trace")
# logging keys the stack acts on, so testbench.yaml's only (a suite runs against a stack as it is)
STACK_LOGGING = {"gap_trace": "the stack records the gap trace for every run or none: "
                              "logging.gap_trace: true in testbench.yaml, then restart the stack"}
# designed, not built: refused by name rather than accepted and ignored
LOGGING_NOT_YET = {"levels": "per-process ROS log levels are not built yet; give a pilot its"
                             " own (e.g. args: {log_level: warn}) where its launch takes one",
                   "debug": "DEBUG on named groups is not built yet; capture: debug records"
                            " every topic"}
SUITE_KEYS = ("scenarios", "parameters", "distribution", "repetitions", "seed", "expect",
              "retry", "pilot_args", "logging", "ssv2")
SUITE_ITEM_KEYS = ("path", "ssv2")  # a scenarios: item as a mapping
EXTERNAL_CONTROLLERS = ("agent", "autoware")
MAPS_PREFIX = "$(env SCENSIM_MAPS)/"
RETIRED_MAPS_PREFIX = "$(env CARLA_MAPS)/"  # the developers' maps (the justfile), not a project's
FETCH = assets.FETCH


@dataclass
class Entity:
    name: str
    controller: str = ""
    properties: dict = field(default_factory=dict)
    catalog_entry: str = None
    geometry: dict = field(default_factory=dict)  # an inline Vehicle's numeric axles and box

    @property
    def ego(self) -> bool:
        """SSv2's ego: what ``managed_ego`` is about (the concealer's Autoware)."""
        return (self.controller.lower() == "autoware"
                or str(self.properties.get("isEgo", "")).lower() == "true")

    @property
    def external(self) -> bool:
        return (self.controller.lower() in EXTERNAL_CONTROLLERS
                or str(self.properties.get("isEgo", "")).lower() == "true")


@dataclass
class Scenario:
    path: Path
    logic_file: str = None
    entities: list = field(default_factory=list)
    distribution_of: str = None  # a ParameterValueDistribution's ScenarioFile
    variants: int = 1  # runs SSv2 makes of it: TIER IV ScenarioModifiers, or a distribution's
    time_limit: float = None  # s of simulation time by which the scenario ends itself (G53)


# --- reading scenarios: --------------------------------------------------------------------

def _element(tag, value):
    """A TIER IV YAML scenario's tree as XML: mappings are elements, scalars attributes."""
    el = ET.Element(tag)
    if isinstance(value, dict):
        for k, v in value.items():
            if isinstance(v, dict):
                el.append(_element(k, v))
            elif isinstance(v, list):
                for item in v:
                    el.append(_element(k, item if isinstance(item, dict) else {}))
            elif v is not None:
                el.set(k, str(v))
    return el


def _modifier_variants(mods) -> int:
    """A TIER IV scenario's ScenarioModifiers: the product of each modifier's values (a
    ``list``, or ``start``/``step``/``stop``)."""
    items = (mods or {}).get("ScenarioModifier") if isinstance(mods, dict) else None
    n = 1
    for m in items or []:
        if not isinstance(m, dict):
            continue
        if isinstance(m.get("list"), list):
            n *= max(1, len(m["list"]))
        elif all(k in m for k in ("start", "step", "stop")):
            try:
                start, step, stop = (float(m[k]) for k in ("start", "step", "stop"))
                if step:
                    n *= max(1, int(round((stop - start) / step)) + 1)
            except (TypeError, ValueError):
                pass
    return n


def _distribution_variants(root) -> int:
    """An OpenSCENARIO ParameterValueDistribution's deterministic runs: the value sets of
    a DeterministicParameterDistribution, or the product of its single distributions."""
    det = root.find("ParameterValueDistribution/Deterministic")
    if det is None:
        return 1
    sets = det.findall("DeterministicMultiParameterDistribution/ValueSetDistribution/"
                       "ParameterValueSet")
    n = len(sets) or 1
    for single in det.findall("DeterministicSingleParameterDistribution"):
        values = single.findall("DistributionSet/Element")
        if values:
            n *= len(values)
            continue
        rng = single.find("DistributionRange")
        r = rng.find("Range") if rng is not None else None
        try:
            step = float(rng.get("stepWidth"))
            lo, hi = float(r.get("lowerLimit")), float(r.get("upperLimit"))
            if step:
                n *= max(1, int(round((hi - lo) / step)) + 1)
        except (AttributeError, TypeError, ValueError):
            pass
    return n


TIME_RULES = ("greaterThan", "greaterOrEqual", "equalTo")


def _parameters(root) -> dict:
    return {d.get("name"): d.get("value")
            for d in root.iterfind("ParameterDeclarations/ParameterDeclaration")}


def _seconds(v, params):
    v = str(v or "").strip()
    if v.startswith("$"):
        v = str(params.get(v[1:].strip("{}"), ""))
    try:
        return float(v)
    except ValueError:
        return None   # an expression: not worked out here


def _time_alone(trigger, params) -> list:
    """The times (s) at which ``trigger`` fires on simulation time alone: each ConditionGroup
    (its conditions ANDed; the groups ORed) of one SimulationTimeCondition, plus its delay."""
    out = []
    for group in trigger.iterfind("ConditionGroup"):
        conds = group.findall("Condition")
        if len(conds) != 1:
            continue
        stc = conds[0].find("ByValueCondition/SimulationTimeCondition")
        if stc is None or stc.get("rule") not in TIME_RULES:
            continue
        t, delay = _seconds(stc.get("value"), params), _seconds(conds[0].get("delay") or 0,
                                                                params)
        if t is not None:
            out.append(t + (delay or 0))
    return out


def time_limit(root):
    """The simulation time (s) by which a scenario ends itself, or None: the earliest of its
    Storyboard's StopTrigger and the StartTrigger of any Event that exits
    (``CustomCommandAction type="exitSuccess|exitFailure"``), on simulation time alone.
    haruna_down's is 600 s (roadmap 077, G53)."""
    params = _parameters(root)
    times = []
    stop = root.find("Storyboard/StopTrigger")
    if stop is not None:
        times += _time_alone(stop, params)
    for ev in root.iter("Event"):
        if any(str(c.get("type") or "").startswith("exit")
               for c in ev.iterfind("Action/UserDefinedAction/CustomCommandAction")):
            st = ev.find("StartTrigger")
            if st is not None:
                times += _time_alone(st, params)
    return min(times) if times else None


def read_scenario(path: Path) -> Scenario:
    """A scenario's map and entities; raises ValueError naming what does not parse."""
    path = Path(path)
    if path.suffix in (".yaml", ".yml"):
        try:
            data = yaml.safe_load(path.read_text())
        except yaml.YAMLError as e:
            raise ValueError(f"not valid YAML: {e}") from None
        if not isinstance(data, dict) or "OpenSCENARIO" not in data:
            raise ValueError("no OpenSCENARIO: at the top")
        root = _element("OpenSCENARIO", data["OpenSCENARIO"])
        variants = _modifier_variants(data.get("ScenarioModifiers"))
    else:
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            raise ValueError(f"not valid XML: {e}") from None
        if root.tag != "OpenSCENARIO":
            raise ValueError(f"the root element is <{root.tag}>, not <OpenSCENARIO>")
        variants = _distribution_variants(root)
    sc = Scenario(path, variants=variants)
    sf = root.find("ParameterValueDistribution/ScenarioFile")
    if sf is not None:
        sc.distribution_of = sf.get("filepath")
        target = Path(sc.distribution_of or "")
        target = target if target.is_absolute() else path.parent / target
        if "$(" not in str(sc.distribution_of) and target.is_file() and target != path:
            try:   # each variant is the scenario file's run, with its limit
                sc.time_limit = read_scenario(target).time_limit
            except (ValueError, OSError):
                pass
        return sc
    sc.time_limit = time_limit(root)
    lf = root.find("RoadNetwork/LogicFile")
    sc.logic_file = lf.get("filepath") if lf is not None else None
    for obj in root.iterfind("Entities/ScenarioObject"):
        ent = Entity(obj.get("name") or "")
        ctl = obj.find("ObjectController/Controller")
        if ctl is not None:
            ent.controller = ctl.get("name") or ""
            for prop in ctl.iterfind("Properties/Property"):
                ent.properties[prop.get("name")] = prop.get("value")
        ref = obj.find("CatalogReference")
        if ref is not None:
            ent.catalog_entry = ref.get("entryName")
        veh = obj.find("Vehicle")
        if veh is not None:
            ent.geometry = vehicle_geometry(veh)
        sc.entities.append(ent)
    return sc


GEOMETRY_TOLERANCE = 0.02   # m: the bridge refuses an ego further than this from its body


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None  # a $parameter: SSv2 fills it in, nothing to compare here


def vehicle_geometry(veh) -> dict:
    """A <Vehicle>'s numeric geometry: axle positions, track, wheel diameter, box."""
    g = {}
    for axle, key in (("FrontAxle", "front"), ("RearAxle", "rear")):
        a = veh.find(f"Axles/{axle}")
        if a is None:
            continue
        for attr, name in (("positionX", "x"), ("trackWidth", "track"),
                           ("wheelDiameter", "wheel_diameter")):
            v = _num(a.get(attr))
            if v is not None:
                g[f"{key}_{name}"] = v
    d = veh.find("BoundingBox/Dimensions")
    if d is not None:
        for attr in ("length", "width", "height"):
            v = _num(d.get(attr))
            if v is not None:
                g[attr] = v
    return g


def catalog_geometry(entry: str, catalog: Path = None):
    """The vehicle catalog's geometry for ``entry`` (sim_vehicle_spec's catalogs/vehicles.xosc),
    or None when the catalog or the entry is not there."""
    if catalog is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            catalog = Path(get_package_share_directory("sim_vehicle_spec")) / "catalogs" / \
                "vehicles.xosc"
        except Exception:  # noqa: BLE001 (not installed: nothing to check against)
            return None
    try:
        root = ET.parse(catalog).getroot()
    except (OSError, ET.ParseError):
        return None
    for veh in root.iter("Vehicle"):
        if veh.get("name") == entry:
            return vehicle_geometry(veh)
    return None


def geometry_problems(declared: dict, catalog: dict) -> list:
    """``name declared x, catalog y`` for every shared value further apart than the
    tolerance."""
    return [f"{k} {declared[k]:g}, the catalog's {catalog[k]:g}" for k in sorted(declared)
            if k in catalog and abs(declared[k] - catalog[k]) > GEOMETRY_TOLERANCE]


def _is_scenario_file(p: Path) -> bool:
    if p.suffix == ".xosc":
        return True
    if p.suffix in (".yaml", ".yml"):  # a TIER IV YAML scenario, not a pose or goal file
        try:
            head = p.read_text(errors="replace")
        except OSError:
            return False
        return re.search(r"^OpenSCENARIO:", head, re.M) is not None
    return False


def _scenarios_in(d: Path):
    return sorted(p for p in d.rglob("*") if p.is_file() and _is_scenario_file(p))


# --- the checks: ---------------------------------------------------------------------------

def _text(path: Path) -> str:
    try:
        return Path(path).read_text()
    except (OSError, UnicodeDecodeError):
        return ""


def _rel(project: Path, p: Path) -> str:
    try:
        return Path(p).resolve().relative_to(project.resolve()).as_posix()
    except ValueError:
        return str(p)


def check_schema(settings: dict) -> list:
    pf = config.PROJECT_FILE
    out = []
    for k in settings:
        if k not in TOP_KEYS:
            out.append(Finding(pf, f"unknown key {k}", f"use only {', '.join(TOP_KEYS)}"))
    for k in REQUIRED:
        if k not in settings:
            out.append(Finding(pf, f"{k}: is required", "see `scensim project new`"))
    default = settings.get("default")
    if "default" in settings and (not isinstance(default, str) or not default or "/" in default):
        out.append(Finding(pf, "default: is not a suite's name", "default: <suite> "
                           "(suites/<suite>.yaml), the suite `scensim run <testbench>` runs"))
    sim = settings.get("simulator")
    if "simulator" in settings and (not isinstance(sim, str) or not sim):
        out.append(Finding(pf, "simulator: is not a name", "carla, isaac or a simulator package"))
    pilots = settings.get("pilots")
    if pilots is not None and not isinstance(pilots, dict):
        out.append(Finding(pf, "pilots: is not a mapping", "pilots: {ego: {kind: autoware}}"))
        pilots = {}
    for name, p in (pilots or {}).items():
        out += _check_pilot(str(name), p, sim if isinstance(sim, str) else "")
    vehicles = settings.get("vehicles")
    if vehicles is not None and (not isinstance(vehicles, list)
                                 or not all(isinstance(v, str) for v in vehicles)):
        out.append(Finding(pf, "vehicles: is not a list of spec files",
                           "vehicles: [vehicles/my_car.yaml]"))
    maps = settings.get("maps")
    if maps is not None and (not isinstance(maps, list) or not maps
                             or not all(isinstance(m, str) and m for m in maps)):
        out.append(Finding(pf, "maps: is not a list of map names",
                           "maps: [Town01, Town05] (each under assets/maps/)"))
    for where, msg in groundmod.problems(None, settings.get("ground")):
        out.append(Finding(pf, f"{where}: {msg}", "ground: {Town01: {surfaces: ground/t.yaml}}"))
    for where, msg in judgesmod.problems(settings.get("judges")):
        out.append(Finding(pf, f"{where}: {msg}",
                           "judges: [{name: comfort, run: [python3, -m, my_judges.comfort]}]"))
    out += _check_logging(pf, "logging", settings.get("logging"), stack=True)
    for where, msg in worldmod.problems(settings, sim if isinstance(sim, str) else ""):
        out.append(Finding(pf, f"{where}: {msg}"))
    for where, msg in ssv2mod.problems(settings.get("ssv2")):
        out.append(Finding(pf, f"{where}: {msg}", "ssv2: {global_timeout: 600, frame_rate: 20}"))
    return out


def _check_pilot(name, p, sim) -> list:
    pf = config.PROJECT_FILE
    where = f"pilots.{name}"
    if not isinstance(p, dict):
        return [Finding(pf, f"{where}: is not a mapping", f"{name}: {{kind: autoware}}")]
    kind = p.get("kind")
    if kind is None:
        return [Finding(pf, f"{where}: kind: is required", f"one of {', '.join(KINDS)}")]
    if kind in LATER_KINDS:
        return [Finding(pf, f"{where}: kind {kind} is not available yet",
                        f"one of {', '.join(KINDS)}")]
    if kind not in KINDS:
        return [Finding(pf, f"{where}: unknown kind {kind}", f"one of {', '.join(KINDS)}")]
    out = []
    allowed = COMMON_PILOT_KEYS + KIND_KEYS.get(kind, ())
    for k in p:
        if k not in allowed:
            out.append(Finding(pf, f"{where}: unknown key {k} for kind {kind}",
                               f"use only {', '.join(allowed)}"))
    req = KIND_REQUIRED.get(kind)
    if req and not p.get(req):
        out.append(Finding(pf, f"{where}: kind {kind} needs {req}:"))
    if kind == "command" and p.get("run") is not None and (
            not isinstance(p["run"], list) or not p["run"]):
        out.append(Finding(pf, f"{where}: run: is not an argument list",
                           "run: [python3, -m, my_pilot]"))
    if kind in CARLA_ONLY and sim and sim != "carla":
        out.append(Finding(pf, f"{where}: kind {kind} runs on CARLA only, the simulator is"
                           f" {sim}", "another kind (autoware, simple, launch, command)"))
    if p.get("lifetime") is not None and p["lifetime"] not in LIFETIMES:
        out.append(Finding(pf, f"{where}: lifetime {p['lifetime']} is not stack or run"))
    inputs = p.get("inputs")
    if inputs is not None:
        bad = [i for i in (inputs if isinstance(inputs, list) else [inputs]) if i not in INPUTS]
        if bad or not isinstance(inputs, list):
            out.append(Finding(pf, f"{where}: inputs {bad or inputs} not among"
                               f" {', '.join(INPUTS)}"))
    if p.get("args") is not None and not isinstance(p["args"], dict):
        out.append(Finding(pf, f"{where}: args: is not a mapping", "args: {name: value}"))
    if p.get("vehicle") is not None and not isinstance(p["vehicle"], str):
        out.append(Finding(pf, f"{where}: vehicle: is not a catalog name"))
    if kind in overlaysmod.KINDS:
        out += overlaysmod.schema_problems(name, p)
    return out


def _check_logging(file, where, lg, stack=False) -> list:
    if lg is None:
        return []
    if not isinstance(lg, dict):
        return [Finding(file, f"{where}: is not a mapping")]
    out = []
    for k in lg:
        if k in STACK_LOGGING and not stack:
            out.append(Finding(file, f"{where}.{k}: {STACK_LOGGING[k]}", f"remove {k}:"))
        elif k in LOGGING_NOT_YET:
            out.append(Finding(file, f"{where}.{k}: {LOGGING_NOT_YET[k]}", f"remove {k}:"))
        elif k not in LOGGING_KEYS:
            out.append(Finding(file, f"{where}: unknown key {k}",
                               f"use only {', '.join(LOGGING_KEYS)}"))
    if lg.get("capture") is not None and lg["capture"] not in LEVELS:
        out.append(Finding(file, f"{where}.capture: {lg['capture']} is not a level",
                           f"one of {', '.join(LEVELS)}"))
    if stack and lg.get("gap_trace") is not None and not isinstance(lg["gap_trace"], bool):
        out.append(Finding(file, f"{where}.gap_trace: {lg['gap_trace']!r} is not true or false"))
    for msg in recordmod.topics_problems(lg.get("topics")):
        out.append(Finding(file, f"{where}.{msg}", "topics: [/localization/kinematic_state]"))
    for msg in budgetmod.problems(lg.get("budget")):
        out.append(Finding(file, f"{where}.budget: {msg}", "budget: {per_run: 2GB, total: 200GB}"))
    keep = lg.get("keep")
    if keep is not None:
        if not isinstance(keep, dict):
            out.append(Finding(file, f"{where}.keep: is not a mapping",
                               "keep: {passed: result, failed: full}"))
        else:
            for v, lvl in keep.items():
                if v not in VERDICTS:
                    out.append(Finding(file, f"{where}.keep: unknown verdict {v}",
                                       f"one of {', '.join(VERDICTS)}"))
                elif lvl not in LEVELS:
                    out.append(Finding(file, f"{where}.keep.{v}: {lvl} is not a level",
                                       f"one of {', '.join(LEVELS)}"))
    return out


def pilot_for(pilots: dict, entity: str):
    """The pilot key for an entity: its own name first, then the first matching pattern."""
    if entity in pilots:
        return entity
    for key in pilots:
        if fnmatch.fnmatchcase(entity, str(key)):
            return key
    return None


def suite_items(listed):
    """A suite's ``scenarios:`` as ``([(path, ssv2 settings)], problems)``: each item a path,
    or a mapping ``{path: <file or directory>, ssv2: {...}}`` (run settings for its runs)."""
    if listed is None:
        return [], []
    if isinstance(listed, str):
        listed = [listed]
    if not isinstance(listed, list):
        return [], ["scenarios: is not a list"]
    out, bad = [], []
    for i, item in enumerate(listed):
        if isinstance(item, dict):
            unknown = [k for k in item if k not in SUITE_ITEM_KEYS]
            if unknown:
                bad.append(f"scenarios[{i}]: unknown key {unknown[0]} (use only "
                           f"{', '.join(SUITE_ITEM_KEYS)})")
            if not isinstance(item.get("path"), str) or not item["path"]:
                bad.append(f"scenarios[{i}]: path: is required (a file or a directory)")
                continue
            probs = ssv2mod.problems(item.get("ssv2"), f"scenarios[{i}].ssv2")
            bad += [f"{w}: {m}" for w, m in probs]
            out.append((item["path"], dict(item.get("ssv2") or {}) if not probs else {}))
        else:
            out.append((str(item), {}))
    return out, bad


def unmanaged_scenarios(project: Path, settings: dict) -> set:
    """The scenario files some layer runs with ``managed_ego: false`` (testbench.yaml's ``ssv2:``
    makes it every one); their ego needs no pilot."""
    base = ssv2mod.merge(settings.get("ssv2") if isinstance(settings.get("ssv2"), dict) else {})
    out = set()
    sdir = project / "scenarios"
    if not ssv2mod.managed(base) and sdir.is_dir():
        out |= {p.resolve() for p in _scenarios_in(sdir)}
    for suite in sorted((project / "suites").glob("*.y*ml")) if (project / "suites").is_dir() \
            else []:
        try:
            data = config.read_yaml(suite)
        except config.ConfigError:
            continue
        top = data.get("ssv2") if isinstance(data.get("ssv2"), dict) else {}
        items, _ = suite_items(data.get("scenarios"))
        for path, own in items:
            if ssv2mod.managed(ssv2mod.merge(base, top, own)):
                continue
            p = project / path
            if not p.exists() and (suite.parent / path).exists():
                p = suite.parent / path
            files = _scenarios_in(p) if p.is_dir() else [p] if p.is_file() else []
            out |= {f.resolve() for f in files}
    return out


def profile_problems(project: Path, settings: dict) -> list:
    """The Autoware profile's refusals (check_launch_args) of each ``autoware`` pilot's
    ``args``, and of a suite's ``pilot_args`` over them (scensim.profile)."""
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    out = []
    for key, p in pilots.items():
        if not isinstance(p, dict) or p.get("kind") != "autoware" or \
                not isinstance(p.get("args") or {}, dict):
            continue
        for msg in profilemod.arg_problems(p.get("args") or {}, project):
            out.append(Finding(config.PROJECT_FILE, f"pilots.{key}.args: {msg}",
                               "the Autoware profile refuses it at start (check_launch_args)"))
    sdir = project / "suites"
    for suite in sorted(sdir.glob("*.y*ml")) if sdir.is_dir() else []:
        try:
            data = config.read_yaml(suite)
        except config.ConfigError:
            continue
        pa = data.get("pilot_args")
        for entity, extra in (pa.items() if isinstance(pa, dict) else []):
            key = pilot_for(pilots, str(entity))
            p = pilots.get(key) if key is not None else None
            if not isinstance(p, dict) or p.get("kind") != "autoware" or \
                    not isinstance(extra, dict):
                continue
            merged = {**(p.get("args") if isinstance(p.get("args"), dict) else {}), **extra}
            own = set(profilemod.arg_problems(p.get("args") or {}, project))
            for msg in profilemod.arg_problems(merged, project):
                if msg not in own:
                    out.append(Finding(_rel(project, suite), f"pilot_args.{entity}: {msg}",
                                       "the Autoware profile refuses it at start"
                                       " (check_launch_args)"))
    return out


def collect_scenarios(project: Path):
    """``(scenario files, findings)``: scenarios/ and every suite's scenarios."""
    files, findings = [], []
    sd = project / "scenarios"
    if sd.is_dir():
        files += _scenarios_in(sd)
    sdir = project / "suites"
    for suite in sorted(sdir.glob("*.y*ml")) if sdir.is_dir() else []:
        rel = _rel(project, suite)
        try:
            data = config.read_yaml(suite)
        except config.ConfigError as e:
            findings.append(Finding(rel, str(e).split(": ", 1)[-1]))
            continue
        for k in data:
            if k not in SUITE_KEYS:
                findings.append(Finding(rel, f"unknown key {k}",
                                        f"use only {', '.join(SUITE_KEYS)}"))
        findings += _check_logging(rel, "logging", data.get("logging"))
        for where, msg in ssv2mod.problems(data.get("ssv2")):
            findings.append(Finding(rel, f"{where}: {msg}"))
        items, bad = suite_items(data.get("scenarios"))
        findings += [Finding(rel, msg) for msg in bad]
        listed = [path for path, _ in items]
        if data.get("distribution"):
            listed = list(listed) + [data["distribution"]]
        for item in listed:
            p = project / str(item)
            if not p.exists() and (suite.parent / str(item)).exists():
                p = suite.parent / str(item)
            if p.is_dir():
                files += _scenarios_in(p)
            elif p.is_file():
                files.append(p)
            else:
                findings.append(Finding(rel, f"scenario {item}: no such file or directory"))
    seen, out = set(), []
    for f in files:
        r = f.resolve()
        if r not in seen:
            seen.add(r)
            out.append(f)
    return out, findings


def _map_fix(name, settings):
    for e in settings.get("assets") or []:
        if isinstance(e, dict) and name in (e.get("take") or []):
            return f"{FETCH}"
    return (f"add it to assets: (e.g. - dataset: carla-town-autoware-maps, take: [{name}]),"
            f" then {FETCH}")


def check_scenario(project: Path, sc: Scenario, settings: dict, managed: bool = True) -> list:
    """``managed``: False when the scenario runs with ``managed_ego: false`` (its ego needs
    no pilot)."""
    rel = _rel(project, sc.path)
    out = []
    if sc.distribution_of is not None:
        target = Path(sc.distribution_of)
        if not target.is_absolute():
            target = sc.path.parent / target
        if "$(" not in sc.distribution_of and not target.is_file():
            out.append(Finding(rel, f"ScenarioFile {sc.distribution_of}: no such file"))
        return out
    lf = sc.logic_file
    if not lf:
        out.append(Finding(rel, "no RoadNetwork/LogicFile",
                           f'<LogicFile filepath="{MAPS_PREFIX}<map>"/>'))
    elif not lf.startswith(MAPS_PREFIX):
        tail = re.sub(r"^\$\(env [A-Z_]+\)/", "", lf).rstrip("/").split("/")[-1]
        what = f"LogicFile {lf} does not name a project map"
        if lf.startswith(RETIRED_MAPS_PREFIX):
            # retired for projects (roadmap 076 step 10): the developers' map directory
            what += " (CARLA_MAPS is the developers' map directory, not the project's)"
        elif lf.startswith("$(env SWB_MAPS)/"):
            what += " (SWB_MAPS was renamed SCENSIM_MAPS with the project, roadmap 080)"
        out.append(Finding(rel, what, f'filepath="{MAPS_PREFIX}{tail}"'))
    else:
        name = lf[len(MAPS_PREFIX):].strip("/")
        if not (project / "assets" / "maps" / name).is_dir():
            out.append(Finding(rel, f"map {name} is not fetched (assets/maps/{name})",
                               _map_fix(name, settings), refused=True, host=True))
    # roadmap 080: the scenario properties of the old name (swb.spec_hash, swb.zone.*, ...)
    # are not read by any part; each is named with its successor
    for old in sorted(set(re.findall(r'name="(swb\.[^"]+)"', _text(sc.path)))):
        out.append(Finding(rel, f"property {old} was renamed with the project (roadmap 080)",
                           f'name="scensim.{old[len("swb."):]}"', refused=True))
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    sim = settings.get("simulator")
    external = [e for e in sc.entities if e.external]
    for ent in external:
        key = pilot_for(pilots, ent.name)
        how = f"controller {ent.controller}" if ent.controller else "isEgo"
        if key is None and not managed and ent.ego:
            continue   # managed_ego: false, SSv2 hands its ego to nobody
        if key is None:
            out.append(Finding(rel, f"entity {ent.name} ({how}) has no pilot",
                               f"pilots: {{{ent.name}: {{kind: autoware}}}} in"
                               f" {config.PROJECT_FILE}"))
            continue
        p = pilots[key] if isinstance(pilots[key], dict) else {}
        if p.get("kind") == "none" and ent.controller.lower() == "autoware" and managed:
            # G34: SSv2's managed ego waits for a vehicle side, and kind none starts none
            out.append(Finding(rel, f"entity {ent.name} (controller {ent.controller}) is bound"
                               f" to pilot {key} of kind none, which starts no vehicle side;"
                               " SSv2's Autoware ego needs one",
                               f"pilots: {{{key}: {{kind: autoware}}}} in {config.PROJECT_FILE};"
                               " or, for a process of your own, the scenario's"
                               ' <Controller name="agent"> on that entity',
                               refused=True))
        car = p.get("vehicle")
        if car and ent.catalog_entry and car != ent.catalog_entry:
            out.append(Finding(rel, f"entity {ent.name} is a {ent.catalog_entry}, its pilot"
                               f" {key} drives a {car}",
                               "one car: change the scenario's CatalogReference or the"
                               " pilot's vehicle:"))
        if car and ent.geometry:
            cat = catalog_geometry(car)
            bad = geometry_problems(ent.geometry, cat) if cat else []
            if bad:
                out.append(Finding(rel, f"entity {ent.name} declares a car that is not the"
                                   f" catalog's {car}: " + "; ".join(bad),
                                   f"use the catalog's {car} (a CatalogReference), or its"
                                   " numbers; the bridge refuses an ego more than"
                                   f" {GEOMETRY_TOLERANCE * 100:g} cm off", refused=True))
    if sim == "isaac":
        # Isaac has no driver of its own (roadmap 077, G66): the world server refuses the
        # spawn, and SSv2 reports only "Spawn entity ... resulted in failure"
        for ent in sc.entities:
            if ent.controller.lower() == "simulator_autopilot":
                out.append(Finding(rel, f"entity {ent.name}: controller simulator_autopilot runs"
                                   " on CARLA only, the simulator is isaac (Isaac has no"
                                   " driver of its own)",
                                   "script the entity's motion (a route or trajectory), or"
                                   " simulator: carla", refused=True))
    if sim == "isaac" and len(external) > 1:
        out.append(Finding(rel, f"{len(external)} externally driven entities"
                           f" ({', '.join(e.name for e in external)}); Isaac drives one",
                           "one ego per scenario on Isaac, or simulator: carla"))
    return out


def own_car_problems(project: Path, settings: dict) -> list:
    """``vehicles:`` (roadmap 077, G5): names and bodies of their own; an autoware pilot of
    an own car needs the car's Autoware packages installed; Isaac's rig is not made yet."""
    pf = config.PROJECT_FILE
    out = [Finding(pf, m, refused=True) for m in vehiclesmod.problems(project, settings)]
    try:
        own = {n for n, _, _ in vehiclesmod.spec_names(project, settings)}
    except vehiclesmod.VehicleError:
        return out
    sim = settings.get("simulator")
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    for key, p in pilots.items():
        if not isinstance(p, dict) or p.get("vehicle") not in own:
            continue
        car = p["vehicle"]
        if sim == "isaac":
            out.append(Finding(pf, f"pilots.{key}: {car} is a car of the project's own; on "
                               "Isaac its rig is not made from the spec yet", "simulator: carla,"
                               " or an installed car", refused=True))
        if p.get("kind") == "autoware" and not _package_installed(f"{car}_description"):
            out.append(Finding(pf, f"pilots.{key}: an autoware pilot of {car} needs Autoware's"
                               f" packages of the car ({car}_description, {car}_sensor_kit_*),"
                               " installed", "generate them from the spec (autoware_sim_vehicle"
                               "'s autoware_gen), build and source them; or another pilot kind",
                               refused=True))
    return out


def _package_installed(name: str) -> bool:
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(name)
        return True
    except Exception:  # noqa: BLE001 (not found, or no ROS environment)
        return False


def project_notes(path) -> list:
    """Warnings for the project at ``path`` that do not stop it (a host path in testbench.yaml)."""
    try:
        return overlaysmod.notes(config.project_settings(config.find_project(path)))
    except config.ConfigError:
        return []


# what an autoware pilot's car needs installed (autoware_sim_vehicle builds them per car): the
# vehicle description and the vehicle launch Autoware's planning simulator includes
VEHICLE_PACKAGES = ("{car}_description", "{car}_launch")


def _installed(pkg: str) -> bool:
    try:
        from ament_index_python.packages import get_package_prefix
        get_package_prefix(pkg)
        return True
    except Exception:  # noqa: BLE001 (not installed, or no ROS environment)
        return False


def vehicle_package_problems(settings: dict, installed=None) -> list:
    """Each ``autoware`` pilot's car whose packages the sourced chain lacks, refused by name
    (081 step 3): the pilot would fail at launch, minutes later, with only Autoware's
    "package not found"."""
    installed = installed or _installed
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    out = []
    for key, p in pilots.items():
        if not isinstance(p, dict) or p.get("kind") != "autoware":
            continue
        car = p.get("vehicle")
        if not isinstance(car, str) or not car:
            continue
        missing = [n.format(car=car) for n in VEHICLE_PACKAGES
                   if not installed(n.format(car=car))]
        if missing:
            out.append(Finding(config.PROJECT_FILE,
                               f"pilots.{key}: vehicle {car}: package "
                               f"{' and '.join(missing)} not installed",
                               "source the workspace that builds the car's Autoware packages "
                               f"(autoware_sim_vehicle for the stock cars), or build {car}'s "
                               "in yours", refused=True, host=True))
    return out


def check_project(path, scenarios: bool = True) -> list:
    """Every finding for the project at ``path`` (a directory or pkg://<package>).
    ``scenarios=False``: the project's own findings only, what a stack starts on; each
    scenario's (`check_scenario`) refuses the run that names it instead (`scenario_problems`,
    G66: one CARLA-only scenario in an Isaac project must not stop the others)."""
    try:
        project = config.find_project(path)
        settings = config.project_settings(project)
    except config.ConfigError as e:
        return [Finding(str(path or "."), str(e), refused=True)]
    out = check_schema(settings)
    try:
        launcher = config.host_settings(project).get("launcher")
    except config.ConfigError:
        launcher = "auto"
    out += on_host(toolsmod.findings(launcher))
    out += on_host(toolsmod.rmw_findings())
    out += on_host(toolsmod.agent_findings(settings))
    out += on_host(assets.check(project, settings))
    out += vehicle_package_problems(settings)
    missing = False
    for v in settings.get("vehicles") or []:
        if isinstance(v, str) and not (project / v).is_file():
            out.append(Finding(config.PROJECT_FILE, f"vehicles: {v}: no such file"))
            missing = True
    if settings.get("vehicles") and not missing and isinstance(settings["vehicles"], list):
        out += own_car_problems(project, settings)
    if isinstance(settings.get("ground"), dict):
        for where, msg in groundmod.problems(project, settings["ground"]):
            if "is not a file" in msg:
                out.append(Finding(config.PROJECT_FILE, f"{where}: {msg}"))
    maps = settings.get("maps")
    for m in maps if isinstance(maps, list) else []:
        if isinstance(m, str) and m and not (project / "assets" / "maps" / m).is_dir():
            out.append(Finding(config.PROJECT_FILE, f"maps: {m} is not fetched (assets/maps/{m})",
                               _map_fix(m, settings), refused=True, host=True))
    sim = settings.get("simulator") if isinstance(settings.get("simulator"), str) else ""
    if sim == "carla" and isinstance(maps, list):
        from scensim import simulator as simmod
        try:
            carla = config.host_settings(project).get("carla") or {}
        except config.ConfigError:
            carla = {}
        for m, town, dirs in simmod.town_problems(maps, carla):
            out.append(Finding(config.PROJECT_FILE, f"maps: {m}: this host's CARLA has no town "
                               f"{town} (no {town}.umap under {', '.join(map(str, dirs))})",
                               "install its CARLA map package, or name where this host's are: "
                               "carla: {map_packages: [<dir>]} in ~/.config/scensim/host.yaml",
                               refused=True, host=True))
    shape = set(worldmod.problems(settings, sim))
    for where, msg in worldmod.problems(settings, sim, project):
        if (where, msg) not in shape:   # the files (check_schema reported the rest)
            out.append(Finding(config.PROJECT_FILE, f"{where}: {msg}",
                               refused="not fetched" in msg, host="not fetched" in msg))
    out += profile_problems(project, settings)
    try:
        host = config.host_settings(project)
    except config.ConfigError:
        host = {}
    out += on_host(overlaysmod.problems(project, settings, host))
    if sim == "isaac":
        out += on_host(isaacmod.findings(config.host_settings(project)))
    files, found = collect_scenarios(project)
    out += found
    if not files:
        out.append(Finding("scenarios/", "no scenario", "add one, e.g. scenarios/drive.xosc"))
    if scenarios:
        out += scenario_problems(project, files, settings)
    return out


def check_stack(path) -> list:
    """What a stack refuses to start on: the project's findings, without its scenarios'."""
    return check_project(path, scenarios=False)


def scenario_problems(project: Path, files, settings: dict = None) -> list:
    """The findings of each scenario in ``files`` against the project (its pilots, its
    simulator, its maps): what refuses a run of it."""
    settings = config.project_settings(project) if settings is None else settings
    unmanaged = unmanaged_scenarios(project, settings)
    out = []
    for f in files:
        try:
            sc = read_scenario(f)
        except (ValueError, OSError) as e:
            out.append(Finding(_rel(project, f), str(e)))
            continue
        out += check_scenario(project, sc, settings, managed=Path(f).resolve() not in unmanaged)
    return out


# --- new: ----------------------------------------------------------------------------------

def templates() -> list:
    return sorted(p.name for p in TEMPLATES.iterdir() if p.is_dir()) if TEMPLATES.is_dir() else []


def new_project(dest: Path, template: str = DEFAULT_TEMPLATE) -> Path:
    src = TEMPLATES / template
    if not src.is_dir():
        raise config.ConfigError(f"no template {template} (fix: one of {', '.join(templates())})")
    dest = _empty_dest(dest)
    dest.mkdir(parents=True, exist_ok=True)
    name = dest.resolve().name
    for f in sorted(src.rglob("*")):
        if f.is_dir() or "__pycache__" in f.parts:
            continue
        rel = f.relative_to(src)
        out = dest / rel.parent / RENAMES.get(rel.name, rel.name)
        out.parent.mkdir(parents=True, exist_ok=True)
        if f.suffix in (".md", ".yaml"):
            out.write_text(f.read_text().replace("@PROJECT@", name))
        else:
            shutil.copyfile(f, out)
    return dest


def _empty_dest(dest: Path) -> Path:
    dest = Path(dest).expanduser()
    if dest.exists() and (not dest.is_dir() or any(dest.iterdir())):
        raise config.ConfigError(f"{dest}: exists and is not empty (fix: a new directory)")
    return dest


def source_project(src) -> Path:
    """The project directory ``src`` names: a directory, or ``pkg://<package>[/<subdir>]``
    (an installed example: ``pkg://scensim_examples/projects/first_drive``)."""
    src = str(src)
    if src.startswith("pkg://"):
        return config.find_project(src)
    p = Path(src).expanduser()
    if not (p / config.PROJECT_FILE).is_file():
        raise config.ConfigError(f"{p}: no {config.PROJECT_FILE} (fix: a project directory,"
                                 " e.g. pkg://scensim_examples/projects/first_drive)")
    return p.resolve()


def copy_project(src, dest: Path) -> Path:
    """A copy of the project ``src`` (e.g. an installed, read-only example) in the new
    directory ``dest``: its files, without what each host makes (assets/, state/, runs,
    .scensim/, play_log/). The copies are the user's own: writable, symlinks followed."""
    root = source_project(src)
    dest = _empty_dest(dest)
    full = dest.resolve()
    if full == root or root in full.parents:
        raise config.ConfigError(f"{dest}: inside the project it copies (fix: another directory)")
    files = []
    for f in sorted(root.rglob("*")):
        rel = f.relative_to(root)
        if rel.parts[0] in NOT_COPIED or "__pycache__" in rel.parts or not f.is_file():
            continue
        files.append(rel)
    dest.mkdir(parents=True, exist_ok=True)
    for rel in files:
        out = dest / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / rel, out)
        out.chmod(((root / rel).stat().st_mode & 0o777) | 0o600)
    return dest


def main(argv=None):
    parser = argparse.ArgumentParser(prog="scensim project",
                                     description="Make a starter project, or check one.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    n = sub.add_parser("new", help="write a starter project into a new directory")
    n.add_argument("dir")
    how = n.add_mutually_exclusive_group()
    how.add_argument("--template", default=None,
                     help=f"one of {', '.join(templates())} (default {DEFAULT_TEMPLATE})")
    how.add_argument("--from", dest="source", metavar="PROJECT",
                     help="copy this project instead (a directory or pkg://<package>/<dir>,"
                          " e.g. an installed example), without assets/, state/, runs/")
    c = sub.add_parser("check", help="validate the project against its scenarios and assets")
    c.add_argument("dir", nargs="?")
    add_project_option(c)
    args = parser.parse_args(ros_args(sys.argv[1:] if argv is None else argv))
    if args.cmd == "new":
        try:
            if args.source:
                dest = copy_project(args.source, Path(args.dir))
                what = f"a copy of {args.source}"
            else:
                dest = new_project(Path(args.dir), args.template or DEFAULT_TEMPLATE)
                what = args.template or DEFAULT_TEMPLATE
        except config.ConfigError as e:
            print(f"problem: {e}", file=sys.stderr)
            return REFUSED
        print(f"wrote {dest} ({what}); next: cd {args.dir} && {FETCH}")
        return 0
    findings = check_project(args.dir or args.project)
    for f in findings:
        print(f)
    try:
        where = config.find_project(args.dir or args.project)
        note = toolsmod.tensorrt_note(where, config.project_settings(where),
                                      config.host_settings(where))
    except config.ConfigError:
        note = ""
    if note:
        print(f"warning: {note}")
    for n in project_notes(args.dir or args.project):
        print(f"warning: {n}")
    try:
        where = config.find_project(args.dir or args.project)
        setting = config.host_settings(where).get("launcher")
        print(toolsmod.launcher_line(setting))
        pilots = config.project_settings(where).get("pilots")
        kinds = [p.get("kind") for p in pilots.values() if isinstance(p, dict)] \
            if isinstance(pilots, dict) else []
        for n in toolsmod.launcher_notes(toolsmod.launcher(setting)[0], kinds):
            print(f"note: {n}")
        for n in toolsmod.rviz_display_note(pilots if isinstance(pilots, dict) else {}):
            print(f"warning: {n}")
    except config.ConfigError:
        pass
    if not findings:
        print("project: ok")
    return exit_code(findings)


if __name__ == "__main__":
    sys.exit(main())
