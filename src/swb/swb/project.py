"""``ros2 run swb project new|check``: a starter project, and a project checked before it runs.

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

from swb import assets, budget as budgetmod, config, judges as judgesmod
from swb.findings import REFUSED, Finding, add_project_option, exit_code, ros_args

TEMPLATES = Path(__file__).resolve().parent / "templates"
DEFAULT_TEMPLATE = "carla-first-drive"
# a template's files whose names a package cannot carry as they are
RENAMES = {"gitignore": ".gitignore"}

TOP_KEYS = ("simulator", "assets", "maps", "pilots", "vehicles", "ground", "logging", "judges")
REQUIRED = ("simulator", "pilots", "assets")
SIMULATORS = ("carla", "isaac")
KINDS = ("autoware", "carla_basic", "simple", "launch", "command", "none")
LATER_KINDS = ("container",)
CARLA_ONLY = ("carla_basic",)
COMMON_PILOT_KEYS = ("kind", "vehicle", "args", "lifetime", "ready", "inputs", "rviz")
KIND_KEYS = {
    "autoware": ("autoware_launch",),
    "launch": ("launch", "agent"),
    "command": ("run",),
}
KIND_REQUIRED = {"launch": "launch", "command": "run"}
LIFETIMES = ("stack", "run")
INPUTS = ("sensors", "map", "ground_truth")
LEVELS = ("result", "standard", "full", "debug")
VERDICTS = ("passed", "failed", "error")
LOGGING_KEYS = ("capture", "keep", "levels", "budget", "debug")
SUITE_KEYS = ("scenarios", "parameters", "distribution", "repetitions", "seed", "expect",
              "retry", "pilot_args", "logging")
EXTERNAL_CONTROLLERS = ("agent", "autoware")
MAPS_PREFIX = "$(env SWB_MAPS)/"
FETCH = assets.FETCH


@dataclass
class Entity:
    name: str
    controller: str = ""
    properties: dict = field(default_factory=dict)
    catalog_entry: str = None

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
    else:
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            raise ValueError(f"not valid XML: {e}") from None
        if root.tag != "OpenSCENARIO":
            raise ValueError(f"the root element is <{root.tag}>, not <OpenSCENARIO>")
    sc = Scenario(path)
    sf = root.find("ParameterValueDistribution/ScenarioFile")
    if sf is not None:
        sc.distribution_of = sf.get("filepath")
        return sc
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
        sc.entities.append(ent)
    return sc


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
            out.append(Finding(pf, f"{k}: is required", "see `ros2 run swb project new`"))
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
    if settings.get("ground") is not None and not isinstance(settings["ground"], dict):
        out.append(Finding(pf, "ground: is not a mapping", "ground: {<map>: ...}"))
    for where, msg in judgesmod.problems(settings.get("judges")):
        out.append(Finding(pf, f"{where}: {msg}",
                           "judges: [{name: comfort, run: [python3, -m, my_judges.comfort]}]"))
    out += _check_logging(pf, "logging", settings.get("logging"))
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
    return out


def _check_logging(file, where, lg) -> list:
    if lg is None:
        return []
    if not isinstance(lg, dict):
        return [Finding(file, f"{where}: is not a mapping")]
    out = []
    for k in lg:
        if k not in LOGGING_KEYS:
            out.append(Finding(file, f"{where}: unknown key {k}",
                               f"use only {', '.join(LOGGING_KEYS)}"))
    if lg.get("capture") is not None and lg["capture"] not in LEVELS:
        out.append(Finding(file, f"{where}.capture: {lg['capture']} is not a level",
                           f"one of {', '.join(LEVELS)}"))
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
        listed = data.get("scenarios") or []
        if isinstance(listed, str):
            listed = [listed]
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


def check_scenario(project: Path, sc: Scenario, settings: dict) -> list:
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
        out.append(Finding(rel, f"LogicFile {lf} does not name a project map",
                           f'filepath="{MAPS_PREFIX}{tail}"'))
    else:
        name = lf[len(MAPS_PREFIX):].strip("/")
        if not (project / "assets" / "maps" / name).is_dir():
            out.append(Finding(rel, f"map {name} is not fetched (assets/maps/{name})",
                               _map_fix(name, settings), refused=True))
    pilots = settings.get("pilots") if isinstance(settings.get("pilots"), dict) else {}
    sim = settings.get("simulator")
    external = [e for e in sc.entities if e.external]
    for ent in external:
        key = pilot_for(pilots, ent.name)
        how = f"controller {ent.controller}" if ent.controller else "isEgo"
        if key is None:
            out.append(Finding(rel, f"entity {ent.name} ({how}) has no pilot",
                               f"pilots: {{{ent.name}: {{kind: autoware}}}} in"
                               f" {config.PROJECT_FILE}"))
            continue
        p = pilots[key] if isinstance(pilots[key], dict) else {}
        car = p.get("vehicle")
        if car and ent.catalog_entry and car != ent.catalog_entry:
            out.append(Finding(rel, f"entity {ent.name} is a {ent.catalog_entry}, its pilot"
                               f" {key} drives a {car}",
                               "one car: change the scenario's CatalogReference or the"
                               " pilot's vehicle:"))
    if sim == "isaac" and len(external) > 1:
        out.append(Finding(rel, f"{len(external)} externally driven entities"
                           f" ({', '.join(e.name for e in external)}); Isaac drives one",
                           "one ego per scenario on Isaac, or simulator: carla"))
    return out


def check_project(path) -> list:
    """Every finding for the project at ``path`` (a directory or pkg://<package>)."""
    try:
        project = config.find_project(path)
        settings = config.project_settings(project)
    except config.ConfigError as e:
        return [Finding(str(path or "."), str(e), refused=True)]
    out = check_schema(settings)
    out += assets.check(project, settings)
    for v in settings.get("vehicles") or []:
        if isinstance(v, str) and not (project / v).is_file():
            out.append(Finding(config.PROJECT_FILE, f"vehicles: {v}: no such file"))
    maps = settings.get("maps")
    for m in maps if isinstance(maps, list) else []:
        if isinstance(m, str) and m and not (project / "assets" / "maps" / m).is_dir():
            out.append(Finding(config.PROJECT_FILE, f"maps: {m} is not fetched (assets/maps/{m})",
                               _map_fix(m, settings), refused=True))
    files, found = collect_scenarios(project)
    out += found
    if not files:
        out.append(Finding("scenarios/", "no scenario", "add one, e.g. scenarios/drive.xosc"))
    for f in files:
        try:
            sc = read_scenario(f)
        except (ValueError, OSError) as e:
            out.append(Finding(_rel(project, f), str(e)))
            continue
        out += check_scenario(project, sc, settings)
    return out


# --- new: ----------------------------------------------------------------------------------

def templates() -> list:
    return sorted(p.name for p in TEMPLATES.iterdir() if p.is_dir()) if TEMPLATES.is_dir() else []


def new_project(dest: Path, template: str = DEFAULT_TEMPLATE) -> Path:
    src = TEMPLATES / template
    if not src.is_dir():
        raise config.ConfigError(f"no template {template} (fix: one of {', '.join(templates())})")
    dest = Path(dest).expanduser()
    if dest.exists() and (not dest.is_dir() or any(dest.iterdir())):
        raise config.ConfigError(f"{dest}: exists and is not empty (fix: a new directory)")
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


def main(argv=None):
    parser = argparse.ArgumentParser(prog="ros2 run swb project",
                                     description="Make a starter project, or check one.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    n = sub.add_parser("new", help="write a starter project into a new directory")
    n.add_argument("dir")
    n.add_argument("--template", default=DEFAULT_TEMPLATE,
                   help=f"one of {', '.join(templates())} (default {DEFAULT_TEMPLATE})")
    c = sub.add_parser("check", help="validate the project against its scenarios and assets")
    c.add_argument("dir", nargs="?")
    add_project_option(c)
    args = parser.parse_args(ros_args(sys.argv[1:] if argv is None else argv))
    if args.cmd == "new":
        try:
            dest = new_project(Path(args.dir), args.template)
        except config.ConfigError as e:
            print(f"problem: {e}", file=sys.stderr)
            return REFUSED
        print(f"wrote {dest} ({args.template}); next: cd {args.dir} && {FETCH}")
        return 0
    findings = check_project(args.dir or args.project)
    for f in findings:
        print(f)
    if not findings:
        print("project: ok")
    return exit_code(findings)


if __name__ == "__main__":
    sys.exit(main())
