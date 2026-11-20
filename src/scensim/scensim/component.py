"""A pilot's own control component (roadmap 091 W3): ``component:`` on a pilot with a vehicle side, on Isaac.

    pilots:
      ego:
        kind: launch
        vehicle: tesla_model3
        overlays: ["{workspace:racer}/install/setup.bash"]
        component:
          entry_point: my_racer.vesc_servo     # its name in group scensim.vehicle_components
          params: {erpm_gain: 4614.0}          # handed to it as is
          truth: {speed: {latency_s: 0.02, noise_std: 0.01}}   # its declared inputs' latency, noise
          workspace: "{workspace:racer}/install/setup.bash"     # where it is; default: the overlays

The component replaces the car's onboard computers (sim_vehicle_ecu's interface; docs/design/
roboracer-indoor-twin.md, section 2). The Isaac world server imports it in Isaac's own Python, so scensim
hands it only the workspace's Python package directories (``paths``), never the workspace's environment:
the stack writes them, with the rest, into the ego config's ``component`` (aib's Rig.make_component puts
them on sys.path).

Pure but for reading files: the entry point is found from the packages' metadata without importing them
(``lookup``); importing and timing it is ``isaac_check``'s, in Isaac's interpreter.
"""
import hashlib
import json
import os
import subprocess
from pathlib import Path

from scensim import config
from scensim import overlays as overlaysmod
from scensim.findings import Finding

KEYS = ("entry_point", "params", "truth", "workspace")
GROUP = "scensim.vehicle_components"
SHIPPED = ("ideal", "ev")
TRUTH_SETTINGS = ("latency_s", "noise_std")
# sim_vehicle_ecu.component.TRUTH_INPUTS's names (checked against it when it imports)
TRUTH_INPUTS = ("pose", "yaw", "velocity", "speed", "yaw_rate", "wheel_slip", "sideslip")
CHECK_STEPS = 1000


def truth_inputs():
    try:
        from sim_vehicle_ecu.component import TRUTH_INPUTS as names
        return tuple(names)
    except ImportError:
        return TRUTH_INPUTS


def spec_of(pilot: dict):
    c = (pilot or {}).get("component")
    return c if isinstance(c, dict) else None


def schema_problems(name: str, pilot: dict, simulator: str) -> list:
    """The shape of ``pilots.<name>.component`` (no host settings needed)."""
    if (pilot or {}).get("component") is None:
        return []
    where = f"pilots.{name}.component"
    c = pilot["component"]
    if not isinstance(c, dict):
        return [Finding(config.PROJECT_FILE, f"{where}: is not a mapping",
                        "component: {entry_point: <name>, params: {...}}")]
    out = []
    unknown = [k for k in c if k not in KEYS]
    if unknown:
        out.append(Finding(config.PROJECT_FILE, f"{where}: unknown key {', '.join(map(str, unknown))}",
                           f"use only {', '.join(KEYS)}"))
    if not (isinstance(c.get("entry_point"), str) and c["entry_point"].strip()):
        out.append(Finding(config.PROJECT_FILE, f"{where}: entry_point: <name> is required",
                           f"its name in entry point group {GROUP}"))
    if c.get("params") is not None and not isinstance(c["params"], dict):
        out.append(Finding(config.PROJECT_FILE, f"{where}.params: is not a mapping"))
    truth = c.get("truth")
    if truth is not None:
        if not isinstance(truth, dict):
            out.append(Finding(config.PROJECT_FILE, f"{where}.truth: is not a mapping",
                               "truth: {speed: {latency_s: 0.02, noise_std: 0.01}}"))
        else:
            names = truth_inputs()
            for k, v in truth.items():
                if k not in names:
                    out.append(Finding(config.PROJECT_FILE, f"{where}.truth: {k} is not a ground-truth "
                                       "input", f"one of {', '.join(names)}"))
                elif not isinstance(v, dict) or set(v) - set(TRUTH_SETTINGS) or not all(
                        isinstance(x, (int, float)) and not isinstance(x, bool) and x >= 0
                        for x in v.values()):
                    out.append(Finding(config.PROJECT_FILE, f"{where}.truth.{k}: needs "
                                       "{latency_s, noise_std}, each a number >= 0"))
    if c.get("workspace") is not None and not (isinstance(c["workspace"], str) and c["workspace"]):
        out.append(Finding(config.PROJECT_FILE, f"{where}.workspace: is not a setup file",
                           "workspace: \"{workspace:<name>}/install/setup.bash\""))
    if simulator and simulator != "isaac":
        out.append(Finding(config.PROJECT_FILE, f"{where}: a control component runs on Isaac only, "
                           f"the simulator is {simulator}", "drop component:, or simulator: isaac"))
    return out


def site_dirs(install: Path) -> list:
    """The Python package directories of a colcon install (isolated or merged), sorted."""
    install = Path(install)
    out = set()
    for pattern in ("*/lib/python3*/site-packages", "lib/python3*/site-packages",
                    "*/local/lib/python3*/dist-packages", "local/lib/python3*/dist-packages"):
        out.update(p for p in install.glob(pattern) if p.is_dir())
    return sorted(str(p) for p in out)


def paths(project, pilot: dict, host: dict) -> list:
    """Where the pilot's component is: the site-packages of its ``workspace`` (a setup file, as an overlay
    entry), else of every overlay's workspace, in order."""
    c = spec_of(pilot) or {}
    setups = [c["workspace"]] if isinstance(c.get("workspace"), str) else overlaysmod.entries(pilot)
    out = []
    for e in setups:
        r = overlaysmod.resolve(project, e, host)
        if r["path"]:
            for d in site_dirs(Path(r["path"]).parent):
                if d not in out:
                    out.append(d)
    return out


def lookup(name: str, dirs: list):
    """The entry point ``name`` among the packages in ``dirs``, from their metadata (nothing imported):
    ``{dist, version, value, file, sha256}``, ``file`` the module's source when found; None when no
    package there declares it."""
    from importlib import metadata
    for dist in metadata.distributions(path=[str(d) for d in dirs]):
        for ep in dist.entry_points:
            if ep.group != GROUP or ep.name != name:
                continue
            module = ep.value.split(":", 1)[0].strip()
            out = {"dist": dist.metadata["Name"], "version": dist.version, "value": ep.value,
                   "file": None, "sha256": None}
            rel = module.replace(".", "/")
            for d in dirs:
                for cand in (Path(d) / f"{rel}.py", Path(d) / rel / "__init__.py"):
                    if cand.is_file():
                        out["file"] = str(cand.resolve())
                        out["sha256"] = hashlib.sha256(cand.read_bytes()).hexdigest()
                        break
                if out["file"]:
                    break
            return out
    return None


def resolved(project, pilot: dict, host: dict):
    """What a stack records of the pilot's component, None without one: the entry point, its params and
    truth settings, the paths handed to the world server, and what was found there (dist, version, the
    module's file and sha256); the run's provenance carries it (J2's twin release)."""
    c = spec_of(pilot)
    if c is None:
        return None
    dirs = paths(project, pilot, host)
    name = str(c.get("entry_point") or "")
    found = None if name in SHIPPED else lookup(name, dirs)
    return {"entry_point": name, "params": dict(c.get("params") or {}),
            "truth": dict(c.get("truth") or {}), "paths": dirs, "found": found}


def ego_entry(recorded: dict) -> dict:
    """The ego config's ``component`` (aib's EgoConfig) from a stack's record of it."""
    out = {"entry_point": recorded["entry_point"], "paths": list(recorded.get("paths") or [])}
    if recorded.get("params"):
        out["params"] = dict(recorded["params"])
    if recorded.get("truth"):
        out["truth"] = dict(recorded["truth"])
    return out


def problems(project, settings: dict, host: dict) -> list:
    """Host findings: a component whose entry point no package in its workspace declares, refused."""
    out = []
    for name, pilot in (settings.get("pilots") or {}).items():
        c = spec_of(pilot if isinstance(pilot, dict) else None)
        if c is None or not isinstance(c.get("entry_point"), str) or c["entry_point"] in SHIPPED:
            continue
        if isinstance(c.get("workspace"), str):
            r = overlaysmod.resolve(project, c["workspace"], host)
            if r["problem"]:
                what, fix = r["problem"]
                out.append(Finding(config.PROJECT_FILE, f"pilots.{name}.component.workspace: {what}", fix,
                                   refused=True))
                continue
        dirs = paths(project, pilot, host)
        if lookup(c["entry_point"], dirs) is None:
            where = ", ".join(dirs) or "no Python package directory (no overlays or workspace)"
            out.append(Finding(config.PROJECT_FILE, f"pilots.{name}.component: no package declares "
                               f"{c['entry_point']} in entry point group {GROUP} ({where})",
                               "build the workspace that declares it, and name it in the pilot's overlays or "
                               "component.workspace", refused=True))
    return out


CHECK_SCRIPT = r"""
import json, sys, time
cfg = json.loads(sys.argv[1])
from aib.rig import EgoConfig
from sim_vehicle_ecu import Request
from sim_vehicle_ecu import component as components
ego = EgoConfig.load(cfg["ego"])
rig = ego.load_rig()
c = rig.make_component()
mean, worst = components.timing(c, Request(1.0, 5.0), None, cfg["dt"], cfg["steps"])
print(json.dumps({"commands": c.commands, "truth": list(getattr(c, "truth", ())), "mean_s": mean,
                  "worst_s": worst, "budget_s": components.budget(cfg["hz"])}))
"""


def isaac_check(isaac_python: str, aib_src: Path, ego_doc: dict, physics_hz: float, work: Path,
                steps: int = CHECK_STEPS) -> dict:
    """Load and time the component as the world server would, in Isaac's interpreter: the ego config
    ``ego_doc`` (with its ``component``) written to ``work``, the rig loaded, the component made and
    stepped ``steps`` times on a canned request. ``{ok, message, commands, truth, mean_s, worst_s,
    budget_s}``; ``ok`` False when it does not load."""
    import yaml
    work = Path(work)
    work.mkdir(parents=True, exist_ok=True)
    ego = work / "component_check_ego.yaml"
    ego.write_text(yaml.safe_dump(ego_doc, sort_keys=False))
    arg = json.dumps({"ego": str(ego), "dt": 1.0 / physics_hz, "hz": physics_hz, "steps": steps})
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME")}
    env["PYTHONPATH"] = str(aib_src)
    try:
        r = subprocess.run([isaac_python, "-c", CHECK_SCRIPT, arg], capture_output=True, text=True,
                           env=env, timeout=120)
    except (OSError, subprocess.TimeoutExpired) as e:
        return {"ok": False, "message": f"Isaac's Python did not run it: {e}"}
    if r.returncode:
        tail = (r.stderr or r.stdout).strip().splitlines()[-1:] or [""]
        return {"ok": False, "message": tail[0][:300]}
    out = json.loads(r.stdout.strip().splitlines()[-1])
    out.update(ok=True, message="")
    return out


def budget_note(name: str, result: dict) -> str:
    """The warning for a component over its per-call budget (60 / physics_hz ms), else ''."""
    if not result.get("ok") or result["mean_s"] <= result["budget_s"]:
        return ""
    return (f"pilots.{name}.component: {result['mean_s'] * 1e3:.3f} ms per step on average "
            f"(worst {result['worst_s'] * 1e3:.3f} ms), over the {result['budget_s'] * 1e3:.3f} ms budget at "
            "this physics rate: the run goes ahead, slower (the simulation is lockstep)")


_RESULTS = {}  # (project, pilot) -> isaac_check's result, for the notes after the check


def isaac_findings(project, settings: dict, host: dict) -> list:
    """Each pilot's component loaded and timed in Isaac's interpreter on the testbench's stock car (when the
    host has Isaac): refused when it does not load or takes commands its vehicle interface does not give
    (Autoware's, today); its timing kept for ``notes``. Nothing when this host has no Isaac or no component
    is found (``problems`` says so)."""
    from scensim import isaac as isaacmod
    from scensim import stack as stackmod
    root = isaacmod.root(host)
    python = Path(root) / "venv" / "bin" / "python" if root else None
    ws = stackmod.workspace_root()
    aib_src = ws / "src" / "autoware_isaac_bridge" / "src" / "aib" if ws else None
    out = []
    for name, pilot in (settings.get("pilots") or {}).items():
        c = spec_of(pilot if isinstance(pilot, dict) else None)
        if c is None or not isinstance(c.get("entry_point"), str):
            continue
        rec = resolved(project, pilot, host)
        if c["entry_point"] not in SHIPPED and rec["found"] is None:
            continue   # problems() refuses it
        if not (python and python.is_file() and aib_src and (aib_src / "aib").is_dir()):
            continue   # no Isaac on this host: checked where the stack runs
        try:
            doc = stackmod.isaac_ego_with_ground_truth()
        except stackmod.StackRefused:
            continue
        doc.pop("ground_truth", None)
        doc["component"] = ego_entry(rec)
        hz = float(((settings.get("isaac") or {}).get("physics_hz")) or 60.0)
        res = isaac_check(str(python), aib_src, doc, hz, Path(project) / "state")
        _RESULTS[(str(project), name)] = res
        if not res["ok"]:
            out.append(Finding(config.PROJECT_FILE, f"pilots.{name}.component: {c['entry_point']} does not "
                               f"load in Isaac's Python: {res['message']}", "it must be pure Python (the "
                               "standard library and numpy); a compiled model goes through the fmu component",
                               refused=True))
        elif res["commands"] != "autoware":
            out.append(Finding(config.PROJECT_FILE, f"pilots.{name}.component: it takes {res['commands']!r} "
                               "commands; the vehicle interface gives Autoware's (ackermann and vesc come with "
                               "roadmap 091 W18)", refused=True))
        else:
            undeclared = sorted(set(c.get("truth") or {}) - set(res["truth"]))
            if undeclared:
                out.append(Finding(config.PROJECT_FILE, f"pilots.{name}.component.truth: {', '.join(undeclared)}"
                                   f": not declared by the component (it declares "
                                   f"{', '.join(res['truth']) or 'none'})", "set latency and noise only for "
                                   "the inputs it declares", refused=True))
    return out


def notes(project, settings: dict) -> list:
    """Warnings: a component over its per-call budget (after ``isaac_findings``)."""
    out = []
    for name in (settings.get("pilots") or {}):
        res = _RESULTS.get((str(project), name))
        if res:
            note = budget_note(name, res)
            if note:
                out.append(note)
    return out
