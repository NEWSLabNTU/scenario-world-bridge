"""A run's result, ``runs/<id>/result.json``: the ``swb.result/1`` schema as code.

docs/design/projects-and-pilots.md, "Results". ``validate`` checks a decoded result by hand
(no jsonschema) and names every problem at once; ``verdict_from`` turns SSv2's JUnit and the
runner's infrastructure errors into the verdict; ``provenance`` records what ran.
"""
import dataclasses
import hashlib
import json
import socket
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

SCHEMA = "swb.result/1"

PASSED, FAILED, ERROR = "passed", "failed", "error"
VERDICTS = (PASSED, FAILED, ERROR)

COLLISION_VEHICLE = "collision.vehicle"
COLLISION_PEDESTRIAN = "collision.pedestrian"
COLLISION_STATIC = "collision.static"
RED_LIGHT = "red_light"
OFF_LANE = "off_lane"
STUCK = "stuck"
ROUTE_COMPLETE = "route_complete"
PILOT_RESTART = "pilot.restart"
STACK_ERROR = "stack.error"
EVENT_TYPES = (COLLISION_VEHICLE, COLLISION_PEDESTRIAN, COLLISION_STATIC, RED_LIGHT, OFF_LANE,
               STUCK, ROUTE_COMPLETE, PILOT_RESTART, STACK_ERROR)

# SSv2's JUnit (simple_junit, written by openscenario_interpreter's withExceptionHandler):
# <failure type="SimulationFailure"> comes only from CustomCommandAction::exitFailure, i.e.
# the scenario judged the run failed. Every <error> (SimulationError, AutowareError,
# SemanticError, SyntaxError, InternalError, UnknownError) means the run could not be judged:
# a simulator that did not answer, a bad scenario, a crash.


class ResultError(ValueError):
    """A result that does not follow ``swb.result/1``; ``problems`` lists every one."""

    def __init__(self, problems):
        self.problems = list(problems)
        super().__init__("not a valid swb.result/1: " + "; ".join(self.problems))


def _empty_scenario():
    return {"file": "", "sha256": None, "parameters": {}, "repetition": 0, "seed": None}


def _empty_ssv2():
    return {"junit": None, "failures": 0, "errors": 0, "message": ""}


@dataclasses.dataclass
class RunResult:
    run_id: str
    verdict: str
    reason: str = ""
    scenario: dict = dataclasses.field(default_factory=_empty_scenario)
    ssv2: dict = dataclasses.field(default_factory=_empty_ssv2)
    events: list = dataclasses.field(default_factory=list)
    metrics: dict = dataclasses.field(default_factory=dict)
    pilots: dict = dataclasses.field(default_factory=dict)
    provenance: dict = dataclasses.field(default_factory=dict)
    schema: str = SCHEMA

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        return {"schema": d.pop("schema"), **d}

    def to_json(self, indent=2) -> str:
        d = self.to_dict()
        validate(d)
        return json.dumps(d, indent=indent, sort_keys=False) + "\n"

    @classmethod
    def from_dict(cls, d: dict) -> "RunResult":
        validate(d)
        return cls(**d)

    @classmethod
    def from_json(cls, text: str) -> "RunResult":
        try:
            d = json.loads(text)
        except json.JSONDecodeError as e:
            raise ResultError([f"not JSON: {e}"]) from None
        return cls.from_dict(d)

    def write(self, path) -> Path:
        path = Path(path)
        tmp = path.with_name(path.name + ".tmp")
        tmp.write_text(self.to_json())
        tmp.replace(path)
        return path

    @classmethod
    def read(cls, path) -> "RunResult":
        path = Path(path)
        try:
            return cls.from_json(path.read_text())
        except ResultError as e:
            raise ResultError([f"{path}: {p}" for p in e.problems]) from None


# --- validation -----------------------------------------------------------------------------

_TOP = ("schema", "run_id", "verdict", "reason", "scenario", "ssv2", "events", "metrics",
        "pilots", "provenance")
_REQUIRED = ("schema", "run_id", "verdict")


def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def problems(d) -> list:
    """Every way ``d`` breaks ``swb.result/1``; empty when it is valid."""
    if not isinstance(d, dict):
        return [f"expected an object at the top, got {type(d).__name__}"]
    out = []
    for k in d:
        if k not in _TOP:
            out.append(f"unknown key {k!r}")
    for k in _REQUIRED:
        if k not in d:
            out.append(f"missing key {k!r}")
    if "schema" in d and d["schema"] != SCHEMA:
        out.append(f"schema is {d['schema']!r}, expected {SCHEMA!r}")
    if "run_id" in d and not (isinstance(d["run_id"], str) and d["run_id"]):
        out.append("run_id: expected a non-empty string")
    if "verdict" in d and d["verdict"] not in VERDICTS:
        out.append(f"verdict: {d['verdict']!r} is not one of {', '.join(VERDICTS)}")
    if "reason" in d and not isinstance(d["reason"], str):
        out.append("reason: expected a string")
    elif "\n" in d.get("reason", ""):
        out.append("reason: expected one line")

    def obj(key):
        v = d.get(key, {})
        if not isinstance(v, dict):
            out.append(f"{key}: expected an object")
            return {}
        return v

    sc = obj("scenario")
    if "file" in sc and not isinstance(sc["file"], str):
        out.append("scenario.file: expected a string")
    if sc.get("sha256") is not None and not isinstance(sc["sha256"], str):
        out.append("scenario.sha256: expected a string or null")
    if "parameters" in sc and not isinstance(sc["parameters"], dict):
        out.append("scenario.parameters: expected an object")
    if "repetition" in sc and not (_is_int(sc["repetition"]) and sc["repetition"] >= 0):
        out.append("scenario.repetition: expected an integer >= 0")
    if sc.get("seed") is not None and not _is_int(sc["seed"]):
        out.append("scenario.seed: expected an integer or null")

    ss = obj("ssv2")
    for k in ("failures", "errors"):
        if k in ss and not (_is_int(ss[k]) and ss[k] >= 0):
            out.append(f"ssv2.{k}: expected an integer >= 0")
    for k in ("junit", "message"):
        if ss.get(k) is not None and not isinstance(ss[k], str):
            out.append(f"ssv2.{k}: expected a string")

    ev = d.get("events", [])
    if not isinstance(ev, list):
        out.append("events: expected a list")
        ev = []
    for i, e in enumerate(ev):
        if not isinstance(e, dict):
            out.append(f"events[{i}]: expected an object")
            continue
        if e.get("type") not in EVENT_TYPES:
            out.append(f"events[{i}].type: {e.get('type')!r} is not one of "
                       + ", ".join(EVENT_TYPES))
        if e.get("t_sim") is not None and not _is_num(e["t_sim"]):
            out.append(f"events[{i}].t_sim: expected a number or null")
        if e.get("t_wall") is not None and not isinstance(e["t_wall"], (str, int, float)):
            out.append(f"events[{i}].t_wall: expected a string, a number or null")
        for k in ("entity", "other"):
            if e.get(k) is not None and not isinstance(e[k], str):
                out.append(f"events[{i}].{k}: expected a string")

    for k, v in obj("metrics").items():
        if v is not None and not _is_num(v):
            out.append(f"metrics.{k}: expected a number or null")

    for name, p in obj("pilots").items():
        if not isinstance(p, dict):
            out.append(f"pilots.{name}: expected an object")
        elif "kind" in p and not isinstance(p["kind"], str):
            out.append(f"pilots.{name}.kind: expected a string")

    obj("provenance")
    return out


def validate(d) -> dict:
    """``d`` when it is a valid ``swb.result/1``; else raises ResultError naming every problem."""
    p = problems(d)
    if p:
        raise ResultError(p)
    return d


# --- SSv2's JUnit and the verdict -----------------------------------------------------------

def read_junit(path) -> dict:
    """SSv2's ``result.junit.xml``: counts and every failure/error, in order.

    ``{"tests", "failures", "errors", "cases": [{"suite", "name", "failures": [(type, msg)],
    "errors": [(type, msg)]}]}``. Raises OSError / ET.ParseError on an unreadable file."""
    root = ET.parse(str(path)).getroot()
    suites = [root] if root.tag == "testsuite" else root.findall("testsuite")
    cases = []
    for s in suites:
        for c in s.findall("testcase"):
            cases.append({
                "suite": s.get("name", ""),
                "name": c.get("name", ""),
                "failures": [(f.get("type", ""), f.get("message") or (f.text or "").strip())
                             for f in c.findall("failure")],
                "errors": [(f.get("type", ""), f.get("message") or (f.text or "").strip())
                           for f in c.findall("error")],
            })
    fail = [x for c in cases for x in c["failures"]]
    err = [x for c in cases for x in c["errors"]]
    return {"tests": len(cases), "failures": len(fail), "errors": len(err), "cases": cases}


def _one_line(s: str, limit=300) -> str:
    s = " ".join(str(s).split())
    return s if len(s) <= limit else s[:limit - 3] + "..."


def verdict_from(junit_path, infra_errors=()) -> tuple:
    """``(verdict, reason)`` for a run.

    ``infra_errors``: what the runner saw go wrong on the stack's side (a world server or
    bridge error, a pilot never ready, SSv2 not starting); any makes the verdict ``error``,
    whatever SSv2 said. Otherwise SSv2's JUnit decides: any ``<error>`` is ``error``, else a
    ``<failure>`` (SSv2's SimulationFailure) is ``failed``, else a testcase is ``passed``.
    A missing, unreadable or empty JUnit is ``error``."""
    infra = [e for e in (infra_errors or ()) if e]
    if infra:
        return ERROR, _one_line(infra[0])
    if junit_path is None or not Path(junit_path).is_file():
        return ERROR, f"SSv2 wrote no JUnit ({junit_path})"
    try:
        j = read_junit(junit_path)
    except (OSError, ET.ParseError) as e:
        return ERROR, _one_line(f"SSv2's JUnit cannot be read: {e}")
    for c in j["cases"]:
        for t, m in c["errors"]:
            return ERROR, _one_line(f"{t or 'error'}: {m}")
    for c in j["cases"]:
        for t, m in c["failures"]:
            return FAILED, _one_line(f"{t or 'failure'}: {m}")
    if j["tests"] == 0:
        return ERROR, "SSv2's JUnit holds no testcase"
    return PASSED, ""


def ssv2_block(junit_path, rel_name="junit.xml") -> dict:
    """The result's ``ssv2`` object from a JUnit (zeros and no file when there is none)."""
    if junit_path is None or not Path(junit_path).is_file():
        return {"junit": None, "failures": 0, "errors": 0, "message": ""}
    try:
        j = read_junit(junit_path)
    except (OSError, ET.ParseError) as e:
        return {"junit": rel_name, "failures": 0, "errors": 0, "message": _one_line(str(e))}
    msgs = [f"{t}: {m}" for c in j["cases"] for t, m in c["errors"] + c["failures"]]
    return {"junit": rel_name, "failures": j["failures"], "errors": j["errors"],
            "message": _one_line(msgs[0]) if msgs else ""}


# --- provenance -----------------------------------------------------------------------------

def _out(argv, cwd=None, timeout=10):
    try:
        r = subprocess.run(argv, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.stdout if r.returncode == 0 else None


def submodule_pins(workspace) -> dict:
    """``{path: sha}`` from ``git submodule status``; ``sha+`` for a checkout off its pin,
    ``sha-`` for one not initialised. Empty when git or the repository is not there."""
    text = _out(["git", "submodule", "status"], cwd=workspace) or ""
    pins = {}
    for line in text.splitlines():
        if len(line) < 42:
            continue
        flag, rest = line[0], line[1:].split()
        if len(rest) < 2:
            continue
        sha, path = rest[0], rest[1]
        pins[path] = sha + ("+" if flag == "+" else "-" if flag == "-" else "")
    return pins


def gpu_names() -> list:
    text = _out(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"])
    return [ln.strip() for ln in (text or "").splitlines() if ln.strip()]


def file_sha256(path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return "sha256:" + h.hexdigest()


BUILD_STAMP = "swb_build.json"


def checkout_commit(workspace):
    """The checkout's commit, ``+dirty`` when its tracked tree has changes; None without git."""
    commit = (_out(["git", "rev-parse", "HEAD"], cwd=workspace) or "").strip() or None
    if commit and (_out(["git", "status", "--porcelain", "--untracked-files=no"],
                        cwd=workspace) or "").strip():
        commit += "+dirty"
    return commit


def stamp_build(workspace, install) -> Path:
    """Write ``<install>/swb_build.json``: the checkout's commit and submodule pins as they
    were when ``just build`` built this install (roadmap 077, G29)."""
    import datetime
    path = Path(install) / BUILD_STAMP
    path.write_text(json.dumps({
        "swb": checkout_commit(workspace), "submodules": submodule_pins(workspace),
        "built": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
    }, indent=2) + "\n")
    return path


def installed_build(prefix=None):
    """The stamp of the install this swb runs from (beside its ``swb`` package's prefix), or
    None: a plain ``colcon build`` writes none."""
    if prefix is None:
        try:
            from ament_index_python.packages import get_package_prefix
            prefix = get_package_prefix("swb")
        except Exception:
            return None
    for d in (Path(prefix), Path(prefix).parent):
        f = d / BUILD_STAMP
        if f.is_file():
            try:
                return json.loads(f.read_text())
            except (OSError, ValueError):
                return None
    return None


def provenance(workspace, project=None, build=None) -> dict:
    """What ran: the build (``build``, the install's stamp: its commit and submodule pins
    when it was built), else the workspace's commit (``+dirty`` when its tree has changes)
    and pins; the checkout's commit now when it differs from the build's; the host, the GPU
    and the project's swb.lock. Every part tolerates being absent."""
    commit = checkout_commit(workspace)
    lock = Path(project) / "swb.lock" if project is not None else None
    gpus = gpu_names()
    extra = {}
    if build and build.get("swb"):
        extra = {"built": build.get("built")}
        if commit != build["swb"]:
            extra["checkout"] = commit
    return {
        "swb": build["swb"] if build and build.get("swb") else commit,
        "submodules": (build.get("submodules") if build and build.get("swb") else None)
        or submodule_pins(workspace),
        **extra,
        "host": socket.gethostname(),
        "gpu": ", ".join(gpus) if gpus else None,
        "swb_lock": file_sha256(lock) if lock is not None and lock.is_file() else None,
    }


if __name__ == "__main__":
    import sys
    if sys.argv[1:2] == ["stamp"] and len(sys.argv) == 4:
        print(stamp_build(sys.argv[2], sys.argv[3]))
    else:
        sys.exit("usage: python3 -m swb.results stamp <workspace> <install>")
