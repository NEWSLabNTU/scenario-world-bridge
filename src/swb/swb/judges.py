"""Judges: named, versioned programs run over a finished run's directory.

docs/design/projects-and-pilots.md, "Results". A project lists them in ``swb.yaml``::

    judges:
      - name: comfort                    # judges/comfort.json
        run: [python3, -m, my_judges.comfort]
        version: "1"                     # recorded with the output
        on: [passed, failed]             # verdicts it runs for (default: passed, failed)
        timeout: 120                     # s

A judge is given the run directory as its last argument, or where ``{run_dir}`` stands in
``run`` (``{scenario}``: the run's scenario file; either one means nothing is appended), and
as ``SWB_RUN_DIR``. It runs in the project directory and prints one JSON object on stdout, or
nothing in particular: then its exit code (0 passed, 1 failed) is its verdict and its last
line the message. swb writes the record, with the judge's name, version, exit code and
duration, to ``judges/<name>.json``. A ``verdict`` key in it
(``passed`` or ``failed``) is the judge's own verdict; it is recorded beside the run's, never in
place of it: scoring stays with the judges, and a third-party evaluator reads the same files.
A judge that exits other than 0 or 1, prints JSON that is not an object, or overruns records
``verdict: error`` with the reason; the run itself is unaffected.
"""
import json
import os
import subprocess
import time
from pathlib import Path

import yaml

from swb import results

DEFAULT_ON = (results.PASSED, results.FAILED)
DEFAULT_TIMEOUT = 120.0
KEYS = ("name", "run", "version", "on", "timeout")
JUDGE_VERDICTS = (results.PASSED, results.FAILED, results.ERROR)


def problems(judges) -> list:
    """Every way a project's ``judges:`` list is wrong, as ``(where, message)``."""
    if judges is None:
        return []
    if not isinstance(judges, list):
        return [("judges", "is not a list")]
    out, names = [], set()
    for i, j in enumerate(judges):
        w = f"judges[{i}]"
        if not isinstance(j, dict):
            out.append((w, "is not a mapping: {name: ..., run: [...]}"))
            continue
        for k in j:
            if k not in KEYS:
                out.append((w, f"unknown key {k} (use only {', '.join(KEYS)})"))
        name = j.get("name")
        if not (isinstance(name, str) and name and "/" not in name and not name.startswith(".")):
            out.append((w, "name: a plain file name is required"))
        elif name in names:
            out.append((w, f"name: {name} twice"))
        else:
            names.add(name)
        run = j.get("run")
        if not (isinstance(run, list) and run and all(isinstance(a, (str, int, float))
                                                      for a in run)):
            out.append((w, "run: a non-empty list of arguments is required"))
        on = j.get("on")
        bad_on = on is not None and (not isinstance(on, list)
                                     or any(v not in results.VERDICTS for v in on))
        if bad_on:
            out.append((w, f"on: a list of verdicts ({', '.join(results.VERDICTS)})"))
        t = j.get("timeout")
        if t is not None and (isinstance(t, bool) or not isinstance(t, (int, float)) or t <= 0):
            out.append((w, "timeout: a positive number of seconds"))
    return out


def _record(j, verdict, message="", code=None, started=None, data=None) -> dict:
    out = dict(data or {})
    out.update(name=j["name"], version=str(j.get("version") or ""), verdict=verdict,
               exit_code=code,
               duration_s=round(time.time() - started, 3) if started is not None else None)
    if message:
        out["message"] = message
    return out


def run_one(j: dict, run_dir: Path, project: Path, env=None) -> dict:
    """Run one judge over ``run_dir``; its record (also written to ``judges/<name>.json``)."""
    run_dir = Path(run_dir)
    sub = {"run_dir": str(run_dir), "scenario": _scenario_path(run_dir)}
    argv = [str(a) for a in j["run"]]
    if any("{run_dir}" in a or "{scenario}" in a for a in argv):
        argv = [_substitute(a, sub) for a in argv]
    else:
        argv.append(str(run_dir))
    e = dict(os.environ if env is None else env)
    e["SWB_RUN_DIR"] = str(run_dir)
    started = time.time()
    try:
        p = subprocess.run(argv, cwd=str(project), env=e, capture_output=True, text=True,
                           timeout=float(j.get("timeout") or DEFAULT_TIMEOUT))
    except subprocess.TimeoutExpired:
        limit = j.get("timeout") or DEFAULT_TIMEOUT
        rec = _record(j, results.ERROR, f"did not end within {limit} s", started=started)
    except OSError as ex:
        rec = _record(j, results.ERROR, f"did not start: {ex}", started=started)
    else:
        data, message = None, ""
        try:
            data = json.loads(p.stdout)
            if not isinstance(data, dict):
                data, message = None, "printed JSON that is not an object"
        except json.JSONDecodeError:
            pass
        if p.returncode not in (0, 1):
            tail = (p.stderr or "").strip().splitlines()[-1:] or [""]
            rec = _record(j, results.ERROR, f"exit {p.returncode}: {tail[0][:200]}".rstrip(": "),
                          p.returncode, started, data)
        elif data is None:
            # a plain program (no JSON): its exit code is its verdict, its last line the message
            last = (p.stdout or "").strip().splitlines()[-1:] or [""]
            v = results.PASSED if p.returncode == 0 else results.FAILED
            if message:
                v = results.ERROR
            rec = _record(j, v, message or last[0][:300], p.returncode, started)
        else:
            v = data.get("verdict")
            if v is None:
                v = results.PASSED if p.returncode == 0 else results.FAILED
            elif v not in JUDGE_VERDICTS:
                v, message = results.ERROR, f"verdict {data.get('verdict')!r} is not a verdict"
            rec = _record(j, v, message or data.get("message", ""), p.returncode, started, data)
        if p.stderr:
            log = run_dir / "logs" / "judge"
            log.mkdir(parents=True, exist_ok=True)
            (log / f"{j['name']}.log").write_text(p.stderr)
    out = run_dir / "judges"
    out.mkdir(exist_ok=True)
    (out / f"{j['name']}.json").write_text(json.dumps(rec, indent=2) + "\n")
    return rec


def _scenario_path(run_dir: Path) -> str:
    """The run's scenario file, from its ``resolved.yaml``; empty when there is none."""
    try:
        r = yaml.safe_load((Path(run_dir) / "resolved.yaml").read_text()) or {}
        return str((r.get("scenario") or {}).get("path") or "")
    except (OSError, ValueError, AttributeError):
        return ""


def _substitute(arg: str, sub: dict) -> str:
    for k, v in sub.items():
        arg = arg.replace("{" + k + "}", v)
    return arg


def run_all(judges, run_dir: Path, project: Path, verdict: str, env=None) -> list:
    """Every judge that runs for ``verdict``, in order; their records. Each also lands in the
    run's timeline as a ``judge`` event."""
    recs = []
    for j in judges or []:
        if verdict not in (j.get("on") or DEFAULT_ON):
            continue
        rec = run_one(j, run_dir, project, env)
        recs.append(rec)
        _timeline(run_dir, rec)
    return recs


def _timeline(run_dir: Path, rec: dict):
    tl = Path(run_dir) / "timeline.jsonl"
    ev = {"t_wall": time.time(), "t_sim": None, "group": "judge", "event": "judge.end",
          "judge": rec["name"], "verdict": rec["verdict"]}
    if rec.get("message"):
        ev["message"] = rec["message"]
    with tl.open("a") as f:
        f.write(json.dumps(ev) + "\n")


def summary(run_dir: Path) -> dict:
    """``{name: verdict}`` of the judges a run directory holds."""
    d = Path(run_dir) / "judges"
    out = {}
    if d.is_dir():
        for p in sorted(d.glob("*.json")):
            try:
                out[p.stem] = json.loads(p.read_text()).get("verdict")
            except (OSError, json.JSONDecodeError):
                out[p.stem] = results.ERROR
    return out
