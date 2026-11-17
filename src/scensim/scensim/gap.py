"""The gap trace's tools as judges: ``gap_oracle`` and ``incident_judge`` over a run directory.

A judge gets the run directory (scensim.judges); these two read a gap trace. With
``logging.gap_trace: true`` (or a judge that ``needs: [gap_trace]``) each run's trace is in
``runs/<id>/gap_trace/`` (scensim.episodes), and these adapters find it there::

    judges:
      - name: gap
        run: [python3, -m, scensim.gap, oracle]          # + gap_oracle's own options
        needs: [gap_trace]
      - name: incident
        run: [python3, -m, scensim.gap, incident, --incident, auto, --context, context.jsonl]
        needs: [gap_trace, context]

``oracle``: ``ros2 run scensim_launch gap_oracle`` on each trace of the run (a TIER IV scenario's
variants are several), with the run's JUnit for G4, findings beside each trace
(``gap_trace/<trace>.findings.json``). Verdict: ``failed`` when a trace's G4 class is a
disagreement (``PASS+contact``, ``PASS+overlap-only-in-CARLA``, ``FAIL(collision)+...``),
``passed`` otherwise (``agree``, ``unknown``); ``--fail-on never`` makes the findings data
only. ``incident``: ``incident_judge`` on the run's trace with ``--context`` (a log of SSv2's
storyboard states, relative to the run directory; ``needs: [context]`` records it, scensim.context)
and the run's JUnit; its PASS or FAIL is the verdict. ``--incident auto`` takes the number from
the run's scenario file name (``..._i<N>_...``, as the demo incidents are named), so one judge
serves a suite of them; without ``--stop-line`` the scenario's ``csbIncidentStopLine``
parameter gives it (incident 3's red light). No trace, no context log, or no incident to judge by is an ``error``
(exit 2), never a pass.
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

from scensim import episodes

DISAGREE = ("PASS+contact", "PASS+overlap-only-in-CARLA", "FAIL(collision)+no-contact",
            "FAIL(collision)+overlap-only-in-SSv2")


def tool(name: str) -> list:
    """argv prefix for a scensim_launch script: installed, else this workspace's source tree."""
    try:
        from ament_index_python.packages import get_package_prefix
        p = Path(get_package_prefix("scensim_launch")) / "lib" / "scensim_launch" / name
        if p.is_file():
            return [sys.executable, str(p)]
    except Exception:  # noqa: BLE001
        pass
    for up in Path(__file__).resolve().parents:
        p = up / "scensim_launch" / "scripts" / name
        if p.is_file():
            return [sys.executable, str(p)]
    raise FileNotFoundError(f"{name}: scensim_launch is neither installed nor beside scensim")


def traces(run_dir: Path) -> list:
    return sorted((Path(run_dir) / episodes.GAP_TRACE).glob("*.jsonl"))


def _junit(run_dir: Path):
    j = Path(run_dir) / "junit.xml"
    return j if j.is_file() else None


def _emit(doc: dict, code: int) -> int:
    print(json.dumps(doc))
    return code


def oracle(run_dir: Path, extra: list, fail_on: str) -> int:
    found = traces(run_dir)
    if not found:
        return _emit({"verdict": "error", "message": f"no gap trace in {run_dir}/gap_trace "
                      "(logging.gap_trace: true in swb.yaml, or needs: [gap_trace])"}, 2)
    junit = _junit(run_dir)
    out, worst = [], "passed"
    for t in found:
        findings = t.with_name(t.stem + ".findings.json")
        argv = tool("gap_oracle") + [str(t), "--out", str(findings)] + list(extra)
        if junit:
            argv += ["--junit", str(junit)]
        p = subprocess.run(argv, capture_output=True, text=True)
        if p.returncode != 0 or not findings.is_file():
            why = (p.stderr or p.stdout).strip().splitlines()[-1:] or [f"exit {p.returncode}"]
            return _emit({"verdict": "error", "message": f"gap_oracle on {t.name}: {why[0]}"}, 2)
        doc = json.loads(findings.read_text())
        g4 = (doc.get("g4") or {}).get("class")
        out.append({"trace": t.name, "g4": g4, "counts": doc.get("counts"),
                    "summary": doc.get("summary"), "findings": findings.name})
        if g4 in DISAGREE and fail_on == "g4":
            worst = "failed"
    bad = [o for o in out if o["g4"] in DISAGREE]
    msg = ("; ".join(f"{o['trace']}: {o['g4']}" for o in bad) if bad else
           f"{len(out)} trace(s), G4 " + ", ".join(sorted({str(o['g4']) for o in out})))
    return _emit({"verdict": worst, "message": msg, "traces": out}, 0 if worst == "passed" else 1)


INCIDENTS = range(1, 9)   # incident_judge's rows (roadmap 033; i7 and i8 since 077 item 10)
INCIDENT_NAME = re.compile(r"_i(\d+)_")


def incident_number(run_dir: Path, given: str):
    """``(number, None)`` from ``--incident`` (a number, or ``auto``: the run's scenario file
    name), else ``(None, why)``."""
    if given != "auto":
        try:
            n = int(given)
        except ValueError:
            return None, f"--incident {given}: a number or auto"
    else:
        try:
            import yaml
            sc = ((yaml.safe_load((Path(run_dir) / "resolved.yaml").read_text()) or {})
                  .get("scenario") or {}).get("file") or ""
        except (OSError, ValueError) as e:
            return None, f"--incident auto: no scenario for the run ({e})"
        m = INCIDENT_NAME.search(Path(sc).name)
        if not m:
            return None, f"--incident auto: {Path(sc).name} is not named ..._i<N>_..."
        n = int(m.group(1))
    if n not in INCIDENTS:
        return None, (f"incident {n}: incident_judge has no criteria for it (it judges "
                      f"{INCIDENTS.start} to {INCIDENTS.stop - 1})")
    return n, None


STOP_LINE_PARAM = "csbIncidentStopLine"


def scenario_stop_line(run_dir: Path):
    """``(x, y, yaw)`` from the run's scenario's ``csbIncidentStopLine`` parameter
    ("x,y,yaw", as gen_demo_incidents writes it for a site with a stop line), else None."""
    import xml.etree.ElementTree as ET
    import yaml
    try:
        sc = ((yaml.safe_load((Path(run_dir) / "resolved.yaml").read_text()) or {})
              .get("scenario") or {}).get("path")
        root = ET.parse(str(sc)).getroot()
    except (OSError, ValueError, TypeError, ET.ParseError):
        return None
    for p in root.iter("ParameterDeclaration"):
        if p.get("name") == STOP_LINE_PARAM:
            try:
                x, y, yaw = (float(v) for v in p.get("value", "").split(","))
            except ValueError:
                return None
            return x, y, yaw
    return None


CONTEXT_INCIDENT = re.compile(r"/i(\d+)_[^/]*$")


def context_incidents(ctx: Path) -> list:
    """The incidents (incident_judge's rows) whose events the storyboard log names."""
    out = set()
    try:
        for line in ctx.read_text().splitlines():
            try:
                m = CONTEXT_INCIDENT.search(json.loads(line).get("element") or "")
            except ValueError:
                continue
            if m and int(m.group(1)) in INCIDENTS:
                out.add(int(m.group(1)))
    except OSError:
        pass
    return sorted(out)


def incident(run_dir: Path, number, context: str, stop_line) -> int:
    """incident_judge on the run; ``--incident auto`` on a scenario whose name has no number
    (the demo tour) judges every incident its storyboard log names, passing when all pass."""
    ctx = Path(context)
    if not ctx.is_absolute():
        ctx = Path(run_dir) / ctx
    if str(number) == "auto":
        n, why = incident_number(run_dir, "auto")
        if n is None and "is not named" in (why or "") and ctx.is_file():
            several = context_incidents(ctx)
            if several:
                docs = [_judge_one(run_dir, k, ctx, stop_line, numbered=True) for k in several]
                bad = [d for d in docs if d["verdict"] != "passed"]
                verdict = "error" if any(d["verdict"] == "error" for d in docs) else \
                    "failed" if bad else "passed"
                msg = (f"{len(docs) - len(bad)} of {len(docs)} incidents passed"
                       + ("; " + "; ".join(f"i{d.get('incident')}: {d.get('message')}"
                                           for d in bad) if bad else ""))
                return _emit({"verdict": verdict, "message": msg, "incidents": docs},
                             {"passed": 0, "failed": 1}.get(verdict, 2))
    doc = _judge_one(run_dir, number, ctx, stop_line)
    return _emit(doc, {"passed": 0, "failed": 1}.get(doc["verdict"], 2))


def _judge_one(run_dir: Path, number, ctx: Path, stop_line, numbered=False) -> dict:
    """One incident's record: incident_judge's, with ``verdict`` passed, failed or error."""
    number, why = incident_number(run_dir, str(number))
    if why:
        return {"verdict": "error", "message": why}
    stop_line = stop_line or scenario_stop_line(run_dir)
    found = traces(run_dir)
    if len(found) != 1:
        return {"verdict": "error", "message": f"{len(found)} gap traces in "
                f"{run_dir}/gap_trace; incident_judge reads one"}
    if not ctx.is_file():
        return {"verdict": "error", "message": f"no context log {ctx} (SSv2's "
                "storyboard states, /simulation/context)"}
    out = Path(run_dir) / episodes.GAP_TRACE / (f"{found[0].stem}.incident{number}.json" if numbered
                                             else f"{found[0].stem}.incident.json")
    argv = tool("incident_judge") + [str(found[0]), "--incident", str(number), "--context",
                                     str(ctx), "--json", str(out)]
    junit = _junit(run_dir)
    if junit:
        argv += ["--junit", str(junit)]
    if stop_line:
        argv += ["--stop-line", *[str(x) for x in stop_line]]
    p = subprocess.run(argv, capture_output=True, text=True)
    if p.returncode not in (0, 1) or not out.is_file():
        why = (p.stderr or p.stdout).strip().splitlines()[-1:] or [f"exit {p.returncode}"]
        return {"verdict": "error", "message": f"incident_judge: {why[0]}"}
    doc = json.loads(out.read_text())
    verdict = "passed" if doc.get("verdict") == "PASS" else "failed"
    return {**doc, "verdict": verdict,
            "message": "; ".join(doc.get("reasons") or []) or doc.get("verdict", "")}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python3 -m scensim.gap", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    o = sub.add_parser("oracle", help="gap_oracle on the run's traces")
    o.add_argument("--fail-on", choices=("g4", "never"), default="g4")
    i = sub.add_parser("incident", help="incident_judge on the run's trace")
    i.add_argument("--incident", required=True, help="its number, or auto (the scenario's name)")
    i.add_argument("--context", required=True, help="the context log, relative to the run dir")
    i.add_argument("--stop-line", type=float, nargs=3, metavar=("X", "Y", "YAW"))
    args, rest = ap.parse_known_args(sys.argv[1:] if argv is None else argv)
    if not rest:
        ap.error("the run directory is the last argument")
    run_dir, extra = Path(rest[-1]), rest[:-1]
    if args.cmd == "oracle":
        return oracle(run_dir, extra, args.fail_on)
    if extra:
        ap.error(f"unknown arguments {' '.join(extra)}")
    return incident(run_dir, args.incident, args.context, args.stop_line)


if __name__ == "__main__":
    sys.exit(main())
