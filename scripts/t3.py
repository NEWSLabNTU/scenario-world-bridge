#!/usr/bin/env python3
"""T3, live regression (roadmap 077): the migrated studies' suites against their references.

    scripts/t3.py list                       # the manifest's suites, and which have a reference
    scripts/t3.py run [--only P/S ...]       # run them (ros2 run swb run, a stack per project)
    scripts/t3.py record [--only P/S ...]    # write each suite's latest results as its reference
    scripts/t3.py compare [--only P/S ...]   # this host's latest results against the references
    ... --part i/n                           # host i of n: its share, whole projects

The manifest is scenarios/t3.yaml. A reference is ``<references>/<project>/<suite>.csv``: one
row per run (scenario file name, repetition, seed) with its verdict and judges' verdicts,
plus the commit and host it was recorded on in ``<suite>.json``. ``compare`` fails on a run
whose verdict or a judge's verdict differs from the reference, or that is missing; a duration
more than 1.5 times the reference's is reported, not failed (hosts differ). Record only on the
release commit, so each reference names one commit.

A project marked ``copy: true`` (an example under src/swb_examples) runs from a copy made
with ``ros2 run swb project new --from``, under ``--work`` (default /var/tmp/$USER/t3), fetched
there; its results are under that copy's name.
"""
import argparse
import csv
import json
import os
import re
import socket
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "scenarios" / "t3.yaml"
SLOW = 1.5
STARTED = re.compile(r"^stack\s+starting on seat \d+ \(pid (\d+)")


def load(path=MANIFEST) -> dict:
    doc = yaml.safe_load(Path(path).read_text()) or {}
    doc.setdefault("references", "scenarios/t3-references")
    for e in doc.get("suites") or []:
        e["key"] = f"{project_key(e['project'])}/{e['suite']}"
    return doc


def project_key(project: str) -> str:
    """A project's name in keys and reference paths: its path under a projects directory."""
    for root in ("scenarios/projects/", "src/swb_examples/projects/"):
        if project.startswith(root):
            return project[len(root):]
    return Path(project).name


def selected(doc: dict, only, part=None) -> list:
    out = [e for e in doc["suites"] if not only or e["key"] in only]
    unknown = set(only or ()) - {e["key"] for e in doc["suites"]}
    if unknown:
        raise SystemExit(f"t3: not in the manifest: {', '.join(sorted(unknown))}")
    return split(out, part) if part else out


def split(entries: list, part: str) -> list:
    """Part i of n (``i/n``, 1-based) of the suites, for n hosts: whole projects (they share a
    stack), longest first onto the host with least so far, by the manifest's ``minutes:``
    (default 10). Every host computes the same split from the same manifest."""
    try:
        i, n = (int(x) for x in part.split("/"))
        assert 1 <= i <= n
    except (ValueError, AssertionError):
        raise SystemExit(f"t3: --part {part}: i/n with 1 <= i <= n")
    projects = {}
    for e in entries:
        projects.setdefault(e["project"], []).append(e)
    load = [0.0] * n
    owner = {}
    # a project the manifest gives a part (`part: k`, for this n) stays there, so adding a
    # suite does not move the others between hosts; the rest are balanced around them
    for project, es in projects.items():
        k = next((e["part"] for e in es if e.get("part") and e.get("parts", n) == n), None)
        if k:
            owner[project] = k - 1
            load[k - 1] += sum(e.get("minutes", 10) for e in es)
    for project, es in sorted(((p, es) for p, es in projects.items() if p not in owner),
                              key=lambda kv: (-sum(e.get("minutes", 10) for e in kv[1]), kv[0])):
        k = min(range(n), key=lambda j: (load[j], j))
        owner[project] = k
        load[k] += sum(e.get("minutes", 10) for e in es)
    return [e for e in entries if owner[e["project"]] == i - 1]


def work_dir(e: dict, work: Path) -> Path:
    """Where the suite runs: the project itself, or its copy for an example."""
    return work / f"t3_{Path(e['project']).name}" if e.get("copy") else ROOT / e["project"]


def summary_path(e: dict, logs: Path, work: Path) -> Path:
    return logs / work_dir(e, work).name / "suites" / e["suite"] / "summary.csv"


def read_summary(path: Path) -> list:
    """The runs of a suite's summary.csv, keyed for comparison."""
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append({"scenario": Path(r["scenario"]).name, "repetition": r["repetition"],
                         "seed": r.get("seed", ""), "params": r.get("params", ""),
                         "verdict": r["verdict"], "judges": r.get("judges", ""),
                         "duration": r.get("duration", ""), "reason": r.get("reason", "")})
    return latest(rows)


def latest(rows: list) -> list:
    """One row per run: a retried run's last try (summary.csv lists every try in order)."""
    seen = {}
    for r in rows:
        seen[key(r)] = r
    return list(seen.values())


def key(r: dict) -> tuple:
    return (r["scenario"], r["params"], r["repetition"], r["seed"])


def judges_of(text: str) -> dict:
    return dict(p.split("=", 1) for p in text.split() if "=" in p)


def compare_rows(ref: list, got: list) -> tuple:
    """(problems, notes): a changed or missing verdict is a problem; a slow run a note."""
    problems, notes = [], []
    have = {key(r): r for r in got}
    for r in ref:
        name = f"{r['scenario']} r{r['repetition']}" + (f" {r['params']}" if r["params"] else "")
        g = have.pop(key(r), None)
        if g is None:
            problems.append(f"{name}: no result (the reference has {r['verdict']})")
            continue
        if g["verdict"] != r["verdict"]:
            problems.append(f"{name}: {g['verdict']}, the reference {r['verdict']}"
                            + (f" ({g['reason']})" if g["reason"] else ""))
        rj, gj = judges_of(r["judges"]), judges_of(g["judges"])
        for j in sorted(set(rj) | set(gj)):
            if rj.get(j) != gj.get(j):
                problems.append(f"{name}: judge {j} {gj.get(j, 'absent')}, "
                                f"the reference {rj.get(j, 'absent')}")
        try:
            if float(g["duration"]) > SLOW * float(r["duration"]):
                notes.append(f"{name}: {float(g['duration']):.0f} s, the reference "
                             f"{float(r['duration']):.0f} s")
        except ValueError:
            pass
    for g in have.values():
        notes.append(f"{g['scenario']} r{g['repetition']}: not in the reference ({g['verdict']})")
    return problems, notes


def write_reference(rows: list, csv_path: Path, meta: dict):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["scenario", "params", "repetition", "seed", "verdict", "judges", "duration"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=key):
            w.writerow(r)
    csv_path.with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n")


def commit() -> str:
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                          capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--untracked-files=no"],
                           capture_output=True, text=True).stdout.strip()
    return head + ("+dirty" if dirty else "")


def gpu_state() -> list:
    """Each GPU's memory in use and load now, and the other users' processes on it: a
    reference recorded on a busy GPU (another user's training) may hold perception-lag
    failures, so the release's are recorded on a quiet one (roadmap 077)."""
    try:
        q = subprocess.run(["nvidia-smi", "--query-gpu=index,memory.used,memory.total,"
                            "utilization.gpu", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    out = []
    for line in q.strip().splitlines():
        i, used, total, util = (x.strip() for x in line.split(","))
        out.append({"gpu": int(i), "used_mib": int(used), "total_mib": int(total),
                    "util_pct": int(util)})
    return out


def busy(gpus: list, load: float = None, cores: int = None) -> bool:
    """A GPU at 50 % use or memory, or a host whose 1-minute load is at its core count
    (swb 139's ladder: other users' jobs at load 330-500 stalled SSv2 on a quiet GPU)."""
    if load is not None and cores and load >= cores:
        return True
    return any(g["util_pct"] >= 50 or g["used_mib"] > 0.5 * g["total_mib"] for g in gpus)


def host_load() -> tuple:
    """(1-minute load average, cores)."""
    return os.getloadavg()[0], os.cpu_count()


def logs_root() -> Path:
    sys.path.insert(0, str(ROOT / "src" / "swb"))
    from swb import config
    return Path(config.host_settings()["logs"]).expanduser()


def run(entries: list, work: Path, dry_run: bool) -> int:
    worst = 0
    by_project = {}
    for e in entries:
        by_project.setdefault(e["project"], []).append(e)
    for project, es in by_project.items():
        d = work_dir(es[0], work)
        steps = []
        if es[0].get("copy") and not (d / "swb.yaml").is_file():
            pkg = f"pkg://swb_examples/projects/{Path(project).name}"
            steps.append(["ros2", "run", "swb", "project", "new", str(d), "--from", pkg])
        # every project fetched first (nothing to download when it is; G35 skips the hashes)
        steps.append(["ros2", "run", "swb", "assets", "fetch"])
        for i, e in enumerate(es):
            argv = ["ros2", "run", "swb", "run", f"suites/{e['suite']}.yaml", "--start-stack",
                    "--ready-timeout", "1800"]
            if i == len(es) - 1:
                argv.append("--stop-stack")
            steps.append(argv)
        started = None
        for argv in steps:
            cwd = d if argv[3] != "project" else d.parent
            print(f"[t3] ({cwd}) {' '.join(argv)}", flush=True)
            if dry_run:
                continue
            cwd.mkdir(parents=True, exist_ok=True)
            p = subprocess.Popen(argv, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True)
            for line in p.stdout:
                sys.stdout.write(line)
                m = STARTED.match(line)
                if m:
                    started = int(m.group(1))
            worst = max(worst, p.wait())
        # --stop-stack stops only a stack that same command started: stop the one an earlier
        # suite of this project started, so each project leaves nothing up
        if started and not dry_run:
            stop_group(started)
    return worst


def stop_group(pid: int, grace: float = 60.0):
    import signal
    import time
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            return
        end = time.time() + grace
        while time.time() < end:
            try:
                os.killpg(pid, 0)
            except ProcessLookupError:
                return
            time.sleep(1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("action", choices=["list", "run", "record", "compare"])
    ap.add_argument("--only", nargs="*", default=[], help="<project>/<suite> keys")
    ap.add_argument("--part", help="i/n: this host's share of the suites, for n hosts (whole "
                                   "projects, balanced by minutes:); each host records or "
                                   "compares its own suites' references")
    ap.add_argument("--manifest", type=Path, default=MANIFEST)
    ap.add_argument("--logs", type=Path, help="the host's log root (default: its settings')")
    ap.add_argument("--work", type=Path, default=Path(f"/var/tmp/{os.environ.get('USER', 'swb')}/t3"))
    ap.add_argument("--dry-run", action="store_true", help="run: print the commands only")
    ap.add_argument("--busy-ok", action="store_true",
                    help="record although the GPU is busy (the reference says so)")
    a = ap.parse_args(argv)
    doc = load(a.manifest)
    entries = selected(doc, a.only, a.part)
    refs = ROOT / doc["references"]
    if a.action == "run":
        return run(entries, a.work, a.dry_run)
    logs = a.logs or logs_root()
    failed = 0
    for e in entries:
        ref = refs / f"{e['key']}.csv"
        summ = summary_path(e, logs, a.work)
        if a.action == "list":
            print(f"{e['key']:32} {e['sim']:6} reference: {'yes' if ref.is_file() else 'no'}"
                  f"  results: {'yes' if summ.is_file() else 'no'}")
            continue
        if not summ.is_file():
            print(f"{e['key']}: no results on this host ({summ})")
            failed += 1
            continue
        rows = read_summary(summ)
        if a.action == "record":
            gpus = gpu_state()
            hload, cores = host_load()
            if busy(gpus, hload, cores) and not a.busy_ok:
                print(f"{e['key']}: not recorded, the host is busy now (load {hload:.0f} on "
                      f"{cores} cores; GPU {gpus}); record on a quiet host, or give --busy-ok")
                failed += 1
                continue
            write_reference(rows, ref, {"commit": commit(), "host": socket.gethostname(),
                                        "summary": str(summ), "runs": len(rows),
                                        "gpu_at_record": gpus,
                                        "load_at_record": [round(hload, 1), cores]})
            print(f"{e['key']}: reference of {len(rows)} runs at {commit()}")
            continue
        if not ref.is_file():
            print(f"{e['key']}: no reference ({ref})")
            failed += 1
            continue
        with open(ref, newline="") as f:
            ref_rows = list(csv.DictReader(f))
        problems, notes = compare_rows(ref_rows, rows)
        print(f"{e['key']}: {'regressed' if problems else 'as the reference'} "
              f"({len(rows)} runs)")
        for p in problems:
            print(f"  problem: {p}")
        for n in notes:
            print(f"  note: {n}")
        failed += bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
