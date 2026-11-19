#!/usr/bin/env python3
"""T3, live regression (roadmap 077): the migrated studies' suites against their references.

    scripts/t3.py list                       # the manifest's suites, and which have a reference
    scripts/t3.py run [--only P/S ...]       # run them (ros2 run scensim run, a stack per project)
    scripts/t3.py record [--only P/S ...]    # write each suite's latest results as its reference
    scripts/t3.py compare [--only P/S ...]   # this host's latest results against the references
    ... --part i/n                           # host i of n: its share, whole projects

The manifest is scenarios/t3.yaml. A reference is ``<references>/<project>/<suite>.csv``: one
row per run (scenario file name, repetition, seed) with its verdict and judges' verdicts,
plus in ``<suite>.json`` the commit the results ran on (their result.json's provenance: one
commit, its submodule pins; a dirty one is noted), t3.py's own commit (``recorded_with``), the
host, and the GPUs and load at record time (``gpu_gated``: the stack's card(s) the busy check
judged, from host ``gpu``, the seat's ``seats.gpus``, CUDA_VISIBLE_DEVICES, else all).
``compare`` fails on a run whose verdict or a judge's verdict differs from the reference, or
that is missing; a duration more than 1.5 times the reference's is reported, not failed (hosts
differ). Record only on the release commit, so each reference names one commit. The busy check
and ``gpu_at_record`` are the host's state when ``record`` runs, not while the suite ran.

A project marked ``copy: true`` (an example under src/scensim_examples) runs from a copy made
with ``ros2 run scensim project new --from``, under ``--work`` (default /var/tmp/$USER/t3), fetched
there; its results are under that copy's name.

``run`` writes each step's output to ``<work>/logs/<project>/<time>_<n>_<step>.log`` and
follows it to its own stdout, so a t3 that dies cannot break its child's writes (077 G80).
SIGINT (even inherited ignored, a script's ``&`` job) and SIGTERM stop the step's group (each
step runs in its own session) and any stack the project's suites started, then exit 130 (SIGINT)
or 143 (SIGTERM); another SIGINT or SIGTERM during that stop moves it to its next stage (SIGTERM,
then SIGKILL) at once, and never abandons it. Stop t3 with SIGINT or SIGTERM, never SIGKILL: the
step and its stack are in a session of their own, and only t3's kill path stops them.

A project's suites share its stack, in the manifest's order; before a suite the stack cannot
serve (scensim's ``switch_problem`` for the suite's maps: an Isaac world holds only the worlds it
loaded at start) t3 stops it, keeper first, and the suite starts a new one (077 G82). ``run``
stamps each suite (``<work>/stamps/``: its start time); ``record`` takes only rows of that
run, and refuses a suite this host never ran with this t3 (one run with an older t3 is
recorded with that t3). ``compare`` needs no stamp.
"""
import argparse
import csv
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "scenarios" / "t3.yaml"
SLOW = 1.5
STOP_GRACE = 180.0   # s for an interrupted scensim run to stop its stack (CARLA, Isaac) before SIGTERM
# scensim run's line for a stack it starts: "stack    <name> on seat 0: starting (pid 1, ...",
# and before 081 "stack    starting on seat 0 (pid 1, ..." (seat, pid); a line t3 does not
# match leaves the project's stack up after its last suite, holding CARLA's world-server lock
STARTED = re.compile(r"^stack\s+(?:\S+\s+)?on seat (\d+):? (?:starting )?\(pid (\d+)")


def load(path=MANIFEST) -> dict:
    doc = yaml.safe_load(Path(path).read_text()) or {}
    doc.setdefault("references", "scenarios/t3-references")
    for e in doc.get("suites") or []:
        e["key"] = f"{project_key(e['project'])}/{e['suite']}"
    return doc


def project_key(project: str) -> str:
    """A project's name in keys and reference paths: its path under a projects directory."""
    for root in ("scenarios/projects/", "src/scensim_examples/projects/"):
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
            rows.append({"run_id": r.get("run_id", ""),
                         "scenario": Path(r["scenario"]).name, "repetition": r["repetition"],
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
    dirty = subprocess.run(["git", "--no-optional-locks", "-C", str(ROOT), "status", "--porcelain",
                            "--untracked-files=no"],
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


def busy(gpus: list, load: float = None, cores: int = None, gated=None) -> bool:
    """A gated GPU at 50 % use or memory, or a host whose 1-minute load is at its core count
    (swb 139's ladder: other users' jobs at load 330-500 stalled SSv2 on a quiet GPU). The load
    is the host's, always; ``gated``: the GPU indices the stack uses (``stack_gpus``), None for
    every card (118: GPU 1 another user's, the stack on GPU 0, refused for ever)."""
    if load is not None and cores and load >= cores:
        return True
    return any(g["util_pct"] >= 50 or g["used_mib"] > 0.5 * g["total_mib"] for g in gpus
               if gated is None or g["gpu"] in gated)


class GpuRefused(ValueError):
    """CUDA_VISIBLE_DEVICES names a GPU nvidia-smi does not list."""


def gpu_uuids() -> dict:
    """nvidia-smi's uuid -> index, for a CUDA_VISIBLE_DEVICES given as UUIDs."""
    try:
        q = subprocess.run(["nvidia-smi", "--query-gpu=index,uuid", "--format=csv,noheader"],
                           capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return {}
    out = {}
    for line in q.strip().splitlines():
        i, u = (x.strip() for x in line.split(","))
        out[u] = int(i)
    return out


def stack_gpus(host: dict, seat_arg=None, environ=None, uuids=gpu_uuids, resolve=None):
    """``(indices or None, source)``: the GPU(s) the stack uses, as scensim takes them: host
    ``gpu``, else the seat's ``seats.gpus`` entry (``--seat`` or host ``seat``; ``auto``
    resolved as `scensim run` resolves it, the first free seat whose ports are free), else
    CUDA_VISIBLE_DEVICES (indices, or GPU UUIDs mapped to nvidia-smi's indices; one it does
    not list is refused), else None: every card."""
    environ = os.environ if environ is None else environ
    if host.get("gpu") is not None:
        return [int(host["gpu"])], f"host gpu: {host['gpu']}"
    gpus = (host.get("seats") or {}).get("gpus")
    if gpus:
        choice = seat_arg if seat_arg is not None else host.get("seat", "auto")
        if str(choice) == "auto":
            n = resolve(host) if resolve else _auto_seat(host)
            if n is None:
                return (sorted({int(g) for g in gpus}),
                        "seats.gpus (seat auto: none free; every seat's)")
            return [int(gpus[n])], f"seats.gpus[{n}] (seat auto)"
        n = int(choice)
        if not 0 <= n < len(gpus):
            raise GpuRefused(f"seat {n}: seats.gpus lists {len(gpus)} seats ({gpus})")
        return [int(gpus[n])], f"seats.gpus[{n}] (seat {n})"
    cvd = environ.get("CUDA_VISIBLE_DEVICES", "").strip()
    if cvd:
        out, table = [], None
        for d in (x.strip() for x in cvd.split(",") if x.strip()):
            if d.isdigit():
                # CUDA's order is FASTEST_FIRST unless CUDA_DEVICE_ORDER=PCI_BUS_ID, not
                # always nvidia-smi's: the same on one card or identical cards (scensim assumes it)
                out.append(int(d))
                continue
            table = uuids() if table is None else table
            hit = next((i for u, i in table.items() if u == d or u.startswith(d)), None)
            if hit is None:
                raise GpuRefused(f"CUDA_VISIBLE_DEVICES={cvd}: {d} is not a GPU nvidia-smi "
                                 f"lists ({', '.join(table) or 'none'})")
            out.append(hit)
        return out, f"CUDA_VISIBLE_DEVICES={cvd}"
    return None, "every GPU (no gpu, seats.gpus or CUDA_VISIBLE_DEVICES)"


def _auto_seat(host: dict):
    """The seat `scensim run` would take now (seat.resolve with its port check), else None."""
    sys.path.insert(0, str(ROOT / "src" / "scensim"))
    from scensim import config, seat as seatmod
    try:
        return seatmod.resolve(dict(host, seat="auto"),
                               usable=lambda s: not seatmod.busy_ports(s)).index
    except (config.ConfigError, seatmod.SeatTaken):
        return None


def results_provenance(rows: list, runs: Path) -> dict:
    """What the results were run on, from each run's result.json (``runs/<run_id>``): its
    ``provenance.swb`` commit and ``submodules``. ``commits``: every distinct one; a part's
    reference names one, so a mix is refused. ``swb`` is the commit the install was built at;
    when the checkout differed (``provenance.checkout``: under --symlink-install the Python ran
    from it, the compiled parts from the build), both are named: ``<built> (checkout <c>)``."""
    commits, subs, missing = {}, {}, []
    for r in rows:
        try:
            p = json.loads((runs / r["run_id"] / "result.json").read_text())["provenance"]
        except (OSError, ValueError, KeyError):
            missing.append(r["run_id"] or r["scenario"])
            continue
        c = str(p.get("swb") or "")
        if p.get("checkout"):
            c += f" (checkout {p['checkout']})"
        commits.setdefault(c, []).append(r["run_id"])
        subs[c] = p.get("submodules") or {}
    return {"commits": commits, "submodules": subs, "missing": missing}


def off_pin(submodules: dict) -> str:
    """The submodules a result's provenance shows off its commit's pins: scensim's
    results.submodule_pins writes ``sha+`` for a checkout off its pin (a stale one after a
    pull: the binaries are not the commit's code) and ``sha-`` for one not initialised.
    "" when none is."""
    return ", ".join(f"{p} ({s})" for p, s in sorted(submodules.items())
                     if str(s).endswith(("+", "-")))


def results_commit(prov: dict):
    """``(commit, problem)``: the one commit the results name, or a problem."""
    if prov["missing"]:
        return None, f"no result.json for {', '.join(prov['missing'][:3])}"
    if len(prov["commits"]) != 1:
        return None, ("results from more than one commit: " +
                      "; ".join(f"{c or '?'} ({len(ids)} runs)"
                                for c, ids in prov["commits"].items()))
    return next(iter(prov["commits"])), None


def host_load() -> tuple:
    """(1-minute load average, cores)."""
    return os.getloadavg()[0], os.cpu_count()


def host_settings() -> dict:
    sys.path.insert(0, str(ROOT / "src" / "scensim"))
    from scensim import config
    return config.host_settings()


def logs_root() -> Path:
    return Path(host_settings()["logs"]).expanduser()


def run(entries: list, work: Path, dry_run: bool, logs: Path = None) -> int:
    worst = 0
    by_project = {}
    for e in entries:
        by_project.setdefault(e["project"], []).append(e)
    for project, es in by_project.items():
        d = work_dir(es[0], work)
        steps = []
        if es[0].get("copy") and not (d / "testbench.yaml").is_file():
            pkg = f"pkg://scensim_examples/projects/{Path(project).name}"
            steps.append((["ros2", "run", "scensim", "project", "new", str(d), "--from", pkg], None))
        # every project fetched first (nothing to download when it is; G35 skips the hashes)
        steps.append((["ros2", "run", "scensim", "assets", "fetch"], None))
        for i, e in enumerate(es):
            argv = ["ros2", "run", "scensim", "run", f"suites/{e['suite']}.yaml", "--start-stack",
                    "--ready-timeout", "1800"]
            if i == len(es) - 1:
                argv.append("--stop-stack")
            steps.append((argv, e))
        started = None    # (pid, seat) of the stack a suite of this project started
        p = None
        try:
            for n, (argv, e) in enumerate(steps):
                cwd = d if argv[3] != "project" else d.parent
                if e is not None and started and not dry_run:
                    # the manifest's order is kept: a suite the project's stack cannot serve
                    # (an Isaac world holds only the worlds it loaded at start) gets a new one
                    why = restart_reason(d, stack_record(logs, d, started[1]), e["suite"])
                    if why:
                        echo(f"[t3] {why}: restarting the stack\n")
                        stop_stack(started, logs, d)
                        started = None
                echo(f"[t3] ({cwd}) {' '.join(argv)}\n")
                if dry_run:
                    continue
                cwd.mkdir(parents=True, exist_ok=True)
                log = step_log(work, d, n, argv)
                if e is not None:
                    write_stamp(work, e)
                p = start_step(argv, cwd, log)
                code, got = watch_step(p, log)
                started = got or started
                worst = max(worst, code)
                p = None
        except KeyboardInterrupt:
            # the step's own group first (scensim run stops what it started), then a stack an
            # earlier suite of this project left up; t3 exits only once both are gone. A
            # second Ctrl-C hurries the stop on, never abandons it
            global _hurry
            _hurry = False
            signal.signal(signal.SIGINT, _on_hurry)
            signal.signal(signal.SIGTERM, _on_hurry)
            warn("t3: interrupted; stopping the suite's run and its stack "
                 "(again: hurry it on to SIGTERM)\n")
            if p is not None:
                stop_group(p.pid, grace=STOP_GRACE, proc=p)
                started = scan_started(log) or started
            if started:
                # normally a no-op (ESRCH): the step's scensim run stopped the stack it started
                # itself; it is not when an earlier suite of the project left one up
                stop_stack(started, logs, d)
            raise
        # --stop-stack stops only a stack that same command started: stop the one an earlier
        # suite of this project started, so each project leaves nothing up
        if started and not dry_run:
            stop_stack(started, logs, d, grace=60.0)
    return worst


def stack_record(logs, d: Path, seat: int):
    """The stack.json of the project's stack on ``seat`` (scensim's ``seat-<n>-current`` link
    under the host's logs), or None."""
    if logs is None:
        return None
    try:
        return json.loads((Path(logs) / d.resolve().name / f"seat-{seat}-current" /
                           "stack.json").read_text())
    except (OSError, ValueError):
        return None


def suite_maps(d: Path, suite: str) -> list:
    """The maps a suite's runs are on, in order, by scensim's own resolver (``scensim run``'s
    ``expand`` and ``scenario_map``); a scenario whose map it cannot name is left out."""
    from scensim import run as runmod
    specs, _ = runmod.expand(d / "suites" / f"{suite}.yaml", d)
    return [m for m in dict.fromkeys(runmod.scenario_map(sp.scenario) for sp in specs) if m]


def restart_reason(d: Path, rec, suite: str):
    """Why the stack of ``rec`` cannot serve ``suite`` (a ``[t3]`` line: ``map X -> Y``), or
    None: scensim's own ``switch_problem`` for each of the suite's maps (077 G82: on Isaac it
    refuses even the stack's own first world once the pilots moved off it)."""
    if not rec:
        return None
    from scensim import stack as stackmod
    try:
        maps = suite_maps(d, suite)
    except Exception as err:   # scensim run refuses such a suite in its own words
        warn(f"t3: {suite}: its maps are not known ({err}); the stack is kept\n")
        return None
    for m in maps:
        if stackmod.switch_problem(rec, m):
            return f"map {rec.get('map')} -> {m}"
    return None


def stop_stack(started: tuple, logs, d: Path, grace: float = STOP_GRACE):
    """A stack a suite started, as ``scensim run --stop-stack`` stops one: its keeper first (it
    stops the pilots and the simulator in order; it runs in a session of its own), then the
    launch's process group; returns once both are gone."""
    pid, seat = started
    rec = stack_record(logs, d, seat)
    # the seat's record is this stack's only while its keeper descends from this launch: by
    # the project's end the seat may hold another's stack, which is not t3's to stop
    if rec and rec.get("pid") and descends(rec["pid"], pid):
        from scensim import stack as stackmod
        try:
            kgroup = os.getpgid(rec["pid"])
        except ProcessLookupError:
            kgroup = None
        stackmod.stop_keeper(rec)
        # its answer (stack.stop) comes before its nested launches are gone, and they are in
        # groups of their own: wait until no process names the stack's dir (their --log-dir;
        # their nodes die with them), so the next stack meets no port or lock of this one's
        stack_dir = str(Path(rec["proc"]).parent) if rec.get("proc") else None
        if stack_dir and not wait_gone(stack_dir, grace) and kgroup:
            stop_group(kgroup, grace=grace)
    stop_group(pid, grace=grace)


def wait_gone(mark: str, timeout: float) -> bool:
    """True once no process's command line contains ``mark``, within ``timeout`` s."""
    end = time.time() + timeout
    while True:
        if not procs_naming(mark):
            return True
        if time.time() >= end:
            return False
        time.sleep(0.5)


def procs_naming(mark: str) -> list:
    """The pids (other than t3's own) whose command line contains ``mark``."""
    out = []
    me = str(os.getpid())
    for d in os.listdir("/proc"):
        if not d.isdigit() or d == me:
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as f:
                if mark.encode() in f.read():
                    out.append(int(d))
        except OSError:
            pass
    return out


def descends(pid: int, ancestor: int) -> bool:
    """``pid`` is ``ancestor`` or below it, by /proc's parent links."""
    seen = 0
    while pid and pid > 1 and seen < 64:
        if pid == ancestor:
            return True
        try:
            with open(f"/proc/{pid}/stat") as f:
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            return False
        seen += 1
    return False


def host_relative(path: Path, logs: Path) -> str:
    """``path`` under the host's log root as ``<host logs>/...``: a reference goes into the
    repository and names no host's disk or user (the RC references' form)."""
    try:   # both resolved: the root may be reached through a link
        return "<host logs>/" + str(Path(path).resolve().relative_to(Path(logs).resolve()))
    except ValueError:   # <project>/suites/<suite>/summary.csv, as under the root
        return "<host logs>/" + "/".join(Path(path).parts[-4:])


def stamp_path(work: Path, e: dict) -> Path:
    return work / "stamps" / (e["key"].replace("/", "__") + ".json")


def write_stamp(work: Path, e: dict):
    """When this host's ``t3 run`` started the suite: ``record`` takes only results from that
    run (077 G82: a stale summary.csv from an earlier commit). No commit: t3's checkout need
    not be the scensim that runs (the RC's runs, a build at the tag, are recorded with a newer
    t3); the rows' own provenance names it, and ``results_commit`` refuses a mix."""
    from scensim import stack as stackmod   # its run_id prefix, local time: the two must match
    path = stamp_path(work, e)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"key": e["key"], "started": stackmod.now_id()}) + "\n")


def stale_rows(rows: list, stamp) -> list:
    """The rows not from the suite's run that ``stamp`` names: started before it, by their
    run_id's time (scensim's ``now_id``, as the stamp's)."""
    return [f"{r['run_id'] or r['scenario']} (before this run, {stamp['started']})"
            for r in rows if (r["run_id"] or "")[:19] < stamp["started"]]


def step_log(work: Path, d: Path, n: int, argv: list) -> Path:
    """A step's own output file: the child writes there, not to a pipe of t3's, so t3's death
    cannot break its writes (a scensim run whose stdout pipe lost its reader died on EPIPE while
    starting its stack and left the stack up; 077 G80)."""
    stamp = time.strftime("%Y-%m-%dT%H-%M-%S")
    what = argv[3] if argv[3] != "run" else Path(argv[4]).stem
    log = work / "logs" / d.name / f"{stamp}_{n}_{what}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    return log


def start_step(argv: list, cwd: Path, log: Path):
    """``argv`` in its own session (t3's kill path signals its group), output to ``log``."""
    with open(log, "wb") as fh:
        return subprocess.Popen(argv, cwd=cwd, stdout=fh, stderr=subprocess.STDOUT,
                                start_new_session=True)


def watch_step(p, log: Path, poll: float = 0.2) -> tuple:
    """``p``'s output followed from ``log`` to t3's stdout until it exits: (exit code,
    ``(pid, seat)`` of a stack it started, or None)."""
    started = None
    for line in follow(p, log, poll):
        echo(line)
        m = STARTED.match(line)
        if m:
            started = int(m.group(2)), int(m.group(1))
    return p.wait(), started


def follow(p, log: Path, poll: float = 0.2):
    """``log``'s lines as ``p`` writes them, until it has exited and all are read."""
    buf = b""
    with open(log, "rb") as rd:
        while True:
            done = p.poll() is not None
            chunk = rd.read()
            if chunk:
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for x in lines:
                    yield x.decode(errors="replace") + "\n"
            elif done:
                if buf:
                    yield buf.decode(errors="replace")
                return
            else:
                time.sleep(poll)


def scan_started(log: Path):
    """``(pid, seat)`` of the stack a step's log says it started, or None."""
    try:
        for line in log.read_text(errors="replace").splitlines():
            m = STARTED.match(line)
            if m:
                return int(m.group(2)), int(m.group(1))
    except OSError:
        pass
    return None


def _write(stream: str, line: str):
    """``line`` on sys.stdout or sys.stderr; once that is gone (EPIPE: ``| tee`` or ``| head``
    killed), it becomes devnull, so neither a later write nor Python's flush at exit raises and
    no stop is abandoned for a message (077 G80, in t3)."""
    f = getattr(sys, stream)
    try:
        f.write(line)
        f.flush()
    except (BrokenPipeError, OSError):
        setattr(sys, stream, open(os.devnull, "w"))


def echo(line: str):
    """A child's line on t3's stdout; once that is gone, only the step's log has it."""
    _write("stdout", line)


def warn(line: str):
    _write("stderr", line)


_hurry = False


def _on_hurry(signum, frame):
    """SIGINT or SIGTERM during the kill path: the stop's next stage now."""
    global _hurry
    _hurry = True


def stop_group(pid: int, grace: float = 60.0, proc=None):
    """``pid``'s process group: SIGINT, then SIGTERM, then SIGKILL, ``grace`` s apart (or at
    once after a hurrying signal, ``_on_hurry``), until it is gone. ``proc``: t3's own child
    leading it, reaped as it goes (a zombie still counts as a member of its group)."""
    global _hurry
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            return
        end = time.time() + grace
        while time.time() < end:
            if _hurry:
                _hurry = False
                break
            if proc is not None:
                proc.poll()
            try:
                os.killpg(pid, 0)
            except ProcessLookupError:
                return
            time.sleep(1)


class Terminated(KeyboardInterrupt):
    """SIGTERM, as an interrupt: ``run``'s kill path stops the step and its stack first."""


def _on_term(signum, frame):
    raise Terminated()


def take_signals():
    """SIGINT and SIGTERM stop t3 through its kill path, even with SIGINT inherited ignored (a
    script's ``&`` job; scensim.signals, dc350cd): first, before any child starts."""
    src = str(ROOT / "src" / "scensim")
    if src not in sys.path:
        sys.path.append(src)   # last: the sourced install's scensim, when there is one, comes first
    from scensim import signals
    return signals.take_sigint(), signal.signal(signal.SIGTERM, _on_term)


def main(argv=None) -> int:
    previous = take_signals()
    try:
        return _main(argv)
    finally:
        # what was replaced, only: signal.signal returns None for a handler not set from Python
        for sig, h in zip((signal.SIGINT, signal.SIGTERM), previous):
            if h is not None:
                signal.signal(sig, h)


def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("action", choices=["list", "run", "record", "compare"])
    ap.add_argument("--only", nargs="*", default=[], help="<project>/<suite> keys")
    ap.add_argument("--part", help="i/n: this host's share of the suites, for n hosts (whole "
                                   "projects, balanced by minutes:); each host records or "
                                   "compares its own suites' references")
    ap.add_argument("--manifest", type=Path, default=MANIFEST)
    ap.add_argument("--logs", type=Path, help="the host's log root (default: its settings')")
    ap.add_argument("--work", type=Path, default=Path(f"/var/tmp/{os.environ.get('USER', 'scensim')}/t3"))
    ap.add_argument("--dry-run", action="store_true", help="run: print the commands only")
    ap.add_argument("--busy-ok", action="store_true",
                    help="record although the GPU is busy (the reference says so)")
    ap.add_argument("--seat", help="record: the seat whose GPU is gated (default: the host's)")
    a = ap.parse_args(argv)
    doc = load(a.manifest)
    entries = selected(doc, a.only, a.part)
    refs = ROOT / doc["references"]
    if a.action == "run":
        try:
            return run(entries, a.work, a.dry_run, a.logs or logs_root())
        except KeyboardInterrupt as e:
            warn("t3: interrupted; what this run started is stopped\n")
            return 143 if isinstance(e, Terminated) else 130
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
        prov = results_provenance(rows, summ.parents[2] / "runs")
        at, why = results_commit(prov)
        if a.action == "record":
            if why:
                print(f"{e['key']}: not recorded, {why}")
                failed += 1
                continue
            try:
                stamp = json.loads(stamp_path(a.work, e).read_text())
            except (OSError, ValueError):
                print(f"{e['key']}: not recorded, no `t3.py run` of it on this host "
                      f"({stamp_path(a.work, e)}); run it, then record")
                failed += 1
                continue
            stale = stale_rows(rows, stamp)
            if stale:
                print(f"{e['key']}: not recorded, results not from this host's last `t3.py "
                      f"run` of it: {'; '.join(stale[:3])}"
                      f"{f' (+{len(stale) - 3} more)' if len(stale) > 3 else ''}; run it again")
                failed += 1
                continue
            off = off_pin(prov["submodules"][at])
            if off:
                print(f"{e['key']}: not recorded, submodules off the commit's pins: {off}; "
                      "git submodule update --init --recursive, rebuild, run again")
                failed += 1
                continue
            notes = []
            if "+dirty" in at:
                # provenance holds only the flag, not the files: `just build` dirties the
                # tracked Cargo.lock ([[patch.unused]]) on every fresh checkout (G79), so for
                # this RC a dirty commit is recorded with a note, not refused
                notes.append("results' checkout was dirty (provenance names no files; "
                             "`just build` changes Cargo.lock, G79)")
                print(f"{e['key']}: warning: {notes[-1]}")
            try:
                gated, source = stack_gpus(host_settings(), a.seat)
            except GpuRefused as err:
                print(f"{e['key']}: not recorded, {err}")
                failed += 1
                continue
            gpus = gpu_state()
            hload, cores = host_load()
            if busy(gpus, hload, cores, gated) and not a.busy_ok:
                cards = [g for g in gpus if gated is None or g["gpu"] in gated]
                print(f"{e['key']}: not recorded, the host is busy now (load {hload:.0f} on "
                      f"{cores} cores; gated GPU {gated if gated is not None else 'all'} "
                      f"from {source}: {cards}); record on a quiet host, or give --busy-ok")
                failed += 1
                continue
            write_reference(rows, ref, {"commit": at, "submodules": prov["submodules"][at],
                                        "recorded_with": commit(),
                                        "host": socket.gethostname(),
                                        "summary": host_relative(summ, logs), "runs": len(rows),
                                        "gpu_at_record": gpus,
                                        "gpu_gated": {"indices": gated, "source": source},
                                        "load_at_record": [round(hload, 1), cores],
                                        "notes": notes})
            print(f"{e['key']}: reference of {len(rows)} runs at {at} (recorded with "
                  f"t3.py at {commit()})")
            continue
        if not ref.is_file():
            print(f"{e['key']}: no reference ({ref})")
            failed += 1
            continue
        with open(ref, newline="") as f:
            ref_rows = list(csv.DictReader(f))
        problems, notes = compare_rows(ref_rows, rows)
        try:
            ref_at = json.loads(ref.with_suffix(".json").read_text()).get("commit", "?")
        except (OSError, ValueError):
            ref_at = "?"
        print(f"{e['key']}: {'regressed' if problems else 'as the reference'} "
              f"({len(rows)} runs at {at or why}; the reference at {ref_at})")
        for p in problems:
            print(f"  problem: {p}")
        for n in notes:
            print(f"  note: {n}")
        failed += bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
