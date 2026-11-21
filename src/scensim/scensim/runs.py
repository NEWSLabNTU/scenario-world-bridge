"""``scensim runs list|show|why|prune|summary``: a project's results and logs.

docs/design/projects-and-pilots.md, "Results" and "Logs". Runs live in ``<project>/runs``
(a link to ``<logs>/<project name>/runs``), one directory per run id; ``--root`` names another
directory of runs. Exit codes: 0 done, 3 refused (no runs directory, an unknown run, bad
arguments).
"""
import argparse
import csv
import fnmatch
import json
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from scensim import config, judges, results, timeline

EXIT_OK, EXIT_FAILED, EXIT_INFRA, EXIT_REFUSED = 0, 1, 2, 3

# What each level keeps in a run directory; each level keeps the one below it too.
LEVELS = ("result", "standard", "full", "debug")
LEVEL_KEEPS = {
    "result": {"result.json", "resolved.yaml", "junit.xml", "timeline.jsonl", "judges",
               "contacts", "sample.json"},   # sample.json: what --replay runs (092 K3)
    "standard": {"logs", "gap_trace", "context.jsonl"},
    "full": {"record", "host.json"},
    "debug": None,  # everything
}


class Refused(Exception):
    pass


# --- keep -----------------------------------------------------------------------------------

def kept_names(level: str):
    """The top-level names a level keeps; None for everything."""
    if level not in LEVELS:
        raise ValueError(f"level {level!r} is not one of {', '.join(LEVELS)}")
    names = set()
    for lv in LEVELS[:LEVELS.index(level) + 1]:
        if LEVEL_KEEPS[lv] is None:
            return None
        names |= LEVEL_KEEPS[lv]
    return names


def _read_by_judge(name: str, reads) -> bool:
    """A top-level entry of the run directory holds a file the judges read (``reads:``)."""
    return any(fnmatch.fnmatchcase(name, Path(r.strip("/")).parts[0]) for r in reads)


def downgrade(run_dir, level: str, dry_run=False, reads=()) -> list:
    """Remove from ``run_dir`` what ``level`` does not keep; returns the removed paths. ``reads``:
    what the judges read (their ``reads:``), kept at ``standard`` and up (issue 080)."""
    keep = kept_names(level)
    if keep is None:
        return []
    if LEVELS.index(level) < LEVELS.index("standard"):
        reads = ()
    removed = []
    for p in sorted(Path(run_dir).iterdir()):
        if p.name in keep or _read_by_judge(p.name, reads):
            continue
        removed.append(p)
        if not dry_run:
            if p.is_dir() and not p.is_symlink():
                shutil.rmtree(p)
            else:
                p.unlink()
    return removed


def has_contact(run_dir) -> bool:
    """The run's contact reports (``contacts/*.json``) name a contact of the ego."""
    for f in sorted((Path(run_dir) / "contacts").glob("*.json")):
        try:
            if json.loads(f.read_text()).get("contacts"):
                return True
        except (OSError, ValueError, AttributeError):
            continue
    return False


def apply_keep(run_dir, verdict: str, keep: dict, judged=(), reads=()) -> str:
    """Prune a finished run to the level ``keep`` names for its verdict (the project's
    ``logging.keep``, e.g. ``{passed: result}``); a verdict it does not name keeps everything.
    A run that failed or ended in error, one a judge failed or could not judge (roadmap 077,
    G52), and one with a contact of the ego keep everything, whatever ``keep`` says (issue 080:
    a failed s5 run with a contact lost the pilot log its judge had read). ``reads``: the files
    the judges read (``reads:``), kept with the logs. Returns the level kept."""
    if verdict in (results.FAILED, results.ERROR) or \
            any(v in (results.FAILED, results.ERROR) for v in judged) or has_contact(run_dir):
        return "debug"
    level = (keep or {}).get(verdict)
    if level is None:
        return "debug"
    downgrade(run_dir, level, reads=reads)
    return level


# --- finding runs ---------------------------------------------------------------------------

def _logs() -> Path:
    try:
        return Path(config.host_settings()["logs"]).expanduser()
    except config.ConfigError as e:
        raise Refused(str(e)) from None


def runs_root(args) -> Path:
    tb = getattr(args, "testbench", None)
    try:
        config.check_renamed()   # a renamed variable first, before what the arguments lack
    except config.ConfigError as e:
        raise Refused(str(e)) from None
    if args.root:
        root = Path(args.root).expanduser()
    elif tb:
        # a testbench's runs: <logs>/<pkg>/runs (081 step 3), built or given as a path
        name = tb if "/" not in tb and not Path(tb).is_dir() else Path(tb).resolve().name
        root = _logs() / name / "runs"
    else:
        try:
            project = config.find_project(args.project)
        except config.ConfigError:
            ident = getattr(args, "id", None)
            if not ident:
                raise Refused("name a testbench: scensim runs list <testbench> (or run it in "
                              "a testbench directory)") from None
            # outside any testbench: the run is found among every testbench's runs
            roots = sorted({p.parent for p in _logs().glob(f"*/runs/*{ident}*") if p.is_dir()})
            if len(roots) == 1:
                return roots[0]
            if not roots:
                raise Refused(f"no run {ident} in any testbench's runs "
                              f"({_logs()}/<testbench>/runs)") from None
            raise Refused(f"run {ident} is in several testbenches' runs ("
                          + ", ".join(r.parent.name for r in roots)
                          + "): name the testbench (--testbench <tb>)"
                          ) from None
        root = project / "runs"
    if not root.is_dir():
        raise NoRuns(root)
    return root


class NoRuns(Exception):
    """A testbench or project that has no runs yet: not an error for `runs list`."""


def run_dirs(root: Path) -> list:
    """Run directories under ``root``, oldest id first (ids start with the time)."""
    return sorted(p for p in root.iterdir() if p.is_dir() and not p.name.startswith("."))


def load(run_dir: Path):
    """The run's result as a dict, or None when it has none (running, or broken)."""
    p = run_dir / "result.json"
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return None


def find_run(root: Path, ident: str) -> Path:
    p = Path(ident)
    if p.is_dir() and (p / "result.json").exists():
        return p
    exact = root / ident
    if exact.is_dir():
        return exact
    hits = [d for d in run_dirs(root) if ident in d.name]
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise Refused(f"no run {ident!r} in {root}")
    raise Refused(f"{ident!r} names {len(hits)} runs: " + ", ".join(d.name for d in hits[:5])
                  + (" ..." if len(hits) > 5 else ""))


def _load(r) -> str:
    """The host's load at the run's start and end (issue 046), or ""."""
    load = ((r or {}).get("provenance") or {}).get("load") or {}

    def one(sample):
        avg = (sample or {}).get("loadavg")
        return "/".join(f"{x:g}" for x in avg) if avg else "?"

    if not load:
        return ""
    cores = (load.get("start") or load.get("end") or {}).get("cores")
    return (f"start {one(load.get('start'))}, end {one(load.get('end'))} (1/5/15 min"
            + (f", {cores} cores)" if cores else ")"))


def _duration(r):
    d = ((r or {}).get("metrics") or {}).get("duration_s")
    return f"{d:.1f}s" if isinstance(d, (int, float)) else "-"


def _scenario_name(r):
    f = ((r or {}).get("scenario") or {}).get("file") or ""
    return Path(f).name or "-"


def table(rows, out):
    if not rows:
        return
    widths = [max(len(str(r[i])) for r in rows) for i in range(len(rows[0]) - 1)]
    for r in rows:
        cells = [str(c).ljust(w) for c, w in zip(r[:-1], widths)]
        out.write("  ".join(cells + [str(r[-1])]).rstrip() + "\n")


# --- commands -------------------------------------------------------------------------------

def _workspace(r) -> str:
    """The workspace the run's testbench was built in (081 step 3), ~ for the home."""
    ws = (((r or {}).get("provenance") or {}).get("testbench") or {}).get("workspace")
    if not ws:
        return "-"
    home = str(Path.home())
    return "~" + ws[len(home):] if ws == home or ws.startswith(home + "/") else ws


def cmd_list(args, out):
    root = runs_root(args)
    rows = [("RUN", "VERDICT", "SCENARIO", "DURATION", "WORKSPACE")]
    for d in run_dirs(root):
        r = load(d)
        rows.append((d.name, (r or {}).get("verdict", "(none)"), _scenario_name(r), _duration(r),
                     _workspace(r)))
    table(rows, out)
    return EXIT_OK


def _fmt_time(t, t0):
    if t is None:
        return "?"
    clock = time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"
    return f"{clock} +{t - t0:7.3f}" if t0 is not None else clock


def _tl_rows(tl):
    times = [e.get("t_wall") for e in tl if e.get("t_wall") is not None]
    t0 = min(times) if times else None
    rows = [("TIME", "T_SIM", "GROUP", "PROCESS", "EVENT", "MESSAGE")]
    for e in tl:
        ts = e.get("t_sim")
        msg = e.get("message") or " ".join(f"{k}={v}" for k, v in e.items() if k not in (
            "t_wall", "t_sim", "group", "process", "event", "level", "logger", "t_inferred"))
        rows.append((_fmt_time(e.get("t_wall"), t0) + ("~" if e.get("t_inferred") else ""),
                     f"{ts:.2f}" if isinstance(ts, (int, float)) else "-",
                     e.get("group") or "-", e.get("process") or "-", e.get("event") or "-",
                     msg))
    return rows


def _header(d, r, out):
    out.write(f"run       {d.name}\n")
    if r is None:
        out.write("verdict   (no result.json)\n")
        return
    out.write(f"verdict   {r.get('verdict')}\n")
    if r.get("reason"):
        out.write(f"reason    {r['reason']}\n")
    out.write(f"scenario  {(r.get('scenario') or {}).get('file', '-')}\n")
    out.write(f"duration  {_duration(r)}\n")
    load = _load(r)
    if load:
        out.write(f"load      {load}\n")


def cmd_show(args, out):
    d = find_run(runs_root(args), args.id)
    r = load(d)
    _header(d, r, out)
    tl_path = d / "timeline.jsonl"
    if not tl_path.is_file():
        out.write("(no timeline.jsonl)\n")
        return EXIT_OK
    out.write("\n")
    table(_tl_rows(timeline.read(tl_path)), out)
    return EXIT_OK


def _problem_line(e):
    where = e.get("group") or "-"
    if e.get("process"):
        where += f"/{e['process']}"
    return f"{e.get('level', '?')} in {where}: {e.get('message') or e.get('event')}"


def cmd_why(args, out):
    d = find_run(runs_root(args), args.id)
    r = load(d)
    out.write(f"run      {d.name}\n")
    out.write(f"verdict  {(r or {}).get('verdict', '(no result.json)')}\n")
    if (r or {}).get("reason"):
        out.write(f"reason   {r['reason']}\n")
    tl_path = d / "timeline.jsonl"
    tl = timeline.read(tl_path) if tl_path.is_file() else []
    first = timeline.why(tl)
    if first is None:
        out.write("first    no WARN or ERROR in the timeline\n")
        return EXIT_OK
    out.write(f"first    {_problem_line(first)}\n")
    err = timeline.first_problem(tl, "ERROR")
    if err is not None and err is not first:
        out.write(f"error    {_problem_line(err)}\n")
    return EXIT_OK


def parse_age(s: str) -> float:
    """Seconds for ``30m``, ``12h``, ``7d``, ``2w``; a bare number is days."""
    units = {"s": 1, "m": 60, "h": 3600, "d": 86400, "w": 604800}
    s = s.strip().lower()
    try:
        if s and s[-1] in units:
            return float(s[:-1]) * units[s[-1]]
        return float(s) * 86400
    except ValueError:
        raise Refused(f"--older-than {s!r}: expected a number of days or <n>s|m|h|d|w")


def cmd_prune(args, out):
    root = runs_root(args)
    now = time.time()
    chosen = []
    for d in run_dirs(root):
        r = load(d)
        if r is None:
            continue  # unfinished (perhaps running) or broken: never pruned
        if args.verdict and r.get("verdict") != args.verdict:
            continue
        chosen.append(d)
    if args.keep_last is not None:
        chosen = chosen[:max(len(chosen) - args.keep_last, 0)]
    if args.older_than is not None:
        age = parse_age(args.older_than)
        chosen = [d for d in chosen if now - (d / "result.json").stat().st_mtime >= age]
    verb = "would " if args.dry_run else ""
    for d in chosen:
        if args.to is None:
            out.write(f"{verb}remove {d.name}\n")
            if not args.dry_run:
                shutil.rmtree(d)
        else:
            removed = downgrade(d, args.to, dry_run=args.dry_run)
            if removed:
                out.write(f"{verb}keep {args.to} of {d.name}: "
                          + ", ".join(p.name for p in removed) + "\n")
    if not chosen:
        out.write("nothing to prune\n")
    return EXIT_OK


def _params(r):
    p = ((r or {}).get("scenario") or {}).get("parameters") or {}
    return ";".join(f"{k}={v}" for k, v in sorted(p.items()))


SUMMARY_FIELDS = ("run_id", "scenario", "params", "repetition", "seed", "verdict", "reason",
                  "duration", "judges", "path", "overload", "seats")


ROLLOUT_FIELDS = ("scenario", "params", "runs", "passed", "failed", "error", "pass_rate",
                  "seeds", "duration_mean", "duration_sd", "duration_min", "duration_max",
                  "overloaded", "seats")
ROLLOUT_METRIC_FIELDS = ("scenario", "params", "metric", "n", "mean", "sd", "min", "max")


def _spread(values: list) -> tuple:
    """``(mean, sample sd, min, max)`` of numbers, rounded; sd empty for one value."""
    n = len(values)
    mean = sum(values) / n
    sd = (sum((v - mean) ** 2 for v in values) / (n - 1)) ** 0.5 if n > 1 else None
    r = lambda v: "" if v is None else round(v, 4)   # noqa: E731
    return r(mean), r(sd), r(min(values)), r(max(values))


def _seeds(seeds: list) -> str:
    """Seeds as ranges: ``100-104 110``."""
    out, s = [], sorted(set(seeds))
    i = 0
    while i < len(s):
        j = i
        while j + 1 < len(s) and s[j + 1] == s[j] + 1:
            j += 1
        out.append(str(s[i]) if i == j else f"{s[i]}-{s[j]}")
        i = j + 1
    return " ".join(out)


def rollouts(run_root: Path, dirs: list) -> tuple:
    """``rollouts.csv`` and ``rollout_metrics.csv`` in ``run_root``: the runs grouped per
    scenario and parameter set (roadmap 088 step 3). The first: runs, verdict counts, the
    pass rate, the seeds, the duration's spread and how many were overloaded; the second, one
    row per numeric metric of result.json: n, mean, sd, min, max."""
    groups = {}
    for d in dirs:
        r = load(d) or {}
        key = ((r.get("scenario") or {}).get("file", ""), _params(r))
        groups.setdefault(key, []).append((d, r))
    path, mpath = run_root / "rollouts.csv", run_root / "rollout_metrics.csv"
    with open(path, "w", newline="") as f, open(mpath, "w", newline="") as mf:
        w, mw = csv.writer(f), csv.writer(mf)
        w.writerow(ROLLOUT_FIELDS)
        mw.writerow(ROLLOUT_METRIC_FIELDS)
        for (file, params), members in groups.items():
            n = len(members)
            count = {v: sum(1 for _, r in members if r.get("verdict") == v)
                     for v in results.VERDICTS}
            seeds = [(r.get("scenario") or {}).get("seed") for _, r in members]
            durs = [m["duration_s"] for _, r in members
                    if isinstance((m := r.get("metrics") or {}).get("duration_s"), (int, float))]
            spread = _spread(durs) if durs else ("", "", "", "")
            w.writerow([file, params, n, count[results.PASSED], count[results.FAILED],
                        count[results.ERROR], round(count[results.PASSED] / n, 4),
                        _seeds([x for x in seeds if isinstance(x, int)]), *spread,
                        sum(1 for d, _ in members if _overload(d)),
                        " ".join(str(k) for k in sorted({_seat_count(d) for d, _ in members}))])
            names = []
            for _, r in members:
                for k, v in (r.get("metrics") or {}).items():
                    if k not in names and isinstance(v, (int, float)) and not isinstance(v, bool):
                        names.append(k)
            for k in names:
                vals = [v for _, r in members
                        if isinstance((v := (r.get("metrics") or {}).get(k)), (int, float))
                        and not isinstance(v, bool)]
                mw.writerow([file, params, k, len(vals), *_spread(vals)])
    return path, mpath


def summarize(run_root: Path) -> tuple:
    """Write ``summary.csv``, ``rollouts.csv`` (with ``rollout_metrics.csv``) and a merged
    ``junit.xml`` in ``run_root``; returns the summary's and the JUnit's paths."""
    dirs = [d for d in run_dirs(run_root) if (d / "result.json").is_file()]
    csv_path = run_root / "summary.csv"
    suites = ET.Element("testsuites", name=str(run_root))
    totals = {"tests": 0, "failures": 0, "errors": 0}
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(SUMMARY_FIELDS)
        for d in dirs:
            r = load(d) or {}
            sc = r.get("scenario") or {}
            dur = (r.get("metrics") or {}).get("duration_s")
            w.writerow([r.get("run_id", d.name), sc.get("file", ""), _params(r),
                        sc.get("repetition", ""), "" if sc.get("seed") is None else sc["seed"],
                        r.get("verdict", ""), r.get("reason", ""),
                        "" if dur is None else dur,
                        " ".join(f"{k}={v}" for k, v in judges.summary(d).items()), str(d),
                        _overload(d), _seat_count(d)])
            suite = _junit_suite(d, r)
            for k in totals:
                totals[k] += int(suite.get(k, 0))
            suites.append(suite)
    rollouts(run_root, dirs)
    for k, v in totals.items():
        suites.set(k, str(v))
    junit_path = run_root / "junit.xml"
    ET.indent(suites)
    ET.ElementTree(suites).write(junit_path, encoding="unicode", xml_declaration=True)
    return csv_path, junit_path


def _overload(d: Path) -> str:
    """Why a run of ``scensim run --seats`` was overloaded (its dispatch.json), else ""."""
    try:
        return str((json.loads((d / "dispatch.json").read_text()) or {}).get("overloaded") or "")
    except (OSError, ValueError, AttributeError):
        return ""


def _seat_count(d: Path) -> int:
    """How many seats were taking work while a run ran (its dispatch.json); 1 without one.
    Simulated durations grow with the seats sharing the host (roadmap 088), so compare them
    only at the same count."""
    try:
        n = (json.loads((d / "dispatch.json").read_text()) or {}).get("seats")
    except (OSError, ValueError, AttributeError):
        return 1
    return n if isinstance(n, int) and not isinstance(n, bool) and n > 0 else 1


def mixed_seats(run_root: Path) -> list:
    """The rollouts.csv groups whose runs ran at different seat counts: ``(scenario, params,
    seats)``; their durations are not comparable (roadmap 088)."""
    try:
        with open(run_root / "rollouts.csv", newline="") as f:
            return [(r["scenario"], r["params"], r["seats"]) for r in csv.DictReader(f)
                    if " " in (r.get("seats") or "")]
    except (OSError, KeyError):
        return []


def mixed_seats_notes(run_root: Path) -> list:
    """One line per group of mixed_seats, for the terminal."""
    return [f"note: {Path(f).name or f}{' ' + p if p else ''} ran at {s} seats; compare its "
            "durations only at one seat count (roadmap 088)" for f, p, s in mixed_seats(run_root)]


def _junit_suite(d: Path, r: dict):
    """One <testsuite> per run: SSv2's testcases when its JUnit is there, else one testcase
    standing for the verdict."""
    run_id = r.get("run_id", d.name)
    suite = ET.Element("testsuite", name=run_id)
    cases = []
    j = d / ((r.get("ssv2") or {}).get("junit") or "junit.xml")
    if j.is_file():
        try:
            cases = results.read_junit(j)["cases"]
        except (OSError, ET.ParseError):
            cases = []
    if not cases:
        cases = [{"name": Path((r.get("scenario") or {}).get("file") or run_id).stem,
                  "failures": [], "errors": []}]
    # the verdict is scensim's: a stack error SSv2 did not see still makes the run an error
    v, reason = r.get("verdict"), r.get("reason", "")
    if v == results.ERROR and not any(c["errors"] for c in cases):
        cases[0] = {**cases[0], "errors": [("scensim", reason or "error")]}
    elif v == results.FAILED and not any(c["failures"] or c["errors"] for c in cases):
        cases[0] = {**cases[0], "failures": [("scensim", reason or "failed")]}
    nf = ne = 0
    for c in cases:
        tc = ET.SubElement(suite, "testcase", name=c["name"], classname=run_id)
        for t, m in c["failures"]:
            ET.SubElement(tc, "failure", type=t, message=m)
            nf += 1
        for t, m in c["errors"]:
            ET.SubElement(tc, "error", type=t, message=m)
            ne += 1
    suite.set("tests", str(len(cases)))
    suite.set("failures", str(nf))
    suite.set("errors", str(ne))
    return suite


def cmd_summary(args, out):
    if args.dir:
        root = Path(args.dir).expanduser()
        if not root.is_dir():
            raise Refused(f"{root}: not a directory")
    else:
        root = runs_root(args)
    csv_path, junit_path = summarize(root)
    out.write(f"{csv_path}\n{junit_path}\n")
    for note in mixed_seats_notes(root):
        out.write(note + "\n")
    return EXIT_OK


def cmd_fidelity(args, out):
    """Replay fidelity (roadmap 092 K3): each replay against its original, by record."""
    from scensim import sampled  # noqa: PLC0415
    dirs = []
    for d in args.dirs:
        p = Path(d).expanduser()
        if (p / "result.json").is_file():
            dirs.append(p)
        elif p.is_dir():
            dirs += [c for c in sorted(p.iterdir()) if (c / "result.json").is_file()]
        else:
            raise Refused(f"{p}: not a run or a directory of runs")
    if not dirs:
        raise NoRuns(" ".join(args.dirs))
    rep = sampled.fidelity(dirs)
    rows = [[p["record"][:12], p["original"], p["replay"],
             "{verdict}/{contact}/{end}".format(**p["original_outcome"]),
             "{verdict}/{contact}/{end}".format(**p["replay_outcome"]),
             (",".join(p["differs"]) or "same")
             + (" (seeds not applied)" if p["seeds_not_applied"] else "")] for p in rep["pairs"]]
    table([["record", "original", "replay", "original (verdict/contact/end)", "replay",
            "differs"]] + rows, out)
    share = "n/a" if rep["share"] is None else f"{rep['share']:.2f}"
    out.write(f"fidelity: {rep['agree']} of {rep['total']} replays reached the original's "
              f"outcome ({share})\n")
    if rep["with_seeds_not_applied"]:
        out.write(f"{rep['with_seeds_not_applied']} pair(s) ran a pilot without the record's seed "
                  f"(a stack pilot): their runs are not the same sample\n")
    for k in rep["unpaired_replays"]:
        out.write(f"unpaired: a replay of {k[:12]} whose original is not among these runs\n")
    if args.json:
        Path(args.json).write_text(json.dumps(rep, indent=2) + "\n")
        out.write(f"{args.json}\n")
    return EXIT_OK


# --- main -----------------------------------------------------------------------------------

def parser():
    ap = argparse.ArgumentParser(prog="scensim runs", description=__doc__.split("\n")[0])
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--testbench", help="the testbench whose runs (<logs>/<name>/runs)")
    common.add_argument("--project", help="a testbench directory (default: found from the "
                                          "working directory)")
    common.add_argument("--root", help="a runs directory instead of the testbench's")
    sub = ap.add_subparsers(dest="cmd", required=True)
    ls = sub.add_parser("list", parents=[common],
                        help="runs: id, verdict, scenario, duration, workspace")
    ls.add_argument("tb", nargs="?", metavar="testbench",
                    help="a testbench (its runs: <logs>/<name>/runs)")
    for name, text in (("show", "a run's result and timeline"),
                       ("why", "a run's reason and first WARN/ERROR")):
        p = sub.add_parser(name, parents=[common], help=text)
        p.add_argument("id", help="run id, a unique part of one, or a run directory")
    p = sub.add_parser("prune", parents=[common], help="remove runs or reduce what they keep")
    p.add_argument("--keep-last", type=int, metavar="N", help="spare the newest N chosen runs")
    p.add_argument("--older-than", metavar="D", help="only runs older than D (days, or 12h...)")
    p.add_argument("--verdict", choices=results.VERDICTS, help="only runs with this verdict")
    p.add_argument("--to", choices=LEVELS[:-1], metavar="LEVEL",
                   help="keep LEVEL (result|standard|full) instead of removing the run")
    p.add_argument("--dry-run", action="store_true", help="print what would go")
    p = sub.add_parser("summary", parents=[common], help="summary.csv and a merged junit.xml")
    p.add_argument("dir", nargs="?", help="a directory of runs (default: the project's)")
    p = sub.add_parser("fidelity", help="replays against their originals: the same verdict, "
                                        "contact and end?")
    p.add_argument("dirs", nargs="+", help="runs, or directories of runs (originals and replays)")
    p.add_argument("--json", help="also write the pairs here")
    return ap


def main(argv=None, out=None) -> int:
    out = out or sys.stdout
    try:
        args = parser().parse_args(argv)
    except SystemExit as e:
        return EXIT_OK if e.code == 0 else EXIT_REFUSED
    if getattr(args, "keep_last", None) is not None and args.keep_last < 0:
        sys.stderr.write("runs: --keep-last must be >= 0\n")
        return EXIT_REFUSED
    if getattr(args, "tb", None):
        args.testbench = args.tb
    try:
        return {"list": cmd_list, "show": cmd_show, "why": cmd_why, "prune": cmd_prune,
                "summary": cmd_summary, "fidelity": cmd_fidelity}[args.cmd](args, out)
    except Refused as e:
        sys.stderr.write(f"runs: {e}\n")
        return EXIT_REFUSED
    except NoRuns as e:
        if args.cmd in ("list", "summary", "prune"):
            out.write("no runs yet\n")
            return EXIT_OK
        sys.stderr.write(f"runs: no runs yet ({e.args[0]})\n")
        return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main())
