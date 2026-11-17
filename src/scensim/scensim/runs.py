"""``scensim runs list|show|why|prune|summary``: a project's results and logs.

docs/design/projects-and-pilots.md, "Results" and "Logs". Runs live in ``<project>/runs``
(a link to ``<logs>/<project name>/runs``), one directory per run id; ``--root`` names another
directory of runs. Exit codes: 0 done, 3 refused (no runs directory, an unknown run, bad
arguments).
"""
import argparse
import csv
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
               "contacts"},
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


def downgrade(run_dir, level: str, dry_run=False) -> list:
    """Remove from ``run_dir`` what ``level`` does not keep; returns the removed paths."""
    keep = kept_names(level)
    if keep is None:
        return []
    removed = []
    for p in sorted(Path(run_dir).iterdir()):
        if p.name in keep:
            continue
        removed.append(p)
        if not dry_run:
            if p.is_dir() and not p.is_symlink():
                shutil.rmtree(p)
            else:
                p.unlink()
    return removed


def apply_keep(run_dir, verdict: str, keep: dict, judged=()) -> str:
    """Prune a finished run to the level ``keep`` names for its verdict (the project's
    ``logging.keep``, e.g. ``{passed: result, failed: full, error: full}``); a verdict it does
    not name keeps everything. ``judged``: its judges' verdicts; a judge's ``failed`` or
    ``error`` keeps what that verdict keeps when it is more (roadmap 077, G52: a run SSv2
    passed and a judge failed kept no logs to look into). Returns the level kept."""
    verdicts = [verdict] + [v for v in judged if v in (results.FAILED, results.ERROR)]
    levels = [(keep or {}).get(v) for v in verdicts]
    if any(lv is None for lv in levels):
        return "debug"
    level = max(levels, key=LEVELS.index)
    downgrade(run_dir, level)
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
                  "duration", "judges", "path")


def summarize(run_root: Path) -> tuple:
    """Write ``summary.csv`` and a merged ``junit.xml`` in ``run_root``; returns both paths."""
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
                        " ".join(f"{k}={v}" for k, v in judges.summary(d).items()), str(d)])
            suite = _junit_suite(d, r)
            for k in totals:
                totals[k] += int(suite.get(k, 0))
            suites.append(suite)
    for k, v in totals.items():
        suites.set(k, str(v))
    junit_path = run_root / "junit.xml"
    ET.indent(suites)
    ET.ElementTree(suites).write(junit_path, encoding="unicode", xml_declaration=True)
    return csv_path, junit_path


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
                "summary": cmd_summary}[args.cmd](args, out)
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
