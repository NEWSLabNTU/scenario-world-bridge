#!/usr/bin/env python3
"""The gap campaign's live loop (roadmap 025 steps 4, 6 and 7; 077 step 4, inventory item 37).

The search (gap_search), the genome and its renderer (gap_genome), the run's outcome and the
campaign store (gap_campaign) and the exploit core (gap_exploit) were built and tested offline;
this runs them against a live stack on the project workflow. A run is one genome: its scenario
rendered into the store, `ros2 run scensim run <xosc>` in the campaign's project (whose stack
records the gap trace and stays up between runs), then the run's JUnit and its trace's findings
(gap_oracle's findings.json, the oracle output `gap_campaign.classify` takes) classified and
told to the search.

    gap_live.py loop --store <dir> [--project scenarios/projects/gap_campaign]
        [--families lead_brake,...] [--budget 60] [--seed 0] [--until C1:vehicle:ssv2]
    gap_live.py exploit --store <dir> --genome <genome.json> [--signature S] [--tol 0.1]
    gap_live.py report --store <dir>

`loop` creates the store on its first call (campaign.yaml: families, budget, seed) and resumes
from it on every later one: a genome begun and not finished (the loop was killed) is run again
before the search asks anything new. `--until` stops at the first run whose signature holds that
contract (step 7: rediscovery). `exploit` confirms a genome's signature (2 of 3), finds the genes
it depends on, bisects each to its boundary and shrinks the genome, every run in
`<store>/exploit/`, and writes `exploit/<genome id>.json`.

The project's stack is started by the first run (`--start-stack`) and left up.
"""
import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gap_campaign as gc  # noqa: E402
import gap_exploit as gx  # noqa: E402
import gap_genome as gg  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "scenarios" / "projects" / "gap_campaign"
INFRASTRUCTURE_RETRIES = 2
MAX_INFRASTRUCTURE_IN_A_ROW = 5  # the loop stops: the pipeline, not the genome, is failing
VERDICT = re.compile(r"^(passed|failed|error|refused)\s+(\S+)\s+(\S+)", re.M)


def classify_run(run_dir, verdict):
    """A scensim run directory as gap_campaign.classify takes it: the run's junit.xml and its gap
    trace's findings (the first trace's; a TIER IV scenario's variants are not used here). A
    run scensim could not judge (`error`) is infrastructure whatever its JUnit says; a refused
    scenario is invalid (the genome rendered something the project cannot run)."""
    if verdict == "refused":
        return gc.RunResult("invalid", gc.gs.Outcome(False, None, 0.0), "none",
                            "refused by scensim", {})
    if verdict == "error" or run_dir is None:
        return gc.RunResult("infrastructure", None, "none", f"scensim run: {verdict}", {})
    run_dir = Path(run_dir)
    junit = run_dir / "junit.xml"
    findings = sorted((run_dir / "gap_trace").glob("*.findings.json"))
    oracle = json.loads(findings[0].read_text()) if findings else None
    if oracle is None:
        # SSv2 ran, but the trace (or the oracle's reading of it) is missing: nothing to tell
        return gc.RunResult("infrastructure", None, "none", "no gap trace findings", {})
    return gc.classify(junit.read_text() if junit.is_file() else None, oracle)


class ScensimRunner:
    """`run(genome) -> RunResult`, each genome rendered into `scenarios` and run in `project`"""

    def __init__(self, project, scenarios, log, timeout=None, start_stack=True):
        # absolute: the runs execute in the project directory, the store may be named from anywhere
        self.project, self.scenarios = Path(project).resolve(), Path(scenarios).resolve()
        self.log = Path(log).resolve()
        self.timeout, self.start_stack = timeout, start_stack
        self.scenarios.mkdir(parents=True, exist_ok=True)
        self.last_run_dir = None

    def __call__(self, genome):
        xosc = self.scenarios / f"{genome.family}_{genome.id}.xosc"
        xosc.write_text(gg.render(genome))
        argv = ["ros2", "run", "scensim", "run", str(xosc)]
        if self.start_stack:
            argv.append("--start-stack")
        if self.timeout:
            argv += ["--timeout", str(self.timeout)]
        t0 = time.time()
        p = subprocess.run(argv, cwd=self.project, capture_output=True, text=True)
        m = VERDICT.search(p.stdout)
        verdict, run_dir = (m.group(1), m.group(3)) if m else (
            "refused" if p.returncode == 3 else "error", None)
        self.last_run_dir = run_dir
        with open(self.log, "a") as f:
            f.write(json.dumps({"genome": genome.id, "family": genome.family, "exit": p.returncode,
                                "verdict": verdict, "run_dir": run_dir,
                                "wall_s": round(time.time() - t0, 1),
                                "stderr": p.stderr[-400:] if p.returncode not in (0, 1) else ""}) + "\n")
        return classify_run(run_dir, verdict)


def holds(signature, contract):
    return bool(signature) and contract in signature.split("+")


def loop(store, run, budget_left=None, until=None, out=print):
    """Run the search until its budget is spent (or `until` is found): the store's pending
    genome first, then the search's asks. Returns the finished runs' count and the run that
    found `until`, if one did."""
    search = store.search()
    found = None
    runs = infra_in_a_row = 0
    pending = store.unfinished()
    while True:
        if pending is not None:
            genome, info = pending, {"resumed": True}
            pending = None
            search.pending.setdefault(genome.id, None)
        else:
            if search.asked >= search.budget or (budget_left is not None and runs >= budget_left):
                break
            genome = search.ask()
            info = search.pending.get(genome.id)
        store.begin(genome, info)
        store.save_search(search)
        for _ in range(INFRASTRUCTURE_RETRIES + 1):  # an infrastructure failure: run it again
            result = run(genome)
            if result.kind != "infrastructure":
                break
        run_dir = getattr(run, "last_run_dir", None)
        store.finish(genome, result, run_dir, info)
        if result.kind != "infrastructure":
            search.tell(genome, result.outcome)
        store.save_search(search)
        runs += 1
        sig = result.outcome.signature if result.outcome else None
        out(f"{runs:4d} {genome.family:<20} {genome.id} {result.kind:<14} {result.verdict:<8} "
            f"{sig or '-'}")
        infra_in_a_row = infra_in_a_row + 1 if result.kind == "infrastructure" else 0
        if infra_in_a_row >= MAX_INFRASTRUCTURE_IN_A_ROW:
            out(f"stopped: {infra_in_a_row} runs in a row ended as infrastructure "
                f"({result.message}); the stack or the project needs a look")
            break
        if until and holds(sig, until):
            found = {"genome": genome.to_json(), "signature": sig, "run": runs,
                     "asked": search.asked, "run_dir": run_dir}
            break
    return runs, found


def exploit(genome, run, signature=None, tol=0.1, repeats=1):
    """Step 6: confirm, the genes that matter, each one's boundary, the shrunk genome."""
    c = gx.confirm(genome, run)
    report = {"genome": genome.to_json(), "confirm": {
        "confirmed": c.confirmed, "signature": c.signature, "signatures": c.signatures,
        "runs": c.runs}}
    sig = signature or (c.signature.split("+")[0] if c.signature else None)
    if not c.confirmed or sig is None:
        return report
    report["signature"] = sig
    rel = gx.relevant(genome, sig, run, repeats=repeats)
    report["relevant"] = {"genes": rel.genes, "runs": rel.runs}
    report["boundaries"] = []
    for gene in rel.genes:
        if not isinstance(gg.FAMILIES[genome.family].genes[gene], gg.Real):
            continue
        b = gx.boundary(genome, gene, sig, run, tol, repeats=repeats)
        report["boundaries"].append({"gene": b.gene, "inside": b.inside, "outside": b.outside,
                                     "estimate": b.estimate, "runs": b.runs})
    s = gx.shrink(genome, sig, run, repeats=repeats)
    report["shrunk"] = {"genome": s.genome.to_json(), "runs": s.runs}
    return report


def summary(store):
    runs = store.runs()
    kinds = {}
    for r in runs:
        kinds[r["kind"]] = kinds.get(r["kind"], 0) + 1
    sigs = {}
    for r in runs:
        s = (r.get("outcome") or {}).get("signature")
        if s:
            sigs.setdefault(s, []).append(r["genome"]["id"])
    return {"runs": len(runs), "kinds": kinds, "signatures": {k: len(v) for k, v in sigs.items()},
            "first": {k: next(i + 1 for i, r in enumerate(runs)
                              if (r.get("outcome") or {}).get("signature") == k) for k in sigs}}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    lp = sub.add_parser("loop")
    lp.add_argument("--store", required=True, type=Path)
    lp.add_argument("--project", type=Path, default=PROJECT)
    lp.add_argument("--families", default="lead_brake")
    lp.add_argument("--budget", type=int, default=60)
    lp.add_argument("--seed", type=int, default=0)
    lp.add_argument("--until", default=None, help="stop at the first run with this contract")
    lp.add_argument("--max-runs", type=int, default=None, help="at most this many runs this call")
    lp.add_argument("--timeout", type=int, default=None, help="SSv2's global timeout per run (s)")
    ex = sub.add_parser("exploit")
    ex.add_argument("--store", required=True, type=Path)
    ex.add_argument("--project", type=Path, default=PROJECT)
    ex.add_argument("--genome", required=True, type=Path)
    ex.add_argument("--signature", default=None)
    ex.add_argument("--tol", type=float, default=0.1)
    ex.add_argument("--repeats", type=int, default=1)
    ex.add_argument("--timeout", type=int, default=None)
    rp = sub.add_parser("report")
    rp.add_argument("--store", required=True, type=Path)
    a = ap.parse_args()
    a.store = a.store.resolve()

    if a.cmd == "report":
        print(json.dumps(summary(gc.Store.open(a.store)), indent=1))
        return 0
    if a.cmd == "loop":
        if (a.store / "campaign.yaml").exists():
            store = gc.Store.open(a.store)
        else:
            store = gc.Store.create(a.store, {
                "sim": "carla", "families": a.families.split(","), "budget": a.budget,
                "seed": a.seed, "project": str(a.project.resolve()), "until": a.until})
        run = ScensimRunner(a.project, a.store / "scenarios", a.store / "scensim_runs.jsonl", a.timeout)
        n, found = loop(store, run, a.max_runs, a.until)
        s = summary(store)
        s["found"] = found
        (a.store / "summary.json").write_text(json.dumps(s, indent=1))
        print(json.dumps({"runs_this_call": n, **s}, indent=1))
        return 0
    genome = gg.Genome.from_json(json.loads(a.genome.read_text()))
    d = a.store / "exploit"
    run = ScensimRunner(a.project, d / "scenarios", d / "scensim_runs.jsonl", a.timeout)
    rep = exploit(genome, run, a.signature, a.tol, a.repeats)
    (d / f"{genome.id}.json").write_text(json.dumps(rep, indent=1))
    print(json.dumps(rep, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
