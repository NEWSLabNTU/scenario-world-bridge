#!/usr/bin/env python3
"""The gap campaign's run outcome and store (roadmap 025 step 4).

`classify(junit_text, oracle)` turns one finished run into what the search is told:

- **infrastructure**: the run says nothing about the genome -- SSv2 had no response from
  the simulator or the simulator reported a failed frame, Autoware failed
  (`AutowareError`), the bridge exited 75, or there is no JUnit at all. Retried, never
  told to the search.
- **invalid**: SSv2 refused the scenario (`SyntaxError`, `SemanticError`,
  `SpecificationViolation`). Told, as an invalid outcome (the family's validity rate).
- **valid**: everything else, a scenario failure included (it is an outcome). The signature
  is the oracle's (contracts joined by `+`); an entity SSv2 found out of its own range
  (`InternalError` "... is out of range") adds `S:entity_limit`, a simulator-side value SSv2
  rejects. Closeness is 1 with a signature, else `exp(-d / CLOSENESS_SCALE_M)` from the
  oracle's `margins.min_disagreement_distance`, else 0.

`Store` is one campaign's directory: `campaign.yaml`, `runs.jsonl` (one line per finished
run), `search.json` (the search's state after its last tell, written atomically),
`pending.json` (the genome being run) and `findings/<signature>/` (each disagreeing genome's
scenario and run). A restored search asks the unfinished genome again, as its state is the
one before that ask.
"""

import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gap_genome as gg  # noqa: E402
import gap_search as gs  # noqa: E402
import yaml  # noqa: E402

CLOSENESS_SCALE_M = 1.0
NOT_SSV2_VERDICTS = {"carla_contact"}  # run_suite's own testcase
INVALID_TYPES = {"SyntaxError", "SemanticError", "SpecificationViolation"}
INFRASTRUCTURE = [
    re.compile(r"No response from the simulator"),
    re.compile(r"The simulator reported a failed frame"),
]
ENTITY_LIMIT = re.compile(r"Entity: .* is out of range")


@dataclass(frozen=True)
class RunResult:
    kind: str  # valid | invalid | infrastructure
    outcome: gs.Outcome | None  # None for infrastructure
    verdict: str  # pass | failure | error | none
    message: str
    margins: dict


def _ssv2_result(junit_text):
    """(verdict, error type, message) of SSv2's testcases"""
    root = ET.fromstring(junit_text)
    for case in root.iter("testcase"):
        if case.get("name") in NOT_SSV2_VERDICTS:
            continue
        for tag in ("error", "failure"):
            el = case.find(tag)
            if el is not None:
                return tag, el.get("type", ""), el.get("message", "")
    return "pass", "", ""


def classify(junit_text, oracle, bridge_exit=None):
    if junit_text is None:
        return RunResult("infrastructure", None, "none", "no JUnit result", {})
    verdict, kind, message = _ssv2_result(junit_text)
    margins = (oracle or {}).get("margins", {})
    if bridge_exit == 75 or kind == "AutowareError" or any(p.search(message) for p in INFRASTRUCTURE):
        return RunResult("infrastructure", None, verdict, message, margins)
    if kind in INVALID_TYPES:
        return RunResult("invalid", gs.Outcome(False, None, 0.0), verdict, message, margins)
    signature = list((oracle or {}).get("signature", []))
    if kind == "InternalError" and ENTITY_LIMIT.search(message):
        signature.append("S:entity_limit")
    distance = margins.get("min_disagreement_distance")
    if signature:
        closeness = 1.0
    elif distance is not None:
        closeness = math.exp(-distance / CLOSENESS_SCALE_M)
    else:
        closeness = 0.0
    sig = "+".join(sorted(signature)) or None
    return RunResult("valid", gs.Outcome(True, sig, closeness), verdict, message, margins)


def classify_dir(run_dir, bridge_exit=None):
    """a run directory as `incident` runs and the campaign loop leave it: `out/**/
    result.junit.xml` and `oracle.json`"""
    run_dir = Path(run_dir)
    junits = sorted(run_dir.glob("out/**/result.junit.xml"))
    oracle_path = run_dir / "oracle.json"
    oracle = json.loads(oracle_path.read_text()) if oracle_path.exists() else None
    return classify(junits[0].read_text() if junits else None, oracle, bridge_exit)


def _write_atomic(path, text):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


class Store:
    def __init__(self, root):
        self.root = Path(root)
        self.campaign = yaml.safe_load((self.root / "campaign.yaml").read_text())

    @staticmethod
    def create(root, campaign):
        """`campaign`: sim, families, budget, seed (and whatever else describes it)"""
        root = Path(root)
        root.mkdir(parents=True)
        (root / "campaign.yaml").write_text(yaml.safe_dump(campaign, sort_keys=False))
        (root / "runs.jsonl").touch()
        return Store(root)

    @staticmethod
    def open(root):
        return Store(root)

    def search(self):
        p = self.root / "search.json"
        if p.exists():
            return gs.Search.from_json(json.loads(p.read_text()))
        c = self.campaign
        return gs.Search(c["families"], budget=c["budget"], seed=c.get("seed", 0))

    def save_search(self, search):
        _write_atomic(self.root / "search.json", json.dumps(search.to_json()))

    def begin(self, genome, info):
        _write_atomic(self.root / "pending.json",
                      json.dumps({"genome": genome.to_json(), "info": info}))

    def unfinished(self):
        """the genome begun and never finished, or None"""
        p = self.root / "pending.json"
        if not p.exists():
            return None
        g = gg.Genome.from_json(json.loads(p.read_text())["genome"])
        return None if any(r["genome"]["id"] == g.id for r in self.runs()) else g

    def finish(self, genome, result, run_dir, info=None):
        line = {
            "genome": genome.to_json(), "info": info, "kind": result.kind,
            "verdict": result.verdict, "message": result.message[:500],
            "outcome": None if result.outcome is None else {
                "valid": result.outcome.valid, "signature": result.outcome.signature,
                "closeness": result.outcome.closeness},
            "margins": result.margins, "run_dir": str(run_dir),
        }
        with open(self.root / "runs.jsonl", "a") as f:
            f.write(json.dumps(line) + "\n")
        if result.outcome is not None and result.outcome.signature:
            d = self.root / "findings" / result.outcome.signature
            d.mkdir(parents=True, exist_ok=True)
            (d / f"{genome.id}.xosc").write_text(gg.render(genome))
            (d / f"{genome.id}.json").write_text(json.dumps(line, indent=1))
        (self.root / "pending.json").unlink(missing_ok=True)

    def runs(self):
        text = (self.root / "runs.jsonl").read_text()
        return [json.loads(x) for x in text.splitlines() if x.strip()]
