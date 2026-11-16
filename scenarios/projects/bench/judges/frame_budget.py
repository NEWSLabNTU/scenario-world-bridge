#!/usr/bin/env python3
"""frame_budget: a run's bridge processing time per frame against a bound.

Reads the run's result.json metrics (swb.frames: the largest p95 of the bridge's 200-frame
windows, a bound on the run's own) and passes when frame_p95_ms is at most --p95-ms.
Roadmap 016's bound: 20 NPCs at 20 Hz under 10 ms (measured 1.79 ms, 2026-10-07). A run
with no full 200-frame window has no metrics: that is an error, not a pass.

    python3 judges/frame_budget.py [--p95-ms 10] <run dir>
"""
import argparse
import json
import sys
from pathlib import Path

KEYS = ("frame_windows", "frame_count", "frame_p50_ms", "frame_p95_ms", "frame_max_ms",
        "tick_p50_ms", "tick_p95_ms", "tick_max_ms", "frames_over_budget", "ticks_over_budget")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--p95-ms", type=float, default=10.0)
    ap.add_argument("run_dir")
    a = ap.parse_args()
    metrics = json.loads((Path(a.run_dir) / "result.json").read_text()).get("metrics") or {}
    if metrics.get("frame_p95_ms") is None:
        print(json.dumps({"message": "no frame-budget window in the run's bridge log"}))
        return 2
    ok = metrics["frame_p95_ms"] <= a.p95_ms
    out = {"verdict": "passed" if ok else "failed", "bound_p95_ms": a.p95_ms,
           **{k: metrics.get(k) for k in KEYS},
           "message": f"processing p95 {metrics['frame_p95_ms']} ms {'<=' if ok else '>'} {a.p95_ms} ms"}
    print(json.dumps(out))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
