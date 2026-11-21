"""Roadmap 091 W23 (the first cut of W11 (b)): the twin's runs against the real car's, by design
note 5.3's bands, per quantity and condition.

    python3 compare_runs.py REAL_LOGS TWIN_LOGS [--json OUT] [--only M1,M5]

Each LOGS directory holds one directory per run: a session.yaml beside the run's log, the real
car's rosbag2 (`bag/`, bags.py) or twin_runs.py's `streams.npz`. A failed band is reported per
quantity and condition, never averaged away. Exit 0 when every band passes, 1 when any fails.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "protocol"))
import protocol as protomod  # noqa: E402
import quantities as qmod  # noqa: E402


def load_runs(logs, only=None):
    """[(session, streams)] of every run under `logs`."""
    out = []
    for d in sorted(Path(logs).iterdir()):
        s = d / "session.yaml"
        if not s.is_file():
            continue
        sess = yaml.safe_load(s.read_text())
        if only and sess["manoeuvre"] not in only:
            continue
        if (d / "bag").is_dir():  # the real car's format (bags.py); W22's logs come so
            import bags
            out.append(bags.read(d, protomod.MOTORS))
            continue
        with np.load(d / "streams.npz") as z:
            out.append((sess, {k: z[k] for k in z.files}))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("real")
    ap.add_argument("twin")
    ap.add_argument("--protocol", default=str(HERE.parent / "protocol" / "protocol.yaml"))
    ap.add_argument("--json", default="")
    ap.add_argument("--only", default="")
    args = ap.parse_args(argv)
    only = [m for m in args.only.split(",") if m] or None
    doc = protomod.load(args.protocol)
    vs = qmod.verdicts(load_runs(args.real, only), load_runs(args.twin, only), doc)
    failed = [v for v in vs if not v.passed]
    for v in vs:
        mark = "pass" if v.passed else "FAIL"
        twin = "none" if v.twin is None else f"{v.twin:.4g}"
        print(f"{mark}  {v.manoeuvre:4} {v.condition:32} {v.quantity:30} twin {twin:>9}  "
              f"band [{v.low:.4g}, {v.high:.4g}]")
    print(f"{len(vs) - len(failed)} of {len(vs)} bands pass")
    if args.json:
        Path(args.json).write_text(json.dumps(
            [dict(v.__dict__, passed=v.passed) for v in vs], indent=1, default=float))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
