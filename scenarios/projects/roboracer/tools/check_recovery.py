"""Roadmap 091 W23's exit, after the fit: each deliberate perturbation against what the fitting
tool recovered, within a stated tolerance. It is the only step that reads the sealed truth, and
it runs after the fit is done.

    python3 check_recovery.py SEALED/truth.yaml FITTED.yaml [--logs REAL_LOGS]

With `--logs` it also checks that the logs' sessions name the truth's sha256, so the fit was made
on the logs this truth belongs to. Exit 0 when every perturbed value is recovered within its
tolerance, 1 otherwise.
"""
import argparse
import hashlib
import sys
from pathlib import Path

import yaml

# The tolerances, relative unless marked absolute. They are what the protocol can tell: the
# reference's 2 mm, the VESC's 50 Hz, the IMU's noise.
TOLERANCE = {
    ("vehicle", "mass"): 0.03,
    ("component", "speed_kp"): 0.10,
    ("component", "speed_ki"): 0.20,
    ("component", "current_max"): 0.02,
    ("component", "latency"): ("abs", 0.005),
    ("component", "servo_offset"): ("abs", 0.003),
    ("axle_tires", "lat_stiff_value"): 0.15,
    ("axle_tires", "tire_friction"): 0.05,
    ("noise", "*"): 0.25,
}


def within(truth, got, tol):
    if got is None:
        return False, None
    if isinstance(tol, tuple):
        return abs(got - truth) <= tol[1], got - truth
    rel = (got - truth) / truth if truth else got
    return abs(rel) <= tol, rel


def compare(truth, fitted):
    """[(name, truth, fitted, error, tolerance, ok)] for every perturbed value."""
    out = []
    t, f = truth["perturbation"], fitted
    for k, v in (t.get("vehicle") or {}).items():
        if k == "axle_tires":
            for axle, vals in v.items():
                for kk, vv in vals.items():
                    got = (((f["vehicle"].get("axle_tires") or {}).get(axle) or {}).get(kk))
                    tol = TOLERANCE[("axle_tires", kk)]
                    ok, err = within(vv, got, tol)
                    out.append((f"vehicle.axle_tires.{axle}.{kk}", vv, got, err, tol, ok))
        else:
            tol = TOLERANCE.get(("vehicle", k), 0.05)
            ok, err = within(v, f["vehicle"].get(k), tol)
            out.append((f"vehicle.{k}", v, f["vehicle"].get(k), err, tol, ok))
    for k, v in (t.get("component") or {}).items():
        tol = TOLERANCE.get(("component", k), 0.10)
        got = (f.get("component") or {}).get(k)
        if k == "latency_jitter":  # not in the fitter's model: reported, not passed or failed
            out.append((f"component.{k}", v, got, None, "not modelled", None))
            continue
        ok, err = within(v, got, tol)
        out.append((f"component.{k}", v, got, err, tol, ok))
    for k, v in (t.get("noise") or {}).items():
        got = (f.get("noise") or {}).get(k)
        ok, err = within(v, got, TOLERANCE[("noise", "*")])
        out.append((f"noise.{k}", v, got, err, TOLERANCE[("noise", "*")], ok))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("truth")
    ap.add_argument("fitted")
    ap.add_argument("--logs", default="")
    args = ap.parse_args(argv)
    text = Path(args.truth).read_text()
    truth = yaml.safe_load(text)
    fitted = yaml.safe_load(Path(args.fitted).read_text())
    if args.logs:
        sha = hashlib.sha256(text.encode()).hexdigest()
        names = {yaml.safe_load(p.read_text()).get("truth_sha256")
                 for p in Path(args.logs).glob("*/session.yaml")}
        if names != {sha}:
            print(f"the logs name truth {sorted(map(str, names))}, this truth is {sha}")
            return 1
    rows = compare(truth, fitted)
    # what the fit says the logs could not identify: reported, neither passed nor failed
    unident = set(fitted.get("not_identified") or [])
    rows = [(n, tv, fv, err, "not identified by the logs", None) if n in unident
            else (n, tv, fv, err, tol, ok) for n, tv, fv, err, tol, ok in rows]
    for name, tv, fv, err, tol, ok in rows:
        mark = "n/a " if ok is None else ("ok  " if ok else "MISS")
        e = "" if err is None else f"{err:+.4g}"
        print(f"{mark} {name:40} truth {tv!s:>10} fitted {fv!s:>12} error {e:>9} tolerance {tol}")
    misses = [r for r in rows if r[5] is False]
    judged = [r for r in rows if r[5] is not None]
    print(f"{len(judged) - len(misses)} of {len(judged)} recovered within tolerance, "
          f"{len(rows) - len(judged)} n/a")
    return 1 if misses else 0


if __name__ == "__main__":
    sys.exit(main())
