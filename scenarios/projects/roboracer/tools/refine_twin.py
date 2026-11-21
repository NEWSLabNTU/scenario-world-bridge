"""Roadmap 091 W10: refine a fit with the twin in the loop. The twin runs on Isaac for each
candidate, and the error is in the protocol's own quantities: the long-horizon error from
matched starts, since every run starts from rest at the start of its manoeuvre.

    python3 refine_twin.py --logs REAL_LOGS --fitted FITTED.yaml --out REFINED.yaml
        --twin-cmd "CMD" [--only M1,M3,M5,M6] [--max-evals 80] [--params a,b] [--anchor 1]

The fit's stages (fit_twin.py) take each parameter from the manoeuvre that shows it most
directly, through simple models. Some parameters those models cannot reach. A rear tyre that
never nears its lateral limit still limits the launch and the braking, and the steady
single-track model misses some of the twin's own behaviour. Here the twin itself decides.
Nelder-Mead runs over the logarithms of PARAMS (or `--params`, dotted names), starting from the
fit:
- the tyres' stiffness and friction, front and rear;
- the drive unit's latency.

The cost is the mean, over the chosen runs' quantities, of the squared difference from the
real runs' mean in units of 5.3's band half-width. `--twin-cmd` runs the twin: a shell command
given `{fitted}` and `{out}`, which writes runs into `{out}` as twin_runs.py does (Isaac's
environment, so it runs apart from this one).

The refinement is anchored to the stages (W23's p2: with the latency held wrong, the twin in the
loop bent the tyres to pass more bands). Each parameter with a stage interval (fit_twin.py) adds
a term to the cost, `--anchor` times its distance from the stage estimate in units of the
interval's half-width; a lower bound only (a friction the logs do not identify) costs nothing
above the bound.

The drift rule (roadmap 091 W10): the result reports, per refined parameter, its stage value,
its refined value and the change, and flags (`drift_flagged`) every one the refinement moved out
of its stage interval. A refined twin that passes its bands with flagged parameters is not a
good fit: the model cannot hold the car, and the parameters are bent to make up for it.
"""
import argparse
import copy
import math
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import minimize

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "protocol"), str(HERE)]
import protocol as protomod  # noqa: E402
import quantities as qmod  # noqa: E402
from compare_runs import load_runs  # noqa: E402

PARAMS = ("vehicle.axle_tires.front.lat_stiff_value", "vehicle.axle_tires.front.tire_friction",
          "vehicle.axle_tires.rear.lat_stiff_value", "vehicle.axle_tires.rear.tire_friction",
          "component.latency")


def get(d, name):
    for k in name.split("."):
        d = d[k]
    return float(d)


def put(d, name, value):
    *path, last = name.split(".")
    for k in path:
        d = d[k]
    d[last] = value


def anchor_terms(values, stage, intervals):
    """[a] per parameter with an interval: its distance from the stage estimate in half-widths,
    or below a lower bound in 5 % of the bound; nothing above a lower bound."""
    out = []
    for name, v in values.items():
        iv = (intervals or {}).get(name)
        if not iv:
            continue
        lo, hi = iv
        if hi is None:
            out.append(max(0.0, lo - v) / (0.05 * abs(lo)) if lo else 0.0)
        else:
            out.append((v - stage[name]) / max((hi - lo) / 2, 1e-12))
    return out


def drift(stage, refined, intervals):
    """The drift rule: {name: {stage, refined, change, interval, outside}} per refined parameter;
    `change` is relative; `outside` when the refined value is out of the stage interval (None
    when the stage gave none)."""
    out = {}
    for name, v in refined.items():
        s = stage[name]
        iv = (intervals or {}).get(name)
        outside = None
        if iv:
            lo, hi = iv
            outside = bool(v < lo or (hi is not None and v > hi))
        out[name] = {"stage": s, "refined": v, "change": round((v - s) / s, 4) if s else None,
                     "interval": iv, "outside": outside}
    return out


def targets(real, doc):
    """{(manoeuvre, condition, quantity): (mean, half-width)} over the real runs."""
    groups = {}
    for s, st in real:
        key = (s["manoeuvre"], qmod.condition_key(s))
        groups.setdefault(key, []).append((s, qmod.quantities(s, st, doc)))
    out = {}
    for (m, c), runs in groups.items():
        for q in runs[0][1]:
            vals = [r[q] for _, r in runs if r.get(q) is not None]
            b = qmod.band(vals, qmod.resolution(q, runs[0][0])) if vals else None
            if b:
                out[(m, c, q)] = (float(np.mean(vals)), (b[1] - b[0]) / 2)
    return out


def cost_of(twin, doc, tg, anchor=()):
    total, n, worst = 0.0, 0, (0.0, None)
    for s, st in twin:
        q = qmod.quantities(s, st, doc)
        for name, v in q.items():
            key = (s["manoeuvre"], qmod.condition_key(s), name)
            if key not in tg or v is None or (isinstance(v, float) and math.isnan(v)):
                continue
            mean, half = tg[key]
            e = (v - mean) / max(half, 1e-9)
            total += e * e
            n += 1
            if abs(e) > abs(worst[0]):
                worst = (e, key)
    total += sum(a * a for a in anchor)
    return total / max(n, 1), worst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--logs", required=True)
    ap.add_argument("--fitted", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--twin-cmd", required=True)
    ap.add_argument("--protocol", default=str(HERE.parent / "protocol" / "protocol.yaml"))
    ap.add_argument("--only", default="M1,M3,M5,M6")
    ap.add_argument("--max-evals", type=int, default=80)
    ap.add_argument("--params", default=",".join(PARAMS))
    ap.add_argument("--anchor", type=float, default=1.0,
                    help="the stage intervals' weight in the cost; 0: unanchored")
    args = ap.parse_args(argv)
    doc = protomod.load(args.protocol)
    only = [m for m in args.only.split(",") if m]
    real = load_runs(args.logs, only)
    tg = targets(real, doc)
    base = yaml.safe_load(Path(args.fitted).read_text())
    names = [n for n in args.params.split(",") if n]
    stage = {n: get(base, n) for n in names}
    ivs = base.get("intervals") or {}
    if not ivs:
        print("no stage intervals in the fit (fit_twin.py --no-intervals?): unanchored, and the "
              "drift rule has nothing to check", flush=True)
    x0 = np.log([stage[n] for n in names])
    work = Path(tempfile.mkdtemp(prefix="refine_"))
    history = []

    def values(z):
        return {n: round(float(v), 6) for n, v in zip(names, np.exp(z))}

    def candidate(z):
        f = copy.deepcopy(base)
        for n, v in values(z).items():
            put(f, n, v)
        return f

    def evaluate(z):
        f = candidate(z)
        fp = work / f"fitted_{len(history)}.yaml"
        fp.write_text(yaml.safe_dump(f, sort_keys=False))
        out = work / f"twin_{len(history)}"
        cmd = args.twin_cmd.format(fitted=shlex.quote(str(fp)), out=shlex.quote(str(out)))
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if r.returncode != 0:
            history.append((float("inf"), z.tolist(), None))
            return 1e6
        anchor = [args.anchor ** 0.5 * a for a in anchor_terms(values(z), stage, ivs)]
        c, worst = cost_of(load_runs(out, only), doc, tg, anchor)
        history.append((c, z.tolist(), worst))
        print(f"eval {len(history):3d} cost {c:9.3f} " +
              " ".join(f"{n.split('.', 1)[1]}={v:.4g}" for n, v in values(z).items()) +
              f" worst {worst[0]:+.2f} {worst[1]}", flush=True)
        shutil.rmtree(out, ignore_errors=True)
        return c

    minimize(evaluate, x0, method="Nelder-Mead",
                 options={"maxfev": args.max_evals, "xatol": 0.01, "fatol": 0.01,
                          "initial_simplex": [x0] + [x0 + 0.15 * e for e in np.eye(len(x0))]})
    # the best evaluation, which may be a trial point the simplex had not yet taken in
    best = min((h for h in history if h[0] != float("inf")), key=lambda h: h[0])
    refined = candidate(np.array(best[1]))
    # what the refinement fitted is no longer unidentified: the twin's runs constrain it
    refined["not_identified"] = [n for n in refined.get("not_identified") or [] if n not in names]
    refined["refine"] = {"cost_start": history[0][0], "cost_end": float(best[0]),
                         "evals": len(history), "runs": only, "params": names,
                         "anchor": args.anchor}
    d = drift(stage, values(np.array(best[1])), ivs)
    refined["drift"] = d
    refined["drift_flagged"] = [n for n, r in d.items() if r["outside"]]
    Path(args.out).write_text(yaml.safe_dump(refined, sort_keys=False))
    for n, r in d.items():
        mark = "DRIFT" if r["outside"] else ("ok   " if r["outside"] is False else "n/a  ")
        print(f"{mark} {n:42} stage {r['stage']:<10.5g} refined {r['refined']:<10.5g} "
              f"change {r['change']:+.1%} interval {r['interval']}")
    if refined["drift_flagged"]:
        print(f"the drift rule: {len(refined['drift_flagged'])} parameter(s) moved out of their "
              "stage interval; the bands this twin passes do not make it a good fit")
    shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
