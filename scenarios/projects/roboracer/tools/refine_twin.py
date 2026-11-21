"""Roadmap 091 W10: refine a fit with the twin in the loop. The twin runs on Isaac for each
candidate, and the error is in the protocol's own quantities: the long-horizon error from
matched starts, since every run starts from rest at the start of its manoeuvre.

    python3 refine_twin.py --logs REAL_LOGS --fitted FITTED.yaml --out REFINED.yaml
        --twin-cmd "CMD" [--only M1,M3,M5,M6] [--max-evals 40]

The fit's stages (fit_twin.py) take each parameter from the manoeuvre that shows it most
directly, through simple models. Some parameters those models cannot reach. A rear tyre that
never nears its lateral limit still limits the launch and the braking, and the steady
single-track model misses some of the twin's own behaviour. Here the twin itself decides.
Nelder-Mead runs over the logarithms of:
- the tyres' stiffness, front and rear;
- the rear tyre's friction;

starting from the fit. The cost is the sum, over the chosen runs' quantities, of the squared
difference from the real runs' mean, in units of 5.3's band half-width. `--twin-cmd` runs the
twin: a shell command given `{fitted}` and `{out}`, which writes runs into `{out}` as
twin_runs.py does (Isaac's environment, so it runs apart from this one).
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

PARAMS = (("front", "lat_stiff_value"), ("rear", "lat_stiff_value"), ("rear", "tire_friction"))


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


def cost_of(twin, doc, tg):
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
    return total / max(n, 1), worst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--logs", required=True)
    ap.add_argument("--fitted", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--twin-cmd", required=True)
    ap.add_argument("--protocol", default=str(HERE.parent / "protocol" / "protocol.yaml"))
    ap.add_argument("--only", default="M1,M3,M5,M6")
    ap.add_argument("--max-evals", type=int, default=40)
    args = ap.parse_args(argv)
    doc = protomod.load(args.protocol)
    only = [m for m in args.only.split(",") if m]
    real = load_runs(args.logs, only)
    tg = targets(real, doc)
    base = yaml.safe_load(Path(args.fitted).read_text())
    x0 = np.log([base["vehicle"]["axle_tires"][a][k] for a, k in PARAMS])
    work = Path(tempfile.mkdtemp(prefix="refine_"))
    history = []

    def candidate(z):
        f = copy.deepcopy(base)
        for (a, k), v in zip(PARAMS, np.exp(z)):
            f["vehicle"]["axle_tires"][a][k] = round(float(v), 5)
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
        c, worst = cost_of(load_runs(out, only), doc, tg)
        history.append((c, z.tolist(), worst))
        print(f"eval {len(history):3d} cost {c:9.3f} " +
              " ".join(f"{a}.{k}={v:.4g}" for (a, k), v in zip(PARAMS, np.exp(z))) +
              f" worst {worst[0]:+.2f} {worst[1]}", flush=True)
        shutil.rmtree(out, ignore_errors=True)
        return c

    minimize(evaluate, x0, method="Nelder-Mead",
                 options={"maxfev": args.max_evals, "xatol": 0.01, "fatol": 0.01,
                          "initial_simplex": [x0, x0 + [0.15, 0, 0], x0 + [0, 0.15, 0],
                                              x0 + [0, 0, 0.15]]})
    # the best evaluation, which may be a trial point the simplex had not yet taken in
    best = min((h for h in history if h[0] != float("inf")), key=lambda h: h[0])
    refined = candidate(np.array(best[1]))
    # what the refinement fitted is no longer unidentified: the twin's runs constrain it
    names = {f"vehicle.axle_tires.{a}.{k}" for a, k in PARAMS}
    refined["not_identified"] = [n for n in refined.get("not_identified") or [] if n not in names]
    refined["refine"] = {"cost_start": history[0][0], "cost_end": float(best[0]),
                         "evals": len(history), "runs": only,
                         "params": [f"{a}.{k}" for a, k in PARAMS]}
    Path(args.out).write_text(yaml.safe_dump(refined, sort_keys=False))
    print(yaml.safe_dump(refined["vehicle"], sort_keys=False))
    shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
