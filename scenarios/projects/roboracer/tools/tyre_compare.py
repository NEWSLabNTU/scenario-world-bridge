"""Roadmap 091 W8: PhysX's tyre on the RoboRacer against identified F1TENTH tyres.

    python3 tyre_compare.py [OUT.json]

PX is PhysX's lateral curve on the example spec, F_y / F_z against slip angle (rad), as
smallcar.py's steering ramps and M6 measured it at 240 Hz (both axles alike). REFS are the
Pacejka tyres ForzaETH identified on their cars (On-Track-SysID, models/<car>/<car>_pacejka.txt:
[B, C, D, E] per axle and mu, taken as F_y = mu F_z D sin(...)). For each reference, the
single-track model (single_track.py) gives M5 (steady circles) and M6 (grip limit) on the
example's geometry, three ways:
- with the spec's tyre;
- with PhysX's curve fitted with one stiffness and friction for all four wheels (what spec v2
  allows);
- fitted with a stiffness and friction per axle.
It prints and writes the understeer gradient, M5's radius errors and M6's.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, str(Path(__file__).resolve().parent))
from single_track import m6, pacejka, steady_cont, table  # noqa: E402

PX = [(0,0),(0.01,0.044),(0.02,0.089),(0.04,0.175),(0.06,0.255),(0.08,0.33),(0.10,0.42),(0.12,0.475),
      (0.14,0.54),(0.16,0.60),(0.18,0.69),(0.20,0.745),(0.24,0.80),(0.28,1.05),(0.30,1.1),(0.5,1.1)]
C0, MU0 = 4.4, 1.1
def physx(c, mu):  # PhysX's measured shape, its slope and peak scaled
    base = table(PX)
    return lambda a: (mu / MU0) * base(a * (c / C0) * (MU0 / mu))

CAR = {"m": 3.74, "lf": 0.1716, "lr": 0.1584}
L = CAR["lf"] + CAR["lr"]
REFS = {  # ForzaETH On-Track-SysID models/<car>/<car>_pacejka.txt, [B, C, D, E], mu
    "NUC6": ([6.7948, 1.3492, 0.8768, 0.6186], [6.9876, 2.6412, 0.9679, 0.1296], 0.9514),
    "NUC7": ([11.1259, 1.2333, 0.8809, 0.7587], [7.5143, 4.2815, 0.7418, 0.8582], 0.9514),
}
M5 = [(a, v) for a in (0.1, 0.2, 0.3, 0.36) for v in (1.0, 2.0, 3.0)]

def run(tf, tr):
    m5 = {}
    for a, v in M5:
        s = steady_cont(CAR, tf, tr, a, v)
        m5[(a, v)] = s
    ok = [(a, s) for (a, v), s in m5.items() if s]
    # understeer gradient: delta - L/R against ay, a least-squares slope
    xs = [s["ay"] for a, s in ok]; ys = [a - L / s["radius"] for a, s in ok]
    K = float(np.polyfit(xs, ys, 1)[0]) if len(xs) > 2 else float("nan")
    six = {d: m6(CAR, tf, tr, d) for d in (0.1, 0.2)}
    return m5, K, six

def err(ref, cand):
    (m5r, Kr, sr), (m5c, Kc, sc) = ref, cand
    e = []
    for k in M5:
        if m5r[k] and m5c[k]:
            e.append((m5c[k]["radius"] - m5r[k]["radius"]) / m5r[k]["radius"])
        else:
            e.append(1.0)
    six = [(sc[d]["ay"] - sr[d]["ay"]) / sr[d]["ay"] if sc[d] and sr[d] else 1.0 for d in (0.1, 0.2)]
    return e, six

out = {}
for name, (pf, pr, mu) in REFS.items():
    ref = run(pacejka(*pf, mu), pacejka(*pr, mu))
    res = {"ref": {"K": ref[1], "m6": {d: ref[2][d]["ay"] for d in ref[2]},
                   "m5": {f"{a}_{v}": ref[0][(a, v)]["radius"] for a, v in M5 if ref[0][(a, v)]}}}
    spec = run(physx(4.4, 1.1), physx(4.4, 1.1))
    e, s6 = err(ref, spec)
    res["spec"] = {"K": spec[1], "m5_err_max": max(map(abs, e)), "m6_err": s6}
    def cost(x, shared):
        cf, muf, cr, mur = (x[0], x[1], x[0], x[1]) if shared else x
        if min(cf, muf, cr, mur) <= 0.1: return 1e3
        e, s6 = err(ref, run(physx(cf, muf), physx(cr, mur)))
        return float(np.sum(np.square(e)) + 4 * np.sum(np.square(s6)))
    for shared, x0 in ((True, [6.0, 0.9]), (False, [6.0, 0.9, 12.0, 0.9])):
        r = minimize(cost, x0, args=(shared,), method="Nelder-Mead", options={"xatol": 1e-3, "fatol": 1e-6, "maxiter": 400})
        x = r.x if not shared else [r.x[0], r.x[1], r.x[0], r.x[1]]
        cand = run(physx(x[0], x[1]), physx(x[2], x[3]))
        e, s6 = err(ref, cand)
        res["shared" if shared else "per_axle"] = {"front": [round(x[0], 2), round(x[1], 3)],
            "rear": [round(x[2], 2), round(x[3], 3)], "K": cand[1],
            "m5_err_max": max(map(abs, e)), "m5_err_rms": float(np.sqrt(np.mean(np.square(e)))), "m6_err": s6}
    out[name] = res
print(json.dumps(out, indent=1, default=float))
json.dump(out, open(sys.argv[1] if len(sys.argv) > 1 else "tyre_compare.json", "w"), indent=1, default=float)
