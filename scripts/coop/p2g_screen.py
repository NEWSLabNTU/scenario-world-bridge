#!/usr/bin/env python3
"""078 P2g (from P2e's screen.py, 133's): a CPU screen of the crossing's G0 table, the arena's job list (czp_rows_arena) run in crossing_game's kinematic
model with the arena's perception (crossing_game.Perception: 10 Hz, OBS late, sigma_x 0.3 m, sigma_v 0.2 m/s), the car's
lags and the arena's hold clip. Not the arena: a fast proxy to compare variants before the CARLA arena decides.

p2g_screen.py <worktree> <out.json> [--seeds N] [--seed S] [--jobs J] [--dts ...] [--v 10,14] [--rest-stamp-bias B]
    [--place A|B --autoware-launch DEAD,ACCEL,V_END] [--rest-lag S]

078 P2g: --place names the place whose car is Autoware (czp in it, and stock's rule in it: t1.5p5 in A, yb2p5 in B):
that car launches from rest as Autoware's ego did on the crossing (--autoware-launch, crossing_game.AUTOWARE_LAUNCH;
measured 0.6,0.68,3.0), the other car as the arena's. Read only that place's cells of the table. --rest-lag: the
tracked rest's lag seen by every car (crossing_game.REST_LAG; P2f: 0.8 s).
"""
import argparse, json, os, random, sys, zlib
from multiprocessing import Pool

ap = argparse.ArgumentParser()
ap.add_argument("wt"); ap.add_argument("out")
ap.add_argument("--seeds", type=int, default=20); ap.add_argument("--seed", type=int, default=100)
ap.add_argument("--jobs", type=int, default=12)
ap.add_argument("--dts", default="-1.5,-0.7,-0.3,-0.1,0,0.1,0.3,0.7,1.5"); ap.add_argument("--v", default="10,14")
ap.add_argument("--pairs", default=None, help="a:b,... only these pairs")
ap.add_argument("--rest-stamp-bias", type=float, default=0.0)
ap.add_argument("--pre", default=None, help="a module to import after czp (P2d's experiment patches)")
ap.add_argument("--place", choices=("A", "B"), default=None)
ap.add_argument("--autoware-launch", default=None, help="DEAD,ACCEL,V_END for the --place car")
ap.add_argument("--rest-lag", type=float, default=0.0)
a = ap.parse_args()
sys.path.insert(0, os.path.join(a.wt, "scripts/coop"))
import crossing_game as X  # noqa
import czp_adapter as CA  # noqa
import czp_rows_arena as RA  # noqa
import czp_rows_g0 as G0  # noqa
CA.BIAS["value"] = a.rest_stamp_bias
if a.pre:
    import importlib.util
    spec = importlib.util.spec_from_file_location("pre", a.pre)
    spec.loader.exec_module(importlib.util.module_from_spec(spec))
LAGS = X.car_lags()
X.REST_LAG = a.rest_lag
if a.autoware_launch:
    X.AUTOWARE_LAUNCH = tuple(float(x) for x in a.autoware_launch.split(","))
    X.AUTOWARE_CARS = frozenset(a.place or "")
X.HOLD_STOP = X.ARENA_HOLD_STOP


def one(j):
    ra = RA.CZP_A if j["a"] == "czp" else next(r for r in X.A_RULES if r[0] == j["a"])
    rb = RA.CZP_B if j["b"] == "czp" else next(r for r in X.B_RULES if r[0] == j["b"])
    if ra[0] == "czp":
        ra = ("go", -X.INF, False, X.INF)
    if rb[0] == "czp":
        rb = ("hold", None, None)
    cars = ("A" if j["a"] == "czp" else "") + ("B" if j["b"] == "czp" else "")
    rng = random.Random(zlib.crc32(json.dumps(j, sort_keys=True).encode()) ^ (a.seed * 7919))
    X._NOISE = (0.3, 0.2, rng)
    with CA.patched_crossing(tuple(cars), CA.CM.CROSS_UNSIGNALLED, "czp"):
        o = X.simulate(ra, rb, j["dt"], j["v"], j["v"], j.get("b_cont") or "hold", lags=LAGS)
    X._NOISE = None
    r = dict(j)
    r.update(contact=int(o.contact), lostA=o.lost[0], lostB=o.lost[1], uA=X.utility(o, 0), uB=X.utility(o, 1),
             first=o.first, both_stopped=int(o.both_stopped), b_cont=j.get("b_cont", ""))
    return r


if __name__ == "__main__":
    prs = RA.pairs("cross")
    if a.pairs:
        want = {tuple(p.split(":")) for p in a.pairs.split(",")}
        prs = [p for p in prs if p in want]
    jobs = []
    for v in [float(x) for x in a.v.split(",")]:
        for dt in [float(x) for x in a.dts.split(",")]:
            for pa, pb in prs:
                for c in (("yield", "hold") if pb == "react" else (None,)):
                    for s in range(a.seeds):
                        j = {"dt": dt, "v": v, "a": pa, "b": pb, "seed": s}
                        if c:
                            j["b_cont"] = c
                        jobs.append(j)
    with Pool(a.jobs) as p:
        rows = p.map(one, jobs, chunksize=8)
    rows = [{k: str(v) for k, v in r.items()} for r in rows]
    res, lines = G0.g0("cross", rows)
    # per-cell contact detail by (v, dt) for czp cells
    det = {}
    for r in rows:
        if "czp" in (r["a"], r["b"]) and int(r["contact"]):
            k = f'{r["a"]}/{r["b"]} v{r["v"]} dt{r["dt"]}'
            det[k] = det.get(k, 0) + 1
    json.dump({"g0": res, "lines": lines, "czp_contacts": det, "runs": len(rows)}, open(a.out, "w"), indent=1)
    print("\n".join(lines))
    print("czp contacts by cell:", json.dumps(det))
