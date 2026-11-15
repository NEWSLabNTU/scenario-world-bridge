"""EV3, the cycle's cost: the ROS adapter's cycle (czp.strategy.Strategy.cycle: candidate zones, relate,
the protocol per zone, the actions) with N synthetic objects around a junction, timed per cycle.

Usage:
  python3 -m czp.bench --zones <zones.yaml> [--objects 20] [--cycles 3000] [--junction <id>] [--seed 1]
"""

from __future__ import annotations

import argparse
import json
import random
import sys
import time
from collections import defaultdict

from .geom import point_at
from .strategy import Obj, Strategy
from .zones import load_zones


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zones", required=True)
    ap.add_argument("--objects", type=int, default=20)
    ap.add_argument("--cycles", type=int, default=3000)
    ap.add_argument("--junction", default="")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="")
    a = ap.parse_args(argv)
    rng = random.Random(a.seed)
    head, zones = load_zones(a.zones)
    by_j = defaultdict(list)
    for z in zones:
        by_j[z.id.split("_")[0]].append(z)
    jid = a.junction or max(by_j, key=lambda k: len(by_j[k]))
    jz = [z for z in by_j[jid] if z.kind in ("cross_unsignalled", "all_way_stop", "left_turn", "merge")] or by_j[jid]
    aps = {ap_.id: ap_ for z in jz for ap_ in z.approaches if len(ap_.centerline) >= 2}
    ids = sorted(aps)
    st = Strategy(zones)
    ego_ap = aps[ids[0]]
    ego_s = max(ego_ap.entry_s - 30.0, 1.0)
    objs = []
    for i in range(a.objects):
        apo = aps[rng.choice(ids)]
        objs.append([f"o{i}", apo, max(apo.entry_s - rng.uniform(5, 80), 0.5), rng.uniform(0, 12)])
    times, n_dec = [], 0
    dt = 0.1
    for k in range(a.cycles):
        t = k * dt
        (ex, ey), eh = point_at(list(ego_ap.centerline), ego_s)
        obs = []
        for oid, apo, s, v in objs:
            (x, y), h = point_at(list(apo.centerline), s)
            obs.append(Obj(oid, 1, x, y, h, v, 0.0, 4.6, 1.9))
        t0 = time.perf_counter()
        acts = st.cycle(t, (ex, ey, eh, 6.0, 0.0), obs, None, (), 0.2, 0.0)
        times.append(time.perf_counter() - t0)
        n_dec += len(acts.records)
        ego_s = ego_s + 6.0 * dt if ego_s < ego_ap.exit_s + 5 else max(ego_ap.entry_s - 30.0, 1.0)
        for o in objs:
            o[2] += o[3] * dt
            if o[2] > o[1].exit_s + 10:
                o[2] = max(o[1].entry_s - 80, 0.5)
    xs = sorted(times)
    q = lambda p: 1e3 * xs[min(len(xs) - 1, int(p * (len(xs) - 1)))]  # noqa: E731
    res = {"map": head.get("name"), "zones_loaded": len(zones), "junction": jid, "junction_zones": len(by_j[jid]),
           "objects": a.objects, "cycles": len(xs), "p50_ms": round(q(0.5), 3), "p90_ms": round(q(0.9), 3),
           "p99_ms": round(q(0.99), 3), "max_ms": round(1e3 * xs[-1], 3), "records": n_dec,
           "python": sys.version.split()[0]}
    print(json.dumps(res))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)
    return 0 if res["p99_ms"] <= 20.0 else 1


if __name__ == "__main__":
    sys.exit(main())
